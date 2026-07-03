#!/usr/bin/env python3
import os
import csv
import io
import threading
from typing import Iterable
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import JointState, Imu, BatteryState
from robot_common_interface.msg import JointSensor

# ===== Global Config =====
MAX_FILE_SIZE = 1024 * 1024 * 200  # 200 MB
LOG_DIR = "/home/robot/record_logs"

# Performance tuning
SIZE_CHECK_EVERY_N_LINES = 2000
FLUSH_EVERY_N_LINES = 5000
PERIODIC_CHECK_SEC = 10.0
BUFFER_BYTES = 1 << 20  # 1 MiB user buffer


# ---------- Time Helpers (Human-readable) ----------
def _extract_header_stamp(msg):
    """Safely extract msg.header.stamp if present, else None."""
    header = getattr(msg, "header", None)
    return getattr(header, "stamp", None) if header is not None else None


def stamp_to_readable_time(node: Node, header_stamp) -> str:
    """
    Convert ROS2 timestamp to human-readable local time:
    'YYYY.MM.DD HH:MM:SS.mmm'
    """
    try:
        if header_stamp and (header_stamp.sec != 0 or header_stamp.nanosec != 0):
            t = Time(seconds=header_stamp.sec,
                     nanoseconds=header_stamp.nanosec)
        else:
            t = node.get_clock().now()
    except Exception:
        t = node.get_clock().now()

    sec, nsec = t.seconds_nanoseconds()
    dt = datetime.fromtimestamp(sec)  # local time
    ms = int(nsec / 1e6)
    return f"{dt:%Y.%m.%d %H:%M:%S}.{ms:03d}"


class RollingCSVWriterPersistent:
    """
    High-performance rolling CSV writer:
    - Persistent handle
    - Auto truncate when file too large
    - Periodic flush and size check
    - Thread-safe
    """

    def __init__(self, base_name: str, header: Iterable[str],
                 log_dir: str = LOG_DIR, max_file_size: int = MAX_FILE_SIZE):
        self.base_name = base_name
        self.header = list(header)
        self.max_file_size = max_file_size
        self.log_dir = log_dir

        os.makedirs(self.log_dir, exist_ok=True)
        self.file_path = os.path.join(self.log_dir, f"{self.base_name}.csv")

        self._lock = threading.Lock()
        self._header_written = False
        self._lines_since_size_check = 0
        self._lines_since_flush = 0
        self._f = None
        self._writer = None

        self._open_append()

    # ---------- File Handling ----------
    def _open_append(self):
        self._close_nolock()
        self._f = open(self.file_path, "a", newline="", buffering=BUFFER_BYTES)
        self._writer = csv.writer(self._f)
        if os.path.getsize(self.file_path) == 0:
            self._writer.writerow(self.header)
            self._header_written = True
            self._f.flush()
        else:
            self._header_written = True

    def _close_nolock(self):
        if self._f is not None:
            try:
                self._f.flush()
            except Exception:
                pass
            try:
                self._f.close()
            except Exception:
                pass
            self._f = None
            self._writer = None

    def close(self):
        with self._lock:
            self._close_nolock()

    # ---------- Truncate Logic ----------
    def _truncate_to_latest_half_nolock(self):
        try:
            size = os.path.getsize(self.file_path)
            if size <= self.max_file_size:
                return

            keep_bytes = self.max_file_size // 2
            start = max(size - keep_bytes, 0)

            with open(self.file_path, "rb") as f:
                f.seek(start, os.SEEK_SET)
                tail_data = f.read()

            nl_pos = tail_data.find(b"\n")
            if nl_pos != -1:
                tail_data = tail_data[nl_pos + 1:]

            tmp_path = self.file_path + ".tmp"
            with open(tmp_path, "w", newline="") as out_f:
                w = csv.writer(out_f)
                w.writerow(self.header)
                text = tail_data.decode("utf-8", errors="ignore")
                out_f.write(text)

            os.replace(tmp_path, self.file_path)
            self._open_append()
            print(
                f"[INFO] {self.base_name}.csv exceeded size limit, truncated to keep recent half.")
        except Exception as e:
            print(f"[WARN] Failed to truncate {self.base_name}.csv: {e}")

    # ---------- Write ----------
    def write_row(self, fields: Iterable):
        with self._lock:
            if self._writer is None:
                self._open_append()

            self._writer.writerow(list(fields))
            self._lines_since_size_check += 1
            self._lines_since_flush += 1

            if self._lines_since_flush >= FLUSH_EVERY_N_LINES:
                try:
                    self._f.flush()
                except Exception:
                    pass
                self._lines_since_flush = 0

            if self._lines_since_size_check >= SIZE_CHECK_EVERY_N_LINES:
                self._lines_since_size_check = 0
                try:
                    if os.path.getsize(self.file_path) > self.max_file_size:
                        self._truncate_to_latest_half_nolock()
                except FileNotFoundError:
                    pass

    # ---------- Periodic Flush ----------
    def periodic_check(self):
        """Periodic flush & size check."""
        with self._lock:
            try:
                if self._f:
                    self._f.flush()
            except Exception:
                pass
            try:
                if os.path.getsize(self.file_path) > self.max_file_size:
                    self._truncate_to_latest_half_nolock()
            except FileNotFoundError:
                pass


class MultiTopicLogger(Node):
    def __init__(self):
        global FLUSH_EVERY_N_LINES, SIZE_CHECK_EVERY_N_LINES, PERIODIC_CHECK_SEC

        super().__init__('multi_topic_logger_csv')

        # ===== Parameters =====
        self.declare_parameter('enable_joint_states', True)
        self.declare_parameter('enable_joint_cmd', True)
        self.declare_parameter('enable_joint_sensor', True)
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('enable_battery1', True)
        self.declare_parameter('enable_battery2', True)

        # Performance parameters
        self.declare_parameter('flush_every_n_lines', FLUSH_EVERY_N_LINES)
        self.declare_parameter('size_check_every_n_lines',
                               SIZE_CHECK_EVERY_N_LINES)
        self.declare_parameter('periodic_check_sec', PERIODIC_CHECK_SEC)

        self._param_mutex = threading.Lock()
        self._enable = {
            "joint_states":  self.get_parameter('enable_joint_states').get_parameter_value().bool_value,
            "joint_cmd":     self.get_parameter('enable_joint_cmd').get_parameter_value().bool_value,
            "joint_sensor":  self.get_parameter('enable_joint_sensor').get_parameter_value().bool_value,
            "imu":           self.get_parameter('enable_imu').get_parameter_value().bool_value,
            "battery1":      self.get_parameter('enable_battery1').get_parameter_value().bool_value,
            "battery2":      self.get_parameter('enable_battery2').get_parameter_value().bool_value,
        }

        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self._on_params_change)

        # Update global config at runtime
        FLUSH_EVERY_N_LINES = self.get_parameter(
            'flush_every_n_lines').get_parameter_value().integer_value
        SIZE_CHECK_EVERY_N_LINES = self.get_parameter(
            'size_check_every_n_lines').get_parameter_value().integer_value
        PERIODIC_CHECK_SEC = float(self.get_parameter(
            'periodic_check_sec').get_parameter_value().double_value)

        # ===== CSV Writers =====
        self.joint_state_writer = RollingCSVWriterPersistent(
            "joint_states",
            ["timestamp", "joint_name", "position", "velocity", "effort"]
        )
        self.joint_cmd_writer = RollingCSVWriterPersistent(
            "joint_cmd_echo",
            ["timestamp", "joint_name", "position", "velocity", "effort"]
        )
        self.joint_sensor_writer = RollingCSVWriterPersistent(
            "joint_sensor",
            ["timestamp", "joint_name", "error", "error_msg", "temp", "voltage"]
        )

        self.imu_writer = RollingCSVWriterPersistent(
            "imu_central",
            ["timestamp", "orientation_x", "orientation_y", "orientation_z", "orientation_w",
             "ang_vel_x", "ang_vel_y", "ang_vel_z",
             "lin_acc_x", "lin_acc_y", "lin_acc_z"]
        )

        # >>> Battery CSV with extra columns
        self.battery1_writer = RollingCSVWriterPersistent(
            "battery1",
            ["timestamp", "voltage_V", "current_A", "charge", "capacity",
             "percentage", "adc", "alarm_info", "bms_status"]
        )
        self.battery2_writer = RollingCSVWriterPersistent(
            "battery2",
            ["timestamp", "voltage_V", "current_A", "charge", "capacity",
             "percentage", "adc", "alarm_info", "bms_status"]
        )

        # ===== QoS Settings =====
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # ===== Topic Subscriptions =====
        self.create_subscription(
            JointState, '/joint_shm_controller/joint_states', self.joint_state_callback, sensor_qos)
        self.create_subscription(
            JointState, '/joint_shm_controller/joint_cmd_echo', self.joint_cmd_callback, reliable_qos)
        self.create_subscription(
            JointSensor, '/joint_shm_controller/joint_sensor', self.joint_sensor_callback, reliable_qos)
        self.create_subscription(
            Imu, '/imu_shm_publisher/imu_central', self.imu_callback, sensor_qos)
        self.create_subscription(
            BatteryState, '/battery_controller/battery1', self.battery1_callback, reliable_qos)
        self.create_subscription(
            BatteryState, '/battery_controller/battery2', self.battery2_callback, reliable_qos)

        # Timer for periodic maintenance
        self._timer = self.create_timer(
            PERIODIC_CHECK_SEC, self._periodic_housekeeping)

        self.get_logger().info("multi_topic_logger_csv started with rolling CSV writing enabled.")

    # ---------- Dynamic Parameter Handling ----------
    def _on_params_change(self, params):
        with self._param_mutex:
            for p in params:
                if p.name.startswith('enable_'):
                    key = p.name.replace('enable_', '')
                    if key in self._enable:
                        self._enable[key] = p.value
                elif p.name == 'flush_every_n_lines':
                    globals()['FLUSH_EVERY_N_LINES'] = int(p.value)
                elif p.name == 'size_check_every_n_lines':
                    globals()['SIZE_CHECK_EVERY_N_LINES'] = int(p.value)
                elif p.name == 'periodic_check_sec':
                    globals()['PERIODIC_CHECK_SEC'] = float(p.value)
                    try:
                        self._timer.cancel()
                    except Exception:
                        pass
                    self._timer = self.create_timer(
                        PERIODIC_CHECK_SEC, self._periodic_housekeeping)
        return SetParametersResult(successful=True)

    # ---------- Periodic Maintenance ----------
    def _periodic_housekeeping(self):
        self.joint_state_writer.periodic_check()
        self.joint_cmd_writer.periodic_check()
        self.joint_sensor_writer.periodic_check()
        self.imu_writer.periodic_check()
        self.battery1_writer.periodic_check()
        self.battery2_writer.periodic_check()

    # ---------- Helpers ----------
    def _write_joint_rows(self, writer, msg, timestamp_str):
        """Helper: write one line per joint (for JointState)."""
        names = list(msg.name or [])
        positions = list(msg.position or [])
        velocities = list(msg.velocity or [])
        efforts = list(msg.effort or [])

        def safe_get(arr, i):
            try:
                return arr[i]
            except Exception:
                return ""

        for i, name in enumerate(names):
            writer.write_row([
                timestamp_str,
                name,
                str(safe_get(positions, i)),
                str(safe_get(velocities, i)),
                str(safe_get(efforts, i)),
            ])

    # ---------- Callbacks ----------
    def joint_state_callback(self, msg: JointState):
        if not self._enable["joint_states"]:
            return
        ts_str = stamp_to_readable_time(self, _extract_header_stamp(msg))
        self._write_joint_rows(self.joint_state_writer, msg, ts_str)

    def joint_cmd_callback(self, msg: JointState):
        if not self._enable["joint_cmd"]:
            return
        ts_str = stamp_to_readable_time(self, _extract_header_stamp(msg))
        self._write_joint_rows(self.joint_cmd_writer, msg, ts_str)

    def joint_sensor_callback(self, msg: JointSensor):
        if not self._enable["joint_sensor"]:
            return

        ts_str = stamp_to_readable_time(self, _extract_header_stamp(msg))

        names = list(msg.name or [])
        errors = list(msg.error or [])
        error_msgs = list(msg.error_msg or [])
        temps = list(msg.temp or [])
        voltages = list(msg.voltage or [])

        def safe_get(arr, i, cast=None, default=""):
            try:
                v = arr[i]
                if v == "" or v is None:
                    return default
                return cast(v) if cast else v
            except Exception:
                return default

        for i, name in enumerate(names):
            err_msg = safe_get(error_msgs, i, str, default="nil")
            writer_row = [
                ts_str,
                name,
                str(safe_get(errors, i, int)),
                err_msg,
                str(safe_get(temps, i, float)),
                str(safe_get(voltages, i, float)),
            ]
            self.joint_sensor_writer.write_row(writer_row)

    def imu_callback(self, msg: Imu):
        if not self._enable["imu"]:
            return
        ts_str = stamp_to_readable_time(self, _extract_header_stamp(msg))
        self.imu_writer.write_row([
            ts_str,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ])

    def battery1_callback(self, msg: BatteryState):
        if not self._enable["battery1"]:
            return
        ts_str = stamp_to_readable_time(self, _extract_header_stamp(msg))
        self.battery1_writer.write_row([
            ts_str,
            msg.voltage,
            msg.current,
            msg.charge,
            msg.capacity,
            msg.percentage,
            msg.charge,          # adc
            msg.capacity,        # alarm_info
            msg.design_capacity  # bms_status
        ])

    def battery2_callback(self, msg: BatteryState):
        if not self._enable["battery2"]:
            return
        ts_str = stamp_to_readable_time(self, _extract_header_stamp(msg))
        self.battery2_writer.write_row([
            ts_str,
            msg.voltage,
            msg.current,
            msg.charge,
            msg.capacity,
            msg.percentage,
            msg.charge,          # adc
            msg.capacity,        # alarm_info
            msg.design_capacity  # bms_status
        ])

    # ---------- Cleanup ----------
    def destroy_node(self):
        try:
            self.joint_state_writer.close()
            self.joint_cmd_writer.close()
            self.joint_sensor_writer.close()
            self.imu_writer.close()
            self.battery1_writer.close()
            self.battery2_writer.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nUser interrupted, exiting safely...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
