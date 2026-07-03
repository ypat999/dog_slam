#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <mutex>
#include <memory>
#include <cmath>
#include <limits>

class ScanMerger : public rclcpp::Node
{
public:
    ScanMerger() : Node("scan_merger")
    {

        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", false);
        }
        this->declare_parameter("front_scan_topic", "scan_front");
        this->declare_parameter("rear_scan_topic", "scan_rear");
        this->declare_parameter("merged_scan_topic", "scan");
        this->declare_parameter("merge_timeout", 0.1);
        this->declare_parameter("angle_increment", 0.00872);
        this->declare_parameter("range_min", 0.5);
        this->declare_parameter("range_max", 100.0);

        front_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("front_scan_topic").as_string(),
            rclcpp::SensorDataQoS(),
            std::bind(&ScanMerger::frontScanCallback, this, std::placeholders::_1));

        rear_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("rear_scan_topic").as_string(),
            rclcpp::SensorDataQoS(),
            std::bind(&ScanMerger::rearScanCallback, this, std::placeholders::_1));

        merged_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            this->get_parameter("merged_scan_topic").as_string(), 10);

        merge_timeout_ = this->get_parameter("merge_timeout").as_double();
        default_angle_increment_ = this->get_parameter("angle_increment").as_double();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();

        RCLCPP_INFO(this->get_logger(), "ScanMerger initialized");
        RCLCPP_INFO(this->get_logger(), "Front scan topic: %s", this->get_parameter("front_scan_topic").as_string().c_str());
        RCLCPP_INFO(this->get_logger(), "Rear scan topic: %s", this->get_parameter("rear_scan_topic").as_string().c_str());
        RCLCPP_INFO(this->get_logger(), "Merged scan topic: %s", this->get_parameter("merged_scan_topic").as_string().c_str());
    }

private:
    void frontScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received front scan!"); // 添加这行
        std::lock_guard<std::recursive_mutex> lock(front_mutex_);
        latest_front_scan_ = scan_msg;
        tryMergeAndPublish();
    }

    void rearScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received rear scan!"); // 添加这行
        std::lock_guard<std::recursive_mutex> lock(rear_mutex_);
        latest_rear_scan_ = scan_msg;
        tryMergeAndPublish();
    }

    void tryMergeAndPublish()
    {
        // RCLCPP_INFO(this->get_logger(), "Attempting to merge scans...");
        sensor_msgs::msg::LaserScan::SharedPtr front_scan, rear_scan;
        
        {
            std::lock_guard<std::recursive_mutex> lock(front_mutex_);
            if (latest_front_scan_)
            {
                front_scan = latest_front_scan_;
            }
        }

        {
            std::lock_guard<std::recursive_mutex> lock(rear_mutex_);
            if (latest_rear_scan_)
            {
                rear_scan = latest_rear_scan_;
            }
        }

        if (!front_scan && !rear_scan)
        {
            return;
        }

        auto merged_scan = std::make_shared<sensor_msgs::msg::LaserScan>();

        if (front_scan && rear_scan)
        {
            mergeTwoScans(*merged_scan, *front_scan, *rear_scan);
            merged_scan->header.stamp = front_scan->header.stamp;
        }
        else if (front_scan)
        {
            *merged_scan = *front_scan;
        }
        else if (rear_scan)
        {
            *merged_scan = *rear_scan;
        }

        // merged_scan->header.stamp = this->now();
        
        merged_scan_pub_->publish(*merged_scan);
    }

    void mergeTwoScans(sensor_msgs::msg::LaserScan &merged,
                       const sensor_msgs::msg::LaserScan &front,
                       const sensor_msgs::msg::LaserScan &rear)
    {
        double angle_min = -M_PI;
        double angle_max = M_PI;
        double angle_increment = default_angle_increment_;
        
        if (std::abs(front.angle_increment) > 1e-6)
        {
            angle_increment = front.angle_increment;
        }

        size_t num_readings = static_cast<size_t>(std::round((angle_max - angle_min) / angle_increment)) + 1;

        merged.header = front.header;
        merged.angle_min = angle_min;
        merged.angle_max = angle_max;
        merged.angle_increment = angle_increment;
        merged.time_increment = front.time_increment;
        merged.scan_time = front.scan_time;
        merged.range_min = range_min_;
        merged.range_max = range_max_;
        merged.ranges.resize(num_readings, std::numeric_limits<float>::infinity());
        merged.intensities.resize(num_readings, 0.0);

        fillScanData(merged, front, angle_min, angle_increment, num_readings);
        fillScanData(merged, rear, angle_min, angle_increment, num_readings);
    }

    void fillScanData(sensor_msgs::msg::LaserScan &merged,
                      const sensor_msgs::msg::LaserScan &source,
                      double angle_min,
                      double angle_increment,
                      size_t num_readings)
    {
        for (size_t i = 0; i < source.ranges.size(); ++i)
        {
            double angle = source.angle_min + i * source.angle_increment;
            
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;

            size_t target_idx = static_cast<size_t>(std::round((angle - angle_min) / angle_increment));
            
            if (target_idx < num_readings)
            {
                float range = source.ranges[i];
                
                if (std::isfinite(range) && range >= range_min_ && range <= range_max_)
                {
                    if (!std::isfinite(merged.ranges[target_idx]) || range < merged.ranges[target_idx])
                    {
                        merged.ranges[target_idx] = range;
                        if (i < source.intensities.size())
                        {
                            merged.intensities[target_idx] = source.intensities[i];
                        }
                    }
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_pub_;

    std::recursive_mutex front_mutex_;
    std::recursive_mutex rear_mutex_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_front_scan_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_rear_scan_;

    double merge_timeout_;
    double default_angle_increment_;
    double range_min_;
    double range_max_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMerger>());
    rclcpp::shutdown();
    return 0;
}
