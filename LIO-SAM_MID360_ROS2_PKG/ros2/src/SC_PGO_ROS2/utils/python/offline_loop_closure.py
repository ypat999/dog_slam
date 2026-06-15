#!/usr/bin/env python3
"""
Super-LIO 离线回环检测工具

读取 Super-LIO 生成的 odom_poses.txt（KITTI 格式）和 Scans/*.pcd 文件，
计算 Scan Context 描述子，检测回环，执行 ICP 验证和位姿图优化。

输出文件：
  - optimized_poses.txt  : KITTI 格式优化后的轨迹
  - loop_pairs.txt       : 检测到的回环对 (frame_a frame_b dist yaw_deg)
  - merged_map.pcd       : 合并后的点云地图（可选）
  - loop_details.txt     : 详细的回环信息

用法：
  python3 offline_loop_closure.py /path/to/save_data/
  python3 offline_loop_closure.py /path/to/save_data/ --sc-dist-thres 0.25 --max-radius 80.0
  python3 offline_loop_closure.py /path/to/save_data/ --no-icp --skip-optimization
"""

import os
import sys
import argparse
import numpy as np
from numpy import linalg as LA
import time
from scipy.spatial import KDTree

try:
    import open3d as o3d
    HAS_O3D = True
except ImportError:
    HAS_O3D = False
    print("[WARN] open3d 未安装，ICP 验证和地图合并功能已禁用。")
    print("       安装: pip install open3d")

try:
    import gtsam
    HAS_GTSAM = True
except ImportError:
    HAS_GTSAM = False
    print("[WARN] gtsam (GTSAM Python) 未安装，位姿图优化功能已禁用。")
    print("       安装: pip install gtsam")

# ======================== Scan Context 参数 ========================
PC_NUM_RING = 20
PC_NUM_SECTOR = 60
PC_UNIT_SECTORANGLE = 360.0 / PC_NUM_SECTOR  # 每个扇区 6 度

# 回环检测参数（默认值，可通过 CLl 覆盖）
SC_DIST_THRES = 0.30
PC_MAX_RADIUS = 80.0
NUM_EXCLUDE_RECENT = 30
NUM_CANDIDATES_FROM_TREE = 5
SEARCH_RATIO = 0.1

# ICP 验证参数
ICP_COARSE_MAX_DIST = 30.0
ICP_COARSE_ITERATIONS = 50
ICP_COARSE_FITNESS_THRES = 1.5
ICP_FINE_MAX_DIST = 2.0
ICP_FINE_ITERATIONS = 100
ICP_FINE_FITNESS_THRES = 0.5

# ======================== 工具函数 ========================

def load_poses(filepath):
    """加载 KITTI 格式位姿（每行 12 列: R3x3 + t3x1）。"""
    poses = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            vals = [float(v) for v in line.split()]
            if len(vals) != 12:
                print(f"[WARN] 跳过来效位姿行: {line[:50]}...")
                continue
            R = np.array(vals[:9]).reshape(3, 3)
            t = np.array(vals[9:12]).reshape(3, 1)
            T = np.vstack([np.hstack([R, t]), [0, 0, 0, 1]])
            poses.append(T)
    return poses


def save_poses_kitti(filepath, poses):
    """保存位姿为 KITTI 格式（每行 12 列）。"""
    with open(filepath, 'w') as f:
        for T in poses:
            R = T[:3, :3]
            t = T[:3, 3]
            f.write(f"{R[0,0]:.10f} {R[0,1]:.10f} {R[0,2]:.10f} {t[0]:.10f} "
                    f"{R[1,0]:.10f} {R[1,1]:.10f} {R[1,2]:.10f} {t[1]:.10f} "
                    f"{R[2,0]:.10f} {R[2,1]:.10f} {R[2,2]:.10f} {t[2]:.10f}\n")


def save_loop_pairs(filepath, pairs):
    """保存回环对: frame_a frame_b sc_dist yaw_deg。"""
    with open(filepath, 'w') as f:
        f.write("# frame_a frame_b sc_distance yaw_degrees\n")
        for a, b, d, yaw in pairs:
            f.write(f"{a} {b} {d:.6f} {yaw:.2f}\n")


def load_scans_o3d(scan_dir, num_poses):
    """使用 Open3D 加载 PCD 点云。"""
    scans = []
    for i in range(num_poses):
        fname = f"{i:06d}.pcd"
        fpath = os.path.join(scan_dir, fname)
        if os.path.exists(fpath):
            pcd = o3d.io.read_point_cloud(fpath)
            scans.append(np.asarray(pcd.points))
        else:
            scans.append(np.empty((0, 3)))
    return scans


# ======================== Scan Context 实现 ========================

def xy2theta(x, y):
    """将 (x, y) 转换为角度 [0, 360)。"""
    eps = 1e-8
    if x >= 0 and y >= 0:
        return np.degrees(np.arctan(y / (x + eps)))
    if x < 0 and y >= 0:
        return 180.0 - np.degrees(np.arctan(y / (-x + eps)))
    if x < 0 and y < 0:
        return 180.0 + np.degrees(np.arctan(y / (x + eps)))
    if x >= 0 and y < 0:
        return 360.0 - np.degrees(np.arctan(-y / (x + eps)))
    return 0.0


def make_scan_context(points, max_radius=PC_MAX_RADIUS):
    """
    从点云计算 Scan Context 描述子。
    返回 (PC_NUM_RING x PC_NUM_SECTOR) 矩阵。
    每个格子存储该极坐标单元内的最大高度 z。
    """
    desc = np.zeros((PC_NUM_RING, PC_NUM_SECTOR))
    if len(points) == 0:
        return desc

    for pt in points:
        d = np.linalg.norm(pt[:2])
        if d > max_radius or d < 0.1:
            continue
        theta = xy2theta(pt[0], pt[1])
        sector_idx = min(int(theta / PC_UNIT_SECTORANGLE), PC_NUM_SECTOR - 1)
        ring_idx = min(int(d / max_radius * PC_NUM_RING), PC_NUM_RING - 1)
        # 使用高度 z 作为值（取最大高度）
        if pt[2] > desc[ring_idx, sector_idx]:
            desc[ring_idx, sector_idx] = pt[2]

    return desc


def make_ring_key(desc):
    """计算 ring key：每环非零条目的平均值。"""
    ring_key = np.zeros(PC_NUM_RING)
    for i in range(PC_NUM_RING):
        row = desc[i, :]
        non_zero = row[row > 0]
        ring_key[i] = np.mean(non_zero) if len(non_zero) > 0 else 0.0
    return ring_key


def make_sector_key(desc):
    """计算 sector key：每扇区非零条目的平均值。"""
    sector_key = np.zeros(PC_NUM_SECTOR)
    for j in range(PC_NUM_SECTOR):
        col = desc[:, j]
        non_zero = col[col > 0]
        sector_key[j] = np.mean(non_zero) if len(non_zero) > 0 else 0.0
    return sector_key


def circshift(mat, num_shift):
    """将矩阵列向右循环平移 num_shift 个位置。"""
    return np.roll(mat, num_shift, axis=1)


def dist_direct_sc(sc1, sc2):
    """
    直接 Scan Context 距离（列向余弦相似度）。
    返回 1 - 平均相似度（0 = 完全一致, 1 = 完全不同）。
    """
    num_eff_cols = 0
    sum_sim = 0.0
    for col_idx in range(sc1.shape[1]):
        col1 = sc1[:, col_idx]
        col2 = sc2[:, col_idx]
        n1 = np.linalg.norm(col1)
        n2 = np.linalg.norm(col2)
        if n1 < 1e-6 or n2 < 1e-6:
            continue
        sim = np.dot(col1, col2) / (n1 * n2)
        sum_sim += sim
        num_eff_cols += 1
    if num_eff_cols == 0:
        return 1.0
    return 1.0 - sum_sim / num_eff_cols


def fast_align_using_ringkey(rk1, rk2):
    """
    使用 ring key 快速对齐。
    返回使 ring key 差异最小的最佳偏移量（扇区数）。
    """
    best_shift = 0
    best_dist = float('inf')
    n = len(rk2)
    for shift in range(n):
        rk2_shifted = np.roll(rk2, shift)
        d = np.linalg.norm(rk1 - rk2_shifted)
        if d < best_dist:
            best_dist = d
            best_shift = shift
    return best_shift


def distance_sc(sc1, sc2):
    """
    计算带偏航对齐的 Scan Context 距离。
    返回 (距离, 偏航偏移扇区数)。
    """
    rk1 = make_ring_key(sc1)
    rk2 = make_ring_key(sc2)
    yaw_shift = fast_align_using_ringkey(rk1, rk2)
    sc2_aligned = circshift(sc2, yaw_shift)
    d = dist_direct_sc(sc1, sc2_aligned)
    return d, yaw_shift


# ======================== ICP 验证 ========================

def verify_loop_icp(source_pts, target_pts, init_yaw_deg=0.0):
    """
    两阶段 ICP 验证。
    阶段 1：使用宽松参数和偏航初始化的粗略 ICP。
    阶段 2：使用严格参数的精细 ICP。
    
    成功时返回 (relative_pose_4x4, fitness_score)，否则返回 None。
    """
    if not HAS_O3D:
        return None

    source = o3d.geometry.PointCloud()
    target = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(source_pts)
    target.points = o3d.utility.Vector3dVector(target_pts)

    # 从 SC 偏航对齐获取初始位姿猜测
    theta_rad = np.radians(init_yaw_deg)
    init_trans = np.eye(4)
    init_trans[:3, :3] = np.array([
        [np.cos(theta_rad), -np.sin(theta_rad), 0],
        [np.sin(theta_rad),  np.cos(theta_rad), 0],
        [0,                  0,                 1]
    ])

    # ---- 阶段 1：粗略 ICP ----
    icp_coarse = o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=ICP_COARSE_ITERATIONS,
        relative_fitness=1e-4,
        relative_rmse=1e-4
    )
    result_coarse = o3d.pipelines.registration.registration_icp(
        source, target, ICP_COARSE_MAX_DIST, init_trans,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        icp_coarse
    )

    if not result_coarse.fitness > 0:
        return None

    if result_coarse.inlier_rmse > ICP_COARSE_FITNESS_THRES:
        print(f"         粗略 ICP 失败: fitness={result_coarse.fitness:.4f}, "
              f"rmse={result_coarse.inlier_rmse:.4f}")
        return None

    print(f"         粗略 ICP 通过: fitness={result_coarse.fitness:.4f}, "
          f"rmse={result_coarse.inlier_rmse:.4f}")

    # ---- 阶段 2：精细 ICP ----
    icp_fine = o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=ICP_FINE_ITERATIONS,
        relative_fitness=1e-6,
        relative_rmse=1e-6
    )
    result_fine = o3d.pipelines.registration.registration_icp(
        source, target, ICP_FINE_MAX_DIST, result_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        icp_fine
    )

    if not result_fine.fitness > 0:
        return None

    if result_fine.inlier_rmse > ICP_FINE_FITNESS_THRES:
        print(f"         精细 ICP 失败: fitness={result_fine.fitness:.4f}, "
              f"rmse={result_fine.inlier_rmse:.4f}")
        return None

    print(f"         精细 ICP 通过: fitness={result_fine.fitness:.4f}, "
          f"rmse={result_fine.inlier_rmse:.4f}")

    return result_fine.transformation, result_fine.inlier_rmse


# ======================== 位姿图优化 ========================

def optimize_pose_graph(poses, loop_pairs):
    """
    使用 GTSAM 进行位姿图优化。
    
    参数:
        poses: 4x4 位姿矩阵列表（里程计）
        loop_pairs: (frame_a, frame_b, sc_dist, yaw_deg) 元组列表
    
    返回:
        optimized_poses: 优化后的 4x4 位姿矩阵列表
    """
    if not HAS_GTSAM:
        print("[SKIP] GTSAM 不可用，跳过位姿图优化。")
        return None

    print("\n[ 使用 GTSAM 进行位姿图优化 ]")

    # 将 KITTI 位姿转换为 GTSAM Pose3
    def to_gtsam_pose3(T):
        R = T[:3, :3]
        t = T[:3, 3]
        q = gtsam.Rot3.RzRyRx(
            np.arctan2(R[2, 1], R[2, 2]),
            np.arcsin(-R[2, 0]),
            np.arctan2(R[1, 0], R[0, 0])
        )
        return gtsam.Pose3(q, gtsam.Point3(t[0], t[1], t[2]))

    def from_gtsam_pose3(p):
        t = p.translation()
        R = p.rotation().matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t[0], t[1], t[2]]
        return T

    # 噪声模型
    prior_noise = gtsam.noiseModel.Diagonal.Variances(
        np.array([1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12])
    )
    odom_noise = gtsam.noiseModel.Diagonal.Variances(
        np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4])
    )
    loop_noise = gtsam.noiseModel.Robust.Create(
        gtsam.noiseModel.mEstimator.Huber.Create(1.345),
        gtsam.noiseModel.Diagonal.Variances(
            np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        )
    )

    # 构建图
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    # 首位姿先验因子
    pose0 = to_gtsam_pose3(poses[0])
    graph.add(gtsam.PriorFactorPose3(0, pose0, prior_noise))
    initial.insert(0, pose0)

    # 里程计因子
    for i in range(1, len(poses)):
        pose_i = to_gtsam_pose3(poses[i])
        initial.insert(i, pose_i)

        T_delta = np.linalg.inv(poses[i-1]) @ poses[i]
        delta = to_gtsam_pose3(T_delta)
        graph.add(gtsam.BetweenFactorPose3(i-1, i, delta, odom_noise))

    # 回环因子
    loop_added = 0
    for a, b, sc_dist, yaw_deg in loop_pairs:
        # 从 SC 偏航 + ICP 提取相对位姿
        # 使用基于里程计的相对位姿作为初始值
        T_rel = np.linalg.inv(poses[a]) @ poses[b]
        rel_pose = to_gtsam_pose3(T_rel)
        graph.add(gtsam.BetweenFactorPose3(a, b, rel_pose, loop_noise))
        loop_added += 1

    print(f"      图: {len(poses)} 个节点, "
          f"{len(poses) - 1 + loop_added} 个因子 "
          f"(里程计={len(poses)-1}, 回环={loop_added})")

    # 优化
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosity("SILENT")
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()

    print(f"      优化完成。最终误差: {optimizer.error():.6f}")

    # 提取优化后的位姿
    opt_poses = []
    for i in range(len(poses)):
        p = result.atPose3(i)
        opt_poses.append(from_gtsam_pose3(p))

    return opt_poses


# ======================== 主程序 ========================

def main():
    parser = argparse.ArgumentParser(
        description="Super-LIO 离线回环检测工具"
    )
    parser.add_argument(
        "data_dir", type=str,
        help="数据目录，包含 odom_poses.txt 和 Scans/"
    )
    parser.add_argument(
        "--sc-dist-thres", type=float, default=SC_DIST_THRES,
        help=f"Scan Context 距离阈值 (默认: {SC_DIST_THRES})"
    )
    parser.add_argument(
        "--max-radius", type=float, default=PC_MAX_RADIUS,
        help=f"Scan Context 最大半径 (默认: {PC_MAX_RADIUS}m)"
    )
    parser.add_argument(
        "--no-icp", action="store_true",
        help="禁用 ICP 回环验证"
    )
    parser.add_argument(
        "--skip-optimization", action="store_true",
        help="跳过位姿图优化（仅检测回环）"
    )
    parser.add_argument(
        "--output-dir", type=str, default=None,
        help="输出目录（默认与 data_dir 相同）"
    )
    parser.add_argument(
        "--save-map", action="store_true",
        help="保存合并后的点云地图（需要 open3d）"
    )

    args = parser.parse_args()

    data_dir = args.data_dir.rstrip('/')
    output_dir = args.output_dir.rstrip('/') if args.output_dir else data_dir
    os.makedirs(output_dir, exist_ok=True)

    sc_dist_thres = args.sc_dist_thres
    max_radius = args.max_radius
    enable_icp = not args.no_icp and HAS_O3D
    skip_optimization = args.skip_optimization
    save_map = args.save_map and HAS_O3D

    print("=" * 70)
    print("  Super-LIO 离线回环检测")
    print("=" * 70)
    print(f"  数据目录    : {data_dir}")
    print(f"  输出目录    : {output_dir}")
    print(f"  SC 阈值     : {sc_dist_thres}")
    print(f"  最大半径    : {max_radius}m")
    print(f"  ICP 验证    : {'启用' if enable_icp else '禁用'}")
    print(f"  图优化      : {'启用' if HAS_GTSAM and not skip_optimization else '跳过'}")
    print(f"  保存地图    : {'是' if save_map else '否'}")
    print("=" * 70)

    t_start = time.time()

    # ==============================================================
    # 步骤 1：加载里程计位姿
    # ==============================================================
    odom_file = os.path.join(data_dir, "odom_poses.txt")
    if not os.path.exists(odom_file):
        print(f"[ERROR] 未找到 odom_poses.txt: {odom_file}")
        sys.exit(1)

    print(f"\n[1/5] 加载里程计位姿 ... ", end="", flush=True)
    poses = load_poses(odom_file)
    print(f"已加载 {len(poses)} 个位姿")

    # ==============================================================
    # 步骤 2：加载关键帧点云
    # ==============================================================
    scan_dir = os.path.join(data_dir, "Scans")
    if not os.path.isdir(scan_dir):
        print(f"[ERROR] 未找到 Scans 目录: {scan_dir}")
        sys.exit(1)

    print(f"[2/5] 加载关键帧点云 ... ", end="", flush=True)
    if HAS_O3D:
        scans = load_scans_o3d(scan_dir, len(poses))
    else:
        print("\n[ERROR] 需要 open3d 来读取 PCD 文件。")
        sys.exit(1)

    n_valid = sum(1 for s in scans if len(s) > 0)
    print(f"已加载 {n_valid}/{len(poses)} 个含有点云的关键帧")

    # ==============================================================
    # 步骤 3：计算 Scan Context 描述子
    # ==============================================================
    print(f"[3/5] 计算 Scan Context 描述子 (R={PC_NUM_RING}, "
          f"S={PC_NUM_SECTOR}, max_r={max_radius}m) ... ", end="", flush=True)

    sc_descs = []
    ring_keys = []
    for i, pts in enumerate(scans):
        sc = make_scan_context(pts, max_radius)
        sc_descs.append(sc)
        ring_keys.append(make_ring_key(sc))

    ring_key_mat = np.array(ring_keys)
    print(f"完成（{len(sc_descs)} 个描述子）")

    # ==============================================================
    # 步骤 4：检测回环
    # ==============================================================
    print(f"[4/5] 检测回环 ...")
    
    # 构建 ring key 的 KD-Tree
    tree = KDTree(ring_key_mat)
    num_candidates = max(1, min(
        NUM_CANDIDATES_FROM_TREE,
        int(len(scans) * SEARCH_RATIO)
    ))
    print(f"      每帧搜索候选数: {num_candidates}, "
          f"排除最近帧数: {NUM_EXCLUDE_RECENT}")

    loop_candidates = []  # (cand_idx, curr_idx, sc_dist, yaw_shift)

    for curr_idx in range(len(scans)):
        if curr_idx < NUM_EXCLUDE_RECENT + 1:
            continue

        # 查询 KD-Tree
        k = min(num_candidates + NUM_EXCLUDE_RECENT + 1, len(ring_key_mat))
        dists, idxs = tree.query(ring_keys[curr_idx].reshape(1, -1), k=k)
        dists = dists[0]
        idxs = idxs[0]

        # 过滤掉最近的帧
        candidates = []
        for d, idx in zip(dists, idxs):
            if curr_idx - idx > NUM_EXCLUDE_RECENT:
                candidates.append((d, idx))
            if len(candidates) >= num_candidates:
                break

        # 使用完整的 SC 距离对候选进行评分
        best_dist = float('inf')
        best_idx = -1
        best_yaw = 0

        for _, cand_idx in candidates:
            d, yaw_shift = distance_sc(sc_descs[curr_idx], sc_descs[cand_idx])
            if d < best_dist:
                best_dist = d
                best_idx = cand_idx
                best_yaw = yaw_shift

        if best_dist < sc_dist_thres and best_idx >= 0:
            yaw_deg = best_yaw * PC_UNIT_SECTORANGLE
            # 将偏航归一化到 [-180, 180)
            if yaw_deg > 180.0:
                yaw_deg -= 360.0
            loop_candidates.append((best_idx, curr_idx, best_dist, yaw_deg))
            print(f"      [候选] 帧 {best_idx:6d} <-> {curr_idx:6d}, "
                  f"SC 距离={best_dist:.4f}, 偏航={yaw_deg:+.1f}deg")

    print(f"      发现 {len(loop_candidates)} 个回环候选")

    # ==============================================================
    # 步骤 4b：ICP 验证
    # ==============================================================
    verified_loops = []
    if enable_icp and loop_candidates:
        print(f"\n      --- ICP 验证 ---")
        for a, b, sc_dist, yaw_deg in loop_candidates:
            print(f"      验证中: 帧 {a} <-> 帧 {b} (偏航={yaw_deg:+.1f}deg) ...")
            
            # 获取车身坐标系下的点云
            source_pts = scans[b]  # 当前帧
            target_pts = scans[a]  # 历史帧

            if len(source_pts) < 100 or len(target_pts) < 100:
                print(f"         跳过: 点数过少 ({len(source_pts)}, {len(target_pts)})")
                continue

            result = verify_loop_icp(source_pts, target_pts, yaw_deg)
            if result is not None:
                verified_loops.append((a, b, sc_dist, yaw_deg))
                print(f"         [已验证] 回环 {a} <-> {b}")
            else:
                print(f"         [已拒绝] ICP 验证失败")

        print(f"\n      已验证回环: {len(verified_loops)}/{len(loop_candidates)}")
    else:
        verified_loops = loop_candidates

    # ==============================================================
    # 步骤 5：位姿图优化与输出
    # ==============================================================
    print(f"\n[5/5] 保存结果 ...")

    # 保存回环对
    loop_file = os.path.join(output_dir, "loop_pairs.txt")
    save_loop_pairs(loop_file, verified_loops)
    print(f"      已保存回环对: {loop_file}")

    # 详细回环信息
    detail_file = os.path.join(output_dir, "loop_details.txt")
    with open(detail_file, 'w') as f:
        f.write(f"# 离线回环检测结果\n")
        f.write(f"# 数据目录: {data_dir}\n")
        f.write(f"# SC 阈值: {sc_dist_thres}, 最大半径: {max_radius}\n")
        f.write(f"# ICP 验证: {'启用' if enable_icp else '禁用'}\n")
        f.write(f"# 总帧数: {len(poses)}, 回环候选: {len(loop_candidates)}, "
                f"已验证: {len(verified_loops)}\n")
        f.write(f"# 生成时间: {time.ctime()}\n")
        f.write(f"# 格式: frame_a frame_b sc_distance yaw_degrees\n")
        for a, b, d, yaw in verified_loops:
            f.write(f"{a} {b} {d:.6f} {yaw:.2f}\n")
    print(f"      已保存回环详情: {detail_file}")

    # 位姿图优化
    opt_poses = None
    if HAS_GTSAM and not skip_optimization:
        if len(verified_loops) > 0:
            opt_poses = optimize_pose_graph(poses, verified_loops)
        else:
            print("      未检测到回环，将里程计位姿复制为优化后位姿。")
            opt_poses = poses
    elif skip_optimization:
        print("      位姿图优化已跳过（--skip-optimization）。")
    else:
        print("      GTSAM 不可用，将里程计位姿复制为优化后位姿。")
        opt_poses = poses

    if opt_poses is not None:
        opt_file = os.path.join(output_dir, "optimized_poses.txt")
        save_poses_kitti(opt_file, opt_poses)
        print(f"      已保存优化后位姿: {opt_file}")

    # 保存合并地图（如果 open3d 可用且用户请求）
    if save_map:
        print(f"      正在生成合并地图 ...")
        
        map_poses = opt_poses if opt_poses is not None else poses
        
        merged_pcd = o3d.geometry.PointCloud()
        for i in range(len(scans)):
            if len(scans[i]) == 0:
                continue
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(scans[i])
            pcd.transform(map_poses[i])
            merged_pcd += pcd

        # 体素降采样
        merged_pcd = merged_pcd.voxel_down_sample(0.1)

        map_file = os.path.join(output_dir, "merged_map.pcd")
        o3d.io.write_point_cloud(map_file, merged_pcd)
        print(f"      已保存合并地图: {map_file} ({len(merged_pcd.points)} 个点)")

    # ==============================================================
    # 汇总
    # ==============================================================
    t_elapsed = time.time() - t_start
    print("\n" + "=" * 70)
    print("  汇总")
    print("=" * 70)
    print(f"  总帧数        : {len(poses)}")
    print(f"  回环候选数    : {len(loop_candidates)}")
    print(f"  已验证回环数  : {len(verified_loops)}")
    print(f"  运行时间      : {t_elapsed:.1f}s")
    print(f"  输出目录      : {output_dir}")
    print("=" * 70)


if __name__ == "__main__":
    main()
