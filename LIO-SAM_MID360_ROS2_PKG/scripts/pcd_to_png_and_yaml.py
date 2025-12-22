import open3d as o3d
import numpy as np
from PIL import Image
import yaml

pcd = o3d.io.read_point_cloud("/home/ztl/slam_data/3d_map/3dmap.pcd")
points = np.asarray(pcd.points)
output_path = "/home/ztl/slam_data/grid_map/"

# -------------------
# 高度范围过滤
z_min = -0.2    # 最低高度
z_max = 1.5    # 最高高度
mask = (points[:,2] >= z_min) & (points[:,2] <= z_max)
points = points[mask]
# -------------------

# # === 3. 旋转点云（例如绕Z轴逆时针旋转90°）===
# R = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.deg2rad(45)))
# points = (R @ points.T).T


# 投影到 XY 平面
x = points[:,0]
y = points[:,1]

# 构建栅格
res = 0.05
xmin, xmax = x.min(), x.max()
ymin, ymax = y.min(), y.max()
nx = int((xmax-xmin)/res)+1
ny = int((ymax-ymin)/res)+1
grid = np.zeros((ny, nx), dtype=np.uint8)

ix = ((x - xmin)/res).astype(int)
iy = ((y - ymin)/res).astype(int)
grid[iy, ix] = 255  # 占用点
grid = 255 - grid # 反色

img = Image.fromarray(np.flipud(grid))
img.save(output_path + "mapfrom3d.png")


yaml_data = {
    "image": "mapfrom3d.png",
    "resolution": 0.05,
    "origin": [0.0, 0.0, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196
}

with open(output_path + "mapfrom3d.yaml", "w") as f:
    yaml.dump(yaml_data, f)

print("✅ 已生成地图：" + output_path + "mapfrom3d.png")
print(f"尺寸: {nx} x {ny}, 分辨率: {res} m/px")
