import open3d as o3d
import numpy as np
from PIL import Image
import yaml

pcd = o3d.io.read_point_cloud("/home/ywj/projects/LOAM/cloudGlobal.pcd")
points = np.asarray(pcd.points)
output_path = "/home/ywj/projects/map_grid/"

# -------------------
# 高度范围过滤
z_min = -1.0    # 最低高度
z_max = 1.5    # 最高高度
mask = (points[:,2] >= z_min) & (points[:,2] <= z_max)
points = points[mask]
# -------------------

# === 3. 旋转点云（例如绕Z轴逆时针旋转90°）===
R = pcd.get_rotation_matrix_from_xyz((0, 0, np.deg2rad(45)))
points = (R @ points.T).T


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
img.save(output_path + "map.png")


yaml_data = {
    "image": "map.png",
    "resolution": 0.05,
    "origin": [0.0, 0.0, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196
}

with open(output_path + "map.yaml", "w") as f:
    yaml.dump(yaml_data, f)

print("✅ 已生成地图：" + output_path + "map.png")
print(f"尺寸: {nx} x {ny}, 分辨率: {res} m/px")
