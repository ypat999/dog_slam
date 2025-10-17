# LiDAR倾斜补偿配置说明

## 问题描述
LiDAR相对于机器人底盘有约30度前倾，导致建图出来的点云也是倾斜的。

## 解决方案
通过修改静态变换发布器的旋转参数来补偿LiDAR的倾斜角度。

### 配置更改
在 `cartographer_3d.launch.py` 文件中，修改了以下静态变换：

```python
# 原来的配置（无旋转补偿）
arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame']

# 新的配置（补偿30度前倾）
arguments=['0', '0', '0', '-0.5236', '0', '0', 'base_link', 'livox_frame']
```

### 参数说明
- 第4个参数（`-0.5236`）：绕X轴的旋转角度（弧度）
- 30度 = -0.5236弧度（负值表示顺时针旋转，用于补偿前倾）
- 其他旋转参数（Y轴和Z轴）保持为0

### 坐标系说明
- `base_link`：机器人底盘坐标系
- `livox_frame`：LiDAR坐标系
- 旋转顺序：XYZ（滚转、俯仰、偏航）

## 验证方法
1. 启动建图系统：`ros2 launch slam_offline cartographer_3d.launch.py`
2. 在RViz中观察点云是否水平
3. 检查TF树：`ros2 run tf2_tools view_frames`
4. 验证变换：`ros2 run tf2_ros tf2_echo base_link livox_frame`

## 调整建议
如果30度补偿不够准确，可以微调这个值：
- 增加补偿：使用更负的值（如-0.6）
- 减少补偿：使用接近0的值（如-0.4）

## 注意事项
1. 确保LiDAR的安装角度测量准确
2. 如果IMU也有倾斜，可能需要额外的IMU校准
3. 在调整参数后，建议重新录制数据集进行测试