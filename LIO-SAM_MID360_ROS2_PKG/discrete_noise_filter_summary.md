# LIO-SAM离散噪点过滤机制

## 问题描述
地图中仍存在零散的离散噪点，需要在建图阶段就排除此类数据。

## 解决方案

### 1. 参数声明
**文件**: `ros2/src/LIO-SAM_MID360_ROS2_DOG/include/lio_sam/utility.hpp`
- 添加离散噪点过滤参数声明
- 包括：启用开关、最小邻居数量、搜索半径、距离范围、强度范围

### 2. 参数配置
**文件**: `ros2/src/LIO-SAM_MID360_ROS2_DOG/config/liosam_params.yaml`
```yaml
# 离散噪点过滤参数
discreteNoiseFilter:
  enableDiscreteNoiseFilter: true
  discreteNoiseMinNeighbors: 5
  discreteNoiseSearchRadius: 0.3
  discreteNoiseMinRange: 0.5
  discreteNoiseMaxRange: 35.0
  discreteNoiseMinIntensity: 0.01
  discreteNoiseMaxIntensity: 1000.0
```

### 3. 过滤函数实现
**文件**: `ros2/src/LIO-SAM_MID360_ROS2_DOG/src/featureExtraction.cpp`

**功能特点**:
- **距离过滤**: 排除距离范围外的点（0.5m-35.0m）
- **强度过滤**: 排除强度异常的点（0.01-1000.0）
- **密度过滤**: 使用KD树半径搜索，邻居点少于5个的点视为噪点

**实现逻辑**:
```cpp
void filterDiscreteNoise()
{
    if (!enableDiscreteNoiseFilter)
        return;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());
    kdtree->setInputCloud(extractedCloud);
    
    int validPoints = 0;
    for (int i = 0; i < extractedCloud->points.size(); i++)
    {
        // 距离过滤
        float range = cloudInfo.point_range[i];
        if (range < discreteNoiseMinRange || range > discreteNoiseMaxRange)
        {
            cloudNeighborPicked[i] = 1;  // 标记为排除
            continue;
        }
        
        // 强度过滤
        float intensity = extractedCloud->points[i].intensity;
        if (intensity < discreteNoiseMinIntensity || intensity > discreteNoiseMaxIntensity)
        {
            cloudNeighborPicked[i] = 1;  // 标记为排除
            continue;
        }
        
        // 密度过滤
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        
        PointType searchPoint = extractedCloud->points[i];
        int neighbors = kdtree->radiusSearch(searchPoint, discreteNoiseSearchRadius, 
                                             pointIdxRadiusSearch, pointRadiusSquaredDistance);
        
        if (neighbors < discreteNoiseMinNeighbors)
        {
            cloudNeighborPicked[i] = 1;  // 邻居点太少，标记为噪点
        }
        else
        {
            validPoints++;
        }
    }
    
    RCLCPP_INFO(get_logger(), "Discrete noise filter: %d points excluded, %d points remain", 
               extractedCloud->points.size() - validPoints, validPoints);
}
```

### 4. 集成方式
在`laserCloudInfoHandler`函数中，按以下顺序调用：
1. `calculateSmoothness()` - 计算曲率
2. `markOccludedPoints()` - 标记遮挡点
3. `filterDiscreteNoise()` - **新增：过滤离散噪点**
4. `extractFeatures()` - 提取特征
5. `publishFeatureCloud()` - 发布特征点云

## 预期效果

### 过滤效果
- **距离过滤**: 排除过近和过远的异常点
- **强度过滤**: 排除反射异常的噪点
- **密度过滤**: 排除孤立的离散点

### 性能影响
- 增加少量计算开销（KD树搜索）
- 提高地图质量和稳定性
- 减少后续处理的计算量

## 参数调优建议

### 保守设置（推荐初始值）
```yaml
discreteNoiseMinNeighbors: 3
discreteNoiseSearchRadius: 0.5
discreteNoiseMinRange: 1.0
discreteNoiseMaxRange: 30.0
```

### 严格设置（噪点较多时使用）
```yaml
discreteNoiseMinNeighbors: 8
discreteNoiseSearchRadius: 0.2
discreteNoiseMinRange: 0.5
discreteNoiseMaxRange: 25.0
```

### 宽松设置（避免误过滤）
```yaml
discreteNoiseMinNeighbors: 2
discreteNoiseSearchRadius: 0.8
discreteNoiseMinRange: 0.3
discreteNoiseMaxRange: 50.0
```

## 验证方法

### 1. 日志验证
- 查看控制台输出过滤统计信息
- 确认过滤前后点云数量变化

### 2. 可视化验证
- 使用RViz观察过滤后的点云
- 对比过滤前后的地图质量

### 3. 性能监控
- 监控处理延迟增加情况
- 确保实时性要求满足

## 注意事项

1. **参数调整**: 根据实际环境和传感器特性调整参数
2. **性能平衡**: 在过滤效果和计算效率之间找到平衡
3. **误过滤**: 避免过度过滤导致有效信息丢失
4. **实时监控**: 定期检查过滤效果和系统性能