# pcl_tools
学习`pcl`,使用`pcl`库的功能实现

# Install
## 1. Prerequisites
### 1.1 **Dependencies**
1. `Eigen3`Tested with Eigen 3.3.7.
2. `libLAS`Tested with libLAS 1.8.1.
3. `PCL`Tested with PCL 1.9.1.

# 功能介绍
## 读取groundtru(src/ReadGroundTruth.h)
待完善
1. 读取`ground truth`存储位姿到`eigen3`

## 构建先验地图(include/BuildPriorMap.h)
实现功能
1. 实现`pcl`读取`bin格式`并转化为`pcd格式`
2. 根据每个点云帧的位姿(使用ground truth或者估计的位姿也行)实现点云拼接
3. 读取`las格式`的点云地图转化到`pcl格式`并提供`pcd格式`保存
4. 实现`pcl::PointXYZ`点云地图的法向量`pcl::Normal`计算
5. 实现大规模点云的按照`x轴`和`y轴`进行分块存储




## BUG总结
### 无符号数
```c++
unsigned int mapCubeSize;
int index_i;
index_i * mapCubeSize;
static_cast<int>(index_i * mapCubeSize);
```
- 当`index_i`为**非负数**时候，`index_i * mapCubeSize;`与`static_cast<int>(index_i * mapCubeSize);`结果一样。
- 但是当`index_i`为**负数**时候，`index_i * mapCubeSize;`直接就是最大的正整数，然后分块索引就出错了。
### 点云坐标变化
```c++
Eigen::Matrix4f pose_GT;
pcl::transformPointCloud(*source, *target, pose_GT);
```
- `Eigen::Matrix4d pose_GT;`矩阵使用`doube`数据类型存储矩阵，后面`transformPointCloud`坐标变化就会出现段错误。
- 这个段错误`Release`编译会出现，但是`Debug`模式就不会报段错误。