/**
 * @file BuildPriorMap.h
 * @author wgq
 * @brief
 * @version 0.1
 * @date 2022-04-11
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef BUILDPRIORMAP_H
#define OPTIMIZER_H

#include <string>
#include <iostream>
#include <set>
#include <fstream>
#include <sstream>
#include <liblas/liblas.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h> //使用OMP需要添加的头文件
#include <pcl/filters/passthrough.h>    //分块使用

namespace PCL_TOOLS
{

    class BuildPriorMap
    {
    public:
        BuildPriorMap(std::string rootfile, std::string PriorMapDir);
        std::string rootfile;                                   //原始的las地图路径
        std::string PriorMapDir;                                //处理后的地图存储路径
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr;      //所有的点云地图
        pcl::PointCloud<pcl::Normal>::Ptr pointCloudNormalsPtr; //所有的点云地图的法向量

    public:
        void readbin2pcd();                                                                                                 // TODO 读取bin格式的转化为pcd
        void MapPointXYZFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr filterptr, float x = 0.2, float y = 0.2, float z = 0.2); //降采样
        void readlas2pcl(bool bSaveAllPCD = false);                                                                         // las格式转换为pcl格式
        void readpcd2pcl(bool bSaveAllPCD = false);                                                                         // TODO 使用ground truth位姿实现 pcd 点云拼接
        void BuildNormalsMap(pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr, pcl::PointCloud<pcl::Normal>::Ptr NormalPtr); //根据PointXYZ点云生成pcl::Normal点云地图
        void SavePriorMap(unsigned int mapCubeSize = 50);                                                                   //按照方格坐标切分保存点云地图
    };
}
#endif