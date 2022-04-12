/**
 * @file ReadGroundTruth.h
 * @author wgq
 * @brief
 * @version 0.1
 * @date 2022-04-11
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef READGROUNDTURTH_H
#define OPTIMIZER_H

#include <vector>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>

// TODO 读取ground truth的坐标存储到eigen3中
namespace PCL_TOOLS
{

    class ReadGroundTruth
    {
    public:
        void GetFiles(std::string rootfile);

    public:
        std::vector<double> timestamp;
        std::string groundtruthfile;
        std::string lasmapfile;
    };
}

#endif