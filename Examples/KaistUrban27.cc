#include <iostream>
#include <ReadGroundTruth.h>
#include <BuildPriorMap.h>
int main(int agrc, char **argv)
{
    // step 1 读取lasmap
    std::string rootfile = "/home/wgq/urban27/";
    std::string PriorMapDir = "/home/wgq/urban27/prior_map/";
    PCL_TOOLS::BuildPriorMap kaisturban27(rootfile, PriorMapDir);
    kaisturban27.readlas2pcl(false, true);
    // kaisturban27.readlas2pcl(true, false);
    //  step 2 分块保存
    kaisturban27.SavePriorMap(50);
    std::cout << "save  successed!" << std::endl;

    return 0;
}