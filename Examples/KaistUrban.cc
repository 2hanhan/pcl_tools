#include <iostream>
#include <BuildPriorMap.h>
int main(int agrc, char **argv)
{
    // step 1 读取lasmap
    std::string rootfile = "/home/wgq/urban39/";
    std::string PriorMapDir = "/home/wgq/urban39/prior_map/";
    PCL_TOOLS::BuildPriorMap kaisturban(rootfile, PriorMapDir);
    kaisturban.readlas2pcl(false, true);
    // kaisturban.readlas2pcl(true, false);
    //  step 2 分块保存
    kaisturban.SavePriorMap(50);
    std::cout << "save  successed!" << std::endl;

    return 0;
}