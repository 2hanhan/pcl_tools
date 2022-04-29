#include <iostream>
#include <BuildPriorMap.h>
int main(int agrc, char **argv)
{
    // step 1 读取bin2pcd
    std::string rootfile = "/home/wgq/datas/KITTI/07/";
    std::string PriorMapDir = "/home/wgq/datas/KITTI/07/prior_map/";
    std::string GroundTruth = "/home/wgq/datas/KITTI/07/07.txt";
    std::string MapbinDir = "/home/wgq/datas/KITTI/07/velodyne/";
    std::string MappcdDir = "/home/wgq/datas/KITTI/07/pcd/";
    PCL_TOOLS::BuildPriorMap kitti(rootfile, PriorMapDir);
    kitti.bin2pcd(MapbinDir, MappcdDir);
    //   step 2 根据GT拼接
    kitti.readpcd2pcl(GroundTruth, true);
    //   step 3 分块保存
    kitti.SavePriorMap(50);
    std::cout << "save  successed!" << std::endl;

    return 0;
}