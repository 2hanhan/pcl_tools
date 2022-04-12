#include <ReadGroundTruth.h>

namespace PCL_TOOLS
{
    void ReadGroundTruth::GetFiles(
        std::string rootfile)
    {
        std::vector<double> timestamp;
        std::string groundtruthfile = rootfile + "/global_pose.csv";
        std::string lasmapfile = rootfile + "sick_pointcloud.las";
    }
}