#include <BuildPriorMap.h>

namespace PCL_TOOLS
{
    BuildPriorMap::BuildPriorMap(std::string rootfile, std::string PriorMapDir) : rootfile(rootfile), PriorMapDir(PriorMapDir), pointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>), pointCloudNormalsPtr(new pcl::PointCloud<pcl::Normal>)
    {
        std::cout << "rootfile:" << rootfile << std::endl;
        std::cout << "PriorMapDir:" << PriorMapDir << std::endl;
    }

    void BuildPriorMap::MapPointXYZFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr filterptr, float x, float y, float z)
    {

        pcl::VoxelGrid<pcl::PointXYZ> filter; //创建滤波器
        filter.setInputCloud(filterptr);      //输入要滤波的点云
        filter.setLeafSize(x, y, z);          //用0.2 x 0.2 x 0.2的立方体对点云进行稀疏化
        filter.filter(*filterptr);            //获得滤波结果
    }

    void BuildPriorMap::readlas2pcl(bool bSaveAllPCD)
    {
        std::string lasfile = rootfile + "sick_pointcloud.las";
        std::ifstream inFile;
        inFile.open(lasfile, std::ios::in | std::ios::binary);

        liblas::ReaderFactory readLas;
        liblas::Reader reader = readLas.CreateWithStream(inFile);
        liblas::Header const &header = reader.GetHeader();

        std::cout << "Compressed: " << (header.Compressed() == true ? "True" : "False") << std::endl;
        std::cout << "Signature: " << header.GetFileSignature() << std::endl;

        int count = header.GetPointRecordsCount();

        while (reader.ReadNextPoint())
        {
            liblas::Point const &point = reader.GetPoint();
            double x, y, z;
            x = point.GetX();
            y = point.GetY();
            z = point.GetZ();

            pcl::PointXYZ p(x, y, z);
            pointCloudPtr->push_back(p);
        }
        std::cout << "las2pcd :successed!  Point num:" << count << std::endl;
        // MapPointXYZFilter(pointCloudPtr, 1, 1, 1);            //滤波
        BuildNormalsMap(pointCloudPtr, pointCloudNormalsPtr); //构建法向量地图

        if (bSaveAllPCD)
        {
            std::string pcdfile = PriorMapDir + "allpointCloud.pcd";
            pcl::io::savePCDFileASCII(pcdfile, *pointCloudPtr);
            std::cout << "allpointCloud.cd save:" << pcdfile << std::endl;

            pcdfile = PriorMapDir + "allnoramls.pcd";
            pcl::io::savePCDFileASCII(pcdfile, *pointCloudNormalsPtr);
            std::cout << "allnoramls.cd save:" << pcdfile << std::endl;
        }

        inFile.close();
    }

    void BuildPriorMap::BuildNormalsMap(pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr, pcl::PointCloud<pcl::Normal>::Ptr NormalPtr)
    {
        //计算法线
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n; // OMP加速
        //建立kdtree来进行近邻点集搜索
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        n.setNumberOfThreads(10); //设置openMP的线程数
        // n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
        n.setInputCloud(PointXYZPtr);
        n.setSearchMethod(tree);
        n.setKSearch(20); //点云法向计算时，需要所搜的近邻点大小
        // n.setRadiusSearch(0.03);//半径搜素
        n.compute(*NormalPtr); //开始进行法向计
    }

    void BuildPriorMap::SavePriorMap(unsigned int mapCubeSize)
    {
        // step 1 写构造config 保存分块大小，以及初始化地图是加载哪几块
        std::ofstream outFile;
        outFile.open(PriorMapDir + "config.txt", std::ios::out);
        outFile << mapCubeSize << std::endl;

        // step 2 分块保存地图
        std::set<std::pair<int, int>> keys; //分块地图的索引

        pcl::PointXYZ min, max;
        pcl::getMinMax3D(*pointCloudPtr, min, max);
        std::cout << "x:[" << min.x << "," << max.x << "]" << std::endl;
        std::cout << "y:[" << min.y << "," << max.y << "]" << std::endl;
        std::cout << "z:[" << min.z << "," << max.z << "]" << std::endl;

        //计算keys范围
        std::pair<int, int> key_x, key_y;
        key_x.first = std::floor(min.x / mapCubeSize);
        key_x.second = std::floor(max.x / mapCubeSize);
        key_y.first = std::floor(min.y / mapCubeSize);
        key_y.second = std::floor(max.y / mapCubeSize);
        std::cout << "key_x:[" << key_x.first << "," << key_x.second << "]" << std::endl;
        std::cout << "key_y:[" << key_y.first << "," << key_y.second << "]" << std::endl;

        //保存索引范围与分块地图
        for (int index_i = key_x.first; index_i < key_x.second + 1; ++index_i)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr subPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PassThrough<pcl::PointXYZ> pass_x, pass_y;                             // 声明直通滤波
            pass_x.setInputCloud(pointCloudPtr);                                        // 传入点云数据
            pass_x.setFilterFieldName("x");                                             // 设置操作的坐标轴
            pass_x.setFilterLimits(index_i * mapCubeSize, (index_i + 1) * mapCubeSize); // 设置坐标范围
            // pass_x.setFilterLimitsNegative(true);//保存范围内or范围外
            for (int index_j = key_y.first; index_j < key_y.second + 1; ++index_j)
            {
                // TODO 分块保存点云地图

                pass_x.filter(*subPointCloudPtr); // 进行滤波输出

                pass_y.setInputCloud(subPointCloudPtr);                                     // 传入点云数据
                pass_y.setFilterFieldName("y");                                             // 设置操作的坐标轴
                pass_y.setFilterLimits(index_j * mapCubeSize, (index_j + 1) * mapCubeSize); // 设置坐标范围
                // pass_y.setFilterLimitsNegative(true);//保存范围内or范围外

                pass_y.filter(*subPointCloudPtr); // 进行滤波输出

                if (!(*subPointCloudPtr).empty())
                {
                    //保存索引
                    std::pair<int, int> key;
                    key.first = index_i;
                    key.second = index_j;
                    outFile << index_i << std::endl
                            << index_j << std::endl;
                    keys.insert(key);

                    std::string subPointCloudFile = PriorMapDir + std::to_string(mapCubeSize) + "_" + std::to_string(index_i) + "_" + std::to_string(index_j) + "_points.pcd"; //分块的3D点云pcd<cubesize_indexi_indexj_point.pcd>
                    pcl::io::savePCDFileASCII(subPointCloudFile, *subPointCloudPtr);
                    std::cout << "sub PointCloud save:" << subPointCloudFile << std::endl;

                    pcl::PointCloud<pcl::Normal>::Ptr subnormalsPtr(new pcl::PointCloud<pcl::Normal>);
                    BuildNormalsMap(subPointCloudPtr, subnormalsPtr);

                    std::string subNormalsFile = PriorMapDir + std::to_string(mapCubeSize) + "_" + std::to_string(index_i) + "_" + std::to_string(index_j) + "_normals.pcd"; //分块的法向量
                    pcl::io::savePCDFileASCII(subNormalsFile, *subnormalsPtr);
                    std::cout << "sub NormalsFile save:" << subNormalsFile << std::endl;
                }
            }
        }
        outFile.close();
    }

}
