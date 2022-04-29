#include <BuildPriorMap.h>

//模板函数：将string类型变量转换为常用的数值类型（此方法具有普遍适用性）
template <class Type>
Type stringToNum(const std::string &str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

bool computePairNum(std::string pair1, std::string pair2)
{
    return pair1 < pair2;
}

namespace PCL_TOOLS
{
    /**
     * @brief Construct a new Build Prior Map:: Build Prior Map object
     *
     * @param rootfile urban格式数据集根目录
     * @param PriorMapDir PCD格式先验地图输出保存路径
     */
    BuildPriorMap::BuildPriorMap(std::string rootfile, std::string PriorMapDir) : rootfile(rootfile), PriorMapDir(PriorMapDir), pointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>), pointCloudNormalsPtr(new pcl::PointCloud<pcl::Normal>)
    {
        std::cout << "rootfile:" << rootfile << std::endl;
        std::cout << "PriorMapDir:" << PriorMapDir << std::endl;
        GroundTruth = rootfile + "07.txt";
        MapbinDir = rootfile + "velodyne/";
        MappcdDir = rootfile + "pcd/";
    }

    /**
     * @brief
     *
     * @param dir 文件夹路径
     * @param Suffix 要获取的文件后缀名
     * @param files 获取文件路径名称存储向量
     */
    void BuildPriorMap::LoadFiles(std::string DirPath, std::string Suffix, std::vector<std::string> &files)
    {
        std::cout << "DirPath : " << DirPath << "  get[*." << Suffix << "]files";
        std::vector<std::string> file_lists;
        // step 1 获取指定后缀的所有文件
        struct dirent *ptr;
        DIR *dir;
        dir = opendir(DirPath.c_str());
        file_lists.clear();
        while ((ptr = readdir(dir)) != NULL)
        {
            std::string tmp_file = ptr->d_name;
            if (tmp_file[0] == '.')
            { //取读取文件的第一个字符，如果为.则说明是隐藏文件不读取
                continue;
            }
            if (Suffix.size() <= 0)
            { //如果无传入的类型，则该路径下所有文件都读取
                file_lists.push_back(ptr->d_name);
            }
            else
            {
                if (tmp_file.size() < Suffix.size())
                {
                    //说明不可能，也就是文件名的长度小于后缀名这3长度
                    continue;
                }
                std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - Suffix.size(), Suffix.size()); //截取后三位
                if (tmp_cut_type == Suffix)
                { //如果截取文件名的后三位和传入的类型一致的话，添加这个文件的全称包括后缀
                    file_lists.push_back(ptr->d_name);
                }
            }
        }
        // step 2 对指定文件进行排序
        if (file_lists.empty())
            return;
        std::sort(file_lists.begin(), file_lists.end(), computePairNum); //按照computePairNum函数的方法进行排序

        // step 3 保存排序结果
        for (int i = 0; i < file_lists.size(); ++i)
        {
            std::string file = file_lists[i];
            files.push_back(file); //将文件名字传入到pcd_filename里
        }
        std::cout << "num:" << files.size() << std::endl;
    }

    /**
     * @brief  PointXYZ地图进行下采样滤波
     *
     * @param filterptr 输入地图
     * @param x 滤波网格大小
     * @param y 滤波网格大小
     * @param z 滤波网格大小
     */
    void BuildPriorMap::MapPointXYZFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr filterptr, float x, float y, float z)
    {

        pcl::VoxelGrid<pcl::PointXYZ> filter; //创建滤波器
        filter.setInputCloud(filterptr);      //输入要滤波的点云
        filter.setLeafSize(x, y, z);          //用0.2 x 0.2 x 0.2的立方体对点云进行稀疏化
        filter.filter(*filterptr);            //获得滤波结果
    }

    /**
     * @brief 读取las数据格式转换为pcd格式
     *
     * @param bSaveAllPCD 是否保存完整的pcd格式地图
     * @param pose2Identity 是否对初始位姿进行坐标变换
     */
    void BuildPriorMap::readlas2pcl(bool bSaveAllPCD, bool pose2Identity)
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
        // MapPointXYZFilter(pointCloudPtr, 1, 1, 1); //滤波

        //初始位姿转换到单位矩阵
        Eigen::Matrix4f T_init = Eigen::Matrix4f::Identity();
        if (pose2Identity)
        {
            std::ifstream inFilecsv(rootfile + "global_pose.csv", std::ios::in);
            std::string initPosestring;
            //读取csv以,为分割
            std::getline(inFilecsv, initPosestring, '\n'); //取出一行
            std::stringstream ss(initPosestring);
            std::string Pose_i;
            std::vector<float> Pose;
            while (std::getline(ss, Pose_i, ','))
            {
                float temp;
                temp = stringToNum<double>(Pose_i);
                Pose.push_back(temp);
            }
            T_init(0, 0) = Pose[1];
            T_init(0, 1) = Pose[2];
            T_init(0, 2) = Pose[3];
            T_init(0, 3) = Pose[4];

            T_init(1, 0) = Pose[5];
            T_init(1, 1) = Pose[6];
            T_init(1, 2) = Pose[7];
            T_init(1, 3) = Pose[8];

            T_init(2, 0) = Pose[9];
            T_init(2, 1) = Pose[10];
            T_init(2, 2) = Pose[11];
            T_init(2, 3) = Pose[12];

            Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
            // z轴旋转90°
            trans << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

            // Urban39
            Eigen::Matrix4f Vehicle2IMU;
            Vehicle2IMU << 1, 0, 0, -0.07,
                0, 1, 0, 0,
                0, 0, 1, 1.7,
                0, 0, 0, 1;

            std::cout << "point[0] pose before:" << (*pointCloudPtr)[0] << std::endl;
            std::cout << "T_init:" << T_init << std::endl;
            pcl::transformPointCloud(*pointCloudPtr, *pointCloudPtr, Vehicle2IMU.inverse() * T_init.inverse()); //将点云进行旋转平移变换到IMU坐标系
            std::cout << "point[0] pose after:" << (*pointCloudPtr)[0] << std::endl;
            std::cout << "pose2Identity succeed" << std::endl;
            inFilecsv.close();
        }

        BuildNormalsMap(pointCloudPtr, pointCloudNormalsPtr); //构建法向量地图

        /*
        {
            for (int index_i = -1; index_i < 26; ++index_i)
            {
                for (int index_j = -22; index_j < 2; ++index_j)
                {
                    for (int j = 0; j < 100; j++)
                    {
                        if (index_i < 0 || index_j < 0)
                        {
                            pcl::PointXYZ z(index_i * 50, index_j * 50, -j - 10);
                            pointCloudPtr->push_back(z);
                        }
                        else
                        {
                            pcl::PointXYZ z(index_i * 50, index_j * 50, j + 10);
                            pointCloudPtr->push_back(z);
                        }
                    }
                }
            }
        }
        */

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
        std::cout << "las2pcl finished!" << std::endl;
    }

    /**
     * @brief
     *
     * @param _MapbinDir bin格式地图路径
     * @param _MappcdDir 保存pcd格式地图路径
     */
    void BuildPriorMap::bin2pcd(std::string _MapbinDir, std::string _MappcdDir)
    {
        std::cout << "load bin map from: " << _MapbinDir << " and save pcd map" << std::endl;
        //设置路径
        MapbinDir = _MapbinDir;
        MappcdDir = _MappcdDir;
        //获取文件名称
        std::vector<std::string> binfiles;
        std::string suffix = "bin";
        LoadFiles(MapbinDir, suffix, binfiles);
        //读取bin
        int32_t num = 1000000;
        int32_t file_size;
        float *data;
        pcl::PointCloud<pcl::PointXYZI> point_cloud;
        for (int i = 0; i < binfiles.size(); ++i)
        {
            data = (float *)malloc(num * sizeof(float));
            // pointers
            float *px = data + 0;
            float *py = data + 1;
            float *pz = data + 2;
            float *pr = data + 3;

            std::string binfile = MapbinDir + binfiles[i]; //存储路径 + 文件名
            std::cout << "binfile:[" << binfile << "]  ";
            std::fstream a_file(binfile.c_str(), std::ios::binary | std::ios::in | std::ios::ate);
            file_size = a_file.tellg();
            std::cout << "file size: " << file_size << "  ";
            a_file.seekg(0, std::ios::beg);
            if (!a_file.read(reinterpret_cast<char *>(data), file_size))
            {
                std::cout << "Error reading from file" << std::endl;
                return;
            }
            point_cloud.clear();
            point_cloud.width = (file_size / sizeof(float)) / 4;
            point_cloud.height = 1;
            point_cloud.is_dense = false;
            point_cloud.points.resize(point_cloud.width * point_cloud.height);
            std::cout << "resized to " << point_cloud.points.size() << std::endl;
            // fill in the point cloud
            for (int j = 0; j < point_cloud.points.size(); ++j)
            {
                point_cloud.points[j].x = *px;
                point_cloud.points[j].y = *py;
                point_cloud.points[j].z = *pz;
                point_cloud.points[j].intensity = *pr;
                px += 4;
                py += 4;
                pz += 4;
                pr += 4;
            }
            //保存pcd格式
            std::string name = binfiles[i].substr(0, binfiles[i].size() - suffix.size() - 1); //去掉后缀名三位
            std::string pcdfile = MappcdDir + name + ".pcd";
            std::cout << "save:[" << pcdfile << "]" << std::endl;
            pcl::io::savePCDFileASCII(pcdfile, point_cloud);
        }
        std::cout << "bin2pcd finished!" << std::endl;
    }

    /**
     * @brief
     *
     * @param GroundTruth  位姿真值的 txt
     * @param bSaveAllPCD  是否保存拼接后的地图
     */
    void BuildPriorMap::readpcd2pcl(std::string _GroundTruth, bool bSaveAllPCD)
    {
        GroundTruth = _GroundTruth;
        //获取文件名称
        std::vector<std::string> pcdfiles;
        std::string suffix = "pcd";
        LoadFiles(MappcdDir, suffix, pcdfiles);

        //读取ground进行地图拼接
        std::ifstream FileIn(GroundTruth);
        std::cout << "GroundTruth:[" << GroundTruth << "]" << std::endl;

        Eigen::Matrix4f pose_GT;
        Eigen::Matrix4f velo2cam, cam2velo;

        //给两个变换矩阵赋初值
        cam2velo << 0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0.08,
            0, 0, 0, 1;

        velo2cam << 0, -1, 0, 0,
            0, 0, -1, 0,
            1, 0, 0, -0.08,
            0, 0, 0, 1;

        int i = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

        while (FileIn >> pose_GT(0, 0) >> pose_GT(0, 1) >> pose_GT(0, 2) >> pose_GT(0, 3) >> pose_GT(1, 0) >> pose_GT(1, 1) >> pose_GT(1, 2) >> pose_GT(1, 3) >> pose_GT(2, 0) >> pose_GT(2, 1) >> pose_GT(2, 2) >> pose_GT(2, 3))
        {
            pose_GT(3, 0) = 0;
            pose_GT(3, 1) = 0;
            pose_GT(3, 2) = 0;
            pose_GT(3, 3) = 1;
            // pose_GT = cam2velo * pose_data2 * velo2cam; //激光坐标系下的
            pose_GT = pose_GT * velo2cam; //相机坐标系下的

            std::string pcdfile = MappcdDir + pcdfiles[i];
            std::cout << "load:[" << pcdfile << "]";

            pcl::io::loadPCDFile(pcdfile, *source); //读入pcd格式的文件
            std::cout << " !" << std::endl;

            //将点云按照transform[i]的变换矩阵进行旋转平移变换，最终存入target中
            MapPointXYZFilter(source, 0.2, 0.2, 0.2); //滤波
            pcl::transformPointCloud(*source, *target, pose_GT);
            //拼接

            *pointCloudPtr = *pointCloudPtr + *target;
            i++;
        }

        std::vector<int> indices_src; //保存去除的点的索引
        //移除 NaNs，从传感器获得的点云可能包含几种测量误差和/或不准确。其中之一是在一些点的坐标中存在NaN（不是数）值，
        pcl::removeNaNFromPointCloud(*pointCloudPtr, *pointCloudPtr, indices_src);

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
        FileIn.close();
        std::cout << "pcl2pcl finished!" << std::endl;
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

    /**
     * @brief  保存分块地图
     *
     * @param mapCubeSize 分块地图大小
     */
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
            pcl::PointCloud<pcl::PointXYZ>::Ptr subTempPtr(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PassThrough<pcl::PointXYZ> pass_x, pass_y;                                                                 // 声明直通滤波
            pass_x.setInputCloud(pointCloudPtr);                                                                            // 传入点云数据
            pass_x.setFilterFieldName("x");                                                                                 // 设置操作的坐标轴
            pass_x.setFilterLimits(static_cast<int>(index_i * mapCubeSize), static_cast<int>((index_i + 1) * mapCubeSize)); // 设置坐标范围
            // pass_x.setFilterLimitsNegative(true);//保存范围内or范围外

            pass_x.filter(*subTempPtr); // 进行滤波输出

            for (int index_j = key_y.first; index_j < key_y.second + 1; ++index_j)
            {

                pass_y.setInputCloud(subTempPtr);                                                                               // 传入点云数据
                pass_y.setFilterFieldName("y");                                                                                 // 设置操作的坐标轴
                pass_y.setFilterLimits(static_cast<int>(index_j * mapCubeSize), static_cast<int>((index_j + 1) * mapCubeSize)); // 设置坐标范围
                // pass_y.setFilterLimitsNegative(true);//保存范围内or范围外

                pass_y.filter(*subPointCloudPtr); // 进行滤波输出

                if (!(*subPointCloudPtr).empty())
                {
                    //保存索引
                    std::pair<int, int> key;
                    key.first = index_i;
                    key.second = index_j;
                    outFile << index_i << " " << index_j << std::endl;
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
