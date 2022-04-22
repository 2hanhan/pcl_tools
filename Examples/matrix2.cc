#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

int main()
{

    Eigen::Matrix4d Vehicle2Left;
    Eigen::Matrix4d Vehicle2IMU;
    Eigen::Matrix4d Vehicle2BackSick;
    Eigen::Matrix4d Left2IMU;

    //! KAIST 数据集给的标定文件Vehicle2IMU实际上是Timu2vehicle

    // KAIST URBAN27

    Vehicle2Left << -0.00680499, -0.0153215, 0.99985, 1.64239,
        -0.999977, 0.000334627, -0.00680066, 0.247401,
        -0.000230383, -0.999883, -0.0153234, 1.58411,
        0, 0, 0, 1;
    Vehicle2IMU << 1, 0, 0, -0.07,
        0, 1, 0, 0,
        0, 0, 1, 1.7,
        0, 0, 0, 1;

    // lefl2imu
    Left2IMU = Vehicle2IMU.inverse() * Vehicle2Left;
    std::cout
        << " 27 left2imu:" << std::endl
        << Left2IMU << std::endl
        << " 27 imu2left:" << std::endl
        << Left2IMU.inverse() << std::endl;

    // KAIST URBAN39
    Vehicle2Left << -0.00680499, -0.0153215, 0.99985, 1.64239,
        -0.999977, 0.000334627, -0.00680066, 0.247401,
        -0.000230383, -0.999883, -0.0153234, 1.58411,
        0, 0, 0, 1;

    Vehicle2IMU << 1, 0, 0, -0.07,
        0, 1, 0, 0,
        0, 0, 1, 1.7,
        0, 0, 0, 1;

    Vehicle2BackSick << -0.0116289, -0.710158, 0.703946, -0.5617,
        -0.999787, -0.0037592, -0.0203085, 0.0516457,
        0.0170685, -0.704032, -0.709963, 1.61702,
        0, 0, 0, 1;

    // lefl2imu
    Left2IMU = Vehicle2IMU.inverse() * Vehicle2Left;
    std::cout
        << " 39 left2imu:" << std::endl
        << Left2IMU << std::endl
        << " 39 imu2left:" << std::endl
        << Left2IMU.inverse() << std::endl;

    // BackSick

    return 0;
}