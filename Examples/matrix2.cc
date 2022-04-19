#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

int main()
{

    Eigen::Matrix4d V2Left;
    Eigen::Matrix4d V2imu;
    // KAIST URBAN27

    V2Left << -0.00680499, -0.0153215, 0.99985, 1.64239,
        -0.999977, 0.000334627, -0.00680066, 0.247401,
        -0.000230383, -0.999883, -0.0153234, 1.58411,
        0, 0, 0, 1;
    V2imu << 1, 0, 0, -0.07,
        0, 1, 0, 0,
        0, 0, 1, 1.7,
        0, 0, 0, 1;

    // lefl2imu
    std::cout << "lefl2imu:" << std::endl
              << V2imu * V2Left.inverse() << std::endl;

    // KAIST URBAN39
    V2Left << -0.00680499, -0.0153215, 0.99985, 1.64239,
        -0.999977, 0.000334627, -0.00680066, 0.247401,
        -0.000230383, -0.999883, -0.0153234, 1.58411,
        0, 0, 0, 1;

    V2imu << 1, 0, 0, -0.07,
        0, 1, 0, 0,
        0, 0, 1, 1.7,
        0, 0, 0, 1;

    // lefl2imu
    std::cout << "lefl2imu:" << std::endl
              << V2imu * V2Left.inverse() << std::endl;
    return 0;
}