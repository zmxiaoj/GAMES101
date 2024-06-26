#include<cmath>
#include<iostream>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>

int main()
{
    Eigen::Vector3d p(2, 1, 1);
    Eigen::Isometry2d T = Eigen::Isometry2d::Identity();

    Eigen::Rotation2D<double> rot(M_PI/4);
    T.rotate(rot);
    std::cout << "T: \n" << T.matrix() << std::endl;

    T.pretranslate(Eigen::Vector2d(1, 2));
    std::cout << "T: \n" << T.matrix() << std::endl;

    Eigen::Vector3d p2 = T * p;
    std::cout << "Result: \n" << p2.transpose() << std::endl;
    return 0;
}