#include "../../Referee.hh"

#include <Eigen/Geometry>
#include <Eigen/Dense>

int main()
{
    Eigen::AngleAxisd angleAxis( M_PI / 4, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d rotationMatrix = angleAxis.toRotationMatrix();
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;

    Referee::Mapping::ChaslesTransformation chaslesTransformation(transformationMatrix);
    Eigen::Vector3d rotationAxis = chaslesTransformation.GetRotationAxis();
    Eigen::Vector3d pointOnAxis = chaslesTransformation.GetPointOnAxis();
    double rotationAngle = chaslesTransformation.GetRotationAngle();

    if(rotationAxis.isApprox(Eigen::Vector3d(0, 0, M_PI / 4), 1e-6) && pointOnAxis.isApprox(Eigen::Vector3d(0, 0, 0), 1e-6) && std::abs(rotationAngle - M_PI / 4) < 1e-6)
    {
        std::cout << "Test passed: Chasles transformation is correct" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Chasles transformation is not correct" << std::endl;
        return 1;
    }
    
}