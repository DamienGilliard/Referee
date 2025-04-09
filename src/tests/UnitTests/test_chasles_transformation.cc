#include "../../Referee.hh"

#include <Eigen/Geometry>
#include <Eigen/Dense>

int main()
{
    Eigen::AngleAxisd angleAxis( M_PI / 2, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d rotationMatrix = angleAxis.toRotationMatrix();
    Eigen::Vector3d translationVector(100, 0, 0);
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    transformationMatrix.block<3, 1>(0, 3) = translationVector;

    Referee::Mapping::ChaslesTransformation chaslesTransformation(transformationMatrix);
    Eigen::Vector3d rotationAxis = chaslesTransformation.GetRotationAxis();
    Eigen::Vector3d pointOnAxis = chaslesTransformation.GetPointOnAxis();
    double rotationAngle = chaslesTransformation.GetRotationAngle();

    if(rotationAxis.isApprox(Eigen::Vector3d(0, 0, M_PI / 2), 1e-6) && pointOnAxis.isApprox(Eigen::Vector3d(50, 50, 0), 1e-6) && std::abs(rotationAngle - M_PI / 2) < 1e-6)
    {
        std::cout << "Test passed: Chasles transformation is correct" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Chasles transformation is not correct" << std::endl;
        std::cout << "Rotation axis: " << rotationAxis.transpose() << std::endl;
        std::cout << "Point on axis: " << pointOnAxis.transpose() << std::endl;
        std::cout << "Rotation angle: " << rotationAngle << std::endl;
        std::cout << "Expected rotation axis: " << Eigen::Vector3d(0, 0, M_PI / 2).transpose() << std::endl;
        std::cout << "Expected point on axis: " << Eigen::Vector3d(50, 50, 0).transpose() << std::endl;
        std::cout << "Expected rotation angle: " << M_PI / 4 << std::endl;
        return 1;
    }
    
}