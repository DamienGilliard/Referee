#include "../../Referee.hh"

int main()
{
    Eigen::Vector3d originalPose(1, 0, 0);
    Eigen::Vector3d rotationAxis(0, 0, 1);
    double rotationAngle = M_PI / 2; // 90 degrees
    Eigen::Vector3d translation(1, -1, 0);
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = Eigen::AngleAxisd(rotationAngle, rotationAxis).toRotationMatrix();
    transformationMatrix.block<3, 1>(0, 3) = translation;
    Eigen::Vector3d resultingTranslation = Referee::Transformations::CalculateResultingTranslation(transformationMatrix, originalPose);
    Eigen::Vector3d expectedTranslation(0, 0, 0);

    if ((resultingTranslation - expectedTranslation).norm() < 1e-6)
    {
        std::cout << "Test passed: Resulting translation is correct" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Resulting translation is not correct" << std::endl;
        std::cout << "Resulting translation: " << resultingTranslation.transpose() << std::endl;
        return 1;
    }
}