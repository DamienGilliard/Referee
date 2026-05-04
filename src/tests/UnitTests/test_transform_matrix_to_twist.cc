#include "../../Referee.hh"

int main()
{
    Eigen::Matrix4d translationMatrix = Eigen::Matrix4d::Identity();
    translationMatrix(0, 3) = 1.0;
    translationMatrix(1, 3) = 1.0;
    translationMatrix(2, 3) = 1.0;
    Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix4d expectedRotationMatrix = Eigen::Matrix4d::Identity();
    expectedRotationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    
    Eigen::Matrix4d expectedTransformationMatrix = translationMatrix * expectedRotationMatrix;

    Eigen::Matrix<double, 6, 1> convertedTwist = Referee::Utils::Conversions::transformMatrixToTwist(expectedTransformationMatrix);
    std::cout << "Converted twist:\n" << convertedTwist.transpose() << std::endl;
    if (std::abs(convertedTwist(0) - 1.0) > 1e-6 
        || std::abs(convertedTwist(1) - 1.0) > 1e-6 
        || std::abs(convertedTwist(2) - 1.0) > 1e-6)
    {
        std::cerr << "Error: Incorrect translation in converted twist." << std::endl;
        return 1;
    }
    
    if (std::abs(convertedTwist(3) - 0.0) > 1e-6 
        || std::abs(convertedTwist(4) - 0.0) > 1e-6 
        || std::abs(convertedTwist(5) - M_PI / 4) > 1e-6)
    {
        std::cerr << "Error: Incorrect rotation in converted twist." << std::endl;
        return 1;
    }
    return 0;
}