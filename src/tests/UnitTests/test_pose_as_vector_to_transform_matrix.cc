#include "../../Referee.hh"

int main()
{
    Eigen::Matrix<double, 6, 1> poseAsVector;
    poseAsVector[0] = 1.0;
    poseAsVector[1] = 1.0;
    poseAsVector[2] = 1.0;
    poseAsVector[3] = 0.0;
    poseAsVector[4] = 0.0;
    poseAsVector[5] = 0.0;

    Eigen::Matrix4d transformationMatrix = Referee::Utils::Conversions::poseAsVectorToTransformationMatrix(poseAsVector);
    std::cout << "Transformation matrix:\n" << transformationMatrix << std::endl;
    if (transformationMatrix(0, 3) != 1.0 || transformationMatrix(1, 3) != 1.0 || transformationMatrix(2, 3) != 1.0)
    {
        std::cerr << "Error: Incorrect translation in transformation matrix." << std::endl;
        return 1;
    }

    poseAsVector[5] = M_PI / 2; // 90 degrees rotation around Z axis

    Eigen::Matrix4d translationMatrix = Eigen::Matrix4d::Identity();
    translationMatrix(0, 3) = 1.0;
    translationMatrix(1, 3) = 1.0;
    translationMatrix(2, 3) = 1.0;
    Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(poseAsVector[5], Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix4d expectedRotationMatrix = Eigen::Matrix4d::Identity();
    expectedRotationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    
    Eigen::Matrix4d expectedTransformationMatrix = translationMatrix * expectedRotationMatrix;
    std::cout << "Expected transformation matrix:\n" << expectedTransformationMatrix << std::endl;
    Eigen::Matrix4d convertedTransformationMatrix = Referee::Utils::Conversions::poseAsVectorToTransformationMatrix(poseAsVector);
    std::cout << "Transformation matrix with rotation:\n" << convertedTransformationMatrix << std::endl;
    if (!convertedTransformationMatrix.isApprox(expectedTransformationMatrix, 1e-6))
    {
        std::cerr << "Error: Transformation matrix does not match expected transformation matrix." << std::endl;
        return 1;
    }
    
    return 0;
}
