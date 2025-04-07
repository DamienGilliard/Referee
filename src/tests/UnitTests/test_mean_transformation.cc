#include "../../Referee.hh"
#include <Eigen/Dense>
int main()
{
    Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d inverseRotationMatrix = Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d halfInverseRotationMatrix = Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d translationVector(1, 2, 3);
    Eigen::Vector3d inverseTranslationVector(-1, -2, -3);

    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    transformationMatrix.block<3, 1>(0, 3) = translationVector;

    Eigen::Matrix4d inverseTransformationMatrix = Eigen::Matrix4d::Identity();
    inverseTransformationMatrix.block<3, 3>(0, 0) = inverseRotationMatrix;
    inverseTransformationMatrix.block<3, 1>(0, 3) = inverseTranslationVector;

    Eigen::Matrix4d halfInverseTransformationMatrix = Eigen::Matrix4d::Identity();
    halfInverseTransformationMatrix.block<3, 3>(0, 0) = halfInverseRotationMatrix;
    halfInverseTransformationMatrix.block<3, 1>(0, 3) = Eigen::Vector3d(-0.5, -1.0, -1.5);
    Referee::Mapping::MappingMatrix mappingMatrix(2);
    mappingMatrix.SetTransformationMatrix(0, 1, transformationMatrix);
    mappingMatrix.SetTransformationMatrix(1, 0, inverseTransformationMatrix);
    mappingMatrix.SetTransformationMatrix(0, 0, inverseTransformationMatrix);
    mappingMatrix.SetTransformationMatrix(1, 1, Eigen::Matrix4d::Identity());

    mappingMatrix.CalculateMeanTransformationMatrices();

    if(mappingMatrix.GetMeanTransformationMatrix(0).isApprox(Eigen::Matrix4d::Identity(), 1e-6) && mappingMatrix.GetMeanTransformationMatrix(1).isApprox(halfInverseTransformationMatrix, 1e-6))
    {
        std::cout << "Test passed: Mean transformation matrices are correct" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Mean transformation matrices are not correct" << std::endl;
        return 1;
    }

}