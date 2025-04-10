#include "../../Referee.hh"
#include <Eigen/Dense>
#include <cmath>
int main()
{
    Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d inverseRotationMatrix = Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d translationVector(1, 0, 0);
    Eigen::Vector3d inverseTranslationVector(-std::cos(M_PI/4) , -std::cos(M_PI/4), 0);

    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    transformationMatrix.block<3, 1>(0, 3) = translationVector;
    Referee::Mapping::ChaslesTransformation transformation(transformationMatrix);

    Eigen::Matrix4d inverseTransformationMatrix = Eigen::Matrix4d::Identity();
    inverseTransformationMatrix.block<3, 3>(0, 0) = inverseRotationMatrix;
    inverseTransformationMatrix.block<3, 1>(0, 3) = inverseTranslationVector;
    Referee::Mapping::ChaslesTransformation inverseTransformation(inverseTransformationMatrix);

    Referee::Mapping::MappingMatrix mappingMatrix(2);
    mappingMatrix.SetChaslesTransformation(0, 1, transformation);
    mappingMatrix.SetChaslesTransformation(1, 0, inverseTransformation);
    mappingMatrix.SetChaslesTransformation(0, 0, inverseTransformation);
    mappingMatrix.SetChaslesTransformation(1, 1, transformation);

    mappingMatrix.CalculateMeanTransformationMatrices();

    if(mappingMatrix.GetMeanChaslesTransformation(0).GetRotationAngle() < 1e-6 &&
       mappingMatrix.GetMeanChaslesTransformation(0).GetTranslation().norm() < 1e-6)
    {
        std::cout << "Test passed: Mean transformation matrice is correct" << std::endl;
        return 0;
    }
    else
    {
        mappingMatrix.GetMeanChaslesTransformation(0).PrintTransformation();
        std::cout << "Test failed: Mean transformation matrice is not correct" << std::endl;
        return 1;
    }

}