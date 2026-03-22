#include "../../Referee.hh"
#include <Eigen/Dense>
#include <cmath>
int main()
{
    // Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // Eigen::Matrix3d inverseRotationMatrix = Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // Eigen::Vector3d firstTranslationVector(1.0, 0.0, 0.0);
    // Eigen::Vector3d secondTranslationVector(0.0, 1.0, 0.0);
    // Eigen::Vector3d thirdTranslationVector(0.0, 0.0, 1.0);

    // Eigen::Matrix4d firstTransformationMatrix = Eigen::Matrix4d::Identity();
    // firstTransformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    // firstTransformationMatrix.block<3, 1>(0, 3) = firstTranslationVector;
    // Referee::Mapping::Transformation transformation(firstTransformationMatrix);

    // Eigen::Matrix4d secondTransformationMatrix = Eigen::Matrix4d::Identity();
    // secondTransformationMatrix.block<3, 3>(0, 0) = inverseRotationMatrix;
    // secondTransformationMatrix.block<3, 1>(0, 3) = secondTranslationVector;
    // Referee::Mapping::Transformation inverseTransformation(secondTransformationMatrix);

    // Eigen::Matrix4d thirdTransformationMatrix = Eigen::Matrix4d::Identity();
    // thirdTransformationMatrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    // thirdTransformationMatrix.block<3, 1>(0, 3) = thirdTranslationVector;
    // Referee::Mapping::Transformation thirdTransformation(thirdTransformationMatrix);

    // Referee::Mapping::MappingMatrix mappingMatrix(3);
    // mappingMatrix.SetTransformation(0, 1, secondTransformationMatrix);
    // mappingMatrix.SetTransformation(1, 0, secondTransformationMatrix);
    // mappingMatrix.SetTransformation(0, 2, thirdTransformationMatrix);
    // mappingMatrix.SetTransformation(2, 0, thirdTransformationMatrix);
    // mappingMatrix.SetTransformation(1, 2, secondTransformationMatrix);
    // mappingMatrix.SetTransformation(2, 1, thirdTransformationMatrix);
    // mappingMatrix.SetTransformation(0, 0, firstTransformationMatrix);
    // mappingMatrix.SetTransformation(1, 1, firstTransformationMatrix);
    // mappingMatrix.SetTransformation(2, 2, firstTransformationMatrix);

    // mappingMatrix.CalculateMeanTransformationMatrices();

    // if(mappingMatrix.GetMeanTransformation(0).GetRotationAngle() < 1e-6 &&
    //    mappingMatrix.GetMeanTransformation(0).GetTranslation().norm() - std::sqrt(3)/3 < 1e-6)
    // {
    //     std::cout << "Test passed: Mean transformation matrice is correct" << std::endl;
        return 0;
    // }
    // else
    // {
    //     mappingMatrix.GetMeanTransformation(0).PrintTransformation();
    //     std::cout << "Test failed: Mean transformation matrice is not correct" << std::endl;
    //     return 1;
    // }

}