#include "GeometricTransformation.hh"

namespace Referee::Transformations
{
    Eigen::Vector3d RecenterTranslationVectors(std::vector<Eigen::Vector3d> &translationVectors)
    {
        Eigen::Vector3d meanTranslationVector = Eigen::Vector3d::Zero();
        for (const auto& translationVector : translationVectors)
        {
            meanTranslationVector += translationVector;
        }
        meanTranslationVector /= static_cast<double>(translationVectors.size());

        for (auto& translationVector : translationVectors)
        {
            translationVector -= meanTranslationVector;
        }
        return meanTranslationVector;
    }

    std::vector<Eigen::Vector3d> CalculateMedianPlane(Eigen::Vector3d point1, Eigen::Vector3d point2)
    {
        std::vector<Eigen::Vector3d> planeOriginAndNormal;
        Eigen::Vector3d planeNormal = (point1 - point2).normalized();
        Eigen::Vector3d planeOrigin = (point1 + point2) / 2.0;
        planeOriginAndNormal.push_back(planeOrigin);
        planeOriginAndNormal.push_back(planeNormal);
        return planeOriginAndNormal;
    }

    Eigen::Vector3d CalculatePlaneIntersection(std::vector<Eigen::Vector3d> plane1, std::vector<Eigen::Vector3d> plane2, std::vector<Eigen::Vector3d> plane3)
    {
        Eigen::Vector3d plane1Origin = plane1[0];
        Eigen::Vector3d plane1Normal = plane1[1];
        Eigen::Vector3d plane2Origin = plane2[0];
        Eigen::Vector3d plane2Normal = plane2[1];
        Eigen::Vector3d plane3Origin = plane3[0];
        Eigen::Vector3d plane3Normal = plane3[1];

        Eigen::Matrix3d A;
        A.row(0) = plane1Normal.transpose();
        A.row(1) = plane2Normal.transpose();
        A.row(2) = plane3Normal.transpose();

        // If the normals are coplanar, replace the last row with (0, 0, 1)
        if (A.determinant() < 1e-5)
        {
            std::cout << "The normals are coplanar, replacing the last row with (0, 0, 1)" << std::endl;
            A.row(2) = Eigen::Vector3d(0, 0, 1);
            plane3Origin = Eigen::Vector3d(0, 0, 0);
        }

        Eigen::Vector3d b;
        b << plane1Origin.dot(plane1Normal), plane2Origin.dot(plane2Normal), plane3Origin.dot(plane3Normal);

        double conditionNumber = A.jacobiSvd().singularValues()(0) / A.jacobiSvd().singularValues()(2);
        if (conditionNumber > 1e12) // Threshold for ill-conditioning
        {
            std::cerr << "Matrix A is ill-conditioned with condition number: " << conditionNumber << std::endl;
            throw std::runtime_error("Cannot solve for intersection point due to ill-conditioned matrix.");
        }
        Eigen::Vector3d intersectionPoint = A.colPivHouseholderQr().solve(b);
        return intersectionPoint;
    }

    Eigen::Vector3d CalculateResultingTranslation(Eigen::Matrix4d transformationMatrix, Eigen::Vector3d poseOrigin)
    {
        Eigen::Matrix3d rotationMatrix = transformationMatrix.block<3, 3>(0, 0);
        Eigen::Vector3d translationVector = transformationMatrix.block<3, 1>(0, 3);
        Eigen::Vector3d resultingTranslation = (rotationMatrix * poseOrigin - poseOrigin) + translationVector;
        return resultingTranslation;
    }

    double CalculateResultingRotationAngle(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> positionsAndTranslations)
    {
        Eigen::Vector3d sumOfMoments = Eigen::Vector3d::Zero();
        Eigen::Vector3d unitCorrection = Eigen::Vector3d::Zero();
        for (std::pair<Eigen::Vector3d, Eigen::Vector3d> pair : positionsAndTranslations)
        {
            sumOfMoments += pair.first.cross(pair.second);
            std::cout << "[DEBUG]Sum of moments: " << sumOfMoments.transpose() << std::endl;
            Eigen::Vector3d unitZVector(0, 0, 1);
            unitCorrection += pair.first.cross(unitZVector).cross(pair.first);
        }
        double angle = std::atan2(sumOfMoments.norm(), unitCorrection.norm());
        if(sumOfMoments.z() < 0)
        {
            angle = -angle; // Ensure the angle is in the correct direction
        }
        std::cout << "[DEBUG]Calculated resulting rotation angle: " << angle << std::endl;
        return angle;
    }
} // Referee::Transformations