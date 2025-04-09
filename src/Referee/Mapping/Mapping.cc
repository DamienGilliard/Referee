#include "Mapping.hh"
#include "../GeometricTransformations/GeometricTransformation.hh"

namespace Referee::Mapping
{
    ChaslesTransformation::ChaslesTransformation(Eigen::Matrix4d transformationMatrix)
    {
        Eigen::Matrix3d rotationMatrix = transformationMatrix.block<3, 3>(0, 0);
        Eigen::Vector3d translationVector = transformationMatrix.block<3, 1>(0, 3);

        std::vector<Eigen::Vector3d> screwAxis = ComputeScrewAxis(transformationMatrix);
        __rotationAxis = screwAxis[0];
        __translation = screwAxis[1];
        __pointOnAxis = screwAxis[2];
        __rotationAngle = __rotationAxis.norm();
    }

    void MappingMatrix::PrintMatrix()
    {
        for(int i = 0; i < __mappingMatrix.size(); i++)
        {
            for(int j = 0; j < __mappingMatrix[i].size(); j++)
            {
                std::cout << i << " " << j << " : " << std::endl;
                std::cout << __mappingMatrix[i][j] << " ";
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    void MappingMatrix::CalculateMeanTransformationMatrices()
    {
        this->__meanTransformationMatrices.resize(__mappingMatrix.size());
        for(int i = 0; i < __mappingMatrix.size(); i++)
        {
            Eigen::Vector3d meanTranslation = Eigen::Vector3d::Zero();
            double meanXRotation = 0;
            double meanYRotation = 0;
            double meanZRotation = 0;
            int nonZeroMatrices = 0;
            for(int j = 0; j < __mappingMatrix[i].size(); j++)
            {
                if(__mappingMatrix[i][j].isZero(0))
                {
                    continue;
                }
                meanTranslation += __mappingMatrix[i][j].block<3, 1>(0, 3);
                Eigen::Matrix3d rotationMatrix = __mappingMatrix[i][j].block<3, 3>(0, 0);
                double xRotation = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));   
                double yRotation = atan2(-rotationMatrix(2, 0), sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + rotationMatrix(2, 2) * rotationMatrix(2, 2)));
                double zRotation = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
                meanXRotation += xRotation;
                meanYRotation += yRotation;
                meanZRotation += zRotation;
                nonZeroMatrices++;
            }
            meanTranslation /= nonZeroMatrices;
            meanXRotation /= nonZeroMatrices;
            meanYRotation /= nonZeroMatrices;
            meanZRotation /= nonZeroMatrices;

            Eigen::Matrix4d meanTransformationMatrix = Eigen::Matrix4d::Identity();
            meanTransformationMatrix.block<3, 1>(0, 3) = meanTranslation;
            Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
            rotationMatrix = Eigen::AngleAxisd(meanXRotation, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(meanYRotation, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(meanZRotation, Eigen::Vector3d::UnitZ());
            meanTransformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
            this->__meanTransformationMatrices[i] = meanTransformationMatrix;
        }
    }

    void MappingMatrix::PrintMeanMatrices()
    {
        for(int i = 0; i < this->__meanTransformationMatrices.size(); i++)
        {
            std::cout << "Mean transformation matrix for point cloud " << i << std::endl;
            std::cout << __meanTransformationMatrices[i] << std::endl;
        }
    }

    void CreateConnectivityMatrix(std::vector<Eigen::Vector3d> geolocations, int knn, double maxDistance, std::vector<std::vector<int>>& matrix)
    {
        std::vector<std::vector<double>> distances;
        std::vector<std::vector<int>> totalMatrix;
        distances.resize(geolocations.size());
        totalMatrix.resize(geolocations.size());
        matrix.resize(geolocations.size());

        for(int i = 0; i < geolocations.size(); i++)
        {
            for(int j = 0; j < geolocations.size(); j++)
            {
                if(i != j)
                {
                    // Calculate distance between geolocations
                    double distance = std::pow(geolocations[i].x() - geolocations[j].x(), 2) + std::pow(geolocations[i].y() - geolocations[j].y(), 2) + std::pow(geolocations[i].z() - geolocations[j].z(), 2);
                    distance = sqrt(distance);
                    distances[i].push_back(distance);
                    totalMatrix[i].push_back(j);
                }
            }
        }

        // Get the knn closest neighbors for each node
        for(int i = 0; i < totalMatrix.size(); i++)
        {
            std::vector<int> neighbors = totalMatrix[i];
            std::sort(neighbors.begin(), neighbors.end(), [&distances, i](int a, int b) { return distances[i][a] < distances[i][b]; });
            matrix[i].resize(knn);
            for(int j = 0; j < knn; j++)
            {
                matrix[i][j] = neighbors[j];
            }
        }
    }

    Eigen::Matrix4d ComputePairwiseTransformation(pcl::PointCloud<pcl::PointNormal>::Ptr source, pcl::PointCloud<pcl::PointNormal>::Ptr target, TransformationComputationMethod method)
    {
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

        if(method == TransformationComputationMethod::GlobalMatch)
        {
            // Compute transformation using GlobalMatch, currently a copy of the main function from GlobalMatch's main.cpp
            std::cout << "Computing transformation using GlobalMatch" << std::endl;
            GlobalMatch::Mapping::Mapping globalMatchMapping;
            pcl::PointCloud<pcl::PointXYZ>::Ptr sourceNoNormals(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr targetNoNormals(new pcl::PointCloud<pcl::PointXYZ>);
            for(auto point : *source)
            {
                pcl::PointXYZ pointNoNormals;
                pointNoNormals.x = point.x;
                pointNoNormals.y = point.y;
                pointNoNormals.z = point.z;
                sourceNoNormals->push_back(pointNoNormals);
            }
            for(auto point : *target)
            {
                pcl::PointXYZ pointNoNormals;
                pointNoNormals.x = point.x;
                pointNoNormals.y = point.y;
                pointNoNormals.z = point.z;
                targetNoNormals->push_back(pointNoNormals);
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePosCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr targetPosCloud(new pcl::PointCloud<pcl::PointXYZ>);
            globalMatchMapping.setInputCloud(sourceNoNormals->makeShared());
            globalMatchMapping.extract(sourcePosCloud);
            globalMatchMapping.setInputCloud(targetNoNormals->makeShared());
            globalMatchMapping.extract(targetPosCloud);
            
            GlobalMatch::Matching::Matching globalMatchMatching;
            globalMatchMatching.setPairwiseStemPositions(sourcePosCloud, targetPosCloud);
            globalMatchMatching.estimateTransformation(transformation);
        }
        else
        {
            std::cerr << "Unknown transformation computation method" << std::endl;
        }

        // convert Matrix4f to Matrix4d
        Eigen::Matrix4d transformationDouble = Eigen::Matrix4d::Identity();
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                transformationDouble(i, j) = transformation(i, j);
            }
        }

        return transformationDouble;
    }

    Eigen::Matrix4d RefinePairwiseTransformation(pcl::PointCloud<pcl::PointNormal>::Ptr target, pcl::PointCloud<pcl::PointNormal>::Ptr source, RefinementMethod method, double maxCorrespondenceDistance)
    {
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

        if(method == RefinementMethod::ICPNormals)
        {
            pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icpNormals;
            icpNormals.setInputSource(source);
            icpNormals.setInputTarget(target);
            icpNormals.setMaximumIterations(50);
            icpNormals.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
            icpNormals.setTransformationEpsilon(1e-8);
            icpNormals.setEuclideanFitnessEpsilon(1);

            pcl::PointCloud<pcl::PointNormal>::Ptr dummy(new pcl::PointCloud<pcl::PointNormal>);
            icpNormals.align(*dummy);
            Eigen::Matrix4f transformationf = icpNormals.getFinalTransformation();
            // convert Matrix4f to Matrix4d
            for(int i = 0; i < 4; i++)
            {
                for(int j = 0; j < 4; j++)
                {
                    transformation(i, j) = transformationf(i, j);
                }
            }
        }
        else
        {
            std::cerr << "Unknown refinement method" << std::endl;
        }
        return transformation;
    }

    std::vector<Eigen::Vector3d> ComputeScrewAxis(Eigen::Matrix4d transformationMatrix)
    {
        Eigen::Matrix3d rotationMatrix = transformationMatrix.block<3, 3>(0, 0);
        Eigen::Vector3d translationVector = transformationMatrix.block<3, 1>(0, 3);

        Eigen::Matrix3d logRotation = rotationMatrix.log();
        Eigen::Vector3d omega = Eigen::Vector3d(logRotation(2, 1), logRotation(0, 2), logRotation(1, 0));

        Eigen::Matrix3d omegaMatrix;
        omegaMatrix << 0, -omega(2), omega(1),
                       omega(2), 0, -omega(0),
                       -omega(1), omega(0), 0;
        double theta = omega.norm();

        Eigen::Vector3d v;
        if (theta > 0) 
        {
            v = (Eigen::Matrix3d::Identity() - 0.5 * omegaMatrix +
                (1.0 / theta * theta - (1.0 - std::cos(theta)) / (2.0 * theta * theta)) *
                omegaMatrix * omegaMatrix) * translationVector;
        } 
        else 
        {
            v = translationVector;
        }

        Eigen::Vector3d omegaNormalized = omega.normalized();

        Eigen::Vector3d vParallel = (omega.dot(v) / omega.dot(omega)) * omega;
        Eigen::Vector3d vPerpendicular = translationVector - vParallel;

        Eigen::Vector3d testPoint1(0, 0, 0);
        Eigen::Vector3d testPoint2(0, translationVector.norm(), 0);
        Eigen::Vector3d testPoint3(0, 0, translationVector.norm());

        Eigen::Vector3d transformedPoint1 = rotationMatrix * testPoint1 + translationVector;
        Eigen::Vector3d transformedPoint2 = rotationMatrix * testPoint2 + translationVector;
        Eigen::Vector3d transformedPoint3 = rotationMatrix * testPoint3 + translationVector;

        std::vector<Eigen::Vector3d> plane1 = {(transformedPoint1 + testPoint1)/2, (transformedPoint1 - testPoint1).normalized()};
        std::vector<Eigen::Vector3d> plane2 = {(transformedPoint2 + testPoint2)/2, (transformedPoint2 - testPoint2).normalized()};
        std::vector<Eigen::Vector3d> plane3 = {(transformedPoint3 + testPoint3)/2, (transformedPoint3 - testPoint3).normalized()};

        Eigen::Vector3d intersectionPoint = Referee::Transformations::CalculatePlaneIntersection(plane1, plane2, plane3);

        // Calculate a point on the axis of rotation
        std::vector<Eigen::Vector3d> screwAxis;
        screwAxis.push_back(omega);
        screwAxis.push_back(vParallel);
        screwAxis.push_back(intersectionPoint);
        return screwAxis;
    }
}