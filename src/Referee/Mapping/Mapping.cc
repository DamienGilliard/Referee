#include "Mapping.hh"
#include "../GeometricTransformations/GeometricTransformation.hh"
#include "../Probability/Probability.hh"

namespace Referee::Mapping
{
    Transformation::Transformation(Eigen::Matrix4d transformationMatrixInGlobalCoordinateSystem)
        : __globalTransformation(transformationMatrixInGlobalCoordinateSystem)
    {
        Eigen::Matrix3d rotationMatrix = transformationMatrixInGlobalCoordinateSystem.block<3, 3>(0, 0);
        Eigen::AngleAxisd angleAxis(rotationMatrix);
        __globalRotationVector = angleAxis.axis() * angleAxis.angle();
        __globalTranslation = transformationMatrixInGlobalCoordinateSystem.block<3, 1>(0, 3);
    }

    void Transformation::PrintTransformation()
    {
        std::cout << "Rotation vector: " << __globalRotationVector.transpose() << std::endl;
        std::cout << "Rotation angle: " << __globalRotationVector.norm() << " radians" << std::endl;
        std::cout << "Translation: " << __globalTranslation.transpose() << std::endl;
    }

    void MappingMatrix::PrintMatrix()
    {
        for(int i = 0; i < __mappingMatrix.size(); i++)
        {
            for(int j = 0; j < __mappingMatrix[i].size(); j++)
            {
                std::cout << i << " " << j << " : " << std::endl;
                __mappingMatrix[i][j].PrintTransformation();
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    void MappingMatrix::CalculateMeanTransformationMatrices()
    {
        // std::vector<double> stdDevRotations;
        this->__stdDevRotations.resize(__mappingMatrix.size());
        this->__meanTransformations.resize(__mappingMatrix.size());
        this->__meanTranslationVectors.resize(__mappingMatrix.size());
        this->__covTranslationVectors.resize(__mappingMatrix.size());
        for(int i = 0; i < __mappingMatrix.size(); i++)
        {
            std::vector<Eigen::Vector3d> translationVectors;
            Eigen::Matrix3d translationCovarianceMatrix = Eigen::Matrix3d::Zero();
            Eigen::Vector3d meanTranslation = Eigen::Vector3d::Zero();
            Eigen::Vector3d stdDevTranslation = Eigen::Vector3d::Zero();
            Eigen::Vector3d meanRotationAxis = Eigen::Vector3d::Zero();
            double meanRotationAngle = 0;
            double stdDevRotation = 0;
            int nonZeroMatrices = 0;
            for(int j = 0; j < __mappingMatrix[i].size(); j++)
            {
                if(__mappingMatrix[i][j].GetRotationAngle() == 0 && __mappingMatrix[i][j].GetTranslation().norm() == 0)
                {
                    continue;
                }

                translationVectors.push_back(__mappingMatrix[i][j].GetTranslation());
                Eigen::Vector3d rotationAxis = __mappingMatrix[i][j].GetRotationVector();
                if(rotationAxis.z() < 0)
                {
                    rotationAxis = -rotationAxis;
                }
                meanRotationAxis += rotationAxis;
                meanRotationAngle += __mappingMatrix[i][j].GetRotationAngle();
                nonZeroMatrices++;
            }
            std::pair<Eigen::Vector3d, Eigen::Matrix3d> translationStats = Referee::Probability::ComputeMeanVectorAndCovarianceMatrix(translationVectors);
            meanRotationAxis /= nonZeroMatrices;
            meanRotationAngle /= nonZeroMatrices;
            this->__meanTranslationVectors[i] = translationStats.first;

            for(int j = 0; j < __mappingMatrix[i].size(); j++)
            {
                if(__mappingMatrix[i][j].GetRotationAngle() == 0 && __mappingMatrix[i][j].GetTranslation().norm() == 0)
                {
                    continue;
                }
                stdDevRotation += std::pow(__mappingMatrix[i][j].GetRotationAngle() - meanRotationAngle, 2);
            }
            __stdDevRotations[i] = stdDevRotation;
            std::cout << "[DEBUG] covariance matrix:" << translationStats.second << std::endl;

            std::cout << "[DEBUG] determinant of covariance matrix: " << translationStats.second.determinant() << std::endl;
            double entropy = 3.0/2.0 * (1.0 + std::log(2.0 * M_PI)) + 0.5 * std::log(translationStats.second.determinant());
            std::cout << "[DEBUG]Entropy for point cloud " << i << ": " << entropy << std::endl;
            Eigen::Matrix4d meanTransformationMatrix = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(meanRotationAngle, meanRotationAxis.normalized()).toRotationMatrix();
            meanTransformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
            meanTransformationMatrix.block<3, 1>(0, 3) = translationStats.first;

            Referee::Mapping::Transformation meanTransformation(meanTransformationMatrix);
            this->__meanTransformations[i] = meanTransformation;
            this->__covTranslationVectors[i] = translationStats.second;
        }
    }

    std::tuple<int, double> MappingMatrix::GetMostProbableRotation()
    {
        int mostProbableIndex = 0;
        double maxProbability = 0;
        for(int i = 0; i < this->__stdDevRotations.size(); i++)
        {
            double probability = Referee::Probability::Compute1DProbabilityDensityFunction(this->__meanTransformations[i].GetRotationAngle(), this->__meanTransformations[i].GetRotationAngle(), this->__stdDevRotations[i]);
            if(probability > maxProbability)
            {
                maxProbability = probability;
                mostProbableIndex = i;
            }
        }
        return std::make_tuple(mostProbableIndex, maxProbability);
    }

    std::pair<int, double> MappingMatrix::GetMostProbableTranslation()
    {
        int mostProbableIndex = 0;
        double maxProbability = 0;
        for(int i = 0; i < this->__mappingMatrix.size(); i++)
        {
            std::vector<Eigen::Vector3d> translationVectors;
            for(int j = 0; j < this->__mappingMatrix[i].size(); j++)
            {
                if(this->__mappingMatrix[i][j].GetTranslation().norm() == 0)
                {
                    continue;
                }
                translationVectors.push_back(this->__mappingMatrix[i][j].GetTranslation());
            }

            double probability = Referee::Probability::Compute3DProbabilityDensityFunction(this->__meanTranslationVectors[i], this->__meanTranslationVectors[i], this->__covTranslationVectors[i]);
            std::cout << "[DEBUG]Probability for point cloud " << i << ": " << probability << std::endl;
            std::cout << "[DEBUG]Mean translation vector for point cloud " << i << ": " << this->__meanTranslationVectors[i].transpose() << std::endl;
            if(probability > maxProbability)
            {
                maxProbability = probability;
                mostProbableIndex = i;
            }
        }
        return std::make_pair(mostProbableIndex, maxProbability);
    }

    std::vector<std::pair<double, double>> MappingMatrix::GetMeanRotationsAndStdDevs()
    {
        std::vector<std::pair<double, double>> meanRotationsAndStdDevs;
        for(int i = 0; i < this->__meanTransformations.size(); i++)
        {
            meanRotationsAndStdDevs.push_back(std::make_pair(this->__meanTransformations[i].GetRotationAngle(), this->__stdDevRotations[i]));
        }
        return meanRotationsAndStdDevs;
    }

    std::vector<std::pair<double, Eigen::Vector3d>> MappingMatrix::GetMeanTranslationVectorsAndStdDevs()
    {
        std::vector<std::pair<double, Eigen::Vector3d>> meanTranslationVectorsAndStdDevs;
        for(int i = 0; i < this->__meanTransformations.size(); i++)
        {
            Eigen::Vector3d translation = this->__meanTransformations[i].GetTranslation();
            double stdDev = this->__stdDevRotations[i]; // Assuming stdDev is the same for all components of the translation vector
            meanTranslationVectorsAndStdDevs.push_back(std::make_pair(translation.norm(), translation));
        }
        return meanTranslationVectorsAndStdDevs;
    }
    
    void MappingMatrix::ComputeRotationCoefficients(int mostTrustworthyPointCloudIndex)
    {
        double mostTrustworthyRotationAngle = this->__meanTransformations[mostTrustworthyPointCloudIndex].GetRotationAngle();
        int numberFiles = this->__mappingMatrix.size();
        Eigen::MatrixXd rotationCoefficients(numberFiles, numberFiles);
        for(int i = 0; i < numberFiles; i++)
        {
            for(int j = 0; j < numberFiles; j++)
            {
                rotationCoefficients(i, j) = 0;
            }
        }

        for(int i : this->__connectivityMatrix[mostTrustworthyPointCloudIndex])
        {
            if(i == mostTrustworthyPointCloudIndex)
            {
                continue;
            }
            double rotationAngle = this->__mappingMatrix[mostTrustworthyPointCloudIndex][i].GetRotationAngle();
            std::cout << "[DEBUG]Rotation angle between point cloud " << mostTrustworthyPointCloudIndex << " and point cloud " << i << ": " << rotationAngle << std::endl;

            double alpha = (mostTrustworthyRotationAngle/rotationAngle);
            rotationCoefficients(mostTrustworthyPointCloudIndex, i) = alpha;

        }

        // starting with the most trustworthy point cloud, onwards,
        for (int i = mostTrustworthyPointCloudIndex; i < numberFiles; i++)
        {
            // we store the connected point clouds
            std::vector<int> connectedPC = this->__connectivityMatrix[i];
            // for each connected point cloud, we initialize the row
            int matchedIndex;
            double rotationAngle;
            for (int j : connectedPC)
            {
                // we verify if the connectivity is reciprocal (i.e. if j is connected to i). 
                // because we work with knn, it is possible that i is connected to j, but j is not connected to i
                for (int k : this->__connectivityMatrix[j])
                {
                    // if indeed it is reciprocal
                    if (i == k)
                    {
                        double alpha = rotationCoefficients(j, i);
                        if (rotationCoefficients(j, i) != 0) 
                        {
                            rotationCoefficients(i, j) = 1 - alpha;
                            matchedIndex = j;
                            rotationAngle = this->__mappingMatrix[i][j].GetRotationAngle() * rotationCoefficients(i, j);
                        } 
                    }
                }
            }
            for (int j : connectedPC)
            {
                // if the connectivity is not reciprocal, we set the rotation coefficient to 0
                if (j != matchedIndex)
                {
                    rotationCoefficients(i, j) = rotationAngle / this->__mappingMatrix[i][j].GetRotationAngle();
                }
            }
        }
        // starting with the most trustworthy point cloud, backwards,
        for (int i = mostTrustworthyPointCloudIndex; i >= 0; i--)
        {
            // we store the connected point clouds
            std::vector<int> connectedPC = this->__connectivityMatrix[i];
            // for each connected point cloud, we initialize the row
            int matchedIndex;
            double rotationAngle;
            for (int j : connectedPC)
            {
                // we verify if the connectivity is reciprocal (i.e. if j is connected to i). 
                // because we work with knn, it is possible that i is connected to j, but j is not connected to i
                for (int k : this->__connectivityMatrix[j])
                {
                    // if indeed it is reciprocal
                    if (i == k)
                    {
                        double alpha = rotationCoefficients(j, i);
                        if (rotationCoefficients(j, i) != 0) 
                        {
                            rotationCoefficients(i, j) = 1 - alpha;
                            matchedIndex = j;
                            rotationAngle = this->__mappingMatrix[i][j].GetRotationAngle() * rotationCoefficients(i, j);
                        } 
                    }
                }
            }
            for (int j : connectedPC)
            {
                // if the connectivity is not reciprocal, we set the rotation coefficient to 0
                if (j != matchedIndex)
                {
                    rotationCoefficients(i, j) = rotationAngle / this->__mappingMatrix[i][j].GetRotationAngle();
                }
            }
        }
    
        this->__rotationCoefficients = rotationCoefficients;
        std::cout << "Rotation coefficients: " << std::endl;
        for(int i = 0; i < rotationCoefficients.rows(); i++)
        {
            for(int j = 0; j < rotationCoefficients.cols(); j++)
            {
                std::cout << rotationCoefficients(i, j) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    void MappingMatrix::ComputeTranslationCoefficients(int mostTrustworthyPointCloudIndex)
    {
        // Initialize the translation factors with rests
        int numberFiles = this->__mappingMatrix.size();
        this->__translationFactorsWithRests.resize(numberFiles, std::vector<std::pair<double, Eigen::Vector3d>>(numberFiles, std::make_pair(0.0, Eigen::Vector3d::Zero())));

        Eigen::Vector3d mostTrustworthyTranslationVector = this->__meanTranslationVectors[mostTrustworthyPointCloudIndex];
        for(int i = 0; i < numberFiles; i++)
        {
            if(i == mostTrustworthyPointCloudIndex)
            {
                this->__translationFactorsWithRests[i][i].first = 1.0;
                this->__translationFactorsWithRests[i][i].second = this->__meanTranslationVectors[i];
                continue;
            }
            Eigen::Vector3d translationVector = this->__mappingMatrix[mostTrustworthyPointCloudIndex][i].GetTranslation();
            double projectionFactor = mostTrustworthyTranslationVector.dot(translationVector.normalized()) / translationVector.norm();
            Eigen::Vector3d projectionOfMeanVectorOnIndividualTranslationVector = projectionFactor * translationVector;
            Eigen::Vector3d rest = mostTrustworthyTranslationVector - projectionOfMeanVectorOnIndividualTranslationVector;
            this->__translationFactorsWithRests[mostTrustworthyPointCloudIndex][i].first = projectionFactor;
            this->__translationFactorsWithRests[mostTrustworthyPointCloudIndex][i].second = rest;
            this->__translationFactorsWithRests[i][mostTrustworthyPointCloudIndex].first = 1.0 - projectionFactor;
            this->__translationFactorsWithRests[i][mostTrustworthyPointCloudIndex].second = rest;
            std::cout << "[DEBUG]Projection factor for point cloud " << i << ": " << projectionFactor << std::endl;

        }
    }

    std::vector<double> MappingMatrix::GetInitialRotationAngles()
    {
        std::vector<double> initialRotationAngles;
        for(int i = 0; i < this->__mappingMatrix.size(); i++)
        {
            for (int j = 0; j < this->__mappingMatrix[i].size(); j++)
            {
                if(this->__mappingMatrix[i][j].GetRotationAngle() != 0)
                {
                    double angle = this->__mappingMatrix[i][j].GetRotationAngle() * this->GetRotationCoefficient(i, j);
                    initialRotationAngles.push_back(angle);
                    break;
                }
            }
        }
        return initialRotationAngles;
    }

    void MappingMatrix::PrintMeanMatrices()
    {
        for(int i = 0; i < this->__meanTransformations.size(); i++)
        {
            std::cout << "Mean transformation matrix for point cloud " << i << std::endl;
            this->__meanTransformations[i].PrintTransformation();
            std::cout << std::endl;
        }
    }

    void CreateConnectivityMatrix(std::vector<Eigen::Vector3d> geolocations, int knn, double maxDistance, std::vector<std::vector<int>>& matrix)
    {
        std::vector<std::vector<std::pair<int, double>>> distancesToOtherPcs;
        std::vector<std::vector<int>> totalMatrix;
        distancesToOtherPcs.resize(geolocations.size());
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
                    std::cout << "Distance between " << i << " and " << j << ": " << distance << std::endl;
                    distancesToOtherPcs[i].push_back(std::make_pair(j, distance));
                    totalMatrix[i].push_back(j);
                }
            }
        }

        // Get the knn closest neighbors for each node
        for(int i = 0; i < totalMatrix.size(); i++)
        {
            std::vector<int> neighbors = totalMatrix[i];
            // Sort distancesToOtherPcs[i] based on the distance
            std::sort(distancesToOtherPcs[i].begin(), distancesToOtherPcs[i].end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                return a.second < b.second;
            });

            // Select the k nearest neighbors
            matrix[i].resize(std::min(knn, static_cast<int>(distancesToOtherPcs[i].size())));
            for (int j = 0; j < matrix[i].size(); j++) {
                matrix[i][j] = distancesToOtherPcs[i][j].first; // Extract the index of the neighbor
            }

            // Debugging output
            std::cout << "Neighbors for " << i << ": ";
            for (int j = 0; j < matrix[i].size(); j++) {
                std::cout << matrix[i][j] << " ";
            }
            std::cout << std::endl;
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