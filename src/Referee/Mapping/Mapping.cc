#include "Mapping.hh"

namespace Referee::Mapping
{
    void MappingMatrix::PrintMatrix()
    {
        for(int i = 0; i < __mappingMatrix.size(); i++)
        {
            for(int j = 0; j < __mappingMatrix[i].size(); j++)
            {
                std::cout << "matrix " << i << " " << j << std::endl;
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
            Eigen::Vector3f meanTranslation = Eigen::Vector3f::Zero();
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
                Eigen::Matrix3f rotationMatrix = __mappingMatrix[i][j].block<3, 3>(0, 0);
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

            Eigen::Matrix4f meanTransformationMatrix = Eigen::Matrix4f::Identity();
            meanTransformationMatrix.block<3, 1>(0, 3) = meanTranslation;
            Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity();
            rotationMatrix = Eigen::AngleAxisf(meanXRotation, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(meanYRotation, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(meanZRotation, Eigen::Vector3f::UnitZ());
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

    void CreateConnectivityMatrix(std::vector<std::vector<double>> geolocations, int knn, double maxDistance, std::vector<std::vector<int>>& matrix)
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
                    double distance = std::pow(geolocations[i][0] - geolocations[j][0], 2) + std::pow(geolocations[i][1] - geolocations[j][1], 2) + std::pow(geolocations[i][2] - geolocations[j][2], 2);
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

    Eigen::Matrix4f ComputePairwiseTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, TransformationComputationMethod method)
    {
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

        if(method == TransformationComputationMethod::GlobalMatch)
        {
            // Compute transformation using GlobalMatch, currently a copy of the main function from GlobalMatch's main.cpp
            std::cout << "Computing transformation using GlobalMatch" << std::endl;
            GlobalMatch::Mapping::Mapping globalMatchMapping;
            pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePosCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr targetPosCloud(new pcl::PointCloud<pcl::PointXYZ>);
            globalMatchMapping.setInputCloud(source->makeShared());
            globalMatchMapping.extract(sourcePosCloud);
            globalMatchMapping.setInputCloud(target->makeShared());
            globalMatchMapping.extract(targetPosCloud);
            
            GlobalMatch::Matching::Matching globalMatchMatching;
            globalMatchMatching.setPairwiseStemPositions(sourcePosCloud, targetPosCloud);
            globalMatchMatching.estimateTransformation(transformation);
        }
        else
        {
            std::cerr << "Unknown transformation computation method" << std::endl;
        }

        return transformation;
    }
}