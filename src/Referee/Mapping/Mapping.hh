#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include "../../3rd_party/GlobalMatch/code/global_match/stem_mapping.h"
#include "../../3rd_party/GlobalMatch/code/global_match/stem_matching.h"

namespace Referee::Mapping
{

    
    /**
     * @brief MappingMatrix class to store the N x N mapping matrix `M` between N point clouds, where each element M_{i,j} is the 4x4 transformation matrix between point cloud i and point cloud j
     */
    class MappingMatrix
    {
    public:
        /**
         * @brief Construct a new MappingMatrix object
         * @param numPointClouds Number of point clouds
         */
        MappingMatrix(int numPointClouds)
        {
            __mappingMatrix.resize(numPointClouds);
            for (int i = 0; i < numPointClouds; i++)
            {
                __mappingMatrix[i].resize(numPointClouds);
            }

            for(int i = 0; i < numPointClouds; i++)
            {
                for(int j = 0; j < numPointClouds; j++)
                {
                    __mappingMatrix[i][j] = Eigen::Matrix4f::Zero();
                }
            }
        }

        /**
         * @brief Getter of the transformation matrix between two point clouds
         * @param i Index of the first point cloud
         * @param j Index of the second point cloud
         * @return Eigen::Matrix4f Transformation matrix between the two point clouds
         */
        Eigen::Matrix4f GetTransformationMatrix(int i, int j)
        {
            return __mappingMatrix[i][j];
        }

        /**
         * @brief Setter for the transformation matrix between two point clouds
         * @param i Index of the first point cloud
         * @param j Index of the second point cloud
         * @param transformationMatrix Transformation matrix between the two point clouds
         */
        void SetTransformationMatrix(int i, int j, Eigen::Matrix4f transformationMatrix)
        {
            __mappingMatrix[i][j] = transformationMatrix;
        }

        /**
         * @brief Calculate the mean transformation matrix per point cloud (i.e. the mean transformation matrix along one row of the mapping matrix)
         */
        void CalculateMeanTransformationMatrices();


        std::vector<std::vector<int>> GetConnectivityMatrix();

        /**
         * @brief Print the mapping matrix to the console
         */
        void PrintMatrix();

        /**
         * @brief Print the mean transformation matrices to the console
         */
        void PrintMeanMatrices();

    private:
        /**
         * @brief Mapping matrix between the point clouds, where each element is the transformation matrix between two point clouds
         */
        std::vector<std::vector<Eigen::Matrix4f>> __mappingMatrix;

        /**
         * @brief Mean transformation matrix per point cloud
         */
        std::vector<Eigen::Matrix4f> __meanTransformationMatrices;
    };
    


    /**
     * @brief Create a connectivity matrix from a set of geolocations
     * @param geolocations Geolocations of the nodes
     * @param knn Number of nearest neighbors to consider
     * @param maxDistance Maximum distance to consider a connection
     * @param matrix Connectivity matrix to be created. Each element of the vector is the list of indices the respective geolocations are connected to
     */
    void CreateConnectivityMatrix(std::vector<std::vector<double>> geolocations, int knn, double maxDistance, std::vector<std::vector<int>>& matrix);
    


    /**
     * @brief enum to store the different transformation calculation methods. Currently only GlobalMatch is supported: https://doi.org/10.1016/j.isprsjprs.2023.01.013 
    */
    enum TransformationComputationMethod
    {
        GlobalMatch,
    };

    /**
     * @brief Computes the 4x4 transformation matrix that transforms the source point cloud to the target point cloud following a given method
     * @param source the source point cloud used as reference
     * @param target the point cloud we want to transform
     * @param method the method used in the computation.  
    */
    Eigen::Matrix4f ComputePairwiseTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, TransformationComputationMethod method);
}