#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
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
                    __mappingMatrix[i][j] = Eigen::Matrix4d::Zero();
                }
            }
        }

        /**
         * @brief Getter of the transformation matrix between two point clouds
         * @param i Index of the first point cloud
         * @param j Index of the second point cloud
         * @return Eigen::Matrix4d Transformation matrix between the two point clouds
         */
        Eigen::Matrix4d GetTransformationMatrix(int i, int j)
        {
            return __mappingMatrix[i][j];
        }

        /**
         * @brief Setter for the transformation matrix between two point clouds
         * @param i Index of the first point cloud
         * @param j Index of the second point cloud
         * @param transformationMatrix Transformation matrix between the two point clouds
         */
        void SetTransformationMatrix(int i, int j, Eigen::Matrix4d transformationMatrix)
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
        std::vector<std::vector<Eigen::Matrix4d>> __mappingMatrix;

        /**
         * @brief Mean transformation matrix per point cloud
         */
        std::vector<Eigen::Matrix4d> __meanTransformationMatrices;
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

    enum RefinementMethod
    {
        ICPNormals,
    };

    /**
     * @brief Computes the 4x4 transformation matrix that transforms the source point cloud to the target point cloud following a given method
     * @param source the source point cloud used as reference
     * @param target the point cloud we want to transform
     * @param method the method used in the computation.  
    */
    Eigen::Matrix4d ComputePairwiseTransformation(pcl::PointCloud<pcl::PointNormal>::Ptr source, pcl::PointCloud<pcl::PointNormal>::Ptr target, TransformationComputationMethod method);

    /**
     * @brief Refines the pairwise transformation using a refinement method
     * @param source the source point cloud used as reference
     * @param target the point cloud we want to transform
     * @param method the method used in the computation. Currently only ICP with normals is supported
     * @param maxCorrespondenceDistance maximum correspondence distance for the ICP algorithm
     */
    Eigen::Matrix4d RefinePairwiseTransformation(pcl::PointCloud<pcl::PointNormal>::Ptr source, pcl::PointCloud<pcl::PointNormal>::Ptr target, RefinementMethod method, double maxCorrespondenceDistance);

    /**
     * @brief Computes the rotation axis and translation along this axis. This relies on the Chasles theorem. 
     * @param transformationMatrix the transformation matrix
     * @return the rotation axis of the transformation matrix (first vector) and the translation along this axis (second vector)
     * @note The rotation axis norm is the rotation angle in radians.
     */
    std::vector<Eigen::Vector3d> ComputeScrewAxis(Eigen::Matrix4d transformationMatrix);
}