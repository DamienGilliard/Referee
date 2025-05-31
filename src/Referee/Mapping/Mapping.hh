#pragma once

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
    enum class CoordinateSystemType
    {
        LV95, // Swiss coordinate system
        WGS84, // World coordinate system
        Local // Local coordinate system
    };

    class Pose
    {
        public:
            Pose() = default;
            Pose(Eigen::Vector3d position, Eigen::Quaterniond orientation)
                : __position(position), __orientation(orientation) {}

            Eigen::Vector3d GetPosition() const { return __position; }
            Eigen::Quaterniond GetOrientation() const { return __orientation; }

        private:
            Eigen::Vector3d __position; // Position in the global coordinate system
            Eigen::Quaterniond __orientation; // Orientation in the global coordinate system
    };

    /**
     * @brief Singleton class to store the global coordinate system
     * 
     * This class is used to store the global coordinate system in which all point clouds are transformed.
     * It is a singleton class, meaning that there is only one instance of this class in the whole program.
     */
    class GlobalCoordinateSystem
    {
        public:
            /**
             * @brief Get the instance of the global coordinate system
             * @return GlobalCoordinateSystem instance
             */
            static GlobalCoordinateSystem& GetInstance()
            {
                if(!__instance){__instance = new GlobalCoordinateSystem();}
                return *__instance;
            }
            static GlobalCoordinateSystem& CreateGlobalCoordinateSystem(CoordinateSystemType type = CoordinateSystemType::WGS84,
                                                                       Eigen::Vector3d origin = Eigen::Vector3d::Zero(),
                                                                       Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity())
            {
                if(!__instance){__instance = new GlobalCoordinateSystem(type, origin, orientation);}
                return *__instance;
            }
        private:
            GlobalCoordinateSystem(CoordinateSystemType type = CoordinateSystemType::WGS84,
                                   Eigen::Vector3d origin = Eigen::Vector3d::Zero(),
                                   Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity())
                : __type(type), __origin(origin), __orientation(orientation) {}
            GlobalCoordinateSystem(const GlobalCoordinateSystem&) = delete; // prevent copying
            GlobalCoordinateSystem& operator=(const GlobalCoordinateSystem&) = delete; // prevent assignment
            GlobalCoordinateSystem(GlobalCoordinateSystem&&) = delete; // prevent moving
            GlobalCoordinateSystem& operator=(GlobalCoordinateSystem&&) = delete; // prevent moving
            ~GlobalCoordinateSystem() = default; // destructor
            /**
             * @brief Type of the global coordinate system
             */
            CoordinateSystemType __type = CoordinateSystemType::WGS84; // type of the global coordinate system
            /**
             * @brief Origin of the global coordinate system
             */
            Eigen::Vector3d __origin = Eigen::Vector3d::Zero(); // origin of the global coordinate system
            /**
             * @brief Orientation of the global coordinate system
             */
            Eigen::Quaterniond __orientation = Eigen::Quaterniond::Identity(); // orientation of the global coordinate system
            /**
             * @brief Static instance of the global coordinate system
             * 
             * This is a static instance of the class, which is used to ensure that there is only one instance of the class in the whole program.
             */
            static GlobalCoordinateSystem* __instance; // static instance of the class
    };
    /**
     * @brief Thic class stores a transformation in 3D in the form of a rotation axis, a point on this axis, a rotation along this axis, and a translation along this axis.
     * 
     */
    class Transformation
    {
        public:
            Transformation() = default;
            Transformation(Eigen::Matrix4d transformationMatrixInGlobalCoordinateSystem);
    // class ChaslesTransformation
    // {
        // public:
            // ChaslesTransformation() = default;
            // ChaslesTransformation(Eigen::Vector3d rotationAxis, Eigen::Vector3d pointOnAxis, double rotationAngle, Eigen::Vector3d translation)
            //     : __rotationAxis(rotationAxis), __pointOnAxis(pointOnAxis), __rotationAngle(rotationAngle), __translation(translation) {}
            // ChaslesTransformation(Eigen::Matrix4d transformationMatrix);

            /**
             * @brief Print the transformation to the console
             */
            void PrintTransformation();

            // Eigen::Vector3d GetRotationAxis() const { return __rotationAxis; }
            // Eigen::Vector3d GetPointOnAxis() const { return __pointOnAxis; }
            double GetRotationAngle() const 
            { 
                if (__globalRotationVector.z() < 0)
                {
                    return -__globalRotationVector.norm();
                } 
                else
                {
                    return __globalRotationVector.norm();
                }
            }
            Eigen::Vector3d GetRotationVector() const { return __globalRotationVector; }
            Eigen::Vector3d GetTranslation() const { return __globalTranslation; }
        
        private:
            Eigen::Vector3d __globalRotationVector; // rotation axis expressed in the global coordinate system, the norm of this vector is the rotation angle in radians
            Eigen::Vector3d __globalTranslation; // translation vector expressed in the global coordinate system
            Eigen::Matrix4d __globalTransformation; // transformation matrix in the global coordinate system
    };

    
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
                        __mappingMatrix[i][j] = Referee::Mapping::Transformation();
                    }
                }
            }

            void SetConnectivityMatrix(std::vector<std::vector<int>> connectivityMatrix)
            {
                this->__connectivityMatrix = connectivityMatrix;
            }

            std::vector<std::vector<int>> GetConnectivityMatrix()
            {
                return this->__connectivityMatrix;
            }

            /**
             * @brief Getter of the transformation matrix between two point clouds
             * @param i Index of the first point cloud
             * @param j Index of the second point cloud
             * @return Referee::Mapping::Transformation between the two point clouds
             */
            Referee::Mapping::Transformation GetTransformation(int i, int j)
            {
                return __mappingMatrix[i][j];
            }

            /**
             * @brief Getter for the standard deviation of the rotation angles of the transformation matrices
             * @param i Index of the point cloud
             * @return Standard deviation of the rotation angles of the transformation matrices
             */
            double GetStdDevRotation(int i){return __stdDevRotations[i];}

            /**
             * @brief Getter for the mean rotation angle of the transformation matrices
             * @param i Index of the point cloud
             * @return Mean rotation angle of the transformation matrices
             */
            double GetMeanRotation(int i){return __meanTransformations[i].GetRotationAngle();}

            /**
             * @brief Setter for the transformation matrix between two point clouds
             * @param i Index of the first point cloud
             * @param j Index of the second point cloud
             * @param transformation Transformation matrix between the two point clouds
             */
            void SetTransformation(int i, int j, Referee::Mapping::Transformation transformation)
            {
                __mappingMatrix[i][j] = transformation;
            }

            /**
             * @brief Calculate the mean transformation matrix per point cloud (i.e. the mean transformation matrix along one row of the mapping matrix)
             */
            void CalculateMeanTransformationMatrices();

            /**
             * @brief Getter for the mean transformation matrix of a point cloud
             * @param i Index of the point cloud
             * @return Mean transformation matrix of the point cloud
             */
            Referee::Mapping::Transformation GetMeanTransformation(int i)
            {
                return __meanTransformations[i];
            }

            /**
             * @brief Compute the rotation coefficients between the point clouds
             * @param mostTrustworthyRotationAngle The rotation angle of the most trustworthy point cloud
             * @param mostTrustworthyPointCloudIndex The index of the most trustworthy point cloud
             */
            void ComputeRotationCoefficients(double mostTrustworthyRotationAngle, int mostTrustworthyPointCloudIndex);

            /**
             * @brief Getter for the rotation coefficient
             * @param i Index of the first point cloud (index of the row)
             * @param j Index of the second point cloud (index of the column)
             * @return Rotation coefficients
             */
            double GetRotationCoefficient(int i, int j)
            {
                return __rotationCoefficients(i, j);
            }

            /**
             * @brief Getter for the rotation angles from the rotation coefficients 
             * @return Rotation angles
             */
            std::vector<double> GetInitialRotationAngles();
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
            std::vector<std::vector<Referee::Mapping::Transformation>> __mappingMatrix;

            /**
             * @brief Mean transformation matrix per point cloud
             */
            std::vector<Referee::Mapping::Transformation> __meanTransformations;

            /**
             * @brief The matrix of the rotation coefficients. The rotation coefficients are by how much we should multiply the rotation angle of the transformation to get a compatible rotation wioth neighboring point clouds
             */
            Eigen::MatrixXd __rotationCoefficients;

            /**
             * @brief standard deviation of the rotation angles of the transformation matrices
             */
            std::vector<double> __stdDevRotations;

            /**
             * @brief for each point cloud, the indices of the other point clouds with which transformations were computed
             */
            std::vector<std::vector<int>> __connectivityMatrix;
    };
    

    /**
     * @brief Create a connectivity matrix from a set of geolocations
     * @param geolocations Geolocations of the nodes
     * @param knn Number of nearest neighbors to consider
     * @param maxDistance Maximum distance to consider a connection
     * @param matrix Connectivity matrix to be created. Each element of the vector is the list of indices the respective geolocations are connected to
     */
    void CreateConnectivityMatrix(std::vector<Eigen::Vector3d> geolocations, int knn, double maxDistance, std::vector<std::vector<int>>& matrix);
    

    /**
     * @brief enum to store the different transformation calculation methods. Currently only GlobalMatch is supported: https://doi.org/10.1016/j.isprsjprs.2023.01.013 
    */
    enum TransformationComputationMethod
    {
        GlobalMatch,
    };

    /**
     * @brief enum to store the different refinement methods. Currently only ICP with normals is supported: https://doi.org/10.1109/ICRA40945.2019.8793880
    */
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
     * @param ChaslesTransformation the transformation matrix
     * @return the rotation axis of the transformation matrix (first vector) and the translation along this axis (second vector), and a ploint on the axis (third vector)
     * @note The rotation axis norm is the rotation angle in radians.
     */
    std::vector<Eigen::Vector3d> ComputeScrewAxis(Eigen::Matrix4d ChaslesTransformation);

}