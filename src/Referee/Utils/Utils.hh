#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>
#include <algorithm>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/io/ply_io.h>
#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include "ceres/rotation.h"
#include "ceres/jet.h"
#include "../../3rd_party/json/single_include/nlohmann/json.hpp"

namespace Referee 
{
    namespace Utils 
    {
        namespace CoordinateSystem
        {
            enum class CoordinateSystem
            /**
             * @brief Enum for different coordinate systems.
             * Currently supported: WGS84, DMS, DEG, LV95
             */
            {
                WGS84,    //World Geodetic System 1984
                DMS,    //Degrees, Minutes, Seconds
                DEG,    //Decimal Degrees
                LV95,   //Swiss Grid (translation of WGS84)
            };
            

            /**
             * @brief Class for geolocation of point clouds.
             * Contains latitude, longitude, altitude, coordinate system as well as point cloud file name.
             */
            class PointCloudGeolocation
            {
            public:
                double lat; 
                double lon; 
                double alt; 
                CoordinateSystem coordSys; 
                std::string fileName; 

                PointCloudGeolocation(double latitude, double longitude, double altitude, CoordinateSystem coordSys)
                    : lat(latitude), lon(longitude), alt(altitude), coordSys(coordSys) {};
                
                /**
                 * @brief Write the geolocation to a GeoJSON file.
                 */
                void WriteToGeojson();
            };
        } // CoordinateSystem


        namespace Conversions
        {
            /**
             * @brief Convert latitude, longitude to Cartesian coordinates (X, Y)
             * @param lat Latitude in degrees
             * @param lon Longitude in degrees
             * @param x X coordinate in meters
             * @param y Y coordinate in meters
             * @param FromCoordSys Coordinate system of input latitude and longitude
             * @param ToCoordSys Coordinate system of output coordinates
             */
            void ConvertLatLonAltToCartesian(double lat, double lon, double alt, double &x, double &y, double &z, CoordinateSystem::CoordinateSystem fromCoordSys, CoordinateSystem::CoordinateSystem toCoordSys);
            void ConvertECEFToLatLonAlt(double x, double y, double z, double &lat, double &lon, double &alt);

            /**
             * @brief Create a LAS file from a point cloud
             * @param cloud Point cloud to convert
             * @param lon Longitude of the point cloud1
             * @param lat Latitude of the point cloud
             * @param alt Altitude of the point cloud
             * @param outputFilePath Path to the output LAS file
             * @param coordSys Coordinate system of the output LAS file (default: LV95)
             */
            void CreateLASFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                                         double lon, 
                                         double lat, 
                                         double alt, 
                                         const std::string& outputFilePath,
                                         Referee::Utils::CoordinateSystem::CoordinateSystem coordSys = Referee::Utils::CoordinateSystem::CoordinateSystem::LV95);
        
            /**
             * @brief Convert a pose represented as a 6D vector (x, y, z, qx, qy, qz, qw) to a 4x4 transformation matrix
             * @param poseVector Pointer to a pose represented as a 6D vector (x, y, z, qx, qy, qz, qw)
             * @return Eigen::Matrix<T, 4, 4> Transformation matrix corresponding to the input pose vector
             */
            template <typename Derived>
            Eigen::Matrix<typename Derived::Scalar, 4, 4> poseAsVectorToTransformationMatrix(const Eigen::MatrixBase<Derived>& poseVector)
            {
                using T = typename Derived::Scalar;
                Eigen::Matrix<T, 4, 4> translationMatrix = Eigen::Matrix<T, 4, 4>::Identity();
                // Extract translation
                translationMatrix(0, 3) = poseVector(0);
                translationMatrix(1, 3) = poseVector(1);
                translationMatrix(2, 3) = poseVector(2);
                // Extract rotation
                const T angle = poseVector.template tail<3>().norm();
                if (angle < T(1e-8))
                {
                    // No rotation, return the transformation matrix with only translation
                    return translationMatrix;
                }
                const Eigen::Matrix<T, 3, 1> axis = poseVector.template tail<3>() / angle;
                const Eigen::Matrix<T, 3, 3> rotationMatrix = Eigen::AngleAxis<T>(angle, axis).toRotationMatrix();
                Eigen::Matrix<T, 4, 4> rotationMatrix4d = Eigen::Matrix<T, 4, 4>::Identity();
                rotationMatrix4d.template block<3, 3>(0, 0) = rotationMatrix;
                Eigen::Matrix<T, 4, 4> transformationMatrix = translationMatrix * rotationMatrix4d;
                return transformationMatrix;
            }

            /**
             * @brief Convert a transformation matrix to a twist (6D vector: 3D translation + 3D rotation)
             * @param transform Transformation matrix to convert
             * @return Eigen::Matrix<T, 6, 1> Twist corresponding to the input transformation matrix
             */
            template <typename Derived>
            Eigen::Matrix<typename Derived::Scalar, 6, 1> transformMatrixToTwist(const Eigen::MatrixBase<Derived>& transform)
            {
                using T = typename Derived::Scalar;
                Eigen::Matrix<T, 6, 1> twist;
                // Extract translation
                twist(0) = transform(0, 3);
                twist(1) = transform(1, 3);
                twist(2) = transform(2, 3);
                // Extract rotation
                Eigen::Matrix<T, 3, 3> rotationMatrix = transform.template block<3, 3>(0, 0);
                Eigen::AngleAxis<T> angleAxis(rotationMatrix);
                twist(3) = angleAxis.angle() * angleAxis.axis().x();
                twist(4) = angleAxis.angle() * angleAxis.axis().y();
                twist(5) = angleAxis.angle() * angleAxis.axis().z();
                return twist;
            }
        
        } // Conversions

        namespace FileIterators
        {
            /**
             * @brief Get all files in a directory with a specific extension
             * @param directory Directory to search for files
             * @param extension File extension to search for
             */
            std::vector<std::string> GetFilesInDirectory(const std::string& directory, const std::string& extension);


            /**
             * @brief Get translation vectors from a file. This file is supposed to be a csv with as first three elements of the first line:
             * latitude; longitude; altitude
             * @param filePath Path to the file
             * @param coordSys Coordinate system we want to convert to
             * @return Eigen::Vector3d Vector of translation vectors
             */
            Eigen::Vector3d GetTranslationVectorFromFile(const std::string& filePath, Referee::Utils::CoordinateSystem::CoordinateSystem coordSys);


            /**
             * @brief Get translation vectors from a vector of files. This vector is supposed to contain the paths to the files.
             * @param filePaths Vector of file paths
             * @param coordSys Coordinate system we want to convert to
             * @return std::vector<Eigen::Vector3d> Vector of translation vectors
             */
            std::vector<Eigen::Vector3d> GetTranslationVectorsFromFiles(const std::vector<std::string>& filePaths, Referee::Utils::CoordinateSystem::CoordinateSystem coordSys);
        } // FileIterators


        namespace Filtering
        {
            /**
             * @brief Crop a point cloud to a specific bounding box
             * 
             * @param cloud Point cloud to crop
             * @param minX Minimum x value of bounding box
             * @param minY Minimum y value of bounding box
             * @param minZ Minimum z value of bounding box
             * @param maxX Maximum x value of bounding box
             * @param maxY Maximum y value of bounding box
             * @param maxZ Maximum z value of bounding box
             * @param negativeCrop If true, crops everything outside the bounding box, otherwise crops everything inside the bounding box
             */
            template<typename PointT>
            void CropPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud, double minX, double minY, double minZ, double maxX, double maxY, double maxZ, bool negativeCrop = false)
            {
                pcl::CropBox<PointT> cropBoxFilter;
                cropBoxFilter.setInputCloud(cloud);
                Eigen::Vector4f minPoint(minX, minY, minZ, 1.0);
                Eigen::Vector4f maxPoint(maxX, maxY, maxZ, 1.0);
                cropBoxFilter.setMin(minPoint);
                cropBoxFilter.setMax(maxPoint);
                cropBoxFilter.setNegative(negativeCrop);
                cropBoxFilter.filter(*cloud);
            }
            

            /**
             * @brief Voxelize a point cloud
             * 
             * @param cloud Point cloud to voxelize
             * @param leafSize Leaf size of the voxel grid
             */
            template<typename PointT>
            void VoxelizePointCloud(typename pcl::PointCloud<PointT>::Ptr &cloud, double leafSize)
            {
                typename pcl::PointCloud<PointT>::Ptr voxelizedCloud(new pcl::PointCloud<PointT>);
                pcl::ApproximateVoxelGrid<PointT> voxelGridFilter;
                voxelGridFilter.setInputCloud(cloud);
                voxelGridFilter.setLeafSize(leafSize, leafSize, leafSize);
                voxelGridFilter.setDownsampleAllData(true); // Ensure all data is downsampled
                voxelGridFilter.filter(*voxelizedCloud);
                cloud.swap(voxelizedCloud);
            }
        } // Filtering


        namespace Coloring
        {
            /**
             * @brief Color a point cloud based on a specific color
             * 
             * @param coloredCloud Point cloud to color
             * @param cloud Point cloud to extract coordinates from
             * @param r Red value of the color
             * @param g Green value of the color
             * @param b Blue value of the color
             * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr Colored point cloud
             */
            void ColorPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b);
        } // Coloring


        namespace NormalCalculation
        {
            /**
             * @brief Calculate the normals of a point cloud
             * 
             * @param cloud Point cloud to calculate normals for
             * @param k Nearest neighbors to use for normal calculation
             */
            void CalculateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int k);
        } // NormalCalculation
    

        namespace IO
        {
            /**
             * @brief Save rotation angles and their standard deviations, as well as initial rotation to a file
             * 
             * @param fileName Name of the file to save to
             * @param meansAndStdDevs Vector of pairs containing mean rotation angles and their standard deviations
             * @param angles Vector of rotation angles
             */
            void SaveRotationAnglesAndStdDevs(const std::string& fileName, const std::vector<std::pair<double, double>>& meansAndStdDevs, const std::vector<double>& correctedAngles);
        }


        namespace Trigonometry
        {
            /**
             * @brief Solve a triangle using the Al-Kashi theorem (law of cosines)
             * 
             * @param sideA Length of side A
             * @param sideB Length of side B
             * @param sideC Length of side C
             * @return std::vector<double> Vector containing the angles opposite to sides A, B, and C (in radians)
             */
            std::vector<double> SolveAlKashi(Eigen::Vector3d sideA, Eigen::Vector3d sideB, Eigen::Vector3d sideC);
        }
    } // Utils
}