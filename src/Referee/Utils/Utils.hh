#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

        /*
        @brief Convert latitude, longitude to Cartesian coordinates (X, Y)
        @param lat Latitude in degrees
        @param lon Longitude in degrees
        @param x X coordinate in meters
        @param y Y coordinate in meters
        @param FromCoordSys Coordinate system of input latitude and longitude
        @param ToCoordSys Coordinate system of output coordinates
        */
        namespace Conversions
        {
            void ConvertLatLonAltToCartesian(double lat, double lon, double alt, double &x, double &y, double &z, CoordinateSystem::CoordinateSystem fromCoordSys, CoordinateSystem::CoordinateSystem toCoordSys);
            void ConvertECEFToLatLonAlt(double x, double y, double z, double &lat, double &lon, double &alt);
        } // Conversions

        namespace FileIterators
        {
            /*
            @brief Get all files in a directory with a specific extension
            @param directory Directory to search for files
            @param extension File extension to search for
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
    } // Utils
}