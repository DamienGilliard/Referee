#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>

namespace Referee 
{
    namespace Utils 
    {
        namespace CoordinateSystem
        {
            enum class CoordinateSystem
            /**
             * @brief Enum for different coordinate systems.
             * Currently supported: UTM, DMS, DEG, LV95
             */
            {
                UTM,    //Universal Transverse Mercator 
                DMS,    //Degrees, Minutes, Seconds
                DEG,    //Decimal Degrees
                LV95,   //Swiss Grid (translation of UTM)
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
             */
            void CropPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
            
            /**
             * @brief Voxelize a point cloud
             * 
             * @param cloud Point cloud to voxelize
             * @param leafSize Leaf size of the voxel grid
             */
            void VoxelizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double leafSize);
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
    } // Utils
}