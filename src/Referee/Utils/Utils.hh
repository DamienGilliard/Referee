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

namespace Referee 
{
    namespace Utils 
    {
        namespace CoordinateSystem
        {
            enum class CoordinateSystem
            {
                UTM,    //Universal Transverse Mercator 
                DMS,    //Degrees, Minutes, Seconds
                DEG,    //Decimal Degrees
                LV95,   //Swiss Grid (translation of UTM)
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
            void CropPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
            
            /**
             * @brief Voxelize a point cloud
             * 
             * @param cloud Point cloud to voxelize
             * @param leafSize Leaf size of the voxel grid
             */
            void VoxelizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double leafSize);
        } // Filtering
    } // Utils
}