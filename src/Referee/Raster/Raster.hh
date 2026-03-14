#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <gdal_priv.h>
#include <cpl_conv.h>
#include <limits>
#include "../Utils/Utils.hh"

namespace Referee::Raster
{
    class Rasterizer
    {
        public:
            Rasterizer() = default;
            ~Rasterizer() = default;

            /**
             * @brief Create a GeoTIFF file from a pcl point cloud
             * 
             * 
             * @param cloud Point cloud to convert
             * @param outputFilePath Path to the output GeoTIFF file
             * @param resolution Resolution of the rasterized output
             */
            void CreateGeoTIFFDEMFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& outputFilePath, double resolution, Referee::Utils::CoordinateSystem::CoordinateSystem coordSys);
    };
}