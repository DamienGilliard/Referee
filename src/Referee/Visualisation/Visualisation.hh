#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>
#include <thread>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCamera.h>

namespace Referee::FileBasedVisualisation
{
    class Visualisation
    {
        public:
            Visualisation() = default;
            ~Visualisation() = default;
            void VisualisePointCloud(const std::string& plyFilePath);
            void VisualisePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
            void ExportImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, const std::string& imagePath);
            void VisualiseTransformations(std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix4d>> originsAndTransformations, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, const std::string& imageFileName = "transformations.png");

    };

    /**
     * @brief Create a coordinate system with the given origin and axis length
     * 
     * @param origin The origin of the coordinate system
     * @param axisLength The length of the axes
     * @return std::vector<Eigen::Vector3d> The coordinates of the end of axes
     */
    std::vector<pcl::PointXYZ> CreateCoordinateSystem(Eigen::Vector3d origin, double axisLength);
} // FileBasedVisualisation

