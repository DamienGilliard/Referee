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
    /**
     * @brief Generalist class for visualising point clouds and transformations
     */
    
    class Visualisation
    {
        public:
            Visualisation() = default;
            ~Visualisation() = default;
            /**
             * @brief Visualise a point cloud from a PLY file path
             * 
             * @param plyFilePath The path to the PLY file
             */
            void VisualisePointCloud(const std::string& plyFilePath);

            /**
             * @brief Visualise a point cloud
             * 
             * @param cloud The point cloud pointer to visualise
             */
            void VisualisePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

            /**
             * @brief Exports a top view of the point cloud to a PNG file
             * 
             * @param pointCloud The point cloud pointer to visualise
             * @param imagePath The path to the image file
             */
            void ExportImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, const std::string& imagePath);

            /**
             * @brief Visualise the transformations applied to the point cloud
             * 
             * @param originsAndTransformations A vector of pairs containing the origin of frames and transformation matrix
             * @param pointCloud The point cloud pointer to visualise, assuming it is composed of point clouds assembles after being transformed by the same transformations as the ones in the vector
             * @param imageFileName The name of the image file to save
             */
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

