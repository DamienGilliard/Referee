#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>
#include <thread>
#include <chrono>

#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/common/centroid.h>


namespace FileBasedVisualisation
{
    class Visualisation
    {
    public:
        Visualisation() = default;
        ~Visualisation() = default;
        void VisualisePointCloud(const std::string& plyFilePath);
        void VisualisePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    };
} // FileBasedVisualisation

