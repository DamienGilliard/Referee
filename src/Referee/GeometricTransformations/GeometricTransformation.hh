#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <iostream>

namespace Referee::Transformations
{
    /**
     * @brief Translate a point cloud by a given vector
     * @tparam PointT Point type of the point cloud
     * @param cloud Point cloud to be translated
     * @param translationVector Vector to translate the point cloud by
     */
    template<typename PointT>
    void TranslatePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<double> translationVector)
    {
        if (translationVector.size() != 3)
        {
            std::cerr << "Translation vector must have 3 elements" << std::endl;
            return;
        }
        double x = translationVector[0];
        double y = translationVector[1];
        double z = translationVector[2];
        for (auto& point : *cloud)
        {
            point.x += x;
            point.y += y;
            point.z += z;
        }
    }
} // Referee::Transformations