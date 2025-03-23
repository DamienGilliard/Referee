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
    /**
     * @brief Transform a point cloud by a given transformation matrix
     * @tparam PointT Point type of the point cloud
     * @param cloud Point cloud to be transformed
     * @param transformationMatrix Transformation matrix to transform the point cloud by
     */
    template<typename PointT>
    void TransformPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Matrix4f transformationMatrix)
    {
        for (auto& point : *cloud)
        {
            Eigen::Vector4f pointVector(point.x, point.y, point.z, 1);
            Eigen::Vector4f transformedPoint = transformationMatrix * pointVector;
            point.x = transformedPoint[0];
            point.y = transformedPoint[1];
            point.z = transformedPoint[2];
        }
    }

    /**
     * @brief Recenter translation vectors by subtracting the mean translation vector
     * @param translationVectors Translation vectors to be recentered. The translation vectors will be modified in place
     * @return std::vector<double> Mean translation vector, which was subtracted from the translation vectors.
     */
    std::vector<double> RecenterTranslationVectors(std::vector<std::vector<double>> &translationVectors)
    {
        std::vector<double> meanTranslationVector(3, 0);
        for (auto& translationVector : translationVectors)
        {
            meanTranslationVector[0] += translationVector[0];
            meanTranslationVector[1] += translationVector[1];
            meanTranslationVector[2] += translationVector[2];
        }
        meanTranslationVector[0] /= translationVectors.size();
        meanTranslationVector[1] /= translationVectors.size();
        meanTranslationVector[2] /= translationVectors.size();

        for (auto& translationVector : translationVectors)
        {
            translationVector[0] -= meanTranslationVector[0];
            translationVector[1] -= meanTranslationVector[1];
            translationVector[2] -= meanTranslationVector[2];
        }
        return meanTranslationVector;
    }

} // Referee::Transformations