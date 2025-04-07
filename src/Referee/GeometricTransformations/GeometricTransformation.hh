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
    void TranslatePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector3d translationVector)
    {
        double x = translationVector.x();
        double y = translationVector.y();
        double z = translationVector.z();
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
    void TransformPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Matrix4d transformationMatrix)
    {
        for (auto& point : *cloud)
        {
            Eigen::Vector4d pointVector(point.x, point.y, point.z, 1);
            Eigen::Vector4d transformedPoint = transformationMatrix * pointVector;
            point.x = transformedPoint[0];
            point.y = transformedPoint[1];
            point.z = transformedPoint[2];
        }
    }

    /**
     * @brief Recenter translation vectors by subtracting the mean translation vector
     * @param translationVectors Translation vectors to be recentered. The translation vectors will be modified in place
     * @return Eigen::Vector3d Mean translation vector, which was subtracted from the translation vectors.
     */
    Eigen::Vector3d RecenterTranslationVectors(std::vector<Eigen::Vector3d> &translationVectors)
    {
        Eigen::Vector3d meanTranslationVector = Eigen::Vector3d::Zero();
        for (const auto& translationVector : translationVectors)
        {
            meanTranslationVector += translationVector;
        }
        meanTranslationVector /= static_cast<double>(translationVectors.size());

        for (auto& translationVector : translationVectors)
        {
            translationVector -= meanTranslationVector;
        }
        return meanTranslationVector;
    }

} // Referee::Transformations