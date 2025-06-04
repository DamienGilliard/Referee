#pragma once

#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

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
    Eigen::Vector3d RecenterTranslationVectors(std::vector<Eigen::Vector3d> &translationVectors);

    /**
     * @brief Calculate the median plane between two points
     * @param point1 First point
     * @param point2 Second point
     * @return std::vector<Eigen::Vector3d> A vector containing two elements: the origin (first element) and the normal vector (second element) of the plane
     */
    std::vector<Eigen::Vector3d> CalculateMedianPlane(Eigen::Vector3d point1, Eigen::Vector3d point2);

    /**
     * @brief Calculate the intersection point of three planes
     * @param plane1 First plane
     * @param plane2 Second plane
     * @param plane3 Third plane
     * @return Eigen::Vector3d Intersection point of the three planes
     * @note The planes are defined by their origin and normal vector. The first element of the vector is the origin, the second element is the normal vector.
     */
    Eigen::Vector3d CalculatePlaneIntersection(std::vector<Eigen::Vector3d> plane1, std::vector<Eigen::Vector3d> plane2, std::vector<Eigen::Vector3d> plane3);

    /**
     * @brief Falculate the resulting translation of a transformation matrix, meaning the translation induced by the rotation plus the translation of the transformation matrix
     * @param transformationMatrix Transformation matrix
     * @param poseOrigin Origin of the pose
     * @return Eigen::Vector3d Resulting translation of the transformation matrix
     */
    Eigen::Vector3d CalculateResultingTranslation(Eigen::Matrix4d transformationMatrix, Eigen::Vector3d poseOrigin);

    /**
     * @brief Compute the resulting rotation angle from a set of positions and translations. It can be compared to the rotational of a set of translations in space
     * @param positions Vector of positions in space
     * @param translations Vector of translations corresponding to the positions
     * @return double Resulting rotation angle in radians
     * @note The positions and translations should be in the same order, meaning that the translation at index i corresponds to the position at index i.
     */
    double CalculateResultingRotationAngle(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> positionsAndTranslations);
} // Referee::Transformations