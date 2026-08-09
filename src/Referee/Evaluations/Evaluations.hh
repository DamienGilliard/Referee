#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <limits>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>

#include "../Mapping/Mapping.hh"

namespace Referee::Evaluations
{
    /**
     * @brief Solve the square assignment problem using the Hungarian algorithm.
     * @param cost Cost matrix where cost[i][j] is the cost of assigning row i to column j.
     * @return A vector where the i-th element is the index of the column assigned to row i, or -1 if no assignment was made.
     */
    std::vector<int> SolveSquareAssignment(const std::vector<std::vector<double>>& cost);


    /**
     * @brief Solve the assignment problem using the Hungarian algorithm with a padding cost.
     * @param cost Cost matrix where cost[i][j] is the cost of assigning row i to column j.
     * @param padCost Cost to assign a row to a dummy column (used for unassigned rows)
     * @return A vector where the i-th element is the index of the column assigned to row i, or -1 if no assignment was made.
     */
    std::vector<int> SolveAssignmentHungarian(const std::vector<std::vector<double>>& cost, double padCost);


    /**
     * @brief Find corresponding stems between two sets of stem cluster clouds using
     *        the Hungarian algorithm, so each stem is matched at most once and the
     *        total centroid-distance cost over all matches is minimized (rather than
     *        each stem greedily grabbing its own nearest neighbor, which can assign
     *        the same stem in stems2 to multiple stems in stems1).
     * @param stems1 First set of stem cluster point clouds
     * @param stems2 Second set of stem cluster point clouds, already in stems1's frame
     * @param distanceThreshold Maximum centroid distance for stems to be considered corresponding
     * @return Vector of pairs (index in stems1, index in stems2) of corresponding stems
     */
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> ExtractClusterClouds(const pcl::PointCloud<pcl::PointNormal>::Ptr& sliceCloud,
                                                                             const std::vector<pcl::PointIndices>& clusters);


    /**
     * @brief Applies a rigid transformation to each stem cluster cloud independently
     * @param stemClouds Vector of stem cluster point clouds
     * @param transformation 4x4 rigid transformation matrix
     * @return Vector of transformed stem cluster point clouds
     */
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> TransformStemClouds(const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& stemClouds,
                                                                            const Eigen::Matrix4d& transformation);


    /**
     * @brief Extract a horizontal slice from a point cloud at a specified height
     * @param cloud Input point cloud
     * @param height Height at which to slice (e.g., 2.0m for breast height)
     * @param heightTolerance Tolerance around the height (e.g., 0.2m for 1.8m-2.2m range)
     * @param gridSize Size of the grid for local floor z estimation
     * @return Pointer to a new point cloud containing only points in the slice
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr ExtractHorizontalSlice(
        const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
        double height,
        double heightTolerance = 0.2,
        double gridSize = 2.0);


    /**
     * @brief Detect stem cross-sections in a horizontal slice using Euclidean clustering
     * @param sliceCloud Input horizontal slice point cloud
     * @param clusterTolerance Maximum distance between points to be in the same cluster
     * @param minClusterSize Minimum number of points in a cluster to be considered a stem
     * @return Vector of cluster point indices (each cluster represents a stem cross-section)
     */
    std::vector<pcl::PointIndices> DetectStemClusters(
        const pcl::PointCloud<pcl::PointNormal>::Ptr& sliceCloud,
        double clusterTolerance = 0.1,
        int minClusterSize = 50);


    /**
     * @brief Fit a cylinder to a set of pcl::PointNormal points
     * @param candidatePoints Point cloud of candidate stem points
     * @return Cylinder parameters
     */
    std::vector<double> FitCylinderToCluster(const pcl::PointCloud<pcl::PointNormal>::Ptr& candidatePoints);

    
    /**
     * @brief From a set of scans, the individual stems are identified, circles are fitted to the stems, and the agreement between the stem positions in the scans is evaluated.
     * 
     * @param mappingMatrix The mapping matrix containing the transformations between the scans, and the scans themselves.
     * @return A pair containing the mean (.first) and standard deviation (.second) of the stem agreements
     */
    std::pair<double, double> EvaluateStemAgreements(Mapping::MappingMatrix& mappingMatrix);
}