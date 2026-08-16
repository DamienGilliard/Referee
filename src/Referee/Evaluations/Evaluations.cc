#include "Evaluations.hh"

namespace Referee::Evaluations
{

    std::vector<int> SolveSquareAssignment(const std::vector<std::vector<double>>& cost)
    {
        const int n = static_cast<int>(cost.size());
            constexpr double kInf = std::numeric_limits<double>::infinity();

        // 1-indexed working arrays, following the classic potentials formulation
        std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
        std::vector<int> p(n + 1, 0), way(n + 1, 0);

        for (int i = 1; i <= n; ++i)
        {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(n + 1, kInf);
            std::vector<bool> used(n + 1, false);

            do
            {
                used[j0] = true;
                int i0 = p[j0];
                int j1 = -1;
                double delta = kInf;

                for (int j = 1; j <= n; ++j)
                {
                    if (!used[j])
                    {
                        double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                        if (cur < minv[j])
                        {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta)
                        {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }

                for (int j = 0; j <= n; ++j)
                {
                    if (used[j])
                    {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    }
                    else
                    {
                        minv[j] -= delta;
                    }
                }

                j0 = j1;
            } while (p[j0] != 0);

            while (j0 != 0)
            {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            }
        }

        std::vector<int> rowToCol(n, -1);
        for (int j = 1; j <= n; ++j)
        {
            if (p[j] != 0)
            {
                rowToCol[p[j] - 1] = j - 1;
            }
        }
        return rowToCol;
    }

    std::vector<int> SolveAssignmentHungarian(const std::vector<std::vector<double>>& cost, double padCost)
    {
        const int nRows = static_cast<int>(cost.size());
        const int nCols = nRows > 0 ? static_cast<int>(cost[0].size()) : 0;
        if (nRows == 0 || nCols == 0)
        {
            return std::vector<int>(nRows, -1);
        }

        const int n = std::max(nRows, nCols);
        std::vector<std::vector<double>> square(n, std::vector<double>(n, padCost));
        for (int i = 0; i < nRows; ++i)
        {
            for (int j = 0; j < nCols; ++j)
            {
                square[i][j] = cost[i][j];
            }
        }

        std::vector<int> rowToCol = SolveSquareAssignment(square);
        rowToCol.resize(nRows);
        for (int i = 0; i < nRows; ++i)
        {
            if (rowToCol[i] >= nCols)
            {
                rowToCol[i] = -1; // matched to a padding column: no real correspondence
            }
        }
            return rowToCol;
    }

    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> ExtractClusterClouds(
        const pcl::PointCloud<pcl::PointNormal>::Ptr& sliceCloud,
        const std::vector<pcl::PointIndices>& clusters)
    {
        std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clusterClouds;
        clusterClouds.reserve(clusters.size());
        for (const auto& cluster : clusters)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointNormal>);
            clusterCloud->points.reserve(cluster.indices.size());
            for (int idx : cluster.indices)
            {
                clusterCloud->points.push_back(sliceCloud->points[idx]);
            }
            clusterCloud->width = static_cast<uint32_t>(clusterCloud->points.size());
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;
            clusterClouds.push_back(clusterCloud);
        }
        return clusterClouds;
    }

    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> TransformStemClouds(
        const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& stemClouds,
        const Eigen::Matrix4d& transformation)
    {
        std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> transformed;
        transformed.reserve(stemClouds.size());
        for (const auto& cloud : stemClouds)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointNormal>);
            pcl::transformPointCloudWithNormals(*cloud, *transformedCloud, transformation.cast<float>());
            transformed.push_back(transformedCloud);
        }
        return transformed;
    }
    

    pcl::PointCloud<pcl::PointNormal>::Ptr ExtractHorizontalSlice(
        const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
        double height,
        double heightTolerance,
        double gridSize)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr sliceCloud(new pcl::PointCloud<pcl::PointNormal>);

        if (cloud->empty() || gridSize <= 0.0)
        {
            return sliceCloud;
        }

        // x y indexing concatenated into a single int64_t key for the unordered_map
        auto tileKey = [gridSize](double x, double y) -> int64_t
        {
            int32_t ix = static_cast<int32_t>(std::floor(x / gridSize));
            int32_t iy = static_cast<int32_t>(std::floor(y / gridSize));
            return (static_cast<int64_t>(ix) << 32) | static_cast<uint32_t>(iy);
        };

        // First pass: find the local floor (minimum z) for each XY tile
        std::unordered_map<int64_t, double> tileFloorZ;
        for (const auto& point : cloud->points)
        {
            int64_t key = tileKey(point.x, point.y);
            auto [it, inserted] = tileFloorZ.try_emplace(key, point.z);
            if (!inserted && point.z < it->second)
            {
                it->second = point.z;
            }
        }

        // Second pass: keep points whose height above their tile's local floor
        // falls within height +- heightTolerance, so the slice follows the terrain
        for (const auto& point : cloud->points)
        {
            double floorZ = tileFloorZ.at(tileKey(point.x, point.y));
            double relativeHeight = point.z - floorZ;
            if (std::abs(relativeHeight - height) <= heightTolerance)
            {
                sliceCloud->push_back(point);
            }
        }

        return sliceCloud;
    }


    std::vector<pcl::PointIndices> DetectStemClusters(
        const pcl::PointCloud<pcl::PointNormal>::Ptr& sliceCloud,
        double clusterTolerance,
        int minClusterSize)
    {
        std::vector<pcl::PointIndices> clusters;
        
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(sliceCloud);
        
        pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(minClusterSize);
        ec.setMaxClusterSize(100000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(sliceCloud);
        ec.extract(clusters);
        
        return clusters;
    }

    /**
     * @brief Fit a cylinder to a set of pcl::PointNormal points
     * @param candidatePoints Point cloud of candidate stem points
     * @return Cylinder parameters
     */
    std::vector<double> FitCylinderToCluster(const pcl::PointCloud<pcl::PointNormal>::Ptr& candidatePoints)
    {
        if (candidatePoints->empty()) 
        {
            std::cerr << "FitCylinderToCluster: Input point cloud is empty." << std::endl;
            return {};
        }
        if (candidatePoints->points[0].normal_x == 0 && candidatePoints->points[0].normal_y == 0 && candidatePoints->points[0].normal_z == 0) 
        {
            std::cerr << "FitCylinderToCluster: Input point cloud has no normals." << std::endl;
            return {};
        }
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        for (const auto& point : candidatePoints->points) 
        {
            pcl::Normal normal;
            normal.normal_x = point.normal_x;
            normal.normal_y = point.normal_y;
            normal.normal_z = point.normal_z;
            normals->push_back(normal);
        }
        pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.03);
        seg.setRadiusLimits(0.05, 0.50);
        seg.setInputCloud(candidatePoints);
        seg.setInputNormals(normals);
        pcl::ModelCoefficients::Ptr cylinderCoefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment(*inliers, *cylinderCoefficients);

        return std::vector<double>(cylinderCoefficients->values.begin(), cylinderCoefficients->values.end());
    }

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
    std::vector<std::pair<int, int>> FindCorrespondingStems(
        const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& stems1,
        const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& stems2,
        double distanceThreshold = 0.5)
    {
        std::vector<std::pair<int, int>> correspondences;

        if (stems1.empty() || stems2.empty()) {
            return correspondences;
        }

        std::vector<Eigen::Vector3d> centroids1(stems1.size());
        std::vector<Eigen::Vector3d> centroids2(stems2.size());
        for (size_t i = 0; i < stems1.size(); ++i) {
            Eigen::Vector4d centroid;
            pcl::compute3DCentroid(*stems1[i], centroid);
            centroids1[i] = centroid.head<3>();
        }
        for (size_t j = 0; j < stems2.size(); ++j) {
            Eigen::Vector4d centroid;
            pcl::compute3DCentroid(*stems2[j], centroid);
            centroids2[j] = centroid.head<3>();
        }

        // Cost = centroid distance; pairs beyond distanceThreshold get a
        // prohibitive cost so the solver leaves them unmatched instead of
        // being forced into a bad long-distance correspondence.
        constexpr double kRejectCost = 1e6;
        std::vector<std::vector<double>> cost(centroids1.size(), std::vector<double>(centroids2.size()));
        for (size_t i = 0; i < centroids1.size(); ++i) {
            for (size_t j = 0; j < centroids2.size(); ++j) {
                double dist = (centroids1[i] - centroids2[j]).norm();
                cost[i][j] = (dist <= distanceThreshold) ? dist : kRejectCost;
            }
        }

        std::vector<int> assignment = SolveAssignmentHungarian(cost, kRejectCost);
        for (size_t i = 0; i < assignment.size(); ++i) {
            int j = assignment[i];
            if (j >= 0 && cost[i][j] <= distanceThreshold) {
                correspondences.emplace_back(static_cast<int>(i), j);
            }
        }

        return correspondences;
    }

    std::pair<double, double> EvaluateStemAgreements(Referee::Mapping::MappingMatrix& mappingMatrix)
    {
        // Configuration parameters
        const double sliceHeight = 2.0; // Breast height in meters
        const double heightTolerance = 0.2; // ±0.2m around slice height
        const double clusterTolerance = 0.1; // 10cm for clustering
        const int minClusterSize = 50; // Minimum points per stem cluster
        const double correspondenceThreshold = 0.5; // 50cm threshold for stem correspondence
        
        std::vector<double> agreementDistances;

        // Process each scan to extract one point cloud per detected stem at breast height
        std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>> allScanStemClouds;

        auto& scans = mappingMatrix.GetScans();
        for (size_t i = 0; i < scans.size(); ++i) {
            auto& scan = scans[i];

            // Load the point cloud if not already loaded
            if (scan.GetCloud()->size() == 0) {
                scan.LoadCloud();
            }

            // Extract horizontal slice at 2m
            auto sliceCloud = ExtractHorizontalSlice(scan.GetCloud(), sliceHeight, heightTolerance);

            if (sliceCloud->size() < static_cast<size_t>(minClusterSize)) {
                // Not enough points in this slice, skip this scan
                allScanStemClouds.emplace_back();
                continue;
            }

            // Cluster the slice into individual stem cross-sections
            auto clusters = DetectStemClusters(sliceCloud, clusterTolerance, minClusterSize);
            allScanStemClouds.push_back(ExtractClusterClouds(sliceCloud, clusters));
        }

        // Compare stem positions between overlapping scans
        for (size_t i = 0; i < scans.size(); ++i) {
            for (size_t j = i + 1; j < scans.size(); ++j) {
                // Check if scans i and j have a transformation between them
                auto transformation = mappingMatrix.GetTransformation(i, j);
                auto transformationMatrix = transformation.GetTransformationMatrix();

                // Get stem clouds for both scans
                auto& stemsI = allScanStemClouds[i];
                auto& stemsJ = allScanStemClouds[j];

                if (stemsI.empty() || stemsJ.empty()) {
                    continue; // Skip if either scan has no stems
                }

                // Transform stems from scan j to scan i's coordinate system
                auto stemsJ_Transformed = TransformStemClouds(stemsJ, transformationMatrix);

                // Find corresponding stems (Hungarian-optimal one-to-one matching)
                auto correspondences = FindCorrespondingStems(stemsI, stemsJ_Transformed, correspondenceThreshold);

                // Compute distances between corresponding stems' centroids
                for (const auto& corr : correspondences) {
                    Eigen::Vector4d centroidI, centroidJ;
                    pcl::compute3DCentroid(*stemsI[corr.first], centroidI);
                    pcl::compute3DCentroid(*stemsJ_Transformed[corr.second], centroidJ);
                    double distance = (centroidI - centroidJ).head<3>().norm();
                    agreementDistances.push_back(distance);
                }
            }
        }
        
        // Compute statistics
        double meanAgreement = 0.0;
        double stdDevAgreement = 0.0;
        
        if (!agreementDistances.empty()) {
            // Compute mean
            double sum = 0.0;
            for (double dist : agreementDistances) {
                sum += dist;
            }
            meanAgreement = sum / static_cast<double>(agreementDistances.size());
            
            // Compute standard deviation
            double sumSq = 0.0;
            for (double dist : agreementDistances) {
                double diff = dist - meanAgreement;
                sumSq += diff * diff;
            }
            stdDevAgreement = std::sqrt(sumSq / static_cast<double>(agreementDistances.size()));
        }
        
        return std::make_pair(meanAgreement, stdDevAgreement);
    }
}