#include "Referee.hh"


std::mutex mutex;
// A function we will run in multiple threads
void ComputeTransformationInThread(int sourcePointCloudFileIndex,
                            int targetPointCloudFileIndex,
                            std::vector<Referee::Mapping::Scan> scans,
                            double voxelSize,
                            Referee::Mapping::MappingMatrix& mappingMatrix)
{
    scans[sourcePointCloudFileIndex].LoadCloud();
    scans[targetPointCloudFileIndex].LoadCloud();

    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(scans[sourcePointCloudFileIndex].GetCloud(), voxelSize);
    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(scans[targetPointCloudFileIndex].GetCloud(), voxelSize);

    std::pair<Eigen::Matrix4d, float> transformationMatrixAndScore = Referee::Mapping::ComputePairwiseTransformation(scans[sourcePointCloudFileIndex].GetCloud(),
                                                                                           scans[targetPointCloudFileIndex].GetCloud(),
                                                                                           Referee::Mapping::TransformationComputationMethod::GlobalMatch);
    mappingMatrix.GetGraph().SetWeight(sourcePointCloudFileIndex, targetPointCloudFileIndex, -transformationMatrixAndScore.second);
    Referee::Mapping::Transformation transformation(transformationMatrixAndScore.first, nullptr, nullptr, transformationMatrixAndScore.second);

    scans[sourcePointCloudFileIndex].FlushCloud();
    scans[targetPointCloudFileIndex].FlushCloud();
    mutex.lock();
    mappingMatrix.SetTransformation(sourcePointCloudFileIndex, targetPointCloudFileIndex, transformation);
    mappingMatrix.SetTransformation(targetPointCloudFileIndex, sourcePointCloudFileIndex, transformation.GetInverse());
    mutex.unlock();
}

int main()
{
    std::vector<std::string> geolocationFiles = Referee::Utils::FileIterators::GetFilesInDirectory("../../test_files/geolocations", ".petitpoucet");
    std::vector<std::string> plyFileNames = Referee::Utils::FileIterators::GetFilesInDirectory("../../test_files/scans", ".ply");
    int numberFiles = plyFileNames.size();
    if (numberFiles != geolocationFiles.size()) 
    {
        std::cerr << "Number of geolocation files and ply files do not match" << std::endl;
        return 1;
    }

    Referee::Utils::CoordinateSystem::CoordinateSystem coordSys = Referee::Utils::CoordinateSystem::CoordinateSystem::LV95;
    std::vector<Eigen::Vector3d> initialTranslationVectors = Referee::Utils::FileIterators::GetTranslationVectorsFromFiles(geolocationFiles, coordSys);
    Eigen::Vector3d meanTranslation = Referee::Transformations::RecenterTranslationVectors(initialTranslationVectors);
    std::cout << "Mean translation vector: " << std::setprecision(15) << meanTranslation.transpose() << std::endl;

    std::vector<Referee::Mapping::Scan> scans;
    int nFiles = initialTranslationVectors.size();
    for (int i = 0; i < nFiles; i++)
    {
        Eigen::Quaterniond baseQuaternion = Eigen::Quaterniond::Identity(); // No rotation
        std::cout << baseQuaternion.coeffs().transpose() << std::endl;
        Referee::Mapping::Pose pose = Referee::Mapping::Pose(initialTranslationVectors[i], baseQuaternion);
        Referee::Mapping::Scan scan = Referee::Mapping::Scan(pose, plyFileNames[i], 0.01);
        scans.push_back(scan);
    }

    std::vector<std::vector<int>> matrix = Referee::Mapping::CreateConnectivityMatrix(initialTranslationVectors, 3, 30);
    Referee::Mapping::MappingMatrix mappingMatrix(numberFiles);
    mappingMatrix.SetConnectivityMatrix(matrix);
    mappingMatrix.SetInitialPositions(initialTranslationVectors);
    mappingMatrix.SetScans(scans);

    Referee::Mapping::Graph& graph = Referee::Mapping::Graph::CreateUndirectedGraph(initialTranslationVectors, matrix);
    graph.PrintGraph();

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredFinalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Limit the number of concurrent threads to 10
    const int maxThreads = 16;
    std::vector<std::thread> threads;
    std::vector<std::tuple<int, int>> jobs;

    // Prepare all jobs
    for (int i = 0; i < matrix.size(); i++)
    {
        for (int j = 0; j < matrix[i].size(); j++)
        {
            jobs.emplace_back(i, matrix[i][j]);
        }
    }

    size_t jobIndex = 0;
    while (jobIndex < jobs.size())
    {
        // Launch up to maxThreads threads
        threads.clear();
        for (int t = 0; t < maxThreads && jobIndex < jobs.size(); ++t, ++jobIndex)
        {
            int i = std::get<0>(jobs[jobIndex]);
            int neighborIndex = std::get<1>(jobs[jobIndex]);
            Eigen::Vector3d initialTranslationSource = initialTranslationVectors[i];
            Eigen::Vector3d initialTranslationTarget = initialTranslationVectors[neighborIndex];

            std::cout << "Computing transformation between point cloud " << i << " and point cloud " << neighborIndex << std::endl;

            threads.emplace_back(ComputeTransformationInThread,
                                 i,
                                 neighborIndex,
                                 scans,
                                 0.01,
                                 std::ref(mappingMatrix));
        }
        // Wait for all threads in this batch to finish
        for (auto& thread : threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

    std::tuple<int, double> mostProbableRotation = mappingMatrix.GetMostProbableRotation();
    std::cout << "Vertex Count: " << mappingMatrix.GetGraph().GetVertexCount() << std::endl;
    mappingMatrix.GetGraph().PrintGraph();
    mappingMatrix.GetGraph().ComputeMinimumSpanningTree(0);
    double overallMeanRotation = mappingMatrix.GetOverallMeanRotation();
    for(int i = 0; i < mappingMatrix.GetGraph().GetMinimumSpanningTree().size(); i++)
    {
        std::pair<int, int> edge = mappingMatrix.GetGraph().GetMinimumSpanningTree()[i];
        std::cout << "Edge from point cloud " << edge.first << " to point cloud " << edge.second << std::endl;
    }

    for(int i = mappingMatrix.GetGraph().GetMinimumSpanningTree().size() - 1; i >= 0; i--)
    {
        std::pair<int, int> edge = mappingMatrix.GetGraph().GetMinimumSpanningTree()[i];
        Eigen::Matrix4d transformation = mappingMatrix.GetTransformation(edge.second, edge.first).GetTransformationMatrix();
        std::cout << "[DEBUG] From point cloud " << edge.second << " to point cloud " << edge.first << ": " << std::endl << transformation << std::endl;

        // apply the transformation to all the point clouds in the subtree
        for(int j = 0; j < mappingMatrix.GetGraph().ExtractMSTSubTree(edge.second).size(); j++)
        {
            int indexInSubtree = mappingMatrix.GetGraph().ExtractMSTSubTree(edge.second)[j];
            std::cout << "[DEBUG] Applying transformation to point cloud " << indexInSubtree << std::endl;
            mappingMatrix.GetScan(indexInSubtree).LoadCloud();
            mappingMatrix.GetScan(indexInSubtree).TransformScan(transformation);
            Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(
                mappingMatrix.GetScan(indexInSubtree).GetCloud(), 0.015);
        }
    }
    // also load the root of the MST
    mappingMatrix.GetScan(mappingMatrix.GetGraph().GetMinimumSpanningTree()[0].first).LoadCloud();
    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(
        mappingMatrix.GetScan(mappingMatrix.GetGraph().GetMinimumSpanningTree()[0].first).GetCloud(),
        0.015);

    Eigen::Matrix4d umeyamaTransformation = mappingMatrix.ComputeUmeyamaTransformationInSubtree(0);
    std::cout << "[DEBUG] final Umeyama transformation: " << std::endl << umeyamaTransformation << std::endl;

    auto catcher = mappingMatrix.GetGraph().GetCorrectionLoops();
    std::vector<int> MSTOrder;
    MSTOrder.push_back(mappingMatrix.GetGraph().GetMinimumSpanningTree()[0].first); // add the root of the MST at the beginning
    for(std::pair<long unsigned int, long unsigned int> edge : mappingMatrix.GetGraph().GetMinimumSpanningTree())
    {
        MSTOrder.push_back(edge.second);
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    int count = 0;
    for(int i : MSTOrder)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        mappingMatrix.GetScan(i).TransformScan(umeyamaTransformation);
        pcl::PointCloud<pcl::PointNormal>::Ptr transformedCloud = mappingMatrix.GetScan(i).GetCloud();
        std::cout << "Transformed point cloud " << i << " has " << transformedCloud->size() << " points." << std::endl;
        for(int j = 0; j < transformedCloud->size(); j++)
        {
            pcl::PointNormal point = transformedCloud->points[j];
            pcl::PointXYZRGB coloredPoint;
            coloredPoint.x = point.x;
            coloredPoint.y = point.y;
            coloredPoint.z = point.z;
            uint8_t r = (((count+1) * 20) % 256);
            uint8_t g = (((count+1) * 40) % 256);
            uint8_t b = (((count+1) * 10) % 256);
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            coloredPoint.rgb = *reinterpret_cast<float*>(&rgb);
            coloredCloud->points.push_back(coloredPoint);
        }
        Referee::Utils::Conversions::CreateLASFromPointCloud(coloredCloud, 
                                                      meanTranslation(0), 
                                                      meanTranslation(1), 
                                                      meanTranslation(2), 
                                                      "intermediate_point_cloud_" + std::to_string(i) + ".las",
                                                      coordSys);
        *finalPointCloud += *coloredCloud;
        mappingMatrix.GetScan(i).FlushCloud();
        coloredCloud->clear();
        count++;
    }
    std::cout << "Final point cloud has " << finalPointCloud->size() << " points." << std::endl;
    Referee::Utils::Conversions::CreateLASFromPointCloud(finalPointCloud, 
                                                      meanTranslation(0), 
                                                      meanTranslation(1), 
                                                      meanTranslation(2), 
                                                      "final_point_cloud.las",
                                                      coordSys);
    return 0;
}