#include "Referee.hh"


std::mutex mutex;
// A function we will run in multiple threads
void ComputeTransformationInThread(Eigen::Vector3d initialTranslationSource,
                            Eigen::Vector3d initialTranslationTarget,
                            int sourcePointCloudFileIndex,
                            int targetPointCloudFileIndex,
                            std::vector<std::string>& sourcePointCloudFileNames,
                            double voxelSize,
                            Referee::Mapping::MappingMatrix& mappingMatrix)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr targetCloud(new pcl::PointCloud<pcl::PointNormal>);
    if (pcl::io::loadPLYFile<pcl::PointNormal>(sourcePointCloudFileNames[sourcePointCloudFileIndex], *sourceCloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", sourcePointCloudFileNames[sourcePointCloudFileIndex].c_str());
        return;
    }
    if (pcl::io::loadPLYFile<pcl::PointNormal>(sourcePointCloudFileNames[targetPointCloudFileIndex], *targetCloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", sourcePointCloudFileNames[targetPointCloudFileIndex].c_str());
        return;
    }

    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(targetCloud, voxelSize);
    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(sourceCloud, voxelSize);

    Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(sourceCloud, initialTranslationSource);
    Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(targetCloud, initialTranslationTarget);

    Eigen::Matrix4d transformationMatrix = Referee::Mapping::ComputePairwiseTransformation(sourceCloud, targetCloud, 
                                                                                      Referee::Mapping::TransformationComputationMethod::GlobalMatch);

    Referee::Mapping::Transformation transformation(transformationMatrix);
    mutex.lock();
    mappingMatrix.SetTransformation(sourcePointCloudFileIndex, targetPointCloudFileIndex, transformation);
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
    Eigen::Vector3d meanTranslation =Referee::Transformations::RecenterTranslationVectors(initialTranslationVectors);

    std::vector<std::vector<int>> matrix;
    Referee::Mapping::CreateConnectivityMatrix(initialTranslationVectors, 3, 30, matrix);
    Referee::Mapping::MappingMatrix mappingMatrix(numberFiles);
    mappingMatrix.SetConnectivityMatrix(matrix);
    mappingMatrix.SetInitialPositions(initialTranslationVectors);

    Referee::Mapping::Graph graph = Referee::Mapping::Graph::CreateUndirectedGraph(initialTranslationVectors, matrix);
    graph.PrintGraph();
    
    std::vector<std::vector<Eigen::Vector3d>> computedTranslations;
    computedTranslations.resize(numberFiles);

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredFinalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Limit the number of concurrent threads to 10
    const int maxThreads = 6;
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
                                 initialTranslationSource,
                                 initialTranslationTarget,
                                 i,
                                 neighborIndex,
                                 std::ref(plyFileNames),
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
    std::vector<std::pair<long unsigned int, long unsigned int>> minimumSpanningTree = graph.ComputeMinimumSpanningTree(initialTranslationVectors[std::get<0>(mostProbableRotation)]);
    std::cout << "Minimum spanning tree edges:" << std::endl;
    for (const auto& edge : minimumSpanningTree)
    {
        std::cout << "Edge: " << edge.first << " - " << edge.second << std::endl;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointClouds(numberFiles);
    for(int i = 0; i < numberFiles; i++)
    {
        pointClouds[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPLYFile(plyFileNames[i], *pointClouds[i]);
        Referee::Transformations::TranslatePointCloud<pcl::PointXYZRGB>(pointClouds[i], initialTranslationVectors[i]);
        Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointXYZRGB>(pointClouds[i], 0.15);
    }

    for(int i = minimumSpanningTree.size() - 1; i >= 0; i--)
    {
        std::pair<int, int> edge = minimumSpanningTree[i];
        Eigen::Matrix4d transformation = mappingMatrix.GetTransformation(edge.second, edge.first).GetTransformationMatrix();
        // Align the second to the first and merge the point clouds
        Referee::Transformations::TransformPointCloud<pcl::PointXYZRGB>(pointClouds[edge.second], transformation);
        for (int j = 0; j < pointClouds[edge.second]->size(); j++)
        {
            pointClouds[edge.second]->points[j].r = 50 + (double)200 * ((double)i/(double)(numberFiles-1));
            pointClouds[edge.second]->points[j].g = 0;
            pointClouds[edge.second]->points[j].b = 250 - (double)200 * ((double)i/(double)(numberFiles-1));
        }
        *pointClouds[edge.first] += *pointClouds[edge.second];
    }

    pcl::io::savePLYFile("coloredFinalPointCloud.ply", *pointClouds[std::get<0>(mostProbableRotation)]);
    return 0;
}