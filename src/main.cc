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
    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(scans[sourcePointCloudFileIndex].GetCloud(), voxelSize);
    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(scans[targetPointCloudFileIndex].GetCloud(), voxelSize);

    Eigen::Matrix4d transformationMatrix = Referee::Mapping::ComputePairwiseTransformation(scans[sourcePointCloudFileIndex].GetCloud(), scans[targetPointCloudFileIndex].GetCloud(),
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

    std::vector<Referee::Mapping::Scan> scans;
    int nFiles = initialTranslationVectors.size();
    for (int i = 0; i < nFiles; i++)
    {
        Eigen::Quaterniond baseQuaternion = Eigen::Quaterniond::Identity(); // No rotation
        std::cout << baseQuaternion.coeffs().transpose() << std::endl;
        Referee::Mapping::Pose pose = Referee::Mapping::Pose(initialTranslationVectors[i], baseQuaternion);
        Referee::Mapping::Scan scan = Referee::Mapping::Scan(pose, plyFileNames[i]);
        Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(scan.GetCloud(), initialTranslationVectors[i]);
        scans.push_back(scan);
    }

    std::vector<std::vector<int>> matrix = Referee::Mapping::CreateConnectivityMatrix(initialTranslationVectors, 3, 30);
    Referee::Mapping::MappingMatrix mappingMatrix(numberFiles);
    mappingMatrix.SetConnectivityMatrix(matrix);
    mappingMatrix.SetInitialPositions(initialTranslationVectors);

    Referee::Mapping::Graph& graph = Referee::Mapping::Graph::CreateUndirectedGraph(initialTranslationVectors, matrix);
    graph.PrintGraph();
    
    std::vector<std::vector<Eigen::Vector3d>> computedTranslations;
    computedTranslations.resize(numberFiles);

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredFinalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Limit the number of concurrent threads to 10
    const int maxThreads = 10;
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
    std::cout << "Vertex Count: " << mappingMatrix.GetGraph().GetVertexCount() << std::endl;
    mappingMatrix.GetGraph().ComputeMinimumSpanningTree(std::get<0>(mostProbableRotation));
    double overallMeanRotation = mappingMatrix.GetOverallMeanRotation();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i = 0; i < mappingMatrix.GetGraph().GetMinimumSpanningTree().size(); i++)
    {
        std::pair<int, int> edge = mappingMatrix.GetGraph().GetMinimumSpanningTree()[i];
        std::cout << "Edge from point cloud " << edge.first << " to point cloud " << edge.second << std::endl;
    }

    for(int i = mappingMatrix.GetGraph().GetMinimumSpanningTree().size() - 1; i >= 0; i--)
    {
        std::pair<int, int> edge = mappingMatrix.GetGraph().GetMinimumSpanningTree()[i];
        std::vector<int> subTree = mappingMatrix.GetGraph().ExtractMSTSubTree(i);
        std::cout << "Subtree: ";
        for (int j = 0; j < subTree.size(); j++)
        {
            std::cout << subTree[j] << " ";
        }
        Eigen::Matrix4d resetRotations = Eigen::Matrix4d::Identity();
        resetRotations.block<3,3>(0,0) = Eigen::AngleAxisd(-overallMeanRotation, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix4d initialTranslation = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d reversedInitialTranslation = Eigen::Matrix4d::Identity();
        initialTranslation.block<3,1>(0,3) = initialTranslationVectors[edge.first];
        reversedInitialTranslation.block<3,1>(0,3) = -initialTranslationVectors[edge.first];

        Eigen::Matrix4d transformation = initialTranslation * resetRotations * reversedInitialTranslation;
        Referee::Transformations::TransformPointCloud<pcl::PointNormal>(scans[edge.second].GetCloud(), transformation);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*scans[edge.second].GetCloud(), *coloredPointCloud);
        for (int j = 0; j < coloredPointCloud->size(); j++)
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