#include "Multithreading.hh"

void Referee::Multithreading::ComputeTransformationInThread(int sourcePointCloudFileIndex,
                                                            int targetPointCloudFileIndex,
                                                            std::string sourcePointCloudFile,
                                                            std::string targetPointCloudFile,
                                                            double voxelSize,
                                                            Referee::Mapping::MappingMatrix& mappingMatrix,
                                                            std::mutex& mutex)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointNormal>::Ptr targetCloud(new pcl::PointCloud<pcl::PointNormal>());
    
    if (pcl::io::loadPLYFile(sourcePointCloudFile, *sourceCloud) == -1)
    {
        std::cerr << "Failed to load source PLY file: " << sourcePointCloudFile << std::endl;
        return;
    }
    if (pcl::io::loadPLYFile(targetPointCloudFile, *targetCloud) == -1)
    {
        std::cerr << "Failed to load target PLY file: " << targetPointCloudFile << std::endl;
        return;
    }
    
    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(sourceCloud, voxelSize);
    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(targetCloud, voxelSize);
    std::pair<Eigen::Matrix4d, float> transformationMatrixAndScore = Referee::Mapping::ComputePairwiseTransformation(sourceCloud,
                                                                                        targetCloud,
                                                                                        Referee::Mapping::TransformationComputationMethod::GlobalMatch);
    Referee::Mapping::Transformation transformation(transformationMatrixAndScore.first, nullptr, nullptr);
        
    std::lock_guard<std::mutex> lock(mutex);
    mappingMatrix.GetGraph().SetWeight(sourcePointCloudFileIndex, targetPointCloudFileIndex, -transformationMatrixAndScore.second);
    mappingMatrix.SetTransformation(sourcePointCloudFileIndex, targetPointCloudFileIndex, transformation.GetInverse());
    mappingMatrix.SetTransformation(targetPointCloudFileIndex, sourcePointCloudFileIndex, transformation);
}


void Referee::Multithreading::SaveEdgeAlignmentInThread(int aIndex,
                                int bIndex,
                                std::string aFile,
                                std::string bFile,
                                double score,
                                double voxelSize,
                                Referee::Mapping::MappingMatrix& mappingMatrix,
                                std::mutex& mutex)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudA(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudB(new pcl::PointCloud<pcl::PointNormal>());
    if (pcl::io::loadPLYFile(aFile, *cloudA) == -1)
    {
        std::cerr << "[EDGE] Failed to load PLY file: " << aFile << std::endl;
        return;
    }
    if (pcl::io::loadPLYFile(bFile, *cloudB) == -1)
    {
        std::cerr << "[EDGE] Failed to load PLY file: " << bFile << std::endl;
        return;
    }

    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(cloudA, voxelSize);
    Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(cloudB, voxelSize);

    // GetTransformation(a, b) maps b's local frame into a's local frame.
    mutex.lock();
    const Eigen::Matrix4d Tab = mappingMatrix.GetTransformation(aIndex, bIndex).GetTransformationMatrix();
    mutex.unlock();
    Referee::Transformations::TransformPointCloud<pcl::PointNormal>(cloudB, Tab);

    const std::string prefix = "edge_" + std::to_string(aIndex) + "_" + std::to_string(bIndex)
                              + "_score" + std::to_string(static_cast<int>(score));
    pcl::io::savePLYFileBinary(prefix + "_target_" + std::to_string(aIndex) + ".ply", *cloudA);
    pcl::io::savePLYFileBinary(prefix + "_aligned_" + std::to_string(bIndex) + ".ply", *cloudB);
    std::cout << "[EDGE] Saved alignment for edge " << aIndex << " -> " << bIndex
              << " (score " << score << ")" << std::endl;
}