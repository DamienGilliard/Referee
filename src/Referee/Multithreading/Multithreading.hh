#pragma once

#include <mutex>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include "../Mapping/Mapping.hh"

namespace Referee::Multithreading
{
    /**
     * @brief Computes the transformation between two point clouds in a separate thread and updates the mapping matrix accordingly.
     * 
     * @param sourcePointCloudFileIndex Index of the source point cloud in the mapping matrix
     * @param targetPointCloudFileIndex Index of the target point cloud in the mapping matrix
     * @param sourcePointCloudFile File path of the source point cloud (PLY format)
     * @param targetPointCloudFile File path of the target point cloud (PLY format)
     * @param voxelSize Voxel size for downsampling the point clouds
     * @param mappingMatrix Reference to the mapping matrix to be updated with the computed transformation
     * @param mutex Reference to a mutex for thread-safe access to the mapping matrix
     */
    void ComputeTransformationInThread(int sourcePointCloudFileIndex,
                    int targetPointCloudFileIndex,
                    std::string sourcePointCloudFile,
                    std::string targetPointCloudFile,
                    double voxelSize,
                    Referee::Mapping::MappingMatrix& mappingMatrix,
                    std::mutex& mutex);

    /**
     * @brief Saves the alignment of two point clouds corresponding to a non-MST connectivity edge in a separate thread. This is to visually inspect whether the signal Prim's algorithm left out of the MST is actually trustworthy or not.
     * 
     * @param aIndex Index of the first point cloud in the mapping matrix
     * @param bIndex Index of the second point cloud in the mapping matrix
     * @param aFile File path of the first point cloud (PLY format)
     * @param bFile File path of the second point cloud (PLY format)
     * @param score Score of the alignment between the two point clouds
     * @param voxelSize Voxel size for downsampling the point clouds
     * @param mappingMatrix Reference to the mapping matrix to be updated with the computed transformation
     * @param mutex Reference to a mutex for thread-safe access to the mapping matrix
     */
    void SaveEdgeAlignmentInThread(int aIndex,
                                int bIndex,
                                std::string aFile,
                                std::string bFile,
                                double score,
                                double voxelSize,
                                Referee::Mapping::MappingMatrix& mappingMatrix,
                                std::mutex& mutex);

}