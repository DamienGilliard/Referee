#include "Referee.hh"

int main()
{
    std::vector<std::string> geolocationFiles = Referee::Utils::FileIterators::GetFilesInDirectory("../../test_files/geolocations", ".petitpoucet");
    std::vector<std::string> plyFileNames = Referee::Utils::FileIterators::GetFilesInDirectory("../../test_files/scans", ".ply");

    if (plyFileNames.size() != geolocationFiles.size()) 
    {
        std::cerr << "Number of geolocation files and ply files do not match" << std::endl;
        return 1;
    }

    Referee::Utils::CoordinateSystem::CoordinateSystem coordSys = Referee::Utils::CoordinateSystem::CoordinateSystem::LV95;
    std::vector<Eigen::Vector3d> translationVectors;
    for (const auto& file : geolocationFiles) 
    {
        Eigen::Vector3d translationVector = Referee::Utils::FileIterators::GetTranslationVectorsFromFile(file, coordSys);
        translationVectors.push_back(translationVector);
    }

    Referee::Mapping::MappingMatrix mappingMatrix(plyFileNames.size());

    Eigen::Vector3d meanTranslation =Referee::Transformations::RecenterTranslationVectors(translationVectors);
    std::cout << "Mean translation vector: " << meanTranslation[0] << " " << meanTranslation[1] << " " << meanTranslation[2] << std::endl;
    std::vector<std::vector<int>> matrix;
    Referee::Mapping::CreateConnectivityMatrix(translationVectors, 2, 30, matrix);

    std::cout << "____________________Connectivity matrix:____________________" << std::endl;
    for(int i = 0; i < matrix.size(); i++)
    {
        std::cout << "Node " << i << " is connected to nodes: ";
        for(int j = 0; j < matrix[i].size(); j++)
        {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "___________________________________________________________" << std::endl;

    for (int i = 0; i < translationVectors.size(); i++)
    {
        std::cout << "Translation vector for point cloud " << i << ": " << translationVectors[i][0] << " " << translationVectors[i][1] << " " << translationVectors[i][2] << std::endl;
    }
    std::cout << "___________________________________________________________" << std::endl;

    int count = 0;
    int totCount = plyFileNames.size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredFinalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Calculate the different relative transformations
    for (int i = 0; i < matrix.size(); i++)
    {
        for (int j = 0; j < matrix[i].size(); j++)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr temporarySourcePointCloud(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr temporaryTargetPointCloud(new pcl::PointCloud<pcl::PointNormal>);
            std::cout << "Computing transformation between point cloud " << i << " and point cloud " << matrix[i][j] << std::endl;
            std::cout << "Loading point cloud " << i << std::endl;
            if (pcl::io::loadPLYFile<pcl::PointNormal>(plyFileNames[i], *temporarySourcePointCloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s \n", plyFileNames[i].c_str());
                return 0;
            }
            std::cout << "Loading point cloud " << matrix[i][j] << std::endl;
            if (pcl::io::loadPLYFile<pcl::PointNormal>(plyFileNames[matrix[i][j]], *temporaryTargetPointCloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s \n", plyFileNames[matrix[i][j]].c_str());
                return 0;
            }
            std::cout << "Cropping point cloud " << i << std::endl;
            Referee::Utils::Filtering::CropPointCloud<pcl::PointNormal>(temporaryTargetPointCloud, -40, -40, -100, 40, 40, 8000);
            std::cout << "Downsampling point cloud " << i << std::endl;
            Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(temporaryTargetPointCloud, 0.05);
            Referee::Utils::Filtering::CropPointCloud<pcl::PointNormal>(temporarySourcePointCloud, -40, -40, -100, 40, 40, 8000);
            std::cout << "Downsampling point cloud " << j << std::endl;
            Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(temporarySourcePointCloud, 0.05);
            Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(temporarySourcePointCloud, translationVectors[i]);
            Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(temporaryTargetPointCloud, -translationVectors[j]);
            Eigen::Matrix4d transformationMatrix = Referee::Mapping::ComputePairwiseTransformation(temporarySourcePointCloud, temporaryTargetPointCloud, Referee::Mapping::TransformationComputationMethod::GlobalMatch);
            mappingMatrix.SetTransformationMatrix(i, j, transformationMatrix);
            
        }
    }

    mappingMatrix.PrintMatrix();
    mappingMatrix.CalculateMeanTransformationMatrices();
    mappingMatrix.PrintMeanMatrices();

    return 0;
}