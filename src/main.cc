#include "Referee.hh"

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
    std::vector<Eigen::Vector3d> translationVectors;
    for (const auto& file : geolocationFiles) 
    {
        Eigen::Vector3d translationVector = Referee::Utils::FileIterators::GetTranslationVectorsFromFile(file, coordSys);
        translationVectors.push_back(translationVector);
    }

    Referee::Mapping::MappingMatrix mappingMatrix(numberFiles);

    Eigen::Vector3d meanTranslation =Referee::Transformations::RecenterTranslationVectors(translationVectors);
    std::cout << "Mean translation vector: " << meanTranslation[0] << " " << meanTranslation[1] << " " << meanTranslation[2] << std::endl;
    std::vector<std::vector<int>> matrix;
    Referee::Mapping::CreateConnectivityMatrix(translationVectors, 3, 30, matrix);

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
            std::cout << "Cropping point cloud " << matrix[i][j] << std::endl;
            Referee::Utils::Filtering::CropPointCloud<pcl::PointNormal>(temporarySourcePointCloud, -40, -40, -100, 40, 40, 8000);
            std::cout << "Downsampling point cloud " << matrix[i][j] << std::endl;
            Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(temporarySourcePointCloud, 0.05);
            Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(temporarySourcePointCloud, translationVectors[i]);
            Eigen::Matrix4d transformationMatrix = Referee::Mapping::ComputePairwiseTransformation(temporarySourcePointCloud, temporaryTargetPointCloud, Referee::Mapping::TransformationComputationMethod::GlobalMatch);
            Referee::Mapping::ChaslesTransformation chaslesTransformation(transformationMatrix);
            mappingMatrix.SetChaslesTransformation(i, matrix[i][j], chaslesTransformation);
            std::cout << "Chasles rotation center: " << chaslesTransformation.GetPointOnAxis() << std::endl;
        }
    }

    mappingMatrix.PrintMatrix();
    mappingMatrix.CalculateMeanTransformationMatrices();
    mappingMatrix.PrintMeanMatrices();

    std::vector<std::tuple<int, double, double>> rotationsPerPC;
    for(int i = 0; i < numberFiles; i++)
    {
        rotationsPerPC.push_back(std::make_tuple(i, mappingMatrix.GetMeanRotation(i), mappingMatrix.GetStdDevRotation(i)));
    }

    mappingMatrix.ComputeRotationCoefficients(std::get<1>(rotationsPerPC[0]), 0);

    std::cout << "Probabilities of rotation angles: " << std::endl;
    for(int i = 0; i < rotationsPerPC.size(); i++)
    {
        std::cout << "For Point cloud " << std::get<0>(rotationsPerPC[i]) << std::endl;
        int indice = 0;
        double rotationCoefficient = mappingMatrix.GetRotationCoefficient(i, indice);
        if(rotationCoefficient == 0)
        {
            indice = 1;
            rotationCoefficient = mappingMatrix.GetRotationCoefficient(i, indice);
        }
        double p = Referee::Probability::Compute1DProbabilityDensityFunction(rotationCoefficient * mappingMatrix.GetChaslesTransformation(i, indice).GetRotationAngle(), std::get<1>(rotationsPerPC[i]), std::get<2>(rotationsPerPC[i]));
        std::cout << "Probability of rotation angle: " << p << std::endl;
    }

    std::vector<double> rotationAngles;
    std::cout << "angles to be applied: " << std::endl;
    for(int i = 0; i < numberFiles; i++)
    {
        int indice = 0;
        double angle = mappingMatrix.GetChaslesTransformation(i, indice).GetRotationAngle();
        if(angle == 0)
        {
            indice = 1;
            angle = mappingMatrix.GetChaslesTransformation(i, indice).GetRotationAngle();
        }
        rotationAngles.push_back(angle * mappingMatrix.GetRotationCoefficient(i, indice));
        std::cout << "Angle to be applied to point cloud " << i << ": " << rotationAngles[i] << std::endl;
    }

    for(int i = 0; i < numberFiles; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPLYFile(plyFileNames[i], *coloredPointCloud);
        Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointXYZRGB>(coloredPointCloud, 0.1);
        for(int j = 0; j < coloredPointCloud->size(); j++)
        {
            coloredPointCloud->points[j].r = 255-i*70;
            coloredPointCloud->points[j].g = i*70;
            coloredPointCloud->points[j].b = 200 - i*25;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloudTransformed(new pcl::PointCloud<pcl::PointXYZRGB>);

        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rotationMatrix = Eigen::AngleAxis<double>(rotationAngles[i], Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
        transform.block<3, 3>(0, 0) = rotationMatrix;
        transform.block<3, 1>(0, 3) = translationVectors[i];
        Referee::Transformations::TransformPointCloud<pcl::PointXYZRGB>(coloredPointCloud, transform);

        *coloredFinalPointCloud += *coloredPointCloud;
    }

    pcl::io::savePLYFile("coloredFinalPointCloud.ply", *coloredFinalPointCloud);
    return 0;
}