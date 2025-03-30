#include "Referee.hh"

int main()
{
    std::vector<std::string> geolocationFiles = Referee::Utils::FileIterators::GetFilesInDirectory("../test_files/geolocations", ".petitpoucet");
    std::vector<std::string> plyFileNames = Referee::Utils::FileIterators::GetFilesInDirectory("../test_files/scans", ".ply");
    std::vector<std::vector<double>> translationVectors;

    if (plyFileNames.size() != geolocationFiles.size()) {
        std::cerr << "Number of geolocation files and ply files do not match" << std::endl;
        return 1;
    }

    for (const auto& file : geolocationFiles) {
        std::ifstream fileStream(file);
        std::string line;
        bool firstLine = true; // flag to skip the header line
        while (std::getline(fileStream, line)) 
        {
            if (firstLine) {
                firstLine = false;
                continue; // skip the header line
            }
            std::istringstream lineStream(line);
            std::string lat, lon, alt;
            std::getline(lineStream, lon, ';');
            std::getline(lineStream, lat, ';');
            std::getline(lineStream, alt, ';');
            double x, y, z;
            Referee::Utils::Conversions::ConvertLatLonAltToCartesian(std::stod(lat), std::stod(lon), std::stod(alt), y, x, z, Referee::Utils::CoordinateSystem::CoordinateSystem::DEG, Referee::Utils::CoordinateSystem::CoordinateSystem::LV95);
            translationVectors.push_back({x, y, z});
        }
    }

    Referee::Mapping::MappingMatrix mappingMatrix(plyFileNames.size());

    std::vector<double> meanTranslation =Referee::Transformations::RecenterTranslationVectors(translationVectors);
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

    int count = 0;
    int totCount = plyFileNames.size();

    // iterate through all other point clouds, register them and merge them to the reference
    for (int i = 1; i < plyFileNames.size(); i++) 
    {
        // read i-th point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr targetPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(plyFileNames[i], *targetPointCloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", plyFileNames[i].c_str());
            return 0;
        }
        Referee::Transformations::TranslatePointCloud<pcl::PointXYZ>(targetPointCloud, translationVectors[i]);

        // load the j-th point cloud
        for(int j = 0; j < matrix[i].size(); j++)
        {
            if (matrix[i][j] < i)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePointCloud(new pcl::PointCloud<pcl::PointXYZ>);
                if (pcl::io::loadPLYFile<pcl::PointXYZ>(plyFileNames[matrix[i][j]], *sourcePointCloud) == -1)
                {
                    PCL_ERROR("Couldn't read file %s \n", plyFileNames[matrix[i][j]].c_str());
                    return 0;
                }
                Referee::Transformations::TranslatePointCloud<pcl::PointXYZ>(sourcePointCloud, translationVectors[matrix[i][j]]);
                Eigen::Matrix4f transformationMatrix = Referee::Mapping::ComputePairwiseTransformation(targetPointCloud, sourcePointCloud, Referee::Mapping::TransformationComputationMethod::GlobalMatch);
                std::cout << "Transformation matrix: " << std::endl << transformationMatrix << std::endl;
                Eigen::Matrix3f rotationMatrixInv = transformationMatrix.block<3, 3>(0, 0).inverse();
                Eigen::Vector3f translationVector = transformationMatrix.block<3, 1>(0, 3);
                Eigen::Vector3f translationVectorInv = -translationVector;
                Eigen::Matrix4f transformationMatrixInv = Eigen::Matrix4f::Identity();
                transformationMatrixInv.block<3, 3>(0, 0) = rotationMatrixInv;
                transformationMatrixInv.block<3, 1>(0, 3) = translationVectorInv;
                mappingMatrix.SetTransformationMatrix(i, matrix[i][j], transformationMatrix);
                mappingMatrix.SetTransformationMatrix(matrix[i][j], i, transformationMatrixInv);
            }
        }
    }

    mappingMatrix.PrintMatrix();
    mappingMatrix.CalculateMeanTransformationMatrices();
    mappingMatrix.PrintMeanMatrices();

    return 0;
}