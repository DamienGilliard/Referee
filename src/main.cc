#include "Referee.hh"

int main()
{
    std::vector<std::string> geolocationFiles = Referee::Utils::FileIterators::GetFilesInDirectory("../../test_files/geolocations", ".petitpoucet");
    std::vector<std::string> plyFileNames = Referee::Utils::FileIterators::GetFilesInDirectory("../../test_files/scans", ".ply");
    std::vector<std::vector<double>> translationVectors;

    if (plyFileNames.size() != geolocationFiles.size()) {
        std::cerr << "Number of geolocation files and ply files do not match" << std::endl;
        return 1;
    }

    Referee::Utils::CoordinateSystem::CoordinateSystem coordSys = Referee::Utils::CoordinateSystem::CoordinateSystem::LV95;

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
            Referee::Utils::Conversions::ConvertLatLonAltToCartesian(std::stod(lat), std::stod(lon), std::stod(alt), y, x, z, Referee::Utils::CoordinateSystem::CoordinateSystem::DEG, coordSys);
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

    for (int i = 0; i < translationVectors.size(); i++)
    {
        std::cout << "Translation vector for point cloud " << i << ": " << translationVectors[i][0] << " " << translationVectors[i][1] << " " << translationVectors[i][2] << std::endl;
    }
    std::cout << "___________________________________________________________" << std::endl;

    int count = 0;
    int totCount = plyFileNames.size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredFinalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // iterate through all other point clouds, register them and merge them to the reference

    if (pcl::io::loadPLYFile<pcl::PointXYZ>(plyFileNames[0], *targetPointCloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", plyFileNames[0].c_str());
        return 0;
    }
    std::cout << "cropping point cloud " << std::endl;
    Referee::Utils::Filtering::CropPointCloud(targetPointCloud, -40, -40, -100, 40, 40, 8000);
    std::cout << "voxelizing point cloud " << std::endl;
    Referee::Utils::Filtering::VoxelizePointCloud(targetPointCloud, 0.025);

    pcl::PointCloud<pcl::Normal>::Ptr targetNormals(new pcl::PointCloud<pcl::Normal>);
    Referee::Utils::NormalCalculation::CalculateNormals(targetPointCloud, targetNormals, 5);
    pcl::PointCloud<pcl::PointNormal>::Ptr targetPointCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
    for (size_t i = 0; i < targetPointCloud->size(); ++i)
    {
        pcl::PointNormal pointWithNormal;
        pointWithNormal.x = targetPointCloud->points[i].x;
        pointWithNormal.y = targetPointCloud->points[i].y;
        pointWithNormal.z = targetPointCloud->points[i].z;
        pointWithNormal.normal_x = targetNormals->points[i].normal_x;
        pointWithNormal.normal_y = targetNormals->points[i].normal_y;
        pointWithNormal.normal_z = targetNormals->points[i].normal_z;
        targetPointCloudWithNormals->push_back(pointWithNormal);
    }
    Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(targetPointCloudWithNormals, translationVectors[0]);

    // load the j-th point cloud
    for(int j = 1; j < plyFileNames.size(); j++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(plyFileNames[j], *sourcePointCloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", plyFileNames[j].c_str());
            return 0;
        }
        Referee::Utils::Filtering::CropPointCloud(sourcePointCloud, -40, -40, -100, 40, 40, 8000);
        Referee::Utils::Filtering::VoxelizePointCloud(sourcePointCloud, 0.025);

        // compute the normals of the source point cloud
        pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
        Referee::Utils::NormalCalculation::CalculateNormals(sourcePointCloud, sourceNormals, 5);
        pcl::PointCloud<pcl::PointNormal>::Ptr sourcePointCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
        for (size_t k = 0; k < sourcePointCloud->size(); ++k)
        {
            pcl::PointNormal pointWithNormal;
            pointWithNormal.x = sourcePointCloud->points[k].x;
            pointWithNormal.y = sourcePointCloud->points[k].y;
            pointWithNormal.z = sourcePointCloud->points[k].z;
            pointWithNormal.normal_x = sourceNormals->points[k].normal_x;
            pointWithNormal.normal_y = sourceNormals->points[k].normal_y;
            pointWithNormal.normal_z = sourceNormals->points[k].normal_z;
            sourcePointCloudWithNormals->push_back(pointWithNormal);
        }
        Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(sourcePointCloudWithNormals, translationVectors[j]);
        Eigen::Matrix4d transformationMatrix = Referee::Mapping::ComputePairwiseTransformation(sourcePointCloudWithNormals, targetPointCloudWithNormals, Referee::Mapping::TransformationComputationMethod::GlobalMatch);
        std::cout << "Transformation matrix: " << std::endl << transformationMatrix << std::endl;
        std::vector<Eigen::Vector3d> screwAxis = Referee::Mapping::ComputeScrewAxis(transformationMatrix);
        std::cout << "Screw axis: " << screwAxis[0][0] << " " << screwAxis[0][1] << " " << screwAxis[0][2] << std::endl;
        std::cout << "Translation along screw axis: " << screwAxis[1][0] << " " << screwAxis[1][1] << " " << screwAxis[1][2] << std::endl;
        std::cout << "Point on screw axis: " << screwAxis[2][0] << " " << screwAxis[2][1] << " " << screwAxis[2][2] << std::endl;
        Referee::Transformations::TransformPointCloud<pcl::PointNormal>(sourcePointCloudWithNormals, transformationMatrix);
            
        // Eigen::Matrix4d refinedTransformationMatrix = Referee::Mapping::RefinePairwiseTransformation(sourcePointCloudWithNormals, targetPointCloudWithNormals, Referee::Mapping::RefinementMethod::ICPNormals, 0.1);
        // std::cout << "Refined transformation matrix: " << std::endl << refinedTransformationMatrix << std::endl;
        // Referee::Transformations::TransformPointCloud<pcl::PointNormal>(sourcePointCloudWithNormals, refinedTransformationMatrix);

        for (auto& point : *sourcePointCloudWithNormals)
        {
            targetPointCloudWithNormals->push_back(point);
            pcl::PointXYZ pointxyz;
            pointxyz.x = point.x;
            pointxyz.y = point.y;
            pointxyz.z = point.z;
            targetPointCloud->push_back(pointxyz);
            pcl::PointXYZRGB coloredPoint;
            coloredPoint.x = point.x;
            coloredPoint.y = point.y;
            coloredPoint.z = point.z;
            coloredPoint.r = 255*(double(j+1))/double(totCount);
            coloredPoint.g = 255 - (double(j+1)/double(totCount))*255;
            coloredPoint.b = 5*j;
            coloredFinalPointCloud->push_back(coloredPoint);
        }
        Eigen::Matrix3d rotationMatrixInv = transformationMatrix.block<3, 3>(0, 0).inverse();
        Eigen::Vector3d translationVector = transformationMatrix.block<3, 1>(0, 3);
        Eigen::Vector3d translationVectorInv = -translationVector;
        Eigen::Matrix4d transformationMatrixInv = Eigen::Matrix4d::Identity();
        transformationMatrixInv.block<3, 3>(0, 0) = rotationMatrixInv;
        transformationMatrixInv.block<3, 1>(0, 3) = translationVectorInv;
    }

    Referee::Utils::CoordinateSystem::PointCloudGeolocation geolocation(translationVectors[0][0] + meanTranslation[0], translationVectors[0][1]+ meanTranslation[1], translationVectors[0][2]+ meanTranslation[2], coordSys);
    geolocation.fileName = "../test_files/registred_scans/geolocation";
    geolocation.WriteToGeojson();
    std::string filename_result = "../test_files/registred_scans/result.ply";
    pcl::io::savePLYFile<pcl::PointXYZRGB>(filename_result, *coloredFinalPointCloud);

    // mappingMatrix.PrintMatrix();
    // mappingMatrix.CalculateMeanTransformationMatrices();
    // mappingMatrix.PrintMeanMatrices();

    return 0;
}