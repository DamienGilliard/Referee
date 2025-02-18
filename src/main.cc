#include "Referee.hh"

int main() {
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

    std::vector<double> meanTranslationVector = {0, 0, 0};
    for (const auto& translationVector : translationVectors) {
        meanTranslationVector[0] += translationVector[0];
        meanTranslationVector[1] += translationVector[1];
        meanTranslationVector[2] += translationVector[2];
    }
    meanTranslationVector[0] /= translationVectors.size();
    meanTranslationVector[1] /= translationVectors.size();
    meanTranslationVector[2] /= translationVectors.size();

    for (auto& translationVector : translationVectors) {
        translationVector[0] -= meanTranslationVector[0];
        translationVector[1] -= meanTranslationVector[1];
        translationVector[2] -= meanTranslationVector[2];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < plyFileNames.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);

        std::cout << "Processing file " << plyFileNames[i] << std::endl;

        if (pcl::io::loadPLYFile<pcl::PointXYZ>(plyFileNames[i], *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", plyFileNames[i].c_str());
            return 0;
        }
        
        Referee::Utils::Filtering::CropPointCloud(pointCloud, -25, -25, 0, 25, 25, 8000);
        Referee::Utils::Filtering::VoxelizePointCloud(pointCloud, 0.05);
        Referee::Transformations::TranslatePointCloud<pcl::PointXYZ>(pointCloud, translationVectors[i]);
        
        *mergedCloud += *pointCloud;
    }

    std::string mergedCloudFileName = "../test_files/registred_scans/merged_cloud.ply";
    pcl::io::savePLYFile(mergedCloudFileName, *mergedCloud);
    // FileBasedVisualisation::Visualisation visualisation;
    // visualisation.VisualisePointCloud(mergedCloudFileName);

    return 0;
}