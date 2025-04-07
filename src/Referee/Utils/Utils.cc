#include "Utils.hh"

namespace Referee::Utils
{
    namespace CoordinateSystem
    {
        void PointCloudGeolocation::WriteToGeojson()
        {
            std::ofstream geojsonFile(fileName + ".geojson");
            if (geojsonFile.is_open())
            {
                geojsonFile << "{\n";
                geojsonFile << "  \"type\": \"FeatureCollection\",\n";
                geojsonFile << "  \"features\": [\n";
                geojsonFile << "    {\n";
                geojsonFile << "      \"type\": \"Feature\",\n";
                geojsonFile << "      \"geometry\": {\n";
                geojsonFile << "        \"type\": \"Point\",\n";
                geojsonFile << "        \"coordinates\": [" << lon << ", " << lat << ", " << alt << "]\n";
                geojsonFile << "      },\n";
                geojsonFile << "      \"properties\": {\n";
                geojsonFile << "        \"coordinate_system\": \"" << static_cast<int>(coordSys) << "\"\n"; // Store enum as int
                geojsonFile << "      }\n";
                geojsonFile << "    }\n";
                geojsonFile << "  ]\n";
                geojsonFile << "}\n";

                geojsonFile.close();
            }
            else
            {
                std::cerr << "Unable to open file: " + fileName + ".geojson" << std::endl;
            }
        }
    } // CoordinateSystem
    namespace Conversions
    {
        void ConvertLatLonAltToCartesian(double lat, double lon, double alt, double &x, double &y, double &z, CoordinateSystem::CoordinateSystem fromCoordSys, CoordinateSystem::CoordinateSystem toCoordSys)
        {
            if(fromCoordSys == CoordinateSystem::CoordinateSystem::DEG && toCoordSys == CoordinateSystem::CoordinateSystem::LV95)
            {
                // according to https://backend.swisstopo.admin.ch/fileservice/sdweb-docs-prod-swisstopoch-files/files/2023/11/14/7f7bf15b-22e2-48b6-b1ab-6905f81dca8a.pdf
                // the conversion from WGS84 to CH1903/LV95 is done by the following formulas
                // conversion from long lat to arcsec:
                double latArcSec = lat * 3600;
                double lonArcSec = lon * 3600;
                // calculation of auxillary values
                double phi = (latArcSec - 169028.66) / 10000;
                double lambda = (lonArcSec - 26782.5) / 10000;
                // calculation of the coordinates
                y = 2600072.37 + 211455.93 * lambda - 10938.51 * lambda * phi - 0.36 * lambda * phi * phi - 44.54 * lambda * lambda * lambda;
                x = 1200147.07 + 308807.95 * phi + 3745.25 * lambda * lambda + 76.63 * phi * phi - 194.56 * lambda * lambda * phi + 119.79 * phi * phi * phi;

                z = alt - 49.55 + 2.73 * lambda + 6.94 * phi;
            }
            else if (fromCoordSys == CoordinateSystem::CoordinateSystem::DMS && toCoordSys == CoordinateSystem::CoordinateSystem::LV95)
            {
                // convert DMS to DEG
                double convertedDegrees = int(lat);
                double convertedMinutes = int((lat - convertedDegrees) * 100);
                double convertedSeconds = (lat - convertedDegrees - convertedMinutes / 100) * 10000;
                double latDeg = convertedDegrees + convertedMinutes/60 + convertedSeconds/3600; ;

                convertedDegrees = int(lon);
                convertedMinutes = int((lon - convertedDegrees) * 100);
                convertedSeconds = (lon - convertedDegrees - convertedMinutes / 100) * 10000;
                double lonDeg = convertedDegrees + convertedMinutes/60 + convertedSeconds/3600;

                ConvertLatLonAltToCartesian(latDeg, lonDeg, alt, x, y, z, CoordinateSystem::CoordinateSystem::DEG, CoordinateSystem::CoordinateSystem::LV95);
            }
            
        }
    }
    namespace FileIterators
    {
        std::vector<std::string> GetFilesInDirectory(const std::string& directory, const std::string& extension)
        {
            std::vector<std::string> files;
            for (const auto& entry : std::filesystem::directory_iterator(directory)) 
            {
                if (entry.path().extension() == extension) 
                {
                    files.push_back(entry.path().string());
                }
            }
            std::sort(files.begin(), files.end());
            return files;
        }

        Eigen::Vector3d GetTranslationVectorsFromFile(const std::string& filePath, Referee::Utils::CoordinateSystem::CoordinateSystem coordSys)
        {
            Eigen::Vector3d translationVector;
            std::ifstream fileStream(filePath);
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
                translationVector.x() = x;
                translationVector.y() = y;
                translationVector.z() = z;
            }
            return translationVector;
        }
    }
    
    namespace Coloring
    {
        void ColorPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b)
        {
            for(auto& point : *cloud)
            {
                pcl::PointXYZRGB coloredPoint;
                coloredPoint.x = point.x;
                coloredPoint.y = point.y;
                coloredPoint.z = point.z;
                coloredPoint.r = r;
                coloredPoint.g = g;
                coloredPoint.b = b;
                coloredCloud->push_back(coloredPoint);
            }
        }
    }
    namespace NormalCalculation
    {
        void CalculateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int k)
        {
            std::cout << "Calculating normals..." << std::endl;
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
            normalEstimation.setInputCloud(cloud);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            normalEstimation.setSearchMethod(tree);
            normalEstimation.setKSearch(k);
            normalEstimation.compute(*normals);
            std::cout << "Normals calculated." << std::endl;
        }
    }
}