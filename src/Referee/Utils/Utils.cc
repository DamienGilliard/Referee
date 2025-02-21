#include "Utils.hh"

namespace Referee::Utils
{
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
    }

    namespace Filtering
    {
        void CropPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
        {
            pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter;
            cropBoxFilter.setInputCloud(cloud);
            Eigen::Vector4f minPoint;
            minPoint[0] = minX;
            minPoint[1] = minY;
            minPoint[2] = minZ;
            Eigen::Vector4f maxPoint;
            maxPoint[0] = maxX;
            maxPoint[1] = maxY;
            maxPoint[2] = maxZ;
            cropBoxFilter.setMin(minPoint);
            cropBoxFilter.setMax(maxPoint);
            cropBoxFilter.filter(*cloud);
        }

        void VoxelizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double leafSize)
        {
            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(leafSize, leafSize, leafSize);
            sor.filter(*cloud);
        }
    }
}