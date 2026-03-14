#include "Raster.hh"

namespace Referee::Raster
{
    void Rasterizer::CreateGeoTIFFDEMFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& outputFileName, double resolution, Referee::Utils::CoordinateSystem::CoordinateSystem coordSys)
    {
        // 1. Find bounds
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        for (const auto& pt : cloud->points) 
        {
            min_x = std::min(min_x, double(pt.x));
            max_x = std::max(max_x, double(pt.x));
            min_y = std::min(min_y, double(pt.y));
            max_y = std::max(max_y, double(pt.y));
        }

        // 2. Define raster size
        int width = static_cast<int>((max_x - min_x) / resolution) + 1;
        int height = static_cast<int>((max_y - min_y) / resolution) + 1;
        std::vector<float> raster(width * height, std::numeric_limits<float>::quiet_NaN());

        // 3. Rasterize by taking the maximum Z value for each grid cell
        for (const auto& pt : cloud->points) 
        {
            int col = static_cast<int>((pt.x - min_x) / resolution);
            int row = static_cast<int>((max_y - pt.y) / resolution); // y axis inverted for raster
            int idx = row * width + col;
            if (std::isnan(raster[idx]) || pt.z > raster[idx])
            {
                raster[idx] = pt.z;
            }
        }

        // 4. Write GeoTIFF using GDAL
        GDALAllRegister();
        const char* format = "GTiff";
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName(format);
        GDALDataset* dataset = driver->Create(outputFileName.c_str(), width, height, 1, GDT_Float32, nullptr);

        // Set geotransform (origin X, pixel size X, 0, origin Y, 0, pixel size Y)
        double geoTransform[6] = { min_x, resolution, 0, max_y, 0, -resolution };
        dataset->SetGeoTransform(geoTransform);

        // Optionally set projection (WGS84 example)
        if (coordSys == Referee::Utils::CoordinateSystem::CoordinateSystem::WGS84)
        {
            dataset->SetProjection("EPSG:4326");
        }

        else if (coordSys == Referee::Utils::CoordinateSystem::CoordinateSystem::LV95)
        {
            // Set projection for LV95 or other coordinate systems as needed
            dataset->SetProjection("EPSG:2056"); // Example for Swiss LV95
        }
        else
        {
            std::cerr << "Unsupported coordinate system for GeoTIFF output." << std::endl;
            GDALClose(dataset);
            return;
        }
        
        // Write data
        GDALRasterBand* band = dataset->GetRasterBand(1);
        band->SetNoDataValue(std::numeric_limits<float>::quiet_NaN());
        band->RasterIO(GF_Write, 0, 0, width, height, raster.data(), width, height, GDT_Float32, 0, 0);

        GDALClose(dataset);
    }
} // namespace Referee::Raster
