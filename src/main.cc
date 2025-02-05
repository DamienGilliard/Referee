#include "Referee.hh"

int main() {
    std::vector<std::string> geolocationFiles = Referee::Utils::FileIterators::GetFilesInDirectory("../test_files/geolocations", ".csv");
    
    std::vector<std::string> plyFiles = Referee::Utils::FileIterators::GetFilesInDirectory("../test_files/scans", ".ply");

    for(const auto& file : plyFiles) {
        std::cout << file << std::endl;
        FileBasedVisualisation::Visualisation visualisation;
        visualisation.VisualisePointCloud(file);

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
            Referee::Utils::Conversions::ConvertLatLonAltToCartesian(std::stod(lat), std::stod(lon), std::stod(alt), x, y, z, Referee::Utils::CoordinateSystem::DEG, Referee::Utils::CoordinateSystem::LV95);
            
            std::cout << std::fixed << std::setprecision(2) << "Lat: " << lat << " Lon: " << lon << " Alt: " << alt << " X: " << x << " Y: " << y << " Z: " << z << std::endl;
        }
    }
    return 0;
}    