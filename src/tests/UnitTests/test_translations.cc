#include "../../Referee.hh"

int main()
{
    std::vector<std::string> geolocationFiles = Referee::Utils::FileIterators::GetFilesInDirectory("../../tests/TestData/", ".petitpoucet");
    Referee::Utils::CoordinateSystem::CoordinateSystem coordSys = Referee::Utils::CoordinateSystem::CoordinateSystem::LV95;
    std::vector<Eigen::Vector3d> translationVectors = Referee::Utils::FileIterators::GetTranslationVectorsFromFiles(geolocationFiles, coordSys);
    Eigen::Vector3d meanTranslation = Referee::Transformations::RecenterTranslationVectors(translationVectors);
    if(translationVectors[0].norm() != 0)
    {
        std::cout << "translation vector: " << translationVectors[0].x() << " " << translationVectors[0].y() << " " << translationVectors[0].z() << std::endl;
        return 1;  
    }
    else
    {
        std::cout << "Mean translation vector is zero" << std::endl;
        return 0;
    }
}
