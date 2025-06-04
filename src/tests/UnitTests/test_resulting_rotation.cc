#include "../../Referee.hh"

int main()
{
    Eigen::Vector3d firstTranslation(0.0,-1.0, 0.0);
    Eigen::Vector3d secondTranslation(0.0, 1.0, 0.0);
    Eigen::Vector3d thirdTranslation(0.0, 2.0, 0.0);
    Eigen::Vector3d firstPosition(-1.0, 0.0, 0.0);
    Eigen::Vector3d secondPosition(1.0, 0.0, 0.0);
    Eigen::Vector3d thirdPosition(2.0, 0.0, 0.0);

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> positionAndTranslationVectors;
    positionAndTranslationVectors.push_back(std::make_pair(firstPosition, firstTranslation));
    positionAndTranslationVectors.push_back(std::make_pair(secondPosition, secondTranslation));
    positionAndTranslationVectors.push_back(std::make_pair(thirdPosition, thirdTranslation));

    double angle = Referee::Transformations::CalculateResultingRotationAngle(positionAndTranslationVectors);

    std::cout << "Resulting rotation angle: " << angle << " radians" << std::endl;

    if (std::abs(angle - (M_PI/4.0)) < 1e-6)
    {
        std::cout << "Test passed: Resulting rotation angle is correct" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Resulting rotation angle is not correct" << std::endl;
        return 1;
    }
}