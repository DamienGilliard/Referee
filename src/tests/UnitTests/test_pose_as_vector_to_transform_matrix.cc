#include "../../Referee.hh"

int main()
{
    Eigen::Matrix<double, 6, 1> poseAsVector;
    poseAsVector[0] = 1.0;
    poseAsVector[1] = 1.0;
    poseAsVector[2] = 1.0;
    poseAsVector[3] = 0.0;
    poseAsVector[4] = 0.0;
    poseAsVector[5] = 0.0;

    Eigen::Matrix4d transformationMatrix = Referee::Utils::Conversions::poseAsVectorToTransformationMatrix(poseAsVector);
    std::cout << "Transformation matrix:\n" << transformationMatrix << std::endl;
    if (transformationMatrix(0, 3) != 1.0 || transformationMatrix(1, 3) != 1.0 || transformationMatrix(2, 3) != 1.0)
    {
        std::cerr << "Error: Incorrect translation in transformation matrix." << std::endl;
        return 1;
    }
    return 0;
}