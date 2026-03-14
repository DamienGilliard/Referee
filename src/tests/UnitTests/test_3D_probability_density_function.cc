#include "../../Referee.hh"
#include <Eigen/Dense>
#include <cmath>

int main()
{
    Eigen::Vector3d vector1(1, 0.1, 0);
    Eigen::Vector3d vector2(2, -0.1, 0.1);
    Eigen::Vector3d vector3(3, 0, -0.1);
    Eigen::Vector3d vector4(2, 0, 0);

    // Create a matrix where each column is one of the vectors
    Eigen::MatrixXd data(3, 4);
    data.col(0) = vector1;
    data.col(1) = vector2;
    data.col(2) = vector3;
    data.col(3) = vector4;

    // Compute the mean of each row (x, y, z coordinates)
    Eigen::Vector3d mean = data.rowwise().mean();

    // Subtract the mean from each column
    Eigen::MatrixXd centered = data.colwise() - mean;

    // Compute the covariance matrix. Check page 85 of https://doi.org/10.1007/978-3-031-45468-4
    Eigen::Matrix3d covarianceMatrix = (centered * centered.transpose()) / (data.cols() - 1);

    // Compute the probability density function
    double expectedProbability = 1/(std::pow((2*M_PI),1.5) * std::pow((2e-20)/3,0.5));
    std::cout << "Expected probability density function: " << expectedProbability << std::endl;
    double probability = Referee::Probability::Compute3DProbabilityDensityFunction(vector4, mean, covarianceMatrix);
    std::cout << "Probability density function: " << probability << std::endl;
    if (std::abs(probability - expectedProbability) / expectedProbability < 1e-2)
    {
        std::cout << "Test passed: Probability density function is correct" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Probability density function is not correct" << std::endl;
        return 1;
    }
}