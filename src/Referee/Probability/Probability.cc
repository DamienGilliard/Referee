#include "Probability.hh"
#include <cmath>

namespace Referee::Probability
{
    double Compute1DProbabilityDensityFunction(double x, double mean, double stdDev)
    {
        double exponent = -0.5 * std::pow((x - mean) / stdDev, 2);
        return (1 / (stdDev * sqrt(2 * M_PI))) * exp(exponent);
    }

    double Compute3DProbabilityDensityFunction(Eigen::Vector3d x, Eigen::Vector3d mean, Eigen::Matrix3d covarianceMatrix)
    {
        Eigen::Vector3d diff = x - mean;
        double exponent = -0.5 * diff.transpose() * covarianceMatrix.inverse() * diff;
        double determinant = covarianceMatrix.determinant();
        return (1 / (std::pow(2 * M_PI, 1.5) * std::sqrt(determinant))) * exp(exponent);
    }
}