#include "Probability.hh"
#include <cmath>
#include <iostream>

namespace Referee::Probability
{
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> ComputeMeanVectorAndCovarianceMatrix(const std::vector<Eigen::Vector3d>& points)
    {
        Eigen::Vector3d meanVector = Eigen::Vector3d::Zero();
        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();

        for (const auto& point : points)
        {
            meanVector += point;
        }
        meanVector /= static_cast<double>(points.size());

        for (const auto& point : points)
        {
            Eigen::Vector3d diff = point - meanVector;
            covarianceMatrix += diff * diff.transpose();
        }
        covarianceMatrix /= static_cast<double>(points.size());

        return std::make_pair(meanVector, covarianceMatrix);
    }

    double Compute1DProbabilityDensityFunction(double x, double mean, double stdDev)
    {
        double exponent = -0.5 * std::pow((x - mean) / stdDev, 2);
        return (1.0 / (stdDev * sqrt(2.0 * M_PI))) * exp(exponent);
    }

    double Compute3DProbabilityDensityFunction(Eigen::Vector3d x, Eigen::Vector3d mean, Eigen::Matrix3d covarianceMatrix)
    {
        Eigen::Vector3d diff = x - mean;
        double exponent = -0.5 * diff.transpose() * covarianceMatrix.inverse() * diff;
        double determinant = covarianceMatrix.determinant();
        return (1 / (std::pow(2 * M_PI, 1.5) * std::sqrt(determinant))) * exp(exponent);
    }

    std::vector<double> Compute1DGradienDescent(std::vector<std::pair<double, double>> pairsOfMeansAndStdDevs, std::vector<double> initialValues, double stepSize, int maxIterations, double tolerance)
    {
        std::vector<double> currentValues = initialValues;
        for (int i = 0; i < maxIterations; ++i)
        {
            std::vector<double> gradients(currentValues.size(), 0.0);

            // Accumulate gradients for all pairs
            for (const auto& [mean, stdDev] : pairsOfMeansAndStdDevs)
            {
                for (size_t k = 0; k < currentValues.size(); ++k)
                {
                    gradients[k] += (Compute1DProbabilityDensityFunction(currentValues[k] + (stepSize / 2), mean, stdDev)
                                    - Compute1DProbabilityDensityFunction(currentValues[k] - (stepSize / 2), mean, stdDev)) / stepSize;
                }
            }

            // Compute the mean gradient over all values
            double meanGradient = 0.0;
            for (double g : gradients)
                meanGradient += g;
            meanGradient /= static_cast<double>(gradients.size());

            for (size_t k = 0; k < currentValues.size(); ++k)
            {
                currentValues[k] += (stepSize * meanGradient);
            }
            if (std::abs((stepSize * meanGradient)) < tolerance)
            {
                break;
            }
        }
        return currentValues;
    }
}