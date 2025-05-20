#include "Probability.hh"
#include <cmath>
#include <iostream>

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

    std::vector<double> Compute1DGradienDescent(std::vector<std::pair<double, double>> pairsOfMeansAndStdDevs, std::vector<double> initialValues, double stepSize, int maxIterations, double tolerance)
    {
        std::vector<double> currentValues = initialValues;
        for (int i = 0; i < maxIterations; ++i)
        {
            std::vector<double> gradients(currentValues.size(), 0.0);
            for (size_t j = 0; j < pairsOfMeansAndStdDevs.size(); ++j)
            {
                double mean = pairsOfMeansAndStdDevs[j].first;
                double stdDev = pairsOfMeansAndStdDevs[j].second;
                for (size_t k = 0; k < currentValues.size(); ++k)
                {
                    gradients[k] = Compute1DProbabilityDensityFunction(currentValues[k]+(stepSize/2), mean, stdDev) - Compute1DProbabilityDensityFunction(currentValues[k]-(stepSize/2), mean, stdDev);
                    gradients[k] /= stepSize;
                }
            }
            double meanGradient = 0.0;
            for (size_t k = 0; k < gradients.size(); ++k)
            {
                meanGradient += gradients[k];
            }
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