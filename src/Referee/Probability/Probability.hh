#include <Eigen/Geometry>

namespace Referee::Probability
{
    /**
     * @brief Computes the probability of a value x given a normal distribution with mean and standard deviation
     * @param x the value to compute the probability for
     * @param mean the mean of the normal distribution
     * @param stdDev the standard deviation of the normal distribution
     * @return the probability of x given the normal distribution
     */
    double Compute1DProbabilityDensityFunction(double x, double mean, double stdDev);

    double Compute3DProbabilityDensityFunction(Eigen::Vector3d x, Eigen::Vector3d mean, Eigen::Matrix3d covarianceMatrix);

}