#pragma once

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <numeric>
#include <vector>

namespace Referee::Probability
{
    /**
     * @brief Computes the mean vector and covariance matrix of a set of 3D vectors
     * @param vectors a vector of 3D vectors
     * @return a pair containing the mean vector and the covariance matrix
     */
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> ComputeMeanVectorAndCovarianceMatrix(const std::vector<Eigen::Vector3d>& vectors);

    /**
     * @brief Computes the probability of a value x given a normal distribution with mean and standard deviation
     * @param x the value to compute the probability for
     * @param mean the mean of the normal distribution
     * @param stdDev the standard deviation of the normal distribution
     * @return the probability of x given the normal distribution
     */
    double Compute1DProbabilityDensityFunction(double x, double mean, double stdDev);

    /**
     * @brief Computes the probability of a 3D point x given a normal distribution with mean and covariance matrix
     * @param x the 3D point to compute the probability for
     * @param mean the mean of the normal distribution
     * @param covarianceMatrix the covariance matrix of the normal distribution
     * @return the probability of x given the normal distribution
     */
    double Compute3DProbabilityDensityFunction(Eigen::Vector3d x, Eigen::Vector3d mean, Eigen::Matrix3d covarianceMatrix);

    /**
     * @brief Computes the gradient descent for a set of 1D normal distributions
     * @param pairsOfMeansAndStdDevs a vector of pairs of means and standard deviations
     * @param initialValues the initial values for the gradient descent
     * @param stepSize the step size for the gradient descent
     * @param maxIterations the maximum number of iterations for the gradient descent
     * @param tolerance the tolerance for the gradient descent
     * @return a vector of values after applying gradient descent
     */
    std::vector<double> Compute1DGradienDescent(std::vector<std::pair<double, double>> pairsOfMeansAndStdDevs, std::vector<double> initialValues, double stepSize, int maxIterations, double tolerance);

}