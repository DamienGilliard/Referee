#include "../../Referee.hh"
#include <cmath>

int main()
{
    double mean = 0.0;
    double stddev = 1.0;
    double x = 0.0;
    double p = Referee::Mapping::ComputeProbabilityDensityFunction(x, mean, stddev);
    if(std::abs(p - (1/std::sqrt(2*M_PI))) < 1e-6)
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