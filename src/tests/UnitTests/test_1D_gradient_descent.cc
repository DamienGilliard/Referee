#include "../../Referee.hh"

int main()
{
    std::vector<double> result = Referee::Probability::Compute1DGradienDescent({{0, 2}, {0, 1}}, {0.5, 0.5}, 1.0, 1000, 0.00001);
    std::cout << "Result: " << result[0] << ", " << result[1] << std::endl;
    std::cout << "Expected: 0.0, 0.0" << std::endl;
    if (std::abs(result[0] - 0.0) < 0.01 && std::abs(result[1] - 0.0) < 0.01)
    {
        std::cout << "Test passed!" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed!" << std::endl;
        return 1;
    }
}