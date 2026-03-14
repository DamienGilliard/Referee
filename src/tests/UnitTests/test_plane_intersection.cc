#include "../../Referee.hh"
#include <Eigen/Dense>

int main()
{
    std::vector<Eigen::Vector3d> plane1 = {Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(1, 0, 0)};
    std::vector<Eigen::Vector3d> plane2 = {Eigen::Vector3d(0, 10, 0), Eigen::Vector3d(0, 1, 0)};
    std::vector<Eigen::Vector3d> plane3 = {Eigen::Vector3d(0, 0, 10), Eigen::Vector3d(0, 0, 1)};

    Eigen::Vector3d intersectionPoint = Referee::Transformations::CalculatePlaneIntersection(plane1, plane2, plane3);

    if(intersectionPoint.isApprox(Eigen::Vector3d(10, 10, 10), 1e-6))
    {
        std::cout << "Test passed: Plane intersection is correct" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Test failed: Plane intersection is not correct" << std::endl;
        return 1;
    }

}