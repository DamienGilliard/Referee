#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Referee::Mapping
{
    /**
     * @brief Create a connectivity graph from a set of geolocations
     * @param geolocations Geolocations of the nodes
     * @param knn Number of nearest neighbors to consider
     * @param maxDistance Maximum distance to consider a connection
     * @param graph Connectivity graph to be created. Each element of the vector is the list of indices the respective geolocations are connected to
     */
    void CreateConnectivityGraph(std::vector<std::vector<double>> geolocations, int knn, double maxDistance, std::vector<std::vector<int>>& graph);
    
}