#include "Mapping.hh"

namespace Referee::Mapping
{
    void CreateConnectivityGraph(std::vector<std::vector<double>> geolocations, int knn, double maxDistance, std::vector<std::vector<int>>& graph)
    {
        if(graph.size() != geolocations.size())
        {
            graph.resize(geolocations.size());
        }
        
        for(int i = 0; i < geolocations.size(); i++)
        {
            for(int j = 0; j < geolocations.size(); j++)
            {
                if(i != j)
                {
                    // Calculate distance between geolocations
                    double distance = 0;
                    for(int k = 0; k < 3; k++)
                    {
                        distance += pow(geolocations[i][k] - geolocations[j][k], 2);
                    }
                    distance = sqrt(distance);

                    // Check if distance is within the maximum distance
                    if(distance < maxDistance)
                    {
                        graph[i].push_back(j);
                    }
                }
            }
        }
    }
}