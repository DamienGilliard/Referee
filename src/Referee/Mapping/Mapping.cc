#include "Mapping.hh"

namespace Referee::Mapping
{
    void Scan::TransformScan(Eigen::Matrix4d transformation)
    {
        Eigen::Vector3d translation = transformation.block<3,1>(0,3);
        Eigen::Matrix3d rotationMatrix = transformation.block<3,3>(0,0);
        Eigen::Quaterniond rotation(rotationMatrix);
        __pose.Rotate(rotation);
        __pose.Translate(translation);
        Referee::Transformations::TransformPointCloud<pcl::PointNormal>(__cloud, transformation);
    }


    void  Scan::LoadCloud()
            {
                if(this->__cloud->size() > 0)
                {
                    std::cout << "Point cloud already loaded." << std::endl;
                    return;
                }
                __cloud.reset(new pcl::PointCloud<pcl::PointNormal>());
                pcl::io::loadPLYFile(__cloudFileName, *__cloud);
                Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(__cloud, 0.01);
                Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
                transformation.block<3,3>(0,0) = this->__pose.GetOrientation().toRotationMatrix();
                transformation.block<3,1>(0,3) = this->__pose.GetPosition();
                this->TransformScan(transformation);
            }


    Transformation::Transformation(Eigen::Matrix4d transformationMatrixInGlobalCoordinateSystem,
                           std::shared_ptr<Scan> fromScan,
                           std::shared_ptr<Scan> toScan)
        : __globalTransformation(transformationMatrixInGlobalCoordinateSystem),
          __fromScan(fromScan),
          __toScan(toScan)
    {
        Eigen::Matrix3d rotationMatrix = transformationMatrixInGlobalCoordinateSystem.block<3, 3>(0, 0);
        __quaternion = Eigen::Quaterniond(rotationMatrix);
        Eigen::AngleAxisd angleAxis(rotationMatrix);
        __globalTwistVector = angleAxis.angle() * angleAxis.axis();
        __globalTranslation = transformationMatrixInGlobalCoordinateSystem.block<3, 1>(0, 3);
    }


    void Transformation::PrintTransformation()
    {
        const std::string fromName = __fromScan ? __fromScan->GetCloudFileName() : std::string("<unknown>");
        const std::string toName   = __toScan   ? __toScan->GetCloudFileName()   : std::string("<unknown>");

        std::cout << "Transformation from scan " << fromName << " to scan " << toName << std::endl;
        std::cout << "Rotation (quaternion): " << __quaternion.coeffs().transpose() << std::endl;
        std::cout << "Twist vector: " << __globalTwistVector.transpose() << std::endl;
        std::cout << "Translation: " << __globalTranslation.transpose() << std::endl;
    }


    Graph& Graph::CreateUndirectedGraph()
    {
        if(!Graph::__instance)
        {
            Graph::__instance = new Graph(GraphType::Undirected);
        }
        return *Graph::__instance;
    }


    Graph& Graph::CreateUndirectedGraph(std::vector<Eigen::Vector3d> vertices, 
                                        std::vector<std::vector<int>> edges)
    {
        Graph& graph = Graph::CreateUndirectedGraph();
        graph.__vertexIndices = std::unordered_map<Eigen::Vector3d, int>(vertices.size());
        for(const auto& vertex : vertices)
        {
            graph.AddVertex(vertex);
            graph.__vertexIndices[vertex] = graph.__nVertices - 1;
        }
        for(int i = 0; i < edges.size(); i++)
        {
            for(int j = 0; j < edges[i].size(); j++)
            {
                if(i != edges[i][j]) // avoid self-loops
                {
                    graph.AddEdge(vertices[i], vertices[edges[i][j]], (vertices[i] - vertices[edges[i][j]]).norm());
                }
            }
        }
        return graph;
    }


    Graph& Graph::GetInstanceOfUndirectedGraph()
    {
        if(!Graph::__instance)
        {
            Graph::__instance = &CreateUndirectedGraph();
        }
        if(Graph::__instance->__isDirected)
        {
            std::cerr << "Error: Trying to get an undirected graph instance, but the instance is directed." << std::endl;
            exit(EXIT_FAILURE);
        }
        return *Graph::__instance;
    }


    void Graph::AddVertex(Eigen::Vector3d vertex)
    {
        int index = this->__nVertices;
        this->__vertexIndices[vertex] = index;
        this->__nVertices++;
        __undirectedGraph.add_vertex(index);
    }


    void Graph::AddEdge(Eigen::Vector3d vertex1, 
                        Eigen::Vector3d vertex2, 
                        double weight)
    {
        int index1 = this->__vertexIndices[vertex1];
        int index2 = this->__vertexIndices[vertex2];
        __undirectedGraph.add_edge(index1, index2, weight);
        this->_nEdges++;
    }


    void Graph::SetWeight(int vertex1, int vertex2, double weight)
    {
        if(!this->__undirectedGraph.has_edge(vertex1, vertex2))
        {
            std::cerr << "Error: Trying to set weight of a non-existing edge." << std::endl;
            exit(EXIT_FAILURE);
        }
        this->__undirectedGraph.get_edge(vertex1, vertex2) = weight;
    }


    std::vector<std::pair<long unsigned int, long unsigned int>> Graph::ComputeMinimumSpanningTree(int rootVertexIndex)
    {
        if(!__undirectedGraph.has_vertex(rootVertexIndex))
        {
            std::cerr << "Error: Root vertex is not part of the graph." << std::endl;
            exit(EXIT_FAILURE);
        }

        auto mstEdgesOpt = graaf::algorithm::prim_minimum_spanning_tree(this->__undirectedGraph, rootVertexIndex);
        if (!mstEdgesOpt) 
        {
            std::cerr << "Error: Could not compute minimum spanning tree." << std::endl;
            return {};
        }
        std::vector<std::pair<long unsigned int, long unsigned int>> mstEdges = mstEdgesOpt.value();
        this->__minimumSpanningTreeEdges = mstEdges;
        this->__minimumSpanningTree = graaf::undirected_graph<int, double>();
        for (const auto& edge : mstEdges) {
            if (!this->__minimumSpanningTree.has_vertex(edge.first)) 
            {
                this->__minimumSpanningTree.add_vertex(this->__undirectedGraph.get_vertex(edge.first), edge.first);
            }
            if (!this->__minimumSpanningTree.has_vertex(edge.second)) 
            {
                this->__minimumSpanningTree.add_vertex(this->__undirectedGraph.get_vertex(edge.second), edge.second);
            }
            this->__minimumSpanningTree.add_edge(edge.first, edge.second, this->__undirectedGraph.get_edge(edge.first, edge.second));
        }
        return mstEdges;
    }


    void Graph::PrintGraph()
    {
        for(const auto& vertex : this->__undirectedGraph.get_vertices())
        {
            std::cout << "Vertex " << vertex.first << " connected to: ";
            for(const auto& neighbor : this->__undirectedGraph.get_neighbors(vertex.first))
            {
                std::cout << neighbor << " (weight: " << this->__undirectedGraph.get_edge(vertex.first, neighbor) << "), ";
            }
            std::cout << std::endl;
        }
        graaf::io::to_dot(this->__undirectedGraph, "./graph.dot");
        std::cout << "Graph has been written to graph.dot" << std::endl;
    }


    std::vector<int> Graph::ExtractMSTSubTree(int startingVertexIndex)
    {
        // 1. Build the MST as an adjacency list
        std::unordered_map<int, std::vector<int>> mst_adj;
        for (const auto& edge : this->__minimumSpanningTreeEdges) {
            int u = edge.first;
            int v = edge.second;
            mst_adj[u].push_back(v);
            mst_adj[v].push_back(u);
        }

        // 2. Build parent map from the original root
        std::unordered_map<int, int> parent_map;
        std::function<void(int, int)> build_parent = [&](int node, int parent) {
            parent_map[node] = parent;
            for (int neighbor : mst_adj[node]) {
                if (neighbor != parent) {
                    build_parent(neighbor, node);
                }
            }
        };
        build_parent(0, -1); // the original root is 0

        // 3. Collect descendants of startingVertexIndex
        std::vector<int> subtree;
        std::function<void(int)> collect_descendants = [&](int node) {
            subtree.push_back(node);
            for (int neighbor : mst_adj[node]) {
                if (parent_map[neighbor] == node) { // Only go to children
                    collect_descendants(neighbor);
                }
            }
        };
        collect_descendants(startingVertexIndex);

        return subtree;
    }


    Graph::Graph(GraphType type): __isDirected(type == GraphType::Directed)
    {
        if(type == GraphType::Undirected)
        {
            this->__undirectedGraph = graaf::undirected_graph<int, double>();
        }
        else
        {
            std::cerr << "Error: Unsupported graph type." << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    
    std::vector<std::pair<long unsigned int, long unsigned int>> Graph::GetNonMSTEdges()
    {
        std::vector<std::pair<long unsigned int, long unsigned int>> nonMSTEdges;
        std::unordered_set<std::pair<long unsigned int, long unsigned int>, boost::hash<std::pair<long unsigned int, long unsigned int>>> mstEdgeSet;
        for (const auto& edge : this->__minimumSpanningTreeEdges) 
        {
            mstEdgeSet.insert(edge);
            mstEdgeSet.insert(std::make_pair(edge.second, edge.first)); // because undirected
        }

        for (const std::pair<std::pair<long unsigned int, long unsigned int>,double>& edgeAndWeight : this->__undirectedGraph.get_edges()) 
        {
            const auto& edge = edgeAndWeight.first;
            std::pair<long unsigned int, long unsigned int> edgePair = std::make_pair(edge.first, edge.second);
            if (mstEdgeSet.find(edgePair) == mstEdgeSet.end()) {
                nonMSTEdges.push_back(edgePair);
            }
        }
        return nonMSTEdges;
    }

    std::vector<std::vector<long unsigned int>> Graph::GetCorrectionLoops()
    {
        std::vector<std::vector<long unsigned int>> correctionLoops;
        std::vector<std::pair<long unsigned int, long unsigned int>> nonMSTEdges = this->GetNonMSTEdges();

        for(std::pair<long unsigned int, long unsigned int> edge : nonMSTEdges)
        {
            std::vector<long unsigned int> correctionLoop;
            auto pathOpt = graaf::algorithm::bfs_shortest_path(this->__minimumSpanningTree, edge.first, edge.second);
            if(pathOpt)
            {
                std::cout << "[DEBUG] Correction loop found between vertices " << edge.first << " and " << edge.second << ": ";
                for(auto vertex : pathOpt.value().vertices)
                {
                    std::cout << vertex << " ";
                    correctionLoop.push_back(vertex);
                }
                std::cout << std::endl;
            }
            else
            {
                std::cerr << "Error: No path found in MST between vertices " << edge.first << " and " << edge.second << std::endl;
            }
            correctionLoops.push_back(correctionLoop);
        }
        return correctionLoops;
    }


    Graph* Graph::__instance = nullptr;


    void MappingMatrix::PrintMatrix()
    {
        for(int i = 0; i < __mappingMatrix.size(); i++)
        {
            for(int j = 0; j < __mappingMatrix[i].size(); j++)
            {
                std::cout << i << " " << j << " : " << std::endl;
                __mappingMatrix[i][j].PrintTransformation();
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }


    void MappingMatrix::ComputeLoopClosures()
    {
        std::vector<std::vector<long unsigned int>> correctionLoops = this->GetGraph().GetCorrectionLoops();
        for(const std::vector<long unsigned int>& loop : correctionLoops)
        {
            std::cout << "[DEBUG] Computing loop closure for loop: ";
            for(const auto& vertex : loop)
            {
                std::cout << vertex << " ";
            }
            std::cout << std::endl;

            ceres::Problem problem;

            std::vector<double*> posesAsVectors;
            std::cout << "[DEBUG] loop size: " << loop.size() << std::endl;
            for(int i = 0; i < loop.size(); i++)
            {
                double *poseAsVector = new double[6];
                if (!poseAsVector) 
                {
                    std::cerr << "[ERROR] Failed to allocate poseAsVector for index " << i << std::endl;
                    continue;
                }
                poseAsVector[0] = this->__initialPositions[loop[i]].x();
                poseAsVector[1] = this->__initialPositions[loop[i]].y();
                poseAsVector[2] = this->__initialPositions[loop[i]].z();
                poseAsVector[3] = 0.0;
                poseAsVector[4] = 0.0;
                poseAsVector[5] = 0.0;

                posesAsVectors.push_back(poseAsVector);
                problem.AddParameterBlock(poseAsVector, 6);
                
                if (i == 0)
                {
                    problem.SetParameterBlockConstant(poseAsVector); // fix the first pose to anchor the loop
                }

            }

            std::vector<Eigen::Matrix4d> loopConstraints;
            for(int i = 0; i < loop.size(); i++)
            {
                int fromIndex = loop[i];
                int toIndex = loop[(i + 1) % loop.size()]; // next vertex in the loop, wrap around at the end

                if (!this->__mappingMatrix[fromIndex][toIndex].GetTransformationMatrix().allFinite()) 
                {
                    std::cerr << "[ERROR] Invalid transformation matrix between vertices " 
                            << fromIndex << " and " << toIndex << std::endl;
                    continue;
                }
                const Eigen::Matrix4d& transformation = this->__mappingMatrix[fromIndex][toIndex].GetTransformationMatrix();
                loopConstraints.push_back(transformation);
            }
            for (int i = 0; i < loopConstraints.size(); ++i)
            {
                ceres::CostFunction* costFunction = Referee::Mapping::TransformationError::Create(loopConstraints[i]);
                problem.AddResidualBlock(costFunction, 
                                         new ceres::HuberLoss(1.0),
                                         posesAsVectors[i], 
                                         posesAsVectors[(i + 1) % posesAsVectors.size()]);
            }
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = true;
            options.function_tolerance = 1e-6; // Adjust tolerance
            options.max_num_iterations = 100;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << std::endl;
            for (int i = 0; i < posesAsVectors.size(); i++)
            {
                delete[] posesAsVectors[i]; // Clean up allocated memory
            }
        }
    }


    void MappingMatrix::CalculateMeanTransformationMatrices()
    {
        // std::vector<double> stdDevRotations;
        this->__stdDevRotations.resize(__mappingMatrix.size());
        this->__meanTransformations.resize(__mappingMatrix.size());
        this->__meanTranslationVectors.resize(__mappingMatrix.size());
        this->__covTranslationVectors.resize(__mappingMatrix.size());
        for(int i = 0; i < __mappingMatrix.size(); i++)
        {
            std::vector<Eigen::Vector3d> translationVectors;
            Eigen::Matrix3d translationCovarianceMatrix = Eigen::Matrix3d::Zero();
            Eigen::Vector3d meanTranslation = Eigen::Vector3d::Zero();
            Eigen::Vector3d stdDevTranslation = Eigen::Vector3d::Zero();
            Eigen::Vector3d meanRotationAxis = Eigen::Vector3d::Zero();
            double meanRotationAngle = 0;
            double stdDevRotation = 0;
            int nonZeroMatrices = 0;
            for(int j = 0; j < __mappingMatrix[i].size(); j++)
            {
                if(__mappingMatrix[i][j].GetRotationAngle() == 0 && __mappingMatrix[i][j].GetTranslation().norm() == 0)
                {
                    continue;
                }

                translationVectors.push_back(__mappingMatrix[i][j].GetTranslation());
                Eigen::Vector3d rotationAxis = __mappingMatrix[i][j].GetRotationVector();
                if(rotationAxis.z() < 0)
                {
                    rotationAxis = -rotationAxis;
                }
                meanRotationAxis += rotationAxis;
                meanRotationAngle += __mappingMatrix[i][j].GetRotationAngle();
                nonZeroMatrices++;
            }
            std::pair<Eigen::Vector3d, Eigen::Matrix3d> translationStats = Referee::Probability::ComputeMeanVectorAndCovarianceMatrix(translationVectors);
            meanRotationAxis /= nonZeroMatrices;
            meanRotationAngle /= nonZeroMatrices;
            this->__meanTranslationVectors[i] = translationStats.first;

            for(int j = 0; j < __mappingMatrix[i].size(); j++)
            {
                if(__mappingMatrix[i][j].GetRotationAngle() == 0 && __mappingMatrix[i][j].GetTranslation().norm() == 0)
                {
                    continue;
                }
                stdDevRotation += std::pow(__mappingMatrix[i][j].GetRotationAngle() - meanRotationAngle, 2);
            }
            __stdDevRotations[i] = std::sqrt(stdDevRotation / nonZeroMatrices);
            std::cout << "[DEBUG] covariance matrix:" << translationStats.second << std::endl;

            std::cout << "[DEBUG] determinant of covariance matrix: " << translationStats.second.determinant() << std::endl;
            double entropy = 3.0/2.0 * (1.0 + std::log(2.0 * M_PI)) + 0.5 * std::log(translationStats.second.determinant());
            std::cout << "[DEBUG]Entropy for point cloud " << i << ": " << entropy << std::endl;
            Eigen::Matrix4d meanTransformationMatrix = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(meanRotationAngle, meanRotationAxis.normalized()).toRotationMatrix();
            meanTransformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
            meanTransformationMatrix.block<3, 1>(0, 3) = translationStats.first;

            Referee::Mapping::Transformation meanTransformation(meanTransformationMatrix);
            this->__meanTransformations[i] = meanTransformation;
            this->__covTranslationVectors[i] = translationStats.second;
        }
    }


    std::tuple<int, double> MappingMatrix::GetMostProbableRotation()
    {
        int mostProbableIndex = 0;
        double maxProbability = 0;
        for(int i = 0; i < this->__stdDevRotations.size(); i++)
        {
            double probability = Referee::Probability::Compute1DProbabilityDensityFunction(this->__meanTransformations[i].GetRotationAngle(), this->__meanTransformations[i].GetRotationAngle(), this->__stdDevRotations[i]);
            if(probability > maxProbability)
            {
                maxProbability = probability;
                mostProbableIndex = i;
            }
        }
        return std::make_tuple(mostProbableIndex, maxProbability);
    }


    double MappingMatrix::GetOverallMeanRotation()
    {
        double overallMeanRotation = 0.0;
        for (int i = 0; i < this->__connectivityMatrix.size(); i++)
        {
            Eigen::Vector3d positionIPointCloud = this->__initialPositions[i];
            double meanRotation = 0.0;
            for (int j : this->__connectivityMatrix[i])
            {
                Eigen::Vector3d positionJPointCloud = this->__initialPositions[j];
                Eigen::Vector3d vectorIJ = positionJPointCloud - positionIPointCloud;
                Eigen::Vector3d translation = this->__mappingMatrix[i][j].GetTranslation();
                std::vector<double> angles = Referee::Utils::Trigonometry::SolveAlKashi(translation, vectorIJ, vectorIJ + translation);
                if (angles.size() > 0)
                {
                    meanRotation += angles[0];
                }
            }
            meanRotation /= this->__connectivityMatrix[i].size();
            overallMeanRotation += meanRotation;
        }
        overallMeanRotation /= this->__connectivityMatrix.size();
         std::cout << "[DEBUG] Overall mean rotation: " << overallMeanRotation << std::endl;
        return overallMeanRotation;
    }


    std::pair<int, double> MappingMatrix::GetMostProbableTranslation()
    {
        int mostProbableIndex = 0;
        double maxProbability = 0;
        for(int i = 0; i < this->__mappingMatrix.size(); i++)
        {
            std::vector<Eigen::Vector3d> translationVectors;
            for(int j = 0; j < this->__mappingMatrix[i].size(); j++)
            {
                if(this->__mappingMatrix[i][j].GetTranslation().norm() == 0)
                {
                    continue;
                }
                translationVectors.push_back(this->__mappingMatrix[i][j].GetTranslation());
            }

            double probability = Referee::Probability::Compute3DProbabilityDensityFunction(this->__meanTranslationVectors[i], this->__meanTranslationVectors[i], this->__covTranslationVectors[i]);
            std::cout << "[DEBUG]Probability for point cloud " << i << ": " << probability << std::endl;
            std::cout << "[DEBUG]Mean translation vector for point cloud " << i << ": " << this->__meanTranslationVectors[i].transpose() << std::endl;
            if(probability > maxProbability)
            {
                maxProbability = probability;
                mostProbableIndex = i;
            }
        }
        return std::make_pair(mostProbableIndex, maxProbability);
    }


    std::vector<std::pair<double, double>> MappingMatrix::GetMeanRotationsAndStdDevs()
    {
        std::vector<std::pair<double, double>> meanRotationsAndStdDevs;
        for(int i = 0; i < this->__meanTransformations.size(); i++)
        {
            meanRotationsAndStdDevs.push_back(std::make_pair(this->__meanTransformations[i].GetRotationAngle(), this->__stdDevRotations[i]));
        }
        return meanRotationsAndStdDevs;
    }


    std::vector<std::pair<double, Eigen::Vector3d>> MappingMatrix::GetMeanTranslationVectorsAndStdDevs()
    {
        std::vector<std::pair<double, Eigen::Vector3d>> meanTranslationVectorsAndStdDevs;
        for(int i = 0; i < this->__meanTransformations.size(); i++)
        {
            Eigen::Vector3d translation = this->__meanTransformations[i].GetTranslation();
            double stdDev = this->__stdDevRotations[i]; // Assuming stdDev is the same for all components of the translation vector
            meanTranslationVectorsAndStdDevs.push_back(std::make_pair(translation.norm(), translation));
        }
        return meanTranslationVectorsAndStdDevs;
    }


    double MappingMatrix::ComputeMeanTranslationInducedRotation()
    {
        double meanTranslationInducedRotation = 0.0;
        int count = 0;
        for (int i = 0; i < this->__connectivityMatrix.size(); i++)
        {
            for (int j = 0; j < this->__connectivityMatrix[i].size(); j++)
            {
                Eigen::Vector3d poseOfI = this->__initialPositions[i];
                Eigen::Vector3d poseOfJ = this->__initialPositions[j];
                Eigen::Vector3d translation = this->__mappingMatrix[i][j].GetTranslation();

                // Compute the translation induced rotation
                Eigen::Vector3d v1 = poseOfJ + translation;
                Eigen::Vector3d v2 = v1 - poseOfI;
                Eigen::Vector3d crossProduct = v1.normalized().cross(v2.normalized());
                double angle = std::asin(crossProduct.norm());
                if (crossProduct.z() < 0)
                {
                    angle = -angle; // Ensure the angle is in the correct direction
                }
                meanTranslationInducedRotation += angle;
                count++;
            }
        }
        meanTranslationInducedRotation /= count;
        return meanTranslationInducedRotation;
    }

    
    void MappingMatrix::ComputeRotationCoefficients(int mostTrustworthyPointCloudIndex)
    {
        double mostTrustworthyRotationAngle = this->__meanTransformations[mostTrustworthyPointCloudIndex].GetRotationAngle();
        int numberFiles = this->__mappingMatrix.size();
        Eigen::MatrixXd rotationCoefficients = Eigen::MatrixXd::Zero(numberFiles, numberFiles);

        for(int i : this->__connectivityMatrix[mostTrustworthyPointCloudIndex])
        {
            if(i == mostTrustworthyPointCloudIndex)
            {
                continue;
            }
            double rotationAngle = this->__mappingMatrix[mostTrustworthyPointCloudIndex][i].GetRotationAngle();
            std::cout << "[DEBUG]Rotation angle between point cloud " << mostTrustworthyPointCloudIndex << " and point cloud " << i << ": " << rotationAngle << std::endl;

            double alpha = (mostTrustworthyRotationAngle/rotationAngle);
            rotationCoefficients(mostTrustworthyPointCloudIndex, i) = alpha;
            std::cout << "[DEBUG]alpha: " << alpha << std::endl;
            rotationCoefficients(i, mostTrustworthyPointCloudIndex) = 1 - alpha;

        }

        // starting with the most trustworthy point cloud, onwards,
        for (int i = mostTrustworthyPointCloudIndex; i < numberFiles; i++)
        {
            // we store the connected point clouds
            std::vector<int> connectedPC = this->__connectivityMatrix[i];
            // for each connected point cloud, we initialize the row
            int matchedIndex;
            double rotationAngle;
            for (int j : connectedPC)
            {
                // we verify if the connectivity is reciprocal (i.e. if j is connected to i). 
                // because we work with knn, it is possible that i is connected to j, but j is not connected to i
                for (int k : this->__connectivityMatrix[j])
                {
                    // if indeed it is reciprocal
                    if (i == k)
                    {
                        double alpha = rotationCoefficients(j, i);
                        if (rotationCoefficients(j, i) != 0) 
                        {
                            rotationCoefficients(i, j) = 1 - alpha;
                            matchedIndex = j;
                            rotationAngle = this->__mappingMatrix[i][j].GetRotationAngle() * rotationCoefficients(i, j);
                        } 
                    }
                }
            }
            for (int j : connectedPC)
            {
                // if the connectivity is not reciprocal, we set the rotation coefficient to 0
                if (j != matchedIndex)
                {
                    rotationCoefficients(i, j) = rotationAngle / this->__mappingMatrix[i][j].GetRotationAngle();
                }
            }
        }

        // starting with the most trustworthy point cloud, backwards,
        for (int i = mostTrustworthyPointCloudIndex; i >= 0; i--)
        {
            // we store the connected point clouds
            std::vector<int> connectedPC = this->__connectivityMatrix[i];
            // for each connected point cloud, we initialize the row
            int matchedIndex;
            double rotationAngle;
            for (int j : connectedPC)
            {
                // we verify if the connectivity is reciprocal (i.e. if j is connected to i). 
                // because we work with knn, it is possible that i is connected to j, but j is not connected to i
                for (int k : this->__connectivityMatrix[j])
                {
                    // if indeed it is reciprocal
                    if (i == k)
                    {
                        double alpha = rotationCoefficients(j, i);
                        if (rotationCoefficients(j, i) != 0) 
                        {
                            rotationCoefficients(i, j) = 1 - alpha;
                            matchedIndex = j;
                            rotationAngle = this->__mappingMatrix[i][j].GetRotationAngle() * rotationCoefficients(i, j);
                        } 
                    }
                }
            }

            for (int j : connectedPC)
            {
                // if the connectivity is not reciprocal, we set the rotation coefficient to 0
                if (j != matchedIndex)
                {
                    rotationCoefficients(i, j) = rotationAngle / this->__mappingMatrix[i][j].GetRotationAngle();
                }
            }
        }
    
        this->__rotationCoefficients = rotationCoefficients;
        std::cout << "Rotation coefficients: " << std::endl;
        for(int i = 0; i < rotationCoefficients.rows(); i++)
        {
            for(int j = 0; j < rotationCoefficients.cols(); j++)
            {
                std::cout << rotationCoefficients(i, j) << "   ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }


    void MappingMatrix::ComputeTranslationCoefficients(int mostTrustworthyPointCloudIndex)
    {
        // Initialize the translation factors with rests
        int numberFiles = this->__mappingMatrix.size();
        this->__translationFactorsWithRests.resize(numberFiles, std::vector<std::pair<double, Eigen::Vector3d>>(numberFiles, std::make_pair(0.0, Eigen::Vector3d::Zero())));

        Eigen::Vector3d mostTrustworthyTranslationVector = this->__meanTranslationVectors[mostTrustworthyPointCloudIndex];
        this->__finalTranslations.resize(numberFiles, Eigen::Vector3d());
        this->__finalTranslations[mostTrustworthyPointCloudIndex] = mostTrustworthyTranslationVector;

        // Compute the translation factors for the most trustworthy point cloud and their reciprocates
        for(int i : this->__connectivityMatrix[mostTrustworthyPointCloudIndex])
        {
            Eigen::Vector3d translationVector = this->__mappingMatrix[mostTrustworthyPointCloudIndex][i].GetTranslation();
            double projectionFactor = mostTrustworthyTranslationVector.dot(translationVector.normalized()) / translationVector.norm();
            Eigen::Vector3d projectionOfMeanVectorOnIndividualTranslationVector = projectionFactor * translationVector;
            Eigen::Vector3d rest = mostTrustworthyTranslationVector - projectionOfMeanVectorOnIndividualTranslationVector;
            this->__translationFactorsWithRests[mostTrustworthyPointCloudIndex][i].first = projectionFactor;
            this->__translationFactorsWithRests[mostTrustworthyPointCloudIndex][i].second = rest;
            this->__translationFactorsWithRests[i][mostTrustworthyPointCloudIndex].first = 1.0 - projectionFactor;
            this->__translationFactorsWithRests[i][mostTrustworthyPointCloudIndex].second = rest;
            Eigen::Vector3d resultingTranslationVector = -translationVector * (1.0 - projectionFactor) + rest;

            if (resultingTranslationVector.norm() > 15.0) // if the resulting translation vector is too large, we skip it
            {
                std::cout << "[DEBUG]Skipping registration of " << mostTrustworthyPointCloudIndex << " on " << i << " because the resulting translation vector is too large" << std::endl;
                continue;
            }
            this->__finalTranslations[i] = resultingTranslationVector;

            std::cout << "[DEBUG]final translation vector for point cloud " << i << ": " << this->GetFinalTranslation(i).transpose() << std::endl;
        }

        // Compute the other translation factors for the connected point clouds
        for(int i : this->__connectivityMatrix[mostTrustworthyPointCloudIndex])
        {
            Eigen::Vector3d translationVector = this->GetFinalTranslation(i); // this is by how much we need to move point cloud i
            
            for (int j : this->__connectivityMatrix[i])
            {
                Eigen::Vector3d initialTranslationVector = this->__mappingMatrix[i][j].GetTranslation();
                double projectionFactor = translationVector.dot(initialTranslationVector.normalized()) / initialTranslationVector.norm();
                Eigen::Vector3d projectionOfMeanVectorOnIndividualTranslationVector = projectionFactor * translationVector;
                Eigen::Vector3d rest = translationVector - projectionOfMeanVectorOnIndividualTranslationVector;
                this->__translationFactorsWithRests[i][j].first = projectionFactor;
                this->__translationFactorsWithRests[i][j].second = rest;
                this->__translationFactorsWithRests[j][i].first = 1.0 - projectionFactor;
                this->__translationFactorsWithRests[j][i].second = rest;
                Eigen::Vector3d resultingTranslationVector = -initialTranslationVector * (1.0 - projectionFactor) + rest;

                if (resultingTranslationVector.norm() > 25.0) // if the resulting translation vector is too large, we skip it
                {
                    std::cout << "[DEBUG]Skipping registration of " << i << " on " << j << " because the resulting translation vector is too large" << std::endl;
                    continue;
                }

                if(this->GetFinalTranslation(j).norm() == 0)
                {
                    this->__finalTranslations[j] = resultingTranslationVector;
                }
                std::cout << "[DEBUG]final translation vector for point cloud " << j << ": " << this->GetFinalTranslation(j).transpose() << std::endl;
            }
        }

        // Compute the translation factors for the non-connected point clouds
        for (int i = mostTrustworthyPointCloudIndex; i < numberFiles; i++)
        {
            std::vector<int> connectedPCs = this->__connectivityMatrix[i];
            Eigen::Vector3d referenceTranslationVector = this->GetFinalTranslation(i);
            for (int j : connectedPCs)
            {
                Eigen::Vector3d initialTranslationVector = this->__mappingMatrix[i][j].GetTranslation();
                double projectionFactor = referenceTranslationVector.dot(initialTranslationVector.normalized()) / initialTranslationVector.norm();
                Eigen::Vector3d projectionOfMeanVectorOnIndividualTranslationVector = projectionFactor * initialTranslationVector;
                Eigen::Vector3d rest = referenceTranslationVector - projectionOfMeanVectorOnIndividualTranslationVector;
                this->__translationFactorsWithRests[i][j].first = projectionFactor;
                this->__translationFactorsWithRests[i][j].second = rest;
                this->__translationFactorsWithRests[j][i].first = 1.0 - projectionFactor;
                this->__translationFactorsWithRests[j][i].second = rest;
                Eigen::Vector3d resultingTranslationVector = -initialTranslationVector * (1.0 - projectionFactor) + rest;
                if(resultingTranslationVector.norm() > 25.0) // if the resulting translation vector is too large, we skip it
                {
                    std::cout << "[DEBUG]Skipping registration of " << i << " on " << j << " because the resulting translation vector is too large" << std::endl;
                    continue;
                }
                if (this->GetFinalTranslation(j).norm() == 0)
                {
                    this->__finalTranslations[j] = resultingTranslationVector;
                }

                std::cout << "[DEBUG]final translation vector for point cloud " << j << ": " << this->GetFinalTranslation(j).transpose() << std::endl;
            }
        }

        for (int i = mostTrustworthyPointCloudIndex; i >= 0; i--)
        {
            if (this->GetFinalTranslation(i).norm() != 0)
            {
                std::cout << "[DEBUG]Skipping point cloud " << i << " because it is already computed" << std::endl;
                continue;
            }

            std::vector<int> connectedPC = this->__connectivityMatrix[i];
            int rowSeed = -1;

            for (int j : connectedPC)
            {
                if (this->__translationFactorsWithRests[i][j].first != 0)
                {
                    rowSeed = j;
                    break;
                }
            }

            Eigen::Vector3d referenceTranslationVector = this->GetFinalTranslation(i);

            for (int j : connectedPC)
            {
                Eigen::Vector3d originalTranslationVector = this->__mappingMatrix[i][j].GetTranslation();
                double projectionFactor = referenceTranslationVector.dot(originalTranslationVector.normalized()) / originalTranslationVector.norm();
                Eigen::Vector3d projectionOfMeanVectorOnIndividualTranslationVector = projectionFactor * originalTranslationVector;
                Eigen::Vector3d rest = referenceTranslationVector - projectionOfMeanVectorOnIndividualTranslationVector;
                this->__translationFactorsWithRests[i][j].first = projectionFactor;
                this->__translationFactorsWithRests[i][j].second = rest;
                this->__translationFactorsWithRests[j][i].first = 1.0 - projectionFactor;
                this->__translationFactorsWithRests[j][i].second = rest;
                Eigen::Vector3d resultingTranslationVector = -originalTranslationVector * (1.0 - projectionFactor) + rest;

                if(resultingTranslationVector.norm() > 25.0) // if the resulting translation vector is too large, we skip it
                {
                    std::cout << "[DEBUG]Skipping registration of " << i << " on " << j << " because the resulting translation vector is too large" << std::endl;
                    continue;
                }

                if (this->GetFinalTranslation(j).norm() == 0)
                {
                    this->__finalTranslations[j] = resultingTranslationVector;
                }
                std::cout << "[DEBUG4]final translation vector for point cloud " << j << ": " << this->GetFinalTranslation(j).transpose() << std::endl;
            }
        }
    }
    

    std::vector<double> MappingMatrix::GetInitialRotationAngles()
    {
        std::vector<double> initialRotationAngles;
        for(int i = 0; i < this->__mappingMatrix.size(); i++)
        {
            for (int j = 0; j < this->__mappingMatrix[i].size(); j++)
            {
                if(this->__mappingMatrix[i][j].GetRotationAngle() != 0)
                {
                    double angle = this->__mappingMatrix[i][j].GetRotationAngle() * this->GetRotationCoefficient(i, j);
                    initialRotationAngles.push_back(angle);
                    break;
                }
            }
        }
        return initialRotationAngles;
    }


    void MappingMatrix::PrintMeanMatrices()
    {
        for(int i = 0; i < this->__meanTransformations.size(); i++)
        {
            std::cout << "Mean transformation matrix for point cloud " << i << std::endl;
            this->__meanTransformations[i].PrintTransformation();
            std::cout << std::endl;
        }
    }


    Eigen::Matrix4d MappingMatrix::ComputeUmeyamaTransformationInSubtree(int i)
    {
        std::vector<int> subtreeIndices = Graph::GetInstanceOfUndirectedGraph().ExtractMSTSubTree(i);
        std::cout << "[DEBUG] Subtree indices starting from " << i << ": ";
        for (int index : subtreeIndices)
        {
            std::cout << index << " ";
        }
        std::cout << std::endl;

        Eigen::MatrixXd sourcePoints(2, subtreeIndices.size());
        Eigen::MatrixXd targetPoints(2, subtreeIndices.size());
        if(subtreeIndices.size() > 1)
        {
            for (size_t k = 0; k < subtreeIndices.size(); ++k) 
            {
                int subtreeIndex = subtreeIndices[k];
                Eigen::Vector3d sourcePoint = this->GetScan(subtreeIndex).GetPose().GetPosition();
                Eigen::Vector3d targetPoint = this->GetInitialPosition(subtreeIndex);
                sourcePoints(0, k) = sourcePoint.x();
                sourcePoints(1, k) = sourcePoint.y();
                targetPoints(0, k) = targetPoint.x();
                targetPoints(1, k) = targetPoint.y();
            }
        }
        else
        {
            std::cerr << "Error: Subtree has only one node, cannot compute Umeyama transformation." << std::endl;
            return Eigen::Matrix4d::Identity();
        }
        std::cout << "[DEBUG] Source points (2D): " << std::endl << sourcePoints << std::endl;
        std::cout << "[DEBUG] Target points (2D): " << std::endl << targetPoints << std::endl;

        // Compute 2D Umeyama transformation -> altitudes are ignored because unreliable
        Eigen::Matrix3d umeyama2D = Eigen::umeyama(sourcePoints, targetPoints, false);

        Eigen::Matrix4d umeyamaTransformation = Eigen::Matrix4d::Identity();
        double cos_theta = umeyama2D(0,0);
        double sin_theta = umeyama2D(1,0);
        umeyamaTransformation(0,0) = cos_theta;
        umeyamaTransformation(0,1) = -sin_theta;
        umeyamaTransformation(1,0) = sin_theta;
        umeyamaTransformation(1,1) = cos_theta;
        umeyamaTransformation(0,3) = umeyama2D(0,2);
        umeyamaTransformation(1,3) = umeyama2D(1,2);

        return umeyamaTransformation;
    }


    void MappingMatrix::SavePosesToFile(std::string fileName)
    {
        std::ofstream file(fileName);
        if(!file.is_open())
        {
            std::cerr << "Error: Could not open file " << fileName << " for writing." << std::endl;
            return;
        }
        file << "Index,OriginalX,OriginalY,OriginalZ,RegisteredX,RegisteredY,RegisteredZ,Roll,Pitch,Yaw" << std::endl;
        for(int i = 0; i < this->__scans.size(); i++)
        {
            Eigen::Vector3d originalPosition = this->__initialPositions[i];
            Eigen::Vector3d position = this->__scans[i].GetPose().GetPosition();
            Eigen::Matrix3d rotation = this->__scans[i].GetPose().GetOrientation().toRotationMatrix();
            Eigen::Vector3d eulerAngles = rotation.eulerAngles(0, 1, 2); // roll, pitch, yaw
            file << i << "," << originalPosition.x() << "," << originalPosition.y() << "," << originalPosition.z() << ","
                 << position.x() << "," << position.y() << "," << position.z() << ","
                 << eulerAngles.x() << "," << eulerAngles.y() << "," << eulerAngles.z() << std::endl;
        }
        file.close();
        std::cout << "Poses saved to file " << fileName << std::endl;
    }

    std::vector<std::vector<int>> CreateConnectivityMatrix(std::vector<Eigen::Vector3d> geolocations, int knn, double maxDistance)
    {
        std::vector<std::vector<std::pair<int, double>>> distancesToOtherPcs;
        std::vector<std::vector<int>> totalMatrix;
        distancesToOtherPcs.resize(geolocations.size());
        totalMatrix.resize(geolocations.size());
        std::vector<std::vector<int>> matrix(geolocations.size());

        for(int i = 0; i < geolocations.size(); i++)
        {
            for(int j = 0; j < geolocations.size(); j++)
            {
                if(i != j)
                {
                    // Calculate distance between geolocations
                    double distance = std::pow(geolocations[i].x() - geolocations[j].x(), 2) + std::pow(geolocations[i].y() - geolocations[j].y(), 2) + std::pow(geolocations[i].z() - geolocations[j].z(), 2);
                    distance = sqrt(distance);
                    distancesToOtherPcs[i].push_back(std::make_pair(j, distance));
                    totalMatrix[i].push_back(j);
                }
            }
        }

        // Get the knn closest neighbors for each node
        for(int i = 0; i < totalMatrix.size(); i++)
        {
            std::vector<int> neighbors = totalMatrix[i];
            // Sort distancesToOtherPcs[i] based on the distance
            std::sort(distancesToOtherPcs[i].begin(), distancesToOtherPcs[i].end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) 
            {
                return a.second < b.second;
            });

            // Select the k nearest neighbors
            matrix[i].resize(std::min(knn, static_cast<int>(distancesToOtherPcs[i].size())));
            for (int j = 0; j < matrix[i].size(); j++)
            {
                matrix[i][j] = distancesToOtherPcs[i][j].first; // Extract the index of the neighbor
            }

            // Debugging output
            std::cout << "Neighbors for " << i << ": ";
            for (int j = 0; j < matrix[i].size(); j++) 
            {
                std::cout << matrix[i][j] << " ";
            }
            std::cout << std::endl;
        }
        return matrix;
    }


    std::pair<Eigen::Matrix4d, float> ComputePairwiseTransformation(pcl::PointCloud<pcl::PointNormal>::Ptr source, pcl::PointCloud<pcl::PointNormal>::Ptr target, TransformationComputationMethod method)
    {
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        float score;

        if(method == TransformationComputationMethod::GlobalMatch)
        {
            // Compute transformation using GlobalMatch, currently a copy of the main function from GlobalMatch's main.cpp
            GlobalMatch::Mapping::Mapping globalMatchMapping;
            pcl::PointCloud<pcl::PointXYZ>::Ptr sourceNoNormals(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr targetNoNormals(new pcl::PointCloud<pcl::PointXYZ>);
            for(auto point : *source)
            {
                pcl::PointXYZ pointNoNormals;
                pointNoNormals.x = point.x;
                pointNoNormals.y = point.y;
                pointNoNormals.z = point.z;
                sourceNoNormals->push_back(pointNoNormals);
            }
            for(auto point : *target)
            {
                pcl::PointXYZ pointNoNormals;
                pointNoNormals.x = point.x;
                pointNoNormals.y = point.y;
                pointNoNormals.z = point.z;
                targetNoNormals->push_back(pointNoNormals);
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePosCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr targetPosCloud(new pcl::PointCloud<pcl::PointXYZ>);
            globalMatchMapping.setInputCloud(sourceNoNormals->makeShared());
            globalMatchMapping.extract(sourcePosCloud);
            globalMatchMapping.setInputCloud(targetNoNormals->makeShared());
            globalMatchMapping.extract(targetPosCloud);
            GlobalMatch::Matching::Matching globalMatchMatching;
            globalMatchMatching.setPairwiseStemPositions(sourcePosCloud, targetPosCloud);
            score = globalMatchMatching.estimateTransformation(transformation);
        }
        else
        {
            std::cerr << "Unknown transformation computation method" << std::endl;
        }
        std::cout << "Transformation matrix: " << std::endl << transformation << std::endl;
        // convert Matrix4f to Matrix4d
        Eigen::Matrix4d transformationDouble = Eigen::Matrix4d::Identity();
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                transformationDouble(i, j) = transformation(i, j);
            }
        }

        return {transformationDouble, score};
    }


    Eigen::Matrix4d RefinePairwiseTransformation(pcl::PointCloud<pcl::PointNormal>::Ptr target, pcl::PointCloud<pcl::PointNormal>::Ptr source, RefinementMethod method, double maxCorrespondenceDistance)
    {
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

        if(method == RefinementMethod::ICPNormals)
        {
            pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
            ne.setInputCloud(source);
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
            ne.setSearchMethod(tree);
            pcl::PointCloud<pcl::PointNormal>::Ptr sourceWithNormals(new pcl::PointCloud<pcl::PointNormal>);
            ne.setRadiusSearch(0.06);
            ne.compute(*sourceWithNormals);
            ne.setInputCloud(target);
            pcl::PointCloud<pcl::PointNormal>::Ptr targetWithNormals(new pcl::PointCloud<pcl::PointNormal>);
            ne.compute(*targetWithNormals);

            std::cout << "Computing transformation using ICP with normals" << std::endl;

            pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icpNormals;
            icpNormals.setInputSource(sourceWithNormals);
            icpNormals.setInputTarget(targetWithNormals);
            icpNormals.setMaximumIterations(50);
            icpNormals.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
            icpNormals.setTransformationEpsilon(0.0001);
            icpNormals.setEuclideanFitnessEpsilon(1);

            pcl::PointCloud<pcl::PointNormal>::Ptr dummy(new pcl::PointCloud<pcl::PointNormal>);
            icpNormals.align(*dummy);
            Eigen::Matrix4f transformationf = icpNormals.getFinalTransformation();
            // convert Matrix4f to Matrix4d
            for(int i = 0; i < 4; i++)
            {
                for(int j = 0; j < 4; j++)
                {
                    transformation(i, j) = transformationf(i, j);
                }
            }
        }
        else if (method == RefinementMethod::ICP)
        {
            pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
            icp.setInputSource(source);
            icp.setInputTarget(target);
            icp.setMaximumIterations(3);
            icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
            icp.setTransformationEpsilon(0.0001);
            icp.setEuclideanFitnessEpsilon(1);

            pcl::PointCloud<pcl::PointNormal>::Ptr dummy(new pcl::PointCloud<pcl::PointNormal>);
            icp.align(*dummy);
            Eigen::Matrix4f transformationf = icp.getFinalTransformation();
            // convert Matrix4f to Matrix4d
            for(int i = 0; i < 4; i++)
            {
                for(int j = 0; j < 4; j++)
                {
                    transformation(i, j) = transformationf(i, j);
                }
            }
        }
        
        else
        {
            std::cerr << "Unknown refinement method" << std::endl;
        }
        return transformation;
    }
}