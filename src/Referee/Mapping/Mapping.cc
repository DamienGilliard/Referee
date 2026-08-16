#include "Mapping.hh"

#include <set>

namespace Referee::Mapping
{
    void Scan::TransformScanPose(Eigen::Matrix4d transformation)
    {
        Eigen::Vector3d translation = transformation.block<3,1>(0,3);
        Eigen::Matrix3d rotationMatrix = transformation.block<3,3>(0,0);
        Eigen::Quaterniond rotation(rotationMatrix);
        Eigen::Vector3d poseTranslation = this->__pose.GetPosition();
        poseTranslation = rotation * poseTranslation + translation;
        __pose.Rotate(rotation);
        __pose.SetPosition(poseTranslation);
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
        if(this->__undirectedGraph.vertex_count() < 1)
        {
            std::cerr << "Error: The graph contains no vertices, and has probably not been properly created yet" << std::endl;
            exit(EXIT_FAILURE);
        }
        if(!this->__undirectedGraph.has_edge(vertex1, vertex2))
        {
            std::cerr << "Error: Trying to set weight of a non-existing edge." << std::endl;
            exit(EXIT_FAILURE);
        }
        this->__undirectedGraph.get_edge(vertex1, vertex2) = weight;
    }


    double Graph::GetWeight(int vertex1, int vertex2)
    {
        if(!this->__undirectedGraph.has_edge(vertex1, vertex2))
        {
            return 0;
        }
        else
        {
            return this->__undirectedGraph.get_edge(vertex1, vertex2);
        }
    }

    // ACTIVE
    std::vector<std::pair<long unsigned int, long unsigned int>> Graph::ComputeMinimumSpanningTree(int rootVertexIndex)
    {
        if(!__undirectedGraph.has_vertex(rootVertexIndex))
        {
            std::cerr << "Error: Root vertex is not part of the graph." << std::endl;
            exit(EXIT_FAILURE);
        }

        this->mstRootIndex = rootVertexIndex;
        auto mstEdgesOpt = graaf::algorithm::prim_minimum_spanning_tree(this->__undirectedGraph, rootVertexIndex);
        if (!mstEdgesOpt) 
        {
            std::cerr << "Error: Could not compute minimum spanning tree. The graph might be disconnected." << std::endl;
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


    long unsigned int Graph::GetClosestVertexToRoot(std::vector<long unsigned int> vertices)
    {
        if(vertices.empty())
        {
            std::cerr << "Error: The list of vertices is empty." << std::endl;
            exit(EXIT_FAILURE);
        }
        long unsigned int closestVertex = vertices[0];
        double minDistance = std::numeric_limits<double>::max();
        for(long unsigned int vertex : vertices)
        {
            auto pathOpt = graaf::algorithm::bfs_shortest_path(this->__minimumSpanningTree, this->mstRootIndex, vertex);
            if(pathOpt)
            {
                double distance = pathOpt.value().vertices.size();
                if(distance < minDistance)
                {
                    minDistance = distance;
                    closestVertex = vertex;
                }
            }
            else
            {
                std::cerr << "Error: No path found in MST between root and vertex " << vertex << std::endl;
            }
        }
        return closestVertex;
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

    void MappingMatrix::OptimiseAllPoses()
    {
        const double kMaxAcceptedLoopWeight = -12.0;
        // Genuine MST drift is a few tens of centimeters; a closing edge disagreeing with the
        // current poses far beyond that is a spurious stem match, not drift.
        const double kMaxLoopTranslationDiscrepancyM = 0.25;
        const double kMaxLoopRotationDiscrepancyRad = 5.0 * M_PI / 180.0;

        // All edges (MST and closing) come from the same stem-matching registration, whose
        // reliability scales with the number of matched stems (the graph weight is -score).
        // Deriving each edge's sigma from its score keeps high-score pairs effectively locked
        // (preserving their crisp pairwise alignment) and concentrates the loop-closure
        // correction in the low-score edges, which is where the drift accumulates.
        auto sigmasFromScore = [](double score) -> std::pair<double, double>
        {
            score = std::max(score, 3.0);
            double translationSigmaM = std::min(std::max(0.5 / score, 0.05), 0.15);
            double rotationSigmaRad  = std::min(std::max(0.02 / score, 0.0005), 0.005);
            return std::make_pair(translationSigmaM, rotationSigmaRad);
        };

        std::vector<std::pair<int, double*>> indicesAndPoseAsVectors;
        ceres::Problem problem;
        for (int i = 0; i < __mappingMatrix.size(); ++i)
        {
            Sophus::SE3d currentPose = this->GetScan(i).GetPose().ToSophusSE3();
            double* poseAsVector = new double[7];
            poseAsVector[0] = currentPose.unit_quaternion().x();
            poseAsVector[1] = currentPose.unit_quaternion().y();
            poseAsVector[2] = currentPose.unit_quaternion().z();
            poseAsVector[3] = currentPose.unit_quaternion().w();
            poseAsVector[4] = currentPose.translation().x();
            poseAsVector[5] = currentPose.translation().y();
            poseAsVector[6] = currentPose.translation().z();

            problem.AddParameterBlock(poseAsVector, 7, new Sophus::Manifold<Sophus::SE3>());
            indicesAndPoseAsVectors.push_back(std::make_pair(i, poseAsVector));
            if (i == this->GetGraph().GetMSTRootIndex())
            {
                problem.SetParameterBlockConstant(poseAsVector);
            }
        }

        for (std::pair<long unsigned int, long unsigned int> mstEdge : this->GetGraph().GetMinimumSpanningTree())
        {
            int from = static_cast<int>(mstEdge.first);
            int to   = static_cast<int>(mstEdge.second);
            const Eigen::Matrix4d& Tij = this->__mappingMatrix[from][to].GetTransformationMatrix();
            std::pair<double, double> edgeSigmas = sigmasFromScore(-this->GetGraph().GetWeight(from, to));
            ceres::CostFunction* costFunction = Referee::Mapping::TransformationError::Create(Tij, edgeSigmas.first, edgeSigmas.second);
            
            double* poseAsVectorFrom = indicesAndPoseAsVectors[from].second;
            double* poseAsVectorTo = indicesAndPoseAsVectors[to].second;

            problem.AddResidualBlock(costFunction, 
                                     new ceres::TukeyLoss(1.0), 
                                     poseAsVectorFrom, 
                                     poseAsVectorTo);

        }

        std::vector<std::pair<int, int>> acceptedClosures;
        for (const std::vector<long unsigned int>& correctionLoop : this->GetGraph().GetCorrectionLoops())
        {
            if (correctionLoop.size() < 2)
            {
                continue;
            }

            int loopFront = static_cast<int>(correctionLoop.front());
            int loopBack  = static_cast<int>(correctionLoop.back());

            Eigen::Matrix4d loopClosureRelativeTransform = this->__mappingMatrix[loopFront][loopBack].GetTransformationMatrix();
            double weight = this->GetGraph().GetWeight(loopFront, loopBack);
            if (weight > kMaxAcceptedLoopWeight)
            {
                std::cout << "[DEBUG] Skipping loop closure between " << loopFront << " and " << loopBack << " because the weight is too high: " << weight << std::endl;
                continue;
            }

            // Gate on how much the measurement disagrees with the current pose estimates.
            Eigen::Matrix3d measuredRotation = loopClosureRelativeTransform.block<3, 3>(0, 0);
            Sophus::SE3d measuredRelative(Eigen::Quaterniond(measuredRotation), loopClosureRelativeTransform.block<3, 1>(0, 3));
            Sophus::SE3d poseFront = this->GetScan(loopFront).GetPose().ToSophusSE3();
            Sophus::SE3d poseBack  = this->GetScan(loopBack).GetPose().ToSophusSE3();
            Sophus::SE3d discrepancy = measuredRelative.inverse() * (poseFront.inverse() * poseBack);
            double translationDiscrepancyM  = discrepancy.translation().norm();
            double rotationDiscrepancyRad = discrepancy.so3().log().norm();
            if (translationDiscrepancyM > kMaxLoopTranslationDiscrepancyM ||
                rotationDiscrepancyRad > kMaxLoopRotationDiscrepancyRad)
            {
                std::cout << "[DEBUG] Rejecting loop closure between " << loopFront << " and " << loopBack
                          << " as a likely spurious match (score " << -weight << "): discrepancy "
                          << translationDiscrepancyM << " m / " << rotationDiscrepancyRad * 180.0 / M_PI
                          << " deg exceeds gate (" << kMaxLoopTranslationDiscrepancyM << " m / "
                          << kMaxLoopRotationDiscrepancyRad * 180.0 / M_PI << " deg)" << std::endl;
                continue;
            }
            std::cout << "[DEBUG] Accepting loop closure between " << loopFront << " and " << loopBack
                      << " (score " << -weight << "): discrepancy " << translationDiscrepancyM << " m / "
                      << rotationDiscrepancyRad * 180.0 / M_PI << " deg" << std::endl;

            // The MST path between loopFront and loopBack is already constrained edge-by-edge by
            // the MST residual blocks above, so this single closing residual is enough: the joint
            // optimization distributes the correction along the path's poses.
            std::pair<double, double> closureSigmas = sigmasFromScore(-weight);
            ceres::CostFunction* costFunction = Referee::Mapping::TransformationError::Create(
                loopClosureRelativeTransform, closureSigmas.first, closureSigmas.second);
            problem.AddResidualBlock(costFunction, new ceres::TukeyLoss(1.0),
                                     indicesAndPoseAsVectors[loopFront].second,
                                     indicesAndPoseAsVectors[loopBack].second);
            acceptedClosures.push_back(std::make_pair(loopFront, loopBack));
        }

        // Soft priors
        for (const std::pair<int, double*>& indexAndPose : indicesAndPoseAsVectors)
        {
            int index = indexAndPose.first;
            double* poseAsVector = indexAndPose.second;
            Eigen::Matrix4d initialPoseTransform = this->GetScan(index).GetPose().ToTransformationMatrix();
            ceres::CostFunction* costFunction = Referee::Mapping::PosePriorError::Create(
                initialPoseTransform, 1.0);
            problem.AddResidualBlock(costFunction, new ceres::TukeyLoss(1.0), poseAsVector);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 100;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << std::endl;

        for (const auto& indexAndPose : indicesAndPoseAsVectors)
        {
            int index = indexAndPose.first;
            double* poseAsVector = indexAndPose.second;
            Eigen::Matrix4d optimizedPoseTransform = Eigen::Matrix4d::Identity();
            Eigen::Quaterniond optimizedQuaternion(
                poseAsVector[3],
                poseAsVector[0],
                poseAsVector[1],
                poseAsVector[2]);
            optimizedQuaternion.normalize();
            optimizedPoseTransform.block<3, 3>(0, 0) = optimizedQuaternion.toRotationMatrix();
            optimizedPoseTransform(0, 3) = poseAsVector[4];
            optimizedPoseTransform(1, 3) = poseAsVector[5];
            optimizedPoseTransform(2, 3) = poseAsVector[6];
            this->GetScan(index).GetPose() = Referee::Mapping::Pose(optimizedPoseTransform);
            delete[] poseAsVector; // Free the allocated memory
        }

        // Report where the remaining error lives: for every constraint, the disagreement
        // between its measurement and the optimized poses. High-score edges should stay in
        // the low-centimeter range; the drift correction should show up in low-score edges.
        auto reportEdgeResidual = [this](int from, int to, const char* edgeType)
        {
            const Eigen::Matrix4d Z = this->__mappingMatrix[from][to].GetTransformationMatrix();
            const Eigen::Matrix3d measuredRotation = Z.block<3, 3>(0, 0);
            const Sophus::SE3d measuredRelative(Eigen::Quaterniond(measuredRotation), Z.block<3, 1>(0, 3));
            const Sophus::SE3d discrepancy = measuredRelative.inverse() *
                (this->GetScan(from).GetPose().ToSophusSE3().inverse() * this->GetScan(to).GetPose().ToSophusSE3());
            std::cout << "[DEBUG] Final residual on " << edgeType << " edge " << from << " -> " << to
                      << " (score " << -this->GetGraph().GetWeight(from, to) << "): "
                      << discrepancy.translation().norm() * 100.0 << " cm / "
                      << discrepancy.so3().log().norm() * 180.0 / M_PI << " deg" << std::endl;
        };
        for (const std::pair<long unsigned int, long unsigned int>& mstEdge : this->GetGraph().GetMinimumSpanningTree())
        {
            reportEdgeResidual(static_cast<int>(mstEdge.first), static_cast<int>(mstEdge.second), "MST");
        }
        for (const std::pair<int, int>& closure : acceptedClosures)
        {
            reportEdgeResidual(closure.first, closure.second, "closure");
        }
    }

    std::vector<std::pair<int, Referee::Mapping::Pose>> MappingMatrix::ComputeLoopClosures()
    {
        std::vector<std::pair<int, Referee::Mapping::Pose>> optimizedPoses;
        const double kMaxAcceptedLoopWeight = -4.0;
        const double kLoopTranslationSigmaM = 1.0;
        const double kLoopRotationSigmaRad = 0.05;
        const double kPriorSqrtWeight = 0.5;

        // The edge that closes the loop (ie outside of the mst) is (correctionLoop.front(), correctionLoop.back())
        std::vector<std::vector<long unsigned int>> correctionLoops = this->GetGraph().GetCorrectionLoops();
        std::sort(correctionLoops.begin(), correctionLoops.end(), [](const auto& a, const auto& b){return a.size() < b.size();});

        for(const std::vector<long unsigned int>& loop : correctionLoops)
        {
            if (loop.size() < 2)
            {
                continue;
            }

            long unsigned int closestVertexToRoot = this->GetGraph().GetClosestVertexToRoot(loop);
            const int loopFront = static_cast<int>(loop.front());
            const int loopBack  = static_cast<int>(loop.back());
            const double weight = this->GetGraph().GetWeight(loopFront, loopBack);
            if(weight >= kMaxAcceptedLoopWeight)
            {
                std::cout << "[DEBUG] Skipping loop closure between " << loopFront << " and " << loopBack << " because the weight is too high: " << weight << std::endl;
                continue;
            }
            bool validPath = true;

            Eigen::Matrix4d TPathFrontToBack = Eigen::Matrix4d::Identity();;

            for (int i = 1; i < static_cast<int>(loop.size()); ++i)
            {
                const int from = static_cast<int>(loop[i - 1]);
                const int to   = static_cast<int>(loop[i]);

                const Eigen::Matrix4d& Tij = this->__mappingMatrix[from][to].GetTransformationMatrix();
                if (!Tij.allFinite())
                {
                    std::cerr << "[ERROR] Invalid MST path edge transform " << from << " -> " << to << std::endl;
                    validPath = false;
                    break;
                }
                TPathFrontToBack = TPathFrontToBack * Tij;
            }

            const Eigen::Matrix4d& TFrontToBack = this->__mappingMatrix[loopFront][loopBack].GetTransformationMatrix();
            if (!validPath || !TFrontToBack.allFinite())
            {
                std::cerr << "[ERROR] Invalid loop closure transforms for loop front/back "
                        << loopFront << " <-> " << loopBack << std::endl;
                continue;
            }

            const Eigen::Matrix4d E_loop = TPathFrontToBack.inverse() * TFrontToBack;
            const Eigen::Matrix<double, 6, 1> xi_loop = Referee::Utils::Conversions::transformMatrixToTwist(E_loop);

            std::cout << "[DEBUG] Loop closure from " << loopFront << " to " << loopBack << std::endl;
            std::cout << "[DEBUG] Error transformation matrix: \n" << E_loop << std::endl;
            std::cout << "[DEBUG] Error twist vector: " << xi_loop.transpose() << std::endl;
            const double transErr = xi_loop.head<3>().norm();
            const double rotErr   = xi_loop.tail<3>().norm();
            std::cout << "[DEBUG] Loop " << loopFront << " -> " << loopBack
                    << " | transErr=" << transErr
                    << " rotErr(rad)=" << rotErr << std::endl;

            ceres::Problem problem;
            std::vector<double*> posesAsVectors;

            // We first set the initial poses into the ceres problem (poses that will be optimized)
            std::unordered_map<int, double*> scanIndexToParamBlock;
            bool allocationFailed = false;
            for(int i = 0; i < loop.size(); i++)
            {
                double *poseAsVector = new double[6];
                if (!poseAsVector) 
                {
                    std::cerr << "[ERROR] Failed to allocate poseAsVector for index " << i << std::endl;
                    allocationFailed = true;
                    break;
                }
                Eigen::Matrix4d poseTransform = this->GetScan(loop[i]).GetPose().ToTransformationMatrix();
                Eigen::Matrix<double, 6, 1> poseVector = Referee::Utils::Conversions::transformMatrixToTwist(poseTransform);
                poseAsVector[0] = poseVector[0];
                poseAsVector[1] = poseVector[1];
                poseAsVector[2] = poseVector[2];
                poseAsVector[3] = poseVector[3];
                poseAsVector[4] = poseVector[4];
                poseAsVector[5] = poseVector[5];

                posesAsVectors.push_back(poseAsVector);
                scanIndexToParamBlock[loop[i]] = poseAsVector;
                problem.AddParameterBlock(poseAsVector, 6);
                                
                std::cout << "[DEBUG] Added parameter block for scan " << loop[i] << " with initial pose: "
                          << poseVector.transpose() << std::endl;

                if (loop[i] == closestVertexToRoot)
                {
                    problem.SetParameterBlockConstant(poseAsVector); // fix the first pose to anchor the loop
                }
            }

            auto cleanupPoseBuffers = [&scanIndexToParamBlock]()
            {
                for (auto& kv : scanIndexToParamBlock)
                {
                    delete[] kv.second;
                }
            };

            if (allocationFailed)
            {
                cleanupPoseBuffers();
                continue;
            }

            // Soft prior: keep each loop pose close to its pre-loop value.
            // This limits over-corrections when one closure edge is unreliable.
            for (int i = 0; i < static_cast<int>(loop.size()); ++i)
            {
                const int scanIdx = static_cast<int>(loop[i]);

                if (scanIdx == static_cast<int>(closestVertexToRoot))
                {
                    continue;
                }

                const Eigen::Matrix4d referencePose =
                    this->GetScan(scanIdx).GetPose().ToTransformationMatrix();

                ceres::CostFunction* priorCost =
                    Referee::Mapping::PosePriorError::Create(referencePose, 1.0 / kPriorSqrtWeight, 1.0 / kPriorSqrtWeight);

                auto paramIt = scanIndexToParamBlock.find(scanIdx);
                if (paramIt == scanIndexToParamBlock.end() || paramIt->second == nullptr)
                {
                    std::cerr << "[ERROR] Missing parameter block for prior at scan " << scanIdx << std::endl;
                    continue;
                }

                problem.AddResidualBlock(
                    priorCost,
                    nullptr,
                    paramIt->second);
            }

            // We then set the constraints (in our case transformation measurements)
            for(int i = 0; i < loop.size()-1; i++)
            {
                int fromIndex = loop[i]; // current vertex in the loop
                int toIndex = loop[i + 1]; // next vertex in the loop, wrap around at the end
                if (!this->__mappingMatrix[fromIndex][toIndex].GetTransformationMatrix().allFinite()) 
                {
                    std::cerr << "[ERROR] Invalid transformation matrix between vertices " 
                            << fromIndex << " and " << toIndex << std::endl;

                    std::cerr << "Transformation matrix: \n" << this->__mappingMatrix[fromIndex][toIndex].GetTransformationMatrix()<< std::endl;
                    continue;
                }
                std::cout << "[DEBUG] Adding constraint between " << fromIndex << " and " << toIndex << std::endl;
                const Eigen::Matrix4d& transformation = this->__mappingMatrix[fromIndex][toIndex].GetTransformationMatrix();
                // const Eigen::Matrix4d& registredPose = transformation * this->GetScan(fromIndex).GetPose().ToTransformationMatrix();
                // const Eigen::Matrix4d& poseDifference = registredPose.inverse() * this->GetScan(toIndex).GetPose().ToTransformationMatrix();
                std::cout << "[DEBUG] Transformation matrix between " << fromIndex << " and " << toIndex << ": \n" << transformation << std::endl;
                ceres::CostFunction* costFunction = Referee::Mapping::TransformationError::Create(
                    transformation,
                    kLoopTranslationSigmaM,
                    kLoopRotationSigmaRad);
                std::cout << "[DEBUG] getting indexes for parameter blocks for edge " << fromIndex << " -> " << toIndex << std::endl;
                auto fromIt = scanIndexToParamBlock.find(fromIndex);
                auto toIt = scanIndexToParamBlock.find(toIndex);
                std::cout << "[DEBUG] Adding residual block for edge " << fromIndex << " -> " << toIndex << std::endl;
                if (fromIt == scanIndexToParamBlock.end() || toIt == scanIndexToParamBlock.end() ||
                    fromIt->second == nullptr || toIt->second == nullptr)
                {
                    std::cerr << "[ERROR] Missing parameter block for loop edge " << fromIndex << " -> " << toIndex << std::endl;
                    continue;
                }
                std::cout << "[DEBUG] Adding residual block for edge " << fromIndex << " -> " << toIndex << std::endl;
                problem.AddResidualBlock(costFunction, 
                                         new ceres::HuberLoss(1.0),
                                         fromIt->second,
                                         toIt->second);
            }

            auto frontIt = scanIndexToParamBlock.find(loopFront);
            auto backIt = scanIndexToParamBlock.find(loopBack);
            if (frontIt == scanIndexToParamBlock.end() || backIt == scanIndexToParamBlock.end() ||
                frontIt->second == nullptr || backIt->second == nullptr)
            {
                std::cerr << "[ERROR] Missing parameter block for loop closure edge " << loopFront << " -> " << loopBack << std::endl;
                cleanupPoseBuffers();
                continue;
            }
            ceres::CostFunction* loopClosureCost = Referee::Mapping::TransformationError::Create(
                TFrontToBack,
                kLoopTranslationSigmaM,
                kLoopRotationSigmaRad);
            problem.AddResidualBlock(
                loopClosureCost,
                new ceres::HuberLoss(1.0),
                frontIt->second,
                backIt->second
            );
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = true;
            options.function_tolerance = 1e-15; 
            options.gradient_tolerance = 1e-15;
            options.parameter_tolerance = 1e-15; 
            options.max_num_iterations = 1000;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << std::endl;
            if (!summary.IsSolutionUsable() || summary.termination_type == ceres::FAILURE)
            {
                std::cerr << "[ERROR] Unusable loop-closure solve for loop " << loopFront
                          << " -> " << loopBack << ". Skipping update." << std::endl;
                // cleanupPoseBuffers();
                continue;
            } 

            std::vector<std::pair<int, Eigen::Matrix4d>> acceptedUpdates;
            bool acceptLoopUpdate = true;
            for (int i = 0; i < posesAsVectors.size(); i++)
            {
                int scanIdx = static_cast<int>(loop[i]);
                auto paramIt = scanIndexToParamBlock.find(scanIdx);
                if (paramIt == scanIndexToParamBlock.end() || paramIt->second == nullptr)
                {
                    acceptLoopUpdate = false;
                    break;
                }

                Eigen::Map<Eigen::Matrix<double, 6, 1>> poseVec(scanIndexToParamBlock[scanIdx]);
                Eigen::Matrix4d initialPoseTransform = this->GetScan(scanIdx).GetPose().ToTransformationMatrix();
                Eigen::Matrix4d optimizedPoseTransform = Referee::Utils::Conversions::poseAsVectorToTransformationMatrix(poseVec);

                if (!optimizedPoseTransform.allFinite())
                {
                    std::cerr << "[ERROR] Non-finite optimized pose for scan " << scanIdx << std::endl;
                    acceptLoopUpdate = false;
                    break;
                }

                const Eigen::Matrix4d correctionTransform = optimizedPoseTransform * initialPoseTransform.inverse();
                const Eigen::Matrix<double, 6, 1> correctionTwist =
                    Referee::Utils::Conversions::transformMatrixToTwist(correctionTransform);
                const double correctionTranslation = correctionTwist.head<3>().norm();
                const double correctionRotation = correctionTwist.tail<3>().norm();

                // if (correctionTranslation > kMaxPerPoseTranslationUpdate ||
                //     correctionRotation > kMaxPerPoseRotationUpdateRad)
                // {
                //     std::cerr << "[ERROR] Rejecting loop update due to large per-pose correction on scan "
                //               << scanIdx << " (dT=" << correctionTranslation
                //               << ", dR=" << correctionRotation << ")" << std::endl;
                //     acceptLoopUpdate = false;
                //     break;
                // }

                acceptedUpdates.push_back(std::make_pair(scanIdx, optimizedPoseTransform));
            }

            if (acceptLoopUpdate)
            {
                for (const auto& update : acceptedUpdates)
                {
                    Referee::Mapping::Pose optimizedPose(update.second);
                    this->GetScan(update.first).GetPose() = optimizedPose;
                    optimizedPoses.push_back(std::make_pair(update.first, optimizedPose));
                }
            }

            cleanupPoseBuffers();
        }
        return optimizedPoses;
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

        // Connect each scan to every scan within maxDistance: any such pair overlaps enough to be
        // stem-registered, and a missing edge means the pair's relative alignment is only inherited
        // through long chains of other measurements.
        for(int i = 0; i < totalMatrix.size(); i++)
        {
            // Sort distancesToOtherPcs[i] based on the distance
            std::sort(distancesToOtherPcs[i].begin(), distancesToOtherPcs[i].end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b)
            {
                return a.second < b.second;
            });

            for (int j = 0; j < distancesToOtherPcs[i].size(); j++)
            {
                if (distancesToOtherPcs[i][j].second <= maxDistance)
                {
                    matrix[i].push_back(distancesToOtherPcs[i][j].first);
                }
            }

            // Fallback for a scan with no neighbor within maxDistance: connect its knn nearest
            // anyway, otherwise the graph is disconnected and no MST can be computed.
            if (matrix[i].size() < knn)
            {
                std::cout << "Warning: scan " << i << " has less than " << knn
                          << " neighbors within " << maxDistance << " m, falling back to its " << knn << " nearest neighbors." << std::endl;
                matrix[i].clear();
                for (int j = 0; j < std::min(knn, static_cast<int>(distancesToOtherPcs[i].size())); j++)
                {
                    matrix[i].push_back(distancesToOtherPcs[i][j].first);
                }
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


    std::pair<Eigen::Matrix4d, float> RefinePairwiseTransformation(pcl::PointCloud<pcl::PointNormal>::Ptr target, pcl::PointCloud<pcl::PointNormal>::Ptr source, RefinementMethod method, double maxCorrespondenceDistance)
    {
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
        std::pair<Eigen::Matrix4d, float> result;

        if(method == RefinementMethod::ICPNormals)
        {
            if(source->points[0].normal_x == 0 && source->points[0].normal_y == 0 && source->points[0].normal_z == 0 || target->points[0].normal_x == 0 && target->points[0].normal_y == 0 && target->points[0].normal_z == 0)
            {
                std::cerr << "Source and or target point cloud has no normals, cannot use ICP with normals" << std::endl;
                return {transformation, 0.0f};
            }

            std::cout << "Computing transformation using ICP with normals" << std::endl;

            pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icpNormals;
            icpNormals.setInputSource(source);
            icpNormals.setInputTarget(target);
            icpNormals.setMaximumIterations(50);
            icpNormals.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
            icpNormals.setTransformationEpsilon(0.0001);
            icpNormals.setEuclideanFitnessEpsilon(0.0001);

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
            result = {transformation, icpNormals.getFitnessScore()};
        }
        
        else if (method == RefinementMethod::ICP)
        {
            pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
            icp.setInputSource(source);
            icp.setInputTarget(target);
            icp.setMaximumIterations(50);
            icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
            icp.setTransformationEpsilon(0.0001);
            icp.setEuclideanFitnessEpsilon(0.0001);

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
            result = {transformation, icp.getFitnessScore()};
        }
        
        else
        {
            std::cerr << "Unknown refinement method, returning identity transformation" << std::endl;
            result = {transformation, 0.0f};
        }
        
        return result;
    }
}