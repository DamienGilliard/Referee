#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <utility>
#include <boost/functional/hash.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <graaflib/graph.h>
#include <graaflib/edge.h>
#include <graaflib/io/dot.h>
#include <graaflib/algorithm/minimum_spanning_tree/prim.h>
#include <graaflib/algorithm/shortest_path/bfs_shortest_path.h>

#include "../../3rd_party/GlobalMatch/code/global_match/stem_mapping.h"
#include "../../3rd_party/GlobalMatch/code/global_match/stem_matching.h"
#include "../GeometricTransformations/GeometricTransformation.hh"
#include "../Probability/Probability.hh"
#include "../Utils/Utils.hh"


// Hash function for Eigen::Vector3d to use it in unordered_map
// This is necessary because Eigen::Vector3d does not have a hash function by default
namespace std 
{
    template <>
    struct hash<Eigen::Vector3d> 
    {
        std::size_t operator()(const Eigen::Vector3d& v) const noexcept 
        {
            std::size_t h1 = std::hash<double>{}(v.x());
            std::size_t h2 = std::hash<double>{}(v.y());
            std::size_t h3 = std::hash<double>{}(v.z());
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}

namespace Referee::Mapping
{
    enum class CoordinateSystemType
    {
        LV95, // Swiss coordinate system
        WGS84, // World coordinate system
        Local // Local coordinate system
    };

    class Pose
    {
        public:
            Pose() = default;
            Pose(Eigen::Vector3d position, Eigen::Quaterniond orientation)
                : position_(position), orientation_(orientation) 
                {
                    std::cout << "Pose created with position: " << position_.transpose() 
                              << " and orientation (quaternion): " << orientation_.coeffs().transpose() << std::endl;
                }


            /**
             * @brief Get the Position object
             * @return Position in the global coordinate system
             */
            Eigen::Vector3d& GetPosition() { return this->position_; }


            /**
             * @brief Set the Position object
             * @param position New position in the global coordinate system
             */
            void SetPosition(Eigen::Vector3d& position) { this->position_ = position; }


            /**
             * @brief Translate the position by a given vector
             * @param translation Translation vector in the global coordinate system
             */
            void Translate(Eigen::Vector3d& translation) { this->position_ += translation; }


            /**
             * @brief Get the Orientation quaternion object
             * @return Orientation in the global coordinate system as a quaternion
             */
            Eigen::Quaterniond& GetOrientation() { return this->orientation_; }



            /**
             * @brief Set the Orientation object
             * @param orientation New orientation in the global coordinate system as a quaternion
             */
            void SetOrientation(Eigen::Quaterniond& orientation) { this->orientation_ = orientation; }


            /**
             * @brief Rotate the orientation by a given quaternion
             * @param rotation Rotation quaternion in the global coordinate system
             */
            void Rotate(Eigen::Quaterniond& rotation) { this->orientation_ = rotation * this->orientation_; }

        private:
            Eigen::Vector3d position_; // Position in the global coordinate system

            Eigen::Quaterniond orientation_; // Orientation in the global coordinate system
    };

    /**
     * @brief Singleton class to store the global coordinate system
     * 
     * This class is used to store the global coordinate system in which all point clouds are transformed.
     * It is a singleton class, meaning that there is only one instance of this class in the whole program.
     */
    class GlobalCoordinateSystem
    {
        public:
            /**
             * @brief Get the instance of the global coordinate system
             * @return GlobalCoordinateSystem instance
             */
            static GlobalCoordinateSystem& GetInstance()
            {
                if(!__instance){__instance = new GlobalCoordinateSystem();}
                return *__instance;
            }


            static GlobalCoordinateSystem& CreateGlobalCoordinateSystem(CoordinateSystemType type = CoordinateSystemType::WGS84,
                                                                       Eigen::Vector3d origin = Eigen::Vector3d::Zero(),
                                                                       Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity())
            {
                if(!__instance){__instance = new GlobalCoordinateSystem(type, origin, orientation);}
                return *__instance;
            }

        private:
            GlobalCoordinateSystem(CoordinateSystemType type = CoordinateSystemType::WGS84,
                                   Eigen::Vector3d origin = Eigen::Vector3d::Zero(),
                                   Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity())
                : __type(type), __origin(origin), __orientation(orientation) {}

            GlobalCoordinateSystem(const GlobalCoordinateSystem&) = delete; // prevent copying

            GlobalCoordinateSystem& operator=(const GlobalCoordinateSystem&) = delete; // prevent assignment

            GlobalCoordinateSystem(GlobalCoordinateSystem&&) = delete; // prevent moving

            GlobalCoordinateSystem& operator=(GlobalCoordinateSystem&&) = delete; // prevent moving

            ~GlobalCoordinateSystem() = default; // destructor

            /**
             * @brief Type of the global coordinate system
             */
            CoordinateSystemType __type = CoordinateSystemType::WGS84; // type of the global coordinate system

            /**
             * @brief Origin of the global coordinate system
             */
            Eigen::Vector3d __origin = Eigen::Vector3d::Zero(); // origin of the global coordinate system

            /**
             * @brief Orientation of the global coordinate system
             */
            Eigen::Quaterniond __orientation = Eigen::Quaterniond::Identity(); // orientation of the global coordinate system

            /**
             * @brief Static instance of the global coordinate system
             * 
             * This is a static instance of the class, which is used to ensure that there is only one instance of the class in the whole program.
             */
            static GlobalCoordinateSystem* __instance; // static instance of the class
    };


    /**
     * @brief Scan class to store a point cloud, its associated pose, and a few useful attributes
     */
    class Scan
    {
        public:

            Scan() = default;
            /**
             * @brief Construct a new Scan object using an initial pose and a point cloud file name to be loaded at construction
             * @param pose Initial pose of the scan in the global coordinate system
             * @param cloudFileName File name of the point cloud to be loaded (PLY format)
             */
            Scan(Pose pose, std::string cloudFileName, double voxelSize) : __pose(pose),
                                                         __cloudFileName(cloudFileName)
            {}


            void TranslateScan(Eigen::Vector3d translation)
            {
                __pose.Translate(translation);
                Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(__cloud, translation);
            }


            /**
             * @brief Rotate the scan by a given transformation matrix
             * @param transformation Transformation matrix in the global coordinate system
             */
            void TransformScan(Eigen::Matrix4d transformation);


            /**
             * @brief Get a pointer to the pose of the scan
             * @return Pointer to the pose in the global coordinate system
             */
            Pose& GetPose() { return this->__pose; }


            /**
             * @brief Load the point cloud from the file
             */
            void LoadCloud();


            /**
             * @brief Get a pointer to the point cloud of the scan
             * @return Pointer to the point cloud
             */
            pcl::PointCloud<pcl::PointNormal>::Ptr& GetCloud() {return this->__cloud;}


            /**
             * @brief Clear the point cloud data
             */
            void FlushCloud() { __cloud.reset(new pcl::PointCloud<pcl::PointNormal>()); }


            /**
             * @brief Get the file name of the point cloud
             * @return File name of the point cloud
             */
            std::string GetCloudFileName() { return __cloudFileName; }


        private:

            Pose __pose;

            pcl::PointCloud<pcl::PointNormal>::Ptr __cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());

            std::string __cloudFileName;
    };


    /**
     * @brief This class stores a transformation in 3D in the form of a rotation axis, a point on this axis, a rotation along this axis, and a translation along this axis. It also stores a score for the transformation (for example the number of correspondance used)
     */
    class Transformation
    {
        public:
            Transformation() = default;

            
            Transformation(Eigen::Matrix4d transformationMatrixInGlobalCoordinateSystem,
                           std::shared_ptr<Scan> fromScan = nullptr,
                           std::shared_ptr<Scan> toScan = nullptr);

            /**
             * @brief Print the transformation to the console
             */
            void PrintTransformation();


            /**
             * @brief Get the transformation matrix in the global coordinate system
             * @return Eigen::Matrix4d transformation matrix in the global coordinate system
             */
            Eigen::Matrix4d GetTransformationMatrix() const { return __globalTransformation; }


            /**
             * @brief Get the inverse transformation matrix in the global coordinate system
             * @return Eigen::Matrix4d inverse transformation matrix in the global coordinate system
             */
            Eigen::Matrix4d GetInverse() const { return __globalTransformation.inverse(); }

            /**
            * @brief Get the rotation part of the transformation as a quaternion
            * @return Eigen::Quaterniond rotation part of the transformation as a quaternion
            */
            Eigen::Quaterniond GetRotationAsQuaternion() const { return __quaternion; }

            
        private:
            Eigen::Matrix4d __globalTransformation; // transformation matrix in the global coordinate system

            Eigen::Quaterniond __quaternion; // quaternion representing the rotation

            std::shared_ptr<Scan> __fromScan = nullptr; // pointer to the scan from which the transformation is computed

            std::shared_ptr<Scan> __toScan = nullptr; // pointer to the scan to which the transformation is computed
    };


    enum class GraphType
    {
        Undirected, // Undirected graph
        Directed // Directed graph
    };

    class Graph
    {
        public: 
            /**
             * @brief Create a new undirected graph instance if none exists (singleton pattern)
             * @return Graph instance
             */
            static Graph& CreateUndirectedGraph();


            /**
             * @brief Create a new undirected graph singleton instance with given vertices and edges
             * @param vertices List of vertices to add to the graph
             * @param edges List of edges to add to the graph
             * @return Graph instance
             */
            static Graph& CreateUndirectedGraph(std::vector<Eigen::Vector3d> vertices, std::vector<std::vector<int>> edges);


            /**
             * @brief Get the instance of the undirected graph
             * @return Graph instance
             */
            static Graph& GetInstanceOfUndirectedGraph();


            /**
             * @brief Add a vertex to the graph
             * @param vertex The vertex to add
             */
            void AddVertex(Eigen::Vector3d vertex);


            /**
             * @brief Add an edge to the graph
             * @param vertex1 The first vertex of the edge
             * @param vertex2 The second vertex of the edge
             * @param weight The weight to assign to the edge
             */
            void AddEdge(Eigen::Vector3d vertex1, 
                         Eigen::Vector3d vertex2, 
                         double weight);


            /**
             * @brief Get the vertex count of the graph
             * @return Number of vertices in the graph
             */
            int GetVertexCount() const { return this->__nVertices; }


            /**
             * @brief Get the edge count of the graph
             * @return Number of edges in the graph
             */
            int GetEdgeCount() const { return this->_nEdges; }


            /**
             * @brief Set the weight of an edge in the graph
             */
            void SetWeight(int vertex1, int vertex2, double weight);


            /**
             * @brief Compute the minimum spanning tree of the graph using Prim's algorithm
             * @param startVertexIndex The index of the root vertex for the algorithm
             * @return A vector of vertices in the minimum spanning tree
             */
            std::vector<std::pair<long unsigned int, long unsigned int>> ComputeMinimumSpanningTree(int startVertexIndex);


            /**
             * @brief Print the graph to the console
             */
            void PrintGraph();


            /**
             * @brief Extract a sub-tree from the graph's minimum spanning tree (computed earlier) and return its vertices. All vertices between the starting vertex and the leaves are included in the sub-tree. The root of the original minimum spanning tree must have ID 0.
             * @param startingVertexIndex The index of the starting vertex for the sub-tree
             * @return A vector containing the sub-tree's vertices
             */
            std::vector<int> ExtractMSTSubTree(int startingVertexIndex);


            /**
             * @brief Get the minimum spanning tree edges
             * @return A vector of edges in the minimum spanning tree
             */
            std::vector<std::pair<long unsigned int, long unsigned int>> GetMinimumSpanningTree()
            {
                return this->__minimumSpanningTreeEdges;
            }


            /**
             * @brief gets the graph edges that do not belong to the minimum spanning tree
             * 
             * @return A vector of edges not in the minimum spanning tree
             */
            std::vector<std::pair<long unsigned int, long unsigned int>> GetNonMSTEdges();


            /**
             * @brief in the minimum spanning tree graph, we find the shortest path between the vertices of the edges that are not part of the minimum spanning tree. 
             * These paths form correction loops that can be used to correct drift in the transformations.
             * Because we work within a tree, there will be only one unique path between two vertices, and we can use
             * a BFS algorithm to find this path (does not use edge weights). Docu: https://bobluppes.github.io/graaf/docs/algorithms/shortest-path/bfs-based-shortest-path
             * 
             * @return A vector of pairs representing the correction loops
             */
            std::vector<std::vector<std::pair<long unsigned int, long unsigned int>>> GetCorrectionLoops();


        private:

            Graph(GraphType type);

            Graph(const Graph&) = delete;

            Graph& operator=(const Graph&) = delete;
            
            Graph(Graph&&) = delete;
            
            Graph& operator=(Graph&&) = delete;

            std::unordered_map<Eigen::Vector3d, int> __vertexIndices; // Map to store the positions to their indices.

            static Graph* __instance;

            int __nVertices = 0; 

            int _nEdges = 0;

            const bool __isDirected;

            graaf::undirected_graph<int, double> __undirectedGraph; // Undirected graph to store the connectivity between the point clouds

            graaf::undirected_graph<int, double> __minimumSpanningTree; // Minimum spanning tree of the graph
            
            std::vector<std::pair<long unsigned int, long unsigned int>> __minimumSpanningTreeEdges; // edges of the minimum spanning tree of the graph, stored as a vector of edges (pairs of vertex indices)
    };


    /**
     * @brief MappingMatrix class to store the N x N mapping matrix `M` between N point clouds, where each element M_{i,j} is the 4x4 transformation matrix between point cloud i and point cloud j
     */
    class MappingMatrix
    {
        public:
            /**
             * @brief Construct a new MappingMatrix object
             * @param numPointClouds Number of point clouds
             */
            MappingMatrix(int numPointClouds)
                : __graph(Graph::GetInstanceOfUndirectedGraph()) // because we need to initialize the graph member. It is a singleton class, so we can initialize it here and fill it later with the actual graph.
            {
                __mappingMatrix.resize(numPointClouds);
                for (int i = 0; i < numPointClouds; i++)
                {
                    __mappingMatrix[i].resize(numPointClouds);
                }

                for(int i = 0; i < numPointClouds; i++)
                {
                    for(int j = 0; j < numPointClouds; j++)
                    {
                        __mappingMatrix[i][j] = Referee::Mapping::Transformation();
                    }
                }
            }


            // Getters and setters

            /**
             * @brief Setter for the connectivity matrix
             * @param connectivityMatrix Connectivity matrix
             */
            void SetConnectivityMatrix(std::vector<std::vector<int>> connectivityMatrix)
            {
                this->__connectivityMatrix = connectivityMatrix;
            }


            /**
             * @brief Getter for the connectivity matrix
             * @return Connectivity matrix
             */
            std::vector<std::vector<int>> GetConnectivityMatrix()
            {
                return this->__connectivityMatrix;
            }

            
            /**
             * @brief Setter for the list of scans (point clouds with associated poses)
             * @param scans List of scans
             */
            void SetScans(std::vector<Referee::Mapping::Scan>& scans)
            {
                this->__scans = scans;
            }


            /**
             * @brief Getter for the list of scans (point clouds with associated poses)
             * @return List of scans
             */
            std::vector<Referee::Mapping::Scan>& GetScans()
            {
                return this->__scans;
            }


            /**
             * @brief Getter for a scan at a given index
             * @param index Index of the scan
             * @return Scan at the given index
             */
            Referee::Mapping::Scan& GetScan(int index)
            {
                return this->__scans[index];
            }


            /**
             * @brief Getter for the graph singleton instance
             * @return Graph
             */
            Graph& GetGraph()
            {
                return this->__graph.GetInstanceOfUndirectedGraph();
            }


            /**
             * @brief Set the initial positions of the point clouds
             * @param initialPositions Initial positions of the point clouds
             */
            void SetInitialPositions(std::vector<Eigen::Vector3d> initialPositions)
            {
                this->__initialPositions = initialPositions;
            }


            /**
             * @brief Getter for the initial position of a point cloud
             * @param i Index of the point cloud
             * @return Initial position of the point cloud
             */
            Eigen::Vector3d GetInitialPosition(int i)
            {
                return this->__initialPositions[i];
            }


            /**
             * @brief Setter for the transformation matrix between two point clouds
             * @param i Index of the first point cloud
             * @param j Index of the second point cloud
             * @param transformation Transformation matrix between the two point clouds
             */
            void SetTransformation(int i, int j, Referee::Mapping::Transformation transformation)
            {
                __mappingMatrix[i][j] = transformation;
            }


            /**
             * @brief Getter of the transformation matrix between two point clouds
             * @param i Index of the first point cloud
             * @param j Index of the second point cloud
             * @return Referee::Mapping::Transformation between the two point clouds
             */
            Referee::Mapping::Transformation GetTransformation(int i, int j)
            {
                return __mappingMatrix[i][j];
            }


            /**
             * @brief Getter for the standard deviation of the rotation angles of the transformation matrices
             * @param i Index of the point cloud
             * @return Standard deviation of the rotation angles of the transformation matrices
             */
            double GetStdDevRotation(int i){return __stdDevRotations[i];}


            /**
             * @brief Getter for the mean rotation angle of the transformation matrices
             * @param i Index of the point cloud
             * @return Mean rotation angle of the transformation matrices
             */
            double GetMeanRotation(int i){return __meanTransformations[i].GetRotationAngle();}


            /**
             * @brief Getter for the mean translation vector of the transformation matrices
             * @param i Index of the point cloud
             * @return Mean translation vector of the transformation matrices
             */
            Eigen::Vector3d GetMeanTranslationVector(int i){return __meanTranslationVectors[i];}


            /**
             * @brief Calculate the mean transformation matrix per point cloud (i.e. the mean transformation matrix along one row of the mapping matrix)
             */
            void CalculateMeanTransformationMatrices();


            /**
             * @brief Getter for the mean transformation matrix of a point cloud
             * @param i Index of the point cloud
             * @return Mean transformation matrix of the point cloud
             */
            Referee::Mapping::Transformation GetMeanTransformation(int i){return __meanTransformations[i];}


            /**
             * @brief Computes the most probable mean rotation angle throughout the point clouds
             * @return A tuple containing the index of the point cloud with the most probable rotation angle and the probability of this rotation angle
             */
            std::tuple<int, double> GetMostProbableRotation();


            /**
             * @brief Computes the overall mean rotation of the original point clouds, i.e. the mean error in orientation of the scanner's compass.
             * @return Overall mean rotation angle (in radians)
             */
            double GetOverallMeanRotation();

            
            /**
             * @brief Computes the most probable translation vector throughout the point clouds
             * @return A pair containing the index of the point cloud with the most probable translation vector and the probability of this translation vector
             */
            std::pair<int, double> GetMostProbableTranslation();


            /**
             * @brief for each point cloud, get the mean rotations and standard deviations of the rotation angles of the transformation matrices
             * @return A vector of pairs, where each pair contains the mean rotation angle and the standard deviation of the rotation angle for each point cloud
             */
            std::vector<std::pair<double, double>> GetMeanRotationsAndStdDevs();


            /**
             * @brief Compute the mean translation vectors for each point cloud
             * @return A vector of pairs, where each pair contains the mean translation vector and its standard deviation for each point cloud
             */
            std::vector<std::pair<double, Eigen::Vector3d>> GetMeanTranslationVectorsAndStdDevs();


            /**
             * @brief Compute the mean pose rotation error by computing the mean translation induced rotation
             * @return Mean translation induced rotation
             */
            double ComputeMeanTranslationInducedRotation();


            /**
             * @brief Compute the rotation coefficients between the point clouds
             * @param mostTrustworthyRotationAngle The rotation angle of the most trustworthy point cloud
             * @param mostTrustworthyPointCloudIndex The index of the most trustworthy point cloud
             */
            void ComputeRotationCoefficients(int mostTrustworthyPointCloudIndex);


            /**
             * @brief Compute the translation coefficients between the point clouds
             * @param mostTrustworthyPointCloudIndex The index of the most trustworthy point cloud
             */
            void ComputeTranslationCoefficients(int mostTrustworthyPointCloudIndex);


            /**
             * @brief Getter for the rotation coefficient
             * @param i Index of the first point cloud (index of the row)
             * @param j Index of the second point cloud (index of the column)
             * @return Rotation coefficients
             */
            double GetRotationCoefficient(int i, int j)
            {
                return __rotationCoefficients(i, j);
            }


            /**
             * @brief Getter for the translation factor with rest
             * @param i Index of the first point cloud (index of the row)
             * @param j Index of the second point cloud (index of the column)
             * @return Translation factor with rest
             */
            std::pair<double, Eigen::Vector3d> GetTranslationFactorWithRest(int i, int j)
            {
                return __translationFactorsWithRests[i][j];
            }


            /**
             * @brief Getter for the rotation angles from the rotation coefficients 
             * @return A vector of the initial rotation angles
             */
            std::vector<double> GetInitialRotationAngles();


            /**
             * @brief Setter for the final translation vectors
             * @param index Index of the point cloud
             * @param finalTranslation Final translation vector to be applied to the point cloud
             */
            void SetFinalTranslation(int index, Eigen::Vector3d finalTranslation)
            {
                this->__finalTranslations[index] = finalTranslation;
            }


            /**
             * @brief Getter for the final translation vector of a given point cloud
             * @param index Index of the point cloud
             * @return The final translation vector
             */
            Eigen::Vector3d GetFinalTranslation(int index)
            {
                return this->__finalTranslations[index];
            }


            // Printers

            /**
             * @brief Print the mapping matrix to the console
             */
            void PrintMatrix();


            /**
             * @brief Print the mean transformation matrices to the console
             */
            void PrintMeanMatrices();


            // Algorithms
            
            /**
             * @brief Compute the Umeyama transformation between the current positions of the scans and their initial positions, for all the scans in the sub tree of the minimum spanning tree, starting from the given point cloud index
             * @param i Index of the point cloud
             * @return Eigen::Matrix4d Umeyama transformation matrix
             */
            Eigen::Matrix4d ComputeUmeyamaTransformationInSubtree(int i);


            // IO

            /**
             * @brief Save the poses of the point clouds to a CSV file. The file will contain the initial and final poses of each point cloud.
             * @param fileName Name of the file to save the poses to
             */
            void SavePosesToFile(std::string fileName);


        private:
            /**
             * @brief Mapping matrix between the point clouds, where each element is the transformation matrix between two point clouds
             */
            std::vector<std::vector<Referee::Mapping::Transformation>> __mappingMatrix;

            /**
             * @brief List of scans (point clouds with associated poses)
             */
            std::vector<Referee::Mapping::Scan> __scans;

            /**
             * @brief Mean transformation matrix per point cloud
             */
            std::vector<Referee::Mapping::Transformation> __meanTransformations;

            /**
             * @brief The matrix of the rotation coefficients. The rotation coefficients are by how much we should multiply the rotation angle of the transformation to get a compatible rotation wioth neighboring point clouds
             */
            Eigen::MatrixXd __rotationCoefficients;

            /**
             * @brief translation to be applied to the point cloud
             */
            std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> __translationFactorsWithRests;

            /**
             * @brief mean of the rotation angles of the transformation matrices
             */
            std::vector<double> __meanRotations;

            /**
             * @brief mean translation vectors for each point cloud
             */
            std::vector<Eigen::Vector3d> __meanTranslationVectors;

            /**
             * @brief standard deviation of the rotation angles of the transformation matrices
             */
            std::vector<double> __stdDevRotations;

            /**
             * @brief standard deviation of the translation vectors of the transformation matrices
             */
            std::vector<Eigen::Matrix3d> __covTranslationVectors;

            /**
             * @brief for each point cloud, the indices of the other point clouds with which transformations were computed
             */
            std::vector<std::vector<int>> __connectivityMatrix;

            /**
             * @brief Initial positions of the point clouds, used to compute the initial rotation angles
             */
            std::vector<Eigen::Vector3d> __initialPositions;

            /**
             * @brief For each point cloud we store the final translation vector that should be applied to the point cloud to get it in the global coordinate system
             */
            std::vector<Eigen::Vector3d> __finalTranslations;

            /**
             * @brief Graph to store the connectivity between the point clouds
             */
            Graph& __graph;
    };


    /**
     * @brief Create a connectivity matrix from a set of geolocations
     * @param geolocations Geolocations of the nodes
     * @param knn Number of nearest neighbors to consider
     * @param maxDistance Maximum distance to consider a connection
     * @return the connectivity matrix. Each element of the vector is the list of indices the respective geolocations are connected to
     */
    std::vector<std::vector<int>> CreateConnectivityMatrix(std::vector<Eigen::Vector3d> geolocations, int knn, double maxDistance);


    /**
     * @brief enum to store the different transformation calculation methods. Currently only GlobalMatch is supported: https://doi.org/10.1016/j.isprsjprs.2023.01.013 
    */
    enum TransformationComputationMethod
    {
        GlobalMatch,
    };


    /**
     * @brief enum to store the different refinement methods. Currently only ICP with normals is supported: https://doi.org/10.1109/ICRA40945.2019.8793880
    */
    enum RefinementMethod
    {
        ICPNormals,
        ICP,
    };


    /**
     * @brief Computes the 4x4 transformation matrix that transforms the source point cloud to the target point cloud following a given method. It also returns the score of the transformation, for example the number of correspondances used.
     * @param source the source point cloud used as reference
     * @param target the point cloud we want to transform
     * @param method the method used in the computation.  
    */
    std::pair<Eigen::Matrix4d, float> ComputePairwiseTransformation(pcl::PointCloud<pcl::PointNormal>::Ptr source, pcl::PointCloud<pcl::PointNormal>::Ptr target, TransformationComputationMethod method);


    /**
     * @brief Refines the pairwise transformation using a refinement method
     * @param source the source point cloud used as reference
     * @param target the point cloud we want to transform
     * @param method the method used in the computation. Currently only ICP with normals is supported
     * @param maxCorrespondenceDistance maximum correspondence distance for the ICP algorithm
     */
    Eigen::Matrix4d RefinePairwiseTransformation(pcl::PointCloud<pcl::PointNormal>::Ptr source, pcl::PointCloud<pcl::PointNormal>::Ptr target, RefinementMethod method, double maxCorrespondenceDistance);
}