#include "Referee.hh"
#include <Eigen/Geometry>

int main()
{
    std::vector<std::string> geolocationFiles = Referee::Utils::FileIterators::GetFilesInDirectory("../../test_files/geolocations", ".petitpoucet");
    std::vector<std::string> plyFileNames = Referee::Utils::FileIterators::GetFilesInDirectory("../../test_files/scans", ".ply");
    int numberFiles = plyFileNames.size();

    if (numberFiles != geolocationFiles.size()) 
    {
        std::cerr << "Number of geolocation files and ply files do not match" << std::endl;
        return 1;
    }

    Referee::Utils::CoordinateSystem::CoordinateSystem coordSys = Referee::Utils::CoordinateSystem::CoordinateSystem::LV95;
    std::vector<Eigen::Vector3d> initialTranslationVectors = Referee::Utils::FileIterators::GetTranslationVectorsFromFiles(geolocationFiles, coordSys);

    Referee::Mapping::MappingMatrix mappingMatrix(numberFiles);

    Eigen::Vector3d meanTranslation =Referee::Transformations::RecenterTranslationVectors(initialTranslationVectors);
    std::vector<std::vector<int>> matrix;
    Referee::Mapping::CreateConnectivityMatrix(initialTranslationVectors, 3, 30, matrix);
    mappingMatrix.SetConnectivityMatrix(matrix);
    mappingMatrix.SetInitialPositions(initialTranslationVectors);

    Referee::Mapping::Graph graph = Referee::Mapping::Graph::CreateUndirectedGraph();
    for (const auto& initialTranslationVector : initialTranslationVectors)
    {
        Eigen::Vector3d vertex = initialTranslationVector;
        graph.AddVertex(vertex);
    }

    for (int i = 0; i < matrix.size(); i++)
    {
        for (int neighbor : matrix[i])
        {
            for (int candidateOpposite : matrix[neighbor])
            {
                    Eigen::Vector3d vertex = initialTranslationVectors[i];
                    Eigen::Vector3d neighborVertex = initialTranslationVectors[neighbor];
                    graph.AddEdge(vertex, neighborVertex, (initialTranslationVectors[i] - initialTranslationVectors[neighbor]).norm());
                    break;
            }
        }
    }

    graph.PrintGraph();

    std::vector<std::vector<Eigen::Vector3d>> computedTranslations;
    computedTranslations.resize(numberFiles);

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredFinalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < matrix.size(); i++)
    {
        for (int j = 0; j < matrix[i].size(); j++)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr temporarySourcePointCloud(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr temporaryTargetPointCloud(new pcl::PointCloud<pcl::PointNormal>);
            std::cout << "Computing transformation between point cloud " << i << " and point cloud " << matrix[i][j] << std::endl;
            if (pcl::io::loadPLYFile<pcl::PointNormal>(plyFileNames[i], *temporarySourcePointCloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s \n", plyFileNames[i].c_str());
                return 0;
            }
            if (pcl::io::loadPLYFile<pcl::PointNormal>(plyFileNames[matrix[i][j]], *temporaryTargetPointCloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s \n", plyFileNames[matrix[i][j]].c_str());
                return 0;
            }
            Referee::Utils::Filtering::CropPointCloud<pcl::PointNormal>(temporaryTargetPointCloud, -30, -30, -100, 30, 30, 8000);
            Referee::Utils::Filtering::CropPointCloud<pcl::PointNormal>(temporarySourcePointCloud, -30, -30, -100, 30, 30, 8000);
            Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(temporaryTargetPointCloud, 0.05);
            Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(temporarySourcePointCloud, 0.05);
            Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(temporarySourcePointCloud, initialTranslationVectors[i]);
            Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(temporaryTargetPointCloud, initialTranslationVectors[matrix[i][j]]);
            Eigen::Matrix4d transformationMatrix = Referee::Mapping::ComputePairwiseTransformation(temporarySourcePointCloud, temporaryTargetPointCloud, Referee::Mapping::TransformationComputationMethod::GlobalMatch);
            Eigen::Vector3d translation = Referee::Transformations::CalculateResultingTranslation(transformationMatrix, initialTranslationVectors[i]);
            Referee::Mapping::Transformation Transformation(transformationMatrix);
            mappingMatrix.SetTransformation(i, matrix[i][j], Transformation);
        }
    }
    mappingMatrix.CalculateMeanTransformationMatrices();
    mappingMatrix.PrintMeanMatrices();

    std::tuple<int, double> mostProbableRotation = mappingMatrix.GetMostProbableRotation();
    int mostProbableIndexRotation = std::get<0>(mostProbableRotation);
    mappingMatrix.ComputeRotationCoefficients(mostProbableIndexRotation);
    std::vector<std::pair<double, double>> meansAndStdDevs = mappingMatrix.GetMeanRotationsAndStdDevs();
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    //                                                 ROTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double meanTranslationInducedRotation = mappingMatrix.ComputeMeanTranslationInducedRotation();
    std::cout << "Mean translation induced rotation: " << meanTranslationInducedRotation << std::endl;
    std::vector<double> initialRotationAngles = mappingMatrix.GetInitialRotationAngles();
    std::vector<double> correctedAngles = Referee::Probability::Compute1DGradienDescent(meansAndStdDevs, initialRotationAngles, 0.00001, 10000, 0.000001);
    for(int i = 0; i < initialRotationAngles.size(); i++)
    {
        std::cout << "Initial angle to be applied to point cloud " << i << ": " << initialRotationAngles[i] << std::endl;
        std::cout << "Corrected angle to be applied to point cloud " << i << ": " << correctedAngles[i] << std::endl;
    }

    Referee::Utils::IO::SaveRotationAnglesAndStdDevs("rotation_angles_and_std_devs.txt", meansAndStdDevs, correctedAngles);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                               TRANSLATION
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // std::pair<int, double> mostProbableTranslation = mappingMatrix.GetMostProbableTranslation();
    int mostProbableIndex = 0;
    Eigen::Vector3d mostProbableTranslationVector = mappingMatrix.GetMeanTranslationVector(0);

    for (int i = 1; i < mappingMatrix.GetConnectivityMatrix().size(); i++)
    {
        Eigen::Vector3d translation = mappingMatrix.GetMeanTranslationVector(i);
        if (translation.norm() < mostProbableTranslationVector.norm())
        {
            mostProbableIndex = i;
            mostProbableTranslationVector = translation;
        }
    }

    mappingMatrix.ComputeTranslationCoefficients(mostProbableIndex);
    std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> translationFactorsWithRest(numberFiles, std::vector<std::pair<double, Eigen::Vector3d>>(numberFiles, std::make_pair(0.0, Eigen::Vector3d::Zero())));
    Eigen::Vector3d meanTranslationVector = mappingMatrix.GetMeanTranslationVector(mostProbableIndex);

    std::vector<Eigen::Vector3d> finalTranslationVectorsPerPc(numberFiles, Eigen::Vector3d::Zero());
    std::vector<Eigen::Matrix4d> finalTransformations;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix4d>> originsAndTransformations;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> originsAndTranslations;

    for(int i = 0; i < numberFiles; i++)
    {
        finalTranslationVectorsPerPc[i] = (mappingMatrix.GetTransformation(i, mostProbableIndex).GetTranslation()) * mappingMatrix.GetTranslationFactorWithRest(i, mostProbableIndex).first + mappingMatrix.GetTranslationFactorWithRest(i, mostProbableIndex).second;        
        std::cout << "Final translation vector for point cloud " << i << ": " << finalTranslationVectorsPerPc[i].transpose() << std::endl;
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rotationMatrix = Eigen::AngleAxis<double>(correctedAngles[i], Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
        transform.block<3, 3>(0, 0) = rotationMatrix;
        transform.block<3, 1>(0, 3) = /*initialTranslationVectors[i] + */finalTranslationVectorsPerPc[i];
        finalTransformations.push_back(transform);
        Eigen::Vector3d initialTranslationVector = initialTranslationVectors[i];
        Eigen::Matrix4d transformation = finalTransformations[i];
        originsAndTransformations.push_back(std::make_pair(initialTranslationVector, transformation));
        originsAndTranslations.push_back(std::make_pair(initialTranslationVector, transformation.block<3, 1>(0, 3)));
    }
    double finalAngle = Referee::Transformations::CalculateResultingRotationAngle(originsAndTranslations);
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                               OUTPUTS
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for(int i = 0; i < numberFiles; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPLYFile(plyFileNames[i], *coloredPointCloud);
        Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointXYZRGB>(coloredPointCloud, 0.03);
        for(int j = 0; j < coloredPointCloud->size(); j++)
        {
            coloredPointCloud->points[j].r = 100 + i*45;
            coloredPointCloud->points[j].g = 100 ;
            coloredPointCloud->points[j].b = 250 - i*45;
        }
        Referee::Transformations::TranslatePointCloud<pcl::PointXYZRGB>(coloredPointCloud, initialTranslationVectors[i]);
        Referee::Transformations::TransformPointCloud<pcl::PointXYZRGB>(coloredPointCloud, finalTransformations[i]);
        *coloredFinalPointCloud += *coloredPointCloud;
    }

    Eigen::Matrix4d finalRotation = Eigen::Matrix4d::Identity();
    finalRotation.block<3, 3>(0, 0) = Eigen::AngleAxisd(-finalAngle, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Referee::Transformations::TransformPointCloud<pcl::PointXYZRGB>(coloredFinalPointCloud, finalRotation);

    // rasterize
    Referee::Raster::Rasterizer rasterizer;
    rasterizer.CreateGeoTIFFDEMFromPointCloud(coloredFinalPointCloud, "output.tif", 1.0, coordSys);
    Referee::FileBasedVisualisation::Visualisation visualisation;
    visualisation.VisualiseTransformations(originsAndTransformations, coloredFinalPointCloud);
    pcl::io::savePLYFile("coloredFinalPointCloud.ply", *coloredFinalPointCloud);
    return 0;
}