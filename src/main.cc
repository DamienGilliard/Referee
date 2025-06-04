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

    int count = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredFinalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // std::vector<std::vector<Eigen::Vector3d>> translationVectorsPerPC(numberFiles, std::vector<Eigen::Vector3d>(numberFiles));

    // Calculate the different relative transformations
    for (int i = 0; i < matrix.size(); i++)
    {
        for (int j = 0; j < matrix[i].size(); j++)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr temporarySourcePointCloud(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr temporaryTargetPointCloud(new pcl::PointCloud<pcl::PointNormal>);
            std::cout << "Computing transformation between point cloud " << i << " and point cloud " << matrix[i][j] << std::endl;
            std::cout << "Loading point cloud " << i << std::endl;
            if (pcl::io::loadPLYFile<pcl::PointNormal>(plyFileNames[i], *temporarySourcePointCloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s \n", plyFileNames[i].c_str());
                return 0;
            }
            std::cout << "Loading point cloud " << matrix[i][j] << std::endl;
            if (pcl::io::loadPLYFile<pcl::PointNormal>(plyFileNames[matrix[i][j]], *temporaryTargetPointCloud) == -1)
            {
                PCL_ERROR("Couldn't read file %s \n", plyFileNames[matrix[i][j]].c_str());
                return 0;
            }
            std::cout << "Cropping point cloud " << i << std::endl;
            Referee::Utils::Filtering::CropPointCloud<pcl::PointNormal>(temporaryTargetPointCloud, -40, -40, -100, 40, 40, 8000);
            std::cout << "Downsampling point cloud " << i << std::endl;
            Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(temporaryTargetPointCloud, 0.05);
            std::cout << "Cropping point cloud " << matrix[i][j] << std::endl;
            Referee::Utils::Filtering::CropPointCloud<pcl::PointNormal>(temporarySourcePointCloud, -40, -40, -100, 40, 40, 8000);
            std::cout << "Downsampling point cloud " << matrix[i][j] << std::endl;
            Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointNormal>(temporarySourcePointCloud, 0.05);
            Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(temporarySourcePointCloud, initialTranslationVectors[i]);
            Referee::Transformations::TranslatePointCloud<pcl::PointNormal>(temporaryTargetPointCloud, initialTranslationVectors[matrix[i][j]]);
            Eigen::Matrix4d transformationMatrix = Referee::Mapping::ComputePairwiseTransformation(temporarySourcePointCloud, temporaryTargetPointCloud, Referee::Mapping::TransformationComputationMethod::GlobalMatch);
            std::cout << "Transformation matrix: " << transformationMatrix << std::endl;
            Eigen::Vector3d translation = Referee::Transformations::CalculateResultingTranslation(transformationMatrix, initialTranslationVectors[i]);
            // translationVectorsPerPC[i][matrix[i][j]] = translation;
            Referee::Mapping::Transformation Transformation(transformationMatrix);
            mappingMatrix.SetTransformation(i, matrix[i][j], Transformation);
        }
    }

    // mappingMatrix.PrintMatrix();
    mappingMatrix.CalculateMeanTransformationMatrices();
    mappingMatrix.PrintMeanMatrices();

    std::tuple<int, double> mostProbableRotation = mappingMatrix.GetMostProbableRotation();
    int mostProbableIndexRotation = std::get<0>(mostProbableRotation);
    mappingMatrix.ComputeRotationCoefficients(mostProbableIndexRotation);
    std::vector<std::pair<double, double>> meansAndStdDevs = mappingMatrix.GetMeanRotationsAndStdDevs();
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    //                                                 ROTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<double> initialRotationAngles = mappingMatrix.GetInitialRotationAngles();
    for(int i = 0; i < initialRotationAngles.size(); i++)
    {
        std::cout << "Initial angle to be applied to point cloud " << i << ": " << initialRotationAngles[i] << std::endl;
    }

    std::vector<double> correctedAngles = Referee::Probability::Compute1DGradienDescent(meansAndStdDevs, initialRotationAngles, 0.00001, 10000, 0.000001);
    for(int i = 0; i < numberFiles; i++)
    {
        std::cout << "Corrected angle to be applied to point cloud " << i << ": " << correctedAngles[i] << std::endl;
    }

    Referee::Utils::IO::SaveRotationAnglesAndStdDevs("rotation_angles_and_std_devs.txt", meansAndStdDevs, correctedAngles);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                               TRANSLATION
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // std::vector<std::pair<Eigen::Vector3d, double>> translationVectorsWithProbability;
    // for(int i = 0; i < numberFiles; i++)
    // {
    //     std::vector<Eigen::Vector3d> translationVectors= translationVectorsPerPC[i];
    //     for(int j = 0; j < translationVectors.size(); j++)
    //     {
    //         Eigen::Vector3d translationVector = translationVectors[j];
    //         Eigen::Vector3d rotationAxis = mappingMatrix.GetTransformation(i, j).GetRotationVector();
    //         Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    //         transformationMatrix.block<3, 3>(0, 0) = Eigen::AngleAxisd(correctedAngles[j], rotationAxis).toRotationMatrix();
    //         Eigen::Vector3d correctionPostRotation = Referee::Transformations::CalculateResultingTranslation(transformationMatrix, initialTranslationVectors[i] + translationVector - initialTranslationVectors[j]);
    //         translationVectors[j] = translationVector + correctionPostRotation;
    //     }
    //     Eigen::Vector3d meanTranslationVector = Eigen::Vector3d::Zero();
    //     for(int j = 0; j < translationVectors.size(); j++)
    //     {
    //         meanTranslationVector += translationVectors[j];
    //     }
    //     meanTranslationVector /= static_cast<double>(translationVectors.size());

    //     Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
    //     for(int j = 0; j < translationVectors.size(); j++)
    //     {
    //         Eigen::Vector3d translationVector = translationVectors[j];
    //         covarianceMatrix += (translationVector - meanTranslationVector) * (translationVector - meanTranslationVector).transpose();
    //     }
    //     covarianceMatrix /= static_cast<double>(translationVectors.size());
    //     double probability = Referee::Probability::Compute3DProbabilityDensityFunction(meanTranslationVector, meanTranslationVector, covarianceMatrix);
    //     translationVectorsWithProbability.push_back(std::make_pair(meanTranslationVector, probability));
    // }

    // int mostProbableIndex = 0;
    // double maxProbability = 0;
    // for(int i = 0; i < translationVectorsWithProbability.size(); i++)
    // {
    //     if (i == mostProbableIndex)
    //     {
    //         continue;
    //     }
    //     if(translationVectorsWithProbability[i].second > maxProbability)
    //     {
    //         maxProbability = translationVectorsWithProbability[i].second;
    //         mostProbableIndex = i;
    //     }
    // }


    // std::cout << "Most probable translation vector: " << translationVectorsWithProbability[mostProbableIndex].first.transpose() << std::endl;
    // std::cout << "Most probable index: " << mostProbableIndex << " With probability: " << maxProbability << std::endl;

    std::pair<int, double> mostProbableTranslation = mappingMatrix.GetMostProbableTranslation();
    int mostProbableIndex = mostProbableTranslation.first;
    mappingMatrix.ComputeTranslationCoefficients(mostProbableIndex);
    std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> translationFactorsWithRest(numberFiles, std::vector<std::pair<double, Eigen::Vector3d>>(numberFiles, std::make_pair(0.0, Eigen::Vector3d::Zero())));
    Eigen::Vector3d meanTranslationVector = mappingMatrix.GetMeanTranslationVector(mostProbableIndex);
    // for (int j = 0; j < translationVectorsPerPC[mostProbableIndex].size(); j++)
    // {
    //     if(mostProbableIndex == j)
    //     {
    //         translationVectorsPerPC[j][mostProbableIndex] = meanTranslationVector;
    //         translationFactorsWithRest[j][mostProbableIndex].first = 1.0;
    //         translationFactorsWithRest[j][mostProbableIndex].second = Eigen::Vector3d::Zero();
    //         continue;
    //     }
    //     std::cout << "Translation of pc " << mostProbableIndex << " With other pc: " << j << ": " << translationVectorsPerPC[mostProbableIndex][j].transpose() << std::endl;
    //     std::cout << "Mean translation vector: " << meanTranslationVector.transpose() << std::endl;
    //     Eigen::Vector3d normalizedTranslationVector = translationVectorsPerPC[mostProbableIndex][j].normalized();
    //     std::cout << "Normalized translation vector: " << normalizedTranslationVector.transpose() << std::endl;
    //     double projectionFactor = meanTranslationVector.dot(normalizedTranslationVector) / translationVectorsPerPC[mostProbableIndex][j].norm();
    //     std::cout << "Projection factor: " << projectionFactor << std::endl;
    //     Eigen::Vector3d projectionOfMeanVectorOnIndividualTranslationVector = projectionFactor * translationVectorsPerPC[mostProbableIndex][j];
    //     std::cout << "Projection of mean vector on individual translation vector: " << projectionOfMeanVectorOnIndividualTranslationVector.transpose() << std::endl;
    //     Eigen::Vector3d rest = meanTranslationVector - projectionOfMeanVectorOnIndividualTranslationVector;
    //     std::cout << "Rest: " << rest.transpose() << std::endl;
    
    //     translationFactorsWithRest[mostProbableIndex][j].first = projectionFactor;
    //     translationFactorsWithRest[mostProbableIndex][j].second = rest;
    //     translationFactorsWithRest[j][mostProbableIndex].first = 1.0 - projectionFactor;
    //     translationFactorsWithRest[j][mostProbableIndex].second = rest;
    // }

    // std::cout << "Translation factors with rest: " << std::endl;
    // for(int i = 0; i < numberFiles; i++)
    // {
    //     for(int j = 0; j < numberFiles; j++)
    //     {
    //         std::cout << "Translation factor " << i << " with other vector: " << j << ": " << mappingMatrix.GetTranslationFactorWithRest(i, j).first << std::endl;
    //         std::cout << "Rest: " << mappingMatrix.GetTranslationFactorWithRest(i, j).second.transpose() << std::endl;
    //     }
    // }

    std::vector<Eigen::Vector3d> finalTranslationVectorsPerPc(numberFiles, Eigen::Vector3d::Zero());
    for(int i = 0; i < numberFiles; i++)
    {
        finalTranslationVectorsPerPc[i] = (mappingMatrix.GetTransformation(i, mostProbableIndex).GetTranslation()) * mappingMatrix.GetTranslationFactorWithRest(i, mostProbableIndex).first + mappingMatrix.GetTranslationFactorWithRest(i, mostProbableIndex).second;        
        std::cout << "Final translation vector for point cloud " << i << ": " << finalTranslationVectorsPerPc[i].transpose() << std::endl;
    }

    std::vector<Eigen::Matrix4d> finalTransformations;
    for(int i = 0; i < numberFiles; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPLYFile(plyFileNames[i], *coloredPointCloud);
        // Referee::Utils::Filtering::CropPointCloud<pcl::PointXYZRGB>(coloredPointCloud, -40, -40, -100, 40, 40, 8000);
        Referee::Utils::Filtering::VoxelizePointCloud<pcl::PointXYZRGB>(coloredPointCloud, 0.2f);
        for(int j = 0; j < coloredPointCloud->size(); j++)
        {
            coloredPointCloud->points[j].r = 150 + i*25;
            coloredPointCloud->points[j].g = 150 ;
            coloredPointCloud->points[j].b = 250 - i*25;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloudTransformed(new pcl::PointCloud<pcl::PointXYZRGB>);

        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rotationMatrix = Eigen::AngleAxis<double>(correctedAngles[i] + 0.02, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
        transform.block<3, 3>(0, 0) = rotationMatrix;
        // if(std::isinf(finalTranslationVectorsPerPc[i][0]))
        // {
        //     transform.block<3, 1>(0, 3) = initialTranslationVectors[i] + finalTranslationVectorsPerPc[i];
        // }
        // else
        // {
        //     transform.block<3, 1>(0, 3) = initialTranslationVectors[i];
        // }
        transform.block<3, 1>(0, 3) = initialTranslationVectors[i] + finalTranslationVectorsPerPc[i];
        finalTransformations.push_back(transform);
        Referee::Transformations::TransformPointCloud<pcl::PointXYZRGB>(coloredPointCloud, transform);

        *coloredFinalPointCloud += *coloredPointCloud;
    }

    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix4d>> originsAndTransformations;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> originsAndTranslations;
    for(int i = 0; i < numberFiles; i++)
    {
        Eigen::Vector3d initialTranslationVector = initialTranslationVectors[i];
        Eigen::Matrix4d transformation = finalTransformations[i];
        originsAndTransformations.push_back(std::make_pair(initialTranslationVector, transformation));
        originsAndTranslations.push_back(std::make_pair(initialTranslationVector, transformation.block<3, 1>(0, 3)));
    }

    double angle = Referee::Transformations::CalculateResultingRotationAngle(originsAndTranslations);

    Referee::FileBasedVisualisation::Visualisation visualisation;
    visualisation.VisualiseTransformations(originsAndTransformations, coloredFinalPointCloud);
    pcl::io::savePLYFile("coloredFinalPointCloud.ply", *coloredFinalPointCloud);
    return 0;
}