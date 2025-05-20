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
    std::vector<Eigen::Vector3d> initialTranslationVectors;
    for (const auto& file : geolocationFiles) 
    {
        Eigen::Vector3d translationVector = Referee::Utils::FileIterators::GetTranslationVectorsFromFile(file, coordSys);
        initialTranslationVectors.push_back(translationVector);
    }

    Referee::Mapping::MappingMatrix mappingMatrix(numberFiles);

    Eigen::Vector3d meanTranslation =Referee::Transformations::RecenterTranslationVectors(initialTranslationVectors);
    std::vector<std::vector<int>> matrix;
    Referee::Mapping::CreateConnectivityMatrix(initialTranslationVectors, 3, 30, matrix);
    mappingMatrix.SetConnectivityMatrix(matrix);

    int count = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredFinalPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<std::vector<Eigen::Vector3d>> translationVectorsPerPC(numberFiles, std::vector<Eigen::Vector3d>(numberFiles));

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
            translationVectorsPerPC[i][matrix[i][j]] = translation;
            Referee::Mapping::ChaslesTransformation chaslesTransformation(transformationMatrix);
            mappingMatrix.SetChaslesTransformation(i, matrix[i][j], chaslesTransformation);
            std::cout << "Chasles rotation center: " << chaslesTransformation.GetPointOnAxis() << std::endl;
        }
    }

    // mappingMatrix.PrintMatrix();
    mappingMatrix.CalculateMeanTransformationMatrices();
    mappingMatrix.PrintMeanMatrices();

    std::vector<std::tuple<int, double, double>> rotationsPerPC;
    for(int i = 0; i < numberFiles; i++)
    {
        rotationsPerPC.push_back(std::make_tuple(i, mappingMatrix.GetMeanRotation(i), mappingMatrix.GetStdDevRotation(i)));
    }

    std::sort(rotationsPerPC.begin(), rotationsPerPC.end(), [](const std::tuple<int, double, double>& a, const std::tuple<int, double, double>& b) {
        return std::get<2>(a) > std::get<2>(b);
    });

    mappingMatrix.ComputeRotationCoefficients(std::get<1>(rotationsPerPC[0]), std::get<0>(rotationsPerPC[0]));

    std::cout << "Probabilities of rotation angles: " << std::endl;
    for(int i = 0; i < rotationsPerPC.size(); i++)
    {
        std::cout << "For Point cloud " << std::get<0>(rotationsPerPC[i]) << std::endl;
        int indice = 0;
        double rotationCoefficient = mappingMatrix.GetRotationCoefficient(i, indice);
        if(rotationCoefficient == 0)
        {
            indice = 1;
            rotationCoefficient = mappingMatrix.GetRotationCoefficient(i, indice);
        }
        double p = Referee::Probability::Compute1DProbabilityDensityFunction(rotationCoefficient * mappingMatrix.GetChaslesTransformation(i, indice).GetRotationAngle(), std::get<1>(rotationsPerPC[i]), std::get<2>(rotationsPerPC[i]));
        std::cout << "Probability of rotation angle: " << p << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                 ROTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<double> initialRotationAngles;
    for(int i = 0; i < numberFiles; i++)
    {
        for (int j = 0; j < numberFiles; j++)
        {
            if(mappingMatrix.GetRotationCoefficient(i, j) != 0)
            {
                double angle = mappingMatrix.GetChaslesTransformation(i, j).GetRotationAngle() * mappingMatrix.GetRotationCoefficient(i, j);
                initialRotationAngles.push_back(angle);
                break;
            }
        }
        // double angle = mappingMatrix.GetChaslesTransformation(i, indice).GetRotationAngle();
        // if(angle == 0)
        // {
        //     indice = 1;
        //     angle = mappingMatrix.GetChaslesTransformation(i, indice).GetRotationAngle();
        // }
        // initiaRotationAngles.push_back(angle * mappingMatrix.GetRotationCoefficient(i, indice));
        std::cout << "Initial angle to be applied to point cloud " << i << ": " << initialRotationAngles[i] << std::endl;
    }
    std::cout << "Corrected angles to be applied: " << std::endl;
    std::vector<std::pair<double, double>> meansAndStdDevs;
    for (int i = 0; i < rotationsPerPC.size(); i++)
    {
        double mean = std::get<1>(rotationsPerPC[i]);
        double stdDev = std::get<2>(rotationsPerPC[i]);
        std::cout << "Mean and stdDev of rotation angles for point cloud " << std::get<0>(rotationsPerPC[i]) << ": " << mean << ", " << stdDev << std::endl;
        meansAndStdDevs.push_back(std::make_pair(mean, stdDev));
    }
    std::vector<double> correctedRotation = Referee::Probability::Compute1DGradienDescent(meansAndStdDevs, initialRotationAngles, 0.001, 1000, 0.00001);

    for(int i = 0; i < numberFiles; i++)
    {
        std::cout << "Corrected angle to be applied to point cloud " << i << ": " << correctedRotation[i] << std::endl;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                               TRANSLATION
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<std::pair<Eigen::Vector3d, double>> translationVectorsWithProbability;
    for(int i = 0; i < numberFiles; i++)
    {
        std::vector<Eigen::Vector3d> translationVectors= translationVectorsPerPC[i];
        for(int j = 0; j < translationVectors.size(); j++)
        {
            Eigen::Vector3d translationVector = translationVectors[j];
            Eigen::Vector3d rotationAxis = mappingMatrix.GetChaslesTransformation(i, j).GetRotationAxis();
            Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
            transformationMatrix.block<3, 3>(0, 0) = Eigen::AngleAxisd(correctedRotation[j], rotationAxis).toRotationMatrix();
            Eigen::Vector3d correctionPostRotation = Referee::Transformations::CalculateResultingTranslation(transformationMatrix, initialTranslationVectors[i] + translationVector - initialTranslationVectors[j]);
            translationVectors[j] = translationVector + correctionPostRotation;
        }
        Eigen::Vector3d meanTranslationVector = Eigen::Vector3d::Zero();
        for(int j = 0; j < translationVectors.size(); j++)
        {
            meanTranslationVector += translationVectors[j];
        }
        meanTranslationVector /= static_cast<double>(translationVectors.size());

        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        for(int j = 0; j < translationVectors.size(); j++)
        {
            Eigen::Vector3d translationVector = translationVectors[j];
            covarianceMatrix += (translationVector - meanTranslationVector) * (translationVector - meanTranslationVector).transpose();
        }
        covarianceMatrix /= static_cast<double>(translationVectors.size());
        double probability = Referee::Probability::Compute3DProbabilityDensityFunction(meanTranslationVector, meanTranslationVector, covarianceMatrix);
        translationVectorsWithProbability.push_back(std::make_pair(meanTranslationVector, probability));
    }

    int mostProbableIndex = 0;
    double maxProbability = 0;
    for(int i = 0; i < translationVectorsWithProbability.size(); i++)
    {
        if (i == mostProbableIndex)
        {
            continue;
        }
        if(translationVectorsWithProbability[i].second > maxProbability)
        {
            maxProbability = translationVectorsWithProbability[i].second;
            mostProbableIndex = i;
        }
    }

    std::cout << "Most probable translation vector: " << translationVectorsWithProbability[mostProbableIndex].first.transpose() << std::endl;
    std::cout << "Most probable index: " << mostProbableIndex << " With probability: " << maxProbability << std::endl;

    std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> translationFactorsWithRest(numberFiles, std::vector<std::pair<double, Eigen::Vector3d>>(numberFiles, std::make_pair(0.0, Eigen::Vector3d::Zero())));
    Eigen::Vector3d meanTranslationVector = translationVectorsWithProbability[mostProbableIndex].first;
    for (int j = 0; j < translationVectorsPerPC[mostProbableIndex].size(); j++)
    {
        if(mostProbableIndex == j)
        {
            translationVectorsPerPC[j][mostProbableIndex] = meanTranslationVector;
            translationFactorsWithRest[j][mostProbableIndex].first = 1.0;
            translationFactorsWithRest[j][mostProbableIndex].second = Eigen::Vector3d::Zero();
            continue;
        }
        std::cout << "Translation of pc " << mostProbableIndex << " With other pc: " << j << ": " << translationVectorsPerPC[mostProbableIndex][j].transpose() << std::endl;
        std::cout << "Mean translation vector: " << meanTranslationVector.transpose() << std::endl;
        Eigen::Vector3d normalizedTranslationVector = translationVectorsPerPC[mostProbableIndex][j].normalized();
        std::cout << "Normalized translation vector: " << normalizedTranslationVector.transpose() << std::endl;
        double projectionFactor = meanTranslationVector.dot(normalizedTranslationVector) / translationVectorsPerPC[mostProbableIndex][j].norm();
        std::cout << "Projection factor: " << projectionFactor << std::endl;
        Eigen::Vector3d projectionOfMeanVectorOnIndividualTranslationVector = projectionFactor * translationVectorsPerPC[mostProbableIndex][j];
        std::cout << "Projection of mean vector on individual translation vector: " << projectionOfMeanVectorOnIndividualTranslationVector.transpose() << std::endl;
        Eigen::Vector3d rest = meanTranslationVector - projectionOfMeanVectorOnIndividualTranslationVector;
        std::cout << "Rest: " << rest.transpose() << std::endl;
    
        translationFactorsWithRest[mostProbableIndex][j].first = projectionFactor;
        translationFactorsWithRest[mostProbableIndex][j].second = rest;
        translationFactorsWithRest[j][mostProbableIndex].first = 1.0 - projectionFactor;
        translationFactorsWithRest[j][mostProbableIndex].second = rest;
    }

    std::cout << "Translation factors with rest: " << std::endl;
    for(int i = 0; i < translationFactorsWithRest.size(); i++)
    {
        for(int j = 0; j < translationFactorsWithRest[i].size(); j++)
        {
            std::cout << "Translation factor " << i << " with other vector: " << j << ": " << translationFactorsWithRest[i][j].first << std::endl;
            std::cout << "Rest: " << translationFactorsWithRest[i][j].second.transpose() << std::endl;
        }
    }

    std::vector<Eigen::Vector3d> finalTranslationVectorsPerPc(numberFiles, Eigen::Vector3d::Zero());
    for(int i = 0; i < numberFiles; i++)
    {
        Eigen::Vector3d translation = (translationVectorsPerPC[i][mostProbableIndex]) * translationFactorsWithRest[i][mostProbableIndex].first + translationFactorsWithRest[i][mostProbableIndex].second;
        finalTranslationVectorsPerPc[i] = translation;
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
            coloredPointCloud->points[j].r = 50 + i*50;
            coloredPointCloud->points[j].g = 50 ;
            coloredPointCloud->points[j].b = 250 - i*50;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloudTransformed(new pcl::PointCloud<pcl::PointXYZRGB>);

        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rotationMatrix = Eigen::AngleAxis<double>(correctedRotation[i], Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
        transform.block<3, 3>(0, 0) = rotationMatrix;
        if(std::isinf(finalTranslationVectorsPerPc[i][0]))
        {
            transform.block<3, 1>(0, 3) = initialTranslationVectors[i] + finalTranslationVectorsPerPc[i];
        }
        else
        {
            transform.block<3, 1>(0, 3) = initialTranslationVectors[i];
        }
        transform.block<3, 1>(0, 3) = initialTranslationVectors[i] + finalTranslationVectorsPerPc[i];
        finalTransformations.push_back(transform);
        Referee::Transformations::TransformPointCloud<pcl::PointXYZRGB>(coloredPointCloud, transform);

        *coloredFinalPointCloud += *coloredPointCloud;
    }

    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix4d>> originsAndTransformations;
    for(int i = 0; i < numberFiles; i++)
    {
        Eigen::Vector3d initialTranslationVector = initialTranslationVectors[i];
        Eigen::Matrix4d transformation = finalTransformations[i];
        originsAndTransformations.push_back(std::make_pair(initialTranslationVector, transformation));
    }

    Referee::FileBasedVisualisation::Visualisation visualisation;
    visualisation.VisualiseTransformations(originsAndTransformations, coloredFinalPointCloud);
    pcl::io::savePLYFile("coloredFinalPointCloud.ply", *coloredFinalPointCloud);
    return 0;
}