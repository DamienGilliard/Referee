#include "../../Referee.hh"
#include <iostream>


int main()
{

    Referee::Mapping::Pose truePose0(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
    Referee::Mapping::Pose truePose1(Eigen::Vector3d(1, 0, 0), Eigen::Quaterniond::Identity());
    Referee::Mapping::Pose truePose2(Eigen::Vector3d(0, 1, 0), Eigen::Quaterniond::Identity());
    Referee::Mapping::Pose truePose3(Eigen::Vector3d(1, 1, 0), Eigen::Quaterniond::Identity());

    Eigen::Vector3d errorTranslation0(0.1, 0.1, 0.1);
    Eigen::Vector3d errorTranslation1(-0.1, 0.1, 0.0);
    Eigen::Vector3d errorTranslation2(0.1, -0.1, -0.1);
    Eigen::Vector3d errorTranslation3(-0.1, -0.1, 0.0);

    Eigen::Quaterniond errorOrientation0 = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond errorOrientation1 = Eigen::AngleAxisd(-0.0, Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond errorOrientation2 = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(-0.0, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond errorOrientation3 = Eigen::AngleAxisd(-0.0, Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(-0.0, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ());

    Referee::Mapping::Pose initPose0(truePose0.GetPosition() + errorTranslation0, errorOrientation0);
    Referee::Mapping::Pose initPose1(truePose1.GetPosition() + errorTranslation1, errorOrientation1);
    Referee::Mapping::Pose initPose2(truePose2.GetPosition() + errorTranslation2, errorOrientation2);
    Referee::Mapping::Pose initPose3(truePose3.GetPosition() + errorTranslation3, errorOrientation3);

    Eigen::Matrix4d errorTransform0 = initPose0.ToTransformationMatrix() * truePose0.ToTransformationMatrix().inverse();
    Eigen::Matrix4d errorTransform1 = initPose1.ToTransformationMatrix() * truePose1.ToTransformationMatrix().inverse();
    Eigen::Matrix4d errorTransform2 = initPose2.ToTransformationMatrix() * truePose2.ToTransformationMatrix().inverse();
    Eigen::Matrix4d errorTransform3 = initPose3.ToTransformationMatrix() * truePose3.ToTransformationMatrix().inverse();
    
    std::cout << "Error Transform 0:\n" << errorTransform0 << std::endl;
    std::cout << "Error Transform 1:\n" << errorTransform1 << std::endl;
    std::cout << "Error Transform 2:\n" << errorTransform2 << std::endl;
    std::cout << "Error Transform 3:\n" << errorTransform3 << std::endl;

    Eigen::Matrix4d measurement01 = errorTransform0.inverse() * errorTransform1;
    Eigen::Matrix4d measurement12 = errorTransform1.inverse() * errorTransform2;
    Eigen::Matrix4d measurement23 = errorTransform2.inverse() * errorTransform3;
    Eigen::Matrix4d measurement03 = errorTransform0.inverse() * errorTransform3;

    std::cout << "Measurement 0->1:\n" << measurement01 << std::endl;
    std::cout << "Measurement 1->2:\n" << measurement12 << std::endl;
    std::cout << "Measurement 2->3:\n" << measurement23 << std::endl;
    std::cout << "Measurement 0->3:\n" << measurement03 << std::endl;

    Referee::Mapping::Transformation transformation01(measurement01);
    Referee::Mapping::Transformation transformation12(measurement12);
    Referee::Mapping::Transformation transformation23(measurement23);
    Referee::Mapping::Transformation transformation03(measurement03);

    std::vector<Referee::Mapping::Pose> poses = {initPose0, initPose1, initPose2, initPose3};
    std::vector<Referee::Mapping::Scan> scans = {Referee::Mapping::Scan(poses[0], "dummy_path", 0.01), 
                                                 Referee::Mapping::Scan(poses[1], "dummy_path", 0.01), 
                                                 Referee::Mapping::Scan(poses[2], "dummy_path", 0.01), 
                                                 Referee::Mapping::Scan(poses[3], "dummy_path", 0.01)};

    std::vector<Eigen::Vector3d> initialTranslationVectors = {initPose0.GetPosition(), 
                                                              initPose1.GetPosition(), 
                                                              initPose2.GetPosition(), 
                                                              initPose3.GetPosition()};

    std::vector<std::vector<int>> connectivityMatrix = {{0, 1}, 
                                                        {1, 2}, 
                                                        {2, 3}, 
                                                        {0, 3}};
    Referee::Mapping::MappingMatrix mappingMatrix(4);
    mappingMatrix.SetConnectivityMatrix(connectivityMatrix);
    mappingMatrix.SetInitialPositions(initialTranslationVectors);
    mappingMatrix.SetScans(scans);
    mappingMatrix.SetTransformation(0, 1, transformation01);
    mappingMatrix.SetTransformation(1, 2, transformation12);
    mappingMatrix.SetTransformation(2, 3, transformation23);
    mappingMatrix.SetTransformation(0, 3, transformation03);

    mappingMatrix.ComputeGraph();

    mappingMatrix.GetGraph().SetWeight(0, 1, 1.0);
    mappingMatrix.GetGraph().SetWeight(1, 2, 1.0);
    mappingMatrix.GetGraph().SetWeight(2, 3, 1.0);
    mappingMatrix.GetGraph().SetWeight(0, 3, 1.0);
    
    mappingMatrix.GetGraph().ComputeMinimumSpanningTree(0);

    std::vector<std::pair<int, Referee::Mapping::Pose>> correctedPoses = mappingMatrix.ComputeLoopClosures();
    
    std::cout << "Corrected Poses:" << std::endl;
    Referee::Mapping::Pose correctedPose1;
    Referee::Mapping::Pose correctedPose3;
    for (const auto& pair : correctedPoses)
    {
        if (pair.first == 1)
        {
            correctedPose1 = pair.second;
        }
        else if (pair.first == 3)
        {
            correctedPose3 = pair.second;
        }
    }
    Eigen::Vector3d correctionTranslation0 = -1 * errorTranslation0;
    correctedPose1.Translate(correctionTranslation0);
    correctedPose3.Translate(correctionTranslation0);
    double relativeResidualTranslationErrorPose1 = (correctedPose1.GetPosition() - truePose1.GetPosition()).norm() / errorTranslation1.norm();
    double relativeResidualRotationErrorPose3 = correctedPose3.GetOrientation().angularDistance(truePose3.GetOrientation()) / errorOrientation3.angularDistance(Eigen::Quaterniond::Identity());
    if ( std::abs(relativeResidualTranslationErrorPose1) < 1e-1 && 
         std::abs(relativeResidualRotationErrorPose3) < 1e-1) // if 90% of the error has disappeared, we can say it is successful... I think
    {
        std::cout << "Loop closure optimization successful!" << std::endl;
    }
    else
    {
        std::cout << "Loop closure optimization failed." << std::endl;
        std::cout << "relative translation error: " << relativeResidualTranslationErrorPose1 << std::endl;
        std::cout << "relative rotation error: " << relativeResidualRotationErrorPose3 << std::endl;
        return 1;
    }
    return 0;
}