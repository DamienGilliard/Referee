#include "Visualisation.hh"

namespace Referee::FileBasedVisualisation
{
    void Visualisation::VisualisePointCloud(const std::string& plyFilePath)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(plyFilePath, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", plyFilePath.c_str());
            return;
        }
        std::cout << "Loaded "
                  << cloud->width * cloud->height
                  << " data points from " << plyFilePath << std::endl;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);

        pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
        viewer.setBackgroundColor(0.0, 0.0, 0.0);
        viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
        viewer.setCameraPosition(
            centroid[0], centroid[1], centroid[2] + 10.0,
            centroid[0], centroid[1], centroid[2],
            0.0, -1.0, 0.0);

        std::cout << "Press q to exit" << std::endl;
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    void Visualisation::VisualisePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);

        pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
        viewer.setBackgroundColor(0.0, 0.0, 0.0);
        viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
        viewer.setCameraPosition(
            -centroid[0], -centroid[1], centroid[2] + 10.0,
            -centroid[0], -centroid[1], centroid[2],
            0.0, -1.0, 0.0);
        viewer.setCameraFieldOfView(0.523599); // 30 degrees field of view

        std::cout << "Centroid: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
        std::cout << "Press q to exit" << std::endl;
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    void Visualisation::ExportImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, const std::string& imagePath)
    {
        pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
        viewer.addPointCloud<pcl::PointXYZRGB>(pointCloud, "sample cloud");
        viewer.addArrow<pcl::PointXYZ>(pcl::PointXYZ(0, 0, 805), pcl::PointXYZ(2, 0, 805), 1.0, 0.0, 0.0, false, "arrow");
        viewer.setCameraPosition(0, 0, 900, 0, 0, 0, 0, 1, 0);
        viewer.setSize(4000, 3000);
        viewer.setBackgroundColor(1.0, 1.0, 1.0);
        viewer.getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
        viewer.getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelScale(30);
        viewer.spinOnce(100);
        viewer.saveScreenshot(imagePath);
    }
    void Visualisation::VisualiseTransformations(std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix4d>> originsAndTransformations, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, const std::string& imageFileName)
    {
        pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
        viewer.setBackgroundColor(0.0, 0.0, 0.0);
        viewer.addPointCloud<pcl::PointXYZRGB>(pointCloud, "sample cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer.addCoordinateSystem(5.0);
        viewer.setSize(3000, 2250);
        viewer.initCameraParameters();
        viewer.setCameraPosition(0, 0, 900, 0, 0, 0, 0, 1, 0);
        viewer.setBackgroundColor(1.0, 1.0, 1.0);
        viewer.getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
        viewer.getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelScale(50);

        int count = 0;
        for (const auto& originAndTransformation : originsAndTransformations)
        {
            Eigen::Vector3d initialOrigin = originAndTransformation.first;
            pcl::PointXYZ point(initialOrigin.x(), initialOrigin.y(), initialOrigin.z());
            std::vector<pcl::PointXYZ> axesEnds = Referee::FileBasedVisualisation::CreateCoordinateSystem(initialOrigin, 4.0);
            std::vector<pcl::PointXYZ> transformedAxesEnds(originsAndTransformations.size());
            Eigen::Vector3d transformedOrigin = originAndTransformation.second.block<3, 3>(0, 0) * initialOrigin + originAndTransformation.second.block<3, 1>(0, 3) - originAndTransformation.first;
            pcl::PointXYZ transformedPoint(transformedOrigin.x(), transformedOrigin.y(), transformedOrigin.z());
            for (int i = 0; i < axesEnds.size(); i++)
            {
                Eigen::Vector3d axisEnd(axesEnds[i].x, axesEnds[i].y, axesEnds[i].z);
                Eigen::Vector3d transformedAxisEnd = originAndTransformation.second.block<3, 3>(0, 0) * axisEnd + originAndTransformation.second.block<3, 1>(0, 3) - originAndTransformation.first;
                transformedAxesEnds[i] = pcl::PointXYZ(transformedAxisEnd.x(), transformedAxisEnd.y(), transformedAxisEnd.z());
            }

            viewer.addArrow<pcl::PointXYZ>(axesEnds[0], point, 0.5f, 0.0f, 0.0f, false, "arrowX" + std::to_string(count));
            viewer.addArrow<pcl::PointXYZ>(axesEnds[1], point, 0.0f, 0.5f, 0.0f, false, "arrowY" + std::to_string(count));
            viewer.addArrow<pcl::PointXYZ>(transformedAxesEnds[0], transformedPoint, 1.0f, 0.0f, 0.0f, false, "arrowXTransformed" + std::to_string(count));
            viewer.addArrow<pcl::PointXYZ>(transformedAxesEnds[1], transformedPoint, 0.0f, 1.0f, 0.0f, false, "arrowYTransformed" + std::to_string(count));
            viewer.addArrow<pcl::PointXYZ>(transformedPoint, point, 0.0f, 0.0f, 1.0f, false, "translation" + std::to_string(count));
            count++;
        }

        // viewer.spinOnce(100);
        viewer.saveScreenshot(imageFileName);
    }
    
    std::vector<pcl::PointXYZ> CreateCoordinateSystem(Eigen::Vector3d origin, double axisLength)
    {
        std::vector<pcl::PointXYZ> coordinates;
        coordinates.push_back(pcl::PointXYZ(origin.x() + axisLength, origin.y(), origin.z()));
        coordinates.push_back(pcl::PointXYZ(origin.x(), origin.y() + axisLength, origin.z()));
        coordinates.push_back(pcl::PointXYZ(origin.x(), origin.y(), origin.z() + axisLength));
        return coordinates;
    }
} // FileBasedVisualisation