#include "Visualisation.hh"

namespace FileBasedVisualisation
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
} // FileBasedVisualisation