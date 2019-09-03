#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>

/*boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    return viewer;
}*/


//构造球体点云
void ConsSphereCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr)
{
    float radius = 1;
    for(int k=0;k<basic_cloud_in->points.size();k++){
    for (float angle1 = 0.0; angle1 <= 180.0; angle1 += 45.0)
    {
        for (float angle2 = 0.0; angle2 <= 360.0; angle2 += 45.0)
        {
            pcl::PointXYZRGB basic_point;
            pcl::PointXYZRGB basic_point1;

            basic_point1.x=basic_cloud_in->points[k].x;
            basic_point1.y=basic_cloud_in->points[k].y;
            basic_point1.z=basic_cloud_in->points[k].z;

            basic_point1.r=255;
            basic_point1.g=1;
            basic_point1.b=1;

            basic_point.x = basic_cloud_in->points[k].x+radius * sinf(pcl::deg2rad(angle1)) * cosf(pcl::deg2rad(angle2));
            basic_point.y = basic_cloud_in->points[k].y+radius * sinf(pcl::deg2rad(angle1)) * sinf(pcl::deg2rad(angle2));
            basic_point.z = basic_cloud_in->points[k].z+radius * cosf(pcl::deg2rad(angle1));
            basic_point.r=255;
            basic_point.g=255;
            basic_point.b=1;
            basic_cloud_ptr->points.push_back(basic_point);
            basic_cloud_ptr->points.push_back(basic_point1);

        }
    }

  }
    basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
    basic_cloud_ptr->height = 1;
}

int main(int _Argc, char *argv[])
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PCDReader pcd;
    if (pcd.read ("test.pcd", *pointCloudPtr) == -1)
      return (-1);
    ConsSphereCloud(pointCloudPtr,basic_cloud_ptr);
    pcl::io::savePCDFileASCII("point.pcd",*basic_cloud_ptr);

 /*   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(basic_cloud_ptr);
    while ( !viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }*/
    return 0;
}

