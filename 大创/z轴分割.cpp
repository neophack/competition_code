//
// Created by ethan on 18-6-26.
//
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <liblas/liblas.hpp>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <iostream>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <fstream>
#include <iosfwd>
#include <math.h>
#include <iomanip>
#include"stdlib.h"
#include "iostream"
#include "string"
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <liblas/liblas.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
typedef pcl::PointXYZI Point;

using namespace std;
void viewer(string name,pcl::PointCloud<Point>::Ptr pointCloudPtr,string type_)
{
    pcl::visualization::PCLVisualizer p (name);
    p.setBackgroundColor (1, 1, 1);

    std::cerr << "PointCloudColorHandlerGenericField demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerGenericField<Point> handler_z (pointCloudPtr, type_);

    p.addPointCloud (pointCloudPtr, handler_z, "cloud_z");
    p.spin ();
    p.removePointCloud ("cloud_z");


    //p.addCoordinateSystem (0.1);

    // Demonstrate usage of spinOnce()
    p.resetStoppedFlag();
    p.removePointCloud ("cloud");
    p.spin ();
    std::cerr << "ok" << std::endl;
}
int main(int argc,char **argv)
{

    pcl::PointCloud<Point>::Ptr pointCloudPtr (new pcl::PointCloud<Point>);
    //pcl::PointCloud<Point>::Ptr pointCloudPtr_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr_xyz (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointCloud<Point>::Ptr pointCloudPtr_o (new pcl::PointCloud<Point>);

        pcl::PCDReader pcd;
        if (pcd.read ("123.pcd", *pointCloudPtr) == -1)
            return (-1);
        viewer("1",pointCloudPtr,"intensity");
        pointCloudPtr_o->points.resize(pointCloudPtr->size());

    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::copyPointCloud(*pointCloudPtr, *pointCloudPtr_xyz);

    pcl::getMinMax3D(*pointCloudPtr_xyz,min,max);

    pcl::PassThrough<Point> pass;
    pass.setInputCloud (pointCloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits ((max.z-min.z)*0.2, max.z);
    //pass.setFilterLimitsNegative (true);true输出滤去部分
    pass.filter (*pointCloudPtr_o);
    viewer("3",pointCloudPtr_o,"intensity");
    pcl::io::savePCDFileASCII("qie1.pcd",*pointCloudPtr_o);

    cout<<"min.z = "<<min.z<<"\n"<<endl;
    cout<<"max.z = "<<max.z<<"\n"<<endl;
    cout<<"min.x = "<<min.x<<"\n"<<endl;
    cout<<"max.x = "<<max.x<<"\n"<<endl;
    cout<<"min.y = "<<min.y<<"\n"<<endl;
    cout<<"max.y = "<<max.y<<"\n"<<endl;
return 0;
}

