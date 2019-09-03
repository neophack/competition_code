#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
PointMatcher<float>::DataPoints PcdtoPointMatcherCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud)
{
   PointMatcher<float>::DataPoints::Labels featLabels,descLabels;
   const size_t count=source_cloud->size();
   featLabels.push_back(PointMatcher<float>::DataPoints::Label("x", 1));
   featLabels.push_back(PointMatcher<float>::DataPoints::Label("y", 1));
   featLabels.push_back(PointMatcher<float>::DataPoints::Label("z", 1));
   featLabels.push_back(PointMatcher<float>::DataPoints::Label("pad", 1));
   // create cloud
   const unsigned pointCount(count);
   PointMatcher<float>::DataPoints cloud(featLabels, descLabels, pointCount);
   cloud.getFeatureViewByName("pad").setConstant(1);
   for(size_t i=0;i<source_cloud->size();i++)
   {
   cloud.features(0,i) = source_cloud->points[i].x;
   cloud.features(1,i) = source_cloud->points[i].y;
   cloud.features(2,i) = source_cloud->points[i].z;
   }
   return cloud;
}
pcl::PointCloud<pcl::PointXYZ> PointMatcherCloudtoPcd(PointMatcher<float>::DataPoints source_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    int count = source_cloud.features.cols();
        cloud->resize(count);
        cloud->width = 1;
        cloud->height = count;
        cloud->is_dense = false;
    for(size_t i=0;i<source_cloud.features.cols();i++)
    {
        cloud->points[i].x=source_cloud.features(0,i);
        cloud->points[i].y=source_cloud.features(1,i);
        cloud->points[i].z=source_cloud.features(2,i);
    }
    return cloud;
}
