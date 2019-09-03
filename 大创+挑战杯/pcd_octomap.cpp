#include <iostream>
#include <limits>
#include <iostream>
#include <limits>
#include <exception>

#include <octomap/octomap.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>


using namespace std;
using namespace octomap;

/*void calcThresholdedNodes(const OcTree tree,
                          unsigned int& num_thresholded,
                          unsigned int& num_other)
{
  num_thresholded = 0;
  num_other = 0;

  for(OcTree::tree_iterator it = tree.begin_tree(), end=tree.end_tree(); it!= end; ++it){
    if (tree.isNodeAtThreshold(*it))
      num_thresholded++;
    else
      num_other++;
  }
}


void outputStatistics(const OcTree tree){
  unsigned int numThresholded, numOther;
  calcThresholdedNodes(tree, numThresholded, numOther);
  size_t memUsage = tree.memoryUsage();
  unsigned long long memFullGrid = tree.memoryFullGrid();
  size_t numLeafNodes = tree.getNumLeafNodes();

  cout << "Tree size: " << tree.size() <<" nodes (" << numLeafNodes<< " leafs). " <<numThresholded <<" nodes thresholded, "<< numOther << " other\n";
  cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
  cout << "Full grid: "<< memFullGrid << " byte (" << memFullGrid/(1024.*1024.) << " MB)" << endl;
  double x, y, z;
  tree.getMetricSize(x, y, z);
  cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
  cout << endl;
}
*/
#include <iostream>
#include <assert.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//octomap
#include <octomap/octomap.h>
using namespace std;

int main() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB> pointcloud, pointcloud_downsampled;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PCDReader pcd;
    pcd.read("sample.pcd", *pointcloud );
    cout << "Cloud loaded with :"  << pointcloud->points.size() << " vertices" << endl;
    octomap::Pointcloud octocloud;

    OcTree tree (0.05);
    //pointcloud.points.size()

    for (int i = 0; i < pointcloud->points.size(); i++) {
        //cout << "X:" << pointcloud.points[i].x << endl;

        point3d endpoint(pointcloud->points[i].x,pointcloud->points[i].y, pointcloud->points[i].z);
        // tree.updateNode(endpoint, true);
        octocloud.push_back(endpoint);
    }

    cout << "Octocloud size is:" << octocloud.size() << endl;
    point3d sensorOrigin(0,0,0);

    tree.insertPointCloud(octocloud, sensorOrigin);
    //outputStatistics(tree);

    tree.write("1.ot");

}

