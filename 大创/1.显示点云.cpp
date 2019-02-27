#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGB Point;
void viewer(pcl::PointCloud<Point>::Ptr cloud){

 pcl::visualization::PCLVisualizer p ("test");
  p.setBackgroundColor (1, 1, 1);

    std::cerr << "PointCloudColorHandlerRandom demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerRandom<Point> handler (cloud);
    p.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7);  

    p.addPointCloud<Point> (cloud, "cloud_random");      // no need to add the handler, we use a random handler by default
    p.spin ();
    p.removePointCloud ("cloud_random");

}
int main (int argc, char **argv)
{

  pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);

  pcl::PCDReader pcd;
  if (pcd.read ("test.pcd", *cloud) == -1)
    return (-1);


   viewer(cloud);
 
 return 0;
}
