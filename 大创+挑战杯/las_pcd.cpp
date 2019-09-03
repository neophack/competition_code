#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <liblas/liblas.hpp>

typedef pcl::PointXYZRGB Point;

using namespace std;


int main()
{
#if(0)
    //确定las文件输入路径以及pcd文件输出路径
    const char* lasfile = "TO_core_last_zoom.las";
    //const char* pcdfile = "123.pcd";

    std::ifstream ifs;
    ifs.open(lasfile, std::ios::in | std::ios::binary);

    liblas::ReaderFactory f ;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();

    pcl::PointCloud<Point>::Ptr pointCloudPtr(new pcl::PointCloud<Point>);
    int count = header.GetPointRecordsCount();
    pointCloudPtr->resize(count);
    pointCloudPtr->width = 1;
    pointCloudPtr->height = count;
    pointCloudPtr->is_dense = false;

    int i = 0;
    while (reader.ReadNextPoint())
    {
        liblas::Point const& p = reader.GetPoint();
        pointCloudPtr->points[i].x = p.GetX();
        pointCloudPtr->points[i].y = p.GetY();
        pointCloudPtr->points[i].z = p.GetZ();
        ++i;
    }
    pcl::io::savePCDFileASCII(pcdfile,*pointCloudPtr);
#elif(1)
    pcl::PointCloud<Point>::Ptr pointCloudPtr (new pcl::PointCloud<Point>);

    pcl::PCDReader pcd;
    if (pcd.read ("123.pcd", *pointCloudPtr) == -1)
      return (-1);

    pcl::visualization::PCLVisualizer p ("test");
    p.setBackgroundColor (1, 1, 1);

    // Handler random color demo
    {
      std::cerr << "PointCloudColorHandlerRandom demo." << std::endl;
      pcl::visualization::PointCloudColorHandlerRandom<Point> handler (pointCloudPtr);

      p.addPointCloud<Point> (pointCloudPtr, "cloud_random");      // no need to add the handler, we use a random handler by default
      p.spin ();
      p.removePointCloud ("cloud_random");

      p.addPointCloud (pointCloudPtr, handler, "cloud_random");
      p.spin ();
      p.removePointCloud ("cloud_random");
    }

    // Handler custom demo
   {
      std::cerr << "PointCloudColorHandlerCustom demo." << std::endl;
      pcl::visualization::PointCloudColorHandlerCustom<Point> handler (pointCloudPtr, 255, 0, 0);

      p.addPointCloud (pointCloudPtr, handler);             // the default id is "cloud"
      p.spin ();
      p.removePointCloud ();                        // the default id is "cloud"

      handler = pcl::visualization::PointCloudColorHandlerCustom<Point> (pointCloudPtr, 255, 0, 0);
      p.addPointCloud (pointCloudPtr, handler, "cloud");
      p.spin ();
      p.removePointCloud ("cloud");
    }

    // Handler RGB demo
    {
      std::cerr << "PointCloudColorHandlerRGBField demo." << std::endl;
      pcl::visualization::PointCloudColorHandlerRGBField<Point> handler (pointCloudPtr);

      p.addPointCloud (pointCloudPtr, handler, "cloud_rgb");
      p.spin ();
      p.removePointCloud ("cloud_rgb");
     }

    // Handler generic field demo
    {
      std::cerr << "PointCloudColorHandlerGenericField demo." << std::endl;
      pcl::visualization::PointCloudColorHandlerGenericField<Point> handler_z (pointCloudPtr, "z");
      pcl::visualization::PointCloudColorHandlerGenericField<Point> handler_x (pointCloudPtr, "x");

      p.addPointCloud (pointCloudPtr, handler_x, "cloud_x");
      p.spin ();
      p.removePointCloud ("cloud_x");

      p.addPointCloud (pointCloudPtr, handler_z, "cloud_z");
      p.spin ();
      p.removePointCloud ("cloud_z");
    }

    p.addCoordinateSystem (0.1);

    // Demonstrate usage of spinOnce()
    p.resetStoppedFlag();
    while (!p.wasStopped())
    {
      static int counter = 0;
      cout << "spinOnce was called "<<++counter<<" times.\n";
      p.spinOnce(1000);  // Give the GUI 1000ms to handle events, then return
    }

    //p.removePointCloud ("cloud");
    //p.spin ();
#endif
    return (0);
}


