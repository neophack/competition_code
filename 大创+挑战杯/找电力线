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
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <strstream>


void GetPointMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloudPtr,cv::Mat &Map)
{
    int width;
    int height;

    pcl::PointXYZI min;
    pcl::PointXYZI max;
    pcl::getMinMax3D(*pointCloudPtr,min,max);
    width=int(max.x-min.x);
    height=int(max.y-min.y);
    Map=cv::Mat::zeros(height+1,width+1,CV_8UC1);
    for (size_t k = 0; k < pointCloudPtr->points.size (); ++k) //显示所有的点
    {
        int j=int(pointCloudPtr->points[k].x-min.x);
        int i=int(pointCloudPtr->points[k].y-min.y);
        Map.at<uchar>(i,j)+=1;
    }

}
void GetPointMaptoImage(cv::Mat &Map,cv::Mat &image)
{

    double minv = 0.0, maxv = 0.0;
    double* minp = &minv;
    double* maxp = &maxv;

    cv::minMaxIdx(Map,minp,maxp);
    // 创建与原图像等尺寸的图像
    image=cv::Mat::zeros(Map.rows,Map.cols,CV_8UC1);
    int nr=Map.rows;

    int nl=Map.cols;
    for(int k=0;k<nr;k++)
    {
        // 每一行图像的指针
        const uchar* inData=Map.ptr<uchar>(k);
        uchar* outData=image.ptr<uchar>(k);
        for(int i=0;i<nl;i++)
        {
            outData[i]=(inData[i]-minv)/(maxv-minv)*255;
        }
    }

}
// 计算叉乘 |P0P1| × |P0P2|
double Multiply(cv::Point p1, cv::Point p2, cv::Point p0)
{
    return ((p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y));
}

bool isContain(cv::Point mp1,cv::Point mp2,cv::Point mp3,cv::Point mp4,cv::Point mp)
{
    if (Multiply(mp, mp1, mp2) * Multiply(mp,mp4, mp3) <= 0

            && Multiply(mp, mp4, mp1) * Multiply(mp, mp3, mp2) <= 0)
        return true;

    return false;
}
void cutcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloudPtr1,cv::Point2f point_lists[],
              pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud1)
{
    pcl::PointXYZI max;
    pcl::PointXYZI min;
    std::vector<pcl::PointXYZI> points;
    std::vector<pcl::PointXYZI> towerpoints;

    pcl::getMinMax3D(*pointCloudPtr1,min,max);
    pcl::PointXYZI point_o_;
    for (size_t k = 0; k < pointCloudPtr1->points.size (); ++k) //显示所有的点
    {
        cv::Point2f point_t;
        point_t.x=pointCloudPtr1->points[k].x-min.x;
        point_t.y=pointCloudPtr1->points[k].y-min.y;
        point_o_=pointCloudPtr1->points[k];
        //point_lists[0],point_lists[1],point_lists[3],point_lists[2]
        if(isContain(point_lists[0],point_lists[1],point_lists[2],point_lists[3],point_t))
        {
            if(point_o_.x!=0||point_o_.y!=0||point_o_.z!=0)
            {
                towerpoints.push_back(point_o_);
            }
        }else
        {
            if(point_o_.x!=0||point_o_.y!=0||point_o_.z!=0)
            {
                points.push_back(point_o_);
            }
        }
    }

    cloud->resize(towerpoints.size());
    cloud->width = towerpoints.size();
    cloud->height = 1;
    cloud->is_dense = false;
    for(int i=0;i<towerpoints.size();i++)
    {
        cloud->points[i]=towerpoints[i];
    }

    cloud1->resize(points.size());
    cloud1->width = points.size();
    cloud1->height = 1;
    cloud1->is_dense = false;
    for(int i=0;i<points.size();i++)
    {
        cloud1->points[i]=points[i];
    }

    //    pointCloudPtr1->points.clear();
    points.clear();
    towerpoints.clear();
}
void get_point_function(float point_cen,float &point_re,int k,int L,int num)
{
    if(num==1)
        point_re=point_cen+(L*k)/(2*sqrt(k*k+1));
    if(num==2)
        point_re=point_cen-(L*k)/(2*sqrt(k*k+1));
    if(num==3)
        point_re=point_cen+L/(2*sqrt(k*k+1));
    if(num==4)
        point_re=point_cen-L/(2*sqrt(k*k+1));
}
void get_up_down_point(cv::Point2f point_cen,cv::Point2f &point_up,cv::Point2f &point_down,int k,int L,bool flag)
{
    if(flag){
        get_point_function(point_cen.y,point_down.y,k,L,1);
        get_point_function(point_cen.y,point_up.y,k,L,2);
        if(k<0)
        {
            get_point_function(point_cen.x,point_down.x,k,L,4);
            get_point_function(point_cen.x,point_up.x,k,L,3);
        }
        else {
            get_point_function(point_cen.x,point_down.x,k,L,3);
            get_point_function(point_cen.x,point_up.x,k,L,4);
        }}
    else{
        get_point_function(point_cen.x,point_down.x,k,L,1);
        get_point_function(point_cen.x,point_up.x,k,L,2);
        if(k>0)
        {
            get_point_function(point_cen.y,point_down.y,k,L,4);
            get_point_function(point_cen.y,point_up.y,k,L,3);
        }
        else
        {
            get_point_function(point_cen.y,point_down.y,k,L,3);
            get_point_function(point_cen.y,point_up.y,k,L,4);
        }
    }
}

void resetpoint(cv::Point2f rect_point[],cv::Point2f center,cv::Point2f rect_point1[])
{
    std::vector<cv::Point2f> rp;
    std::vector<cv::Point2f> lp;
    for(int i=0;i<4;i++)
    {
        if(center.x<rect_point[i].x)
            rp.push_back(rect_point[i]);
        else lp.push_back(rect_point[i]);
    }
    for(int i=0;i<2;i++)
    {
        if(center.y<lp[i].y)
            rect_point1[3]=lp[i];
        else rect_point1[0]=lp[i];
        if(center.y<rp[i].y)
            rect_point1[2]=rp[i];
        else rect_point1[1]=rp[i];
    }
}
double GetHeightDiff(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::PointXYZI max;
    pcl::PointXYZI min;
    pcl::getMinMax3D(*cloud,min,max);
    double diff=max.z-min.z;
    return diff;
}
double Getdistance(cv::Point2f point1,cv::Point2f point2)
{
    return sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
}
void viewer(std::string name,pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr,std::string type_)
{
    pcl::visualization::PCLVisualizer p ("viewer");

    std::cerr << "PointCloudColorHandlerGenericField demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler_z (pointCloudPtr, type_);

    p.addPointCloud (pointCloudPtr, handler_z, name);
    p.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, name);
    p.spin ();
    p.removePointCloud (name);
}
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include"math.h"
#include <list>
int main(int argc,char **argv)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lines (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr towers (new pcl::PointCloud<pcl::PointXYZI>);

    cv::Mat Map;

    pcl::PCDReader pcd;
    if (pcd.read ("/home/ysl/qt_project/build-huangmaochun-Desktop_Qt_5_11_1_GCC_64bit-Debug/2018-06-04-55-line-intensity.pcd", *pointCloudPtr) == -1)
        return (-1);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pointCloudPtr, *pointCloudPtr, indices);

    // test.GetPointCloud(pointCloudPtr,t,pointCloudPtr1,pointCloudPtr2);
    GetPointMap(pointCloudPtr,Map);
    cv::Mat Image;
    cv::Mat imageline=Image.clone();
    GetPointMaptoImage(Map,Image);
    cv::imshow("0",Image);
    cv::waitKey(0);
    cv::Mat drawImage=cv::Mat::zeros(Image.rows,Image.cols,CV_8UC1);

    //获取自定义核
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
    //膨胀操作
    cv::dilate(Image, Image, element);
    cv::erode(Image, Image, element);
    double minv = 0.0, maxv = 0.0;
    double* minp = &minv;
    double* maxp = &maxv;

    cv::minMaxIdx(Image,minp,maxp);
    cv::threshold(Image, Image,int(maxv/3), 255, cv::THRESH_BINARY);
    //获取自定义核
    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 5)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
    //膨胀操作
    cv::dilate(Image, Image, element);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(Image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    cv::RotatedRect box; //定义最小外接矩形

    pcl::PointCloud<pcl::PointXYZI>::Ptr line (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tower (new pcl::PointCloud<pcl::PointXYZI>);

    cv::Point2f rect_point[4];
    std::vector<cv::Point2f> cpoints;
    int num=1;
    double diff=0;
    double diff_sum=0;
    double dist_num=10;
    std::vector<int> contourid;

    for (std::vector<std::vector<cv::Point>>::iterator itc = contours.begin(); itc!=contours.end(); itc++)
    {
        box = cv::minAreaRect(cv::Mat(*itc));
        cpoints.push_back(box.center);
    }
    std::vector<cv::Point2f>::iterator itc_end=cpoints.end()-1;
    for(std::vector<cv::Point2f>::iterator itc = cpoints.begin();itc!=itc_end; itc++)
    {
        for(std::vector<cv::Point2f>::iterator itc1 =itc+1;itc1!=cpoints.end(); itc1++)
        {
            double dist=Getdistance(*itc,*itc1);
            std::cout<<dist<<std::endl;
            if(dist<dist_num)
            {
                std::distance(cpoints.begin(),itc);
                std::distance(cpoints.begin(),itc1);
                contourid.push_back(std::distance(cpoints.begin(),itc));
                contourid.push_back(std::distance(cpoints.begin(),itc1));
            }
        }
    }
    std::vector<std::vector<cv::Point>>::iterator contours_itc = contours.begin();
    std::vector<int>::iterator contours_id = contourid.end()-(contourid.size()/2);

       for(std::vector<int>::iterator id = contourid.begin(); id!=contours_id; id=id+1)
       {
           contours[(*id)*2+*id].insert(contours[(*id)*2+*id].end(), contours[(*id)*2+*id+1].begin(), contours[(*id)*2+*id+1].end());
           contours.erase(contours_itc+(*id)*2+*id+1);
       }

    for (std::vector<std::vector<cv::Point>>::iterator itc = contours.begin(); itc!=contours.end(); itc++)
    {

        box = cv::minAreaRect(cv::Mat(*itc));  //计算每个轮廓最小外接矩形(旋转)

        //boundRect = box.boundingRect();
        //boundRect = boundingRect(Mat(*itc));
        //cv::circle(Image, cv::Point(box.center.x, box.center.y), 5, cv::Scalar(255,255, 255), -1, 8);  //绘制最小外接矩形的中心点
        // rectangle(dstImg, Point(boundRect.x, boundRect.y), Point(boundRect.x + boundRect.width, boundRect.y + boundRect.height), Scalar(0, 255, 0), 2, 8);
        //cv::rectangle(Image, boundRect.tl(), boundRect.br() , cv::Scalar(255, 255, 255), 3, 8);
        box.points(rect_point);  //把最小外接矩形四个端点复制给rect数组

        cv::Point2f center=box.center;
        resetpoint(rect_point,center,rect_point);

        //        double K=(rect_point[0].y-rect_point[3].y)/(rect_point[0].x-rect_point[3].x);
        //        double L_rl=20;
        //        double L_ud=10;
        //        //zuoyou
        //        get_up_down_point(center,point1,point2,K,L_rl,false);
        //        //shangxia
        //        get_up_down_point(point1,rect_point[0],rect_point[3],K,L_ud,true);
        //        get_up_down_point(point2,rect_point[1],rect_point[2],K,L_ud,true);

        cutcloud(pointCloudPtr,rect_point,tower,pointCloudPtr);
        diff=GetHeightDiff(tower);
        diff_sum+=diff;
//        std::cout<<diff_sum/num<<std::endl;
        num++;
        if(diff>=diff_sum/num)
            *towers=*tower+*towers;
        else *pointCloudPtr=*pointCloudPtr+*tower;
        for (int j = 0; j<4; j++)
        {
            cv::line(drawImage, rect_point[j], rect_point[(j + 1) % 4], cv::Scalar(255, 255, 255), 1, 8);  //绘制最小外接矩形每条边
        }

        //        std::cout << "angle " << i << " :" << box.angle << std::endl;
        //        std::cout << "width " << i << " :" << box.size.width << std::endl;
        //        std::cout << "height " << i << " :" << box.size.height << std::endl<<std::endl;
    }
    viewer("tower",towers,"z");
    viewer("line",pointCloudPtr,"z");
    cv::imshow("1",drawImage);
    cv::waitKey(0);
    return 0;
}



