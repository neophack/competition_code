#include<stdio.h>
#include<math.h>
#include<time.h>
#include<sys/timeb.h>
#include<stdlib.h>
#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<opencv/cxcore.h>
#include<vector>
#include<cstdlib>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;

typedef struct _Circle
{
    CvPoint2D32f centre;//圆心
    float radius;//半径
    float CircleMatchRate;//圆周匹配率

} Circle;
typedef vector<Circle> CircleVec;
#define minpoints 10 //包含圆的轮廓的最小点数目

#define cvCopyImage( src, dst ) cvCopy( src, dst, 0 )
int g_sampleRate = 73;
int g_minPtsOnRate = 78;
int g_rmin = 37;
int g_rmax = 300;
int g_edge_thresh = 80;
int g_blur_Size = 5;


//定义结构体指针
IplImage* pSrc;
IplImage* pSrcCopy = NULL;
IplImage* pGray = NULL;
IplImage* pGrayCopy = NULL;
IplImage* pEdge = NULL;
IplImage* pSrcBlur = NULL;

CvMemStorage* storage = NULL;

CircleVec circleArray;
CvContour* contour = NULL;

const char* ImagePath = "D:/All_data/Capture4.bmp";
const char* windowname = "RHT";
const char* edgewndname = "edge";
const char* smoothname = "smooth";

clock_t start, tend;
int sec = 0;
int second = 0;
bool CacuclateCircle(CvPoint p1, CvPoint p2, CvPoint p3, Circle* pCircle);
float CheckPointsOnCircle(IplImage* img, Circle circle);
void RhtCircleDetect(IplImage* image/*二值边缘图*/, CvContour* contour/*轮廓*/, float SampleTHRate/*采样门限率*/, \
    float MinPtsOnRate, float rmin/*最小半径*/, float rmax/*最大半径*/, CircleVec* pcircleArray);
void DoReCal(float SampleTHRate, float MinPtsOnRate, float rmin/*最小半径*/, float rmax/*最大半径*/);


void on_change(int pos)
{
    DoReCal(g_sampleRate / 10.0f, g_minPtsOnRate / 100.0f, (float)g_rmin, (float)g_rmax);
}
void on_trackbar(int pos)
{
    //cvSmooth(pSrc, pSrcBlur, CV_BLUR, g_blur_Size, 3, 0, 0);
    //cvShowImage(smoothname, pSrcBlur);
    cvCvtColor(pSrcBlur, pGray, CV_BGR2GRAY);
    cvCanny(pGray, pEdge, (double)g_edge_thresh, (double)g_edge_thresh * 4, 3);
    cvShowImage(edgewndname, pEdge);
    cvClearMemStorage(storage);
    IplImage* edgecopy = cvCloneImage(pEdge);
    cvFindContours(edgecopy, storage, (CvSeq**)&contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
    cvReleaseImage(&edgecopy);
    printf("edgeeeeeeeeeeee");


    DoReCal(g_sampleRate / 10.0f, g_minPtsOnRate / 100.0f, (float)g_rmin, (float)g_rmax);
}
void on_smooth(int smooth)
{
    if (smooth % 2 == 0)
    {
        smooth = smooth + 1;
    }
    cvSmooth(pSrc, pSrcBlur, CV_GAUSSIAN, smooth, smooth);
    cvShowImage(smoothname, pSrcBlur);
    printf("smooth = %d", smooth);
}

int main(int argc, char** argv)
{
    CvCapture* capture = cvCreateCameraCapture(0);
    //CvCapture* capture = cvCreateFileCapture("bad.avi");
    // IplImage* frame;
    /* if (argc<2)
    {
        printf("Input Image!\n");
        return -1;
    } */

    //	if ((pSrc = cvLoadImage(argv[1], 1)) == NULL)
    /* if ((pSrc = cvQueryFrame(capture)) == NULL)
    {
        printf("Load image failed!\n");
        return -2;
    } */
    while (true)
    {
        pSrc = cvQueryFrame(capture);
        //pSrc = cvLoadImage("D:/All_data/Capture4.bmp");

        //Blur!
        //cvSmooth(pSrc, pSrcBlur, CV_MEDIAN, 3, pSrc->nChannels);
        second = second + 1;
        if (pSrc == NULL)
            break;
        //cvNamedWindow(windowname, CV_WINDOW_AUTOSIZE);
        //cvNamedWindow(edgewndname, CV_WINDOW_AUTOSIZE);
        cvNamedWindow(windowname, WINDOW_NORMAL);
        cvNamedWindow(edgewndname, WINDOW_NORMAL);
        cvNamedWindow(smoothname, WINDOW_NORMAL);
        cvCreateTrackbar("采样门限率", windowname, &g_sampleRate, 100, on_change);
        cvCreateTrackbar("圆周点数周长比", windowname, &g_minPtsOnRate, 100, on_change);
        cvCreateTrackbar("最大半径", windowname, &g_rmax, 500, on_change);
        cvCreateTrackbar("最小半径", windowname, &g_rmin, 100, on_change);
        cvCreateTrackbar("Canny阈值", edgewndname, &g_edge_thresh, 250, on_trackbar);
        cvCreateTrackbar("滤波值", smoothname, &g_blur_Size, 21, on_smooth);

        //cvCreateTrackbar("降噪程度", windowname, &g_blur_Size, 20, on_trackbar);

        pSrcCopy = cvCloneImage(pSrc);
        pSrcBlur = cvCloneImage(pSrc);
        //pSrcBlur = cvCreateImage(cvSize(pSrc->width, pSrc->height), pSrc->depth, 1);
        //pSrcBlur->origin = pSrc->origin;
        pGray = cvCreateImage(cvSize(pSrc->width, pSrc->height), pSrc->depth, 1);
        pGray = cvCreateImage(cvGetSize(pSrcBlur), IPL_DEPTH_8U, 1);
        pGrayCopy = cvCreateImage(cvSize(pSrc->width, pSrc->height), pSrc->depth, 1);
        pGrayCopy->origin = pSrc->origin;
        cvCvtColor(pSrcBlur, pGray, CV_BGR2GRAY);
        //pEdge = cvCreateImage(cvSize(pSrc->width, pSrc->height), pSrc->depth, 1);
        pEdge = cvCreateImage(cvGetSize(pGray), IPL_DEPTH_8U, 1);
        //pEdge->origin = pSrc->origin;

        //cvCvtColor(pSrc, pGray, CV_BGR2GRAY);
        storage = cvCreateMemStorage(0);
        on_trackbar(0);
        char c = cvWaitKey(30);
        if (c == 27)
        {
            break;
        }
    }
    printf("精度： %f",(float)sec / (float)second);
    cvWaitKey(0);
    circleArray.clear();
    cvReleaseMemStorage(&storage);
    //cvReleaseCapture(&capture);
    cvReleaseImage(&pSrcBlur);
    cvReleaseImage(&pSrc);
    cvReleaseImage(&pGray);
    cvReleaseImage(&pGrayCopy);
    cvReleaseImage(&pSrcCopy);
    cvReleaseImage(&pEdge);
    cvDestroyAllWindows();
    return 0;

}
void DoReCal(float SampleTHRate, float MinPtsOnRate, float rmin/*最小半径*/, float rmax/*最大半径*/)
{
    start = clock();
    RhtCircleDetect(pEdge, contour, SampleTHRate, MinPtsOnRate, rmin, rmax, &circleArray);
    cvCopyImage(pSrcCopy, pSrc);

    //	system("clc");

    for (unsigned int i = 0; i<circleArray.size(); i++)
    {
        printf("Centre: (  %.0f,\t%.0f  ),\t\tRadius: %.0f\n", circleArray.at(i).centre.x, circleArray.at(i).centre.y, circleArray.at(i).radius);
        cvCircle(pSrc, cvPoint((int)circleArray.at(i).centre.x, (int)circleArray.at(i).centre.y), 2, CV_RGB(0, 255, 0), -1, 8, 0);
        cvCircle(pSrc, cvPoint((int)circleArray.at(i).centre.x, (int)circleArray.at(i).centre.y), (int)circleArray.at(i).radius, cvScalar(0, 0, 255), 2, 8, 0);
        sec = sec + 1;
    }
    tend = clock();
    if (second == 200)
    {
        printf("精度： %f", (float)sec / (float)second);
        second = 0;
        sec = 0;
    }
    printf("CostTime: %dms   检测精度: %d  / %d\n", (int)(tend - start), sec, second);
    cvShowImage(windowname, pSrc);


}

//通过三点，计算圆的参数，三点共线返回FALSE
inline bool CacuclateCircle(CvPoint p1, CvPoint p2, CvPoint p3, Circle* pCircle)
{
    int x1, x2, x3, y1, y2, y3;
    int delta;
    x1 = p1.x;
    y1 = p1.y;
    x2 = p2.x;
    y2 = p2.y;
    x3 = p3.x;
    y3 = p3.y;
    delta = (x2 - x1) * (y2 - y3) - (x3 - x2) * (y1 - y2);
    //判断是否共线
    if (delta == 0)
    {
        return false;
    }
    float m = ((x2*x2 - x1*x1)*(y2 - y3) - (x3*x3 - x2*x2)*(y1 - y2) + (y1 - y2)*(y2 - y3)*(y3 - y2)) / 2.0f;
    float n = ((x2 - x1)*(x3 - x2)*(x1 - x3) - (y1*y1 - y2*y2)*(x3 - x2) + (y2*y2 - y3*y3)*(x2 - x1)) / 2.0f;
    pCircle->centre.x = m / delta;
    pCircle->centre.y = n / delta;
    float a = pCircle->centre.x - x1;
    float b = pCircle->centre.y - y1;
    pCircle->radius = sqrtf(a*a + b*b);//半径
    return true;
}




//前景点落在圆周上的比率，代码大部分来源于OpenCV的cvCircle()函数，速度会很快
float CheckPointsOnCircle(IplImage* img, Circle circle)
{
    int width = img->width;
    int height = img->height;
    int step = img->widthStep;
    uchar* ptr = (uchar*)(img->imageData);
    int radius = (int)circle.radius;
    int centerx = (int)circle.centre.x;
    int centery = (int)circle.centre.y;
    int inside = centerx >= radius && centerx < width - radius &&
        centery >= radius && centery < height - radius;
    int err = 0, dx = radius, dy = 0, plus = 1, minus = (radius << 1) - 1;
    int totalpoints = 0;
    int OnPoints = 0;
    while (dx >= dy)
    {
        int mask;
        int y11 = centery - dy, y12 = centery + dy, y21 = centery - dx, y22 = centery + dx;
        int x11 = centerx - dx, x12 = centerx + dx, x21 = centerx - dy, x22 = centerx + dy;
        if (inside)
        {
            uchar *tptr0 = ptr + y11 * step;
            uchar *tptr1 = ptr + y12 * step;
            if (*(tptr0 + x11) == 255)
            {
                OnPoints++;
            }
            if (*(tptr1 + x11) == 255)
            {
                OnPoints++;
            }
            if (*(tptr0 + x12) == 255)
            {
                OnPoints++;
            }
            if (*(tptr1 + x12) == 255)
            {
                OnPoints++;
            }
            tptr0 = ptr + y21 * step;
            tptr1 = ptr + y22 * step;
            if (*(tptr0 + x21) == 255)
            {
                OnPoints++;
            }
            if (*(tptr1 + x21) == 255)
            {
                OnPoints++;
            }
            if (*(tptr0 + x22) == 255)
            {
                OnPoints++;
            }
            if (*(tptr1 + x22) == 255)
            {
                OnPoints++;
            }
            totalpoints += 8;
        }
        else if (x11 < width && x12 >= 0 && y21 < height && y22 >= 0)
        {
            if ((unsigned)y11 < (unsigned)height)
            {
                uchar *tptr = ptr + y11 * step;

                if (x11 >= 0)
                {
                    totalpoints++;
                    if (*(tptr + x11) == 255)
                        OnPoints++;
                }
                if (x12 < width)
                {
                    totalpoints++;
                    if (*(tptr + x12) == 255)
                        OnPoints++;
                }
            }
            if ((unsigned)y12 < (unsigned)height)
            {
                uchar *tptr = ptr + y12 * step;
                if (x11 >= 0)
                {
                    totalpoints++;
                    if (*(tptr + x11) == 255)
                        OnPoints++;
                }
                if (x12 <width)
                {
                    totalpoints++;
                    if (*(tptr + x12) == 255)
                        OnPoints++;
                }
            }
            if (x21 < width && x22 >= 0)
            {
                if ((unsigned)y21 < (unsigned)height)
                {
                    uchar *tptr = ptr + y21 * step;

                    if (x21 >= 0)
                    {
                        totalpoints++;
                        if (*(tptr + x21) == 255)
                            OnPoints++;
                    }
                    if (x22 < width)
                    {
                        totalpoints++;
                        if (*(tptr + x22) == 255)
                            OnPoints++;
                    }
                }

                if ((unsigned)y22 < (unsigned)height)
                {
                    uchar *tptr = ptr + y22 * step;

                    if (x21 >= 0)
                    {
                        totalpoints++;
                        if (*(tptr + x21) == 255)
                            OnPoints++;
                    }
                    if (x22 < width)
                    {
                        totalpoints++;
                        if (*(tptr + x22) == 255)
                            OnPoints++;
                    }
                }
            }
        }
        dy++;
        err += plus;
        plus += 2;

        mask = (err <= 0) - 1;

        err -= minus & mask;
        dx += mask;
        minus -= mask & 2;
    }
    return (float)OnPoints / (float)totalpoints;
}
/*===========================================================================================
RhtCircleDetect(IplImage* image,CvContour* contour,float SampleTHRate,float MinPtsOnRate,float rmin,float rmax,CircleVec* pcircleArray)
描述：随机Hough变换检测边缘二值图中的圆
参数：image，输入边缘二值图像；
contour，轮廓，用cvFindContours()函数得到的
SampleTHRate，对于每个轮廓随机取三点的次数与轮廓的长度的比值
MinPtsOnRate，对于检测的圆最少的点数与圆的周长的比率
rmin，检测的圆的最小半径
rmax，检测的圆的最大半径
pcircleArray，将检测到的真的圆存入其中
===========================================================================================*/
void RhtCircleDetect(IplImage* image/*二值边缘图*/, CvContour* contour/*轮廓*/, float SampleTHRate/*采样门限率*/, \
    float MinPtsOnRate, float rmin/*最小半径*/, float rmax/*最大半径*/, CircleVec* pcircleArray)
{
    if (rmin>rmax)
    {
        return;
    }
    CvContour* tmpcontour = contour;
    int raver = (int)((rmin + rmax) / 2);
    int hashi, hashj, hashk;
    IplImage* newimage;
    newimage = cvCreateImage(cvGetSize(image), image->depth, image->nChannels);//创建一个新的图
    cvCopyImage(image, newimage);
    cvDilate(newimage, newimage, NULL, 1);
    if (!pcircleArray->empty())//如果圆的Vector不为空，就清空
    {
        pcircleArray->clear();
    }

    while (tmpcontour != NULL)//对每一个轮廓
    {
        if (tmpcontour->total<minpoints)//过滤掉太小的轮廓
        {
            tmpcontour = (CvContour*)tmpcontour->h_next;
            continue;
        }
        int tableWidth = (int)(tmpcontour->rect.width / 2.0f + 0.5);//hash表的宽度
        int tableHeight = (int)(tmpcontour->rect.height / 2.0f + 0.5);//hash表的高度
        int tableThickness = (int)((rmax - rmin) + 0.5);//hash表的厚度
        uchar* HashTable = new uchar[tableWidth*tableHeight*tableThickness];//为哈希表分配内存
        memset(HashTable, 0, tableWidth*tableHeight*tableThickness);//哈希表清零
        int MaxSampleTimes = (int)(tmpcontour->total / 3 * SampleTHRate);//最大采样次数
        CvPoint* PointArray = new CvPoint[tmpcontour->total];//为轮廓的所有点分配内存
        cvCvtSeqToArray((CvSeq*)tmpcontour, PointArray, CV_WHOLE_SEQ);//将轮廓中点转到点数组中
                                                                      //		int PointsSampled = 0;//记录被采样的点数目
        int SampleTimes = 0;//用来计数采样次数
        int index;
        CvRNG rng_state = cvRNG(0xffffffff);//随机数种子
        while (SampleTimes<MaxSampleTimes)//未到达最大采样次数
        {
            Circle tmpCircle;//临时圆变量
            unsigned Rand1, Rand2, Rand3;//用于存放随机取出来的数据的索引
                                         //产生在0-contour->total之间不重复的三个随机数
            Rand1 = cvRandInt(&rng_state) % tmpcontour->total;
            Rand2 = (Rand1 - raver) % tmpcontour->total;
            Rand3 = (Rand1 + raver) % tmpcontour->total;
            //////////////////////////////////////////////////////////////////////////随机数产生完毕
            if (!CacuclateCircle(PointArray[Rand1], PointArray[Rand2], PointArray[Rand3], &tmpCircle))
            {
                //如果三点共线，一次采样结束
                SampleTimes++;
                continue;
            }
            if (tmpCircle.radius<rmin || tmpCircle.radius>rmax)//半径不在要求的范围之内
            {
                SampleTimes++;
                continue;
            }
            hashi = (int)((tmpCircle.centre.x - tmpcontour->rect.x) / 2 + 0.5);
            hashj = (int)((tmpCircle.centre.y - tmpcontour->rect.y) / 2 + 0.5);
            hashk = (int)((tmpCircle.radius - rmin) / 2 + 0.5);
            if (hashi<0 || hashi >= tableWidth || hashj<0 || hashj >= tableHeight)//说明圆心不在轮廓对应的外接矩形中
            {
                SampleTimes++;
                continue;
            }
            index = (hashj*tableWidth + hashi)*tableThickness + hashk;
            if (HashTable[index] == -1)//已经检测到的圆
            {
                SampleTimes++;
                continue;
            }
            HashTable[index]++;
            if (HashTable[index]>1)//这边是一个阈值，一般设为1或2，即是可能的圆
            {
                float CircleMatchRate = CheckPointsOnCircle(newimage, tmpCircle);
                //通过原图上信息，判断该圆的真伪
                if (CircleMatchRate<MinPtsOnRate)//假的
                {
                    HashTable[index] = -1;
                }
                else//真的圆
                {
                    tmpCircle.CircleMatchRate = CircleMatchRate;
                    if (!pcircleArray->empty())
                    {
                        CircleVec::iterator ittmp = pcircleArray->end() - 1;
                        float dx = ittmp->centre.x - tmpCircle.centre.x;
                        float dy = ittmp->centre.y - tmpCircle.centre.y;
                        if (sqrtf(dx*dx + dy*dy)<max(ittmp->radius, tmpCircle.radius))//两个圆有重叠，选择最优的
                        {
                            if (ittmp->CircleMatchRate*ittmp->radius<tmpCircle.CircleMatchRate*tmpCircle.radius)
                            {
                                pcircleArray->pop_back();
                                pcircleArray->push_back(tmpCircle);
                                cvCircle(newimage, cvPoint((int)tmpCircle.centre.x, (int)tmpCircle.centre.y), (int)tmpCircle.radius, cvScalar(128), 3, 8, 0);
                            }
                        }
                        else
                        {
                            pcircleArray->push_back(tmpCircle);
                            cvCircle(newimage, cvPoint((int)tmpCircle.centre.x, (int)tmpCircle.centre.y), (int)tmpCircle.radius, cvScalar(128), 3, 8, 0);
                        }
                    }
                    else
                    {
                        pcircleArray->push_back(tmpCircle);//找到一个圆
                        cvCircle(newimage, cvPoint((int)tmpCircle.centre.x, (int)tmpCircle.centre.y), (int)tmpCircle.radius, cvScalar(128), 1, 8, 0);

                    }

                    HashTable[index] = -1;
                }

            }
            SampleTimes++;
        }
        delete[] HashTable;
        delete[] PointArray;
        tmpcontour = (CvContour*)tmpcontour->h_next;
    }
    cvReleaseImage(&newimage);

}
