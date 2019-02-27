#include <cstddef>
#include <ctime>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "function.h"
using namespace std;
using namespace cv;

int main(int argc, char** argv) {

  cv::VideoCapture decoder(argv[1]);
  cv::Mat frame;

  while (decoder.read(frame)) {

vector<link_circle> circle_list;
     circle_list=EDcircles(frame);
     vector<link_circle>::iterator circle_list_iter = circle_list.begin();
     vector<link_circle>::iterator circle_list_end = circle_list.end();

     while (circle_list_iter != circle_list_end)
     {
        cricle(frame,circle_list_iter->o,circle_list_iter->r,Scalar(255,0,0));
     }
    imshow("Input video", frame);

    waitKey(1);
  }

  destroyAllWindows();
  decoder.release();

  return EXIT_SUCCESS;
}
