#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
class link_pix{
public:
    bool ifend = 0;
    int x;
    int y;
    link_pix* next;
    link_pix* prev;
};

class link_line{
public:
    Vec4f line;
    vector<link_pix*>* pix_chain;
};

class link_seg{
public:
    vector<link_pix*>* pix_chain;
    vector<link_line*>* line_chain;
    link_pix* addr;
    link_seg* next;
};

class link_arc{
public:
    Point o;
    float r;
    vector<link_pix*>* pix_chain;
    vector<link_line*>* line_chain;
};

class link_circle{
public:
    vector<link_arc>* arc_chain;
    vector<link_pix*>* pix_chain;
    Point o;
    float r;
};

class link_ellips{
public:
    vector<link_arc>* arc_chain;
    vector<link_pix*>* pix_chain;
    float A[6];
    Point o;
    float a;
    float b;
    float angle;
};
 vector<link_circle> EDcircles(Mat image);
