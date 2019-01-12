#ifndef LANEDETECT_H
#define LANEDETECT_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;
using namespace cv;

class LaneDetect
{
public:
    LaneDetect();
    ~LaneDetect();

    void update(const Mat &src);

    int mode;
    
    vector<Point> getLeftLane();
    vector<Point> getRightLane();

    static int slideThickness;

    static int BIRDVIEW_WIDTH;
    static int BIRDVIEW_HEIGHT;

    static int VERTICAL;
    static int HORIZONTAL;

    static Point null;

private:
    Mat preProcess(const Mat &src, bool rev);

    Mat morphological(const Mat &imgHSV);
    Mat birdViewTranform(const Mat &source);
    void fillLane(Mat &src);
    vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
    vector<vector<Point> > centerRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    void detectLeftRight(const vector<vector<Point> > &points);
    Mat laneInShadow(const Mat &src);

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};
    // int minThreshold[3] = {0, 0, 100};
    // int maxThreshold[3] = {179, 255, 255};
    // int minThreshold[3] = {60, 0, 100};
    // int maxThreshold[3] = {179, 255, 255};
    int minShadowTh[3] = {90, 43, 36};
    int maxShadowTh[3] = {120, 81, 171};
    int minLaneInShadow[3] = {90, 43, 97};
    int maxLaneInShadow[3] = {120, 80, 171};
    int binaryThreshold = 180;

    int skyLine = 85;
    int shadowParam = 40;

    vector<Point> leftLane, rightLane;
};

#endif


