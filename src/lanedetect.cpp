#include "lanedetect.h"

int min(int a, int b)
{
    return a < b ? a : b;
}

static bool initFlag = 1;
int LaneDetect::slideThickness = 10;
int LaneDetect::BIRDVIEW_WIDTH = 240;
int LaneDetect::BIRDVIEW_HEIGHT = 320;
int LaneDetect::VERTICAL = 0;
int LaneDetect::HORIZONTAL = 1;
Point LaneDetect::null = Point();
int curLeft, curRight, curCenter;

void reverse(Mat &src)
{
    int width = src.size().width;
    int height = src.size().height;
    for(int i = 0; i < height; ++i)
        for(int j = 0; j < width; ++j)
        {
            src.at<uchar>(i,j) = 255 - src.at<uchar>(i,j);
        }
}

LaneDetect::LaneDetect()
{
    createTrackbar("LowH", "Threshold", &minThreshold[0], 179);
    createTrackbar("HighH", "Threshold", &maxThreshold[0], 179);

    createTrackbar("LowS", "Threshold", &minThreshold[1], 255);
    createTrackbar("HighS", "Threshold", &maxThreshold[1], 255);

    createTrackbar("LowV", "Threshold", &minThreshold[2], 255);
    createTrackbar("HighV", "Threshold", &maxThreshold[2], 255);

    createTrackbar("Shadow Param", "Threshold", &shadowParam, 255);
}

LaneDetect::~LaneDetect() {}

vector<Point> LaneDetect::getLeftLane()
{
    return leftLane;
}

vector<Point> LaneDetect::getRightLane()
{
    return rightLane;
}

bool check(Mat img, int i, int j)
{
    for(int k = j; k < j + 5; k++)
        if(img.at<uchar>(i,k) != 0) return 0;
    return 1;
}

void LaneDetect::update(const Mat &src)
{
    Mat img;
    Mat birdView, lane;

    // BEGIN SCENARIO_1
    // For turning
    if(mode == 1)
    {
        minThreshold[0] = 0;
        minThreshold[1] = 0;
        minThreshold[2] = 100;

        maxThreshold[0] = 179;
        maxThreshold[1] = 255;
        maxThreshold[2] = 255;

        img = preProcess(src,true);
        birdView = Mat::zeros(img.size(), CV_8UC3);
        lane = Mat::zeros(img.size(), CV_8UC3);

        int BIRDVIEW_CENTER = BIRDVIEW_WIDTH / 2;
        leftLane.clear();
        rightLane.clear();
        
        int dis = 10;

        curCenter = BIRDVIEW_CENTER;
        
        for(int i = BIRDVIEW_HEIGHT - 1; i >= 0; i -= slideThickness)
        {
            circle(lane, Point(curCenter,i), 1, Scalar(0,255,0), 2, 8, 0 );

            leftLane.push_back(null);

            for(int j = curCenter; j >= 0; --j)
            {
                if(img.at<uchar>(i,j) == 0)
                {
                    if(!initFlag && abs(j-curLeft) > dis) continue; 
                    leftLane.back() = Point(j,i);
                    curLeft = j;
                    break;
                }
            }

            rightLane.push_back(null);

            for(int j = curCenter; j < BIRDVIEW_WIDTH; ++j)
            {
                if(img.at<uchar>(i,j) == 0)
                {
                    if(!initFlag && abs(j-curRight) > dis) continue;
                    rightLane.back() = Point(j,i);
                    curRight = j;
                    break;
                }
            }

            curCenter = (curLeft + curRight) / 2;

        }
        reverse(leftLane.begin(),leftLane.end());
        reverse(rightLane.begin(),rightLane.end());
    }
    // END SCENARIO_1
    
    // BEGIN SCENARIO_2
    if(mode == 2)
    {
        minThreshold[0] = 0;
        minThreshold[1] = 0;
        minThreshold[2] = 180;

        maxThreshold[0] = 179;
        maxThreshold[1] = 30;
        maxThreshold[2] = 255;
        
        img = preProcess(src,false);
        birdView = Mat::zeros(img.size(), CV_8UC3);
        lane = Mat::zeros(img.size(), CV_8UC3);

        int BIRDVIEW_CENTER = BIRDVIEW_WIDTH / 2;
        leftLane.clear();
        rightLane.clear();
        vector<Mat> layers1 = splitLayer(img);
        vector<vector<Point> > points1 = centerRoadSide(layers1);
        // vector<Mat> layers2 = splitLayer(img, HORIZONTAL);
        // vector<vector<Point> > points2 = centerRoadSide(layers2, HORIZONTAL);

        detectLeftRight(points1);
        

        for (int i = 0; i < points1.size(); i++)
        {
            for (int j = 0; j < points1[i].size(); j++)
            {
                circle(birdView, points1[i][j], 1, Scalar(0,0,255), 2, 8, 0 );
            }
        }

        imshow("Debug", birdView);
        
        // for (int i = 0; i < points2.size(); i++)
        //  {
        //     for (int j = 0; j < points2[i].size(); j++)
        //     {
        //         circle(birdView, points2[i][j], 1, Scalar(0,255,0), 2, 8, 0 );
        //     }
        // }

        // imshow("Debug", birdView);
    }
    // END SCENARIO_2

    for (int i = 1; i < leftLane.size(); i++)
    {
        if (leftLane[i] != null)
        {
            circle(lane, leftLane[i], 1, Scalar(0,0,255), 2, 8, 0 );
        }
    }

    for (int i = 1; i < rightLane.size(); i++)
    {
        if (rightLane[i] != null) {
            circle(lane, rightLane[i], 1, Scalar(255,0,0), 2, 8, 0 );
        }
    }

    imshow("Lane Detect", lane);
}

Mat LaneDetect::preProcess(const Mat &src, bool rev)
{
    Mat imgShadow, imgThresholded, imgHSV, dst;

    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]), 
        Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), 
        imgThresholded);

    imgShadow = laneInShadow(src);
    bitwise_or(imgShadow,imgThresholded,dst);

    if(rev) reverse(dst);

    // imshow("Debug",dst);

    dst = birdViewTranform(dst);

    imshow("Bird View", dst);

    fillLane(dst);

    imshow("Binary", imgThresholded);

    return dst;
}

Mat LaneDetect::laneInShadow(const Mat &src)
{
    Mat shadowMask, shadow, imgHSV, shadowHSV, laneShadow;
    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(minShadowTh[0], minShadowTh[1], minShadowTh[2]),
    Scalar(maxShadowTh[0], maxShadowTh[1], maxShadowTh[2]),  
    shadowMask);

    src.copyTo(shadow, shadowMask);

    cvtColor(shadow, shadowHSV, COLOR_BGR2HSV);

    inRange(shadowHSV, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]), 
        Scalar(maxLaneInShadow[0], maxLaneInShadow[1], maxLaneInShadow[2]), 
        laneShadow);

    // imshow("Debug", laneShadow);

    return laneShadow;
}

void LaneDetect::fillLane(Mat &src)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI/180, 1);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 1, LINE_AA);
    }
}

vector<Mat> LaneDetect::splitLayer(const Mat &src, int dir)
{
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;

    if (dir == VERTICAL)
    {
        for (int i = 0; i < rowN - slideThickness; i += slideThickness)
        {
            Mat tmp;
            Rect crop(0, i, colN, slideThickness);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    else 
    {
        for (int i = 0; i < colN - slideThickness; i += slideThickness)
        {
            Mat tmp;
            Rect crop(i, 0, slideThickness, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    
    return res;
}

vector<vector<Point> > LaneDetect::centerRoadSide(const vector<Mat> &src, int dir)
{
    vector<std::vector<Point> > res;
    int inputN = src.size();
    for (int i = 0; i < inputN; i++) {
        std::vector<std::vector<Point> > cnts;
        std::vector<Point> tmp;
        findContours(src[i], cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
        int cntsN = cnts.size();
        if (cntsN == 0) {
            res.push_back(tmp);
            continue;
        }

        for (int j = 0; j < cntsN; j++) {
            int area = contourArea(cnts[j], false);
            if (area > 3) {
                Moments M1 = moments(cnts[j], false);
                Point2f center1 = Point2f(static_cast<float> (M1.m10 / M1.m00), static_cast<float> (M1.m01 / M1.m00));
                if (dir == VERTICAL) {
                    center1.y = center1.y + slideThickness*i;
                } 
                else
                {
                    center1.x = center1.x + slideThickness*i;
                }
                if (center1.x > 0 && center1.y > 0) {
                    tmp.push_back(center1);
                }
            }
        }
        res.push_back(tmp);
    }

    return res;
}

void LaneDetect::detectLeftRight(const vector<vector<Point> > &points)
{
    static vector<Point> lane1, lane2;
    lane1.clear();
    lane2.clear();
    
    leftLane.clear();
    rightLane.clear();
    for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i ++)
    {
        leftLane.push_back(null);
        rightLane.push_back(null);
    }

    int pointMap[points.size()][20];
    int prePoint[points.size()][20];
    int postPoint[points.size()][20];
    int dis = 10;
    int max = -1, max2 = -1;
    Point2i posMax, posMax2;

    memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }

    for (int i = points.size() - 2; i >= 0; i--)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            int err = 320;
            for (int m = 1; m < min(points.size() - 1 - i, 5); m++)
            {
                bool check = false;
                for (int k = 0; k < points[i + 1].size(); k ++)
                {
                    if (abs(points[i + m][k].x - points[i][j].x) < dis && 
                    abs(points[i + m][k].x - points[i][j].x) < err) {
                        err = abs(points[i + m][k].x - points[i][j].x);
                        pointMap[i][j] = pointMap[i + m][k] + 1;
                        prePoint[i][j] = k;
                        postPoint[i + m][k] = j;
                        check = true;
                    }
                }   
                break;
            }
            
            if (pointMap[i][j] > max) 
            {
                max = pointMap[i][j];
                posMax = Point2i(i, j);
            }
        }
    }

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max2 && (i != posMax.x || j != posMax.y) && postPoint[i][j] == -1)
            {
                max2 = pointMap[i][j];
                posMax2 = Point2i(i, j);
            }
        }
    }

    if (max == -1) return;

    while (max >= 1)
    {
        lane1.push_back(points[posMax.x][posMax.y]);
        if (max == 1) break;

        posMax.y = prePoint[posMax.x][posMax.y];
        posMax.x += 1;
        
        max--;
    }

    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1) break;

        posMax2.y = prePoint[posMax2.x][posMax2.y];
        posMax2.x += 1;
        
        max2--;
    }
    
    vector<Point> subLane1(lane1.begin(), lane1.begin() + 5);
    vector<Point> subLane2(lane2.begin(), lane2.begin() + 5);

    Vec4f line1, line2;

    fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
    fitLine(subLane2, line2, 2, 0, 0.01, 0.01);

    int lane1X = (BIRDVIEW_WIDTH - line1[3]) * line1[0] / line1[1] + line1[2];
    int lane2X = (BIRDVIEW_WIDTH - line2[3]) * line2[0] / line2[1] + line2[2];

    if (lane1X < lane2X)
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
        for (int i = 0; i < lane2.size(); i++)
        {
            rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
    }
    else
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
        for (int i = 0; i < lane1.size(); i++)
        {
            rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
    }
}


Mat LaneDetect::morphological(const Mat &img)
{
    Mat dst;

    // erode(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );
    // dilate( dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );

    // dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)) );
    // erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)) );

    // blur(dst, dst, Size(3, 3));

    return dst;
}

void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst){
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

Mat LaneDetect::birdViewTranform(const Mat &_src)
{
    Mat src = _src;

    Point2f src_vertices[4];

    int width = src.size().width;
    int height = src.size().height;

    src_vertices[0] = Point(0, skyLine);
    src_vertices[1] = Point(width, skyLine);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0);
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - 105, BIRDVIEW_HEIGHT);
    dst_vertices[3] = Point(105, BIRDVIEW_HEIGHT);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);

    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dst;
}
