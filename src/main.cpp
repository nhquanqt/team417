#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

#include "signclassify.h"
#include "lanedetect.h"
#include "objectdetect.h"
#include "carcontrol.h"

#include <time.h>

using namespace std;

const int INF = 1e9;
const float WAIT_SECOND = 5;

bool STREAM = true;

SignClassify *signClassify;
LaneDetect *laneDetect;
ObjectDetect *objectDetect;
CarControl *car;
int skipFrame = 1;

clock_t tLeft, tRight;
bool flag;
float velocity = 40;

float timeInSecond(clock_t t)
{
    return (clock() - t) / CLOCKS_PER_SEC;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::imshow("View", cv_ptr->image);
	    waitKey(1);
        
        if(!flag && (timeInSecond(tLeft) <= WAIT_SECOND || timeInSecond(tRight) <= WAIT_SECOND)) laneDetect->mode = 1;
        else laneDetect->mode = 2;
        laneDetect->update(cv_ptr->image);

        if(flag) velocity = 25;
        else velocity = 40;

        objectDetect->update(cv_ptr->image);
        vector<int> ClassIds = objectDetect->ClassIds;
        vector<Rect> Data = objectDetect->Data;

        for(int i = 0; i < (int)ClassIds.size(); ++i){
            int id = ClassIds[i];
            if(id >= 2){
                Rect roi = Data[i];
                int centerX = roi.x + roi.width / 2;
                if(centerX < cv_ptr->image.cols / 2) car->moveRightSide(laneDetect->getRightLane(),velocity * 0.8);
                else car->moveLeftSide(laneDetect->getLeftLane(),velocity * 0.8);
                return;
            }
        }
        
        if(!flag && timeInSecond(tLeft) <= WAIT_SECOND){
            car->turnLeft(laneDetect->getLeftLane(),25);
            return;
        }
        if(!flag && timeInSecond(tRight) <= WAIT_SECOND){
            car->turnRight(laneDetect->getRightLane(),25);
            return;
        }

        flag = 0;
        for(int i = 0; i < (int)Data.size(); ++i){
            Mat img;
            if(signClassify->cropImage(cv_ptr->image,img,Data[i]) == -1) continue;
            int id = signClassify->classify(img);
            if(id == 0){
                tLeft = clock();
                flag = 1;
                break;
            }
            if(id == 1){
                tRight = clock();
                flag = 1;
                break;
            }
        }
        
        car->driverCar(laneDetect->getLeftLane(), laneDetect->getRightLane(), velocity);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    cv::namedWindow("Binary");
    cv::namedWindow("Threshold");
    cv::namedWindow("PID");
    cv::namedWindow("Bird View");
    cv::namedWindow("Lane Detect");
    cv::namedWindow("Object Detect");
    cv::namedWindow("Debug");

    signClassify = new SignClassify();
    laneDetect = new LaneDetect();
    objectDetect = new ObjectDetect();
    car = new CarControl();

    cv::startWindowThread();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("Team417_image", 1, imageCallback);

    ros::spin();

    cv::destroyAllWindows();
}
