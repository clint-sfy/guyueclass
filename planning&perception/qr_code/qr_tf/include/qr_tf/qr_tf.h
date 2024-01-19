#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Header.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <zbar.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>

#include "qr_tf/logger.h"
#include <dmtx.h>

using namespace std;
using namespace cv;
using namespace zbar;


class ImageConverter
{
public:
    ImageConverter(ros::NodeHandle nh,const string& calibFile, const string& saveFile);

    ~ImageConverter(){}

    //订阅回调函数
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void TaskSwitchCallback(const std_msgs::HeaderPtr &task_switch_msg);
private:
	void ProcessFrame(cv_bridge::CvImagePtr cv_ptr);
	
    void QRDecode(Mat img);
    void DMDecode(Mat img);

    void readParameters();
    int readCalibPara(string filename);

    void DrawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
    Scalar& color, int thickness, int lineType);
    float GetAngelOfTwoVector(Point2f &pt1, Point2f &pt2, Point2f &c);
private:
    //ros::NodeHandle _nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    ros::Subscriber task_switch_sub_;
    //tf
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::Quaternion q;

    Size m_markerSize;
    vector<Point2f> m_markerCorners2d;  // marker's 4 corners projection
    vector<Point3f> m_markerCorners3d;
    Mat m_camMat;    // 内参矩阵
    Mat m_distCoeff; // 畸变矩阵
    double unit_x;   // 相机内参 f_x
    double unit_y;   // 相机内参 f_y

    string _calibFile;       //存取相机标定的xml文件
    ofstream sampleRead;     //保存 x坐标  y坐标  角度
    Scalar lineColor;

    bool run_task;
};