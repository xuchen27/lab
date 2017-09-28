/**
******************************************************************************
* @file    union_calibration.cpp
* @author  FrankChen
* @version V1.1.0
* @date    4-May-2016
* @brief   This file provides basic funtion that solve Mathematical problems which
*          use in calibration and reconstruct 3D information.
******************************************************************************
*/

//include ros library
#include "ros/ros.h"

//include std/string libraries
#include<sstream>
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<vector>
//include messege libraries
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

//include eigen library
#include "eigen3/Eigen/Dense"

//include user library
#include "user.h"

//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace Eigen;
using namespace std;
using namespace cv;

vector <bit_pose> pose;

//bit_pose pose1,pose2,pose3,pose4;
bit_position roi_begin,roi_end;

Mat rosImage;
//ros::Subscriber pose_1_sub;

image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;
MatrixXf CM;
MatrixXf DM;

bool found;


Mat pre_image,next_image,subtract_background;
bool frame_flag = true;

int transportMat(Mat src)
{
    rosImage = src.clone();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat image_in;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    image_in = cv_ptr -> image;
    transportMat(image_in);

    //temp add
//    roi_begin.u       = 344;
//    roi_begin.v       = 550;
//    roi_end.u         = 1032;
//    roi_end.v         = 1100;
    Mat image_temp;
    image_temp=image_in(Rect( 355, 550, 1032-344, 1100-550 ));
//    image_temp=image_in;
    if(frame_flag == true)
    {
        pre_image = image_temp.clone();
        frame_flag = false;
        imshow("image_odd",pre_image);
    }
    else
    {
        next_image = image_temp.clone();
        frame_flag =true;
        subtract(next_image,pre_image,subtract_background);
        imshow("image_even",next_image);
        imshow("subtract_background",subtract_background);
    }
}

int eigen()
{
    //    float r_pos_x,r_pos_y,r_pos_z;
    //    float diff_21_x,diff_43_x,diff_21_y,diff_43_y,diff_21_z,diff_43_z;
    //    diff_21_x = pose2.pos.x_w-pose1.pos.x_w;
    //    diff_21_y = pose2.pos.y_w-pose1.pos.y_w;
    //    diff_21_z = pose2.pos.z_w-pose1.pos.z_w;

    //    diff_43_x = pose4.pos.x_w-pose3.pos.x_w;
    //    diff_43_y = pose4.pos.y_w-pose3.pos.y_w;
    //    diff_43_z = pose4.pos.z_w-pose3.pos.z_w;

}

int feature_extract()
{
    //    cv_bridge::CvImage image_msg;
    //    Mat roi_image;
    //    if((point1.reliable>=0.5)&&(point2.reliable>=0.5)&&(point3.reliable>=0.5)&&(point4.reliable>=0.5))
    //    {
    //        Rect roi_rect=Rect( (int)point1.u, (int)point1.v, (int)point4.u-(int)point1.u, (int)point4.v-(int)point1.v );
    //        //Rect roi_rect=Rect( (int)point1.u, (int)point1.v, (int)point4.u-(int)point1.u, (int)point4.v-(int)point1.v );
    //        roi_image=rosImage(roi_rect);
    //        //imshow("ROI",roi_image);

    //        image_msg.image = roi_image;
    //        image_msg.header.frame_id = "ROI_of_Union_calibration_Board";
    //        image_msg.header.stamp = ros::Time::now();
    //        image_msg.encoding = "mono8";
    //        image_pub.publish(image_msg.toImageMsg());

    //    }
    //    else
    //        ;
    //        //imshow("ROI",rosImage);
}


int main(int argc,char **argv)
{
    ros::init(argc, argv, "union_calibration");
    ros::NodeHandle n,nh;
    image_transport::ImageTransport it(nh);

    string ros_image_topic;
    string aruco_marker_map;
    float  aruco_marker_size;
    image_sub = it.subscribe("/camera_gx2750/image_raw" , 10, imageCallback);
    image_pub = it.advertise("aruco_board_roi",10);

    Mat chess_gray,roi_image;
    Size chess_size;
    vector<Point2f> corner_point;
    Rect roi_rect;
    int found_count =1;

    //per define parameter and varible
    chess_size.height = 4;
    chess_size.width  = 4;
    roi_begin.u       = 344;
    roi_begin.v       = 550;
    roi_end.u         = 1032;
    roi_end.v         = 1100;

    while(rosImage.empty())
        ros::spinOnce();

    Mat image_a,image_b,image_c;




    while(n.ok()&&('q' != waitKey(10)) )
    {
        char c=waitKey(10);
        roi_rect=Rect( (int)roi_begin.u, (int)roi_begin.v, (int)roi_end.u-(int)roi_begin.u, (int)roi_end.v-(int)roi_begin.v );
        roi_image=rosImage(roi_rect);

        if('a' == c)
        {
            image_a = roi_image.clone();
            imshow("image_a",image_a);
            //cout << "capture image a!" << endl;
            ROS_INFO("capture image a!");

        }
        if('b' == c)
        {
            image_b = roi_image.clone();
            imshow("image_b",image_b);
            //cout << "capture image b!" << endl;
            ROS_INFO("capture image b!");

        }
        if('d' == c)
        {
            cout << "diff between image a and b" <<endl;
            ROS_INFO("diff between image a and b!");

            subtract(image_a,image_b,image_c);

            imshow("diff",image_c);
        }


//        if(frame_flag == true)
//        {
//            pre_image = roi_image.clone();
//            frame_flag = false;
//            imshow("image_odd",pre_image);
//        }
//        else
//        {
//            next_image = roi_image.clone();
//            frame_flag =true;
//            subtract(pre_image,next_image,subtract_background);
//            imshow("image_even",next_image);
//            imshow("subtract_background",subtract_background);
//        }



//        found = findChessboardCorners( roi_image, chess_size, corner_point,
//                                       CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK);


//        if(found==true)
//        {

//            cornerSubPix( roi_image, corner_point, Size(11,11),
//                          Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
//            drawChessboardCorners(roi_image, chess_size, Mat(corner_point), found);
//            cout << "found chess board!" << found_count << endl;
//            cout << corner_point << endl;
//            found_count++;
//        }
        imshow("Image",roi_image);
//        imshow("ROI",roi_image);

        ros::spinOnce();
    }



    return 0;
}

