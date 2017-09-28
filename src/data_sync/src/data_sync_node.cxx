#include "stdlib.h"
#include "iostream"
#include "stdio.h"
#include "sstream"


#include "sync.h"
#include "ros/ros.h"

#include "std_msgs/Byte.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace ros;
using namespace cv;
#define MAXIMUBUFF 1000


sensor_msgs::Imu imu_msg;


AHRS_data imu[MAXIMUBUFF];
AHRS_data imu_data;

int buff_num=0;
int imu_send_ptr=0;
int image_send_ptr=0;
long image_seq;
double image_time;
Mat image_in;




ros::Time image_stamp;
image_transport::Publisher  image_pub;     //publish sync
image_transport::Subscriber image_sub;     //surscribe image
ros::Subscriber             imu_sub;       //subscribe imu
ros::Publisher              imu_pub;       //publish sync imu
ros::Publisher              sync_cmd_pub;  //publish sync command

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{

    static long imu_seq;
    imu[buff_num].accel.acc_x = msg->linear_acceleration.x;
    imu[buff_num].accel.acc_y = msg->linear_acceleration.y;
    imu[buff_num].accel.acc_z = msg->linear_acceleration.z;

    imu[buff_num].gyro.gyro_x = msg->angular_velocity.x;
    imu[buff_num].gyro.gyro_y = msg->angular_velocity.y;
    imu[buff_num].gyro.gyro_z = msg->angular_velocity.z;

    imu[buff_num].seq   = msg->header.seq;
    imu[buff_num].stamp = msg->header.stamp.toSec();


    if(buff_num < MAXIMUBUFF-1)
    {
        buff_num++;
    }
    else
    {
        buff_num=0;
    }
    if(image_seq != 0)
    {
        imu_msg.linear_acceleration.x = imu[imu_send_ptr].accel.acc_x;
        imu_msg.linear_acceleration.y = imu[imu_send_ptr].accel.acc_y;
        imu_msg.linear_acceleration.z = imu[imu_send_ptr].accel.acc_z;

        imu_msg.angular_velocity.x    = imu[imu_send_ptr].gyro.gyro_x;
        imu_msg.angular_velocity.y    = imu[imu_send_ptr].gyro.gyro_y;
        imu_msg.angular_velocity.z    = imu[imu_send_ptr].gyro.gyro_z;

        imu_msg.header.frame_id       = "mpu9250";
        imu_msg.header.seq            = imu[imu_send_ptr].seq;
        imu_seq = imu_msg.header.seq;
        imu_msg.header.stamp          = ros::Time::now();

        imu_pub.publish(imu_msg);
        ROS_INFO("IMU-CAMERA data sync! imu seq: %ld, image seq: %ld",imu_seq, image_seq);
        //image_pub.publish();

        if(imu_send_ptr < MAXIMUBUFF-1)
        {
            imu_send_ptr++;
        }
        else
        {
            imu_send_ptr=0;
        }
    }


}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    image_in = cv_ptr -> image;
    image_seq = msg->header.seq;
    image_stamp = msg->header.stamp;
    image_time = msg->header.stamp.toSec();

    //transportMat(image_in);
    //publish image and set image check flag
}



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "imu_camera_sync_node");
    ros::NodeHandle n,nh;
    image_transport::ImageTransport it(nh);



    cv_bridge::CvImage image_out_msg;

    string GX2750_topic = "/camera_gx2750/image_raw",image_pub_topic = "/camera/image_mono8",
            AHRS_topic   = "/ahrs_data",imu_topic="/imu";



    image_pub = it.advertise(image_pub_topic, 25);               //publish sync
    image_sub = it.subscribe(GX2750_topic , 25, imageCallback);  //surscribe image
    imu_sub   = n.subscribe(AHRS_topic , 400, imu_callback);     //subscribe imu
    imu_pub   = n.advertise<sensor_msgs::Imu>(imu_topic, 400);   //publish sync imu
    sync_cmd_pub  = n.advertise<std_msgs::Byte>("sync_cmd",50);  //publish sync command




    ros::Rate loopRate(1000);

    while(nh.ok() )
    {

        ros::spinOnce();
        loopRate.sleep();
    }


    return 0;
}


































