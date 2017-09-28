//include ros libraries
#include<ros/ros.h>

//include messege libraries
#include <sensor_msgs/image_encodings.h>
//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
using namespace cv;
using namespace std;


void image_callback(const sensor_msgs::ImageConstPtr& msg);


int main(int argc,char **argv)
{
    ros::init(argc, argv, "ad130_encode");
    ros::NodeHandle nh;

    cout<< "here is the start."<<endl;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub;

    //img_sub     = it.subscribe("/camera/image_raw", 5, image_callback);
    img_sub     = it.subscribe("/camera/image_raw", 5, image_callback);

    ros::spin();

    return 0;
}




void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cout<< "get a Image."<<endl;
    Mat image;

    cout<< "get 2 Image."<<endl;
    cv_bridge::CvImagePtr cv_ptr;
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BAYER_RGGB8 );
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8 );
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8 );
    cout<< "get 3 Image."<<endl;

    image = cv_ptr -> image;
    //resize(Image,Image,Size(480,360));
    cout<< "get 4 Image."<<endl;

    namedWindow("img",WINDOW_NORMAL);
    imshow("img",image);
    cout<< "get 5 Image."<<endl;

}
