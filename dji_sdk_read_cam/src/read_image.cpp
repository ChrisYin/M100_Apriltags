#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <assert.h>
#include <sys/types.h>
#include <unistd.h>
#include "cv.h"
#include "highgui.h"
#include "djicam.h"

void imageshow_callback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::imshow("viewer",cv_bridge::toCvShare(msg,"mono8")->image);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'mono8'.",msg->encoding.c_str());
	}

}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"image_show");
	ros::NodeHandle node;
	cv::namedWindow("viewer");
	cv::startWindowThread();
	image_transport::ImageTransport transport(node);
	image_transport::Subscriber image_sub = transport.subscribe("dji_sdk/image_raw",1,imageshow_callback);
	ros::spin();
	cv::destroyWindow("viewer");
	

}
