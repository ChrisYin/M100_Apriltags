#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltags/AprilTagDetections.h"
#include "apriltags/AprilTagDetection.h"
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

void Cam_infoCallback(apriltags::AprilTagDetections msg)
{
	unsigned int id;
	if(msg.flag==false)
		{
			ROS_INFO("BAD");
		}
		else
		{
	for(unsigned int i = 0; i < msg.detections.size(); ++i)
	{
		id = msg.detections[i].id;
		
			geometry_msgs::Pose pose=msg.detections[i].pose;
			ROS_INFO("Messages have been received successfully!~%d",id);
			ROS_INFO("POSITION X: %f",pose.position.x);
			ROS_INFO("POSITION Y: %f",pose.position.y);
			ROS_INFO("POSITION Z: %f",pose.position.z);
	}
}

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("apriltags/detections",1, Cam_infoCallback);

  ros::spin();

  return 0;
}

