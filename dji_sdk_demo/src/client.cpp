/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */



#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "std_msgs/String.h"
#include "apriltags/AprilTagDetections.h"
#include "apriltags/AprilTagDetection.h"
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include "math.h"
//#include <Eigen/Core>
//#include <Eigen/Geometry>

using namespace DJI::onboardSDK;

//! Function Prototypes for Mobile command callbacks - Core Functions
void Cam_infoCallback(apriltags::AprilTagDetections msg);
void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void TakeOffMobileCallback(DJIDrone *drone);
void LandingMobileCallback(DJIDrone *drone);
void GetSDKVersionMobileCallback(DJIDrone *drone);
void ArmMobileCallback(DJIDrone *drone);
void DisarmMobileCallback(DJIDrone *drone);
void GoHomeMobileCallback(DJIDrone *drone);
void TakePhotoMobileCallback(DJIDrone *drone);
void StartVideoMobileCallback(DJIDrone *drone);
void StopVideoMobileCallback(DJIDrone *drone);
//! Function Prototypes for Mobile command callbacks - Custom Missions
void DrawCircleDemoMobileCallback(DJIDrone *drone);
void DrawSquareDemoMobileCallback(DJIDrone *drone);
void GimbalControlDemoMobileCallback(DJIDrone *drone);
void AttitudeControlDemoMobileCallback(DJIDrone *drone);
void LocalNavigationTestMobileCallback(DJIDrone *drone);
void GlobalNavigationTestMobileCallback(DJIDrone *drone);
void WaypointNavigationTestMobileCallback(DJIDrone *drone);
void VirtuaRCTestMobileCallback(DJIDrone *drone);

//! For LAS logging
void StartMapLASLoggingMobileCallback(DJIDrone *drone);
void StopMapLASLoggingMobileCallback(DJIDrone *drone);
void StartCollisionAvoidanceCallback(DJIDrone *drone);
void StopCollisionAvoidanceCallback(DJIDrone *drone);


static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [1]  SDK Version Query        | [20] Set Sync Flag Test          |\n");
	printf("| [2]  Request Control          | [21] Set Msg Frequency Test      |\n");	
	printf("| [3]  Release Control          | [22] Waypoint Mission Upload     |\n");	
	printf("| [4]  Takeoff                  | [23] Hotpoint Mission Upload     |\n");	
	printf("| [5]  Landing                  | [24] Followme Mission Upload     |\n");	
	printf("| [6]  Go Home                  | [25] Mission Start               |\n");	
	printf("| [7]  Gimbal Control Sample    | [26] Mission Pause               |\n");	
	printf("| [8]  Attitude Control Sample  | [27] Mission Resume              |\n");	
	printf("| [9]  Draw Circle Sample       | [28] Mission Cancel              |\n");	
	printf("| [10] Draw Square Sample       | [29] Mission Waypoint Download   |\n");	
	printf("| [11] Take a Picture           | [30] Mission Waypoint Set Speed  |\n");	
	printf("| [12] Start Record Video       | [31] Mission Waypoint Get Speed  |\n");	 
	printf("| [13] Stop Record Video        | [32] Mission Hotpoint Set Speed  |\n");	
	printf("| [14] Local Navigation Test    | [33] Mission Hotpoint Set Radius |\n");	
	printf("| [15] Global Navigation Test   | [34] Mission Hotpoint Reset Yaw  |\n");	
	printf("| [16] Waypoint Navigation Test | [35] Mission Followme Set Target |\n");	
	printf("| [17] Arm the Drone            | [36] Mission Hotpoint Download   |\n");	
	printf("| [18] Disarm the Drone         | [37] Enter Mobile commands mode  |\n");
    printf("| [19] Virtual RC Test           \n");
    printf("+------------------------------12345678-----------------------------------+\n");
    printf("input 1/2/3 etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}

float err_x; //need to subscribe from cam programe
float err_sum_x = 0;
float err_de_x = 0;
float err_pre_x = 0;

float err_y; //need to subscribe from cam programe
float err_sum_y = 0;
float err_de_y = 0;
float err_pre_y = 0;

float x_kp = 1;
float x_ki = 0.01;
float x_kd = 0.02;

float y_kp = 1;
float y_ki = 0.01;
float y_kd = 0.02;

float x_control_vel = 0;
float y_control_vel = 0;

short cam_cap_flag = 0;

static unsigned int id = 0;
   
int main(int argc, char *argv[])
{
    //int main_operate_code = 0;
    int temp32;
    int circleRadius;
    int circleHeight;
    float Phi, circleRadiusIncrements;
    int x_center, y_center, yaw_local;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);

	//virtual RC test data
	uint32_t virtual_rc_data[16];

	//set frequency test data
	uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
	//waypoint action test data
    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

	//groundstation test data
	dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint 	 waypoint;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	dji_sdk::MissionFollowmeTarget followme_target;
    uint8_t userData = 0;
    ros::spinOnce();
    
    //! Setting functions to be called for Mobile App Commands mode 
    drone->setObtainControlMobileCallback(ObtainControlMobileCallback, &userData);
    drone->setReleaseControlMobileCallback(ReleaseControlMobileCallback, &userData);
    drone->setTakeOffMobileCallback(TakeOffMobileCallback, &userData);
    drone->setLandingMobileCallback(LandingMobileCallback, &userData);
    drone->setGetSDKVersionMobileCallback(GetSDKVersionMobileCallback, &userData);
    drone->setArmMobileCallback(ArmMobileCallback, &userData);
    drone->setDisarmMobileCallback(DisarmMobileCallback, &userData);
    drone->setGoHomeMobileCallback(GoHomeMobileCallback, &userData);
    drone->setTakePhotoMobileCallback(TakePhotoMobileCallback, &userData);
    drone->setStartVideoMobileCallback(StartVideoMobileCallback,&userData);
    drone->setStopVideoMobileCallback(StopVideoMobileCallback,&userData);
    drone->setDrawCircleDemoMobileCallback(DrawCircleDemoMobileCallback, &userData);
    drone->setDrawSquareDemoMobileCallback(DrawSquareDemoMobileCallback, &userData);
    drone->setGimbalControlDemoMobileCallback(GimbalControlDemoMobileCallback, &userData);
    drone->setAttitudeControlDemoMobileCallback(AttitudeControlDemoMobileCallback, &userData);
    drone->setLocalNavigationTestMobileCallback(LocalNavigationTestMobileCallback, &userData);
    drone->setGlobalNavigationTestMobileCallback(GlobalNavigationTestMobileCallback, &userData);
    drone->setWaypointNavigationTestMobileCallback(WaypointNavigationTestMobileCallback, &userData);
    drone->setVirtuaRCTestMobileCallback(VirtuaRCTestMobileCallback, &userData);

    drone->setStartMapLASLoggingMobileCallback(StartMapLASLoggingMobileCallback, &userData);
    drone->setStopMapLASLoggingMobileCallback(StopMapLASLoggingMobileCallback, &userData);
    drone->setStartCollisionAvoidanceCallback(StartCollisionAvoidanceCallback, &userData);
    drone->setStopCollisionAvoidanceCallback(StopCollisionAvoidanceCallback, &userData);

	ros::Subscriber sub = nh.subscribe("apriltags/detections",1, Cam_infoCallback);

	
    //Display_Main_Menu();
	
//ROS_INFO("OK");
	
    while(1)
    {
        		ros::spinOnce();
			drone->request_sdk_permission_control();
			//drone->gimbal_angle_control(0, 0, 0, 20);
                	sleep(1);
			drone->takeoff();
                	sleep(3);
			//sleep(1);
			//while(!cam_cap_flag)
				//drone->velocity_control(0,0.1,0,0,0);
			while(!cam_cap_flag)
			{
				if(id == 0)
				{
					drone->velocity_control(0, 0.15, 0, 0, 0);
					usleep(20000);
				}
				if(id == 6)
				{
					drone->velocity_control(0,0,0.2,0,0);
					usleep(20000);
				}
				if(id == 5)
				{
					drone->velocity_control(0,0,0,0,0);
					usleep(20000);
				}
				ros::spinOnce();
				usleep(20000);
				ROS_INFO("id: %d",int(id));
			} 
			int count_stable_flag = 0;
			int land_point = 0;
			while(cam_cap_flag)
			{
				//drone->gimbal_angle_control(0, -900, 0, 20);
				//usleep(20000);
				//ROS_INFO("ERROR X: %f",err_x);
				//ROS_INFO("ERROR Y: %f",err_y);
				switch(id)
				{
					case 5:
						drone->gimbal_angle_control(0, -900, 0, 20);
						land_point = 1;
						while(land_point == 1)
						{
							if(fabs(err_x) < 0.04 && fabs(err_y) < 0.04)
							{
								drone->attitude_control(0x40, 0, 0, 0, 0);
								count_stable_flag++;
								sleep(1);
								if(count_stable_flag == 5)
								{
									drone->landing();
									break;
								}		
							}
							else
							{
								err_de_x = err_x - err_pre_x;
								err_sum_x += err_x; 
								err_pre_x = err_x;

								err_de_y = err_y - err_pre_y;
								err_sum_y += err_y; 
								err_pre_y = err_y;

								if(err_sum_x>10)
									err_sum_x = 10;
								if(err_sum_x<-10)
									err_sum_x = -10;

								if(err_sum_y>10)
									err_sum_y = 10;
								if(err_sum_y<-10)
									err_sum_y = -10;
		
								x_control_vel = x_kp * err_x + x_ki * err_sum_x + x_kd * err_de_x ;
								y_control_vel = y_kp * err_y + y_ki * err_sum_y + y_kd * err_de_y ;
					
								if(x_control_vel>0.2)
									x_control_vel = 0.2;
								if(y_control_vel>0.2)
									y_control_vel = 0.2;
								if(x_control_vel<-0.2)
									x_control_vel = -0.2;
								if(y_control_vel<-0.2)
									y_control_vel = -0.2;
					
						
								drone->velocity_control(0,x_control_vel,y_control_vel,0,0);
								ROS_INFO("id: %d",int(id));
								usleep(20000);
								//ROS_INFO("err_x: %f",err_x);
								//ROS_INFO("err_y: %f",err_y);
								//ROS_INFO("velocity_x: %f",x_control_vel);
								//ROS_INFO("velocity_y: %f",y_control_vel);
								//printf("current control velocity x: %f\n", x_control_vel);
								//printf("current control velocity y: %f\n", y_control_vel);
							}
							ros::spinOnce();
						}
						
					case 6:
						drone->gimbal_angle_control(0, 300, 0, 20);
						for(int i = 0;i<100;i++)
						{
							drone->velocity_control(0,0,0,0.15,0);
							usleep(20000);
						}
						for(int j = 0;j<100;j++)
						{
							drone->velocity_control(0,0,0.2,0,0);
							usleep(20000);
						}
						ros::spinOnce();
						break;
					default:
						break;
				}
					/*if(cam_cap_flag == true)
					ROS_INFO("cap_cam_flag is true");
					else if(cam_cap_flag == false)
					ROS_INFO("cap_cam_flag is false");*/
			}     
        //main_operate_code = -1;
        //Display_Main_Menu();
    }
    return 0;
}

//! Callback functions for Mobile Commands
bool temp666=true;
void Cam_infoCallback(apriltags::AprilTagDetections msg)
{
	if(msg.flag==false)
		{
			if(temp666!=msg.flag)
			{
				ROS_INFO("BAD");
				temp666=false;	
				cam_cap_flag = msg.flag;
				for(unsigned int i = 0; i < msg.detections.size(); ++i)
				id = msg.detections[i].id;
			}
		}
		else
		{
			for(unsigned int i = 0; i < msg.detections.size(); ++i)
			{
				if(temp666!=msg.flag)
				{
					ROS_INFO("GOOD");
					temp666=true;
				}
				id = msg.detections[i].id;
		
					geometry_msgs::Pose pose=msg.detections[i].pose;
					//ROS_INFO("Messages have been received successfully!~%d",id);
					//ROS_INFO("POSITION X: %f",pose.position.x);
					//ROS_INFO("POSITION Y: %f",pose.position.y);
					//ROS_INFO("POSITION Z: %f",pose.position.z);
					err_x = pose.position.y;			
					err_y = -pose.position.x;
			
					cam_cap_flag = msg.flag;
			}
		}

}
    void ObtainControlMobileCallback(DJIDrone *drone)
    {
      drone->request_sdk_permission_control();
    }

    void ReleaseControlMobileCallback(DJIDrone *drone)
    {
      drone->release_sdk_permission_control();
    }

    void TakeOffMobileCallback(DJIDrone *drone)
    {
      drone->takeoff();
    }

    void LandingMobileCallback(DJIDrone *drone)
    {
      drone->landing();
    }

    void GetSDKVersionMobileCallback(DJIDrone *drone)
    {
      drone->check_version();
    }

    void ArmMobileCallback(DJIDrone *drone)
    {
      drone->drone_arm();
    }

    void DisarmMobileCallback(DJIDrone *drone)
    {
      drone->drone_disarm();
    }

    void GoHomeMobileCallback(DJIDrone *drone)
    {
      drone->gohome();
    }

    void TakePhotoMobileCallback(DJIDrone *drone)
    {
      drone->take_picture();
    }

    void StartVideoMobileCallback(DJIDrone *drone)
    {
      drone->start_video();
    }

    void StopVideoMobileCallback(DJIDrone *drone)
    {
      drone->stop_video();
    }

    void DrawCircleDemoMobileCallback(DJIDrone *drone)
    {
        static float R = 2;
        static float V = 2;
        static float x;
        static float y;
        int circleRadius;
        int circleHeight;
        float Phi =0, circleRadiusIncrements;
        int x_center, y_center, yaw_local; 

        circleHeight = 7;
        circleRadius = 7;

        x_center = drone->local_position.x;
        y_center = drone->local_position.y;
        circleRadiusIncrements = 0.01;

        for(int j = 0; j < 1000; j ++)
        {   
            if (circleRadiusIncrements < circleRadius)
            {
                x =  x_center + circleRadiusIncrements;
                y =  y_center;
                circleRadiusIncrements = circleRadiusIncrements + 0.01;
                drone->local_position_control(x ,y ,circleHeight, 0);
                usleep(20000);
            }
                else
            {
                break;
            }
        }
        

        /* start to draw circle */
        for(int i = 0; i < 1890; i ++)
        {   
            x =  x_center + circleRadius*cos((Phi/300));
            y =  y_center + circleRadius*sin((Phi/300));
            Phi = Phi+1;
            drone->local_position_control(x ,y ,circleHeight, 0);
            usleep(20000);
        }

    }
    void DrawSquareDemoMobileCallback(DJIDrone *drone)
    {
    /*draw square sample*/
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, 3, 0, 0 );
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, 3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, -3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, -3, 0, 0);
            usleep(20000);
        }
    }

     void GimbalControlDemoMobileCallback(DJIDrone *drone)
        {
        drone->gimbal_angle_control(0, 0, 1800, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 0, -1800, 20);
        sleep(2);
        drone->gimbal_angle_control(300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(-300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 300, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, -300, 0, 20);
        sleep(2);
        drone->gimbal_speed_control(100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(-100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(0, 0, 200);
        sleep(2);
        drone->gimbal_speed_control(0, 0, -200);
        sleep(2);
        drone->gimbal_speed_control(0, 200, 0);
        sleep(2);
        drone->gimbal_speed_control(0, -200, 0);
        sleep(2);
        drone->gimbal_angle_control(0, 0, 0, 20);
        }

    void AttitudeControlDemoMobileCallback(DJIDrone *drone)
    {
        /* attitude control sample*/
        drone->takeoff();
        sleep(8);


        for(int i = 0; i < 100; i ++)
        {
            if(i < 90)
                drone->attitude_control(0x40, 0, 2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 2, 0, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, -2, 0, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, -2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 0, 0.5, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 0, -0.5, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0xA, 0, 0, 0, 90);
            else
                drone->attitude_control(0xA, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0xA, 0, 0, 0, -90);
            else
                drone->attitude_control(0xA, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        drone->landing();

    }
    void LocalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void GlobalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void WaypointNavigationTestMobileCallback(DJIDrone *drone)
    {
        
    }
    void VirtuaRCTestMobileCallback(DJIDrone *drone)
    {
        //virtual RC test data
        uint32_t virtual_rc_data[16];
        //virtual rc test 1: arm & disarm
        drone->virtual_rc_enable();
        usleep(20000);

        virtual_rc_data[0] = 1024;  //0-> roll      [1024-660,1024+660] 
        virtual_rc_data[1] = 1024;  //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024+660;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;  //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1684;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for (int i = 0; i < 100; i++){
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }

        //virtual rc test 2: yaw 
        drone->virtual_rc_enable();
        virtual_rc_data[0] = 1024;      //0-> roll      [1024-660,1024+660] 
        virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024-200;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;      //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1324;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for(int i = 0; i < 100; i++) {
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }
        drone->virtual_rc_disable();
    }

void StartMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("roslaunch point_cloud_las start_velodyne_and_loam.launch &");
  system("rosrun point_cloud_las write _topic:=/laser_cloud_surround _folder_path:=. &");
}

void StopMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("rosnode kill /write_LAS /scanRegistration /laserMapping /transformMaintenance /laserOdometry  &");
}

void StartCollisionAvoidanceCallback(DJIDrone *drone)
{ 
  uint8_t freq[16];
  freq[0] = 1;    // 0 - Timestamp
  freq[1] = 4;    // 1 - Attitude Quaterniouns
  freq[2] = 1;    // 2 - Acceleration
  freq[3] = 4;    // 3 - Velocity (Ground Frame)
  freq[4] = 4;    // 4 - Angular Velocity (Body Frame)
  freq[5] = 3;    // 5 - Position
  freq[6] = 0;    // 6 - Magnetometer
  freq[7] = 3;    // 7 - M100:RC Channels Data, A3:RTK Detailed Information
  freq[8] = 0;    // 8 - M100:Gimbal Data, A3: Magnetometer
  freq[9] = 3;    // 9 - M100:Flight Status, A3: RC Channels
  freq[10] = 0;   // 10 - M100:Battery Level, A3: Gimble Data
  freq[11] = 2;   // 11 - M100:Control Information, A3: Flight Status

  drone->set_message_frequency(freq);
  usleep(1e4);
  system("roslaunch dji_collision_avoidance from_DJI_ros_demo.launch &");
}

void StopCollisionAvoidanceCallback(DJIDrone *drone)
{
  drone->release_sdk_permission_control();
  system("rosnode kill /drone_tf_builder /dji_occupancy_grid_node /dji_collision_detection_node /collision_velodyne_nodelet_manager /manual_fly");
  usleep(1e4);
  drone->request_sdk_permission_control();
}
