#ifndef TURTLEBOT_CONTROLLER
#define TURTLEBOT_CONTROLLER

#include <stdint.h>
#include <sensor_msgs/Image.h>

#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
#include<kobuki_msgs/WheelDropEvent.h>
#include<kobuki_msgs/BumperEvent.h>
#include<kobuki_msgs/CliffEvent.h>
#include<kobuki_msgs/Sound.h>
#include<kobuki_msgs/SensorState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include "minimal_turtlebot/turtlebot_controller.h"
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <math.h>



struct turtlebotInputs
{
	// time
	uint64_t nanoSecs;

	//wheel drop states
	uint8_t leftWheelDropped;
	uint8_t rightWheelDropped; 
	
	//bumper states 
	uint8_t leftBumperPressed; 
	uint8_t centerBumperPressed;
	uint8_t rightBumperPressed;
	
	//color and depth images
	sensor_msgs::Image colorImage;  
	sensor_msgs::Image depthImage;  
	
	//cliff states
	uint8_t sensor0State; 
	uint8_t sensor1State; 
	uint8_t sensor2State; 
	
	//laserscan data
	std::vector<float> ranges;
	float minAngle; 
	float maxAngle; 
	float angleIncrement; 
	int numPoints; 
	
	//imu data 
	float linearAccelX; 
	float linearAccelY; 
	float linearAccelZ; 
	
	float angularVelocityX; 
	float angularVelocityY; 
	float angularVelocityZ;
	
	float orientationX;
	float orientationY;
	float orientationZ;
	float orientationW; 
	
	//batt voltage
	float battVoltage; 
	
	//odom
	
	float x; 
	float y; 
	float z_angle; 
	float orientation_omega; 
	
	
	
};

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel);
#endif
