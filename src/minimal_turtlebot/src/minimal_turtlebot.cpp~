#include "minimal_turtlebot/turtlebot_controller.h"

//instantiate some special types for our commands  
kobuki_msgs::Sound soundValue; 
geometry_msgs::Twist base_cmd;

uint8_t localSoundValue=0; 
float localLinearSpeed=0.0; 
float localAngularSpeed=0.0; 

uint8_t soundValueUpdateCounter = 0; 
  
turtlebotInputs localTurtleBotInputs; 

void odomCallback(const geometry_msgs::PoseWithCovarianceStamped& pose) 
{ 
	
	localTurtleBotInputs.x = pose.pose.pose.position.x; 
	localTurtleBotInputs.y = pose.pose.pose.position.y; 
	localTurtleBotInputs.z_angle = pose.pose.pose.orientation.z; 
	localTurtleBotInputs.orientation_omega = pose.pose.pose.orientation.w; 
	
	
}  

void coreCallback(const kobuki_msgs::SensorState& sensor_state) 
{ 
	
	localTurtleBotInputs.battVoltage = sensor_state.battery; 
	localTurtleBotInputs.battVoltage = localTurtleBotInputs.battVoltage*0.1; 
	
	
}  
void imuCallback(const sensor_msgs::Imu& imu_data) 
{ 
	
	localTurtleBotInputs.linearAccelX = imu_data.linear_acceleration.x; 
	localTurtleBotInputs.linearAccelY = imu_data.linear_acceleration.y; 
	localTurtleBotInputs.linearAccelZ = imu_data.linear_acceleration.z; 
	
	localTurtleBotInputs.angularVelocityX = imu_data.angular_velocity.x;
	localTurtleBotInputs.angularVelocityY = imu_data.angular_velocity.y; 
	localTurtleBotInputs.angularVelocityZ = imu_data.angular_velocity.z;
	
	localTurtleBotInputs.orientationX = imu_data.orientation.x;
	localTurtleBotInputs.orientationY = imu_data.orientation.y;
	localTurtleBotInputs.orientationZ = imu_data.orientation.z;
	
}  

void scanCallback(const sensor_msgs::LaserScan& scan_data) 
{ 
	
	localTurtleBotInputs.ranges=scan_data.ranges; 
	localTurtleBotInputs.minAngle=scan_data.angle_min; 
	localTurtleBotInputs.maxAngle=scan_data.angle_max; 
	localTurtleBotInputs.angleIncrement=scan_data.angle_increment; 
	localTurtleBotInputs.numPoints=localTurtleBotInputs.ranges.size(); 
	//ROS_INFO("number scan points is: %i",localTurtleBotInputs.numPoints); 
	
} 

void cliffCallback(const kobuki_msgs::CliffEvent& cliff_data) 
{ 
	if (cliff_data.sensor == 0)
	{
		localTurtleBotInputs.sensor0State = cliff_data.state;
		ROS_INFO("cliff sensor 0 state is: %u",localTurtleBotInputs.sensor0State); 
	}
	else if (cliff_data.sensor == 1)
	{
		localTurtleBotInputs.sensor1State = cliff_data.state;
		ROS_INFO("cliff sensor 1 state is: %u",localTurtleBotInputs.sensor1State); 
	}
	else if (cliff_data.sensor == 2)
	{
		localTurtleBotInputs.sensor2State = cliff_data.state;
		ROS_INFO("cliff sensor 2 state is: %u",localTurtleBotInputs.sensor2State); 
	}
	
} 

void colorImageCallback(const sensor_msgs::Image& image_data_holder) 
{ 
	static uint32_t colorImageInfoCounter = 0; 
	
	localTurtleBotInputs.colorImage=image_data_holder;
	if (colorImageInfoCounter > 30)
	{
		ROS_INFO("color image height: %u",image_data_holder.height);
		ROS_INFO("color image width: %u",image_data_holder.width);
		colorImageInfoCounter=0; 
	}
	else
	{
		colorImageInfoCounter++; 
	}

} 

void depthImageCallback(const sensor_msgs::Image& image_data_holder) 
{ 
	static uint32_t depthImageInfoCounter = 0; 
	
	localTurtleBotInputs.depthImage=image_data_holder; 
	if (depthImageInfoCounter > 1)
	{
		ROS_INFO("depth image height: %u",image_data_holder.height);
		ROS_INFO("depth image width: %u",image_data_holder.width);
		//ROS_INFO("depth image encoding: %s",image_data_holder.encoding.c_str());
		depthImageInfoCounter=0; 
	}
	else
	{
		depthImageInfoCounter++; 
	}
} 


void wheelDropCallBack(const kobuki_msgs::WheelDropEvent& wheel_data_holder) 
{ 
	
	if (wheel_data_holder.wheel == wheel_data_holder.LEFT)
	{
		localTurtleBotInputs.leftWheelDropped = wheel_data_holder.state; 
		ROS_INFO("left wheel dropped state is: %u",wheel_data_holder.state); 
	}
	
	if (wheel_data_holder.wheel == wheel_data_holder.RIGHT)
	{
		localTurtleBotInputs.rightWheelDropped = wheel_data_holder.state; 
		ROS_INFO("right wheel dropped state is: %u",wheel_data_holder.state); 
	}

} 

void bumperMessageCallback(const kobuki_msgs::BumperEvent& bumper_data_holder) 
{ 
	
	if (bumper_data_holder.bumper == bumper_data_holder.LEFT)
	{
		localTurtleBotInputs.leftBumperPressed = bumper_data_holder.state; 
		ROS_INFO("left bumper pressed state is: %u",bumper_data_holder.state); 
	}
	
	if (bumper_data_holder.bumper == bumper_data_holder.CENTER)
	{
		localTurtleBotInputs.centerBumperPressed = bumper_data_holder.state; 
		ROS_INFO("center bumper pressed state is: %u",bumper_data_holder.state); 
	}
	
	if (bumper_data_holder.bumper == bumper_data_holder.RIGHT)
	{
		localTurtleBotInputs.rightBumperPressed = bumper_data_holder.state; 
		ROS_INFO("right bumper pressed state is: %u",bumper_data_holder.state); 
	}

} 

int main(int argc, char **argv) 
{ 
  ros::init(argc,argv,"my_minimal_subscriber"); //name this node 
  // when this compiled code is run, ROS will recognize it as a node called "minimal_subscriber" 
  ros::NodeHandle n; // need this to establish communications with our new node 
  //create a Subscriber object and have it subscribe to the topic "topic1" 
  // the function "myCallback" will wake up whenever a new message is published to topic1 
  // the real work is done inside the callback function 
  
  ros::Rate naptime(10); // use to regulate loop rate 
  
  //subscribe to wheel drop and bumper messages
  ros::Subscriber my_wheel_drop_subscription= n.subscribe("mobile_base/events/wheel_drop",1,wheelDropCallBack); 
  ros::Subscriber my_bumper_subscription= n.subscribe("mobile_base/events/bumper",1,bumperMessageCallback); 
  ros::Subscriber my_cliff_subscription= n.subscribe("mobile_base/events/cliff",1,cliffCallback); 
  ros::Subscriber my_imu_subscription= n.subscribe("mobile_base/sensors/imu_data_raw",1,imuCallback); 
  ros::Subscriber my_core_subscription= n.subscribe("mobile_base/sensors/core",1,coreCallback); 
  ros::Subscriber my_odom_subscription= n.subscribe("odom_combined",1,odomCallback); 
  
  //We don't need color image or depth images for this semester, let's just look at laser scan topics
  //ros::Subscriber colorImageSubscription= n.subscribe("camera/rgb/image_rect_color",1,colorImageCallback); 
  //ros::Subscriber depthSubscription= n.subscribe("camera/depth/image_raw",1,depthImageCallback); 
  
  ros::Subscriber scanSubscription= n.subscribe("scan",1,scanCallback); 
  
  //publish sound and command vel messages 
  
  ros::Publisher my_publisher_object = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1);
  ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
  

  
  while(ros::ok())
  {
	ros::spinOnce();
	localTurtleBotInputs.nanoSecs = ros::Time::now().toNSec();
	turtlebot_controller(localTurtleBotInputs, &localSoundValue, &localLinearSpeed, &localAngularSpeed);
	
	soundValue.value=localSoundValue;
	base_cmd.linear.x=localLinearSpeed;
	base_cmd.angular.z=localAngularSpeed;
	
	if (soundValueUpdateCounter > 10)
	{
		my_publisher_object.publish(soundValue); 
		soundValueUpdateCounter = 0; 
	}
	else
	{
		soundValueUpdateCounter++;
	}
	
	cmd_vel_pub_.publish(base_cmd);
	
	naptime.sleep(); 
	
  }
  return 0; // should never get here, unless roscore dies 
} 

