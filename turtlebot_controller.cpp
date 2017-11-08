#include <math.h>
#include "minimal_turtlebot/turtlebot_controller.h"


enum AvoidanceState {MOVING = 1, BACKTRACKING = 2, TURNING = 3, PANIC = 4, WAIT = 5};
AvoidanceState state = MOVING; //the initial state will be MOVING
bool turningRight = false; //the magic for controlling where the robot is turning

const int TIMEOUTLENGTH = 15; //Set how long will each state run for
const int MAXIMUM_WAIT = 100;
int timeInState = 0;

const float DISTANCE_FOR_FULL_SPEED = 2.0;
const float SPEED_MULTIPLIER = .2;



//Different calculated values
bool testForCollision(turtlebotInputs turtlebot_inputs) //the control for if the collision of bumper happened or not and if so where the robot should turn
{
	if(turtlebot_inputs.leftBumperPressed || turtlebot_inputs.sensor0State) {
		turningRight = true;
		return true;
	}
	if(turtlebot_inputs.rightBumperPressed || turtlebot_inputs.sensor2State) {
		turningRight = false;
		return true;
	}
	return turtlebot_inputs.centerBumperPressed || turtlebot_inputs.sensor1State;
}

float calculateAccelerationVectorDegrees(turtlebotInputs turtlebot_inputs) //Calculates the robot final acceleration vector
{
	float x = turtlebot_inputs.linearAccelX;
	float y = turtlebot_inputs.linearAccelY;
	float z = turtlebot_inputs.linearAccelZ;
	//ROS_INFO("Acceleration vector is: %f\n", fabs(atan2f(sqrt(x*x + y*y),z)));
	return fabs(atan2f(sqrt(x*x + y*y),z));
}

bool shouldPanic(turtlebotInputs turtlebot_inputs) //Test if the robot meet the standard to run or not
{
	return (turtlebot_inputs.leftWheelDropped
	|| turtlebot_inputs.rightWheelDropped
	|| calculateAccelerationVectorDegrees(turtlebot_inputs)*180/(2*M_PI) > 20.0
	|| turtlebot_inputs.battVoltage < 5.0);
}




void transitionState(AvoidanceState newState) //Everytime this is called, the robot will change state!
{
	state = newState;
	timeInState = 0;
	ROS_INFO("state is: %u",newState);
}

void transitionOnCollision(turtlebotInputs turtlebot_inputs, AvoidanceState newState) //Check if robot collides with anything or not
{
	if(testForCollision(turtlebot_inputs))
	{ 
		transitionState(newState);
	}
}

void transitionOnTimeOut(turtlebotInputs turtlebot_inputs, AvoidanceState newState, int timeOutLength) //Counts how long it has been in this state.
{
	if(timeInState++ >= timeOutLength)
			{
				transitionState(newState);
			}
}

struct LaserData {
	float lowest;
	int lowestIndex;
	float highest;
	int highestIndex;
	bool anyGoodData;
};

//Experimental; Will bite! Refer : http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
struct LaserData laserInterpretation(turtlebotInputs turtlebot_inputs){
	LaserData result;
	result.lowest = INFINITY;
	result.lowestIndex = -1;
	result.highest = -INFINITY;
	result.highestIndex = -1;
	result.anyGoodData = false;
	for (int i = 0; i < turtlebot_inputs.numPoints; i++) {
		float current = turtlebot_inputs.ranges[i];
		if (isnan(current)) continue;
		result.anyGoodData = true;
		if (current < result.lowest) {
			result.lowest = current;
			result.lowestIndex = i;
		}
		if (current > result.highest) {
			result.highest = current;
			result.highestIndex = i;
		}
	}
	if (timeInState % 10 == 0) ROS_INFO("Highest: %f at %u\nLowest: %f at %u", result.highest,result.highestIndex, result.lowest, result.lowestIndex);
	return result;
}

float angularVelocityIntensity(struct LaserData laserData) {
	int distanceFromMiddle = abs(320 - laserData.lowestIndex);
	int closenessToMiddle = 320 - distanceFromMiddle;
	float result = (float)closenessToMiddle / 320.0;
	if (fabs(result) > 1) ROS_INFO("Bug: AVI is %f", result);
	return result; 
}


//This is the section where magic all happens, including the switch states and other shits.
void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel)
 {
	//General Start of the program
	if(shouldPanic(turtlebot_inputs))
	{
		transitionState(PANIC);
	}
	
	*ang_vel = 0; //Should not change this
	
	struct LaserData laserData = laserInterpretation(turtlebot_inputs);
	
	//TODO: we need to add a state called WAIT so that when there is something in front, it will behave like a gentleman.
	//TODO: we need to modify the state moving to adapt if cloud distance larger than 0.5m, then we use the cloud data to decide the speed, if already, the distance is smaller than 0.5, we should wait, and announce presence.
	switch(state) {
		case MOVING:
			*soundValue = 5;
			transitionOnCollision(turtlebot_inputs, BACKTRACKING);
			if (!laserData.anyGoodData || laserData.lowest < 0.5) {
				transitionState(WAIT);
			}
			float distance;
			distance = fmin(laserData.lowest, DISTANCE_FOR_FULL_SPEED);
			*vel = distance * SPEED_MULTIPLIER;
			
			if (laserData.lowest < 1.5) {
				// Swerve
				turningRight = laserData.lowestIndex > 320;
				float avi = angularVelocityIntensity(laserData);
				*ang_vel = turningRight ? avi * -.6 : avi * .6;
				ROS_INFO("Swerving with ang_vel %f", avi);
			} else {
				*ang_vel = 0;
			}
			
			break;
			
		case BACKTRACKING:
			*vel = -.2; 
			*ang_vel = 0;
			transitionOnTimeOut(turtlebot_inputs, TURNING, TIMEOUTLENGTH);
			break;
			
		case TURNING:
			transitionOnCollision(turtlebot_inputs, BACKTRACKING);
			*vel = 0;
			*ang_vel = turningRight ? -.8 : .8;
			transitionOnTimeOut(turtlebot_inputs, MOVING, TIMEOUTLENGTH);
			break;
			
		case PANIC:
			*vel = 0;
			*ang_vel = 0;
			*soundValue = 4;
			if(!shouldPanic(turtlebot_inputs))
			{
				transitionState(MOVING);
			}
			break;
			
		case WAIT:
			*vel = 0;
			*ang_vel = 0;
			*soundValue = 2;
			if (laserData.anyGoodData && laserData.lowest > 0.5) {
				transitionState(MOVING);
			}
			transitionOnTimeOut(turtlebot_inputs, TURNING, MAXIMUM_WAIT);
			break;
	}
	
	

}

