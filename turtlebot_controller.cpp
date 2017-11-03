#include <math.h>
#include "minimal_turtlebot/turtlebot_controller.h"

enum AvoidanceState {MOVING, BACKTRACKING, TURNING, PANIC};
AvoidanceState state = MOVING;
bool turningRight = false;

const int TIMEOUTLENGTH = 20;
int timeInState = 0;

bool testForCollision(turtlebotInputs turtlebot_inputs)
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

void transitionState(AvoidanceState newState)
{
	state = newState;
	timeInState = 0;
	ROS_INFO("state is: %u",state);
}

void transitionOnCollision(turtlebotInputs turtlebot_inputs, AvoidanceState newState)
{
	if(testForCollision(turtlebot_inputs))
	{ 
		transitionState(newState);
	}
}

void transitionOnTimeOut(turtlebotInputs turtlebot_inputs, AvoidanceState newState)
{
	if(timeInState++ >= TIMEOUTLENGTH)
			{
				transitionState(newState);
			}
}

float calculateAccelerationVectorDegrees(turtlebotInputs turtlebot_inputs)
{
	float x = turtlebot_inputs.linearAccelX;
	float y = turtlebot_inputs.linearAccelY;
	float z = turtlebot_inputs.linearAccelZ;
	//ROS_INFO("x: %f y: %f z: %f acceleration vector is: %f\n", x, y, z, atan2f(sqrt(x*x + y*y),z)/3.14*180);
	return fabs(atan2f(sqrt(x*x + y*y),z));
}

bool shouldPanic(turtlebotInputs turtlebot_inputs)
{
	return (turtlebot_inputs.leftWheelDropped
	|| turtlebot_inputs.rightWheelDropped
	|| calculateAccelerationVectorDegrees(turtlebot_inputs)*180/(2*M_PI) > 20.0
	|| turtlebot_inputs.battVoltage < 0.0);
}

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel)
 {
	//Place your code here! you can access the left / right wheel 
	//dropped variables declared above, as well as information about
	//bumper status. 
	
	//outputs have been set to some default values. Feel free 
	//to change these constants to see how they impact the robot.
	
	if(shouldPanic(turtlebot_inputs))
	{
		transitionState(PANIC);
	}
	
	//TODO: differentiate turning left from right
	*ang_vel = 0;
	switch(state) {
		case MOVING:
			transitionOnCollision(turtlebot_inputs, BACKTRACKING);
			*vel = .2;
			break;
		case BACKTRACKING:
			*vel = -.2;
			transitionOnTimeOut(turtlebot_inputs, TURNING);
			break;
		case TURNING:
			transitionOnCollision(turtlebot_inputs, BACKTRACKING);
			*vel = 0;
			*ang_vel = turningRight ? -.5 : .5;
			transitionOnTimeOut(turtlebot_inputs, MOVING);
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
	}
	
	
	//*vel = 0.0; // Robot forward velocity in m/s
	//0.7 is max and is a lot
	//*ang_vel = 0;  // Robot angular velocity in rad/s
	//0.7 is max and is a lot 
  
  
	//here are the various sound value enumeration options
	//soundValue.OFF
	//soundValue.RECHARGE
	//soundValue.BUTTON
	//soundValue.ERROR
	//soundValue.CLEANINGSTART
	//soundValue.CLEANINGEND 

}

