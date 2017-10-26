#include "minimal_turtlebot/turtlebot_controller.h"

enum AvoidanceState {MOVING, BACKTRACKING, TURNING, PANIC};
AvoidanceState state = MOVING;

int timeInState = 0;

bool testForCollision(turtlebotInputs turtlebot_inputs)
{
	return (turtlebot_inputs.leftBumperPressed
	 || turtlebot_inputs.rightBumperPressed
	 || turtlebot_inputs.centerBumperPressed);
}

void transitionState(AvoidanceState newState)
{
	state = newState;
	timeInState = 0;
}

void transitionOnCollision(turtlebotInputs turtlebot_inputs, AvoidanceState newState)
{
	if(testForCollision(turtlebot_inputs))
	{ 
		transitionState(newState);
	}
}

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel)
 {
	//Place your code here! you can access the left / right wheel 
	//dropped variables declared above, as well as information about
	//bumper status. 
	
	//outputs have been set to some default values. Feel free 
	//to change these constants to see how they impact the robot.
	
	if(turtlebot_inputs.leftWheelDropped || turtlebot_inputs.rightWheelDropped)
	{
		transitionState(PANIC);
	}
	
	*ang_vel = 0;
	switch(state) {
		case MOVING:
			transitionOnCollision(turtlebot_inputs, BACKTRACKING);
			*vel = .2;
			break;
		case BACKTRACKING:
			*vel = -.2;
			*soundValue = 2;
			if(timeInState++ >= 20)
			{
				transitionState(TURNING);
				*soundValue = 0;

			}
			break;
		case TURNING:
			transitionOnCollision(turtlebot_inputs, BACKTRACKING);
			*vel = 0;
			*ang_vel = .5;
			if(timeInState++ >= 20)
			{
				transitionState(MOVING);
			}
			break;
		case PANIC:
			*vel = 0;
			*ang_vel = 0;
			*soundValue = 1;

			if(!turtlebot_inputs.leftWheelDropped && !turtlebot_inputs.rightWheelDropped)
			{
				transitionState(MOVING);
				*soundValue = 0;

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

