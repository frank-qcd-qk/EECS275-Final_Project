#include <math.h>
#include "minimal_turtlebot/turtlebot_controller.h"


enum AvoidanceState {MOVING, BACKTRACKING, TURNING, PANIC,WAIT};
AvoidanceState state = MOVING; //the initial state will be MOVING
bool turningRight = false; //the magic for controlling where the robot is turning
const int TIMEOUTLENGTH = 15; //Set how long will each state run for
int timeInState = 0;



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
	ROS_INFO("state is: %u",20);
}

void transitionOnCollision(turtlebotInputs turtlebot_inputs, AvoidanceState newState) //Check if robot collides with anything or not
{
	if(testForCollision(turtlebot_inputs))
	{ 
		transitionState(newState);
	}
}

void transitionOnTimeOut(turtlebotInputs turtlebot_inputs, AvoidanceState newState) //Counts how long it has been in this state.
{
	if(timeInState++ >= TIMEOUTLENGTH)
			{
				transitionState(newState);
			}
}

//Experimental; Will bite! Refer : http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
void laserinterpretation(turtlebotInputs turtlebot_inputs){
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
	
	//TODO: we need to add a state called WAIT so that when there is something in front, it will behave like a gentleman.
	//TODO: we need to modify the state moving to adapt if cloud distance larger than 0.5m, then we use the cloud data to decide the speed, if already, the distance is smaller than 0.5, we should wait, and announce presence.
	switch(state) {
		case MOVING:
			*soundValue = 5;
			laserinterpretation(turtlebot_inputs);
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
			*ang_vel = turningRight ? -.8 : .8;
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
			
		case WAIT:
			*vel = 0;
			*ang_vel = 0;
			*soundValue = 2;
			break;
	}

}

