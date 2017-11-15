#include <math.h>
#include "minimal_turtlebot/turtlebot_controller.h"


enum AvoidanceState {MOVING = 1, BACKTRACKING = 2, TURNING = 3, PANIC = 4, WAIT = 5, SPINNING = 6};
AvoidanceState state = MOVING; //the initial state will be MOVING
bool turningRight = false; //the magic for controlling where the robot is turning
bool fromGoal = false; //whether the robot is moving to or from the goal

const int TIMEOUTLENGTH = 15; //Set how long will each state run for
const int MAXIMUM_WAIT = 100;
int timeInState = 0;

const float DISTANCE_FOR_FULL_SPEED = 2.0;
const float SPEED_MULTIPLIER = .2;

const float ROTATION_SPEED = .2;

const float GOAL_ROTATION_TOLERANCE = .04; //TODO: adjust?
const float GOAL_POSITION_TOLERANCE = .05; //TODO: adjust?

//the goal position we're trying to reach TODO: change these
const float goalX = 2.0;
const float goalY = 2.0;
const float goalZ = 1.0;

//the intermediate goal, either the goal position or the starting point
float targetX = goalX;
float targetY = goalY;
float targetZ = goalZ;

float spinInitialRotation = 0.0;
float lastRotationValue;
int spinNumber = 0;

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
	//ROS_INFO("Acceleration vector is: %f\n", fabs(atanROTATION_SPEED2f(sqrt(x*x + y*y),z)));
	return fabs(atan2f(sqrt(x*x + y*y),z));
	}

bool shouldPanic(turtlebotInputs turtlebot_inputs) //Test if the robot meet the standard to run or not
{
	return (turtlebot_inputs.leftWheelDropped
	|| turtlebot_inputs.rightWheelDropped
	|| calculateAccelerationVectorDegrees(turtlebot_inputs)*180/(2*M_PI) > 20.0
	|| turtlebot_inputs.battVoltage < -5.0); //todo:This need to be changed
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

void transitionOnRotations(turtlebotInputs turtlebot_inputs, AvoidanceState newState)
{
	float currentRotation = turtlebot_inputs.orientation_omega;
	if(currentRotation < lastRotationValue)
	{
		spinNumber++;
		if(spinNumber > 4)
		{
			transitionState(newState);
		}
	}
	lastRotationValue = currentRotation;
}

struct LaserData {
	float lowest;
	int lowestIndex;
	float highest;
	int highestIndex;
};

struct LaserData laserInterpretation(turtlebotInputs turtlebot_inputs)
{
	LaserData result;
	result.lowest = INFINITY;
	result.lowestIndex = -1;
	result.highest = -INFINITY;
	result.highestIndex = -1;
	for (int i = 0; i < turtlebot_inputs.numPoints; i++) {
		float current = turtlebot_inputs.ranges[i];
		if (isnan(current)) continue;
		if (current < result.lowest) {
			result.lowest = current;
			result.lowestIndex = i;
		}
		if (current > result.highest) {
			result.highest = current;
			result.highestIndex = i;
		}
	}
	if (timeInState % 10 == 0) 
	//ROS_INFO("Highest: %f at %u\nLowest: %f at %u", result.highest,result.highestIndex, result.lowest, result.lowestIndex);
	return result;
}

float angularVelocityIntensity(struct LaserData laserData) 
{
	int distanceFromMiddle = abs(320 - laserData.lowestIndex);
	int closenessToMiddle = 320 - distanceFromMiddle;
	float result = (float)closenessToMiddle / 320.0;
	if (fabs(result) > 1) //ROS_INFO("Bug: AVI is %f", result);
	return result;
}

void quaternionToZAngle(float w, float z, float& roll, float& pitch, float& yaw)
{
	float x = 0;
	float y = 0;
	float sinr = 2*w*x+ 2*y*z;
	float cosr = 1 - 2*(x*x+y*y);
	roll = atan2(sinr, cosr);
	float sinp = 2*(w*y-z*x);
	if(fabs(sinp) >= 1)
	{
		pitch = copysign(M_PI/2, sinp);
	}
	else
	{
		pitch = asin(sinp);
	}
	float siny = 2*(w*z+x*y);
	float cosy = 1-2*(y*y+z*z);
	yaw = atan2(siny, cosy);
}

float calculateTranslationalDistanceFromGoal(float currentX, float currentY, float goalX,
	float goalY)
{
	float x = goalX - currentX;
	float y = goalY - currentY;
	return sqrt(x*x + y*y);
}

float calculateRotationalDistanceFromGoal(float robotOmega, float robotQuaternionZ, float currentX, 
	float currentY, float goalX, float goalY)
{
	float pitch, yaw, roll;
	quaternionToZAngle(robotOmega, robotQuaternionZ, roll, pitch, yaw);
	float goalOrientation = atan2(goalY-currentY, goalX-currentX);
	ROS_INFO("Omega: %f, Quaternion Z: %f, Yaw: %f, Goal Orientation: %f", robotOmega, robotQuaternionZ, yaw, goalOrientation);
	return goalOrientation-yaw;
}

//TODO: to get it to go back from the goal, call this but with 0, 0, 0 for goal coordinates
//returns whether it has reached the goal yet
bool moveToTarget(turtlebotInputs turtlebot_inputs, float *vel, float *ang_vel, LaserData laserData,
	float goalX, float goalY)
{
	ROS_INFO("target is %f, %f; current position is %f, %f", goalX, goalY,
		turtlebot_inputs.x, turtlebot_inputs.y);
	//float robotOrientation, float currentX, float currentY, float goalX,
	//float goalY
	float robotX_rot = 0;
	float robotY_rot = 0;
	float robotZ_rot;
	float rotationalDistanceFromGoal = calculateRotationalDistanceFromGoal(
		turtlebot_inputs.orientation_omega, turtlebot_inputs.z_angle, turtlebot_inputs.x, 
		turtlebot_inputs.y, targetX, targetY);
	//float currentX, float currentY, float goalX,
	//float goalY
	float translationalDistanceFromGoal = calculateTranslationalDistanceFromGoal(
		turtlebot_inputs.x, turtlebot_inputs.y, targetX, targetY);
	//1. if it's at the goal, stop
	if(translationalDistanceFromGoal < GOAL_POSITION_TOLERANCE)
	{
		*vel = 0;
		*ang_vel = 0;
		return true;
	}

	//2. set translational and rotational velocity to move towards the goal
	//if it's not pointing in the right direction, rotate it, otherwise move forward
	if(fabs(rotationalDistanceFromGoal) > GOAL_ROTATION_TOLERANCE)
	{
		//ROS_INFO("rotation is %f; tolerance is %f", rotationalDistanceFromGoal, GOAL_ROTATION_TOLERANCE*translationalDistanceFromGoal);
		*vel = 0;
		*ang_vel = (rotationalDistanceFromGoal > 0) ? ROTATION_SPEED : -ROTATION_SPEED;
	}
	else
	{
		*vel = translationalDistanceFromGoal*.5*SPEED_MULTIPLIER;
		*ang_vel = 0;
	}
	/*
	//3. adjust for obstacles based on their proximity
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
	*/
	ROS_INFO("vel: %f, ang_vel: %f", *vel, *ang_vel);
	return false;
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
			if (laserData.lowest < 0.5) {
				transitionState(WAIT);
			}
			if(moveToTarget(turtlebot_inputs, vel, ang_vel, laserData, targetX, targetY) && !fromGoal)
			{
				fromGoal = true;
				targetX = 0;
				targetY = 0;
				targetZ = 0;
				spinInitialRotation = turtlebot_inputs.orientation_omega;
				lastRotationValue = spinInitialRotation;
				spinNumber = 0;
				transitionState(SPINNING);
			}
			break;
			
		case SPINNING:
			*vel = 0;
			*ang_vel = ROTATION_SPEED;
			transitionOnRotations(turtlebot_inputs, MOVING);
			break;

		case BACKTRACKING:
			*vel = -.2;
			*ang_vel = 0;
			transitionOnTimeOut(turtlebot_inputs, TURNING, TIMEOUTLENGTH);
			break;

		case TURNING:
			transitionOnCollision(turtlebot_inputs, BACKTRACKING);
			*vel = 0;
			*ang_vel = turningRight ? -ROTATION_SPEED : ROTATION_SPEED;
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
			if (laserData.lowest > 0.5) {
				transitionState(MOVING);
			}
			transitionOnTimeOut(turtlebot_inputs, TURNING, MAXIMUM_WAIT);
			break;
	}



}
