#include <math.h>
#include "minimal_turtlebot/turtlebot_controller.h"

//State definitions
enum AvoidanceState
{
	MOVING = 1,
	BACKTRACKING = 2,
	TURNING = 3,
	PANIC = 4,
	WAIT = 5,
	SPINNING = 6,
	STRAFING = 7,
	WALL_FOLLOWING = 8
};

AvoidanceState state = PANIC;

//Variable for robot steering
bool turningRight = false; //Robot Turn direction controll

//Variable for robot speeds.
const float SPEED_MULTIPLIER = .2;
const float ROTATION_SPEED = .8;

//Variable for robot time control
const int TIMEOUTLENGTH = 15; //Set how long will each state run for
const int FOUR_SPINS_TIMEOUT = 300;
const int MAXIMUM_WAIT = 100;
int timeInState = 0;

//Varibale for goal seeking
bool fromGoal = false; //Define the robot is moving to or from the goal
const float GOAL_ROTATION_TOLERANCE = .1;
const float GOAL_POSITION_TOLERANCE = .05;
//Goal position
const float goalX = -5.0;
const float goalY = 2.0;
const float goalZ = 1.0;
//the intermediate goal, either the goal position or the starting point
float targetX = goalX;
float targetY = goalY;
float targetZ = goalZ;
//Rpbot spin control
float spinInitialRotation = 0.0;
float lastRotationValue;
int spinNumber = 0;

//Robot obstical avoidance
const float DISTANCE_TO_WALL_FOLLOW = 1.0;
const float DISTANCE_TO_STEER_AWAY = 2.0;
const float DISTANCE_FOR_FULL_SPEED = 2.5;
float wallFollowEntrySlope;
int wallFollowTime = 0;


//Returns x, min, or max.  min <= output <= max
float clamp(float min, float x, float max)
{
	return std::max(min, std::min(x, max));
}

//Different calculated values
bool testForCollision(turtlebotInputs turtlebot_inputs) //the control for if the collision of bumper happened or not and if so where the robot should turn
{
	if (turtlebot_inputs.leftBumperPressed || turtlebot_inputs.sensor0State)
	{
		turningRight = true;
		return true;
	}
	if (turtlebot_inputs.rightBumperPressed || turtlebot_inputs.sensor2State)
	{
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
	return fabs(atan2f(sqrt(x * x + y * y), z));
}

bool shouldPanic(turtlebotInputs turtlebot_inputs) //Test if the robot meet the standard to run or not
{
	return (turtlebot_inputs.leftWheelDropped || turtlebot_inputs.rightWheelDropped || calculateAccelerationVectorDegrees(turtlebot_inputs) * 180 / (2 * M_PI) > 20.0 || turtlebot_inputs.battVoltage < -0.0 || isnan(turtlebot_inputs.orientation_omega) || isnan(turtlebot_inputs.z_angle));
}

void transitionState(AvoidanceState newState) //Everytime this is called, the robot will change state!
{
	state = newState;
	timeInState = 0;
}

void transitionOnCollision(turtlebotInputs turtlebot_inputs, AvoidanceState newState) //Check if robot collides with anything or not
{
	if (testForCollision(turtlebot_inputs))
	{
		transitionState(newState);
	}
}

void transitionOnTimeOut(turtlebotInputs turtlebot_inputs, AvoidanceState newState, int timeOutLength) //Counts how long it has been in this state.
{
	if (timeInState++ >= timeOutLength)
	{
		transitionState(newState);
	}
}

void transitionOnRotations(turtlebotInputs turtlebot_inputs, AvoidanceState newState)
{
	float currentRotation = turtlebot_inputs.orientation_omega;
	if (currentRotation < lastRotationValue)
	{
		spinNumber++;
		if (spinNumber > 4)
		{
			transitionState(newState);
		}
	}
	lastRotationValue = currentRotation;
}

struct LaserData
{
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
	for (int i = 0; i < turtlebot_inputs.numPoints; i++)
	{
		float current = turtlebot_inputs.ranges[i];
		if (isnan(current))
			continue;
		if (current < result.lowest)
		{
			result.lowest = current;
			result.lowestIndex = i;
		}
		if (current > result.highest)
		{
			result.highest = current;
			result.highestIndex = i;
		}
	}
	if (timeInState % 10 == 0)
		ROS_INFO("Highest: %f at %u\nLowest: %f at %u", result.highest, result.highestIndex, result.lowest, result.lowestIndex);
	return result;
}

/*	THE PROBLEM IS THAT ANGULARVELOCITYINTENSITY RETURNS ZERO, SO THE
 * 	ROBOT DOES NOT TURN TO AVOID OBSTACLES BECAUSE IT SWERVES WITH ANGULAR
 *  VELOCITY OF 0.  I would fix this myself, but
 * 	I don't know if 320 is just a magic number or where it comes from.
 * 	THIS IS THE PART WE NEED TO FIX.  ONCE WE FIX THIS, IT WILL STEER
 * 	AWAY FROM OBSTACLES WITHIN A CERTAIN RANGE AND STEER TOWARDS THE
 * 	GOAL IF NO OBSTACLES ARE IN SIGHT
 * */
float angularVelocityIntensity(struct LaserData laserData)
{
	int distanceFromMiddle = abs(320 - laserData.lowestIndex);
	int closenessToMiddle = 320 - distanceFromMiddle;
	ROS_INFO("CLOSENESS TO MIDDLE: %u", closenessToMiddle);
	float result = (float)closenessToMiddle / 320.0;
	if (fabs(result) > 1) //ROS_INFO("Bug: AVI is %f", result);
		return result;
}

void quaternionToZAngle(float w, float z, float &roll, float &pitch, float &yaw)
{
	float x = 0;
	float y = 0;
	float sinr = 2 * w * x + 2 * y * z;
	float cosr = 1 - 2 * (x * x + y * y);
	roll = atan2(sinr, cosr);
	float sinp = 2 * (w * y - z * x);
	if (fabs(sinp) >= 1)
	{
		pitch = copysign(M_PI / 2, sinp);
	}
	else
	{
		pitch = asin(sinp);
	}
	float siny = 2 * (w * z + x * y);
	float cosy = 1 - 2 * (y * y + z * z);
	yaw = atan2(siny, cosy);
}

float calculateTranslationalDistanceFromGoal(float currentX, float currentY, float goalX,
											 float goalY)
{
	float x = goalX - currentX;
	float y = goalY - currentY;
	return sqrt(x * x + y * y);
}

float calculateRotationalDistanceFromGoal(float robotOmega, float robotQuaternionZ, float currentX,
										  float currentY, float goalX, float goalY)
{
	float pitch, yaw, roll;
	ROS_INFO("OMEGA: %f, QUATERNIONZ: %f", robotOmega, robotQuaternionZ);
	quaternionToZAngle(robotOmega, robotQuaternionZ, roll, pitch, yaw);
	float goalOrientation = atan2(goalY - currentY, goalX - currentX);
	float result = goalOrientation - yaw;
	if (result > M_PI)
		result -= M_PI * 2;
	if (result < -M_PI)
		result += M_PI * 2;
	ROS_INFO("Yaw: %f, Goal Orientation: %f, Difference: %f", yaw, goalOrientation, result);
	return result;
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

	//set the velocity and base rotational velocity to move towards the goal
	*vel = fmin(translationalDistanceFromGoal * SPEED_MULTIPLIER, .5 * SPEED_MULTIPLIER);
	//*vel = clamp(.1, translationalDistanceFromGoal*.5*SPEED_MULTIPLIER, .9);
	*ang_vel = (rotationalDistanceFromGoal > 0) ? .2 : -.2;

	//1. if it's at the goal, stop
	if (translationalDistanceFromGoal < GOAL_POSITION_TOLERANCE)
	{
		*vel = 0;
		*ang_vel = 0;
		return true;
	}
	turningRight

		//2. if it's not pointing in the right direction, rotate it
		float ang_vel_to_goal = 0;
	if (fabs(rotationalDistanceFromGoal) > GOAL_ROTATION_TOLERANCE)
	{
		ROS_INFO("rotation is %f; tolerance is %f", rotationalDistanceFromGoal, GOAL_ROTATION_TOLERANCE);
		//*vel = 0;
		float rotationSpeed = .5; // = clamp(.25, fabs(rotationalDistanceFromGoal)*.5, .8);
		ang_vel_to_goal = (rotationalDistanceFromGoal > 0) ? rotationSpeed : -rotationSpeed;
		//return false;
	}

	//3. adjust rotational velocturningRightity for obstacles based on their proximity
	//float ang_vel_from_obstacle = 0;
	//if(laserData.lowest < DISTANCE_TO_STEER_AWAY)
	//{
	//	ang_vel_from_obstacle = .5;
	//}

	//set ang_vel to the weighted average of influences
	float weighted_ang_vel_from_obstacle = 0; //clamp(0, ang_vel_from_obstacle/laserData.lowest, 1);
	float weighted_ang_vel_to_goal = 1 - weighted_ang_vel_from_obstacle;
	*ang_vel = (weighted_ang_vel_from_obstacle + weighted_ang_vel_to_goal * ang_vel_to_goal);

	//4. if there's an obstacle in the way, wall follow it
	if (laserData.lowest < DISTANCE_TO_WALL_FOLLOW)
	{
		wallFollowTime = 0;
		wallFollowEntrySlope = (goalY - turtlebot_inputs.y) / (goalX - turtlebot_inputs.x);
		transitionState(WALL_FOLLOWING);
	}

	ROS_INFO("vel: %f, ang_vel: %f", turningRight * vel, *ang_vel);
	return false;
}

bool followWall(turtlebotInputs turtlebot_inputs, float *vel, float *ang_vel, LaserData laserData,
				float goalX, float goalY)
{
	wallFollowTime++;
	float non_directional_ang_vel = .5;
	*vel = .3;
	//if there's nothing in the way, turn back to the obstacle
	if (laserData.lowest > DISTANCE_TO_WALL_FOLLOW)
	{
		*ang_vel = non_directional_ang_vel;
	}
	//otherwise, turn away from it
	else
	{
		*ang_vel = -non_directional_ang_vel;
	}
	//check to see if we're on the other side
	if (fabs((goalY - turtlebot_inputs.y) / (goalX - turtlebot_inputs.x) - wallFollowEntrySlope) < GOAL_POSITION_TOLERANCE && wallFollowTime > 50)
	{
		transitionState(MOVING);
	}
	ROS_INFO("target is %f, %f; current position is %f, %f", goalX, goalY,
			 turtlebot_inputs.x, turtlebot_inputs.y);
	ROS_INFO("vel: %f, ang_vel: %f", *vel, *ang_vel);
	return false;
}

//This is the section where magic all happens, including the switch states and other shits.
void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel)
{
	//General Start of the program
	ROS_INFO("state is: %u", state);

	if (shouldPanic(turtlebot_inputs))
	{
		transitionState(PANIC);
	}

	struct LaserData laserData = laserInterpretation(turtlebot_inputs);

	//TODO: we need to add a state called WAIT so that when there is something in front, it will behave like a gentleman.
	//TODO: we need to modify the state moving to adapt if cloud distance larger than 0.5m, then we use the cloud data to decide the speed, if already, the distance is smaller than 0.5, we should wait, and announce presence.
	switch (state)
	{
	case MOVING:
		*soundValue = 5;
		transitionOnCollision(turtlebot_inputs, BACKTRACKING);

		if (laserData.lowest < .5)
		{
			transitionState(WAIT);
		}

		if (moveToTarget(turtlebot_inputs, vel, ang_vel, laserData, targetX, targetY) && !fromGoal)
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
		transitionOnTimeOut(turtlebot_inputs, MOVING, FOUR_SPINS_TIMEOUT);
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
		transitionOnTimeOut(turtlebot_inputs, STRAFING, TIMEOUTLENGTH);
		break;

	case STRAFING:
		transitionOnCollision(turtlebot_inputs, BACKTRACKING);
		*vel = SPEED_MULTIPLIER;
		*ang_vel = 0;
		transitionOnTimeOut(turtlebot_inputs, MOVING, TIMEOUTLENGTH);
		break;

	case WALL_FOLLOWING:
		transitionOnCollision(turtlebot_inputs, BACKTRACKING);
		if (laserData.lowest < .5)
		{
			transitionState(WAIT);
		}
		if (followWall(turtlebot_inputs, vel, ang_vel, laserData, targetX, targetY))
		{
			transitionState(MOVING);
		}
		break;

	case PANIC:
		*vel = 0;
		*ang_vel = 0;
		*soundValue = 4;
		if (!shouldPanic(turtlebot_inputs))
		{
			transitionState(MOVING);
		}
		break;

	case WAIT:
		*vel = 0;
		*ang_vel = 0;
		*soundValue = 2;
		if (laserData.lowest > 0.5)
		{
			transitionState(MOVING);
		}
		transitionOnTimeOut(turtlebot_inputs, BACKTRACKING, MAXIMUM_WAIT);
		break;
	}
}
