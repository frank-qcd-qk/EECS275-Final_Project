#include "minimal_turtlebot/turtlebot_controller.h"
#include <math.h> 
//Gloable Variable
bool turningRight = false; //the magic for controlling where the robot is turning

void treatLaserData (){
/*
 * This part will incooperate the last four run datas, take average of all 5 datas. 
 * If they have nan inside, they should not be take in consideration. 
 */ 
}

float perfectDistance (turtlebotInputs turtlebot_inputs){
	float safeDistance = 1.5;
	float angleIncrement = turtlebot_inputs.angleIncrement;
	float leftMostAngle = turtlebot_inputs.minAngle;
	float laserReadPerfect = safeDistance * sin(M_PI/2-leftMostAngle);	
	return laserReadPerfect;
} //This method should be fine, not need to modify!!!


void parallelDriving (turtlebotInputs turtlebot_inputs){
	float referenceReading = turtlebot_inputs.Ranges[640];
 	float distanceReference = perfectDistance(turtlebot_inputs);
	//Modify here to a fancier expression !!!
	if (referenceReading > distanceReference){
		
	}
}




/*
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
	//ROS_INFO("Left most: %f ", );
	return result;
}

*/

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel)
 {
	 	ROS_INFO("laserReadPerfect is %f ",distanceReference);

}

