#include "StateEstimator.hpp"
#include "math/point.hpp"
#include "math/angle_functions.hpp"
#include "RobotConstants.hpp"

maebot_pose_t eecs467::advanceState(const maebot_pose_t& pose, int32_t deltaRight, int32_t deltaLeft, int64_t deltaTime) {
	maebot_pose_t newPose;

	float distance = eecs467::metersPerTick * 
		(float)(deltaRight + deltaLeft) / 2.0f;
	float theta = eecs467::metersPerTick * 
		(float)(deltaRight - deltaLeft) / eecs467::baseLength;

	float alpha = theta / 2.0f;
	newPose.x = pose.x + distance * cos(pose.theta + alpha);
	newPose.y = pose.y + distance * sin(pose.theta + alpha);
	newPose.theta = angle_sum(pose.theta, theta);
	newPose.utime = pose.utime + deltaTime;
	return newPose;
}


