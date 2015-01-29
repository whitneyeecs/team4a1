#include "StateEstimator.hpp"
#include "math/point.hpp"
#include "math/angle_functions.hpp"
#include "RobotConstants.hpp"

eecs467::StateEstimator::StateEstimator(float x,
	float y, float heading) : _pose({-1, x, y, heading}) { }

void eecs467::StateEstimator::update(float velRight,
	float velLeft, int64_t utime) {
	float deltaTime = utime - _pose.utime;
	_pose.utime = utime;

	float deltaRight = velRight * deltaTime;
	float deltaLeft = velLeft * deltaTime;

	float distance = (deltaRight + deltaLeft) / 2;
	float theta = (deltaRight - deltaLeft) / eecs467::baseLength;

	float alpha = theta / 2;
	_pose.x += distance * cos(_pose.theta + alpha);
	_pose.y += distance * sin(_pose.theta + alpha);
	_pose.theta = angle_sum(_pose.theta, theta);
}

void eecs467::StateEstimator::update(float x, float y, float theta, int64_t utime) {
	_pose.x = x;
	_pose.y = y;
	_pose.theta = theta;
	_pose.utime = utime;
}

void eecs467::StateEstimator::update(const maebot_pose_t& pose) {
	_pose = pose;
}

maebot_pose_t eecs467::StateEstimator::getLatestPoseCopy() const {
	return _pose;
}

const maebot_pose_t& eecs467::StateEstimator::getLatestPose() const {
	return _pose;
}


