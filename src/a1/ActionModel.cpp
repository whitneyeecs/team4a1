#include "ActionModel.hpp"

#include "a1/StateEstimator.hpp"

#include <cmath>

eecs467::ActionModel::ActionModel(float k1, float k2) 
	: _normDist(0, 1), _k1(k1), _k2(k2) { }

void eecs467::ActionModel::apply(maebot_pose_t& pose, int32_t deltaRight, int32_t deltaLeft, int64_t deltaTime) {
	maebot_pose_t nextPose = eecs467::advanceState(pose,
		deltaRight, deltaLeft, deltaTime);

	float deltaX = nextPose.x - pose.x;
	float deltaY = nextPose.y - pose.y;
	float deltaTheta = nextPose.theta - pose.theta;

	float deltaS = sqrt(deltaX * deltaX + deltaY * deltaY);
	float alpha = atan2(deltaY, deltaX) - pose.theta;

	// getting noise
	float e1 = _normDist(_randGen) * alpha * _k1;
	float e2 = _normDist(_randGen) * deltaS * _k2;
	float e3 = _normDist(_randGen) * (deltaTheta - alpha) * _k1;

	pose.x += (deltaS + e2)* cos(pose.theta + alpha + e1);
	pose.y += (deltaS + e2)* sin(pose.theta + alpha + e1);
	pose.theta += deltaTheta + e1 + e3;
}
