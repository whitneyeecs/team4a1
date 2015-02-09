#include <stdio.h>
#include <stdint.h>

#include "a1/VirtualOdometry.hpp"
#include "a1/StateEstimator.hpp"
#include "a1/ActionModel.hpp"

void VirtualOdometryTest();
void StateEstimatorTest();
void ActionModelTest();

int main() {
	VirtualOdometryTest();
	StateEstimatorTest();
	ActionModelTest();
}

void VirtualOdometryTest() {
	// printf("Odometry Test\n");
	// eecs467::VirtualOdometry vo(0, 0, 0);

	// for (int i = 1; i <= 100; ++i) {
	// 	vo.update(i * 10, i * 20, i * 10, i * 10 - 2);
	// }

	// printf("%d\t%d\t%ld\n", vo.getDeltaRight(), vo.getDeltaLeft(), vo.getUtime());

	// printf("%d\t%d\t%ld\n", vo.getRightTicks(), vo.getLeftTicks(), vo.getUtime());
}

void StateEstimatorTest() {
	printf("State Estimator Test\n");
	maebot_pose_t pose = { 0, 0, 0, 0 };
	
	maebot_pose_t newPose = eecs467::advanceState(pose, 100, 200, 100);
	printf("%f, %f, %f, %ld\n", newPose.x, newPose.y, newPose.theta, newPose.utime);
}

void ActionModelTest() {
	printf("Action Model Test\n");
	eecs467::ActionModel model(0.05, 0.05);

	maebot_pose_t pose = { 0, 0, 0, 0 };
	model.apply(pose, 100, 102, 1000);
	printf("%f, %f, %f, %ld\n", pose.x, pose.y, pose.theta, pose.utime);
}

