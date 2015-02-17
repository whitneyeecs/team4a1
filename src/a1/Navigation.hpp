#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "a1/RobotConstants.hpp"
#include "a1/SlamConstants.hpp"

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"

#include "math/angle_functions.hpp"
#include "math/point.hpp"
#include <cmath>

namespace eecs467{

class Navigation{
private:
	bool _driving;
	bool _turning;

	//size 2. [0] = origin, [1] = dest
	std::vector<Point<float>> _line;

	std::vector<Point<float>> _crumbs;	

	maebot_pose_t _pose;
	maebot_motor_feedback_t _odo;


	//used for control
	float _prev_error;


public:

Navigation();

// pushOdometry()
//pushes most current odometery
//
void pushOdometry(const maebot_motor_feedback_t& odo) { _odo = odo;}// printf("pushed odemetry\n"); }


// pushPose()
//pushes most probable pose after every particle filter iteration
//this pose will be used for navigating
//
void pushPose(const maebot_pose_t& pose) { _pose = pose; printf("pushed pose, theta is:\t%f\tx:\t%f\ty:\t%f\n", pose.theta, pose.x, pose.y); }

bool driving(){ return _driving; }

void driveTo(Point<float> dest);

maebot_motor_command_t correct(bool& driving);

bool turning(){ return _turning; }

};//end class
}//end namespace

#endif
