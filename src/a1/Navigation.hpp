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

	//size 2. [0] = origin, [1] = dest
	std::vector<Point<float>> _line;

	std::vector<Point<float>> _crumbs;	

	maebot_pose_t _pose;
	maebot_motor_feedback_t _odo;

	pthread_mutex_t _cmdLock;

	//used for control
	float _prev_error;

private:

void turn(float angle, lcm::LCM& lcm);

public:

Navigation();

// pushOdometry()
//pushes most current odometery
//
void pushOdometry(const maebot_motor_feedback_t& odo) { _odo = odo; }


// pushPose()
//pushes most probable pose after every particle filter iteration
//this pose will be used for navigating
//
void pushPose(const maebot_pose_t& pose) { _pose = pose; }

// driving()
//returns true when destination has been set
//but not met
//
bool driving(){ return _driving; }

// driveTo()
//primary function. drives straight line from 
//current position to dest
//does not check to see if it is traversable
//
void driveTo(Point<float> dest, lcm::LCM& lcm);

// correct()
//implements a line following feedback control
//stops when target is reached
void correct(lcm::LCM& lcm);


};//end class
}//end namespace

#endif
