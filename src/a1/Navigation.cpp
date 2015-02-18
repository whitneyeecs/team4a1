#include "Navigation.hpp"
#include <unistd.h>
//#include <cmath.h>


eecs467::Navigation::Navigation(): 
	_driving(false),
	_turning(false),
	_prev_error(0.0) { 
	_line.resize(2);
}

void eecs467::Navigation::driveTo(Point<float> dest){


	//add current location to bread-crumb path
	Point<float> cur_spot;
	cur_spot.x = _pose.x;
	cur_spot.y = _pose.y;
	
	//store endpoints of current line
	
	_line[0] = cur_spot;
	_line[1] = dest;
	//turn to orient bot with destination

	return ;

}

maebot_motor_command_t eecs467::Navigation::correct(bool& driving){
	
	maebot_motor_command_t cmd;
printf("\n\n\n\nentering correct\n\n\n\n\n\n");
/*
if(_line[1].x == 0.0 && _line[1].y == 0.0){
printf("BAD\n");
		cmd.motor_left_speed = go*0.9;
		cmd.motor_right_speed = go*0.9;
		return cmd;
}
*/
	Point<float> cur;
	cur.x = _pose.x;
	cur.y = _pose.y;
	float dist_to_target = distance_between_points(cur, _line[1]);

printf("\n\ndistance to target:\t%f\n", dist_to_target);
printf("cur x:\t%f\tcur y:\t%f\n", cur.x, cur.y);
printf("target x:\t%f\ttarget y:\t%f\n", _line[1].x, _line[1].y);
	
	if(dist_to_target <= target_radius){
		cmd.motor_left_speed = stop;
		cmd.motor_right_speed = stop;
		_line.clear();
		driving = false;
printf("exiting correct with target met\n");
		return cmd;
	}

	float target_angle;
	target_angle = atan2(_line[1].y - _pose.y, _line[1].x - _pose.x);
printf("pose angle:\t%f\n", _pose.theta);
printf("target angle:\t%f\n", target_angle);

	float correct = target_angle - _pose.theta;

	cmd.motor_left_speed = go;
	cmd.motor_right_speed = go * (1.0  + correct * correct_scale);
	if(correct > .25){
		cmd.motor_right_speed = go * 0.7;
		cmd.motor_left_speed = go * -0.7;
	}
	if(correct  < -0.25){
		cmd.motor_right_speed = go * -0.7;
		cmd.motor_left_speed = go * 0.7;
	}
printf("correct is:\t%f\n", correct);
printf("left motor:\t%f\tright motor:\t%f\n\n\n",cmd.motor_left_speed,cmd.motor_right_speed); 
	
	return cmd;


}






















