#include "Navigation.hpp"

eecs467::Navigation::Navigation(): 
	_driving(false),
	_prev_error(1000.0)
	{ 
	_crumbs.resize(8); 
	_line.resize(2);
	}

void eecs467::Navigation::driveTo(Point<float> dest, lcm::LCM& lcm){

	//let everyone know that we are moving
	_driving = true;

	//add current location to bread-crumb path
	Point<float> cur_spot;
	cur_spot.x = _pose.x;
	cur_spot.y = _pose.y;
	_crumbs.push_back(cur_spot);
	
	//store endpoints of current line
	
	_line.push_back(cur_spot);
	_line.push_back(dest);
	
	//turn to orient bot with destination
	float path_angle = atan2(dest.y - _pose.x, dest.x - _pose.x);
	float angle_to_turn = angle_diff(path_angle,  _pose.theta);
	turn(angle_to_turn, lcm);

	//begin moving. correct will be called with pose 
	//updates and will be resposible for stopping
	maebot_motor_command_t cmd;
	cmd.motor_left_speed = go;
	cmd.motor_right_speed = go;
	pthread_mutex_lock (&_cmdLock);
	lcm.publish("MAEBOT_MOTOR_COMMAND", &cmd);
	pthread_mutex_unlock (&_cmdLock);


}

void eecs467::Navigation::turn(float angle, lcm::LCM& lcm){

	//decide if turning left or right
	float sign;
	if(angle > 0.0)
			sign = -1.0;
	else
			sign = 1.0;

	//determine the distance in ticks that each wheel must 
	//travel
	float delta_meters = circumference * angle / (2.0 * M_PI);
	int32_t delta_ticks = delta_meters / metersPerTick;
	int32_t ticks = _odo.encoder_left_ticks;

	//begin turning. probably want to do this a bit
	//slower than the forward motion speed
	maebot_motor_command_t cmd;
	cmd.motor_left_speed = sign * go * 0.7f;
	cmd.motor_right_speed = -1.0 * sign * go * 0.7f;
	pthread_mutex_lock (&_cmdLock);
	lcm.publish("MAEBOT_MOTOR_COMMAND", &cmd);
	pthread_mutex_unlock (&_cmdLock);

	//stop when target is met
	while(abs(_odo.encoder_left_ticks - ticks) < delta_ticks){

	}
	cmd.motor_left_speed = stop;
	cmd.motor_right_speed = stop;
	pthread_mutex_lock (&_cmdLock);
	lcm.publish("MAEBOT_MOTOR_COMMAND", &cmd);
	pthread_mutex_unlock (&_cmdLock);
	
}

void eecs467::Navigation::correct(lcm::LCM& lcm){

	maebot_motor_command_t cmd;

	//calculate distance to target and stop if close enough
	//
	Point<float> cur;
	cur.x = _pose.x;
	cur.y = _pose.y;
	float dist_to_target = distance_between_points(cur, _line[1]);

	if(dist_to_target <= target_radius){
		cmd.motor_left_speed = stop;
		cmd.motor_right_speed = stop;
		pthread_mutex_lock (&_cmdLock);
		lcm.publish("MAEBOT_MOTOR_COMMAND", &cmd);
		pthread_mutex_unlock (&_cmdLock);
		_line.clear();
		return;
	}

	//calculate the distance to the line
	std::vector<float> a;
	std::vector<float> b;
	a.push_back(_line[1].x - _line[0].x);
	a.push_back(_line[1].y - _line[0].y);
	b.push_back(cur.x - _line[0].x);
	b.push_back(cur.y - _line[0].y);
	
	//dot product
	float scale = (b[0]*a[0] + b[1]*a[1]) / (a[0]*a[0] + a[1]*a[1]);

	Point<float> proj_point;
	proj_point.x = _line[0].x + scale * _line[1].x;
	proj_point.y = _line[0].y + scale * _line[1].y;

	float distance_to_line = distance_between_points(proj_point, cur);

	//calculate which side of the line you are on
	float side_of_line = (_line[1].x - _line[0].x)*(cur.y - _line[0].y)
							-(cur.x - _line[0].x)*(_line[1].y - _line[0].y);


	float error;
	if(side_of_line > 0.0){
		error = -1 * distance_to_line;
	}else if(side_of_line < 0.0){
		error = distance_to_line;
	}else{
		error = 0.0;
	}	

	float correct = Kp*error + Kd*(error - _prev_error);
	_prev_error = error;

	cmd.motor_left_speed = go;
	cmd.motor_right_speed = go * (1 + correct) * correct_scale;

	pthread_mutex_lock (&_cmdLock);
	lcm.publish("MAEBOT_MOTOR_COMMAND", &cmd);
	pthread_mutex_unlock (&_cmdLock);


}






















