#include "a1/ParticleFilter.hpp"
#include "a1/RobotConstants.hpp"
#include "a1/SlamConstants.hpp"
#include "a1/LaserCorrector.hpp"

#include "math/gsl_util.h"
#include "math/gsl_util_rand.h"
#include "math/angle_functions.hpp"
#include "math/point.hpp"

#include <cmath>

eecs467::ParticleFilter::ParticleFilter() :
	_actionModel(eecs467::actionModelK1, eecs467::actionModelK2) { }

eecs467::LaserCorrector pf_laser;

eecs467::ParticleFilter::ParticleComp sort;

void eecs467::ParticleFilter::pushMap(
	const eecs467::OccupancyGrid& map){
	
	_map = map;
}


void eecs467::ParticleFilter::init(const maebot_motor_feedback_t* msg){
	maebot_particle_t  particle;
	Point<float> point;
	float theta;
	
//	int32_t seed = gslu_rand_seed();
	gsl_rng * r = gslu_rand_rng_alloc();
	
	//generate random particles, if valid
	//add to _prior
	for(int i = 0; i < eecs467::numParticles; ++i){
		
		//generate point in global coordinates
		//used 1.99 instead of 2. out of paranoia that random draw right
		//on edge of map would cause seg fault. 
		point.x = gslu_rand_uniform_pos(r) * _map.widthInMeters() - _map.widthInMeters() / 2.0;
		point.y = gslu_rand_uniform_pos(r) * _map.heightInMeters() - _map.heightInMeters() / 2.0;
		theta = wrap_to_pi(2 * M_PI * gslu_rand_uniform(r));
/*		point.x = (gslu_rand_uniform_pos(r) * _map.widthInMeters(); * 1.99)
					- _map.widthInMeters();
		point.y = (gslu_rand_uniform_pos(r) * _map.heightInMeters() * 1.99)
					- _map.widthInMeters();
		theta = wrap_to_pi(2 * M_PI * gslu_rand_uniform(r));
*/

//printf("#: %d\t x: %f\ty: %f\ttheta: %f\n", i, point.x, point.y, theta);


	
		//find grid cell that point is in
		Point<int> cell = global_position_to_grid_cell(
			point, _map);

//printf("Grid cell: %i, %i\n", cell.x, cell.y);
//printf("Grid cell logOdds: %i\n", _map.logOdds(cell.x, cell.y));


		//if grid cell is occupied discard and try again
		if(_map.logOdds(cell.x, cell.y) > -120){
			--i;
			continue;
		}
		

		particle.pose.utime = msg->utime;
		particle.pose.x = point.x;
		particle.pose.y = point.y;
		particle.pose.theta = theta;
		particle.l_ticks = msg->encoder_left_ticks;
		particle.r_ticks = msg->encoder_right_ticks;
		particle.prob = 1.0 / eecs467::numParticles;
//printf("about to push\n");

		_prior.push_back(particle);
//printf("pushed\n\n");
	}//end for
	
	_random_samples.resize(eecs467::numParticles);
	_post_action.resize(eecs467::numParticles);
}


void eecs467::ParticleFilter::pushOdometry(const maebot_motor_feedback_t& odometry){
	_odo.update(odometry, _scan.utime);
	_odometry = odometry;
}

void eecs467::ParticleFilter::pushScan(const maebot_laser_scan_t& scan){
	_scan = scan;
	_scan_to_process = &_scan;
}

void eecs467::ParticleFilter::drawRandomSamples(){

	gsl_rng * r = gslu_rand_rng_alloc();
	
	float target = 0.0;
	float weight = 0.0;
	int j = 0;

	for(int i = 0; i < eecs467::numParticles; ++i){
		target = gslu_rand_uniform(r);

//printf("target is: %f\n\n", target);

		for( ; weight < target && j < eecs467::numParticles; ++j){
			weight += _prior[j].prob;
		}
		if(j >= eecs467::numParticles)
				--j;
//printf("adding sample #: %i\n", i);
		_random_samples[i] = _prior[j];
		j = 0;
		weight = 0.0;
//		_random_samples[i].prob = 0.0;
	}
}

void eecs467::ParticleFilter::normalizeAndSort(){

	std::sort(_post_action.begin(), _post_action.end(), sort);

	float scale = _post_action.back().prob;
	float weight = 0.0;

	//scale to zero
	for(int i = 0; i < eecs467::numParticles; ++i){
		_post_action[i].prob += -1.0 * scale;
		_post_action[i].prob = exp(_post_action[i].prob/100.0);
	}

	//get total probability
	//
	for(int i = 0; i < eecs467::numParticles; ++i){
		weight += _post_action[i].prob;
	}

	//normalize
	//
	for(int i = 0; i < eecs467::numParticles; ++i){
		_post_action[i].prob /= weight;
		_prior[i] = _post_action[i];
	}
	
}

maebot_particle_map_t
eecs467::ParticleFilter::toLCM(){
	
	maebot_particle_map_t msg;
	msg.utime = _prior.front().pose.utime;
	msg.grid = _map.toLCM();
	msg.num_particles = (int32_t)_prior.size();
	msg.particles = _prior;
	return msg;
}

void eecs467::ParticleFilter::actionModel(maebot_pose_t& pose, 
		int32_t delta_l, int32_t delta_r){
	
	//quick and dirty action model, problems with other one
	//
	float delta_sl = (float)delta_l * eecs467::metersPerTick;
	float delta_sr = (float)delta_r * eecs467::metersPerTick;

	float delta_s = (delta_sl + delta_sr) / 2.0;
	float delta_theta = (delta_sr - delta_sl) / eecs467::baseLength;


	//add random-ness
	//
	gsl_rng * r = gslu_rand_rng_alloc();
	
	delta_s *= (1.0 + 1.25 * gslu_rand_normal(r));
	delta_theta *= (1.0 + 0.15 * gslu_rand_normal(r));
	//use this for aproximation update to x and y
	float alpha = pose.theta + (delta_theta / 2.0);

	pose.theta += delta_theta;
	pose.x += delta_s*cos(alpha);
	pose.y += delta_s*sin(alpha);

	return;
	
}


float eecs467::ParticleFilter::getProb(const maebot_processed_laser_scan_t& msg){
	
	float new_prob = 0.0;

	Point<float> end;
	Point<float> start;

	for(int i = 0; i < msg.num_ranges; ++i){
		start.x = msg.x_pos[i];
		start.y = msg.y_pos[i];
		
		end.x = msg.x_pos[i] + msg.ranges[i]*cos(msg.thetas[i]);
		end.y = msg.y_pos[i] + msg.ranges[i]*sin(msg.thetas[i]);

		Point<int> cell = global_position_to_grid_cell(end, _map);
		Point<int> origin = global_position_to_grid_cell(start, _map);
//printf("end.x: %f\tend.y: %f\n", end.x, end.y);
//printf("the odds for the endpoint are: %i\n", _map.logOdds(cell.x, cell.y));
//
		if(!_map.isCellInGrid(origin.x, origin.y)){
			new_prob -= 30;
			
		}

		if(!_map.isCellInGrid(cell.x, cell.y))
				new_prob -= 14;
		else if(_map.logOdds(cell.x, cell.y) > 120) // wall
				new_prob -= 4;
		else if(_map.logOdds(cell.x, cell.y) < -120) 
				new_prob -= 8;
		else
				new_prob -= 12;
	}

	
	return new_prob;
}

void eecs467::ParticleFilter::process(){
	
	_processing = true;
	
	drawRandomSamples();
	
	int64_t new_time;
	
	if(_scan.times[0] > _scan.times[_scan.num_ranges -1])
	 	new_time = _scan.times[0];
	else
		new_time = _scan.times[_scan.num_ranges -1];

	float scaling =(float) (new_time - _random_samples[0].pose.utime)/
					(float)(_odometry.utime - _random_samples[0].pose.utime);

//printf("scaling factor: %f\n", scaling);

	int32_t delta_l = _odometry.encoder_left_ticks - _random_samples[0].l_ticks;
	int32_t delta_r = _odometry.encoder_right_ticks - _random_samples[0].r_ticks;
	delta_l = (int)((float)delta_l * scaling);
	delta_r = (int)((float)delta_r * scaling);
	int64_t delta_t = new_time - _random_samples[0].pose.utime; 



//printf("delta left: %i\tdelta right: %i\tdelta time: %li\n\n", delta_l, delta_r, delta_t);

	for(int i = 0; i < eecs467::numParticles; ++i){
			_post_action[i] = _random_samples[i];
			_post_action[i].pose.utime = new_time;
			_post_action[i].l_ticks += delta_l;
			_post_action[i].r_ticks += delta_r;


//			_actionModel.apply(_post_action[i].pose, delta_r, delta_l, delta_t);


			actionModel(_post_action[i].pose, delta_l, delta_r);


//printf("prior x: %f\tprior y: %f\tprior theta: %f\n", _prior[i].pose.x, _prior[i].pose.y,_prior[i].pose.theta);
//printf("random x: %f\trandom y: %f\trandom theta: %f\n", _random_samples[i].pose.x, _random_samples[i].pose.y, _random_samples[i].pose.theta);
//printf("post x:  %f\tpost y:  %f\tpost theta: %f\n", _post_action[i].pose.x, _post_action[i].pose.y, _post_action[i].pose.theta);

			pf_laser.pushNewPose(_random_samples[i].pose);
			pf_laser.pushNewPose(_post_action[i].pose);

			pf_laser.pf_process(_scan_to_process);

			maebot_processed_laser_scan_t scans;
			pf_laser.getCorrectedLcmMsg(scans);
			pf_laser.clearPoses();
			pf_laser.clearScans();
printf("The old prob is %f\n", _post_action[i].prob);			
			_post_action[i].prob = getProb(scans);
printf("The new prob is %f\n\n", _post_action[i].prob);
	}
	
	normalizeAndSort();
	_scan_to_process = NULL;
	_processing = false;

printf("there are now %i particles in prior\n", _prior.size());
//exit(0);
};







