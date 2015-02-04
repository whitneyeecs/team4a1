#include "ParticleFilter.hpp"
#include "RobotConstants.hpp"

#include "math/gsl_util_rand.h"
#include "math/angle_functions.hpp"
#include "math/point.hpp"

eecs467::ParticleFilter::ParticleFilter() { }

eecs467::ParticleFilter::ParticleComp sort;

void eecs467::ParticleFilter::pushMap(
	const eecs467::OccupancyGrid& map){
	
	_map = map;
}


void eecs467::ParticleFilter::init(const int64_t utime){
	maebot_particle_t  particle;
	Point<float> point;
	float theta;
	
//	int32_t seed = gslu_rand_seed();
	gsl_rng * r = gslu_rand_rng_alloc();
	
	//generate random particles, if valid
	//add to _prior
	for(int i = 0; i < eecs467::numParticles; ++i){
		
		//generate point in grid coordinates
		point.x = gslu_rand_uniform_pos(r) * _map.widthInMeters();
		point.y = gslu_rand_uniform_pos(r) * _map.heightInMeters();
		theta = wrap_to_pi(2 * M_PI * gslu_rand_uniform(r));

		//convert to global coordinates
		point = grid_position_to_global_position(point, _map);

printf("x: %f\ty: %f\ttheta: %f\n", point.x, point.y, theta);
	
		//find grid cell that point is in
		Point<int> cell = global_position_to_grid_cell(
			point, _map);
		
		//if grid cell is occupied discard and try again
		if(_map.logOdds(cell.x, cell.y) < 120){
			--i;
			continue;
		}
		
		particle.pose.utime = utime;
		particle.pose.x = point.x;
		particle.pose.y = point.y;
		particle.pose.theta = theta;
		particle.prob = 0.0;

		_prior.push_back(particle);

	}//end for
}


void eecs467::ParticleFilter::pushOdometry(maebot_motor_feedback_t& odometry){
	_odo.update(odometry, _scan.utime);
}

void eecs467::ParticleFilter::pushScan(maebot_laser_scan_t& scan){
	_scan = scan;
}

void eecs467::ParticleFilter::drawRandomSamples(){

	gsl_rng * r = gslu_rand_rng_alloc();
	
	float target = 0.0;
	float weight = 0.0;
	int j = 0;

	for(int i = 0; i < eecs467::numParticles; ++i){
		target = gslu_rand_uniform(r);
		for( ; weight < target; ++j){
			weight += _prior[j].prob;
		}
		_random_samples[i] = _prior[j];
		j = 0;
		weight = 0.0;
		_random_samples[i].prob = 0.0;
	}
}

void eecs467::ParticleFilter::normalizeAndSort(){

	std::sort(_post_action.begin(), _post_action.end(), sort);
	
	float weight = 0.0;

	//get total probability
	//
	for(int i = 0; i < numParticles; ++i){
		weight += _post_action[i].prob;
	}

	//normalize
	//
	for(int i = 0; i < numParticles; ++i){
		_post_action[i].prob /= weight;
	}
	
}

void eecs467::ParticleFilter::actionModel(maebot_particle_t& particle) {

}


















