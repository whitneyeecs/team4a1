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
	_actionModel(eecs467::actionModelK1, eecs467::actionModelK2),
	_hasMap(false), _processing(false), _hasScan(false) { }

eecs467::ParticleFilter::ParticleComp sort;

void eecs467::ParticleFilter::pushMap(
	const eecs467::OccupancyGrid& map) { 
	_sensorModel.pushMap(map);
	_hasMap = true;
}

void eecs467::ParticleFilter::init(const maebot_motor_feedback_t* msg){
	_prior.reserve(eecs467::numParticles);
	_random_samples.resize(eecs467::numParticles);


	gsl_rng * r = gslu_rand_rng_alloc();


	maebot_particle_t particle;
	particle.pose.x = 0;
	particle.pose.y = 0;
	particle.pose.theta = 0;
	particle.prob = 1.0/eecs467::numParticles;

	for (int i = 0; i < eecs467::numParticles; ++i) {
		_prior.push_back(particle);
	}

	_odo.set(*msg);
}

void eecs467::ParticleFilter::pushOdometry(const maebot_motor_feedback_t& odometry){
	// push with last laser timestamp
	_odo.update(odometry);
}

void eecs467::ParticleFilter::pushScan(const maebot_laser_scan_t& scan){
	_scan = scan;
	_hasScan = true;
}

void eecs467::ParticleFilter::drawRandomSamples(){
	gsl_rng * r = gslu_rand_rng_alloc();
	
	float target = 0.0;
	float weight = 0.0;
	int j = 0;

	for(int i = 0; i < eecs467::numParticles; ++i){
		target = gslu_rand_uniform(r);

// printf("target is: %f\n", target);
		for( ; weight < target && j < eecs467::numParticles; ++j){
			weight += _prior[j].prob;
		}
		if(j >= eecs467::numParticles)
				--j;
// printf("drawing from #: %i\n\n", j);
		_random_samples[i] = _prior[j];
		j = 0;
		weight = 0.0;
//		_random_samples[i].prob = 0.0;
	}

//		for(int i = 0; i< eecs467::numParticles; ++i){
//			_random_samples[i] = _prior[i];
//
//		}	
}

void eecs467::ParticleFilter::normalizeAndSort(){
	std::sort(_random_samples.begin(), _random_samples.end(), sort);
if(_random_samples.front().prob < _random_samples.back().prob)
		exit(4);
	float scale = _random_samples.front().prob;
	float weight = 0.0;

	//scale to zero
	for(int i = 0; i < eecs467::numParticles; ++i){
		_random_samples[i].prob += -1.0 * scale;
		_random_samples[i].prob = exp(_random_samples[i].prob);
	}

	//get total probability
	//
	for(int i = 0; i < eecs467::numParticles; ++i){
		weight += _random_samples[i].prob;
	}

	//normalize
	//
	for(int i = 0; i < eecs467::numParticles; ++i){
		_random_samples[i].prob /= weight;
		_prior[i] = _random_samples[i];
printf("new prob: %f\n", _prior[i].prob);
	}
}

maebot_particle_map_t
eecs467::ParticleFilter::toLCM(){
	maebot_particle_map_t msg;
	msg.utime = _prior.front().pose.utime;
	msg.grid = _sensorModel.getGrid().toLCM();
	msg.num_particles = (int32_t)_prior.size();
	msg.particles = _prior;
	return msg;
}

void eecs467::ParticleFilter::process() {
	_processing = true;

	drawRandomSamples();
	int64_t laserTime = _scan.times[_scan.num_ranges - 1];
	std::array<int32_t, 2> interpolate = _odo.interpolate(laserTime);
	std::array<int32_t, 2> deltas = _odo.deltas(interpolate);
	_odo.set(interpolate, laserTime);

	for (auto& particle : _random_samples) {
		maebot_pose_t oldPose = particle.pose;
		if(particle.prob < (0.90 * 1.0/eecs467::numParticles)){
printf("particle prob in random samples: %f\n", particle.prob);
			_actionModel.apply(particle.pose, deltas[0], deltas[1], laserTime,1);

		} else{
			_actionModel.apply(particle.pose, deltas[0], deltas[1], laserTime,0);
		}
		particle.prob = 0.0;
		_sensorModel.apply(particle, _scan, oldPose);
	}

	normalizeAndSort();

	_hasScan = false;
	_processing = false;
};







