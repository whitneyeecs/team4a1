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
	_random_samples.reserve(eecs467::numParticles);
	_post_action.reserve(eecs467::numParticles);

	maebot_particle_t particle;
	particle.pose.x = 0;
	particle.pose.y = 0;
	particle.pose.theta = 0;

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
	std::sort(_random_samples.begin(), _random_samples.end(), sort);

	float scale = _random_samples.back().prob;
	float weight = 0.0;

	//scale to zero
	for(int i = 0; i < eecs467::numParticles; ++i){
		_random_samples[i].prob += -1.0 * scale;
		_random_samples[i].prob = exp(_random_samples[i].prob/100.0);
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

printf("process\n");
	drawRandomSamples();
printf("size: %ld\n", _random_samples.size());
	int64_t laserTime = _scan.times[_scan.num_ranges - 1];
	std::array<int32_t, 2> interpolate = _odo.interpolate(laserTime);
	std::array<int32_t, 2> deltas = _odo.deltas(interpolate);
	_odo.set(interpolate, laserTime);

	for (auto& particle : _random_samples) {
		maebot_pose_t oldPose = particle.pose;
printf("before: %f, %f, %f\n", oldPose.x, oldPose.y, oldPose.theta);
		_actionModel.apply(particle.pose, deltas[0], deltas[1], laserTime);
printf("after: %f, %f, %f\n", particle.pose.x, particle.pose.y, particle.pose.theta);
		// _sensorModel.apply(particle, _scan, oldPose);
	}

	normalizeAndSort();

	_hasScan = false;
	_processing = false;
};







