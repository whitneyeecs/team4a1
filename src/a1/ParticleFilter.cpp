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
	_hasMap(false), _processing(false), _hasScan(false) {
	randGen = gslu_rand_rng_alloc();
}

eecs467::ParticleFilter::ParticleComp sort;

void eecs467::ParticleFilter::pushMap(
	const eecs467::OccupancyGrid& map) { 
	_sensorModel.pushMap(map);
	_hasMap = true;
}

void eecs467::ParticleFilter::init(const maebot_motor_feedback_t* msg) {
	_prior.reserve(eecs467::numPreviousParticles + eecs467::numRandomParticles);
	_random_samples.resize(eecs467::numPreviousParticles + eecs467::numRandomParticles);

	maebot_particle_t particle;
	particle.pose.x = 0;
	particle.pose.y = 0;
	particle.pose.theta = 0;
	particle.prob = 1.0 / eecs467::numPreviousParticles;

	for (int i = 0; i < eecs467::numPreviousParticles + eecs467::numRandomParticles; ++i) {
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
	maebot_particle_t mostProbable = _prior.front();

	float target = 0.0;
	float weight = 0.0;
	int j = 0;

	for (int i = 0; i < eecs467::numPreviousParticles; ++i) {
		target = gslu_rand_uniform(randGen);

		for ( ; weight < target && j < eecs467::numPreviousParticles; ++j) {
			weight += _prior[j].prob;
		}
		if (j >= eecs467::numPreviousParticles) {
			--j;
		}
		_random_samples[i] = _prior[j];
		j = 0;
		weight = 0.0;
	}

	for (int i = 0; i < eecs467::numRandomParticles; ++i) {
		float randX = (gslu_rand_uniform(randGen) - 0.5) * 0.05;
		float randY = (gslu_rand_uniform(randGen) - 0.5) * 0.05;
		float randTheta = (gslu_rand_uniform(randGen) - 0.5) * 0.01;

		maebot_particle_t newParticle = mostProbable;
		newParticle.pose.x += randX;
		newParticle.pose.y += randY;
		newParticle.pose.theta += randTheta;

		_random_samples[eecs467::numPreviousParticles + i] = newParticle;
	}
}

void eecs467::ParticleFilter::normalizeAndSort(){
	std::sort(_random_samples.begin(), _random_samples.end(), sort);
if(_random_samples.front().prob < _random_samples.back().prob)
		exit(4);
	float scale = _random_samples.front().prob;
	float weight = 0.0;

	//scale to zero
	for(int i = 0; i < eecs467::numPreviousParticles; ++i){
		_random_samples[i].prob += -1.0 * scale;
		_random_samples[i].prob = exp(_random_samples[i].prob);
	}

	//get total probability
	//
	for(int i = 0; i < eecs467::numPreviousParticles; ++i){
		weight += _random_samples[i].prob;
	}

	//normalize
	//
	for(int i = 0; i < eecs467::numPreviousParticles; ++i){
		_random_samples[i].prob /= weight;
		_prior[i] = _random_samples[i];
// printf("new prob: %f\n", _prior[i].prob);
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

	// precomputing deltaS to save computation!
	float deltaS = eecs467::metersPerTick * (float)(deltas[0] + deltas[1]) / 2.0f;

	for (auto& particle : _random_samples) {
		maebot_pose_t oldPose = particle.pose;
		_actionModel.apply(particle.pose, deltas[0], deltas[1], deltaS, laserTime);
		// _actionModel.apply(particle.pose, deltas[0], deltas[1], laserTime);
		_sensorModel.apply(particle, _scan, oldPose);
	}

	normalizeAndSort();

	_hasScan = false;
	_processing = false;
};







