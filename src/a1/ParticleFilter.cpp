#include "a1/ParticleFilter.hpp"
#include "a1/RobotConstants.hpp"
#include "a1/SlamConstants.hpp"
#include "a1/LaserCorrector.hpp"

#include "math/gsl_util.h"
#include "math/gsl_util_rand.h"
#include "math/angle_functions.hpp"
#include "math/point.hpp"

#include <cmath>

eecs467::ParticleFilter::ParticleComp sort;

eecs467::ParticleFilter::ParticleFilter() :
	_actionModel(eecs467::actionModelK1, eecs467::actionModelK2),
	_hasMap(false), _processing(false), _hasScan(false) {
	randGen = gslu_rand_rng_alloc();
}

void eecs467::ParticleFilter::pushMap(const eecs467::OccupancyGrid* map) { 
	_sensorModel.pushMap(map);
	_hasMap = true;
}

void eecs467::ParticleFilter::init(const maebot_motor_feedback_t* msg) {
	int totalNumParticles = eecs467::numPreviousParticles 
		+ eecs467::numRandomParticles;
	_prior.reserve(totalNumParticles);
	_random_samples.resize(totalNumParticles);

	maebot_particle_t particle;
	particle.pose.x = 0;
	particle.pose.y = 0;
	particle.pose.theta = 0;
	particle.prob = 1.0 / totalNumParticles;

	for (int i = 0; i < totalNumParticles; ++i) {
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

maebot_particle_map_t
eecs467::ParticleFilter::toLCM(){
	maebot_particle_map_t msg;
	msg.utime = _prior.front().pose.utime;
	msg.grid = _sensorModel.getGrid()->toLCM();
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

	bool move = true;
	if (deltas[0] == 0 && deltas[1] == 0) {
		move = false;
	}

	// precomputing deltaS to save computation!
	float deltaS = eecs467::metersPerTick * (float)(deltas[0] + deltas[1]) / 2.0f;

	for (auto& particle : _random_samples) {
		maebot_pose_t oldPose = particle.pose;

		if (move) {
			_actionModel.apply(particle.pose,
					deltas[0], deltas[1],
					deltaS, laserTime);
		} else {
			particle.pose.utime = laserTime;
		}

#ifdef SENSOR_RAY_TRACE
		_sensorModel.applyRayTrace(particle, _scan,
			oldPose, particle.pose);
#else
		_sensorModel.applyEndPoints(particle, _scan,
			oldPose, particle.pose);
#endif /* SENSOR_RAY_TRACE */
	}

	normalizeAndSort(deltas, laserTime);

	_hasScan = false;
	_processing = false;
};

maebot_pose_t eecs467::ParticleFilter::getBestPose(){
	return _prior.front().pose;
}

const maebot_laser_scan_t* eecs467::ParticleFilter::getScan() const {
	return &_scan;
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
#ifdef CIRCLE_RANDOM_DRAW_PARTICLES
		float randRadius = gslu_rand_uniform(randGen) 
			* eecs467::randParticleSpread;
		float randRadiusTheta = gslu_rand_uniform(randGen) * M_PI * 2;
		float randX = randRadius * cos(randRadiusTheta);
		float randY = randRadius * sin(randRadiusTheta);
#else
		float randX = (gslu_rand_uniform(randGen) - 0.5) 
			* eecs467::randParticleSpread;
		float randY = (gslu_rand_uniform(randGen) - 0.5) 
			* eecs467::randParticleSpread;
#endif /* CIRCLE_RANDOM_DRAW_PARTICLES */

		float randTheta = (gslu_rand_uniform(randGen) - 0.5) * 0.01;

		maebot_particle_t newParticle = mostProbable;
		newParticle.pose.x += randX;
		newParticle.pose.y += randY;
		newParticle.pose.theta += randTheta;

		_random_samples[eecs467::numPreviousParticles + i] = newParticle;
	}
}

void eecs467::ParticleFilter::normalizeAndSort(const std::array<int32_t, 2>& deltas, 
	int64_t utime){
	std::sort(_random_samples.begin(), _random_samples.end(), sort);

	int totalNumParticles = eecs467::numPreviousParticles +
		eecs467::numRandomParticles;

	float scale = _random_samples.front().prob;
	float weight = 0.0;

	//scale to zero
	for(int i = 0; i < totalNumParticles; ++i){
		_random_samples[i].prob += -1.0 * scale;
		_random_samples[i].prob = exp(_random_samples[i].prob);
	}

	//get total probability
	for(int i = 0; i < totalNumParticles; ++i){
		weight += _random_samples[i].prob;
	}

	//normalize
	for(int i = 0; i < totalNumParticles; ++i){
		_random_samples[i].prob /= weight;
		_prior[i] = _random_samples[i];
	}
}
