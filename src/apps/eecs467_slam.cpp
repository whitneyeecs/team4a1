#include <unistd.h>

// c++
#include <vector>
#include <list>
#include <iostream>

// lcm
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/maebot_occupancy_grid_t.hpp>
#include <lcmtypes/maebot_motor_feedback_t.hpp>
#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_map_data_t.hpp>
#include "mapping/occupancy_grid.hpp"

#include "a1/LaserCorrector.hpp"
#include "a1/ParticleFilter.hpp"
#include "a1/Mapper.hpp"


class StateHandler {
public:
	eecs467::LaserCorrector laser;
	eecs467::ParticleFilter pf;
	eecs467::Mapper mapper;
	pthread_mutex_t dataMutex;

	lcm::LCM lcm;

public:
	StateHandler() : mapper(1, 5, 5, 0.05) {
		if (!lcm.good()) {
			printf("lcm unable to initialize\n");
			exit(1);
		}

		if (pthread_mutex_init(&dataMutex, NULL)) {
			printf("data mutex not initialized\n");
			exit(1);
		}

		lcm.subscribe("MAEBOT_LASER_SCAN", &StateHandler::handleLaserMessage, this);
		lcm.subscribe("MAEBOT_MOTOR_FEEDBACK", &StateHandler::handleMotorFeedbackMessage, this);

		pf.pushMap(mapper.getGrid());
	}

private:

	void handleLaserMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_laser_scan_t* msg) {
		pthread_mutex_lock(&dataMutex);
		if(!pf.processing()) {
			pf.pushScan(*msg);
		}
		pthread_mutex_unlock(&dataMutex);
	}

	void handleMotorFeedbackMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_motor_feedback_t* msg) {
		pthread_mutex_lock(&dataMutex);
		pf.pushOdometry(*msg);

		if(pf.readyToInit() && !pf.initialized()) {
			pf.init(msg);
			printf("Initialized particle filter\n");
		}

		// if ready to process
		if(pf.readyToProcess() && pf.initialized()) {
			// get pose right now
			maebot_pose_t oldPose = pf.getBestPose();

			// process particle filter
			pf.process();

			// get pose after a move
			maebot_pose_t newPose = pf.getBestPose();

			// get corrected laser scans
			maebot_processed_laser_scan_t processedScans = laser.processSingleScan(*pf.getScan(), oldPose, newPose);

			// update map
			mapper.update(processedScans);

			maebot_particle_map_t pfMsg = pf.toLCM();
			lcm.publish("MAEBOT_PARTICLE_MAP", &pfMsg);
		}

		pthread_mutex_unlock(&dataMutex);
	}
};

int main() {
	StateHandler state;

	while(1) {
		state.lcm.handle();
	}
}
