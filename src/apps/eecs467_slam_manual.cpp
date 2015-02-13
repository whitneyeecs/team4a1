#include <unistd.h>

// c++
#include <vector>
#include <list>
#include <iostream>

// lcm
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/maebot_occupancy_grid_t.hpp>
#include <lcmtypes/maebot_motor_feedback_t.hpp>
#include <lcmtypes/maebot_motor_command_t.hpp>
#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_map_data_t.hpp>
#include "mapping/occupancy_grid.hpp"

#include "a1/LaserCorrector.hpp"
#include "a1/ParticleFilter.hpp"
#include "a1/Mapper.hpp"
#include "a1/SlamConstants.hpp"

class StateHandler {
public:
	eecs467::LaserCorrector laser;
	eecs467::ParticleFilter pf;
	eecs467::Mapper mapper;

	int initialScanCounter;

	pthread_mutex_t dataMutex;

	pthread_t controlThreadPid;

	lcm::LCM lcm;

public:
	StateHandler() : mapper(eecs467::gridSeparationSize,
		eecs467::gridWidthMeters,
		eecs467::gridHeightMeters,
		eecs467::gridCellSizeMeters) {
		if (!lcm.good()) {
			printf("lcm unable to initialize\n");
			exit(1);
		}

		if (pthread_mutex_init(&dataMutex, NULL)) {
			printf("data mutex not initialized\n");
			exit(1);
		}

		lcm.subscribe("MAEBOT_LASER_SCAN",
			&StateHandler::handleLaserMessage, this);
		lcm.subscribe("MAEBOT_MOTOR_FEEDBACK",
			&StateHandler::handleMotorFeedbackMessage, this);

		pf.pushMap(mapper.getGrid());

		initialScanCounter = 0;
	}

	void launchThreads() {
		pthread_create(&controlThreadPid, NULL,
			&StateHandler::controlThread, this);
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
			maebot_processed_laser_scan_t processedScans = 
				laser.processSingleScan(*pf.getScan(), oldPose, newPose);

			// update map
			mapper.update(processedScans);

			maebot_particle_map_t pfMsg = pf.toLCM();
			lcm.publish("MAEBOT_PARTICLE_MAP", &pfMsg);

			initialScanCounter++;
		}

		pthread_mutex_unlock(&dataMutex);
	}

	static void* controlThread(void* arg) {
		StateHandler* state = (StateHandler*) arg;

		// wait for at least 2 initial scans
		while (1) {
			pthread_mutex_lock(&state->dataMutex);
			if (state->initialScanCounter >= 2) {
				pthread_mutex_unlock(&state->dataMutex);
				break;
			}
			pthread_mutex_unlock(&state->dataMutex);
		}

		// main control loop
		maebot_motor_command_t commandMsg;
		float leftWheelSpeed = 0.25;
		float rightWheelSpeed = 0.25;
		while (1) {
			char inChar;
			std::cout << "enter command: ";
			std::cin >> inChar;
			switch (inChar) {
				case 'w':
					commandMsg.motor_left_speed = leftWheelSpeed;
					commandMsg.motor_right_speed = rightWheelSpeed;
					break;
				case 'a':
					commandMsg.motor_left_speed = -leftWheelSpeed;
					commandMsg.motor_right_speed = rightWheelSpeed;
					break;
				case 's':
					commandMsg.motor_left_speed = -leftWheelSpeed;
					commandMsg.motor_right_speed = -rightWheelSpeed;
					break;
				case 'd':
					commandMsg.motor_left_speed = leftWheelSpeed;
					commandMsg.motor_right_speed = -rightWheelSpeed;
					break;
				default:
					printf("Not a command");
					continue;
			}

			state->lcm.publish("MAEBOT_MOTOR_COMMAND", &commandMsg);
			switch (inChar) {
				case 'w':
				case 's':
					usleep(1e6);
					break;
				case 'a':
				case 'd':
					usleep(5e5);
			}
			commandMsg.motor_left_speed = 0;
			commandMsg.motor_right_speed = 0;
			state->lcm.publish("MAEBOT_MOTOR_COMMAND", &commandMsg);
		}
		
		return NULL;
	}
};

int main() {
	StateHandler state;
	state.launchThreads();

	while(1) {
		state.lcm.handle();
	}
}
