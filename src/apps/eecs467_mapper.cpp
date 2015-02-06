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

eecs467::LaserCorrector laser;
eecs467::ParticleFilter pf;

class StateHandler {
public:
	eecs467::Mapper mapper;
	float heading;
	std::vector<float> path_x;
	std::vector<float> path_y;
	pthread_mutex_t dataMutex;

	lcm::LCM lcm;
	
	pthread_t publish_map_data_pid;

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
		lcm.subscribe("MAEBOT_POSE", &StateHandler::handlePoseMessage, this);
	}

	void launchThreads() {
		pthread_create(&publish_map_data_pid, NULL, &StateHandler::processMapDataThread, this);
	}

private:


	void handleLaserMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_laser_scan_t* msg) {
	
		pthread_mutex_lock(&dataMutex);
		laser.pushNewScans(*msg);
		pf.pushScan(*msg);
		pthread_mutex_unlock(&dataMutex);
	}

	void handleMotorFeedbackMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_motor_feedback_t* msg) {
			
			pthread_mutex_lock(&dataMutex);
			pf.pushMap(mapper.getGrid());
			pthread_mutex_unlock(&dataMutex);
		
		
			pthread_mutex_lock(&dataMutex);
			if(pf.readyToInit() && !pf.initialized()){
				pf.init(0);
printf("finished initialization\n");
			}
			pthread_mutex_unlock(&dataMutex);
	}

	void handlePoseMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_pose_t* msg) {

		pthread_mutex_lock(&dataMutex);
		laser.pushNewPose(*msg);
		path_x.push_back(msg->x);
		path_y.push_back(msg->y);
		heading = msg->theta;
		pthread_mutex_unlock(&dataMutex);
	}

	static void* processMapDataThread(void* arg) {
		StateHandler* state = (StateHandler*) arg;
		while (1) {
			usleep(1000);
			pthread_mutex_lock(&state->dataMutex);
			laser.process();
			maebot_processed_laser_scan_t message;
			if (!laser.getCorrectedLcmMsg(message)) {

			if(pf.initialized()){
				maebot_particle_map_t pf_msg = pf.toLCM();
				state->lcm.publish("MAEBOT_PARTICLE_MAP", &pf_msg);
				exit(0);
			}
				pthread_mutex_unlock(&state->dataMutex);
				continue;
			}
			state->mapper.update(message);

//			pf.pushMap(mapper.getGrid());
			
			maebot_map_data_t msg;
			msg.scan = message;
			msg.utime = 0; // not used right now
			msg.grid = state->mapper.getGrid().toLCM();
			msg.path_num = state->path_x.size();
			msg.path_x = state->path_x;
			msg.path_y = state->path_y;
			state->lcm.publish("MAEBOT_MAP_DATA", &msg);
			state->path_x.clear();
			state->path_y.clear();

			//generate and broadcast particle filter and map

			if(pf.initialized()){
				maebot_particle_map_t pf_msg = pf.toLCM();
				state->lcm.publish("MAEBOT_PARTICLE_MAP", &pf_msg);
				exit(0);
			}
			pthread_mutex_unlock(&state->dataMutex);
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
