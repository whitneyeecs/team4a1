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

class StateHandler {
public:
	eecs467::OccupancyGrid grid;
	float heading;
	std::vector<float> path_x;
	std::vector<float> path_y;
	pthread_mutex_t dataMutex;

	lcm::LCM lcm;
	
	pthread_t publish_map_data_pid;

public:
	StateHandler() : grid(5, 5, 0.05) {
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
		pthread_create(&publish_map_data_pid, NULL, &StateHandler::publishMapDataThread, this);
	}

private:
	void handleLaserMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_laser_scan_t* msg) {
		printf("utime: %ld\n", msg->utime);
		for (int i = 0; i < msg->num_ranges; i++) {
			if (msg->intensities[i] != 0)
				printf("%ld,\t%f,\t%f\n", msg->times[i], msg->thetas[i], msg->ranges[i]);
		}
		printf("diff in time: %ld\n", msg->times[msg->num_ranges - 1] - msg->times[0]);
		exit(1);

	}

	void handleMotorFeedbackMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_motor_feedback_t* msg) {

	}

	void handlePoseMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_pose_t* msg) {

	}

	static void* publishMapDataThread(void* arg) {
		StateHandler* state = (StateHandler*) arg;
		while (1) {
			pthread_mutex_lock(&state->dataMutex);
			maebot_map_data_t msg;
			msg.utime = 0; // not used right now
			msg.grid = state->grid.toLCM();
			msg.path_num = state->path_x.size();
			msg.path_x = state->path_x;
			msg.path_y = state->path_y;
			msg.lidar_num = 0;
			state->lcm.publish("MAEBOT_MAP_DATA", &msg);
			state->path_x.clear();
			state->path_y.clear();
			pthread_mutex_unlock(&state->dataMutex);
			usleep(10000);
		}

		return NULL;
	}
};

int main() {
	StateHandler state;
	state.launchThreads();

	// test data 
	// pthread_mutex_lock(&state.dataMutex);
	// for (unsigned int i = 0; i < state.grid.widthInCells(); ++i) {
	// 	state.grid.setLogOdds(i, 0, 127);
	// 	state.grid.setLogOdds(0, i, -128);
	// }
	// std::vector<float> x_points = { 0, 0, 1 };
	// std::vector<float> y_points = { 0, 1, 1 };
	// state.path_x.insert(state.path_x.end(), 
	// 	x_points.begin(), x_points.end());
	// state.path_y.insert(state.path_y.end(), 
	// 	y_points.begin(), y_points.end());
	// pthread_mutex_unlock(&state.dataMutex);

	// usleep(10000);

	// pthread_mutex_lock(&state.dataMutex);
	// std::vector<float> x_points2 = { 1, 2, 2 };
	// std::vector<float> y_points2 = { 1, 1, 2 };
	// state.path_x.insert(state.path_x.end(), 
	// 	x_points2.begin(), x_points2.end());
	// state.path_y.insert(state.path_y.end(), 
	// 	y_points2.begin(), y_points2.end());


	pthread_mutex_unlock(&state.dataMutex);
	while(1) {
		state.lcm.handle();
	}
}
