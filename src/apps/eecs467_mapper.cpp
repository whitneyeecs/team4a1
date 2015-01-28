#include <unistd.h>

// c++
#include <vector>

// lcm
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/maebot_occupancy_grid_t.hpp>
#include <lcmtypes/maebot_motor_feedback_t.hpp>
#include <lcmtypes/maebot_laser_t.hpp>
#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_map_data_t.hpp>
#include "mapping/occupancy_grid.hpp"

class StateHandler {
public:
	eecs467::OccupancyGrid grid;
	float heading;
	std::vector<float> path;
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
		const maebot_laser_t* msg) {

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
			msg.path_num = state->path.size();
			msg.path.clear();
			msg.path = state->path;
			pthread_mutex_unlock(&state->dataMutex);
			state->lcm.publish("MAEBOT_MAP_DATA", &msg);
			usleep(10000);
		}

		return NULL;
	}
};

int main() {
	StateHandler state;
	state.launchThreads();

	pthread_mutex_lock(&state.dataMutex);
	for (unsigned int i = 0; i < state.grid.widthInCells(); ++i) {
		state.grid.setLogOdds(i, 0, 127);
		state.grid.setLogOdds(0, i, -128);
	}
	pthread_mutex_unlock(&state.dataMutex);

	while(1) {
		state.lcm.handle();
	}
}
