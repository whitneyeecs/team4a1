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
#include <lcmtypes/maebot_sensor_data_t.hpp>
#include "mapping/occupancy_grid.hpp"

// math
#include "math/point.hpp"

//our files
#include "a1/LaserCorrector.hpp"
#include "a1/ParticleFilter.hpp"
#include "a1/Mapper.hpp"
#include "a1/Navigation.hpp"
#include "a1/SlamConstants.hpp"
#include "a1/RobotConstants.hpp"
#include "a1/Explore.hpp"

using namespace eecs467;

class StateHandler {
public:
	eecs467::LaserCorrector laser;
	eecs467::ParticleFilter pf;
	eecs467::Mapper mapper;
	eecs467::Navigation nav;
	eecs467::Explore pathPlanner;
	eecs467::Point<double> wayPoint;
	pthread_mutex_t dataMutex;

	lcm::LCM lcm;

	pthread_t nav_thread;
	pthread_t cmd_thread;

	maebot_motor_command_t command;
	maebot_motor_command_t pcommand;

//	bool turn;
	bool driving;

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
		
		lcm.subscribe("MAEBOT_SENSOR_DATA",
			&StateHandler::handleSensorMessage, this);

		pf.pushMap(mapper.getGrid());
	}

	void launchThreads(){
		pthread_create(&nav_thread, NULL, &StateHandler::NavigationThread, this);
		pthread_create(&cmd_thread, NULL, &StateHandler::CommandThread, this);
	}

private:

	void handleLaserMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_laser_scan_t* msg) {
		pthread_mutex_lock(&dataMutex);
//printf("\n\npushing scan\n\n\n\n");
		if(!pf.processing()) {
			pf.pushScan(*msg);
		}
		pthread_mutex_unlock(&dataMutex);
	}
	
	void handleSensorMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_sensor_data_t* msg) {
/*		pthread_mutex_lock(&dataMutex);
printf("got sensor data:\t%d\n", msg->range);
		if(msg->range > 800){
			turn = true;
		}else
			turn = false;
printf("turn?:\t%d\n", turn);
		pthread_mutex_unlock(&dataMutex);
*/	}

	void handleMotorFeedbackMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_motor_feedback_t* msg) {
		pthread_mutex_lock(&dataMutex);
//printf("got odometry\n");
		pf.pushOdometry(*msg);
	
		bool updated = false;
	
		if(pf.readyToInit() && !pf.initialized()) {
			pf.init(msg);
			printf("Initialized particle filter\n");
			nav.pushPose(pf.getBestPose());
		}

		// if ready to process
		if(pf.readyToProcess() && pf.initialized()) {
			// get pose right now
			maebot_pose_t oldPose = pf.getBestPose();

			// process particle filter
			pf.process();

			// get pose after a move
			maebot_pose_t newPose = pf.getBestPose();
			nav.pushPose(newPose);

			// get corrected laser scans
			maebot_processed_laser_scan_t processedScans = 
				laser.processSingleScan(*pf.getScan(), oldPose, newPose);

			// update map
			mapper.update(processedScans);

			maebot_particle_map_t pfMsg;
			pf.toLCM(pfMsg);
			pathPlanner.toLCM(pfMsg, *mapper.getGrid());
			pfMsg.num_path++;
			pfMsg.path_x.push_back(wayPoint.x);
			pfMsg.path_y.push_back(wayPoint.y);
			lcm.publish("MAEBOT_PARTICLE_MAP", &pfMsg);
			updated = true;
		}
		if(updated && driving){
			command = nav.correct(driving);
		}
		pthread_mutex_unlock(&dataMutex);
	}

	static void* NavigationThread(void* arg){
		StateHandler* state = (StateHandler*) arg;
		state->driving = false; 
		usleep(5000000);

		while(1){
			if(state->pf.initialized()){

				const OccupancyGrid& grid = *state->mapper.getGrid();
				if (state->driving) {
					Point<int> dest;
					if (!state->pathPlanner.getCurrentDestination(dest)) {
						continue;
					}
					if (grid(dest) < wallThreshold && grid(dest) > emptyThreshold) {
						continue;
					}
				}
				// if(!state->driving){

				pthread_mutex_lock(&state->dataMutex);
				// getting our current position
				maebot_pose_t pfPose = state->pf.getBestPose();
				Point<int> currPos = 
					global_position_to_grid_cell(Point<double>{pfPose.x, pfPose.y}, 
						grid);
				// get the next waypoint
				if (!state->pathPlanner.getNextWayPoint(grid,
					currPos, state->wayPoint)) {
					// stop
					continue;
				}
				state->driving = true;
				state->nav.driveTo(state->wayPoint);
				pthread_mutex_unlock(&state->dataMutex);
				// }
			}
			usleep(1000000);
		}

		return NULL;
	}
	
	static void* CommandThread(void* arg){
		StateHandler* state = (StateHandler*) arg;
		while(1){
			pthread_mutex_lock(&state->dataMutex);
			if(state->command.motor_right_speed == 0.7 * eecs467::go || state->command.motor_right_speed == -0.7 * eecs467::go){
				for(int i = 0; i < 5; ++i) {
					state->lcm.publish("MAEBOT_MOTOR_COMMAND", &state->command);
				}
				pthread_mutex_unlock(&state->dataMutex);
//				state->command.motor_left_speed = eecs467::stop;
//				state->command.motor_right_speed = eecs467::stop;
//				
//				usleep(10000);
				
//				pthread_mutex_lock(&state->dataMutex);
//				state->lcm.publish("MAEBOT_MOTOR_COMMAND", &state->command);
//				pthread_mutex_unlock(&state->dataMutex);

				usleep(3000000);
				continue;
			} else {
		//		for (int i = 0; i < 1000; ++i) {
					state->lcm.publish("MAEBOT_MOTOR_COMMAND", &state->command);
		//		}
				pthread_mutex_unlock(&state->dataMutex);
				usleep(150000);
				continue;
			}
			pthread_mutex_unlock(&state->dataMutex);

		//	usleep(4000000);
			
//			pthread_mutex_lock(&state->dataMutex);
//			state->lcm.publish("MAEBOT_MOTOR_COMMAND", &state->command);
//			pthread_mutex_unlock(&state->dataMutex);
//			usleep(200000);
		}

		return NULL;
	}
};

int main() {
	StateHandler state;
	state.launchThreads();
printf("thread launched\n\n");

	while(1) {
		state.lcm.handle();
	}
}
