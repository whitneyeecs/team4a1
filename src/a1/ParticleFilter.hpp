#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <vector>
#include <algorithm>

#include "a1/VirtualOdometry.hpp"
#include "a1/StateEstimator.hpp"
#include "a1/ActionModel.hpp"

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"

#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_processed_laser_scan_t.hpp>
#include <lcmtypes/maebot_occupancy_grid_t.hpp>
#include <lcmtypes/maebot_particle_t.hpp>
#include <lcmtypes/maebot_motor_feedback_t.hpp>
#include <lcmtypes/maebot_particle_map_t.hpp>

namespace eecs467{

/////////////////////////////////////////////////////////////////
//This particle filter class maintains particle states and 
//performs necessary updates.
/////////////////////////////////////////////////////////////////
class ParticleFilter{
private:
//	typedef struct Particle{
//		maebot_pose_t pose;
//		float probability;
//	}Particle;

	eecs467::OccupancyGrid _map;
	
	ActionModel _actionModel;
	VirtualOdometry _odo;
	maebot_laser_scan_t _scan;

	std::vector<maebot_particle_t> _prior;
	std::vector<maebot_particle_t> _random_samples;
	std::vector<maebot_particle_t> _post_action;

public:
	typedef struct ParticleComp{
		bool operator()(maebot_particle_t a, maebot_particle_t b) const{
			return a.prob > b.prob;
		}
	} ParticleComp;

public:
	ParticleFilter();

	//
	//pushes current map to be used with particle filter
	//
	void pushMap(const eecs467::OccupancyGrid& map);

	//
	//initializes prior particle vector with random poses
	//
	//
	void init(const int64_t utime);	

	//
	//pushes odometry to be used in action model
	//
	void pushOdometry(maebot_motor_feedback_t& odometry);

	void pushScan(maebot_laser_scan_t& scan);

	void drawRandomSamples();

	void normalizeAndSort();

}; //end class

}//end namespace
#endif
