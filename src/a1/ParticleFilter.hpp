#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <vector>
#include <algorithm>

#include "a1/VirtualOdometry.hpp"
#include "a1/StateEstimator.hpp"
#include "a1/ActionModel.hpp"
#include "a1/SensorModel.hpp"

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
	ActionModel _actionModel;
	SensorModel _sensorModel;
	VirtualOdometry _odo;

	bool _hasMap;
	bool _processing;
	bool _hasScan;
	
	gsl_rng* randGen;

	maebot_laser_scan_t _scan;
	maebot_particle_map_t _particle_map;

	std::vector<maebot_particle_t> _prior;
	std::vector<maebot_particle_t> _random_samples;

public:
	typedef struct ParticleComp{
		bool operator()(const maebot_particle_t& a, 
			const maebot_particle_t& b) const{
			return a.prob > b.prob;
		}
	} ParticleComp;

public:
	ParticleFilter();

	//
	//pushes current map to be used with particle filter
	//
	void pushMap(const eecs467::OccupancyGrid* map);

	//
	//initializes prior particle vector with random poses
	//
	//
	void init(const maebot_motor_feedback_t* msg);	

	//
	//pushes odometry to be used in action model
	//
	void pushOdometry(const maebot_motor_feedback_t& odometry);

	/**
	 * @brief pushes a scan into the particle filter
	 */
	void pushScan(const maebot_laser_scan_t& scan);

	/**
	 * @brief [brief description]
	 * @details [long description]
	 * @return [description]
	 */
	bool readyToInit(){ return _hasMap; }

	bool initialized(){ return !_prior.empty(); }

	bool readyToProcess(){ return (_hasScan &&
		_odo.getUtime() > _scan.times[_scan.num_ranges - 1]); }

	maebot_particle_map_t toLCM();

	void process();

	bool processing(){ return _processing; }

	maebot_pose_t getBestPose();

	const maebot_laser_scan_t* getScan() const;

private:
	void drawRandomSamples();

	void normalizeAndSort(const std::array<int32_t, 2>& deltas, int64_t utime);

}; //end class

}//end namespace
#endif
