#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <vector>
#include <algorithm>

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
	
#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_processed_laser_scan_t.hpp>
#include <lcmtypes/maebot_occupancy_grid_t.hpp>
#include <lcmtypes/maebot_particle_t.hpp>

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

	std::vector<maebot_particle_t> _prior;
	std::vector<maebot_particle_t> _post_unnormalized;
	std::vector<maebot_particle_t> _post_normalized;

public:
	typedef struct ParticleComp{
		bool operator()(maebot_particle_t* a, maebot_particle_t* b) const{
			return a->prob > b->prob;
		}
	}ParticleComp;

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
	void init();	

}; //end class

}//end namespace
#endif
