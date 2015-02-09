#ifndef SENSOR_MODEL_HPP
#define SENSOR_MODEL_HPP

#include <stdint.h>

#include "a1/LaserCorrector.hpp"

#include "mapping/occupancy_grid.hpp"

#include "lcmtypes/maebot_particle_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"

namespace eecs467 {

class SensorModel {
private:
	eecs467::OccupancyGrid _map;
	eecs467::LaserCorrector _laserCorrector;

public:
	SensorModel(const eecs467::OccupancyGrid& map);

	void apply(maebot_particle_t& particle, const maebot_laser_scan_t& scan, const maebot_pose_t& begin);

};

}

#endif /* SENSOR_MODEL_HPP */
