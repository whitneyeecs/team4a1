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
	SensorModel();

	SensorModel(const eecs467::OccupancyGrid& map);

	void pushMap(const eecs467::OccupancyGrid& map);

	/**
	 * @brief applies the sensor model to a particle
	 * @details particle needs to contain the pose of the maebot at the end of the laser scan
	 * @param particle particle to apply sensor to
	 * @param scan laser scan to apply
	 * @param begin pose that the maebot was at when the scan began
	 */
	void apply(maebot_particle_t& particle, const maebot_laser_scan_t& scan, const maebot_pose_t& begin);

	const eecs467::OccupancyGrid getGrid() const;

};

}

#endif /* SENSOR_MODEL_HPP */
