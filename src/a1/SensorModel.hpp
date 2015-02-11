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
	const eecs467::OccupancyGrid* _map;
	eecs467::LaserCorrector _laserCorrector;

public:
	SensorModel();

	SensorModel(const eecs467::OccupancyGrid* map);

	void pushMap(const eecs467::OccupancyGrid* map);

	/**
	 * @brief applies the sensor model to a particle
	 * @details uses only the endpoints to determine probability
	 * 
	 * @param particle particle to apply sensor to
	 * @param scan laser scan to apply
	 * @param beginPose pose that the maebot was at when the scan began
	 * @param endPose pose that the maebot was at when the scan ended
	 */
	void applyEndPoints(maebot_particle_t& particle,
		const maebot_laser_scan_t& scan,
		const maebot_pose_t& beginPose,
		const maebot_pose_t& endPose);

	/**
	 * @brief applies the sensor model to a particle
	 * @details ray traces to find probability
	 * 
	 * @param particle particle to apply sensor to
	 * @param scan laser scan to apply
	 * @param beginPose pose that the maebot was at when the scan began
	 * @param endPose pose that the maebot was at when the scan ended
	 */
	void applyRayTrace(maebot_particle_t& particle,
		const maebot_laser_scan_t& scan,
		const maebot_pose_t& beginPose,
		const maebot_pose_t& endPose);

	const eecs467::OccupancyGrid* getGrid() const;

private:
	void adjustProb(Point<float> point, float& prob,
		float offGrid, float wall, float empty,
		float unknown) const;
};

}

#endif /* SENSOR_MODEL_HPP */
