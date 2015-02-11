#ifndef MAPPER_HPP
#define MAPPER_HPP

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "lcmtypes/maebot_processed_laser_scan_t.hpp"

namespace eecs467 {

class Mapper {
private:
	eecs467::OccupancyGrid _grid;
	float _separationSize; // size between sampling on scan lines

public:
	Mapper(float separationSize = 1,
		float widthInMeters = 5,
		float heightInMeters = 5,
		float metersPerCell = 0.05);

	void update(const maebot_processed_laser_scan_t& scan);

	const eecs467::OccupancyGrid* getGrid() const;
};

}

#endif /* MAPPER_HPP */
