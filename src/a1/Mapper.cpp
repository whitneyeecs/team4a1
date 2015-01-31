#include "Mapper.hpp"
#include "math/point.hpp"
#include <cmath>

eecs467::Mapper::Mapper(float separationSize, float widthInMeters, 
	float heightInMeters, float metersPerCell) :
	_grid(widthInMeters, heightInMeters, metersPerCell),
	_separationSize(separationSize) { }

void eecs467::Mapper::update(const maebot_processed_laser_scan_t& scan) {
	Point<float> point;
	for (int i = 0; i < scan.num_ranges; ++i) {
		if (scan.intensities[i] == 0) {
			// don't update cells if intensity is 0
			continue;
		}
		// start from laser origin
		point.x = scan.x_pos[i];
		point.y = scan.y_pos[i];

		float deltaX = _separationSize * _grid.metersPerCell() *
			cos(scan.thetas[i]);
		float deltaY = _separationSize * _grid.metersPerCell() *
			sin(scan.thetas[i]);
		int steps = scan.ranges[i] / (_separationSize * _grid.metersPerCell());

		Point<int> cellPos = { -1, -1 };
		for (int i = 0; i < steps; ++i) {
			cellPos = global_position_to_grid_cell(point, _grid);
			// update empty cells

			if (_grid(cellPos.y, cellPos.x) < -127) {
				_grid(cellPos.y, cellPos.x) = -128;
			} else {
				_grid(cellPos.y, cellPos.x) -= 1;
			}
			point.x += deltaX;
			point.y += deltaY;
		}
		if (cellPos.x != -1) {
			// update full cells
			if (_grid(cellPos.y, cellPos.x) != 127) { 
				_grid(cellPos.y, cellPos.x) += 1;
			}
		}
	}
}

const eecs467::OccupancyGrid& eecs467::Mapper::getGrid() const {
	return _grid;
}
