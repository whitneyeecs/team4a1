#include "a1/SensorModel.hpp"

#include "mapping/occupancy_grid_utils.hpp"

#include "lcmtypes/maebot_processed_laser_scan_t.hpp"

eecs467::SensorModel::SensorModel() { }

eecs467::SensorModel::SensorModel(const eecs467::OccupancyGrid& map) {
	_map = map;
}

void eecs467::SensorModel::pushMap(const eecs467::OccupancyGrid& map) {
	_map = map;
}

void eecs467::SensorModel::apply(maebot_particle_t& particle, const maebot_laser_scan_t& scan, const maebot_pose_t& begin) {
	maebot_processed_laser_scan_t processedScans = 
		_laserCorrector.processSingleScan(scan, begin, particle.pose);

	float newProb = 0.0f;
	Point<float> start;
	Point<float> point, end;
	// Point<int> cellPos;

	// for (int i = 0; i < processedScans.num_ranges; ++i) {
	// 	// points are in meters relative to where robot first started
	// 	point.x = processedScans.x_pos[i];
	// 	point.y = processedScans.y_pos[i];

	// 	end.x = point.x + 
	// 		processedScans.ranges[i] * 
	// 		cos(processedScans.thetas[i]);
	// 	end.y = point.y + 
	// 		processedScans.ranges[i] * 
	// 		sin(processedScans.thetas[i]);

	// 	float stepX = (end.x - point.x) / eecs467::sensorModelStepsPerLaser;
	// 	float stepY = (end.y - point.y) / eecs467::sensorModelStepsPerLaser;

	// 	// last step is the end point
	// 	for (int i = 0; i < steps - 1; ++i) {
	// 		cellPos = global_position_to_grid_cell(start);
	// 		if (!_map.isGridInCell(cellPos.y, cellPos.x)) {

	// 		}

	// 		point.x += stepX;
	// 		point.y += stepY;
	// 	}

	// }

	for (int i = 0; i < processedScans.num_ranges; ++i) {
		start.x = processedScans.x_pos[i];
		start.y = processedScans.y_pos[i];

		end.x = processedScans.x_pos[i] + 
			processedScans.ranges[i] * 
			cos(processedScans.thetas[i]);
		end.y = processedScans.y_pos[i] + 
			processedScans.ranges[i] * 
			sin(processedScans.thetas[i]);

		Point<int> cell = global_position_to_grid_cell(end, _map);
		Point<int> origin = global_position_to_grid_cell(start, _map);

		if(!_map.isCellInGrid(origin.x, origin.y)){
			// our position is not in the grid
			newProb -= 30;
		}

		if(!_map.isCellInGrid(cell.x, cell.y)) {
			// out of grid
			newProb -= 16;
		} else if(_map.logOdds(cell.x, cell.y) > 120) {
			// wall
			newProb -= 1;
		} else if(_map.logOdds(cell.x, cell.y) < -120) {
			// open
			newProb -= 6;
		} else {
			// unexplored
			newProb -= 12;
		}
	}
	particle.prob = newProb;
//printf("the new prob from sensor model: %f\n", particle.prob);
}

const eecs467::OccupancyGrid eecs467::SensorModel::getGrid() const {
	return _map;
}

