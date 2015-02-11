#include "a1/SensorModel.hpp"
#include "a1/SlamConstants.hpp"

#include "mapping/occupancy_grid_utils.hpp"

#include "lcmtypes/maebot_processed_laser_scan_t.hpp"

eecs467::SensorModel::SensorModel() { }

eecs467::SensorModel::SensorModel(const eecs467::OccupancyGrid* map) 
	: _map(map) {
}

void eecs467::SensorModel::pushMap(const eecs467::OccupancyGrid* map) {
	_map = map;
}

void eecs467::SensorModel::apply(maebot_particle_t& particle, const maebot_laser_scan_t& scan, const maebot_pose_t& begin) {
	maebot_processed_laser_scan_t processedScans = 
		_laserCorrector.processSingleScan(scan, begin, particle.pose);

	float newProb = 0.0f;
	Point<float> start;
	Point<float> point, end;
	Point<int> cellPos;

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

	// 	adjustProb(point, newProb, 30, 5, 0, 0);

	// 	// last step is the end point
	// 	// this loops through the middle points
	// 	for (int i = 1; i < eecs467::sensorModelStepsPerLaser - 1; ++i) {
	// 		adjustProb(point, newProb, 0, 0.5, 0, 0.25);

	// 		point.x += stepX;
	// 		point.y += stepY;
	// 	}

	// 	adjustProb(end, newProb, 0, 0, 1, 2);
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

		Point<int> cell = global_position_to_grid_cell(end, *_map);
		Point<int> origin = global_position_to_grid_cell(start, *_map);

		if(!_map->isCellInGrid(origin.x, origin.y)){
			// our position is not in the grid
			newProb -= 30;
		}

		if(!_map->isCellInGrid(cell.x, cell.y)) {
			// out of grid
			newProb -= eecs467::sensorModelOutOfGrid;
		} else if(_map->logOdds(cell.x, cell.y) > 120) {
			// wall
			newProb -= eecs467::sensorModelWall;
		} else if(_map->logOdds(cell.x, cell.y) < -120) {
			// open
			newProb -= eecs467::sensorModelOpen;
		} else {
			// unexplored
			newProb -= eecs467::sensorModelUnexplored;
		}
	}
	particle.prob = newProb;
}

const eecs467::OccupancyGrid* eecs467::SensorModel::getGrid() const {
	return _map;
}

void eecs467::SensorModel::adjustProb(Point<float> point, float& prob, float offGrid,
	float wall, float empty, float unknown) const {
	Point<int> cellPos = global_position_to_grid_cell(point, *_map);
	if (!_map->isCellInGrid(cellPos.y, cellPos.x)) {
		// laser goes off grid
		prob -= offGrid;
	} else if (_map->logOdds(cellPos.x, cellPos.y) > 120) {
		// wall
		prob -= wall;
	} else if (_map->logOdds(cellPos.x, cellPos.y) < -120) {
		// empty
		prob -= empty;
	} else {
		// unknown
		prob -= unknown;
	}
}

