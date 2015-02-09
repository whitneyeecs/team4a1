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
	Point<float> end, start;

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
			newProb -= 14;
		} else if(_map.logOdds(cell.x, cell.y) > 120) {
			// wall
			newProb -= 4;
		} else if(_map.logOdds(cell.x, cell.y) < -120) {
			// open
			newProb -= 8;
		} else {
			// unexplored
			newProb -= 12;
		}
	}
}


const eecs467::OccupancyGrid eecs467::SensorModel::getGrid() const {
	return _map;
}

