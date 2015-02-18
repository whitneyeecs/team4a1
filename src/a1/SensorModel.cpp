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

maebot_processed_laser_scan_t eecs467::SensorModel::applyEndPoints(maebot_particle_t& particle, const maebot_laser_scan_t& scan, const maebot_pose_t& beginPose, const maebot_pose_t& endPose) {

	maebot_processed_laser_scan_t processedScans = 
		_laserCorrector.processSingleScan(scan, beginPose, endPose);

	float newProb = 0.0f;
	Point<float> start, end;

	for (int i = 0; i < processedScans.num_ranges; ++i) {
		start.x = processedScans.x_pos[i];
		start.y = processedScans.y_pos[i];

		end.x = processedScans.x_pos[i] + 
			processedScans.ranges[i] * 
			cos(processedScans.thetas[i]);
		end.y = processedScans.y_pos[i] + 
			processedScans.ranges[i] * 
			sin(processedScans.thetas[i]);

		Point<int> origin = global_position_to_grid_cell(start, *_map);

		if(!_map->isCellInGrid(origin.x, origin.y)){
			// our position is not in the grid
			newProb -= 30;
		}

		adjustProb(end, newProb, eecs467::sensorModelOutOfGrid,
			eecs467::sensorModelWall,
			eecs467::sensorModelOpen,
			eecs467::sensorModelUnexplored);
	}

	particle.prob = newProb;
	return processedScans;
}

maebot_processed_laser_scan_t eecs467::SensorModel::applyRayTrace(maebot_particle_t& particle, const maebot_laser_scan_t& scan, const maebot_pose_t& beginPose, const maebot_pose_t& endPose) {

	maebot_processed_laser_scan_t processedScans = 
		_laserCorrector.processSingleScan(scan, beginPose, endPose);

	float newProb = 0.0f;
	Point<float> point, end;
	Point<int> cellPos;

	for (int i = 0; i < processedScans.num_ranges; ++i) {
		// points are in meters relative to where robot first started
		point.x = processedScans.x_pos[i];
		point.y = processedScans.y_pos[i];

		end.x = point.x + 
			processedScans.ranges[i] * 
			cos(processedScans.thetas[i]);
		end.y = point.y + 
			processedScans.ranges[i] * 
			sin(processedScans.thetas[i]);

		float stepX = (end.x - point.x) / eecs467::sensorModelStepsPerLaser;
		float stepY = (end.y - point.y) / eecs467::sensorModelStepsPerLaser;

		adjustProb(point, newProb, eecs467::sensorStartOutOfGrid,
			eecs467::sensorStartWall,
			eecs467::sensorStartOpen,
			eecs467::sensorStartUnexplored);

		// last step is the end point
		// this loops through the middle points
		for (int i = 1; i < eecs467::sensorModelStepsPerLaser - 1; ++i) {
			adjustProb(point, newProb,
				eecs467::sensorMiddleOutOfGrid,
				eecs467::sensorMiddleWall,
				eecs467::sensorMiddleOpen,
				eecs467::sensorMiddleUnexplored);

			point.x += stepX;
			point.y += stepY;
		}

		adjustProb(end, newProb,
			eecs467::sensorEndOutOfGrid,
			eecs467::sensorEndWall,
			eecs467::sensorEndOpen,
			eecs467::sensorEndUnexplored);
	}

	particle.prob = newProb;
	return processedScans;
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
	} else if (_map->logOdds(cellPos.x, cellPos.y) > eecs467::wallThreshold) {
		// wall
		prob -= wall;
	} else if (_map->logOdds(cellPos.x, cellPos.y) < eecs467::emptyThreshold) {
		// empty
		prob -= empty;
	} else {
		// unknown
		prob -= unknown;
	}
}

