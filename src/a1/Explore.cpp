#include "math/angle_functions.hpp"
#include "a1/SlamConstants.hpp"
#include "a1/RobotConstants.hpp"
#include "a1/Explore.hpp"

#include <deque>

namespace eecs467 {

Explore::Explore() { }

bool Explore::detectObstacle(const OccupancyGrid& grid, 
	const Point<int>& start, 
	const Point<int>& end, double steps){

	Point<int> cellPos;
	
	Point<double> point;
	point.x = start.x;
	point.y = start.y;
	
	Point<double> endpoint;
	endpoint.x = end.x;
	endpoint.y = end.y; 

	float stepx = (endpoint.x - point.x) / steps;
	float stepy = (endpoint.y - point.y) / steps;

	for(unsigned int i = 1; i < steps; i++)  {
		point.x += stepx;
		point.y += stepy;

		Point<int> cellPos = Point<int>{(int)point.x,
			(int)point.y};
		
		if (state(grid(cellPos.x, cellPos.y)) == WALL) {
			return true;
		}
	}
	return false;
}

bool Explore::getNextWayPoint(const OccupancyGrid& grid, 
	const Point<int>& currPos,
	Point<double>& nextWayPoint) {

	OccupancyGrid space = getConfigurationSpace(grid, baseLength / 2.0f);
	// need a copy of configspace because the search will modify it
	OccupancyGrid spaceCopy = space;
	std::vector<Point<int>> points = breadthFirstSearch(space, currPos);

	// if no path exists, return
	if (points.size() == 0) {
		return false;
	}

	// if we have chosen a different end point, clear queue
	if (points.front() != _dest) {
		_wayPoints.clear();
	}

	points = pickWayPoints(spaceCopy, points);

	_wayPoints = points;


	nextWayPoint = grid_position_to_global_position(_wayPoints.back(), grid);
	_dest = _wayPoints.front();
	_wayPoints.pop_back();
	return true;
}

void Explore::clearPath() {
	_wayPoints.clear();
}

void Explore::toLCM(maebot_particle_map_t& map,
	const OccupancyGrid& grid) {
	OccupancyGrid configSpace = getConfigurationSpace(grid, baseLength / 2.0f);
	map.config_space = configSpace.toLCM();
	map.num_path = _wayPoints.size();
	for (auto& point : _wayPoints) {
		Point<double> dPoint = grid_position_to_global_position(point, grid);

		map.path_x.push_back(dPoint.x);
		map.path_y.push_back(dPoint.y);
	}
}

OccupancyGrid Explore::getConfigurationSpace(const OccupancyGrid& grid, float radius) {
	eecs467::OccupancyGrid configSpace(grid.widthInMeters(),
		grid.heightInMeters(), grid.metersPerCell());

	// float cellDiagonal = sqrt(2) * grid.metersPerCell();
	// float deltaTheta = (cellDiagonal / 2) / radius;

	for (int y = 0; y < (int)grid.heightInCells(); y++) {
		for (int x = 0; x < (int)grid.widthInCells(); x++) {
			// draw circle around point
			if (grid(x, y) > eecs467::wallThreshold) {
				// Point<int> gridOriginPt{x, y};
				// Point<double> globalOriginPt = 
				// 	grid_position_to_global_position(gridOriginPt, grid);
				// for (float theta = 0; theta < 2 * M_PI; 
				// 	theta += deltaTheta) {
				// 	Point<double> edgePt{globalOriginPt.x + cos(theta), 
				// 		globalOriginPt.y + sin(theta)};
				// 	Point<int> gridEdgePt = 
				// 		global_position_to_grid_cell(edgePt, grid);
				// 		if (grid.isCellInGrid(gridEdgePt.x, gridEdgePt.y)) {
				// 			configSpace(gridEdgePt.x, gridEdgePt.y) = WALL;
				// 		}
				// }
				configSpace(x, y) = WALL;
				configSpace(x - 1, y) = WALL;
				configSpace(x + 1, y) = WALL;
				configSpace(x, y - 1) = WALL;
				configSpace(x, y + 1) = WALL;
				configSpace(x + 1, y + 1) = WALL;
				configSpace(x + 1, y - 1) = WALL;
				configSpace(x - 1, y + 1) = WALL;
				configSpace(x - 1, y - 1) = WALL;
			} else if (grid(x, y) < eecs467::emptyThreshold) {
				if (state(configSpace(x, y)) != WALL) {
					configSpace(x, y) = EMPTY;
				}
			} else {
				if (state(configSpace(x, y)) != WALL) {
					configSpace(x, y) = UNKNOWN;
				}
			}
		}
	}
	return configSpace;
}

std::vector<Point<int>> Explore::pickWayPoints(const OccupancyGrid& grid, 
	const std::vector<Point<int>>& points) {

	std::vector<Point<int>> updateWaypoints;
	if (points.size() == 0) {
		return updateWaypoints;
	}

	Point<int> startPoint;
	Point<int> checkPoint;

	int s = 0; //index of startPoint
	startPoint = points[s];
	updateWaypoints.push_back(startPoint);	

	for(unsigned int i = 1; i < points.size(); i++) {
		checkPoint = points[i];
		int step = i - s; //# of steps checkpoint is away from startpoint

		if(detectObstacle(grid, startPoint, checkPoint, step * 2)) {
			updateWaypoints.push_back(points[i - 1]);
			startPoint = points[i - 1];
			s = i - 1; //update index of startpoint
		}
	}
	return updateWaypoints;
}

// helper function for breadth first search
inline bool expandNode(std::deque<Point<int>>& exploreQueue, 
	OccupancyGrid& grid, int x, int y, Point<int>& pathPoint, Direction dir) {
	if (grid.isCellInGrid(x, y) &&
		state(grid(x, y)) != WALL) {
		if (state(grid(x, y)) == UNKNOWN) {
			pathPoint = Point<int>{x, y};
			grid(x, y) |= dir;
			return true;
		}
		exploreQueue.push_back(Point<int>{x, y});
		grid(x, y) = WALL | dir;
	}
	return false;
}

std::vector<Point<int>> Explore::breadthFirstSearch(OccupancyGrid& grid, 
	const Point<int>& currPos) {

	std::deque<Point<int>> exploreQueue;
	exploreQueue.push_back(currPos);
	grid(currPos) = NONE;

	Point<int> pathPoint;
	while (!exploreQueue.empty()) {
		Point<int> currPoint = exploreQueue.front();
		exploreQueue.pop_front();

		// left
		if (expandNode(exploreQueue, grid, 
			currPoint.x - 1, currPoint.y, pathPoint, RIGHT)) {
			break;
		}
		// right
		if (expandNode(exploreQueue, grid, 
			currPoint.x + 1, currPoint.y, pathPoint, LEFT)) {
			break;
		}
		// down
		if (expandNode(exploreQueue, grid, 
			currPoint.x, currPoint.y - 1, pathPoint, UP)) {
			break;
		}
		// up
		if (expandNode(exploreQueue, grid, 
			currPoint.x, currPoint.y + 1, pathPoint, DOWN)) {
			break;
		}
	}
	std::vector<Point<int>> path;

	if (exploreQueue.empty()) {
		// no path found
		return path;
	}

	while (dir(grid(pathPoint)) != NONE && pathPoint != currPos) {
		path.push_back(pathPoint);
		switch (dir(grid(pathPoint))) {
			case LEFT:
				pathPoint.x -= 1;
				break;
			case RIGHT:
				pathPoint.x += 1;
				break;
			case DOWN:
				pathPoint.y -= 1;
				break;
			case UP:
				pathPoint.y += 1;
				break;
		}
	}

	return path;
}


}
