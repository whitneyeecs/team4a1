#include "math/angle_functions.hpp"
#include "a1/SlamConstants.hpp"
#include "a1/RobotConstants.hpp"
#include "a1/Explore.hpp"

#include <deque>

namespace eecs467 {

Explore::Explore() { }

Point<int> Explore::getNextWayPoint(const OccupancyGrid& grid, const Point<double>& currPos) {

	//OccupancyGrid space = getConfigurationSpace(grid, baseLength/2);
	
	//std::vector< Point<int> > points = breadthfirstSearch(space, currPos);

	//points = pickWayPoints(space, points);

	//_waypoints = points;

	Point<int> waypoint = _wayPoints.back();
	_wayPoints.pop_back();

	return waypoint;
}

void Explore::clearPath() {
	_wayPoints.clear();

}

OccupancyGrid Explore::getConfigurationSpace(const OccupancyGrid& grid, float radius) {
	eecs467::OccupancyGrid configSpace(grid.widthInMeters(),
		grid.heightInMeters(), grid.metersPerCell());

	float cellDiagonal = sqrt(2) * grid.metersPerCell();
	float deltaTheta = (cellDiagonal / 2) / radius;

	for (int y = 0; y < (int)grid.heightInCells(); y++) {
		for (int x = 0; x < (int)grid.widthInCells(); x++) {
			// draw circle around point
			if (grid(x, y) > eecs467::wallThreshold) {
				Point<int> gridOriginPt{x, y};
				Point<double> globalOriginPt = 
					grid_position_to_global_position(gridOriginPt, grid);
				for (float theta = 0; theta < 2 * M_PI; 
					theta += deltaTheta) {const OccupancyGrid& grid
					Point<double> edgePt{globalOriginPt.x + cos(theta), 
						globalOriginPt.y + sin(theta)};
					Point<int> gridEdgePt = 
						global_position_to_grid_cell(edgePt, grid);
						if (grid.isCellInGrid(gridEdgePt.x, gridEdgePt.y)) {
							configSpace(gridEdgePt.x, gridEdgePt.y) = WALL;
						}
				}
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

std::vector<Point<int>> Explore::pickWayPoints(const OccupancyGrid& grid, const std::vector<Point<int>>& points) {

	std::vector< Point<int> > updateWaypoints;

	Point<int> startPoint;
	Point<int> checkPoint; 
	
	int s = 0; //index of startPoint
	startPoint = points[s];
	updateWaypoints.push_back(startPoint);	

	for(unsigned int i = 1; i < points.size(); i++) {
		
		checkPoint = points[i];
		int step = i - s; //# of steps checkpoint is away from startpoint

		if(detectObstacle(grid, startPoint, checkPoint, step)) {
			updateWaypoints.push_back(points[i-1]);
			startPoint = checkPoint;
			s = i; //update index of startpoint
		}

		else 
			startPoint = startPoint;
	}

	return updateWaypoints;
}

std::vector<Point<int>> Explore::breadthFirstSearch(OccupancyGrid& grid, 
	const Point<int>& currPos) {
	std::deque<Point<int>> exploreQueue;
	exploreQueue.push_back(currPos);
	grid(currPos) = NONE;

	while (!exploreQueue.empty()) {
		Point<int> currPoint = exploreQueue.front();
		if (state(grid(currPoint)) == UNKNOWN) {
			break;
		}

		exploreQueue.pop_front();

		// left
		if (currPoint.x > 0 && 
			state(grid(currPoint.x - 1, currPoint.y)) == EMPTY) {
			exploreQueue.push_back(Point<int>{currPoint.x - 1, currPoint.y});
			grid(currPoint.x - 1, currPoint.y) = WALL | RIGHT;
		}
		// right
		if (currPoint.x < grid.widthInCells() -1 && 
			state(grid(currPoint.x + 1, currPoint.y)) == EMPTY) {
			exploreQueue.push_back(Point<int>{currPoint.x + 1, currPoint.y});
			grid(currPoint.x + 1, currPoint.y) = WALL | LEFT;
		}
		// down
		if (currPoint.y > 0 &&
			state(grid(currPoint.x, currPoint.y - 1)) == EMPTY) {
			exploreQueue.push_back(Point<int>{currPoint.x, currPoint.y - 1});
			grid(currPoint.x, currPoint.y - 1) = WALL | UP;
		}
		// up
		if (currPoint.y < grid.heightInCells() - 1 &&
			state(grid(currPoint.x, currPoint.y + 1)) == EMPTY) {
			exploreQueue.push_back(Point<int>{currPoint.x, currPoint.y + 1});
			grid(currPoint.x, currPoint.y + 1) = WALL | DOWN;
		}
	}
	std::vector<Point<int>> path;

	if (exploreQueue.empty()) {
		// no path found
		return path;
	}

	Point<int> pathPoint = exploreQueue.front();
	while (dir(grid(pathPoint)) != NONE) {
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
