#include "math/angle_functions.hpp"
#include "a1/SlamConstants.hpp"
#include "a1/RobotConstants.hpp"
#include "a1/Explore.hpp"

#include <deque>

namespace eecs467 {

enum Direction : char { NONE, LEFT, RIGHT, DOWN, UP };

enum State : char { EMPTY = 0, WALL = 1 << 4, UNKNOWN = 2 < 4 };

const char StateMask = 0xF0;
const char DirectionMask = 0x0F;

#define state(x) (x & StateMask)
#define dir(x) (x & DirectionMask)

Explore::Explore() { }

Point<int> Explore::getNextWayPoint(const OccupancyGrid& grid, const Point<double>& currPos) {
	return Point<int>();
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
					theta += deltaTheta) {
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

std::vector<Point<int>> Explore::pickWayPoints(const OccupancyGrid& grid, 
	const std::vector<Point<int>>& points) {
	return std::vector<Point<int>>();
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