#ifndef EXPLORE_HPP
#define EXPLORE_HPP

#include <vector>
#include <algorithm>

#include "a1/SlamConstants.hpp"

#include <math/point.hpp>
#include <mapping/occupancy_grid.hpp>
#include <mapping/occupancy_grid_utils.hpp>
#include <lcmtypes/maebot_processed_laser_scan_t.hpp>

namespace eecs467 {

// for use with breadth first search and configuration space
enum Direction : char { NONE, LEFT, RIGHT, DOWN, UP };

enum State : char { EMPTY = 0, WALL = 1 << 4, UNKNOWN = 2 < 4 };

const char StateMask = 0xF0;
const char DirectionMask = 0x0F;

#define state(x) (x & StateMask)
#define dir(x) (x & DirectionMask)

/**
 * @brief this class handles choosing points to go explore
 */
class Explore {
private:
	std::vector<Point<int>> _wayPoints;

	bool detectObstacle(const OccupancyGrid& grid, const Point<int>& start, const Point<int>& end, double steps){
		
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
	
			Point<int> cellPos = global_position_to_grid_cell(point, grid);

			if(grid.logOdds(cellPos.y, cellPos.y) > eecs467::wallThreshold)
				return false; //hit a wall

		}
		
		return false;
	}

public:
	Explore();

	Point<int> getNextWayPoint(const OccupancyGrid& grid, const Point<double>& currPos);

	void clearPath();

	/**
	 * @brief generates a configuration space
	 * @param grid reference to occupancy grid
	 * @param radius radius of the robot
	 * @return configuration space
	 */
	static OccupancyGrid getConfigurationSpace(const OccupancyGrid&, float radius);

private:
	/**
	 * @brief takes in a path generated by breadthFirstSearch and picks
	 * important waypoints from it (aka corners and whatnot)
	 * @param grid reference to our configuration space
	 * @param points path where the first element is the destination and the last element is the start
	 * @return specific points from the path where the first element is the destination
	 * and the last element is the start
	 */
	std::vector<Point<int>> pickWayPoints(const OccupancyGrid& grid, 
		const std::vector<Point<int>>& points);

	/**
	 * @brief generates a vector of points consisting of 
	 * adjacent cells that represent a path from our position 
	 * to the nearest unknown location
	 * @param grid const ref to a configuration space
	 * @return path where the first element is the destination and the last element is the start
	 */
	std::vector<Point<int>> breadthFirstSearch(OccupancyGrid& grid, 
		const Point<int>& currPos);

};

}

#endif /* EXPLORE_HPP */
