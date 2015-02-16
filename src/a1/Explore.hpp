#ifndef EXPLORE_HPP
#define EXPLORE_HPP

#include <deque>
#include <list>
#include <algorithm>

#include <mapping/occupancy_grid.hpp>
#include <mapping/occupancy_grid_utils.hpp>
#include <lcmtypes/maebot_processed_laser_scan_t.hpp>

namespace eecs467 {

class Explore {
private:
	float exploreX; //grid cell coordinates
	float exploreY; //grid cell coordinates

	struct Beam {
		float range;
		float theta;
		float posX;
		float posY;
	};
	
	std::vector<Beam> beams;
	std::vector<Point<int>> locs;

	static bool sortRanges(const Beam& a, const Beam& b)
	{
		return a.range < b.range;
	}

public:
	Explore();

	/**
	 * @brief takes a processed scan and occupancy grid, to determine the 
     * furthest unkown cell. puts the rest of the cell locs on a deque with the
     * furthest cells going in the deque first
	*/
	void furthestUnkownLoc(const maebot_processed_laser_scan_t& msg, const OccupancyGrid& grid);

	OccupancyGrid getConfigurationSpace(const OccupancyGrid& grid);

	/**
	  * assume we will need an actually "drive" or "explore" function. the 
      * next location to explore exists in this space so the function
      * would not need any inputs. just a function on how to actually.
	  * get to the next location.
	*/
	void drive();

	/**
	 * @brief clears all beams
	 */
	void clearBeams();

	/**
	 * @brief clears all Locs
	 */
	void clearLocs();	
};

}

#endif /* EXPLORE_HPP */
