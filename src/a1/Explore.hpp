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
	
	std::list <Beam> beams;
	std::deque < Point<int> > locs;

	struct sortRanges{
		bool operator() (const Beam *a, const Beam *b)
		{
			return a->range < b->range;
		}
	};

	//bool sortRanges (const Beam& a, const Beam& b) { 
	//	return a.range < b.range; // greatest to least sort
	//}

public:
	Explore();

	/**
	 * @brief takes a processed scan and occupancy grid, to determine the 
     * furthest unkown cell. puts the rest of the cell locs on a deque with the
     * furthest cells going in the deque first
	*/
	void furthestUnkownLoc(const maebot_processed_laser_scan_t& msg, const OccupancyGrid& grid);

	/**
	  * assume we will need an actuall "drive" or "explore" function. the 
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
