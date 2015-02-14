#include "Explore.hpp"
#include "math/angle_functions.hpp"

eecs467::Explore::Explore() { }

void eecs467::Explore::furthestUnkownLoc(const maebot_processed_laser_scan_t& msg, const OccupancyGrid& grid){

	int i = 0;
	for ( ; i < msg.num_ranges; i++) {		

		Beam beam = { 
			msg.ranges[i],
			msg.thetas[i],
			msg.x_pos[i],
			msg.y_pos[i]
		};
	
	beams.push_back(beam);

	}

	//sort list of laser beams
	std::sort(beams.begin(), beams.end(), sortRanges);

	Point<int> cellPos = { -1, -1 };
	std::list<Beam>::iterator iter;

	for(iter = beams.begin(); iter != beams.end(); iter++){
		
		//calculate endpoints
		Point<float> point;
		point.x = iter->posX + iter->range * wrap_to_pi(cos(iter->theta));
		point.y = iter->posY + iter->range * wrap_to_pi(sin(iter->theta));

		cellPos = global_position_to_grid_cell(point, grid);

		if(grid.isCellInGrid(cellPos.x, cellPos.y)) {
			if(grid(cellPos.x, cellPos.y) == 0) { //unexplored ????????????				
				locs.push_back(cellPos); //add grid cell to deque
			}
		}		
	
	} 

	//pop off furthest cell 
	if(!locs.empty()) {
		cellPos = locs.back();
		locs.pop_back();
		exploreX = cellPos.x;
		exploreY = cellPos.y;
	}
	else {
		exploreX = exploreX;
		exploreY = exploreY;
	}		 

};

void eecs467::Explore::drive(){ 

}

void eecs467::Explore::clearBeams() {
	beams.clear();
}

void eecs467::Explore::clearLocs(){
	locs.clear();
}
