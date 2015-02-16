#include "Explore.hpp"
#include "math/angle_functions.hpp"
#include "a1/SlamConstants.hpp"
#include "a1/RobotConstants.hpp"

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

	// sort list of laser beams
	std::sort(beams.begin(), beams.end(), eecs467::Explore::sortRanges);

	Point<int> cellPos = { -1, -1 };

	for(auto iter = beams.begin(); iter != beams.end(); iter++){
		
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

eecs467::OccupancyGrid eecs467::Explore::getConfigurationSpace(const OccupancyGrid& grid) {
	eecs467::OccupancyGrid configSpace(grid.widthInMeters(),
		grid.heightInMeters(), grid.metersPerCell());

	float cellDiagonal = sqrt(2) * grid.metersPerCell();
	float deltaTheta = (cellDiagonal / 2) / (eecs467::baseLength / 2);

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
							configSpace(gridEdgePt.x, gridEdgePt.y) = 0xFF;
						}
				}
			}
		}
	}
	return configSpace;
}

void eecs467::Explore::drive(){ 

}

void eecs467::Explore::clearBeams() {
	beams.clear();
}

void eecs467::Explore::clearLocs(){
	locs.clear();
}
