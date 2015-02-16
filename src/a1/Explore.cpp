#include "Explore.hpp"
#include "math/angle_functions.hpp"
#include "a1/SlamConstants.hpp"
#include "a1/RobotConstants.hpp"

namespace eecs467 {

Explore::Explore() { }

Point<int> Explore::getNextWayPoint(const OccupancyGrid& grid, const Point<double>& currPos) {

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
							configSpace(gridEdgePt.x, gridEdgePt.y) = 0xFF;
						}
				}
			}
		}
	}
	return configSpace;
}

std::vector<Point<int>> Explore::pickWayPoints(const OccupancyGrid& grid, const std::vector<Point<int>>& points) {

}

std::vector<Point<int>> Explore::breadthFirstSearch(const OccupancyGrid& grid, const Point<double>& currPos) {

}


}