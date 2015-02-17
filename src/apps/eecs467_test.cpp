#include "a1/Explore.hpp"

using namespace eecs467;

void printGrid(const OccupancyGrid& grid) {
	for (int y = 0; y < grid.heightInCells(); y++) {
		for (int x = 0; x < grid.widthInCells(); x++) {
			printf("%x\t", grid(x, y));
		}
		printf("\n");
	}
}


int main() {
	OccupancyGrid grid(10, 10, 0.5);
	for (int i = 0; i < grid.heightInCells() / 2; ++i) {
		grid(10, i) = 120;
	}
	for (int i = 0; i < grid.heightInCells(); ++i) {
		grid(14, i) = 10;
	}
	OccupancyGrid grid2 = grid;
	grid2(0, 0) = 99;

	printGrid(grid);
	std::cout << std::endl;
	printGrid(grid2);

	// OccupancyGrid config = Explore::getConfigurationSpace(grid, 0.6);
	// printGrid(config);

	// printf("\n%d\t%d\n", WALL, EMPTY);
}