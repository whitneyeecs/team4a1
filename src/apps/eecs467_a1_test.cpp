#include <stdio.h>
#include <stdint.h>

#include "a1/VirtualOdometry.hpp"
#include "a1/StateEstimator.hpp"

int main() {
	eecs467::VirtualOdometry vo(0, 0, 0);

	for (int i = 1; i <= 100; ++i) {
		vo.update(i * 10, i * 20, i * 10, i * 10 - 2);
	}

	printf("%d\t%d\t%ld\n", vo.getDeltaRight(), vo.getDeltaLeft(), vo.getUtime());

	printf("%d\t%d\t%ld\n", vo.getRightTicks(), vo.getLeftTicks(), vo.getUtime());
}