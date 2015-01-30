#ifndef ROBOT_CONSTANTS_HPP
#define ROBOT_CONSTANTS_HPP

#include <cmath>

namespace eecs467 {

static const float baseLength = 0.08; // meters
static const float metersPerTick = 0.00020944; // meters per tick of encoder
static const int maxNumLasersPerScan = 400; // a max number of laser readings per laser scan

/**
 * @brief takes theta given by lidar and returns it in the frame of the robot
 * @param laserTheta laser theta in radians
 * @return theta in robot frame in radians
 */
static inline float laserThetaToMaebotTheta(float laserTheta) {
	return - laserTheta;
}

}

#endif /* ROBOT_CONSTANTS_HPP */
