#ifndef SLAM_CONSTANTS_HPP
#define SLAM_CONSTANTS_HPP

namespace eecs467 {

static const int numParticles = 1000;

static const int emptyEvidenceStrength = 2;
static const int occupiedEvidenceStrength = 1;

static const float actionModelK1 = 0.01;
static const float actionModelK2 = 0.1;

static const float sensorModelStepsPerLaser = 5;

}

#endif /* SLAM_CONSTANTS_HPP */
