#ifndef SLAM_CONSTANTS_HPP
#define SLAM_CONSTANTS_HPP

namespace eecs467 {

// number of particles drawn from prior
static const int numPreviousParticles = 1000;
// number of particles to be sampled randomly around most probable particle
static const int numRandomParticles = 300;

// mapping strengths
static const int emptyEvidenceStrength = 2;
static const int occupiedEvidenceStrength = 1;

// action model constants
static const float actionModelK1 = 0.01;
static const float actionModelK2 = 0.1;

// sensor model constants
static const float sensorModelStepsPerLaser = 5;
static const float sensorModelOutOfGrid = 0;
static const float sensorModelWall = 0;
static const float sensorModelOpen = 1.5;
static const float sensorModelUnexplored = 1.5;

}

#endif /* SLAM_CONSTANTS_HPP */
