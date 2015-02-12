#ifndef SLAM_CONSTANTS_HPP
#define SLAM_CONSTANTS_HPP

// constants used in slam

namespace eecs467 {

// define ODOMETRY_COMPENSATE
// if you want to use constants
// that more heavily penalize unexplored spaces
// so as to depend on odometry more
#define ODOMETRY_COMPENSATE	

// define SENSOR_RAY_TRACE if you want to
// use ray tracing in the sensor model
// instead of the default which is just looking
// at the end points
// #define SENSOR_RAY_TRACE

// #define CIRCLE_RANDOM_DRAW_PARTICLES

// number of particles drawn from prior
static const int numPreviousParticles = 1000;
// number of particles to be sampled randomly around most probable particle
static const int numRandomParticles = 300;

static const float randParticleSpread = 0.1;

// mapping strengths
static const int emptyEvidenceStrength = 2;
static const int occupiedEvidenceStrength = 1;

// action model constants
static const float actionModelK1 = 0.01;
static const float actionModelK2 = 0.1;


static const float sensorModelStepsPerLaser = 5;
static const float odometryCompensateThreshold = -400;

// sensor model constants
#ifdef ODOMETRY_COMPENSATE
// constants for applyEnd
static const float sensorModelOutOfGrid = 0;
static const float sensorModelWall = 0;
static const float sensorModelOpen = 1.5;
static const float sensorModelUnexplored = 2;

// constants for applyRayTrace

static const float sensorStartOutOfGrid = 30;
static const float sensorStartWall = 5;
static const float sensorStartOpen = 0;
static const float sensorStartUnexplored = 0;

static const float sensorMiddleOutOfGrid = 0;
static const float sensorMiddleWall = 0.5;
static const float sensorMiddleOpen = 0;
static const float sensorMiddleUnexplored = 0.5;

static const float sensorEndOutOfGrid = 0;
static const float sensorEndWall = 0;
static const float sensorEndOpen = 1.5;
static const float sensorEndUnexplored = 1.5;
#else
// constants for applyEnd
static const float sensorModelOutOfGrid = 0;
static const float sensorModelWall = 0;
static const float sensorModelOpen = 1.5;
static const float sensorModelUnexplored = 0;

// constants for applyRayTrace
static const float sensorStartOutOfGrid = 30;
static const float sensorStartWall = 5;
static const float sensorStartOpen = 0;
static const float sensorStartUnexplored = 0;

static const float sensorMiddleOutOfGrid = 0;
static const float sensorMiddleWall = 0.5;
static const float sensorMiddleOpen = 0;
static const float sensorMiddleUnexplored = 0;

static const float sensorEndOutOfGrid = 0;
static const float sensorEndWall = 0;
static const float sensorEndOpen = 1.5;
static const float sensorEndUnexplored = 0;
#endif /* ODOMETRY_COMPENSATE */

// thresholds for whether something is counted as a wall or empty space
static const int wallThreshold = 100;
static const int emptyThreshold = -100;

// grid stuff
static const float gridWidthMeters = 5;
static const float gridHeightMeters = 5;
static const float gridCellSizeMeters = 0.05;
static const float gridSeparationSize = 1; // in cells

}

#endif /* SLAM_CONSTANTS_HPP */
