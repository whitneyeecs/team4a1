#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

#include <stdint.h>
#include "lcmtypes/maebot_pose_t.h"

namespace eecs467 {

/**
 * @brief State Estimator for the maebot
 */
class StateEstimator {
private:
	maebot_pose_t _pose;

public:
	StateEstimator(float x = 0, float y = 0, float heading = 0);

	/**
	 * @brief updates the state with a given velocity and timestamp
	 * @details [long description]
	 * 
	 * @param velRIght velocity of right wheel
	 * @param velLeft velocity of left wheel
	 * @param utime [description]
	 */
	void update(float velRight, float velLeft, int64_t utime);

	void update(float x, float y, float theta, int64_t utime);

	void update(const maebot_pose_t& pose);

	maebot_pose_t getLatestPoseCopy() const;

	const maebot_pose_t& getLatestPose() const;
};

}




#endif /* STATE_ESTIMATOR_HPP */