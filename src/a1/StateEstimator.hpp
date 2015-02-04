#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

// C++
#include <stdint.h>


// lcm types
#include "lcmtypes/maebot_pose_t.h"

namespace eecs467 {

/**
 * @brief simple funtion that takes a pose and changes it by delta tick values
 * @param pose pose to advance
 * @param deltaRight number of ticks of right wheel since last pose
 * @param deltaLeft number of ticks of left wheel since last pose
 * @return new pose
 */
maebot_pose_t advanceState(const maebot_pose_t& pose, int32_t deltaRight, int32_t deltaLeft, int64_t deltaTime);

}

#endif /* STATE_ESTIMATOR_HPP */
