#ifndef ACTION_MODEL_HPP
#define ACTION_MODEL_HPP

#include <stdint.h>

#include "lcmtypes/maebot_pose_t.hpp"

#include "math/gsl_util_rand.h"

namespace eecs467 {

class ActionModel {
private:
	gsl_rng* _rand;
	float _k1;
	float _k2;
public:
	ActionModel(float k1, float k2);

	/**
	 * @brief applies action model to a pose
	 * @param pose pose to apply action model to
	 * @param deltaRight right wheel ticks
	 * @param deltaLeft left wheel ticks
	 * @param utime new time for the pose
	 */
	void apply(maebot_pose_t& pose, int32_t deltaRight, int32_t deltaLeft, int64_t utime);
};

}

#endif /* ACTION_MODEL_HPP */
