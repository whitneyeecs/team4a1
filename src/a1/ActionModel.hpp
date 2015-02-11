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
	/**
	 * @param k1 angular error factor
	 * @param k2 distance error factor
	 */
	ActionModel(float k1, float k2);

	/**
	 * @brief applies action model to a pose
	 * @param pose pose to apply action model to
	 * @param deltaRight right wheel ticks
	 * @param deltaLeft left wheel ticks
	 * @param utime new time for the pose
	 */
	void apply(maebot_pose_t& pose, int32_t deltaRight, int32_t deltaLeft, int64_t utime);

	/**
	 * @brief applies action model to pose has extra parameter
	 * for when this is called a lot so it saves computation of deltaS
	 * @param pose pose to apply action model to
	 * @param deltaRight right wheel ticks
	 * @param deltaLeft left wheel ticks
	 * @param deltaS precomputed sqrt(deltaRight^2 + deltaLeft^2)
	 * @param utime new time for the pose
	 */
	void apply(maebot_pose_t& pose, int32_t deltaRight, int32_t deltaLeft, float deltaS, int64_t utime);
};

}

#endif /* ACTION_MODEL_HPP */
