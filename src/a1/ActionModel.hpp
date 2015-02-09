#ifndef ACTION_MODEL_HPP
#define ACTION_MODEL_HPP

#include <stdint.h>
#include <random>

#include "lcmtypes/maebot_pose_t.hpp"

namespace eecs467 {

class ActionModel {
private:
	std::mt19937 _randGen;
	std::normal_distribution<float> _normDist;
	float _k1;
	float _k2;
public:
	ActionModel(float k1, float k2);

	void apply(maebot_pose_t& pose, int32_t deltaRight, int32_t deltaLeft, int64_t deltaTime);
};

}

#endif /* ACTION_MODEL_HPP */
