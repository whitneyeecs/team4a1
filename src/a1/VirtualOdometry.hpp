#ifndef VIRTUAL_ODOMETRY_HPP
#define VIRTUAL_ODOMETRY_HPP

#include <stdint.h>

#include "lcmtypes/maebot_motor_feedback_t.hpp"

namespace eecs467 {

class VirtualOdometry {
private:
	int32_t _lastRightTick;
	int32_t _lastLeftTick;
	int64_t _lastUtime;

	int32_t _currRightTick;
	int32_t _currLeftTick;
	int64_t _currUtime;

public:
	VirtualOdometry(int32_t rightTick = 0, int32_t leftTick = 0, int64_t utime = -1);

	void setOdometry(int32_t rightTick, int32_t leftTick, int64_t utime);

	void update(int32_t rightTick, int32_t leftTick, int64_t utime, int64_t measuredUtime);

	void update(const maebot_motor_feedback_t& msg, int64_t measuredUtime);

	int32_t getDeltaRight() const;

	int32_t getDeltaLeft() const;

	int64_t getUtime() const;

	int32_t getRightTicks() const;

	int32_t getLeftTicks() const;
};

}

#endif /* VIRTUAL_ODOMETRY_HPP */
