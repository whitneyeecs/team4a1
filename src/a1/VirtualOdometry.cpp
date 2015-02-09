#include "VirtualOdometry.hpp"

eecs467::VirtualOdometry::VirtualOdometry(int32_t rightTick, int32_t leftTick, int64_t utime) :
	_lastRightTick(rightTick), _lastLeftTick(leftTick), _lastUtime(utime),
	_currRightTick(rightTick), _currLeftTick(leftTick), _currUtime(utime) { }

void eecs467::VirtualOdometry::set(int32_t rightTick, int32_t leftTick, int64_t utime) {
	_lastRightTick = rightTick;
	_lastLeftTick = leftTick;
	_lastUtime = utime;
}

void eecs467::VirtualOdometry::set(const maebot_motor_feedback_t& msg) {
	set(msg.encoder_right_ticks, msg.encoder_left_ticks, msg.utime);
}

void eecs467::VirtualOdometry::set(const std::array<int32_t, 2>& ticks, int64_t utime) {
	set(ticks[0], ticks[1], utime);
}

void eecs467::VirtualOdometry::update(int32_t rightTick, int32_t leftTick, int64_t utime) {
	_currRightTick = rightTick;
	_currLeftTick = leftTick;
	_currUtime = utime;

	if (_lastUtime == -1) {
		set(rightTick, leftTick, utime);
	}
}

void eecs467::VirtualOdometry::update(const maebot_motor_feedback_t& msg) {
	update(msg.encoder_right_ticks, msg.encoder_left_ticks,
		msg.utime);
}

void eecs467::VirtualOdometry::update(const std::array<int32_t, 2>& ticks, int64_t utime) {
	update(ticks[0], ticks[1], utime);
}

std::array<int32_t, 2> eecs467::VirtualOdometry::interpolate(int64_t utime) {
	std::array<int32_t, 2> ret;

	float scaling = (float) (utime - _lastUtime) / (float) (_currUtime - _lastUtime);
	ret[0] = _lastRightTick + scaling * (float)(_currRightTick - _lastRightTick);
	ret[1] = _lastLeftTick + scaling * (float)(_currLeftTick - _lastLeftTick);
	return ret;
}

std::array<int32_t, 2> eecs467::VirtualOdometry::deltas(const std::array<int32_t, 2>& odo) const {
	std::array<int32_t, 2> ret;
	ret[0] = odo[0] - _lastRightTick;
	ret[1] = odo[1] - _lastLeftTick;
	return ret;
}


