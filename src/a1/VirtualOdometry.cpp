#include "VirtualOdometry.hpp"

eecs467::VirtualOdometry::VirtualOdometry(int32_t rightTick, int32_t leftTick, int64_t utime) :
	_lastRightTick(rightTick), _lastLeftTick(leftTick), _lastUtime(utime),
	_currRightTick(rightTick), _currLeftTick(leftTick), _currUtime(utime) { }

void eecs467::VirtualOdometry::setOdometry(int32_t rightTick, int32_t leftTick, int64_t utime) {
	_currRightTick = rightTick;
	_currLeftTick = leftTick;
	_currUtime = utime;
}

void eecs467::VirtualOdometry::update(int32_t rightTick, int32_t leftTick, int64_t utime, int64_t measuredUtime) {
	_lastRightTick = _currRightTick;
	_lastLeftTick = _currLeftTick;
	_lastUtime = _currUtime;

	float scaling = (float) (measuredUtime - _currUtime) / (float) (utime - _currUtime);
	// printf("scaling: %f\n", scaling);
	// printf("%f\t%f\n", (float)(rightTick - _currRightTick),
		// (float)(leftTick - _currLeftTick));
	_currRightTick += scaling * (float)(rightTick - _currRightTick);
	_currLeftTick += scaling * (float)(leftTick - _currLeftTick);
	_currUtime = measuredUtime;
}

void eecs467::VirtualOdometry::update(const maebot_motor_feedback_t& msg, int64_t measuredUtime) {
	update(msg.encoder_right_ticks, msg.encoder_left_ticks,
		msg.utime, measuredUtime);
}

int32_t eecs467::VirtualOdometry::getDeltaRight() const {
	return _currRightTick - _lastRightTick;
}

int32_t eecs467::VirtualOdometry::getDeltaLeft() const {
	return _currLeftTick - _lastLeftTick;
}

int64_t eecs467::VirtualOdometry::getUtime() const {
	return _currUtime;
}

int32_t eecs467::VirtualOdometry::getRightTicks() const {
	return _currRightTick;
}

int32_t eecs467::VirtualOdometry::getLeftTicks() const {
	return _currLeftTick;
}