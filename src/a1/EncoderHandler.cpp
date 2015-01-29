#include "EncoderHandler.hpp"
#include "RobotConstants.hpp"

eecs467::EncoderHandler::EncoderHandler(int64_t utime,
	int32_t rightEncoder, int32_t leftEncoder) :
	_currUtime(utime), 
	_currRightEncoder(rightEncoder),
	_currLeftEncoder(leftEncoder), 
	_lastUtime(utime),
	_lastRightEncoder(rightEncoder), 
	_lastLeftEncoder(leftEncoder) { }

void eecs467::EncoderHandler::update(int64_t utime, 
	int32_t rightEncoder, 
	int32_t leftEncoder) {
	if (_currUtime == -1) {
		_currUtime = utime;
		_currRightEncoder = rightEncoder;
		_currLeftEncoder = leftEncoder;

		_lastUtime = utime;
		_lastRightEncoder = rightEncoder;
		_lastLeftEncoder = leftEncoder;
	} else {
		_lastUtime = _currUtime;
		_lastRightEncoder = _currRightEncoder;
		_lastLeftEncoder = _currLeftEncoder;

		_currUtime = utime;
		_currRightEncoder = rightEncoder;
		_currLeftEncoder = leftEncoder;
	}
}

void eecs467::EncoderHandler::update(const maebot_motor_feedback_t& feedback) {
	update(feedback.utime, feedback.encoder_right_ticks,
		feedback.encoder_left_ticks);
}

void eecs467::EncoderHandler::resetTicks(int64_t utime, int32_t rightEncoder, 
	int32_t leftEncoder) {
	_currUtime = utime;
	_currRightEncoder = rightEncoder;
	_currLeftEncoder = leftEncoder;

	_lastUtime = utime;
	_lastRightEncoder = rightEncoder;
	_lastLeftEncoder = leftEncoder;
}

int32_t eecs467::EncoderHandler::deltaRight() const {
	return _currRightEncoder - _lastRightEncoder;
}

int32_t eecs467::EncoderHandler::deltaLeft() const {
	return _currLeftEncoder - _lastLeftEncoder;
}

int32_t eecs467::EncoderHandler::deltaTime() const {
	return _currUtime - _lastUtime;
}

float eecs467::EncoderHandler::velRight() const {
	float deltaTime = (float) this->deltaTime() / 1e6;
	if (deltaTime == 0) {
		return 0;
	} else {
		return eecs467::metersPerTick * (float) deltaRight() / deltaTime;
	}
}

float eecs467::EncoderHandler::velLeft() const {
	int32_t deltaTime = (float) this->deltaTime() / 1e6;
	if (deltaTime == 0) {
		return 0;
	} else {
		return eecs467::metersPerTick * (float) deltaLeft() / deltaTime;
	}
}
