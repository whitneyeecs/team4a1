#ifndef ENCODER_HANDLER_HPP
#define ENCODER_HANDLER_HPP

#include <stdint.h>
#include <lcmtypes/maebot_motor_feedback_t.hpp>

namespace eecs467 {

class EncoderHandler {
private:
	int64_t _currUtime;
	int32_t _currRightEncoder;
	int32_t _currLeftEncoder;

	int64_t _lastUtime;
	int32_t _lastRightEncoder;
	int32_t _lastLeftEncoder;

public:
	EncoderHandler(int64_t utime = -1, 
		int32_t rightEncoder = 0, 
		int32_t leftEncoder = 0);

	void update(int64_t utime, int32_t rightEncoder, int32_t leftEncoder);

	void update(const maebot_motor_feedback_t& feedback);

	void resetTicks(int64_t utime, int32_t rightEncoder, int32_t leftEncoder);

	/**
	 * @brief gets difference between right encoder ticks
	 * between most recent and last encoder values
	 */
	int32_t deltaRight() const;

	/**
	 * @brief gets difference between left encoder ticks
	 * between most recent and last encoder values
	 */
	int32_t deltaLeft() const;

	/**
	 * @brief gets difference in time from most recent encoder 
	 * to last encoder values
	 */
	int32_t deltaTime() const;

	/**
	 * @brief gets right wheel velocity in meters / second
	 */
	float velRight() const;

	/**
	 * @brief gets left wheel velocity in meters / second
	 */
	float velLeft() const;
};

}

#endif /* ENCODER_HANDLER_HPP */
