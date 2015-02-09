#ifndef VIRTUAL_ODOMETRY_HPP
#define VIRTUAL_ODOMETRY_HPP

#include <stdint.h>
#include <array>

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

	/**
	 * @brief sets the last tick that deltas will be taken from
	 * @details deltas will be newOdo - setOdo, where setOdo
	 * is the odometry that has been set using this function
	 * @param rightTick right wheel ticks
	 * @param leftTick left wheel ticks
	 * @param utime utime stamp
	 */
	void set(int32_t rightTick, int32_t leftTick, int64_t utime);

	/**
	 * @brief sets the last tick that deltas will be taken from
	 * @param msg msg to set
	 */
	void set(const maebot_motor_feedback_t& msg);

	/**
	 * @brief set last tick taht deltas will be taken from
	 * @param ticks index 0 is right wheel ticks, index 1 is left wheel ticks
	 * @param utime utime stamp
	 */
	void set(const std::array<int32_t, 2>& ticks, int64_t utime);

	/**
	 * @brief updates the latest tick the virtual odometry keeps track of 
	 * @details will replace the last updated odometry
	 * @param rightTick right wheel ticks
	 * @param leftTick left wheel ticks
	 * @param utime utime stamp
	 */
	void update(int32_t rightTick, int32_t leftTick, int64_t utime);

	/**
	 * @brief updates the latest tick the virtual odometry keeps track of
	 * @details will replace the last updated odometry
	 * @param msg message containing right and left ticks
	 */
	void update(const maebot_motor_feedback_t& msg);

	/**
	 * @brief updates the latest tick the virtual odometry keeps track of
	 * @details will replace the last updated odometry
	 * @param ticks index 0 is right wheel ticks, index 1 is left wheel ticks
	 * @param utime utime stamp
	 */
	void update(const std::array<int32_t, 2>& ticks, int64_t utime);

	/**
	 * @brief interpolates odometry between odometry that has been set and updated
	 * @details utime should be between the utime of the odometry
	 * that is set and the update odometry
	 * @param utime time to interpolate ticks
	 * @returns array with index 0 as right wheel ticks and index 1 as left wheel ticks
	 */
	std::array<int32_t, 2> interpolate(int64_t utime);

	/**
	 * @brief gets deltas from odo to set odometry
	 * @param odo odometry to get deltas from, index 0 is right wheel, index 1 is left wheel
	 * @return deltas, index 0 is right wheel, index 1 is left wheel
	 */
	std::array<int32_t, 2> deltas(const std::array<int32_t, 2>& odo) const;
};

}

#endif /* VIRTUAL_ODOMETRY_HPP */
