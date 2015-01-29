#ifndef LASER_CORRECTOR_HPP
#define LASER_CORRECTOR_HPP

#include <vector>
#include <list>

#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_processed_laser_scan_t.hpp>

namespace eecs467 {

/**
 * @brief Corrects laser thetas and start positions by the pose
 */
class LaserCorrector {
private:
	struct SingleLaser {
		float range;
		float theta;
		int64_t time;
		float intensity;
	};

	std::vector<SingleLaser> _scansToProcess;
	std::vector<SingleLaser> _processedScans;

	std::list<maebot_pose_t> _poses;

public:
	LaserCorrector();

	/**
	 * @brief puts each range in the scan onto
	 * a queue to be processed
	 * @param scan scan to be processed
	 * @return true if there are no other scans in process
	 * false if there are (and it will drop the scan)
	 */
	bool processNewScans(const maebot_laser_scan_t& scan);

	/**
	 * @brief pushes a new pose onto the queue
	 * @param pose [description]
	 */
	void pushNewPose(maebot_pose_t pose);

	/**
	 * @brief tries to process as many laser ranges as possible
	 * @return returns true of all lasers are processed
	 */
	bool process();

	/**
	 * @brief fills an lcm msg with corrected scans
	 * @details will only work if all scans have been processed
	 * @param msg msg to fill
	 * @return returns true if msg was succesfully filled
	 */
	bool createCorrectedLcmMsg(maebot_processed_laser_scan_t& msg);
};

}

#endif /* LASER_CORRECTOR_HPP */
