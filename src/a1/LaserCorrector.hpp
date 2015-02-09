#ifndef LASER_CORRECTOR_HPP
#define LASER_CORRECTOR_HPP

#include <vector>
#include <list>
#include <deque>

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
		int64_t utime;
		float intensity;
		float posX;
		float posY;
	};

	std::deque<SingleLaser> _scansToProcess;
	maebot_processed_laser_scan_t _processedScans;

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
	bool pushNewScans(const maebot_laser_scan_t& scan);

	/**
	 * @brief pushes a new pose onto the queue
	 * @param pose [description]
	 */
	void pushNewPose(const maebot_pose_t& pose);

	/**
	 * @brief tries to process as many laser ranges as possible
	 */
	void process();

	maebot_processed_laser_scan_t single_scan_process(const maebot_laser_scan_t& msg,
		const maebot_pose_t& begin,
		const maebot_pose_t& end);

	void pf_process(maebot_laser_scan_t* msg);

	void clearPoses();

	void clearScans();

	/**
	 * @brief does the same thing as process however it will not
	 * clear the scans after it's processed it (so when this is
	 * called again it will be processing the same scans);
	 * @details [long description]
	 */
	void processSaveScans();

	/**
	 * @brief fills an lcm msg with corrected scans
	 * @details will only work if all scans have been processed
	 * this will also clean out internal memory that saved the processed scans
	 * @param msg msg to fill
	 * @return returns true if msg was succesfully filled
	 */
	 bool getCorrectedLcmMsg(maebot_processed_laser_scan_t& msg);
};

}

#endif /* LASER_CORRECTOR_HPP */
