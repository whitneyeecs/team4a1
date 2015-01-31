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
		float scale;
	};

	std::deque<SingleLaser> _scansToProcess;
	// maebot_processed_laser_scan_t* _currMsg;
	std::list<maebot_processed_laser_scan_t> _msgQueue;
	std::list<maebot_processed_laser_scan_t>::iterator _currMsg;

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

	/**
	 * @brief fills an lcm msg with corrected scans
	 * @details will only work if all scans have been processed
	 * this will also clean out internal memory that saved the processed scans
	 * @param msg msg to fill
	 * @return returns true if msg was succesfully filled
	 */
	 bool getCorrectedLcmMsg(maebot_processed_laser_scan_t& msg);

private:
	static bool createLcmMsg(std::vector<SingleLaser> scan,
		maebot_processed_laser_scan_t& msg);
};

}

#endif /* LASER_CORRECTOR_HPP */
