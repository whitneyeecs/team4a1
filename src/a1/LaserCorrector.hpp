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
 * This is operated in two ways. you can use either
 * 1) just the processSingleScan() function to process a 
 * single scan if you happen to know begin and end poses from before and after the laser
 * 2) push scans and poses and call process() to process as many scans
 * as possible given the poses
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

	/**
	 * @brief clears all poses
	 */
	void clearPoses();

	/**
	 * @brief clears all scans
	 */
	void clearScans();

	/**
	 * @brief fills an lcm msg with corrected scans
	 * @details will only work if all scans have been processed
	 * this will also clean out internal memory that saved the processed scans
	 * @param msg msg to fill
	 * @return returns true if msg was succesfully filled
	 */
	 bool getCorrectedLcmMsg(maebot_processed_laser_scan_t& msg);

	 /**
	 * @brief processes a single scan
	 * @param msg lasers to process
	 * @param begin pose that has a timestamp earlier than the earliest timestamped laser
	 * @param end pose that has a timestamp later than the last laser timestamp
	 * @return processed lasers
	 */
	maebot_processed_laser_scan_t processSingleScan(const maebot_laser_scan_t& msg,
		const maebot_pose_t& begin,
		const maebot_pose_t& end);
};

}

#endif /* LASER_CORRECTOR_HPP */
