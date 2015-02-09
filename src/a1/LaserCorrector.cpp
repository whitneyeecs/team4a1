#include "LaserCorrector.hpp"
#include "RobotConstants.hpp"
#include "math/angle_functions.hpp"

eecs467::LaserCorrector::LaserCorrector() { }

bool eecs467::LaserCorrector::pushNewScans(const maebot_laser_scan_t& scan) {
	// printf("pushed scan: %ld\t%ld\n", scan.utime, scan.times[scan.num_ranges - 1]);
	for (int32_t i = 0; i < scan.num_ranges; ++i) {
		SingleLaser laser = { 
			scan.ranges[i], 
			scan.thetas[i],
			scan.times[i],
			scan.intensities[i], 0, 0 };
		_scansToProcess.push_back(laser);
	}
	return true;
}

void eecs467::LaserCorrector::pushNewPose(const maebot_pose_t& pose) {
	_poses.push_back(pose);
	// printf("pushed pose: %ld\n", pose.utime);
}

void eecs467::LaserCorrector::pf_process(maebot_laser_scan_t* msg) {


	// process until scansToProcess is empty
	for(int i  = 0; i < msg->num_ranges; ++i) {
		// the laser scan with the smallest timestamp still unprocessed
		SingleLaser laser = {
				msg->ranges[i],
				msg->thetas[i],
				msg->times[i],
				msg->intensities[i], 0, 0
		};

		maebot_pose_t oldest;
		// after this loop poses.front() should contain the first 
		// pose that has a greater time stamp or will become empty
		while(!_poses.empty() 
			&& (_poses.front().utime < laser.utime)) {
			oldest = _poses.front();
			_poses.pop_front();
		}

		// if all poses are used up, we will have to wait for more
		// pop last one back on and return false
		if (_poses.empty()) {
			_poses.push_front(oldest);
			return;
		}
 		
 		float delta_theta = _poses.front().theta - oldest.theta;
// 		if (delta_theta > 2) {
//printf("Turning too fast\n");
 			// we are turning too fast or somethings gone wrong with the pose
//			_scansToProcess.clear();
			// _poses.clear();
 //			return;
// 		}

		// interpolate the position of the vehicle for the scan
		float scaling = (float) (laser.utime - oldest.utime) /
			(float) (_poses.front().utime - oldest.utime);
		float poseX = oldest.x + scaling * (_poses.front().x - oldest.x);
		float poseY = oldest.y + scaling * (_poses.front().y - oldest.y);
		float poseTheta = oldest.theta + scaling * (_poses.front().theta - oldest.theta);

		// push processed scan into current message
		_processedScans.ranges.push_back(laser.range);
		_processedScans.thetas.push_back(angle_sum(poseTheta,
			laserThetaToMaebotTheta(laser.theta)));
		_processedScans.times.push_back(laser.utime);
		_processedScans.intensities.push_back(laser.intensity);
		_processedScans.x_pos.push_back(poseX);
		_processedScans.y_pos.push_back(poseY);

		// push older pose back on
//		_poses.push_front(oldest);

		// pop recently processed scan
//		_scansToProcess.pop_front();
	}
}

void eecs467::LaserCorrector::process() {
	// if there are no scans to process return false
	if (_scansToProcess.empty()) {
		return;
	}

	// if the smallest pose time is greater than the smallest scan time
	// then we have no pose before the scans to interpolate with
	// throwout the set of scans
	if (_poses.empty() || _poses.front().utime > _scansToProcess.back().utime) {
		// printf("first!\n");
		_scansToProcess.clear();
		return;
	}

	// process until scansToProcess is empty
	while (!_scansToProcess.empty()) {
		// the laser scan with the smallest timestamp still unprocessed
		SingleLaser& laser = _scansToProcess.front();

		maebot_pose_t oldest;
		// after this loop poses.front() should contain the first 
		// pose that has a greater time stamp or will become empty
		while(!_poses.empty() 
			&& (_poses.front().utime < laser.utime)) {
			oldest = _poses.front();
			_poses.pop_front();
		}

		// if all poses are used up, we will have to wait for more
		// pop last one back on and return false
		if (_poses.empty()) {
			_poses.push_front(oldest);
			return;
		}
 		
 		float delta_theta = _poses.front().theta - oldest.theta;
 		if (delta_theta > 2) {
 			// we are turning too fast or somethings gone wrong with the pose
			_scansToProcess.clear();
			// _poses.clear();
 			return;
 		}

		// interpolate the position of the vehicle for the scan
		float scaling = (float) (laser.utime - oldest.utime) /
			(float) (_poses.front().utime - oldest.utime);
		float poseX = oldest.x + scaling * (_poses.front().x - oldest.x);
		float poseY = oldest.y + scaling * (_poses.front().y - oldest.y);
		float poseTheta = oldest.theta + scaling * (_poses.front().theta - oldest.theta);

		// push processed scan into current message
		_processedScans.ranges.push_back(laser.range);
		_processedScans.thetas.push_back(angle_sum(poseTheta,
			laserThetaToMaebotTheta(laser.theta)));
		_processedScans.times.push_back(laser.utime);
		_processedScans.intensities.push_back(laser.intensity);
		_processedScans.x_pos.push_back(poseX);
		_processedScans.y_pos.push_back(poseY);

		// push older pose back on
		_poses.push_front(oldest);

		// pop recently processed scan
		_scansToProcess.pop_front();
	}
}

void eecs467::LaserCorrector::clearPoses() {
	_poses.clear();
}

void eecs467::LaserCorrector::clearScans() {
	_scansToProcess.clear();
}

/**
 * @brief does the same thing as process however it will not
 * clear the scans after it's processed it (so when this is
 * called again it will be processing the same scans);
 * @details [long description]
 */
void processSaveScans();



bool eecs467::LaserCorrector::getCorrectedLcmMsg(maebot_processed_laser_scan_t& msg) {
	if (_processedScans.ranges.empty()) {
		return false;
	}
	_processedScans.utime = _processedScans.times[0];
	_processedScans.num_ranges = _processedScans.ranges.size();
	msg = _processedScans;
	_processedScans.ranges.clear();
	_processedScans.thetas.clear();
	_processedScans.times.clear();
	_processedScans.intensities.clear();
	_processedScans.x_pos.clear();
	_processedScans.y_pos.clear();
	return true;
}
