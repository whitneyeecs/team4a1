#include "LaserCorrector.hpp"
#include "RobotConstants.hpp"
#include "math/angle_functions.hpp"

eecs467::LaserCorrector::LaserCorrector() {
	_currMsg = _msgQueue.end();
}

bool eecs467::LaserCorrector::pushNewScans(const maebot_laser_scan_t& scan) {
	// if scans haven't been processed or processed scans haven't been read out
	if (_scansToProcess.size() != 0) {
		printf("separator\n");
		// push a separator value
		SingleLaser laser = { -1, 0, 0, 0, 0, 0 };
		_scansToProcess.push_back(laser);
		// if we are not currently processing a message,
		// set current message to the one we just created
	}

	printf("pushed scan: %ld\t%ld\n", scan.utime, scan.times[scan.num_ranges - 1]);
	maebot_processed_laser_scan_t newScan;
	newScan.utime = scan.utime;
	newScan.num_ranges = scan.num_ranges;
	_msgQueue.push_back(newScan);

	if (_currMsg == _msgQueue.end()) {
		_currMsg = _msgQueue.begin();
	}

	// push scanned lasers in backwards (so they can be popped out with pop_back)
	for (int32_t i = 0; i < scan.num_ranges; ++i) {
		SingleLaser laser = { scan.ranges[i], 
			scan.thetas[i],
			scan.times[i],
			scan.intensities[i], 0, 0 };
		_scansToProcess.push_back(laser);
	}
	return true;
}

void eecs467::LaserCorrector::pushNewPose(const maebot_pose_t& pose) {
	_poses.push_back(pose);
	printf("pushed pose: %ld\n", pose.utime);
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
		printf("first!\n");
		_currMsg = _msgQueue.end();
		_msgQueue.clear();
		_scansToProcess.clear();
		return;
	}

	// process until scansToProcess is empty
	while (!_scansToProcess.empty()) {
		// the laser scan with the smallest timestamp still unprocessed
		SingleLaser& laser = _scansToProcess.front();

		// printf("%f\t%f\n", laser.range, laser.theta);

		if (laser.range == -1) {
			// read a separator
			_currMsg++;

			// pop separator
			_scansToProcess.pop_front();
			continue;
		}

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

		// interpolate the position of the vehicle for the scan
		float scaling = (laser.utime - oldest.utime) /
			(_poses.front().utime - oldest.utime);
		float poseX = oldest.x + scaling * (_poses.front().x - oldest.x);
		float poseY = oldest.y + scaling * (_poses.front().y - oldest.y);
		float poseTheta = oldest.theta + scaling * (_poses.front().theta - oldest.theta);

		printf("%f\n", wrap_to_2pi(poseTheta +
			laserThetaToMaebotTheta(laser.theta)));

		// push processed scan into current message
		_currMsg->ranges.push_back(laser.range);
		_currMsg->thetas.push_back(wrap_to_2pi(poseTheta +
			laserThetaToMaebotTheta(laser.theta)));
		_currMsg->times.push_back(laser.utime);
		_currMsg->intensities.push_back(laser.intensity);
		_currMsg->x_pos.push_back(poseX);
		_currMsg->y_pos.push_back(poseY);

		// push older pose back on
		_poses.push_front(oldest);

		// pop recently processed scan
		_scansToProcess.pop_front();
	}
	// processed one entire scan
	_currMsg++;
}

bool eecs467::LaserCorrector::getCorrectedLcmMsg(maebot_processed_laser_scan_t& msg) {
	if (_msgQueue.empty()) {
		return false;
	}
	msg = _msgQueue.front();
	if (msg.num_ranges != msg.ranges.size()) {
		return false;
	}
	_msgQueue.pop_front();
	return true;
}
