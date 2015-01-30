#include "LaserCorrector.hpp"
#include "RobotConstants.hpp"
#include "math/angle_functions.hpp"

eecs467::LaserCorrector::LaserCorrector() :
	_utime(0) {
	_scansToProcess.reserve(maxNumLasersPerScan + 100);
	_processedScans.reserve(maxNumLasersPerScan + 100);
}

bool eecs467::LaserCorrector::pushNewScans(const maebot_laser_scan_t& scan) {
	// if scans haven't been processed or processed scans haven't been read out
	if (_scansToProcess.size() != 0 || _processedScans.size() != 0) {
		// printf("scans to process: %d\n", _scansToProcess.size());
		// printf("processed scans: %d\n", _processedScans.size());
		// printf("IGNORED\n");
		// exit(1);
		// return false;
		// if a scan is already in flight push a separator in
		SingleLaser laser = { -1, 0, 0, 0, 0, 0 };
	}
	printf("pushed scan: %ld\t%ld\n", scan.utime, scan.times[scan.num_ranges - 1]);

	_utime = scan.utime;

	// push scanned lasers in backwards (so they can be popped out with pop_back)
	for (int32_t i = scan.num_ranges - 1; i >= 0; --i) {
		SingleLaser laser = {scan.ranges[i], 
			scan.thetas[i],
			scan.times[i],
			scan.intensities[i], 0, 0};
		_scansToProcess.push_back(laser);
	}

	return true;
}

void eecs467::LaserCorrector::pushNewPose(const maebot_pose_t& pose) {
	_poses.push_back(pose);
	printf("pushed pose: %ld\n", pose.utime);
}

bool eecs467::LaserCorrector::process() {
	// if there are no scans to process return false
	if (_scansToProcess.empty()) {
		return false;
	}

	// if the smallest pose time is greater than the smallest scan time
	// then we have no pose before the scans to interpolate with
	// throwout the set of scans
	if (_poses.empty() || _poses.front().utime > _scansToProcess.back().utime) {
		printf("first!\n");
		_scansToProcess.clear();
		_processedScans.clear();
		return false;
	}

	// process until scansToProcess is empty
	printf("ready to process\n");
	while (!_scansToProcess.empty()) {
		// the laser scan with the smallest timestamp still unprocessed
		const SingleLaser& laser = _scansToProcess.back();

		maebot_pose_t oldest;
		// after this loop poses.front() should contain the first 
		// pose that has a greater time stamp or will become empty
		while(!_poses.empty() 
			&& (_poses.front().utime < laser.utime)) {
			oldest = _poses.front();
			_poses.pop_front();
			printf("oldest: %ld\t%d\n", oldest.utime, _poses.size());
		}

		// if all poses are used up, we will have to wait for more
		// pop last one back on and return false
		if (_poses.empty()) {
			printf("empty!\n");
			printf("laser time: %ld\n", laser.utime);
			_poses.push_front(oldest);
			return false;
		}

		// interpolate the position of the vehicle for the scan
		float scaling = (laser.utime - oldest.utime) /
			(_poses.front().utime - oldest.utime);
		float poseX = oldest.x + scaling * (_poses.front().x - oldest.x);
		float poseY = oldest.y + scaling * (_poses.front().y - oldest.y);
		float poseTheta = oldest.theta + scaling * (_poses.front().theta - oldest.theta);

		SingleLaser newLaser = {
			laser.range,
			wrap_to_pi(poseTheta + laserThetaToMaebotTheta(laser.theta)),
			laser.utime,
			laser.intensity,
			poseX,
			poseY };

		// push processed scan onto _processedScans
		_processedScans.push_back(newLaser);

		// pop the scan we just processed
		_scansToProcess.pop_back();

		// push older pose back on
		_poses.push_front(oldest);
	}
	return true;
}

bool eecs467::LaserCorrector::createCorrectedLcmMsg(maebot_processed_laser_scan_t& msg) {
	if (_processedScans.empty() || !_scansToProcess.empty()) {
		return false;
	}

	msg.utime = _utime;
	msg.num_ranges = _processedScans.size();
	
	msg.ranges.clear();
	msg.thetas.clear();
	msg.times.clear();
	msg.intensities.clear();
	msg.x_pos.clear();
	msg.y_pos.clear();

	for (auto& scan : _processedScans) {
		msg.ranges.push_back(scan.range);
		msg.thetas.push_back(scan.theta);
		msg.times.push_back(scan.utime);
		msg.intensities.push_back(scan.intensity);
		msg.x_pos.push_back(scan.posX);
		msg.y_pos.push_back(scan.posY);
	}
	return true;
}
