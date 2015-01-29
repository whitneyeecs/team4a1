#include "LaserCorrector.hpp"
#include "RobotConstants.hpp"

eecs467::LaserCorrector::LaserCorrector() {
	_scansToProcess.reserve(maxNumLasersPerScan + 100);
	_processedScans.reserve(maxNumLasersPerScan + 100);
}

bool eecs467::LaserCorrector::pushNewScans(const maebot_laser_scan_t& scan) {
	if (_scansToProcess.size() != 0 || _processedScans.size() != 0) {
		printf("IGNORED\n");
		exit(1);
		return false;
	}

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

void eecs467::LaserCorrector::pushNewPose(maebot_pose_t pose) {
	_poses.push_back(pose);
}

bool eecs467::LaserCorrector::process() {
	//const SingleLaser& laser = _scansToProcess.back();
	if(_poses.size()<2){
		return false;
	}

	while (_poses.size() && _poses.front().utime > _scansToProcess.back().utime) {
		_scansToProcess.pop_back();
	}

	while (!_scansToProcess.empty()) {
		maebot_pose_t oldest;
		while(_poses.size() && _poses.front().utime < _scansToProcess.back().utime){
			oldest = _poses.front();
			_poses.pop_front();
		}

		if (_poses.empty()) {
			return false;
		}

		const SingleLaser& laser = _scansToProcess.back();
		// interpolate
		float scaling = (laser.utime - oldest.utime) /
			(_poses.front().utime - oldest.utime);
		float poseX = oldest.x + scaling * (_poses.front().x - oldest.x);
		float poseY = oldest.y + scaling * (_poses.front().y - oldest.y);
		float poseTheta = oldest.theta + scaling * (_poses.front().theta - oldest.theta);

		SingleLaser newLaser = {
			laser.range,
			poseTheta + laserThetaToMaebotTheta(laser.theta),
			laser.utime,
			laser.intensity,
			poseX,
			poseY };

		// push processed scan onto _processedScans
		_processedScans.push_back(newLaser);

		_poses.push_front(oldest);
	}
	return true;
}

bool eecs467::LaserCorrector::createCorrectedLcmMsg(maebot_processed_laser_scan_t& msg) {
	// msg.
}
