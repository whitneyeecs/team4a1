#include "LaserCorrector.hpp"
#include "RobotConstants.hpp"

eecs467::LaserCorrector::LaserCorrector() :
	_utime(0) {
	_scansToProcess.reserve(maxNumLasersPerScan + 100);
	_processedScans.reserve(maxNumLasersPerScan + 100);
}

bool eecs467::LaserCorrector::pushNewScans(const maebot_laser_scan_t& scan) {
	if (_scansToProcess.size() != 0 || _processedScans.size() != 0) {
		printf("IGNORED\n");
		exit(1);
		return false;
	}

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
		while(!_poses.empty() && _poses.front().utime < _scansToProcess.back().utime){
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
