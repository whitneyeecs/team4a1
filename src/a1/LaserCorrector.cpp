#include "LaserCorrector.hpp"
#include "RobotConstants.hpp"

eeccs467::LaserCorrector::LaserCorrector() {
	_scansToProcess.reserve(maxNumLasersPerScan + 100);
	_processedScans.reserve(maxNumLasersPerScan + 100);
}

bool eeccs467::LaserCorrector::processNewScans(const maebot_laser_scan_t& scan) {
	if (_scansToProcess.size() != 0 || _processedScans.size() != 0) {
		return false;
	}

	// push scanned lasers in backwards (so they can be popped out with pop_back)
	for (int32_t i = num_ranges - 1; i >= 0; --i) {
		_scansToProcess.push_back({scan.ranges[i], 
			scan.thetas[i],
			scan.times[i],
			scan.intensities[i], 0, 0});
	}
}