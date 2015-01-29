#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP


namespace eecs467 {

class StateEstimator {
private:
	float _x; // in meters
	float _y; // in meters
	float _heading; // in radians

public:
	StateEstimator(float x = 0, float y = 0, float heading = 0);

	void update();

};


}

#endif /* STATE_ESTIMATOR_HPP */