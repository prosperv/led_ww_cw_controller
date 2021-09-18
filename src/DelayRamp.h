#pragma once
#include <math.h>

class DelayRamp {
public:
    DelayRamp() : _target(0), _stepSize(0), _output(0){};
    DelayRamp(const int& stepSize) : _target(0), _stepSize(stepSize), _output(0){};

    int setTarget(const int& target) {
        _target = target;
    }

    int setStepSize(const int& stepSize) {
        _stepSize = stepSize;
    };

    int computeValue() {
        int diff = _target - _output;

        if (diff < 0 && diff < -_stepSize) {
            diff = -_stepSize;
        } else if (diff > 0 && diff > _stepSize) {
            diff = _stepSize;
        }

        _output += diff;
        return _output;
    };

private:
    int _target;
    int _stepSize;
    int _output;
};