#pragma once

#include <Arduino.h>

class Hysteresis {
public:
    Hysteresis(int threshold) : _threshold(threshold){};

    int process(int input) {
        int diff = abs(input - _value);
        if (_isSteady) {
            if (diff > _threshold) {
                _value = input;
                _isSteady = false;
            }
        }
        else
        {
            if (diff < _threshold) {
                _isSteady = true;
            }
            _value = input;
        }
    }

private:
    int _threshold;
    int _value;
    bool _isSteady = false;
};
