#include "Dosage.h"

Dosage::Dosage(int id, float pwr, float step) {
    _id = id;
    _pwr = pwr;
    _step = step;
}

void Dosage::setValue(float value) {
    _pwr = value;
}

void Dosage::modifyValue(bool dir) {
    if (dir) {
        if (_pwr + _step < 100) {
            _pwr += _step;
        } else {
            _pwr = 100;
        }
    } else {
        if (_pwr - _step > 0) {
            _pwr -= _step;
        } else {
            _pwr = 0;
        }
    }
}

float Dosage::getValue() {
    return _pwr;
}

int Dosage::getId() {
    return _id;
}
