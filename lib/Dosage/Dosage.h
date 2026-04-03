#ifndef DOSAGE_H
#define DOSAGE_H

class Dosage {
private:
    float _pwr;
    float _step;
    int _id;

public:
    Dosage(int id, float pwr, float step);

    void setValue(float value);
    void modifyValue(bool dir);
    float getValue();
    int getId();
};

#endif
