#ifndef GoodStepper_h
#define GoodStepper_h

#include "Arduino.h"

class VStepper {
  public:
    VStepper(int dirPin, int pulsePin); // Конструктор
    void step();           // make a step
    void invertDir();
    void setPosition(long int newPosition);
    void setDirection(bool newDirection);
    void direction();
    long int position();      // get actual position
    long int lastActivation(); // time from last step

  private:
    int _dirPin; // direction pin
    int _pulsePin; // pulse pin
    long int _position; // current position, default -999
    bool _defaultDirection; // set default direction. It should correspond to positive ! 
    bool _direction; // actual direction in relation to default. True - default, false - inverse
    unsigned long _last;
    bool _dirState; // time of last activation
};

#endif