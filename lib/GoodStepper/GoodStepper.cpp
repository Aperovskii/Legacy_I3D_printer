#include "Arduino.h"
#include "GoodStepper.h"

// Class initiation
VStepper::VStepper(int dirPin, int pulsePin) {
  _dirPin = dirPin;
  _pulsePin = pulsePin;
  pinMode(_dirPin, OUTPUT);
  pinMode(_pulsePin, OUTPUT);
  _position = -999;
  _defaultDirection = true;
  // Not calibrated value
  _direction = true; // Direction set to fwd
  digitalWrite(_dirPin, HIGH);
  _dirState=true;
}

// Do a step
void VStepper::step() {
  digitalWrite(_pulsePin, HIGH);
  digitalWrite(_pulsePin, LOW);
  if (_defaultDirection){
    if (_direction){
      _position-=1;
    }else{
      _position+=1;
    }
    
  }else{
    if (_direction){
      _position+=1;
    }else{
      _position-=1;
    }
  }
    // register a step
  //_last=micros(); // make a timestamp in microseconds
}


void VStepper::invertDir() { 
  if (_defaultDirection){
    _defaultDirection=false;
    if(_direction){
      digitalWrite(_dirPin, LOW);
      _dirState=false;
    }else{
      digitalWrite(_dirPin, HIGH);
      _dirState=true;
    }
    
  }else{
    _defaultDirection=true;
    if(_direction){
      digitalWrite(_dirPin, HIGH);
      _dirState=true;
    }else{
      digitalWrite(_dirPin, LOW);
      _dirState=false;
    }
  }
}
// Direction forward
// 
void VStepper::setPosition(long int newPosition) {
  _position = newPosition;
}

void VStepper::direction() {
  Serial.print("Default direction is ");
  Serial.println(_defaultDirection);
  Serial.print("Direction is ");
  Serial.println(_direction); 
  Serial.print("Pin state is ");
  Serial.println(_dirState); 
}

void VStepper::setDirection(bool newDirection) {
  _direction = newDirection;
  if (_defaultDirection){
    if (_direction){
      digitalWrite(_dirPin, HIGH);
      _dirState=true;
    }else{
      digitalWrite(_dirPin, LOW);
      _dirState=false;
      }
  }else{
    if(_direction){
      digitalWrite(_dirPin, LOW);
      _dirState=false;
    }else{
      digitalWrite(_dirPin, HIGH);
      _dirState=true;
    }
  }
  //Serial.print("Direction of stepper is set to ");
  //Serial.println(_direction);
  //Serial.print("Default is ");
  //Serial.println(_defaultDirection);
}

// Метод для получения текущего состояния светодиода
long int VStepper::position(){
  return _position;
}

long int VStepper::lastActivation(){
  return _last;
}