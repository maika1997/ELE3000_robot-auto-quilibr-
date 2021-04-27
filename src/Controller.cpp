#include <Controller.h>
#include <Arduino.h>

#define COMPUTE_TIME 100
#define MAX_SPEED 255

#define KP 8 // essaie : 7
#define KI 0.0
#define KD 0.6 // essaie : 0.6


Controller::Controller()
{ 
    _kp = KP;
    _ki = KI;
    _kd = KD;

    _setpoint = 0;
    _error = 0;
    _previous_angle = 0;

    _integral = 0;
    _angularVel = 0;
    _lastTime = 0;

    _output = 0;
}

Controller::~Controller()
{

}

void Controller::initialize(){
    _setpoint = 0;
    _error = 0;
    _previous_angle = 0;

    _integral = 0;
    _angularVel = 0;
    _lastTime = millis();

    _output = 0;
}

int Controller::computeOutput(float currentAngle, float angularVel){
    float currentTime = millis();
    float deltaTime = (currentTime-_lastTime)/1000.0; // [sec]
    
    _error = currentAngle - _setpoint;

    _integral += _ki*_error*deltaTime;
    _angularVel = angularVel;

    _pAction = _kp * _error;
    _dAction = _kd*_angularVel;

    _output = _pAction + _integral + _dAction;
    
    if(_output <= -MAX_SPEED ){
        _output = -MAX_SPEED;
    } else if (_output >= MAX_SPEED){
        _output = MAX_SPEED;
    } 

    _lastTime = currentTime;
    _previous_angle = currentAngle;

    return _output;
}

float Controller::getKp(){
    return _kp;
}

float Controller::getKi(){
    return _ki;
}

float Controller::getKd(){
    return _kd;
}

float Controller::getPAction(){
    return _pAction;
}

float Controller::getIAction(){
    return _integral;
}

float Controller::getDAction(){
    return _dAction;
}
        
void Controller::setKp(float newKp){
    _kp = newKp;
}

void Controller::setKi(float newKi){
    _ki = newKi;
}

void Controller::setKd(float newKd){
    _kd = newKd;
}

void Controller::setGains(float newKp, float newKi, float newKd){
    _kp = newKp;
    _ki = newKi;
    _kd = newKd; 
}