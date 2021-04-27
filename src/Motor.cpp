#include <Motor.h>
#include <Encoder.h>

#define COEF 1.0/(34.0*4.0)
#define DELTA_MIN 5

  Motor::Motor(int motorpin1, int motorpin2)
  { 
  //, uint8_t encoderpin1, uint8_t encoderpin2):encoder(encoderpin1, encoderpin2)
  motorPin1 = motorpin1;
  motorPin2 = motorpin2;
  }

  Motor::~Motor()
  {

  }

  // void Motor::computeSpeed(){
  //   curentTime = millis();
  //   position = encoder.read()*COEF;
  //   // uncoment if speed values are off
  //   if ((curentTime-previousTime) > DELTA_MIN){
	//    speed = (position-previousPosition)/(curentTime-previousTime);
  //   }
  //   //speed = (position-previousPosition)/deltaT;
  //   previousTime = curentTime;
  //   previousPosition = position;
  // }

  void Motor::move(int pwm){
   if (pwm <= 0){
    analogWrite(motorPin1, abs(pwm));
    analogWrite(motorPin2, 0);
   }
   else {
    analogWrite(motorPin2, abs(pwm));
    analogWrite(motorPin1,0);
   }
  }

  // double Motor::getSpeed(){
  //   return speed;
  // }

  // double Motor::getPosition(){
  //   position = double(encoder.read());
  //   return position;
  // }

 