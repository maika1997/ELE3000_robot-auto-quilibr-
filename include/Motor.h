#include <Encoder.h>


class Motor{
  public:

  // functions
  Motor(int motorpin1, int motorpin2);
  // uint8_t encoderpin1, uint8_t encoderpin2
  ~Motor();
  
  // void computeSpeed();
  void move(int pwm);
  double getSpeed();
  // double getPosition();

  // Encoder encoder;

  private:
  
  // Attributes
  double position;
  double previousPosition;
  uint32_t curentTime;
  uint32_t previousTime;
  uint32_t deltaT;
  double speed;
  int motorPin1;
  int motorPin2;
  uint8_t encoderPin1;
  uint8_t encoderPin2;
  
};
