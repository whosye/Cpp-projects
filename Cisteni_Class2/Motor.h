#ifndef Motor_h
#define  Motor_h
#include "Sensor.h"


class Motor
{
private:
  byte m_pinA;
  byte m_pinB;
  byte m_pinPwm; 
  Sensor sensor1;
  Sensor sensor2;
  const int m_distance;

public:

  Motor(byte pinA, byte pinB, byte pinPwm,Sensor sensor1,Sensor sensor2, const int distance );
  void initiallizePins();
  void setMotor(int pwm);
  void turnOffMotor();
  bool runMotor(const int& desiredPwm, const int& pwmStep);
  bool runMotorMainCylinder(const int& desiredPwm, float timeToSpin);
};

#endif

