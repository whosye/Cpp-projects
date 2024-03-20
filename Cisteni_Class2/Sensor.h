#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
class Sensor
{
private:
  byte m_triggerPin;
  byte m_echoPin;
public:
  Sensor(const int trigger,const int echo);
  long m_duration; 
  int m_distance; 
  void init();
  void measureDistance();
};


#endif