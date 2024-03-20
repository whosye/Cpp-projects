#include "Sensor.h"


Sensor::Sensor(const int trigger,const int echo) : m_triggerPin(trigger), m_echoPin(echo) {}; 

void Sensor::init(){
    pinMode(m_triggerPin, OUTPUT); 
    pinMode(m_echoPin, INPUT); 
}
void Sensor::measureDistance(){
    digitalWrite(m_triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(m_triggerPin, HIGH);
    delayMicroseconds(10);
    m_duration = pulseIn(m_echoPin, HIGH);
    m_distance = m_duration * 0.034 / 2;
  
}