#include "Motor.h"

  Motor::Motor(byte pinA, byte pinB, byte pinPwm,Sensor sensor1,Sensor sensor2, const int distance ) : m_pinA(pinA), m_pinB(pinB), m_pinPwm(pinPwm), sensor1(sensor1), sensor2(sensor2), m_distance(distance){};


  void  Motor::initiallizePins(){
    pinMode(m_pinA, OUTPUT);
    pinMode(m_pinB, OUTPUT);
    pinMode(m_pinPwm, OUTPUT);
  }

  void  Motor::setMotor(int pwm){
    digitalWrite(m_pinA, HIGH);
    digitalWrite(m_pinB, LOW);
    analogWrite(m_pinPwm, pwm);
    Serial.print("Setting Motor: ");
    Serial.println(pwm);
  }

  void  Motor::turnOffMotor(){
    digitalWrite(m_pinA, LOW);
    digitalWrite(m_pinB, LOW);
    analogWrite(m_pinPwm, 0);
  }


bool Motor::runMotor(const int& desiredPwm, const int& pwmStep){
  Serial.println("init runMotor fnc");
  while (true){

    sensor1.measureDistance();
    sensor2.measureDistance(); 
    Serial.print(sensor1.m_distance);
    Serial.print(" ");
    Serial.println(sensor2.m_distance);

    if (sensor1.m_distance > m_distance && sensor2.m_distance >m_distance ){
      Serial.println("First initial pass");
      break;
    }else{
      return false;
    }
  }

  Serial.println("Starting cleaning");

  for (int x =0; x<=desiredPwm; x+=pwmStep){
    sensor1.measureDistance();
    sensor2.measureDistance(); 
    Serial.print(sensor1.m_distance);
    Serial.print(", ");
    Serial.println(sensor2.m_distance);
    if (sensor1.m_distance  < m_distance  or sensor2.m_distance < m_distance){
      Serial.println("Object detected while cleaning");
      Serial.print(sensor1.m_distance);
      Serial.println(sensor2.m_distance);
      
      return false;
    }else{
      if (x >= 255){
        x = 255;
      }
      Serial.print("Setting motor pwm: ");
      Serial.println(x);
      setMotor(x);
      delay(10);
    }
  }
  return true;
}

bool Motor::runMotorMainCylinder(const int& desiredPwm, float timeToSpin){
  unsigned long initTime = millis(); 
  while (millis() - initTime < timeToSpin){
    sensor1.measureDistance();
    sensor2.measureDistance(); 
    if (sensor1.m_distance  < m_distance  or sensor2.m_distance < m_distance){
      Serial.println("Object detected while cleaning will all three motors");
      Serial.print(sensor1.m_distance);
      Serial.print(", ");
      Serial.println(sensor2.m_distance);
      if (millis() - initTime < timeToSpin*(3/4)){ // if 75% of cleaning done, then consider job done. 
        return true;
        Serial.println("Consider job done"); 
      }else{
        return false; // -> RestartCleaning = false, that means VibrationCounter will not reset. 
      }
    }else{
      setMotor(desiredPwm);
    }
  }
  return true;
  Serial.println("Job done");
}
