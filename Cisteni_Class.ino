
// Stepper
#include <AccelStepper.h>
#define dir 5
#define step 2

AccelStepper stepper(1, step,dir);

// Servo
#include <Servo.h>
Servo servo;
byte pinServo = 6;
const int cleaningPos = 45; 
const int noClean = 0;
class Sensor
{
private:
  byte triggerPin;
  byte echoPin;

public:
  long duration; 
  int distance; 

  Sensor(const int trigger,const int echo) : triggerPin(trigger), echoPin(echo) {}; 

  void init(){
    pinMode(triggerPin, OUTPUT); 
    pinMode(echoPin, INPUT); 
  }
  void measureDistance(){
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    
  }
};




class Motor
{
private:
  byte pinA;
  byte pinB;
  byte pinPwm; 
  Sensor sensor1;
  Sensor sensor2;

public:

  Motor(byte pinA, byte pinB, byte pinPwm,Sensor sensor1,Sensor sensor2 ) : pinA(pinA), pinB(pinB), pinPwm(pinPwm), sensor1(sensor1), sensor2(sensor2){};
  static bool Stop; 

  void initiallizePins(){
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(pinPwm, OUTPUT);
  }

  void setMotor(int pwm){
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    analogWrite(pinPwm, pwm);
  }

  void turnOffMotor(){
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    analogWrite(pinPwm, 0);
  }


  bool runMotor(const int& desiredPwm, const int& pwmStep){
    Serial.println("init runMotor fnc");
    while (true){

      sensor1.measureDistance();
      sensor2.measureDistance(); 
      Serial.print(sensor1.distance);
      Serial.print(" ");
      Serial.println(sensor2.distance);

      if (sensor1.distance > 50 && sensor2.distance >50 ){
        Serial.println("First initial pass");
        break;
      }else{
        Serial.println("Object in way");
      }
    }

    Serial.println("Starting cleaning");

    for (int x =0; x<=desiredPwm; x+=pwmStep){
      sensor1.measureDistance();
      sensor2.measureDistance(); 
      Serial.print(sensor1.distance);
      Serial.println(sensor2.distance);
      if (sensor1.distance  < 50  or sensor2.distance <50){
        Serial.println("Object detected while cleaning");
        Serial.print(sensor1.distance);
        Serial.println(sensor2.distance);
        
        return false;
      }else{
        setMotor(x);
        delay(200);
      }
    }

    return true;
  }
};


class Vibration
{
private:
  byte pin;
public:
  int referenceVal = 1000;
  int threshHold;
  
  
  Vibration( byte pin) : pin(pin) {}; 

  void init(){
    pinMode(pin, INPUT);
  }
  void setReferece(){
    long sum = 0;
    for (int j =0; j<30; j++){
      sum+=analogRead(pin);
      Serial.println(sum);
      delay(10);
    }
    referenceVal = int(sum/30);

    Serial.print("Reference Value was set to: ");
    Serial.println(referenceVal);
    threshHold = (referenceVal*0.8);
    Serial.println(threshHold);
  }
  bool vibrationSensing(){
    int val = analogRead(pin);
    threshHold = (referenceVal*0.8);
    if (val < threshHold){
      return true;
    }else{
      return false;
    }
  }

};
// ultrasonic pins 
byte trigPin2 = A2;
byte echoPin2 = A3;
byte trigPin1 = A0;
byte echoPin1 = A1;
// DC 12V motor //

const int pinA = 7;
const int pinB = 8;
const int pwmPinAB = 11;

// DC 21V motor //

const int pinC = 9;
const int pinD = 10;
const int pwmPinCD = 12;

Sensor sensor1(trigPin1, echoPin1);
Sensor sensor2(trigPin2, echoPin2);
Motor motor1(pinA, pinB, pwmPinAB, sensor1, sensor2);
Motor motor2(pinC, pinD, pwmPinCD, sensor1, sensor2);
Vibration vibrationSensor(A6);
static int counter_vib =0;
void setup() {
  Serial.begin(9600);
  servo.attach(pinServo);
  sensor1.init();
  sensor2.init();
  motor1.initiallizePins();
  motor2.initiallizePins();
  vibrationSensor.init();
  stepper.setMaxSpeed(400);
  servo.write(noClean);

}

void clean();
void loop() {



  while (counter_vib < 3){
    if (vibrationSensor.vibrationSensing()){
      counter_vib++;
      Serial.println("vibrace");
    }
    Serial.println("waiting");
    delay(100);
  }
  Serial.println("here goes");
  clean();
  Serial.println("after");
  counter_vib = 0;
  
}


void clean(){
  int desiredPwm = 255;
  int desiredPwm2 = 255;
  int pwmStep = 20;
  int pwmStep2 = 20;
  bool var2;
  servo.write(cleaningPos);
  bool var = motor2.runMotor(desiredPwm, pwmStep2); //starts 21V motor
  //bool var = true;
  if (var){
    var2 = motor1.runMotor(desiredPwm, pwmStep2); // if no object detected turn on also 12V motor
    //var2 = true;
  }else{
    motor2.turnOffMotor();
    motor1.turnOffMotor();
    servo.write(noClean);
  }
  
  if (var2){ // if succesfuly turned 12V motor on, try turn stepper motor 

    sensor1.measureDistance();
    sensor2.measureDistance();
    if (sensor1.distance > 50 && sensor2.distance >50 ){
      Serial.println("START");
      digitalWrite(dir,HIGH); // Enables the motor to move in a particular direction
      // Makes 200 pulses for making one full cycle rotation
      for(int x = 0; x < 600; x++) {
        digitalWrite(step,HIGH); 
        delayMicroseconds(10000);    // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(step,LOW); 
        delayMicroseconds(10000); 
        if (x % 50 == 0){
          sensor1.measureDistance();
          sensor2.measureDistance();
          Serial.print(sensor1.distance);
          Serial.print(" ");
          Serial.println(sensor2.distance);
          if (sensor1.distance<50  || sensor2.distance <50){
            stepper.stop();
            motor2.turnOffMotor();
            motor1.turnOffMotor();
            servo.write(noClean);
            return;
          }
        }
      } 
      stepper.stop();
      motor2.turnOffMotor();
      motor1.turnOffMotor();
      servo.write(noClean);
      return;
    }else{
      Serial.println("init detekce");
      stepper.stop();
      motor2.turnOffMotor();
      motor1.turnOffMotor();
      servo.write(noClean);
      return;
    }
  }
  else{
    stepper.stop();
    motor2.turnOffMotor();
    motor1.turnOffMotor();
    servo.write(noClean);
    return;
  }
}







