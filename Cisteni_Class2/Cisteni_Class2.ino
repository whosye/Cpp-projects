

// ----------------------------------------------------------------------------------------------------------------//
//Variables to change 
const int Distance_to_Stop = 20; // Distance for object in  "Cm" 
const int Number_of_Counts_to_Clean = 3;  // Number of counts from vibrating sensors to start cleaning cycle 
float cleanTime = 5000;  // for this time the main cylinder will spin, 5000 -> 5 seconds 
const int desiredPwm1 = 40; // Speed of main cylinder, type in range (0- 255)
const int desiredPwm2 = 255; //Speed of two cleaning brushesh , type in range (0- 255)

// ----------------------------------------------------------------------------------------------------------------//


#include "Motor.h"
#include "Sensor.h"

// ultrasonic pins 
byte trigPin2 = A2;
byte echoPin2 = A3;
byte trigPin1 = A0;
byte echoPin1 = A1;
// DC 12V motor //
byte pinA = 7;
byte  pinB =6;
byte  pwmPinAB = 3;
// DC 21V motor // --> This will turn out as the parameters  for main cylinder pins 
byte pinC = 9;
byte pinD = 8;
byte pwmPinCD = 5;

// others   //
byte  pinE=4; // -> Rele module trigger 
byte pinVib = 10;

#include <Servo.h>
Servo servo;
byte pinServo = 11;
Sensor sensor1(trigPin1, echoPin1);
Sensor sensor2(trigPin2, echoPin2);
Motor motor1(pinA, pinB, pwmPinAB, sensor1, sensor2, Distance_to_Stop);
Motor motor2(pinC, pinD, pwmPinCD, sensor1, sensor2, Distance_to_Stop);
void countVib(int* P);
void countVibMax(int* P);
unsigned long previousTime;
unsigned int checkTime = 1000; // every second

void setup() {
  Serial.begin(9600);

  
  sensor1.init();
  sensor2.init();
  motor1.initiallizePins();
  motor2.initiallizePins();
  pinMode(10,INPUT);  // -> Vibration sensor pin 
  pinMode(pinE, OUTPUT); // -> initializing pin for Rele module 
  
  //pinMode(3,INPUT_PULLUP);
  servo.attach(pinServo);
  servo.write(0);
  previousTime= millis();

  
}

void loop() {


  // Set Static variables for countiong vibrations ans storing bool variables if restart cleaning or if vibration true; 
  static int vibrationCounter = 0;
  static bool restartCleaning = false;  
  static bool vibration = false; 



  if (digitalRead(pinVib) == LOW){
    vibration = true; 
  }


  if (millis() - previousTime >= checkTime){ // with this timer only one vibration per checkTime
    previousTime = millis();
    Serial.println("Time to Check:"); 
    if (vibration == true){
      countVib(&vibrationCounter);
      vibration = false;
    }
  }

  if (vibrationCounter >= Number_of_Counts_to_Clean){
    //Start cleaning 
    Serial.println("Start cleaning loop");




    // Starting the LUX ///////////////////////////////////////////////////////////////////////////////////////////////
    while (true){
      sensor1.measureDistance();
      sensor2.measureDistance();
      if (sensor1.m_distance > Distance_to_Stop && sensor2.m_distance >Distance_to_Stop){
        Serial.println("Starting LUX, right now");
        digitalWrite(pinE, HIGH); 
        break;
      }
      Serial.println("Starting LUX, but something is in the way");
      delay(2000); // If there is something in the way, take 2 second break 
    }

    // Starting cleaning brushes  ///////////////////////////////////////////////////////////////////////////////////////////////
    Serial.println("Approaching to start 2 cleaning brushes");
    int pwmStep = 25; 
    bool startCleaningBrushes = motor2.runMotor(desiredPwm2,pwmStep);

    if (startCleaningBrushes == false){
      digitalWrite(pinE, LOW); // Turn down the LUX
      motor1.turnOffMotor(); // Turn  off brushes 
    }else{
      Serial.println("Cleaning brushes and lux is running, startingt to run main cylinder");

      // Main cylinder spinning ///////////////////////////////////////////////////////////////////////////////////////////////
      servo.write(45);
      bool finishedCleaning = motor1.runMotorMainCylinder(desiredPwm1, cleanTime);

      if (finishedCleaning){
        Serial.print("vibrationCounter: ");
        Serial.println(vibrationCounter);
        vibrationCounter = 0; 
        Serial.print("Reset->vibrationCounter: ");
        Serial.println(vibrationCounter);
        digitalWrite(pinE, LOW); // Turn down the LUX
        motor1.turnOffMotor(); // Turn  off brushes 
        motor2.turnOffMotor(); // Turn  off main cylinder
        Serial.println("Cleaning OVer");
        delay(5000); // 5 sec off
        servo.write(0);
      }else{
        digitalWrite(pinE, LOW); // Turn down the LUX
        motor1.turnOffMotor(); // Turn  off brushes 
        motor2.turnOffMotor(); // Turn  off main cylinder
        delay(5000); // 5 sec off
        servo.write(0);
        // the counter of vibration remains same, so the cleaning process starts again
      }
    }
  } 
}



void countVib(int* P){
  *P += 1; 
  Serial.print("Vib detected, Current number of Count: ");
  Serial.println(*P);
}

