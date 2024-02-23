/*
Feedback control using PID regulator with Kalman filter. 
Sensor MPU6050 with Arduino Nano and two DC motors for balancing robot. 
Regulating motor PWM frequency to minimaze the error function in term "error = setPoint - previous_angle"
*/

#include<Wire.h>
float RateRoll, RatePitch, RateYaw;
float RollPrevious, PitchPrevious, YawPrevious;
float Roll, Pitch, Yaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber ;
unsigned long LoopTimer;
float sampleTime = 0.005;
float error_sum =0;

float AccX, AccY, AccZ; 
float CalibAccX, CalibAccY, CalibAccZ;
float AngleRoll, AnglePitch;


float KalmanAngleRoll = 0; 
float KalmanUncertaintyAngleRoll = 2*2;
float Kalman1DOutput[] ={0, 0}; 
float previous_KalmanAngleRoll;
// PID setup 
int P = 0; 
int I = 0;
int D = 0;
 

void gyro_signals(void){

  // Low Pass
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  // Scale 
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  // Data from ACC
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();



  AccX = (float)AccXLSB/4096; 
  AccY = (float)AccYLSB/4096; 
  AccZ = (float)AccZLSB/4096; 

  AngleRoll  = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))* 1/(3.14/180); 
  AnglePitch = atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))* 1/(3.14/180); 

    // Scale 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  // Data from gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX/65.5; 
  RatePitch = (float)GyroY/65.5; 
  RateYaw = (float)GyroZ/65.5; 

}


void kalman(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement){

  KalmanState = KalmanState + 0.004*KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004*4*4;
  float KalmanGain = KalmanUncertainty * 1/(1*KalmanUncertainty + 3*3);

  KalmanState = KalmanState + KalmanGain*(KalmanMeasurement - KalmanState );
  KalmanUncertainty = (1-KalmanGain)*KalmanUncertainty;
  
  Kalman1DOutput[0]= KalmanState;
  Kalman1DOutput[1]= KalmanUncertainty;
}


void StartMotor(bool state);
void RightWheelSpeed(int PWM);
void LeftWheelSpeed(int PWM);
float PID(float KalmanAngleRoll, float setPoint,int I, int P, int D);

void setup() {

  for(int i=5; i<10;i++){
    pinMode(i, OUTPUT);
  }
 
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for(RateCalibrationNumber = 0;
      RateCalibrationNumber < 500;
      RateCalibrationNumber ++)
        {
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        delay(1);
        }
        
     RateCalibrationRoll /= 500;
     RateCalibrationPitch /= 500;
     RateCalibrationYaw /= 500;

     Serial.println("Calibration Done");


    for ( int i = 0; i <500; i++){
    gyro_signals();
    CalibAccX += AccX;
    CalibAccY += AccY;
    CalibAccZ += AccZ; 
    delay(1);
    }
    CalibAccX /= 500;
    CalibAccY /= 500;
    CalibAccZ /= 500;
  
Serial.println("Calibration Done");

gyro_signals();
LoopTimer=micros();

}

void loop() {

  // HANDLE ANGLE 
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  kalman(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll  = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];


  int(PWM) = PID(KalmanAngleRoll, 0.2, 60, 15,0);
  Serial.println(KalmanAngleRoll);


  if (KalmanAngleRoll > 0){
    StartMotor(false);
    RightWheelSpeed(PWM*-1);
    LeftWheelSpeed(PWM*-1);
  }
  else{
    StartMotor(true);
    RightWheelSpeed(PWM);
    LeftWheelSpeed(PWM);
  }
 
  while (micros() - LoopTimer <sampleTime);
  LoopTimer=micros();
  
}


void StartMotor(bool state){

  if (state){
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
  }
  else{
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
  }
}

void LeftWheelSpeed(int PWM){
  analogWrite(6, PWM);
}
void RightWheelSpeed(int PWM){
  analogWrite(5, PWM);
}

float PID(float KalmanAngleRoll, float setPoint,int I, int P, int D){
  float error = setPoint - KalmanAngleRoll;
  error_sum +=  error; 
  error_sum = constrain(error_sum, -300, 300);
  int PWM = P*error + ( I * error_sum *  sampleTime); 
  PWM = constrain(PWM, -255, 255);

  
  return PWM;
}





