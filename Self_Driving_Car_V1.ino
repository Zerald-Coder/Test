/* Link for colour sensor interfacing: https://randomnerdtutorials.com/arduino-color-sensor-tcs230-tcs3200/
 * Link for servo motor interfacing: https://microcontrollerslab.com/mg995-servo-motor-pinout-interfacing-with-arduino-features-examples/
 * Link for acelerometer and gyroscope interfacing: https://electrosome.com/interfacing-mpu-6050-gy-521-arduino-uno/
 * By Team Hexa Infinity
*/
#include <Servo.h>
#include <Wire.h>
#include <math.h>

#define TrigPin1 13
#define EchoPin1 12
#define TrigPin2 11
#define EchoPin2 10

#define FrequencyScalerPin1 9
#define FrequencyScalerPin2 8
#define ColourFilterPin1 7
#define ColourFilterPin2 6
#define ColourGetterPin 5

#define Servo_PWM 4
Servo MG995_Servo;

#define SpeedPin A1

long pulseDuration1;
long pulseDuration2;

int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

int redColor = 0;
int greenColor = 0;
int blueColor = 0;

const int MPU=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double yawCorrection = 0;

int _quarterTurn = 0;
bool _clockwise;

void setup() {
  Serial.begin(9600);
  pinMode(TrigPin1, OUTPUT);
  pinMode(EchoPin1, INPUT);
  pinMode(TrigPin2, OUTPUT);
  pinMode(EchoPin2, INPUT);
  pinMode(FrequencyScalerPin1, OUTPUT);
  pinMode(FrequencyScalerPin2, OUTPUT);
  pinMode(ColourFilterPin1, OUTPUT);
  pinMode(ColourFilterPin2, OUTPUT);
  pinMode(ColourGetterPin, INPUT);
  pinMode(Servo_PWM, OUTPUT);
  pinMode(SpeedPin, OUTPUT);
  digitalWrite(ColourFilterPin1,HIGH);
  digitalWrite(ColourFilterPin2,LOW);
  Wire.begin(); //initiate wire library and I2C
  Wire.beginTransmission(MPU); //begin transmission to I2C slave device
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)  
  Wire.endTransmission(true); //ends transmission to I2C slave device
}

void loop() {
  // Assume vehicle perfectly aligned
  yawCorrection = 0 - GYRO_GET_YAW();
  MG995_Servo.write(90);
  analogWrite(SpeedPin,1023);
  while(ORANGE_BLUE_DETECTOR() == 2);
  _clockwise = ORANGE_BLUE_DETECTOR();
  while(_clockwise){
    _quarterTurn++;
    STEER(1,100);
    while(GYRO_GET_YAW() + yawCorrection < (_quarterTurn * 90 )%360);
    STEER(-1,100);
    if(_quarterTurn == 12) break;
    while(ORANGE_BLUE_DETECTOR() == 2){
      if(RIGHT_ULTRASONIC() > 7){
        STEER(1,100);
        STEER(-1,200);
        STEER(1,100);
      }else if(LEFT_ULTRASONIC() > 13)
        STEER(-1,100);
        STEER(1,200);
        STEER(-1,100);
    }
  }
  while(!_clockwise){
    _quarterTurn++;
    STEER(-1,100);
    while(GYRO_GET_YAW() + yawCorrection < (_quarterTurn * -90 )%360);
    STEER(1,100);
    if(_quarterTurn == 12) break;
    while(ORANGE_BLUE_DETECTOR() == 2){
      if(LEFT_ULTRASONIC() > 7){
        STEER(-1,100);
        STEER(1,200);
        STEER(-1,100);
      }else if(RIGHT_ULTRASONIC() > 13)
        STEER(1,100);
        STEER(-1,200);
        STEER(1,100);
    }
  }
  delay(1000);
  analogWrite(SpeedPin,0);
  while(1);
}

double LEFT_ULTRASONIC(){
  digitalWrite(TrigPin1,HIGH);
  delayMicroseconds(1);
  digitalWrite(TrigPin1,LOW);
  pulseDuration1 = pulseIn(EchoPin1, HIGH);
  delayMicroseconds(50);
  return (pulseDuration1 / 58.2);
}

double RIGHT_ULTRASONIC(){
  digitalWrite(TrigPin2,HIGH);
  delayMicroseconds(1);
  digitalWrite(TrigPin2,LOW);
  pulseDuration2 = pulseIn(EchoPin2, HIGH);
  delayMicroseconds(50);
  return (pulseDuration2 / 58.2);
}

int ORANGE_BLUE_DETECTOR(){
  digitalWrite(ColourFilterPin1,LOW);
  digitalWrite(ColourFilterPin2,LOW);
  redFrequency = pulseIn(ColourGetterPin, LOW);
  redColor = map(redFrequency, 70, 120, 255,0);
  digitalWrite(ColourFilterPin1,HIGH);
  digitalWrite(ColourFilterPin2,HIGH);
  greenFrequency = pulseIn(ColourGetterPin, LOW);
  greenColor = map(redFrequency, 70, 120, 255,0);
  digitalWrite(ColourFilterPin1,LOW);
  digitalWrite(ColourFilterPin2,HIGH);
  blueFrequency = pulseIn(ColourGetterPin, LOW);
  blueColor = map(redFrequency, 70, 120, 255,0);
  if(redColor > 230 && greenColor > 140 && greenColor < 180 && blueColor < 20){
    return 1; //Orange detected
  }else if(redColor < 20 && greenColor < 20 && blueColor > 230){
    return 0; //Blue detected
  }else{
    return 2; //None detected
  }
}

double GYRO_GET_YAW(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);

  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  
  //read temperature data 
  Tmp=Wire.read()<<8|Wire.read();
  
  //read gyroscope data
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();

  /*Calculation to get the yaw
   * ...
   */
   return 0;
}

void STEER(int direction,int time){
  MG995_Servo.write(90 + direction * 90);
  delay(time);
  MG995_Servo.write(90);
}
