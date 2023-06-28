#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float rollAccel = 0;           // X Axis
float pitchAccel = 0;         // Y Axis

///////////////////////////////////////////////////////////////////

#include "PWM.hpp"



PWM elevatorRx(2); // Setup pin 2 for PWM
PWM aileronRx(3); // Setup pin 2 for PWM


int elevatorRxVal = 0;
int aileronRxVal = 0;
///////////////////////////////////////////////////////////////////

#include<Servo.h>

Servo elevatorServo;
Servo aileronServo;

/////////////////////////////////////////////////////////////////

#include <PID_v1.h>
double ElevatorSetpoint, ElevatorInput, ElevatorOutput;

double ElevatorKp=2, ElevatorKi=0, ElevatorKd=5;
PID ElevatorPID(&ElevatorInput, &ElevatorOutput, &ElevatorSetpoint, ElevatorKp, ElevatorKi, ElevatorKd, DIRECT);



double AileronSetpoint, AileronInput, AileronOutput;

double AileronKp=2, AileronKi=5, AileronKd=1;
PID AileronPID(&AileronInput, &AileronOutput, &AileronSetpoint, AileronKp, AileronKi, AileronKd, DIRECT);

void setup(){
  Serial.begin(115200);
    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  elevatorRx.begin(true);
  aileronRx.begin(true);


  elevatorServo.attach(4);
  aileronServo.attach(6);

  delay(200);

  elevatorServo.write(90);
  aileronServo.write(90);

  ///////////////////////////////////
  //ElevatorInput = ;
  AileronSetpoint = 90;
  ElevatorSetpoint = 90;;

  //turn the PID on
  ElevatorPID.SetMode(AUTOMATIC);
  AileronPID.SetMode(AUTOMATIC);
  delay(100);

}










void loop() {
    
    //ControlServos();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
   rollAccel =  a.acceleration.x;
   pitchAccel =  a.acceleration.y;
//   Serial.print(rollAccel);
//   Serial.print("  ");
//   Serial.print(pitchAccel);
//   Serial.print("  ");
   rollAccel = map(rollAccel,-10,10,0,180);
   pitchAccel = map(pitchAccel,-10,10,0,180);
  Serial.print(rollAccel);
   Serial.print("  ");
   Serial.print(pitchAccel);
   Serial.print("  ");
delay(100);
  TxInput();
  ElevatorPIDControl();
 // AileronPIDControl();
  Serial.println("   ");
}



void TxInput(){
  elevatorRxVal = elevatorRx.getValue();
  aileronRxVal = aileronRx.getValue();
//  Serial.print(elevatorRxVal);
//  Serial.print("  ");
//  Serial.print(aileronRxVal);
//  Serial.print("  ");
  AileronSetpoint = map(aileronRxVal,1000,2000,-50,250);
  ElevatorSetpoint = map(elevatorRxVal,1000,2000,-50,250);
//    Serial.print(ElevatorSetpoint);
//     Serial.print("  ");
//    Serial.print(AileronSetpoint);
//    Serial.println("  ");
  //AileronSetpoint = aileronRxVal;
  //ElevatorSetpoint = elevatorRxVal;
}




void ControlServos(){                //Temporary loop..... Delete later
  elevatorServo.write(ElevatorSetpoint);
  aileronServo.write(AileronSetpoint);
}



void AccelData(){
  
}


void ElevatorPIDControl(){
  ElevatorInput = pitchAccel;
  ElevatorPID.Compute();
  elevatorServo.write(ElevatorOutput);
  Serial.print(ElevatorOutput);
}


void AileronPIDControl(){
  AileronInput = rollAccel;
  AileronPID.Compute();
  aileronServo.write(AileronOutput);
  Serial.print(AileronOutput);
}
