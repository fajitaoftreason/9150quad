

#define NO_PORTD_PINCHANGES
#define NO_PORTC_PINCHANGES

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <Servo.h>



MPU9150Lib MPU;                                              // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (60)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (60)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       30                  // a good mix value 

//  MPU_LPF_RATE is the low pass filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   40

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

//the pins to which each ESC is attached, the orientation is defined as north to the front of the quadcopter.
#define SW_ESC 4
#define SE_ESC 5
#define NW_ESC 6
#define NE_ESC 7

//the pins to which each channel of the RF receiver is attached
#define CH1 8
#define CH2 9
#define CH3 10
#define CH4 11

//which axis corrosponds to each channel of the RF reciever
#define ROLL 0  //X Axis
#define PITCH 1  // Y Axis
#define THROTTLE 2
#define YAW 3   //Z Axis

// my sensor chip is not mounted level, these values are in radians.
#define X_CENTER -.07
#define Y_CENTER .04

//PID tuning values
#define KP 100
#define YAW_KP 50

#define TRUE 1
#define FALSE 0

byte inputpins[] = {CH1, CH2, CH3, CH4};
volatile float rfInput[4] = {0};
volatile byte currentCH = 0;
unsigned long time = 0;
float errorX, errorY, errorZ, setpointX, setpointY, setpointZ, correctionX, correctionY, correctionZ;
int throttle;
int nwPow, nePow, swPow, sePow;
Servo nwEsc;
Servo neEsc;
Servo swEsc;
Servo seEsc;

void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
  
  for (byte i = 0; i<4; i++){
    pinMode(inputpins[i], INPUT);
    digitalWrite(inputpins[i], HIGH);
    PCintPort::attachInterrupt(inputpins[i], &pinChange, CHANGE);
  }
  
  nwEsc.attach(NW_ESC);
  neEsc.attach(NE_ESC);
  swEsc.attach(SW_ESC);
  seEsc.attach(SE_ESC);
  
  setpointZ = 0;
}

void loop()
{  
  if (MPU.read()) {                                        // get the latest data if ready yet
    //MPU.printAngles(MPU.m_fusedEulerPose);                 // print the output of the data fusion
    /*for (byte i = 0; i<4; i++){
      Serial.print("\t");
      Serial.print(rfInput[i]);
    }*/
    
    
    throttle = constrain(map(rfInput[THROTTLE], 1100, 1800, 900, 2000), 900, 2000);
    //Serial.print("\t");
    //Serial.print(rfInput[THROTTLE]);
    
    if (micros() - time > 100000){   //lost connection for more than 1/10 of a second
      //Serial.print("\t Connection Lost");
      rfInput[THROTTLE] *= .995; //the throttle will ramp down (fairly quickly) when the connection to the RF transmitter is lost
    }

    nePow = sePow = nwPow = swPow = throttle;
    
    //PID time
    
    //X axis (roll) controller
    setpointX = X_CENTER + (rfInput[ROLL] - 1500)/1300;
    errorX = setpointX - MPU.m_fusedEulerPose[0];
    correctionX = errorX * KP;
    
    if(correctionX < 0){
      nwPow += correctionX;
      swPow += correctionX;
    }else{
      nePow -= correctionX;
      sePow -= correctionX;
    }
    
    //Y axis (pitch) controller
    setpointY = Y_CENTER + (rfInput[PITCH] - 1500)/1300;
    errorY = setpointY - MPU.m_fusedEulerPose[1];
    correctionY = errorY * KP;
    
    if(correctionY < 0){
      nwPow += correctionY;
      nePow += correctionY;
    }else{
      swPow -= correctionY;
      sePow -= correctionY;
    }
    
    //Z axis (yaw) controller
    //todo: make the yaw stick actually affect this
    errorZ = setpointZ - MPU.m_fusedEulerPose[2];
    correctionZ = errorZ * YAW_KP;
    
    if(correctionZ < 0){
      nwPow +=correctionZ;
      sePow +=correctionZ;
    }else{
      nePow -=correctionZ;
      swPow -=correctionZ;
    }
    
    /*
    Serial.print("\t");
    Serial.print(constrain(nwPow, 1000, 2000));
    Serial.print("\t");
    Serial.print(constrain(nePow, 1000, 2000));
    Serial.print("\t");
    Serial.print(constrain(swPow, 1000, 2000));
    Serial.print("\t");
    Serial.print(constrain(sePow, 1000, 2000));*/
    Serial.println();
    
    neEsc.write(constrain(nePow, 900, 2000));
    nwEsc.write(constrain(nwPow, 900, 2000));
    seEsc.write(constrain(sePow, 900, 2000));
    swEsc.write(constrain(swPow, 900, 2000));
  }
   
}

void pinChange(){
  if (PCintPort::arduinoPin == inputpins[currentCH]){
    if (PCintPort::pinState == HIGH){   //on rise start timer
      time = micros();
    }else{
      rfInput[currentCH] = micros() - time;  //on fall get the how long it was high for
      currentCH++;                  //change which pin we're going to listen to
      currentCH = currentCH % 4;
    }  
  }
}
