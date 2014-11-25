

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
#include <TimerOne.h>
#include <Servo.h>



MPU9150Lib MPU;                                              // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (60)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (10)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       20                  // a good mix value 

//  MPU_LPF_RATE is the low pass filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   40

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200


#define ESC1 4

#define CH1 8
#define CH2 9
#define CH3 10
#define CH4 11

#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define YAW 3


byte inputpins[] = {CH1, CH2, CH3, CH4};
volatile float rfInput[4] = {0};
volatile byte currentCH = 0;
long throttle = 0;
Servo esc1;

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
  
  esc1.attach(ESC1);
    
    Timer1.initialize(2200);    //TODO: set this to a timout duration and add an interrupt for connection lost
    Timer1.stop();                //stop the counter
    Timer1.restart();            //set the clock to zero
  
}

void loop()
{  
  if (MPU.read()) {                                        // get the latest data if ready yet
    MPU.printAngles(MPU.m_fusedEulerPose);                 // print the output of the data fusion
    for (byte i = 0; i<4; i++){
      Serial.print("\t");
      Serial.print(rfInput[i]);
    }
    throttle = constrain(map(rfInput[THROTTLE], 1100, 1800, 700, 2300), 700, 2300);
    Serial.print("\t");
    Serial.print(rfInput[THROTTLE]);
    Serial.println();
    
    esc1.writeMicroseconds(rfInput[THROTTLE]);  //this is actually going to make a motor do stuff.  scary.  It's got a sharp propeller attached to it.
  }
   
}

void pinChange(){
  if (PCintPort::arduinoPin == inputpins[currentCH]){
    if (PCintPort::pinState == HIGH){
      Timer1.restart();
      Timer1.start();
    }else{
      rfInput[currentCH] = Timer1.read();
      Timer1.stop();
      //PCintPort::detachInterrupt(inputpins[currentCH]);     //stop listening to this pin
      currentCH++;                                          //change which pin we're going to listen to
      currentCH = currentCH % 4;

      //PCintPort::attachInterrupt(inputpins[currentCH], &pinChange, CHANGE);        //listen to it
    }  
  }
}
