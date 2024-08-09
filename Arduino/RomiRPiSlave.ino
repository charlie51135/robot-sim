#include <DeadManSwitch.h>

#include <Servo.h>
#include <Romi32U4.h>
#include <PololuRPiSlave.h>
#include "DeadManSwitch.h"
#include <limits.h>
/*
  This program makes the Romi 32U4 Control Board a Raspberry Pi I2C slave.
  Commands and states are exchanged using shared registers. The Romi 32U4
  board does motor control, analog inputs, and other low-level IO, while 
  more complex processing is done on the Raspberry Pi.

  Custom data structure for the buffer. Changes to this buffer must also be
  made in romi_base/src/a_star.cpp

    For the python a_star script provided by Pololu (not used), the sizes are
    descibed by the following standard:
    https://docs.python.org/3.6/library/struct.html#format-characters

    ? - bool           - 1 byte
    c - char           - 1 byte
    B - unsigned char  - 1 byte
    h - short          - 2 bytes
    H - unsigned short - 2 bytes
    f - float          - 4 bytes
    s - char[]         - preceding # is bytes
*/

struct Data
{                                    //Address (byte)
  bool yellow, green, red;             //0 1 2
  bool buttonA, buttonB, buttonC;      //3 4 5

  int16_t leftMotor, rightMotor;       //6-7 8-9
  uint16_t batteryMillivolts;          //10-11
  uint16_t analog[6];                  //12-23

  bool playNotes;                      //24
  char notes[14];                      //25-38

  int16_t leftEncoder, rightEncoder;   //39-40 41-42

  bool newTarget;                      //43
};

PololuRPiSlave<struct Data,5> slave;
PololuBuzzer buzzer;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Encoders encoders;

bool drivingFlag = false;
uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, float distance);
void driveStraight(int speedLeft, int speedRight, float distance);

DeadManSwitch deadManSwitch(1000);  // Set timeout period to 1 second


void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(20);

  // Play startup sound.
  buzzer.play("v10>>g16>>>c16");
}


/* 
  This loop must run continuously for the encoder states to be accurate in ROS.
  Any additional functions must have no delay. They must save their state and
  return to the main loop. An example is given in the drive_straight function.
*/
void loop()
{
  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();

  // Write various values into the data structure.
  slave.buffer.buttonA = buttonA.isPressed();
  slave.buffer.buttonB = buttonB.isPressed();
  slave.buffer.buttonC = buttonC.isPressed();

  if (slave.buffer.buttonA && drivingFlag == false) {
    Serial.print("button pressed\n");
    driveStraight(100,104,100);
  }

  if (drivingFlag == true) {
    driveStraight(100,104,100);
  } else motors.setSpeeds(slave.buffer.leftMotor, slave.buffer.rightMotor);

  // Change this to readBatteryMillivoltsLV() for the LV model.
  slave.buffer.batteryMillivolts = readBatteryMillivolts();

  for(uint8_t i=0; i<6; i++) {
    slave.buffer.analog[i] = analogRead(i);
  }

  // READING the buffer is allowed before or after finalizeWrites().
  ledYellow(slave.buffer.yellow);
  ledGreen(slave.buffer.green);
  ledRed(slave.buffer.red);


  // Update dead man's switch
   deadManSwitch.update(&slave.buffer.newTarget);
  // Stop motors if no new target is received within timeout period
   if (!deadManSwitch.isActive()) {
     motors.setSpeeds(0, 0);
   } else {
     motors.setSpeeds(slave.buffer.leftMotor, slave.buffer.rightMotor);
   }

   // Playing music involves both reading and writing to play once
  static bool startedPlaying = false;
 
  if(slave.buffer.playNotes && !startedPlaying) {
    buzzer.play(slave.buffer.notes);
    startedPlaying = true;
  }
  else if (startedPlaying && !buzzer.isPlaying()) {
    slave.buffer.playNotes = false;
    startedPlaying = false;
  }

  slave.buffer.leftEncoder = encoders.getCountsLeft();
  slave.buffer.rightEncoder = encoders.getCountsRight();

  // When done WRITING, call finalizeWrites() to make modified
  // data available to I2C master.
  slave.finalizeWrites();
}

/*
  Calculates the number of encoder pulses to drive a specific distance
*/ 
uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, float distance) {
  float temp = (wheel_diam * PI) / cnt_per_rev;
  temp = distance / temp;
  return int(temp);
}


/*
  Drives the robot in a straight line while not interrupting the main loop.
  Global variables must be used to save the driving state between function
  calls.
*/ 
int x, leftPulseCount, rightPulseCount, leftPulseCountReq, rightPulseCountReq;
void driveStraight(int speedLeft, int speedRight, float distance) {

  //Drive motor until it has received x pulses
  if (drivingFlag == false) {
    Serial.print("Set Driving Flag true\n");

    drivingFlag = true;
    //Total amount of encoder pulses received
    leftPulseCountReq = encoders.getCountsLeft();
    rightPulseCountReq = encoders.getCountsRight();

    //Amount of encoder pulses needed to achieve distance
    x = countForDistance(7, 1437, distance);

    // Add x to both left and right encoder counts
    leftPulseCountReq = (leftPulseCountReq + x) % INT_MAX;
    rightPulseCountReq = (rightPulseCountReq + x) % INT_MAX;
   
    motors.setSpeeds(speedLeft, speedRight);

  } else { // driving flag == true
    if ((encoders.getCountsLeft() % INT_MAX) < leftPulseCountReq && (encoders.getCountsRight() % INT_MAX) < rightPulseCountReq) {
      
      Serial.print("Currently driving\n");
      // Assign variable to hold encoder count
      leftPulseCount = encoders.getCountsLeft() % INT_MAX;
      rightPulseCount = encoders.getCountsRight() % INT_MAX;

     
      //If the left encoder count is less than the right, increase the left motor speed
      if (leftPulseCount < rightPulseCount) {
        motors.setSpeeds(speedLeft+4, speedRight);
      }
     
      //If the right encoder count is less than the left, increase the right motor speed
      else if (rightPulseCount < leftPulseCount) {
        motors.setSpeeds(speedLeft, speedRight+4);
      }
      else {
        motors.setSpeeds(speedLeft, speedRight);
      }
    } else { // done driving
      motors.setSpeeds(0, 0);
      drivingFlag = false;
    }
  }
}