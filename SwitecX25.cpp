/*
 *  SwitecX25 Arduino Library
 *  Guy Carpenter, Clearwater Software - 2012
 *
 *  Licensed under the BSD2 license, see license.txt for details.
 *
 *  All text above must be included in any redistribution.
 */

#include <Arduino.h>

#include "SwitecX25.h"

// During zeroing we will step the motor CCW 
// with a fixed step period defined by RESET_STEP_MICROSEC
#define RESET_STEP_MICROSEC 800

// This table defines the acceleration curve as a list of (step, delay) pairs.
// 1st value is the cumulative step count since starting from rest, 2nd value is delay in microseconds.
// 1st value in each subsequent row must be > 1st value in previous row
// The delay in the last row determines the maximum angular velocity.
static unsigned short defaultAccelTable[][2] = {
  {   20, 3000},
  {   50, 1500},
  {  100, 1000},
  {  150,  800},
  {  300,  600}
};
#define DEFAULT_ACCEL_TABLE_SIZE (sizeof(defaultAccelTable)/sizeof(*defaultAccelTable))

// experimentation suggests that 400uS is about the step limit 
// with my hand-made needles made by cutting up aluminium from
// floppy disk sliders.  A lighter needle will go faster.
  
// State  3 2 1 0   Value
// 0      1 0 0 1   0x9
// 1      0 0 0 1   0x1
// 2      0 1 1 1   0x7
// 3      0 1 1 0   0x6
// 4      1 1 1 0   0xE
// 5      1 0 0 0   0x8
static byte stateMap[] = {0x9, 0x1, 0x7, 0x6, 0xE, 0x8};

SwitecX25::SwitecX25(unsigned int steps, unsigned char pin1, unsigned char pin2, unsigned char pin3, unsigned char pin4)
{
  this->currentState = 0;
  this->steps = steps;
  this->pins[0] = pin1;
  this->pins[1] = pin2;
  this->pins[2] = pin3;
  this->pins[3] = pin4;
  for (int i=0;i<pinCount;i++) {
    pinMode(pins[i], OUTPUT);
  }
  
  vel = 0; 
  currentStep = 0;
  targetStep = 0;
  nextTime = 0;

  accelTable = defaultAccelTable;
  maxVel = defaultAccelTable[DEFAULT_ACCEL_TABLE_SIZE-1][0]; // last value in table.
}

bool SwitecX25::isStopped(void) { 
  return currentStep == targetStep && vel == 0; 
}

void SwitecX25::writeIO()
{
  byte mask = stateMap[currentState];  
  for (int i=0;i<pinCount;i++) {
    digitalWrite(pins[i], mask & 0x1);
    mask >>= 1;
  }
}

void SwitecX25::stepUp()
{
  if (currentStep < steps) {
    currentStep++;
    currentState = (currentState + 1) % stateCount;
    writeIO();
  }
}

void SwitecX25::stepDown()
{ 
  if (currentStep > 0) {
    currentStep--;
    currentState = (currentState + 5) % stateCount;
    writeIO();
  }
}

void SwitecX25::zero()
{
  currentStep = steps - 1;
  for (unsigned int i=0;i<steps;i++) {
    stepDown();
    delayMicroseconds(RESET_STEP_MICROSEC);
  }
  currentStep = 0;
  targetStep = 0;
  vel = 0;
}

// This function determines the speed and accel
// characteristics of the motor.  Ultimately it 
// steps the motor once (up or down) and computes
// the delay until the next step.  Because it gets
// called once per step per motor, the calcuations
// here need to be as light-weight as possible, so
// we are avoiding floating-point arithmetic.
//
// To model acceleration we maintain vel, which indirectly represents
// velocity as the number of motor steps travelled under acceleration
// since starting.  This value is used to look up the corresponding
// delay in accelTable.  So from a standing start, vel is incremented
// once each step until it reaches maxVel.  Under deceleration 
// vel is decremented once each step until it reaches zero.

void SwitecX25::advance()
{   
  // If we have nowhere to go then just return
  if (this->isStopped()) { 
    return;
  }
  
  // determine distance, number of steps  to target.
  // may be negative if we are headed away from target
  int distance = (int) targetStep - (int) currentStep;
  
  if (distance - vel > 0 && vel < (int) maxVel) {
	  vel++;
  }
  else if (distance - vel < 0 && vel > -((int) maxVel)) {
	  vel--;
  }

  if (vel > 0) {
    stepUp();
  } else if (vel < 0) {
    stepDown();
  }
   
  // vel now defines delay
  unsigned char i = 0;
  // this is why vel must not be greater than the last vel in the table.
  while (accelTable[i][0] < abs(vel)) {
    i++;
  }

  nextTime = micros() + accelTable[i][1];
}

void SwitecX25::setPosition(unsigned int pos)
{
  // pos is unsigned so don't need to check for <0
  if (pos >= steps) pos = steps-1;
  targetStep = pos;
}

void SwitecX25::update()
{
  if (micros() >= nextTime) {
    advance();
  }
}

//This updateMethod is blocking, it will give you smoother movements, but your application will wait for it to finish
void SwitecX25::updateBlocking()
{
  while (!this->isStopped()) {
    update();
  }
}

