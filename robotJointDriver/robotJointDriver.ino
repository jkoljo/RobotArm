//***********************************************************
//*  Robot Joint Driver by Juha Koljonen                    *
//*  Takes 100*deg/s for each axis (will divide on rx event)*
//*  Reports 100*deg for each axis (multiplies before send) *
//***********************************************************

#include "ServoMod.h"
#include <AccelStepper.h>
#define teensyLEDPin 13
#define j0enablePin 2
#define j0stepPin 1
#define j0dirPin 0
#define j0faultPin 3
#define j1qaPin 14
#define j1qbPin 15
#define j1drive0Pin 23
#define j1drive1Pin 22
#define j2qaPin 17
#define j2qbPin 16
#define j2drive0Pin 21
#define j2drive1Pin 20
#define j3drivePin 4
#define j3enablePin 12
#define j0scaler 32.0*400.0*110.0/(360.0*23.0)
#define j1scaler 8*(512.0*16.0/(360.0*43.0))
#define j2scaler 15.0/143.0
#define j3scaler 1

#define j1VelocityKp 3
#define j1VelocityKi 18
#define j1VelocityKd 0.001

#define j2VelocityKp 0.5
#define j2VelocityKi 4
#define j2VelocityKd 0.00005

#define j3VelocityKp 1

// Joint state control
AccelStepper joint0(1, j0stepPin, j0dirPin); 
Servo joint3;
boolean j0fault = false;
// Setpoints are in deg/s
float j0sp = 0.0;
float j1sp = 0.0;
float j2sp = 0.0;
float j3sp = 0.0;
// Positions are in quadrature tics
volatile int j1pos = 0;
volatile int j2pos = 0;
volatile boolean j1forward = true;
volatile boolean j2forward = true;
volatile unsigned long lastj1tick = 0;
volatile unsigned long j1ticktime = 100000000;
volatile unsigned long lastj2tick = 0;
volatile unsigned long j2ticktime = 100000000;

boolean lastj1forward = true;
boolean lastj2forward = true;
float j3pos = 90.0;
int lastj3pos = 0;
// Velocity is in deg/s
float j1vel = 0.0;
float j2vel = 0.0;
float j3vel = 0.0;
float j1err = 0.0;
float j1errSum = 0.0;
float j1lastErr = 0.0;
float j2err = 0.0;
float j2errSum = 0.0;
float j2lastErr = 0.0;

// General robot state control
boolean stringComplete = false;
byte bufferLength = 0;
void enableMotors(void);
void disableMotors(void);
void disableJoints(void);
boolean motorsEnabled = false;
long lastUpdateTime = 0;
boolean newData = false;
boolean buzzUp = true;

void setup() {
  pinMode(j0faultPin, INPUT);
  pinMode(j0enablePin, OUTPUT);
  pinMode(j1drive0Pin, OUTPUT);
  pinMode(j1drive1Pin, OUTPUT);
  pinMode(j1qaPin, INPUT_PULLUP); //Enable internal pull up for HP HEDS-5600 encoder
  pinMode(j1qbPin, INPUT_PULLUP);
  pinMode(j2qaPin, INPUT);
  pinMode(j2qbPin, INPUT);
  pinMode(j3enablePin, OUTPUT);
  pinMode(j3drivePin, OUTPUT);
  pinMode(teensyLEDPin, OUTPUT);      // LED to show Teensy is alive
  digitalWriteFast(teensyLEDPin, HIGH);
  disableMotors();
  analogWriteFrequency(j3drivePin, 50);  
  
  // ISR interrupt attachment, handled in ISR_functions file
  attachInterrupt(j2qaPin, isrUpdateJ2ar, RISING);
  attachInterrupt(j1qaPin, isrUpdateJ1ac, CHANGE);
  attachInterrupt(j1qbPin, isrUpdateJ1bc, CHANGE); 
  attachInterrupt(j0faultPin, isrFaultJ0, FALLING);  
  
  joint0.setMaxSpeed(12000);
  joint0.setAcceleration(10);
  
  Serial.begin(9600);
  while (!Serial.dtr()) ;  // Wait for user to start the serial comms
  
  delay(250);
  Serial.println("INFO: Initialization finished");
  Serial.println("INFO: # Syntax #");
  Serial.println("INFO: Control joint velocity (100*deg/s): Ax Bx Cx Dx");
  Serial.println("INFO: Enable motors: T");
  Serial.println("INFO: Disable motors: F");
  Serial.println("INFO: Zero joint positions: Z");
}

void loop() {
  // If input data is available, SerialComm file handles it and writes stringComplete
  if (Serial.available()) serialRead();
  
  if (stringComplete) {
    if (code_seen('F')) disableMotors();
    else if (code_seen('T')) enableMotors();
    else if (code_seen('Z')) zeroJoints();
    else {
      // Get joint velocity setpoints from received serial data
      if (getBlock('A', j0sp)) {
        // No need to constrain velocity, this is handled by AccelStepper lib
        if (motorsEnabled) joint0.setSpeed(j0sp*j0scaler/100);
      }
    
      if (getBlock('B', j1sp)) {
        j1sp = constrain(j1sp/100, -60, 60);
      }
    
      if (getBlock('C', j2sp)) {
        j2sp = constrain(j2sp/100, -60, 60);
      }
      
      if (getBlock('D', j3sp)) {
       j3sp = constrain(j3sp/100, -100, 100);
      }
    }
    // Got all we want, dump buffer
    stringComplete = false;
    bufferLength = 0; 
  }
 
  int updatePeriod = 0;  
  if (j1sp < 5.0)
    updatePeriod = 10000;
  else
    updatePeriod = 5000;
 
  
  if (micros() > (lastUpdateTime + updatePeriod)) {
    // Time in seconds = difference in microseconds / 1 000 000
    double time = ((double)(micros() - (double)lastUpdateTime))/1000000.0;
    
    if (micros() > lastj1tick + 200000) {
      // Pulse read timeout, velocity zero
      j1vel = 0.0;
    } else if (micros() > (j1ticktime + lastj1tick)) {
      // Going slower than last tick period, extrapolate time and thus speed
      double j1ticktimeEst = (micros()-lastj1tick);
      // j1 forward!?!?!
      j1vel = abs(j1vel);
      j1vel = 0.15*j1vel + 0.85*((j1scaler/128) / (((double)j1ticktimeEst)/1000000.0));
    } else j1vel = (j1scaler/128) / (((double)j1ticktime)/1000000.0);

    if (micros() > lastj2tick + 200000) {
      // Pulse read timeout, velocity zero
      j2vel = 0.0;
    } else if (micros() > (j2ticktime + lastj2tick)) {
      // Going slower than last tick period, extrapolate time and thus speed
      double j2ticktimeEst = (micros()-lastj2tick);
      j2vel = abs(j2vel);
      j2vel = 0.15*j2vel + 0.85*((j2scaler) / (((double)j2ticktimeEst)/1000000.0));
    } else j2vel = (j2scaler) / (((double)j2ticktime)/1000000.0);

    if (j1vel < 10.0) {
      if (!j1forward) j1vel *= -1;
      lastj1forward = j1forward;
    } else if (!lastj1forward) j1vel *= -1;
    
    if (j2vel < 10.0) {
      if (!j2forward) j2vel *= -1;
      lastj2forward = j2forward;
    } else if (!lastj2forward) j2vel *= -1;
    
    lastUpdateTime = micros();
    
    if (motorsEnabled) {
      driveJoint1Velocity(time);
      driveJoint2Velocity(time);
      driveJoint3Velocity(time);
    }
    
    Serial.print(int((joint0.currentPosition()*50/93)));
    Serial.print("\t ");
    Serial.print(int((j1pos*100)/j1scaler));
    Serial.print("\t ");
    Serial.print(int((j2pos*100)*j2scaler));
    Serial.print("\t ");
    Serial.println(int((j3pos-90)*(-100)));
    newData = false;
  }
  
  // Run joint0 separately as fast as possible
  if (motorsEnabled) {
    joint0.runSpeed();
  }
}

void driveJoint1Velocity(double time) {
  // Drive joint 1 velocity deg/s
  j1err = (int(j1sp*100) - (int(j1vel*100)));
  j1err = j1err/100.0;
  int j1effort = 0;
  j1errSum += (j1err * time);
  j1errSum = constrain(j1errSum, -70,70);
  double dErr = (j1err - j1lastErr) / time;
  dErr = constrain(dErr, -50, 50);
  
  j1effort = constrain(j1err*j1VelocityKp + j1errSum*j1VelocityKi + dErr*j1VelocityKd, -127, 127);  
  
  /*if (buzzUp) {
    j1effort = constrain(j1effort+15, -127, 127);
    buzzUp = !buzzUp;
  } else {
    j1effort = constrain(j1effort-15, -127, 127);
    buzzUp = !buzzUp;
  }*/
  
  analogWrite(j1drive0Pin, 127+j1effort);
  analogWrite(j1drive1Pin, 127-j1effort);
}

void driveJoint2Velocity(double time) {
  // Drive joint 2 velocity deg/s
  j2err = (int(j2sp*100) - (int(j2vel*100)));
  j2err = j2err/100.0;
  int j2effort = 0;
  j2errSum += (j2err * time);
  j2errSum = constrain(j2errSum, -70,70);
  double dErr = (j2err - j2lastErr) / time;
  dErr = constrain(dErr, -50, 50);
  
  if ((j2sp < 5.0) && (j2err < 5.0))
    j2effort = constrain(j2err*j2VelocityKp + j2errSum*j2VelocityKi + dErr*j2VelocityKd, -127, 127);  
  else j2effort = constrain(j2err*j2VelocityKp + j2errSum*j2VelocityKi*2 + dErr*j2VelocityKd, -127, 127);  
  analogWrite(j2drive0Pin, 127+j2effort);
  analogWrite(j2drive1Pin, 127-j2effort);
}

void driveJoint3Velocity(double time) {
  // Do the inverse of j1 and j2 behavior, calculate pos setpoint from velocity
  j3pos = constrain(j3pos-j3sp*time, 0, 180);
  joint3.write(int(j3pos));
}

void zeroJoints(void) {
  joint0.setCurrentPosition(0);
  j1pos = 0;
  j2pos = 0;
  j3pos = 90.0;
}

void enableMotors(void) {
  // Enable joint 0 motor controller (required for operation)
  digitalWrite(j0enablePin, LOW);
  joint3.attach(j3drivePin, 720, 2680);   // Attach servo with 544-2400us ctrl range
  digitalWrite(j3enablePin, HIGH);  
  motorsEnabled = true;
  Serial.println("INFO: motors enabled");
}

void disableMotors(void) {
  // Disable drive immediately
  joint0.setSpeed(0);
  digitalWrite(j0enablePin, HIGH);
  analogWrite(j1drive1Pin, 127);
  analogWrite(j1drive0Pin, 127);
  analogWrite(j2drive0Pin, 127);
  analogWrite(j2drive1Pin, 127);
  digitalWriteFast(j3enablePin, LOW);
  joint3.detach();
  j1errSum = 0.0;
  j2errSum = 0.0;
  motorsEnabled = false;
  Serial.println("INFO: motors disabled");
}

