#include <SR04.h>

#include "TimerOne.h"
#include <SD.h>


#define TRIG_PIN A0
#define ECHO_PIN A1

const int SensorRight_2 = A4;     // Right  Infrared sensor
const int SensorLeft_2 = A5;     // Left  Infrared sensor


int Left_motor_back = 9; 
int Left_motor_go = 5; 
int Right_motor_go = 6; 
int Right_motor_back = 10; 
int Right_motor_en = 8; 
int Left_motor_en = 7; 
int Sensor1 = 13;
int Sensor2 = 12;
int m2 = 5; // Links
int m1 = 6;  // Rechts
int key = 4;

unsigned long m1_pulses_0=0;
unsigned long m2_pulses_0=0;

unsigned long t_speed_0=0;
double m1_speed=0;
unsigned long m1_speed_0=0;
double m2_speed=0;
unsigned long m2_speed_0=0;

const int SPEED_TIMEFRAME=200;

double m1_desiredSpeed = 0;
double m2_desiredSpeed = 0;

bool m1_forward = true;
bool m2_forward = true;

double m1_pulses = 0;
int sensor1State = 0;

double m2_pulses = 0;
int sensor2State = 0;

double m1_controllVariable = 0;
double m1_dV = 0;

unsigned long m1_t0;
double m1_I = 0;
double m1_P0 = 0;

double m2_controllVariable = 0;
unsigned long m2_t0;
double m2_I = 0;
double m2_P0 = 0;

unsigned long t0 = 0;
unsigned long t1 = 0;
unsigned long t2 = 0;
unsigned long tdata_0 = 0;

const int row = 20;
int data_s[row*10];
int data_d[row*10];
int i = 0;

int driveState = 0;

const int STOP_SPEED = 0;
const int M1_NORMAL_SPEED = 50;
const int M2_NORMAL_SPEED = 50;

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

void setup() {
  pinMode(SensorRight_2, INPUT);
  pinMode(SensorLeft_2, INPUT);

  pinMode(key,INPUT);

  digitalWrite(key,HIGH);
 
  pinMode(Sensor1, INPUT); 
  pinMode(Sensor2, INPUT); 

  Serial.begin(57600, SERIAL_8N1);
  while (! Serial);

  analogWrite(Left_motor_go, 0);
}


void driveM1(bool forward, int myspeed) {
  if (forward) {
    digitalWrite(Right_motor_back,LOW);    
  } else {
    digitalWrite(Right_motor_back,HIGH);    
  }
  analogWrite(Right_motor_go, myspeed);
}

void driveM2(bool forward, int myspeed) {
  if (forward) {
    digitalWrite(Left_motor_back,LOW);    
  } else {
    digitalWrite(Left_motor_back,HIGH);    
  }
  analogWrite(Left_motor_go, myspeed);
}

double countPulses(int sensor, unsigned long now, int& state, double& pulses) {
   int SR = digitalRead(sensor); 
   switch(state) {
    case 0:
      if (SR == HIGH ) {
        state = 1;    
      }
      break;
    case 1:
       if (SR == LOW ) {
          state = 2;
       }  
       break;
    case 2:
       if (SR == HIGH ) {
          state = 3;    
       }  
       break;   

    case 3:
       if (SR == LOW ) {
          state = 2;  
          pulses += 1;  
       }  
       break;      
   }
}

void updateController(unsigned long now, double sollWert, double istWert, double kp, double ki, double kd, unsigned long& tc0, double& I, double& P0, double& result) {
  if (now >= (tc0 + 100)){
      double P = (sollWert - istWert);    
      double dt = (now - tc0)/1000.0;
      I = I + P * dt;
      if (ki > 0) {
        I = min(1.0/ki, max(-1.0/ki, I));
      }
      double D = (P - P0)/dt;
      double dv = kp * P + ki * I + kd * D;
      result = min(255, max(0, (int)(result + dv)));

      tc0 = now;
      P0 = P;
  }
}

bool alreadySend = false;

void runEvery(unsigned long now, long runIn, unsigned long& start, void f()) {
  if (now >= start + runIn){
    f();
    start = now;
  }
}

void goForward() {
  driveState = 2;
}

void stop() {
  driveState = 0;
  m1_forward=true;
  m2_forward=true;
  m1_desiredSpeed = STOP_SPEED;
  m2_desiredSpeed = STOP_SPEED;
}


void checkDistance() {  
  long dist = sr04.Distance();
  if (dist < 30) {
    driveState = 4;
//    Serial.println("blockiert! turn...");
  }
}
void checkForEmergencyStop() {
  int SR = digitalRead(SensorRight_2);
  int SL = digitalRead(SensorLeft_2);
  if (SR == LOW || SL == LOW) {
    stop();
//    Serial.println("emergency stop.");
  }
}


void calculateSpeed(){
  m1_speed = (m1_pulses - m1_speed_0) * 1000/SPEED_TIMEFRAME;
  m2_speed = (m2_pulses - m2_speed_0) * 1000/SPEED_TIMEFRAME;
  m1_speed_0 = m1_pulses;
  m2_speed_0 = m2_pulses;
}

void controlMotors() {
  // best: double kp = 0.6; double ki = 0.75; double kd = 0.1;
  double kpkrit = 1.95; 
  double tkrit = 1.0;
   
  //double kp = 0.45 * kpkrit; double ki = 0.85 * tkrit; double kd = 0;
  double kp = 0.6 * kpkrit; double ki = 0.5 * tkrit; double kd = 0.12 * tkrit;
  
  unsigned long now = millis();
  updateController(now, m1_desiredSpeed, m1_speed, kp, ki, kd, m1_t0, m1_I, m1_P0, m1_controllVariable);
  driveM1(m1_forward, m1_controllVariable);
  //driveM1(m1_forward, m1_desiredSpeed);
  
  updateController(now, m2_desiredSpeed, m2_speed, kp, ki, kd, m2_t0, m2_I, m2_P0, m2_controllVariable);
  driveM2(m2_forward, m2_controllVariable);
  //driveM2(m2_forward, m2_desiredSpeed);
}
void sendMotorTrackingData() {
  if (!alreadySend) {
     for (int j=0;j<row*10;j++){
        Serial.print(j);
        Serial.print(";");
        Serial.print(data_d[j]);
        Serial.print(";");
        Serial.println(data_s[j]);
        alreadySend = true;
      }
      i=0;
   }
    
}

void loop() { 
  unsigned long now = millis(); 

  countPulses(Sensor1, now, sensor1State, m1_pulses);
  countPulses(Sensor2, now, sensor2State, m2_pulses);
  
  runEvery(now, SPEED_TIMEFRAME, t_speed_0, calculateSpeed);
  
  checkForEmergencyStop();
  
  controlMotors();
  
  //runEvery(now, 10000, t1, sendMotorTrackingData);
  
  switch(driveState) {
    case 0: // init state, wait for button
      if (digitalRead(key) == LOW) {
        t0 = now;
        t1 = now;
        driveState = 1;
        m1_pulses=0;
        m1_pulses_0=0;
        tdata_0 = now;
        i=0;
      }
      break;
    case 1: // wait for time to start
      runEvery(now, 1000, t0, goForward);
      break;
    case 2: // forward state
      m1_forward = true;
      m2_forward = true;
      m1_desiredSpeed = M1_NORMAL_SPEED;
      m2_desiredSpeed = M2_NORMAL_SPEED;
      driveState = 3;    
      t2 = now;
      break;  
    case 3: // normale fahrt gerade aus, check for distance
      runEvery(now, 500, t2, checkDistance);

      //runEvery(now, 8000, t2, stop);
      break;
    case 4: // init turn  
      m1_forward = true;
      m2_forward = false;
      m1_desiredSpeed = M1_NORMAL_SPEED;
      m2_desiredSpeed = M1_NORMAL_SPEED;
      m1_pulses_0 = m1_pulses;
      driveState = 5;
      break;
    case 5:
      // wait for turn complete
      runEvery(m1_pulses, 9, m1_pulses_0, goForward);
      break;
  }   
  
  if (now > tdata_0 + 50 && i < row*10) {
      data_s[i] = m1_speed;
      data_d[i] = m1_desiredSpeed;
      tdata_0 = now;
      i++;
  } 
}

