#include <PID_v1.h>

#include "TimerOne.h"
#include <SD.h>

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

//PID m1PID(&m1_speed, &m1_controllVariable, &m1_desiredSpeed,1,3,0.1, DIRECT);
//PID m2PID(&m2_speed, &m2_controllVariable, &m2_desiredSpeed,1,3,0.1, DIRECT);

PID m1PID(&m1_speed, &m1_controllVariable, &m1_desiredSpeed,3,1.8,0.05, DIRECT);
PID m2PID(&m2_speed, &m2_controllVariable, &m2_desiredSpeed,3,1.8,0.05, DIRECT);
unsigned long t0 = 0;
unsigned long t1 = 0;
unsigned long tdata_0 = 0;

const int row = 20;

int data_s[row*10];
int data_d[row*10];

int i = 0;
int driveState = 0;

const int STOP_SPEED = 0;
const int NORMAL_SPEED = 60;

void setup()
{
  pinMode(key,INPUT);// Set button as input

  digitalWrite(key,HIGH);//Initialize button
 
  
  //Initialize motor drive for output mode
//  pinMode(Left_motor_go,OUTPUT);
//  pinMode(Left_motor_back,OUTPUT);
    pinMode(Sensor1, INPUT); 

  Serial.begin(57600, SERIAL_8N1);
  while (! Serial);

//   if (!SD.begin(chipSelect)) {
//    Serial.println("Card failed, or not present");
//    // don't do anything more:
//    return;
//  }
//  Serial.println("card initialized.");
   //digitalWrite(Right_motor_go,HIGH);// right motor go ahead
  //digitalWrite(Right_motor_back,LOW);  
 
  m1PID.SetMode(AUTOMATIC);
  m1PID.SetSampleTime(100);
  m2PID.SetMode(AUTOMATIC);
  m2PID.SetSampleTime(100);

//  attachInterrupt(digitalPinToInterrupt(2), pulse, FALLING);
//  attachInterrupt(digitalPinToInterrupt(2), pulse, FALLING);

  digitalWrite(Left_motor_go,HIGH);
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go, 0);
}


void forwardM1(int myspeed) {
  digitalWrite(Right_motor_back,LOW);  
  analogWrite(Right_motor_go, myspeed);
}

void forwardM2(int myspeed) {
  digitalWrite(Left_motor_back,LOW);  
  analogWrite(Left_motor_go, myspeed);
}

void backwardM1(int myspeed) {
  digitalWrite(Right_motor_go,LOW);
  digitalWrite(Right_motor_back,HIGH);  
  analogWrite(Right_motor_go, myspeed);
}

void backwardM2(int myspeed) {
  digitalWrite(Left_motor_go,LOW);
  digitalWrite(Left_motor_back,HIGH);  
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
   //Serial.println(state);
}

void updateController(unsigned long now, double sollWert, double istWert, double kp, double ki, double kd, unsigned long& tc0, double& I, double& P0, double& result) {
  if (now >= (tc0 + 100)){
      double P = (sollWert - istWert);
      
      //if (abs(P) > 0.01) {
        double dt = (now - tc0)/1000.0;
        I = I + P * dt;
  
        if (ki > 0) {
          I = min(1.0/ki, max(-1.0/ki, I));
        }
        double D = (P - P0)/dt;
        
        double dv = kp * P + ki * I + kd * D;
       
        result = min(255, max(0, (int)(result + dv)));
     
//      } else {
//        Serial.println("<0.01");
//      }
    tc0 = now;
    P0 = P;
  }
}

bool alreadySend = false;

void loop() { 
  bool m1_ready = false;
  bool m2_ready = false;
  
  unsigned long now = millis(); 
  
  countPulses(Sensor1, now, sensor1State, m1_pulses);
  countPulses(Sensor2, now, sensor2State, m2_pulses);
  
  if (now > t_speed_0 + SPEED_TIMEFRAME) {
    m1_speed = (m1_pulses - m1_speed_0) * 1000/SPEED_TIMEFRAME;
    m2_speed = (m2_pulses - m2_speed_0) * 1000/SPEED_TIMEFRAME;
    t_speed_0 = now;
    m1_speed_0 = m1_pulses;
    m2_speed_0 = m2_pulses;
  }



//  m1PID.Compute();
//  m2PID.Compute();
//  best 00:34 0.6, 0.75, 0.1
  updateController(now, m1_desiredSpeed, m1_speed, 0.6, 0.75, 0.1, m1_t0, m1_I, m1_P0, m1_controllVariable);
  forwardM1(m1_controllVariable);

  
  updateController(now, m2_desiredSpeed, m2_speed, 0.6, 0.75, 0.1, m2_t0, m2_I, m2_P0, m2_controllVariable);
  forwardM2(0);

  

  if (now > t1 + 10000) {
    if (!alreadySend) {
      
    
     for (int j=0;j<row*10;j++){
      
      Serial.print(j);
      Serial.print(";");
      Serial.print(data_d[j]);
      Serial.print(";");
      Serial.println(data_s[j]);
      alreadySend = true;
      }
    }
      

//    String dataString = String(now);
//    dataString += ";";
//    dataString += String(m1_desiredSpeed);
//    dataString += ";";
//    dataString += String(m1_speed);
//    dataString += ";";
//    dataString += String(m1_controllVariable);
//    dataString += ";";
//    dataString += String(m2_desiredSpeed);
//    dataString += ";";
//    dataString += String(m2_speed);
//    dataString += ";";
//    dataString += String(m2_controllVariable);
//    Serial.println(dataString);
    t1 = now;
    i=0;
  }

  switch(driveState) {
    case 0:
      if (digitalRead(key) == LOW) {
        t0=now;
        driveState = 1;
        m1_pulses=0;
        m2_pulses=0;
        m1_pulses_0=0;
        m2_pulses_0=0;
        
      }
      break;
    case 1: 
      if (now > t0 + 2000) {

        
        m1_desiredSpeed = NORMAL_SPEED;
        m2_desiredSpeed = NORMAL_SPEED;
        t0 = now;
        t1 = now;
        driveState = 2;
        i=0;
      }
      break;
    case 2: 
      if (m1_pulses > m1_pulses_0 + 300) {
        
        
        m1_desiredSpeed = STOP_SPEED;
        driveState=3;
      } else if (m2_pulses > m2_pulses_0 + 300) {
        Serial.print(driveState);
        
        m2_desiredSpeed = STOP_SPEED;
        driveState=4;
      }
      break;
    case 3: 
      if (m2_pulses > m2_pulses_0 + 300) {
        Serial.print(driveState);
        
        m2_desiredSpeed = STOP_SPEED;
        driveState=0;
        m1_pulses_0 = m1_pulses;
        m2_pulses_0 = m2_pulses;
      }
      break;  
    case 4: 
      if (m1_pulses > m1_pulses_0 + 300) {
        Serial.print(driveState);
        
        m1_desiredSpeed = STOP_SPEED;
        driveState=0;
        m1_pulses_0 = m1_pulses;
        m2_pulses_0 = m2_pulses;
      }
      break;    
    case 5: 
      break;      
    case 98: 
      if (digitalRead(key) == LOW) {
          
          t0=now;
          driveState = 0;
          forwardM1(STOP_SPEED);
//          forwardM2(STOP_SPEED);  
          delay(1000); 
        }
      break;      
    case 99: 
      break;      
  }
    
  
  //calcIstWert(Sensor1, now, sensor1StartTime, sensor1State, sensor1istWertCPM);
  //calcIstWert(Sensor2, now, sensor2StartTime, sensor2State, sensor2istWertCPM);
  //m1PID.Compute();
  //m2PID.Compute();
  
//  run(m1, m1_controllVariable);   
//  run(m2, m2_controllVariable);   
   
   if (now > tdata_0 + 50 && i < row*10) {
      data_s[i] = m1_speed;
      data_d[i] = m1_desiredSpeed;
      tdata_0 = now;
      i++;
    } 

//  if (now > t1 + 1000) {
//    Serial.println(currentSpeed);
    
//    for (int j=0;j<row*10;j++){
//      Serial.print(data_now[j]);
//      Serial.print(";");
//      Serial.println(data_ist[j]);
//      
//      i = 0;
    
    
//    t1 = now;
//  }
}


