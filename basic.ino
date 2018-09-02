#include <PID_v1.h>
#include "TimerOne.h"

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
int beep=3; 

//unsigned long sensor1StartTime = 0;
double m1_pulses = 0;
int sensor1State = 0;

//unsigned long sensor2StartTime = 0;
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

double sollWert = 0;


volatile unsigned long t_start = micros();

int currentSpeed = 0;

//PID m1PID(&sensor1istWertCPM, &m1_controllVariable, &sollWert,0.8,0.3,0.01, DIRECT);
//PID m2PID(&sensor2istWertCPM, &m2_controllVariable, &sollWert,2,1,0.1, DIRECT);

volatile unsigned long pulseCounter = 0;
unsigned long oldCounter = 0;
unsigned long t0 = 0;
unsigned long t1 = 0;

const int row = 10;

double data_now[row*10];
double data_ist[row*10];
double data_stell[row*10];
int i = 0;
int driveState = 0;

const int STOP_SPEED = 0;
const int NORMAL_SPEED = 100;
const int TURN_SPEED = 100;

void setup()
{
  pinMode(key,INPUT);// Set button as input
  pinMode(beep,OUTPUT); // Set buzzer as output

  digitalWrite(key,HIGH);//Initialize button
  digitalWrite(beep,HIGH);// set buzzer mute
  
  //Initialize motor drive for output mode
//  pinMode(Left_motor_go,OUTPUT);
//  pinMode(Left_motor_back,OUTPUT);
    pinMode(Sensor1, INPUT); 

  Serial.begin(9600);
  while (! Serial);
   Serial.println("setup");
   
   //digitalWrite(Right_motor_go,HIGH);// right motor go ahead
  //digitalWrite(Right_motor_back,LOW);  
 
  //m1PID.SetMode(AUTOMATIC);
  //m1PID.SetSampleTime(10);
//  m2PID.SetMode(AUTOMATIC);
//  m2PID.SetSampleTime(10);

//  attachInterrupt(digitalPinToInterrupt(2), pulse, FALLING);
//  attachInterrupt(digitalPinToInterrupt(2), pulse, FALLING);
}

//void resetPulseInt() {
//  Timer1.detachInterrupt();
//  attachInterrupt(digitalPinToInterrupt(2), pulse, FALLING);
//}

//void pulse() {
//  pulseCounter++;
//  Timer1.initialize(1 * 100);
//  Timer1.attachInterrupt(resetPulseInt);
//  detachInterrupt(digitalPinToInterrupt(2));
//}

void forwardM1(int myspeed) {
  digitalWrite(Right_motor_go,HIGH);
  digitalWrite(Right_motor_back,LOW);  
  analogWrite(Right_motor_go, myspeed);
}

void forwardM2(int myspeed) {
  digitalWrite(Left_motor_go,HIGH);
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

void updateController(unsigned long now, double sollWert, double istWert, double kp, double ki, double kd, unsigned long& t0, double& I, double& P0, double& result) {
  if (now >= (t0 + 100)){
      double P = (sollWert - istWert);
      
      if (abs(P) > 0.01) {
        double dt = (now - t0)/1000.0;
        I = I + P * dt;
  
        if (ki > 0) {
          I = min(1.0/ki, max(-1.0/ki, I));
        }
        double D = (P - P0)/dt;
        
        double dv = kp * P + ki * I + kd * D;
       
        result = min(255, max(0, (int)(result + dv)));
     
      }
    t0 = now;
    P0 = P;
  }
}



unsigned long m1_pulses_0=0;
unsigned long m2_pulses_0=0;


void loop() { 
  bool m1_ready = false;
  bool m2_ready = false;
  
  unsigned long now = millis(); 
  
  countPulses(Sensor1, now, sensor1State, m1_pulses);
  countPulses(Sensor2, now, sensor2State, m2_pulses);
 

  if (now > t1 + 1000) {
    Serial.print(driveState);
    Serial.print(": ( ");
    Serial.print(m1_pulses);
    Serial.print(" ) - ( ");
    Serial.print(m2_pulses);
    Serial.println(" )");
    t1 = now;
  }

  switch(driveState) {
    case 0:
      if (digitalRead(key) == LOW) {
        Serial.println("start!");
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
        Serial.print(driveState);
        Serial.println(": go forward");
        forwardM1(NORMAL_SPEED);
        forwardM2(NORMAL_SPEED);
        t0 = now;
        driveState = 2;
      }
      break;
    case 2: 
      if (m1_pulses > m1_pulses_0 + 80) {
        Serial.print(driveState);
        Serial.println(": stop m1");
        forwardM1(STOP_SPEED);  
        driveState=3;
      } else if (m2_pulses > m2_pulses_0 + 80) {
        Serial.print(driveState);
        Serial.println(": stop m2");
        forwardM2(STOP_SPEED);   
        driveState=4;
      }
      break;
    case 3: 
      if (m2_pulses > m2_pulses_0 + 80) {
        Serial.print(driveState);
        Serial.println(": stop m2");
        forwardM2(STOP_SPEED);   
        driveState=0;
        m1_pulses_0 = m1_pulses;
        m2_pulses_0 = m2_pulses;
      }
      break;  
    case 4: 
      if (m1_pulses > m1_pulses_0 + 80) {
        Serial.print(driveState);
        Serial.println(": stop m1");
        forwardM1(STOP_SPEED);   
        driveState=0;
        m1_pulses_0 = m1_pulses;
        m2_pulses_0 = m2_pulses;
      }
      break;    
    case 5: 
      break;      
    case 98: 
      if (digitalRead(key) == LOW) {
          Serial.println("stop");
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
  
//  updateController(now, sollWert, m1_currentSpeed, 0.2, 0.01, 0.05, m1_t0, m1_I, m1_P0, m1_controllVariable);


  
//  updateController(now, sollWert, m2_currentSpeed, 0.2, 0.01, 0.006, m2_t0, m2_I, m2_P0, m2_controllVariable);
//  run(m1, m1_controllVariable);   
//  run(m2, m2_controllVariable);   
   
//   if (now > t0 + 200) {
//        unsigned long currentCounter = pulseCounter;
//        currentSpeed = (currentCounter - oldCounter) ;
//        oldCounter = currentCounter;
////      data_now[i] = now;
////      data_ist[i] = sensor1istWertCPM;
////      data_stell[i] = m1_controllVariable;
//        t0 = now;
////      i++;
//    } 

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


