#include <PID_v1.h>

int Left_motor_back = 9; 
int Left_motor_go = 5; 
int Right_motor_go = 6; 
int Right_motor_back = 10; 
int Right_motor_en = 8; 
int Left_motor_en = 7; 
int Sensor1 = 13;
int Sensor2 = 12;
int m2 = 5; 
int m1 = 6; 

unsigned long sensor1StartTime = 0;
double sensor1istWertCPM = 0;
int sensor1State = 0;

unsigned long sensor2StartTime = 0;
double sensor2istWertCPM = 0;
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

double sollWert = 120;

PID m1PID(&sensor1istWertCPM, &m1_controllVariable, &sollWert,2,1,0.1, DIRECT);
PID m2PID(&sensor2istWertCPM, &m2_controllVariable, &sollWert,2,1,0.1, DIRECT);

void setup()
{
  //Initialize motor drive for output mode
  pinMode(Left_motor_go,OUTPUT);
  pinMode(Left_motor_back,OUTPUT);
  pinMode(Sensor1, INPUT); 

  Serial.begin(9600);
  while (! Serial);
   Serial.println("Start");
   digitalWrite(Right_motor_go,HIGH);// right motor go ahead
  digitalWrite(Right_motor_back,LOW);  
 
  m1PID.SetMode(AUTOMATIC);
  m1PID.SetSampleTime(10);
  m2PID.SetMode(AUTOMATIC);
  m2PID.SetSampleTime(10);
}

void run(int motor, int myspeed) // go ahead 
{
 
  analogWrite(motor, myspeed);
  
}


unsigned long t0 = 0;
unsigned long t1 = 0;

double calcIstWert(int sensor, unsigned long now, unsigned long& startTime, int& state, double& cpm) {
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
          startTime = now;
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
          cpm = 1000.0 * 60.0 / ((now - startTime) * 20.0);  
          startTime = now;
       }  
       break;      
   }
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







void loop() { 
  unsigned long now = millis(); 
  
  calcIstWert(Sensor1, now, sensor1StartTime, sensor1State, sensor1istWertCPM);
  calcIstWert(Sensor2, now, sensor2StartTime, sensor2State, sensor2istWertCPM);
  m1PID.Compute();
  m2PID.Compute();
  
  //updateController(now, sollWert, sensor1istWertCPM, 0.2, 0.01, 0.006, m1_t0, m1_I, m1_P0, m1_controllVariable);


  
  //updateController(now, 120, sensor1istWertCPM, 0.2, 0.01, 0.006, m2_t0, m2_I, m2_P0, m2_controllVariable);
  run(m1, m1_controllVariable);   
  run(m2, m2_controllVariable);   
   
   if (now > t0 + 1000) {
      Serial.print(sollWert);
      Serial.print(";   ");
      Serial.print(sensor1istWertCPM);
      Serial.print(";");
      Serial.print(m1_controllVariable);
      Serial.print(";      ");
      Serial.print(sensor2istWertCPM);
      Serial.print(";");
      Serial.print(m2_controllVariable);
      Serial.print(";");
      Serial.println("");
      
      t0 = now;
    } 

//  if (now > t1 + 10000) {
//    sollWert+=10;
//    t1 = now;
//  }
}


