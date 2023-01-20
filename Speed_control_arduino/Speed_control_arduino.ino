#include <util/atomic.h>

#define ENCA 2
#define ENCB 3
#define R 5
#define L 6
float pre_e ;
long prevT= 0;
int posPrev = 0;
volatile int pos_i = 0;
int speedd;
float vt;
float v1Filt = 0;
float v1Filtac = 0;
float v1Prev = 0;
float v1Prevac = 0;
float eintergral;
void setup(){
  Serial.begin(9600);
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
  pinMode(R,OUTPUT);
  pinMode(L,OUTPUT);
  attachInterrupt(0,readEncoder,RISING);
  }

 void loop(){

  // read the position in an atomic block 
  // to avoid potetial misreads
  int pos = 0;

    pos = pos_i;
  
 // compute velocity 
  long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  float veloc = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  float v1 = veloc/650*60;
  float v1ac = veloc/65*60;
  //  Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Filtac = 0.854*v1Filtac + 0.0728*v1ac + 0.0728*v1Prevac;
  v1Prev = v1;
  v1Prevac = v1ac;

  // Set desired speed
  float vt = 50;

  // PI function
  float kp = 1;
  float ki = 8;
  float e = vt-v1Filt;
  eintergral += e*deltaT;
  float u = kp*e+ki*eintergral;

  int dir = -1;
  if(u<0){
    dir = 1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir,pwr,R,L);
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filtac);
  Serial.print(" ");
  Serial.println(v1Filt);
  
 }

void setMotor(int dir , int pwmVal,int in1,int in2){
  if (dir == 1){
    analogWrite(in1,pwmVal);
    analogWrite(in2,0);
  }
  if (dir == -1){
    analogWrite(in1,0);
    analogWrite(in2,pwmVal);
  }
  else{
    analogWrite(in1,0);
    analogWrite(in2,0);
  }
}

void readEncoder(){
  if (digitalRead(ENCB) == 0){
    pos_i--;
  }else{pos_i++;}
}
