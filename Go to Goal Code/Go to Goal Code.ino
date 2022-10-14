#include <math.h>

int optokiri          =  2;
int optokanan          = 3;
int Motorkiri_A       = 8; 
int Motorkiri_B     =  9;
int Motorkanan_B      =  11;
int Motorkanan_A      =  12;
int a = 0;
float duration;
float distance;
int jarak = 50;
int servo          =  7;
const int trig  =  A5;
const int echo  =  A4;
unsigned long count = 0;
unsigned long countkanan = 0;
  
float pi = 3.14;                  
float R  = 3.25;                  
int N = 12;                       
int L = 12;                       



//Servo
  int Period = 5000;
  int first = 700;
  int finish = 2500;
  int increment = 10;
  int current = 0;
  int finish_old;
  int first_old;

  
//PID
int maxspd = 255;                 
int Vo= 80;                       
float V_kiri;                     
float V_kanan;                    
float error   = 0;                
float error2  = 0;                
float error_P = 0;                
float error_I = 0;                
float error_D = 0;                
float P = 0;                      
float I = 0;                      
float D = 0;                      
float w = 0;                      
float Kp = 6 ;             
float Ki = 1/1000 ;               
float Kd = 7 ;              

  
  
//Go to goal
  float count_new = 0;             
  float countkanan_new = 0;        
  float count_old = 0;             
  float countkanan_old = 0;        
  float D_countkanan = 0;          
  float D_count = 0;               
  float DR = 0;                     
  float DL = 0;                     
  float DC = 0;                     
  float X_new = 0;                  
  float Y_new = 0;                  
  float tetha_new = 0;              
  float X_old = 0;                  
  float Y_old = 0;                  
  float tetha_old = 0;              
  float goalX = 0;                  
  float goalY = 0;                  
  float dX = 0;                     
  float dY = 0;                     
  float goal_tetha  = 0;            
  float error_tetha = 0;            
  float dt = 1/10;                  
  float range = 8;                  
  int   q = 1;                      
  int   cm = 0;
  int pwm;
  int angle;


  
//Matrix  
  int  k,l;
  float d1 = 0;
  float d2 = 0;
  float d3 = 0;
  float d4 = 0;
  float d5 = 0;
  float t1 = 0;
  float t2 = 0;
  float t3 = 0;
  float t4 = 0;
  float t5 = 0;
  
  float M1_1, M1_2, M1_3;
  float M2_1, M2_2, M2_3;
  float M3_1, M3_2, M3_3;
  float M4_1, M4_2, M4_3;
  float M5_1, M5_2, M5_3;
  
  float Mx1_1, Mx1_2, Mx1_3;
  float Mx2_1, Mx2_2, Mx2_3;
  float Mx3_1, Mx3_2, Mx3_3;
  float Mx4_1, Mx4_2, Mx4_3;
  float Mx5_1, Mx5_2, Mx5_3;
  
  float J_X, J_Y, J_T;

void setup() {
  
  Serial.begin(9600);
  attachInterrupt(optokiri, encoderL, FALLING);
  attachInterrupt(optokanan, encoderR, FALLING);
  pinMode(Motorkiri_A, OUTPUT);
  pinMode(Motorkiri_B, OUTPUT);
  pinMode(Motorkanan_A, OUTPUT);
  pinMode(Motorkanan_B, OUTPUT);
  pinMode(servo, OUTPUT);
}


void loop() {
  
  geserkiri();
  geserkanan();
  cekobs();
  
}


void cekobs() {
  
  if ((d1 < jarak) || (d2 < jarak) || (d3 < jarak) || (d4 < jarak) || (d5 < jarak)){
    hindari();
  }
  else {
    execute_go_to_goal();
  }
  
}


void hindari() {
  Kp = 6;
  Kd = 7;
  goalX = J_X/5; goalY = J_Y/5;

   for(k=0; k<300; k++) {    
    countkanan_new = countkanan;
    count_new = count;
    
    go_to_goal();
    
    if (d1 < d2) { V_kanan = V_kanan*1.2; }
    else if (d1 > d2) {V_kanan = V_kanan*1.3; }
    if (d5 < d4) { V_kiri = V_kiri*1.2; }
    else if (d5 > d4) {V_kiri = V_kiri*1.3; }
    
    if (d2 < 15 && d3 < 15 && d1 < 15) {V_kanan = 70; V_kiri = 5; }
    if (d4 < 15 && d3 < 15 && d5 < 15) {V_kiri = 70; V_kanan = 5;}
    absolute_V();
    execute_motor();
    delay (dt*1000);    
  }
  
  stopped();
  delay (500);
}


void execute_go_to_goal() {

  goalX = 180; goalY = 50;
  
  for (l=0; l<600; l++) {
    
    countkanan_new = countkanan;
    count_new = count;
    
    go_to_goal();
    absolute_V();
    execute_motor();
    delay (dt*1000);
    
    while ((X_new < (goalX + range)) && (X_new > (goalX - range)) && (Y_new < (goalY + range)) && (Y_new > (goalY - range))) {
    
      stopped();
      for (a=0; a<=100; a++){
      delay (1000);
      a=0;
      }
      
  }
  }
  stopped();
  delay(500);
}



void go_to_goal () {
  
  estimate_position();
  
  dX = goalX - X_new;
  dY = goalY - Y_new;
  
  goal_tetha = atan2 (dY,dX);
  error_tetha = goal_tetha - tetha_new;
  error_tetha = atan2 (sin(error_tetha),cos(error_tetha));
  
  error_P = error_tetha ;
  P = Kp * error_P ;
  error_I = error_tetha + error_I;
  I = Ki * error_I ;
  error_D = error_tetha - error2;
  D = Kd * error_D ;
  
  error2 = error_tetha ;
  
  w = P + I + D ;
  
  V_kanan = ((2*Vo + w*L)/(2*R));
  V_kiri  = ((2*Vo - w*L)/(2*R));
  
}


void estimate_position () {
  
  D_countkanan = countkanan_new - countkanan_old;
  D_count = count_new - count_old;
  
  DR = 2* pi * R * D_countkanan / N ;
  DL = 2* pi * R * D_count / N ;
  DC = (DR + DL)/2 ;
  
  X_new = X_old + DC * cos (tetha_old);
  Y_new = Y_old + DC * sin (tetha_old);
  tetha_new = tetha_old + ((DR - DL) / L);
  tetha_new = atan2(sin(tetha_new),cos(tetha_new));
  
  countkanan_old = countkanan_new;
  count_old = count_new;
  X_old = X_new;
  Y_old = Y_new;
  tetha_old = tetha_new;
  
}


void absolute_V () {
  
  if (V_kiri>maxspd) {
    V_kiri=maxspd; 
  }
  else if (V_kiri<-maxspd) {
    V_kiri=-maxspd; 
  }
  if (V_kanan>maxspd) {
    V_kanan=maxspd; 
  }
  else if (V_kanan<-maxspd) {
    V_kanan=-maxspd; 
  }
  
}


void execute_motor () {

  if ((V_kiri > 0)&&(V_kanan >0))  {
    forward();
  }
  else if((V_kiri < 0)&&(V_kanan < 0)) {
    V_kiri=0-V_kiri;
    V_kanan=0-V_kanan;
    backward();
  }
  else if((V_kiri > 0)&&(V_kanan < 0)) {
    V_kanan=0-V_kanan;
    turn_right();
  }
  else if((V_kiri < 0)&&(V_kanan > 0)) {
    V_kiri=0-V_kiri;
    turn_left();
  }
}



void stopped() {
  
  analogWrite  (Motorkiri_A  ,  0      );
  analogWrite  (Motorkiri_B,  0      );
  analogWrite  (Motorkanan_A ,  0      );
  analogWrite  (Motorkanan_B ,  0      );
}

void forward() {
  
  V_kiri  = map(V_kiri , 0,100,0,255);
  V_kanan = map(V_kanan, 0,100,0,255);
  
  analogWrite  (Motorkiri_A  ,  V_kiri );
  digitalWrite  (Motorkiri_B,  LOW      );
  analogWrite  (Motorkanan_A ,  V_kanan);
  digitalWrite  (Motorkanan_B ,  LOW      );
}

void backward() {
  
  V_kiri  = map(V_kiri , 0,100,0,255);
  V_kanan = map(V_kanan, 0,100,0,255);
  
  digitalWrite  (Motorkiri_A  ,  LOW      );
  analogWrite  (Motorkiri_B,  V_kiri );
  digitalWrite  (Motorkanan_A ,  LOW      );
  analogWrite  (Motorkanan_B ,  V_kanan);
}

void turn_right() {
  
  V_kiri  = map(V_kiri , 0,100,0,255);
  V_kanan = map(V_kanan, 0,100,0,255);
  
  analogWrite  (Motorkiri_A  ,  V_kiri );
  digitalWrite  (Motorkiri_B,  LOW      );
  digitalWrite  (Motorkanan_A ,  LOW      );
  analogWrite  (Motorkanan_B ,  V_kanan);
}

void turn_left() {
  
  V_kiri  = map(V_kiri , 0,100,0,255);
  V_kanan = map(V_kanan, 0,100,0,255);
  
  digitalWrite  (Motorkiri_A  ,  LOW      );
  analogWrite  (Motorkiri_B,  V_kiri );
  analogWrite  (Motorkanan_A ,  V_kanan);
  digitalWrite  (Motorkanan_B ,  LOW      );
}


void ultra()
{
   long duration, inches;

  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);

  pinMode(echo, INPUT);
  duration = pulseIn(echo, HIGH);
  cm = microsecondsToCentimeters(duration);
  distance = cm;
  Serial.println(cm);
  if (distance > 100) {
    distance = 100;
  }

  delay(50);
  
}



void servo_to_right(){
   for(current = first_old; current >first; current-=increment){
         digitalWrite(servo, HIGH);
         delayMicroseconds(current);
         digitalWrite(servo, LOW);
         delayMicroseconds(2000);
    }
}
void servo_to_left(){
  for(current = finish_old; current <finish; current+=increment){
         digitalWrite(servo, HIGH);
         delayMicroseconds(current);
         digitalWrite(servo, LOW);
         delayMicroseconds(25000-current);
    }
  finish_old=finish;
}


void geserkiri(){
  ultra();
  d1= distance;
  t1= -1.57079;
  m1();

  finish_old=first;
  finish = 1250 ;
  servo_to_left();
  delay (200);
  ultra();
  d2= distance;
  t2= -0.78539;
  m2();

  finish = 1600;
  servo_to_left();
  delay (200);
  ultra();
  d3= distance;
  t3= 0;
  m3();

  finish = 2050;
  servo_to_left();
  delay (200);
  ultra();
  d4= distance;
  t4= 0.78539;
  m4();
 
  finish = 2500;
  servo_to_left();
  delay (200);
  ultra();
  d5= distance;
  t5= 1.57079;
  m5();

 jumlahmatrix();
}

void geserkanan(){
  
  first_old = finish;
  first = 700;
  servo_to_right();
  delay (200);

}

void m1() {
  
  M1_1 = cos(t1)*d1 - sin(t1)*0 + 0*1;
  M1_2 = sin(t1)*d1 + cos(t1)*0 + 0*0;
  M1_3 = 0*d1 + 0*0 + 1*1;
  
  Mx1_1 = cos(tetha_new)*M1_1 - sin(tetha_new)*M1_2 + X_new*M1_3;
  Mx1_2 = sin(tetha_new)*M1_1 + cos(tetha_new)*M1_2 + Y_new*M1_3;
  Mx1_3 = 0*M1_1 + 0*M1_2 + 1*M1_3;
  
}

void m2() {
  
  M2_1 = cos(t2)*d2 - sin(t2)*0 + 0*1;
  M2_2 = sin(t2)*d2 + cos(t2)*0 + 0*0;
  M2_3 = 0*d2 + 0*0 + 1*1;
  
  Mx2_1 = cos(tetha_new)*M2_1 - sin(tetha_new)*M2_2 + X_new*M2_3;
  Mx2_2 = sin(tetha_new)*M2_1 + cos(tetha_new)*M2_2 + Y_new*M2_3;
  Mx2_3 = 0*M2_1 + 0*M2_2 + 1*M2_3;
  
}

void m3() {
  
  M3_1 = cos(t3)*d3 - sin(t3)*0 + 0*1;
  M3_2 = sin(t3)*d3 + cos(t3)*0 + 0*0;
  M3_3 = 0*d3 + 0*0 + 1*1;
  
  Mx3_1 = cos(tetha_new)*M3_1 - sin(tetha_new)*M3_2 + X_new*M3_3;
  Mx3_2 = sin(tetha_new)*M3_1 + cos(tetha_new)*M3_2 + Y_new*M3_3;
  Mx3_3 = 0*M3_1 + 0*M3_2 + 1*M3_3;
  
}

void m4() {
  
  M4_1 = cos(t4)*d4 - sin(t4)*0 + 0*1;
  M4_2 = sin(t4)*d4 + cos(t4)*0 + 0*0;
  M4_3 = 0*d4 + 0*0 + 1*1;
  
  Mx4_1 = cos(tetha_new)*M4_1 - sin(tetha_new)*M4_2 + X_new*M4_3;
  Mx4_2 = sin(tetha_new)*M4_1 + cos(tetha_new)*M4_2 + Y_new*M4_3;
  Mx4_3 = 0*M4_1 + 0*M4_2 + 1*M4_3;
  
}

void m5() {
  
  M5_1 = cos(t5)*d5 - sin(t5)*0 + 0*1;
  M5_2 = sin(t5)*d5 + cos(t5)*0 + 0*0;
  M5_3 = 0*d5 + 0*0 + 1*1;
  
  Mx5_1 = cos(tetha_new)*M5_1 - sin(tetha_new)*M5_2 + X_new*M5_3;
  Mx5_2 = sin(tetha_new)*M5_1 + cos(tetha_new)*M5_2 + Y_new*M5_3;
  Mx5_3 = 0*M5_1 + 0*M5_2 + 1*M5_3;
  
}

void jumlahmatrix() {
  
  J_X = Mx1_1 + Mx2_1 + Mx3_1 + Mx4_1 + Mx5_1;
  J_Y = Mx1_2 + Mx2_2 + Mx3_2 + Mx4_2 + Mx5_2;
  
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The trig travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 26 / 2;
}


void encoderL() {

   count++;

}

void encoderR() {
  
   countkanan++;
   
}
