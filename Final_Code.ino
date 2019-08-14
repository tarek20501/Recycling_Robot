#include <Servo.h>
//pins definitions
#define SLsensorD   A0
#define SRsensorD   A1
#define LsensorD    A2
#define RsensorD    2
#define IR          3

#define ServoPow 1

#define LmotorA     5
#define RmotorA     6
//#define S0        LOW
//#define S1        HIGH
#define S2          4
#define S3          7

#define photocell A3
#define halleffect A4
#define sensorOut   A5

#define Gled  11     
#define Bled  12     
#define Rled  8 
#define Yled  13 

//PID Parameters
float Kp=10,Ki=1,Kd=0;
int initial_motor_speed=235;
#define corr_speed 200
#define hard_speed 190
int turn;
#define max_speed 255
#define min_speed  0
int Lfor = 0;
int Rfor = 0;

//PID Variables
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0;
int sensor[4]={0, 0, 0, 0};

//color sensor calibration
#define COLOR_ERROR 0.22
#define BLUE_ERROR 0.15

#define W_RF 800
#define W_GF 900
#define W_BF 750

#define R_RF 1000
#define R_GF 2000
#define R_BF 1600

#define G_RF 1150
#define G_GF 1510
#define G_BF 1480

#define B_RF 1500
#define B_GF 1600
#define B_BF 1100


//Photo Sensor Variables
//frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;
// Stores the red. green and blue colors
int redColor = 0;
int greenColor = 0;
int blueColor = 0;

//Color Counters
unsigned int Rcount = 0;
unsigned int Gcount = 0;
unsigned int Bcount = 0;
unsigned int Kcount = 0;

//Mode of operation vairables
boolean swtch = 0;
//Black timer
boolean Bsw = 0;
unsigned long PtimerB = 0;
#define Binterval  1000
//White Timer
boolean Wsw = 0;
unsigned long PtimerW = 0;
#define Winterval  3000
//LED Timer
unsigned long PtimerLED = 0;
#define LEDinterval  1000

//Anti Wobble Variables
boolean Uwobble = 0; //Upcoming wobble warning
boolean Kwobble = 0; //it is time to kill the wobble switch
#define BreakInterval  200
unsigned long BreakTimer = 0;

//Sorting Sensors Variables
#define Nthres 530
#define Sthres 498
#define Pthres 230

#define DOWN 150
#define UP 20

#define LOAD 105

#define DISP1 137
#define DISP2 87
#define DISP3 39
  
#define SCAN1 87
#define SCAN2 37
#define SCAN3 0

char Cube[3]={'E','E','E'};
int dispenseCompartment[3] = {DISP1,DISP2,DISP3};
int scanCompartment[3] = {SCAN1,SCAN2,SCAN3};
Servo pushservo;
Servo rotservo;

//Dispose Variable
unsigned long IgnoreTimer = 0;
#define IgnoreInterval  500
boolean Ignore = 1;
//boolean Ignoresw = 0;
unsigned long FlickerTimer = 0;
#define FlickerInterval  500
boolean lock = 0;
boolean Stopsw = 0;
unsigned long StopTimer = 0;
#define StopInterval  0
boolean Scansw = 0;
unsigned long ScanTimer = 0;
#define ScanInterval 1000
boolean AntiVib = 1;

void setup() {
  //Serial.begin(9600);
  //IR Sensors
  pinMode(SLsensorD,INPUT);
  pinMode(LsensorD,INPUT);
  pinMode(RsensorD,INPUT);
  pinMode(SRsensorD,INPUT);
  //Color Sensor
  pinMode(sensorOut,INPUT);
  //pinMode(S0,OUTPUT);
  //pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);
  pinMode(S3,OUTPUT);
  //DC Motors
  pinMode(LmotorA,OUTPUT);
  pinMode(RmotorA,OUTPUT);
  //LEDs
  pinMode(Gled, OUTPUT);
  pinMode(Bled, OUTPUT);
  pinMode(Rled, OUTPUT);
  pinMode(Yled, OUTPUT);
  //Bin IR
  pinMode(IR,INPUT);
  //Sorting Sensors
  pinMode(photocell, INPUT);
  pinMode(halleffect, INPUT);

  pinMode(ServoPow,OUTPUT);
  
  // Setting color sensor frequency scaling to 2%
  //digitalWrite(S0,LOW);
  //digitalWrite(S1,HIGH);
  
  //Servo management
  digitalWrite(ServoPow,HIGH);
  
  rotservo.attach(9);
  rotservo.write(dispenseCompartment[1]);
  delay(700);
  rotservo.detach();
  
  pushservo.attach(10);
  pushservo.write(DOWN);
  delay(500); 
  pushservo.detach();

  rotservo.attach(9);
  rotservo.write(LOAD);
  delay(700);
  rotservo.detach();

  delay(6500);
  obscan();
  rotservo.attach(9);
  rotservo.write(dispenseCompartment[1]);
  delay(700);
  rotservo.detach();
  
  digitalWrite(ServoPow,LOW);
  
}

void loop() {
    read_sensor_values();
    
    Serial.print("\t");
    Serial.print(error);
    Serial.print("\t");
    Serial.print(digitalRead(IR));
    Serial.print("\t");
    calculate_pid();
    
    if(swtch){
      if( (Kwobble && (millis() - BreakTimer <= BreakInterval)) )
      stp();
      else{
      Kwobble = 0;
      rotservo.write(dispenseCompartment[1]);
      motor_control();
      }
    }
    
    //in range 0
    //out of range 1
    
    if(!Ignore && !digitalRead(IR)){
      IgnoreTimer = millis();
      }
    
    if( !Ignore && digitalRead(IR) && (millis()-IgnoreTimer>=IgnoreInterval)){ 
      Ignore = 1;
      //Ignoresw = 0;
      }
    
    if( Ignore && ( !digitalRead(IR) || (millis() - FlickerTimer <= FlickerInterval) || lock ) ){
      if (!digitalRead(IR)){
        FlickerTimer = millis();
        }
      if (!Stopsw){
        Stopsw = 1;
        lock = 1;
        StopTimer = millis();
        }
      if (millis() - StopTimer >= StopInterval){
          stp();
          swtch = 0;
          AntiVib = 0;
          //moveBackToLine ();
        if(!Scansw){
          Scansw = 1;
          ScanTimer = millis();
          }
        if(millis() - ScanTimer <= ScanInterval){
          ColorCount(WhatColor());
          }
        else{
          LEDup(ColorCount('L'));
          digitalWrite(ServoPow,HIGH);
          whatToDispense(ColorCount('L'));
          digitalWrite(ServoPow,LOW);
          //delay(500);
          LEDup('C');
          Ignore = 0;
          swtch = 1;
          lock = 0;
          ColorCount('D');
          Stopsw = 0;
          Scansw = 0;
          Wsw = 0;
          Bsw = 0;
          IgnoreTimer = millis();
          AntiVib = 1;
          }
        }
      }
      
    Serial.println();
}

void read_sensor_values()
{
  sensor[0]=digitalRead(SLsensorD);
  sensor[1]=digitalRead(LsensorD);
  sensor[2]=digitalRead(RsensorD);
  sensor[3]=digitalRead(SRsensorD);
  
  Serial.print(sensor[3]); Serial.print(" ");
  Serial.print(sensor[2]); Serial.print(" ");
  Serial.print(sensor[1]); Serial.print(" ");
  Serial.print(sensor[0]); Serial.print(" ");
  Serial.print("\t");
  if ((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)){ //if in the air turn off. Don't wase energy
      error=error;
      Wsw = 0;
      if (!Bsw){ //turnig on timer
        Bsw = 1;
        PtimerB = millis(); //setting begining time
        }
      if(millis() - PtimerB >= Binterval){
        stp();
        swtch = 0;
        }
      }
  else if( ((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)) || sensor[0]==1 ){
    if (AntiVib) swtch = 1;
    error=3;
    I = initial_motor_speed * error/3 - error * Kp;
    Bsw = 0;
    Wsw = 0;
  }
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)){
    if (AntiVib) swtch = 1;
    error=1;
    I = 0;
    Bsw = 0;
    Wsw = 0;
    if(Uwobble) KillWobble();
  }
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)){
    if (AntiVib) swtch = 1;
    error=0;
    I = 0;
    Bsw = 0;
    Wsw = 0;
  }
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0)){
    if (AntiVib) swtch = 1;
    error=-1;
    I = 0;
    Bsw = 0;
    Wsw = 0;
    if(Uwobble) KillWobble();
  }
  else if( ((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)) || sensor[3]==1 )
  {
    if (AntiVib) swtch = 1;
    error=-3;
    I = initial_motor_speed * error/3 - error * Kp;
    Bsw = 0;
    Wsw = 0;
  }
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)){ //if lost, find the black line
       if (AntiVib) swtch = 1;
       if(error==0) error=error;
       if(error==1) error=2;
       if(error==-1) error=-2;
       if(error==2 || error==-2){ 
        I = initial_motor_speed * error/2 - error * Kp;
        turn = corr_speed;
       }
       if(error==3 || error==-3){ 
        I = initial_motor_speed * error/3 - error * Kp;
        turn = hard_speed ;
        Uwobble = 1;
       }
       Bsw = 0;
       if (!Wsw){
          Wsw = 1;
          PtimerW = millis();
          }
       if (millis() - PtimerW >= Winterval){
          stp();
          swtch = 0;
          }
      }
  else error = error;
}

void calculate_pid()
{
    P = error;
    D = error - previous_error;
    
    PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    previous_error=error;
}

void motor_control()
{
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed - PID_value;
    int right_motor_speed = initial_motor_speed + PID_value;


    //limit hard turn speed to the initial speed
    if(left_motor_speed == 0) right_motor_speed = turn;
    if(right_motor_speed == 0) left_motor_speed = turn;

    //Forward compensation
    left_motor_speed -= Lfor;
    right_motor_speed += Rfor;
    
    // The motor speed should not exceed the max PWM value
    if (left_motor_speed > max_speed)   left_motor_speed = max_speed;
    if (left_motor_speed < min_speed && left_motor_speed != 0)   left_motor_speed = min_speed;
    if (right_motor_speed > max_speed)  right_motor_speed = max_speed;
    if (right_motor_speed < min_speed && right_motor_speed != 0)  right_motor_speed = min_speed;
    
    
    Serial.print(left_motor_speed); Serial.print(" ");
    Serial.print(right_motor_speed); Serial.print(" ");
    Serial.print("\t");
    
    analogWrite(LmotorA,left_motor_speed);  //Left Motor Speed
    analogWrite(RmotorA,right_motor_speed); //Right Motor Speed
}

void stp(){
  analogWrite(LmotorA,0);
  analogWrite(RmotorA,0);
  }

char WhatColor(){
   
  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  delay(1);
  
  // Reading the output frequency
  int redFrequency = pulseIn(sensorOut, LOW);
  // Printing the RED (R) value
  Serial.print(" Rf = ");
  Serial.print(redFrequency);
  Serial.print("\t");

  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  delay(1);
  
  // Reading the output frequency
  int greenFrequency = pulseIn(sensorOut, LOW);

  // Printing the GREEN (G) value  
  Serial.print(" Gf = ");
  Serial.print(greenFrequency);
  Serial.print("\t");
 
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  delay(1);  
   // Reading the output frequency
  int blueFrequency = pulseIn(sensorOut, LOW);
  
  // Printing the BLUE (B) value 
  Serial.print(" Bf = ");
  Serial.print(blueFrequency);
  Serial.print("\t");
  
 
  // Checks the current detected color and prints
  // a message in the serial monitor
  if
(    redFrequency  <(W_RF*(1+COLOR_ERROR))
  && greenFrequency<(W_GF*(1+COLOR_ERROR))
  && blueFrequency <(W_BF*(1+COLOR_ERROR)) 
)
  {
      Serial.print(" - White detected!");
      return 'K'; 
   } 
 else if 
 (
    (redFrequency  < (G_RF*(1+COLOR_ERROR)) && redFrequency  >(G_RF*(1-COLOR_ERROR))) 
 && (greenFrequency< (G_GF*(1+COLOR_ERROR)) && greenFrequency>(G_GF*(1-COLOR_ERROR)))
 && (blueFrequency < (G_BF*(1+COLOR_ERROR)) && blueFrequency >(G_BF*(1-COLOR_ERROR)))
 )
 {
    Serial.print("- GREEN detected!");
    return 'G';
  }
  else if 
  (
    (redFrequency  < (B_RF*(1+COLOR_ERROR)) && redFrequency  >(B_RF*(1-COLOR_ERROR))) 
 && (greenFrequency< (B_GF*(1+COLOR_ERROR)) && greenFrequency>(B_GF*(1-COLOR_ERROR)))
 && (blueFrequency < (B_BF*(1+COLOR_ERROR+BLUE_ERROR)) && blueFrequency >(B_BF*(1-COLOR_ERROR-BLUE_ERROR)))
  )
  {
    Serial.print("- BLUE detected!");
    return 'B';
  }
 
  else if 
 (
    (redFrequency  < (R_RF*(1+COLOR_ERROR)) && redFrequency  >(R_RF*(1-COLOR_ERROR))) 
 && (greenFrequency< (R_GF*(1+COLOR_ERROR)) && greenFrequency>(R_GF*(1-COLOR_ERROR)))
 && (blueFrequency < (R_BF*(1+COLOR_ERROR)) && blueFrequency >(R_BF*(1-COLOR_ERROR)))
 )
  {
    Serial.print("- RED detected!");
    return 'R';
  }
  else
   Serial.print("- Nothing detected!");
   return 'N';
}

void LEDup(char color){
  if (color == 'R'){
  digitalWrite(Rled, HIGH);
    }
  if (color == 'G'){
  digitalWrite(Gled, HIGH);
  }
  if (color == 'B'){
  digitalWrite(Bled, HIGH);
  }

  if (color == 'K'){
  digitalWrite(Yled, HIGH);
  }
  
  if (color == 'C'){
  digitalWrite(Rled, LOW);
  digitalWrite(Gled, LOW);
  digitalWrite(Bled, LOW);
  digitalWrite(Yled, LOW);
  }
}

char ColorCount(char color) {
  unsigned int  mx;
  char mxc;
  if(color == 'R') Rcount++;
  if(color == 'G') Gcount++;
  if(color == 'B') Bcount++;
  if(color == 'K') Kcount++;
  if(color == 'D'){
    Rcount = 0;
    Gcount = 0;
    Bcount = 0;
    Kcount = 0;
    }
  if(color == 'L'){
    if(Rcount > Gcount) { mx = Rcount; mxc = 'R'; } else { mx = Gcount; mxc = 'G'; }
    if(mx < Bcount) { mx = Bcount; mxc = 'B'; }
    if(mx < Kcount) { mx = Kcount; mxc = 'K'; }
    return mxc;
    }
  }

void KillWobble() {
  Uwobble = 0;
  Kwobble = 1;
  BreakTimer = millis();
  }
//if(Uwobble) KillWobble();

void obscan() 
{
  while(Cube[0] == Cube[1] || Cube[0] == Cube[2] || Cube[1] == Cube[2])
  {
  reading(0);
  reading(1);
  reading(2);
  }
}


void reading(int index){
  LEDup('K');
  rotservo.attach(9);
  rotservo.write(scanCompartment[index]);
  delay(700);
  rotservo.detach();
  int count = 0;
  int magnet = 0;
  int plastic = 0;
  int glass = 0;
  

 for(count=0;count<100;count++){
  int photo = analogRead(photocell);
  int hall = analogRead(halleffect);
  Serial.print("photo = ");
  Serial.print(photo);
  Serial.print("  ");
  Serial.print("hall = ");
  Serial.println(hall);
  if (hall >= Nthres || hall<= Sthres){
    magnet++;
    }
  else if(photo<Pthres)
  {
    glass++;
  }
  else
  {
    plastic++;
  }
  delay(3);
}

   Serial.print(magnet);
   Serial.print(" ");
   Serial.print(glass);
   Serial.print(" ");
   Serial.print(plastic);
   Serial.println(" ");   

 if(magnet>plastic && magnet>glass){
  Cube[index] = 'M';
  LEDup('G');
  delay(500);
  LEDup('C');
 }
 if(plastic>magnet && plastic>glass){
  Cube[index] = 'P';
  LEDup('R');
  delay(500);
  LEDup('C');
 }
 if(glass>magnet && glass>plastic){
  Cube[index] = 'G';
  LEDup('B');
  delay(500);
  LEDup('C');  
 }
 
  
}


void dispense(int index)
{
      rotservo.attach(9);
      rotservo.write(dispenseCompartment[index]);
      Cube[index]='E';
      delay(700);
      rotservo.detach();
      pushservo.attach(10);
      pushservo.write(UP);
      delay(500);
      pushservo.write(DOWN);
      delay(500);
      pushservo.detach();
}

void whatToDispense ( char color)
{
  int i;
  for(i=0;i<3;i++)
  {
    if (color == 'R' && Cube[i] == 'P' )
    {
    Serial.print(color);
    Serial.print(Cube[i]); Serial.print("\t");
    dispense(i);
    }
  }
  
  for(i=0;i<3;i++)
  {
    if (color == 'B' && Cube[i] == 'G' )
    {
    Serial.print(color);
    Serial.print(Cube[i]); Serial.print("\t");
    dispense(i);
    }
  }
  
  for(i=0;i<3;i++)
  {
    if (color == 'G' && Cube[i] == 'M' )
     {
    Serial.print(color);
    Serial.print(Cube[i]); Serial.print("\t");
    dispense(i);
    }
  }
  rotservo.attach(9);
  rotservo.write(dispenseCompartment[1]);
  delay(500);
  rotservo.detach();
}

void moveBackToLine () 
{
  read_sensor_values();
  while (error>1 || error<-1)
  {
    read_sensor_values();
    if(error > 1 )
    {
      analogWrite(RmotorA,200);
      analogWrite(LmotorA,0);
    }
    else if (error < -1 )  
    {
      analogWrite(RmotorA,0);
      analogWrite(LmotorA,200);
    }
    else stp();
  }
 stp();
}





