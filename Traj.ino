#include <Stepper.h>

//#include <D:\#Bachelor\Arduino code\ArduinoTry\InverseKinematics\PrintSerial.ino>
// Moves all stepper motor simultaneously with acceleration and deceleration. 

#include <AccelStepper.h>
// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, 2, 5);;
AccelStepper stepper2(AccelStepper::DRIVER, 3, 6);
AccelStepper stepper3(AccelStepper::DRIVER, 4, 7);

//int initialMove = 2400; //4800 is max range of motion
//int MaxAcceleration = 75000.0;
//int MaxSpeed = 20000.0;
int initialMove = 24; //4800 is max range of motion
int MaxAcceleration = 75.0;
int MaxSpeed = 200.0;
int j = 0;

float xPos = 0;  // x axis position
float yPos = 0; // y axis position
float zPos = -180; // z axis position, negative is up, positive is down. Delta bots are usually used up side down...

//float xPos = 78.231; 
//float yPos = -78.231; 
//float zPos = -377.824; 

float t1;  //stepper angle t for 'theta', 1 for stepper 1
float t2;
float t3;

int result;
int data = 0;
int serialTeller=0;

#define MAXANGLE 98.73 //maximum angle allowed by the Steppers
#define MINANGLE -42.92   //minimum angle alloweb by the Steppers



// defines pins numbers
//const int stepX = 2;
//const int dirX  = 5;
//const int stepY = 3;
//const int dirY  = 6;
//const int stepZ = 4;
//const int dirZ  = 7;
//const int enPin = 8;

////////////////////#############################################////////////////////
// print some values in the serial  
 void  printSerial(){
  Serial.print(result == 0 ? "ok" : "no");
  char buf[10];
  dtostrf(xPos, 4, 0, buf);
  Serial.print(" X");
  Serial.print(buf);
  dtostrf(yPos, 4, 0, buf);
  Serial.print(" Y");
  Serial.print(buf);
  dtostrf(zPos, 4, 0, buf);
  Serial.print(" Z");
  Serial.print(buf);

  dtostrf(t1, 6, 2, buf);
  Serial.print(" T1");
  Serial.print(buf);
  dtostrf(t2, 6, 2, buf);
  Serial.print(" T2");
  Serial.print(buf);
  dtostrf(t3, 6, 2, buf);
  Serial.print(" T3");
  Serial.print(buf);

  Serial.println("");
  }
  
void setup() {


// initialize the serial port:
Serial.begin(9600);
  stepper1.setMaxSpeed(MaxSpeed);//max 20000.0 with main linkage only
  stepper1.setAcceleration(MaxAcceleration);//max 90000.0 with main linkage only

  stepper2.setMaxSpeed(MaxSpeed);
  stepper2.setAcceleration(MaxAcceleration);

  stepper3.setMaxSpeed(MaxSpeed);//lower is slower... higher number is faster...
  stepper3.setAcceleration(MaxAcceleration);//lower is slower ...higher number is faster

  // Sets the two pins as Outputs

//  pinMode(stepX,OUTPUT);
//  pinMode(dirX,OUTPUT);
//  pinMode(stepY,OUTPUT);
//  pinMode(dirY,OUTPUT);
//  pinMode(stepZ,OUTPUT);
//  pinMode(dirZ,OUTPUT);
//  pinMode(enPin,OUTPUT);
//
//  digitalWrite(enPin,LOW);
//  digitalWrite(dirX,HIGH);
//  digitalWrite(dirY,HIGH);
//  digitalWrite(dirZ,HIGH);

}
  void GoToXYZ(int x, int y, int z) {

  stepper1.moveTo(x);
  stepper2.moveTo(y);
  stepper3.moveTo(z);

  stepper1.run();
  stepper2.run();
  stepper3.run();

  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  
}

void PickUpXYZ(int x, int y, int z) {

  int objectHeight = 8;//This equals inches
  
  stepper1.moveTo(x-objectHeight);
  stepper2.moveTo(y-objectHeight);
  stepper3.moveTo(z-objectHeight);

  stepper1.run();
  stepper2.run();
  stepper3.run();

  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }


  stepper1.moveTo(x);
  stepper2.moveTo(y);
  stepper3.moveTo(z);

  stepper1.run();
  stepper2.run();
  stepper3.run();

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }

  stepper1.moveTo(x-objectHeight);
  stepper2.moveTo(y-objectHeight);
  stepper3.moveTo(z-objectHeight);

  stepper1.run();
  stepper2.run();
  stepper3.run();

  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  } 
}


void loop(){
//int variable1 = int(random(100));
//Serial.print(variable1);
  char xxPos[4];  // x position taken from processing
  char yyPos[4];  // y position taken from processing
  char zzPos[4];  // z position taken from processing
  char tempo;     // temp variable

  if (Serial.available() > 12) {
    tempo=Serial.read();  //read from serial
    int conta=0;
    while (tempo != 'X')  // wait for the X position
    { 
      tempo=Serial.read();
       
      }
  
    if (Serial.available() >= 11) {
  
      xxPos[0]=Serial.read();  //read X byte
      xxPos[1]=Serial.read();  //read X byte
      xxPos[2]=Serial.read();  //read X byte
      xxPos[3]=Serial.read();  //read X byte
      
      xPos=atoi(xxPos); //transfor bytes in integer
    
    
     tempo=Serial.read();  //read Y char
     
      yyPos[0]=Serial.read(); //read Y byte
      yyPos[1]=Serial.read(); //read Y byte
      yyPos[2]=Serial.read(); //read Y byte
      yyPos[3]=Serial.read(); //read Y byte
     
      yPos=atoi(yyPos); //transform bytes in integer
    
      tempo=Serial.read(); //read Z Char
     
      zzPos[0]=Serial.read(); //read Z byte
      zzPos[1]=Serial.read(); //read Z byte
      zzPos[2]=Serial.read(); //read Z byte
      zzPos[3]=Serial.read(); //read Z byte
     
      zPos=atoi(zzPos); //transform bytes in integer
    
}
  } 



//  result = delta_calcInverse(xPos, yPos, zPos, t1, t2, t3);


//   delay(6000);
    
//  GoToXYZ(xxPos[4],yyPos[4], zzPos[4]);
//  GoToXYZ(xxPos[0],yyPos[0], zzPos[0]);
//  GoToXYZ(xxPos[1],yyPos[1], zzPos[1]);
//  GoToXYZ(xxPos[2],yyPos[2], zzPos[2]);
//  GoToXYZ(xxPos[3],yyPos[3], zzPos[3]);
GoToXYZ(xPos,yPos, zPos);

//  delay(2000);
//  
//  PickUpXYZ(xxPos[4],yyPos[4], zzPos[4]);
//  PickUpXYZ(xxPos[0],yyPos[0], zzPos[0]);
//  PickUpXYZ(xxPos[1],yyPos[1], zzPos[1]);
//  PickUpXYZ(xxPos[2],yyPos[2], zzPos[2]);
//  PickUpXYZ(xxPos[3],yyPos[3], zzPos[3]);
//  GoToXYZ(xxPos[4],yyPos[4], zzPos[4]);
//  
//  delay(10000);


}


  //  stepperWriteFloat(&stepX, t1); //write angles value into the servo
//  stepperWriteFloat(&stepY, t2+10); //write angles value into the servo - the 10 value is due to not perfect alignement of the servo
//  stepperWriteFloat(&stepZ, t3+5); //write angles value into the servo - the 5 value is due to not perfect alignement of the servo
