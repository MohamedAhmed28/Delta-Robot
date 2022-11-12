import processing.serial.*;

Serial myPort;    // The serial port:

int XXX = 0; // x zero
int XXXlst; 
int YYY = 0; // y zero
int YYYlst;
int ZZZ = -180; // z zero (the robot is upsidedown)
int ZZZlst;
int serialBegin = 255;


void setup() {
  size(600,600);

  // open serial port
  //myPort = new Serial(this, Serial.list()[1], 9600);
  myPort = new Serial(this, "COM4", 9600);
  frameRate(100);
  noCursor();
}

void draw() {

  background(255);

  //triangle draw
  triangle(width/2, height, 0, 200, width, 200);

  strokeWeight(3);

  //line draw
  line(300,200,mouseX,mouseY);
  line(150,400,mouseX,mouseY);
  line(450,400,mouseX,mouseY);

  // X and y value are scaled in order to fit the robot space work 
  XXXlst = XXX;
  XXX=round(float((mouseX-300))*2/5);
  YYYlst = YYY;
  YYY=round(float((mouseY-300))*2/5);

  // if mouse is pressed the Z value change
  ZZZlst = ZZZ;
  if (mousePressed && (mouseButton == LEFT)) {
    ZZZ -= 1;
  }
  if (mousePressed && (mouseButton == RIGHT)) {
    ZZZ += 1;
  }

  String xPos;
  String yPos;
  String zPos;


  //write X value in serial port
  xPos="X";
  if(XXX != XXXlst){

  if (XXX > 0) 
    xPos += '+';
  else 
    xPos += '-';
  if (abs(XXX) < 100)
    xPos += '0';
  if (abs(XXX) < 10)
    xPos += '0';
  xPos += abs(XXX);
  }
  
  // write Y value in serial port
  yPos="Y";
  if(YYY != YYYlst){
  
  if (YYY > 0) 
    yPos += '+';
  else 
    yPos += '-';
  if (abs(YYY) < 100)
    yPos += '0';
  if (abs(YYY) < 10)
    yPos += '0';
  yPos += abs(YYY);
  }
  // write Z value in serial port
  zPos="Z";
  if(ZZZ != ZZZlst){
  
    if (ZZZ > 0) 
    zPos += '+';
  else 
    zPos += '-';
  if (abs(ZZZ) < 100)
    zPos += '0';
  if (abs(ZZZ) < 10)
    zPos += '0';
  zPos += abs(ZZZ);
  }
  if(XXX != XXXlst || YYY != YYYlst || ZZZ != ZZZlst ){
  // write x,y,z in the serial port
  myPort.write(xPos);
  myPort.write(yPos);
  myPort.write(yPos);
   // write x,y,z in the monitor
   println(xPos);
   println(yPos);
   println(zPos);
  }
//    if( abs( xPos - lastxPos) > 1){
//   myPort.write(xPos);
//   lastxPos = xPos;
//   println(xPos);
//}
//    if( abs( yPos - lastyPos) > 1){
//   myPort.write(yPos);
//   lastyPos = yPos;
//   println(yPos);
//}
//     if( abs( yPos - lastzPos) > 1){
//   myPort.write(yPos);
//   lastzPos = yPos;
//   println(zPos);
//}
  
  
}
