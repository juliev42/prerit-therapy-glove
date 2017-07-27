#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Mouse.h>

//right hand optimized! 

#define LPF_ALPHA .2//alpha value for the LP Filter 
//#define numMPUMeasures 6; 
#define button1 4 //will map left click to opponance with pointer finger 
#define button2 5 //will map right click to opponance with middle finger 

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
int vx, vy;

int buttonState1 = 0; 
int buttonState2 = 0; 

float MPUState_gyroZ;
float MPUState_gyroY;

void setup() {

  Serial.begin(9600);
  Wire.begin();
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  mpu.initialize();
  if (!mpu.testConnection()) { while (1); }

//MPU offsets 
   //mpu.setXGyroOffset(-200);
    //mpu.setYGyroOffset(76);
    //mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //accelerometer then gyroscope data 

//  vx = (gx+15)/150;  
//  vy = -(gz-100)/150;

  Serial.print("gx = ");
  Serial.print(gx);
   Serial.print(" | gy = ");
  Serial.print(gy);
  Serial.print(" | gz = ");
  Serial.print(gz);

  
  Serial.print("        | X = ");
  Serial.print(vx);
  Serial.print(" | Y = ");
  Serial.println(vy);
//
//
//  Serial.print("ax = ");
//  Serial.print(ax);
//   Serial.print(" | ay = ");
//  Serial.print(ay);
//  Serial.print(" | az = ");
//  Serial.println(az);

  //moveMouseAVG(2); 

//for LP filter, comment out if going unfiltered
  vy = int(readMPU_gyroY_LPF(gy)); 
  vx = int(readMPU_gyroZ_LPF(gz)); //happy with this function 
  
  Mouse.move(vx, vy);
  
  buttonState1 = digitalRead(button1);
  buttonState2 = digitalRead(button2);

//uncomment when you're ready to deal with random clicking or have grounded the wires for pins 4 + 5 
// if (buttonState1 == HIGH) {
//    Mouse.press(MOUSE_LEFT);
//    delay(100);
//    Mouse.release(MOUSE_LEFT);
//    delay(200);
//  } 
//  else if(buttonState2 == HIGH) {
//    Mouse.press(MOUSE_RIGHT);
//    delay(100);
//    Mouse.release(MOUSE_RIGHT);
//    delay(200);
//  }
  delay(200);
}
 

void moveMouseAVG(int numSamples){ //simple average filter for gyro data (not moving avg) 
  int gxsum = 0; 
  //int gysum = 0; 
  int gzsum = 0; 
  for(int i =0; i<numSamples; i++){
    gxsum += mpu.getRotationX(); 
    //gysum += mpu.getRotationY(); 
    gzsum += mpu.getRotationZ(); 
    delay(20); 
  }
    float newX = (gxsum/numSamples+150)/200; 
    float newY = -(gzsum/numSamples-100)/200; //get y value from gz samples 
  
  Mouse.move(newX, newY); 
  
}



float readMPU_gyroY_LPF(int inputReading){
 float updateVal = float(inputReading);
  MPUState_gyroY = updateVal*LPF_ALPHA + MPUState_gyroY*(1-LPF_ALPHA);
  return -(MPUState_gyroY-50)/100; 
}


float readMPU_gyroZ_LPF(int inputReading){
 float updateVal = float(inputReading);
  MPUState_gyroZ = updateVal*LPF_ALPHA + MPUState_gyroZ*(1-LPF_ALPHA);
  //rightHandRAmplify(); //something that amplifies pos x values because it's much harder to turn the right hand to the right (and vice versa)
  return (MPUState_gyroZ+50)/100; 
}

void rightHandRAmplify(){
  float rawVal = (MPUState_gyroZ+50)/100;
  if(rawVal > 0){
    rawVal = rawVal*5; 
  }
  MPUState_gyroZ = rawVal*300; 
}


//int readAxis(int gyroAxis){
//  MPUState = 
//}

