#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Mouse.h>

#define LPF_ALPHA .2; //alpha value for the LP Filter 

int button1 = 4; //will map click to pointer opponance exercise later, to be updated 
int button2 = 7;

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
int vx, vy;

int buttonState1 = 0; 
int buttonState2 = 0; 

void setup() {

  Serial.begin(9600);
  Wire.begin();
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  mpu.initialize();
  if (!mpu.testConnection()) { while (1); }
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


  Serial.print("ax = ");
  Serial.print(ax);
   Serial.print(" | ay = ");
  Serial.print(ay);
  Serial.print(" | az = ");
  Serial.println(az);

  moveMouseAVG(5); 
  
  //Mouse.move(vx, vy);
  
  buttonState1 = digitalRead(button1);
  buttonState2 = digitalRead(button2);
  
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
    float newX = (gxsum/numSamples+15)/150; 
    float newY = -(gzsum/numSamples-100)/150; //get y value from gz samples 
  
  Mouse.move(newX, newY); 
  
}

