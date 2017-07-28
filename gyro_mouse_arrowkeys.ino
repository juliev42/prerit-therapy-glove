/*Created by Julie Vaughn and Sahil Kargwal for use as a glove-enabled mouse and game controller. Works via USB connection between glove and computer.
 * */

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h> //preferred IMU
#include <Mouse.h>
#include <Keyboard.h>


//IMU wiring notes: for ATMega boards (eg Leonardo, Yun, Pro Micro), the SCL is pin 3 and the SDA is pin 2 
//SCL, SDA, 5V and GND are necessary for I2C communication between the IMU and Arduino (4 connections) 
//This code only works with ATMega (keyboard-enabled boards) for now 

//Note that mouse mode is the first, and to activate arrow key mode instead you must type 'k' into the serial monitor

//currently right hand optimized! 

//Finger to arrow keys and mouse mapping:
//pointer finger -> pin 4 -> left arrow OR left click
//middle finger -> pin 5 -> right arrow OR right click 
//ring finger -> pin 6 -> up arrow 
//pinkie finger -> pin 8 -> down arrow



#define LPF_ALPHA .2//alpha value for the LP Filter on mouse data 
#define RESPONSE_TIME 100
//#define numMPUMeasures 6; 
#define YOFFSET 50 //Y offset value for my IMU, you may need to adjust up or down for your IMU (see readMPU_gyroY_LPF filter function)
#define XOFFSET 50 //same but for x -> larger will make it go up more 

//mouse click buttons 
#define button1 4 //will map left click to opponance with pointer finger 
#define button2 5 //will map right click to opponance with middle finger 

//sensor pins for arrow key fingers (opponance)
#define sensorPin1 4
#define sensorPin2 5
#define sensorPin3 6
#define sensorPin4 8


bool arrowKeysON = 0; //for toggling arrow key ability on and off 


MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
int vx, vy;


float MPUState_gyroZ; //keeps track of filtered data value from z gyro
float MPUState_gyroY; //keeps track of filtered value from y gyro 

void setup() {

  Serial.begin(9600);
  
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);

  while (!Serial); //wait for serial monitor to be initialized (this is only necessary for the Leo/Micro ATMega processor I think)

  Serial.println(F("Send any key to begin mouse function...")); //to activate the mouse function so it doesn't happen immediately when uploaded
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  
  
  Serial.println(F("Initializing glove mouse...")); 
  Serial.println(F("Send k at any time to intiailize RLUD arrow keys on fingers")); 
  mpu.initialize();
  Wire.begin();
  if (!mpu.testConnection()) { while (1); }

//MPU offsets 
   //mpu.setXGyroOffset(-200);
    //mpu.setYGyroOffset(76);
    //mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //accelerometer then gyroscope data 


//  vx = (gx+15)/150;  deprecated
//  vy = -(gz-100)/150; deprecated

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


//  this mouse is only using gyro data but in case we want to change that 
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
  

//uncomment the code below when you're ready to deal with random clicking or have grounded the wires for pins 4 + 5 
//currently have it so that clicking is disabled if arrow keys are on 

//left click pointer finger 
 if (digitalRead(button1) ==HIGH && !arrowKeysON) {
    Mouse.press(MOUSE_LEFT);
    delay(100);
    Mouse.release(MOUSE_LEFT);
    delay(200);
  } 

 //right click index finger 
  else if(digitalRead(button2) ==HIGH && !arrowKeysON) {
    Mouse.press(MOUSE_RIGHT);
    delay(100);
    Mouse.release(MOUSE_RIGHT);
    delay(200);
  }



  delay(RESPONSE_TIME); //change RESPONSE_TIME at top depending on how fast you want the mouse to respond to commands. 100-300 is safe

  

if(Serial.available() && Serial.read() == 'k'){
  Serial.println("LRUD arrow keys initialized on fingertips...");
  Serial.println(); 

  arrowKeysON = 1; 
}


if(arrowKeysON){

  Keyboard.begin();                                   ///started the keyboard
                                                        // if want a continous press then only use keyboard.press
                                                        // keyboard.print will take control over your keyboard so make sure to control it by ext pushbutton
                                                        //keyboard.write is similar to pressing and then releasing a key on keyboard(Also takes over your keyboard)
  


if (digitalRead(sensorPin1) == HIGH )
{
    Serial.print(1);

    Keyboard.write('l');
    Keyboard.press(KEY_LEFT_ARROW); 
    delay(500);
    Keyboard.release(KEY_LEFT_ARROW);

}



if (digitalRead(sensorPin2) == HIGH )
{
    Serial.print(2);

    
     Keyboard.write('r');
    Keyboard.press(KEY_RIGHT_ARROW); 
    delay(500);
    Keyboard.release(KEY_RIGHT_ARROW);


}



if (digitalRead(sensorPin3) == HIGH )
{

    Serial.print(3);


Keyboard.write('u');
     Keyboard.press(KEY_UP_ARROW); 
    delay(500);
    Keyboard.release(KEY_UP_ARROW);
}


if (digitalRead(sensorPin4) == HIGH )
{

    Serial.print(4);

Keyboard.write('d');
     Keyboard.press(KEY_DOWN_ARROW); 
    delay(500);
    Keyboard.release(KEY_DOWN_ARROW);

}



//BTserial.print("1234");

//BTserial.print(";");

Keyboard.end();                                   ///Ending the keyboard


delay(20);

}


  
} //end main loop 
 

//[not in use] function for a simple avg filter 
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



//alpha filters yay
float readMPU_gyroY_LPF(int inputReading){
 float updateVal = float(inputReading);
  MPUState_gyroY = updateVal*LPF_ALPHA + MPUState_gyroY*(1-LPF_ALPHA);
  return -(MPUState_gyroY-YOFFSET)/100; 
}


float readMPU_gyroZ_LPF(int inputReading){
 float updateVal = float(inputReading);
  MPUState_gyroZ = updateVal*LPF_ALPHA + MPUState_gyroZ*(1-LPF_ALPHA);
  //rightHandRAmplify(); //something that amplifies pos x values because it's much harder to turn the right hand to the right (and vice versa)
  return (MPUState_gyroZ+XOFFSET)/100; 
}

void rightHandRAmplify(){
  float rawVal = MPUState_gyroZ;
  if(rawVal > 0){
    rawVal = rawVal*5; 
  }
  MPUState_gyroZ = rawVal; 
}


//int readAxis(int gyroAxis){
//  MPUState = 
//}

