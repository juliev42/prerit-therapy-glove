/******************************************************************************
Flex_Sensor_Example.ino
Example sketch for SparkFun's flex sensors
  (https://www.sparkfun.com/products/10264)
Jim Lindblom @ SparkFun Electronics
April 28, 2016

Create a voltage divider circuit combining a flex sensor with a 47k resistor.
- The resistor should connect from A0 to GND.
- The flex sensor should connect from A0 to 3.3V
As the resistance of the flex sensor increases (meaning it's being bent), the
voltage at A0 should decrease.

Development environment specifics:
Arduino 1.6.7
******************************************************************************/
const int FLEX_PIN0 = A0; // Pin connected to voltage divider output //index finger 
const int FLEX_PIN1 = A1; //middle finger
const int FLEX_PIN2 = A2; //ring finger
const int FLEX_PIN3 = A3; //pinkie finger
const int buttonPin = 2; 
const int N = 10; //how many values go into the calibration session 


int sensorPin1 = 8;
int sensorPin2 = 6;
int sensorPin3 = 4;
int sensorPin4 = 2;

int buttonState = 0;

#include <SoftwareSerial.h>
#include "Keyboard.h"

SoftwareSerial BTserial(11, 12); // RX | TX



// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 46400.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
// may need to specify for each user --> make a calibrate function and button for finding sensor values 
const float STRAIGHT_RESISTANCE = 23000.0; // resistance when straight
const float BEND_RESISTANCE = 28500.0; // resistance at 90 deg
 

void setup() 
{
  Serial.begin(9600);
  BTserial.begin(9600); 
  pinMode(FLEX_PIN0, INPUT);
  pinMode(FLEX_PIN1, INPUT);
  pinMode(FLEX_PIN2, INPUT);
  pinMode(FLEX_PIN3, INPUT);

  pinMode(buttonPin, INPUT);  

   pinMode(sensorPin1, INPUT);
    pinMode(sensorPin2, INPUT);
    pinMode(sensorPin3, INPUT);
    pinMode(sensorPin4, INPUT);  

}

void loop() 
{
//  BTserial.print("test");
//  BTserial.print(","); 

  //flex sensor 1
  int pin = FLEX_PIN0;
  int flexADC = analogRead(pin);
  //int flexADC = analogRead(A0);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);                        // not getting why have we done this???
  Serial.println(String(pin)); 
  Serial.println("Resistance: " + String(flexR) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);                                    // so flex sensors never calculated angles ??? it was just suitable mapping(or bringing numbers down) we can 
                                                                // use this further for more precsion
               
  Serial.println("Bend: " + String(angle) + " degrees");
  Serial.println();

  BTserial.print(String(angle)); 
  BTserial.print(","); 



  //flex sensor 2
  pin = FLEX_PIN1;
 flexADC = analogRead(pin);
  //int flexADC = analogRead(A0);
  flexV = flexADC * VCC / 1023.0;
  flexR = R_DIV * (VCC / flexV - 1.0);
  Serial.println(String(pin)); 
  Serial.println("Resistance: " + String(flexR) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
  float angle1 = angle; 
               
  Serial.println("Bend: " + String(angle) + " degrees");
  Serial.println();

  BTserial.print(String(angle)); 
  BTserial.print(","); 



//flex sensor 3
  pin = FLEX_PIN2;
  flexADC = analogRead(pin);
  //int flexADC = analogRead(A0);
  flexV = flexADC * VCC / 1023.0;
  flexR = R_DIV * (VCC / flexV - 1.0);
  Serial.println(String(pin)); 
  Serial.println("Resistance: " + String(flexR) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
               
  Serial.println("Bend: " + String(angle) + " degrees");
  Serial.println();

  BTserial.print(String(angle)); 
  BTserial.print(","); 



  //flex sensor 4
  pin = FLEX_PIN3;
  flexADC = analogRead(pin);
  //int flexADC = analogRead(A0);
  flexV = flexADC * VCC / 1023.0;
  flexR = R_DIV * (VCC / flexV - 1.0);
  Serial.println(String(pin)); 
  Serial.println("Resistance: " + String(flexR) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
               
  Serial.println("Bend: " + String(angle) + " degrees");
  Serial.println();

  BTserial.print(String(angle)); 
  BTserial.print(","); 



    Keyboard.begin();                                   ///started the keyboard
                                                        // if want a continous press then only use keyboard.press
                                                        // keyboard.print will take control over your keyboard so make sure to control it by ext pushbutton
                                                        //keyboard.write is similar to pressing and then releasing a key on keyboard(Also takes over your keyboard)

     
if (digitalRead(sensorPin1) == HIGH )
{
    BTserial.print(1);
    Keyboard.write('r');
    Keyboard.press(KEY_RIGHT_ARROW); 
    delay(500);
    Keyboard.release(KEY_RIGHT_ARROW);
}

else BTserial.print(0);
   
   // BTserial.print(",");


if (digitalRead(sensorPin2) == HIGH && !digitalRead(sensorPin3) == HIGH )
{
    BTserial.print(2);
    Keyboard.press(KEY_LEFT_ARROW); 
    delay(500);
    Keyboard.release(KEY_LEFT_ARROW);
}

else BTserial.print(0);
   
    //BTserial.print(",");

if (digitalRead(sensorPin3) == HIGH )
{
    BTserial.print(3);
    Keyboard.press(KEY_UP_ARROW); 
    delay(500);
    Keyboard.release(KEY_UP_ARROW);
}

else BTserial.print(0);
   
    //BTserial.print(",");

if (digitalRead(sensorPin4) == HIGH )
{
    BTserial.print(4);
    Keyboard.press(KEY_DOWN_ARROW); 
    delay(500);
    Keyboard.release(KEY_DOWN_ARROW);

    
}

else BTserial.print(0);
   
    BTserial.print(";");

//BTserial.print("1234");

//BTserial.print(";");
    Keyboard.end();                                   ///Ending the keyboard


delay(20);
  
  

  delay(500);

// getReadingPin(FLEX_PIN0);  
// getReadingPin(FLEX_PIN1); 
// getReadingPin(FLEX_PIN2); 
// getReadingPin(FLEX_PIN3); 

 //buttonState = digitalRead(buttonPin);

 //BTserial.print(";"); 


// if(buttonState == LOW) {     
//  calibrate(); 
//  buttonState = HIGH; 
// } 

}


void getReadingPin(int pin){
  int flexADC = analogRead(pin);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  Serial.println(String(pin)); 
  Serial.println("Resistance: " + String(flexR) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
               
  Serial.println("Bend: " + String(angle) + " degrees");
  Serial.println();

  BTserial.print(String(angle)); 
  BTserial.print(","); 


  

  delay(500);

  
}

float getDegrees(int pin){
  int flexADC = analogRead(pin);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
  return(angle);                  
                    
}



void calibrate(){ //stores values for the angles for each of the pins 
 Serial.println("Calibrating...");

//float indexFinger[N];
//float middleFinger[N];
//float ringFinger[N];
//float pinkieFinger[N];

float indexSum = 0;
float middleSum = 0;
float ringSum = 0;
float pinkieSum = 0;

  for(int i = 0; i<N; i++){
  indexSum += getDegrees(FLEX_PIN0);
  middleSum += getDegrees(FLEX_PIN1);
  ringSum += getDegrees(FLEX_PIN2);
  pinkieSum += getDegrees(FLEX_PIN3);
  delay(500);  
  }
  
  float indexAvg = indexSum/N;
  float middleAvg = middleSum/N;
  float ringAvg = ringSum/N;
  float pinkieAvg = pinkieSum/N;
  
  Serial.println("Index average:" + String(indexAvg)); 
  Serial.println("Middle average:" + String(middleAvg)); 
  Serial.println("Ring average:" + String(ringAvg)); 
  Serial.println("Pinkie average:" + String(pinkieAvg)); 


}

