int sensorPin1 = 5;
int sensorPin2 = 6;
int sensorPin3 = 4;
int sensorPin4 = 3;

#include <SoftwareSerial.h>
#include <Keyboard.h>

SoftwareSerial BTSerial(0,1);


void setup() 
{
  Serial.begin(9600);
  BTSerial.begin(9600);
  BTSerial.setTimeout(5);
  Serial.setTimeout(5);
  
    pinMode(sensorPin1, INPUT);
    pinMode(sensorPin2, INPUT);
    pinMode(sensorPin3, INPUT);
    pinMode(sensorPin4, INPUT);  

}


void loop() 
{

  if(BTSerial.available()>0)
  {


  Keyboard.begin(); 
  if (digitalRead(sensorPin1) == HIGH )
 {
    //BTserial.print(1); // I think this was only for the app thing
    Serial.println("right");
    //Keyboard.write('r');
    Keyboard.press(char(KEY_RIGHT_ARROW)); 
    delay(100);
    Keyboard.release(KEY_RIGHT_ARROW);
    
 }

   //else BTserial.print(0);
   
   // BTserial.print(",");


   else if (digitalRead(sensorPin2) == HIGH ) // need to add else then what about multiple presses
  {
    //BTserial.print(2);
    Serial.println("left");
    Keyboard.press(KEY_LEFT_ARROW); 
    delay(100);
    Keyboard.release(KEY_LEFT_ARROW);
  }

//else BTserial.print(0);
   
    //BTserial.print(",");

 else if (digitalRead(sensorPin3) == HIGH )
  {
    //BTserial.print(3);
    Serial.println("up");
    Keyboard.press(KEY_UP_ARROW); 
    delay(100);
    Keyboard.release(KEY_UP_ARROW);
  }

//else BTserial.print(0);
   
    //BTserial.print(",");

  else if (digitalRead(sensorPin4) == HIGH)
  {
  //  BTserial.print(4);
    Serial.println("down");
    Keyboard.press(KEY_DOWN_ARROW); 
    delay(100);
    Keyboard.release(KEY_DOWN_ARROW);

    
  }

//else BTserial.print(0);
   
   // BTserial.print(";");

//BTserial.print("1234");

//BTserial.print(";");
    Keyboard.end();                                   ///Ending the keyboard


delay(20);
  
  

 // delay(500);   




  }  // end of the first if statement
  
}

