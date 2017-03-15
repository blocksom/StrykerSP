#include <SoftwareSerial.h>
SoftwareSerial BTserial(9, 10); // RX | TX
// Connect the HC-05 TX to Arduino pin 9 RX. 
// Connect the HC-05 RX to Arduino pin 10 TX through a voltage divider.
 
// this constant won't change:
const int  buttonPin = 4;    // the pin that the pushbutton is attached to
const int potPin = 2;        // select the input pin for the potentiometer
const int txPin = 1;         // transmission pin for knee angle 

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int calibrationStep;
int lowEnd=0;
int highEnd=1024;
int offset=0;
int val=0;
int kneeAngle=0;
 
void setup() 
{
    Serial.begin(9600);
    // HC-06 default serial speed for communcation mode is 9600
    BTserial.begin(9600);  
}
 
void loop() 
{

    //BTserial.println("Bluetooth Initialize..."); 
    //BTserial.println("Bluetooth Test..."); 
    //delay(1000);   
    
    // read the pushbutton input pin:
    buttonState = digitalRead(buttonPin);
    // compare the buttonState to its previous state
    if (buttonState != lastButtonState) {
      // if the state has changed, increment the counter
      if (buttonState == HIGH) {
        // if the current state is HIGH then the button
        // wend from off to on:
        buttonPushCounter++;
      } else {
        // if the current state is LOW then the button
        // wend from on to off:
         if (calibrationStep==1){
            BTserial.println("Place brace in closed state and press button to continue.");
         }
         else if(calibrationStep==2){
           BTserial.println("Place brace in open state and press button to continue");
         }
         else if(calibrationStep==3){
            BTserial.println("Lock brace at 0 degrees and press button to finish calibration.");
         }    
         else if(calibrationStep==4){
            BTserial.println("Calibration completed.");
         }   
      }
      // Delay a little bit to avoid bouncing
      //delay(50);
    }
    // save the current state as the last state,
    //for next time through the loop
    lastButtonState = buttonState;
    
    calibrationStep=buttonPushCounter;
    //delay(50);
  
    if (calibrationStep==1){
          lowEnd=analogRead(potPin);
    }
    else if(calibrationStep==2){
         highEnd=analogRead(potPin);
    }
    else if(calibrationStep==3){
         offset=analogRead(potPin);
    }       
    else if(calibrationStep>3){
        val=analogRead(potPin);
        kneeAngle=(180.0/(highEnd-lowEnd))*(val-offset);
        BTserial.println(kneeAngle);
    }
}
