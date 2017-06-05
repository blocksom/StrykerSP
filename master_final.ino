#include <SoftwareSerial.h>
SoftwareSerial BTserial(9, 10); // RX | TX
// Connect the HC-05 TX to Arduino pin 9 RX. 
// Connect the HC-05 RX to Arduino pin 10 TX through a voltage divider.

// this constant won't change:
const int buttonPin = 4;    // the pin that the pushbutton is attached to
const int validateButton = 5;   // the pin that validation button is attached to 
const int potPin = 2;        // select the input pin for the potentiometer
const int txPin = 1;         // transmission pin for knee angle 

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int validationState = 0;     // current state of validation button
int lastButtonState = 0;     // previous state of the button
int calibrationStep;
int lowEnd=0;
int highEnd=1024;
int offset=0;
int val=0;
int incomingAngle=0;
 
void setup() 
{
    Serial.begin(9600);   
    Serial.println("Please Press Button to Begin Calibration");  
    // HC-05 default serial speed for commincation mode is 9600
    BTserial.begin(9600);  
}
 
void loop()
{
    // Keep reading from HC-05 and send to Arduino Serial Monitor
    if (BTserial.available())
    {  
        incomingAngle = BTserial.read();
        Serial.write(incomingAngle);
    }
}
