//STRYKER SMART POSITION INDICATOR

//PROJECT SPONSORED BY: STRYKER ENDOSCOPY, SAN JOSE CALIFORNIA

//PROJECT MEMBERS:
//DANIEL MONTOYA - MECHANICAL ENGINEERING
//KATHERINE MAVROMMATI - BIOMEDICAL ENGINEERING
//KYLE BLOCKSOM - COMPUTER ENGINEERING
//TRISTAN HONDA - COMPUTER ENGINEERING

//ESSENTIAL HARDWARE:
//2X - HC-05 BLUETOOTH MODULE
//1X - LINEAR 10KOHM ROTARY POTENTIOMETER
//1X - ADAFRUIT 16 LED NEOPIXEL RING
//2X - MOMENTARY BUTTON
//1X - ADAFRUIT BNO055 ABSOLUTE ORIENTATION SENSOR

//REVISION - FINAL
//REVISION DATE - 06/04/2017
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

//IMPORT NECESSARY LIBRARIES

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

//DEFINE ADAFRUIT BNO055

Adafruit_BNO055 bno = Adafruit_BNO055();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

//LED RING SETUP AND LIGHTING FUNCTION

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, 6, NEO_GRB + NEO_KHZ800);

// Define colors for use throughout script. Colors can be any
// RGB combination.
uint32_t green = strip.Color(0, 255, 0);
uint32_t yellow = strip.Color(255, 200, 0);
uint32_t red = strip.Color(255, 0, 0);
uint32_t blue = strip.Color(0, 255, 255);
uint32_t off = strip.Color(0, 0, 0);
uint32_t purple = strip.Color(230, 230, 250);
uint32_t orange = strip.Color(255, 153, 0);
uint32_t dark_blue = strip.Color(0, 0, 255);
uint32_t white = strip.Color(255, 255, 255);

void startLight(uint32_t color, int t, int I, int limit) {
  if (I == limit) // terminating condition
    return;

  strip.setPixelColor(I, color);
  strip.show();
  delay(t);
  startLight(color, t, I + 1, limit);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

//HC-05 BLUETOOTH COMUNICATION SET UP

SoftwareSerial BTserial(9, 10); // RX | TX
// Connect the HC-05 TX to Arduino pin 9 RX.
// Connect the HC-05 RX to Arduino pin 10 TX through a voltage divider.
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

//DEFINITION OF PIN READOUTS

// this constant won't change:
const int verifyPin = 5;     // the pin that the verify pushbutton is attached to
const int  buttonPin = 4;    // the pin that the pushbutton is attached to
const int potPin = 2;        // select the input pin for the potentiometer

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

// Verification vars
unsigned int verifyFlag;
int verifyPushCounter = 0;
int verifyState = 0;
int lastVerifyState = 0;
int verifyStep;
static int verifyCount = 0;

int lowEnd = 1024;
int highEnd = 0;
int kneeOffset = 0;
int kneeAngle = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

//SET UP COMMUNICATION AND SENSORS

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // HC-06 default BTserial speed for communcation mode is 9600
  BTserial.begin(9600);

  //INITIALIZE ADAFRUIT BNO05 SENSOR
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);

  // initialize LED ring
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(10);
  startLight(blue, 50, 0, 16);  // flashes blue ring indicating start up success
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

//MAIN LOOP OF FUNCTIONAL CODE

void loop()
{
  //SETUP VECTOR TO IMPORT ANGLE DATA FROM ADAFRUIT BNO05
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> Mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  uint8_t system, gyro, accel, mag = 0;
  static float imuOffset[3];
  float rawIMU[3];
  float abductionAngle = 0;
  float flexionAngle = 0;
  float externalRotationAngle = 0;

  //SETUP VECTOR TO STORE POSITIONA INFORMATION AND VERIFICATION MODE
  static int saved[6];
  static int savedCount = 0, loopCount = 0, flag = 0;
  float *dest1;
  int counter = 0, i = 0;

  //BEGIN MAIN PUSHBUTTON FUNCTIONALITY
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
      if (buttonPushCounter == 1) {
        BTserial.println("Fully open and close brace and press button to continue.");
      }
      else if (buttonPushCounter == 2) {
        BTserial.println("Lay brace on flat surface until green light advances.");
      }
      else if (buttonPushCounter == 3) {
        BTserial.println("Complete one full rotation of knee brace about long axis, pause every 45 degrees for three seconds. Hold still or lay flat once complete until green light advances.");
      }
      else if (buttonPushCounter == 4) {
        BTserial.println("Place brace on patient, wave leg in figure 8 until green light advances.");
      }
      else if (buttonPushCounter == 5) {
        BTserial.println("Place patient in reference configuration and press button to finish calibration.");
      }
      else if (buttonPushCounter == 6) {
        BTserial.println("Calibration completed.");
      }
    }
    // DELAY TO AVOID BOUNCING
    delay(50);
  }

  // save the current state as the last state,
  //for next time through the loop
  lastButtonState = buttonState;
  delay(50);

  //BEGIN OPERATIONS BASED ON NUMBER OF BUTTON PUSHES
  if (buttonPushCounter == 1) {               //BEGIN CALIBRATION STEP 1
    startLight(red, 1, 0, 10);                 //DETERMINED HIGH AND LOW END POTENTIOMETER VALUES
    if (lowEnd > analogRead(potPin))
      lowEnd = analogRead(potPin);
    if (highEnd < analogRead(potPin))
      highEnd = analogRead(potPin);
  }                                           //END CALIBRATION STEP 1

  else if (buttonPushCounter == 2) {          //BEGIN CALIBRATION STEP 2
    startLight(green, 1, 0, 2);
    bno.getCalibration(&system, &gyro, &accel, &mag);

    if (gyro > 2) {
      startLight(green, 1, 2, 4);
      buttonPushCounter = 3;
    }
  }                                           //END CALIBRATION STEP 2

  else if (buttonPushCounter == 3) {          //BEGIN CALIBRATION STEP 3
    bno.getCalibration(&system, &gyro, &accel, &mag);

    if (accel > 2) {
      startLight(green, 1, 4, 6);
      buttonPushCounter = 4;
    }
  }                                           //END CALIBRATION STEP 3

  else if (buttonPushCounter == 4) {          //BEGIN CALIBRATION STEP 4
    bno.getCalibration(&system, &gyro, &accel, &mag);

    if (mag > 2)
      startLight(green, 1, 6, 8);
    if (mag > 2 && system > 2) {
      startLight(green, 1, 8, 10);
      buttonPushCounter = 5;
    }
  }                                           //END CALIBRATION STEP 4

  else if (buttonPushCounter == 5) {          //BEGIN CALIBRATION STEP 5
    //RECORD OFFSET VALUES FOR SENSORS
    imuOffset[0] = euler.x(); //yaw
    imuOffset[1] = euler.y(); //pitch
    imuOffset[2] = euler.z(); //roll
    kneeOffset = analogRead(potPin);
  }                                           //END CALIBRATION STEP 5
  //END CALIBRATION

  else if (buttonPushCounter == 6) {          //BEGIN DATA TRANSMISION LOOP
    startLight(green, 1, 10, 12);
    startLight(green, 100, 12, 16);
    startLight(off, 1, 0, 16);
    delay(100);
    startLight(green, 1, 0, 16);
    delay(100);
    startLight(off, 1, 0, 16);
    delay(100);
    startLight(green, 1, 0, 16);
    delay(100);
    startLight(off, 1, 0, 16);
    delay(100);
    startLight(white, 1, 0, 16);
    buttonPushCounter = 7;
  }

  else if (buttonPushCounter > 6) {

    static int holdcount = 0;
    
    kneeAngle = (140.0 / (highEnd - lowEnd)) * (analogRead(potPin) - kneeOffset);       //BEGIN CALCULATION OF ANGLES

    rawIMU[0] = euler.x(); //yaw
    rawIMU[1] = euler.y(); //pitch
    rawIMU[2] = euler.z(); //roll

    abductionAngle = rawIMU[0] - imuOffset[0];
    flexionAngle = -(rawIMU[1] - imuOffset[1]);
    externalRotationAngle = rawIMU[2] - imuOffset[2];                                   //END CALCULATION OF ANGLES

    //POSITION SAVING BUTTON PRESS FUNCTIONALITY
    if (digitalRead(buttonPin) == HIGH) {
      while (digitalRead(buttonPin) == HIGH) {
        holdcount++;
        delay(1);
        if (holdcount == 1500) {
          if (loopCount == 6) {
            startLight(off, 1, 0, 16);
            delay(100);
            startLight(yellow, 1, 0, 16);
            delay(400);
          }

          if (savedCount <= 5 && !flag) {
            saved[verifyPushCounter] = kneeAngle;

            savedCount++;
            if (loopCount != 6)
              loopCount = savedCount;
          } else {
            if (savedCount == 6)
              savedCount = 0;

            flag = 1;
            saved[savedCount] = kneeAngle;
            //saved[savedCount, 1] = theta;
            //saved[savedCount, 2] = phi;
            //saved[savedCount, 3] = psi;
          }

          startLight(off, 1, 0, 16);
          delay(100);
          startLight(blue, 1, 0, 16);
          delay(400);
          startLight(off, 1, 0, 16);
          delay(100);
          startLight(white, 1, 0, 16);
          BTserial.print("store me: ");
          BTserial.println(kneeAngle);
          holdcount = 0;
          return;
        }
      }
    }

    // read the pushbutton input pin:
    verifyState = digitalRead(verifyPin);
    // compare the buttonState to its previous state
    if (verifyState != lastVerifyState) {
      // if the state has changed, increment the counter
      if (verifyState == HIGH) {
        verifyPushCounter++;
      } else ;
      // Delay a little bit to avoid bouncing
    }
    lastVerifyState = verifyState;
    if (verifyPushCounter == 6 || verifyPushCounter == loopCount)
      verifyPushCounter = 0;

    verifyCount = 0;
    if (digitalRead(verifyPin) == HIGH) {
      while (digitalRead(verifyPin) == HIGH) {
        verifyCount++;
        delay(1);
        if (verifyCount == 1500) {
          verifyPushCounter--;

          if (verifyFlag)
            verifyFlag = 0;
          else
            verifyFlag = 1;

          startLight(off, 1, 0, 16);
          delay(100);
          startLight(orange, 1, 0, 16);
          delay(400);
          startLight(off, 1, 0, 16);
          delay(100);
          holdcount = 0;
          return;
        }
      }
    }  // end of if digitalRead high statement

    if (verifyFlag) {
      BTserial.print("v");      // tells GUI we're in verification mode (ascii 118)
      BTserial.println();
      startLight(off, 1, 4, 5);
      startLight(off, 1, 11, 12);
      startLight(dark_blue, 1, 5, 5 + loopCount);

      if (verifyPushCounter == -1)
        verifyPushCounter = 0;
      if (saved[verifyPushCounter] != -1) {
        BTserial.print(saved[verifyPushCounter]);      // tells GUI verification knee angle
        BTserial.println();
        startLight(yellow, 1, 5 + verifyPushCounter, 6 + verifyPushCounter);

        if ((abs(kneeAngle - saved[verifyPushCounter]) <= 10) && (abs(kneeAngle - saved[verifyPushCounter]) > 3)) {
          startLight(yellow, 1, 0, 4);
          startLight(yellow, 1, 12, 16);
          BTserial.print("ky");      // tells GUI we're knee yellow (ascii 121)
          BTserial.println();
        }
        else if ((abs(kneeAngle - saved[verifyPushCounter]) <= 3)) {
          startLight(green, 1, 0, 4);
          startLight(green, 1, 12, 16);
          BTserial.print("kg");      // tells GUI we're knee green (ascii 103)
          BTserial.println();
        }
        else {
          startLight(red, 1, 0, 4);
          startLight(red, 1, 12, 16);
          BTserial.print("kr");      // tells GUI we're knee red (ascii 114)
          BTserial.println();
        }
      }
    }


  }
}



