#include <SoftwareSerial.h>
#include <Wire.h>
#include "MPU9250.h"
#include "Vector.h"
#include <math.h>
#include <Adafruit_NeoPixel.h>

#define PIN 6

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, PIN, NEO_GRB + NEO_KHZ800);

// Define colors for use throughout script. Colors can be any 
// RGB combination.
uint32_t green = strip.Color(0, 255, 0);
uint32_t yellow = strip.Color(255,200,0);
uint32_t red = strip.Color(255, 0, 0);
uint32_t blue = strip.Color(0, 255, 255);
uint32_t off = strip.Color(0, 0, 0);
uint32_t purple = strip.Color(230, 230, 250);
uint32_t orange = strip.Color(255, 153, 0);
uint32_t dark_blue = strip.Color(0, 0, 255);
uint32_t white = strip.Color(255, 255, 255);

SoftwareSerial BTserial(9, 10); // RX | TX
// Connect the HC-05 TX to Arduino pin 9 RX. 
// Connect the HC-05 RX to Arduino pin 10 TX through a voltage divider.

class GForce {
  public:
  float gForce[3] = {0.0, 0.0, 0.0};
  float Calibration_Matrix[3][3] = {};
  float final_gForce[3][0] = {};
  float magRaw[3] = {};
  float roll;
  float pitch;
  float yaw;
};

long accelX, accelY, accelZ;
//float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

long magX, magY, magZ;
float flex, ext, abd; 

long asaX, asaY,asaZ;
 
// this constant won't change:
const int verifyPin = 5;
const int  buttonPin = 4;    // the pin that the pushbutton is attached to
const int potPin = 2;        // select the input pin for the potentiometer
const int txPin = 1;         // transmission pin for knee angle 

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int calibrationStep;

// Verification vars
unsigned int verifyFlag;
int verifyPushCounter = 0;
int verifyState = 0;
int lastVerifyState = 0;
int verifyStep;
static int verifyCount = 0;

int lowEnd=0;
int highEnd=1024;
int offset=0;
int val=0;
int kneeAngle=0;

int M = 3;
int N = 3;
int R = 3;
int L = 1;

void setup() {
   Serial.begin(9600);
   Wire.begin();
   setupMPU();

   // HC-06 default BTserial speed for communcation mode is 9600
   BTserial.begin(9600);  

   // initialize LED ring
   strip.begin();
   strip.show(); // Initialize all pixels to 'off'
   strip.setBrightness(10);
   startLight(blue, 50, 0, 16);  // flashes blue ring indicating start up success
}

void loop() {
  recordAccelRegisters();
  recordMagRegisters();
  IMU_calibration();
  delay(100);
}

void startLight(uint32_t color,int t,int I, int limit) {
   //static int I=0;
   if (I==limit)  // terminating condition
      return;
   
   strip.setPixelColor(I, color);
   strip.show();
   delay(t);
   startLight(color, t, I + 1, limit);
}

MPU9250 myIMU;
void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
  
  Wire.write(0x0C); //Accessing the register 0C - Magnetometer Configuration (Sec. 5.1)
  //Wire.write(0b00000000); //Setting the magnetometer to 14bit
  Wire.endTransmission();

     byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
   #define SerialDebug true // Set to true to get Serial output for debugging
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    
    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
       
    if (SerialDebug)
    {
       // Serial.println("Calibration values: ");
//      Serial.print("X-Axis sensitivity adjustment value ");
//      Serial.println(myIMU.magCalibration[0], 2);
//      Serial.print("Y-Axis sensitivity adjustment value ");
//      Serial.println(myIMU.magCalibration[1], 2);
//      Serial.print("Z-Axis sensitivity adjustment value ");
//      Serial.println(myIMU.magCalibration[2], 2);
    }
    else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
  } // end of if statement WHO_AM_I should be 0x68
} // end of setupMPU()

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
} // end of recordAccelRegisters()

GForce g;
Vector v;
void processAccelData(){
  g.gForce[0] = (accelX / 16384.0)*-1;
  g.gForce[1] = (accelY / 16384.0); 
  g.gForce[2] = (accelZ / 16384.0)*-1;
    
} // end of processAccelData()

void recordMagRegisters(){ 
 
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){
  myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
   myIMU.my = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
   myIMU.mx = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
   myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  g.magRaw[0] = (float)myIMU.mx;
  g.magRaw[1] = (float)myIMU.my;
  g.magRaw[2] = (float)myIMU.mz;
  
   } // end of if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
} // end of void recordMagRegisters()

void magcalMPU9250(float * dest1) 
{
 uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
 
  //Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
   sample_count = 64;
   for(ii = 0; ii < sample_count; ii++) {
    myIMU.readMagData(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
   }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*myIMU.mRes*myIMU.magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*myIMU.mRes*myIMU.magCalibration[1]; 
    dest1[2] = (float) mag_bias[2]*myIMU.mRes*myIMU.magCalibration[2];        

   //BTserial.println("Mag Calibration done!"); 
}

void IMU_calibration() 
{   
   static int saved[6];
   static int savedCount = 0, loopCount = 0, flag = 0;
   float *dest1;
   int counter = 0, i = 0;
    
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
            BTserial.println("Cycle brace through most open and most closed positions.");
         }
         else if(calibrationStep==2){
           BTserial.println("Wave device in figure eight until done!");
         }
         else if(calibrationStep==3){
            BTserial.println("Place leg such that it is at zero abd/add, zero flex/ext and zero int/ext and press button to finish calibration.");
         } 
         else if(calibrationStep==4){
            BTserial.println("Calibration completed.");
         }   
      } // else
      // Delay a little bit to avoid bouncing
      delay(50);
    }
    // save the current state as the last state,
    //for next time through the loop
    lastButtonState = buttonState;
    calibrationStep=buttonPushCounter;
    delay(50);
  
    
    if (calibrationStep==1){
          if (lowEnd>analogRead(potPin)){
          lowEnd=analogRead(potPin);}
       if(highEnd<analogRead(potPin)){
          highEnd=analogRead(potPin);}
          startLight(red, 1, 0, 6);
    } // end calibration step 1
//    else if(calibrationStep==2) {
//        magcalMPU9250(dest1) ;
//        startLight(green, 1, 0, 2); 
//       } // end of calibrationStep 2
       
      else if (calibrationStep==2) {
      float Norm[3];
      v.Vector_Norm(g.gForce,Norm);           //Normalizing the output of accelerometer
      float correct_vect[3] = {0,0,1};            //Definining what the output should be
      float Cross[3];
      v.Vector_Cross_Product(Norm,correct_vect,Cross);       // cross product between normalized acc output and what output should be;
      float SSC[3][3];
      v.Create_SSC((float*)Cross,SSC);           // Creating SSC matrix
      float identity[3][3];
      v.Identity(identity);              // Creating 3x3 identity matrix
      float dot_prod = v.Vector_Dot_Product((float*)Norm,(float*)correct_vect); // dot product b/w normalized accel output and what output should be
      float C = v.Norm_const(Cross);
      float divisor = pow(C,2);
      float mult_const = ((1-dot_prod)/divisor);
      float squared_SSC[3][3];
      v.Multiply((float*)SSC,(float*)SSC,M,N,N,(float*)squared_SSC);
      float new_SSC[3][3];
      v.Matrix_Const_Mult(squared_SSC,mult_const,new_SSC);
      float Add_1[3][3];
      v.Add((float*)identity,(float*) SSC,M,N,(float*)Add_1);
      v.Add((float*)Add_1,(float*) new_SSC,M,N,(float*)g.Calibration_Matrix);
//      float gForce_transpose[3][1];
//      v.Transpose((float*)g.gForce, L,N, (float*)gForce_transpose); // Transpose of normalized accel vector
//      float final_gForce[3][1];
//      v.Multiply((float*)g.Calibration_Matrix,(float*)gForce_transpose,M,N,L,(float*)final_gForce); 
     
      float mag_HIO[3];
      mag_HIO[0] = {g.magRaw[0]-dest1[0]};
      mag_HIO[1] = {g.magRaw[1]-dest1[1]};
      mag_HIO[2] = {g.magRaw[2]-dest1[2]};
      float Norm_mag[3];
      v.Vector_Norm(mag_HIO,Norm_mag);
       
       // Find pitch and yaw during initial pose
//      float roll;
//      float pitch;
//      float yaw;
      g.roll = atan2(Norm[1],Norm[2]);
      //delay(10);
      g.pitch = atan(-Norm[0]/((Norm[1]*sin(g.roll))+(Norm[2]*cos(g.roll))));
      //delay(10);
      g.yaw = atan2(-Norm_mag[2],((Norm_mag[0]*cos(g.pitch))+(Norm_mag[1]*sin(g.pitch))));

      offset=analogRead(potPin);
        
      startLight(green, 1, 2, 4);
 
   }   // end of Calibration step 3 
                    
        else if(calibrationStep>3){  
        static int k=1;
      if (k==1){
         startLight(green,50,9,16); 
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
         k++;
      } else {
         static int holdcount=0;
         val=analogRead(potPin);

         kneeAngle=(140.0/(highEnd-lowEnd))*(val-offset);

         if (digitalRead(buttonPin)==HIGH) {
            while(digitalRead(buttonPin)==HIGH) {
                holdcount++;
                delay(1);
                if (holdcount==1500){     
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
                    holdcount=0;
                    return;

                  
               }
           }
         }

        // Create Rz rotation matrix
        float Rz[3][3];
        Rz[0][0] = {cos(g.yaw)};
        Rz[0][1] = {sin(g.yaw)};
        Rz[0][2] = {0};
        Rz[1][0] = {-sin(g.yaw)};
        Rz[1][1] = {cos(g.yaw)};
        Rz[1][2] = {0};
        Rz[2][0] = {0};
        Rz[2][1] = {0};
        Rz[2][2] = {1};

        float Cal_Final[3][3];
        v.Multiply((float*)Rz,(float*)g.Calibration_Matrix,M,N,N,(float*)Cal_Final);

         // Hard Iron Offset
//        float dest1[3] ;
//        dest1[0] = {95.20};
//        dest1[1] = {218.28};
//        dest1[2] = {293.15};
        
        // Calculating angles 
        
        float phi_rad;
        float theta_rad;
        float psi_rad;  
        float accelX[20];
        float accelY[20];
        float accelZ[20];
        float magnetX[20];
        float magnetY[20];
        float magnetZ[20];
        int i;
        
        for(i=0; i<20; i++){
          accelX[i] = g.gForce[0];
          accelY[i] = g.gForce[1];
          accelZ[i] = g.gForce[2];
          magnetX[i] = g.magRaw[0];//-dest1[0];
          magnetY[i] = g.magRaw[1];//-dest1[1];
          magnetZ[i] = g.magRaw[2];//-dest1[2];
        }
    
        float mean_accX = v.Average(accelX, 20);
        float mean_accY = v.Average(accelY,20);
        float mean_accZ = v.Average(accelZ,20);
        float mean_magX = v.Average(magnetX,20);
        float mean_magY = v.Average(magnetY,20);
        float mean_magZ = v.Average(magnetZ,20);

        //need to normalize mag and acc and hard iron
         float mean_accel[3];
         mean_accel[0] = mean_accX;
         mean_accel[1] = mean_accY;
         mean_accel[2] = mean_accZ;

         float mean_mag[3];
         mean_mag[0] = mean_magX;
         mean_mag[1] = mean_magY;
         mean_mag[2] = mean_magZ;

         float gF_Norm[3];
         float mag_Norm[3];
         v.Vector_Norm(mean_accel,gF_Norm);
         v.Vector_Norm(mean_mag,mag_Norm);

         float gForce_transpose2[3][1];
         float mag_transpose[3][1];
         v.Transpose((float*)gF_Norm, L,N, (float*)gForce_transpose2);
         v.Transpose((float*)mag_Norm, L,N, (float*)mag_transpose);


        float final_gForce [3];
         float final_mag [3];
         //Multiplication function
         v.Multiply((float*)Cal_Final,(float*)gForce_transpose2,M,N,L,(float*)final_gForce);
         v.Multiply((float*)Cal_Final,(float*)mag_transpose,M,N,L,(float*)final_mag);
         
        
        float HIO_norm[3];
        v.Vector_Norm(dest1,HIO_norm);
        delay(100);
        phi_rad = atan2(mean_accY,mean_accZ);
        
        delay(100);
        theta_rad = atan((mean_accX*(-1))/((mean_accY*sin(phi_rad))+(mean_accZ*cos(phi_rad))));
        
        float Bfx;
        float Bfy;
        delay(100);
        Bfx = (((mean_magX-HIO_norm[0])*cos(theta_rad))+((mean_magY-HIO_norm[1])*sin(theta_rad)*sin(phi_rad))+((mean_magZ-HIO_norm[2])*sin(theta_rad)*cos(phi_rad)));
        Bfy = (((mean_magZ-HIO_norm[2])*sin(phi_rad)) - ((mean_magY-HIO_norm[1])*cos(phi_rad)));
        psi_rad = atan2(Bfy,Bfx);
        
        float phi;
        float theta;
        float psi;
        phi = phi_rad*(180/3.1459265);
        theta = theta_rad*(180/3.1459265);
        psi = (psi_rad*(180/3.1459265));

        BTserial.print("Roll: ");
        BTserial.print(phi);
        BTserial.println();
        BTserial.print("Pitch: ");
        BTserial.print(theta);
        BTserial.println();
        BTserial.print("Yaw: ");
        BTserial.print(psi);
        BTserial.println(); 

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
         if (digitalRead(verifyPin)==HIGH) {
            while(digitalRead(verifyPin)==HIGH) {
               verifyCount++;
               delay(1);
               if (verifyCount == 1500){
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
                  holdcount=0;
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
        
            
// Uncomment the following to get xyz data of raw and calibrated
// magnetometer data.
//        float mag[3];
//        mag[0] = mean_magX-HIO_norm[0];
//        mag[1] = mean_magY-HIO_norm[1];
//        mag[2] = mean_magZ-HIO_norm[2];
//        BTserial.print(mag[0]);
//        BTserial.print(",");
//        BTserial.print(mag[1]);
//        BTserial.print(",");
//        BTserial.print(mag[2]);
//        BTserial.print(",");
//        BTserial.print(g.magRaw[0]);
//        BTserial.print(",");
//        BTserial.print(g.magRaw[1]);
//        BTserial.print(",");
//        BTserial.print(g.magRaw[2]);
//        BTserial.print(",");
//        BTserial.println();
//           
        
    } // end of else statement in calibration step 6
  } // calibration step 6 ends
}// IMU_calibration loop ends


