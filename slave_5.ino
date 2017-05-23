// CHECK THIS OUT https://github.com/usgs/geomag-algorithms/blob/master/docs/algorithms/Adjusted.md
#include <SoftwareSerial.h>
#include <Wire.h>
#include "MPU9250.h"
#include "Vector.h"
#include <math.h>


SoftwareSerial BTserial(9, 10); // RX | TX
 //Connect the HC-05 TX to Arduino pin 9 RX. 
 //Connect the HC-05 RX to Arduino pin 10 TX through a voltage divider.


class GForce {
  public:
  float gForce[3] = {0.0, 0.0, 0.0};
  //float Calibration_Matrix[3][3] = {};
  //float cal_gForceX, cal_gForceY, cal_gForceZ;
  float final_gForce[3][0] = {};
  float magRaw[3] = {};
  
   
};
long accelX, accelY, accelZ;
//float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

long magX, magY, magZ;
float flex, ext, abd; 

long asaX, asaY,asaZ;
 
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

int M = 3;
int N = 3;
int R = 3;
int L = 1;
 
void setup() 
{
    BTserial.begin(9600);
    Wire.begin();
    setupMPU();
    
    // HC-06 default BTserial speed for communcation mode is 9600
    //Serial.begin(9600);  
    
}

void loop() {
  recordAccelRegisters();
  recordMagRegisters();
  accel_calibration();
  //Mag_Angles();
  delay(100);
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
    //Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
//    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
//    Serial.print(" I should be "); Serial.println(0x48, HEX);
//    Serial.println("MPU9250 is online...");
    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    //Serial.println("AK8963 initialized for active data mode....");
   
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
  }
}

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
}

GForce g;
Vector v;
void processAccelData(){
  g.gForce[0] = (accelY / 16384.0);
  g.gForce[1] = (accelX / 16384.0); 
  g.gForce[2] = (accelZ / 16384.0)*-1;
    
}

void recordMagRegisters(){ 
 
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){
  myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
   myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
   myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
   myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  g.magRaw[0] = (float)myIMU.mx;
  g.magRaw[1] = (float)myIMU.my;
  g.magRaw[2] = (float)myIMU.mz;
  
   } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
} // void recordMagRegisters()

void accel_calibration() 
{
    
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
            BTserial.println("Lock brace at 0 degrees and press button to continue.");
         } 
        else if(calibrationStep==4){
          BTserial.print("Wave device in figure eight until done!");
        }
        else if(calibrationStep==5){
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
          lowEnd=analogRead(potPin);
    }
    else if(calibrationStep==2){
         highEnd=analogRead(potPin);
    }
    else if(calibrationStep==3){
         offset=analogRead(potPin);
    }  
       else if(calibrationStep==4){
//    uint16_t ii = 0, sample_count = 0;
//    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
//    int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};
// 
//    sample_count = 128;
// 
//    for(ii = 0; ii < sample_count; ii++) {
//        myIMU.readMagData(mag_temp);  // Read the mag data
//        for (int jj = 0; jj < 3; jj++) {
//            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
//            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
//        }
//    delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
//    }
//    // Get hard iron correction
//    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
//    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
//    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
//
//    float dest[3];
//    dest[0] = (float) mag_bias[0]*myIMU.mRes*myIMU.magCalibration[0];  // save mag biases in G for main program
//    dest[1] = (float) mag_bias[1]*myIMU.mRes*myIMU.magCalibration[1];
//    dest[2] = (float) mag_bias[2]*myIMU.mRes*myIMU.magCalibration[2];
//    BTserial.println("mag Calibration Complete");
//    BTserial.println(dest[0]);
//    BTserial.println(dest[1]);
//    BTserial.println(dest[2]);
       }
    // Get soft iron correction estimate
//    mag_scale[0]  = (mag_max[0] – mag_min[0])/2;  // get average x axis max chord length in counts
//    mag_scale[1]  = (mag_max[1] – mag_min[1])/2;  // get average y axis max chord length in counts
//    mag_scale[2]  = (mag_max[2] – mag_min[2])/2;  // get average z axis max chord length in counts
//
//       float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
//    avg_rad /= 3.0;
//    g.dest2[0] = avg_rad/((float)mag_scale[0]);
//    g.dest2[1] = avg_rad/((float)mag_scale[1]);
//    g.dest2[2] = avg_rad/((float)mag_scale[2]);
                       
      //Getting Magnetometer angles
               
        else if(calibrationStep>4){              
        // Hard Iron Offset
        float dest1[3] ;
        dest1[0] = {-156.69};
        dest1[1] = {216.86};
        dest1[2] = {322.01};
        
        // Calculating angles 
        
        float phi;
        float theta;
        float psi;  
        int sampling= 30;
        float accelX[sampling];
        float accelY[sampling];
        float accelZ[sampling];
        float magnetX[sampling];
        float magnetY[sampling];
        float magnetZ[sampling];
        int i;
        
        for(i=0; i<sampling; i++){
          accelX[i] = g.gForce[0];
          accelY[i] = g.gForce[1];
          accelZ[i] = g.gForce[2];
          magnetX[i] = g.magRaw[0];//-dest1[0];
          magnetY[i] = g.magRaw[1];//-dest1[1];
          magnetZ[i] = g.magRaw[2];//-dest1[2];
        }
  
  
        float mean_accX = v.Average(accelX, sampling);
        float mean_accY = v.Average(accelY,sampling);
        float mean_accZ = v.Average(accelZ,sampling);
        float mean_magX = v.Average(magnetX,sampling);
        float mean_magY = v.Average(magnetY,sampling);
        float mean_magZ = v.Average(magnetZ,sampling);

        //need to normalize mag and acc and hard iron
        float accel_norm = sqrt((pow(mean_accX,2)+pow(mean_accY,2)+pow(mean_accZ,2))/3);
        mean_accX = mean_accX/accel_norm;
        mean_accY = mean_accY/accel_norm;
        mean_accZ = mean_accZ/accel_norm;
        float mag_norm = sqrt((pow(mean_magX,2)+pow(mean_magY,2)+pow(mean_magZ,2))/3);
        mean_magX = mean_magX/mag_norm;
        mean_magY = mean_magY/mag_norm;
        mean_magZ = mean_magZ/mag_norm;
        float HIO_norm[3];
        v.Vector_Norm(dest1,HIO_norm);
//        BTserial.print("mean accX: ");
//        BTserial.print(mean_accX);
//        BTserial.println();
//        BTserial.print("mean accY: ");
//        BTserial.print(mean_accY);
//        BTserial.println();
//        BTserial.print("mean accZ: ");
//        BTserial.print(mean_accZ);
//        BTserial.println();
//        BTserial.print("mean magX: ");
//        BTserial.print(mean_magX);
//        BTserial.println();
//        BTserial.print("mean magY: ");
//        BTserial.print(mean_magY);
//        BTserial.println();
//        BTserial.print("mean magZ: ");
//        BTserial.print(mean_magZ);
//        BTserial.println();
        delay(100);
        //phi = atan2(mean_accY,mean_accZ);
        phi = atan2(mean_accX,(sqrt(pow(mean_accY,2)+pow(mean_accZ,2))));
        delay(100);
        //theta = atan((mean_accX*(-1))/((mean_accY*sin(phi))+(mean_accZ*cos(phi))));
        theta = atan(mean_accY/(sqrt(pow(mean_accX,2)+pow(mean_accZ,2))));
        float Bfx;
        float Bfy;
        delay(100);
        Bfx = (((mean_magX-HIO_norm[0])*cos(theta))+((mean_magY-HIO_norm[1])*sin(theta)*sin(phi))+((mean_magZ-HIO_norm[2])*sin(theta)*cos(phi)));
        Bfy = (((mean_magZ-HIO_norm[2])*sin(phi)) - ((mean_magY-HIO_norm[1])*cos(phi)));
        psi = atan2(Bfy,Bfx);
        float phi_deg;
        float theta_deg;
        float psi_deg;
        phi_deg = phi*(180/3.1459265);
        theta_deg = theta*(180/3.1459265);
        psi_deg = (psi*(180/3.1459265));

        BTserial.print("Roll: ");
        BTserial.print(phi_deg);
        BTserial.println();
        BTserial.print("Pitch: ");
        BTserial.print(theta_deg);
        BTserial.println();
        BTserial.print("Yaw: ");
        BTserial.print(psi_deg);
        BTserial.println();     

//        BTserial.print((int)phi_deg);
//        BTserial.print(",");
//        BTserial.print((int)psi_deg);
//        BTserial.print(",");
//        BTserial.print(0);
//        BTserial.print(",");
//        BTserial.println((int)theta_deg);
//        delay(10);

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
        
 } // calibration step 8 ends
} // accelcalibration loop ends
