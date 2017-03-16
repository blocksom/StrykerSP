/*
===Contact & Support===
Website: http://eeenthusiast.com/
Youtube: https://www.youtube.com/EEEnthusiast
Facebook: https://www.facebook.com/EEEnthusiast/
Patreon: https://www.patreon.com/EE_Enthusiast
Revision: 1.0 (July 13th, 2016)
===Hardware===
- Arduino Uno R3
- MPU-6050 (Available from: http://eeenthusiast.com/product/6dof-mpu-6050-accelerometer-gyroscope-temperature/)
===Software===
- Latest Software: https://github.com/VRomanov89/EEEnthusiast/tree/master/MPU-6050%20Implementation/MPU6050_Implementation
- Arduino IDE v1.6.9
- Arduino Wire library
===Terms of use===
The software is provided by EEEnthusiast without warranty of any kind. In no event shall the authors or 
copyright holders be liable for any claim, damages or other liability, whether in an action of contract, 
tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in 
the software.
*/

#include <Wire.h>
#include "MPU9250.h"
#include "Vector.h"
#include <math.h>

class GForce {
  public:
  float gForceX, gForceY, gForceZ;
  float gForce[3] = {0.0, 0.0, 0.0};
  float Calibration_Matrix[3][3] = {};
  float cal_gForceX, cal_gForceY, cal_gForceZ;
  float final_gForce[3][0] = {};
    
};
long accelX, accelY, accelZ;
//float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

long magX, magY, magZ;
float flex, ext, abd; 

long asaX, asaY,asaZ;

const int  buttonPin = 4;    // the pin that the pushbutton is attached to
const int potPin = 2;    // select the input pin for the potentiometer

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

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
    // initialize the button pin as an input:
  pinMode(buttonPin, INPUT);
  // initialize the LED as an output:
  Serial.begin(9600);
  Serial.println("Please Press Button to Begin Calibration");
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  //recordMagRegisters();
  //  recordMagSensitivity();
//  printData();
  //setup_button();
  accel_calibration();
  //calibrated_accel();
  delay(100);
}

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
void processAccelData(){
  g.gForceX = accelX / 16384.0;
  g.gForceY = accelY / 16384.0; 
  g.gForceZ = accelZ / 16384.0;
  g.gForce[0] = g.gForceX;
  g.gForce[1] = g.gForceY;
  g.gForce[2] = g.gForceZ;
    
}


//float gForce[3] = {g.gForceX, g.gForceY, g.gForceZ};


void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into gyroX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into gyroY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into gyroZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
  
}

// Calibrate Accelerometer and Gyroscope

void accel_calibration() {
  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter++;
      //Serial.println("on");
      //Serial.print("number of button pushes:  ");
      //Serial.println(buttonPushCounter);
    } else {
      // if the current state is LOW then the button
      // went from on to off:
      //Serial.println("off");
       if (calibrationStep==1){
        Serial.println("Place leg such that it is at zero abd/add, zero flex/ext and zero int/ext and press button to continue, wait 1 min");
 }
       else if(calibrationStep==2){
       Serial.println("Press button to finish calibration");
 }
//       else if(calibrationStep==3){
//        Serial.println("Press button to stop taking in data and stop calibration.");
// }    
//       else if(calibrationStep==4){
//        Serial.println("Calibration completed.");
// }   
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state,
  //for next time through the loop
  lastButtonState = buttonState;
  
 calibrationStep=buttonPushCounter;
 delay(50);

Vector v;

 if (calibrationStep==1){
        //float gForce_fake[3] = {1,2,3};
        //v.Print(g.gForce, L,M, "Accelerometer Raw: ");
        float Norm[3];
        v.Vector_Norm(g.gForce,Norm);           //Normalizing the output of accelerometer
        //v.Print(Norm,L,M,"Normalized accelerometer: ");
        float correct_vect[3] = {0,0,1};            //Definining what the output should be
        float Cross[3];
        v.Vector_Cross_Product(Norm,correct_vect,Cross);       // cross product between normalized acc output and what output should be
        //v.Print(Cross,L,M, "Cross Product of norm accel and correct_vect: ");
        float SSC[3][3];
        v.Create_SSC((float*)Cross,SSC);           // Creating SSC matrix
        //v.Print((float*)SSC,M,N, "SSC Matrix: ");
        float identity[3][3];
        v.Identity(identity);              // Creating 3x3 identity matrix
        //v.Print((float*)identity,M,N,"Identity Matrix: ");
        float dot_prod = v.Vector_Dot_Product((float*)Norm,(float*)correct_vect); // dot product b/w normalized accel output and what output should be
        //Serial.print("dot product: ");
        //Serial.print(dot_prod);
        float C = v.Norm_const(Cross);
        //Serial.print("constant: ");
        //Serial.print(C);
        float divisor = pow(C,2);
        //Serial.print("divisor: ");
        //Serial.print(divisor);
        float mult_const = ((1-dot_prod)/divisor);
        //Serial.print("constant for multiplication: ");
        //Serial.print(mult_const);
        float squared_SSC[3][3];
        v.Multiply((float*)SSC,(float*)SSC,M,N,N,(float*)squared_SSC);
        //v.Print((float*)squared_SSC,M,N, "Squared SSC: ");
        float new_SSC[3][3];
        v.Matrix_Const_Mult(squared_SSC,mult_const,new_SSC);
        //v.Print((float*)new_SSC, M,N, "new SSC: ");
        float Add_1[3][3];
        v.Add((float*)identity,(float*) SSC,M,N,(float*)Add_1);
        //float Calibration_Matrix[3][3];
        v.Add((float*)Add_1,(float*) new_SSC,M,N,(float*)g.Calibration_Matrix);
        //v.Print((float*)Calibration_Matrix, M,N, "Calibration Matrix: ");
        float gForce_transpose[3][1];
        v.Transpose((float*)g.gForce, L,N, (float*)gForce_transpose); // Transpose of normalized accel vector
        ///v.Print((float*)Norm_transpose, L,N, "Transpose of nomalized vect: ");
        float final_gForce[3][1];
        v.Multiply((float*)g.Calibration_Matrix,(float*)gForce_transpose,M,N,L,(float*)final_gForce);
        //v.Print((float*)final_gForce,M,L,"Final gForce: ");
        
                   
}

       else if(calibrationStep==2){
       
       
 }
 
       else if(calibrationStep>2){
        //Serial.print(" Calibration is complete");
          float gForce_transpose2[3][1];
          v.Transpose((float*)g.gForce, L,N, (float*)gForce_transpose2);
          v.Multiply((float*)g.Calibration_Matrix,(float*)gForce_transpose2,M,N,L,(float*)g.final_gForce);
          v.Print((float*)g.final_gForce,M,L,"Final gForce: ");
          Serial.print("Ax out: ");
          Serial.print(gForce_transpose2[0][0]);
          Serial.println();
          Serial.print("Ay out: ");
          Serial.print(gForce_transpose2[1][0]);
          Serial.println();
          Serial.print("Az out: ");
          Serial.print(gForce_transpose2[2][0]);
          Serial.println();
          float numerator_theta = gForce_transpose2[0][0];
          float denom_theta_part1 = pow(gForce_transpose2[1][0],2)+pow(gForce_transpose2[2][0],2);
          float denom_theta_final = sqrt(denom_theta_part1);
          float theta_rad = atan2(numerator_theta,denom_theta_final);
          float theta = round(theta_rad*180/3.1459265); // rad to deg
          Serial.print("Theta: ");
          Serial.print(theta);
          Serial.println();
          float numerator_psi = gForce_transpose2[1][0];
          float denom_psi_part1 = pow(gForce_transpose2[0][0],2)+pow(gForce_transpose2[2][0],2);
          float denom_psi_final = sqrt(denom_psi_part1);
          float psi_rad = atan2(numerator_psi,denom_psi_final);
          float psi = round(psi_rad*180/3.1459265); // rad to deg
          Serial.print("Psi: ");
          Serial.print(psi);
          Serial.println();
          float denom_phi = gForce_transpose2[2][0];
          float numerator_phi_part1 = pow(gForce_transpose2[1][0],2)+pow(gForce_transpose2[0][0],2);
          float numerator_phi_final = sqrt(numerator_phi_part1);
          float phi_rad = atan2(numerator_phi_final,denom_phi);
          float phi = round(phi_rad*180/3.1459265); // rad to deg
          Serial.print("Phi: ");
          Serial.print(phi);
          Serial.println();
 }

}




//MPU9250 myIMU;
//
//void recordMagRegisters(){
//   byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
//   #define SerialDebug true // Set to true to get Serial output for debugging
//  if (c == 0x71) // WHO_AM_I should always be 0x68
//  {
//    // Get magnetometer calibration from AK8963 ROM
//    myIMU.initAK8963(myIMU.magCalibration);
//    // Initialize device for active mode read of magnetometer
//    Serial.println("AK8963 initialized for active data mode....");
//    if (SerialDebug)
//    {
//      //  Serial.println("Calibration values: ");
//      //Serial.print("X-Axis sensitivity adjustment value ");
//      //Serial.println(myIMU.magCalibration[0], 2);
//      //Serial.print("Y-Axis sensitivity adjustment value ");
//      //Serial.println(myIMU.magCalibration[1], 2);
//      //Serial.print("Z-Axis sensitivity adjustment value ");
//      //Serial.println(myIMU.magCalibration[2], 2);
//    }
//    else
//  {
//    Serial.print("Could not connect to MPU9250: 0x");
//    Serial.println(c, HEX);
//    while(1) ; // Loop forever if communication doesn't happen
//  }
//  }
// 
//  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){
//  myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
//    myIMU.getMres();
// // User environmental x-axis correction in milliGauss, should be
//    // automatically calculated
//    myIMU.magbias[0] = +470.;
//    // User environmental x-axis correction in milliGauss TODO axis??
//    myIMU.magbias[1] = +120.;
//    // User environmental x-axis correction in milliGauss
//    myIMU.magbias[2] = +125.;
//
//    // Calculate the magnetometer values in milliGauss
//    // Include factory calibration per data sheet and user environmental
//    // corrections
//    // Get actual magnetometer value, this depends on scale being set
//    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
//               myIMU.magbias[0];
//    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
//               myIMU.magbias[1];
//    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
//               myIMU.magbias[2];
//    Serial.print("mx = "); Serial.print((int)myIMU.mx);
//    Serial.print("my = "); Serial.print((int)myIMU.my);
//    Serial.print("mz = "); Serial.print((int)myIMU.mz);
//  }
//}
//
// Printing Final Data
// void printData() {
//  //Serial.print("Gyro (deg)");
//  //Serial.print(" X=");
//  //Serial.print(rotX);
//  //Serial.print(";");
//  //Serial.print(" Y=");
//  //Serial.print(rotY);
//  //Serial.print(";");
//  //Serial.print(" Z=");
//  //Serial.print(rotZ);
//  //Serial.print(";");
//  Serial.print(" Accel (g)");
//  Serial.print(" X=");
//  Serial.print(gForceX);
//  Serial.print(";");
//  Serial.print(" Y=");
//  Serial.print(gForceY);
//  Serial.print(";");
//  Serial.print(" Z=");
//  Serial.println(gForceZ);
//  Serial.print(";");
////  Serial.print(" Flex/Ext: ");
////  Serial.print(magZ);
////  //Serial.print(";");
////  Serial.print("Int/Ext: ");
////  Serial.print(magY);
////  //Serial.print(";");
////  Serial.print("Abd/Add: ");
////  Serial.print(magZ);
//  }

