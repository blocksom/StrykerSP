#include <SoftwareSerial.h>
#include <Wire.h>
#include "MPU9250.h"
#include "Vector.h"
#include <math.h>

SoftwareSerial BTserial(9, 10); // RX | TX
// Connect the HC-05 TX to Arduino pin 9 RX. 
// Connect the HC-05 RX to Arduino pin 10 TX through a voltage divider.

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
    Serial.begin(9600);
    Wire.begin();
    setupMPU();
    
    // HC-06 default serial speed for communcation mode is 9600
    BTserial.begin(9600);  
}

void loop() {
  recordAccelRegisters();
  //recordGyroRegisters();
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
Vector v;
void processAccelData(){
  g.gForceX = accelX / 16384.0;
  g.gForceY = accelY / 16384.0; 
  g.gForceZ = accelZ / 16384.0;
  g.gForce[0] = g.gForceX;
  g.gForce[1] = g.gForceY;
  g.gForce[2] = g.gForceZ;
  //v.Print((float*)g.gForce,M,L,"Raw gForce: ");
  
    
}

void accel_calibration() 
{
   // Vector v;
    //GForce g;
  
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
            BTserial.println("Lock brace at 0 degrees and press button to continue.");
         }    
        else if (calibrationStep==4){
        BTserial.println("Place leg such that it is at zero abd/add, zero flex/ext and zero int/ext and press button to finish calibration.");
 }
         else if(calibrationStep==5){
            BTserial.println("Calibration completed.");
         }   
      }
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
    else if (calibrationStep==4){
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

 else if (calibrationStep>4) {
        
        val=analogRead(potPin);
        
        kneeAngle=(140.0/(highEnd-lowEnd))*(val-offset);
        
        //BTserial.println(kneeAngle);
        float gForce_transpose2[3][1];
          float gF_Norm[3];
          v.Vector_Norm(g.gForce,gF_Norm);
          
          v.Transpose((float*)gF_Norm, L,N, (float*)gForce_transpose2);
          float final_gForce [3][0];
         
          //Multiplication function
          v.Multiply((float*)g.Calibration_Matrix,(float*)gForce_transpose2,M,N,L,(float*)final_gForce);
                   
          //v.Print((float*)g.final_gForce,M,L,"Final gForce: ");
          float numerator_theta = final_gForce[0][0];
          
          float denom_theta_part1 = pow(final_gForce[0][1],2)+pow(final_gForce[0][2],2);
          float denom_theta_final = sqrt(denom_theta_part1);
          float theta_rad = atan2(numerator_theta,denom_theta_final);
          float theta = round(theta_rad*180/3.1459265); // rad to deg
//          Serial.print("Theta accel: "); // 
//          Serial.print(theta);
//          Serial.println();
          float numerator_psi = final_gForce[0][1];
          float denom_psi_part1 = pow(final_gForce[0][0],2)+pow(final_gForce[0][2],2);
          float denom_psi_final = sqrt(denom_psi_part1);
          float psi_rad = atan2(numerator_psi,denom_psi_final);
          float psi = round(psi_rad*180/3.1459265); // rad to deg
//          Serial.print("Psi accel: ");
//          Serial.print(psi);
//          Serial.println();
          float denom_phi = final_gForce[0][2];
          float numerator_phi_part1 = pow(final_gForce[0][1],2)+pow(final_gForce[0][0],2);
          float numerator_phi_final = sqrt(numerator_phi_part1);
          float phi_rad = atan2(numerator_phi_final,denom_phi);
          float phi = round(phi_rad*180/3.1459265); // rad to deg
//          Serial.print("Phi accel: ");
//          Serial.print(phi);
//          Serial.println();

//          float finalAngles[4] = {};
//          finalAngles[0] = kneeAngle;
//          finalAngles[1] = theta;
//          finalAngles[2] = psi;
//          finalAngles[3] = phi;
          
          BTserial.println(kneeAngle);
          BTserial.println(theta);
          BTserial.println(psi);
          BTserial.println(phi);
        delay(10);
 }
// Serial.println("calibration step: ");
// Serial.println(calibrationStep);
}




