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
  float gForceX, gForceY, gForceZ;
  float gForce[3] = {0.0, 0.0, 0.0};
  float Calibration_Matrix[3][3] = {};
  float cal_gForceX, cal_gForceY, cal_gForceZ;
  float final_gForce[3][0] = {};
  float magRaw[3] = {};
  float magRaw_Norm[3] = {};
  float mx_xy_0, my_xy_0; // mx_xy_180, my_xy_180;
  float my_yz_0, mz_yz_0; // my_yz_180, mz_yz_180;
  float mx_xz_0, mz_xz_0; // mx_xz_180, mz_xz_180;
  float mag_offset_z, mag_offset_x, mag_offset_y;    
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
const int buttonPin = 4;    // the pin that the pushbutton is attached to
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
 
void setup() 
{
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
  accel_calibration();
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
MPU9250 myIMU;
void recordMagRegisters(){
 
 
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){
  myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
 // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

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
  Vector v;
  
  v.Vector_Norm(g.magRaw,g.magRaw_Norm);
  //v.Print((float*)g.magRaw,M,L," magRaw 1: ");
//    BTserial.print("mx = "); BTserial.print((int)myIMU.mx);
//    BTserial.print("my = "); BTserial.print((int)myIMU.my);
//    BTserial.print("mz = "); BTserial.print((int)myIMU.mz);

  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
} // void recordMagRegisters()

void accel_calibration() 
{
   // Vector v;
    //GForce g;
  
    //BTserial.println("Bluetooth Initialize..."); 
    //BTserial.println("Bluetooth Test..."); 
    //delay(1000);   

    static int saved[6];
    static int savedCount = 0, loopCount = 0, flag = 0;
    int counter = 0, i = 0;
       
    i = 0;
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
         BTserial.println("Place xy plane face up");   
 }
        else if (calibrationStep==5){
         BTserial.println("Rotate xy plane 180 degrees");   
 }
        else if (calibrationStep==6){
         BTserial.println("Place yz plane face up");   
 }
        else if (calibrationStep==7){
         BTserial.println("Rotate yz plane 180 degrees");   
 }
        else if (calibrationStep==8){
         BTserial.println("Place xz plane face up");   
 }
        else if (calibrationStep==9){
         BTserial.println("Rotate xz plane 180 degrees");   
 }   
        else if (calibrationStep==10){
        BTserial.println("Place leg such that it is at zero abd/add, zero flex/ext and zero int/ext and press button to finish calibration.");
 }
         else if(calibrationStep==11){
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
          startLight(red, 1, 0, 9);
    }
    else if(calibrationStep==2){
         highEnd=analogRead(potPin);
         startLight(green, 1, 0, 1);
    }
    else if(calibrationStep==3){
         offset=analogRead(potPin);
         startLight(green, 1, 1, 2);
    }  
       else if(calibrationStep==4){
          //Serial.print(" Place with positive xy axis");
          g.mx_xy_0 = g.magRaw[0];
          g.my_xy_0 = g.magRaw[1];
          startLight(green, 1, 2, 3);
 }
       else if(calibrationStep==5){
          //Serial.print("Rotate 180 degrees");
          float mx_xy_180 = g.magRaw[0];
          float my_xy_180 = g.magRaw[1];

          // this takes magnitude of first reading and magnitude of second, then adds
          // and divides by 2

          float R1_xy = sqrt(pow(g.mx_xy_0,2)+pow(g.my_xy_0,2));
          float R2_xy = sqrt(pow(mx_xy_180,2)+pow(my_xy_180,2));
          g.mag_offset_z = (R1_xy+R2_xy)/2;
          startLight(green, 1, 3, 4);
 }

       else if (calibrationStep==6){
          //Serial.print(" Place with positive yz axis");
          g.my_yz_0 = g.magRaw[1];
          g.mz_yz_0 = g.magRaw[2];
          startLight(green, 1, 4, 5);
 }
       else if (calibrationStep==7){
        //Serial.print("Rotate 180 degrees");
        float my_yz_180 = g.magRaw[1];
        float mz_yz_180 = g.magRaw[2];

        float R1_yz = sqrt(pow(g.my_yz_0,2)+pow(g.mz_yz_0,2));
        float R2_yz = sqrt(pow(my_yz_180,2)+pow(mz_yz_180,2));
        g.mag_offset_x = (R1_yz+R2_yz)/2;
        startLight(green, 1, 5, 6);
 }

       else if (calibrationStep==8){
        //Serial.print(" Place with positive xy axis");
        float mx_xz_0 = g.magRaw[0];
        float mz_xz_0 = g.magRaw[2];
        startLight(green, 1, 6, 7);
 }
        else if (calibrationStep==9){
        //Serial.print("Rotate 180 degrees");
        float mx_xz_180 = g.magRaw[0];
        float mz_xz_180 = g.magRaw[2];

        float R1_xz = sqrt(pow(g.mx_xz_0,2)+pow(g.mz_xz_0,2));
        float R2_xz = sqrt(pow(mx_xz_180,2)+pow(mz_xz_180,2));
        g.mag_offset_x = (R1_xz+R2_xz)/2;  
        startLight(green, 1, 7, 8);        
 }     
    else if (calibrationStep==10){
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
        //BTserial.print("dot product: ");
        //BTserial.print(dot_prod);
        float C = v.Norm_const(Cross);
        //BTserial.print("constant: ");
        //BTserial.print(C);
        float divisor = pow(C,2);
        //BTserial.print("divisor: ");
        //BTserial.print(divisor);
        float mult_const = ((1-dot_prod)/divisor);
        //BTserial.print("constant for multiplication: ");
        //BTserial.print(mult_const);
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
        startLight(green, 1, 8, 9); 
                   
}

 else if (calibrationStep>10) {
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
        k++;}
    else{
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
                       //BTserial.println(saved[savedCount]);
                       savedCount++;
                       if (loopCount != 6)
                          loopCount = savedCount;
                    } else {
                       if (savedCount == 6)
                          savedCount = 0;
                    
                       BTserial.print("Overwriting Saved Position: ");
                       BTserial.println(savedCount);
                       flag = 1;
                       saved[savedCount] = kneeAngle;
                       BTserial.println(saved[savedCount]);
                       //savedCount++;
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
          BTserial.print("Theta accel: "); // 
          BTserial.print(theta);
          BTserial.println();
          float numerator_psi = final_gForce[0][1];
          float denom_psi_part1 = pow(final_gForce[0][0],2)+pow(final_gForce[0][2],2);
          float denom_psi_final = sqrt(denom_psi_part1);
          float psi_rad = atan2(numerator_psi,denom_psi_final);
          float psi = round(psi_rad*180/3.1459265); // rad to deg
          BTserial.print("Psi accel: ");
          BTserial.print(psi);
          BTserial.println();
          float denom_phi = final_gForce[0][2];
          float numerator_phi_part1 = pow(final_gForce[0][1],2)+pow(final_gForce[0][0],2);
          float numerator_phi_final = sqrt(numerator_phi_part1);
          float phi_rad = atan2(numerator_phi_final,denom_phi);
          float phi = round(phi_rad*180/3.1459265); // rad to deg
          BTserial.print("Phi accel: ");
          BTserial.print(phi);
          BTserial.println();
          BTserial.print("Knee Angle: ");
          BTserial.print(kneeAngle);
          BTserial.println();
          BTserial.print("First check saved[0] is: ");
          BTserial.println(saved[0]);

    
          // read the pushbutton input pin:
          verifyState = digitalRead(verifyPin);
          // compare the buttonState to its previous state
          if (verifyState != lastVerifyState) {
            // if the state has changed, increment the counter
            if (verifyState == HIGH) {
              verifyPushCounter++;
            } else ;
            // Delay a little bit to avoid bouncing
            //delay(20);
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
                    startLight(white, 1, 0, 16);
                    BTserial.print("store me: ");
                    BTserial.println(kneeAngle);
                    holdcount=0;
                    return;                       
                }
            }
          }        

          //BTserial.print("Second check saved[0] is: ");
          //BTserial.println(saved[0]);

          BTserial.print("Number pushes: ");
          BTserial.println(verifyPushCounter);        

          BTserial.print("Loop Count: ");
          BTserial.println(loopCount);      

          if (verifyFlag) {
                startLight(off, 1, 4, 5); 
                startLight(off, 1, 11, 12);
                startLight(dark_blue, 1, 5, 5 + loopCount); 
                if (verifyPushCounter == -1)
                    verifyPushCounter = 0;                 
                if (saved[verifyPushCounter] != -1) {
                     startLight(yellow, 1, 5 + verifyPushCounter, 6 + verifyPushCounter);
                   /*startLight(yellow, 1, 5 + loopCount, 5 + verifyPushCounter - 1);
                   startLight(off, 1, 5 + verifyPushCounter, 11);       
                   startLight(yellow, 1, 5 + verifyPushCounter, 11);*/
                  //BTserial.println("in loop");
                   if ((abs(kneeAngle - saved[verifyPushCounter]) <= 10) && (abs(kneeAngle - saved[verifyPushCounter]) > 3)) {
                      startLight(yellow, 1, 0, 4);
                      startLight(yellow, 1, 12, 16);
                   }
                   else if ((abs(kneeAngle - saved[verifyPushCounter]) <= 3)) {
                      startLight(green, 1, 0, 4);
                      startLight(green, 1, 12, 16);
                   }
                   else {
                      startLight(red, 1, 0, 4);
                      startLight(red, 1, 12, 16);
                   }
                }
                // counter++;
          }
          
          //}
          

//          float finalAngles[4] = {};
//          finalAngles[0] = kneeAngle;
//          finalAngles[1] = theta;
//          finalAngles[2] = psi;
//          finalAngles[3] = phi;

      //Getting Magnetometer angles
          float magX = g.magRaw_Norm[0]+g.mag_offset_x;
          float magY = g.magRaw_Norm[1]+g.mag_offset_y;
          float magZ = g.magRaw_Norm[2]+g.mag_offset_z;
          float numerator_theta_mag = magX;
          float denom_theta_part1_mag = pow(magY,2)+pow(magZ,2);
          float denom_theta_final_mag = sqrt(denom_theta_part1_mag);
          float theta_rad_mag = atan2(numerator_theta_mag,denom_theta_final_mag);
          float theta_mag = round(theta_rad_mag*180/3.1459265); // rad to deg
          Serial.print("Theta mag: "); // 
          Serial.print(theta_mag);
          Serial.println();
          float numerator_psi_mag = magY;
          float denom_psi_part1_mag = pow(magX,2)+pow(magZ,2);
          float denom_psi_final_mag = sqrt(denom_psi_part1_mag);
          float psi_rad_mag = atan2(numerator_psi_mag,denom_psi_final_mag);
          float psi_mag = round(psi_rad_mag*180/3.1459265); // rad to deg
          Serial.print("Psi mag: ");
          Serial.print(psi_mag);
          Serial.println();
          float denom_phi_mag = magZ;
          float numerator_phi_part1_mag = pow(magX,2)+pow(magY,2);
          float numerator_phi_final_mag = sqrt(numerator_phi_part1_mag);
          float phi_rad_mag = atan2(numerator_phi_final_mag,denom_phi_mag);
          float phi_mag = round(phi_rad_mag*180/3.1459265); // rad to deg
          Serial.print("Phi mag: ");
          Serial.print(phi_mag);
          Serial.println();
          
         // BTserial.println(kneeAngle);
//          BTserial.println(theta);
//          BTserial.println(psi);
//          BTserial.println(phi);
        //delay(10);
 }}
// BTserial.println("calibration step: ");
// BTserial.println(calibrationStep);
}
