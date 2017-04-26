
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

// this constant won't change:
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


void setup() {
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);

  // initialize serial connection
  Serial.begin(9600);
  Serial.println("Please Press Button to Begin Calibration");
  
  // initialize LED ring
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(10);
  startLight(blue, 50, 0, 16);  // flashes blue ring indicating start up success
}

// Function for controll of light ring.
// Parameter 1: color of light
// Parameter 2: time delay between consecutive lights activating
// Parameter 3: LED light to start command 
//              choose 0 through 15
// Parameter 4: LED light to end command +1
//              i.e. to end on LED 15 - Parameter = 16
void startLight(uint32_t color,int t,int I, int limit) {
  //static int I=0;
  if (I==limit)  // terminating condition
        return;
  strip.setPixelColor(I, color);
  strip.show();
  delay(t);
  startLight(color, t, I + 1, limit);
}

void loop() {
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
      // wend from on to off:
      //Serial.println("off");
       if (calibrationStep==1){
        Serial.println("Place brace in closed state and press button to continue.");
 }
       else if(calibrationStep==2){
       Serial.println("Place brace in open state and press button to continue");
 }
       else if(calibrationStep==3){
        Serial.println("Lock brace at 0 degrees and press button to finish calibration.");
 }    
       else if(calibrationStep==4){
        Serial.println("Calibration completed.");
        
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
 //Serial.println(buttonPushCounter); 

 if (calibrationStep==1){
     startLight(red, 1, 0, 2);}
        
 else if(calibrationStep==2){
     highEnd=analogRead(potPin);
     strip.setPixelColor(calibrationStep-2, green);
     strip.show();}
 
 else if(calibrationStep==3){
     offset=analogRead(potPin);
     strip.setPixelColor(calibrationStep-2, green);
     strip.show();}   
      
 else if(calibrationStep>3){
    static int k=1;
    if (k==1){
        startLight(green,50,2,16); 
        k++;}
    else{
        static int holdcount=0;
//        Serial.println(holdcount);      
        val=analogRead(potPin);
        kneeAngle=(125.0/(lowEnd-highEnd))*(val-offset);
        if (digitalRead(buttonPin)==HIGH) {
            while(digitalRead(buttonPin)==HIGH) {
                holdcount++;
                delay(1);
                if (holdcount==1500){
                    startLight(off, 1, 0, 16);
                    delay(100);
                    startLight(blue, 1, 0, 16);
                    delay(400);
                    startLight(off, 1, 0, 16);
                    delay(100);
                    startLight(green, 1, 0, 16);
                    Serial.print("store me: ");
                    Serial.println(kneeAngle);
                    holdcount=0;
                    return;}}}
     Serial.println(kneeAngle);}}}

