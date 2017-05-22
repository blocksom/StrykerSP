/*
 * Project:  Senior Project
 * Company:  Stryker
 * Date:     1/23/2017
 * Engineer: Kyle T. Blocksom
 * Purpose:  Control arduino from PC Via Bluetooth
 */

// Connect ...
// arduino>>bluetooth
// D11   >>>  Rx
// D10   >>>  Tx

// Arduino Serial Library
#include <SoftwareSerial.h>

SoftwareSerial Genotronex(10, 11); // RX, TX
int ledpin = 13; // led on D13 will show blink on / off
int BluetoothData; // the data given from Computer

void setup() {
  // put your setup code here, to run once:
  Genotronex.begin(9600);
  Genotronex.println("Bluetooth On: Please press 1 or 0 to blink LED ...");
  pinMode(ledpin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
   if (Genotronex.available()) {
      BluetoothData=Genotronex.read();
      if(BluetoothData=='1'){   // if number 1 pressed ....
         digitalWrite(ledpin,1);
         Genotronex.println("LED  On D13 ON ! ");
      }
      if (BluetoothData=='0'){// if number 0 pressed ....
         digitalWrite(ledpin,0);
         Genotronex.println("LED  On D13 Off ! ");
      }
}
delay(100);// prepare for next data ...
}