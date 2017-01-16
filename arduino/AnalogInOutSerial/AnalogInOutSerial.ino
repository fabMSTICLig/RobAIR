/*
  Analog input, analog output, serial output
 
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.
 
 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground
 
 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
 
 */

// These constants won't change.  They're used to give names
// to the pins used:
const int an0 = A0;  // Analog input pin that the potentiometer is attached to
const int an2 = A1;
const int an4 = A2;  // Analog input pin that the potentiometer is attached to
const int an5 = A3;

int sensorValue = 0;        // value read from the pot
int sensorF = 0;        // value read from the pot
int sensorR = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  sensorF = analogRead(A0);
  sensorR = analogRead(A1);
  sensorF = analogRead(A2);
  sensorR = analogRead(A3);
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(A0);    
  Serial.print("A0 = " );                       
  Serial.println(sensorValue);   
  
  sensorValue = analogRead(A1);    
  Serial.print("A1 = " );                       
  Serial.println(sensorValue);      
  sensorValue = analogRead(A2);    
  Serial.print("A2 = " );                       
  Serial.println(sensorValue);      
  sensorValue = analogRead(A3);    
  Serial.print("A3 = " );                       
  Serial.println(sensorValue);                   
  Serial.println();                        
  Serial.println();      

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(400);                     
}
