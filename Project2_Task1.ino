//www.elegoo.com
//2023.05.06

#include "Stepper.h"
#include "IRremote.h"

#define PWMA 5
#define PWMB 6
#define AIN 7
#define B_IN 8
#define STBY 3
;

int receiver = 9; // Signal Pin of IR receiver to Arduino Digital Pin 12


IRrecv irrecv(receiver);    // create instance of 'irrecv'
uint32_t last_decodedRawData = 0;//vairable uses to store the last decodedRawData

void setup()
{
  irrecv.enableIRIn(); // Start the receiver
  pinMode (PWMA, OUTPUT);
  pinMode (PWMB, OUTPUT);
  pinMode (AIN, OUTPUT);
  pinMode (B_IN, OUTPUT);
  pinMode (STBY, OUTPUT);
}
// Function to spin CCW
void spinCCW() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN, HIGH);
  digitalWrite(B_IN, LOW);
  analogWrite(PWMA, 50);
  analogWrite(PWMB, 50);
  delay(1200);
  digitalWrite(STBY, LOW);
}

// Function to spin CW
void spinCW() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN, LOW);
  digitalWrite(B_IN, HIGH);
  analogWrite(PWMA, 50);
  analogWrite(PWMB, 50);
  delay(1320);
  digitalWrite(STBY , LOW);
}

// Function to move forward
void moveForward() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN, HIGH);
  digitalWrite(B_IN, HIGH);
  analogWrite(PWMA, 103);
  analogWrite(PWMB, 110);
  delay(3800);
  digitalWrite(STBY, LOW);
}

// Function to move backward
void moveBackward() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN, LOW);
  digitalWrite(B_IN, LOW);
  analogWrite(PWMA, 110);
  analogWrite(PWMB, 110);
}
void loop()
{
  if (irrecv.decode()) // have we received an IR signal?
  {
    // Check if it is a repeat IR code
    if (irrecv.decodedIRData.flags)
    {
      //set the current decodedRawData to the last decodedRawData
      irrecv.decodedIRData.decodedRawData = last_decodedRawData;
    }
    switch (irrecv.decodedIRData.decodedRawData)
    {

      case 0xB946FF00: // VOL+ button pressed
        moveForward();
        break;

      case 0xEA15FF00: // VOL- button pressed
        moveBackward();
        break;

      case 0xBC43FF00:
        spinCW();
        break;

      case 0xBB44FF00:
        spinCCW();
        break;

      case 0xBF40FF00:
        digitalWrite(STBY, LOW);
        break;

    }
  }
  irrecv.resume();
}/* --end main loop -- */
