#include <Servo.h>
#include <IRremote.h>
#include <IR.h


// Define motor pins
int motor_toggle = 3;
int motor_directiona = 7;
int motor_directionb = 8;
int motor_pwma = 5;
int motor_pwmb = 6;

// Define servo and ultrasonic sensor pins
Servo myservo;
int trig = 13;
int echo = 12;

// Define IR remote control pins
int irReceiverPin = 9;
IRrecv irrecv(irReceiverPin);
decode_results results;

void setup() {
  myservo.write(90);
  delay(500);
  myservo.attach(10);

  pinMode(motor_pwma, OUTPUT);
  pinMode(motor_pwmb, OUTPUT);
  pinMode(motor_directiona, OUTPUT);
  pinMode(motor_directionb, OUTPUT);
  pinMode(motor_toggle, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  irrecv.enableIRIn();  // Start the IR receiver
  Serial.begin(9600);
}

// Function to spin CCW
void spinCCW() {
  digitalWrite(motor_toggle, HIGH);
  digitalWrite(motor_directiona, HIGH);
  digitalWrite(motor_directionb, LOW);
  analogWrite(motor_pwma, 50);
  analogWrite(motor_pwmb, 50);
  delay(500);
  digitalWrite(motor_toggle, LOW);
  delay(2000);
}

// Function to spin CW
void spinCW() {
  digitalWrite(motor_toggle, HIGH);
  digitalWrite(motor_directiona, LOW);
  digitalWrite(motor_directionb, HIGH);
  analogWrite(motor_pwma, 50);
  analogWrite(motor_pwmb, 50);
  delay(500);
  digitalWrite(motor_toggle, LOW);
  delay(2000);
}

// Function to move forward
void moveForward() {
  digitalWrite(motor_toggle, HIGH);
  digitalWrite(motor_directiona, HIGH);
  digitalWrite(motor_directionb, HIGH);
  analogWrite(motor_pwma, 50);
  analogWrite(motor_pwmb, 50);
  delay(500);
  digitalWrite(motor_toggle, LOW);
  delay(2000);
}

// Function to move backward
void moveBackward() {
  digitalWrite(motor_toggle, HIGH);
  digitalWrite(motor_directiona, LOW);
  digitalWrite(motor_directionb, LOW);
  analogWrite(motor_pwma, 50);
  analogWrite(motor_pwmb, 50);
  delay(500);
  digitalWrite(motor_toggle, LOW);
  delay(2000);
}

void loop() {
  if (irrecv.decode(&results)) {
    // If a button is pressed on the remote, perform the corresponding action
    switch (results.value) {
      case button code
        spinCCW();
        break;
      case button code
        spinCW();
        break;
      case button code
        moveForward();
        break;
      case button code
        moveBackward();
        break;
      //to turn right
      case button code
        spinCW();
        moveForward();
        break;
      //to turn left
      case button code
        spinCCW();
        moveForward();
        break;
    }
    }
    }
    
    irrecv.resume();  // Receive the next value
  }
}
