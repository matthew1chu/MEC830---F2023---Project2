//MEC830 F2023 Project 1 - Using Ultrasonic Sensor Method
// Matthew Chu (500948484) 

#include <PID_v1.h> // PID Control Library
#include "SR04.h" // Ultrasonic Distance Sensor Control Library
#include <Servo.h>

#define motor_pwma 5
#define motor_pwmb 6
#define motor_directionb 8
#define motor_directiona 7
#define motor_toggle 3
#define speed_Max 255
#define trig 13
#define echo 12

Servo myservo;

// create right and left ultrasonic sensor objects
SR04 sr04 = SR04(echo,trig);
double distance;
const int numAngles = 19;
const int targetAngle = 90;
int i=0;
int distanceArray[numAngles];

void sweepAndStoreData() {
  for (int i = 0; i < numAngles; i++) {
    int angle = i * 10;
    myservo.write(angle);
    delay(500); // Adjust the delay as needed

    // Replace the following line with your distance measurement code
    // For demonstration purposes, a random distance value is generated
    int distance = sr04.Distance();
    distanceArray[i] = distance;
  }
}

int findClosestTarget() {
  int minDistance = distanceArray[0];
  int closestAngle = 0;

  for (int i = 1; i < numAngles; i++) {
    if (distanceArray[i] < minDistance) {
      minDistance = distanceArray[i];
      closestAngle = i * 10;
    }
  }
  return closestAngle;
}


void printDistanceData() {
  Serial.println("Distance data:");
  for (int i = 0; i < numAngles; i++) {
    Serial.print("Angle: ");
    Serial.print(i * 10);
    Serial.print(" degrees, Distance: ");
    Serial.println(distanceArray[i]);
  }
  Serial.println();
}



// initialize I/O pins and begin serial comm.
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
  delay(1000);
  Serial.begin(9600);
}

void loop() {
  // Sweep the servo and store distance values
  sweepAndStoreData();

  // Find the closest target angle
  int closestAngle = findClosestTarget();

  // Orient the robot towards the closest target using PWM control
  //orientRobot(closestAngle);

  // Print the distance data
  printDistanceData();
  
  // Delay for a moment before the next iteration
  delay(500);
}

/*
  distance = sr04.Distance();
  // serial print sensor measurements and PID output for debugging
  Serial.print("distance: ");
  Serial.println(distance);
  delay(200);

  while (i < 3) {

  myservo.write(0);
  //spin CCW
  digitalWrite(motor_toggle, HIGH);
  digitalWrite(motor_directiona,HIGH);
  digitalWrite(motor_directionb, LOW);
  analogWrite(motor_pwma, 50);
  analogWrite(motor_pwmb, 50);
  delay(500);
  digitalWrite(motor_toggle, LOW);
  delay(2000);

  //spin CW
  digitalWrite(motor_toggle, HIGH);
  digitalWrite(motor_directiona, LOW);
  digitalWrite(motor_directionb, HIGH);
  analogWrite(motor_pwma, 50);
  analogWrite(motor_pwmb, 50);
  delay(500);
  digitalWrite(motor_toggle, LOW);
  delay(2000);

  //forward
  digitalWrite(motor_toggle, HIGH);
  digitalWrite(motor_directiona, HIGH);
  digitalWrite(motor_directionb, HIGH);
  analogWrite(motor_pwma, 50);
  analogWrite(motor_pwmb, 50);
  delay(500);
  digitalWrite(motor_toggle, LOW);
  delay(2000);

  //reverse
  digitalWrite(motor_toggle, HIGH);
  digitalWrite(motor_directiona, LOW);
  digitalWrite(motor_directionb, LOW);
  analogWrite(motor_pwma, 50);
  analogWrite(motor_pwmb, 50);
  delay(500);
  digitalWrite(motor_toggle, LOW);
  delay(2000);
  i++; 
  
  }*/
