#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

MPU6050 mpu;
const int trig = 13;
const int echo = 12;
const int PWMA = 5;
const int PWMB = 6;
const int AIN = 7;
const int B_IN = 8;
const int STBY = 3;
// Define PID parameters
double kp = 2;
double ki = 1;
double kd = 1;
double error, integral, lastError, output;
double setpoint = 1010;  // Use the observed bias as the initial setpoint
long duration;
float distance;

// Create PID instance
PID pid(&error, &output, &setpoint, kp, ki, kd, DIRECT);

// Time variables
unsigned long startTime;
unsigned long elapsedTime;
const unsigned long moveDuration = 2400;  // Move forward for 2000 milliseconds
const unsigned int numIterations = 4;  // Number of move-forward and turn iterations
unsigned int iterationCount = 0;
bool isMovingForward = true;  // State flag to indicate forward movement
unsigned long interval1 = 100;  // Interval for Function 1 in milliseconds
unsigned long interval2 = 100;  // Interval for Function 2 in milliseconds

unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Initialize MPU6050
  mpu.initialize();
  mpu.setFullScaleGyroRange(0);  // Adjust the gyro scale if needed
  mpu.setFullScaleAccelRange(0);

  calibrateMPU6050();

  // Set the initial setpoint for the PID controller
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);  // Adjust based on your motor driver specifications
  pid.SetSampleTime(1);

  Serial.println("MPU6050 initialized");
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(B_IN, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis1 >= interval1){
      moveForward();
  }

  if (currentMillis - previousMillis2 >= interval2){
    distance = calculateDistance();
    if (distance < 15) {
      analogWrite (PWMA, 0);
      analogWrite (PWMB, 0);
      digitalWrite (STBY, LOW);
    }
  }

}

void moveForward() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.println(gz);

  error = gz - setpoint;
  pid.Compute();
  int motorSpeed = constrain(int(output), -255, 255);

  // Adjust motor speeds based on the PID output
  analogWrite(PWMA, motorSpeed);
  analogWrite(PWMB, motorSpeed);

  // Adjust motor directions based on the error
  if (error < 0) {
    analogWrite(PWMA, motorSpeed - 20);
  } else {
    analogWrite(PWMB, motorSpeed - 20);
  }

  digitalWrite(AIN, HIGH);
  digitalWrite(B_IN, HIGH);
}

void turn90Degrees() {
  delay (100);
  digitalWrite(STBY, HIGH);  // Keep the motors enabled

  digitalWrite(AIN, LOW);
  digitalWrite(B_IN, HIGH);
  analogWrite(PWMA, 100);
  analogWrite(PWMB, 100);

  delay(850);

  // Stop the motors
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}


void calibrateMPU6050() {
  Serial.println("Calibrating MPU6050... Please make sure the sensor is stationary.");

  // Variables for calibration
  const int numSamples = 1000;
  long gyroZOffset = 0;

  // Collect samples for calibration
  for (int i = 0; i < numSamples; ++i) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    gyroZOffset += gz;
    delay(5);  // Add a short delay between samples
  }

  // Calculate average value for calibration
  gyroZOffset /= numSamples;

  // Set the calibration offset
  mpu.setZGyroOffset(gyroZOffset);

  Serial.println("Calibration complete.");
}

float calculateDistance (){
  digitalWrite (trig, LOW);
  delay(2);
  digitalWrite (trig, HIGH);
  delay(10);
  digitalWrite(trig, LOW);
  duration = pulseIn (echo, HIGH);
  distance = duration*0.034/2;
  return distance;
}
