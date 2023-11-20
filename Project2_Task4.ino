#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

MPU6050 mpu;

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

// Create PID instance
PID pid(&error, &output, &setpoint, kp, ki, kd, DIRECT);

// Time variables
unsigned long startTime;
unsigned long elapsedTime;
const unsigned long moveDuration = 2400;  // Move forward for 2000 milliseconds
const unsigned int numIterations = 4;  // Number of move-forward and turn iterations
unsigned int iterationCount = 0;
bool isMovingForward = true;  // State flag to indicate forward movement

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


  // Record the start time
  startTime = millis();
}

void loop() {
  // Get the elapsed time
  elapsedTime = millis() - startTime;

  if (iterationCount < numIterations) {
    if (isMovingForward) {
      // Move forward using PID control
      moveForward();

      // Check if the move duration has elapsed
      if (elapsedTime >= moveDuration) {
        // Stop the car by disabling the motor driver
        digitalWrite(STBY, LOW);
        isMovingForward = false;  // Change state to indicate the end of forward movement
        startTime = millis();  // Reset the start time for the turn
      }
    } else {
      // Perform a 90-degree turn using the gyroscope
      turn90Degrees();

      // Increment the iteration count after completing a turn
      iterationCount++;

      // Set the state back to forward movement for the next iteration
      isMovingForward = true;
      startTime = millis();  // Reset the start time for the next forward movement
    }
  } else {
    // End the loop after the specified number of iterations
    digitalWrite(STBY, LOW);  // Disable the motors
    while (true) {
      // Do nothing or add any additional logic if needed
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
