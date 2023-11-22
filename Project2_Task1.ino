#include <IRremote.h>
#include <Wire.h>

//IR RECEIVER
int RECV_PIN = 9;
IRrecv irrecv(RECV_PIN);
decode_results results; 

//Control Pings for Motors
const int PWMA = 5; //RIGHT SPEED
const int PWMB = 6; //LEFT SPEED
const int AIN = 7; //RIGHT DIRECTION
const int B_IN = 8; //LEFT DIRECTION
const int STBY = 3;

//Timings and MPU6050
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

//MOTOR VALUES
const int maxSpeed = 135; //max PWM value written to motor speed pin.
const int minSpeed = 60; //min PWM value 
float angle; //due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle = 0;
int equilibriumSpeed = 128; //rough estimate desired PWM to motor
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false; //it the car driving forward OR rotate/stationary
bool prevIsDriving = true; //equals isDriving in the previous iteration of void loop()
bool paused = false; //is the program paused

bool iD = false; // Set to false when cart is not currently moving 
unsigned long check = 0xFFA857;
unsigned long lastRead;
unsigned long readValue;

int numInt = 4, iterCount = 0;

//PID Gains;
int kp = 2;

void setup() {
  //MPU6050 Setup
  Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();
  delay(20);
  //MOTOR PIN SETUP
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(B_IN, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  irrecv.enableIRIn(); // Start the receiver
  currentTime = micros(); //Intialize Timing
}

void loop() {

  angle = getAngle();

  //BELOW IS THE SEQUENCE OF THE BOT
  if (irrecv.decode(&results)) { // have we received an IR signal?
    readValue = results.value;

    switch (readValue){
      case 0xFF629D:
        Serial.println("forward");
        isDriving = true;
        lastRead = check;
        delay(100);
        break;
      case 0xFF22DD:
        Serial.println("left");
        targetAngle += 90;
        isDriving = false;
        lastRead = check;
        delay(300);
        break;
      case 0xFFC23D:
        Serial.println("right");
        targetAngle -= 90;
        isDriving = false;
        lastRead = check;
        delay(300);
        break;
      case 0xFFA857:
        Serial.println("stop");
        isDriving = false;
        lastRead = check;
        delay(100);
        break;
      case 0xFF02FD:
        paused = !paused;
        stopCar();
        isDriving = false;
        Serial.println("Pause was pressed, which pauses/unpauses the program");
        lastRead = check;
        delay(100);
        break;
    }
    irrecv.resume();
  }


  //DRIVE STRAIGHT, STOP, OR ROTATE BASED ON SWITCH CASE STATED
  static int count;
  static int countStraight;
  if (count < 6){  
    count ++;
  } else { //runs once after void loop() runs 7 times. void loop runs about every 2.8ms, so this else condition runs every 19.6ms or 50 times/second
    count = 0;
    if (!paused){
      if (isDriving != prevIsDriving){
          leftSpeedVal = equilibriumSpeed;
          countStraight = 0;
          Serial.print("mode changed, isDriving: ");
          Serial.println(isDriving);
      }
      if (isDriving) {
        if (abs(targetAngle - angle) < 3){
          if (countStraight < 20){
            countStraight ++;
          } 
          else {
            countStraight = 0;
            equilibriumSpeed = leftSpeedVal; //to find equilibrium speed, 20 consecutive readings need to indicate car is going straight
            Serial.print("EQUILIBRIUM reached, equilibriumSpeed: ");
            Serial.println(equilibriumSpeed);
          }
        } 
        else {
          countStraight = 0;
        }
        iD = driving(iD);
      } 
      else {
        rotate();
      }
      prevIsDriving = isDriving;
    }
  }
}

float getAngle(){
    // === Read accelerometer (on the MPU6050) data === //
  readAcceleration();
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
  
  // === Read gyroscope (on the MPU6050) data === //
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
  readGyro();
  // Correct the outputs with the calculated error values
  GyroX -= GyroErrorX; //GyroErrorX is calculated in the calculateError() function
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
  //combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  angle = yaw;
  return(angle);
}

bool driving (bool iD){//called by void loop(), which isDriving = true
  int deltaAngle = round(targetAngle - angle); //rounding is neccessary, since you never get exact values in reality
  forward();
  if (!iD){
    iD = true;
    analogWrite(PWMA, equilibriumSpeed);
    analogWrite(PWMB, equilibriumSpeed);
  }
  if (deltaAngle != 0){
    controlSpeed ();
    rightSpeedVal = maxSpeed;
    analogWrite(PWMA, rightSpeedVal);
    analogWrite(PWMB, leftSpeedVal);
  }
  return(iD);
}

void controlSpeed (){//this function is called by driving ()
  int deltaAngle = round(targetAngle - angle);
  int targetGyroZ;
  
  //setting up PID (Proportional Control only: P-Controller)
  if (deltaAngle > 30){
      targetGyroZ = 60;
  } else if (deltaAngle < -30){
    targetGyroZ = -60;
  } else {
    targetGyroZ = kp * deltaAngle;
  }
  
  if (round(targetGyroZ - GyroZ) == 0){
    ;
  } else if (targetGyroZ > GyroZ){
    leftSpeedVal = changeSpeed(leftSpeedVal, -1); //would increase GyroZ
  } else {
    leftSpeedVal = changeSpeed(leftSpeedVal, +1);
  }
}

void rotate (){//called by void loop(), which isDriving = false
  int deltaAngle = round(targetAngle - angle);
  int targetGyroZ;
  if (abs(deltaAngle) <= 1){
    stopCar();
  } else {
    if (angle > targetAngle) { //turn left
      left();
    } else if (angle < targetAngle) {//turn right
      right();
    }
    //setting up PID (Just P-controller)
    if (abs(deltaAngle) > 30){
      targetGyroZ = 60;
    } else {
      targetGyroZ = kp * abs(deltaAngle); //Proportional Control
    }
    
    if (round(targetGyroZ - abs(GyroZ)) == 0){
      ;
    } else if (targetGyroZ > abs(GyroZ)){
      leftSpeedVal = changeSpeed(leftSpeedVal, +1); //would increase abs(GyroZ)
    } else {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1);
    }
    rightSpeedVal = leftSpeedVal;
    analogWrite(PWMA, rightSpeedVal);
    analogWrite(PWMB, leftSpeedVal);
  }
  iD = false;
}   

int changeSpeed (int motorSpeed, int increment){
  motorSpeed += increment;
  if (motorSpeed > maxSpeed){ //to prevent motorSpeed from exceeding 255, which is a problem when using analogWrite
    motorSpeed = maxSpeed;
  } else if (motorSpeed < minSpeed){
    motorSpeed = minSpeed;
  }
  return motorSpeed;
}

//Gyroscope Functions -----------------------------------------------------//
void calculateError() {
  //When this function is called, ensure the car is stationary. See Step 2 for more info
  // Read accelerometer values 200 times
  c = 0;
  while (c < 200) {
    readAcceleration();
    // Sum all readings
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  //Divide the sum by 200 to get the error value, since expected value of reading is zero
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  
  // Read gyro values 200 times
  while (c < 200) {
    readGyro();
    // Sum all readings
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}

void readAcceleration() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050 datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}
//-----------------------------------------------------//
//Changes Direction of the Motor

void stopCar(){
  digitalWrite(STBY, LOW);
}
void forward(){ //drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN, HIGH);
  digitalWrite(B_IN, HIGH);
}
void left(){ //rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN, LOW);
  digitalWrite(B_IN, HIGH);
}
void right(){
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN, HIGH);
  digitalWrite(B_IN, LOW);
}
