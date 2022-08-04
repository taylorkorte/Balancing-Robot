/*Robotics HW4: Balancing Bike
 *ESP Code
 *11/1/21
 */

#include <ESP32Servo.h> //Servo Package
#include "Adafruit_VL53L0X.h" //LiDAR Package
#include <Adafruit_LSM6DSOX.h> //Accelerometer Package - Basic Code from Adafruit
#include <math.h>

//Initialize Servo
Servo myservo;  // create servo object to control a servo
int servoPin = 4;
int turnAngle = 90;

//Initialize Accelerometer
Adafruit_LSM6DSOX sox;

//Initialize Lidar
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float LiDAR_Angle = 0;

//Initialize variables
float bike_height = 119;
int roll = 0;
int pitch = 0;
float c = 0;
int mappedAngleX = 0;
float angle = 0;
float Kp = 1;

void setup() {

  //Set up serial port
  Serial.begin(115200);
  while (!Serial) {
      delay(1);
  }

  //Lidar setup
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
 
  //Accelerometer setup
  Serial.println("Adafruit LSM6DSOX test!");
  if (!sox.begin_I2C()) {
    while (1) {
      delay(10);
    }
  }
  Serial.println("LSM6DSOX Found!");
  AccelerometerSetup();
  
  //Servo setup: allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50); //standard 50 hz servo
  myservo.attach(servoPin, 500, 2400); //attaches the servo on pin 18 to the servo object
  delay(500);
}


void loop() {

  //Get a sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  //Get acceleration and gyro data
  float accelX = accel.acceleration.x;
  float accelY = accel.acceleration.y;
  float accelZ = accel.acceleration.z;
  float gyroX = gyro.gyro.x;
  float prevGyroX = gyroX; //save gyro as previous gyro reading
  float gyroAngleX = discreteGyroIntegral(prevGyroX, gyroX); //calculate angle from gyro
  float gyroY = gyro.gyro.y;
  float gyroZ = gyro.gyro.z;

  //Map X Acceleration Data to Roll
  int mappedRoll = mapToRoll(accelX, accelY, accelZ);
  Serial.print(mappedRoll); Serial.print("  ");

  //Get LiDAR Data
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    LiDAR_Angle = LidarToAngle(measure.RangeMilliMeter);
  }

  //Map roll to servo turn angle
  turnAngle = motorAngle(mappedRoll);
  Serial.println(turnAngle);
  myservo.write(turnAngle); //turn servo
}

//Use roll as error and proportionally calculate the turn angle of the front servo
float motorAngle(float roll){
  if(roll > 20 || roll < -20){
    Kp = 1.15;
  }
  else{
    Kp = 1;
  }
  turnAngle = (90 + Kp*roll); //proportional control
  if(turnAngle > 180){
    turnAngle = 180; //upper turn limit
  }
  else if(turnAngle < 0){
    turnAngle = 0; //lower turn limit
  }
  return turnAngle;
}

//Map LiDAR to angle value
float LidarToAngle(float lidar){
  c = sqrt(sq(lidar) + sq(bike_height));
  angle = asin(lidar/c)*180/PI;
  return angle;
}

//Map acceleration to roll
float mapToRoll(float accelX, float accelY, float accelZ){
  roll = atan2(accelX,(sqrt((accelY * accelY) + (accelZ * accelZ))))*180/PI;
  return roll;
}

//Get angle from gyro data using discrete integration
float discreteGyroIntegral(float gyro1, float gyro2){
  float avgGyro = (gyro1+gyro2)/2; //take the average of the two readings
  float changeTime = 0.1; //time between gyro readings - 100ms
  float gyroAngle = (avgGyro*changeTime)*180/PI; 
  return gyroAngle;
}

//Accelerometer setup
void AccelerometerSetup(){
  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

  // sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
  delay(500);
  
}
