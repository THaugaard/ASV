// ========== Includes ========== //
#include <NMEAGPS.h>
//#include <AltSoftSerial.h>
#include <GPSport.h>
#include "NMEAGPS_cfg.h"
#include <Wire.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#include <Servo.h>
#include <ServoTimer2.h>

// ========== Defines ========== //
#define gps_Port Serial1
#define GPS_PORT_NAME "Serial1"
#define DEBUG_PORT Serial
#define compassaddress 0x1E //I2C 7-Bit address of HMC5883l (address to drag/read from)
#define PIN_LEFT_MOTOR 5
#define PIN_RIGHT_MOTOR 6
//#define SERVO_MIN_MS 1
//#define SERVO_MAX_MS 2

// ========== Instantiation ========== //
NMEAGPS gps;
gps_fix fix;
MPU6050 mpu;
ServoTimer2 LeftMotor;    // Create a servo object to control the left motor
ServoTimer2 RightMotor;   // Create a servo object to control the right motor

void navSetup() {
  Serial.print("Initialising GPS... ");
  gps_Port.begin(9600);
  gps.send_P( &gps_Port, F("$PMTK251,115200*1F\r\n") ); // set 115200 baud rate
  gps_Port.flush();                              // wait for the command to go out
  delay(100);                                    // wait for the GPS device to change speeds
  gps_Port.end();                                // empty the input buffer, too

  gps_Port.begin(115200);                        // use the new baud rate
  gps.send_P(&gpsPort, F("$PMTK314,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n") ); // disable unnecessary data
  gps.send_P(&gpsPort, F("$PMTK300,100,0,0,0,0*2C\r\n") );      // set 10Hz update rate

  Serial.println("Done");

  int startLocationSaved = 0;

  while (startLocationSaved != 1) {
    long timeNow, timeLast;
    int isRun = 0;
    while (gps.available(gps_Port)) {
      isRun = 1;
      //long timeBegin = micros();
      fix = gps.read();
      if (fix.valid.location) {
        startLocationSaved = 1;
        _waypointArray[0][0] = fix.latitude();
        _waypointArray[0][1] = fix.longitude();
        Serial.print("Start coordinates saved - ");
        Serial.print("latitude: ");
        Serial.print(_waypointArray[0][0], 7);
        Serial.print(" || longitude: ");
        Serial.println(_waypointArray[0][1], 7);
      }
      else {
        timeLast = millis();
        Serial.println("Error: Invalid GPS data");
      }
    }
    timeNow = millis();
    if (isRun == 0 && (timeNow - timeLast >= 1000)) {
      Serial.println("GPS data not available");
      timeLast = timeNow;
    }
    isRun = 0;
  }
}

void m_setup() {
  // ===== Initialise gyro/accelerometer ===== //
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Cannot find MPU6050 - check connection!");
    delay(500);
  }
  Serial.println("MPU6050 initialised");
  // Gyroscope calibration
  mpu.calibrateGyro();
  Serial.println("Gyroscope calibrated");
  // Set the sensitivity
  mpu.setThreshold(3);
  Serial.println("Gyroscope threshold set");

  // ===== Initialise compass ===== //
  Wire.begin();
  Wire.beginTransmission(compassaddress); // set up the connection with the sensor at the specific address
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission(); // end the transmission
  Serial.print("Compass initialised at address: "); Serial.println(compassaddress);
}

void c_setup() {
  // Allow ESC's to activate
  Serial.print("Initialising left ESC on pin "); Serial.print(PIN_LEFT_MOTOR);
  LeftMotor.attach(PIN_LEFT_MOTOR);
  LeftMotor.write(1500);
  for (int i = 0; i < 8; i++) {
    delay(500);
    Serial.print(".");
  }
  delay(500);
  Serial.println(".");
  
  Serial.print("Initialising right ESC on pin "); Serial.print(PIN_RIGHT_MOTOR);
  RightMotor.attach(PIN_RIGHT_MOTOR);
  RightMotor.write(1500);
  for (int i = 0; i < 8; i++) {
    delay(500);
    Serial.print(".");
  }
  delay(500);
  Serial.println(".");

  Serial.println("Testing left motor");
  LeftMotor.write(1650);
  RightMotor.write(1500);
  delay(500);
  Serial.println("Testing right motor");
  LeftMotor.write(1500);
  RightMotor.write(1350);
  delay(500);
  RightMotor.write(1500);
  LeftMotor.write(1500);
  delay(100);
}

void getCurrentPos() {
  while (gps.available(gps_Port)) {
    //long timeBegin = micros();
    fix = gps.read();
    if (fix.valid.location) {
      //timeCurrentSpeedUpdate = millis()/1000;
      counter_CyclesSinceLastGPS = 0;
      currentPosLat = fix.latitude();
      currentPosLng = fix.longitude();
      velocity = fix.speed_kph() * 1000 / 3600; // NeoGPS km/h function converted to m/s
    }
    else {
      Serial.println("Invalid");
    }
  }
}

void incrementPointer() {
  if (_arrayPointer == (sizeof(_waypointArray) / sizeof(_waypointArray[0])) - 1) { // If reached final waypoint
    _pointerDir = !_pointerDir;
  }
  else if (_arrayPointer == 0) { // If reached origin point
    _pointerDir = 1;
    _arrayPointer = 1;
    while (true){ // Infinite loop to stop boat returned to origin
    }
  }

  if (_pointerDir == 1) { // If still moving to destination
    _arrayPointer++;
  }
  else {  // If moving back to origin
    _arrayPointer--;
  }
}

float getWaypoint(int select) {
  return (_waypointArray[_arrayPointer][select]);
}

float getAngVelocity() {
  Vector normGyro = mpu.readNormalizeGyro();
  return normGyro.ZAxis * M_PI / 180;
}

float getBearing() {
  Wire.beginTransmission(compassaddress); // set up the connection with the sensor at the specific address
  Wire.write(0x03); //select register 3
  Wire.endTransmission(); // end the transmission

  //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(compassaddress, 6); // requesting the data, requesting 6 bytes

  while (6 <= Wire.available()) { // is there any information/data available? / for every 6 bytes
    x = Wire.read() << 8; // //MSB  x, a data string of 8-bits
    x |= Wire.read(); //LSB  x
    z = Wire.read() << 8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read() << 8; //MSB y
    y |= Wire.read(); //LSB y
  }

  // Insert value from Magneto: "Combined bias (b)", Order in Magneto,X Y Z.
  float Xm_off = x * (100000.0 / 1370) + (6179.130117); // subtraction by default
  float Ym_off = y * (100000.0 / 1370) + (26133.980735); // subtraction by default
  float Zm_off = z * (100000.0 / 1370) + (16517.439401); // subtraction by default

  ////  //Insert value from Magneto: "Correction for combined scale factors", first box is X: Remember to check if the bias has decreased to see whether the data is ok.
  ////  // https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
  float Xm_cal = (1.183726 * Xm_off) + (-0.009987 * Ym_off) + (0.026466 * Zm_off);
  float Ym_cal = (-0.009987 * Xm_off) + (1.189913 * Ym_off) + (-0.004735 * Zm_off);
  // float Zm_cal = (0.026466 * Xm_off) + (-0.004735 * Ym_off) + (1.353480 * Zm_off);


  // The value in degree. This is only true, if we expect the output to be a part of a unitcircle.
  // The output is a Raw Value, which I dont understand.
  heading = atan2(Xm_cal, Ym_cal); // calculationg into degrees

  float declinationAngle = (3.0 + (25.0 / 60.0)) / (180 / M_PI); // 3 and + 25 its change according to https://www.magnetic-declination.com/.
  heading += declinationAngle; // - is change according to the eksempel from the book

  // custom fuckery



  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }



  //if (heading >= 2*M_PI) heading-=2*M_PI;
  //  Convert to degrees
  heading = 2 * M_PI - heading;
  //  float headingDegrees = heading * 180 / M_PI;
  //  headingDegrees = 359.99 - headingDegrees;

  //Serial.println(heading*180/M_PI);
  return heading;
}

void setSaturations_u() {
  if (u1 > Umax) u1 = Umax;
  if (u1 < Umin) u1 = Umin;
  if (u2 > Umax) u2 = Umax;
  if (u2 < Umin) u2 = Umin;
}

void setSaturations_ut() {
  if (ut > Umax) ut = Umax;
  if (ut < Umin) ut = Umin;
}

void generatePWM() {
  if (u1 >= 0) {
    u1 = mapfloat(u1, 0, Umax, 1500, 2250);
  }
  else u2 = 0;
//  else if (u1 < 0) {
//    u1 = mapfloat(u1, Umin, 0, 750, 1500);
//  }
  if (u2 >= 0) {
    u2 = mapfloat(u2, 0, Umax, 1500, 750);
  }
  else u2 = 0;
//  else if (u2 < 0) {
//    u2 = mapfloat(u2, Umin, 0, 750, 2250);
//  }

  //if (u1>1600) u1 = 1600;   // Capping left motor output for chair test
  //if (u2<1400) u2 = 1400;   // Capping left motor output for chair test
  LeftMotor.write(u1);
  RightMotor.write(u2);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void controller() {
  //isRun = 1;
  //cli();
  if (counter_CyclesSinceLastGPS <= 150) {
    //long timeBegin = micros();
    counter_CyclesSinceLastGPS++;
    //Serial.print("lat: "); Serial.print(currentPosLat, 7);
    //Serial.print(" || lon: "); Serial.println(currentPosLng, 7);
    // Increment counter
    counter++;
    // Read variables
    w = getAngVelocity();
    v = velocity;

    // Controller
    e_w = (w_ref - w);
    //ur = ur_prev + B0_w * e_w - B1_w * e_w_prev;

    e_v = (v_ref - v);
    ut = ut_prev + B0_v * e_v - B1_v * e_v_prev;

    setSaturations_ut();



    if (e_B <= 0) // Turn left
    {
      ur = ur_prev - B0_w * e_w + B1_w * e_w_prev;
      u1 = ut - ur;
      u2 = ut;
      turnRight = 0;
      turnLeft = 1;
    }
    if (e_B > 0)  // Turn right
    {
      ur = ur_prev + B0_w * e_w - B1_w * e_w_prev;
      u1 = ut;
      u2 = ut - ur;
      turnRight = 1;
      turnLeft = 0;
    }
    //Serial.print(" || ur: "); Serial.print(ur);


    // Update values
    e_w_prev = e_w;
    e_v_prev = e_v;
    ut_prev = ut;
    ur_prev = ur;

    if (counter == 10)
    {
      //Reset counter
      counter = 0;
      // Read variables
      B = getBearing();

      //      Serial.print("lat1: "); Serial.print(currentPosLat, 7);
      //      Serial.print(" || lat2: "); Serial.print(getWaypoint(0), 7);
      //      Serial.print(" || lon1: "); Serial.print(currentPosLng, 7);
      //      Serial.print(" || lon2: "); Serial.println(getWaypoint(1), 7);

      phi_ref = getWaypoint(0) * M_PI / 180;
      phi = currentPosLat * M_PI / 180;
      lambda_ref = getWaypoint(1) * M_PI / 180;
      lambda = currentPosLng * M_PI / 180;

      //Serial.print("latR1: "); Serial.print(phi, 7);
      //Serial.print(" || latR2: "); Serial.print(phi_ref, 7);
      //Serial.print(" || lonR1: "); Serial.print(lambda, 7);
      //Serial.print(" || lonR2: "); Serial.print(lambda_ref, 7);

      double lat1 = phi;
      double lat2 = phi_ref;
      double lon1 = lambda;
      double lon2 = lambda_ref;
      double dlat = lat2 - lat1;
      double dlon = lon2 - lon1;
      double a, e;
      double dist;
      // Compute distance
      a = (sq(sin(dlat / 2))) + cos(lat1) * cos(lat2) * (sq(sin(dlon / 2)));
      e = 2 * atan2(sqrt(a), sqrt(1 - a));
      D = R * e * 1000; // Calculates the distance. Multiplied by 1000 to get distance in meters

      //Compute bearing
      //double y = cos(phi_ref) * sin(lambda_ref - lambda);
      //double x = cos(phi) * sin(phi_ref) - sin(phi) * cos(phi_ref) * cos(lambda_ref - lambda);

      //B_ref = atan2(y, x);  //Returns bearing in radians

      //B_ref = atan2(sin(dlon)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon));

      double x = cos(lat2) * sin(lon2 - lon1);
      double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);

      //Serial.print(" || x: "); Serial.print(x, 7);
      //Serial.print(" || y: "); Serial.print(y, 7);

      B_ref = atan2(x, y);

      if (B_ref < 0) {
        B_ref += 2 * PI;
      }

      if (B_ref > 2 * PI) {
        B_ref -= 2 * PI;
      }

      //Serial.print(" || B_ref: "); Serial.println(B_ref, 7);

      //Calculate B_ref and D using haversine formula
      //B_ref = atan2(sin(lambda_ref - lambda) * cos(phi_ref), cos(phi) * sin(phi_ref) - sin(phi) * cos(phi_ref) * cos(lambda_ref - lambda));

      //D = 2 * R * 1000 * asin(sqrt(sin((phi_ref - phi) / 2) ^ 2 + cos(phi) * cos(phi_ref) * sin((lambda_ref - lambda) / 2) ^ 2));

      // Controller
      e_B = (B_ref - B);

      if (e_B > M_PI) e_B = (e_B - 2 * M_PI);
      if (e_B < -M_PI) e_B = (e_B + 2 * M_PI);

      w_ref = B0_B * e_B;

      e_D = D;

    if (e_D < 3) {
      incrementPointer();
    }
      
      v_ref = B0_D * e_D;
    }
    setSaturations_u();
    generatePWM();
    //    Serial.print(" || u1: "); Serial.print(u1);
    //    Serial.print(" || u2: "); Serial.println(u2);
    //long timeDone = micros();
    //Serial.println(timeDone-timeBegin);
  }
  else {
    LeftMotor.write(1500);
    RightMotor.write(1500);
    //Serial.println("ERROR: No new GPS data received. Setting motors to idle.");
  }
  //sei();
}
