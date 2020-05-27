#include <HMC5883L.h>
#include <Wire.h>
#include <math.h>

// Arduino is the Master

int x, y, z;
int X_sen, Y_sen, Z_sen;
float heading;

int minX = 1000;
int maxX = -1000;
int minY = 1000;
int maxY = -1000;

#define address 0x1E //I2C 7-Bit address of HMC5883l (address to drag/read from)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(address); // set up the connection with the sensor at the specific address
  Wire.write(0x02);
  //  Wire.write(B00000001);
  Wire.write(0x00);
  Wire.endTransmission(); // end the transmission
}

void loop() {
  Wire.beginTransmission(address); // set up the connection with the sensor at the specific address
  Wire.write(0x03); //select register 3
  Wire.endTransmission(); // end the transmission

  //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(address, 6); // requesting the data, requesting 6 bytes

  while (6 <= Wire.available()) { // is there any information/data available? / for every 6 bytes
    x = Wire.read() << 8; // //MSB  x, a data string of 8-bits
    x |= Wire.read(); //LSB  x
    z = Wire.read() << 8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read() << 8; //MSB y
    y |= Wire.read(); //LSB y
    //    processData();
  }

//  float Xm_print, Ym_print, Zm_print;
//
//  Xm_print = x * (100000.0 / 1370); // Sensitivity division (and nano Tesla...)
//  Ym_print = y * (100000.0 / 1370);
//  Zm_print = z * (100000.0 / 1370);

  float Xm_off, Ym_off, Zm_off;
  float Xm_cal, Ym_cal, Zm_cal;

  // Insert value from Magneto: "Combined bias (b)", Order in Magneto,X Y Z.
  Xm_off = x * (100000.0 / 1370) + (6179.130117); // subtraction by default
  Ym_off = y * (100000.0 / 1370) + (26133.980735); // subtraction by default
  Zm_off = z * (100000.0 / 1370) + (16517.439401); // subtraction by default

 //Insert value from Magneto: "Correction for combined scale factors", first box is X: Remember to check if the bias has decreased to see whether the data is ok. 
 // https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
  Xm_cal = (1.183726 * Xm_off) + (-0.009987 * Ym_off) + (0.026466 * Zm_off); 
  Ym_cal = (-0.009987 * Xm_off) + (1.189913 * Ym_off) + (-0.004735 * Zm_off);
  Zm_cal = (0.026466 * Xm_off) + (-0.004735 * Ym_off) + (1.353480 * Zm_off);

//    Serial.print(Xm_print, 10);
//    Serial.print(" ");
//    Serial.print(Ym_print, 10);
//    Serial.print(" ");
//    Serial.println(Zm_print, 10);
//    delay(125);

//    Serial.print(Xm_cal, 10);
//    Serial.print(" ");
//    Serial.print(Ym_cal, 10);
//    Serial.print(" ");
//    Serial.println(Zm_cal, 10);
//    delay(125);

//   The value in degree. This is only true, if we expect the output to be a part of a unitcircle.
//   The output is a Raw Value, which I dont understand.
  heading = atan2(Xm_cal, Ym_cal); // calculationg into degrees

  float declinationAngle = (3.0 + (25.0 / 60.0)) / (180 / M_PI); // 3 and + 25 its change according to https://www.magnetic-declination.com/.
  heading += declinationAngle; // - is change according to the eksempel from the book


// Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
    {
      heading += 2 * PI;
    }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  //  Convert to degrees
  float headingDegrees = heading * 180 / M_PI;
  Serial.println(headingDegrees);

}


// --------------------------------------------------------------------------------------------------------------------
// Explanation of: <<
//int b = a << 3;   // binary: 101000, or 40 in decimal. (msb..lsb.<--read the numbers from this direction)
//int c = b >> 3;   // binary: 101, or 5. (-->lsb..msb.Read the numbers from this direction)


// Explanation of: |=
//0  0  1  1    operand1
//0  1  0  1    operand2
//----------
//0  0  0  1    (operand1 & operand2) = result
