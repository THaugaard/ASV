/*
    Program for operating an autonomous surface vessel
    Created by Alvaro Martin Gomez, Pero Le and Thomas Haugaard in 2020
*/

// =========================== Includes =========================== //

// ========================== Initialise ========================== //

// ========================== Variables ========================== //
double currentPosLat;     // Holds the current position latitude
double currentPosLng;     // Holds the current position longitude
double dir;               // Holds the direction towards the next waypoint
double dist;              // Holds the distance to the next waypoint
float velocity;           // Holds last velocity received from the GPS
int isAtTarget = 0;       // Is set high when the target destination is reached
float zGyro;
float accel;
//Errors
float e_w = 0;
float e_w_prev = 0;
float e_v = 0;
float e_v_prev = 0;
float e_B = 0;
float e_B_prev = 0;
float e_D = 2;
float e_D_prev = 0;

//Saturations
float Umax = 22.5;
float Umin = 0;

//Control signals
float ut = 0;
float ut_prev = 0;
float ur = 0;
float ur_prev = 0;
float u1 = 0;
float u2 = 0;

//Inputs and outputs
float w = 0;
float w_ref = 0;
float v = 0;
float v_ref = 0;
float B = 0;
float B_ref = 0;
float D = 0;
float phi = 0;
float phi_ref = 0;
float lambda = 0;
float lambda_ref = 0;

//Constants for the discretized controllers
float B0_w = 64.63;
//float B0_w = 6.47;
float B1_w = -56.11;
//float B1_w = 5.61;
float B0_v = 165.47;
//float B0_v = 16.54;
float B1_v = -165.3;
//float B1_v = 16.54;
float B0_B = -0.6;
//float B0_B = 0.06;
float B0_D = 0.6;
//float B0_D = 0.06;

//Counter for timer 2 interrupts
int counter = 0;

//Counter to check run cycles since last GPS update
int counter_CyclesSinceLastGPS = 0;

//Flags that are checked and unchecked under certain conditions to alter code execution
int flag_GPSNoData = 0;

//Other
double R = 6371;

double _waypointArray[3][2] = {{1, 1}, {57.0241253, 9.9471116}, {57.0240025, 9.9473131}};
//double _waypointArray[4][2] = {{1, 1}, {57.0157285, 9.9858467}, {57.0159567, 9.9860624}, {57.0159916, 9.9858370}}; // Containes starting location, final destination (4 coordinates, 2 parts to each)
void _reversePointer();   // Changes between incrementing and decrementing
bool _pointerDir = 1;     // Determines whether pointer increments or decrements
int _arrayPointer = 1;    // Points to a given entry
int _accelVal;            // Holds the value from the accelerometer to be returned in the associated function.
int _compVal;             // Holds the value from the compass to be returned in the associated function.
int x, y, z;              // x, y, z values for the compass
int X_sen, Y_sen, Z_sen;  // Sensitivity values for the compass
float heading;            // Heading as calculated using the compass
int minX = 1000;          // Used by the compass
int maxX = -1000;         // Used by the compass
int minY = 1000;          // Used by the compass
int maxY = -1000;         // Used by the compass
int turnRight = 0;
int turnLeft = 0;


// Time variables
long timeLastCont;
long timeCurrent;
long timeLastCycle;
//int isRun = 0;

// -------------------------- Setup --------------------------- //
void setup() {
  Serial.begin(115200);
  
  m_setup();    // Module setup
  navSetup();   // Nav setup (GPS)
  c_setup();   // Controller setup
}


// --------------------------- Loop --------------------------- //
void loop() {
  getCurrentPos();
  timeCurrent = millis();
  if (timeCurrent - timeLastCont >= 10) {
    timeLastCont = millis();
    controller();
  }


  if (counter_CyclesSinceLastGPS <= 15) {
//    Serial.println((sizeof(_waypointArray) / sizeof(_waypointArray[0])) - 1);
        //Serial.print("lat: "); Serial.print(currentPosLat, 7);
        //Serial.print(" || lon: "); Serial.print(currentPosLng, 7);
        Serial.print("D: "); Serial.print(D);
        Serial.print(" || B: "); Serial.print(heading * 180 / M_PI);
        Serial.print(" || B_ref: "); Serial.print(B_ref * 180 / M_PI);
        Serial.print(" || B_err: "); Serial.print(e_B * 180 / M_PI);
        if (turnRight == 1) Serial.print(" || Right");
        else if (turnLeft == 1) Serial.print(" || Left");
        Serial.print(" || Left motor: "); Serial.print(u1);
        Serial.print(" || Right motor: "); Serial.print(mapfloat(u2,750,1500,2250,1500));
        Serial.print(" || ptr: "); Serial.print(_arrayPointer);
        Serial.print(" || ptr_dir: "); Serial.println(_pointerDir);
  }
  else if (timeCurrent - timeLastCycle >= 1000) {
    Serial.println("No GPS Data");
    timeLastCycle = millis();
  }
}
