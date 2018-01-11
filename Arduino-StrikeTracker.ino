// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Modified By: Joshua Dusome
// Date: 1/10/2018
// This program is used to record the maximum acceleration, final velocity, and duration of a punch. After recording the statistics,
// It sends the information to the Thingspeak API.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

//Define Pins
#define INTERRUPT_PIN 2

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//Time Variables
long int now=0, previous=-1000, initTime=0, recordTime=0, strikeTime=0; 

//Interval
unsigned int interval = 1000; // 1 second
unsigned int waitInterval = 60000; //1 minute
unsigned int recordDuration, strikeDuration;

//Acceleration
double accelerationRawX, accelerationRawY, gForceX, gForceY, convertedAccelX, convertedAccelY, peakAcceleration;

//Velocity
double velocityX[2] = {0, 0};
double velocityY[2] = {0, 0};

//Booleans
boolean recording;


// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                       Wifi                               ===
// ================================================================


//Thingspeak Api Key
String writeAPIKey = "ZA5FQO7KPOW3CRER"; 

//Thingspeak Server
#define DST_IP "api.thingspeak.com"  

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    //SETUP: ESP8266
    
    //Initialize Serial Connection to ESP8266
    Serial.begin(115200);

    //Reset Module
    Serial.println("AT+RST");
    delay(1000);

    //Set Single Client Mode
    Serial.println("AT+CWMODE=1"); 
    delay(5000);

    //Sets up Single connection mode.
    Serial.println("AT+CIPMUX=0");  
    delay(2000);

    //SETUP: Gyro
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    //Gyro Offsets
    mpu.setXGyroOffset(211);
    mpu.setYGyroOffset(-30);
    mpu.setZGyroOffset(-2);
    mpu.setXAccelOffset(-2002);
    mpu.setYAccelOffset(289);
    mpu.setZAccelOffset(431);

    //Set Acceleration Range to +/-8G
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        //Set the Initialization Time (Used to wait for voltage bias on accelerometer)
        initTime = millis();

        //Set Recording Flag
        recording = false;
        
    } else {
        // ERROR!
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        //Get Current Time
        now=millis();

        //Wait 60 seconds for accelerometer voltage bias to settle
        if (now - initTime >= waitInterval){

          //Get Raw Values
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

          //Convert Raw acceleration values to Doubles
          accelerationRawX = (double) aaReal.x;
          accelerationRawY = (double) aaReal.y;

          //Calculate G Force (Based on +/-8G Scale)
          gForceX = accelerationRawX / 4096;
          gForceY = accelerationRawY / 4096;

          //Convert G Force to acceleration in m/s2 (multiply by force of gravity)
          convertedAccelX = gForceX * 9.80665;
          convertedAccelY = gForceY * 9.80665;

          //1) Begin Recording - If Acceleration is greater than 3 m/s (squared) & Not currently recording
          if (convertedAccelX > 3 && !recording){

            //Set recording boolean to true
            recording = true;

            //Initialize Velocity to 0 m/s
            velocityX[0] = 0;
            velocityX[1] = 0;
            velocityY[0] = 0;
            velocityY[1] = 0;

            //Initialize Peak Acceleration
            peakAcceleration = 0;

            //Initialize Strike duration
            recordDuration = 0;

            //Initialize Strike time
            strikeTime = millis();

            //Initialize Record Time to Current Time
            recordTime = millis();
          }

          //2) Continue Recording - If Accelaration is greather than 0 m/s (squared) & currently recording
          if (convertedAccelX > 0 && recording){

            //Determine current recordDuration (in milliseconds)
            recordDuration = millis() - recordTime;

            //Convert recordDuration to seconds
            double recordDurationSeconds = (double) recordDuration;
            recordDurationSeconds = recordDurationSeconds / 1000;

            //Replace initial Velocity with old final Velocity
            velocityX[1] = velocityX[0];
            velocityY[1] = velocityY[0];
            
            //Calculate Final Velocity (in m/s): v(f) = v(i) + at
            velocityX[0] = velocityX[1] + (convertedAccelX * recordDurationSeconds);
            velocityY[0] = velocityY[1] + (convertedAccelY * recordDurationSeconds);

            //Set Maximum Combined Acceleration (if higher than previously recorded value)
            if (sqrt(convertedAccelX*convertedAccelX + convertedAccelY*convertedAccelY) > peakAcceleration){
              peakAcceleration = sqrt(convertedAccelX*convertedAccelX + convertedAccelY*convertedAccelY);
            }

            //Update recordTime
            recordTime = millis();
          }

          //3) Finish Recording - If Acceleration is less than 0/ms (squared) & currently recording
          if (convertedAccelX < 0 && recording){

            //Set recording boolean to false
            recording = false;

            //Calculate Combined Velocity
            double combinedVelocity = sqrt(velocityX[0]*velocityX[0] + velocityY[0]*velocityY[0]);

            //Calculate Total Time of Strike (ms)
            strikeDuration = millis() - strikeTime;
  
            //Convert strikeDuration to seconds
            double strikeDurationSeconds = (double) strikeDuration;
            strikeDurationSeconds = strikeDurationSeconds / 1000;
              
            /*
            //Log Stats to Console
            
            String VelocityXString = "Velocity X: ";
            VelocityXString += velocityX[0];
            VelocityXString += "m/s";
            String VelocityYString = "Velocity Y: ";
            VelocityYString += velocityY[0];
            VelocityYString += "m/s";
            String VelocityCombinedString = "Combined Velocity: ";
            VelocityCombinedString += combinedVelocity;
            VelocityCombinedString += "m/s";
            String PeakAccelerationString = "Peak Acceleration: ";
            PeakAccelerationString += peakAcceleration;
            PeakAccelerationString += "m/s²";
            String StrikeDurationString = "Strike Duration: ";
            StrikeDurationString += strikeDurationSeconds;
            StrikeDurationString += " seconds"; 
              
            Serial.println(VelocityXString);
            Serial.println(VelocityYString);
            Serial.println(VelocityCombinedString);
            Serial.println(PeakAccelerationString);
            Serial.println(StrikeDurationString);
            Serial.println();
            */

            //Only record punches shorter with a duration shorter than 1 second, this helps prevent any velocity drift or odd values
            if (strikeDurationSeconds < 1 && peakAcceleration > 8){

              //Send Statistics to Thingspeak
              updateThingSpeak(combinedVelocity, peakAcceleration, strikeDurationSeconds);
            }
          }
        }
    } 
}

void updateThingSpeak(double velocity, double acceleration, double duration){ 

   //Builds the connection string for the ESP8266
    String cmd = "AT+CIPSTART=\"TCP\",\""; //These 3 lines build a connection string, make TCP connection api.thingspeak.com on port 80
    cmd += DST_IP;
    cmd += "\",80";
    Serial.println(cmd);  //Run the command
    delay(1000);

    //Create HTTP Request
    String httpcmd="GET /update?api_key=";
    httpcmd += writeAPIKey+"&field1=";
    httpcmd += velocity;
    httpcmd += "&field2=";
    httpcmd += acceleration;
    httpcmd += "&field3=";
    httpcmd += duration;
    httpcmd += " HTTP/1.1\r\n";
    httpcmd += "Host: api.thingspeak.com\n";
    httpcmd += "Connection: close\r\n\r\n";

    //These lines tell the ESP chip how long the request string is
    Serial.print("AT+CIPSEND=");
    Serial.println(httpcmd.length());
    delay(1000);

    //This line sends the HTTP command
    Serial.println(httpcmd);
    delay(3000);

    //Close the Web Connection, Required to make a new request
    Serial.println("AT+CIPCLOSE");
}
