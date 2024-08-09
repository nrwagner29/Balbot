#include <iq_module_communication.hpp>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 i2cmpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  //= { 0.0, -9.81, 0.0 };  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 gyro;     // [x, y, z]            Gyro Measurements
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


int i = 0;
int motordrive = 0;  /*change if want motors on or off*/
int serialprint = 1; /*change to 1 if want telemetrics to print to serial*/
int sdprint = 0;     /*change to 1 if want print to sd card*/
//Balbot chassis
float Ixb = 0.0003478;
float Iyb = 0.0004134;
float Izb = 0.0004908;
//Wheel 1
float Ixwl = 0.0002695;
float Iywl = 0.0002695;
float Izwl = 0.000539;
//Wheel 2
float Ixwr = 0.0002695;
float Iywr = 0.0002695;
float Izwr = 0.000539;
float theta = M_PI / 6;
//masses
float bmass = .436;
float wmass = .110;
//consts
float gr = 9.81;
float l = .13;
//code params
static float velocityr = 0;
static float velocityl = 0;
float Motorcommandr = 0;
float Motorcommandl = 0;
float voltager = 0;
float voltagel = 0;
float taul = 0;
float taur = 0;
float currl = 0;
float currr = 0;
float rmotorR = 4.7;
float lmotorR = 4.7;
float lemf = 0;
float remf = 0;
float kt = 0;
float q1 = 0;
float q2 = 0;
float q3 = 0;
float q4 = 0;
float q5 = 0;
float dq1 = 0;
float dq2 = 0;
float dq3 = 0;
float dq4 = 0;
float dq5 = 0;
int16_t x;
int16_t y;
int16_t z;
float ax = 0;
float ay = 0;
float FREQ = 200;
float zero = 0.5; //drive .5 volts
unsigned long starttime = 0;
unsigned long imutime = 0;
unsigned long motortime = 0;
unsigned long time = 0;
unsigned long timeend = 0;


const int chipSelect = 4;
IqSerial ser(Serial1);
SerialInterfaceClient sicr(0);
SerialInterfaceClient sicl(1);
BrushlessDriveClient motr(0);
BrushlessDriveClient motl(1);
PowerMonitorClient pwrr(0);
PowerMonitorClient pwrl(1);
MultiTurnAngleControlClient angr(0);
MultiTurnAngleControlClient angl(1);

File dataFile;

uint32_t L1baud = 0;
uint32_t R0baud = 0;



void setup() {
  // Initialize serial and wait for port to open:
  // Initialize the IqSerial object
  ser.begin(921600);


  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif


  Serial.begin(921600);
  while (!Serial)
    ;  // will pause MKR until serial console opens


  ser.set(sicl.baud_rate_, (uint32_t)921600);
  ser.get(sicl.baud_rate_, L1baud);
  Serial.print(L1baud);

  ser.set(sicr.baud_rate_, (uint32_t)921600);
  ser.get(sicr.baud_rate_, R0baud);
  Serial.print(R0baud);

  Serial.println("Adafruit MPU6050 and Vertiq Motor Test!");
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (sdprint != 0) {
    if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      while (1)
        ;
    }
    Serial.println("card initialized.");
    // Try to initialize!
    Serial.println(F("Initializing I2C devices..."));
    i2cmpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(i2cmpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  }
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read())
    ;  // empty buffer
  while (!Serial.available())
    ;  // wait for data
  while (Serial.available() && Serial.read())
    ;  // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = i2cmpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  i2cmpu.setXGyroOffset(220);
  i2cmpu.setYGyroOffset(76);
  i2cmpu.setZGyroOffset(-85);
  i2cmpu.setZAccelOffset(1688);  // 1688 factory default for my test chip
  i2cmpu.setXAccelOffset(0);
  i2cmpu.setYAccelOffset(0);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    i2cmpu.CalibrateAccel(6);
    i2cmpu.CalibrateGyro(6);
    i2cmpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    i2cmpu.setDMPEnabled(true);

    //   // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = i2cmpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = i2cmpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  /*zero all motor control and read values*/
  ser.set(angl.obs_angular_displacement_, zero);
  ser.set(angr.obs_angular_displacement_, zero);

  ser.set(angl.ctrl_volts_, zero);
  ser.set(angr.ctrl_volts_, zero);

  dataFile = SD.open("test.txt", FILE_WRITE);  // O_WRITE | O_CREAT
  if (!dataFile) {
    Serial.println("Failed to open file for writing");
    return;
  }
  delay(1000);
}












void loop() {
  // Your code here
  starttime = micros();
  Serial.print("Start Loop Time: ");
  Serial.println(starttime);

  if (!dmpReady) return;
  // read a packet from FIFO
  if (i2cmpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
                                                     // #ifdef OUTPUT_READABLE_QUATERNION
                                                     //     // display quaternion values in easy matrix form: w x y z
                                                     //     i2cmpu.dmpGetQuaternion(&q, fifoBuffer);
                                                     //     Serial.print(" quat\t");
                                                     //     Serial.print(q.w);
                                                     //     Serial.print("\t");
                                                     //     Serial.print(q.x);
                                                     //     Serial.print("\t");
                                                     //     Serial.print(q.y);
                                                     //     Serial.print("\t");
                                                     //     Serial.print(q.z);
                                                     // #endif

    // #ifdef OUTPUT_READABLE_EULER
    //     // display Euler angles in degrees
    //     i2cmpu.dmpGetQuaternion(&q, fifoBuffer);
    //     i2cmpu.dmpGetEuler(euler, &q);
    //     Serial.print(" euler\t");
    //     Serial.print(euler[0] * 180 / M_PI);
    //     Serial.print("\t");
    //     Serial.print(euler[1] * 180 / M_PI);
    //     Serial.print("\t");
    //     Serial.print(euler[2] * 180 / M_PI);
    // #endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    i2cmpu.dmpGetQuaternion(&q, fifoBuffer);
    i2cmpu.dmpGetGravity(&gravity, &q);
    i2cmpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    i2cmpu.dmpGetGyro(&gyro, fifoBuffer);
    // Serial.print(" | ypr ");
    // Serial.print(ypr[0] * 180 / M_PI);
    // Serial.print(" ");
    // Serial.print(ypr[1] * 180 / M_PI);
    // Serial.print(" ");
    // Serial.print(ypr[2] * 180 / M_PI);
    // Serial.print(" | Gyro ");
    // Serial.print(gyro.x);
    // Serial.print(" ");
    // Serial.print(gyro.y);
    // Serial.print(" ");
    // Serial.print(gyro.z);
#endif

    // #ifdef OUTPUT_READABLE_REALACCEL
    //     // display real acceleration, adjusted to remove gravity
    //     i2cmpu.dmpGetQuaternion(&q, fifoBuffer);
    //     //i2cmpu.dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
    //     i2cmpu.dmpGetAccel(&aa, fifoBuffer);
    //     i2cmpu.dmpGetGravity(&gravity, &q);
    //     i2cmpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //     Serial.print(" areal");
    //     Serial.print(aaReal.x);
    //     Serial.print(" ");
    //     Serial.print(aaReal.y);
    //     Serial.print(" ");
    //     Serial.print(aaReal.z);
    // #endif

    // #ifdef OUTPUT_READABLE_WORLDACCEL
    //     // display initial world-frame acceleration, adjusted to remove gravity
    //     // and rotated based on known orientation from quaternion
    //     i2cmpu.dmpGetQuaternion(&q, fifoBuffer);
    //     i2cmpu.dmpGetAccel(&aa, fifoBuffer);
    //     i2cmpu.dmpGetGravity(&gravity, &q);
    //     i2cmpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //     i2cmpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    //     Serial.print(" aworld\t");
    //     Serial.print(aaWorld.x);
    //     Serial.print("\t");
    //     Serial.print(aaWorld.y);
    //     Serial.print("\t");
    //     Serial.print(aaWorld.z);
    // #endif

    // #ifdef OUTPUT_TEAPOT
    //     // display quaternion values in InvenSense Teapot demo format:
    //     teapotPacket[2] = fifoBuffer[0];
    //     teapotPacket[3] = fifoBuffer[1];
    //     teapotPacket[4] = fifoBuffer[4];
    //     teapotPacket[5] = fifoBuffer[5];
    //     teapotPacket[6] = fifoBuffer[8];
    //     teapotPacket[7] = fifoBuffer[9];
    //     teapotPacket[8] = fifoBuffer[12];
    //     teapotPacket[9] = fifoBuffer[13];
    //     Serial.write(teapotPacket, 14);
    //     teapotPacket[11]++;  // packetCount, loops at 0xFF on purpose
    // #endif
  }
  imutime = micros();
  Serial.print("IMU get time: ");
  Serial.println(imutime);

  //ser.get(pwrr.volts_, voltager);
  // Serial.print(" | Motor R Supp Volt: ");
  // Serial.print(voltager);

  // ser.get(pwrl.volts_, voltagel);
  // Serial.print(" | Motor L Supp Volt: ");
  // Serial.print(voltagel);

  // Get the velocity. If a response was received...

  // ser.get(angr.obs_angular_velocity_, dq5);

  // // Serial.print(" | Motor R Velo: ");
  // // Serial.print(velocityr);

  // ser.get(angl.obs_angular_velocity_, dq4);
  // // Display the reported velocity

  // // Serial.print(" | Motor L Velo: ");
  // // Serial.print(velocityl);
  // ser.get(angl.obs_angular_displacement_, q4);
  // ser.get(angr.obs_angular_displacement_, q5);

  /*get params*/
  // ser.get(motr.motor_R_ohm_, rmotorR);
  // //   Serial.print(rmotorR);
  // ser.get(motl.motor_R_ohm_, lmotorR);
  // //   Serial.print(lmotorR);
  // ser.get(motr.motor_emf_calc_, remf);
  // //   Serial.print(remf);
  // ser.get(motl.motor_emf_calc_, lemf);
  //   // Serial.print(lemf);

  motortime = micros();
  Serial.print("get motor data time: ");
  Serial.println(motortime);
  kt = .053;

  /*states (q1 and dq1 not used)*/
  q1 = -ypr[0];  //radians
  q2 = ypr[1];
  q3 = ypr[2];
  /* q4 and q5 defined above*/
  dq1 = gyro.z / 16.4 / 180 * M_PI;   //radians
  dq2 = -gyro.y / 16.4 / 180 * M_PI;  //radians
  dq3 = gyro.x / 16.4 / 180 * M_PI;   //radians
                                      /* dq4 and dq5 defined above*/
  time = micros();

  /*print to serial*/
  if (serialprint != 0) {
    Serial.print(" | q1: ");
    Serial.print(q1, 3);
    Serial.print(" q2: ");
    Serial.print(q2, 3);
    Serial.print(" q3: ");
    Serial.print(q3, 3);
    Serial.print(" q4: ");
    Serial.print(q4, 3);
    Serial.print(" q5: ");
    Serial.print(q5, 3);
    Serial.print(" | dq1: ");
    Serial.print(dq1, 3);
    Serial.print(" dq2: ");
    Serial.print(dq2, 3);
    Serial.print(" dq3: ");
    Serial.print(dq3, 3);
    Serial.print(" dq4: ");  //lowpass filter?
    Serial.print(dq4, 3);
    Serial.print(" dq5: ");  //Lowpass Filter?
    Serial.print(dq5, 3);
    Serial.println(" ");
    /*time*/
    // time = micros();
    // Serial.print("Print States time: ");
    // Serial.println(time);
  }


  /*controller code*/
  float t2 = q3;
  float t4 = wmass * 2.0;
  float t8 = wmass * 1.69E-2;
  float t9 = q2 * 1.203210011390811E+2;
  float t10 = q3 * 1.198501540420636E+2;
  float t11 = q4 * 9.287448442614035E-1;
  float t12 = q5 * 9.211640318625102E-1;
  float t13 = dq3 * 2.229627428782927E+1;
  float t14 = dq2 * 2.233333295757935E+1;
  float t15 = dq5 * 8.472453159249492E-1;
  float t16 = dq4 * 8.523940595548498E-1;
  float t3 = cos(t2);
  float t5 = t2 * 2.0;
  float t7 = bmass + t4;
  float t17 = t10 + t12 + t13 + t15;
  float t18 = t9 + t11 + t14 + t16;
  float t6 = cos(t5);
  taul = (t18 * (Ixb / 2.0 + Ixwr / 2.0 + Iywl / 2.0 + Izb / 2.0 + Izwl / 2.0 + Izwr / 2.0 + bmass * 8.45E-3 + t8 + (Ixb * t6) / 2.0 + (Ixwr * t6) / 2.0 + (Iywl * t6) / 2.0 - (Izb * t6) / 2.0 - (Izwl * t6) / 2.0 - (Izwr * t6) / 2.0 + bmass * t6 * 8.45E-3 + t6 * t8) + gr * l * t7 * sin(q2)) / t3 - t3 * t18 * (Iywl + t8);
  taur = t17 * (Ixwl + Iyb + Iywr + bmass * 1.69E-2 + wmass * 3.38E-2) - t17 * (Iywr + t8) + gr * l * t7 * sin(q3);

  // time = micros();
  // Serial.print("Controller code time: ");
  // Serial.println(time);


  /*Torque to Voltage path */
  currl = taul / kt;
  currr = taur / kt;
  /*print current commanded from controller codd*/
  if (serialprint != 0) {
    Serial.print(" c L: ");
    Serial.print(currl, 3);
    Serial.print(" c R: ");
    Serial.println(currr, 3);
  }
  /*current limiter*/
  if (currl > 5) {
    currl = 5;
  } else if (currl < -5) {
    currl = -5;
  }

  if (currr > 5) {
    currr = 5;
  } else if (currr < -5) {
    currr = -5;
  }

  /*current to voltage calc*/
  lemf = kt * velocityl;
  remf = kt * velocityr;
  Motorcommandl = lemf + (lmotorR * currl);
  Motorcommandr = remf + (rmotorR * currr);

  /*set motors to desired voltage */

  if (abs(q2) > .20 | abs(q3) > .20) {
    ser.set(motr.drive_brake_);
    ser.set(motl.drive_brake_);
  } else if (motordrive = 0) {
    ser.set(motr.drive_spin_volts_, zero);
    ser.set(motl.drive_spin_volts_, zero);
  } else if (motordrive != 0) {
    ser.set(motr.drive_spin_volts_, Motorcommandr);
    ser.set(motl.drive_spin_volts_, Motorcommandl);
  }

  // time = micros();
  // Serial.print("Voltage to motor driver time: ");
  // Serial.print(time);

  if (serialprint != 0) {
    Serial.print(" V L: ");
    Serial.print(Motorcommandl);
    Serial.print(" V R: ");
    Serial.println(Motorcommandr);
  }
  /*Write data to sd card*/

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  time = micros();
  if (sdprint != 0) {
    // if the file is available, write to it: CSV format
    if (dataFile) {

      Serial.print(i);
      Serial.println(dataFile);
      if (i == 0) {

        dataFile.print("Time");
        Serial.println("Time");
        dataFile.print(", q1");
        dataFile.print(", q2");
        dataFile.print(", q3");
        dataFile.print(", q4");
        dataFile.print(", q5");
        dataFile.print(", dq1");
        dataFile.print(", dq2");
        dataFile.print(", dq3");
        dataFile.print(", dq4");
        dataFile.print(", dq5");
        // // dataFile.print(" | Motor R Supp Volt: ");
        // // dataFile.print(" | Motor L Supp Volt: ");
        // // dataFile.print(" | Motor R Velo: ");
        // // dataFile.print(" | Motor L Velo: ");
        // dataFile.print(", Volt L Comm: ");
        // dataFile.print(", Volt R Comm: ");
        // dataFile.println(" ");
      }
      dataFile.print(time / 1000);
      Serial.println(time / 1000);
      dataFile.print(", ");
      dataFile.print(q1);
      dataFile.print(", ");
      dataFile.print(q2);
      dataFile.print(", ");
      dataFile.print(q3);
      dataFile.print(", ");
      dataFile.print(q4);
      dataFile.print(", ");
      dataFile.print(q5);
      dataFile.print(", ");
      dataFile.print(dq1);
      dataFile.print(", ");
      dataFile.print(dq2);
      dataFile.print(", ");
      dataFile.print(dq3);
      dataFile.print(", ");
      dataFile.print(dq4);
      dataFile.print(", ");
      dataFile.print(dq5);
      // dataFile.print(", ");
      // dataFile.print(voltager);
      // dataFile.print(", ");
      // dataFile.print(voltagel);
      // dataFile.print(", ");
      // dataFile.print(velocityr);
      // dataFile.print(", ");
      // dataFile.print(velocityl);
      // dataFile.print(", ");
      // dataFile.print(Motorcommandl);
      // dataFile.print(", ");
      // dataFile.print(Motorcommandr);
      // dataFile.print(", ");

      dataFile.println(" ");

      // dataFile.close();
      i = i + 1;
    }
  }
  timeend = micros();
  Serial.print(" SD write time: ");
  Serial.println(timeend - time);
}
