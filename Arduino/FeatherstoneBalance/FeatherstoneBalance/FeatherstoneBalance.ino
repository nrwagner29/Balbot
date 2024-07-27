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

const int chipSelect = 4;
IqSerial ser(Serial1);
BrushlessDriveClient motr(0);
BrushlessDriveClient motl(2);
PowerMonitorClient pwrr(0);
PowerMonitorClient pwrl(2);
MultiTurnAngleControlClient angr(0);
MultiTurnAngleControlClient angl(2);


int i = 0;

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
float l = .1;
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
float rmotorR = 0;
float lmotorR = 0;
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
float zero = 0;

void setup() {
  // Initialize serial and wait for port to open:
  // Initialize the IqSerial object
  ser.begin(115200);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif


  Serial.begin(115200);
  while (!Serial)
    ;  // will pause MKR until serial console opens


  Serial.println("Adafruit MPU6050 and Vertiq Motor Test!");
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
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


  delay(1000);
}

void loop() {
  // Your code here



  if (!dmpReady) return;
  // read a packet from FIFO
  if (i2cmpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    i2cmpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print(" quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.print(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    i2cmpu.dmpGetQuaternion(&q, fifoBuffer);
    i2cmpu.dmpGetEuler(euler, &q);
    Serial.print(" euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    i2cmpu.dmpGetQuaternion(&q, fifoBuffer);
    i2cmpu.dmpGetGravity(&gravity, &q);
    i2cmpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    i2cmpu.dmpGetGyro(&gyro, fifoBuffer);
    Serial.print(" | ypr ");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print(" ");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print(" ");
    Serial.print(ypr[2] * 180 / M_PI);
    // Serial.print(" | Gyro ");
    // Serial.print(gyro.x);
    // Serial.print(" ");
    // Serial.print(gyro.y);
    // Serial.print(" ");
    // Serial.print(gyro.z);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    i2cmpu.dmpGetQuaternion(&q, fifoBuffer);
    //i2cmpu.dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
    i2cmpu.dmpGetAccel(&aa, fifoBuffer);
    i2cmpu.dmpGetGravity(&gravity, &q);
    i2cmpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print(" areal");
    Serial.print(aaReal.x);
    Serial.print(" ");
    Serial.print(aaReal.y);
    Serial.print(" ");
    Serial.print(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    i2cmpu.dmpGetQuaternion(&q, fifoBuffer);
    i2cmpu.dmpGetAccel(&aa, fifoBuffer);
    i2cmpu.dmpGetGravity(&gravity, &q);
    i2cmpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    i2cmpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print(" aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.print(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++;  // packetCount, loops at 0xFF on purpose
#endif
  }

  if (ser.get(pwrr.volts_, voltager)) {
    Serial.print(" | Motor R Supp Volt: ");
    Serial.print(voltager);
  }
  if (ser.get(pwrl.volts_, voltagel)) {
    Serial.print(" | Motor L Supp Volt: ");
    Serial.print(voltagel);
  }
  // Get the velocity. If a response was received...

  if (ser.get(motr.obs_velocity_, velocityr)) {
    // Display the reported velocity

    Serial.print(" | Motor R Velo: ");
    Serial.print(velocityr);
  } else {
    Serial.print(" NRR ");
  }

  if (ser.get(motl.obs_velocity_, velocityl)) {
    // Display the reported velocity

    Serial.print(" | Motor L Velo: ");
    Serial.print(velocityl);
  } else {
    Serial.print(" NRR ");
  }
  Serial.println(" ");

  /*get params*/
  ser.get(motr.motor_R_ohm_, rmotorR);
  //   Serial.print(rmotorR);
  ser.get(motl.motor_R_ohm_, lmotorR);
  //   Serial.print(lmotorR);
  ser.get(motr.emf_, remf);
  //   Serial.print(remf);
  ser.get(motl.emf_, lemf);
  //   Serial.print(lemf);
  kt = .053;

  /*states (q1 and dq1 not used)*/
  q1 = ypr[0];  //radians
  Serial.print(" | q1: ");
  Serial.print(q1);
  q2 = ypr[1];
  Serial.print(" q2: ");
  Serial.print(q2);
  q3 = ypr[2];
  Serial.print(" q3: ");
  Serial.print(q3);
  ser.get(angl.obs_angular_displacement_, q4);
  Serial.print(" q4: ");
  Serial.print(q4);
  ser.get(angr.obs_angular_displacement_, q5);
  Serial.print(" q5: ");
  Serial.print(q5);
  dq1 = gyro.z / 16.4 ;  //radians
  Serial.print(" | dq1: ");
  Serial.print(dq1);
  dq2 = -gyro.y / 16.4 ;  //radians
  Serial.print(" dq2: ");
  Serial.print(dq2);
  dq3 = gyro.x / 16.4;  //radians
  Serial.print(" dq3: ");
  Serial.print(dq3);
  dq4 = velocityl;
  Serial.print(" dq4: ");
  Serial.print(dq4);
  dq5 = velocityr;
  Serial.print(" dq5: ");
  Serial.print(dq5);


  /*controller code*/
  float t2 = q3;
  float t4 = wmass * 2.0;
  float t8 = wmass / 1.0E+2;
  float t9 = q2 * 1.429673802570823E+2;
  float t10 = q3 * 1.420289341108403E+2;
  float t11 = q5 * 1.020570946849774;
  float t12 = q4 * 1.034433339236874;
  float t13 = dq2 * 2.404855958330733E+1;
  float t14 = dq3 * 2.397991359830334E+1;
  float t15 = dq4 * 8.661464499620468E-1;
  float t16 = dq5 * 8.574709315181061E-1;
  float t3 = cos(t2);
  float t5 = t2 * 2.0;
  float t7 = bmass + t4;
  float t17 = t9 + t12 + t13 + t15;
  float t18 = t10 + t11 + t14 + t16;
  float t6 = cos(t5);
  taul = (t17 * (Ixb / 2.0 + Ixwr / 2.0 + Iywl / 2.0 + Izb / 2.0 + Izwl / 2.0 + Izwr / 2.0 + bmass / 2.0E+2 + t8 + (Ixb * t6) / 2.0 + (Ixwr * t6) / 2.0 + (Iywl * t6) / 2.0 - (Izb * t6) / 2.0 - (Izwl * t6) / 2.0 - (Izwr * t6) / 2.0 + (bmass * t6) / 2.0E+2 + t6 * t8) + gr * l * t7 * sin(q2)) / t3 - t3 * t17 * (Iywl + t8);
  taur = t18 * (Ixwl + Iyb + Iywr + bmass / 1.0E+2 + wmass / 5.0E+1) - t18 * (Iywr + t8) + gr * l * t7 * sin(q3);

  /*Torque to Voltage path */
  currl = taul / kt;
  currr = taur / kt;

  Serial.print(" curr L des: ");
  Serial.print(currl);


  Serial.print(" curr R des: ");
  Serial.print(currr);
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
  Motorcommandl = lemf + lmotorR * currl;
  Motorcommandr = remf + rmotorR * currr;


  /*voltage limiter*/
  if (Motorcommandl > voltagel) {
    Motorcommandl = voltagel;
  } else if (Motorcommandl < -voltagel) {
    Motorcommandl = -voltagel;
  }
  if (Motorcommandr > voltager) {
    Motorcommandr = voltager;
  } else if (Motorcommandr < -voltager) {
    Motorcommandr = -voltager;
  }
  /*set motors to desired voltage */
  ser.set(motr.drive_spin_volts_, Motorcommandr); /*Motorcommandr*/
  ser.set(motl.drive_spin_volts_, Motorcommandl); /*Motorcommandl*/

  Serial.print(" Volt L Comm: ");
  Serial.print(Motorcommandl);


  Serial.print(" Volt R Comm: ");
  Serial.print(Motorcommandr);


  /*Write data to sd card*/

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {


    dataFile.print(" | q1: ");
    dataFile.print(q1);

    dataFile.print(" q2: ");
    dataFile.print(q2);

    dataFile.print(" q3: ");
    dataFile.print(q3);

    dataFile.print(" q4: ");
    dataFile.print(q4);

    dataFile.print(" q5: ");
    dataFile.print(q5);

    dataFile.print(" | dq1: ");
    dataFile.print(dq1);

    dataFile.print(" dq2: ");
    dataFile.print(dq2);

    dataFile.print(" dq3: ");
    dataFile.print(dq3);

    dataFile.print(" dq4: ");
    dataFile.print(dq4);

    dataFile.print(" dq5: ");
    dataFile.print(dq5);

    /*supply voltage data recieve*/

    dataFile.print(" | Motor R Supp Volt: ");
    dataFile.print(voltager);

    dataFile.print(" | Motor L Supp Volt: ");
    dataFile.print(voltagel);


    // Display the reported velocity

    dataFile.print(" | Motor R Velo: ");
    dataFile.print(velocityr);

    dataFile.print(" | Motor L Velo: ");
    dataFile.print(velocityl);

    dataFile.print(" Volt L Comm: ");
    dataFile.print(Motorcommandl);

    dataFile.print(" Volt R Comm: ");
    dataFile.print(Motorcommandr);

    dataFile.println(" ");

    dataFile.close();
  }
  else{
    Serial.print("error opening datalog.txt");
  }
}