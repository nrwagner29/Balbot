// Main Code to run BALBOT as an inverted pendulum with the wheels at a 90 deg angle
// #include <list>
#include <iostream>
#include <fstream>
#include <cmath>
#include "lib/brushless_drive_client.hpp"
#include "lib/client_communication.hpp"
#include "lib/coil_temperature_estimator_client.hpp"
#include "lib/esc_propeller_input_parser_client.hpp"
#include "lib/generic_interface.hpp"
#include "lib/iquart_flight_controller_interface_client.hpp"
#include "lib/persistent_memory_client.hpp"
#include "lib/power_monitor_client.hpp"
#include "lib/power_safety_client.hpp"
#include "lib/propeller_motor_control_client.hpp"
#include "lib/pulsing_rectangular_input_parser_client.hpp"
#include "lib/serial_interface_client.hpp"
#include "lib/system_control_client.hpp"
#include "lib/temperature_estimator_client.hpp"
#include "lib/temperature_monitor_uc_client.hpp"
#include "lib/voltage_superposition_client.hpp"
#include "lib/byte_queue.h"
#include "lib/crc_helper.h"
#include "lib/generic_interface.hpp"
#include "lib/packet_finder.h"
#include "lib/SerialPort.h"
#include "lib/SerialStream.h"
#include "lib/multi_turn_angle_control_client.hpp"
#include <string>
#include <iostream>
#include "unistd.h"
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include "lib/MPU6050_6Axis_MotionApps20.cpp"
#include "lib/MPU6050_6Axis_MotionApps20.h"
#include "lib/MPU6050.cpp"
#include "lib/MPU6050.h"
#include "lib/I2Cdev.h"
#include "lib/I2Cdev.cpp"
#include "lib/helper_3dmath.h"
#include "lib/wiringPi.h"
#include "lib/wiringPiI2C.h"



#define PI 3.14159265359
// //WiringPi talkiking to IMU, Change to I2cDev for MPU
// #define Device_Address 0x68 /*Device Address/Identifier for MPU6050*/
// #define PWR_MGMT_1 0x6B
// #define SMPLRT_DIV 0x19
// #define CONFIG 0x1A
// #define GYRO_CONFIG 0x1B
// #define INT_ENABLE 0x38
// #define ACCEL_XOUT_H 0x3B
// #define ACCEL_YOUT_H 0x3D
// #define ACCEL_ZOUT_H 0x3F
// #define GYRO_XOUT_H 0x43
// #define GYRO_YOUT_H 0x45
// #define GYRO_ZOUT_H 0x47

// #include "MPU6050.h" // not necessary if using MotionApps include file

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
// #define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
// #define OUTPUT_READABLE_EULER

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
// #define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
// #define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
// #define OUTPUT_TEAPOT

bool blinkState = false;

MPU6050 mpu;
I2Cdev i2c;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 gyro;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

using namespace LibSerial;
using namespace std;
uint64_t microsec();
uint64_t nanosec();
uint64_t millisec();
int fd;

/*Balbot Parameters*/
float Ixb = 0.0003478;
float Iyb = 0.0004134;
float Izb = 0.0004908;
float Ixwl = 0.00017645;
float Iywl = 0.00017645;
float Izwl = 0.000352;
float Ixwr = 0.00017645;
float Iywr = 0.00017645;
float Izwr = 0.000352;
float bmass = .436;
float wmass = .110;
float gr = 9.81;
float l = .1;

/*Balbot states*/
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

/*code vars*/
float currlimit = 0;
float kt = 0;
float leftcurr = 0;
float rightcurr = 0;
float lemf = 0;
float remf = 0;
float R = 0;
float leftvolt = 0;
float rightvolt = 0;
float zero = 0;
float Tauleft = 0;
float Tauright = 0;

/*code commands*/
float motordrive = 0; // set to 1 to drive motors
float termprint = 1;  // set to 1 to print states to terminal
float recorddata = 1; // set to 1 to save data to a file

// void MPU6050_Init()
// {
//     wiringPiI2CWriteReg8(fd, SMPLRT_DIV, 0x07); /* Write to sample rate register */
//     wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x01); /* Write to power management register */
//     wiringPiI2CWriteReg8(fd, CONFIG, 0);        /* Write to Configuration register */
//     wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 24);  /* Write to Gyro Configuration register */
//     wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01); /*Write to interrupt enable register */

// }
// short read_raw_data(int addr)
// {
//     short high_byte, low_byte, value;
//     high_byte = wiringPiI2CReadReg8(fd, addr);
//     low_byte = wiringPiI2CReadReg8(fd, addr + 1);
//     value = (high_byte << 8) | low_byte;
//     return value;
// }

void ms_delay(int val)
{
    int i, j;
    for (i = 0; i <= val; i++)
        for (j = 0; j < 1200; j++)
            ;
}

int main()
{

    // Setup the serial interface
    SerialPort my_serial_port("/dev/ttyAMA10");
    my_serial_port.SetBaudRate(BaudRate::BAUD_921600);

    // Make a communication interface object
    // This is what creates and parses packets
    GenericInterface com;

    // Make a Temperature Estimator Client object with obj_id 0
    MultiTurnAngleControlClient leftwheel(1);
    BrushlessDriveClient leftdrive(1);
    MultiTurnAngleControlClient rightwheel(0);
    BrushlessDriveClient rightdrive(0);

    // open a file
    //  Create and open a text file
    ofstream MyFile("filename.txt");

    // float Acc_x, Acc_y, Acc_z;
    // float Gyro_x, Gyro_y, Gyro_z;
    // float Ax = 0, Ay = 0, Az = 0;
    // float Gx = 0, Gy = 0, Gz = 0;
    // fd = wiringPiI2CSetup(Device_Address); /*Initializes I2C with device Address this is through wiringPi*/
    i2c.initialize("/dev/i2c-1");
    // MPU6050_Init();                        /* Initializes MPU6050 , though we want to do it through i2cdev*/
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();
    // pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    printf("Testing device connections...\n");
    printf("%f",mpu.testConnection());

    // wait for ready need to find c++ version not arduino
    // printf("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    printf("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...");
        mpu.setDMPEnabled(true);
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed %u", devStatus);
    }

    while (true)
    {
        /*START LOOP HERE*/

        if (!dmpReady){
            return 0;
        }
            
        // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        { // Get the Latest packet
#ifdef OUTPUT_READABLE_QUATERNION
          // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            printf("quat\t");
            printf("%.3f", q.w);
            printf("\t");
            printf("%.3f", q.x);
            printf("\t");
            printf("%.3f", q.y);
            printf("\t");
            printf("%.3f", q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            printf("euler\t");
            printf("%.3f", euler[0] * 180 / M_PI);
            printf("\t");
            printf("%.3f", euler[1] * 180 / M_PI);
            printf("\t");
            printf("%.3f", euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            printf("ypr\t");
            printf("%.3f", ypr[0] * 180 / M_PI);
            printf("\t");
            printf("%.3f", ypr[1] * 180 / M_PI);
            printf("\t");
            printf("%.3f", ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            printf("areal\t");
            printf("%.3f", aaReal.x);
            printf("\t");
            printf("%.3f", aaReal.y);
            printf("\t");
            printf("%.3f", aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            printf("aworld\t");
            printf("%.3f", aaWorld.x);
            printf("\t");
            printf("%.3f", aaWorld.y);
            printf("\t");
            printf("%.3f", aaWorld.z);
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
            printf("%.14f", teapotPacket);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
        }

        /**********************************************************************
         *********************** Sending Get Command **************************
         *********************************************************************/

        /*PUT IMU TO STATES CODE HERE (ALSO DMP)*/
        leftwheel.obs_angular_displacement_.get(com);  // q4
        rightwheel.obs_angular_displacement_.get(com); // q5
        leftwheel.obs_angular_velocity_.get(com);      // dq4
        rightwheel.obs_angular_velocity_.get(com);     // dq5

        uint8_t packet_buf[64];
        uint8_t length = 8;

        // Get the packet from the com interface and place it into the packet buffer
        if (com.GetTxBytes(packet_buf, length))
        {

            // C is a strong typed language -_-
            // so we need to convert to a string buffer to interface with LibSerial
            std::string string_buf((char *)packet_buf, length);

            // Send the get packet request to the motor
            my_serial_port.Write(string_buf);
        }

        /**********************************************************************
         ************************** Receiving Temp Value **********************
         *********************************************************************/

        // Need to wait for the Motor Controller to Respond
        usleep(5000);

        // Serial Receive Buffer
        std::string read_buf;

        // How many bytes are in the read buffer
        length = my_serial_port.GetNumberOfBytesAvailable();

        // Read the packet from Serial
        my_serial_port.Read(read_buf, length);

        // Again C is strongly types so we have to convert back to byte buffer
        uint8_t *cbuf = (uint8_t *)read_buf.c_str();

        // Transfer the buffer into the com interface
        com.SetRxBytes(cbuf, length);

        /**************************************************************************
        **************************  Reading the Value  ***************************
        *************************************************************************/

        // Temporary Pointer to the packet data location
        uint8_t *packet_data;
        uint8_t packet_length;

        // Loads the packet data buffer with data receieved from the motor
        com.PeekPacket(&packet_data, &packet_length);
        com.DropPacket();

        // Loads data into the temperature client
        leftwheel.ReadMsg(packet_data, packet_length);
        rightwheel.ReadMsg(packet_data, packet_length);

        // /*Read raw value of Accelerometer and gyroscope from MPU6050*/ This is all wiringPi code
        // Acc_x = read_raw_data(ACCEL_XOUT_H);
        // Acc_y = read_raw_data(ACCEL_YOUT_H);
        // Acc_z = read_raw_data(ACCEL_ZOUT_H);

        // Gyro_x = read_raw_data(GYRO_XOUT_H);
        // Gyro_y = read_raw_data(GYRO_YOUT_H);
        // Gyro_z = read_raw_data(GYRO_ZOUT_H);

        // /* Divide raw value by sensitivity scale factor */
        // Ax = Acc_x / 16384.0;
        // Ay = Acc_y / 16384.0;
        // Az = Acc_z / 16384.0;

        // Gx = Gyro_x / 131;
        // Gy = Gyro_y / 131;
        // Gz = Gyro_z / 131;

        /*states (q1 and dq1 not used)*/
        q1 = -ypr[0]; // radians
        q2 = ypr[1];
        q3 = ypr[2];
        /* q4 and q5 defined below*/
        dq1 = gyro.z / 16.4 / 180 * PI;  // radians
        dq2 = -gyro.y / 16.4 / 180 * PI; // radians
        dq3 = gyro.x / 16.4 / 180 * PI;  // radians

        /*PUT IMU TO STATES CODE HERE (ALSO DMP)*/
        q4 = leftwheel.obs_angular_displacement_.get_reply();
        q5 = rightwheel.obs_angular_displacement_.get_reply();
        dq4 = leftwheel.obs_angular_velocity_.get_reply();
        dq5 = rightwheel.obs_angular_velocity_.get_reply();
        /*Controller code Here*/

        /*From MATLAB ccode (*recalculate*)*/
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
        /*torque output of controller here*/
        Tauleft = (t18 * (Ixb / 2.0 + Ixwr / 2.0 + Iywl / 2.0 + Izb / 2.0 + Izwl / 2.0 + Izwr / 2.0 + bmass * 8.45E-3 + t8 + (Ixb * t6) / 2.0 + (Ixwr * t6) / 2.0 + (Iywl * t6) / 2.0 - (Izb * t6) / 2.0 - (Izwl * t6) / 2.0 - (Izwr * t6) / 2.0 + bmass * t6 * 8.45E-3 + t6 * t8) + gr * l * t7 * sin(q2)) / t3 - t3 * t18 * (Iywl + t8);
        Tauright = t17 * (Ixwl + Iyb + Iywr + bmass * 1.69E-2 + wmass * 3.38E-2) - t17 * (Iywr + t8) + gr * l * t7 * sin(q3);
        /*End MATLAB ccode*/

        /*torque to current to voltage command*/
        /*torque to current*/
        kt = 0.053;                // torque constant
        leftcurr = Tauleft / kt;   // amps
        rightcurr = Tauright / kt; // amps

        /*current limiter*/
        currlimit = 5; // AMPS
        if (leftcurr > currlimit)
        {
            leftcurr = currlimit;
        }
        else if (leftcurr < -currlimit)
        {
            leftcurr = -currlimit;
        }
        if (rightcurr > currlimit)
        {
            rightcurr = currlimit;
        }
        else if (rightcurr < -currlimit)
        {
            rightcurr = -currlimit;
        }

        /*Get EMF from speed calculation*/
        lemf = kt * dq4; // lwheel velocity times back emf constant, kt to produce the back emf
        remf = kt * dq5;

        /*current to voltage*/
        R = 4.7; // omhs
        leftvolt = lemf + R * leftcurr;
        rightvolt = remf + R * rightcurr;

        if (abs(q2) > .20 | abs(q3) > .20) // rads
        {                                  // motor stop for falling over
            leftdrive.drive_brake_.set(com);
            rightdrive.drive_brake_.set(com);
        }
        else if (motordrive == 1)
        { // Can the motors voltage command be sent from down here or before the TX call?
            leftdrive.drive_spin_volts_.set(com, leftvolt);
            rightdrive.drive_spin_volts_.set(com, rightvolt);
        }
        else if (motordrive == 0)
        {
            leftdrive.drive_spin_volts_.set(com, zero);
            rightdrive.drive_spin_volts_.set(com, zero);
        }

        if (termprint == 1)
        {
            printf(" LMotor Command (v): %.3f RMotor Command (v): %.3f q4: %.3f q5:%.3f\n", leftvolt, rightvolt, q4, q5);
            // printf(states);
        }

        if (recorddata == 1)
        {
            // save states to file here
            // Write to the file
            MyFile << "Files can be tricky, but it is fun enough!";
        }
    }
    // Close the file
    MyFile.close();
    return 0;
}

// Get time stamp in milliseconds.
uint64_t millisec()
{
    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::high_resolution_clock::now().time_since_epoch())
                      .count();
    return ms;
}

// Get time stamp in microseconds.
uint64_t microsec()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::high_resolution_clock::now().time_since_epoch())
                      .count();
    return us;
}

// Get time stamp in nanoseconds.
uint64_t nanosec()
{
    uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::high_resolution_clock::now().time_since_epoch())
                      .count();
    return ns;
}