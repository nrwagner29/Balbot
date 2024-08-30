// Main Code to run BALBOT as an inverted pendulum with the wheels at a 90 deg angle
// #include <list>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sys/time.h>
#include <time.h>
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

#pragma region // start code and initalize all variables and classes for communication
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
int fd;

/*Balbot Parameters*/
float Ixb = 0.00012628;
float Iyb = 0.00028395;
float Izb = 0.0003798;
float Ixwl = 0.0002695;
float Iywl = 0.0002695;
float Izwl = 0.000539;
float Ixwr = 0.0002695;
float Iywr = 0.0002695;
float Izwr = 0.000539;
float bmass = .259; //.457 w battery
float wmass = .111;
float gr = 9.81;
float l = .13817;
float m = bmass + 2 * wmass;

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
float q1a[100000]; // approx 90sec of storage with 115Hz // 18000 for 90 sec of storage at 200Hz
float q2a[100000];
float q3a[100000];
float q4a[100000];
float q5a[100000];
float dq1a[100000];
float dq2a[100000];
float dq3a[100000];
float dq4a[100000];
float dq5a[100000];

/*code vars*/
float currlimit = 0;
float kt = 0;
float leftcurr = 0;
float rightcurr = 0;
float lcurr = 0;
float rcurr = 0;
float lemf = 0;
float remf = 0;
float R = 0;
float leftvolt = 0;
float rightvolt = 0;
float leftvolta[100000];
float rightvolta[100000];
float zero = 0;
float Tauleft = 0;
float Tauright = 0;
int x = 0; // loop counter
int i = 0; // savedata counter
int buttonpress = 0;
int packet = 0;
string trial;
string day;
string month;
string year;
string filename;


/*Time vars*/
struct timeval startt, contrt, IMUt, motorsendt, motort, voltsett, printt, recordt, closet;
float starttime = 0;
float IMUtime = 0;
float motorgettime = 0;
float voltagesettime = 0;
float printtime = 0;
float datarecordtime = 0;
float controllertime = 0;
float closetime = 0;
float oldstarttime = 0;
float startT = 0;
float IMUT = 0;
float motorgetT = 0;
float voltagesetT = 0;
float printT = 0;
float datarecordT = 0;
float controllerT = 0;
float closeT = 0;
float totalT = 0;
float startTa[100000];
float motorsendtime = 0;
float motorsendT = 0;
float totalmotorT = 0;

/*Controller Vars*/
float tcx = 0.12169;
float tcy = 0.12239;
float gwx = -0.17527;
float gwy = -0.17327;
float H22 = 0.0136;
float H24 = 0.0024;
float H42 = 0.0024;
float H44 = 0.0024;
float H33 = 0.0138;
float H35 = 0.0024;
float H53 = 0.0024;
float H55 = 0.0024;
float Yx1 = 0;
float Yx2 = 0;
float Yy1 = 0;
float Yy2 = 0;
float Mx = 0;
float Mdx = 0;
float Mddx = 0;
float f = .25;
float g1x = 0;
float g2x = 0;
float g3x = 0;
float g4x = 0;
float a3x = 0;
float a2x = 0;
float a1x = 0;
float a0x = 0;
float kddx = 0;
float kdx = 0;
float kmx = 0;
float kqx = 0;
float mu1x = 0;
float mu2x = 0;
float alphax1 = 0;
float alphax2 = 0;
float qcx = 0;
float dqcx = 0;
float ddqcx = 0;
float ux = 0;
float Mdddx = 0;
float My = 0;
float Mdy = 0;
float Mddy = 0;
float g1y = 0;
float g2y = 0;
float g3y = 0;
float g4y = 0;
float a3y = 0;
float a2y = 0;
float a1y = 0;
float a0y = 0;
float kddy = 0;
float kdy = 0;
float kmy = 0;
float kqy = 0;
float mu1y = 0;
float mu2y = 0;
float alphay1 = 0;
float alphay2 = 0;
float qcy = 0;
float dqcy = 0;
float ddqcy = 0;
float uy = 0;
float Mdddy = 0;


/*code commands*/
float motordrive = 1; // set to 1 to drive motors
float termprint = 1;  // set to 1 to print states to terminal
float recorddata = 1; // set to 1 to save data to a file
float timeprint = 0;  // set to 1 to print time intervals of the code

#pragma endregion

int main()
{
#pragma region // start up, initialization of ports, motors, and IMU
    // Setup the serial interface
    SerialPort my_serial_port("/dev/ttyAMA10");
    my_serial_port.SetBaudRate(BaudRate::BAUD_921600);

    // Make a communication interface object
    // This is what creates and parses packets
    GenericInterface com;

    // Make a Temperature Estimator Client object with obj_id 0
    MultiTurnAngleControlClient leftwheel(1);
    MultiTurnAngleControlClient rightwheel(0);

    // open a file
    //  Create and open a text file
    ofstream myfile;
    cout << "enter trial number";
    cin >> trial;
    cout << "enter date: ";
    cout <<  "day: ";
    cin >> day;
    cout << "month: ";
    cin >> month;
    cout << "year: ";
    cin >> year;
    filename = "/home/neilrw2/trial_" + trial + "--" + year + "-" + month + "-" + day + ".csv";
    cout << "\n" << filename;
    myfile.open(filename);

    // initialize device
    printf("Initializing I2C devices...\n");
    i2c.initialize("/dev/i2c-1");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    // printf("%f", mpu.testConnection());
    mpu.testConnection();
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
    /*set motors to zero voltage*/
    leftwheel.ctrl_volts_.set(com, zero);
    rightwheel.ctrl_volts_.set(com, zero);
    /*set motors to zero angle*/
    leftwheel.obs_angular_displacement_.set(com, zero);
    rightwheel.obs_angular_displacement_.set(com, zero);

#pragma endregion

    while (buttonpress == 0)
    {
        /*START LOOP HERE*/
        /*timer start here*/
        gettimeofday(&startt, NULL);
        starttime = startt.tv_usec;

#pragma region // IMU and Motor Communications and receiving data

        if (!dmpReady)
        {
            printf("DMP not ready");
        }
        else if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) // read a packet from FIFO
        {                                                 // Get the Latest packet
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
            mpu.dmpGetGyro(&gyro, fifoBuffer);
            // printf("ypr\t");
            // printf("%.3f", ypr[0] * 180 / M_PI);
            // printf("\t");
            // printf("%.3f", ypr[1] * 180 / M_PI);
            // printf("\t");
            // printf("%.3f", ypr[2] * 180 / M_PI);
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
        /*IMU timer*/
        gettimeofday(&IMUt, NULL);
        IMUtime = IMUt.tv_usec;

        /*states (q1 and dq1 not used)*/
        // from quaternion
        q1 = atan((2 * (q.w * q.z - q.x * q.y)) / (1 - 2 * (q.z * q.z + q.x * q.x))); // radians
        q2 = asin((2 * (q.w * q.x + q.z * q.y)));
        q3 = atan((2 * (q.w * q.y - q.z * q.x)) / (1 - 2 * (q.x * q.x + q.y * q.y)));
        q1a[x] = q1;
        q2a[x] = q2;
        q3a[x] = q3;

        /* q4 and q5 defined below after motor response*/
        dq1 = gyro.z / 16.4 / 180 * PI; // radians
        dq1a[x] = dq1;
        dq2 = gyro.y / 16.4 / 180 * PI; // radians
        dq2a[x] = dq2;
        dq3 = gyro.x / 16.4 / 180 * PI; // radians
        dq3a[x] = dq3;

        /**********************************************************************
         *********************** Sending Get Command **************************
         *********************************************************************/
        uint8_t packet_buf[64];
        uint8_t length = 8;
        uint8_t *packet_data; // Temporary Pointer to the packet data location
        uint8_t packet_length;
        /*timer start here*/
        gettimeofday(&motorsendt, NULL);
        motorsendtime = motorsendt.tv_usec;

        while (packet == 0)
        {                                                  // Loads the packet data buffer with data receieved from the motor
            leftwheel.obs_angular_displacement_.get(com);  // q4
            rightwheel.obs_angular_displacement_.get(com); // q5
            leftwheel.obs_angular_velocity_.get(com);      // dq4
            rightwheel.obs_angular_velocity_.get(com);     // dq5

            if (com.GetTxBytes(packet_buf, length)) // Get the packet from the com interface and place it into the packet buffer
            {
                std::string string_buf((char *)packet_buf, length); // C is a strong typed language -_- so we need to convert to a string buffer to interface with LibSerial
                my_serial_port.Write(string_buf);                   // Send the get packet request to the motor
            }

            /**********************************************************************
             ************************** Receiving Values **********************
             *********************************************************************/
            // Need to wait for the Motor Controller to Respond

            std::string read_buf;                                // Serial Receive Buffer
            length = my_serial_port.GetNumberOfBytesAvailable(); // How many bytes are in the read buffer
            my_serial_port.Read(read_buf, length);               // Read the packet from Serial
            uint8_t *cbuf = (uint8_t *)read_buf.c_str();         // Again C is strongly types so we have to convert back to byte buffer
            com.SetRxBytes(cbuf, length);                        // Transfer the buffer into the com interface

            /**************************************************************************
            **************************  Reading the Value  ***************************
            *************************************************************************/

            packet = com.PeekPacket(&packet_data, &packet_length);
            // cout << packet << "\n";
        }
        packet = 0;
        leftwheel.ReadMsg(packet_data, packet_length); // Loads data into the temperature client
        rightwheel.ReadMsg(packet_data, packet_length);
        com.DropPacket();
        x++;
        /*PUT IMU TO STATES CODE HERE (ALSO DMP)*/
        q4 = leftwheel.obs_angular_displacement_.get_reply();
        q4a[x] = q4;
        q5 = rightwheel.obs_angular_displacement_.get_reply();
        q5a[x] = q5;
        dq4 = leftwheel.obs_angular_velocity_.get_reply();
        dq4a[x] = dq4;
        dq5 = rightwheel.obs_angular_velocity_.get_reply();
        dq5a[x] = dq5;

        /*Motor Communication timer*/
        gettimeofday(&motort, NULL);
        motorgettime = motort.tv_usec;

#pragma endregion

#pragma region // Controller code and torque command
        ////////////////////////
        /*Controller code Here*/
        ////////////////////////
        // /*From MATLAB ccode (*recalculate*)*/
        // float t2 = q3;
        // float t4 = wmass * 2.0;
        // float t8 = q3 * 1.189149697475401E+2;
        // float t9 = q2 * 1.199641144518069E+2;
        // float t10 = q5 * 9.010851487501317E-1;
        // float t11 = q4 * 9.177937453718946E-1;
        // float t12 = wmass * 1.78623225E-2;
        // float t13 = dq3 * 2.222248647904258E+1;
        // float t14 = dq2 * 2.230524933343801E+1;
        // float t15 = dq4 * 8.437080976593504E-1;
        // float t16 = dq5 * 8.323297792399134E-1;
        // float t3 = cos(t2);
        // float t5 = t2 * 2.0;
        // float t7 = bmass + t4;
        // float t17 = t9 + t11 + t14 + t15;
        // float t18 = t8 + t10 + t13 + t16;
        // float t6 = cos(t5);
        // /*torque output of controller here*/
        // Tauleft = (t17 * (Ixb / 2.0 + Ixwr / 2.0 + Iywl / 2.0 + Izb / 2.0 + Izwl / 2.0 + Izwr / 2.0 + bmass * 8.93116125E-3 + t12 + (Ixb * t6) / 2.0 + (Ixwr * t6) / 2.0 + (Iywl * t6) / 2.0 - (Izb * t6) / 2.0 - (Izwl * t6) / 2.0 - (Izwr * t6) / 2.0 + bmass * t6 * 8.93116125E-3 + t6 * t12) + gr * l * t7 * sin(q2)) / t3 - t3 * t17 * (Iywl + t12);
        // Tauright = t18 * (Ixwl + Iyb + Iywr + bmass * 1.78623225E-2 + wmass * 3.5724645E-2) - t18 * (Iywr + t12) + gr * l * t7 * sin(q3);
        // /*End MATLAB ccode*/

        // mass matrix calcs

        // featherstone control law x (q2)
        // Yx1 = -1 / (tcx * tcx * gwx);
        // Yx2 = 1 / (gwx);

        // Mx = tcx * tcx * (dq2 - gwx * dq4);
        // Mdx = q2;
        // Mddx = dq2;

        // f = .25; // pole factor (larger to increase mag of poles)

        // // selection of poles
        // g1x = -1 / tcx * f;
        // g2x = -1 / tcx * f;
        // g3x = -1 / tcx * f;
        // g4x = -10 * f;

        // a3x = -g1x - g2x - g3x - g4x;
        // a2x = g1x * g2x + g1x * g3x + g1x * g4x + g2x * g3x + g2x * g4x + g3x * g4x;
        // a1x = -g1x * g2x * g3x - g1x * g2x * g4x - g1x * g3x * g4x - g2x * g3x * g4x;
        // a0x = g1x * g2x * g3x * g4x;

        // kddx = -a3x;
        // kdx = -a2x + a0x * Yx2 / Yx1;
        // kmx = -a1x;
        // kqx = 0; //-a0x / Yx1;

        // // location of zeros
        // mu1x = g2x * 100;
        // mu2x = g3x * 100;

        // alphax1 = -(mu1x + mu2x) / (mu1x * mu2x);
        // alphax2 = 1 / (mu1x * mu2x);

        // // qc are commanded values of q4
        // qcx = 0;
        // dqcx = 0;
        // ddqcx = 0;

        // ux = qcx + alphax1 * dqcx + alphax2 * ddqcx;

        // Mdddx = (kddx * Mddx + kdx * Mdx + kmx * Mx + kqx * (q4 - ux));

        // Tauleft = H42 * Mdddx + H44 * (m * l * gr * sin(q2) - H22 * Mdddx) / H24;

        // // featherstone control law y (q3)
        // Yy1 = -1 / (tcy * tcy * gwy);
        // Yy2 = 1 / (gwy);

        // My = tcy * tcy * (dq3 - gwy * dq5);
        // Mdy = q3;
        // Mddy = dq3;

        // // selection of poles
        // g1y = -1 / tcy * f;
        // g2y = -1 / tcy * f;
        // g3y = -1 / tcy * f;
        // g4y = -10 * f;

        // a3y = -g1y - g2y - g3y - g4y;
        // a2y = g1y * g2y + g1y * g3y + g1y * g4y + g2y * g3y + g2y * g4y + g3y * g4y;
        // a1y = -g1y * g2y * g3y - g1y * g2y * g4y - g1y * g3y * g4y - g2y * g3y * g4y;
        // a0y = g1y * g2y * g3y * g4y;

        // kddy = -a3y;
        // kdy = -a2y + a0y * Yy2 / Yy1;
        // kmy = -a1y;
        // kqy = 0; //-a0y / Yy1;

        // // location of zeros
        // mu1y = g2y;
        // mu2y = g3y;

        // alphay1 = -(mu1y + mu2y) / (mu1y * mu2y);
        // alphay2 = 1 / (mu1y * mu2y);

        // // qc are commanded values of q5
        // qcy = 0;
        // dqcy = 0;
        // ddqcy = 0;

        // uy = qcy + alphay1 * dqcy + alphay2 * ddqcy;

        // Mdddy = (kddy * Mddy + kdy * Mdy + kmy * My + kqy * (q5 - uy));

        // Tauright = H53 * Mdddy + H55 * (m * l * gr * sin(q3) - H33 * Mdddy) / H35;


        // PD controller
        
        float kp = 1;
        float kd = .1;
        float offleft = 0;
        float doffleft = 0;
        Tauleft = -kp*(offleft - q2) + -kd * (doffleft - dq2);

        float offright = 0;
        float doffright = 0;
        Tauright = -kp*(offright - q3) + -kd * (doffright - dq3);

        /*Controller timer*/
        gettimeofday(&contrt, NULL);
        controllertime = contrt.tv_usec;
        //////////////////
        /*end Controller*/
        //////////////////

#pragma endregion

#pragma region // torque to voltage command, current limiter and voltage command to motors + fall detection
        /*torque to current to voltage command*/
        /*torque to current*/
        kt = 0.053;                // torque constant
        leftcurr = Tauleft / kt;   // amps
        rightcurr = Tauright / kt; // amps
        rcurr = rightcurr;
        lcurr = leftcurr;
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
        lemf = kt * dq4 *60/PI ; // lwheel velocity times back emf constant, kt to produce the back emf
        remf = kt * dq5 *60/PI ; //changes dq from rad/s to RPM then to volt as kt is volt/PRM

        /*current to voltage*/
        R = 4.7; // omhs
        leftvolt = lemf + R * leftcurr;
        leftvolta[x] = leftvolt;
        rightvolt = remf + R * rightcurr;
        rightvolta[x] = rightvolt;
        /*Motor commands here*/
        if (abs(q2) > .20 | abs(q3) > .20) // rads
        {                                  // motor stop for falling over
            leftwheel.ctrl_brake_.set(com);
            rightwheel.ctrl_brake_.set(com);
        }
        else if (motordrive == 1)
        { // Can the motors voltage command be sent from down here or before the TX call?
            leftwheel.ctrl_volts_.set(com, leftvolt);
            rightwheel.ctrl_volts_.set(com, rightvolt);
        }
        else if (motordrive == 0)
        {
            leftwheel.ctrl_volts_.set(com, zero);
            rightwheel.ctrl_volts_.set(com, zero);
        }

        /*voltage set timer*/
        gettimeofday(&voltsett, NULL);
        voltagesettime = voltsett.tv_usec;

#pragma endregion

#pragma region // terminal print, time calculations, data storage and offload, time print
        if (termprint == 1)
        {
            printf(" LMotor curr: %.3f \tRMotor curr: %.3f \tLMotor volt: %.3f \tRMotor volt: %.3f \tq1: %.3f \tq2: %.3f \tq3:%.3f \tq4: %.3f \tq5:%.3f \tdq1: %.3f \tdq2: %.3f \tdq3:%.3f \tdq4: %.3f \tdq5:%.3f\n", lcurr, rcurr, leftvolt, rightvolt, q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5);
            /*printtimer timer*/
            gettimeofday(&printt, NULL);
            printtime = printt.tv_usec;
            printT = printtime - voltagesettime;
        }

        /*time calcs*/
        totalT = starttime - oldstarttime;
        startTa[x] = starttime;
        IMUT = IMUtime - starttime;
        motorsendT = motorsendtime - IMUtime;
        motorgetT = motorgettime - motorsendtime;
        controllerT = controllertime - motorgettime;
        voltagesetT = voltagesettime - controllertime;
        
        oldstarttime = starttime;

        if (timeprint == 1)
        {
            printf("Time to run: %.3fms \t| Motor send time: %.3fms \t| IMU time: %.3fms \t| Motor get time: %.3fms \t| controller time: %.3fms \t| voltage set time: %.3fms \t| Terminal Print time: %.3fms\t\n", totalT / 1000, motorsendT / 1000, IMUT / 1000, motorgetT / 1000, controllerT / 1000, voltagesetT / 1000, printT / 1000);
        }

        /*exiter on the while loop*/
        if (x == 9999) // this is on a loop timer, want to change on key press to do this size of txt file based on x for amount of rows
        {

            /*THIS IS END OF CODE here are the closing statements to safely exit code and shutdown motors/IMU */
            leftwheel.ctrl_brake_.set(com);
            rightwheel.ctrl_brake_.set(com);

            /*save data array to file in csv*/
            if (recorddata == 1)
            {

                if (myfile.is_open())
                {
                    // save states to file here
                    // Write to the file
                    myfile << "loop time, q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5, LeftMotorVoltage, RightMotorVoltage\n";
                    printf("File Printed\n");
                    while (i < x)
                    {
                        i++;
                        leftwheel.ctrl_brake_.set(com);
                        rightwheel.ctrl_brake_.set(com);
                        myfile << startTa[i] << ", " << q1a[i] << ", " << q2a[i] << ", " << q3a[i] << ", " << q4a[i] << ", " << q5a[i] << ", " << dq1a[i] << ", " << dq2a[i] << ", " << dq3a[i] << ", " << dq4a[i] << ", " << dq5a[i] << ", " << leftvolta[i] << ", " << rightvolta[i] << "\n";
                        // cout << startTa[i] << ", " << q1a[i] << ", " << q2a[i] << ", " << q3a[i] << ", " << q4a[i] << ", " << q5a[i] << ", " << dq1a[i] << ", " << dq2a[i] << ", " << dq3a[i] << ", " << dq4a[i] << ", " << dq5a[i] << ", " << leftvolta[i] << ", " << rightvolta[i]<< "\n";
                    }
                }
            }

            myfile.close(); // Close the file
            buttonpress = 1;
        }
#pragma endregion
    }

    return 0;
}
