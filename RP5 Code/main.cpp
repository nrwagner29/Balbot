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
float q1a[10000]; // approx 90sec of storage with 115Hz // 18000 for 90 sec of storage at 200Hz
float q2a[10000];
float q3a[10000];
float q4a[10000];
float q5a[10000];
float dq1a[10000];
float dq2a[10000];
float dq3a[10000];
float dq4a[10000];
float dq5a[10000];

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
float leftvolta[10000];
float rightvolta[10000];
float zero = 0;
float Tauleft = 0;
float Tauright = 0;
int x = 0; // loop counter
int i = 0; // savedata counter
int buttonpress = 0;

/*code commands*/
float motordrive = 0; // set to 1 to drive motors
float termprint = 1;  // set to 1 to print states to terminal
float recorddata = 1; // set to 1 to save data to a file
float timeprint = 0;  // set to 1 to print time intervals of the code

/*Time vars*/
struct timeval startt, contrt, IMUt, motort, voltsett, printt, recordt, closet;
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
    MultiTurnAngleControlClient rightwheel(0);

    // open a file
    //  Create and open a text file
    ofstream myfile;
    myfile.open("/home/neilrw2/filename.txt");

    // initialize device
    printf("Initializing I2C devices...\n");
    i2c.initialize("/dev/i2c-1");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf("%f", mpu.testConnection());

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


    while (buttonpress == 0)
    {

        /*START LOOP HERE*/
        /*timer start here*/

        gettimeofday(&startt, NULL);
        starttime = startt.tv_usec;

        if (!dmpReady)
        {
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

        /**********************************************************************
         *********************** Sending Get Command **************************
         *********************************************************************/
        leftwheel.obs_angular_displacement_.get(com);  // q4
        rightwheel.obs_angular_displacement_.get(com); // q5
        leftwheel.obs_angular_velocity_.get(com);      // dq4
        rightwheel.obs_angular_velocity_.get(com);     // dq5
        uint8_t packet_buf[64];
        uint8_t length = 8;
        if (com.GetTxBytes(packet_buf, length)) // Get the packet from the com interface and place it into the packet buffer
        {
            std::string string_buf((char *)packet_buf, length); // C is a strong typed language -_- so we need to convert to a string buffer to interface with LibSerial
            my_serial_port.Write(string_buf);                   // Send the get packet request to the motor
        }
        /**********************************************************************
         ************************** Receiving Temp Value **********************
         *********************************************************************/
        usleep(2500);                                        // Need to wait for the Motor Controller to Respond
        std::string read_buf;                                // Serial Receive Buffer
        length = my_serial_port.GetNumberOfBytesAvailable(); // How many bytes are in the read buffer
        my_serial_port.Read(read_buf, length);               // Read the packet from Serial
        uint8_t *cbuf = (uint8_t *)read_buf.c_str();         // Again C is strongly types so we have to convert back to byte buffer
        com.SetRxBytes(cbuf, length);                        // Transfer the buffer into the com interface
        /**************************************************************************
        **************************  Reading the Value  ***************************
        *************************************************************************/
        uint8_t *packet_data; // Temporary Pointer to the packet data location
        uint8_t packet_length;
        com.PeekPacket(&packet_data, &packet_length); // Loads the packet data buffer with data receieved from the motor
        com.DropPacket();
        leftwheel.ReadMsg(packet_data, packet_length); // Loads data into the temperature client
        rightwheel.ReadMsg(packet_data, packet_length);
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

        /*states (q1 and dq1 not used)*/
        q1 = -ypr[0]; // radians
        q1a[x] = q1;
        q2 = ypr[1];
        q2a[x] = q2;
        q3 = ypr[2];
        q3a[x] = q3;

        /* q4 and q5 defined below*/
        dq1 = gyro.z / 16.4 / 180 * PI; // radians
        dq1a[x] = dq1;
        dq2 = -gyro.y / 16.4 / 180 * PI; // radians
        dq2a[x] = dq2;
        dq3 = gyro.x / 16.4 / 180 * PI; // radians
        dq3a[x] = dq3;

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

        /*Controller timer*/
        gettimeofday(&contrt, NULL);
        controllertime = contrt.tv_usec;

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

        if (termprint == 1)
        {
            printf(" LMotor volt: %.3f RMotor volt: %.3f q1: %.3f q2: %.3f q3:%.3f q4: %.3f q5:%.3f dq1: %.3f dq2: %.3f dq3:%.3f dq4: %.3f dq5:%.3f\n", leftvolt, rightvolt, q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5);
            /*printtimer timer*/
            gettimeofday(&printt, NULL);
            printtime = printt.tv_usec;
            printT = printtime - voltagesettime;
        }

        /*time calcs*/
        totalT = starttime - oldstarttime;
        IMUT = IMUtime - starttime;
        motorgetT = motorgettime - IMUtime;
        controllerT = controllertime - motorgettime;
        voltagesetT = voltagesettime - controllertime;
        oldstarttime = starttime;

        if (timeprint == 1)
        {
            printf("Time to run: %.3fms | IMU time: %.3fms | Motor get time: %.3fms | controller time: %.3fms | voltage set time: %.3fms | Terminal Print time: %.3fms \t\r", totalT / 1000, IMUT / 1000, motorgetT / 1000, controllerT / 1000, voltagesetT / 1000, printT / 1000);
        }

        /*exiter on the while loop*/
        if (x == 999)//this is on a loop timer, want to change on key press to do this
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
                    myfile << "q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5, LeftMotorVoltage, RightMotorVoltage";
                    printf( "File Printed\n");
                    while (i <= x)
                    {
                        i++;

                        myfile << q1a[i] << ", " << q2a[i] << ", " << q3a[i] << ", " << q4a[i] << ", " << q5a[i] << ", " << dq1a[i] << "S, " << dq2a[i] << ", " << dq3a[i] << ", " << dq4a[i] << ", " << dq5a[i] << ", " << leftvolta[i] << ", " << rightvolta[i] << "\n";
                        // cout << q1a[i] << ", " << q2a[i] << ", " << q3a[i] << ", " << q4a[i] << ", " << q5a[i] << ", " << dq1a[i] << ", " << dq2a[i] << ", " << dq3a[i] << ", " << dq4a[i] << ", " << dq5a[i] << ", " << leftvolta[i] << ", " << rightvolta[i]<< "\n";
                    }
                }
            }

            myfile.close(); // Close the file
            buttonpress = 1;
        }

        x++;
    }

    return 0;
}
