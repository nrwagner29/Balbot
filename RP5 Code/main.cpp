// Main Code to run BALBOT as an inverted pendulum with the wheels at a 90 deg angle

#pragma region // start code and initalize all variables and classes for communication

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
#include <cstdio>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include "lib/wiringPi.h"
#include "lib/wiringPiI2C.h"
#include "lib/bno085_driver.h"
#include "lib/sh2.h"
#include "lib/sh2_err.h"
#include "lib/sh2_hal.h"
#include "lib/sh2_SensorValue.h"
#include "lib/sh2_util.h"
#include "lib/shtp.h"
#define PI 3.14159265359

BNO085_IMU bno085;
sh2_SensorValue_t sensorValue;

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
float q1a[1000000]; // approx 90sec of storage with 115Hz // 18000 for 90 sec of storage at 200Hz
float q2a[1000000];
float q3a[1000000];
float q4a[1000000];
float q5a[1000000];
float dq1a[1000000];
float dq2a[1000000];
float dq3a[1000000];
float dq4a[1000000];
float dq5a[1000000];
float qw = 0;
float qx = 0;
float qy = 0;
float qz = 0;
float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

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
float leftvolta[1000000];
float rightvolta[1000000];
float leftcurra[1000000];
float rightcurra[1000000];
float Taulefta[1000000];
float Taurighta[1000000];
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
struct euler_t
{
    float yaw;
    float pitch;
    float roll;
} ypr;

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
float startTa[1000000];
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

#pragma endregion

/*code commands*/
float motordrive = 1; // set to 1 to drive motors
float termprint = 1;  // set to 1 to print states to terminal
float recorddata = 1; // set to 1 to save data to a file
float timeprint = 0;  // set to 1 to print time intervals of the code

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
    if (recorddata == 1)
    {

        cout << "enter trial number";
        cin >> trial;
        cout << "enter date: ";
        cout << "day: ";
        cin >> day;
        cout << "month: ";
        cin >> month;
        cout << "year: ";
        cin >> year;
        filename = "/home/neilrw2/trial_" + trial + "--" + year + "-" + month + "-" + day + ".csv";
        cout << "\n"
             << filename;
        myfile.open(filename);
    }

    // initialize device
    printf("Initializing I2C devices...\n");
    if (!bno085.begin_I2C())
    {
        cout << "Failed to find BNO085 chip" << endl;
        while (true)
            ;
    }
    if (!bno085.enableReport(SH2_GYRO_INTEGRATED_RV, 1000))
    {
        cout << "Could not enable Gyro-Integrated Rotation Vector report" << endl;
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
#pragma region // IMU and Motor Communications and receiving data
        /*START LOOP HERE*/
        /*timer start here*/
        gettimeofday(&startt, NULL);
        starttime = startt.tv_usec;

        if (bno085.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_GYRO_INTEGRATED_RV)
            {
                qw = sensorValue.un.gyroIntegratedRV.real;
                qx = sensorValue.un.gyroIntegratedRV.i;
                qy = sensorValue.un.gyroIntegratedRV.j;
                qz = sensorValue.un.gyroIntegratedRV.k;
                gyrox = sensorValue.un.gyroIntegratedRV.angVelX;
                gyroy = sensorValue.un.gyroIntegratedRV.angVelY;
                gyroz = sensorValue.un.gyroIntegratedRV.angVelZ;

                // printf("FastVec: %.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f", qw, qx, qy, qz, gyrox, gyroy, gyroz);
            }
        }

        /*IMU timer*/
        gettimeofday(&IMUt, NULL);
        IMUtime = IMUt.tv_usec;

        /*states (q1 and dq1 not used)*/
        // from quaternion
        q1 = atan((2 * (qw * qz - qx * qy)) / (1 - 2 * (qz * qz + qx * qx))); // radians
        q2 = asin((2 * (qw * qx + qz * qy)));
        q3 = atan((2 * (qw * qy - qz * qx)) / (1 - 2 * (qx * qx + qy * qy)));
        q1a[x] = q1;
        q2a[x] = q2;
        q3a[x] = q3;

        /*multiply through augmented jacobian to produce dq values from angular velocities*/
        dq1 = (sin(q1) * sin(q2)) / (cos(q2) * cos(q1) * cos(q2) * cos(q1) + cos(q2) * sin(q1) * cos(q2) * sin(q1)) * gyrox + -(cos(q1) * sin(q2)) / (cos(q2) * cos(q1) * cos(q2) * cos(q1) + cos(q2) * sin(q1) * cos(q2) * sin(q1)) * gyroy + gyroz; // radians/second
        dq1a[x] = dq1;
        dq2 = cos(q1) / (cos(q1) * cos(q1) + sin(q1) * sin(q1)) * gyrox + sin(q1) / (cos(q1) * cos(q1) + sin(q1) * sin(q1)) * gyroy; // radians/second
        dq2a[x] = dq2;
        dq3 = -sin(q1) / (cos(q2) * cos(q1) * cos(q2) * cos(q1) + cos(q2) * sin(q1) * cos(q2) * sin(q1)) * gyrox + cos(q1) / (cos(q2) * cos(q1) * cos(q2) * cos(q1) + cos(q2) * sin(q1) * cos(q2) * sin(q1)) * gyroy; // radians/second
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
        // featherstone control law x (q2)
        // Yx1 = -1 / (tcx * tcx * gwx);
        // Yx2 = 1 / (gwx);

        // Mx = tcx * tcx * (dq2 - gwx * dq4);
        // Mdx = q2;
        // Mddx = dq2;

        // f = .5; // pole factor (larger to increase mag of poles)

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

        float kp = 0;
        float kd = 1;
        // float offleft = 0;
        // float doffleft = 0;
        // Tauleft = -kp*(offleft - q2) + -kd * (doffleft - dq2);

        float offright = 0;
        float doffright = 0;
        Tauright = -kp * (offright - q3) + -kd * (doffright - dq3);

        // IMU based control
        //  Tauleft =  .1*q2;
        //  Tauright = -.1*q3;

        // Sinusoidal control

        // Tauleft = .1*sin(starttime/1000000 * 6.28);
        // Tauright = .1*cos(starttime/1000000 * 6.28);
        Tauleft = 0;
        // Tauright = 0.01;
        Taulefta[x] = Tauleft;
        Taurighta[x] = Tauright;
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
        leftcurra[x] = leftcurr;
        rightcurra[x] = rightcurr;
        /*Get EMF from speed calculation*/
        lemf = kt * dq4; // lwheel velocity times back emf constant, kt to produce the back emf
        remf = kt * dq5; // dq is rad/s to volt as kt is V*s/rad

        /*current to voltage*/
        R = 4.7; // omhs
        leftvolt = lemf + R * leftcurr;
        leftvolta[x] = leftvolt;
        rightvolt = remf + R * rightcurr;
        rightvolta[x] = rightvolt;
        /*Motor commands here*/
        if (abs(q2) > .35 | abs(q3) > .45) // rads
        {                                  // motor stop for falling over
            leftwheel.ctrl_brake_.set(com);
            rightwheel.ctrl_brake_.set(com);
        }
        else if (motordrive == 1)
        { // Can the motors voltage command be sent from down here or before the TX call? Yes
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
            printf(" LMotor tau: %.3f \tRMotor tau: %.3f \tLMotor volt: %.3f \tRMotor volt: %.3f \tq1: %.3f \tq2: %.3f \tq3:%.3f \tq4: %.3f \tq5:%.3f \tdq1: %.3f \tdq2: %.3f \tdq3:%.3f \tdq4: %.3f \tdq5:%.3f\n", Tauleft, Tauright, leftvolt, rightvolt, q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5);
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
                    myfile << "loop time, q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5, LeftMotorVoltage, RightMotorVoltage, LeftMotorCurrent, RightMotorCurrent, LeftMotorTorque, RightMotorTorque\n";
                    printf("File Printed\n");
                    while (i < x)
                    {
                        i++;

                        myfile << startTa[i] << ", " << q1a[i] << ", " << q2a[i] << ", " << q3a[i] << ", " << q4a[i] << ", " << q5a[i] << ", " << dq1a[i] << ", " << dq2a[i] << ", " << dq3a[i] << ", " << dq4a[i] << ", " << dq5a[i] << ", " << leftvolta[i] << ", " << rightvolta[i] << ", " << leftcurra[i] << ", " << rightcurra[i] << ", " << Taulefta[i] << ", " << Taurighta[i] << "\n";
                        // cout << startTa[i] << ", " << q1a[i] << ", " << q2a[i] << ", " << q3a[i] << ", " << q4a[i] << ", " << q5a[i] << ", " << dq1a[i] << ", " << dq2a[i] << ", " << dq3a[i] << ", " << dq4a[i] << ", " << dq5a[i] << ", " << leftvolta[i] << ", " << rightvolta[i] << ", " << leftcurra[i] << ", " << rightcurra[i] << ", " << Taulefta[i] << ", " << Taurighta[i] << "\n";
                    }
                }
            }
            leftwheel.ctrl_brake_.set(com);
            rightwheel.ctrl_brake_.set(com);
            myfile.close(); // Close the file
            buttonpress = 1;
        }
#pragma endregion
    }

    return 0;
}
