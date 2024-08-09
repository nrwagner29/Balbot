#include <iostream>
#include <cmath>
// #include <list>
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
#include "../../usr/include/libserial/SerialPort.h"
#include "../../usr/include/libserial/SerialStream.h"
#include <string>
#include <iostream>
#include "unistd.h"
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>


#define Device_Address 0x68 /*Device Address/Identifier for MPU6050*/
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

using namespace LibSerial;

int fd;
float voltage1;
float voltage2;

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

void MPU6050_Init() // Write offsets here too
{

    wiringPiI2CWriteReg8(fd, SMPLRT_DIV, 0x07); /* Write to sample rate register */
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x01); /* Write to power management register */
    wiringPiI2CWriteReg8(fd, CONFIG, 0);        /* Write to Configuration register */
    wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 24);  /* Write to Gyro Configuration register */
    wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01); /*Write to interrupt enable register */
}
short read_raw_data(int addr)
{
    short high_byte, low_byte, value;
    high_byte = wiringPiI2CReadReg8(fd, addr);
    low_byte = wiringPiI2CReadReg8(fd, addr + 1);
    value = (high_byte << 8) | low_byte;
    return value;
}

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
    SerialPort my_serial_port("/dev/ttyS0");
    my_serial_port.SetBaudRate(BaudRate::BAUD_115200);

    // Make a communication interface object
    // This is what creates and parses packets
    GenericInterface com;

    // Make a Temperature Estimator Client object with obj_id 0
    TemperatureEstimatorClient templeft(2);
    BrushlessDriveClient leftwheel(2);
    TemperatureEstimatorClient tempright(0);
    BrushlessDriveClient rightwheel(0);
    float Acc_x, Acc_y, Acc_z;
    float Gyro_x, Gyro_y, Gyro_z;
    float Ax = 0, Ay = 0, Az = 0;
    float Gx = 0, Gy = 0, Gz = 0;
    fd = wiringPiI2CSetup(Device_Address); /*Initializes I2C with device Address*/
    MPU6050_Init();                        /* Initializes MPU6050 */

    while (true)
    {

        /**********************************************************************
         *********************** Sending Get Command **************************
         *********************************************************************/

        // Forms a packet in the com interface with the following:
        // type:        (77) Temperature Estimator ID Number
        // subtype:     ( 0) temp
        // obj/access   ( 0) get
        templeft.temp_.get(com);
        tempright.temp_.get(com);

        /*PUT IMU TO STATES CODE HERE (ALSO DMP)*/
        leftwheel.obs_angle_.get(com); //q4
        rightwheel.obs_angle_.get(com); //q5
        leftwheel.obs_velocity_.get(com); //dq4
        rightwheel.obs_velocity_.get(com); //dq5
        /*Get EMF Data from each motor*/
        leftwheel.motor_emf_calc_.get(com);   // volts (need to get from motor)
        rightwheel.motor_emf_calc_.get(com); // volts

        uint8_t packet_buf[64];
        uint8_t length = 0;

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

        // Loads data into the temperature client
        templeft.ReadMsg(packet_data, packet_length);
        tempright.ReadMsg(packet_data, packet_length);
        com.DropPacket();

        // Reads the data from the temperature client
        float temperatureleft = templeft.temp_.get_reply();
        float temperatureright = tempright.temp_.get_reply();

        /*Read raw value of Accelerometer and gyroscope from MPU6050*/
        Acc_x = read_raw_data(ACCEL_XOUT_H);
        Acc_y = read_raw_data(ACCEL_YOUT_H);
        Acc_z = read_raw_data(ACCEL_ZOUT_H);

        Gyro_x = read_raw_data(GYRO_XOUT_H);
        Gyro_y = read_raw_data(GYRO_YOUT_H);
        Gyro_z = read_raw_data(GYRO_ZOUT_H);

        /* Divide raw value by sensitivity scale factor */
        Ax = Acc_x / 16384.0;
        Ay = Acc_y / 16384.0;
        Az = Acc_z / 16384.0;

        Gx = Gyro_x / 131;
        Gy = Gyro_y / 131;
        Gz = Gyro_z / 131;

        /*PUT IMU TO STATES CODE HERE (ALSO DMP)*/
        float leftang = leftwheel.obs_angle_.get_reply(); //q4
        float rightang = rightwheel.obs_angle_.get_reply(); //q5
        float leftvelo = leftwheel.obs_velocity_.get_reply(); //dq4
        float rightvelo = rightwheel.obs_velocity_.get_reply(); //dq5
        /*Controller code Here*/
        q4 = leftang;
        q5 = rightang;
        dq4 = leftvelo;
        dq5 = rightvelo;


        /*From MATLAB ccode*/
        float t2 = (q3); // fine as long as q3 is real only
        float t4 = wmass * 2.0;
        float t8 = wmass / 1.0E+2;
        float t9 = q2 * 1.405512958801151E+2;
        float t10 = q3 * 1.416399323695648E+2;
        float t11 = q4 * 2.106736049221242;
        float t12 = q5 * 2.140339324824921;
        float t13 = dq3 * 2.395140112673594E+1;
        float t14 = dq2 * 2.387142724903835E+1;
        float t15 = dq4 * 1.779729334865774;
        float t16 = dq5 * 1.800856349596192;
        float t3 = cos(t2);
        float t5 = t2 * 2.0;
        float t7 = bmass + t4;
        float t17 = t9 + t11 + t14 + t15;
        float t18 = t10 + t12 + t13 + t16;
        float t6 = cos(t5);
        float Tauleft = (t17 * (Ixb / 2.0 + Ixwr / 2.0 + Iywl / 2.0 + Izb / 2.0 + Izwl / 2.0 + Izwr / 2.0 + bmass / 2.0E+2 + t8 + (Ixb * t6) / 2.0 + (Ixwr * t6) / 2.0 + (Iywl * t6) / 2.0 - (Izb * t6) / 2.0 - (Izwl * t6) / 2.0 - (Izwr * t6) / 2.0 + (bmass * t6) / 2.0E+2 + t6 * t8) + gr * l * t7 * sin(q2)) / t3 - t3 * t17 * (Iywl + t8);
        float Tauright = t18 * (Ixwl + Iyb + Iywr + bmass / 1.0E+2 + wmass / 5.0E+1) - t18 * (Iywr + t8) + gr * l * t7 * sin(q3);
        /*torque output of controller here*/
        /*End MATLAB ccode*/

        /*torque to current to voltage command*/
        /*torque to current*/
        float kt = 0.053;                // torque constant
        float leftcurr = Tauleft / kt;   // amps
        float rightcurr = Tauright / kt; // amps

        /*current to voltage*/
        /*Get EMF Data from each motor*/
        float leftemf = leftwheel.motor_emf_calc_.get_reply();   // volts (need to get from motor)
        float rightemf = rightwheel.motor_emf_calc_.get_reply(); // volts
        float R = 4.7;                                     // omhs
        printf("left EMF: %.3f  right EMF: %.3f ", leftemf, rightemf);
        float leftvolt = leftemf + R * leftcurr;
        float rightvolt = rightemf + R * rightcurr;

        leftwheel.drive_spin_volts_.set(com, leftvolt);
        rightwheel.drive_spin_volts_.set(com, rightvolt);
        printf("Gx=%.3f°/s Gy=%.3f°/s Gz=%.3f°/s Ax=%.3fg Ay=%.3fg Az=%.3fg  LMotor Command (v): %.3f RMotor Command (v): %.3f \r", Gx, Gy, Gz, Ax, Ay, Az, voltage1, voltage2);
        ms_delay(10);
    }

    return 0;
}