#include <iostream>
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

void MPU6050_Init() //Write offsets here too
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
    TemperatureEstimatorClient tempright(1);
    BrushlessDriveClient rightwheel(1);
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

        voltage1 = Ay * 3;
        voltage2 = Az * 3;
        leftwheel.drive_spin_volts_.set(com, voltage1);
        rightwheel.drive_spin_volts_.set(com, voltage2);
        printf("Gx=%.3f°/s Gy=%.3f°/s Gz=%.3f°/s Ax=%.3fg Ay=%.3fg Az=%.3fg LMotor Temperature: %.3f LMotor Command (v): %.3f RMotor Temperature: %.3f RMotor Command (v): %.3f \r", Gx, Gy, Gz, Ax, Ay, Az, temperatureleft, voltage1, temperatureright, voltage2);
        ms_delay(10);
    }

    return 0;
}