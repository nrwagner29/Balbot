/*
* Vertiq read motor coil temperature.
*
* This code shows how to use Serial over USB to read the
* motor coil temperature
*
*
* The circuit:
* Connected to FTDI usb to Serial
*
* This example uses:
*   - LibSerial (https://libserial.readthedocs.io/en/latest/index.html)
*
*   This demo works for POSIX supported systems and was ran using Linux Ubuntu 20.04.1 LTS
*
*
* Created 2021/03/31 by Malik B. Parker
*
* This example code is in the public domain.
*/

#include <chrono>

// NB: ALL OF THESE 3 FUNCTIONS BELOW USE SIGNED VALUES INTERNALLY AND WILL
// EVENTUALLY OVERFLOW (AFTER 200+ YEARS OR SO), AFTER WHICH POINT THEY WILL
// HAVE *SIGNED OVERFLOW*, WHICH IS UNDEFINED BEHAVIOR (IE: A BUG) FOR C/C++.
// But...that's ok...this "bug" is designed into the C++11 specification, so
// whatever. Your machine won't run for 200 years anyway...

#include "lib/multi_turn_angle_control_client.hpp"
#include "lib/generic_interface.hpp"
#include "lib/temperature_estimator_client.hpp"
#include "lib/SerialPort.h"
#include "lib/SerialStream.h"
#include "lib/brushless_drive_client.hpp"
#include <string>
#include <cmath>

#include <iostream>
#include "unistd.h"

uint64_t micros();
float zero = 0; // drive 0 volts

using namespace LibSerial;

int main(){

    // Setup the serial interface
    SerialPort my_serial_port("/dev/ttyAMA10");
    my_serial_port.SetBaudRate(BaudRate::BAUD_921600);

    // Make a communication interface object
    // This is what creates and parses packets
    GenericInterface com;

    // Make a Temperature Estimator Client object with obj_id 1

    MultiTurnAngleControlClient angleft(1);
    BrushlessDriveClient leftdrive(1);

    while(true){

        /**********************************************************************
         *********************** Sending Get Command **************************
         *********************************************************************/

         // Forms a packet in the com interface with the following:
        // type:        (77) Temperature Estimator ID Number
        // subtype:     ( 0) temp
        // obj/access   ( 0) get
        angleft.obs_angular_displacement_.get(com);
        zero = sin(.5*micros()/100000);
        

        uint8_t packet_buf[64];
        uint8_t length = 8;

        // Get the packet from the com interface and place it into the packet buffer
        if(com.GetTxBytes(packet_buf, length)){

            // C is a strong typed language -_-
            // so we need to convert to a string buffer to interface with LibSerial
            std::string string_buf((char*)packet_buf, length);

            // Send the get packet request to the motor
            my_serial_port.Write(string_buf);
            printf("TX! ");
        }

        /**********************************************************************
         ************************** Receiving Temp Value **********************
         *********************************************************************/

        // Need to wait for the Motor Controller to Respond
        usleep(5000);

        // Serial Receive Buffer
        std::string read_buf;

        //Checks
        
        // printf("Number of stop bits: %.3f ", my_serial_port.GetStopBits());
        // printf("Size of Character: %.3f ", my_serial_port.GetCharacterSize());
        

    //     if(my_serial_port.IsOpen()){
    //         printf("port is open ");
    //     }
    //    if(my_serial_port.IsDataAvailable()){
    //         printf("port has data ");
    //     } else {
    //         printf(" no data here ");
    //     }
        // How many bytes are in the read buffer
        length = my_serial_port.GetNumberOfBytesAvailable();

        // Read the packet from Serial
        my_serial_port.Read(read_buf, length);

        // Again C is strongly types so we have to convert back to byte buffer
        uint8_t * cbuf = (uint8_t *) read_buf.c_str();

        // Transfer the buffer into the com interface
        if(com.SetRxBytes(cbuf, length)){
            // printf(" packet available ");
        } else{
            //  printf(" no packet here! ");
        }

        /**************************************************************************
        **************************  Reading the Value  ***************************
        *************************************************************************/

        // Temporary Pointer to the packet data location
        uint8_t *packet_data;
        uint8_t packet_length;

        // Loads the packet data buffer with data receieved from the motor
        if(com.PeekPacket(&packet_data, &packet_length)){
            // printf("packet peeked");
        } else {
            // printf(" didnt peek packet ");
        }
        if(com.DropPacket()){
            // printf("packet Dropped");
        } else if(com.DropPacket()==0) {
            // printf("no packet to drop");
        } else {
            printf("failue");
        }
        // Loads data into the temperature client
        angleft.ReadMsg(packet_data, packet_length);

        

        // Reads the data from the temperature client
        float langle = angleft.obs_angular_displacement_.get_reply();

        // Drive the motor via command voltage
        leftdrive.drive_spin_volts_.set(com, zero);

        printf("Motor Angle: %.3f  MotorVolt %.3f \r", langle ,zero);//Packet_data: %u bytes in read buffer: %u packet length: %u, packet_data, length, packet_length


    }

    return 0;
}

// Get time stamp in milliseconds.
uint64_t millis()
{
    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    return ms; 
}

// Get time stamp in microseconds.
uint64_t micros()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    return us; 
}

// Get time stamp in nanoseconds.
uint64_t nanos()
{
    uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    return ns; 
}