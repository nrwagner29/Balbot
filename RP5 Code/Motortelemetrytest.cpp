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


#include "lib/generic_interface.hpp"
#include "lib/temperature_estimator_client.hpp"
#include "lib/SerialPort.h"
#include "lib/SerialStream.h"

#include <string>
#include <iostream>
#include "unistd.h"

using namespace LibSerial;

int main(){

    // Setup the serial interface
    SerialPort my_serial_port("/dev/ttyS0");
    my_serial_port.SetBaudRate(BaudRate::BAUD_115200);

    // Make a communication interface object
    // This is what creates and parses packets
    GenericInterface com;

    // Make a Temperature Estimator Client object with obj_id 0
    TemperatureEstimatorClient templeft(2);


    while(true){

        /**********************************************************************
         *********************** Sending Get Command **************************
         *********************************************************************/

         // Forms a packet in the com interface with the following:
        // type:        (77) Temperature Estimator ID Number
        // subtype:     ( 0) temp
        // obj/access   ( 0) get
        templeft.temp_.get(com);

        uint8_t packet_buf[64];
        uint8_t length = 0;

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
        // usleep(5000);

        // Serial Receive Buffer
        std::string read_buf;

        //Checks
        
        printf("Number of stop bits: %.0f ", my_serial_port.GetStopBits());
        printf("Size of Character: %.0f ", my_serial_port.GetCharacterSize());
        

        if(my_serial_port.IsOpen()){
            printf("port is open ");
        }
       if(my_serial_port.IsDataAvailable()){
            printf("port has data ");
        }
        // How many bytes are in the read buffer
        length = my_serial_port.GetNumberOfBytesAvailable();

        // Read the packet from Serial
        my_serial_port.Read(read_buf, length);

        // Again C is strongly types so we have to convert back to byte buffer
        uint8_t * cbuf = (uint8_t *) read_buf.c_str();

        // Transfer the buffer into the com interface
        if(com.SetRxBytes(cbuf, length)){
            printf("packet available ");
        } else{
             printf("no packet here! ");
        }

        /**************************************************************************
        **************************  Reading the Value  ***************************
        *************************************************************************/

        // Temporary Pointer to the packet data location
        uint8_t *packet_data;
        uint8_t packet_length;

        // Loads the packet data buffer with data receieved from the motor
        if(com.PeekPacket(&packet_data, &packet_length)){
            printf("packet peeked");
        }

        // Loads data into the temperature client
        templeft.ReadMsg(packet_data, packet_length);

        com.DropPacket();

        // Reads the data from the temperature client
        float temperature = templeft.temp_.get_reply();



        printf("Temperature: %f  Packet_data: %u  cbuf: %u  bytes in read buffer: %u packet length: %u       \r", temperature, *packet_data, *cbuf, length, packet_length);
    }

    return 0;
}