// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char SSID[]     = 'IllinoisNet';    // Network SSID (name)
const char PASS[]     = 'Nrwagner122001';    // Network password (use for WPA, or use as key for WEP)
const char USER[]     = 'neilrw2';    //Network Username
void onStatusChange();

int status;

void initProperties(){

  ArduinoCloud.addProperty(status, READWRITE, ON_CHANGE, onStatusChange);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
