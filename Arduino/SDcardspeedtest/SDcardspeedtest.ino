#include <SD.h>
int time;
int timetwo;
File file;
void setup() {
  Serial.begin(9600);
  SD.begin(10);
  time = micros();
  file = SD.open("test.txt", O_CREAT | O_WRITE | O_TRUNC);
  timetwo = micros();
  while(millis() < 1000);  // delay so mills() is four digits
 
  for (uint8_t i = 0; i < 100; i++) {
    file.println(millis());
    // Serial.println(millis());
  }
  file.close();
  Serial.println(timetwo-time);
}
void loop() {}