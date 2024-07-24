#include <iq_module_communication.hpp>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>



IqSerial ser(Serial1);
Adafruit_MPU6050 mpu;
BrushlessDriveClient motr(0);
BrushlessDriveClient motl(2);
PowerMonitorClient pwrr(0);
PowerMonitorClient pwrl(2);
int i = 0;
int x = 0;
int y = 0;
float axo = -0.8;
float ayo = 0.05;
float azo = 1.83;
float gxo = 0.06;
float gyo = 0.01;
float gzo = 0.02;

void setup() {
  // Initialize serial and wait for port to open:
  // Initialize the IqSerial object    
  ser.begin();

  Serial.begin(9600);
  while (!Serial);  // will pause MKR until serial console opens
  Serial.println("Adafruit MPU6050 and Vertiq Motor Test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }
  Serial.println("");

  delay(100);
}

void loop() {
  // Your code here
 
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Send voltage signal to motors
  static float velocityr = 0;
  static float velocityl = 0;
  float Motorcommandr = a.acceleration.z + azo;
  float Motorcommandl = a.acceleration.x + axo;
  float voltager = 0;
  float voltagel = 0;
  //supply voltage
  ser.set(motr.drive_spin_volts_, Motorcommandr);
  ser.set(motl.drive_spin_volts_, Motorcommandl);


    if (ser.get(pwrr.volts_, voltager)) {
      Serial.print("Motor R Supply Voltage: ");
      Serial.print(voltager);
    }
    if (ser.get(pwrl.volts_, voltagel)) {
      Serial.print(" Motor L Supply Voltage: ");
      Serial.print(voltagel);
    }
    // Get the velocity. If a response was received...

    if (ser.get(motr.obs_velocity_, velocityr)) {
      // Display the reported velocity
      Serial.print(" Voltage R Command: ");
      Serial.print(Motorcommandr);
      Serial.print(" Motor R Velocity: ");
      Serial.print(velocityr);
    } else {
      Serial.println("No Response Received");
    }

    if (ser.get(motl.obs_velocity_, velocityl)) {
      // Display the reported velocity
      Serial.print(" Voltage L Command: ");
      Serial.print(Motorcommandl);
      Serial.print(" Motor L Velocity: ");
      Serial.print(velocityl);
    } else {
      Serial.println("No Response Received");
    }

    /* Print out the values */
    Serial.print(" Acceleration X: ");
    Serial.print(a.acceleration.x + axo);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y + ayo);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z + azo);
    Serial.print(" m/s^2 ");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x + gxo);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y + gyo);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z + gzo);
    Serial.print(" rad/s ");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");
   
    
}