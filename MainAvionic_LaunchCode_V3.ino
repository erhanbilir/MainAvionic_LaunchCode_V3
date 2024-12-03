#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include "Arduino.h"
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_BNO055.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>

#define GPS_BAUD    9600
#define LORA_BAUD   9600
#define SERIAL_BAUD 9600
#define GPS_SERIAL  Serial2
#define LORA_SERIAL Serial1

#define SD_PIN          49
#define PHASE_1_PIN     5
#define PHASE_2_PIN     4
#define PHASE_1_LED_PIN 6

#define ACTIVATION_TIME         2000
#define SEA_LEVEL_PRESSURE      912.3f
#define STAGE_2_ALTITUDE        600
#define SAFETY_ALTITUDE         500
#define STAGE_1_SAFETY_ALTITUDE 2000

SimpleKalmanFilter rollKalmanFilter(0.5, 0.5, 0.01);
SimpleKalmanFilter yawKalmanFilter(0.5, 0.5, 0.01);
SimpleKalmanFilter pitchKalmanFilter(0.5, 0.5, 0.01);

SimpleKalmanFilter altitudeKalmanFilter(1.5, 1.5, 0.01);


TinyGPSPlus gps;
HardwareSerial &ss2 = GPS_SERIAL;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

Adafruit_BME280 bme; // I2C

// -------------------------------------

struct Signal {
  byte altitude[4];
  byte gpsAltitude[4];
  byte lat[4];
  byte lng[4];
  byte gyroX[4];
  byte gyroY[4];
  byte gyroZ[4];
  byte accelX[4];
  byte accelY[4];
  byte accelZ[4];
  byte angle[4];
  byte state[1];
} data;

int rocketState = 1;
float pressure;
float altitude;
float maxAltitude = 0.0f;
bool stage1 = false;
bool stage2 = false;
uint8_t state = 0;


void setup(){
	Serial.begin(SERIAL_BAUD);
	while (!Serial) {
	    ; // wait for serial port to connect. Needed for native USB
    }
	delay(100);

  // Kontrol Ledlerini tanimlanmasi
  pinMode(PHASE_1_PIN, OUTPUT);
  pinMode(PHASE_2_PIN, OUTPUT);
  pinMode(PHASE_1_LED_PIN, OUTPUT);

  //Gps
  GPS_SERIAL.begin(GPS_BAUD);
  delay(2000);

  //BNO055
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("There was a problem detecting the BNO055 ... check your connections");
    while(1);
  }
  delay(1000);

  //BME280
  if (!bme.begin(0x77)) {
    Serial.println("Could not find a validdf BME280 sensor, check wiring!");
    while (1) {
      ;
    }
  }

  //LORAE32
  LORA_SERIAL.begin(LORA_BAUD); //LoRa kütüphanesi sabit duracak fakat serial1.begin başlayacak.

  delay(2000);
  Serial.println("initialization done.");

}

void loop(){
  sensors_event_t orientationData;
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  altitude = bme.readAltitude(SEA_LEVEL_PRESSURE);
  
  float latitude = (float)gps.location.lat();
  float longtitude = (float)gps.location.lng();
  float gpsAltitude = (float)gps.altitude.meters();
  float x = orientationData.orientation.x;
  float y = orientationData.orientation.y;
  float z = orientationData.orientation.z;
  float accelX = (float)(accel.x());
  float accelY = (float)(accel.z());
  float accelZ = (float)(accel.y());

  float angleY = (float)(euler.y());

  /*x = abs(rollKalmanFilter.updateEstimate(x));
  y = abs(yawKalmanFilter.updateEstimate(y));
  z = abs(pitchKalmanFilter.updateEstimate(z));*/
  x = abs(x);
  y = abs(y);
  z = abs(z);
  altitude = altitudeKalmanFilter.updateEstimate(altitude);

  /* Yer istasyonuna gönderilecek bilgi için roket durumu */
  *(data.state) = rocketState;
  
  if(!stage1 && !stage2)
    rocketState = 1;
  else if(stage1 && !stage2)
    rocketState = 2;
  else if(!stage1 && stage2)
    rocketState = 3;
  else if(stage1 && stage2)
    rocketState = 4;
    
  /* Ayrılma kontrol algoritması */  
  if(!stage1 && !stage2){
    if(altitude > maxAltitude) {
      maxAltitude = altitude;
    }
    if(maxAltitude > SAFETY_ALTITUDE){
      if(y < 15 && y > 0)
      {
        digitalWrite(PHASE_1_LED_PIN, HIGH);
        stage1Seperation();
        digitalWrite(PHASE_1_LED_PIN, LOW);
      }
      else
      {
        digitalWrite(PHASE_1_LED_PIN, LOW);
      }
    }

    if((altitude > STAGE_1_SAFETY_ALTITUDE) && (maxAltitude - altitude > 50)) {
      stage1Seperation();
    }
  }
  else if(stage1 && !stage2)
  {
    if(altitude > SAFETY_ALTITUDE && altitude < STAGE_2_ALTITUDE) {
      digitalWrite(PHASE_1_LED_PIN, HIGH);
      stage2Seperation();
      digitalWrite(PHASE_1_LED_PIN, LOW);
    }
  }

/* Yer istasyonuna iletilecek paket */
String strData = String(altitude) + "," +
                 String(gpsAltitude) + "," +
                 String(latitude,6) + "," +
                 String(longtitude,6) + "," +
                 String(x )+ "," +
                 String(y) + "," +
                 String(z) + "," +
                 String(accelX) + "," +
                 String(accelY) + "," +
                 String(accelZ) + "," +
                 String(angleY) + "," +
                 String(*(data.state));

  String strPacket = "STR," + strData  + ",END";
 
  /* Kontrol için seri porttan basılacak paket */
  String strData2 = "Altitude: " + String(altitude) + "\n\rGPS Altitude: " +
                 String(gpsAltitude) + "\n\rLatitude: " +
                 String(latitude,6) + "\n\rLongtitude; " +
                 String(longtitude,6) + "\n\rRoll: " +
                 String(x)+ "\n\rYaw: " +
                 String(y) + "\n\rPitch: " +
                 String(z) + "\n\rAccel X: " +
                 String(accelX) + "\n\rAccel Y: " +
                 String(accelY) + "\n\rAccel Z: " +
                 String(accelZ) + "\n\rAngle: " +
                 String(angleY) + "\n\rStatus: " +
                 String(*(data.state)) + "\n\r----------------------";

  LORA_SERIAL.println(strPacket);
  Serial.println(strData2);

  smartDelay(1250);
}

// GPS için delay fonksiyonu
static void smartDelay(unsigned long ms) { 
  unsigned long start = millis();
  do {
    while (ss2.available()>0)
      gps.encode(ss2.read());
  } while (millis() - start < ms);
}

// 1. ayrılma
void stage1Seperation() 
{
  stage1 = 1;
  digitalWrite(PHASE_1_PIN, HIGH); 
  Serial.println("Ayrılma gerçekleşti");
  delay(ACTIVATION_TIME);               
  digitalWrite(PHASE_1_PIN, LOW); 
}

// 2. ayrılma
void stage2Seperation() 
{
  stage2 = 1;
  digitalWrite(PHASE_2_PIN, HIGH); 
  Serial.println("Ayrılma gerçekleşti");
  delay(ACTIVATION_TIME);               
  digitalWrite(PHASE_2_PIN, LOW); 
}

// void hotStart()
// {
//   //byte hotStartCommand[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x68};
//   byte hotStart[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x39 };

//   GPS_SERIAL.write(hotStart, sizeof(hotStart));

// }
