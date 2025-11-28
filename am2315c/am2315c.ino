#include <Wire.h>
#include <WiFi.h>
#include "ThingSpeak.h"

#define AM2315_ADDR 0x38
#define CHANNEL_ID 3010443
#define CHANNEL_API_KEY "X65QDJ4NWEEA02PI"

WiFiClient client;
const char* ssid = "wifi-name";
const char* password = "wifi-pass";

uint8_t computeCRC8(uint8_t *data, int len) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80)
        //100110001 → 1 + x^4 + x^5 + x^8
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(9600);

  //WiFI Begin
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  //WiFi End

  //ThingSpeak Begin
  ThingSpeak.begin(client);
  //ThingSpeak End

  Wire.begin(21, 22);
  delay(150);
}

void loop() {
  float temperature = NAN;
  float humidity = NAN;

  Wire.beginTransmission(AM2315_ADDR);
  Wire.write(0xAC); //Trig enable
  Wire.write(0x33); //first command parameter
  Wire.write(0x00); //second command parameter
  Wire.endTransmission();
  delay(100);

  Wire.requestFrom(AM2315_ADDR, 7);

  //I2C Buffer
  if (Wire.available() == 7) {
    uint8_t buffer[7];
    for (int i = 0; i < 7; i++) {
      buffer[i] = Wire.read();
    }

    if (computeCRC8(buffer, 6) != buffer[6]) {
      Serial.println("CRC failed — skipping");
      delay(2000);
      return;
    }
    /*buffer[1]->19:12 bits of RH MSB buffer[2]->11:4 bits of RH mid
      buffer[3] & 0xF0 -> RH 3:0 nibble and T 19:16 nibble*/
    /*RH 3:0 is upper nibble (LSB) of buffer T 19:16 is lower nibble (MSB)*/
    uint32_t rawHum = ((uint32_t)buffer[1] << 12) |
                      ((uint32_t)buffer[2] << 4) |
                      ((buffer[3] & 0xF0) >> 4);

    uint32_t rawTemp = ((uint32_t)(buffer[3] & 0x0F) << 16) |
                       ((uint32_t)buffer[4] << 8) |
                       buffer[5];

    humidity = (rawHum / 1048576.0) * 100.0;      
    temperature = (rawTemp / 1048576.0) * 200.0 - 50.0;

    Serial.print("Humidity: ");
    Serial.print(humidity, 2);
    Serial.print(" %\t");

    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");
  } else {
    Serial.println("Sensor not responding properly.");
  }

  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);

  int httpCode = ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY);

  if (httpCode == 200) {
    Serial.println("Channel updated");
  } else {
    Serial.print("ThingSpeak error ");
    Serial.println(httpCode);
  }

  delay(15000);
}
