#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"

#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

#define I2C_ADDRESS 0x3C

SSD1306AsciiAvrI2c oled;








Adafruit_SHT31 sht31 = Adafruit_SHT31();

void setup() {
  
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(font5x7);
  oled.clear();
  oled.set2X(); 
  
  Serial.begin(9600);

  while (!Serial)
    delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("SHT31 test");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
}


void loop() {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  int t1=(int)t;
  int t2=((int)((t+0.05)*10))%10;
  
  int h1=(int)(h+0.5);
  char msg[10]; 
  sprintf(msg, "%d.%d %d%%", t1,t2,h1);
  oled.setCursor(0,0);
  oled.print(msg);

  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("Temp *C = "); Serial.println(t);
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("Hum. % = "); Serial.println(h);
  } else { 
    Serial.println("Failed to read humidity");
  }
  Serial.println();
  delay(1000);
}
