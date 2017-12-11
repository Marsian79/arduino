#include "SSD1306AsciiWire.h"
SSD1306AsciiWire oled;

void setup() {
  
  
  //receiver
  pinMode(16,OUTPUT);
  digitalWrite(16,HIGH);

  //transmitter
  pinMode(15,OUTPUT);
  //digitalWrite(15,HIGH);
  digitalWrite(15,LOW);


  oled.begin(&Adafruit128x32, 0x3C);
  oled.setFont(font5x7);
  oled.clear();
  oled.set2X();
  //oled.println("IR TEST");
}

void loop() {
  uint16_t v1=0,v2=0;
  
  v1+=analogRead(3);
  v1+=analogRead(3);
  v1+=analogRead(3);
  v1+=analogRead(3);
  v1+=analogRead(3);
  v1+=analogRead(3);
  v1+=analogRead(3);
  v1+=analogRead(3);
  digitalWrite(15,HIGH);
  delay(1);
  v2+=analogRead(3);
  v2+=analogRead(3);
  v2+=analogRead(3);
  v2+=analogRead(3);
  v2+=analogRead(3);
  v2+=analogRead(3);
  v2+=analogRead(3);
  v2+=analogRead(3);
  digitalWrite(15,LOW);
  
  v1=(v1-v2)/8;
  if(v1>1023) v1=0;
  
  oled.setCursor(0,0);
  oled.print(v1);
  oled.print("     ");
  delay(200);
}
