#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
SSD1306AsciiWire oled;

#define OLEDI2CADDR 0x3C
#define SHT31I2CADDR 0x44
#define MEASUREINDELAY 10000
#define MEASUREOUTDELAY 9000


float stemp,shum;
int32_t temperature;
uint32_t pressure;
uint32_t humidity;
uint32_t vcc;

uint8_t i=0;
unsigned long lastread=0;
unsigned long lastmeasurein=MEASUREINDELAY+1000;
unsigned long lastmeasureout=0;
unsigned long now;

int32_t rnd(float n)
{
  return (int32_t)(n > 0 ? n + 0.5 : n - 0.5);
}


void space10(char *str){
  uint8_t j=10;
  while(j--) *str++=' ';
  str[10]=0;
}

void scpy(const char *a,char *b){
  while(*b++=*a++) ;
}

uint8_t i2a(int32_t value, char* result) {
    char* ptr = result, *ptr1 = result, tmp_char;
    uint8_t neg=0;
    uint8_t cnt=0;
    int32_t tmp_value;
    if(value<0) {neg=1;value=-value;}
    
    do {
      tmp_value = value;
      value /= 10;
      //*ptr++ = "0123456789" [tmp_value - value * 10];
      *ptr++ = '0' + (tmp_value - value * 10);
      cnt++;
    } while ( value );

    
    if (neg) {*ptr++ = '-';cnt++;}
    //*ptr-- = 0;
    *ptr--;
    while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
    }
    return cnt;
}

uint8_t I2CRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  uint8_t ret=1;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  if(Wire.endTransmission()) ret=0;
  
  Wire.requestFrom(dev_addr, cnt);
  uint8_t available = Wire.available();
  if(available != cnt) ret=0;
  for(uint8_t i = 0; i < available; i++) if(i < cnt) *(reg_data + i) = Wire.read();else Wire.read();
  return ret;
}

uint8_t I2CWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{  
  uint8_t ret=1;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(reg_data, cnt);
  if(Wire.endTransmission()) ret=0;
  return ret;
}


uint8_t readsht31()
{
  uint8_t readbuffer[6];
  readbuffer[0]=0;
  if(!I2CWrite(SHT31I2CADDR, 0x24, readbuffer, 1)) return 0;
  delay(16);
  if(!I2CRead(SHT31I2CADDR, 0x00, readbuffer, 6)) return 0;

  uint16_t ST, SRH;
  ST = readbuffer[0];
  ST <<= 8;
  ST |= readbuffer[1];
  SRH = readbuffer[3];
  SRH <<= 8;
  SRH |= readbuffer[4];

  stemp = ST;
  stemp *= 175;
  stemp /= 0xFFFF;
  stemp -= 45;
  
  shum = SRH;
  shum *= 100;
  shum /= 0xFFFF;
  
  return 1;
}



void measure()
{
  //Serial.println("MEASURE");
  
  
  char msg[21];
  oled.setCursor(0,0);
  uint8_t n;
  int32_t t;
  
  if(!readsht31()) oled.println("IN FAIL   ");
  else {
    space10(msg);
    
    t=rnd(stemp*10.0);
    n=i2a(t,msg);
    if(n==1 || (n==2 && t<0))
    {
      msg[n+1]=msg[n-1];msg[n-1]='0';msg[n]='.';
    }
    else {msg[n]=msg[n-1];msg[n-1]='.';}
    
    
    n=i2a(rnd(shum),msg+6);
    msg[6+n]='%';
    oled.println(msg);
  }
  
  if((now-lastmeasureout)>MEASUREOUTDELAY || lastmeasureout==0) oled.println("OUT NO COM");
  else if(pressure==0) oled.println("OUT FAIL  ");
  else {
    space10(msg);
    
    t=rnd(temperature/10.0);
    n=i2a(t,msg);
    if(n==1 || (n==2 && t<0))
    {
      msg[n+1]=msg[n-1];msg[n-1]='0';msg[n]='.';
    }
    else {msg[n]=msg[n-1];msg[n-1]='.';}
    
    
    n=i2a(rnd(humidity/1000.0),msg+6);
    msg[6+n]='%';
    oled.println(msg);
  }
}

void setup() {
  
  //i2c_init();
  Serial.begin(115200);
  Wire.begin();
  
  oled.begin(&Adafruit128x32, OLEDI2CADDR);
  oled.setFont(font5x7);
  oled.clear();
  oled.set2X();
}

void loop() {

  uint32_t buff[4];
  uint8_t *p=(uint8_t *)buff;
  
  now=millis();
  while(Serial.available()) {if(i<sizeof(buff)) p[i]=Serial.read();else Serial.read(); i++; lastread=now;}
  
  if((now-lastread)>100)
  {
    if(i==sizeof(buff)) {
      lastmeasureout=now;
      temperature=buff[0];humidity=buff[1];pressure=buff[2];vcc=buff[3];
      
      Serial.print("[OUT] T: ");Serial.println(temperature/100.0);
      Serial.print("[OUT] H: ");Serial.println(humidity/1000.0);
      Serial.print("[OUT] P: ");Serial.println(pressure/100.0);
      Serial.print("[OUT] V: ");Serial.println(vcc);
      Serial.print("[OUT] S: ");Serial.println(rnd(pressure*pow(1.0-((0.0065*610)/(temperature*0.01+273.15+0.0065*610)),-5.257)/100.0));
      Serial.println();
    }
    i=0;
  }

  now=millis();
  if((now-lastmeasurein)>MEASUREINDELAY) {
    lastmeasurein=now;
    measure();

    Serial.print("[IN] T: ");Serial.println(stemp);
    Serial.print("[IN] H: ");Serial.println(shum);
    Serial.println();
  }
  
  
}

 

