#define SDA_PORT PORTD
#define SDA_PIN 2
#define SCL_PORT PORTD
#define SCL_PIN 3

#define I2C_TIMEOUT 100
#define I2C_NOINTERRUPT 1

#include <SoftI2CMaster.h>
#include <avr/io.h>

//--------------------------------------

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "SSD1306Ascii.h"
//#include "SSD1306AsciiAvrI2c.h"
#include "SSD1306AsciiWire.h"

#define BME280_CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)

#define NUMB 2

#define DBDELAY 10
#define MDELAY 5000
#define MENUTO 2000
#define CDELAY 50
#define ADELAY 1000
#define PUPDELAY 2

#define BME280POWER 9
#define BME280SS 10
#define OLEDI2CADDR 0x3C

#define DEBUG
//#define I2C
#define BMEI2CADDR 0x77

#ifdef DEBUG
  //#define BMEI2CADDR 0x76
  const uint8_t buttonPin[NUMB] = {5,8};
  #define sb(msg) Serial.begin(msg)
  #define sp(msg) Serial.print(msg)
  #define spln(msg) Serial.println(msg)
  #define d5() delay(5)
#else
  //#define BMEI2CADDR 0x77
  const uint8_t buttonPin[NUMB] = {3,6};
  #define sb(msg)
  #define sp(msg)
  #define spln(msg)
  #define d5()
#endif



#ifdef I2C
  #define pup()
  #define pdown()
  #define bme280read(reg_addr,reg_data,cnt) I2CRead(BMEI2CADDR,reg_addr,reg_data,cnt)
  #define bme280write(reg_addr,reg_data,cnt) I2CWrite(BMEI2CADDR,reg_addr,reg_data,cnt)
#else
  #define pup() powerup();
  #define pdown() powerdown();
  #define bme280read(reg_addr,reg_data,cnt) SPIRead(BME280SS,reg_addr,reg_data,cnt)
  #define bme280write(reg_addr,reg_data,cnt) SPIWrite(BME280SS,reg_addr,reg_data,cnt)
#endif




//SSD1306AsciiAvrI2c oled;
SSD1306AsciiWire oled;

int32_t temperature;
const int32_t temperature_min = -4000;
const int32_t temperature_max = 8500;

uint32_t pressure=0;
const uint32_t pressure_min = 30000;
const uint32_t pressure_max = 110000;

uint32_t humidity;
const uint32_t humidity_max = 100000;

uint8_t reg_data[26];

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
uint8_t  dig_H1;
int16_t dig_H2;
uint8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;
int32_t t_fine;



//-----------------------------------


char msg[21];
uint8_t menu=0;
uint8_t menulvl=0;

uint8_t holdalt;
int16_t alt;
uint32_t seap;
int8_t pcal=0;
const uint8_t wdtdelay[6]={0b00000100,0b00000101,0b00000110,0b00000111,0b00100000,0b00100001};
const uint16_t wdtms[6]={250,500,1000,2000,4000,8000};
uint8_t wdtselect;

uint8_t state=HIGH;
uint8_t laststate[NUMB]={HIGH,HIGH};
uint8_t curstate[NUMB]={HIGH,HIGH};

unsigned long now=0,lastclick=0,lastpush=0,lastchangeany=0;
unsigned long lastchange[NUMB]={0,0};

float stemp,shum;

uint8_t adc;

void readsht31()
{
  
  //pinMode(4,OUTPUT);
  //digitalWrite(4, HIGH);
  //delay(2);
  
  
  uint8_t readbuffer[6];

  if(!i2c_start(0x44<<1 | I2C_WRITE)) spln("START1 ERR");
  if(!i2c_write(0x24)) spln("WRITE11 ERR");
  if(!i2c_write(0x00)) spln("WRITE12 ERR");
  i2c_stop();

  delay(16);

  if(!i2c_start(0x44<<1 | I2C_WRITE)) spln("START2 ERR");
  if(!i2c_write(0x00)) spln("WRITE21 ERR");
  if(!i2c_rep_start(0x44<<1 | I2C_READ)) spln("REPSTART ERR");
  for(uint8_t i=0;i<6;i++) readbuffer[i]=i2c_read(i==6-1);
  i2c_stop();

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
  
  //sp(stemp);sp(' ');spln(shum);

  //delay(2);
  //digitalWrite(4, LOW);
}

float readvcc ()
{
  ADCSRA =  bit (ADEN);   // turn ADC on
  ADCSRA |= bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);  // Prescaler of 128
  ADMUX = bit (REFS0) | bit (MUX3) | bit (MUX2) | bit (MUX1);
  
  const float InternalReferenceVoltage = 1100.0; // as measured
  bitSet (ADCSRA, ADSC);  // start a conversion  
  while (bit_is_set(ADCSRA, ADSC)){ }
  float results = InternalReferenceVoltage / float (ADC + 0.5) * 1024.0;
  return results;
}



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

void I2CRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  //uint8_t ret=1;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();
  
  Wire.requestFrom(dev_addr, cnt);
  uint8_t available = Wire.available();
  //if(available != cnt) ret=0;
  for(uint8_t i = 0; i < available; i++) if(i < cnt) *(reg_data + i) = Wire.read();else Wire.read();
  //return ret;
}

void I2CWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{  
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(reg_data, cnt);
  Wire.endTransmission();
  
}
void SPIRead(uint8_t dev_addr,uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dev_addr, LOW);
  SPI.transfer(reg_addr | 0x80);
  for (uint8_t i = 0; i < cnt; i++) *(reg_data + i) = SPI.transfer(0);
  digitalWrite(dev_addr, HIGH);
  SPI.endTransaction();
}

void SPIWrite(uint8_t dev_addr,uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dev_addr, LOW);
  for (uint8_t i = 0; i < cnt; i++) {SPI.transfer((reg_addr++) & 0x7F);SPI.transfer(*(reg_data + i));}
  digitalWrite(dev_addr, HIGH);
  SPI.endTransaction();
}

uint8_t bme280readid()
{
  bme280read(0xD0, reg_data, 1);
  if(reg_data[0]==0x60) return 1;else return 0;
}
uint8_t bme280init(uint8_t dev_addr=BMEI2CADDR)
{
  
  pup();
  if(!bme280readid()){pdown();return 0;}
  
  bme280read(0x88, reg_data, 26);
  
  dig_T1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
  dig_T2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
  dig_T3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);

  dig_P1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
  dig_P2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
  dig_P3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
  dig_P4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
  dig_P5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
  dig_P6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
  dig_P7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
  dig_P8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
  dig_P9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
  
  dig_H1 = reg_data[25];
  
  bme280read(0xE1, reg_data, 7);
  
  int16_t dig_H45_lsb;
  int16_t dig_H45_msb;

  dig_H2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
  
  dig_H3 = reg_data[2];

  dig_H45_msb = (int16_t)(int8_t)reg_data[3] * 16;
  dig_H45_lsb = (int16_t)(reg_data[4] & 0x0F);
  dig_H4 = dig_H45_msb | dig_H45_lsb;

  dig_H45_msb = (int16_t)(int8_t)reg_data[5] * 16;
  dig_H45_lsb = (int16_t)(reg_data[4] >> 4);
  dig_H5 = dig_H45_msb | dig_H45_lsb;
  
  dig_H6 = (int8_t)reg_data[6];

  
  pdown();
  return 1;
}

uint8_t bme280measure(uint8_t dev_addr)
{
  
  pup();
  if(!bme280readid()){pdown();return 0;}
  
  reg_data[0]=0b00000001;bme280write(0xf2, reg_data, 1);
  reg_data[0]=0b00100101;bme280write(0xf4, reg_data, 1);
  delay(10);
  bme280read(0xF7, reg_data, 8);
  
  
  int32_t var1;
  int32_t var2;
  int32_t var3;
  int32_t var4;
  int32_t var5;
  uint32_t var5p;
  
  uint32_t data_xlsb;
  uint32_t data_lsb;
  uint32_t data_msb;
  
  //--------------------------read uncomp
  data_msb = (uint32_t)reg_data[0] << 12;
  data_lsb = (uint32_t)reg_data[1] << 4;
  data_xlsb = (uint32_t)reg_data[2] >> 4;
  uint32_t upressure = data_msb | data_lsb | data_xlsb;

  
  data_msb = (uint32_t)reg_data[3] << 12;
  data_lsb = (uint32_t)reg_data[4] << 4;
  data_xlsb = (uint32_t)reg_data[5] >> 4;
  uint32_t utemperature = data_msb | data_lsb | data_xlsb;
  
  data_lsb = (uint32_t)reg_data[6] << 8;
  data_msb = (uint32_t)reg_data[7];
  uint32_t uhumidity = data_msb | data_lsb;

  
  //--------------------------temperature
  var1 = (int32_t)((utemperature / 8) - ((int32_t)dig_T1 * 2));
  var1 = (var1 * ((int32_t)dig_T2)) / 2048;
  var2 = (int32_t)((utemperature / 16) - ((int32_t)dig_T1));
  var2 = (((var2 * var2) / 4096) * ((int32_t)dig_T3)) / 16384;
  t_fine = var1 + var2;
  temperature = (t_fine * 5 + 128) / 256;

  if (temperature < temperature_min)
    temperature = temperature_min;
  else if (temperature > temperature_max)
    temperature = temperature_max;

  //temperature=-2344;
  
  //--------------------------pressure
  var1 = (((int32_t)t_fine) / 2) - (int32_t)64000;
  var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)dig_P6);
  var2 = var2 + ((var1 * ((int32_t)dig_P5)) * 2);
  var2 = (var2 / 4) + (((int32_t)dig_P4) * 65536);
  var3 = (dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
  var4 = (((int32_t)dig_P2) * var1) / 2;
  var1 = (var3 + var4) / 262144;
  var1 = (((32768 + var1)) * ((int32_t)dig_P1)) / 32768;
   /* avoid exception caused by division by zero */
  if (var1) {
    var5p = (uint32_t)((uint32_t)1048576) - upressure;
    pressure = ((uint32_t)(var5p - (uint32_t)(var2 / 4096))) * 3125;
    if (pressure < 0x80000000)
      pressure = (pressure << 1) / ((uint32_t)var1);
    else
      pressure = (pressure / (uint32_t)var1) * 2;

    var1 = (((int32_t)dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
    var2 = (((int32_t)(pressure / 4)) * ((int32_t)dig_P8)) / 8192;
    pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + dig_P7) / 16));

    pressure+=pcal;
    
    if (pressure < pressure_min)
      pressure = pressure_min;
    else if (pressure > pressure_max)
      pressure = pressure_max;
  } else {
    pressure = pressure_min;
  }
  
  
  //--------------------------humidity
  var1 = t_fine - ((int32_t)76800);
  var2 = (int32_t)(uhumidity * 16384);
  var3 = (int32_t)(((int32_t)dig_H4) * 1048576);
  var4 = ((int32_t)dig_H5) * var1;
  var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
  var2 = (var1 * ((int32_t)dig_H6)) / 1024;
  var3 = (var1 * ((int32_t)dig_H3)) / 2048;
  var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
  var2 = ((var4 * ((int32_t)dig_H2)) + 8192) / 16384;
  var3 = var5 * var2;
  var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
  var5 = var3 - ((var4 * ((int32_t)dig_H1)) / 16);
  var5 = (var5 < 0 ? 0 : var5);
  var5 = (var5 > 419430400 ? 419430400 : var5);
  humidity = (uint32_t)(var5 / 4096);

  if (humidity > humidity_max)
    humidity = humidity_max;
  
  pdown();
  return 1;
}
void measure()
{
    
    
    if(!bme280measure(BMEI2CADDR)) {spln("Measuring failed.");oled.setCursor(0,0);oled.println("MEASURING ");oled.println("FAILED    ");return;}
    
    calc_seap_alt();
    
   
    spln();
    spln("MEASURE");
    sp("Temperature: ");
    sp(temperature / 100.0);
    spln(" [C]");

    sp("Temperature SHT31: ");
    sp(stemp);
    spln(" [C]");

    sp("Temperature DIFF: ");
    sp(stemp - temperature / 100.0);
    spln(" [C]");
    
    
    sp("Humidity: ");
    sp(humidity/1000.0);
    spln(" [%]");

    sp("Humidity SHT31: ");
    sp(shum);
    spln(" [%]");
    
    sp("Humidity DIFF: ");
    sp(shum-humidity/1000.0);
    spln(" [%]");

    
    sp("Pressure: ");
    sp(pressure/100.0);
    spln(" [hPa]");
    sp("Sea Pressure: ");
    sp(seap/100.0);
    spln(" [hPa]");
    sp("Altitude: ");
    sp(alt);
    spln(" [M]");

    #ifdef DEBUG
      ADCSRA=adc;
      float vcc=0;
      for(uint8_t i=0;i<10;i++) vcc+=readvcc();
      sp("Voltage: ");
      Serial.print(rnd(vcc/10.0));
      spln(" [mV]");
      ADCSRA=0;

      readsht31();
    #endif

    showlvl0();
}

int16_t getalt()
{
  
 //return rnd(-29.27175933288182*(temperature/100.0+273.15)*log(seap/(float)pressure));
 return rnd((pow(seap/(float)pressure,(0.19022256))-1.0)*(temperature*0.01+273.15)*153.84615384);
}
uint32_t getseap()
{
  
  //return rnd(pressure/exp(alt/(temperature/100.0+273.15)/-29.27175933288182));
  return rnd(pressure*pow(1.0-((0.0065*alt)/(temperature*0.01+273.15+0.0065*alt)),-5.257));
}
void calc_seap_alt()
{
  if(holdalt) seap=getseap();
  else alt=getalt();
}

void showlvl2(uint8_t update=false)
{
   
   oled.setCursor(0,0);

   if(menu==0)
   {
    if(update) seap=getseap();
    oled.println("ALTITUDE: ");
    space10(msg);i2a(alt,msg);
   }
   else if(menu==1)
   {
    if(update) alt=getalt();
    oled.println("SEA PRESS:");
    space10(msg);i2a(rnd(seap/100.0),msg);
   }
   else if(menu==3)
   {
    oled.println("DELAY MS: ");
    space10(msg);i2a(wdtms[wdtselect],msg);
   }
   else if(menu==4)
   {
    if(pcal>100) pcal=100;else if(pcal<-100) pcal=-100;
    oled.println("CAL PRESS:");
    space10(msg);i2a(pcal,msg);
   }
   
   oled.println(msg);
   
}

void showlvl1()
{
   
   oled.setCursor(0,0);
   if(menu/2==0)
   {
      scpy(" 1SET ALT ",msg);
      if(menu==0) msg[0]='>';
      oled.println(msg);
      
      scpy(" 2SET SEA ",msg);
      if(menu==1) msg[0]='>';
      oled.println(msg);
   }
   else if(menu/2==1)
   {
      if(holdalt) scpy(" 3HOLD:ALT",msg);
      else scpy(" 3HOLD:SEA",msg);
      if(menu==2) msg[0]='>';
      oled.println(msg);

      scpy(" 4DELAY MS",msg);
      if(menu==3) msg[0]='>';
      oled.println(msg);
    }
    else if(menu/2==2)
    {
      scpy(" 5CAL PRES",msg);
      if(menu==4) msg[0]='>';
      oled.println(msg);

      scpy("          ",msg);
      oled.println(msg);
    }
   
}


void showlvl0()
{
    //unsigned long start=millis();
    
    if(!pressure) return;
    spln();
    spln("SHOW MEASURE");
    
    uint8_t n;
    calc_seap_alt();
    
    oled.setCursor(0,0);
    
    space10(msg);
    
    //n=i2a(rnd(temperature/10.0)/10,msg);
    //msg[n]='.';
    //i2a(rnd(temperature/10.0,1)%10,msg+n+1);
    
    int32_t t=rnd(temperature/10.0);
    n=i2a(t,msg);
    if(n==1 || (n==2 && t<0))
    {
      msg[n+1]=msg[n-1];msg[n-1]='0';msg[n]='.';
    }
    else {msg[n]=msg[n-1];msg[n-1]='.';}
    
    
    n=i2a(rnd(humidity/1000.0),msg+6);
    msg[6+n]='%';
    oled.println(msg);
    sp("[");sp(msg);spln("]");
    
    space10(msg);
    i2a(alt,msg+1);
    i2a(rnd(seap/100.0),msg+6);
    if(holdalt) msg[0]='*'; else msg[5]='*';
    oled.println(msg);
    sp("[");sp(msg);spln("]");
    
    //spln(millis()-start);
 
}

void bclick(uint8_t i)
{
          lastclick=now;
          if(i==0)
          {
            if(menulvl==0) {menulvl=1;showlvl1();}
            else if(menulvl==1) {menu=(menu+1)%5;showlvl1();}
            else if(menulvl==2) {if(menu==0) alt--; else if(menu==1) seap=rnd(seap/100.0)*100-100; else if(menu==3 && wdtselect>0) wdtselect--; else if(menu==4) pcal-=10; showlvl2(true);}
          }
          else if(i==1)
          {
            if(menulvl==1)
            {
              if(menu==2) {holdalt=(holdalt+1)%2;showlvl1();}
              else {menulvl=2;showlvl2();}
            }
            else if(menulvl==2) {if(menu==0) alt++; else if(menu==1) seap=rnd(seap/100.0)*100+100; else if(menu==3 && (wdtselect < (sizeof(wdtdelay)-1))) wdtselect++; else if(menu==4) pcal+=10; showlvl2(true);}
          }
}



void powerup()
{
  PORTB|=0b00000110;
  delay(PUPDELAY);
}

void powerdown()
{
  SPI.end();
  delay(PUPDELAY);
  PORTB&=0b11010001;
}

void gotosleep()
{
    spln("ENTERING SLEEP...");d5();
    uint8_t wdtd = wdtdelay[wdtselect];
    
    noInterrupts ();
    wdt_reset();
    WDTCSR = bit (WDCE) | bit (WDE);
    WDTCSR = bit (WDIE) | wdtd;
    sleep_enable();
    interrupts ();
    sleep_cpu ();
    sleep_disable();
    wdt_disable();
}

EMPTY_INTERRUPT (WDT_vect);
#ifdef DEBUG
  EMPTY_INTERRUPT (PCINT0_vect);
  EMPTY_INTERRUPT (PCINT2_vect);
#else
  EMPTY_INTERRUPT (PCINT2_vect);
#endif



void setup() {
  for(uint8_t i=0;i<NUMB;i++) pinMode(buttonPin[i], INPUT_PULLUP);
  
  adc=ADCSRA;
  ADCSRA = 0;
  
  #ifndef I2C
    DDRB|= 0b00101110;
  #endif
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  
  #ifdef DEBUG
    PCICR = bit(PCIE0) | bit(PCIE2);
    PCMSK0 = bit(PCINT0);
    PCMSK2 = bit(PCINT21);
  #else
    PCICR = bit(PCIE2);
    PCMSK2 = bit(PCINT19) | bit(PCINT22);
  #endif

  
  sb(115200);
  spln("Starting...");
  
  oled.begin(&Adafruit128x32, OLEDI2CADDR);
  oled.setFont(font5x7);
  oled.clear();
  oled.set2X();
  
  while(!bme280init())
  {
    spln("Sensor not found!.");
    oled.setCursor(0,0);oled.println("SENSOR    ");oled.println("NOT FOUND ");
    delay(1000);
  }


  EEPROM.get(0, holdalt);if(isnan(holdalt) || holdalt==0xFF) holdalt=0;
  EEPROM.get(1, alt);if(isnan(alt) || alt==0xFFFF) alt=610;
  EEPROM.get(3, seap);if(isnan(seap) || seap==0xFFFFFFFF) seap=101325;
  EEPROM.get(7, pcal);if(isnan(pcal) || pcal==0xFF) pcal=0;
  EEPROM.get(8, wdtselect);if(isnan(wdtselect) || wdtselect==0xFF || wdtselect>=sizeof(wdtdelay)) wdtselect=4;


  pinMode(4,OUTPUT);
  digitalWrite(4, HIGH);
  delay(2);
  i2c_init(); 
  
}

void loop() {

  uint8_t i;
  now=millis();
  for(i=0;i<NUMB;i++)
  {
    state = digitalRead(buttonPin[i]);
    
    if (state != laststate[i]) {lastchangeany=now;lastchange[i]=now;laststate[i]=state;}
    else if ((now-lastchange[i])>DBDELAY)
    {
      if(curstate[i]!=laststate[i])
      {
        curstate[i]=laststate[i];
        if (curstate[i] == HIGH) {
          sp("HIGH");spln(i);
        }
        else {
          sp("LOW");spln(i);
          if((now-lastclick)>CDELAY) {lastpush=now;bclick(i);}
        }    
      }
    }
    
    if(curstate[i] == LOW && (now-lastpush)>ADELAY && (now-lastclick)>CDELAY) bclick(i);
  }

 
  if(menulvl!=0 && (now-lastclick)>MENUTO)
  {
      uint8_t holdaltee;int16_t altee;uint32_t seapee;int8_t pcale;uint8_t wdtselecte;
      EEPROM.get(0, holdaltee);if(holdaltee!=holdalt){EEPROM.put(0, holdalt);spln("HOLDALT SAVED!");}
      EEPROM.get(1, altee);if(altee!=alt){EEPROM.put(1, alt);spln("ALT SAVED!");}
      EEPROM.get(3, seapee);if(seapee!=seap){EEPROM.put(3, seap);spln("SEAP SAVED!");}
      EEPROM.get(7, pcale);if(pcale!=pcal){EEPROM.put(7, pcal);spln("PCAL SAVED!");}
      EEPROM.get(8, wdtselecte);if(wdtselecte!=wdtselect){EEPROM.put(8, wdtselect);spln("WDTSELECT SAVED!");}
      menulvl=0;
   }

   
   if(menulvl==0 && (now-lastchangeany)>DBDELAY)
   {
      measure();
      uint8_t lowany=0;
      now=millis();
      for(i=0;i<NUMB;i++) if (digitalRead(buttonPin[i])==LOW) {lowany=1;lastchangeany=now;}
      if(!lowany) gotosleep();
   }
}
