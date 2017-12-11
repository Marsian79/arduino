#define SCL_PORT PORTC
#define SCL_PIN 5
#define SDA_PORT PORTC
#define SDA_PIN 4
#define I2C_TIMEOUT 100
//#define I2C_NOINTERRUPT 1
#include <SoftI2CMaster.h>
//#include <avr/io.h>

//--------------------------------

//#include <Wire.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define BMEI2CADDR 0x76
#define BME280_CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)
#define bme280read(reg_addr,reg_data,cnt) I2CRead(BMEI2CADDR,reg_addr,reg_data,cnt)
#define bme280write(reg_addr,reg_data,cnt) I2CWrite(BMEI2CADDR,reg_addr,reg_data,cnt)

#define SCL 19
#define SDA 18
#define POWER 17
#define M0 16
#define M1 15
#define AUX 14

#define BME_DELAY 2
#define RADIOUP_DELAY 14

uint32_t vcc;
uint8_t adc;

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

void readvcc ()
{
  ADCSRA=adc;
  
  ADCSRA =  bit (ADEN);   // turn ADC on
  ADCSRA |= bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);  // Prescaler of 128
  ADMUX = bit (REFS0) | bit (MUX3) | bit (MUX2) | bit (MUX1);
  
  bitSet (ADCSRA, ADSC);  // start a conversion  
  while (bit_is_set(ADCSRA, ADSC)){ }
  //vcc = 1100.0 / float (ADC + 0.5) * 1024.0;
  vcc = 1056.0 / float (ADC + 0.5) * 1024.0;
  
  ADCSRA=0;
}
/*
uint8_t I2CRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  uint8_t ret=1;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();
  
  Wire.requestFrom(dev_addr, cnt);
  uint8_t available = Wire.available();
  if(available != cnt) ret=0;
  for(uint8_t i = 0; i < available; i++) if(i < cnt) *(reg_data + i) = Wire.read();else Wire.read();
  return ret;
}

void I2CWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{  
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(reg_data, cnt);
  Wire.endTransmission();
  
}*/

uint8_t I2CRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  if(cnt==0) return 1;
  if(!i2c_start(dev_addr<<1 | I2C_WRITE)) {i2c_stop();return 0;}
  if(!i2c_write(reg_addr)) {i2c_stop();return 0;}
  if(!i2c_rep_start(dev_addr<<1 | I2C_READ)) {i2c_stop();return 0;}
  for(uint8_t j=0;j<cnt;j++) reg_data[j]=i2c_read(j+1==cnt);
  i2c_stop();
  return 1;
}

uint8_t I2CWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  if(cnt==0) return 1;
  if(!i2c_start(dev_addr<<1 | I2C_WRITE)) {i2c_stop();return 0;}
  if(!i2c_write(reg_addr)) {i2c_stop();return 0;}
  for(uint8_t j=0;j<cnt;j++) if(!i2c_write(reg_data[j])) {i2c_stop();return 0;}
  i2c_stop();
  return 1;
}



uint8_t bme280readid()
{
  bme280read(0xD0, reg_data, 1);
  if(reg_data[0]==0x60) return 1;else return 0;
}

uint8_t bme280init()
{
  if(!bme280readid()) return 0;
  
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

  
  return 1;
}

uint8_t bme280measure()
{
  if(!bme280readid()) return 0;
  
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

  if (temperature < temperature_min) temperature = temperature_min;
  else if (temperature > temperature_max) temperature = temperature_max;

  
  
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

    //pressure+=pcal;
    
    if (pressure < pressure_min) pressure = pressure_min;
    else if (pressure > pressure_max) pressure = pressure_max;
  }
  else pressure = pressure_min;
  
  
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

  if (humidity > humidity_max) humidity = humidity_max;
  
  return 1;
}

void bmeup()
{
  digitalWrite(POWER,HIGH);
  delay(BME_DELAY);
}

void bmedown()
{
  delay(BME_DELAY);
  digitalWrite(POWER,LOW);
  digitalWrite(SCL,LOW);
  digitalWrite(SDA,LOW);
}

void radioup()
{
  digitalWrite(M0,LOW);
  digitalWrite(M1,LOW);
  
  //start=millis();
  //while(digitalRead(AUX)==HIGH) {}
  //while(digitalRead(AUX)==LOW) {}
  //start=millis()-start;
  
  //delay(2);
}

void radiodown()
{
  digitalWrite(M0,HIGH);
  digitalWrite(M1,HIGH);
}

EMPTY_INTERRUPT (WDT_vect);
void gotosleep()
{
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    noInterrupts();
    wdt_reset();
    WDTCSR = bit (WDCE) | bit (WDE);
    WDTCSR = bit (WDIE) | 0b00100001;//8 sec
    //WDTCSR = bit (WDIE) | 0b00000110;//1 sec
    sleep_enable();
    interrupts();
    sleep_cpu();
    sleep_disable();
    wdt_disable();
}

void setup() {
  pinMode(POWER,OUTPUT);//digitalWrite(POWER,LOW);
  pinMode(M0,OUTPUT);//digitalWrite(M0,HIGH);
  pinMode(M1,OUTPUT);//digitalWrite(M1,HIGH);
  pinMode(AUX,INPUT);
  adc=ADCSRA;ADCSRA=0;
  
  //Wire.begin();
  i2c_init();
  Serial.begin(115200);
  bmeup();bme280init();bmedown();
}

void loop() {
  uint32_t buff[4];
  unsigned long radiouptime;
  
  radioup();//needs 14ms delay
  radiouptime=millis();
  
  bmeup(); if(!bme280measure()) pressure=0; bmedown();
  readvcc();
  
  buff[0]=temperature;buff[1]=humidity;buff[2]=pressure;buff[3]=vcc;
  //radioup();
  
  while((millis()-radiouptime)<=RADIOUP_DELAY) {}
  Serial.write((uint8_t *)buff,sizeof(buff));
  /*Serial.println(temperature/100.0);
  Serial.println(humidity/1000.0);
  Serial.println(pressure/100.0);
  Serial.println(vcc);
  Serial.println();*/
  Serial.flush();
  radiodown();
  gotosleep();
}
