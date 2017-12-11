// -*-c++-*-
// Scan I2C bus for device responses

// use ports usually used for hardware I2C
#define SDA_PORT PORTD
#define SDA_PIN 2
#define SCL_PORT PORTD
#define SCL_PIN 3

#define I2C_TIMEOUT 100
#define I2C_NOINTERRUPT 1

#include <SoftI2CMaster.h>
#include <avr/io.h>

uint8_t initi2c;
//------------------------------------------------------------------------------

void setup(void) {

  pinMode(4,OUTPUT);
  digitalWrite(4, HIGH);
  delay(5);

  Serial.begin(115200);
  initi2c=i2c_init(); 
}

void loop(void)
{
  uint8_t readbuffer[6];

  if(!initi2c) Serial.println("INIT ERR");
  
  if(!i2c_start(0x44<<1 | I2C_WRITE)) Serial.println("START1 ERR");
  if(!i2c_write(0x24)) Serial.println("WRITE11 ERR");
  if(!i2c_write(0x00)) Serial.println("WRITE12 ERR");
  i2c_stop();

  delay(16);

  if(!i2c_start(0x44<<1 | I2C_WRITE)) Serial.println("START2 ERR");
  if(!i2c_write(0x00)) Serial.println("WRITE21 ERR");
  if(!i2c_rep_start(0x44<<1 | I2C_READ)) Serial.println("REPSTART ERR");
  for(uint8_t i=0;i<6;i++) readbuffer[i]=i2c_read(i==6-1);
  i2c_stop();

  uint16_t ST, SRH;
  ST = readbuffer[0];
  ST <<= 8;
  ST |= readbuffer[1];
  SRH = readbuffer[3];
  SRH <<= 8;
  SRH |= readbuffer[4];

  float stemp = ST;
  stemp *= 175;
  stemp /= 0xffff;
  stemp = -45 + stemp;
  
  float shum = SRH;
  shum *= 100;
  shum /= 0xFFFF;
  
  Serial.print(stemp);Serial.print(' ');Serial.println(shum);

  //for(uint8_t i=0;i<6;i++) Serial.print(readbuffer[i],HEX);Serial.println();

  delay(1000);
  
}
