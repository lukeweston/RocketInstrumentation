// Test Program for MobSenDat board
// Luke Weston, January 2011

// Runs the BMP085 pressure and temperature sensor the DS18B20 and the accelerometer
// and the real-time clock, and reads the battery voltage, logging the data to the SD card.

// BMP085 sensor on the I2C bus. Requires the Wire (I2C) library.
// Battery voltage measured through a voltage divider connected to analog pin 1.

// Modified from code provided by ladyada
// https://github.com/adafruit/Light-and-Temp-logger

// You'll need the SD card and RTC libraries provided by ladyada
// http://www.ladyada.net/make/logshield/download.html

// This requires the Spi arduino library: http://www.arduino.cc/playground/Code/Spi
// Note that there is an Arduino SPI library and an Arduino Spi library - they are not the same!!

#include <SdFat.h>
#include <Wire.h>
#include <OneWire.h>
#include <Spi.h> // NOT upper-case SPI !!
#include <DallasTemperature.h>
#include "RTClib.h"

#define LOG_INTERVAL  1000 // mills between entries
#define SERIAL_OUTPUT   1 // echo data to serial port
#define SYNC_INTERVAL 1000 // mills between calls to sync()
#define RESET_RTC_TIME 1

#define BMP085_I2C_ADDRESS 0x77
// In the RTC library, the I2C address of the DS1307/DS1338 is taken to be 0x68.

uint32_t syncTime = 0;     // time of last sync()

const int SD_CHIP_SELECT = 9;
const int ONE_WIRE_BUS = 6;
const int CARD_DETECT = 7;
const int VOLTAGE_MONITOR = 1;
const int accelChipSelectPin = 10;
// The other hardware pins used for MISO, MOSI, SCK are fixed.

const int delay_time = 200;
const unsigned char oversampling_setting = 3;
const unsigned char pressure_waittime[4] = {5, 8, 14, 26};

int ac1;
int ac2;
int ac3;
int b1;
int b2;
int mb;
int mc;
int md;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;

byte tmp;
byte readData; // The byte that data read from Spi interface will be stored in 
byte fifo[6]; // data read for x,y,z axis from the accelerometer's FIFO buffer
float x,y,z; // The ranged x,y,z data
float range; // The range of the x,y,z data returned

RTC_DS1307 RTC;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature oneWireSensors(&oneWire);

Sd2Card card;
SdVolume volume;
SdFile root;
SdFile file;

void setup(void)
{
  Wire.begin();
  Serial.begin(38400);
  oneWireSensors.begin();
  bmp085_get_cal_data();

  // initialize the SD card
  if (digitalRead(CARD_DETECT))
    error("microSD card is not inserted!");
#if SERIAL_OUTPUT      
  if (!(digitalRead(CARD_DETECT)))
    Serial.println("microSD card inserted.");
#endif    
  if (!card.init(SPI_HALF_SPEED, SD_CHIP_SELECT)) error("card.init");
  // initialize a FAT volume
  if (!volume.init(card)) error("volume.init");
  // open root directory
  if (!root.openRoot(volume)) error("openRoot");
  
  // create a new file
  char name[] = "LOGGER00.TXT";
  
  // Makes a new incremented filename every time you boot
  for (uint8_t i = 0; i < 100; i++)
  {
    name[6] = i/10 + '0';
    name[7] = i%10 + '0';
    if (file.open(root, name, O_CREAT | O_EXCL | O_WRITE)) break;
  }
  
  if (!file.isOpen()) error("file.create");
#if SERIAL_OUTPUT
  Serial.print("Logging to: ");
  Serial.println(name);
#endif
  file.writeError = 0;

  Wire.begin();  
  if (!RTC.begin())
  {
    file.println("RTC failed");
#if SERIAL_OUTPUT
    Serial.println("RTC failed");
#endif
  }
  
  file.println("millis,stamp,datetime,pressure,temp,dallas_temp,x_accel,y_accel,z_accel,vcc");    
#if SERIAL_OUTPUT
  Serial.println("millis,stamp,datetime,pressure,temp,dallas_temp,x_accel,y_accel,z_accel,vcc");
#endif

  if (file.writeError || !file.sync())
    error("write header");
    
  Spi.mode((1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) | (1 << SPR1) | (1 << SPR0));
  // Set select high, slave disabled waiting to pull low for first exchange
  digitalWrite(accelChipSelectPin, HIGH);
  delay(4000);
  // Wait for POWER_CTL register to go to correct state
  readData = 0x00;
  
  while (readData != 0x28)
  {    
    // POWER_CTL register: measure
    digitalWrite(accelChipSelectPin, LOW);
    Spi.transfer(0x2D);
    Spi.transfer(0x28); // Measure
    digitalWrite(accelChipSelectPin, HIGH);
    delay(5);
    digitalWrite(accelChipSelectPin, LOW);
    Spi.transfer(1<<7 | 0x2D); // Set "read" MSb
    readData = Spi.transfer(0x00); // Send dummy byte to keep clock pulse going!
    digitalWrite(accelChipSelectPin, HIGH);
    delay(1000);
  }
  
  // Set format
  digitalWrite(accelChipSelectPin, LOW);
  Spi.transfer(0x31);
  Spi.transfer(0x08); 
  digitalWrite(accelChipSelectPin, HIGH);
  delay(5);
  // Readback format
  digitalWrite(accelChipSelectPin, LOW);
  Spi.transfer(1<<7 | 0x31);
  readData = Spi.transfer(0x00); 
  digitalWrite(accelChipSelectPin, HIGH);
  readData = readData & 0x03;
  
  switch (readData)
  {
    case 0:
      range = 2.0;
      break;
    case 1:
      range = 4.0;
      break;
    case 2:
      range = 8.0;
      break;
    case 3:
      range = 16.0;
      break;
  }
  // Set FIFO
  digitalWrite(accelChipSelectPin, LOW);
  Spi.transfer(0x38);
  Spi.transfer(0x00); 
  digitalWrite(accelChipSelectPin, HIGH);
  delay(5);
  // Readback FIFO
  digitalWrite(accelChipSelectPin, LOW);
  Spi.transfer(1<<7 | 0x38);
  readData = Spi.transfer(0x00); 
  digitalWrite(accelChipSelectPin, HIGH);
  
#if RESET_RTC_TIME  
  RTC.adjust(DateTime(__DATE__, __TIME__));
#endif  

  delay(4000);
}

void loop(void)
{
  DateTime now;
  oneWireSensors.requestTemperatures();
  int raw_temperature = 0;
  float pressure = 0;
  long raw_pressure = 0;
  float voltage = 0.0;
  float temperature = 0.0;
  
  file.writeError = 0;

  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  // log milliseconds since starting
  uint32_t m = millis();
  file.print(m);
  file.print(", ");    
#if SERIAL_OUTPUT
  Serial.print(m);
  Serial.print(", ");  
#endif

  // fetch the time
  now = RTC.now();
  // log time
  file.print(now.unixtime());
  file.print(", ");
  file.print('"');
  file.print(now.year(), DEC);
  file.print("/");
  file.print(now.month(), DEC);
  file.print("/");
  file.print(now.day(), DEC);
  file.print(" ");
  file.print(now.hour(), DEC);
  file.print(":");
  file.print(now.minute(), DEC);
  file.print(":");
  file.print(now.second(), DEC);
  file.print('"');
#if SERIAL_OUTPUT
  Serial.print(now.unixtime());
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
#endif

  bmp085_read_temperature_and_pressure(&raw_temperature, &raw_pressure);
  temperature = (((raw_temperature - (raw_temperature % 10)) / 10) + (float)((raw_temperature % 10) / (float)10));
  pressure = (raw_pressure / (float)100);
  voltage = ((float)(analogRead(VOLTAGE_MONITOR) / (float)97) + 0.6);
  
     // All x,y,z data must be read from FIFO in a multiread burst
  digitalWrite(accelChipSelectPin, LOW);
  // Start reading at 0x32 and set "Read" and "Multi" bits
  Spi.transfer(1<<7 | 1<<6 | 0x32);
  for (int i=0; i<6; i++)
    fifo[i] = Spi.transfer(0x00);
  digitalWrite(accelChipSelectPin, HIGH);
  delay(5);
  // The measurements in the FIFO 10 bit  
  x = (float)((fifo[1]<<8) | fifo[0]) * range / 512.0;
  y = (float)((fifo[3]<<8) | fifo[2]) * range / 512.0;
  z = (float)((fifo[5]<<8) | fifo[4]) * range / 512.0;
  
  file.print(", ");    
  file.print(pressure);
  file.print(", ");    
  file.print(temperature);
  file.print(", ");
  file.print(oneWireSensors.getTempCByIndex(0));
  file.print(", ");
  file.print(x);
  file.print(", ");
  file.print(y);
  file.print(", ");
  file.print(z);
  file.print(", ");
  file.print(voltage);
  file.println();
  
#if SERIAL_OUTPUT
  Serial.print(", ");   
  Serial.print(pressure);
  Serial.print(", ");    
  Serial.print(temperature);
  Serial.print(", ");
  Serial.print(oneWireSensors.getTempCByIndex(0));
  Serial.print(", ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.print(", ");
  Serial.print(voltage);
  Serial.println();
#endif

  if (file.writeError) error("write data");
  
  // don't sync too often - requires 2048 bytes of I/O to SD card
  if ((millis() - syncTime) <  SYNC_INTERVAL) return;
  syncTime = millis();
  if (!file.sync()) error("sync");
}

void error(char *str)
{
#if  SERIAL_OUTPUT
  Serial.print("error: ");
  Serial.println(str);
#endif
}

void bmp085_read_temperature_and_pressure(int* temperature, long* pressure)
{
  int ut = bmp085_read_ut();
  long up = bmp085_read_up();
  long x1, x2, x3, b3, b5, b6, p;
  unsigned long b4, b7;

  //calculate the temperature
  x1 = ((long)ut - ac6) * ac5 >> 15;
  x2 = ((long) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  *temperature = (b5 + 8) >> 4;

  //calculate the pressure
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11;
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;

  if (oversampling_setting == 3) b3 = ((int32_t) ac1 * 4 + x3 + 2) << 1;
  if (oversampling_setting == 2) b3 = ((int32_t) ac1 * 4 + x3 + 2);
  if (oversampling_setting == 1) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 1;
  if (oversampling_setting == 0) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;

  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) up - b3) * (50000 >> oversampling_setting);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  *pressure = p + ((x1 + x2 + 3791) >> 4);
}

unsigned int bmp085_read_ut()
{
  write_register(0xf4,0x2e);
  delay(5);
  return read_int_register(0xf6);
}

void bmp085_get_cal_data()
{
  ac1 = read_int_register(0xAA);
  ac2 = read_int_register(0xAC);
  ac3 = read_int_register(0xAE);
  ac4 = read_int_register(0xB0);
  ac5 = read_int_register(0xB2);
  ac6 = read_int_register(0xB4);
  b1 = read_int_register(0xB6);
  b2 = read_int_register(0xB8);
  mb = read_int_register(0xBA);
  mc = read_int_register(0xBC);
  md = read_int_register(0xBE);
}

long bmp085_read_up()
{
  write_register(0xf4,0x34+(oversampling_setting<<6));
  delay(pressure_waittime[oversampling_setting]);
  unsigned char msb, lsb, xlsb;
  
  Wire.beginTransmission(BMP085_I2C_ADDRESS);
  Wire.send(0xf6);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_I2C_ADDRESS, 3);
  while(!Wire.available());
  msb = Wire.receive();
  while(!Wire.available());
  lsb |= Wire.receive();
  while(!Wire.available());
  xlsb |= Wire.receive();

  return (((long)msb << 16) | ((long)lsb << 8) | ((long)xlsb)) >> (8 - oversampling_setting);
}

void write_register(unsigned char r, unsigned char v)
{
  Wire.beginTransmission(BMP085_I2C_ADDRESS);
  Wire.send(r);
  Wire.send(v);
  Wire.endTransmission();
}

char read_register(unsigned char r)
{
  unsigned char v;
  Wire.beginTransmission(BMP085_I2C_ADDRESS);
  Wire.send(r); // register to read
  Wire.endTransmission();

  Wire.requestFrom(BMP085_I2C_ADDRESS, 1); // read a byte
  while(!Wire.available());
  v = Wire.receive();
  return v;
}

int read_int_register(unsigned char r)
{
  unsigned char msb, lsb;
  Wire.beginTransmission(BMP085_I2C_ADDRESS);
  Wire.send(r); // register to read
  Wire.endTransmission();

  Wire.requestFrom(BMP085_I2C_ADDRESS, 2); // read a byte
  while(!Wire.available());
  msb = Wire.receive();
  while(!Wire.available());
  lsb = Wire.receive();
  return (((int)msb<<8) | ((int)lsb));
}
