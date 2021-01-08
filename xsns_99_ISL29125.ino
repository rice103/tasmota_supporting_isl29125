/*
  xsns_99_isl29125.ino - ISL29125 RGB sensor library

  Copyright (C) 2020  Rice Cipriani and Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_I2C
#ifdef USE_ISL29125
/*********************************************************************************************\
 * DHT12 - Temperature and Humidity
 *
 * I2C Address: 0x44
 * References:
 * - https://github.com/sparkfun/SparkFun_ISL29125_Breakout_Arduino_Library/
\*********************************************************************************************/

#define XSNS_99              99
#define XI2C_96              96  // See I2CDEVICES.md
#define ISL_I2C_ADDR 0x44

// ISL29125 Registers
#define DEVICE_ID 0x00
#define CONFIG_1 0x01
#define CONFIG_2 0x02
#define CONFIG_3 0x03
#define THRESHOLD_LL 0x04
#define THRESHOLD_LH 0x05
#define THRESHOLD_HL 0x06
#define THRESHOLD_HH 0x07
#define STATUS 0x08 
#define GREEN_L 0x09 
#define GREEN_H 0x0A
#define RED_L 0x0B
#define RED_H 0x0C
#define BLUE_L 0x0D
#define BLUE_H 0x0E

// Configuration Settings
#define CFG_DEFAULT 0x00

// CONFIG1
// Pick a mode, determines what color[s] the sensor samples, if any
#define CFG1_MODE_POWERDOWN 0x00
#define CFG1_MODE_G 0x01
#define CFG1_MODE_R 0x02
#define CFG1_MODE_B 0x03
#define CFG1_MODE_STANDBY 0x04
#define CFG1_MODE_RGB 0x05
#define CFG1_MODE_RG 0x06
#define CFG1_MODE_GB 0x07

// Light intensity range
// In a dark environment 375Lux is best, otherwise 10KLux is likely the best option
#define CFG1_375LUX 0x00
#define CFG1_10KLUX 0x08

// Change this to 12 bit if you want less accuracy, but faster sensor reads
// At default 16 bit, each sensor sample for a given color is about ~100ms
#define CFG1_16BIT 0x00
#define CFG1_12BIT 0x10

// Unless you want the interrupt pin to be an input that triggers sensor sampling, leave this on normal
#define CFG1_ADC_SYNC_NORMAL 0x00
#define CFG1_ADC_SYNC_TO_INT 0x20

// CONFIG2
// Selects upper or lower range of IR filtering
#define CFG2_IR_OFFSET_OFF 0x00
#define CFG2_IR_OFFSET_ON 0x80

// Sets amount of IR filtering, can use these presets or any value between 0x00 and 0x3F
// Consult datasheet for detailed IR filtering calibration
#define CFG2_IR_ADJUST_LOW 0x00
#define CFG2_IR_ADJUST_MID 0x20
#define CFG2_IR_ADJUST_HIGH 0x3F

// CONFIG3
// No interrupts, or interrupts based on a selected color
#define CFG3_NO_INT 0x00
#define CFG3_G_INT 0x01
#define CFG3_R_INT 0x02
#define CFG3_B_INT 0x03

// How many times a sensor sample must hit a threshold before triggering an interrupt
// More consecutive samples means more times between interrupts, but less triggers from short transients
#define CFG3_INT_PRST1 0x00
#define CFG3_INT_PRST2 0x04
#define CFG3_INT_PRST4 0x08
#define CFG3_INT_PRST8 0x0C

// If you would rather have interrupts trigger when a sensor sampling is complete, enable this
// If this is disabled, interrupts are based on comparing sensor data to threshold settings
#define CFG3_RGB_CONV_TO_INT_DISABLE 0x00
#define CFG3_RGB_CONV_TO_INT_ENABLE 0x10

// STATUS FLAG MASKS
#define FLAG_INT 0x01
#define FLAG_CONV_DONE 0x02
#define FLAG_BROWNOUT 0x04
#define FLAG_CONV_G 0x10
#define FLAG_CONV_R 0x20
#define FLAG_CONV_B 0x30

uint16_t redValue=0;
uint16_t greenValue=0;
uint16_t blueValue=0;
uint16_t readDone = false;
int isl29125Found = 0;

void Isl29125Detect(void)
{
  if (I2cActive(ISL_I2C_ADDR)) { return; }

  if (isl29125_read()){
    if (isl29125Found==0)
      if (isl29125_init())
        isl29125Found = 1;
    I2cSetActiveFound(ISL_I2C_ADDR, "ISL29125");
  }
}

bool isl29125_init()
{
  bool ret = true;
  uint8_t data = 0x00;
  
  // Check device ID
  data = isl29125_read8(DEVICE_ID);
  if (data != 0x7D)
  {
    ret &= false;
  }

  // Reset registers
  ret &= isl29125_reset();

  // Set to RGB mode, 10k lux, and high IR compensation
  ret &= isl29125_config(CFG1_MODE_RGB | CFG1_10KLUX, CFG2_IR_ADJUST_HIGH, CFG_DEFAULT);

  return ret;
}

// Reset all registers - returns true if successful
bool isl29125_reset()
{
  uint8_t data = 0x00;
  // Reset registers
  isl29125_write8(DEVICE_ID, 0x46);
  // Check reset
  data = isl29125_read8(CONFIG_1);
  data |= isl29125_read8(CONFIG_2);
  data |= isl29125_read8(CONFIG_3);
  data |= isl29125_read8(STATUS);
  if (data != 0x00)
  {
    return false;
  }
  return true;
}

// Setup Configuration registers (three registers) - returns true if successful
// Use CONFIG1 variables from SFE_ISL29125.h for first parameter config1, CONFIG2 for config2, 3 for 3
// Use CFG_DEFAULT for default configuration for that register
bool isl29125_config(uint8_t config1, uint8_t config2, uint8_t config3)
{
  bool ret = true;
  uint8_t data = 0x00;

  // Set 1st configuration register
  isl29125_write8(CONFIG_1, config1);
  // Set 2nd configuration register
  isl29125_write8(CONFIG_2, config2);
  // Set 3rd configuration register
  isl29125_write8(CONFIG_3, config3);

  // Check if configurations were set correctly
  data = isl29125_read8(CONFIG_1);
  if (data != config1)
  {
    ret &= false;
  }
  data = isl29125_read8(CONFIG_2);
  if (data != config2)
  {
    ret &= false;
  }
  data = isl29125_read8(CONFIG_3);
  if (data != config3)
  {
    ret &= false;
  }
  return ret;
}

// Sets upper threshold value for triggering interrupts
void isl29125_setUpperThreshold(uint16_t data)
{
  isl29125_write16(THRESHOLD_HL, data);
}

// Sets lower threshold value for triggering interrupts
void isl29125_setLowerThreshold(uint16_t data)
{
  isl29125_write16(THRESHOLD_LL, data);
}

// Check what the upper threshold is, 0xFFFF by default
uint16_t isl29125_readUpperThreshold()
{
  return isl29125_read16(THRESHOLD_HL);
}

// Check what the upper threshold is, 0x0000 by default
uint16_t isl29125_readLowerThreshold()
{
  return isl29125_read16(THRESHOLD_LL);
}

// Read the latest Sensor ADC reading for the color Red
bool isl29125_read()
{
  readDone = 0;
  Wire.beginTransmission(ISL_I2C_ADDR);
  Wire.write(0);
  if (Wire.endTransmission() != 0) { return false; }

  redValue = isl29125_read16(RED_L);
  greenValue = isl29125_read16(GREEN_L);
  blueValue = isl29125_read16(BLUE_L);
  readDone = 1;
  return true;
}

// Check status flag register that allows for checking for interrupts, brownouts, and ADC conversion completions
uint8_t isl29125_readStatus()
{
  return isl29125_read8(STATUS);
}

// Generic I2C read register (single byte)
uint8_t isl29125_read8(uint8_t reg)
{
  Wire.beginTransmission(ISL_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
//  Wire.beginTransmission(ISL_I2C_ADDR);
  Wire.requestFrom(ISL_I2C_ADDR,(uint8_t)1);
  uint8_t data = Wire.read();
//  Wire.endTransmission();

  return data;
}

// Generic I2C write data to register (single byte)
void isl29125_write8(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(ISL_I2C_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();

  return;
}

// Generic I2C read registers (two bytes, LSB first)
uint16_t isl29125_read16(uint8_t reg)
{
  uint16_t data = 0x0000;

  Wire.beginTransmission(ISL_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

//  Wire.beginTransmission(ISL_I2C_ADDR);
  Wire.requestFrom(ISL_I2C_ADDR, (uint8_t)2); // request 2 bytes of data
  data = Wire.read();
  data |= (Wire.read() << 8);
//  Wire.endTransmission();

  return data;
}

// Generic I2C write data to registers (two bytes, LSB first)
void isl29125_write16(uint8_t reg, uint16_t data)
{
  Wire.beginTransmission(ISL_I2C_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.write(data>>8);
  Wire.endTransmission();
}

#ifdef USE_WEBSERVER
const char HTTP_SNS99[] PROGMEM =
  "{s}ISL29125 {m}R: %s; G: %s; B: %s; D: %s{e}";  // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
#endif  // USE_WEBSERVER

void Isl29125Show(bool json)
{
    char red_chr[33];
    dtostrfd(redValue, 3, red_chr);
    char green_chr[33];
    dtostrfd(greenValue, 3, green_chr);
    char blue_chr[33];
    dtostrfd(blueValue, 3, blue_chr);
    char done_chr[33];
    dtostrfd(readDone, 3, done_chr);

    if(json) {
      ResponseAppend_P(PSTR(",\"ISL29125\":{\"" D_JSON_RED "\":%s,\"" D_JSON_GREEN "\":%s,\"" D_JSON_BLUE "\":%s,\"" D_JSON_DONE "\":%s}"), red_chr, green_chr, blue_chr, done_chr);
#ifdef USE_WEBSERVER
    } else {
      WSContentSend_PD(HTTP_SNS99, red_chr, green_chr, blue_chr,done_chr);
#endif  // USE_WEBSERVER
    }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns99(uint8_t function)
{
  if (!I2cEnabled(XI2C_96)) { return false; }

  bool result = true;
  switch (function) 
  {
    case FUNC_INIT:
      Isl29125Detect();
      break;
    case FUNC_EVERY_SECOND:
      if (!isl29125_read())
      {
        isl29125Found=0;
        result=false;
      }
      break;
    case FUNC_JSON_APPEND:
      Isl29125Show(1);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      Isl29125Show(0);
      break;
#endif  // USE_WEBSERVER
   }
  return result;
}

#endif  // USE_ISL29125
#endif  // USE_I2C
