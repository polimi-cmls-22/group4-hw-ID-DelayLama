/**************************************************************************/
/*!
    @file     Adafruit_TCS34725.cpp
    @author   KTOWN (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the TCS34725 digital color sensors.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include <stdlib.h>
#include <math.h>

#include "Adafruit_TCS34725.h"

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Implements missing powf function
*/
/**************************************************************************/
float powf(const float x, const float y)
{
  return (float)(pow((double)x, (double)y));
}

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_TCS34725::write8 (uint8_t reg, uint8_t value)
{
	reg = (uint8_t)(TCS34725_COMMAND_BIT | reg);
	uint8_t buf[2] = { reg, value };
//	std::cout << "write register " << (int)reg << " i2C_file " << i2C_file << " on TCS34725\n"<< (int) value;
	if(write(i2C_file, buf, 2) != 2)
	{
		std::cout << "Failed to write register " << (int)reg << " i2C_file " << i2C_file << " on TCS34725\n";
		return;
	}
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t Adafruit_TCS34725::readRegister8(uint8_t reg)
{
    unsigned char inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    
  //    Wire.beginTransmission(TCS34725_ADDRESS);
  //#if ARDUINO >= 100
  //Wire.write(TCS34725_COMMAND_BIT | reg);
  //#else
  //Wire.send(TCS34725_COMMAND_BIT | reg);
  //#endif
  //Wire.endTransmission();
  
   //rt_printf("(readRegister8) reg %i\n",reg);
    write8(reg, 0);

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = TCS34725_ADDRESS;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = TCS34725_ADDRESS;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    
    int ret = ioctl(i2C_file, I2C_RDWR, &packets);
    if(ret < 0) {
        rt_printf("(readRegister8) Unable to send data\n");
        return 0;
    }

    return inbuf;
}



/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/

uint16_t Adafruit_TCS34725::readRegister16(uint8_t reg)
{
    unsigned char inbuf[2], outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
	write8(reg, 0);
    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = TCS34725_ADDRESS; //JB inserted the address manually
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;
    
  
    /* The data will get returned in this structure */
    messages[1].addr  = TCS34725_ADDRESS;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(i2C_file, I2C_RDWR, &packets) < 0) {
        rt_printf("(readRegister16) Unable to send data\n");
        return 0;
    }

	// JB: changed this line, the order of the bytes were inverted
    return (uint16_t(inbuf[1]) << 8) | (uint16_t(inbuf[0]) & 0x00FF);
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void Adafruit_TCS34725::enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  //delay(3);
  usleep(3000);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);  
  std::cout << TCS34725_ENABLE << ";" << TCS34725_ENABLE_PON << "\n";
  
  //JB: chaged this to the version of the arduino file
  usleep((256 - _tcs34725IntegrationTime) * 12000 / 5 + 1000);
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void Adafruit_TCS34725::disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = readRegister8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
  
}

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
Adafruit_TCS34725::Adafruit_TCS34725(tcs34725IntegrationTime_t it, tcs34725Gain_t gain) 
{
  _tcs34725Initialised = false;
  _tcs34725IntegrationTime = it;
  _tcs34725Gain = gain;
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
boolean Adafruit_TCS34725::begin(void) 
{
  if(initI2C_RW(1, TCS34725_ADDRESS, 0) > 0) { //hard coded bus (1) and addr (TCS34725_ADDRESS)
      rt_printf("initI2C_RW(1, TCS34725_ADDRESS, 0) > 0\n");
	  return false;
  }
  
  ///* Make sure we're actually connected */
  uint8_t x = readRegister8(TCS34725_ID);
  if ((x != 0x4d) && (x != 0x44) && (x != 0x10)) //JB changed this ti the version of the arduino file, it just adds another possible address. 
  {
    return false;
  }
  _tcs34725Initialised = true;

  /* Set default integration time and gain */
  setIntegrationTime(_tcs34725IntegrationTime);
  setGain(_tcs34725Gain);

  /* Note: by default, the device is in power down mode on bootup */
  enable();

  return true;
}
  
/**************************************************************************/
/*!
    Sets the integration time for the TC34725
*/
/**************************************************************************/
void Adafruit_TCS34725::setIntegrationTime(tcs34725IntegrationTime_t it)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_ATIME, it);

  /* Update value placeholders */
  _tcs34725IntegrationTime = it;
}

/**************************************************************************/
/*!
    Adjusts the gain on the TCS34725 (adjusts the sensitivity to light)
*/
/**************************************************************************/
void Adafruit_TCS34725::setGain(tcs34725Gain_t gain)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_CONTROL, gain);

  /* Update value placeholders */
  _tcs34725Gain = gain;
}

/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
void Adafruit_TCS34725::getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (!_tcs34725Initialised) begin();

  *c = readRegister16(TCS34725_CDATAL);
  *r = readRegister16(TCS34725_RDATAL);
  *g = readRegister16(TCS34725_GDATAL);
  *b = readRegister16(TCS34725_BDATAL);
  
  usleep((256 - _tcs34725IntegrationTime) * 12000 / 5 + 1000); //JB: changed this to the version of the arduino file.
  
  /* Set a delay for the integration time */
  /*switch (_tcs34725IntegrationTime)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      //delay(3);
      usleep(3000);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      //delay(24);
          usleep(24000);
  break;
    case TCS34725_INTEGRATIONTIME_50MS:
      //delay(50);
          usleep(50000);
  break;
    case TCS34725_INTEGRATIONTIME_101MS:
      //delay(101);
          usleep(101000);
  break;
    case TCS34725_INTEGRATIONTIME_154MS:
      //delay(154);
         usleep(154000);
   break;
    case TCS34725_INTEGRATIONTIME_700MS:
      //delay(700);
          usleep(700000);
  break;
  }
  */
}

/*!
 *  @brief  Read the RGB color detected by the sensor.
 *  @param  *r
 *          Red value normalized to 0-255
 *  @param  *g
 *          Green value normalized to 0-255
 *  @param  *b
 *          Blue value normalized to 0-255
 */
void Adafruit_TCS34725::getRGB(float *r, float *g, float *b) {
  uint16_t red, green, blue, clear;
  getRawData(&red, &green, &blue, &clear);
  uint32_t sum = clear;

  // Avoid divide by zero errors ... if clear = 0 return black
  if (clear == 0) {
    *r = *g = *b = 0;
    return;
  }

  *r = (float)red / sum * 255.0;
  *g = (float)green / sum * 255.0;
  *b = (float)blue / sum * 255.0;
}




uint16_t Adafruit_TCS34725::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to lux
*/
/**************************************************************************/
uint16_t Adafruit_TCS34725::calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}


void Adafruit_TCS34725::setInterrupt(boolean i) {
  uint8_t r = readRegister8(TCS34725_ENABLE);
  if (i) {
    r |= TCS34725_ENABLE_AIEN;
  } else {
    r &= ~TCS34725_ENABLE_AIEN;
  }
  write8(TCS34725_ENABLE, r);
}

void Adafruit_TCS34725::clearInterrupt(void) {
  //ks note: i'm not sure what's going on here, since the write functions are not being passed a value?
	
  //Wire.beginTransmission(TCS34725_ADDRESS);
  //#if ARDUINO >= 100
  //Wire.write(TCS34725_COMMAND_BIT | 0x66);
  //write8(TCS34725_COMMAND_BIT | 0x66)
  //#else
  //Wire.send(TCS34725_COMMAND_BIT | 0x66);
  //#endif
  //Wire.endTransmission();
}


void Adafruit_TCS34725::setIntLimits(uint16_t low, uint16_t high) {
   write8(0x04, low & 0xFF);
   write8(0x05, low >> 8);
   write8(0x06, high & 0xFF);
   write8(0x07, high >> 8);
}
