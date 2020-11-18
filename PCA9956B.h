/*!
 *  @file PCA9956.h
 *
  *  Library to interface PCA9956B 24-channel LED driver with Arduino etc.
 *  The code is based on Adafruit-PWM-Servo-Driver-Library for PCA 9685 (https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
 *  I just modified some register adresses and functions.

 *  The code was successfully tested on an ESP32, but should work on any Arduino board.
 *  For further information refer to the PCA9956B data sheet: https://www.nxp.com/docs/en/data-sheet/PCA9956B.pdf
 *  or feel free to email me (github@larss.de)
 *  
 */
#ifndef _PCA9956B_H
#define _PCA9956B_H

#include <Arduino.h>
#include <Wire.h>

// REGISTER ADDRESSES
#define PCA9956B_MODE1 0x00      /**< Mode Register 1 */
#define PCA9956B_MODE2 0x01      /**< Mode Register 2 */
#define PCA9956B_LEDOUT0 0x02      /**< LED state Register 0 */
#define PCA9956B_LEDOUT1 0x03      /**< LED state Register 1 */
#define PCA9956B_LEDOUT2 0x04      /**< LED state Register 2 */
#define PCA9956B_LEDOUT3 0x05      /**< LED state Register 3 */
#define PCA9956B_LEDOUT4 0x06      /**< LED state Register 4 */
#define PCA9956B_LEDOUT5 0x07      /**< LED state Register 5 */
#define PCA9956B_GRPPWM 0x08      /**< group duty cycle control */
#define PCA9956B_GRPFREQ 0x09      /**< group frequency */

#define PCA9956B_PWM0 0x0A      /**< brightness LED0 */
#define PCA9956B_PWM1 0x0B      /**< brightness LED1 */
#define PCA9956B_PWM2 0x0C      /**< brightness LED2 */
#define PCA9956B_PWM3 0x0D      /**< brightness LED3 */
#define PCA9956B_PWM4 0x0E      /**< brightness LED4 */
#define PCA9956B_PWM5 0x0F      /**< brightness LED5 */
#define PCA9956B_PWM6 0x10      /**< brightness LED6 */
#define PCA9956B_PWM7 0x11      /**< brightness LED7 */
#define PCA9956B_PWM8 0x12      /**< brightness LED8 */
#define PCA9956B_PWM9 0x13      /**< brightness LED9 */
#define PCA9956B_PWM10 0x14      /**< brightness LED10 */
#define PCA9956B_PWM11 0x15      /**< brightness LED11 */
#define PCA9956B_PWM12 0x16      /**< brightness LED12 */
#define PCA9956B_PWM13 0x17      /**< brightness LED13 */
#define PCA9956B_PWM14 0x18      /**< brightness LED14 */
#define PCA9956B_PWM15 0x19      /**< brightness LED15 */
#define PCA9956B_PWM16 0x1A      /**< brightness LED16 */
#define PCA9956B_PWM17 0x1B      /**< brightness LED17 */
#define PCA9956B_PWM18 0x1C      /**< brightness LED18 */
#define PCA9956B_PWM19 0x1D      /**< brightness LED19 */
#define PCA9956B_PWM20 0x1E      /**< brightness LED20 */
#define PCA9956B_PWM21 0x1F      /**< brightness LED21 */
#define PCA9956B_PWM22 0x20      /**< brightness LED22 */
#define PCA9956B_PWM23 0x21      /**< brightness LED23 */

#define PCA9956B_IREF0 0x22      /**< output current LED0 */
#define PCA9956B_IREF1 0x23      /**< output current LED1 */
#define PCA9956B_IREF2 0x24      /**< output current LED2 */
#define PCA9956B_IREF3 0x25      /**< output current LED3 */
#define PCA9956B_IREF4 0x26      /**< output current LED4 */
#define PCA9956B_IREF5 0x27      /**< output current LED5 */
#define PCA9956B_IREF6 0x28      /**< output current LED6 */
#define PCA9956B_IREF7 0x29      /**< output current LED7 */
#define PCA9956B_IREF8 0x2A      /**< output current LED8 */
#define PCA9956B_IREF9 0x2B      /**< output current LED9 */
#define PCA9956B_IREF10 0x2C      /**< output current LED10 */
#define PCA9956B_IREF11 0x2D      /**< output current LED11 */
#define PCA9956B_IREF12 0x2E      /**< output current LED12 */
#define PCA9956B_IREF13 0x2F      /**< output current LED13 */
#define PCA9956B_IREF14 0x30      /**< output current LED14 */
#define PCA9956B_IREF15 0x31      /**< output current LED15 */
#define PCA9956B_IREF16 0x32      /**< output current LED16 */
#define PCA9956B_IREF17 0x33      /**< output current LED17 */
#define PCA9956B_IREF18 0x34      /**< output current LED18 */
#define PCA9956B_IREF19 0x35      /**< output current LED19 */
#define PCA9956B_IREF20 0x36      /**< output current LED20 */
#define PCA9956B_IREF21 0x37      /**< output current LED21 */
#define PCA9956B_IREF22 0x38      /**< output current LED22 */
#define PCA9956B_IREF23 0x39      /**< output current LED23 */

#define PCA9956B_OFFSET 0x3B    /**< Delay on LEDn outputs */
#define PCA9956B_SUBADR1 0x3B    /**< I2C-bus subaddress 1 */
#define PCA9956B_SUBADR2 0x3C    /**< I2C-bus subaddress 2 */
#define PCA9956B_SUBADR3 0x3D    /**< I2C-bus subaddress 3 */
#define PCA9956B_ALLCALLADR 0x3E /**< LED All Call I2C-bus address */
#define PCA9956B_PWMALL 0x3F    /**< brightness control for all LEDn */
#define PCA9956B_IREFALL 0x40    /**< output current control for all registers IREF0 to IREF23 */
#define PCA9956B_EFLAG0 0x41    /**< output error flag 0 */
#define PCA9956B_EFLAG1 0x42    /**< output error flag 1 */
#define PCA9956B_EFLAG2 0x43    /**< output error flag 2 */
#define PCA9956B_EFLAG3 0x44    /**< output error flag 3 */
#define PCA9956B_EFLAG4 0x45    /**< output error flag 4 */
#define PCA9956B_EFLAG5 0x46    /**< output error flag 5 */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI0 0x20      /**< Auto-Increment bit 0*/
#define MODE1_AI1 0x40      /**< Auto-Increment bit 2 */
#define MODE1_AIF 0x80      /**< Register Auto-Increment enabled */
// MODE2 bits
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_CLRERR 0x10  /**< clear all EFLAGn bits */
#define MODE2_DMBLNK 0x20  /**< group control blinking vs dimming */
#define MODE2_ERROR 0x40  /**< error in EFLAGn detected */
#define MODE2_OVERTEMP 0x80  /**< overtemp detection */

//#define PCA9956B_I2C_ADDRESS 0x40      /**< Default PCA9956B I2C Slave Address */
//#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */
//#define PCA9956B_PRESCALE_MIN 3   /**< minimum prescale value */
//#define PCA9956B_PRESCALE_MAX 255 /**< maximum prescale value */

/*!
 *  @brief  Class that stores state and functions for interacting with PCA9956B
 * PWM chip
 */
class PCA9956B {
public:
  PCA9956B(const uint8_t addr);
  void begin();
  void reset();
  void sleep();
  void wakeup();
  void error();
  void clrerr();
  void setPWM(uint8_t pin, uint8_t val);
  void setIREF(uint8_t pin, uint8_t val);
  void getREG(uint8_t reg);
  void setREG(uint8_t reg, uint8_t d);
  

private:
  uint8_t _i2caddr;
  TwoWire *_i2c;

  void chkerrflag(uint8_t reg, uint8_t num);
  uint8_t read8(uint8_t addr);
  void write8(uint8_t addr, uint8_t d);
};

#endif
