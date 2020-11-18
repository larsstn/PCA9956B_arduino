/*!
 *  @file PCA9956B.cpp
 *
 *  @mainpage PCA9956B Arduino Driver Library
 *
 *  @section intro_sec Introduction
 *
 *  Library to interface PCA9956B 24-channel LED driver with Arduino etc.
 *  The code is based on Adafruit-PWM-Servo-Driver-Library for PCA 9685 (https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
 *  I just modified some register adresses and functions.

 *  The code was successfully tested on an ESP32, but should work on any Arduino board.
 *  For further information refer to the PCA9956B data sheet: https://www.nxp.com/docs/en/data-sheet/PCA9956B.pdf
 *  or feel free to email me (github@larss.de)
 *
 *  @section author Author
 *
 *  larsstn
 *
 * 
 */

#include "PCA9956B.h"

#define ENABLE_DEBUG_OUTPUT 0

/*!
 *  @brief  Instantiates a new PCA9956B PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 */
PCA9956B::PCA9956B(const uint8_t addr)
    : _i2caddr(addr), _i2c(&Wire) {}


/*!
 *  @brief  Setups the I2C interface and hardware
 */
void PCA9956B::begin() {
  _i2c->begin();
  reset();
}

/*!
 *  @brief  Sends a reset command to the PCA9956B chip over I2C
 */
void PCA9956B::reset() { 
  _i2c->beginTransmission(0x00);
  _i2c->write(0x06);
  _i2c->endTransmission();
  delay(10);
}

/*!
 *  @brief  Puts board into sleep mode
 */
void PCA9956B::sleep() {
  uint8_t awake = read8(PCA9956B_MODE1);
  uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
  write8(PCA9956B_MODE1, sleep);
  delay(5); // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Wakes board from sleep
 */
void PCA9956B::wakeup() {
  uint8_t sleep = read8(PCA9956B_MODE1);
  uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
  write8(PCA9956B_MODE1, wakeup);
}

/*!
 *  @brief  check for errors
 */
void PCA9956B::error() {
  uint8_t mode2 = read8(PCA9956B_MODE2);
  if ((mode2 & 64) > 0) {
    Serial.println("Error detected");
    uint8_t eflag0 = read8(PCA9956B_EFLAG0);
    uint8_t eflag1 = read8(PCA9956B_EFLAG1);
    uint8_t eflag2 = read8(PCA9956B_EFLAG2);
    uint8_t eflag3 = read8(PCA9956B_EFLAG3);
    uint8_t eflag4 = read8(PCA9956B_EFLAG4);
    uint8_t eflag5 = read8(PCA9956B_EFLAG5);

    chkerrflag(eflag0, 0);
    chkerrflag(eflag1, 1);
    chkerrflag(eflag2, 2);
    chkerrflag(eflag3, 3);
    chkerrflag(eflag4, 4);
    chkerrflag(eflag5, 5);
  }

    if ((mode2 & 128) > 0) {
    Serial.println("Overtemp error");
    }
}

/*!
 *  @brief  clear errors
 */
void PCA9956B::clrerr() {
  uint8_t mode2 = read8(PCA9956B_MODE2);
  uint8_t clr = mode2 | 0x10; // set clrerr bit high
  write8(PCA9956B_MODE2, clr);
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9956B pins
 */
void PCA9956B::setPWM(uint8_t pin, uint8_t val) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM pin:");
  Serial.println(pin);
  Serial.print("value:");
  Serial.println(val);
#endif
  
  write8(PCA9956B_PWM0 + pin,val);
}


/*!
 *  @brief  Sets the IREF of one of the PCA9956B pins
 */
void PCA9956B::setIREF(uint8_t pin, uint8_t val) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting IREF pin:");
  Serial.println(pin);
  Serial.print("value:");
  Serial.println(val);
#endif
  
  write8(PCA9956B_IREF0 + pin,val);
}

void PCA9956B::getREG(uint8_t reg) {
  uint8_t val = read8(reg); 
  Serial.print("Reading Register:");
  Serial.println(reg);
  Serial.print("value:");
  Serial.println(val);
}

void PCA9956B::setREG(uint8_t reg, uint8_t d) {
  Serial.print("Writing Register:");
  Serial.println(reg);
  write8(reg,d);
}


void PCA9956B::chkerrflag(uint8_t reg, uint8_t num) {

    
        if ((reg & 1) > 0){
            Serial.print("short-curcuit on LED");
            Serial.println(4*num);
        } else if ((reg & 2) > 0){
            Serial.print("open-curcuit on LED");
            Serial.println(4*num);
        }
         if ((reg & 4) > 0){ 
            Serial.print("short-curcuit on LED");
            Serial.println(4*num+1);
        } else if ((reg & 8) > 0){
            Serial.print("open-curcuit on LED");
            Serial.println(4*num+1);
        }
         if ((reg & 16) > 0){
            Serial.print("short-curcuit on LED");
            Serial.println(4*num+2);
        } else if ((reg & 32) > 0){
            Serial.print("open-curcuit on LED");
            Serial.println(4*num+2);
        }
         if ((reg & 64) > 0){
            Serial.print("short-curcuit on LED");
            Serial.println(4*num+3);
        } else if ((reg & 128) > 0){
            Serial.print("open-curcuit on LED");
            Serial.println(4*num+3);
        }
}



/******************* Low level I2C interface */
uint8_t PCA9956B::read8(uint8_t addr) {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return _i2c->read();
}

void PCA9956B::write8(uint8_t addr, uint8_t d) {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);                        //AIF = 0
  _i2c->write(d);
  _i2c->endTransmission();
}
