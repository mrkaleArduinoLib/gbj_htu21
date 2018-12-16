/*
  NAME:
  gbjHTU21

  DESCRIPTION:
  Library for humidity and temperature sensors HTU21D(F), SHT21, SHT20, HDC1080
  on two-wire (I2C) bus.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the license GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
  (related to original code) and MIT License (MIT) for added code.

  CREDENTIALS:
  Author: Libor Gabaj
  GitHub: https://github.com/mrkaleArduinoLib/gbj_htu21.git
*/
#ifndef GBJ_HTU21_H
#define GBJ_HTU21_H

#include "gbj_twowire.h"


class gbj_htu21 : public gbj_twowire
{
public:
//------------------------------------------------------------------------------
// Public constants
//------------------------------------------------------------------------------
static const String VERSION;
enum Addresses
{
  ADDRESS = 0x40,  // Hardware address
};
enum ErrorCodes
{
  ERROR_RESET = 255,  // Sensor reset failure
  ERROR_SERIAL_A = 254,  // Serial number upper double word reading failure
  ERROR_SERIAL_B = 253,  // Serial number upper double word reading failure
  ERROR_REG_RHT_READ = 252,  // Reading RH/T User Register 1 failure
  ERROR_MEASURE_RHUM = 251,  // Measuring relative humidity failure
  ERROR_MEASURE_TEMP = 250,  // Measuring temperature failure
};
enum Resolutions  // In bits
{
  RESOLUTION_T14_RH12 = 14,
  RESOLUTION_T13_RH10 = 13,
  RESOLUTION_T12_RH8 = 12,
  RESOLUTION_T11_RH11 = 11,
};


//------------------------------------------------------------------------------
// Public methods
//------------------------------------------------------------------------------
/*
  Constructor taken from parent class.
*/
gbj_htu21(uint32_t clockSpeed = CLOCK_100KHZ, uint8_t pinSDA = 4, uint8_t pinSCL = 5) \
: gbj_twowire(clockSpeed, pinSDA, pinSCL) {};


/*
  Initialize sensor with parameters specific for it

  DESCRIPTION:
  The method sanitizes and stores input parameters to the class instance object,
  which determines the operation modus of the sensor.

  PARAMETERS:
  holdMasterMode - Flag about blocking (holding) serial clock line during
                   measurement. At no holding master mode other communication
                   on the bus can be performed.
                   - Data type: boolean
                   - Default value: false
                   - Limited range: true, false
  RETURN:
  Result code.
*/
uint8_t begin(bool holdMasterMode = true);


/*
  Reset sensor.

  DESCRIPTION:
  The method resets the sensor and sets control registers to their reset settings
  values.

  PARAMETERS: none

  RETURN:
  Result code.
*/
uint8_t reset();


/*
  Measure relative humidity.

  DESCRIPTION:
  The method measures relative humidity in per cents.

  PARAMETERS: none

  RETURN:
  Relative humidity in per cents or error code ERROR_MEASURE_RHUM.
*/
float measureHumidity();


/*
  Measure temperature.

  DESCRIPTION:
  The method measures ambient temperature in centigrades.

  PARAMETERS: none

  RETURN:
  Temperature in centigrades or error code ERROR_MEASURE_TEMP.
*/
float measureTemperature();


//------------------------------------------------------------------------------
// Public setters - they usually return result code.
//------------------------------------------------------------------------------
inline void setUseValuesTyp() { _status.useValuesTyp = true; };
inline void setUseValuesMax() { _status.useValuesTyp = false; };
inline uint8_t setAddress() { return gbj_twowire::setAddress(ADDRESS); };
inline uint8_t setHeaterEnabled() { return setHeaterStatus(true); };  // Turn on sensor's heater
inline uint8_t setHeaterDisabled() { return setHeaterStatus(false); };  // Turn off sensor's heater
//
inline uint8_t setResolutionTemp14() { return setBitResolution(false, false); };
inline uint8_t setResolutionTemp13() { return setBitResolution(true, false); };
inline uint8_t setResolutionTemp12() { return setBitResolution(false, true); };
inline uint8_t setResolutionTemp11() { return setBitResolution(true, true); };
//
inline uint8_t setResolutionRhum12() { return setResolutionTemp14(); };
inline uint8_t setResolutionRhum10() { return setResolutionTemp13(); };
inline uint8_t setResolutionRhum11() { return setResolutionTemp11(); };
inline uint8_t setResolutionRhum8() { return setResolutionTemp12(); };
//
inline void setHoldMasterMode(bool holdMasterMode) { _status.holdMasterMode = holdMasterMode; };



/*
  Set measurement resolution for temperature and relative humidity at once.

  DESCRIPTION:
  The method sets the bit resolution by input parameter.
  The resolution is determined by corresponding constant but in fact it is
  the bit resolution for temperature.

  PARAMETERS:
  resolution  - Desired measurement resolution.
              - Data type: non-negative integer
              - Default value: RESOLUTION_T14_RH12
              - Limited range: RESOLUTION_T11_RH11 ~ RESOLUTION_T14_RH12

  RETURN:
  Result code.
*/
uint8_t setResolution(uint8_t resolution = RESOLUTION_T14_RH12);


//------------------------------------------------------------------------------
// Public getters
//------------------------------------------------------------------------------
inline uint16_t getSNA() { return _status.serialSNA; };
inline uint32_t getSNB() { return _status.serialSNB; };
inline uint16_t getSNC() { return _status.serialSNC; };
uint64_t getSerialNumber();
inline bool getHoldMasterMode()  { return _status.holdMasterMode; };
bool getVddStatus();  // Flag about correct operating voltage
bool getHeaterEnabled();  // Flag about enabling the sensor's heater
uint8_t getResolutionTemp();  // Temperature resolution in bits
uint8_t getResolutionRhum();  // Relative humidity resolution in bits
inline float getErrorRHT() { return (float) PARAM_BAD_RHT; };


private:
//------------------------------------------------------------------------------
// Private constants
//------------------------------------------------------------------------------
enum Commands
{
  CMD_MEASURE_RH_HOLD = 0xE5,  // Measure Relative Humidity, Hold Master Mode
  CMD_MEASURE_RH_NOHOLD = 0xF5,  // Measure Relative Humidity, No Hold Master Mode
  CMD_MEASURE_TEMP_HOLD = 0xE3,  // Measure Temperature, Hold Master Mode
  CMD_MEASURE_TEMP_NOHOLD = 0xF3,  // Measure Temperature, No Hold Master Mode
  CMD_RESET = 0xFE,  // Soft reset
  CMD_REG_RHT_WRITE = 0xE6,  // Write RH/T User Register
  CMD_REG_RHT_READ = 0xE7,  // Read RH/T User Register
  CMD_READ_SNB = 0xFA0F,  // Read Electronic ID SNB bytes
  CMD_READ_SNAC = 0xFCC9,  // Read Electronic ID SNA and SNC bytes
  };
enum Timing
{
  TIMING_RESET = 15,  // Resetting delay in milliseconds
};
enum Resetting
{
  RESET_REG_USER = 0x02,  // Reset Settings = 0000_0010 (datasheet User Register)
};
enum Parameters
{
  PARAM_CRC_CHECKS = 3,  // Number of repeating action at wrong CRC
  PARAM_BAD_RHT = 999,  // Unreasonable wrong relative humidity or temperature value
};

//------------------------------------------------------------------------------
// Private attributes
//------------------------------------------------------------------------------
struct
{
  uint32_t serialSNB;  // 4 SNB bytes of serial number
  uint16_t serialSNC;  // 2 SNC bytes of serial number
  uint16_t serialSNA;  // 2 SNA bytes of serial number
  bool holdMasterMode;  // Flag about active hold master mode at measuring
  bool useValuesTyp;  // Flag about using typical values from datasheet
} _status;
struct
{
  bool read;  // Flag about initialization (reading) the user register
  uint8_t value;  // Value of user register 1
} _userReg;  // Parameters of user register
struct
{
  uint8_t tempBits[4] = {14, 12, 13, 11};  // List of temperature resolutions
  uint8_t rhumBits[4] = {12,  8, 10, 11};  // List of humidity resolutions
  uint8_t tempConvTimeMax[4] = {50, 13, 25, 7};  // Maximal conversion times of temperature in milliseconds
  uint8_t tempConvTimeTyp[4] = {44, 11, 22, 6};  // Maximal conversion times of temperature in milliseconds
  uint8_t rhumConvTimeMax[4] = {16, 3, 5, 8};  // Maximal conversion times of humidity in milliseconds
  uint8_t rhumConvTimeTyp[4] = {14, 3, 4, 7};  // Maximal conversion times of humidity in milliseconds
} _resolusion;


//------------------------------------------------------------------------------
// Private methods
//------------------------------------------------------------------------------
inline bool getUseValuesTyp() { return _status.useValuesTyp; };
uint8_t getConversionTimeTempMax();
uint8_t getConversionTimeTempTyp();
uint8_t getConversionTimeRhumMax();
uint8_t getConversionTimeRhumTyp();

/*
  Calculate CRC8 checksum for 32-bit integer

  DESCRIPTION:
  The method calculates CRC8 checksum of long integer by 9-bit polynomial
  x^8+x^5+x^4+x^0 (0x131) with initialization 0x00 according to the datasheet.

  PARAMETERS:
  data  - Number for which the CRC to be calculated
          - Data type: non-negative integer
          - Default value: none
          - Limited range: 0 ~ 2^32 - 1

  RETURN:
  CRC8 checksum.
*/
uint8_t crc8(uint32_t data);


/*
  Validate 32-bit integer by provided CRC8 checksum with polynom x^8+x^5+x^4+1.

  DESCRIPTION:
  The method checks whether provided CRC8 checksum is valid for input long
  integer.

  PARAMETERS:
  data  - Number for which the CRC to be calculated
          - Data type: non-negative integer
          - Default value: none
          - Limited range: 0 ~ 2^32 - 1

  crc   - Expected CRC8 checksum.
          - Data type: non-negative integer
          - Default value: none
          - Limited range: 0 ~ 255

  RETURN:
  Flag about correct checksum.
*/
bool checkCrc8(uint32_t data, uint8_t crc);


/*
  Calculate resolution code from user register byte.

  DESCRIPTION:
  The method calculates resolution code from D7 and D0 bit of the user register
  byte.

  PARAMETERS: none

  RETURN:
  Resolution code (0 ~ 3).
*/
uint8_t resolution();


/*
  Read electronic serial number.

  DESCRIPTION:
  The method reads all bytes constituting a serial number, checks them with
  crc codes read from the sensor, and stores it in the class instance object.

  PARAMETERS: none

  RETURN:
  Result code.
*/
uint8_t readSerialNumber();


/*
  Read user register.

  DESCRIPTION:
  The method reads the user register byte and parses relevant data to the
  individual items. Then all of them stores in the class instance object.

  PARAMETERS: none

  RETURN:
  Result code.
*/
uint8_t readUserRegister();


/*
  Read user register if needed.

  DESCRIPTION:
  The method reads the user register if internal flag is reset.

  PARAMETERS: none

  RETURN:
  Result code.
*/
uint8_t reloadUserRegister();


/*
  Write user register.

  DESCRIPTION:
  The method writes the user register byte stored in the class instance object
  to the user register.

  PARAMETERS: none

  RETURN:
  Result code.
*/
uint8_t writeUserRegister();


/*
  Set sensor's heater status.

  DESCRIPTION:
  The method sets the heater status bit HTRE (D2) in the user register byte.

  PARAMETERS:
  status  - Desired flag about heater enabling
          - Data type: bool
          - Default value: none
          - Limited range: true, false

  RETURN:
  Result code.
*/
uint8_t setHeaterStatus(bool status);


/*
  Set sensor's resolution.

  DESCRIPTION:
  The method sets the resolution bits RES1 (D7) and RES0 (D0) in the user
  register byte.

  PARAMETERS:
  bitRes1 - Desired flag about setting upper resolution bit (true == 1, false == 0)
            - Data type: bool
            - Default value: none
            - Limited range: true, false

  bitRes0 - Desired flag about setting lower resolution bit (true == 1, false == 0)
            - Data type: bool
            - Default value: none
            - Limited range: true, false

  RETURN:
  Result code.
*/
uint8_t setBitResolution(bool bitRes1, bool bitRes0);


/*
  Calculate temperature.

  DESCRIPTION:
  The method wraps a formula for calculating temperature in centigrade from
  16-bit word.

  PARAMETERS:
  wordMeasure - Measured binary word.
                - Data type: integer
                - Default value: none
                - Limited range: 0 ~ 0xFFFF
  RETURN:
  Temperature in centigrade or error code either ERROR_MEASURE_RHUM
  or ERROR_MEASURE_TEMP.
*/
float calculateTemperature(uint16_t wordMeasure);


/*
  Calculate relative humidity.

  DESCRIPTION:
  The method wraps a formula for calculating relative humidity in per-cents
  from 16-bit word.

  PARAMETERS:
  wordMeasure - Measured binary word.
                - Data type: integer
                - Default value: none
                - Limited range: 0 ~ 0xFFFF
  RETURN:
  Relative humidity in per-cents.
*/
float calculateHumidity(uint16_t wordMeasure);

};

#endif
