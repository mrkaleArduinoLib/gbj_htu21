/*
  NAME:
  gbjHTU21

  DESCRIPTION:
  Library for humidity and temperature sensors HTU21D(F), SHT21, SHT20, HDC1080
  on two-wire (I2C) bus.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the license GNU GPL v3
  http://www.gnu.org/licenses/gpl-3.0.html (related to original code) and MIT
  License (MIT) for added code.

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
  gbj_htu21(ClockSpeeds clockSpeed = ClockSpeeds::CLOCK_100KHZ,
            uint8_t pinSDA = 4,
            uint8_t pinSCL = 5)
    : gbj_twowire(clockSpeed, pinSDA, pinSCL){};

  /*
    Initialize sensor.

    DESCRIPTION:
    The method sanitizes and stores input parameters to the class instance
    object, which determines the operation modus of the sensor.

    PARAMETERS:
    holdMasterMode - Flag about blocking (holding) serial clock line during
    measurement. At no holding master mode other communication on the bus can be
    performed.
      - Data type: boolean
      - Default value: false
      - Limited range: true, false

    RETURN: Result code
  */
  inline ResultCodes begin(bool holdMasterMode = true)
  {
    if (isError(gbj_twowire::begin()))
    {
      return getLastResult();
    }
    if (isError(setAddress()))
    {
      return getLastResult();
    }
    setUseValuesMax();
    setHoldMasterMode(holdMasterMode);
    if (isError(reset()))
    {
      return getLastResult();
    }
    return readSerialNumber();
  }

  /*
    Reset sensor.

    DESCRIPTION:
    The method resets the sensor and sets control registers to their reset
    settings values.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes reset()
  {
    if (isError(busSend(Commands::CMD_RESET)))
    {
      return getLastResult();
    }
    wait(Timing::TIMING_RESET);
    // Check user register reset value
    if (isError(readUserRegister()))
    {
      return getLastResult();
    }
    if (_userReg.value != Resetting::RESET_REG_USER)
    {
      return setLastResult(static_cast<ResultCodes>(ResultCodes::ERROR_RESET));
    }
    // Turn off heater
    return setHeaterDisabled();
  }

  /*
    Measure temperature.

    DESCRIPTION:
    The method measures ambient temperature in centigrades.

    PARAMETERS: none

    RETURN: Temperature in centigrades or bad measure value
  */
  inline float measureTemperature() { return readTemperature(); }

  /*
    Measure relative humidity or retrieve temperature as well.

    DESCRIPTION:
    The particular method measures either relative humidity solely or
    temperature on the same time and retrieves it through input parameter.
    - If the temperature argument is used, the humidity is compensated by the
    temperature coefficient.

    PARAMETERS:
    temperature - Referenced variable for placing a temperature value.
      - Data type: float
      - Default value: none
      - Limited range: sensor specific

    RETURN: Relative humidity in per cents or bad measure value
  */
  inline float measureHumidity()
  {
    float humidity = readHumidity();
    if (isError())
    {
      return humidity;
    }
    return sanitizeHumidity(humidity);
  }
  inline float measureHumidity(float &temperature)
  {
    temperature = measureTemperature();
    if (isError())
    {
      return getErrorRHT();
    }
    float humidity = readHumidity();
    if (isError())
    {
      return humidity;
    }
    // Temperature compensation
    humidity += (temperature - 25.0) *
                static_cast<float>(Params::PARAM_TEMP_COEF) / 1000.0;
    return sanitizeHumidity(humidity);
  }

  // Setters
  inline void setUseValuesTyp() { _status.useValuesTyp = true; }
  inline void setUseValuesMax() { _status.useValuesTyp = false; }
  inline ResultCodes setAddress()
  {
    return gbj_twowire::setAddress(Addresses::ADDRESS);
  }
  // Turn on sensor's heater
  inline ResultCodes setHeaterEnabled() { return setHeaterStatus(true); }
  // Turn off sensor's heater
  inline ResultCodes setHeaterDisabled() { return setHeaterStatus(false); }
  // Set resolution
  inline ResultCodes setResolutionTemp14()
  {
    return setBitResolution(false, false);
  }
  inline ResultCodes setResolutionTemp13()
  {
    return setBitResolution(true, false);
  }
  inline ResultCodes setResolutionTemp12()
  {
    return setBitResolution(false, true);
  }
  inline ResultCodes setResolutionTemp11()
  {
    return setBitResolution(true, true);
  }
  //
  inline ResultCodes setResolutionRhum12() { return setResolutionTemp14(); }
  inline ResultCodes setResolutionRhum10() { return setResolutionTemp13(); }
  inline ResultCodes setResolutionRhum11() { return setResolutionTemp11(); }
  inline ResultCodes setResolutionRhum8() { return setResolutionTemp12(); }
  //
  inline void setHoldMasterMode(bool holdMasterMode)
  {
    _status.holdMasterMode = holdMasterMode;
  }

  // Getters
  inline uint16_t getSNA() { return _status.serialSNA; }
  inline uint32_t getSNB() { return _status.serialSNB; }
  inline uint16_t getSNC() { return _status.serialSNC; }
  inline uint64_t getSerialNumber()
  {
    uint64_t serial;
    serial = _status.serialSNA;
    serial <<= 32;
    serial |= _status.serialSNB;
    serial <<= 16;
    serial |= _status.serialSNC;
    return serial;
  }
  inline bool getHoldMasterMode() { return _status.holdMasterMode; }
  // Flag about correct operating voltage
  inline bool getVddStatus()
  {
    // Always read user register, because the voltage status is updated after
    // each measurement
    if (isError(readUserRegister()))
    {
      return false;
    }
    uint8_t status = (_userReg.value >> 6) & B1;
    return (status == 0);
  }
  // Flag about enabling the sensor's heater
  inline bool getHeaterEnabled()
  {
    if (isError(reloadUserRegister()))
    {
      return false;
    }
    // Separate heater status from HTRE (D2) bit of user register byte
    uint8_t status = (_userReg.value >> 2) & B1;
    return (status == 1);
  }

  // Temperature resolution in bits
  inline uint8_t getResolutionTemp()
  {
    return _resolusion
      .tempBits[isSuccess(reloadUserRegister()) ? resolution() : 0];
  }

  // Relative humidity resolution in bits
  inline uint8_t getResolutionRhum()
  {
    return _resolusion
      .rhumBits[isSuccess(reloadUserRegister()) ? resolution() : 0];
  }

  // Bad measurement value
  inline float getErrorRHT()
  {
    return static_cast<float>(Params::PARAM_BAD_RHT);
  }

private:
  enum Addresses
  {
    // Hardware address
    ADDRESS = 0x40,
  };
  enum Commands : uint16_t
  {
    // Measure Relative Humidity, Hold Master Mode
    CMD_MEASURE_RH_HOLD = 0xE5,
    // Measure Relative Humidity, No Hold Master Mode
    CMD_MEASURE_RH_NOHOLD = 0xF5,
    // Measure Temperature, Hold Master Mode
    CMD_MEASURE_TEMP_HOLD = 0xE3,
    // Measure Temperature, No Hold Master Mode
    CMD_MEASURE_TEMP_NOHOLD = 0xF3,
    // Soft reset
    CMD_RESET = 0xFE,
    // Write RH/T User Register
    CMD_REG_RHT_WRITE = 0xE6,
    // Read RH/T User Register
    CMD_REG_RHT_READ = 0xE7,
    // Read Electronic ID SNB bytes
    CMD_READ_SNB = 0xFA0F,
    // Read Electronic ID SNA and SNC bytes
    CMD_READ_SNAC = 0xFCC9,
  };
  enum Timing : uint8_t
  {
    // Resetting delay in milliseconds
    TIMING_RESET = 15,
  };
  enum Resetting : uint8_t
  {
    // Reset Settings = 0000_0010 (datasheet User Register)
    RESET_REG_USER = 0x02,
  };
  enum Params : uint8_t
  {
    // Number of repeating action at wrong CRC
    PARAM_CRC_CHECKS = 3,
    // Unreasonable wrong relative humidity or temperature value
    PARAM_BAD_RHT = 255,
    // Temperature coefficient - absolute value in millipercentage per degree
    PARAM_TEMP_COEF = 150,
  };
  struct Status
  {
    // 2 SNA bytes of serial number
    uint16_t serialSNA;
    // 4 SNB bytes of serial number
    uint32_t serialSNB;
    // 2 SNC bytes of serial number
    uint16_t serialSNC;
    // Flag about active hold master mode at measuring
    bool holdMasterMode;
    // Flag about using typical values from datasheet
    bool useValuesTyp;
  } _status;
  // Parameters of user register
  struct UserReg
  {
    // Flag about initialization (reading) the user register
    bool read;
    // Value of user register 1
    uint8_t value;
  } _userReg;
  // Indexes by resolution bits D7 and D0 value in user register
  struct Resolution
  {
    // Temperature resolutions in bits
    uint8_t tempBits[4] = {
      14,
      12,
      13,
      11,
    };
    // Humidity resolutions in bits
    uint8_t rhumBits[4] = {
      12,
      8,
      10,
      11,
    };
    // Maximal conversion times of temperature in milliseconds
    uint8_t tempConvTimeMax[4] = {
      50,
      13,
      25,
      7,
    };
    // Typical conversion times of temperature in milliseconds
    uint8_t tempConvTimeTyp[4] = {
      44,
      11,
      22,
      6,
    };
    // Maximal conversion times of humidity in milliseconds
    uint8_t rhumConvTimeMax[4] = {
      16,
      3,
      5,
      8,
    };
    // Maximal conversion times of humidity in milliseconds
    uint8_t rhumConvTimeTyp[4] = {
      14,
      3,
      4,
      7,
    };
  } _resolusion;
  inline bool getUseValuesTyp() { return _status.useValuesTyp; };

  inline uint8_t getConversionTimeTempTyp()
  {
    return _resolusion
      .tempConvTimeTyp[isSuccess(reloadUserRegister()) ? resolution() : 0];
  }
  inline uint8_t getConversionTimeTempMax()
  {
    uint8_t resIdx = resolution();
    return _resolusion
      .tempConvTimeMax[isSuccess(reloadUserRegister()) ? resIdx : 0];
  }
  inline uint8_t getConversionTimeTemp()
  {
    return getUseValuesTyp() ? getConversionTimeTempTyp()
                             : getConversionTimeTempMax();
  }

  inline uint8_t getConversionTimeRhumTyp()
  {
    return _resolusion
      .rhumConvTimeTyp[isSuccess(reloadUserRegister()) ? resolution() : 0];
  }
  inline uint8_t getConversionTimeRhumMax()
  {
    return _resolusion
      .rhumConvTimeMax[isSuccess(reloadUserRegister()) ? resolution() : 0];
  }
  inline uint8_t getConversionTimeRhum()
  {
    return getUseValuesTyp() ? getConversionTimeRhumTyp()
                             : getConversionTimeRhumMax();
  }

  /*
    Validate 3 byte array by provided

    DESCRIPTION:
    The method checks whether provided CRC8 checksum is valid for input byte
    array.
    - The method utilizes CRC8 checksum with polynom x^8+x^5+x^4+1.
    - For 2 LSB items (bytes) of provided array the CRC is calculated and
    compared to the 3rd item.

    PARAMETERS:
    byteArray - Pointer to an array of bytes
      - Data type: pointer
      - Default value: none
      - Limited range: none

    dataBytes - Number of bytes to be checked
      - Data type: non-negative integer
      - Default value: 2
      - Limited range: 0 ~ 255

    RETURN: Flag about correct checksum
  */
  inline bool checkCrc8(uint8_t *byteArray, uint8_t dataBytes = 2)
  {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < dataBytes; i++)
    {
      crc ^= byteArray[i];
      for (int8_t b = 7; b >= 0; b--)
      {
        if (crc & 0x80)
        {
          crc = (crc << 1) ^ 0x31;
        }
        else
        {
          crc = (crc << 1);
        }
      }
    }
    return crc == byteArray[dataBytes];
  }

  /*
    Calculate resolution code from user register byte.

    DESCRIPTION:
    The method calculates resolution code from D7 and D0 bit of the user
    register byte.

    PARAMETERS: none

    RETURN: Resolution code (0 ~ 3)
  */
  inline uint8_t resolution()
  {
    // Separate resolution from RES1 (D7) and RES0 (D0) bit of user register
    uint8_t res0 = (_userReg.value >> 0) & B1;
    uint8_t res1 = (_userReg.value >> 7) & B1;
    return (res1 << 1) | res0;
  }

  /*
    Read electronic serial number.

    DESCRIPTION:
    The method reads all bytes constituting a serial number, checks them with
    CRC codes read from the sensor, and stores it in the class instance object.

    PARAMETERS: none

    RETURN: Result code
  */
  ResultCodes readSerialNumber();

  /*
    Read user register.

    DESCRIPTION:
    The method reads the user register byte and parses relevant data to the
    individual items. Then all of them stores in the class instance object.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes readUserRegister()
  {
    uint8_t data[1];
    if (isError(busReceive(Commands::CMD_REG_RHT_READ, data, 1)))
    {
      return setLastResult(ResultCodes::ERROR_REGISTER);
    }
    _userReg.value = data[0];
    _userReg.read = true;
    return getLastResult();
  }

  /*
    Read user register if needed.

    DESCRIPTION:
    The method reads the user register if internal flag is reset.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes reloadUserRegister()
  {
    return _userReg.read ? getLastResult() : readUserRegister();
  }

  /*
    Write user register.

    DESCRIPTION:
    The method writes the user register byte stored in the class instance object
    to the user register.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes writeUserRegister()
  {
    if (isError(busSend(Commands::CMD_REG_RHT_WRITE, _userReg.value)))
    {
      return getLastResult();
    }
    // Reread the user register the next time for sure
    _userReg.read = false;
    return getLastResult();
  }

  /*
    Set sensor's heater status.

    DESCRIPTION:
    The method sets the heater status bit HTRE (D2) in the user register byte.

    PARAMETERS:
    status - Desired flag about heater enabling
      - Data type: bool
      - Default value: none
      - Limited range: true, false

    RETURN: Result code
  */
  inline ResultCodes setHeaterStatus(bool status)
  {
    if (isError(reloadUserRegister()))
    {
      return getLastResult();
    }
    // Write heater status for HTRE (D2) bit of user register byte if needed
    if (!getHeaterEnabled() && status)
    {
      // Set HTRE to 1
      _userReg.value |= B00000100;
      return writeUserRegister();
    }
    if (getHeaterEnabled() && !status)
    {
      // Set HTRE to 0
      _userReg.value &= B11111011;
      return writeUserRegister();
    }
    return getLastResult();
  }

  /*
    Set sensor's resolution.

    DESCRIPTION:
    The method sets the resolution bits RES1 (D7) and RES0 (D0) in the user
    register byte.

    PARAMETERS:
    bitRes1 - Desired flag about setting upper resolution bit (true == 1, false
    == 0)
      - Data type: bool
      - Default value: none
      - Limited range: true, false

    bitRes0 - Desired flag about setting lower resolution bit (true == 1, false
    == 0)
      - Data type: bool
      - Default value: none
      - Limited range: true, false

    RETURN: Result code
  */
  inline ResultCodes setBitResolution(bool bitRes1, bool bitRes0)
  {
    if (isError(reloadUserRegister()))
    {
      return getLastResult();
    }
    // Determine resolution code
    uint8_t code = ((bitRes1 ? B1 : B0) << 1) | (bitRes0 ? B1 : B0);
    // Write resolution bits RES1 (D7) and RES0 (D0) to user register byte
    if (resolution() != code)
    {
      // Set RES0
      if (bitRes0)
      {
        // Set RES0 to 1
        _userReg.value |= B00000001;
      }
      else
      {
        // Set RES0 to 0
        _userReg.value &= B11111110;
      }
      // Set RES1
      if (bitRes1)
      {
        // Set RES1 to 1
        _userReg.value |= B10000000;
      }
      else
      {
        // Set RES1 to 0
        _userReg.value &= B01111111;
      }
      return writeUserRegister();
    }
    return getLastResult();
  }

  /*
    Calculate temperature.

    DESCRIPTION:
    The method wraps a formula for calculating temperature in centigrade from
    16-bit word.

    PARAMETERS:
    wordMeasure - Measured binary word.
      - Data type: integer
      - Default value: none
      - Limited range: 0x0000 ~ 0xFFFF

    RETURN: Temperature in centigrade.
  */
  inline float calculateTemperature(uint16_t wordMeasure)
  {
    float temperature = static_cast<float>(wordMeasure);
    temperature *= 175.72;
    temperature /= 65536.0;
    temperature -= 46.85;
    return temperature;
  }
  float readTemperature();

  /*
    Calculate relative humidity.

    DESCRIPTION:
    The method wraps a formula for calculating relative humidity in per-cents
    from 16-bit word.

    PARAMETERS:
    wordMeasure - Measured binary word.
      - Data type: integer
      - Default value: none
      - Limited range: 0x0000 ~ 0xFFFF

    RETURN: Relative humidity in per-cents
  */
  inline float calculateHumidity(uint16_t wordMeasure)
  {
    float humidity = static_cast<float>(wordMeasure);
    humidity *= 125.0;
    humidity /= 65536.0;
    humidity -= 6.0;
    return humidity;
  }

  /*
    Sanitized relative humidity.

    DESCRIPTION:
    The method limits the input humidity to a valid range.

    PARAMETERS:
    humidity - Measured humidity.
      - Data type: float
      - Default value: none
      - Limited range: none

    RETURN: Valid relative humidity in per-cents
  */
  inline float sanitizeHumidity(float humidity)
  {
    return constrain(humidity, 0.0, 100.0);
  }
  float readHumidity();
};

#endif
