#include "gbj_htu21.h"
const String gbj_htu21::VERSION = "GBJ_HTU21 1.0.0";


uint8_t gbj_htu21::begin(bool holdMasterMode)
{
  if (gbj_twowire::begin()) return getLastResult();
  if (setAddress()) return getLastResult();
  setUseValuesMax();
  setHoldMasterMode(holdMasterMode);
  if (reset()) return getLastResult();
  if (readSerialNumber()) return getLastResult();
  return getLastResult();
}


uint8_t gbj_htu21::reset()
{
  if (busSend(CMD_RESET)) return getLastResult();
  wait(TIMING_RESET);
  // Check user register reset value
  if (readUserRegister()) return getLastResult();
  if (_userReg.value != RESET_REG_USER) return setLastResult(ERROR_RESET);
  if (setHeaterDisabled()) return getLastResult();
  return getLastResult();
}


float gbj_htu21::measureHumidity()
{
  bool origBusStop = getBusStop();
  uint8_t data[3];
  uint16_t wordMeasure;
  for (uint8_t i = 0; i < PARAM_CRC_CHECKS; i++)
  {
    if (getHoldMasterMode())
    {
      setBusRpte();
      if (busSend(CMD_MEASURE_RH_HOLD)) break;
      setBusStopFlag(origBusStop);
      if (busReceive(data, sizeof(data)/sizeof(data[0]))) break;
    }
    else
    {
      uint32_t waitNACK = (getUseValuesTyp() ? getConversionTimeRhumTyp() : getConversionTimeRhumMax());
      setBusRpte();
      if (busSend(CMD_MEASURE_RH_NOHOLD)) break;
      setBusStopFlag(origBusStop);
      while (busReceive(data, sizeof(data)/sizeof(data[0])) == ERROR_RCV_DATA)
      {
        wait(waitNACK);
      };
      if (getLastResult()) break;
    }
    wordMeasure = data[0] << 8;   // MSB
    wordMeasure |= data[1];       // LSB
    // Test status bits (last 2 from LSB) and CRC
    if ((wordMeasure & B11) == B10 && checkCrc8((uint32_t) wordMeasure, data[2])) \
      return calculateHumidity(wordMeasure);
}
  setLastResult(getLastResult() == SUCCESS ? ERROR_MEASURE_RHUM : getLastResult());
  return (float) PARAM_BAD_RHT;
}


float gbj_htu21::measureTemperature()
{
  bool origBusStop = getBusStop();
  uint8_t data[3];
  uint16_t wordMeasure;
  for (uint8_t i = 0; i < PARAM_CRC_CHECKS; i++)
  {
    if (getHoldMasterMode())
    {
      setBusRpte();
      if (busSend(CMD_MEASURE_TEMP_HOLD)) break;
      setBusStopFlag(origBusStop);
      if (busReceive(data, sizeof(data)/sizeof(data[0]))) break;
    }
    else
    {
      setBusRpte();
      uint32_t waitNACK = (getUseValuesTyp() ? getConversionTimeTempTyp() : getConversionTimeTempMax());
      if (busSend(CMD_MEASURE_TEMP_NOHOLD)) break;
      setBusStopFlag(origBusStop);
      while (busReceive(data, sizeof(data)/sizeof(data[0])) == ERROR_RCV_DATA)
      {
        wait(waitNACK);
      };
      if (getLastResult()) break;
    }
    wordMeasure = data[0] << 8;   // MSB
    wordMeasure |= data[1];       // LSB
    // Test status bits (last 2 from LSB)
    if ((wordMeasure & B11) == B00 && checkCrc8((uint32_t) wordMeasure, data[2])) \
      return calculateTemperature(wordMeasure);
  }
  setLastResult(getLastResult() == SUCCESS ? ERROR_MEASURE_TEMP : getLastResult());
  return (float) PARAM_BAD_RHT;
}


//-------------------------------------------------------------------------
// Setters
//-------------------------------------------------------------------------
uint8_t gbj_htu21::setResolution(uint8_t resolution)
{
  initLastResult();
  switch (resolution)
  {
    case RESOLUTION_T11_RH11:
      setResolutionTemp11();
      break;

    case RESOLUTION_T12_RH8:
      setResolutionTemp12();
      break;

    case RESOLUTION_T13_RH10:
      setResolutionTemp13();
      break;

    default:
      setResolutionTemp14();
      break;
  }
  return getLastResult();
}


//------------------------------------------------------------------------------
// Getters
//------------------------------------------------------------------------------
bool gbj_htu21::getVddStatus()
{
  // Always read user register, because the voltage status is updated after
  // each measurement
  if (readUserRegister()) return false;
  uint8_t status = (_userReg.value >> 6) & B1;
  return (status == 0);
}


bool gbj_htu21::getHeaterEnabled()
{
  if (reloadUserRegister()) return false;
  // Separate heater status from HTRE (D2) bit of user register byte
  uint8_t status = (_userReg.value >> 2) & B1;
  return (status == 1);
}


uint64_t gbj_htu21::getSerialNumber()
{
  uint64_t serial;
  serial = _status.serialSNA;
  serial <<= 32;
  serial |= _status.serialSNB;
  serial <<= 16;
  serial |= _status.serialSNC;
  return serial;
}


uint8_t gbj_htu21::getResolutionTemp()
{
  if (reloadUserRegister()) return false;
  return _resolusion.tempBits[resolution()];
}


uint8_t gbj_htu21::getConversionTimeTempMax()
{
  if (reloadUserRegister()) return false;
  return _resolusion.tempConvTimeMax[resolution()];
}


uint8_t gbj_htu21::getConversionTimeTempTyp()
{
  if (reloadUserRegister()) return false;
  return _resolusion.tempConvTimeTyp[resolution()];
}


uint8_t gbj_htu21::getResolutionRhum()
{
  if (reloadUserRegister()) return false;
  return _resolusion.rhumBits[resolution()];
}


uint8_t gbj_htu21::getConversionTimeRhumMax()
{
  if (reloadUserRegister()) return false;
  return _resolusion.rhumConvTimeMax[resolution()];
}


uint8_t gbj_htu21::getConversionTimeRhumTyp()
{
  if (reloadUserRegister()) return false;
  return _resolusion.rhumConvTimeTyp[resolution()];
}


//------------------------------------------------------------------------------
// Auxilliary methods
//------------------------------------------------------------------------------
// Calculate checksum
uint8_t gbj_htu21::crc8(uint32_t data)
{
  uint8_t crc = 0x00;                        // Initialization by datasheet
  for (uint8_t i = 4; i > 0; i--)
  {
    crc ^= (data >> (8 * (i - 1))) & 0xFF;  // Separate byte from MSB to LSB
    for (uint8_t j = 8; j > 0; j--)         // Bitwise division by polynomial
    {
      if (crc & 0x80) crc = (crc << 1) ^ 0x131; // Polynomial x^8+x^5+x^4+x^0 by datasheet
      else crc = (crc << 1);
    }
  }
  return crc;
}


// Validate checksum
bool gbj_htu21::checkCrc8(uint32_t data, uint8_t crc)
{
  return crc == crc8(data);
}


// Calculate resolution code
uint8_t  gbj_htu21::resolution()
{
  // Separate resolution from D7 and D0 bit of user register byte
  uint8_t res0 = (_userReg.value >> 0) & B1;
  uint8_t res1 = (_userReg.value >> 7) & B1;
  return (res1 << 1) | res0;
}


uint8_t gbj_htu21::readSerialNumber()
{
  // Ask for SNB bytes
  {
    uint8_t data[8];
    // Read and validate 4 SNB bytes of serial number including CRCs
    if (busReceive(CMD_READ_SNB, data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_SERIAL_A);
    _status.serialSNB = 0x00000000;
    /* From SNB_3 to SNB_0
      After each SNB byte the CRC byte follows, i.e., there are 4 pairs of
      SNB-CRC bytes. Each SNB byte is CRC checked separately.
    */
    for (uint8_t i = 0; i < (sizeof(data) / sizeof(data[0]) / 2); i++)
    {
      _status.serialSNB <<= 8;
      _status.serialSNB |= data[2*i];
      if (!checkCrc8(data[2*i], data[2*i + 1])) return setLastResult(ERROR_SERIAL_A);
    }
  }
  // Ask for SNC, SNA bytes
  {
    // Read and validate 2 SNC and 2 SNA bytes of serial number including CRC
    uint8_t data[6];
    if (busReceive(CMD_READ_SNAC, data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_SERIAL_B);
    /* From SNC to SNA
      After each pair of SNC and SNA bytes the CRC byte follows, i.e., there are
      2 byte tripples: SNC1-SNC0-CRC, SNA1-SNA0-CRC.
    */
    _status.serialSNC = (data[0] << 8) | data[1];
    if (!checkCrc8(_status.serialSNC, data[2])) return setLastResult(ERROR_SERIAL_B);
    _status.serialSNA = (data[3] << 8) | data[4];
    if (!checkCrc8(_status.serialSNA, data[5])) return setLastResult(ERROR_SERIAL_B);
  }
  return getLastResult();
}


uint8_t gbj_htu21::readUserRegister()
{
  uint8_t data[1];
  if (busReceive(CMD_REG_RHT_READ, data, 1)) return setLastResult(ERROR_REG_RHT_READ);
  _userReg.value = data[0];
  _userReg.read = true;
  return getLastResult();
}


uint8_t gbj_htu21::reloadUserRegister()
{
  if (!_userReg.read)
  {
    return readUserRegister();
  }
   return getLastResult();
}


uint8_t gbj_htu21::writeUserRegister()
{
  // One transmission with command and data
  if (busSend(CMD_REG_RHT_WRITE, _userReg.value)) return getLastResult();
  _userReg.read = false;  // Reread the user register the next time for sure
  return getLastResult();
}


uint8_t gbj_htu21::setHeaterStatus(bool status)
{
  if (reloadUserRegister()) return false;
  // Write heater status for D2 bit of user register byte if needed
  if (!getHeaterEnabled() && status)
  {
    _userReg.value |= B00000100;  // Set HTRE to 1
    return writeUserRegister();
  }
  if (getHeaterEnabled() && !status)
  {
    _userReg.value &= B11111011;  // Set HTRE to 0
    return writeUserRegister();
  }
  return getLastResult();
}


uint8_t gbj_htu21::setBitResolution(bool bitRes1, bool bitRes0)
{
  if (reloadUserRegister()) return getLastResult();
  // Determine resolution code
  uint8_t code = ((bitRes1 ? B1 : B0) << 1) | (bitRes0 ? B1 : B0);
  // Write resolution bits D7 and D0 to user register byte if needed
  if (resolution() != code)
  {
    if (bitRes0)
    {
      _userReg.value |= B00000001;  // Set D0 to 1
    }
    else
    {
      _userReg.value &= B11111110;  // Set D0 to 0
    }
    if (bitRes1)
    {
      _userReg.value |= B10000000;  // Set D7 to 1
    }
    else
    {
      _userReg.value &= B01111111;  // Set D7 to 0
    }
    return writeUserRegister();
  }
  return getLastResult();
}


float gbj_htu21::calculateTemperature(uint16_t wordMeasure)
{
  float temperature = (float) wordMeasure;
  temperature *= 175.72;
  temperature /= 65536.0;
  temperature -= 46.85;
  return temperature;
}


float gbj_htu21::calculateHumidity(uint16_t wordMeasure)
{
  float humidity = (float) wordMeasure;
  humidity *= 125.0;
  humidity /= 65536.0;
  humidity -= 6.0;
  return humidity;
}
