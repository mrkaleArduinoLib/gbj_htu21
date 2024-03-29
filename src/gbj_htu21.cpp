#include "gbj_htu21.h"

float gbj_htu21::readTemperature()
{
  uint8_t data[3];
  for (uint8_t i = 0; i < Params::PARAM_CRC_CHECKS; i++)
  {
    if (getHoldMasterMode())
    {
      setDelayReceive(getConversionTimeTempMax());
      if (isError(busReceive(Commands::CMD_MEASURE_TEMP_HOLD,
                             data,
                             sizeof(data) / sizeof(data[0]))))
      {
        break;
      }
    }
    else
    {
      setDelayReceive(0);
      while (busReceive(Commands::CMD_MEASURE_TEMP_NOHOLD,
                        data,
                        sizeof(data) / sizeof(data[0])) ==
             ResultCodes::ERROR_RCV_DATA)
      {
        wait(getConversionTimeTemp());
      };
      if (isError())
      {
        break;
      }
    }
    // Test status bits (last 2 from LSB) and CRC, calculate without status bits
    if ((data[1] & B11) == B00 && checkCrc8(data))
    {
      return calculateTemperature((data[0] << 8) | (data[1] & 0xFC));
    }
  }
  setLastResult(isSuccess() ? ResultCodes::ERROR_MEASURE : getLastResult());
  return getErrorRHT();
}

float gbj_htu21::readHumidity()
{
  uint8_t data[3];
  for (uint8_t i = 0; i < Params::PARAM_CRC_CHECKS; i++)
  {
    if (getHoldMasterMode())
    {
      setDelayReceive(getConversionTimeRhumMax());
      if (isError(busReceive(Commands::CMD_MEASURE_RH_HOLD,
                             data,
                             sizeof(data) / sizeof(data[0]))))
      {
        break;
      }
    }
    else
    {
      setDelayReceive(0);
      while (busReceive(Commands::CMD_MEASURE_RH_NOHOLD,
                        data,
                        sizeof(data) / sizeof(data[0])) ==
             ResultCodes::ERROR_RCV_DATA)
      {
        wait(getConversionTimeRhum());
      };
      if (isError())
      {
        break;
      }
    }
    // Test status bits (last 2 from LSB) and CRC, calculate without status bits
    if ((data[1] & B11) == B10 && checkCrc8(data))
    {
      return calculateHumidity((data[0] << 8) | (data[1] & 0xFC));
    }
  }
  setLastResult(isSuccess() ? ResultCodes::ERROR_MEASURE : getLastResult());
  return getErrorRHT();
}

gbj_htu21::ResultCodes gbj_htu21::readSerialNumber()
{
  setDelayReceive(0);
  // Ask for SNB bytes
  {
    uint8_t data[8];
    // Read and validate 4 SNB bytes of serial number including CRCs
    if (isError(busReceive(
          Commands::CMD_READ_SNB, data, sizeof(data) / sizeof(data[0]))))
    {
      return setLastResult(ResultCodes::ERROR_SN);
    }
    status_.serialSNB = 0x00000000;
    /* From SNB_3 to SNB_0.
      After each SNB byte the CRC byte follows, i.e., there are 4 pairs of
      SNB-CRC bytes. Each SNB byte is CRC checked separately.
    */
    for (uint8_t i = 0; i < (sizeof(data) / sizeof(data[0]) / 2); i++)
    {
      status_.serialSNB <<= 8;
      status_.serialSNB |= data[2 * i];
      if (!checkCrc8(&data[2 * i], 1))
      {
        return setLastResult(ResultCodes::ERROR_SN);
      }
    }
  }
  // Ask for SNC, SNA bytes
  {
    // Read and validate 2 SNC and 2 SNA bytes of serial number including CRC
    uint8_t data[6];
    if (isError(busReceive(
          Commands::CMD_READ_SNAC, data, sizeof(data) / sizeof(data[0]))))
    {
      return setLastResult(ResultCodes::ERROR_SN);
    }
    /* From SNC to SNA
      After each pair of SNC and SNA bytes the CRC byte follows, i.e., there
      are 2 byte tripples: SNC1-SNC0-CRC, SNA1-SNA0-CRC.
    */
    status_.serialSNC = (data[0] << 8) | data[1];
    if (!checkCrc8(data))
    {
      return setLastResult(ResultCodes::ERROR_SN);
    }
    status_.serialSNA = (data[3] << 8) | data[4];
    if (!checkCrc8(&data[3]))
    {
      return setLastResult(ResultCodes::ERROR_SN);
    }
  }
  return getLastResult();
}
