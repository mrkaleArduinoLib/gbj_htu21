# gbjHTU21

Library for the humidity and temperature sensors _HTUD(F)_ with `two-wire` (also known as <abbr title='Inter-Integrated Circuit'>I2C</abbr>) bus interface.
* It is compatible with sensors `SHT21`, `SHT20`, `HDC1080`.
* Sensor address is `0x40` hardcoded and cannot be changed by any library method.
* The library provides measured temperature in degrees of Celsius and relative humidity in percentage.
* For conversion among various temperature unit scales and for calculating dew point temperature use library `gbjAppHelpers`.
* At erroneous measurement of relative humidity or temperature the corresponding method returns erroneous value `255.0`.


#### Particle hardware configuration
* Connect microcontroller's pin `D0` to sensor's pin **SDA** (Serial Data).
* Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).

#### Arduino UNO hardware configuration
* Connect microcontroller's pin `A4` to sensor's pin **SDA** (Serial Data).
* Connect microcontroller's pin `A5` to sensor's pin **SCL** (Serial Clock).

#### Espressif - ESP8266, ESP32 default hardware configuration
* Connect microcontroller's pin `D2` to sensor's pin **SDA** (Serial Data).
* Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).


<a id="dependency"></a>

## Dependency

#### Particle platform
* **Particle.h**: Includes alternative (C++) data type definitions.

#### Arduino platform
* **Arduino.h**: Main include file for the Arduino SDK version greater or equal to 100.
* **inttypes.h**: Integer type conversions. This header file includes the exact-width integer definitions and extends them with additional facilities provided by the implementation.
* **TwoWire**: I2C system library loaded from the file `Wire.h`.

#### Custom Libraries
* **gbjTwoWire**: I2C custom library loaded from the file `gbj_twowire.h`, which provides common bus functionality.


<a id="constants"></a>

## Constants
The library does not have specific error codes. Error codes as well as result code are inherited from the parent library [gbjTwoWire](#dependency) only. The result code and error codes can be tested in the operational code with its method `getLastResult()`, `isError()` or `isSuccess()`.


<a id="interface"></a>

## Interface

#### Main
* [gbj_htu21()](#gbj_htu21)
* [begin()](#begin)
* [reset()](#reset)
* [measureHumidity()](#measureHumidity)
* [measureTemperature()](#measureTemperature)

#### Setters
* [setResolutionTemp14()](#setResolutionTemp)
* [setResolutionTemp13()](#setResolutionTemp)
* [setResolutionTemp12()](#setResolutionTemp)
* [setResolutionTemp11()](#setResolutionTemp)
* [setResolutionRhum12()](#setResolutionRhum)
* [setResolutionRhum11()](#setResolutionRhum)
* [setResolutionRhum10()](#setResolutionRhum)
* [setResolutionRhum8()](#setResolutionRhum)
* [setHeaterEnabled()](#setHeater)
* [setHeaterDisabled()](#setHeater)
* [setHoldMasterMode()](#setHoldMasterMode)
* [setUseValuesTyp()](#setUseValues)
* [setUseValuesMax()](#setUseValues)

#### Getters
* [getResolutionTemp()](#getResolutionTemp)
* [getResolutionRhum()](#getResolutionRhum)
* [getHeaterEnabled()](#getHeaterEnabled)
* [getSNA()](#getSerial)
* [getSNB()](#getSerial)
* [getSNC()](#getSerial)
* [getSerialNumber()](#getSerial)
* [getVddStatus()](#getVddStatus)
* [getHoldMasterMode()](#getHoldMasterMode)
* [getErrorRHT()](#getErrorRHT)

Other possible setters and getters are inherited from the parent library [gbjTwoWire](#dependency) and described there.


<a id="gbj_htu21"></a>

## gbj_htu21()

#### Description
The library does not need special constructor and destructor, so that the inherited ones are good enough and there is no need to define them in the library, just use it with default or specific parameters as defined at constructor of parent library [gbjTwoWire](#dependency).
* Constructor sets parameters specific to the two-wire bus in general.
* All the constructor parameters can be changed dynamically with corresponding setters later in a sketch.

#### Syntax
    gbj_htu21(uint32_t clockSpeed, uint8_t pinSDA, uint8_t pinSCL)

#### Parameters
* **clockSpeed**: Two-wire bus clock frequency in Hertz.
  * *Valid values*:ClockSpeeds::CLOCK\_100KHZ, ClockSpeeds::CLOCK\_400KHZ
  * *Default value*: ClockSpeeds::CLOCK\_100KHZ

* **pinSDA**: Microcontroller's pin for serial data. It is not a board pin but GPIO number. For hardware two-wire bus platforms it is irrelevant and none of methods utilizes this parameter for such as platforms for communication on the bus. On the other hand, for those platforms the parameters might be utilized for storing some specific attribute in the class instance object.
  * *Valid values*: positive integer
  * *Default value*: 4 (GPIO4, D2)

* **pinSCL**: Microcontroller's pin for serial clock. It is not a board pin but GPIO number. For hardware two-wire bus platforms it is irrelevant and none of methods utilizes this parameter for such as platforms. On the other hand, for those platforms the parameters might be utilized for storing some specific attribute in the class instance object.
  * *Valid values*: positive integer
  * *Default value*: 5 (GPIO5, D1)

#### Returns
Object performing the sensor management.
The constructor cannot return [a result or error code](#constants) directly, however, it stores them in the instance object.

#### Example
The method has all arguments defaulted and calling without any parameters is equivalent to the calling with all arguments set by corresponding constant with default value:

```cpp
  gbj_htu21 sensor = gbj_htu21(); // It is equivalent to
  gbj_htu21 sensor = gbj_htu21(sensor.CLOCK_100KHZ, D2, D1)
```

[Back to interface](#interface)


<a id="begin"></a>

## begin()

#### Description
The method takes, sanitizes, and stores sensor parameters to a class instance object and initiates two-wire bus.
* The method sets parameters specific to the sensor itself.
* All the method parameters can be changed dynamically with corresponding [setters](#interface) later in a sketch.

#### Syntax
    ResultCodes begin(bool holdMasterMode)

#### Parameters
* **holdMasterMode**: Logical flag about blocking (holding) serial clock line during measurement. At no holding master mode other communication on the bus can be performed.
  * *Valid values*: true, false
  * *Default value*: true

#### Returns
Some of [result or error codes](#constants).

#### See also
[setHoldMasterMode()](#setHoldMasterMode)

[Back to interface](#interface)


<a id="reset"></a>

## reset()

#### Description
The method resets the sensor and sets control registers to their reset settings values.

#### Syntax
    ResultCodes reset()

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

[Back to interface](#interface)


<a id="measureHumidity"></a>

## measureHumidity()

#### Description
The method is overloaded and measures either relative humidity alongside with temperature at once or the humidity alone.
* The temperature is returned through referenced input parameter.
* If the temperature input parameter is used, the humidity is compensated by the temperature coefficient according to the data sheet.

#### Syntax
    float measureHumidity()
    float measureHumidity(float &temperature)

#### Parameters
* **temperature**: Referenced variable for placing a temperature value in centigrade.
  * *Valid values*: sensor specific
  * *Default value*: none

#### Returns
Relative humidity truncated and compensated to range 0 - 100 °C or erroneous value returned by [getErrorRHT()](#getErrorRHT).

#### Example
``` cpp
gbj_htu21 sensor = gbj_htu21();
float tempValue, rhumValue;
setup()
{
  if (sensor.isSuccess(sensor.begin()))
  {
    rhumValue = sensor.measureHumidity(tempValue);
    rhumValue = sensor.measureHumidity();
  }
}
```

#### See also
[measureTemperature()](#measureTemperature)

[Back to interface](#interface)


<a id="measureTemperature"></a>

## measureTemperature()

#### Description
The method measures temperature.

#### Syntax
    float measureTemperature()

#### Parameters
None

#### Returns
Temperature in centigrade or erroneous value returned by [getErrorRHT()](#getErrorRHT).

#### See also
[measureHumidity()](#measureHumidity)

[Back to interface](#interface)


<a id="setResolutionTemp"></a>

## setResolutionTemp11(), setResolutionTemp12(), setResolutionTemp13(), setResolutionTemp14()

#### Description
The particular method sets the bit resolution for temperature measurement to the value in its name.
The method sets the corresponding bit resolution for the relative humidity measurement at the same time by this relation:

Temperature | Relative Humidity
------ | -------
11 | 11
12 | 8
13 | 10
14 | 12

#### Syntax
    ResultCodes setResolutionTemp11()
    ResultCodes setResolutionTemp12()
    ResultCodes setResolutionTemp13()
    ResultCodes setResolutionTemp14()

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

#### See also
[getResolutionTemp()](#getResolutionTemp)

[Back to interface](#interface)


<a id="getResolutionTemp"></a>

## getResolutionTemp()

#### Description
The method returns the temperature measurement resolution in bits.

#### Syntax
    uint8_t getResolutionTemp()

#### Parameters
None

#### Returns
Bit resolution (11, 12, 13, or 14) or some of [error codes](#errors).

#### See also
[setResolutionTemp11(), setResolutionTemp12(), setResolutionTemp13(), setResolutionTemp14()](#setResolutionTemp)

[Back to interface](#interface)


<a id="setResolutionRhum"></a>

## setResolutionRhum8(), setResolutionRhum10(), setResolutionRhum11(), setResolutionRhum12()

#### Description
The particular method sets the bit resolution for relative humidity measurement to the value in its name.
The method sets the corresponding bit resolution for the temperature measurement at the same time by this relation:

Relative Humidity | Temperature
------ | -------
11 | 11
8 | 12
10 | 13
12 | 14

#### Syntax
    ResultCodes setResolutionRhum8()
    ResultCodes setResolutionRhum10()
    ResultCodes setResolutionRhum11()
    ResultCodes setResolutionRhum12()

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

[Back to interface](#interface)


<a id="getResolutionRhum"></a>

## getResolutionRhum()

#### Description
The method returns the relative humidity measurement resolution in bits.

#### Syntax
    uint8_t getResolutionRhum()

#### Parameters
None

#### Returns
Bit resolution (8, 10, 11, or 12) or some of [error codes](#errors).

#### See also
[setResolutionRhum8(), setResolutionRhum10(), setResolutionRhum11(), setResolutionRhum12()](#setResolutionRhum)

[Back to interface](#interface)


<a id="setHeater"></a>

## setHeaterEnabled(), setHeaterDisabled()

#### Description
The particular method turns on or off a heater built-in in the sensor.

#### Syntax
    ResultCodes setHeaterEnabled()
    ResultCodes setHeaterDisabled()

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

[Back to interface](#interface)


<a id="getHeaterEnabled"></a>

## getHeaterEnabled()

#### Description
The method returns the status of the sensor's heater.

#### Syntax
    bool getHeaterEnabled()

#### Parameters
None

#### Returns
Flag about the heater switched on or off.
* **true**: The heater is on.
* **false**: The heater is off.

#### See also
[setHeaterEnabled()](#setHeater)

[setHeaterDisabled()](#setHeater)

[Back to interface](#interface)


<a id="getSerial"></a>

## getSNA(), getSNB(), getSNC(), getSerialNumber()

#### Description
The particular method returns the corresponding 16-bit or 32-bit part of the serial number and the entire 64-bit serial number of the sensor.

#### Syntax
    uint16_t getSNA()
    uint32_t getSNB()
    uint16_t getSNC()
    uint64_t getSerialNumber()

#### Parameters
None

#### Returns
Upper (<abbr title='Serial Number byte A'>SNA</abbr>), middle (<abbr title='Serial Number byte B'>SNB</abbr>), and lower (<abbr title='Serial Number byte C'>SNC</abbr>) serial simple or double word, or entire serial number.

[Back to interface](#interface)


<a id="getVddStatus"></a>

## getVddStatus()

#### Description
The method returns the status of the supply voltage, which the sensor is powered by.

#### Syntax
    bool getVddStatus()

#### Parameters
None

#### Returns
Flag about the correctness of the operating voltage.
* **true**: The voltage is correct.
* **false**: The voltage is incorrect.

[Back to interface](#interface)


<a id="setHoldMasterMode"></a>

## setHoldMasterMode()

#### Description
The method sets internal flag about particular measuring hold master mode.

#### Syntax
    void setHoldMasterMode(bool holdMasterMode)

#### Parameters
* **holdMasterMode**: See the same parameter in the method [begin()](#begin).

#### Returns
None

#### See also
[begin()](#begin)

[getHoldMasterMode()](#getHoldMasterMode)

[Back to interface](#interface)


<a id="getHoldMasterMode"></a>

## getHoldMasterMode()

#### Description
The method sets internal flag about particular measuring hold master mode.

#### Syntax
    bool getHoldMasterMode()

#### Parameters
None

#### Returns
Current flag about measuring hold master mode.

#### See also
[setHoldMasterMode()](#setHoldMasterMode)

[Back to interface](#interface)


<a id="setUseValues"></a>

## setUseValuesTyp(), setUseValuesMax()

#### Description
The particular method sets the internal flag whether typical or maximal values from the datasheet should be used regarding conversion and reset times.

#### Syntax
    void setUseValuesTyp()
    void setUseValuesMax()

#### Parameters
None

#### Returns
None

[Back to interface](#interface)


<a id="getErrorRHT"></a>

## getErrorRHT()

#### Description
The method returns virtually wrong relative humidity or temperature value at erroneous measurement usually at incorrect CRC from the sensor.

#### Syntax
    float getErrorRHT()

#### Parameters
None

#### Returns
Erroneous relative humidity or temperature.

#### See also
[measureTemperature()](#measureTemperature)

[measureHumidity()](#measureHumidity)

[Back to interface](#interface)
