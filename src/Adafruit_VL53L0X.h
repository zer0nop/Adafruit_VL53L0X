/***************************************************
  This is a library for the Adafruit VL53L0X Sensor Breakout

  Designed specifically to work with the VL53L0X sensor from Adafruit
  ----> https://www.adafruit.com/products/3317

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#if ( ARDUINO >= 100 )
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Wire.h"
#include "vl53l0x_api.h"

#define VL53L0X_I2C_ADDR  0x29 // default VL53L0X I2C Address

class Adafruit_VL53L0X
{
  public:
	Adafruit_VL53L0X(int xshutpin = -1);
    boolean       begin( boolean debug = false , uint8_t deviceAddr = VL53L0X_I2C_ADDR);
    VL53L0X_Error 
      rangingTest(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData, 
		  boolean debug = false) 
    { getSingleRangingMeasurement(pRangingMeasurementData, debug); };

    VL53L0X_Error getSingleRangingMeasurement( VL53L0X_RangingMeasurementData_t* pRangingMeasurementData, boolean debug = false );
    void          printRangeStatus( VL53L0X_RangingMeasurementData_t* pRangingMeasurementData );
	
	/**
	* @brief Will return a boolean for whether Single Ranging Measurement was Valid.
	*
	* @param   reading     the sensor reading if valid reading, otherwise -1
	* @param   goodRange   true if status = 0
	* @return  boolean     true if it was a valid reading (status = 0 or status = 2)
	*/
	boolean getValidSingleRangingMeasurement(int &reading, bool &goodRange);

    VL53L0X_Error                     Status      = VL53L0X_ERROR_NONE;
	int XShutPin;

 private:
  VL53L0X_Dev_t                       MyDevice;
  VL53L0X_Dev_t                       *pMyDevice  = &MyDevice;
  VL53L0X_Version_t                   Version;
  VL53L0X_Version_t                   *pVersion   = &Version;
  VL53L0X_DeviceInfo_t                DeviceInfo;
};
