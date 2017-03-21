#include "Adafruit_VL53L0X.h"

#define VERSION_REQUIRED_MAJOR  1
#define VERSION_REQUIRED_MINOR  0
#define VERSION_REQUIRED_BUILD  1

#define STR_HELPER( x ) #x
#define STR( x )        STR_HELPER(x)

Adafruit_VL53L0X::Adafruit_VL53L0X(int xshutpin) {
  XShutPin = xshutpin; // set the xShut pin
  if (XShutPin != -1) {
    pinMode(XShutPin, OUTPUT); // turn off the sensor device
  }
}

boolean Adafruit_VL53L0X::begin( boolean debug , uint8_t deviceAddr) {
  if (XShutPin != -1) {
	pinMode(XShutPin, INPUT); // turn on the sensor
	delay(10);// give it a little time to start
  }
  int32_t   status_int;
  int32_t   init_done         = 0;

  uint32_t  refSpadCount;
  uint8_t   isApertureSpads;
  uint8_t   VhvSettings;
  uint8_t   PhaseCal;

  // Initialize Comms
  pMyDevice->I2cDevAddr      =  VL53L0X_I2C_ADDR;  // 7 bit addr
  pMyDevice->comms_type      =  1;
  pMyDevice->comms_speed_khz =  400;

  if (deviceAddr != VL53L0X_I2C_ADDR) {
	  VL53L0X_SetDeviceAddress(pMyDevice, deviceAddr);
	  pMyDevice->I2cDevAddr = deviceAddr;
  }

  //Wire.begin();     // VL53L0X_i2c_init();
  
  // unclear if this is even needed:
  if( VL53L0X_IMPLEMENTATION_VER_MAJOR != VERSION_REQUIRED_MAJOR ||
      VL53L0X_IMPLEMENTATION_VER_MINOR != VERSION_REQUIRED_MINOR ||
      VL53L0X_IMPLEMENTATION_VER_SUB != VERSION_REQUIRED_BUILD )  {
      if( debug ) {
          Serial.println( F( "Found " STR(VL53L0X_IMPLEMENTATION_VER_MAJOR) "." STR(VL53L0X_IMPLEMENTATION_VER_MINOR) "."  STR(VL53L0X_IMPLEMENTATION_VER_SUB) " rev " STR(VL53L0X_IMPLEMENTATION_VER_REVISION) ) );
          Serial.println( F( "Requires " STR(VERSION_REQUIRED_MAJOR) "." STR(VERSION_REQUIRED_MINOR) "." STR(VERSION_REQUIRED_BUILD) ) );
      }

      Status = VL53L0X_ERROR_NOT_SUPPORTED;

      return false;
  }

  Status = VL53L0X_DataInit( &MyDevice );         // Data initialization

  Status = VL53L0X_GetDeviceInfo( &MyDevice, &DeviceInfo );

  if( Status == VL53L0X_ERROR_NONE )  {
      if( debug ) {
         Serial.println( F( "VL53L0X Info:" ) );
         Serial.print( F( "Device Name: ")  ); Serial.print( DeviceInfo.Name );
         Serial.print( F( ", Type: " ) ); Serial.print( DeviceInfo.Type );
         Serial.print( F( ", ID: " ) ); Serial.println( DeviceInfo.ProductId );

         Serial.print( F( "Rev Major: " ) ); Serial.print( DeviceInfo.ProductRevisionMajor );
         Serial.print( F( ", Minor: " ) ); Serial.println( DeviceInfo.ProductRevisionMinor );
      }

      if( ( DeviceInfo.ProductRevisionMinor != 1 ) && ( DeviceInfo.ProductRevisionMinor != 1 ) ) {
          if( debug ) {
              Serial.print( F( "Error expected cut 1.1 but found " ) );
              Serial.print( DeviceInfo.ProductRevisionMajor );
              Serial.print( ',' );
              Serial.println( DeviceInfo.ProductRevisionMinor );
          }

          Status = VL53L0X_ERROR_NOT_SUPPORTED;
      }
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      if( debug ) {
          Serial.println( F( "VL53L0X: StaticInit" ) );
      }

      Status = VL53L0X_StaticInit( pMyDevice ); // Device Initialization
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      if( debug ) {
          Serial.println( F( "VL53L0X: PerformRefSpadManagement" ) );
      }

      Status = VL53L0X_PerformRefSpadManagement( pMyDevice, &refSpadCount, &isApertureSpads ); // Device Initialization

      if( debug ) {
          Serial.print( F( "refSpadCount = " ) );
          Serial.print( refSpadCount );
          Serial.print( F( ", isApertureSpads = " ) );
          Serial.println( isApertureSpads );
      }
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      if( debug ) {
          Serial.println( F( "VL53L0X: PerformRefCalibration" ) );
      }

      Status = VL53L0X_PerformRefCalibration( pMyDevice, &VhvSettings, &PhaseCal );           // Device Initialization
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
      if( debug ) {
          Serial.println( F( "VL53L0X: SetDeviceMode" ) );
      }

      Status = VL53L0X_SetDeviceMode( pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING );        // Setup in single ranging mode
  }

  // Enable/Disable Sigma and Signal check
  if( Status == VL53L0X_ERROR_NONE ) {
      Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1 );
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      Status = VL53L0X_SetLimitCheckValue( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)( 1.5 * 0.023 * 65536 ) );
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      return true;
  } else {
      if( debug ) {
          Serial.print( F( "VL53L0X Error: " ) );
          Serial.println( Status );
      }

      return false;
  }
}



VL53L0X_Error Adafruit_VL53L0X::getSingleRangingMeasurement( VL53L0X_RangingMeasurementData_t *RangingMeasurementData, boolean debug )
{
    VL53L0X_Error   Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t  LimitCheckCurrent;


    /*
     *  Step  4 : Test ranging mode
     */

    if( Status == VL53L0X_ERROR_NONE ) {
        if( debug ) {
            Serial.println( F( "sVL53L0X: PerformSingleRangingMeasurement" ) );
        }
        Status = VL53L0X_PerformSingleRangingMeasurement( pMyDevice, RangingMeasurementData );

        if( debug ) {
            printRangeStatus( RangingMeasurementData );
        }

        if( debug ) {
            VL53L0X_GetLimitCheckCurrent( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent );

          	Serial.print( F( "RANGE IGNORE THRESHOLD: " ) );
          	Serial.println( (float)LimitCheckCurrent / 65536.0 );

          	Serial.print( F( "Measured distance: " ) );
          	Serial.println( RangingMeasurementData->RangeMilliMeter );
        }
    }

    return Status;
}




void Adafruit_VL53L0X::printRangeStatus( VL53L0X_RangingMeasurementData_t* pRangingMeasurementData )
{
    char buf[ VL53L0X_MAX_STRING_LENGTH ];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString( RangeStatus, buf );

    Serial.print( F("Range Status: " ) );
    Serial.print( RangeStatus );
    Serial.print( F( " : " ) );
    Serial.println( buf );

}


boolean Adafruit_VL53L0X::getValidSingleRangingMeasurement(int &reading, bool &goodRange) {

	VL53L0X_RangingMeasurementData_t measure;
	rangingTest(&measure, false);
	reading = measure.RangeMilliMeter;
	if (measure.RangeStatus == 0 ) { // Valid!
		goodRange = true;
		return true;
	}
	else if (measure.RangeStatus == 1) { 
		// Sigma fail will trigger particularly in ambient light, when the amount of ambient
		// light is adding too much noise onto the ranging measurement.
		goodRange = false;
		return true;
	}
	else if (measure.RangeStatus == 2) {
		// Signal fail will trigger when the return signal is too low to give enough
		// confidence on the range measured.The limit will be given by either the signal
		// limit or the RIT(Range Ignore Threshold).
		goodRange = false;
		return true;
	}
	else {
		reading = -1;
		return false;
	}
}
