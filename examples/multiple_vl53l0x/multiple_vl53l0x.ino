#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include <Servo.h>

#define XSHUT_pin2 6
#define XSHUT_pin1 5

#define Sensor1_newAddress 41
#define Sensor2_newAddress 42

Adafruit_VL53L0X Sensor1 = Adafruit_VL53L0X(XSHUT_pin1);
Adafruit_VL53L0X Sensor2 = Adafruit_VL53L0X(XSHUT_pin2);

Servo myservo;

int pos = 0;
void setup()
{
  myservo.attach(23);

  Serial.begin(115200);
  Wire.begin();

  Sensor2.begin(false, Sensor2_newAddress);
  Sensor1.begin(false, Sensor1_newAddress);
}

void loop()
{
  int sensor1 = -1; // sensor reading, -1 if it is not valid
  bool goodReading1; // goodReading if it is accurate(confident)
  bool validReading1 = Sensor1.getValidSingleRangingMeasurement(sensor1, goodReading1); // valid reading? not valid if out of range or error

  int sensor2 = -1;
  bool goodReading2;
  bool validReading2 = Sensor2.getValidSingleRangingMeasurement(sensor2, goodReading2);


  // Open the Serial Monitor/Plotter to see the output
  Serial.print(goodReading1 ? sensor1 : -1);

  Serial.print(",");

  Serial.print(goodReading2 ? sensor2 : -1);

  Serial.print(",");

  Serial.print(!goodReading1 ? sensor1 : -1);

  Serial.print(",");

  Serial.print(!goodReading2 ? sensor2 : -1);

  Serial.println();

  if (sensor2 > 0 && sensor2 < 200) {
    myservo.write(100);
  }
  else {
    myservo.write(160);
  }

}
