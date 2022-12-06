#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lidar = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  lidar.begin();
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lidar.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  }
  else {
    Serial.println(" out of range ");
  }
    
  delay(100);
}
