// Include libraries
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
// Create a servo object
Servo Servo1;


// Declare the Servo pin
int servoPin = 3;
int indice = 0;

void setup() {
  // We need to attach the servo to the used pin number
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
//  while (! Serial) {
//    delay(1);
//  }

//  Serial.println("Adafruit VL53L0X test");
//  if (!lox.begin()) {
//    Serial.println(F("Failed to boot VL53L0X"));
//    while (1);
//  }
  Servo1.attach(servoPin);
}

void loop() {  
  for (indice = 0; indice <= 180; indice = indice + 10) { 
//    VL53L0X_RangingMeasurementData_t measure;
//    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    Servo1.write(indice);
//    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//      Serial.print("Distance (mm): "); Serial.print(measure.RangeMilliMeter); Serial.print(" Angle (°): "); Serial.println(indice);
//    } else {
//      Serial.println(" out of range ");
//    }
    delay(200);
  }
}
