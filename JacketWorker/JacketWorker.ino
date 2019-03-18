/*
 * JacketWorker firmware that runs on the Circuit Playground Express from Adafruit
 * and sends sensor data to the ESP32 over serial.
 * 
 * Started: March 2019
 */
#include <Adafruit_CircuitPlayground.h>
#include "ArduinoJson.h"

void setup() {
  /* NOTE: Call CircuitPlayground.begin() prior to any other *.begin()'s. Otherwise it
   *  reassigns the pins and you will wonder why serial io isn't working.
   */
  CircuitPlayground.begin();
  Serial1.begin(9600);      // on CPX, Serial1 maps to the hw UART
  delay(100);
}

void loop() {
  constructDataBlock();
  delay(1000);
}

/*
 * Construct and send the sensor data block in JSON format over Serial1
 */
void constructDataBlock() {
  const size_t capacity = 2*JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4);
  DynamicJsonDocument doc(capacity);
  
  doc["name"] = "JacketWorker";
  doc["time"] = 1351824120;
  
  JsonObject data = doc.createNestedObject("data");
  data["lightLevel"] = CircuitPlayground.lightSensor();
  data["temperature"] = CircuitPlayground.temperature();
  data["soundLevel"] = CircuitPlayground.mic.soundPressureLevel(10);
  
  JsonObject data_motion = data.createNestedObject("motion");
  data_motion["x"] = CircuitPlayground.motionX();
  data_motion["y"] = CircuitPlayground.motionY();
  data_motion["z"] = CircuitPlayground.motionZ();
  
  serializeJson(doc, Serial1); // push out over the HW Serial1 port
}
  
