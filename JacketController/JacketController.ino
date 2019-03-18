/*
 *  JacketController firmware that runs on the ESP32 and manages all interactions
 *  with sensors and manages the CPX over serial.
 *  
 *  Started: March 2019
 */

#include "ArduinoJson.h"
#include <Wire.h>
#include "Adafruit_DRV2605.h"

HardwareSerial Serial3(2);
Adafruit_DRV2605 hapticMotor;               // driver for the rumble motors
const int PulseSensorPin = 13;               // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
const int HeartSignalThreshold = 550;       // Determine which Signal to "count as a beat", and which to ingore.
const int UART_RX2_PIN = 16;
const int UART_TX2_PIN = 17;
const int LED_PIN = 2;

void setup()
{
  // set up the serial connections
  Serial.begin(115200);       // USB println debugging
  Serial3.begin(9600, SERIAL_8N1, UART_RX2_PIN, UART_TX2_PIN);    // HW UART for talking with CPX
  Serial3.flush();

  // set up the haptic driver
  hapticMotor.begin();
  hapticMotor.selectLibrary(1);
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  hapticMotor.setMode(DRV2605_MODE_INTTRIG); 

  Serial.println("Setup done");
}

void loop()
{
  // talk to circuit playground express
  exchangePlaygroundData();
  
  // get heart rate, returns a value 0-1023
  int heartSignal = getHeartSignal();

  if (heartSignal > HeartSignalThreshold) {
    // play bone conductor audio at certain thresholds
    playBoneAudio(heartSignal);
    
    // play rumble motors
    playRumbleMotors(heartSignal);
  }
  delay(1000);
}

/*
 * Checks to see if the Circuit Playground Express has sent any data and  
 * parses it
 */
void exchangePlaygroundData() {
  if (Serial3.available() > 0) {
    //Serial.print("bytes available: ");
    //Serial.println(Serial3.available());
    parseDataBlock();
  }
}

/*
 * Get the heart signal and pulse the led accordingly
 */
int getHeartSignal() {
  int heartSignal = analogRead(PulseSensorPin);  // read the PulseSensor's value.
  if(heartSignal > HeartSignalThreshold){                          // if the signal is above "550", then "turn-on" Arduino's on-Board LED.
     digitalWrite(LED_PIN,HIGH);
   } else {
     digitalWrite(LED_PIN,LOW);
   }
   return heartSignal;
}

/*
 * Plays audio to the bone conductor via a TPA2012 amplifier
 */
void playBoneAudio(int heartSignal) {
  // TODO play audio
}

/*
 *  Plays an effect chain on the rumble motors
 */
void playRumbleMotors(int heartSignal) {
  // set the effect to play
  // see Examples->Adafruit 2605 Library->basi.ino for the full list of of the 117 effects
  // supported by the driver
  int effect = 1;   // 1 − Strong Click - 100%

  hapticMotor.setWaveform(0, effect);  // play effect 
  hapticMotor.setWaveform(1, 0);       // end waveform

  // play the effect!
  hapticMotor.go();
}

/*
 * Parses the data from the CPX
 * TODO: decide what to do with the data
 */
boolean parseDataBlock() {
  // NOTE: capacity is specific to example
  // calculated with https://arduinojson.org/v6/assistant/
  // and the payload from the worker
  const size_t capacity = 2*JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + 90;
  DynamicJsonDocument doc(capacity);
  String json = Serial3.readString();
  //Serial.print(json);   // dump for verification

  DeserializationError err = deserializeJson(doc, json);
  if (!err) {
    // grab data from JSON doc
    const char* name = doc["name"]; // "JacketWorker"
    long time = doc["time"];
    
    JsonObject data = doc["data"];
    int data_lightLevel = data["lightLevel"];
    float data_temperature = data["temperature"];
    float data_soundLevel = data["soundLevel"];
    
    JsonObject data_motion = data["motion"];
    float data_motion_x = data_motion["x"];
    float data_motion_y = data_motion["y"];
    float data_motion_z = data_motion["z"]; 

    // TODO something with this data

    serializeJson(doc, Serial);     // forward to USB serial
    return true;
  }

  Serial.print(F("deserializeJson() failed with code "));
  Serial.println(err.c_str());
  return false;
}
