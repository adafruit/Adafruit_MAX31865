/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  Modified by budulinek for everyone.

  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "MAX31865_NonBlocking.h"

// uses hardware SPI, just pass in the CS pin (mandatory) and SPI bus (optional)
// MAX31865 rtd(5, &SPI);
MAX31865 rtd(5);

// Resistance of the reference resistor. Check Rref resistor value on your module,
// should be 430 for PT100 and 4300 for PT1000
#define RREF 4300
// Nominal resistance of the RTD sensor at 0°C,
// use 100 for PT100, 1000 for PT1000
#define RNOMINAL 1000

// state machine
byte state;

// Timer class for non-blocking delays
class Timer {
private:
  uint32_t timestampLastHitMs;
  uint32_t sleepTimeMs;
public:
  boolean isOver();
  void sleep(uint32_t sleepTimeMs);
};
boolean Timer::isOver() {
  if ((uint32_t)(millis() - timestampLastHitMs) > sleepTimeMs) {
    return true;
  }
  return false;
}
void Timer::sleep(uint32_t sleepTimeMs) {
  this->sleepTimeMs = sleepTimeMs;
  timestampLastHitMs = millis();
}
Timer rtdTimer;

void setup() {
  Serial.begin(115200);
  Serial.println("MAX31865 PT1000 Sensor Test!");

  rtd.begin(MAX31865::RTD_3WIRE, MAX31865::FILTER_50HZ);
}

void loop() {
  // Read and print temperature and faults (non-blocking)
  readRtd();

  // Place your other functions here...

}

void readRtd() {
  if (rtdTimer.isOver() == false) {
    return;
  }
  switch (state) {
    case 0:
      {
        rtd.enableBias(true);  // enable bias voltage
        rtdTimer.sleep(10);    // and wait 10 ms
        state++;
      }
      break;
    case 1:
      {
        rtd.singleConvert();  // trigger single resistance conversion
        rtdTimer.sleep(65);   // and wait 65 ms
        state++;
      }
      break;
    case 2:
      {
        uint16_t rtdVal = rtd.getRTD();

        Serial.print("RTD value: ");
        Serial.println(rtdVal);
        float ratio = rtdVal;
        ratio /= 32768;
        Serial.print("Ratio = ");
        Serial.println(ratio, 8);
        Serial.print("Resistance = ");
        Serial.println(RREF * ratio, 8);
        Serial.print("Temperature = ");
        Serial.println(rtd.getTemp(RNOMINAL, RREF));

        // Check and print any faults
        uint8_t fault = rtd.getFault();
        if (fault) {
          Serial.print("Fault 0x");
          Serial.println(fault, HEX);
          if (fault & MAX31865::FAULT_HIGHTHRESH_BIT) {
            Serial.println("RTD High Threshold");
          }
          if (fault & MAX31865::FAULT_LOWTHRESH_BIT) {
            Serial.println("RTD Low Threshold");
          }
          if (fault & MAX31865::FAULT_REFINLOW_BIT) {
            Serial.println("REFIN- > 0.85 x Bias");
          }
          if (fault & MAX31865::FAULT_REFINHIGH_BIT) {
            Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
          }
          if (fault & MAX31865::FAULT_RTDINLOW_BIT) {
            Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
          }
          if (fault & MAX31865::FAULT_OVUV_BIT) {
            Serial.println("Under/Over voltage");
          }
          rtd.clearFault();
        }
        Serial.println();

        rtd.enableBias(false);  // disable bias voltage
        rtdTimer.sleep(1000);
        state = 0;
      }
      break;
    default:
      break;
  }
}
