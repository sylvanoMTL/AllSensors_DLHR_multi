/*

This software is licensed under the Revised (3-clause) BSD license as follows:

Copyright (c) 2018, Jeremy Cole <jeremy@jcole.us>

All rights reserved.

See the LICENSE file for more details.

*/

#include "AllSensors_DLHR.h"

#include <math.h>
#include <util/delay.h>

AllSensors_DLHR::AllSensors_DLHR(TwoWire *bus, SensorType type, SensorResolution pressure_resolution, float pressure_max) :
  pressure_unit(PressureUnit::IN_H2O),
  temperature_unit(TemperatureUnit::CELCIUS)
{
  this->bus = bus;
  this->type = type;
  this->pressure_resolution = pressure_resolution;
  this->pressure_max = pressure_max;

  // Produce bitmasks to mask out undefined bits of both sensors. The pressure sensor's resolution
  // depends on the purchased option (-6, -7, -8 for 16-, 17-, and 18-bit resolution respectively),
  // and the temperature sensor is always 16-bit resolution. Note that the *lower* bits are the unused
  // bits, so that it is the *UPPER* bits that should be kept and the lower ones discarded.
  pressure_resolution_mask    = ~(((uint32_t) 1 << (FULL_SCALE_RESOLUTION - pressure_resolution)) - 1);
  temperature_resolution_mask = ~(((uint32_t) 1 << (FULL_SCALE_RESOLUTION - TEMPERATURE_RESOLUTION)) - 1);

  // Store a few scaling factors depending on the pressure sensor output type.
  switch (type) {
    case GAGE:
      pressure_zero_ref = 0.1 * FULL_SCALE_REF;
      pressure_range = pressure_max;
      break;
    case DIFFERENTIAL:
      pressure_zero_ref = 0.5 * FULL_SCALE_REF;
      pressure_range = pressure_max * 2;
      break;
  }
}

void AllSensors_DLHR::startMeasurement(MeasurementType measurement_type) {
  bus->beginTransmission(I2C_ADDRESS);
  bus->write((uint8_t) measurement_type);
  bus->write(0x00);
  bus->write(0x00);
  bus->endTransmission();
}

uint8_t AllSensors_DLHR::readStatus() {
  bus->requestFrom(I2C_ADDRESS, READ_STATUS_LENGTH);
  status = bus->read();
  bus->endTransmission();

  return status;
}

bool AllSensors_DLHR::readData(bool wait) {
try_again:
  bus->requestFrom(I2C_ADDRESS, (uint8_t) (READ_STATUS_LENGTH + READ_PRESSURE_LENGTH + READ_TEMPERATURE_LENGTH));

  // Read the 8-bit status data.
  status = bus->read();

  if (isError(status)) {
    // An ALU or memory error occurred.
    bus->endTransmission();
    goto error;
  }

  if (isBusy(status)) {
    // The sensor is still busy; either retry or fail.
    bus->endTransmission();

    if (wait) {
      // Wait just a bit so that we don't completely hammer the bus with retries.
      _delay_us(100);
      goto try_again;
    }

    goto error;
  }

  // Read the 24-bit (high 16-18 bits defined) of raw pressure data.
  *((uint8_t *)(&raw_p)+2) = bus->read();
  *((uint8_t *)(&raw_p)+1) = bus->read();
  *((uint8_t *)(&raw_p)+0) = bus->read();

  // Read the 24-bit (high 16 bits defined) of raw temperature data.
  *((uint8_t *)(&raw_t)+2) = bus->read();
  *((uint8_t *)(&raw_t)+1) = bus->read();
  *((uint8_t *)(&raw_t)+0) = bus->read();
  
  bus->endTransmission();

  pressure = convertPressure(transferPressure(raw_p & pressure_resolution_mask));
  temperature = convertTemperature(transferTemperature(raw_t & temperature_resolution_mask));

  return isError(status);

error:
  pressure = NAN;
  temperature = NAN;

  return true;
}



bool AllSensors_DLHR::readDataAsynchro(MeasurementType measurement_type){
   bool dataReady = false;
   
   switch (this->state){
      case State::STATE0:
        Serial.println("start conversion");
        startMeasurement(measurement_type);  //start conversion
        Serial.println("going to State 1");
        this->state = State::STATE1;
        break;

      case State::STATE1:
         // Read the 8-bit status data.
        status = bus->read();
        if (isError(status)) {
          // An ALU or memory error occurred.
          bus->endTransmission();
          goto error;
        }
        if (isBusy(status)) {
          // The sensor is still busy; either retry or fail.
          bus->endTransmission();
        }
        else{
          Serial.println("going to State 2");
          this->state = State::STATE2;
        }
        break;


      case State::STATE2:
          // Read the 24-bit (high 16-18 bits defined) of raw pressure data.
          *((uint8_t *)(&raw_p)+2) = bus->read();
          *((uint8_t *)(&raw_p)+1) = bus->read();
          *((uint8_t *)(&raw_p)+0) = bus->read();

          // Read the 24-bit (high 16 bits defined) of raw temperature data.
          *((uint8_t *)(&raw_t)+2) = bus->read();
          *((uint8_t *)(&raw_t)+1) = bus->read();
          *((uint8_t *)(&raw_t)+0) = bus->read();
          
          bus->endTransmission();

          pressure = convertPressure(transferPressure(raw_p & pressure_resolution_mask));
          temperature = convertTemperature(transferTemperature(raw_t & temperature_resolution_mask));
          dataReady = true;
          Serial.println("going to State 0");
          this->state = State::STATE0;
        break;
   }

   

  error:
    this->pressure = NAN;
    this->temperature = NAN; 
   


  return dataReady;
}