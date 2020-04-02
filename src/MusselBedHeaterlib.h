//
//  MusselBedHeaterlib.h
//  
//
//  Created by Luke Miller on 3/30/20.
//
//

#ifndef MusselBedHeaterlib_H
#define MusselBedHeaterlib_H

#include <Arduino.h> // to get access to pinMode, digitalRead etc functions
#include "SdFat.h"	// https://github.com/greiman/SdFat
#include <SPI.h>
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include <OneWire.h>  // For MAX31820 temperature sensor https://github.com/PaulStoffregen/OneWire
#include <DallasTemperature.h> // For MAX31820 sensors https://github.com/milesburton/Arduino-Temperature-Control-Library

// Various additional libraries for access to sleep mode functions
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include <avr/wdt.h>
#include <math.h>


class ADG725 {
public:
    ADG725();
    ~ADG725();
    void begin(uint8_t CS_MUX, uint32_t SPI_SPEED);
    void begin();
    void setADG725channel(uint8_t ADGchannel);
    
private:
    uint8_t m_CS_MUX;
    uint32_t m_SPI_SPEED;
};

//--------- Public functions
    /**
      Constructor

      @param max31820 - a OneWire object 
      @param refSensors - a DallasTemperature object representing the MAX31820 sensors - linked to the max31820 OneWire object
      @param numRefSensors - number of MAX31820 sensors found, defined as a global variable in the calling program
      @param sensorAddr - an array, [numRefSensors] rows by 8 bytes wide, defined as a global variable in the calling program
    */
// Get addresses for all available OneWire devices
void getRefSensorAddresses(OneWire& max31820, 
							DallasTemperature& refSensors, 
							uint8_t numRefSensors, 
							uint8_t sensorAddr[][8]);
// Version with user-specified temperature precision
void getRefSensorAddresses(OneWire& max31820, 
						DallasTemperature& refSensors, 
						uint8_t numRefSensors, 
						uint8_t sensorAddr[][8], 
						uint8_t TEMPERATURE_PRECISION);

// Print formatted Date + Time to Serial monitor
void printTimeSerial(DateTime now);

// Print formatted Date + Time to SD card csv file. Notice that this passes the
// SdFile object by reference (SdFile& mylogfile) instead of making a copy and
// passing by value (which SdFile mylogfile would do).
void printTimeToSD(SdFile& mylogfile, DateTime now);

// Initialize a new output csv file. Note that this writes a header row
// to the file, so you may want to tweak the column labels in this function.
void initFileName(SdFat& sd, SdFile& logfile, DateTime time1, char *filename, bool serialValid, char *serialNumber);

// Start the TIMER2 timer, using a 32.768kHz input from a DS3231M
// real time clock as the signal.
DateTime startTIMER2(DateTime currTime, RTC_DS3231& rtc, byte SPS);

// Put the AVR to sleep until a TIMER2 interrupt fires to awaken it
void goToSleep();

// Function to read supply battery voltage
float readBatteryVoltage(byte BATT_MONITOR_EN, byte BATT_MONITOR, float dividerRatio, float refVoltage);

#endif /* MusselBedHeaterlib_H */
