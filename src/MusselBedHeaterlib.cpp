//
//  MusselBedHeaterlib.cpp
//  
//
//  Created by Luke Miller on 3/30/20.
//
//

#include "MusselBedHeaterlib.h"


ADG725::ADG725(){}
ADG725::~ADG725(){}

void ADG725::begin(uint8_t CS_MUX, uint32_t SPI_SPEED){
    m_CS_MUX = CS_MUX;
    m_SPI_SPEED = SPI_SPEED;
    pinMode(m_CS_MUX, OUTPUT);
}

void ADG725::begin(){
    m_CS_MUX = 7; // Default chip select pin for MusselBedHeater Rev C hardware
    m_SPI_SPEED = 4000000L; // 4MHz seems to work on 8MHz internal oscillator
    pinMode(m_CS_MUX, OUTPUT);
}

//************ Function to set input/output channel pair on ADG725 multiplexer
/*	@param ADGchannel A byte value from 0 to 15 denoting which channel should
	be enabled on the ADG725. Enables both the A and B sides of the multiplexer
*/
void ADG725::setADG725channel(uint8_t ADGchannel) {
    // Reset the SPI bus settings and mode
    SPI.beginTransaction(SPISettings(m_SPI_SPEED, MSBFIRST, SPI_MODE1));
    // Activate the ADG725 SYNC line (CS_MUX)
    digitalWrite(m_CS_MUX, LOW); // activate ADG725 SYNC line by pulling low
    // Transfer the new channel address for the ADG725 as an 8-bit value
    SPI.transfer(ADGchannel); // Send 8-bit byte to set addresses and EN line
    // Deactivate the ADG725 SYNC line
    digitalWrite(m_CS_MUX, HIGH); // pull high to stop sending data to ADG725
    // End the transaction, release the SPI bus
    SPI.endTransaction();
}
//*************************************************************************
// Disable all channels on the ADG725, effectively shutting it off
void ADG725::disableADG725(void){
	// Reset the SPI bus settings and mode
    SPI.beginTransaction(SPISettings(m_SPI_SPEED, MSBFIRST, SPI_MODE1));
    // Activate the ADG725 SYNC line (CS_MUX)
    digitalWrite(m_CS_MUX, LOW); // activate ADG725 SYNC line by pulling low
    // Transfer hex value for the ADG725 to disable all channels
    SPI.transfer(0x80); // Send 0x80 byte to pull EN line high, disables all
    // Deactivate the ADG725 SYNC line
    digitalWrite(m_CS_MUX, HIGH); // pull high to stop sending data to ADG725
    // End the transaction, release the SPI bus
    SPI.endTransaction();
}
//*************************************************************************



//************** PID function *********************************************
// This is a stripped down version of the PID_v1 library from
// https://github.com/br3ttb/Arduino-PID-Library/ by Brett Beauregard
// Designed to save some memory space because we're using the same sort of
// heating elements on all mussels, so the tuning parameters can be shared
// It also eliminates some of the flexibility of the original library in 
// order to save memory.

PID::PID(){}
// De-constructor function, for completeness. 
PID::~PID(){}

void PID::begin(double* kp, 
				double* ki, 
				double* kd, 
				int pidSampleTime,
				bool Pon)
{
   double SampleTimeInSec = ((double)pidSampleTime)/1000;
   // Scale the ki and kd value from milliseconds to seconds, leave kp unchanged
   // This is working via the pointers to the ki, kd values, and re-writing the 
   // values in the global variable versions of ki, kd (kp is left unchanged)
   *ki =  *ki * SampleTimeInSec;
   *kd = *kd / SampleTimeInSec;
   PonE = Pon;
}

/* Compute() **********************************************************************
 *   This function should be called every time "void loop()" executes.  
 *   The function will decide for itself whether a new
 *   pid Output needs to be computed.  Returns true when the output is computed,
 *   false when nothing has been done.
 *
 *	Necessary global variables:
 *	@param pidInput - float value (temperature)
 *  @param pidSetpoint - float value (target temperature)
 *  @param pidOutput - float value (PWM setting, 0-4095)
 *  @param kp - proportional tuning parameter
 *  @param ki - integrative tuning parameter
 *  @param kd - derivative tuning parameter
 *  @param pidSampleTime - milliseconds
 *  @param lastTime - milliseconds value from previous round of calcs
 *
 **********************************************************************************/
bool PID::Compute(double pidInput[], 
					double pidOutput[], 
					double pidOutputSum[], 
					double pidLastInput[], 
					double pidSetpoint, 
					unsigned int pidSampleTime,
					unsigned long lastTime,
					double kp,
					double ki,
					double kd,
					uint8_t NUM_THERMISTORS,
					bool deadband)
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange >= pidSampleTime)
   {
	   for (byte i = 0; i < NUM_THERMISTORS; i++){
		   /*Compute all the working error variables*/
		   double input = pidInput[i]; // Input temperature
		   double error = pidSetpoint - input; 
		   double dInput = (input - pidLastInput[i]);
		   // If input temperature is reasonable (higher than -20)
		   if ( (input > -20) & (input < 60) ){
			   pidLastInput[i] = input;
			   pidOutputSum[i] += (ki * error);
			   /*If using Proportional on Measurement, adjust output sum*/
			   if (!PonE) pidOutputSum[i] -= kp * dInput;
			   
			   if(pidOutputSum[i] > 4095) pidOutputSum[i] = 4095; // Hardcoded max for 16-bit PWM
			   else if(pidOutputSum[i] < 0) pidOutputSum[i] = 0; // Hardcoded minimum
			   /*If using Proportional on Error, add error to Output term*/
			   if (PonE) {
				   pidOutput[i] = kp * error;
			   } else { 
				pidOutput[i] = 0; /*If using Proportional on Measurement, reset output term to 0 */
			   } 
			   
			   
			   /*Compute Rest of PID Output*/
			   pidOutput[i] += pidOutputSum[i] - (kd * dInput);
			   if (pidOutput[i] > 4095) pidOutput[i] = 4095; // Hardcoded max for 16-bit PWM
			   else if (pidOutput[i] < 0) pidOutput[i] = 0;	 // Hardcoded minimum	
			   if (deadband){
				   // If deadband argument is true, set output to zero whenever
				   // the input temperature is substantially larger than the 
				   // pidSetpoint temperature
					if (error < -0.25) {
						pidOutput[i] = 0; 
						pidOutputSum[i] = 0;
						pidLastInput[i] = input;
					}   
			   }
			   
		   } else {
			   // If input temperature is out of bounds, zero everything out
			   pidLastInput[i] = input;
			   pidOutputSum[i] = 0; // zero this out
			   pidOutput[i] = 0; // zero this out, turns off heater
		   }
		   

	   }

      lastTime = now; // update lastTime since we did calculations
	  return true;
   }
   else return false; // not enough time elapsed, no calculation done this time
}

/**********************************************
* resetPID - when heaters are off, reset PID output, output sum, and last time
* values, so that when the PID routine restarts later on it has a fresh start.
*/
void PID::resetPID(double pidOutput[], 
					double pidOutputSum[],
					unsigned long lastTime,
					uint8_t NUM_THERMISTORS)
{
	for (byte i = 0; i < NUM_THERMISTORS; i++){
		pidOutput[i] = 0;
		pidOutputSum[i] = 0;
	}
	lastTime = millis();
						
	
}

//-------------------------------------------------------------------
// Initialize a RGB LED
// Default intialization using pins defined in MusselBedHeaterlib.h

RGBLED::RGBLED(){};
RGBLED::~RGBLED(){};

void RGBLED::begin(){
	m_redled = 9;
	m_greenled = 5;
	m_blueled = 6;
	pinMode(m_redled, OUTPUT);
	pinMode(m_greenled, OUTPUT);
	pinMode(m_blueled, OUTPUT);
	if (COMMON_ANODE){
		digitalWrite(m_redled, HIGH); // for common anode LED, set high to shut off
		digitalWrite(m_greenled, HIGH);
		digitalWrite(m_blueled, HIGH);	
	} else {
		digitalWrite(m_redled, LOW); // for common cathode LED, set low to shut off
		digitalWrite(m_greenled, LOW);
		digitalWrite(m_blueled, LOW);			
	}
	
}

// User-specified pins for red, green, blue channels of LED
void RGBLED::begin(uint8_t redpin, uint8_t greenpin, uint8_t bluepin){
	m_redled = redpin;
	m_greenled = greenpin;
	m_blueled = bluepin;
	pinMode(m_redled, OUTPUT);
	pinMode(m_greenled, OUTPUT);
	pinMode(m_blueled, OUTPUT);
	if (COMMON_ANODE){
		digitalWrite(m_redled, HIGH); // for common anode LED, set high to shut off
		digitalWrite(m_greenled, HIGH);
		digitalWrite(m_blueled, HIGH);	
	} else {
		digitalWrite(m_redled, LOW); // for common cathode LED, set low to shut off
		digitalWrite(m_greenled, LOW);
		digitalWrite(m_blueled, LOW);			
	}
}
//-----------------setColor---------------------
// Enter a set of values 0-255 for the red, green, and blue LED channels
void RGBLED::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(m_redled, red);
  analogWrite(m_greenled, green);
  analogWrite(m_blueled, blue);  
}


//------------------------------------------------------------
// Other public functions

void printTimeSerial(DateTime now){
    //------------------------------------------------
    // printTime function takes a DateTime object from
    // the real time clock and prints the date and time
    // to the serial monitor.
    Serial.print(now.year(), DEC);
    Serial.print('-');
    if (now.month() < 10) {
        Serial.print(F("0"));
    }
    Serial.print(now.month(), DEC);
    Serial.print('-');
    if (now.day() < 10) {
        Serial.print(F("0"));
    }
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    if (now.hour() < 10){
        Serial.print(F("0"));
    }
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    if (now.minute() < 10) {
        Serial.print("0");
    }
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    if (now.second() < 10) {
        Serial.print(F("0"));
    }
    Serial.print(now.second(), DEC);
    // You may want to print a newline character
    // after calling this function i.e. Serial.println();
    
}

//*********************************************************
// OneWire temperature sensor functions
// sensorAddr is expected to be at least a 4row x 8byte array global variable
void getRefSensorAddresses(OneWire& max31820, 
							DallasTemperature& refSensors, 
							uint8_t numRefSensors, 
							uint8_t sensorAddr[][8]){
   
    uint8_t addr[8]; // OneWire address array, 8 bytes long
	max31820.reset_search();
    for (uint8_t i = 0; i < numRefSensors; i++){
        max31820.search(addr); // read next sensor address into addr
        // Copy address values to sensorAddr array
        for (uint8_t j = 0; j < 8; j++){
            sensorAddr[i][j] = addr[j];
        }
        // Set sensor resolution to 11 bits, approx 400ms conversion time
        refSensors.setResolution(addr, 11);
        
    }
    max31820.reset_search();
    
    // Tell the DallasTemperature library to not wait for the
    // temperature reading to complete after telling devices
    // to take a new temperature reading (so we can do other things
    // while the temperature reading is being taken by the devices).
    // You will have to arrange your code so that an appropriate
    // amount of time passes before you try to use getTempC() after
    // requestTemperatures() is used
    refSensors.setWaitForConversion(false);
}


//*********************************************************
// OneWire temperature sensor functions
// sensorAddr is expected to be at least a 4row x 8byte array

void getRefSensorAddresses(OneWire& max31820, 
						DallasTemperature& refSensors, 
						uint8_t numRefSensors, 
						uint8_t sensorAddr[][8],
						uint8_t TEMPERATURE_PRECISION){
    
    uint8_t addr[8]; // OneWire address array, 8 bytes long
    
    max31820.reset_search();
    for (uint8_t i = 0; i < numRefSensors; i++){
        max31820.search(addr); // read next sensor address into addr
        // Copy address values to sensorAddr array
        for (uint8_t j = 0; j < 8; j++){
            sensorAddr[i][j] = addr[j];
        }
        
        refSensors.setResolution(addr, TEMPERATURE_PRECISION);
        
    }
    max31820.reset_search();
    
    // Tell the DallasTemperature library to not wait for the
    // temperature reading to complete after telling devices
    // to take a new temperature reading (so we can do other things
    // while the temperature reading is being taken by the devices).
    // You will have to arrange your code so that an appropriate
    // amount of time passes before you try to use getTempC() after
    // requestTemperatures() is used
    refSensors.setWaitForConversion(false);
}




//---------------printTimeToSD----------------------------------------
// printTimeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void printTimeToSD (SdFile& mylogfile, DateTime tempTime) {
    // Write the date and time in a human-readable format
    // to the file on the SD card.
    mylogfile.print(tempTime.year(), DEC);
    mylogfile.print(F("-"));
    if (tempTime.month() < 10) {
        mylogfile.print("0");
    }
    mylogfile.print(tempTime.month(), DEC);
    mylogfile.print(F("-"));
    if (tempTime.day() < 10) {
        mylogfile.print("0");
    }
    mylogfile.print(tempTime.day(), DEC);
    mylogfile.print(F(" "));
    if (tempTime.hour() < 10){
        mylogfile.print("0");
    }
    mylogfile.print(tempTime.hour(), DEC);
    mylogfile.print(F(":"));
    if (tempTime.minute() < 10) {
        mylogfile.print("0");
    }
    mylogfile.print(tempTime.minute(), DEC);
    mylogfile.print(F(":"));
    if (tempTime.second() < 10) {
        mylogfile.print("0");
    }
    mylogfile.print(tempTime.second(), DEC);
}

//-------------- initFileName --------------------------------------------------
// initFileName - a function to create a filename for the SD card based
// on the 4-digit year, month, day, hour, minutes and a 2-digit counter.
// The character array 'filename' was defined as a global array
// at the top of the sketch in the form "YYYYMMDD_HHMM_00.csv"
void initFileName(SdFat& sd, SdFile& logfile, DateTime time1, char *filename, bool serialValid, char *serialNumber) {
    
    char buf[5];
    // integer to ascii function itoa(), supplied with numeric year value,
    // a buffer to hold output, and the base for the conversion (base 10 here)
    itoa(time1.year(), buf, 10);
    // copy the ascii year into the filename array
    for (byte i = 0; i < 4; i++){
        filename[i] = buf[i];
    }
    // Insert the month value
    if (time1.month() < 10) {
        filename[4] = '0';
        filename[5] = time1.month() + '0';
    } else if (time1.month() >= 10) {
        filename[4] = (time1.month() / 10) + '0';
        filename[5] = (time1.month() % 10) + '0';
    }
    // Insert the day value
    if (time1.day() < 10) {
        filename[6] = '0';
        filename[7] = time1.day() + '0';
    } else if (time1.day() >= 10) {
        filename[6] = (time1.day() / 10) + '0';
        filename[7] = (time1.day() % 10) + '0';
    }
    // Insert an underscore between date and time
    filename[8] = '_';
    // Insert the hour
    if (time1.hour() < 10) {
        filename[9] = '0';
        filename[10] = time1.hour() + '0';
    } else if (time1.hour() >= 10) {
        filename[9] = (time1.hour() / 10) + '0';
        filename[10] = (time1.hour() % 10) + '0';
    }
    // Insert minutes
    if (time1.minute() < 10) {
        filename[11] = '0';
        filename[12] = time1.minute() + '0';
    } else if (time1.minute() >= 10) {
        filename[11] = (time1.minute() / 10) + '0';
        filename[12] = (time1.minute() % 10) + '0';
    }
    // Insert another underscore after time
    filename[13] = '_';
    // If there is a valid serialnumber, insert it into
    // the file name in positions 17-20.
    if (serialValid) {
        byte serCount = 0;
        for (byte i = 17; i < 21; i++){
            filename[i] = serialNumber[serCount];
            serCount++;
        }
    }
    // Next change the counter on the end of the filename
    // (digits 14+15) to increment count for files generated on
    // the same day. This shouldn't come into play
    // during a normal data run, but can be useful when
    // troubleshooting.
    for (uint8_t i = 0; i < 100; i++) {
        filename[14] = i / 10 + '0';
        filename[15] = i % 10 + '0';
        
        if (!sd.exists(filename)) {
            // when sd.exists() returns false, this block
            // of code will be executed to open the file
            if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
                // If there is an error opening the file, notify the
                // user. Otherwise, the file is open and ready for writing
                // Turn both indicator LEDs on to indicate a failure
                // to create the log file
                //				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led
                //				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led
                delay(5);
            }
            break; // Break out of the for loop when the
            // statement if(!logfile.exists())
            // is finally false (i.e. you found a new file name to use).
        } // end of if(!sd.exists())
    } // end of file-naming for loop
    //------------------------------------------------------------
    // Write 1st header line
    logfile.print(F("POSIXt,DateTime"));
	// write column headers for the 4 reference mussel temperatures
	for (byte i = 1; i <=4; i++){
		logfile.print(F(",RefTemp")); // column title
		logfile.print(i);		// add channel number to title
		logfile.print(F(".C")); // add units Celsius on end
	}
	// write header for Setpoint temperature
	logfile.print(F(",Setpoint.C"));
	
	// write column headers for the 16 heated mussel temperatures
    for (byte i = 1; i <= 16; i++){
        
        logfile.print(F(",Heated")); // column title
        logfile.print(i);		     // add channel number to title
		logfile.print(F(".C"));      // add units Celsius on end
    }
    logfile.print(F(",Battery.V"));
	logfile.print(F(",Tide.ft"));
	logfile.print(F(",State"));
    logfile.println();
    // Update the file's creation date, modify date, and access date.
    logfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    logfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    logfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    logfile.close(); // force the data to be written to the file by closing it
} // end of initFileName function


//-------------- initTuningFileName --------------------------------------------------
// initTuningFileName - a function to create a filename for the SD card based
// on the 4-digit year, month, day, hour, minutes and a 2-digit counter.
// This version is set up to record additional columns of data from the PID output
// routine associated with each thermistor channel.
// The character array 'filename' was defined as a global array
// at the top of the sketch in the form "YYYYMMDD_HHMM_00.csv"
void initTuningFileName(SdFat& sd, SdFile& logfile, DateTime time1, char *filename, bool serialValid, char *serialNumber, byte nChannels, double Kp, double Ki, double Kd, double tempIncreaseC) {
    
    char buf[5];
    // integer to ascii function itoa(), supplied with numeric year value,
    // a buffer to hold output, and the base for the conversion (base 10 here)
    itoa(time1.year(), buf, 10);
    // copy the ascii year into the filename array
    for (byte i = 0; i < 4; i++){
        filename[i] = buf[i];
    }
    // Insert the month value
    if (time1.month() < 10) {
        filename[4] = '0';
        filename[5] = time1.month() + '0';
    } else if (time1.month() >= 10) {
        filename[4] = (time1.month() / 10) + '0';
        filename[5] = (time1.month() % 10) + '0';
    }
    // Insert the day value
    if (time1.day() < 10) {
        filename[6] = '0';
        filename[7] = time1.day() + '0';
    } else if (time1.day() >= 10) {
        filename[6] = (time1.day() / 10) + '0';
        filename[7] = (time1.day() % 10) + '0';
    }
    // Insert an underscore between date and time
    filename[8] = '_';
    // Insert the hour
    if (time1.hour() < 10) {
        filename[9] = '0';
        filename[10] = time1.hour() + '0';
    } else if (time1.hour() >= 10) {
        filename[9] = (time1.hour() / 10) + '0';
        filename[10] = (time1.hour() % 10) + '0';
    }
    // Insert minutes
    if (time1.minute() < 10) {
        filename[11] = '0';
        filename[12] = time1.minute() + '0';
    } else if (time1.minute() >= 10) {
        filename[11] = (time1.minute() / 10) + '0';
        filename[12] = (time1.minute() % 10) + '0';
    }
    // Insert another underscore after time
    filename[13] = '_';
    // If there is a valid serialnumber, insert it into
    // the file name in positions 17-20.
    if (serialValid) {
        byte serCount = 0;
        for (byte i = 17; i < 21; i++){
            filename[i] = serialNumber[serCount];
            serCount++;
        }
    }
    // Next change the counter on the end of the filename
    // (digits 14+15) to increment count for files generated on
    // the same day. This shouldn't come into play
    // during a normal data run, but can be useful when
    // troubleshooting.
    for (uint8_t i = 0; i < 100; i++) {
        filename[14] = i / 10 + '0';
        filename[15] = i % 10 + '0';
        
        if (!sd.exists(filename)) {
            // when sd.exists() returns false, this block
            // of code will be executed to open the file
            if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
                // If there is an error opening the file, notify the
                // user. Otherwise, the file is open and ready for writing
                // Turn both indicator LEDs on to indicate a failure
                // to create the log file
                //				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led
                //				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led
                delay(5);
            }
            break; // Break out of the for loop when the
            // statement if(!logfile.exists())
            // is finally false (i.e. you found a new file name to use).
        } // end of if(!sd.exists())
    } // end of file-naming for loop
    //------------------------------------------------------------
    // Write 1st header line
	logfile.print(F("Kp,"));
	logfile.print(Kp,4);
	logfile.print(F(",Ki,"));
	logfile.print(Ki,3);
	logfile.print(F(",Kd,"));
	logfile.print(Kd,4);
	logfile.print(F(",TempIncrease,"));
	logfile.println(tempIncreaseC,1);
	// Write 2nd header line
    logfile.print(F("POSIXt,DateTime"));
	// write column headers for the 4 reference mussel temperatures
	for (byte i = 1; i <=4; i++){
		logfile.print(F(",RefTemp")); // column title
		logfile.print(i);		// add channel number to title
		logfile.print(F(".C")); // add units Celsius on end
	}
	// write header for Setpoint temperature
	logfile.print(F(",Setpoint.C"));
	
	// write column headers for the 16 heated mussel temperatures
    for (byte i = 1; i <= nChannels; i++){
        
        logfile.print(F(",Heated")); // column title
        logfile.print(i);		     // add channel number to title
		logfile.print(F(".C"));      // add units Celsius on end
    }
	// write column headers for the 16 PID output values for each heated mussel
	for (byte i = 1; i <= nChannels; i++){
        
        logfile.print(F(",PIDoutput")); // column title
        logfile.print(i);		     // add channel number to title
		
    }
    logfile.print(F(",Battery.V"));
    logfile.println();
    // Update the file's creation date, modify date, and access date.
    logfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    logfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    logfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    logfile.close(); // force the data to be written to the file by closing it
} // end of initFileName function



//---------- startTIMER2 ----------------------------------------------------
// startTIMER2 function
// Starts the 32.768kHz clock signal being fed into XTAL1 from the
// real time clock to drive the
// quarter-second interrupts used during data-collecting periods.
// Supply a current DateTime time value, the real time clock object, and
// a sample per second value (SPS) of 1, 2 , or 4
// This function returns a DateTime value that can be used to show the
// current time when returning from this function.
DateTime startTIMER2(DateTime currTime, RTC_DS3231& rtc, byte SPS){
    TIMSK2 = 0; // stop timer 2 interrupts
    
    rtc.enable32kHz(true);
    ASSR = _BV(EXCLK); // Set EXCLK external clock bit in ASSR register
    // The EXCLK bit should only be set if you're trying to feed the
    // 32.768 clock signal from the Chronodot into XTAL1.
    
    ASSR = ASSR | _BV(AS2); // Set the AS2 bit, using | (OR) to avoid
    // clobbering the EXCLK bit that might already be set. This tells
    // TIMER2 to take its clock signal from XTAL1/2
    TCCR2A = 0; //override arduino settings, ensure WGM mode 0 (normal mode)
    
    // Set up TCCR2B register (Timer Counter Control Register 2 B) to use the
    // desired prescaler on the external 32.768kHz clock signal. Depending on
    // which bits you set high among CS22, CS21, and CS20, different
    // prescalers will be used. See Table 18-9 on page 158 of the AVR 328P
    // datasheet.
    //  TCCR2B = 0;  // No clock source (Timer/Counter2 stopped)
    // no prescaler -- TCNT2 will overflow once every 0.007813 seconds (128Hz)
    //  TCCR2B = _BV(CS20) ;
    // prescaler clk/8 -- TCNT2 will overflow once every 0.0625 seconds (16Hz)
    if (SPS == 16){
        TCCR2B = _BV(CS21) ;
    } else if (SPS == 4){
        // prescaler clk/32 -- TCNT2 will overflow once every 0.25 seconds
        TCCR2B = _BV(CS21) | _BV(CS20);
    } else if (SPS == 2) {
        TCCR2B = _BV(CS22) ; // prescaler clk/64 -- TCNT2 will overflow once every 0.5 seconds
    } else if (SPS == 1){
        TCCR2B = _BV(CS22) | _BV(CS20); // prescaler clk/128 -- TCNT2 will overflow once every 1 second
    }
    
    
    // Pause briefly to let the RTC roll over a new second
    DateTime starttime = currTime;
    // Cycle in a while loop until the RTC's seconds value updates
    while (starttime.second() == currTime.second()) {
        delay(1);
        currTime = rtc.now(); // check time again
    }
    
    TCNT2 = 0; // start the timer at zero
    // wait for the registers to be updated
    while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB))) {}
    TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2); // clear the interrupt flags
    TIMSK2 = _BV(TOIE2); // enable the TIMER2 interrupt on overflow
    // TIMER2 will now create an interrupt every time it rolls over,
    // which should be every 0.0625, 0.25, 0.5 or 1 seconds (depending on value
    // of SPS "SAMPLES_PER_SECOND") regardless of whether the AVR is awake or asleep.
    return currTime;
}

//--------------------goToSleep-----------------------------------------------
// goToSleep function. When called, this puts the AVR to
// sleep until it is awakened by an interrupt (TIMER2 in this case)
// This is a higher power sleep mode than the lowPowerSleep function uses.
void goToSleep() {
    // Create three variables to hold the current status register contents
    byte adcsra, mcucr1, mcucr2;
    // Cannot re-enter sleep mode within one TOSC cycle.
    // This provides the needed delay.
    OCR2A = 0; // write to OCR2A, we're not using it, but no matter
    while (ASSR & _BV(OCR2AUB)) {} // wait for OCR2A to be updated
    // Set the sleep mode to PWR_SAVE, which allows TIMER2 to wake the AVR
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    adcsra = ADCSRA; // save the ADC Control and Status Register A
    ADCSRA = 0; // disable ADC by zeroing out the ADC status register
    sleep_enable();
    // Do not interrupt before we go to sleep, or the
    // ISR will detach interrupts and we won't wake.
    noInterrupts ();
    
    // wdt_disable(); // turn off the watchdog timer
    
    //ATOMIC_FORCEON ensures interrupts are enabled so we can wake up again
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        // Turn off the brown-out detector
        mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);
        mcucr2 = mcucr1 & ~_BV(BODSE);
        MCUCR = mcucr1; //timed sequence
        // BODS stays active for 3 cycles, sleep instruction must be executed
        // while it's active
        MCUCR = mcucr2;
    }
    // We are guaranteed that the sleep_cpu call will be done
    // as the processor executes the next instruction after
    // interrupts are turned on.
    interrupts();  // one cycle, re-enables interrupts
    sleep_cpu(); //go to sleep
    //wake up here
    sleep_disable(); // upon wakeup (due to interrupt), AVR resumes here
    // watchdogSetup(); // re-enable watchdog timer
    ADCSRA = adcsra; // re-apply the previous settings to the ADC status register
    
}


//------------readBatteryVoltage-------------------
// readBatteryVoltage function. This will read the AD convertor
// and calculate the approximate battery voltage (before the
// voltage regulator). Returns a floating point value for
// voltage.
float readBatteryVoltage(byte BATT_MONITOR_EN, 
							byte BATT_MONITOR, 
							float dividerRatio, 
							float refVoltage){
    // Turn on the battery voltage monitor circuit
    digitalWrite(BATT_MONITOR_EN, HIGH);
    delay(1);
    // Read the analog input pin
    unsigned int rawAnalog = 0;
    analogRead(BATT_MONITOR); // This initial value is ignored
    delay(3); // Give the ADC time to stablize
    // Take 4 readings
    for (byte i = 0; i<4; i++){
        rawAnalog = rawAnalog + analogRead(BATT_MONITOR);
        delay(2);
    }
    // Do a 2-bit right shift to divide rawAnalog
    // by 4 to get the average of the 4 readings
    rawAnalog = rawAnalog >> 2;
    // Shut off the battery voltage sense circuit
    digitalWrite(BATT_MONITOR_EN, LOW);
    // Convert the rawAnalog count value (0-1023) into a voltage
    // Relies on variables dividerRatio and refVoltage
    float reading = (rawAnalog  * (refVoltage / 1023.0)) * dividerRatio;
    return reading; // return voltage result
}




