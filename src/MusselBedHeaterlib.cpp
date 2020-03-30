//
//  MusselBedHeaterlib.cpp
//  
//
//  Created by Luke Miller on 3/30/20.
//
//

#include "MusselBedHeaterlib.h"


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
    for (byte i = 0; i < 16; i++){
        // Cycle through channels to create headers for each column
        logfile.print(F(",Hall"));
        logfile.print(i);
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
float readBatteryVoltage(byte BATT_MONITOR_EN, byte BATT_MONITOR, float dividerRatio, float refVoltage){
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
    // Convert the rawAnalog count value (0-1024) into a voltage
    // Relies on global variables dividerRatio and refVoltage
    float reading = rawAnalog * dividerRatio * refVoltage / 1024;
    return reading; // return voltage result
}