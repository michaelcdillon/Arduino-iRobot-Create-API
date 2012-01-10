#include <SoftwareSerial.h>
#include "COI.h"

#define SERIAL_DEBUG_BAUD   111111
#define CREATE_TX            2
#define CREATE_RX           3

note_t readySong[13];
long CLOCK_VALUE;

/**
  * Sets up the serial communication for debuging,
  * the Create Robot is initialized, sent a simple song,
  * and then the battery stats are read and displayed.
  */
void setup () {
    Serial.begin (SERIAL_DEBUG_BAUD);

    COIInitialize (CREATE_TX, CREATE_RX);
    COITurnOnPlayLED ();   
    COITurnOnAdvanceLED (); 
    Serial.println ("iRobot Create Arduino Interface Demo");
    
    for (int i = 0; i < 13; i++) {
        readySong[i].notePitch = 69 + i;
        readySong[i].noteDuration = 8; 
    }

    COIDefineNewSong (0, readySong, 13);
    COIPlaySong (0);
    
    int data = COIFetchSingleSensor (BAT_VOLTAGE);
    Serial.print ("Bat Voltage: ");
    Serial.println ((unsigned int) data);
    
    data = COIFetchSingleSensor (BAT_CURRENT);
    Serial.print ("Bat Current: ");
    Serial.println (data);
    
    data = COIFetchSingleSensor (BAT_CAPACITY);
    Serial.print ("Bat Capacity: ");
    Serial.println ((unsigned int) data);
    
    data = COIFetchSingleSensor (BAT_CHARGE);
    Serial.print ("Bat Charge: ");
    Serial.println ((unsigned int) data);
    delay (500);
    COITurnOffPlayLED ();   
    COITurnOffAdvanceLED (); 
}

/**
  * Checks the bumper sensors for bumps and then turns
  * the robot in the opposite direction. This doesn't 
  * check cliffs or wheel drops so becareful
  * where you test.
  */
void checkSensorsAndReact () {
    if (BUMP_LEFT(BUMPS_WHEEL_DROPS_STATE) && BUMP_RIGHT(BUMPS_WHEEL_DROPS_STATE)) {
        Serial.println ("Bump Front");
        COIDriveStop ();
         COIDrive (300, DRIVE_TURN_IN_PLACE_C);
        COITurnOnPlayLED ();   
        COITurnOnAdvanceLED ();
        delay (500);
        COITurnOffPlayLED ();
        COITurnOffAdvanceLED ();
    }
    else if (BUMP_LEFT(BUMPS_WHEEL_DROPS_STATE)) {
        Serial.println ("Bump Left");
        COIDriveStop ();
        COIDrive (300, DRIVE_TURN_IN_PLACE_C);
        COITurnOnPlayLED ();   
        delay (250); 
        COITurnOffPlayLED ();   
    }
    else if (BUMP_RIGHT(BUMPS_WHEEL_DROPS_STATE)) {
        Serial.println ("Bump Right");
        COIDriveStop ();
        COIDrive (300, DRIVE_TURN_IN_PLACE_CW);
        COITurnOnAdvanceLED (); 
        delay (250);
        COITurnOffAdvanceLED (); 
    }
}

byte sensorsToRead[1] = {BUMPS_WHEELS};

/**
  * Drive the Create around at full velocity and
  * straight. Read the sensors every 25 miliseconds
  * and see if we need to react to any obsticales.
    */
void loop () {
    CLOCK_VALUE = millis (); 
    if (CLOCK_VALUE % 25 == 0) { 
        COIFetchAndUpdateSensors (sensorsToRead, 1);
        checkSensorsAndReact ();
        COIDrive (500, DRIVE_STRAIGHT_RADIUS);
    }
}
