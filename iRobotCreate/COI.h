#include <Arduino.h>

#define COI_BAUDRATE            57600
#define SENSOR_UPDATE_RATE      50      // microseconds. DO NOT go lower than 15ms
#define SERIAL_TIMEOUT          25      // milliseconds. 
#define IROBOT_CREATE_BOOT_TIME 4000    // milliseconds. 4 secs for the create to boot.
// Commands
#define CMD_START               0x80
#define CMD_MODE_SAFE           0x83
#define CMD_MODE_FULL           0x84
#define CMD_MODE_PASSIVE        0x80
#define CMD_DRIVE               0x89
#define CMD_DRIVE_DIRECT        0x91
#define CMD_LEDS                0x8B
#define CMD_DIGITAL_OUTPUTS     0x93
#define CMD_PWM_LSD             0x90
#define CMD_LSD                 0x8A
#define CMD_SEND_IR             0x97
#define CMD_SONG                0x8C
#define CMD_PLAY_SONG           0x8D
#define CMD_SENSORS             0x8E
#define CMD_QUERY_LIST          0x95
#define CMD_STREAM              0x94
#define CMD_PAUSE_RESUME_STREAM 0x96
#define CMD_SCRIPT              0x98
#define CMD_PLAY_SCRIPT         0x99
#define CMD_SHOW_SCRIPT         0x9A
#define CMD_WAIT_TIME           0x9B
#define CMD_WAIT_DIST           0x9C
#define CMD_WAIT_ANGLE          0x9D
#define CMD_WAIT_EVENT          0x9E

// Packet ids
#define BUMPS_WHEELS            0x7   
#define WALL                    0x8
#define CLIFF_LEFT              0x9
#define CLIFF_FRONT_LEFT        0xA
#define CLIFF_FRONT_RIGHT       0xB
#define CLIFF_RIGHT             0xC
#define VIRTUAL_WALL            0xD
#define LSD_WHEEL_OVERCUR       0xE
#define UNUSED_ONE              0xF
#define UNUSED_TWO              0x10
#define INFRARED_BYTE           0x11
#define BUTTONS                 0x12
#define DIST_SINCE_LAST         0x13
#define ANGLE_SINCE_LAST        0x14
#define CHARGING_STATE          0x15
#define BAT_VOLTAGE             0x16
#define BAT_CURRENT             0x17
#define BAT_TEMP                0x18
#define BAT_CHARGE              0x19
#define BAT_CAPACITY            0x1A
#define WALL_SIG                0x1B
#define CLIFF_LEFT_SIG          0x1C
#define CLIFF_FRONT_LEFT_SIG    0x1D
#define CLIFF_FRONT_RIGHT_SIG   0x1E
#define CLIFF_RIGHT_SIG         0x1F
#define CARGO_DIGITAL_IN        0x20
#define CARGO_ANALOG_IN         0x21
#define CHARGING_SOURCES        0x22
#define OI_MODE                 0x23
#define CURRENT_SONG            0x24
#define SONG_PLAYER_STATE       0x25
#define NUM_STREAM_PACKETS      0x26
#define REQUESTED_VELOCITY      0x27
#define REQUESTED_RADIUS        0x28
#define REQUESTED_RIGHT_VEL     0x29
#define REQUESTED_LEFT_VEL      0x2A

// Stream packet groups
#define BOOLEAN_SENSORS_STATS   0x0     // includes 7-26
#define BOOLEAN_SENSORS         0x1     // includes 7-16
#define BUTTONS_IR_DISTANCES    0x2     // includes 17-20
#define BATTERY_STATS           0x3     // includes 21-26
#define DETAIL_SENSORS          0x4     // includes 27-34
#define VELOCITIES_SONGS_MODE   0x5     // includes 35-42
#define ALL_PACKETS             0x6     // includes 7-42

#define STREAM_HEADER           0x13

// IR Byte Codes
#define IR_RC_LEFT              0x81
#define IR_RC_FORWARD           0x82
#define IR_RC_RIGHT             0x83
#define IR_RC_SPOT              0x84
#define IR_RC_MAX               0x85
#define IR_RC_SMALL             0x86
#define IR_RC_MEDIUM            0x87
#define IR_RC_LARGE_CLEAN       0x88
#define IR_RC_PAUSE             0x89
#define IR_RC_POWER             0x8A
#define IR_RC_ARC_FORWARD_LEFT  0x8B
#define IR_RC_ARC_FORWARD_RIGHT 0x8C
#define IR_RC_DRIVE_STOP        0x8D
#define IR_SCHED_RC_SEND_ALL    0x8E
#define IR_SCHED_RC_SEEK_DOCK   0x8F
#define IR_HB_RESERVED          0xF0
#define IR_HB_RED_BOUY          0xF8
#define IR_HB_GREEN_BOUY        0xF4
#define IR_HB_FORCE_FIELD       0xF2
#define IR_HB_RED_GREEN_BOUYS   0xFC
#define IR_HB_RED_FORCE_FIELD   0xFA
#define IR_HB_GREEN_FORCE_FIELD 0xF6
#define IR_HB_RED_GREEN_FORCE   0xFE

// Charging State codes
#define NOT_CHARGING            0x0
#define RECONDITIONING_CHARGING 0x1
#define FULL_CHARGING           0x2
#define TRICKLE_CHARGING        0x3
#define WAITING_CHARGING        0x4
#define CHARGING_FAULT          0x5

// OI Modes from a request
#define OI_OFF                  0x0
#define OI_PASSIVE              0x1
#define OI_SAFE                 0x2
#define OI_FULL                 0x3

// bit positions in command set packets
#define SET_ADVANCE_LED         3
#define SET_PLAY_LED            1
#define SET_DIGITAL_OUT_0       0
#define SET_DIGITAL_OUT_1       1
#define SET_DIGITAL_OUT_2       2
#define SET_LSD_2               2
#define SET_LSD_1               1
#define SET_LSD_0               0

// bit positions in returned packets
#define READ_WHEELDROP_CASTER   4
#define READ_WHEELDROP_LEFT     3
#define READ_WHEELDROP_RIGHT    2
#define READ_BUMP_LEFT          1
#define READ_BUMP_RIGHT         0
#define READ_LSD0               1
#define READ_LSD1               0
#define READ_LSD2               2
#define READ_LEFT_WHEEL_OC      4
#define READ_RIGHT_WHEEL_OC     3
#define READ_ADVANCE_BUTTON     2
#define READ_PLAY_BUTTON        0
#define READ_DIGITAL_INPUT_3    3
#define READ_DIGITAL_INPUT_2    2
#define READ_DIGITAL_INPUT_1    1
#define READ_DIGITAL_INPUT_0    0
#define READ_CHARGE_HOME_BASE   1
#define READ_CHARGE_INTERNAL    0 


// Boolean macros for sensor bit flags
#define WHEELDROP_CASTER(p)         bitRead(p,READ_WHEELDROP_CASTER)
#define WHEELDROP_LEFT(p)           bitRead(p,READ_WHEELDROP_LEFT)
#define WHEELDROP_RIGHT(p)          bitRead(p,READ_WHEELDROP_RIGHT)
#define BUMP_LEFT(p)                bitRead(p,READ_BUMP_LEFT)
#define BUMP_RIGHT(p)               bitRead(p,READ_BUMP_RIGHT)
#define LSD0_OVERCURRENT(p)         bitRead(p,READ_LSD0)
#define LSD1_OVERCURRENT(p)         bitRead(p,READ_LSD1)
#define LSD2_OVERCURRENT(p)         bitRead(p,READ_LSD2)
#define LEFT_WHEEL_OVERCURRENT(p)   bitRead(p,READ_LEFT_WHEEL_OC)
#define RIGHT_WHEEL_OVERCURRENT(p)  bitRead(p,READ_RIGHT_WHEEL_OC)
#define ADVANCE_BUTTON(p)           bitRead(p,READ_ADVANCE_BUTTON)
#define PLAY_BUTTON(p)              bitRead(p,READ_PLAY_BUTTON)
#define DIGITAL_INPUT_3(p)          bitRead(p,READ_DIGITAL_INPUT_3)
#define DIGITAL_INPUT_2(p)          bitRead(p,READ_DIGITAL_INPUT_2)
#define DIGITAL_INPUT_1(p)          bitRead(p,READ_DIGITAL_INPUT_1)
#define DIGITAL_INPUT_0(p)          bitRead(p,READ_DIGITAL_INPUT_0)
#define CHARGE_SOURCE_HOME_BASE(p)  bitRead(p,READ_CHARGE_HOME_BASE)
#define CHARGE_SOURCE_INTERNAL(p)   bitRead(p,READ_CHARGE_INTERNAL)

// Special cases
#define DRIVE_STRAIGHT_RADIUS   0x7FFF
#define DRIVE_TURN_IN_PLACE_C   0xFFFF
#define DRIVE_TURN_IN_PLACE_CW  0x0001
#define DRIVE_STOP              0x0000

// structs
typedef struct note_t {
    byte notePitch;
    byte noteDuration;
} note_t;

// Global varss
SoftwareSerial coiSerial = SoftwareSerial(2,3);

// Create state vars
byte            BUMPS_WHEEL_DROPS_STATE;
byte            WALL_STATE;
byte            CLIFF_LEFT_STATE;
byte            CLIFF_FRONT_LEFT_STATE;
byte            CLIFF_FRONT_RIGHT_STATE;
byte            CLIFF_RIGHT_STATE;
byte            VIRTUAL_WALL_STATE;
byte            LSD_WHEEL_OVERCURRENTS_STATE;
byte            INFRARED_RX_STATE;
byte            BUTTONS_STATE;
int             DISTANCE_STATE;
int             ANGLE_STATE;
byte            CHARGING_STATUS_STATE;
unsigned int    VOLTAGE_STATE;
int             CURRENT_STATE;
int             BATTERY_TEMP_STATE;
unsigned int    BATTERY_CHARGE_STATE;
unsigned int    BATTERY_CAPACITY_STATE;    
unsigned int    WALL_SIGNAL_STATE;
unsigned int    CLIFF_LEFT_SIGNAL_STATE;
unsigned int    CLIFF_FRONT_LEFT_SIGNAL_STATE;
unsigned int    CLIFF_FRONT_RIGHT_SIGNAL_STATE;
unsigned int    CLIFF_RIGHT_SIGNAL_STATE;
byte            CARGO_BAY_DIGITAL_INPUTS_STATE;
unsigned int    CARGO_BAY_ANALOG_SIGNAL_STATE;
byte            CHARGING_SOURCE_STATE;
byte            OI_MODE_STATE;
byte            SONG_NUMBER_STATE;
byte            SONG_PLAYING_STATE;
byte            NUMBER_OF_STREAM_PACKETS_STATE;
int             REQUESTED_VELOCITY_STATE;
int             REQUESTED_RADIUS_STATE;
int             REQUESTED_RIGHT_VELOCITY_STATE;
int             REQUESTED_LEFT_VELOCITY_STATE;
byte            LEDS_STATE;
byte            PWR_LED_COLOR;
byte            PWR_LED_INTENSITY;
byte            DIGITAL_OUTPUTS_STATE;
byte            LSD_STATE;

/**
  * Combines two 8bit bytes into 1 16bit signed int.
  */
int combineHighLowBytesSigned (byte low, byte high) {
    return (high << 8) | low;
}

/**
  * Combines two 8bit bytes into 1 16bit unsigned int.
  */
unsigned int combineHighLowBytesUnsigned (byte low, byte high) {
    return (high << 8) | low;
} 

/**
  * Sets up the serial connection to the 
  * iRobot Create using the pins passed in
  */
void COISerialSetup (int rxPin, int txPin) {
    coiSerial = SoftwareSerial (rxPin, txPin);
    coiSerial.begin (COI_BAUDRATE);
}

/**
  * Writes a byte to the iRobot create over
  * the serial connection.
  */
void COISerialWriteByte (byte data) {
    coiSerial.write (data);    
}

/**
  * Reads a byte from the iRobot create serial
  * connection.
  */
byte COISerialReadByte () {
    return coiSerial.read ();
}

/**
  * Returns the number of bytes available to read
  * fromt he iRobot create serial connection.
  */
int COISerialAvailable () {
    return coiSerial.available ();
}

/**
  * Waits for serial data from the robot, it has
  * a timeout so that it doesn't get stuck in a loop.
  * the timeout flag is returned with 0.
  */
int COIWaitForData () {
    unsigned long start;
    unsigned long t;
    
    start = millis ();

    while (!COISerialAvailable ()) {
        t = millis ();
        if ((t - start) > SERIAL_TIMEOUT) {
            return 0;
        }
    }
    return 1;
}

/**
  * Writes the mode to the robot.
  */
void COIChangeMode (byte mode) {
    COISerialWriteByte (mode);
}

/**
  * Sends the current values of the led state vars
  * to the robot.
  */
void COISendLEDCMD () {
    COISerialWriteByte (CMD_LEDS);            // send the led cmd
    COISerialWriteByte (LEDS_STATE);          // send the leds state var
    COISerialWriteByte (PWR_LED_COLOR);       // send the power led color
    COISerialWriteByte (PWR_LED_INTENSITY);   // send the powre led intensity
}

/**
  * Iniitalizes the robot communications and by
  * default sets the robot in the FULL OI mode.
  * 
  * This function takes care of waiting for the
  * irobot create to boot before we attempt to 
  * send it commands.
  */
void COIInitialize (int txPin, int rxPin) {
    delay (IROBOT_CREATE_BOOT_TIME);
    COISerialSetup (txPin, rxPin);

    COISerialWriteByte (CMD_START);
    COIChangeMode (CMD_MODE_FULL);

    LEDS_STATE = 0x0;
    PWR_LED_COLOR = 0x0;
    PWR_LED_INTENSITY = 0xFF;
    COISendLEDCMD ();
}

/**
  * Sends the instructions to turn on the play led
  * and maintains current state of the other leds.
  */
void COITurnOnPlayLED () {
    bitSet(LEDS_STATE, SET_PLAY_LED);  
    COISendLEDCMD (); 
}

/**
  * Sends the instruction to turn off the play led
  * and maintains current state of the other leds.
  */
void COITurnOffPlayLED () {
    bitClear(LEDS_STATE, SET_PLAY_LED);    
    COISendLEDCMD (); 
}

/**
  * Sends the instrunction to turn on the advance led
  * and maintains current state of the other leds.
  */
void COITurnOnAdvanceLED () {
    bitSet(LEDS_STATE, SET_ADVANCE_LED);
    COISendLEDCMD (); 
}


/**
  * Sends the instruction to turn off the advance led
  * and maintains current state of the other leds.
  */
void COITurnOffAdvanceLED () {
    bitClear(LEDS_STATE, SET_ADVANCE_LED);
    COISendLEDCMD (); 
}

/**
  * Sends a drive command to the robot. The velocity is in
  * mm/s and its range is -500 - 500. The radius is in mm 
  * and its range is -2000 - 2000.
  */
void COIDrive (int velocity, int radius) {
    COISerialWriteByte (CMD_DRIVE);
    COISerialWriteByte (highByte (velocity));
    COISerialWriteByte (lowByte (velocity));
    COISerialWriteByte (highByte (radius));
    COISerialWriteByte (lowByte (radius));
}

/**
  * Stops the robot.
  */
void COIDriveStop () {
    COIDrive (DRIVE_STOP, DRIVE_STOP);
}

/**
  * Sends a direct drive command to the robot and its wheels
  * can then be controlled independently of each other. The
  * velocities are in mm/s and their range is -500 - 500.
  */
void COIDirectDrive (int rightVelocity, int leftVelocity) {
    COISerialWriteByte (CMD_DRIVE_DIRECT);
    COISerialWriteByte (highByte (rightVelocity));
    COISerialWriteByte (lowByte (rightVelocity));
    COISerialWriteByte (highByte (leftVelocity));
    COISerialWriteByte (lowByte (leftVelocity));
}

/**
  * Controls the 3 digital outputs from the cargo bay connector.
  */
void COISetDigitalOutputs () {
    COISerialWriteByte (CMD_DIGITAL_OUTPUTS);
    COISerialWriteByte (DIGITAL_OUTPUTS_STATE);
}

/**
  * Sets the passed in output number in the digital output and
  * then sends the instruction to the robot.
  */
void COISetDigitalOutput (byte output, boolean state) {
    if (output <= SET_DIGITAL_OUT_2) {
        if (state)
            bitSet (DIGITAL_OUTPUTS_STATE, output);
        else
            bitClear (DIGITAL_OUTPUTS_STATE, output);
        
        COISetDigitalOutputs ();
    } 
}

/**
  * Sets the low side drivers (LSDs) to the passed in pwm settings.
  * The acceptable pwm value range is 0 - 128.
  */
void COISetLowSideDriversPWM (byte pwm0, byte pwm1, byte pwm2) {
    if (pwm0 > 0x80)
        pwm0 = 0x80;
    else if (pwm0 < 0x0)
        pwm0 = 0x0;
    
    if (pwm1 > 0x80)
        pwm1 = 0x80;
    else if (pwm1 < 0x0)
        pwm1 = 0x0;
    
    if (pwm2 > 0x80)
        pwm2 = 0x80;
    else if (pwm2 < 0x0)
        pwm2 = 0x0;

    COISerialWriteByte (CMD_PWM_LSD);
    COISerialWriteByte (pwm0);
    COISerialWriteByte (pwm1);
    COISerialWriteByte (pwm2);
}

/**
  * Sets the low side drivers to either on or off.
  * On is equivalent to 100% pwm.
  */
void COISetLowSideDrivers () {
    COISerialWriteByte (CMD_LSD);
    COISerialWriteByte (LSD_STATE);
}

/**
  * Sets the specified low side driver to either on or off
  * and then sends the command to the robot.
  */
void COISetLowSideDriver (byte driver, boolean state) {
    if (driver <= SET_LSD_2) {
        if (state)
            bitSet (LSD_STATE, driver);
        else
            bitClear (LSD_STATE, driver);
        
        COISetLowSideDrivers ();
    }
}

/**
  * Sends a byte through low side driver 1 when it is
  * wired accordingly with an IR emitter. Valid values 
  * can be 0 - 255.
  */
void COISendIR (byte data) {
    COISerialWriteByte (CMD_SEND_IR);
    COISerialWriteByte (data);
}

/**
  * Defines a new song. The song
  * id may be from 0 - 15. If a song already exists at that
  * posistion then it will be over written. Each song can
  * contain up to 16 notes.
  */
void COIDefineNewSong (byte id, note_t* notes, byte numNotes) {
    note_t curNote;
    
    if (numNotes > 0x10 || numNotes < 0x0)
        return; 

    COISerialWriteByte (CMD_SONG);
    COISerialWriteByte (id);
    COISerialWriteByte (numNotes); 
     
    for (int i = 0; i < numNotes; i++) {
        curNote = notes[i]; 
        COISerialWriteByte (curNote.notePitch);
        COISerialWriteByte (curNote.noteDuration); 
    } 
}

/**
  * Plays the passed in song id. This will have no effect if
  * a song is currently being played.
  */
void COIPlaySong (byte id) {
    COISerialWriteByte (CMD_PLAY_SONG);
    COISerialWriteByte (id);
}

/**
  * Reads a particular sensor value from the robot and updates
  * the state variable associated with it.
  */
void COIRecieveAndUpdateSensor (byte sensorId) {
    byte low;
    byte high;

    if (!COIWaitForData ())
        return;
    
    high = COISerialReadByte ();

    switch (sensorId) {
		case BUMPS_WHEELS:
            BUMPS_WHEEL_DROPS_STATE = high;         
			break;
		case WALL:
            WALL_STATE = high; 
			break;
		case CLIFF_LEFT:
            CLIFF_LEFT_STATE = high;
			break;
		case CLIFF_FRONT_LEFT:
            CLIFF_FRONT_LEFT_STATE = high;
			break;
		case CLIFF_FRONT_RIGHT:
            CLIFF_FRONT_RIGHT_STATE = high;
			break;
		case CLIFF_RIGHT:
            CLIFF_RIGHT_STATE = high;
			break;
		case VIRTUAL_WALL:
            VIRTUAL_WALL_STATE = high;
			break;
		case LSD_WHEEL_OVERCUR:
            LSD_WHEEL_OVERCURRENTS_STATE = high; 
			break;
		case UNUSED_ONE:
			break;
		case UNUSED_TWO:
			break;
		case INFRARED_BYTE:
            INFRARED_RX_STATE = high;
			break;
		case BUTTONS:
            BUTTONS_STATE = high;
			break;
		case DIST_SINCE_LAST:
            low = COISerialReadByte ();
            DISTANCE_STATE = (int) (high << 8) | low;
			break;
		case ANGLE_SINCE_LAST:
            low = COISerialReadByte ();
            ANGLE_STATE = (int) (high << 8) | low;
			break;
		case CHARGING_STATE:
            CHARGING_STATUS_STATE = high;
			break;
		case BAT_VOLTAGE:
            low = COISerialReadByte ();
            VOLTAGE_STATE = (unsigned int) (high << 8) | low; 
			break;
		case BAT_CURRENT:
            low = COISerialReadByte ();
            CURRENT_STATE = (int) (high << 8) | low; 
			break;
		case BAT_TEMP:
            low = COISerialReadByte ();
            BATTERY_TEMP_STATE = (int) (high << 8) | low;
			break;
		case BAT_CHARGE:
            low = COISerialReadByte ();
            BATTERY_CHARGE_STATE = (unsigned int) (high << 8) | low;
			break;
		case BAT_CAPACITY:
            low = COISerialReadByte ();
            BATTERY_CAPACITY_STATE = (unsigned int) (high << 8) | low;
			break;
		case WALL_SIG:
            low = COISerialReadByte ();
            WALL_SIGNAL_STATE = (unsigned int) (high << 8) | low;
			break;
		case CLIFF_LEFT_SIG:
            low = COISerialReadByte ();
            CLIFF_LEFT_SIGNAL_STATE = (unsigned int) (high << 8) | low;
			break;
		case CLIFF_FRONT_LEFT_SIG:
            low = COISerialReadByte ();
            CLIFF_FRONT_LEFT_SIGNAL_STATE = (unsigned int) (high << 8) | low;
			break;
		case CLIFF_FRONT_RIGHT_SIG:
            low = COISerialReadByte ();
            CLIFF_FRONT_RIGHT_SIGNAL_STATE = (unsigned int) (high << 8) | low;
			break;
		case CLIFF_RIGHT_SIG:
            low = COISerialReadByte ();
            CLIFF_RIGHT_SIGNAL_STATE = (unsigned int) (high << 8) | low;
			break;
		case CARGO_DIGITAL_IN:
            CARGO_BAY_DIGITAL_INPUTS_STATE = high;
			break;
		case CARGO_ANALOG_IN:
            low = COISerialReadByte ();
            CARGO_BAY_ANALOG_SIGNAL_STATE = (unsigned int) (high << 8) | low;
			break;
		case CHARGING_SOURCES:
            CHARGING_SOURCE_STATE = high;
			break;
		case OI_MODE:
            OI_MODE_STATE = high;
			break;
		case CURRENT_SONG:
            SONG_NUMBER_STATE = high;
			break;
		case SONG_PLAYER_STATE:
            SONG_PLAYING_STATE = high;
			break;
		case NUM_STREAM_PACKETS:
            NUMBER_OF_STREAM_PACKETS_STATE = high;
			break;
		case REQUESTED_VELOCITY:
            low = COISerialReadByte ();
            REQUESTED_VELOCITY_STATE = (int) (high << 8) | low;
			break;
		case REQUESTED_RADIUS:
            low = COISerialReadByte ();
            REQUESTED_RADIUS_STATE = (int) (high << 8) | low;
			break;
		case REQUESTED_RIGHT_VEL:
            low = COISerialReadByte ();
            REQUESTED_RIGHT_VELOCITY_STATE = (int) (high << 8) | low;
			break;
		case REQUESTED_LEFT_VEL:
            low = COISerialReadByte ();
            REQUESTED_LEFT_VELOCITY_STATE = (int) (high << 8) | low;
			break;
        default:
            return;
    }
}

/**
  * Takes a list of sensor ids and reads them from the robot and
  * updates the state variables for those sensors.
  */
void COIFetchAndUpdateSensors (byte* sensorIds, byte numSensors) {
    byte i;

    COISerialWriteByte (CMD_QUERY_LIST);
    COISerialWriteByte (numSensors);
    for (i = 0; i < numSensors; i++) {
        COISerialWriteByte (sensorIds[i]);   
    }
    for (i = 0; i < numSensors; i++) {
        COIRecieveAndUpdateSensor (sensorIds[i]);
    }
}

/**
  * Recieves a single sensor value and then returns the value of the sensor
  * that is sent in from the robot.
  */
int COIRecieveSingleSensor () {
    byte low;
    byte high;
    int  data;
    
    if (!COIWaitForData ())
        return 0;
    
    high = COISerialReadByte ();
    
    if (COISerialAvailable ()) {
        low = COISerialReadByte ();
        data = (high << 8) | low;
    }
    else {
        data = high;
    }

    return data;
}

/**
  * Fetches a single sensor value based on the passed sensor
  * packet id. All sensor values are returned as signed ints, and
  * it is up to the user to cast the values accordingly to OI datasheet. 
  */ 
int COIFetchSingleSensor (byte sensorId) {
    COISerialWriteByte (CMD_SENSORS);
    COISerialWriteByte (sensorId);
    return COIRecieveSingleSensor ();    
}
