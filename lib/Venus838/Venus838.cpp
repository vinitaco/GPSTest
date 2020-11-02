#include "Venus838.h"

Venus838::Venus838(HardwareSerial &gpsPort, HardwareSerial &debugPort, bool debug) : 
    _gpsPort(gpsPort),
    _debugPort(debugPort),
    _debug(debug) {}

void Venus838::initialize(int baudrate, uint8_t updaterate) {
    bool status = true;

    int currentBaudrate = getBaudRate();
    
    if ( currentBaudrate == -1 ) {
        if ( _debug ) _debugPort.println("Setting GPS baud rate to default (9600 baud)");
        _gpsPort.begin(9600);
        setFactoryDefaults(true);
        delay(100);
        currentBaudrate = 9600;
        _gpsPort.end();
    }

    _gpsPort.begin(currentBaudrate);
    delay(50);

    if (currentBaudrate != baudrate) status = setBaudRate(baudrate);

    if ( _debug ) { _debugPort.print("Status: "); _debugPort.println(status); }

    status = setUpdateRate(updaterate);
    if ( _debug ) { _debugPort.print("Status: "); _debugPort.println(status); }

    status = cfgWAAS(true);
    if ( _debug ) { _debugPort.print("Status: "); _debugPort.println(status); }

    status = cfgNavigationMode(true);
    if ( _debug ) { _debugPort.print("Status: "); _debugPort.println(status); }

}


// Resets the gps to factory values, reboot after reset option
bool Venus838::setFactoryDefaults(bool reboot) {
    if (_debug ) {Serial.print("Setting factory defaults (reboot: "); Serial.print(reboot); Serial.println(")"); }
    char command[] = {0x04, reboot? 0x01 : 0x00};
    return sendCommand(command, sizeof(command));
}

/*
Configures baud rate of GPS
    0x00 : 4800
    0x01: 9600
    0x02: 19200
    0x03: 38400
    0x04: 57600
    0x05: 115200
*/
bool Venus838::setBaudRate(int baudrate) {
    if ( _debug ) {_debugPort.print("Setting baud rate at "); _debugPort.println(baudrate);}
    
    int baudrates[6] = { 4800, 9600, 19200, 38400, 57600, 115200 };
    int baudrateIndex = -1;
    for( int i = 0; i < 6; i++ ) {
        if ( baudrate == baudrates[i] ) baudrateIndex = i;
    }
    if ( baudrateIndex == -1 ) {
        if (_debug) _debugPort.println("Invalid baud rate");
        return false;
    } 

    char command[4];
    command[0] = 0x05;
    command[1] = 0x00;
    command[2] = baudrateIndex;
    command[3] = 0x00;

    if ( sendCommand(command, 4) ) {
        _gpsPort.end();
        _gpsPort.begin(baudrate);
        return true;
    } else return false;
}

// Gets the current baudrate to initialize the instrument
int Venus838::getBaudRate() {
    if (_debug ) _debugPort.println("Getting baud rate");
    int baudrates[6] = { 4800, 9600, 19200, 38400, 57600, 115200 };
    for(int i = 0; i < 12; i++) {
        if (_debug ) { _debugPort.print("Starting gps port at "); _debugPort.println(baudrates[i % 6]); }

        _gpsPort.begin(baudrates[i % 6]);

        char command[2] = {0x02, 0x01};  
        bool baudrateFound = sendCommand(command, 2);

        _gpsPort.end();

        if (baudrateFound) {
            if (_debug ) { _debugPort.print("Baud rate found: "); _debugPort.println(baudrates[i % 6]); }
            return baudrates[i % 6];
        }
    }
    if ( _debug ) _debugPort.println("Baud rate not found");

    return -1;

}

/*
Configures update rate of GPS position
    0x01: 1 Hz
    0x02: 2 Hz
    0x04: 4 Hz
    0x05: 5 Hz
    0x08: 8 Hz
    0x0A: 10 Hz
    0x14: 20 Hz
*/
bool Venus838::setUpdateRate(uint8_t updaterate) {
    if ( _debug ) _debugPort.println("Setting update rate");

    uint8_t updaterates[] = {1, 2, 4, 5, 8, 10, 20};
    uint8_t updaterateIndex = -1;
    for( int i = 0; i < 7; i++ ) {
        if ( updaterate == updaterates[i] ) updaterateIndex = i; 
    }
    if ( updaterateIndex == -1 ) return false;

    char command[3];
    command[0] = 0x0E;
    command[1] = updaterates[updaterateIndex];
    command[2] = 0x00;

    return sendCommand(command, 3);
}

// Enables WAAS
bool Venus838::cfgWAAS(bool enable) {
    if ( _debug ) _debugPort.println("Setting up WAAS");
    
    char command[3] = {0x37, enable? 0x01 : 0x00, 0x00};

    return sendCommand(command, 3);
}


/*
Configures navigation mode
    true: car
    false: pedestrian
*/
bool Venus838::cfgNavigationMode(bool carMode) {
    if ( _debug ) _debugPort.println("Setting up to Car navigation mode");

    char command[3] = {0x3C, carMode? 0x00 : 0x01, 0x00};
    return sendCommand(command, 3);
}

/*
Configures the interval of NMEA messages 
NMEAMessage structure:
    byte 1: GGA interval = 01 (0 - 255, 00: disable, 01: default)
    byte 2: GSA interval = 01 (0 - 255, 00: disable, 01: default)
    byte 3: GSV interval = 01 (0 - 255, 00: disable, 01: default)
    byte 4: GLL interval = 00 (0 - 255, 00: disable, 01: default)
    byte 5: RMC interval = 01 (0 - 255, 00: disable, 01: default)
    byte 6: VTG interval = 00 (0 - 255, 00: disable, 01: default)
    byte 7: ZDA interval = 00 (0 - 255, 00: disable, 01: default)
*/
bool Venus838::cfgNMEAMessage(char* NMEAMessage) {
    char command[9];
    // Message id
    command[0] = 0x08;
    for(int i = 1; i < 8; i++) {
        command[i] = NMEAMessage[i - 1];
    }
    command[8] = 0x00;
    return sendCommand(command, 9);
}

/*
Returns the software version
Buffer length: 
*/
bool Venus838::querySoftwareVersion() {
    if (_debug ) _debugPort.println("Querying software version");

    char command[2] = {0x02, 0x01};
    bool status = sendCommand(command, 2);

    char response[21];
    getResponse(response, 21);

    // TO PARSE LATER ON
    if (_debug ) { _debugPort.print("Software version: "); printBuffer(response, 21), _debugPort.print("\n"); }

    return status;

}



bool Venus838::sendCommand(char* command, uint commandSize) {
    // See https://github.com/reed-foster/Venus838/blob/master/doc/gpscommands.md
    // Create command buffer
    char buffer[commandSize + 7];
    
    // buffer[0], buffer[1]: 0xA0, 0xA1
    buffer[0] = (char) 0xA0;
    buffer[1] = (char) 0xA1;

    // buffer[2], buffer[3]: payload size
    buffer[2] = (char) commandSize >> 8;
    buffer[3] = (char) commandSize;

    // buffer[4],.., buffer[4 + size of command - 1]: payload
    for(int i = 0; i < commandSize; i++) {
        buffer[i + 4] = command[i];
    }

    // buffer[4 + size of command]: checksum
    buffer[commandSize + 4] = 0x00;
    for(uint i = 0; i < commandSize; i++) {
        buffer[4 + commandSize] ^= command[i]; // ^ is the bitwise XOR operator
    }

    // buffer[5 + size of command], buffer[6 + size of command]: 0x0D, 0x0A
    buffer[commandSize + 5] = 0x0D;
    buffer[commandSize + 6] = 0x0A;

    if (_debug ) { _debugPort.print("Command sent: "); printBuffer(buffer, commandSize + 7); _debugPort.print("\n"); }

    char response[9];

    _gpsPort.write(buffer, commandSize + 7);

    getResponse(response, 9);

    // Get 5th element of response: ACK or NACK
    if ( response[4] != ACK ) {
        if (_debug ) { _debugPort.println("Command failed. Trying again"); }
        while(!_gpsPort.available()) {}
        _gpsPort.write(buffer, commandSize + 7);
        getResponse(response, 9);
    }

    return response[4] == ACK ;

}

// Gets the response from the GPS
void Venus838::getResponse(char* buffer, uint bufferSize) {

    if ( _debug ) _debugPort.println("Checking response");

    uint start = millis();
    // Read gps port and update buffer until 0xA0 and 0xA1 received // && response[messageSize + 5] != 0x0D && response[messageSize + 6] != 0x0A
    while ( !(buffer[0] == 0xA0 && buffer[1] == 0xA1 && buffer[bufferSize - 2] == 0x0D && buffer[bufferSize - 1] == 0x0A) && millis() - start < _timeout ) {
        for(uint i = 0; i < bufferSize - 1; i++) {
            buffer[i] = buffer[i + 1];
        }

        while(!_gpsPort.available()) {}
        buffer[bufferSize - 1] = _gpsPort.read();
        //if ( _debug ) { _debugPort.print("PACKET: "); printBuffer(buffer, bufferSize); _debugPort.print("\n"); }
    }

    if ( _debug && millis() - start >= _timeout ) { _debugPort.println("Timeout reached"); }
    if ( _debug ) { _debugPort.print("Returned packet: "); printBuffer(buffer, bufferSize); _debugPort.print("\n"); }    
}


/*
bool Venus838::getCommandResult() {
    if ( _debug ) _debugPort.println("Checking command result");

    char buffer[5] = {0x00, 0x00, 0x00, 0x00, 0x00};


    // Read gps port and update buffer until 0xA0 and 0xA1 received // && response[messageSize + 5] != 0x0D && response[messageSize + 6] != 0x0A
    for( uint start = millis(); millis() - start < _timeout; ) {
        while (_gpsPort.available()) {
            buffer[0] = buffer[1];
            buffer[1] = buffer[2];
            buffer[2] = buffer[3];
            buffer[3] = buffer[4];
            buffer[4] = _gpsPort.read();
            if ( buffer[0] == 0xA0 ) return buffer[4] == ACK ;
        }
    }

    if ( _debug  ) { _debugPort.println("Timeout reached"); }

    return buffer[4] == ACK;
}
*/

void Venus838::printBuffer(char* buffer, uint bufferSize) {
    char hexval[4];
    for(uint i = 0; i < bufferSize; i++) {
        sprintf(hexval, "0x%02X", buffer[i]);
        _debugPort.print(hexval);
        if (i < bufferSize - 1) {
            _debugPort.print(", ");
        }
    }
}