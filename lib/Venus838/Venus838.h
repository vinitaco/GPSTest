#include "NMEAGPS.h"

class Venus838 : public NMEAGPS {
    public:
        Venus838(HardwareSerial &gpsPort, HardwareSerial &debugPort, bool debug);
        void initialize(int baudrate, uint8_t updaterate);
        bool setBaudRate(int baudrate);
        int getBaudRate();
        bool setUpdateRate(uint8_t updaterate);
        bool cfgWAAS(bool enable);
        bool cfgNavigationMode(bool carMode);
        //bool cfgNMEAMessage()
        //char getUpdateRate();
        bool querySoftwareVersion();
        bool setFactoryDefaults(bool reboot);


    
    private:
        bool sendCommand(char* command, uint commandSize);
        void getResponse(char* buffer, uint bufferSize);
        void printBuffer(char* buffer, uint bufferSize);
        HardwareSerial &_gpsPort;
        HardwareSerial &_debugPort;
        bool _debug;
        uint _timeout = 1000;
        
        char ACK = 0x83;
        char NACK = 0x84;
};