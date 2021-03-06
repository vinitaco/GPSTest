#include "NMEAGPS.h"

class Venus838 : public NMEAGPS {
    public:
        Venus838(HardwareSerial &gpsPort, HardwareSerial &debugPort, bool debug);
        void initialize(int baudrate, uint8_t updaterate);
        bool setBaudRate(int baudrate);
        int getBaudRate();
        bool setUpdateRate(uint8_t updaterate);
        uint8_t getUpdateRate();
        bool cfgWAAS(bool enable);
        bool cfgNavigationMode(bool carMode);
        bool cfgNMEAMessage(char* NMEAMessage);
        bool querySoftwareVersion();
        bool setFactoryDefaults(bool reboot);


    
    private:
        void sendCommand(char* command, uint commandSize);
        bool getResponse();
        void getResponse(char* buffer, uint bufferSize);
        void printBuffer(char* buffer, uint bufferSize);
        HardwareSerial &_gpsPort;
        HardwareSerial &_debugPort;
        bool _debug;
        uint _timeout = 500;
        
        char ACK = 0x83;
        char NACK = 0x84;
};