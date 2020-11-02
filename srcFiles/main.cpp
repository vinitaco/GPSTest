#include <NMEAGPS.h>

//======================================================================
//  Program: NMEAsimple.ino
//
//  Description:  This program shows simple usage of NeoGPS
//
//  Prerequisites:
//     1) NMEA.ino works with your device (correct TX/RX pins and baud rate)
//     2) At least one of the RMC, GGA or GLL sentences have been enabled in NMEAGPS_cfg.h.
//     3) Your device at least one of those sentences (use NMEAorder.ino to confirm).
//     4) LAST_SENTENCE_IN_INTERVAL has been set to one of those sentences in NMEAGPS_cfg.h (use NMEAorder.ino).
//     5) LOCATION and ALTITUDE have been enabled in GPSfix_cfg.h
//
//  'Serial' is for debug output to the Serial Monitor window.
//
//  License:
//    Copyright (C) 2014-2017, SlashDevin
//
//    This file is part of NeoGPS
//
//    NeoGPS is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    NeoGPS is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with NeoGPS.  If not, see <http://www.gnu.org/licenses/>.
//
//======================================================================

//#include <GPSport.h>

#define gpsPort Serial2
#define GPS_PORT_NAME "Serial2"
#define DEBUG_PORT Serial

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

void sendCommand(uint8_t* Command, uint8_t sizeOfArray) {
  gpsPort.write(Command, sizeOfArray);
  gpsPort.flush();
  delay(10);
}

void setup() {
    DEBUG_PORT.begin(9600);
    while (!Serial);
    DEBUG_PORT.print( F("NMEAsimple.INO: started\n") );

    gpsPort.begin(230400);
    uint8_t baudRateCmd[11] = {0xA0, 0xA1, 0x00, 0x04, 0x05, 0x00, 0x05, 0x00, 0x00, 0x0D, 0x0A};
    uint8_t refreshRateCmd[10] = {0xA0, 0xA1, 0x00, 0x03, 0x0E, 0x14, 0x00, 0x1A, 0x0D, 0x0A};
    
    sendCommand(baudRateCmd, sizeof(baudRateCmd));
    
    sendCommand(refreshRateCmd, sizeof(refreshRateCmd));
    gpsPort.begin(115200);

    DEBUG_PORT.print("GPS Availability: ");
    DEBUG_PORT.println(gps.available(gpsPort));
}

//--------------------------

void loop() {
    
    if (gps.available( gpsPort )) {

      fix = gps.read();
      
      /*
      DEBUG_PORT.print( F("Location: ") );
      if (fix.valid.location) {
          DEBUG_PORT.print( fix.latitude(), 6 );
          DEBUG_PORT.print( ',' );
          DEBUG_PORT.print( fix.longitude(), 6 );
      }
      */

      DEBUG_PORT.print( F(", Altitude: ") );
      if (fix.valid.altitude) {
          DEBUG_PORT.print( fix.altitude(), 6 );
      }
      DEBUG_PORT.println( fix.altitude(), 6 );
    }
}
