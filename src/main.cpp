
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include  <iomanip>

#include "gps.hpp"



int main( int argc, char **argv ) {
    const char *GPS_SERIALS[] = { "/dev/ttyACM0" , "/dev/ttyACM1" } ;

    GPS gps( GPS_SERIALS, 2 ) ;
    if( gps ) {
        // reboot( gpsDev, false ) ;
        // turnOffMessage( gpsDev, NMEA_MSG, NMEA_MSG_GGA ) ;
        // turnOnMessage( gpsDev, NMEA_MSG, NMEA_MSG_ZDA ) ;
        gps.turnOffAllMessages() ;
        gps.turnOnMessage( GPS::NMEA_MSG, GPS::NMEA_MSG_RMC ) ;

        gps.readDevice() ;
    }
	return 0 ;
}
