
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

        gps.start() ;

        double lon = 0 ;
        double lat = 0 ;
        for( int i=0 ; i<100 ;  ) {
            if( !gps ) continue ;
            double lo = gps.getLongitude() ;
            double la = gps.getLatitude() ;
            if( lo != lon  || la != lat ) {
                i++ ;
                lon = lo ; lat = la ;
                if( gps ) {
                    std::cout << std::setprecision(7) << std::fixed 
                        << lon << " "  << lat << std::endl ;
                }
            }
        }
    }
	return 0 ;
}
