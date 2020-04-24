

#include <fcntl.h>
#include <unistd.h>
#include <cstring>

#include "gps.hpp"

int parseFloat( double &val, const char *p, int len ) ;
double convertToDegrees( double degMin ) ;

//
// A list of all the NMEA message IDs ( so we can turn them off )
//
constexpr uint8_t ALL_NMEA_MSGS[] = {
        GPS::NMEA_MSG_DTM ,
        GPS::NMEA_MSG_GBQ ,
        GPS::NMEA_MSG_GBS ,
        GPS::NMEA_MSG_GGA ,
        GPS::NMEA_MSG_GLL ,
        GPS::NMEA_MSG_GLQ ,
        GPS::NMEA_MSG_GNQ ,
        GPS::NMEA_MSG_GNS ,
        GPS::NMEA_MSG_GPQ ,
        GPS::NMEA_MSG_GRS ,
        GPS::NMEA_MSG_GSA ,
        GPS::NMEA_MSG_GST ,
        GPS::NMEA_MSG_GSV ,
        GPS::NMEA_MSG_RMC ,
        GPS::NMEA_MSG_TXT ,
        GPS::NMEA_MSG_VLM ,
        GPS::NMEA_MSG_VTG ,
        GPS::NMEA_MSG_ZDA 
    } ;

/*
 * Create a new instance of the GPS, search (in order) the
 * given serial devices for anything available to open. The
 * first openable serial port is considered to be a GPS device
 * 
 * @param serialDevices an array of const device names (e.g. /dev/ttyACM0 )
 * #param numDevices count of possible devices to search
 */
GPS::GPS( const char *serialDevices[], int numDevices ) {
                        
    for( int i=0 ; i<numDevices; i++ ) {
        gpsDev = open( serialDevices[i], O_RDWR  ) ;
        if( gpsDev > 0 ) break ;
    }
    // Couldn't get anything ?
    if (gpsDev < 0) {
        perror("open: ");
    }

    gps_reader_thread = nullptr ;
    // Assume we're in error until we've got a good signal
    errorFlag = true ;    
}

/*
 * The destructor makes a request to shutdown the background thread
 * If the thread is running, wait for it to complete. Then close the
 * file.
 */
GPS::~GPS() {
    stop() ;
    if( gpsDev ) {
        close( gpsDev ) ;
        gpsDev = 0 ;
    }
}

/*
 * This starts the background thread to read the GPS. It's not 
 * part of the constructor so that initialization of reading
 * is under external control. (and it's really bad to start a
 * thread in a constructor)
 * 
 * This clears out any pending messages in the buffer.
 */
void GPS::start() {
    threadAlive = true ;
    
    char buffer[32] ;
    int n = read( gpsDev, buffer, sizeof(buffer) ) ; 
    while( n==sizeof(buffer) ) {
        n = read( gpsDev, buffer, sizeof(buffer) ) ; 
    }

    gps_reader_thread = new std::thread( &GPS::readDevice, this ) ;    
}

/*
 * This stop the background thread which reads the GPS. 
 * This isn't strictly thread safe ( don't call stop()
 * from many threads! )
 */
void GPS::stop() {
    threadAlive = false ;
    if( gps_reader_thread != nullptr ) {
        gps_reader_thread->join() ;
        delete gps_reader_thread ;
        gps_reader_thread = nullptr ;
    }
}

/*
 * Parse out the RMC message, which is the minumum message 
 * supported by a GPS receiver. This method is called from the 
 * background thread.
 * 
 * @param msg the full text of an RMC message "$GPRMC,012345.00,...""
 */
void GPS::parseRMC( const char *msg ) {
    int l = strlen( msg ) ;
    if( l>20 ) {
        // Make sure we start with the proper msg type
        if( msg[0] == '$' &&
            msg[3] == 'R' &&
            msg[4] == 'M' &&
            msg[5] == 'C' &&
            msg[6] == ',' ) {

            // Skip the msg name
            const char *p = msg+7 ;
            l-= 7 ;

            // Read 1 float number - the timestamp in GPS time
            double tim ;
            int n = parseFloat( tim, p, l ) ;
            p += n ;
            l -= n ;

            // Check the valid flag A=OK, V=bad (huh?)
            if( *p++ != 'A' ) {
                errorFlag = true ;
                return ;
            }
            if( *p++ != ',' ) {
                errorFlag = true ;
                return ;
            }

            // Read another float
            double latitude ;
            n = parseFloat( latitude, p, l ) ;
            p += n ;
            // This indicates sign (Australia is negative)
            if( *p++ == 'S' ) {
                latitude = -latitude ;
            }
            if( *p++ != ',' ) {
                errorFlag = true ;
                return ;
            }
            // And one more float
            double longitude ;
            n = parseFloat( longitude, p, l ) ;
            p += n ;
            // West is negative
            if( *p++ == 'W' ) {
                longitude = -longitude ;
            }

            // If we got here - the data is valid
            errorFlag = false ;

            // Set the values into class member (volatile)
            this->latitude = convertToDegrees( latitude ) ;
            this->longitude = convertToDegrees( longitude ) ;
        }
    }
}

double convertToDegrees( double degMin ) {
    int deg = (int)( degMin / 100.0 ) ;
    double min = degMin - ( deg * 100 ) ;
    return deg + min / 60.0 ;
}

// Helper method to read a float from a string - like atof() 
// without the null terminator
int parseFloat( double &result, const char *p, int len ) {
    double val = 0 ;
    double scale = 0 ;
    int n = 0 ;
    while( *p != ',' && --len>0 ) {
        if( *p == '.' ) {
            scale = 0.1 ;
        } else {
            val *= 10.0 ;
            val += ( *p - '0' ) ;
        }
        if( scale > 0 ) {
            scale *= 10.0 ;
        }
        p++ ;
        n++ ;
    }
    result = val / scale ;
    return n + 1 ;
}

/*
 * This is the background thread method, which checks the 
 * device for new data. Once it detects a full message
 * it's sent for parsing (see parseRMC)
 * It will run until the flag threadAlive is cleared
 */
void GPS::readDevice() {

    char buffer[256] ;
    char *buf_end = buffer + sizeof(buffer) ;
    char *writeStart = buffer ; 
    while( threadAlive )  {
        // read from gpsDev into availableSpace plus maximum left in buffer
        int n = read( gpsDev, writeStart, buf_end - writeStart ) ; 
        if( n<0 ) {
            // If we get an error, we'll shutdown. Is that right?
            errorFlag = true ;
            perror( "read: " ) ;
            stop() ;
            continue ;
        }

        // replace /r & /n in the just read text
        char *p = writeStart ;
        writeStart += n ;
        while( p < writeStart ) {
            if( *p == '\r' || *p == '\n' ) {
                *p = 0 ;
            } ;
            p++ ;
        }

        // Move p to start of first message ( a $ sign )
        p = buffer ;
        while( p < writeStart ) {
            if( *p == '$' ) break ;
            p++ ;
        }

        while( *p == '$' && p<writeStart ) {
            // std::cout << p << std::endl ;
            parseRMC( p ) ;
            int l = 0 ;
            while( p < writeStart ) {
                p++ ;
                if( *p=='$' ) break ;
            }

            // Make sure the next string is completely read
            char *q = p ;
            while( q < writeStart ) {
                q++ ;
                if( *q==0 ) break ;
            }
            if( q == writeStart ) {
                break ;
            }
        }

        int len = writeStart - p ;
        memcpy( buffer, p, len ) ;
        writeStart = buffer + len ;
    } 
}


void GPS::reboot( bool cold ) {
	
    uint8_t payload[4] ;
    // This is a 16 bit jobby - why? WTFK
    payload[0] = cold ? 0xff : 0x01 ;   // HOT start 0x0000 is not done
    payload[1] = cold ? 0xff : 0x00 ;   
    payload[2] = 0x01 ;                 // nicest start - see docs for details
    payload[3] = 0x00 ;

    sendMessage( PUBX_MSG_CFG, 4, sizeof(payload), payload ) ;
}


void GPS::turnOffAllMessages() {
    for( auto msgId : ALL_NMEA_MSGS ) {
        turnOffMessage( NMEA_MSG, msgId ) ;
    }
}

void GPS::turnOffMessage( uint8_t msgClass, uint8_t msgId ) {
	
    uint8_t payload[3] ;
    payload[0] = msgClass ;
    payload[1] = msgId ;
    payload[2] = 0x00 ;

    sendMessage( PUBX_MSG_CFG, 1, sizeof(payload), payload ) ;
}


void GPS::turnOnMessage( uint8_t msgClass, uint8_t msgId ) {
	
    uint8_t payload[3] ;
    payload[0] = msgClass ;
    payload[1] = msgId ;
    payload[2] = 0x01 ;

    sendMessage( PUBX_MSG_CFG, 1, sizeof(payload), payload ) ;
}


void GPS::sendMessage( uint8_t msgClass, uint8_t msgId, uint16_t len, uint8_t *payload ) {

    uint8_t buffer[1024] ;
    buffer[0] = 0xb5 ;
    buffer[1] = 0x62 ;
    buffer[2] = msgClass ;
    buffer[3] = msgId ;
    buffer[4] = len & 0xff ;
    buffer[5] = len >> 8 ;
    
    std::memcpy( buffer+6, payload, len ) ;
    
    uint8_t a = 0 ;
    uint8_t b = 0 ;

    uint8_t *p = buffer+2 ;
    for( int i=0 ; i<4+len ; ++i ) {
        a += *p++ ;
        b += a ;
    }

    *p++ = a ;
    *p++ = b ;

    *p = 0 ;

    int n = write( gpsDev, buffer, p-buffer ); 
}

