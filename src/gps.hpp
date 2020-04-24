
#pragma once

#include <thread>        

class GPS {

public :
    //
    // NMEA standard messages e.g. VTG is F0 05
    //
    static constexpr uint8_t NMEA_MSG = 0xF0; 
    static constexpr uint8_t NMEA_MSG_DTM = 0x0A; //DTM
    static constexpr uint8_t NMEA_MSG_GBQ = 0x44; //GBQ
    static constexpr uint8_t NMEA_MSG_GBS = 0x09; //GBS
    static constexpr uint8_t NMEA_MSG_GGA = 0x00; //GGA
    static constexpr uint8_t NMEA_MSG_GLL = 0x01; //GLL
    static constexpr uint8_t NMEA_MSG_GLQ = 0x43; //GLQ
    static constexpr uint8_t NMEA_MSG_GNQ = 0x42; //GNQ
    static constexpr uint8_t NMEA_MSG_GNS = 0x0D; //GNS
    static constexpr uint8_t NMEA_MSG_GPQ = 0x40; //GPQ
    static constexpr uint8_t NMEA_MSG_GRS = 0x06; //GRS
    static constexpr uint8_t NMEA_MSG_GSA = 0x02; //GSA
    static constexpr uint8_t NMEA_MSG_GST = 0x07; //GST
    static constexpr uint8_t NMEA_MSG_GSV = 0x03; //GSV
    static constexpr uint8_t NMEA_MSG_RMC = 0x04; //RMC
    static constexpr uint8_t NMEA_MSG_TXT = 0x41; //TXT
    static constexpr uint8_t NMEA_MSG_VLM = 0x0F; //VLW
    static constexpr uint8_t NMEA_MSG_VTG = 0x05; //VTG
    static constexpr uint8_t NMEA_MSG_ZDA = 0x08; //ZDA


    //
    // NMEAX messages
    //
    static constexpr uint8_t NMEAX_MSG = 0xF1; 

    static constexpr uint8_t NMEAX_MSG_CFG = 0x41; 
    static constexpr uint8_t NMEAX_MSG_POS = 0x00; 
    static constexpr uint8_t NMEAX_MSG_RAT = 0x40; 
    static constexpr uint8_t NMEAX_MSG_SYS = 0x03; 
    static constexpr uint8_t NMEAX_MSG_TIM = 0x04; 


    static constexpr uint8_t PUBX_MSG_NAV = 0x01 ; // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
    static constexpr uint8_t PUBX_MSG_RXM = 0x02 ; // Receiver Manager Messages: Satellite Status, RTC Status
    static constexpr uint8_t PUBX_MSG_INF = 0x04 ; // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
    static constexpr uint8_t PUBX_MSG_ACK = 0x05 ; // Ack/Nak Messages: Acknowledge or Reject messages to CFG input messages
    static constexpr uint8_t PUBX_MSG_CFG = 0x06 ; // Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
    static constexpr uint8_t PUBX_MSG_UPD = 0x09 ; // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
    static constexpr uint8_t PUBX_MSG_MON = 0x0A ; // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
    static constexpr uint8_t PUBX_MSG_AID = 0x0B ; // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
    static constexpr uint8_t PUBX_MSG_TIM = 0x0D ; // Timing Messages: Time Pulse Output, Time Mark Results
    static constexpr uint8_t PUBX_MSG_ESF = 0x10 ; // External Sensor Fusion Messages: External Sensor Measurements and Status Information
    static constexpr uint8_t PUBX_MSG_MGA = 0x13 ; // Multiple GNSS Assistance Messages: Assistance data for various GNSS
    static constexpr uint8_t PUBX_MSG_LOG = 0x21 ; // Logging Messages: Log creation, deletion, info and retrieval
    static constexpr uint8_t PUBX_MSG_SEC = 0x27 ; // Security Feature Messages
    static constexpr uint8_t PUBX_MSG_HNR = 0x28 ; // High Rate Navigation Results Messages: High rate time, position, speed, heading


protected:
    volatile int threadAlive ;
    int gpsDev ;
    int errorFlag ;
    void sendMessage( uint8_t cls, uint8_t subcls, uint16_t len, uint8_t *payload ) ;

    double longitude ;
    double latitude ;
    std::thread gps_reader_thread ;

    void parseRMC( const char *msg ) ;
    void readDevice() ;


public :
    GPS( const char *serialDevices[], int numDevices ) ;
    virtual ~GPS() ;
    
    void start() ;

    int operator !() const { return errorFlag ; }
    operator int() const { return gpsDev ; }

    void turnOffMessage(  uint8_t msgClass, uint8_t msgId ) ;
    void turnOnMessage( uint8_t msgClass, uint8_t msgId ) ;
    void reboot( bool cold=false ) ;
    void turnOffAllMessages( ) ;

    double getLongitude() { return this->longitude ; }
    double getLatitude() { return this->latitude ; }
} ;


