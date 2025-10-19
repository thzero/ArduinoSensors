#ifndef _SENSOR_GPS_H
#define _SENSOR_GPS_H

// based on nmea from madflight
// https://github.com/qqqlab/madflight/blob/main/src/gps/nmea/gps_nmea_pubx_parser.h

#include <stdint.h>
#include <Arduino.h>
#include <limits.h>

#include "sensorBase.h"

// #define DEBUG

class sensorGPS {
  public:
    sensorGPS() {
      // _ptr = _buffer;
      // if (_bufferLen) {
      //   *_ptr = '\0';
      //   _buffer[_bufferLen - 1] = '\0';
      // }
      
      // clear();
    }

    virtual byte setup(HardwareSerial& port, int baud);

    int32_t altitude() {
      // return _alt / 1000.0;
      return _alt;
    }
    int32_t altitudeFiltered() {
      return altitude(); // TODO
    }
    int32_t altitudeRaw() {
      return _alt;
    }
    bool altitudeValid() {
      return _alt != LONG_MIN;
    }
    int32_t altitudeMSL() {
      // return _altMSL / 1000.0;
      return _altMSL;
    }
    int32_t altitudeMSLFiltered() {
      return altitudeMSL(); // TODO
    }
    int32_t altitudeMSLRaw() {
      return _altMSL;
    }
    bool altitudeMSLValid() {
      return _altMSL != LONG_MIN;
    }
    // geoid separation: difference between ellipsoid and mean sea level in millimetres
    int32_t altitudeSep() {
      return _altSep / 1000.0;
    }
    int32_t altitudeSepFiltered() {
      return altitudeSep(); // TODO
    }
    int32_t altitudeSepRaw() {
      return _altSep;
    }
    bool altitudeSepValid() {
      return _altSep != LONG_MIN;
    }
    char* checksumCalc() {
      return _checksumCalc;
    }
    char* checksumRcv() {
      return _checksumRcv;
    }
    // course over ground in degrees * 1000
    int32_t cog() {
      return _cog;
    }
    // date as DDMMYY
    int32_t dateDDMMYY() {
      return _date;
    }
    // 0: Fix not valid 1: StandardGPS fix (2D/3D) 2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
    // 3: Not applicable 4: RTK Fixed, xFill, RTX 5: RTK Float, OmniSTAR XP/HP, Location RTK, QZSS CLAS 6: INS Dead reckoning
    int8_t fix() {
      return _fix;
    }
    bool fixValid() {
      return _fix != SCHAR_MIN;
    }
    int32_t fixAge() {
      return _fixAge;
    }
    int32_t fixAgeFiltered() {
      return fixAge(); // TODO
    }
    bool fixAgeValid() {
      return true;
    }
    // 0: no fix, 1: 2D fix, 2: 3D fix of last received GSA message
    int8_t fixType() {
      return _fixType;
    }
    bool fixTypeValid() {
      return _fixType != SCHAR_MIN;
    }
    // horizontal accuracy estimate in millimeters
    int32_t hacc() {
      return _hacc;
    }
    // HDOP Horizontal Dilution of Precision
    int16_t hdop() {
      // return _hdop / 1000.0;
      return _hdop;
    }
    int16_t hdopFiltered() {
      return hdop(); // TODO
    }
    int16_t hdopRaw() {
      return _hdop;
    }
    bool hdopValid() {
      return _hdop >= 0;
    }
    // latitude in degrees
    int32_t latitude() {
      // return _lat / 10000000.0;
      return _lat;
    }
    int32_t latitudeFiltered() {
      return latitude(); // TODO
    }
    // latitude in degrees * 10e7
    int32_t latitudeRaw() {
      return _lat;
    }
    bool latitudeValid() {
      return _lat != LONG_MIN;
    }
    // longitude in degrees
    int32_t longitude() {
      // return _lon / 10000000.0;
      return _lon;
    }
    int32_t longitudeFiltered() {
      return longitude(); // TODO
    }
    // longitude in degrees * 10e7
    int32_t longitudeRaw() {
      // return _lon / 10000000.0;
      return _lon;
    }
    bool longitudeValid() {
      return _lon != LONG_MIN;
    }
    // PDOP Position Dilution of Precision
    int16_t pdop() {
      // return _pdop / 1000.0;
      return _pdop;
    }
    int16_t pdopFiltered() {
      return pdop(); // TODO
    }
    int16_t pdopRaw() {
      return _pdop;
    }
    bool pdopValid() {
      return _pdop >= 0;
    }
    // number of satellites
    uint16_t satellites() {
      return _satellites;
    }
    uint16_t satellitesFiltered() {
      return satellites(); // TODO
    }
    bool satellitesValid() {
      return _satellites >= 0;
    }
    // speed over ground in mm/s
    int32_t sog() {
      return _sog;
    }
    // TDOP Time Dilution of Precision
    uint16_t tdop() {
      // return _tdop / 1000.0;
      return _tdop;
    }
    uint16_t tdopFiltered() {
      return tdop(); // TODO
    }
    uint16_t tdopRaw() {
      return _tdop;
    }
    bool tdopValid() {
      return _tdop >= 0;
    }
    // time in milliseconds since midnight UTC
    int32_t timeMs() {
      return _time;
    }
    // Vertical accuracy estimate in millimeters
    int32_t vacc() {
      return _vacc;
    }
    // VDOP Vertical Dilution of Precision
    uint16_t vdop() {
      // return _vdop / 1000.0;
      return _vdop;
    }
    uint16_t vdopFiltered() {
      return vdop(); // TODO
    }
    int16_t vdopRaw() {
      return _vdop;
    }
    bool vdopValid() {
      return _vdop >= 0;
    }
    int32_t veld() {
      return _veld;
    }

    // Get received NMEA sentence
    const char* getSentence(void) const {
      return _buffer;
    }
    
    uint32_t failed() {
      return _failed;
    }
    uint32_t failedOver() {
      return _failed_over;
    }
    uint32_t processed() {
      return _processed;
    }
    uint32_t processedOver() {
      return _processed_over;
    }
    uint32_t processedChecksum() {
      return _processedChecksum;
    }
    uint32_t processedChecksumOver() {
      return _processedChecksum_over;
    }
    uint32_t processedFix() {
      return _processedFix;
    }
    uint32_t processedFixOver() {
      return _processedFix_over;
    }
    uint32_t processedGGA() {
      return _processedGGA;
    }
    uint32_t processedGGAOver() {
      return _processedGGA_over;
    }
    uint32_t processedGSA() {
      return _processedGSA;
    }
    uint32_t processedGSAOver() {
      return _processedGSA_over;
    }
    uint32_t processedPUBX() {
      return _processedPUBX;
    }
    uint32_t processedPUBXOver() {
      return _processedPUBX_over;
    }
    uint32_t processedRMC() {
      return _processedRMC;
    }
    uint32_t processedRMCOver() {
      return _processedRMC_over;
    }
    uint32_t processedZDA() {
      return _processedZDA;
    }
    uint32_t processedZDAOver() {
      return _processedZDA_over;
    }

    // Send a NMEA sentence to the GNSS receiver. The sentence must start with `$`; the checksum and `\r\n` terminators will be appended automatically.
    static Stream& sendSentence(Stream& s, const char* sentence) {
      char checksum[3];
      generateChecksum(sentence, checksum);
      checksum[2] = '\0';
      s.print(sentence);
      s.print('*');
      s.print(checksum);
      s.print("\r\n");
      return s;
    }

    void clear(void);
    // process a character, returns true if position was updated
    bool process(char c);

  protected:
    static const char* generateChecksum(const char* s, char* checksum) {
      uint8_t c = 0;
      
      // Initial $ is omitted from checksum, if present ignore it.
      if (*s == '$')
        ++s;

      while (*s != '\0' && *s != '*')
        c ^= *s++;

      if (checksum) {
        checksum[0] = toHex(c / 16);
        checksum[1] = toHex(c % 16);
      }

      return s;
    }

    static inline bool isEndOfFields(char c) {
      return c == '*' || c == '\0' || c == '\r' || c == '\n';
    }

  private:
    static char toHex(uint8_t nibble) {
      if (nibble >= 10)
        return nibble + 'A' - 10;
      return nibble + '0';
    }

    void handleFix(int32_t fix);

    //move s to next field, set s=nullpointer if no more fields
    bool parseFloatRef(const char * &s, int scaledigits, int8_t &v);
    bool parseFloatRef(const char * &s, int scaledigits, int16_t &v);
    bool parseFloatRef(const char * &s, int scaledigits, int32_t &v);
    bool parseFloatRef(const char * &s, int scaledigits, uint16_t &v);
    //chop an integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    bool parseIntRef(const char * &s, int8_t &v);
    bool parseIntRef(const char * &s, int16_t &v);
    bool parseIntRef(const char * &s, int32_t &v);
    bool parseIntRef(const char * &s, uint8_t &v);
    bool parseIntRef(const char * &s, uint16_t &v);
    bool parseIntRef(const char * &s, uint32_t &v);
    bool parseLatLngRef(const char* &s, int32_t &v);
    //chop an integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    bool parseUInt8Ref(const char * &s, uint8_t &v);
    //chop an integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    bool parseUInt16Ref(const char * &s, uint16_t &v);
    //chop a float as scaled integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    //sets v=LONG_MIN if field was empty
    bool parseTimeRef(const char* &s, int32_t &v);
    bool skipFieldRef(const char* &s);

    bool processGGA(const char *s);
    bool processGSA(const char *s);
    bool processPUBX00(const char* s);
    bool processRMC(const char* s);
    bool processZDA(const char* s);
    bool testChecksum(const char* s);

    // Sentence buffer and associated pointers
    static const uint16_t _bufferLen = 255;
    char _buffer[_bufferLen];
    char *_ptr;
    
    int32_t _alt; // Altitude in millimeters above user datum ellipsoid (= MSL Altitude + Geoid Separation from GGA message)
    int32_t _altMSL;
    int32_t _altSep; // geoid separation: difference between ellipsoid and mean sea level in millimetres
    char _checksumRcv[2];
    char _checksumCalc[3];
    int32_t _cog; // course over ground in degrees * 1000
    uint8_t _day; // day
    int32_t _date; // date as DDMMYY
    // 0: Fix not valid 1: StandardGPS fix (2D/3D) 2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
    // 3: Not applicable 4: RTK Fixed, xFill, RTX 5: RTK Float, OmniSTAR XP/HP, Location RTK, QZSS CLAS 6: INS Dead reckoning 
    int8_t _fix; 
    int32_t _fixAge; // age of last fix
    int8_t _fixType; // 0: no fix, 1: 2D fix, 2: 3D fix
    int32_t _hacc; // horizontal accuracy estimate in millimeters
    int16_t _hdop; // HDOP Horizontal Dilution of Precision
    int32_t _lat; // latitude in degrees * 10e7
    int32_t _lon; // longitude in degrees * 10e7
    uint8_t _month; // month
    int16_t _pdop; // PDOP Position Dilution of Precision
    int32_t _satellites;  // number of satellites
    int32_t _sog; // speed over ground in mm/s
    int16_t _tdop; // TDOP Time Dilution of Precision
    int32_t _time; // time in milliseconds since midnight UTC
    int8_t _time_offset_hours; // time zone offset from GMT in hours
    int8_t _time_offset_minutes; // time zone offset from GMT in minutes
    int32_t _vacc; // Vertical accuracy estimate in millimeters
    int16_t _vdop; // VDOP Vertical Dilution of Precision
    int32_t _veld; // vertical downward speed in mm/s
    uint8_t _year; // year
    
    uint32_t _update_ms_fixAge;
    uint32_t _update_ms; // millisecond timestamp last pos update

    uint32_t _failed = 0;
    uint32_t _failed_over = 0;
    uint32_t _processed = 0;
    uint32_t _processed_over = 0;
    uint32_t _processedChecksum = 0;
    uint32_t _processedChecksum_over = 0;
    uint32_t _processedFix = 0;
    uint32_t _processedFix_over = 0;
    uint32_t _processedGGA = 0;
    uint32_t _processedGGA_over = 0;
    uint32_t _processedGSA = 0;
    uint32_t _processedGSA_over = 0;
    uint32_t _processedPUBX = 0;
    uint32_t _processedPUBX_over = 0;
    uint32_t _processedRMC = 0;
    uint32_t _processedRMC_over = 0;
    uint32_t _processedZDA = 0;
    uint32_t _processedZDA_over = 0;

    bool _use_only_pubx00;
};

#endif