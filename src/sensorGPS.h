// based on nmea from madflight
// https://github.com/qqqlab/madflight/blob/main/src/gps/nmea/gps_nmea_pubx_parser.h

#include <stdint.h>
#include <limits.h>  //for LONG_MIN
#include <Arduino.h>

// #define DEBUG

class sensorGPS {
  public:
    sensorGPS() {
      _ptr = _buffer;
      if (_bufferLen) {
        *_ptr = '\0';
        _buffer[_bufferLen - 1] = '\0';
      }
      
      clear();
    }

    float altitude() {
      return _alt / 1000.0;
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
    uint32_t failed_over() {
      return _failed_over;
    }
    // 0:no fix, 1:fix 2:2D fix, 3:3D fix
    int32_t fix() {
      return _fix;
    }
    int32_t fixAge() {
      return _fixAge;
    }
    // horizontal accuracy estimate in millimeters
    int32_t hacc() {
      return _hacc;
    }
    // HDOP Horizontal Dilution of Precision
    int32_t hdop() {
      return _hdop;
    }
    // latitude in degrees * 10e7
    float latitude() {
      return _lat / 10000000.0;
    }
    // longitude in degrees * 10e7
    float longitude() {
      return _lon / 10000000.0;
    }
    // number of satellites
    int32_t satellites() {
      return _satellites;
    }
    // geoid separation: difference between ellipsoid and mean sea level in millimetres
    int32_t sep() {
      return _sep;
    }
    // speed over ground in mm/s
    int32_t sog() {
      return _sog;
    }
    // TDOP Time Dilution of Precision
    int32_t tdop() {
      return _tdop;
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
    int32_t vdop() {
      return _vdop;
    }
    // VDOP Vertical Dilution of Precision
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
    uint32_t processed() {
      return _processed;
    }
    uint32_t processed_over() {
      return _processed_over;
    }
    uint32_t processedChecksum() {
      return _processedChecksum;
    }
    uint32_t processedChecksum_over() {
      return _processedChecksum_over;
    }
    uint32_t processedFix() {
      return _processedFix;
    }
    uint32_t processedFix_over() {
      return _processedFix_over;
    }
    uint32_t processedGGA() {
      return _processedGGA;
    }
    uint32_t processedGGA_over() {
      return _processedGGA_over;
    }
    uint32_t processedPUBX() {
      return _processedPUBX;
    }
    uint32_t processedPUBX_over() {
      return _processedPUBX_over;
    }
    uint32_t processedRMC() {
      return _processedRMC;
    }
    uint32_t processedRMC_over() {
      return _processedRMC_over;
    }
    uint32_t processedZDA() {
      return _processedZDA;
    }
    uint32_t processedZDA_over() {
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

    bool parseLatLngRef(const char* &s, int32_t &v);
    bool parseTimeRef(const char* &s, int32_t &v);
    //move s to next field, set s=nullpointer if no more fields
    bool skipFieldRef(const char* &s);
    //chop an integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    bool parseUInt8Ref(const char * &s, uint8_t &v);
    //chop an integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    bool parseUInt16Ref(const char * &s, uint16_t &v);
    //chop an integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    bool parseIntRef(const char * &s, int32_t &v);
    //chop an integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    bool parseInt8Ref(const char * &s, int8_t &v);
    //chop a float as scaled integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    //sets v=LONG_MIN if field was empty
    bool parseFloatRef(const char * &s, int scaledigits, int32_t &v);

    bool processGGA(const char *s);
    bool processPUBX00(const char* s);
    bool processRMC(const char* s);
    bool processZDA(const char* s);
    bool testChecksum(const char* s);

    // Sentence buffer and associated pointers
    static const uint16_t _bufferLen = 255;
    char _buffer[_bufferLen];
    char *_ptr;
    
    int32_t _alt; // Altitude in millimeters above user datum ellipsoid (= MSL Altitude + Geoid Separation from GGA message)
    char _checksumRcv[2];
    char _checksumCalc[3];
    int32_t _cog; // course over ground in degrees * 1000
    uint8_t _day; // day
    int32_t _date; // date as DDMMYY
    int32_t _fix;  // 0:no fix, 1:fix 2:2D fix, 3:3D fix
    uint32_t _fixAge;  // age of last fix
    int32_t _hacc; // horizontal accuracy estimate in millimeters
    int32_t _hdop; // HDOP Horizontal Dilution of Precision
    int32_t _lat;  // latitude in degrees * 10e7
    int32_t _lon;  // longitude in degrees * 10e7
    uint8_t _month; // month
    int32_t _satellites;  // number of satellites
    int32_t _sep; // geoid separation: difference between ellipsoid and mean sea level in millimetres
    int32_t _sog;  // speed over ground in mm/s
    int32_t _tdop; // TDOP Time Dilution of Precision
    int32_t _time; // time in milliseconds since midnight UTC
    int8_t _time_offset_hours; // time zone offset from GMT in hours
    int8_t _time_offset_minutes; // time zone offset from GMT in minutes
    int32_t _vacc; // Vertical accuracy estimate in millimeters
    int32_t _vdop; // VDOP Vertical Dilution of Precision
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
    uint32_t _processedPUBX = 0;
    uint32_t _processedPUBX_over = 0;
    uint32_t _processedRMC = 0;
    uint32_t _processedRMC_over = 0;
    uint32_t _processedZDA = 0;
    uint32_t _processedZDA_over = 0;

    bool _use_only_pubx00;
};