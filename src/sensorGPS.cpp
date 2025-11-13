// based on nmea from madflight
// https://github.com/qqqlab/madflight/blob/main/src/gps/nmea/gps_nmea_pubx_parser.h

#include "sensorGPS.h"

int8_t sensorGPS::setup(HardwareSerial& port, int baud) {
  _ptr = _buffer;
  if (_bufferLen) {
    *_ptr = '\0';
    _buffer[_bufferLen - 1] = '\0';
  }
  
  clear();
  return 0;
}

void sensorGPS::clear(void)
{
  _alt = LONG_MIN;  // altitude in millimeters above user datum ellipsoid (= MSL Altitude + Geoid Separation from GGA message)
  _altMSL = LONG_MIN;  // altitude in millimeters above user datum ellipsoid (= MSL Altitude + Geoid Separation from GGA message)
  _altSep = LONG_MIN; // height above WGS84 Geoid in millimetres
  _cog = LONG_MIN;  // course over ground in degrees * 1000
  _date = 0; // date as DDMMYY
  _day = 0;
  _fix = SCHAR_MIN;  // 0:no fix, 1:fix 2:2D fix, 3:3D fix
  _fixAge = LONG_MIN;
  _fixType = SCHAR_MIN;
  _hacc = LONG_MIN; // horizontal accuracy estimate in millimeters
  _hdop = SHRT_MIN; // HDOP Horizontal Dilution of Precision
  _lat = LONG_MIN;  // Latitude in degrees * 10e7
  _lon = LONG_MIN;  // Longitude in degrees * 10e7
  _month = 0;
  _pdop = SHRT_MIN; // PDOP Position Dilution of Precision
  _satellites = LONG_MIN;  // number of satellites
  _sog = LONG_MIN;  // speed over ground in mm/s
  _tdop = SHRT_MIN; // TDOP Time Dilution of Precision
  _time = 0; // time in milliseconds since midnight UTC
  _time_offset_hours = 0;
  _time_offset_minutes = 0;
  _year = 0;
  _vacc = LONG_MIN; // vertical accuracy estimate in millimeters
  _vdop = SHRT_MIN; // VDOP Vertical Dilution of Precision
  _veld = LONG_MIN; // vertical downward speed in mm/s

  _use_only_pubx00 = false;
  _update_ms = 0; // millisecond timestamp last pos update
  
  memset(_checksumRcv, 0, 2);
  memset(_checksumCalc, 0, 2);
}

// process a character, returns true if position was updated
bool sensorGPS::process(char c)
{
  if (_buffer == nullptr || _bufferLen == 0) 
    return false;
  
  if (c == 0 || c == '\n' || c == '\r') {
// #ifdef DEBUG
    // Serial.print("_buffer: ");
    // Serial.println(_buffer);
// #endif
    _processed++;
    if (_processed == UINT32_MAX) {
      _processed = 0;
      _processed_over++;
    }

    // Terminate buffer then reset pointer
    *_ptr = 0;
    _ptr = _buffer;

    if (_buffer[0] == '$' && testChecksum(_buffer)) {
      // Valid message
// #ifdef DEBUG
      // Serial.println("...valid checksum");
// #endif
      _processedChecksum++;
      if (_processedChecksum == UINT32_MAX) {
        _processedChecksum = 0;
        _processedChecksum++;
      }

      if (_buffer[1] == 'G' && strncmp(&_buffer[3], "GGA,", 4) == 0)
          return processGGA(_buffer);
      if (_buffer[1] == 'G' && strncmp(&_buffer[3], "GSA,", 4) == 0)
          return processGSA(_buffer);
      if (_buffer[1] == 'G' && strncmp(&_buffer[3], "RMC,", 4) == 0)
          return processRMC(_buffer);
      if (strncmp(_buffer, "$GNZDA,", 9) == 0)
          return processZDA(_buffer);
      if (strncmp(_buffer, "$GPZDA,", 9) == 0)
          return processZDA(_buffer);
      if (strncmp(_buffer, "$PUBX,00,", 9) == 0)
          return processPUBX00(_buffer);
    }

    return false;
  }

  *_ptr = c;
  if (_ptr < &_buffer[_bufferLen - 1]) 
    ++_ptr;

  return false;
}

void sensorGPS::handleFix(int32_t fix) {
  _fix = fix;
  if (!fix) {
    // If lost fix, then reset age...
    _fixAge = SCHAR_MIN;
    return;
  }
  
  unsigned long now = millis();
#ifdef DEBUG
  Serial.print("_update_ms_fixAge: ");
  Serial.print(_update_ms_fixAge);
  Serial.print(" ");
  Serial.print((_fixAge == LONG_MIN));
  Serial.print(" ");
  Serial.print(now);
#endif
  if (_fixAge == LONG_MIN) {
    _update_ms_fixAge = now;
    _fixAge = 0;
  }
  else {
    _fixAge = now - _update_ms_fixAge;
    _update_ms_fixAge = now;
  }
#ifdef DEBUG
  Serial.print(" _update_ms_fixAge: ");
  Serial.print(_update_ms_fixAge);
  Serial.print(" _fixAge: ");
  Serial.println(_fixAge);
#endif

  _processedFix++;
  if (_processedFix == UINT32_MAX) {
    _processedFix = 0;
    _processedFix++;
  }
}

// chop a float as scaled integer of string, returns true if field found or empty
// sets s=nullptr if whole string was parsed
// sets v=LONG_MIN if field was empty
bool sensorGPS::parseFloatRef(const char * &s, int scaledigits, int32_t &v) {
  bool empty = true;
  
  // exit if no more fields
  if (s == nullptr)
    return false;

  // test negative sign
  bool neg = (s[0]=='-');
  if (neg) 
    s++; //skip over '-'

  //before decimal point
  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    empty = false;
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }

  // after decimal point
  if (s[0] == '.') {
    s++; //skip decimal point
    while (s[0]>='0' && s[0]<='9') {
      empty = false;
      if (scaledigits>0) {
        v = v * 10 + (s[0] - '0');
        scaledigits--;
      }
      s++; // next digit
    }
  }

  // scale any remaining scaledigits
  while (scaledigits>0) {
    v = v * 10;
    scaledigits--;
  }

  // apply negative sign
  if (neg) 
    v = -v;

  // empty check
  if (empty) 
    v = LONG_MIN;

  // check end of string
  if (isEndOfFields(s[0])) {
      s = nullptr; 
      return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; //skip delimiter
    return true;
  }
  
  return false; // not at end of string or at delimiter - something is wrong
}

// chop a float as scaled integer of string, returns true if field found or empty
// sets s=nullptr if whole string was parsed
// sets v=LONG_MIN if field was empty
bool sensorGPS::parseFloatRef(const char * &s, int scaledigits, int8_t &v) {
  bool empty = true;
  
  // exit if no more fields
  if (s == nullptr)
    return false;

  // test negative sign
  bool neg = (s[0]=='-');
  if (neg) 
    s++; //skip over '-'

  //before decimal point
  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    empty = false;
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }

  // after decimal point
  if (s[0] == '.') {
    s++; //skip decimal point
    while (s[0]>='0' && s[0]<='9') {
      empty = false;
      if (scaledigits>0) {
        v = v * 10 + (s[0] - '0');
        scaledigits--;
      }
      s++; // next digit
    }
  }

  // scale any remaining scaledigits
  while (scaledigits>0) {
    v = v * 10;
    scaledigits--;
  }

  // empty check
  if (empty) 
    v = (int8_t)SHRT_MIN;

  // check end of string
  if (isEndOfFields(s[0])) {
      s = nullptr; 
      return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; //skip delimiter
    return true;
  }
  
  return false; // not at end of string or at delimiter - something is wrong
}

bool sensorGPS::parseFloatRef(const char * &s, int scaledigits, int16_t &v) {
  bool empty = true;
  
  // exit if no more fields
  if (s == nullptr)
    return false;

  // test negative sign
  bool neg = (s[0]=='-');
  if (neg) 
    s++; //skip over '-'

  //before decimal point
  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    empty = false;
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }

  // after decimal point
  if (s[0] == '.') {
    s++; //skip decimal point
    while (s[0]>='0' && s[0]<='9') {
      empty = false;
      if (scaledigits>0) {
        v = v * 10 + (s[0] - '0');
        scaledigits--;
      }
      s++; // next digit
    }
  }

  // scale any remaining scaledigits
  while (scaledigits>0) {
    v = v * 10;
    scaledigits--;
  }

  // empty check
  if (empty) 
    v = USHRT_MAX;

  // check end of string
  if (isEndOfFields(s[0])) {
      s = nullptr; 
      return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; //skip delimiter
    return true;
  }
  
  return false; // not at end of string or at delimiter - something is wrong
}

bool sensorGPS::parseFloatRef(const char * &s, int scaledigits, uint16_t &v) {
  bool empty = true;
  
  // exit if no more fields
  if (s == nullptr)
    return false;

  // test negative sign
  bool neg = (s[0]=='-');
  if (neg) 
    s++; //skip over '-'

  //before decimal point
  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    empty = false;
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }

  // after decimal point
  if (s[0] == '.') {
    s++; //skip decimal point
    while (s[0]>='0' && s[0]<='9') {
      empty = false;
      if (scaledigits>0) {
        v = v * 10 + (s[0] - '0');
        scaledigits--;
      }
      s++; // next digit
    }
  }

  // scale any remaining scaledigits
  while (scaledigits>0) {
    v = v * 10;
    scaledigits--;
  }

  // empty check
  if (empty) 
    v = USHRT_MAX;

  // check end of string
  if (isEndOfFields(s[0])) {
      s = nullptr; 
      return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; //skip delimiter
    return true;
  }
  
  return false; // not at end of string or at delimiter - something is wrong
}

// chop an integer of string, returns true if field found or empty
// sets s=nullptr if whole string was parsed
bool sensorGPS::parseIntRef(const char * &s, int8_t &v) {
  if (s == nullptr) 
    return false;

  bool neg = (s[0]=='-');
  if (neg) 
    s++; // skip over '-'

  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }
  if (neg) 
    v = -v;
  
  // check end of string
  if (isEndOfFields(s[0])) {
    s = nullptr; 
    return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; // skip delimiter
    return true;
  }
  
  return false; // not at end of string or at delimiter - something is wrong
}

// chop an integer of string, returns true if field found or empty
// sets s=nullptr if whole string was parsed
bool sensorGPS::parseIntRef(const char * &s, int16_t &v) {
  if (s == nullptr) 
    return false;

  bool neg = (s[0]=='-');
  if (neg) 
    s++; // skip over '-'

  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }
  if (neg) 
    v = -v;
  
  // check end of string
  if (isEndOfFields(s[0])) {
    s = nullptr; 
    return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; // skip delimiter
    return true;
  }
  
  return false; // not at end of string or at delimiter - something is wrong
}

// chop an integer of string, returns true if field found or empty
// sets s=nullptr if whole string was parsed
bool sensorGPS::parseIntRef(const char * &s, int32_t &v) {
  if (s == nullptr) 
    return false;

  bool neg = (s[0]=='-');
  if (neg) 
    s++; // skip over '-'

  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }
  if (neg) 
    v = -v;
  
  // check end of string
  if (isEndOfFields(s[0])) {
      s = nullptr; 
      return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; // skip delimiter
    return true;
  }

  return false; // not at end of string or at delimiter - something is wrong
}

// chop an integer of string, returns true if field found or empty
// sets s=nullptr if whole string was parsed
bool sensorGPS::parseIntRef(const char * &s, uint8_t &v) {
  if (s == nullptr) 
    return false;

  bool neg = (s[0]=='-');
  if (neg) 
    s++; // skip over '-'

  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }
  
  // check end of string
  if (isEndOfFields(s[0])) {
    s = nullptr; 
    return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; // skip delimiter
    return true;
  }
  
  return false; // not at end of string or at delimiter - something is wrong
}

// chop an integer of string, returns true if field found or empty
// sets s=nullptr if whole string was parsed
bool sensorGPS::parseIntRef(const char * &s, uint16_t &v) {
  if (s == nullptr) 
    return false;

  bool neg = (s[0]=='-');
  if (neg) 
    s++; // skip over '-'

  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }
  
  // check end of string
  if (isEndOfFields(s[0])) {
    s = nullptr; 
    return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; // skip delimiter
    return true;
  }
  
  return false; // not at end of string or at delimiter - something is wrong
}

// chop an integer of string, returns true if field found or empty
// sets s=nullptr if whole string was parsed
bool sensorGPS::parseIntRef(const char * &s, uint32_t &v) {
  if (s == nullptr) 
    return false;

  bool neg = (s[0]=='-');
  if (neg) 
    s++; // skip over '-'

  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }
  
  // check end of string
  if (isEndOfFields(s[0])) {
    s = nullptr; 
    return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; // skip delimiter
    return true;
  }
  
  return false; // not at end of string or at delimiter - something is wrong
}

bool sensorGPS::parseLatLngRef(const char* &s, int32_t &v) {
  int32_t degsec = 0, sec = 0;
  parseIntRef(s, degsec);
  if (s[0] != '.') 
      return false;

  if (!parseFloatRef(s, 7, sec)) 
      return false;

  int32_t deg = degsec / 100;
  sec += (degsec - deg * 100) * 10000000;
  sec = (sec + 30) / 60;
  deg = deg * 10000000 + sec;
  if (s[0] == 'S' || s[0] == 'W')
      deg = -deg;
  else if (s[0] != 'N' && s[0] != 'E')
      return false;

  s++; // skip S/N E/W indicator
  if (s[0] != ',')
    return false;

  s++; // skip ,
  v = deg;

  return true;
}

bool sensorGPS::parseTimeRef(const char* &s, int32_t &v) {
  int32_t t;
  if (!parseFloatRef(s, 3, t)) 
    return false;

  int32_t h = t / 10000000;
  t -= h * 10000000;
  int32_t m = t / 100000;
  t -= m * 100000;
  t += m * 60000 + h * 3600000;
  v =t;

  return true;
}

// chop an integer of string, returns true if field found or empty
// sets s=nullptr if whole string was parsed
bool sensorGPS::parseUInt8Ref(const char * &s, uint8_t &v) {
  if (s == nullptr) 
    return false;

  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }
  
  // check end of string
  if (isEndOfFields(s[0])) {
    s = nullptr; 
    return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; // skip delimiter
    return true;
  }

  return false; // not at end of string or at delimiter - something is wrong
}

// chop an integer of string, returns true if field found or empty
// sets s=nullptr if whole string was parsed
bool sensorGPS::parseUInt16Ref(const char * &s, uint16_t &v) {
  if (s == nullptr) 
    return false;

  v = 0;
  while (s[0]>='0' && s[0]<='9') {
    v = v * 10 + (s[0] - '0');
    s++; // next digit
  }
  
  // check end of string
  if (isEndOfFields(s[0])) {
    s = nullptr; 
    return true;
  }
  
  // check ending delimiter
  if (s[0] == ',') {
    s++; //skip delimiter
    return true;
  }

  return false; // not at end of string or at delimiter - something is wrong
}

// move s to next field, set s=nullpointer if no more fields
bool sensorGPS::skipFieldRef(const char* &s) {
  if (s == nullptr) 
    return false;

  while (s[0] != ',' && !isEndOfFields(s[0])) 
    s++;

  if (s[0] == ',')
      s++; // skip delimiter
  else
      s = nullptr; // no more fields

  return true;
}

bool sensorGPS::processGGA(const char *s) {
#ifdef DEBUG
  Serial.print("processGGA: ");
  Serial.println(_buffer);
#endif

  // skip message ID
  if (!skipFieldRef(s))
    return false;

  // UTC Time
  int32_t tmp_time;
  if (!parseTimeRef(s, tmp_time)) 
    return false;

  // Latitude + N/S indicator
  int32_t tmp_lat;
  if (!parseLatLngRef(s, tmp_lat))
    return false;

  // Longitude + E/W indicator
  int32_t tmp_lon;
  if (!parseLatLngRef(s, tmp_lon))
    return false;

  //Status
  /*
  0: Fix not valid
  1: StandardGPS fix (2D/3D)
  2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
  3: Not applicable
  4: RTK Fixed, xFill, RTX
  5: RTK Float, OmniSTAR XP/HP, Location RTK, QZSS CLAS
  6: INS Dead reckoning
  */
  int8_t tmp_fix = (*s >= '1' && *s <= '9' ? 1 : 0);
  s += 2; // Skip position fix flag and comma

  // Number of satellites
  int32_t tmp_sat;
  if (!parseFloatRef(s, 0, tmp_sat))
    return false;

  // HDOP Horizontal Dilution of Precision
  int16_t tmp_hdop;
  if (!parseFloatRef(s, 3, tmp_hdop))
    return false;

  // MSL Altitude in m
  int32_t tmp_altMSL;
  if (!parseFloatRef(s, 3, tmp_altMSL)) 
    return false;

  // skip 'M'
  if (!skipFieldRef(s))
    return false;

  // Geoid Separation in m
  int32_t tmp_altSep;
  if (!parseFloatRef(s, 3, tmp_altSep))
    return false;

  // skip 'M'
  if (!skipFieldRef(s))
    return false;

  _processedGGA++;
  if (_processedGGA == UINT32_MAX) {
    _processedGGA = 0;
    _processedGGA_over++;
  }

  // That's all we care about, save received data
  // only save geoid altitude if using PUBX00 (PUBX00 does not have this field)
  if (_use_only_pubx00) {
    _altSep = tmp_altSep;
    return false;
  }

  handleFix(tmp_fix);

  if (tmp_altMSL != LONG_MIN && tmp_altSep != LONG_MIN)
    _alt = tmp_altMSL + tmp_altSep; // altitude above geoid
  else
    _alt = tmp_altMSL;
  _altMSL = tmp_altMSL;

  _hdop = tmp_hdop;
  _lat = tmp_lat;
  _lon = tmp_lon;
  _satellites = tmp_sat;
  _altSep = tmp_altSep;
  _time = tmp_time;

  _update_ms = millis();

  return true;
}

bool sensorGPS::processGSA(const char *s) {
#ifdef DEBUG
  Serial.print("processGSA: ");
  Serial.println(_buffer);
#endif

  // skip message ID
  if (!skipFieldRef(s))
    return false;

  // skip mode op, m/a
  if (!skipFieldRef(s))
    return false;

  // Fix type
  /*
  1 = not available
  2 = 2D
  3 = 3D
  */
  int8_t tmp_fixType = (*s >= '1' && *s <= '3' ? 1 : 0);
  s += 2; // Skip position fix flag and comma

  // Serial.println(s);
  // Serial.println("4...");
  for (int i = 0; i < 12; i++) {
    // Serial.print("\t4...");
    // Serial.println(i);
    skipFieldRef(s);
    // Serial.println(s);
  }

  // Serial.println(s);
  // PDOP Position Dilution of Precision
  int16_t tmp_pdop;
  if (!parseFloatRef(s, 3, tmp_pdop))
    return false;

  // HDOP Horizontal Dilution of Precision
  int16_t tmp_hdop;
  if (!parseFloatRef(s, 3, tmp_hdop))
    return false;

  // VDOP Vertical Dilution of Precision
  int16_t tmp_vdop;
  if (!parseFloatRef(s, 3, tmp_vdop))
    return false;

  // // GNSS ID
  // int32_t tmp_gnss;
  // if (!parseIntRef(s, tmp_gnss)) 
  //   return false;

  _processedGSA++;
  if (_processedGSA == UINT32_MAX) {
    _processedGSA = 0;
    _processedGSA_over++;
  }

  if (_use_only_pubx00)
    return false;

  _fixType = tmp_fixType;
  _hdop = tmp_hdop;
  _pdop = tmp_pdop;
  _vdop = tmp_vdop;

  _update_ms = millis();

  return true;
} 

bool sensorGPS::processPUBX00(const char* s) {
#ifdef DEBUG
  Serial.println("processPUBX00");
  Serial.println(_buffer);
#endif

  // skip message ID
  if (!skipFieldRef(s)) 
    return false;

  // skip 00
  if (!skipFieldRef(s)) 
    return false;

  // UTC Time
  int32_t tmp_time;
  if (!parseTimeRef(s, tmp_time)) 
    return false;

  // Latitude + N/S indicator
  int32_t tmp_lat;
  if (!parseLatLngRef(s, tmp_lat)) 
    return false;

  // Longitude + E/W indicator
  int32_t tmp_lon;
  if (!parseLatLngRef(s, tmp_lon)) 
    return false;

  // Altitude in meters above user datum ellipsoid (= MSL Altitude + Geoid Separation from GGA message)
  int32_t tmp_alt;
  if (!parseFloatRef(s, 3, tmp_alt)) 
    return false;

  // Navigation Status (2 char)
  /*
  NF No Fix
  DR Dead reckoning only solution
  G2 Stand alone 2D solution
  G3 Stand alone 3D solution
  D2 Differential 2D solution
  D3 Differential 3D solution
  RK Combined GPS + dead reckoning solution
  TT Time only solution  
  */
  int8_t tmp_fix;
  if (s[0] == 'D' && s[1] == 'R') 
    tmp_fix = 1; 
  else if (s[0] == 'G' && s[1] == '2') 
    tmp_fix = 2;
  else if (s[0] == 'G' && s[1] == '3') 
   tmp_fix = 3;
  else if (s[0] == 'D' && s[1] == '2') 
    tmp_fix = 2;
  else if (s[0] == 'D' && s[1] == '3') 
    tmp_fix = 3;
  else if (s[0] == 'R' && s[1] == 'K') 
    tmp_fix = 3;
  else tmp_fix = 0;

  s += 3; // Skip incl comma

  // Horizontal accuracy estimate in meters
  int32_t tmp_hacc;
  if (!parseFloatRef(s, 3, tmp_hacc)) 
    return false;

  // Vertical accuracy estimate in meters
  int32_t tmp_vacc;
  if (!parseFloatRef(s, 3, tmp_vacc)) 
    return false;

  // Speed over ground in km/h (1 km/h = 1/3.6 m/s)
  int32_t tmp_sog;
  if (!parseFloatRef(s, 3, tmp_sog)) 
    return false;
  tmp_sog = (tmp_sog * 10 + 18) / 36;

  // Course over ground in degrees
  int32_t tmp_cog;
  if (!parseFloatRef(s, 3, tmp_cog)) 
    return false;

  // Vertical downward velocity in m/s
  int32_t tmp_veld;
  if (!parseFloatRef(s, 3, tmp_veld)) 
    return false;

  // Age of most recent DGPS corrections in seconds, empty = none available
  if (!skipFieldRef(s)) 
    return false;

  // HDOP Horizontal Dilution of Precision
  int16_t tmp_hdop;
  if (!parseFloatRef(s, 3, tmp_hdop)) 
    return false;

  // VDOP Vertical Dilution of Precision
  int16_t tmp_vdop;
  if (!parseFloatRef(s, 3, tmp_vdop)) 
    return false;

  // TDOP Time Dilution of Precision
  int16_t tmp_tdop;
  if (!parseFloatRef(s, 3, tmp_tdop)) 
    return false;

  // Number of GPS satellites
  int32_t tmp_satGps;
  if (!parseFloatRef(s, 0, tmp_satGps)) 
    return false;

  // Number of GLONASS satellites
  int32_t tmp_satGlo;
  if (!parseFloatRef(s, 0, tmp_satGlo)) 
    return false;

  _processedPUBX++;
  if (_processedPUBX == UINT32_MAX) {
    _processedPUBX = 0;
    _processedPUBX_over++;
  }

  handleFix(tmp_fix);

  // That's all we care about, save received data
  _alt = tmp_alt;
  _altMSL = tmp_alt;
  _cog = tmp_cog;
  _hacc = tmp_hacc;
  _hdop = tmp_hdop;
  _lat = tmp_lat;
  _lon = tmp_lon;
  _satellites = tmp_satGps + tmp_satGlo;
  _sog = tmp_sog;
  _tdop = tmp_tdop;
  _time = tmp_time;
  _vacc = tmp_vacc;
  _vdop = tmp_vdop;
  _veld = tmp_veld;
  
  _use_only_pubx00 = true;

  _update_ms = millis();

  return true;
}

bool sensorGPS::processRMC(const char* s) {
#ifdef DEBUG
  Serial.print("processRMC: ");
  Serial.println(_buffer);
#endif

  // skip message ID
  if (!skipFieldRef(s))
    return false;

  // UTC Time
  int32_t tmp_time;
  if (!parseTimeRef(s, tmp_time))
    return false;

  // Status
  s += 2; // Skip validity and comma

  // Latitude + N/S indicator
  int32_t tmp_lat;
  if (!parseLatLngRef(s, tmp_lat))
    return false;

  // Longitude + E/W indicator
  int32_t tmp_lon;
  if (!parseLatLngRef(s, tmp_lon))
    return false;

  // Speed over ground in knots (1 kt = 514.44 mm/s)
  int32_t tmp_sog;
  if (!parseFloatRef(s, 3, tmp_sog))
    return false;

  tmp_sog = (tmp_sog * 514 + 257) / 1000;

  // Course over ground in degrees
  int32_t tmp_cog;
  if (!parseFloatRef(s, 3, tmp_cog))
    return false;

  // Date in day, month, year format
  int32_t tmp_date;
  if (!parseIntRef(s, tmp_date))
    return false;

  _processedRMC++;
  if (_processedRMC == UINT32_MAX) {
    _processedRMC = 0;
    _processedRMC_over++;
  }

  // That's all we care about, save received data
  // only save date/time if using PUBX00 (PUBX00 does not have date field)
  if (_use_only_pubx00) {
      _time = tmp_time;
      _date = tmp_date;
      return false;
  }
  
  _cog = tmp_cog;
  _date = tmp_date;
  _lat = tmp_lat;
  _lon = tmp_lon;
  _sog = tmp_sog;
  _time = tmp_time;

  _update_ms = millis();

  return true;
}

bool sensorGPS::processZDA(const char* s) {
#ifdef DEBUG
  Serial.print("processZDA: ");
  Serial.println(_buffer);
#endif

  // skip message ID
  if (!skipFieldRef(s))
    return false;

  // UTC Time
  int32_t tmp_time;
  if (!parseTimeRef(s, tmp_time))
    return false;

  // Day
  uint8_t tmp_day;
  if (!parseUInt8Ref(s, tmp_day))
    return false;

  // Month
  uint8_t tmp_month;
  if (!parseUInt8Ref(s, tmp_month))
    return false;

  // Year
  uint8_t tmp_year;
  if (!parseUInt8Ref(s, tmp_year))
    return false;


  // Time Offset Hours from GMT
  int8_t tmp_time_offset_hours;
  if (!parseIntRef(s, tmp_time_offset_hours))
    return false;

  // Time Offset Minutes from GMT
  int8_t tmp_time_offset_minutes;
  if (!parseIntRef(s, tmp_time_offset_minutes))
    return false;

  _processedZDA++;
  if (_processedZDA == UINT32_MAX) {
    _processedZDA = 0;
    _processedZDA_over++;
  }

  // That's all we care about, save received data
  // only save date/time if using PUBX00 (PUBX00 does not have date field)
  if (_use_only_pubx00)
    return false;

  _day = tmp_day;
  _month = tmp_month;
  _time = tmp_time;
  _time_offset_hours = tmp_time_offset_hours;
  _time_offset_minutes = tmp_time_offset_minutes;
  _year = tmp_year;

  _update_ms = millis();

  return true;
}

bool sensorGPS::testChecksum(const char* s) {
  // char checksum[2];
  const char* p = generateChecksum(s, _checksumCalc);
  _checksumRcv[0] = p[1];
  _checksumRcv[1] = p[2];

  if (*p == '*')
      return p[1] == _checksumCalc[0] && p[2] == _checksumCalc[1];

  _failed++;
  if (_failed == UINT32_MAX) {
      _failed = 0;
      _failed++;
  }

  return false;
}