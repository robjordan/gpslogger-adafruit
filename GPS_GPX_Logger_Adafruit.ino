// Log GPS coordinates to SD card as a GPX file.
//
// Based on: Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
// This version sets up LOCUS logging (autonomous logging to flash by the GPS Featherwing). 
// Then reads the flash at reset, and writes it to a GPX file on SD card, before restarting.

     
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SdFat.h>
#include <stdlib.h>
#include <string.h>
#include <RTCZero.h>
#include <ArduinoLog.h>

#define FALSE 0
#define TRUE (!FALSE)

// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select 
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
#define SPI_SPEED SD_SCK_MHZ(4)

#define LOCUS_BATCH_TIME 1200 // seconds

#define PMTK_SET_Nav_Speed_0_0 "$PMTK386,0*23"
#define PMTK_SET_Nav_Speed_0_2 "$PMTK386,0.2*3F"
#define PMTK_SET_Nav_Speed_0_4 "$PMTK386,0.4*39"
#define PMTK_SET_Nav_Speed_0_6 "$PMTK386,0.6*3B"
#define PMTK_SET_Nav_Speed_0_8 "$PMTK386,0.8*35"
#define PMTK_SET_Nav_Speed_1_0 "$PMTK386,1.0*3C"
#define PMTK_SET_Nav_Speed_1_5 "$PMTK386,1.5*39"
#define PMTK_SET_Nav_Speed_2_0 "$PMTK386,2.0*3F"
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"
#define PMTK_LOCUS_STOP_LOGGER "$PMTK185,1*23"
#define PMTK_LOCUS_INTERVAL_5s "$PMTK187,1,5*38"
#define PMTK_LOCUS_LOG_NOW "$PMTK186,1*20"
#define PMTK_Q_LOCUS_DATA_USED "$PMTK622,1*29"
#define PMTK_Q_LOCUS_DATA_FULL "$PMTK622,0*28"
#define PMTK_CMD_HOT_START "$PMTK101*32"

// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define GPSECHO  true

// GLOBALS
Adafruit_GPS GPS(&GPSSerial);
File gpx;		// handle for output in GPX format
bool gpx_open = FALSE;
uint32_t timer = millis();
char buf[256];		// generic buffer for formatting printed output

// File system objects
SdFat sd;		// file system
SdFile sdLogger; 	// logging file
SdFile locusFile;	// temporary file to store locus track output in NMEA format

// Serial stream for debug output in C++ style
ArduinoOutStream cout(Serial);

// Real-time clock
RTCZero rtc;

// Persistent sequence number, for storing files with unique and sequential names
const uint32_t seqno = 0;

// Pre-declare functions other than those returning int
File create_gpx();
void SD_setup();
void write_trkpt_to_gpx();
char *filename(const char *suffix);
void write_to_gpx(const char *s);
void print_GPS();
char *convert_coord(double nmea, char compass, char *s);
char *dtostrf(double val, int width, unsigned int prec, char *sout);
void dateTime(uint16_t* date, uint16_t* time);
void GPS_setup();
double checkBattery();
bool startsWith(const char *pre, const char *str);
void standBy(int sec);
void alarmMatch();

// Use the following aproach to create an output file in GPX format, with unlimited 
// duration. The primary storage location for new GPS track points is the flash 
// memory on board the MKT3339 chip. This has capacity for approx 11 hours at 5s
// logging interval. After initialising LOCUS logging (i.e. logging to flash),
// the host processor can go to sleep for an extended period. We will use 1 hour, but 
// it could just as well be 10 hours. At each wake-up, when the loop() function runs,
// we append all of the contents of the LOCUS storage in flash to a working file on 
// the SD card, named "CURRENT.LOC". The LOCUS storage is then erased.
// Conversion of LOCUS data to GPX is triggered by a RESET of the card, and hence the 
// running of the setup() function. This checks for any addditional LOCUS data, appends
// this to "CURRENT.LOC" and then proceeds to convert CURRENT.LOC to GPX format. 
// CURRENT.LOC is renamed to a date-stamped .LOC file (as a precaution against faulty GPX 
// conversion. LOCUS logging is then re-initialised.

void setup()
{

  // initialize digital pin 13 (green LED) to flash on SD card writes
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  // while (!Serial);   // uncomment to have the sketch wait until Serial is ready
  delay(3000);          // Time for USB connection, GPS to finish reboot, etc.
  
  // Setup the SD card
  SD_setup();

  // Create a logger (might be Serial or SD file
  Log.begin(LOG_LEVEL_VERBOSE, &sdLogger);
  // Log.begin(LOG_LEVEL_VERBOSE, &Serial);


  // connect over USB at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Log.notice("setup(): Check GPS module flash for log points & write to SD card as a GPX file." CR);
     
  // start a real-time clock
  rtc.begin();

  // Setup the GPS device
  GPS_setup();

  // If there was a logging activity going on before the reset, now's the time to convert it to GPX.
  // First retrieve any outstanding LOCUS data from the GPS module and store it on SD. Then rename 
  // the LOCUS file
  
  // Open the LOCUS data-in-progress (or create if there isn't one)
  if (!locusFile.open("CURRENT.LOC", O_WRITE | O_APPEND | O_CREAT)) {
    Log.error("Unable to open LOCUS file CURRENT.LOC" CR);
  }
  process_locus_data();

  convert_to_gpx();
  // rename old file just as precaution

  FatFile *fv = sd.vwd();
  char *newfilename = filename("LOC");
  Log.trace("sd.vwd(): %x, filename: %s" CR, fv, newfilename);
  if (!locusFile.rename(fv, newfilename)) {
    Log.error("Unable to rename LOCUS file CURRENT.LOC" CR);
  }  
  if (!locusFile.close()) {
    Log.error("Unable to close LOCUS file CURRENT.LOC" CR);
  }

  // Re-open the LOCUS file for new data
  if (!locusFile.open("CURRENT.LOC", O_WRITE | O_CREAT)) {
    Log.error("Unable to open LOCUS file CURRENT.LOC" CR);
  }
  
  
}

void loop() {

  Log.trace("loop(): About to sleep, milliseconds: %d" CR, millis());

  // now we've handled a batch, sleep until it's next time to unload the LOCUS data
  standBy(LOCUS_BATCH_TIME);
  // delay(LOCUS_BATCH_TIME*1000);

  Log.trace("loop(): Wakeup, milliseconds: %d" CR, millis());

  // Check for GPS module reset and re-initialise if it has happened

  // Loop occurs every wakeup... make sure that we have all in-progress locus data transferred to SD file
  process_locus_data();

  // make sure files are flushed before going to sleep
  gpx.flush();
  sdLogger.flush();
  locusFile.flush();


    
}

// Stop LOCUS logging, copy all of the data from flash to SD file (in NMEA format) then restart LOCUS logging
void process_locus_data() {
  int numPoints = 0, numTransferred = 0;

  Log.trace("process_locus_data()" CR);

  if (!GPS.LOCUS_StopLogger()) {
    Log.notice("GPS.LOCUS_StopLogger error" CR);
  }

  // if there are log entries in LOCUS flash, write them to SD card
  if (numPoints = GPS_LOCUS_records()) {
    // Weird thing occurs where 5 points in every 256 are corrupt / invalid.
    // I would love to check that every valid point is written to file. But 
    // in the circumstances, let's just approximate. If there are n points, 
    // there must be at least n/6 NMEA sentences (since each contains 6 points).
    if ((numTransferred = append_locus_data_to_file()) > numPoints/6) {
      
      // Seem to be transferred safely, erase the flash
      Log.trace("Copied LOCUS data to file, numPoints=%d, numTransferred=%d" CR, numPoints, numTransferred);
      
      GPS.sendCommand(PMTK_LOCUS_ERASE_FLASH);
      
      // All the data is now safe, give a reassuring little flash
      for (int i=0; i<5; i++) {
        digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(250);                // wait briefly
        digitalWrite(13, LOW);
        delay(250);
      }
    } else {
      Log.error("Error copying LOCUS data to file, numPoints=%d, numTransferred=%d" CR, numPoints, numTransferred);
    }
  } 

  if (!GPS.LOCUS_StartLogger()) {
    Log.error("GPS.LOCUS_StartLogger error" CR);
  }
  
}

void convert_to_gpx () {
  Log.trace("convert_to_gps()" CR);
}

// void loop() // run over and over again
// {
  
//   // read data from the GPS in the 'main loop'
//   char c = GPS.read();
//   // if you want to debug, this is a good time to do it!
//   if (GPSECHO)
//     if (c) Serial.print(c);
//   // if a sentence is received, we can check the checksum, parse it...
//   if (GPS.newNMEAreceived()) {
//     Log.trace("Milliseconds: %d" CR, millis());
//     // a tricky thing here is if we print the NMEA sentence, or data
//     // we end up not listening and catching other sentences!
//     // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
//     Log.notice("%s" CR, GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    
//     if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
//       return; // we can fail to parse a sentence in which case we should just wait for another
//     else {

      
//       // We have received and parsed a sentence. Does it have date? If we haven't created a dated GPX file, create it now
//       if (!gpx_open && GPS.fix && GPS.year) {
//          if (create_gpx ())
//           gpx_open = TRUE;
//       	else
//       	  Log.error("Failed to create GPX file" CR);
//       }

//       // Now log the new data to the gpx file
//       if (gpx_open && GPS.fix && startsWith("$GPGGA", GPS.lastNMEA())) {
//         write_trkpt_to_gpx();
//       	Log.notice("BATT:,\"%d/%d/%d %d:%d:%d\",%s" CR, 
//       		   GPS.year, 
//       		   GPS.month, 
//       		   GPS.day, 
//       		   GPS.hour, 
//       		   GPS.minute, 
//       		   GPS.seconds, 
//       		   dtostrf(checkBattery(), 5, 3, buf));
//       }
      
//       if (startsWith("$PMTK010,001", GPS.lastNMEA())) { // GPS device has reset
//         // We need to reinitialise the settings
//         GPS_setup();
//       }
 
//     }
//   }
// }


int append_locus_data_to_file () {
  int recordsTransferred = 0;
  int recordsAvailable = 0;
  
  Log.trace("append_locus_data_to_file()" CR);
  
  // Request all the LOCUS data from flash on the GPS module
  GPS.sendCommand(PMTK_Q_LOCUS_DATA_USED);
  if (!GPS.waitForSentence("$PMTKLOX,0")) {
    Log.error("Didn't receive $PMTKLOX,0 as expected, got this instead: %s" CR, GPS.lastNMEA());
  } 
  else {
    Log.trace(GPS.lastNMEA());
    char *token = strtok(GPS.lastNMEA(), "*,");  // $PMTLOX
    Log.trace("token: %s" CR, token);
    token = strtok(NULL, "*,");             // 0
    Log.trace("token: %s" CR, token);
    token = strtok(NULL, "*,");             // Now we've got the record count
    Log.trace("token: %s" CR, token);
    recordsAvailable = atoi(token);
    Log.trace("recordsAvailable: %d" CR, recordsAvailable);
  }

  char kloxType = '1';
  while (kloxType == '1' && GPS.waitForSentence("$PMTKLOX")) {

    // Two cases:
    // $PMTKLOX,1,... It's data that needs to be copied to SD
    // $PMTKLOX,2,... Its the end of the data dump, break out of the loop
    // Don't forget the NMEA line is preceded by a carriage return!!
    char *nmea = GPS.lastNMEA();
    char *p = nmea;
    while (!isdigit(*p))
      p++;
    if ((kloxType = *p) == '1') {
      // copy 1 sentence to SD locus file
      if (locusFile.write(GPS.lastNMEA()) < 0) {
	      Log.error("Failed writing to SD Locus file");
      }
      else {
	      recordsTransferred++;
      }
    }
  }

  (void)GPS.waitForSentence("$PMTK001,622");// Confirmation that PMTK_Q_LOCUS_DATA_USED completed

  Log.trace("recordsTransferred: %d" CR, recordsTransferred);
  return recordsTransferred;
}

// File create_gpx() {

//   // GPX file header
//   const char gpxhead[] = "<?xml version=\'1.0\' encoding=\'UTF-8\' standalone=\'yes\' ?>\n<gpx version=\"1.1\"\n creator=\"RMJ-GPXLogger\"\n xmlns=\"http://www.topografix.com/GPX/1/1\"\n xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n<trk>\n<trkseg>\n";

//   gpx = sd.open(filename("gpx"), FILE_WRITE);
//   if (gpx) {
//     write_to_gpx(gpxhead);
//   } else {
//     Log.error("Open file failed: %s" CR, gpx_filename());
//   }
//   return (gpx); 
// }

char *filename(const char *suffix) {
  static char filename[19];  // 15+3 filename and a null terminator
  char _suffix[3];

  // truncate suffix at 3 characters just in case we are passed a long one.
  strncpy(_suffix, suffix, sizeof _suffix);
  sprintf(filename, "%04d%02d%02d-%02d%02d%02d.%s", 
    GPS.year+2000, 
    GPS.month, 
    GPS.day, 
    GPS.hour,
    GPS.minute,
    GPS.seconds, 
    _suffix
    );
  Log.trace("Filename: %s" CR, filename);
  return(filename);
}

// void write_trkpt_to_gpx() {
//   const char trkpt[] = "  <trkpt lat=\"%s\" lon=\"%s\">\n\t<ele>%s</ele>\n\t<time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n  </trkpt>\n";
//   char s[sizeof(trkpt) + 32];
//   char lat[16];
//   char lon[16];
//   char ele[16];
//   static uint8_t prev_hour, prev_day = 0;

//   print_GPS();

//   // there is a situation where the hour has ticked over but the date has not. fix it here.
//   if (GPS.hour < prev_hour && GPS.day == prev_day) {
//     GPS.day++;
//   }
//   prev_hour = GPS.hour;
//   prev_day = GPS.day;
  
//   sprintf(s, trkpt, 
//     convert_coord(GPS.latitude, GPS.lat, lat), 
//     convert_coord(GPS.longitude, GPS.lon, lon), 
//     dtostrf(GPS.altitude,6,1,ele), 
//     GPS.year+2000, 
//     GPS.month, 
//     GPS.day, 
//     GPS.hour, 
//     GPS.minute, 
//     GPS.seconds);
//   write_to_gpx(s);
// }

char *convert_coord(double nmea, char compass, char *s) {
  unsigned deg = floor(nmea/100);
  double dec_deg = (nmea - 100*deg) / 60;
  dec_deg = deg + dec_deg;
  if (compass == 'S' || compass == 'W') 
    dec_deg = -1 * dec_deg;
  s = dtostrf(dec_deg, 12, 7, s);
  while (*s == ' ')
    s++;
  return(s);
}

// void write_to_gpx(const char *s) {
  
//   const char gpxtail[] = "</trkseg>\n</trk>\n</gpx>\n";
//   static uint32_t sizeLessTail = 0;
//   const unsigned long lenTail = strlen(gpxtail);
  
//   if (gpx) {
//     unsigned long lenS = strlen(s);
//     boolean truncRC;
    
//     truncRC = gpx.truncate(sizeLessTail);
//     Log.trace("Truncated, size: %d, rc: %d" CR, sizeLessTail, truncRC);    
//     gpx.write(s, lenS);
//     Log.trace("Wrote: %d" CR, lenS);
//     Log.trace(s);
//     sizeLessTail = gpx.size();
    
//     gpx.write(gpxtail, lenTail);
//     Log.trace("Wrote: %d" CR, lenTail);

//     digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
//     delay(50);                // wait briefly
//     digitalWrite(13, LOW);

//     // now we've got a track point, sleep for 4 seconds
//     standBy(4);
    
//   } else {
//     Log.error("GPX file not open." CR);
//   }  
//   return;
  
// }


void cardOrSpeed() {
  cout << F("Try another SD card or reduce the SPI bus speed.\n");
  cout << F("Edit SPI_SPEED in this program to change it.\n");
}

void reformatMsg() {
  cout << F("Try reformatting the card.  For best results use\n");
  cout << F("the SdFormatter program in SdFat/examples or download\n");
  cout << F("and use SDFormatter from www.sdcard.org/downloads.\n");
}

void SD_setup()
{
  // SD card chip select
  const int chipSelect = 4;

  Log.notice(CR "Initializing SD card..." CR);

  if (!sd.begin(chipSelect, SPI_SPEED)) {
    if (sd.card()->errorCode()) {
      cout << F(
             "\nSD initialization failed.\n"
             "Do not reformat the card!\n"
             "Is the card correctly inserted?\n"
             "Is chipSelect set to the correct value?\n"
             "Does another SPI device need to be disabled?\n"
             "Is there a wiring/soldering problem?\n");
      cout << F("\nerrorCode: ") << hex << showbase;
      cout << int(sd.card()->errorCode());
      cout << F(", errorData: ") << int(sd.card()->errorData());
      cout << dec << noshowbase << endl;
      return;
    }
    cout << F("\nCard successfully initialized.\n");
    if (sd.vol()->fatType() == 0) {
      cout << F("Can't find a valid FAT16/FAT32 partition.\n");
      reformatMsg();
      return;
    }
    if (!sd.vwd()->isOpen()) {
      cout << F("Can't open root directory.\n");
      reformatMsg();
      return;
    }
    cout << F("Can't determine error type\n");
    return;
  }
  cout << F("\nCard successfully initialized.\n");
  cout << endl;

  uint32_t size = sd.card()->cardSize();
  if (size == 0) {
    cout << F("Can't determine the card size.\n");
    cardOrSpeed();
    return;
  }
  uint32_t sizeMB = 0.000512 * size + 0.5;
  cout << F("Card size: ") << sizeMB;
  cout << F(" MB (MB = 1,000,000 bytes)\n");
  cout << endl;
  cout << F("Volume is FAT") << int(sd.vol()->fatType());
  cout << F(", Cluster size (bytes): ") << 512L * sd.vol()->blocksPerCluster();
  cout << endl << endl;

  cout << F("Files found (date time size name):\n");
  sd.ls(LS_R | LS_DATE | LS_SIZE);

  if ((sizeMB > 1100 && sd.vol()->blocksPerCluster() < 64)
      || (sizeMB < 2200 && sd.vol()->fatType() == 32)) {
    cout << F("\nThis card should be reformatted for best performance.\n");
    cout << F("Use a cluster size of 32 KB for cards larger than 1 GB.\n");
    cout << F("Only cards larger than 2 GB should be formatted FAT32.\n");
    reformatMsg();
    return;
  }

  // Setup callback to dateTime so that file creation dates are correct
  SdFile::dateTimeCallback(dateTime);

  // Open a log file
  if (!sdLogger.open(filename(".log"), O_WRITE | O_APPEND | O_CREAT)) {
    cout << F("Unable to open sdLogger file.\n");
  } 
}

void print_GPS() {
  char lat[16];
  char lon[16];

  Log.trace("Date: %d/%d/%d" CR, GPS.day, GPS.month, GPS.year);
  Log.trace("Time: %d:%d:%d" CR, GPS.hour, GPS.minute, GPS.seconds);
  Log.trace("Fix: %d, quality %d" CR, (int)GPS.fix, (int)GPS.fixquality);
  if (GPS.fix) {
    Log.trace("Location: %s, %s" CR, convert_coord(GPS.latitude, GPS.lat, lat),  convert_coord(GPS.longitude, GPS.lon, lon));
    Log.trace("Speed (knots): %s" CR, dtostrf((double)GPS.speed, 6, 2, buf)); 
    Log.trace("Angle: %s" CR, dtostrf((double)GPS.angle, 6, 2, buf));
    Log.trace("Altitude: %s" CR, dtostrf((double)GPS.altitude, 7, 1, buf));
    Log.trace("Satellites: %d " CR, (int)GPS.satellites);
  }
}

// dtostrf: credit to jsmith (http://forum.arduino.cc/index.php?topic=368720.0)
char *dtostrf(double val, int width, unsigned int prec, char *sout) {
  int decpt, sign, reqd, pad;
  const char *s, *e;
  char *p;
  s = fcvt(val, prec, &decpt, &sign);
  if (prec == 0 && decpt == 0) {
  s = (*s < '5') ? "0" : "1";
    reqd = 1;
  } else {
    reqd = strlen(s);
    if (reqd > decpt) reqd++;
    if (decpt == 0) reqd++;
  }
  if (sign) reqd++;
  p = sout;
  e = p + reqd;
  pad = width - reqd;
  if (pad > 0) {
    e += pad;
    while (pad-- > 0) *p++ = ' ';
  }
  if (sign) *p++ = '-';
  if (decpt <= 0 && prec > 0) {
    *p++ = '0';
    *p++ = '.';
    e++;
    while ( decpt < 0 ) {
      decpt++;
      *p++ = '0';
    }
  }    
  while (p < e) {
    *p++ = *s++;
    if (p == e) break;
    if (--decpt == 0) *p++ = '.';
  }
  if (width < 0) {
    pad = (reqd + width) * -1;
    while (pad-- > 0) *p++ = ' ';
  }
  *p = 0;
  return sout;
}

// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {

 // return date using FAT_DATE macro to format fields
 *date = FAT_DATE(GPS.year+2000, GPS.month, GPS.day);

 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(GPS.hour, GPS.minute, GPS.seconds);
}


void GPS_setup() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);


  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_200_MILLIHERTZ); // One fix every 5 seconds
  GPS.waitForSentence("$PMTK001,300");
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); // One reading every 5 seconds
  GPS.waitForSentence("$PMTK001,220");
  GPS.sendCommand(PMTK_SET_Nav_Speed_0_6);  // Suppress readings where movement is < 0.6 m/s
  GPS.waitForSentence("$PMTK001,386");
  // Ask for firmware version
  GPS.sendCommand(PMTK_Q_RELEASE);
  GPS.waitForSentence("$PMTK705");
  
  // Wait for a GPS fix and set date/time of RTC
  while (!GPS.fix) {
    GPS.waitForSentence("$GPRMC");
    GPS.parse(GPS.lastNMEA());
  }
  rtc.setDate(GPS.day, GPS.month, GPS.year+2000);
  rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);

  // Now switch to LOCUS mode, turning off normal NMEA traffic
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_OFF);
  GPS.waitForSentence("$PMTK001,314");  
  GPS.sendCommand(PMTK_LOCUS_INTERVAL_5s); // One reading every 5 seconds
  GPS.waitForSentence("$PMTK001,187");
     
  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  

}



// int GPS_read_response() {
//   static char response[256];

//   for (i=0; response[i]=i<
// }

double checkBattery(){
  //This returns the current voltage of the battery on a Feather 32u4.
  float measuredvbat = analogRead(9);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  
  return (double)measuredvbat;
}

bool startsWith(const char *pre, const char *str)
{
  const char *realStart = str;
  while (isspace(*realStart))
    realStart++;
  return (strncmp(pre, realStart, strlen(pre)) == 0);
}

void standBy(int sleepS) {
  
  int AlarmS = rtc.getSeconds();
  int AlarmM = rtc.getMinutes();
  int AlarmH = rtc.getHours();
  Log.trace("Current time: %d:%d:%d, sleep time %d seconds" CR, AlarmH, AlarmM, AlarmS, sleepS);
  AlarmS += sleepS;       // Adds S seconds to alarm time
  AlarmM += AlarmS / 60;  // Carry any excess over into the minutes
  AlarmS = AlarmS % 60;   // and get the seconds back into range 0-59
  AlarmH += AlarmM / 60;  // Carry excess minutes into hours
  AlarmH = AlarmH % 24;   // and get hours into range 0-23
  Log.trace("Alarm time: %d:%d:%d" CR, AlarmH, AlarmM, AlarmS);
  rtc.setAlarmSeconds(AlarmS); // Wakes at next alarm time
  rtc.setAlarmMinutes(AlarmM);
  rtc.setAlarmHours(AlarmH);
  
  rtc.enableAlarm(rtc.MATCH_HHMMSS); // Match hours, minutes and seconds 
  rtc.attachInterrupt(alarmMatch); // Attach function to interupt
  rtc.standbyMode();    // Sleep until next alarm match

}

void alarmMatch() // Do something when interrupt called
{
         
    
}

int GPS_LOCUS_records() {
  if (!GPS.LOCUS_ReadStatus()) {
    Log.error("GPS.LOCUS_ReadStatus error" CR);
  }
  return GPS.LOCUS_records;
}

