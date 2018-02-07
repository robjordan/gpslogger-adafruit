// Log GPS coordinates to SD card as a GPX file.
//
// Based on: Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
// This version sets up LOCUS logging (autonomous logging to flash by the GPS Featherwing). 
// Then reads the flash at reset, and writes it to a GPX file on SD card, before restarting.

     
#include <Adafruit_GPS.h>
#include <Adafruit_ASFcore.h>
#include <reset.h>
#include <SPI.h>
#include <SdFat.h>
#include <stdlib.h>
#include <string.h>
#include <RTCZero.h>
#include <ArduinoLog.h>
#include <Time.h>
// #include <Math.h>

#define FALSE 0
#define TRUE (!FALSE)
#define EXTLED (18)

// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select 
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
#define SPI_SPEED SD_SCK_MHZ(4)

#define LOCUS_BATCH_TIME 1200 // seconds
#define MAXLINELENGTH 256

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
#define PMTK_SET_NMEA_BAUDRATE_14400 "$PMTK251,14400*29"
#define PMTK_SET_NMEA_BAUDRATE_19200 "$PMTK251,19200*22"
#define PMTK_SET_NMEA_BAUDRATE_38400 "$PMTK251,38400*27"
#define PMTK_SET_NMEA_BAUDRATE_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_BAUDRATE_115200 "$PMTK251,115200*1F"

// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define GPSECHO  true

// GLOBALS
Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();
char buf[256];		// generic buffer for formatting printed output

// File system objects
SdFat sd;		// file system
SdFile sdLogger; 	// logging file
SdFile locusFile;	// temporary file to store locus track output in NMEA format
SdFile gpx;       // to write GPX
const char *locusFilename = "CURRENT.LOC";

// Serial stream for debug output in C++ style
ArduinoOutStream cout(Serial);

// Real-time clock
RTCZero rtc;

// Persistent sequence number, for storing files with unique and sequential names
const uint32_t seqno = 0;

typedef struct point {
  double latitude;
  double longitude;
  int16_t height;
  time_t timestamp;
  uint8_t fix;
};

// Pre-declare functions other than those returning int
boolean create_gpx();
void SD_setup();
int write_trkpt_to_gpx(point *p);
char *filename(const char *suffix);
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

  // initialize digital pin (external LED) to flash 
  pinMode(EXTLED, OUTPUT);
  digitalWrite(EXTLED, HIGH);  // keep it illuminated until we have secured data to SD file
  
  // while (!Serial);   // uncomment to have the sketch wait until Serial is ready
  delay(1000);          // Time for USB connection, GPS to finish reboot, etc.
  
  // Setup the SD card
  SD_setup();
  // connect over USB at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);

  // Create a logger (might be Serial or SD file)
  Log.begin(LOG_LEVEL_VERBOSE, &sdLogger);
  // Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  
  Log.notice("setup(): Check GPS module flash for log points & write to SD card as a GPX file." CR);
  Log.trace("setup(): system_reset_cause: %d" CR, system_get_reset_cause());


  // Establish basic comms with GPS device and handshake to avoid any old, lingering LOCUS data
  GPS_setup(); 

  // Get date/time from GPS. It should be valid, even without a fix, provided on-board battery is good
  rtc.begin();
  GPS_setRTC();
  Log.trace("setup(): Date of wakeup: %d:%d:%d" CR, rtc.getDay(), rtc.getMonth(), rtc.getYear());
  Log.trace("setup(): Time of wakeup: %d:%d:%d" CR, rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

  // If there was a logging activity going on before the reset, now's the time to convert it to GPX.
  // First retrieve any outstanding LOCUS data from the GPS module and store it on SD. Then rename 
  // the LOCUS file
  
  // Open the LOCUS data-in-progress (or create if there isn't one)
  if (!locusFile.open(locusFilename, O_WRITE | O_APPEND | O_CREAT)) {
    Log.error("Unable to open LOCUS file CURRENT.LOC for writing" CR);
  }
  Log.trace("Opened %s, size %d\n" CR, locusFilename, locusFile.fileSize());
  sdLogger.flush();

  // pull any new records off flash and into the current LOCUS file on SD
  process_locus_data();
  sdLogger.flush();
  locusFile.flush();

  // convert the current LOCUS file on SD to GPX
  convert_to_gpx();

  // rename old LOCUS file just as precaution in case data needs to be recovered
  Log.trace("about to close %s, size %d\n" CR, locusFilename, locusFile.fileSize());
  if (!locusFile.close()) {
    Log.error("Unable to close LOCUS file CURRENT.LOC" CR);
  }
  Log.trace("Closed %s, size %d\n" CR, locusFilename, locusFile.fileSize());
  FatFile *fv = sd.vwd();
  char *newfilename = filename("LOC");
  Log.trace("Renaming %s as %s" CR, locusFilename, newfilename);
  if (sd.exists(newfilename)) {
    Log.error("File renaming to exists, wait one second and generate a new name." CR);
    delay(1000);
    newfilename = filename("LOC");
  }
  if (!sd.rename(locusFilename, newfilename)) {
    Log.error("Unable to rename LOCUS file CURRENT.LOC" CR);
  }  
  
  // All the data is now safe, give a reassuring little flash, leaving LED off
  flash(5, 250, 250);
  sdLogger.flush();

  // Re-open the LOCUS file for new data
  if (!locusFile.open(locusFilename, O_WRITE | O_CREAT)) {
    Log.error("Unable to open LOCUS file CURRENT.LOC" CR);
  }

  // Instruct the GPS to start capturing LOCUS data (it may be running already if it was a soft reset
  GPS_startLOCUS();

  Log.trace("setup() exiting" CR);
  sdLogger.flush();
  
}

void loop() {

  Log.trace("loop(): About to sleep, milliseconds: %d" CR, millis());

  // make sure files are flushed before going to sleep
  locusFile.flush();
  sdLogger.flush();

  // now we've handled a batch, sleep until it's next time to unload the LOCUS data
  standBy(LOCUS_BATCH_TIME);

  Log.trace("loop(): Wakeup, milliseconds: %d" CR, millis());
  Log.notice("BATT:,\"%d/%d/%d %d:%d:%d\",%s" CR, 
             rtc.getYear(), 
             rtc.getMonth(), 
             rtc.getDay(), 
             rtc.getHours(), 
             rtc.getMinutes(), 
             rtc.getSeconds(), 
             dtostrf(checkBattery(), 5, 3, buf));
  sdLogger.flush();

  // Check for GPS module reset and re-initialise if it has happened *MISSING*
  //       if (startsWith("$PMTK010,001", GPS.lastNMEA())) { // GPS device has reset
  //         // We need to reinitialise the settings
  //         GPS_setup();
  //       }

  // Loop occurs every wakeup... make sure that we have all in-progress locus data transferred to SD file
  process_locus_data();

  Log.trace("loop() exiting" CR);
  sdLogger.flush();

}

void flash(unsigned repeats, unsigned ontime, unsigned offtime) {
  for (int i=0; i<repeats; i++) {
    digitalWrite(EXTLED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(ontime);                // wait briefly
    digitalWrite(EXTLED, LOW);
    delay(offtime);
  }
}

// Stop LOCUS logging, copy all of the data from flash to SD file (in NMEA format) then restart LOCUS logging
void process_locus_data() {
  int numPoints = 0, recsTransferred = 0;

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
    if ((recsTransferred = append_locus_data_to_file()) > numPoints/6) {
      
      // Seem to be transferred safely, erase the flash
      Log.trace("Copied LOCUS data to file, numPoints=%d, recsTransferred=%d" CR, numPoints, recsTransferred);
      
      GPS.sendCommand(PMTK_LOCUS_ERASE_FLASH);
      
    } else {
      Log.error("Error copying LOCUS data to file, numPoints=%d, recsTransferred=%d" CR, numPoints, recsTransferred);
    }
  } 

  if (!GPS.LOCUS_StartLogger()) {
    Log.error("GPS.LOCUS_StartLogger error" CR);
  }
  
}

// read from SD file up to newline or end-of-file. Return num chars read
int readline(SdFile *sdfile, char *buf, unsigned size) {
  char *c = buf;
  int onebyte;
  int nread = 0;

  // Log.trace("readline()" CR);
  while ( (nread < (size-1)) && ((onebyte=sdfile->read()) >= 0) ) {
    *c++ = (char)onebyte;
    nread++;
    if (onebyte == (int)'\n') {
      break;
    }
  }

  // null-terminate under any circumstances
  *c = '\0';
  // Log.trace("readline() exiting, nread: %d, onebyte %d, buf: %s" CR, nread, onebyte, buf);
  return nread;
}

uint8_t checksum(const char *data) {
  // XOR all the chars in the line except leading $
  uint8_t check = 0;
  for (const char *p=&data[1]; *p != '\0'; p++) {
    check ^= *p;
  }
  return check;
}

// take the first 4 bytes and convert to a 32-bit value
uint32_t parseLong(uint8_t *byteArray) {
  uint32_t longValue = byteArray[3]<<24 | byteArray[2]<<16 | byteArray[1]<<8 | byteArray[0];
  // Log.trace("parseLong returning: %x (%l)" CR, longValue, longValue);
  return (longValue);
}

// take the first 2 bytes and convert to a 16-bit value
int16_t parseInt(uint8_t *byteArray) {
  int16_t shortValue = byteArray[1]<<8 | byteArray[0];
  // Log.trace("parseShort returning: %x (%d)" CR, shortValue, shortValue);
  return (shortValue);
}

double parseFloat(uint8_t *byteArray) {

  // Log.trace("parseFloat()" CR);
  uint32_t longValue = parseLong(byteArray);

  // borrowed code from https://github.com/douggilliland/Dougs-Arduino-Stuff/blob/master/Host%20code/parseLOCUS/parseLOCUS.cpp
  double exponent = ((longValue >> 23) & 0xff); 
  exponent -= 127.0;
  exponent = pow(2,exponent);
  double mantissa = (longValue & 0x7fffff);
  mantissa = 1.0 + (mantissa/8388607.0);
  double floatValue = mantissa * exponent;
  // Log.trace("parseFloat(), high-order bit: %x" CR, (longValue & 0x80000000));
  // Log.trace("parseFloat(), floatValue before hi-bit test: %s" CR, dtostrf(floatValue, 12, 7, buf));
  if ((longValue & 0x80000000) > 0)
    floatValue = -floatValue;

  // Log.trace("parseFloat returning: %s" CR, dtostrf(floatValue, 12, 7, buf));
  return floatValue; 
}

boolean parseBasicDataRecord(point *p, uint8_t *byteArray) {

  boolean result = TRUE;

  // for (int i = 0; i < 16; i++) 
  //   Log.trace("%x ", byteArray[i]);
  // Log.trace("" CR);
  
  p->timestamp = parseLong(&byteArray[0]); // caution, trusting parseLong to take just 4 bytes
  // Log.trace("parseBasicDataRecord(), timestamp: %d" CR, p->timestamp);
  p->fix = byteArray[4];	       // Fix quality: 0 = invalid
			                           // 1 = GPS fix (SPS)
                                 // 2 = DGPS fix
                                 // 3 = PPS fix
			                           // 4 = Real Time Kinematic
			                           // 5 = Float RTK
                                 // 6 = estimated (dead reckoning) (2.3 feature)
			                           // 7 = Manual input mode
			                           // 8 = Simulation mode
  // Log.trace("Fix: %d" CR, p->fix);     
  p->latitude = parseFloat(&byteArray[5]);
  p->longitude = parseFloat(&byteArray[9]);
  p->height = parseInt(&byteArray[13]);
  // Log.trace("Elevation: %d" CR, p->height);

  // some error conditions that should cause point to be discarded
  if (p->timestamp > 4290000000) { // December 2105
    // Log.warning("Invalid timestamp: %l" CR, p->timestamp);
    result = FALSE;
  } else if ((p->fix == 0) || (p->fix > 4)) {
    // Log.warning("Fix quality invalid: %d" CR, p->fix);
    result = FALSE;    
  }

  // Log.trace("parseBasicDataRecord(), result: %b" CR, result);
  return result;
}

int parseLine(char *lbuf) {
  int nbytes = 0;  // number of bytes written
  int npoints = 0; // number of trackpoints written   

  // Log.trace("parseLine()" CR);

  if (startsWith("$PMTKLOX,1", lbuf)) {

    // split the checksum from the data body
    char *data = strtok(lbuf, "*");
    uint8_t actualChecksum = (uint8_t)strtol(strtok(NULL, "*"), NULL, 16);
    // Log.trace("data: %s, checksum: %d" CR, data, actualChecksum);

    // calculate our own checksum from the data
    uint8_t generatedChecksum = checksum(data);
    if (generatedChecksum != actualChecksum) {
      Log.warning("Checksums don't match, discarding line: actual: %x, generated: %x" CR, actualChecksum, generatedChecksum);
    } else {
      // ignore the first three fields
      (void)strtok(data, ",");	// $PMTLOX
      (void)strtok(NULL, ",");	// "1"
      (void)strtok(NULL, ",");	// Record number

      // Now convert the remainder (which should be 24 x 32-bit hex values) to a byte array
      uint8_t byteArray[24*4];
      uint8_t *b = byteArray;
      while (char *hex=strtok(NULL, ",")) {
      	uint32_t value=strtoul(hex, NULL, 16);
      	// Log.trace("Long: %l" CR, value);
      	*b++ = (value&0xFF000000)>>24;
      	*b++ = (value&0x00FF0000)>>16;
      	*b++ = (value&0x0000FF00)>>8;
      	*b++ = (value&0x000000FF);
      	// Log.trace("Bytes: %x %x %x %x" CR, *(b-4), *(b-3), *(b-2), *(b-1));
      }

      // Parse in 16 byte chunks
      unsigned chunkSize = 16;
      
      // no protection against called function extending beyond the 6 bytes... this is C!
      for (int i=0; i<96; i+=chunkSize) {
      	point p;
      	if (parseBasicDataRecord(&p, &byteArray[i])) {
      	  if (write_trkpt_to_gpx(&p) < 0)
            Log.error("Error writing track point to GPX file, timestamp: %d" CR, p.timestamp);
          else
            npoints++;
      	}
      }
    }
  }
  // Log.trace("parseLine() exiting, npoints: %d" CR, npoints);
  return (npoints);
}

void convert_to_gpx() {
  SdFile locus;	// handle for reading LOCUS file
  char inbuf[MAXLINELENGTH];
  int npoints = 0;
  
  Log.trace("convert_to_gpx()" CR);

  if (!locus.open("CURRENT.LOC", O_READ)) {
    Log.error("Unable to open LOCUS file CURRENT.LOC for reading" CR);
  } else if (create_gpx()) {

    while (readline(&locus, inbuf, sizeof inbuf)) {
      // process one line of LOCUS output and write it to GPX
      npoints += parseLine(inbuf);
    }
    
    // write the GPX file tail
    Log.trace("convert_to_gpx(): about to close_gpx()" CR);
    close_gpx();
  }

  Log.trace("convert_to_gpx() exiting, npoints: %d" CR, npoints);
}


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
    // Log.trace(GPS.lastNMEA());
    char *token = strtok(GPS.lastNMEA(), "*,");  // $PMTLOX
    // Log.trace("token: %s" CR, token);
    token = strtok(NULL, "*,");             // 0
    // Log.trace("token: %s" CR, token);
    token = strtok(NULL, "*,");             // Now we've got the record count
    // Log.trace("token: %s" CR, token);
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

boolean create_gpx() {

  boolean isSuccessful;

  // GPX file header
  const char gpxhead[] = "<?xml version=\'1.0\' encoding=\'UTF-8\' standalone=\'yes\' ?>\n<gpx version=\"1.1\"\n creator=\"RMJ-GPXLogger\"\n xmlns=\"http://www.topografix.com/GPX/1/1\"\n xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n<trk>\n<trkseg>\n";

  if (isSuccessful = gpx.open(filename("gpx"), O_WRITE | O_CREAT | O_APPEND)) {
    gpx.write(gpxhead, strlen(gpxhead));
  } else {
    Log.error("Open file failed: %s" CR, filename("gpx"));
  }
  return (isSuccessful); 
}

void close_gpx() {
  const char gpxtail[] = "</trkseg>\n</trk>\n</gpx>\n";

  gpx.write(gpxtail, strlen(gpxtail));
  gpx.close();
}

char *filename(const char *suffix) {
  static char filename[19];  // 15+3 filename and a null terminator
  char _suffix[4];

  // truncate suffix at 3 characters just in case we are passed a long one.
  strncpy(_suffix, suffix, sizeof _suffix);
  sprintf(filename, "%04d%02d%02d-%02d%02d%02d.%s", 
    rtc.getYear()+2000, 
    rtc.getMonth(), 
    rtc.getDay(), 
    rtc.getHours(),
    rtc.getMinutes(),
    rtc.getSeconds(), 
    _suffix
    );
  
  return(filename);
}

int write_trkpt_to_gpx(point *p) {
  const char trkpt[] = "  <trkpt lat=\"%s\" lon=\"%s\">\n\t<ele>%d</ele>\n\t<time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n  </trkpt>\n";
  char s[sizeof(trkpt) + 32];
  char lat[16];
  char lon[16];
  char ele[16];
  TimeElements tm;

  Log.trace("write_trkpt_to_gpx(): %l" CR, p->timestamp);
  breakTime(p->timestamp, tm);
  sprintf(s, trkpt, 
    dtostrf(p->latitude, 12, 7, lat),
    dtostrf(p->longitude, 12, 7, lon),
    p->height, 
    tm.Year+1970, 
    tm.Month, 
    tm.Day, 
    tm.Hour, 
    tm.Minute, 
    tm.Second);
  return(gpx.write(s, strlen(s)));
  // Log.trace("write_trkpt_to_gpx() exiting: %s" CR, s);
}


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
  if (!sdLogger.open("GPSlogger.log", O_WRITE | O_APPEND | O_CREAT)) {
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
 *date = FAT_DATE(rtc.getYear()+2000, rtc.getMonth(), rtc.getDay());

 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
}


void GPS_setup() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // Reboot the GPS module to avoid any output from old LOCUS fetch commands
  GPS.sendCommand(PMTK_CMD_HOT_START);
  GPS.waitForSentence("$PMTK010,001");

  // Increase the communication speed with GPS module to get the data quicker
  // GPS.sendCommand(PMTK_SET_NMEA_BAUDRATE_19200);
  // GPS.begin(19200);
  // GPS.waitForSentence("$PMTK001,251");// Confirmation that PMTK_SET_NMEA_BAUDRATE completed
  
  // Ask for firmware version as handshake
  GPS.sendCommand(PMTK_Q_RELEASE);
  GPS.waitForSentence("$PMTK705");

  
}

void GPS_setRTC() {
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
  
  // $GPRMC appears to emit a plausible date/time even in absence of a proper fix
  // using battery-backed RTC no doubt. So no need to wait for fix. Just check for sane values.
  do {
    GPS.waitForSentence("$GPRMC");
    GPS.parse(GPS.lastNMEA());
    Log.trace("Date from $GPRMC: %d/%d/%d" CR, GPS.day, GPS.month, GPS.year);
  } while (GPS.year < 11 || GPS.year > 79); // see https://forums.adafruit.com/viewtopic.php?f=19&t=28088
  rtc.setDate(GPS.day, GPS.month, GPS.year);
  Log.trace("Date from RTS: %d/%d/%d" CR, rtc.getDay(), rtc.getMonth(), rtc.getYear());
  rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_OFF);
}

void GPS_startLOCUS() {
  // Now switch to LOCUS mode, turning off normal NMEA traffic
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_OFF);
  GPS.waitForSentence("$PMTK001,314");  
  GPS.sendCommand(PMTK_LOCUS_INTERVAL_5s); // One reading every 5 seconds
  GPS.waitForSentence("$PMTK001,187");
     
  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
}


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
  AlarmS %= 60;           // and get the seconds back into range 0-59
  
  AlarmH += AlarmM / 60;  // Carry excess minutes into hours
  AlarmM %= 60;           // and get th eminutes back into range 0-59
  AlarmH %= 24;           // and get hours into range 0-23
  Log.trace("Alarm time: %d:%d:%d" CR, AlarmH, AlarmM, AlarmS);
  sdLogger.flush();
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

