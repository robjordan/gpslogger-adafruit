// Log GPS coordinates to SD card as a GPX file.
//
// Based on: Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//

     
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>
#include <stdlib.h>
#include <string.h>

#define FALSE 0
#define TRUE (!FALSE)
#define PMTK_SET_Nav_Speed_0_0 "$PMTK386,0*23"
#define PMTK_SET_Nav_Speed_0_2 "$PMTK386,0.2*3F"
#define PMTK_SET_Nav_Speed_0_4 "$PMTK386,0.4*39"
#define PMTK_SET_Nav_Speed_0_6 "$PMTK386,0.6*3B"
#define PMTK_SET_Nav_Speed_0_8 "$PMTK386,0.8*35"
#define PMTK_SET_Nav_Speed_1_0 "$PMTK386,1.0*3C"
#define PMTK_SET_Nav_Speed_1_5 "$PMTK386,1.5*39"
#define PMTK_SET_Nav_Speed_2_0 "$PMTK386,2.0*3F"

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// Dump the contents of the GPX file after each write, to save removing SD card
#ifdef DEBUG_GPX
#define PRINT_GPX(gpx) int savepos = gpx.position(); gpx.seek(0); while (gpx.available()) {Serial.write(gpx.read());}; gpx.seek(savepos);
#else
#define PRINT_GPX(gpx)
#endif

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// GLOBALS

// Connect to the GPS on the hardware port
// GPS data structure is global, as is the file handle for our GPX file
Adafruit_GPS GPS(&GPSSerial);
File gpx;
File debug;
bool gpx_open = FALSE;
uint32_t timer = millis();
Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = 4;

// Pre-declare functions other than those returning int
File create_gpx();
void SD_setup();
void write_trkpt_to_gpx();
char *gpx_filename();
void write_trkpt_to_gpx();
void write_to_gpx(const char *s);
void print_GPS();
char *dtostrf(double __val,signed char __width,unsigned char __prec,char * __s);
char *convert_coord(double nmea, char compass, char *s);
char *dtostrf(double val, int width, unsigned int prec, char *sout);
void dateTime(uint16_t* date, uint16_t* time);
void SD_println(const char *s);
void GPS_setup();
float checkBattery();
void SD_print_battery();
bool startsWith(const char *pre, const char *str);


void setup()
{

  
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Log GPS coordinates to SD card as a GPX file.");
     
  // Setup the GPS device
  GPS_setup();

  // Setup the SD card
  SD_setup();
}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
  
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    SD_println(GPS.lastNMEA());
    
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
    else {

      
      // We have received and parsed a sentence. Does it have date? If we haven't created a dated GPX file, create it now
      if (!gpx_open && GPS.fix && GPS.year) {
        SdFile::dateTimeCallback(dateTime);
        if (create_gpx ())
          gpx_open = TRUE;
          PRINT_GPX(gpx);
      }

      // Now log the new data to the gpx file
      if (gpx_open && GPS.fix && startsWith("$GPGGA", GPS.lastNMEA())) {
        write_trkpt_to_gpx();
        PRINT_GPX(gpx);
        SD_print_battery();
        return;
      }
      
      if (startsWith("$PMTK010,001", GPS.lastNMEA())) { // GPS device has reset
        // We need to reinitialise the settings
        GPS_setup();
      }
 
    }
  }
}

File create_gpx() {

  // GPX file header
  const char gpxhead[] = "<?xml version=\'1.0\' encoding=\'UTF-8\' standalone=\'yes\' ?>\n<gpx version=\"1.1\"\n creator=\"RMJ-GPXLogger\"\n xmlns=\"http://www.topografix.com/GPX/1/1\"\n xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n<trk>\n<trkseg>\n";

  gpx = SD.open(gpx_filename(), FILE_WRITE);
  if (gpx) {
    write_to_gpx(gpxhead);
  } else {
    Serial.print("Open file failed: "); Serial.println(gpx);
  }
  return (gpx); 
}

char *gpx_filename() {
  static char filename[13];  // 8+3 filename and a null terminator
  static uint8_t seqno = 0;
  const char alphabet[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

  sprintf(filename, "%02d%02d%02d%c%c.gpx", 
    GPS.year, 
    GPS.month, 
    GPS.day, 
    alphabet[(GPS.hour*60+GPS.minute)/sizeof(alphabet)], 
    alphabet[(GPS.hour*60+GPS.minute)%sizeof(alphabet)]);
  Serial.print("GPX filename: "); Serial.println(filename);
  return(filename);
}

void write_trkpt_to_gpx() {
  const char trkpt[] = "  <trkpt lat=\"%s\" lon=\"%s\">\n\t<ele>%s</ele>\n\t<time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>\n  </trkpt>\n";
  char s[sizeof(trkpt) + 32];
  char lat[16];
  char lon[16];
  char ele[16];

  print_GPS();
  // sprintf(s, trkpt, GPS.latitude, GPS.longitude, GPS.altitude, GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
  
  sprintf(s, trkpt, 
    convert_coord(GPS.latitude, GPS.lat, lat), 
    convert_coord(GPS.longitude, GPS.lon, lon), 
    dtostrf(GPS.altitude,6,1,ele), 
    GPS.year+2000, 
    GPS.month, 
    GPS.day, 
    GPS.hour, 
    GPS.minute, 
    GPS.seconds);
  write_to_gpx(s);
}

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

void write_to_gpx(const char *s) {
  
  const char gpxtail[] = "</trkseg>\n</trk>\n</gpx>\n";
  
  if (gpx) {
    unsigned int l = strlen(s);
    gpx.write(s, l);
    Serial.print("Wrote: "); Serial.println(l);
    Serial.print(s);
    // Serial.print("Position: "); Serial.println(gpx.position());
    l = strlen(gpxtail);
    gpx.write(gpxtail, l);
    // Serial.print("Wrote: "); Serial.println(l);
    // Serial.print("Position: "); Serial.println(gpx.position());
    gpx.seek(gpx.position() - l);
    // Serial.print("Seeking, position: "); Serial.println(gpx.position());
    gpx.flush();

  } else {
    Serial.print("GPX file not open: "); Serial.println(gpx);
  }  
  return;
  
}

void SD_setup()
{
  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    return;
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.println();
  Serial.print("Card type:         ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return;
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());

  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  Serial.print("Volume size (Kb):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (Mb):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);

  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
  } else {
    Serial.println("card initialized.");
  }
}

void print_GPS() {
  Serial.print("\nTime: ");
  Serial.print(GPS.hour, DEC); Serial.print(':');
  Serial.print(GPS.minute, DEC); Serial.print(':');
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  Serial.println(GPS.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
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

void SD_println(const char *s) {

  char filename[13];

  if (!debug) {
    randomSeed(analogRead(14));
    unsigned seqno = random(99999999);
    sprintf(filename, "%08d.NM", seqno++);
    debug = SD.open(filename, FILE_WRITE);
  }

  if (debug) {
    unsigned l = strlen(s);
    debug.write(s, l);
    debug.write("\n", 1);
    debug.flush();
  } else {
    Serial.print("Open debug file failed: ");
  }
}

void GPS_setup() {
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand( PMTK_API_SET_FIX_CTL_200_MILLIHERTZ); // One fix every 5 seconds
  GPS.sendCommand( PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); // One reading every 5 seconds
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  GPS.sendCommand(PMTK_SET_Nav_Speed_0_6);  // Suppress readings where movement is < 0.6 m/s
     
  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

float checkBattery(){
  //This returns the current voltage of the battery on a Feather 32u4.
  float measuredvbat = analogRead(9);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  
  return measuredvbat;
}

void SD_print_battery() {
  char* fmt="BATT:,\"%04d/%02d/%02d %02d:%02d:%02d\",%s\n"; 
  char line[40];
  char batt[8];

  // Serial.println("pre-checkBattery");
  dtostrf((double)checkBattery(), 5, 3, batt);
  // Serial.println("post-checkBattery");
  // Serial.println(batt);
  sprintf(line, fmt, 
    GPS.year+2000, 
    GPS.month, 
    GPS.day, 
    GPS.hour, 
    GPS.minute, 
    GPS.seconds,     
    batt);
  SD_println(line);
  Serial.println(line);
}

bool startsWith(const char *pre, const char *str)
{
  bool result;
  const char *realStart = str;
  while (isspace(*realStart))
    realStart++;
  // Serial.print("Pre: "); Serial.print("#");Serial.print(pre); Serial.println("#");
  // Serial.print("Str: "); Serial.print("#");Serial.println(realStart);Serial.println("#");
  result = (strncmp(pre, realStart, strlen(pre)) == 0);
  // Serial.print("Rslt: "); Serial.println(result?1:0);
  return result;
}

