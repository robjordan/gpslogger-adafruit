# gpslogger-adafruit
GPS logger using Adafruit M0 Feather Datalogger and GPS Ultimate Featherwing

Notes on LOCUS flash capacity - for the MKT3339 GPS device to autonomously store track points 
without host microcontroller intervention.

The PA6H data sheet (http://thatsentropy.co.uk/info/GlobalTop-FGPMMOPA6H-Datasheet-V0A.pdf) doesn't specify 
Flash capacity but says "max log days can up to 2 days under AlwaysLocateTM condition. Data size per log was 
shrunk from 24 bytes to 15 bytes."

Adafruit Ultimate GPS (same MKT3339 module) says (https://www.adafruit.com/product/746) "The time, date, 
longitude, latitude, and height is logged every 15 seconds and only when there is a fix. The internal FLASH 
can store about 16 hours of data." This implies there is capacity for 16 * 60 * 60 / 15 = 3840 location log entries.

Analysis of the raw and parsed output using the Adafruit online parser 
(https://learn.adafruit.com/custom/ultimate-gps-parser) shows that:
1365 raw log output sentences, each with 96 bytes, translates into 8186 log entries, thus:
One raw log sentence contains 6 location log entries.
Location log entries consume on average 16 bytes.

Based on the Adafruit statement, the capacity is: 3840 * 16 = 60 kBytes.
However I have seen 8186 * 16 = 128 kBytes.

8186 log entries at 5 second logging interval will last 11 hours.
