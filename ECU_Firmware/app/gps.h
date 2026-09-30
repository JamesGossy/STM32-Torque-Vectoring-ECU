/* NMEA reader for the ATGM336H GPS module. Only the RMC sentence is used: fix,
   position, ground speed and course. The control loop does not use GPS yet;
   it is shown on the console for checking the receiver. */
#ifndef GPS_H
#define GPS_H

#include <stdint.h>

typedef struct {
    int valid;      // receiver reports a fix
    double lat_deg; // north positive
    double lon_deg; // east positive
    float speed_ms;
    float course_deg;
    uint32_t sentences; // good RMC sentences seen, to check the link is alive
} GpsFix;

void gps_init(void);
void gps_feed(char c);
const GpsFix *gps_fix(void);

#endif
