/* Collects one NMEA line at a time, checks its checksum, then splits the RMC
   fields. Any talker id is accepted ($GPRMC, $GNRMC, $BDRMC). */
#include "gps.h"
#include <stdlib.h>
#include <string.h>

#define LINE_MAX    96
#define KNOTS_TO_MS 0.514444f

static char line[LINE_MAX];
static int length;
static GpsFix fix;

void gps_init(void)
{
    length = 0;
    memset(&fix, 0, sizeof fix);
}

const GpsFix *gps_fix(void)
{
    return &fix;
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Checks "$...*hh" and cuts the line at the '*'.
static int checksum_ok(char *text)
{
    char *star = strchr(text, '*');
    if (text[0] != '$' || !star || hex_value(star[1]) < 0 || hex_value(star[2]) < 0) return 0;
    uint8_t sum = 0;
    for (char *p = text + 1; p < star; p++)
        sum ^= (uint8_t)*p;
    *star = 0;
    return sum == hex_value(star[1]) * 16 + hex_value(star[2]);
}

// NMEA angles are ddmm.mmmm, so the degrees are everything before the last two digits.
static double to_degrees(const char *text, int degree_digits)
{
    if (strlen(text) <= (size_t)degree_digits) return 0.0;
    char head[4] = { 0 };
    memcpy(head, text, (size_t)degree_digits);
    double minutes = atof(text + degree_digits);
    return atof(head) + minutes / 60;
}

// Splits on commas in place, keeping empty fields.
static int split(char *text, char **fields, int most)
{
    int count       = 0;
    fields[count++] = text;
    for (char *p = text; *p && count < most; p++) {
        if (*p == ',') {
            *p              = 0;
            fields[count++] = p + 1;
        }
    }
    return count;
}

static void parse_rmc(char **f)
{
    // $xxRMC,time,status,lat,N/S,lon,E/W,speed knots,course,date,...
    fix.valid  = f[2][0] == 'A';
    double lat = to_degrees(f[3], 2), lon = to_degrees(f[5], 3);
    fix.lat_deg    = f[4][0] == 'S' ? -lat : lat;
    fix.lon_deg    = f[6][0] == 'W' ? -lon : lon;
    fix.speed_ms   = (float)atof(f[7]) * KNOTS_TO_MS;
    fix.course_deg = (float)atof(f[8]);
    fix.sentences++;
}

static void handle_line(void)
{
    char *fields[20];
    if (!checksum_ok(line)) return;
    int count = split(line, fields, 20);
    if (count >= 10 && strlen(fields[0]) == 6 && !strcmp(fields[0] + 3, "RMC")) parse_rmc(fields);
}

void gps_feed(char c)
{
    if (c == '$') length = 0; // a new sentence always restarts the line
    if (c == '\r' || c == '\n') {
        if (length > 0) {
            line[length] = 0;
            handle_line();
        }
        length = 0;
    } else if (length < LINE_MAX - 1) {
        line[length++] = c;
    } else {
        length = 0; // too long to be NMEA, throw it away
    }
}
