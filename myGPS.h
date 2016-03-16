/*
 * myGPS.h
 *
 * Created: 3/8/2016 8:59:35 PM
 *  Author: Riley
 */ 


#ifndef MYGPS_H_
#define MYGPS_H_

#include <stdint.h>
#include <math.h>


#define IS_PMTK_PACKET	1
#define KNOT2MPH		1.15077945F
#define DEG2RAD			(M_PI/180)
#define RAD2DEG			(180/M_PI)
#define R_EARTH			6371000.0
#define METERS2MILES	0.00062137119F

#define MAG_DECLINATION	3.22F

/* gps_t status_flags defines */
#define GPS_VALID		0x01
#define	GPS_UNREAD		0X02
#define GPS_NEW_DEST	0x04

/* Account for time zone change in SD and Daylight Savings Time 3/13->11/6 of any year */
#define SOUTH_DAKOTA_MOUNTAIN_TIME_LAT	100.368793F
#define CENTRAL_TIME_UTC				6 //UTC-6:00
#define MOUNTAIN_TIME_UTC				7 //UTC-7:00

/* Example: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A  */


/*!
* @brief struct for parsed GPS data
*/
struct gps_t {
	uint8_t hours;		 /* 12 Hr time; Accounts for CT/MT zones and DST */
	uint8_t minutes;
	uint8_t seconds;
	char  GPSvalidity;	/* Status A=active or V=Void. I also use this for */
						/* 0 = invalid packet, IS_PMTK_PACKET = PMTK packet */
	float latitude_deg;
	char lat_N_S;
	float longitude_deg;
	char lon_E_W;
	float speed_mph;	/* Conversion factor KNOT2MPH 1.15077945F */
	float gps_bearing;	/* Degrees from North, i.e. 90* = East */
	uint8_t day;
	uint8_t month;
	uint8_t year;		/* 20xx */
	float gps_mag_variation;
	char mag_E_W;
	char fix_type;		/* A=autonomous, D=differential, E=Estimated, N=not valid, S=Simulator, or can be null */
	uint8_t status_flags; //0b87654321 1= 
	/*
	* 1= 0x01=	valid gps data available			GPS_VALID
	* 2= 0x02=	new gps data available				GPS_UNREAD
	* 3= 0x04=	new destination has been entered	GPS_NEW_DEST
	* 4= 0x08=
	* 5= 0x10=
	* 6= 0x20=
	* 7= 0x40=
	* 8= 0x80=
	*/
};

/*!
* @brief struct to put the destinations in.  Initialize one for every destination you want
*/
//TODO:: add _enum??
struct dest_t{
	float latitude_deg;
	float longitude_deg;
	char name[20];
	//uint8_t completed?
};


/*!
* @brief struct for distance t
*/
struct directions_t{
	float distance;
	float bearing;
};


/* GPS commands */
// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F\r\n" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B\r\n"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"
// Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C\r\n" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18\r\n"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C\r\n"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F\r\n"
// Can't fix position faster than 5 times a second!

#define PMTK_SET_BAUD_115200 "$PMTK251115200*33\r\n" //I haven't gotten this to work
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C\r\n" //not sure how to reliably switch to this in all cases
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17\r\n"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"

// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html

#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22\r\n"
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23\r\n"
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C\r\n"
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38\r\n"
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22\r\n"
#define LOCUS_OVERLAP 0
#define LOCUS_FULLSTOP 1

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E\r\n"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E\r\n"

// standby command & boot successful message
#define PMTK_STANDBY "$PMTK161,0*28\r\n"
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36\r\n"  // Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D\r\n"

// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31\r\n"

// request for updates on antenna status
#define PGCMD_ANTENNA "$PGCMD,33,1*6C\r\n"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D\r\n"



/* function prototypes */
int8_t parse_nmea_rmc(char *nmea_buffer, struct gps_t *gps);
float degreeString2float(char *degString);
void print_gps_data(struct gps_t *gps);
void haversine(struct gps_t *gps, struct dest_t *dest, struct directions_t *directions);


#endif /* MYGPS_H_ */