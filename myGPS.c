/*
 * myGPS.c
 *
 * Created: 3/8/2016 8:59:25 PM
 *  Author: Riley
 */ 

#include "myGPS.h"
#include "uart.h"
#include <stdlib.h>
#include <string.h> //for strcmp


/* Example: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A  */

/*!
 *	Function: parse_nmea_rmc
 *	@brief This function takes a NMEA sentence and puts the data into the gps_t struct
 *		passed to it.
 *		
 *		If the data isn't an RMC sentence, we abort and return -1;
 *		If the data is a single character, it is put directly into the gps_t struct
 *		If the data is a string/number, the function first puts the chunk of the NMEA
 *			string into its own string.  Then it uses string functions like atof to
 *			get numbers and puts them into the gps_t struct.
 *		Some data processing is also done:
 *			-GPS Validity is used to pass info of a PMTK acknowledgment or invalid packet
 *			-Speed is converted from knots to MPH
 *			-Latitude and Longitude are converted from Degrees/Minutes to decimal degrees
 *				using the degreeString2float function.
 *			-Time is separated into hours, minutes, seconds, formatted to 12hr, and
 *				adjusted for DST and Mountain/Central time zone.
 *
 *		I couldn't get the checksum whoring right now, so I commented it out
 *
 *		Note: My NMEA sentences aren't including magnetic variation data for some reason
 *
 *	@param *nmea_buffer : A pointer to your nmea string
 *	@param *gps : A pointer to your gps struct
 *
 *	@return floatDegrees : a floating point value of degrees
 */

int8_t parse_nmea_rmc(char *nmea_buffer, struct gps_t *gps)
{
	uint8_t nmea_index = 0;
	uint8_t index = 0;
	int8_t utc_time_correction;
	//uint16_t nmea_checksum = 0;

	char sentence[7] = {0};
	char time[12] = {0};
	char latitude[10] = {0};
	char longitude[12] = {0};
	char speed[8] = {0};
	char bearing[8] = {0};
	char date[8] = {0};
	char variation[8] = {0};
	//char checksum[3] = {0};

	/* if we're passed a null string terminate (this happens on startup) */
	if(nmea_buffer[0] == '\0')
		return 0;
	
	/* Extract NMEA sentence type */
	while(nmea_buffer[nmea_index] != ',') 
	{
		sentence[index++] = nmea_buffer[nmea_index++];
	} 
	
	sentence[index] = '\0';
	index = 0;
	nmea_index++;

	/* If it's not a RMC sentence, abort */
	if(strcmp(sentence,"$GPRMC") != 0)
	{
		if(strcmp(sentence,"$PMTK001") == 0)
		{
			gps->GPSvalidity = IS_PMTK_PACKET;
			uart_puts(nmea_buffer); //print an acknowledgment packet
		}
		else		
			gps->GPSvalidity = 0; //not valid or invalid
		return -1;
	}

	/* Extract the time */
	while(nmea_buffer[nmea_index] != ',')
	{
		time[index++] = nmea_buffer[nmea_index++];
	}
	
	time[index] = '\0';
	index = 0;
	nmea_index++;
	
	/* Extract validity, A=Active, V=void.  Store in gps_t struct */
	if(nmea_buffer[nmea_index] != ',')
		gps->GPSvalidity = nmea_buffer[nmea_index++];
	nmea_index++;
	
	/* Extract the latitude */
	while(nmea_buffer[nmea_index] != ',')
	{
		latitude[index++] = nmea_buffer[nmea_index++];
	}
	
	latitude[index] = '\0';
	index = 0;
	nmea_index++;
	
	/* Extract the latitude's direction, Store in gps_t struct */
	if(nmea_buffer[nmea_index] != ',')
		gps->lat_N_S = nmea_buffer[nmea_index++];
	nmea_index++;
	
	/* Extract the longitude */
	while(nmea_buffer[nmea_index] != ',')
	{
		longitude[index++] = nmea_buffer[nmea_index++];
	}
	
	longitude[index] = '\0';
	index = 0;
	nmea_index++;
	
	/* Extract the longitude's direction, Store in gps_t struct */
	if(nmea_buffer[nmea_index] != ',')
		gps->lon_E_W = nmea_buffer[nmea_index++];
	nmea_index++;
	
	/* Extract the speed */
	while(nmea_buffer[nmea_index] != ',')
	{
		speed[index++] = nmea_buffer[nmea_index++];
	}
	
	speed[index] = '\0';
	index = 0;
	nmea_index++;
	
	/* Extract the bearing (heading, direction, yaw) */
	while(nmea_buffer[nmea_index] != ',')
	{
		bearing[index++] = nmea_buffer[nmea_index++];
	}
	
	bearing[index] = '\0';
	index = 0;
	nmea_index++;
	
	/* Extract the date */
	while(nmea_buffer[nmea_index] != ',')
	{
		date[index++] = nmea_buffer[nmea_index++];
	}
	
	date[index] = '\0';
	index = 0;
	nmea_index++;
	
	/* Extract the magnetic variation */
	while(nmea_buffer[nmea_index] != ',')
	{
		variation[index++] = nmea_buffer[nmea_index++];
	}
	
	variation[index] = '\0';
	index = 0;
	nmea_index++;
	
	/* Extract the mag variation's direction, Store in gps_t struct, default to E */
	if(nmea_buffer[nmea_index] != ',')
		gps->mag_E_W = nmea_buffer[nmea_index++];
	else
		gps->mag_E_W = 'E';
	nmea_index++;
	
	/* Extract the Fix Type, Store in gps_t struct*/
	if(nmea_buffer[nmea_index] != ',')
		gps->fix_type = nmea_buffer[nmea_index++];
	nmea_index++;

	/* Extract the Checksum */
	//checksum[0] = nmea_buffer[nmea_index++];
	//checksum[1] = nmea_buffer[nmea_index];
	//checksum[2] = '\0';

	/* Convert these parameters and store them in the struct */
	gps->latitude_deg = degreeString2float(latitude);
	gps->longitude_deg = degreeString2float(longitude);
	gps->speed_mph = atof(speed) * KNOT2MPH;
	gps->gps_bearing = atof(bearing);
	gps->gps_mag_variation = atof(variation);
	
	/* Convert the date and store the parts in their place in the struct */
	uint32_t u32date = atol(date);
	gps->day =  (u32date/10000);
	gps->month = (u32date % 10000) / 100;
	gps->year = u32date % 100;
	
	/* Convert time, account for DST and CT/MT time zones, make 12 hr, and store in struct */
	float ftime = atof(time);
	uint32_t u32time = ftime; //truncate the decimal
	
	/* If we're west of Pierre, SD's latitude, we're in Mountain time */
	if(gps->latitude_deg > SOUTH_DAKOTA_MOUNTAIN_TIME_LAT)
		utc_time_correction = MOUNTAIN_TIME_UTC * -1;
	else
		utc_time_correction = CENTRAL_TIME_UTC * -1;
	
	/* If the date is between March 3 and Nov 11, add an hour for DST */
	if((gps->month > 3 && gps->month < 11) || (gps->month == 3 && gps->day >= 13) ||
		 (gps->month == 11 && gps->day <= 6))
	{
		utc_time_correction += 1;	//advance an hour in the summer	DST
	}
	
	gps->hours = (u32time/10000) + utc_time_correction;
	if(gps->hours > 12)
		gps->hours -= 12;
	gps->minutes = (u32time % 10000) / 100;
	gps->seconds = u32time % 100;
	/* Skip milliseconds */
	
	/* Convert and check checksum */
	/*
	nmea_checksum = atoi(checksum);

	uint8_t asterisk_location = (strlen(nmea_buffer)-5);
	
	for (index=1; index <= asterisk_location; index++)
	{
		nmea_checksum ^= nmea_buffer[index];
	}
	*/
	
	gps->status_flags |= GPS_UNREAD;		//set the 'new gps data available' flag
	
	return 0;
}




/*!
 *	Function: degreeString2float
 *	@brief This function converts the NMEA formatted latitude or longitude
 *		to straight decimal degrees.
 *		
 *		The NMEA format is DDMM.MMMM, D=degrees, M=minutes (the amount of digits may vary)
 *		A minute is 1/60 a degree
 *		the equation to convert to degrees is DD+(MM.MMMM)/60 = DD.DDDDDD
 *
 *	@param *degString : A pointer to your nmea string format DDMM.MMMM where
 *		the number of digits can vary, as long as there are exactly two minute digits
 *		on the left side of the decimal point (I believe this is NMEA standard)
 *
 *	@return floatDegrees : a floating point value of degrees
 */

float degreeString2float(char *degString)
{		
	/* If we're given a null string, return 0 */
	if(degString[0] == '\0')
		return 0;
		
	uint8_t index = 0;
	
	char nmea_deg[11] = {0};
	char nmea_min[11] = {0};
	
	/* Copy the input string into a new string until we read the decimal point */
	while(degString[index] != '.')
	{	
		nmea_deg[index] = degString[index];
		index++;
	}
	
	/* Go back two decimal places and put a null terminator there */
	nmea_deg[index-2] = '\0';
	index-=2;
	uint8_t wholePart = index;	

	/* Now put the minutes into a new string, two digits, decimal point, then the rest */
	while(degString[index] != '\0')
	{
		nmea_min[index-wholePart] = degString[index];
		index++;
	}
	nmea_min[index] = '\0';
	
	/* Combine the degrees and the minutes and return the decimal degrees */
	float floatDegrees = atof(nmea_deg) + atof(nmea_min)/60;
	
	return floatDegrees;
}



/*!
 *	Function: print_gps_data
 *	@brief This function prints all the data in the gps struct, formatted
 *
 *	@param *gps : A pointer to your gps_t struct
 *
 *	@return void
 */
void print_gps_data(struct gps_t *gps)
{
	char print_buffer[100] = {0};
		
	if(gps->GPSvalidity != 'A')
	{
		if(gps->GPSvalidity == 'V')
			uart_puts("Invalid GPS data\r\n");	
		else if(gps->GPSvalidity == IS_PMTK_PACKET)
			{}//print the packet in the parse function
		else
			uart_puts("Not an RMC Sentence\r\n");
				
		return;	
	}

	/* Print Time HH:MM:SS, use a leading 0 if needed*/
	uart_puts("Time: ");
	if(gps->hours < 10)
		uart_putc('0');
	uart_puts(itoa(gps->hours,print_buffer,10));
	uart_puts(":");
	if(gps->minutes < 10)
		uart_putc('0');
	uart_puts(itoa(gps->minutes,print_buffer,10));
	uart_puts(":");
	if(gps->seconds < 10)
		uart_putc('0');
	uart_puts(itoa(gps->seconds,print_buffer,10));
	
	/* Print Validity A=Active, V=Void */
	uart_puts("\r\nActive/Void: ");
	uart_putc(gps->GPSvalidity);
	
	/* Print Latitude in degrees to 4 decimal places */
	uart_puts("\r\nLatitude: ");
	uart_puts(dtostrf(gps->latitude_deg,1,4,print_buffer));
	uart_putc(gps->lat_N_S);
	
	/* Print Longitude in degrees to 4 decimal places */
	uart_puts("\r\nLongitude: ");
	uart_puts(dtostrf(gps->longitude_deg,1,4,print_buffer));
	uart_putc(gps->lon_E_W);
	
	/* Print Speed in MPH to 1 decimal place */
	uart_puts("\r\nSpeed: ");
	uart_puts(dtostrf(gps->speed_mph,1,1,print_buffer));
	uart_puts(" MPH");
	
	/* Print Bearing in degrees to 1 decimal place */
	/* Bearing is degrees clockwise from North, i.e. East is 90* */
	uart_puts("\r\nBearing: ");
	uart_puts(dtostrf(gps->gps_bearing,1,1,print_buffer));
	
	/* Print Date MM/DD/YYYY */
	uart_puts("\r\nDate: ");
	uart_puts(itoa(gps->month,print_buffer,10));
	uart_puts("/");
	uart_puts(itoa(gps->day,print_buffer,10));
	uart_puts("/20");
	uart_puts(itoa(gps->year,print_buffer,10));
	
	/* Print Magnetic Variation in Degrees */
	uart_puts("\r\nVariation: ");
	uart_puts(dtostrf(gps->gps_mag_variation,1,1,print_buffer));
	uart_putc(gps->mag_E_W);
	uart_puts("\r\n");

}

/*!
 *	Function: haversine
 *	@brief This function finds the great-circle distance and bearing between two points (radians)
 *		on a sphere in meters
 *
 *	@param *gps : A pointer to your gps_t struct
 *	@param *dest : a pointer to your dest_t struct, which contains the lat/lon of your destination
 *	@param *dest : a pointer to your directions_t struct, which contains distance/bearing to your destination
 *
 *	Pass in your gps_t data as your coordinates
 *	Pass in your dest_t data as the destination
 *	The resulting distance (in meters) and bearing (in degrees) will get put in the directions_t struct
 *
 *	@return void
 */
void haversine(struct gps_t *gps, struct dest_t *dest, struct directions_t *directions) //pass in lat/lon in degrees
{
	float myLat = gps->latitude_deg * DEG2RAD;
	float myLon = gps->longitude_deg * DEG2RAD;
	float d_Lat = dest->latitude_deg * DEG2RAD;
	float d_Lon = dest->longitude_deg * DEG2RAD;

	/* Calculate distance with the haversine equation */
	float h = square((sin((myLat - d_Lat) / 2.0))) + (cos(myLat) * cos(d_Lat) * square((sin((myLon - d_Lon) / 2.0))));
	directions->distance = 2.0 * R_EARTH * asin (sqrt(h));
	
	/* calculate bearing */
	float y = sin(myLon - d_Lon) * cos(d_Lat);
	float x = cos(myLat) * sin(d_Lat) - sin(myLat)*cos(d_Lat)*cos(myLon-d_Lon);
	directions->bearing = atan2(y,x); //bearing is a global
	directions->bearing *= RAD2DEG;
	
	//Serial.print("bearing: ");
	//Serial.print(bearing);
	
	/* Bearing will range from 0-180*, positive if clockwise from North and negitive if CCW */
	/* To fix the bearing from 0-360* Clockwise from North, add 360 if it's negative  */
	if(directions->bearing < 0)
		directions->bearing += 360;
	
	//Serial.print("fixed bearing: ");
	//Serial.print(bearing);
	
	//Serial.print("\tdistance: ");
	//Serial.println(distance);
}