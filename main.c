/*
 * ECompass_V1
 *
 * Created: 3/8/2016 12:35:43 PM
 * Author : Riley
 *
 *	Note: This was written for my own project, apologies for bad code/poor flexibility
 *	If I had to do it again, I might have rather used ARM (Arduino Due) instead of AVR
 *
 *	Operating System: Windows 10
 *	Editor: Atmel Studio 7
 *
 *	Hardware: AVR ATMega2560 @16MHZ (Arduino Mega)
 *
 *	Hardware: BNO055 hooked up to I2C
 *		Note: check level shifting, I'm using Adafruit's board
 *		https://www.adafruit.com/products/2472
 *
 *	Hardware:  HD44780 LCD connected in 4-wire mode to an MCP23017 I/O expander
 *		which is connected to the 2560 via I2C.
 *		I'm using an Adafruit kit:
 *		"RGB LCD Shield Kit w/ 16x2 Character Display - Only 2 pins used! - POSITIVE DISPLAY"
 *		https://www.adafruit.com/products/716
 *
 *	Hardware: 60 WS2812B LEDS hooked up to Arduino Mega digital pin 13 (Port B pin 7)
 *		I'm using an Adafruit 60 LED 'Neopixel' Ring
 *		https://www.adafruit.com/products/1768
 *
 *	Files: i2cmaster.S and i2cmaster.h
 *	Files: uart.c and uart.h
 *		Use Peter Fleury's wonderful AVR UART and I2C Libraries
 *		http://homepage.hispeed.ch/peterfleury/avr-software.html
 *
 *	Files: mcp23017.c and mcp23017.h
 *		Originally written by Alan K. Duncan
 *		I got these from part of BlackBear's AVR-library
 *		https://github.com/BlackBears/AVR-library
 *
 *	Files: mcp23017_lcd.c and mcp23017_lcd.h
 *		I wrote these, loosely based off Peter Fleury's HD44780 LCD library
 *		I used an LCD initialization algorithm found on Alfred State's website
 *		http://web.alfredstate.edu/weimandn/lcd/lcd_initialization/lcd_initialization_index.html
 *
 *	Files: light_ws2812.c, light_ws2812.h, and ws2812_config.h
 *		Use cpldcpu's awesome WS2812 drivers
 *		https://github.com/cpldcpu/light_ws2812
 *		Use the light_ws2812_AVR files.  Edit the ws2812_config.h file to set your output pin
 *	
 *	Files: bno055.c and bno055.h
 *		Use Bosch's bno055 c driver
 *		https://github.com/BoschSensortec/BNO055_driver
 *
 *		Also make your own file based off Bosch's bno055_support.c
 *		I added bno055_support.c to my project for reference, but set
 *			its 'build action' to 'none' so it isn't included in compilation
 *		To get this working, you need to take support.c code and fill in:
 *			1.) An i2c write function/routine
 *			2.) An i2c read function/routine
 *			3.) A delay function
 *		Once you successfully add these, the whole bno055 driver will work
 *		I put my work in the 'myBNO055' .c and .h files
 *	Files: myBNO055.c and myBNO055.h
 *		(or add blank .c and .h files to your project and write your own with your i2c library)
 * 
 *
 *
 *	To get the project working in Atmel Studio 7:
 *	1.) Create a new project, of type GCC C executable project, with device ATmega2560
 *
 *	2.) Navigate to the project folder and add (copy/paste from downloads) all the .c, .S, and .h files
 *			note: My directory happens to be Documents->Atmel Studio->7.0->[project name]->[project name]
 *			note: Your main.c should be in this folder
 *
 *	3.) Right click on your project in the solution explorer->Add->Existing Item
 *			add all of the relevant files you just pasted in
 *
 *	4.) IMPORTANT!!!!	Right click on your project in the solution explorer -> properties -> 
 *			Toolchain -> AVR/GNU C Compiler -> Symbols -> Defined Symbols (-D) -> 
 *			Add Symbol and call it "F_CPU=16000000UL"
 *
 *	5.) Edit the port and pin definitions in i2cMaster.S to:
 *
 *			#define SDA             1           // SDA Port D pin 1, Arduino Mega Pin 20
 *			#define SCL             0           // SCL Port D pin 0, Arduino Mega Pin 21
 *			#define SDA_PORT        PORTD       // SDA Port D
 *			#define SCL_PORT        PORTD       // SCL Port D
 * 
 *	6.) Edit the port and pin definitions in ws2812_config.h to:
 *
 *		//Port B7 is Digital pin 13 on the Arduino Mega, and is a PWM pin
 *		#define ws2812_port B     // Data port
 *		#define ws2812_pin  7     // Data out pin
 *
 *	7.) I chose to leave out the global.h file in mcp23017.h.  Instead, we need to clean up
 *		mcp23017.c and mcp23017.h.  Go into those files, use ctrl=f to find and replace all
 *		of the 'u08' to 'uint8_t' and 'u16' to 'uint16_t'
 *
 *		Also, in mcp23017.c, comment out the #include "global.h" line.
 *
 *	8.) Connect the Arduino Mega to your computer, note the COM port, and create you tool
 *
 *		To upload to Arduino Mega board:
 *		Install winAVR - https://sourceforge.net/projects/winavr/
 *		Make sure you're in advanced mode so 'External Tools' shows up under 'Tools'
 *		Go to 'Tools'->'External Tools' and create a new tool
 *			Title: ATMega2560 (can be whatever you want)
 *			Command: C:\WinAVR-20100110\bin\avrdude.exe (whatever your install directory is)
 *			Arguments:  -P\\.\COM4 C:\WinAVR-20100110\bin\avrdude.conf -v -v -p atmega2560
 *				-c avrispmkII -U flash:w:"$(ProjectDir)Debug\$(TargetName).hex":i
 *				(You must have your COM port and path to .conf file)
 *				(check documentation to find out what all the -v and -p things mean)
 *		IMPORTANT!!!!
 *		IMPORTANT!!!! - With the Arduino Mega, you must press the Arduino's reset button right
 *		IMPORTANT!!!!	as you click to run your tool.
 *		IMPORTANT!!!!
  *
 *	9.) Code-  Get UART working then I2C and finally read your sensor
 *		
 *	Notes:
 *	-The I2C addresses might have to be left shifted 1 for Peter Fleury's I2C library
 *		I think this is because he masks a read/write bit on??
 *		If your I2C code hangs when you try to read/write, try fixing the I2c address
 *	-Press the Arduino Mega's resest button as you upload your code
 *	-Add F_CPU=16000000UL to your project properties
 *	-Use UART_BAUD_SELECT_DOUBLE_SPEED to initialize UART baud rates above 57600
 *	-Remember to change your i2c pin and port definitions
 *	-Remember to change your ws2812 pin and port definitions
 *  -Remember to clean up if you choose to exclude global.h of it will not build
 *	-current draw: 17-18mA @4.86V
 *	-Counter instructions // http://maxembedded.com/2011/07/avr-timers-ctc-mode/
 */ 

//TODO:: save and load BNO055 calibration values in EEPROM
//TODO:: implement a sleep mode?
//TODO:: Properly implement BNO055 I2C functions to return 0 or 1
//TODO:: Get the RGB LCD Backlight control working
//TODO:: Create a list of destinations and a way to switch to them (enum of structs?)
//TODO:: get the input buttons to use interrupts and flags so we don't miss a press

//Go into uart.h and incrase the Tx and Rx buffers from 32 bytes to 128 bytes
//It needs to be large enough to hold a whole NMEA sentence/command
//We have 8k of SRAM with the ATmega2560, so we can afford this easily
//#define UART_RX_BUFFER_SIZE 128
// #define UART_TX_BUFFER_SIZE 128

#include <util/delay.h>		// for _delay_ms()
#include <stdlib.h>			// for itoa(), dtostrf(), and NULL
#include <avr/io.h>			// for pin descriptions? types?
#include "i2cmaster.h"		// for i2c
#include "myGPS.h"			//	for the GPS
#include "uart.h"			// for UART
#include <avr/interrupt.h>  // for UART
#include "bno055.h"			// Bosch BNO055 driver
#include "myBNO055.h"		// my BNO055 implementation functions
#include "light_ws2812.h"	// for LED ring
#include "mcp23017.h"		// for the mcp23017 I/O expander
#include "mcp23017_lcd.h"	// for the LCD display

/*  UART defines */
#define UART_BAUD_RATE			115200	//uart = Terminal window
#define UART1_BAUD_RATE			9600	//UART1 = GPS
#define	DTOSTRF_BUFFER_MAX_SIZE	12		//12 character long buffer

/*  ws2812 defines */
#define MAXPIX 60
#define COLORLENGTH MAXPIX/2
#define FADE 256/COLORLENGTH

/* main.c function prototypes */
void uart_putu8_hex(uint8_t number);
s8 I2C_routine(void);

/*  Initialize ws2812 structs */
struct cRGB colors[8];
struct cRGB led[MAXPIX];
 
/*  Initialize bno055 struct */
struct bno055_t bno055;

/* Initialize lcd struct */
MCP23017 lcd;

/* Create a volatile global variable for the timer */
volatile uint16_t millis0 = 0; //make volatile so it can be modified inside an ISR
volatile uint16_t millis1 = 0; //make volatile so it can be modified inside an ISR

/*---------------------------------------------------------------------------*
 *********************** START OF MAIN ************************
 *--------------------------------------------------------------------------*/

int main(void)
{
	/*  Set up Timer 1 to give 1 millisecond interrupts */
	TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10); //prescaler 64
	TCNT1 = 0; // initialize the counter
	// OCR1A Timer Count = (Required Delay (in seconds) * Timer Frequency) - 1
	//	where Timer Frequency = 16MHz / prescaler
	OCR1A = 249; // Set the counter compare value
	TIMSK1 |= (1 << OCIE1A); // enable compare interrupt
	
	/*  Initialize UART0 */
	uart_init( UART_BAUD_SELECT_DOUBLE_SPEED(UART_BAUD_RATE,F_CPU) );	// Init uart0 at 115200
	uart1_init(UART_BAUD_SELECT(UART1_BAUD_RATE,F_CPU));				// Init UART1 (GPS) at 9600
	sei(); //enable interrupts for the UART library
	uart_puts("BNO055 Test Program\r\n");
	char dtostrfBuffer[DTOSTRF_BUFFER_MAX_SIZE];
	//unsigned int c;  // character read in on UART0
	unsigned int c1; // character read in on UART1

	/*  Initialize BNO055 */
	s32 comres = ERROR; //communication result
  	I2C_routine(); // maps functions to the global bno055_t struct
	comres = bno055_init(&bno055);
	u8 power_mode = POWER_MODE_NORMAL;
	comres += bno055_set_power_mode(power_mode);
	comres += bno055_set_operation_mode(OPERATION_MODE_NDOF);
	
	/*  Initialize ws2812 */
	DDRB|=_BV(ws2812_pin);
	uint8_t pixel = 0;
	uint8_t old_pixel = 0;
	for(pixel=MAXPIX; pixel>0; pixel--) //set all LEDS to off
	{
		led[pixel-1].r=0;
		led[pixel-1].g=0;
		led[pixel-1].b=0;
	}
	ws2812_setleds(led,60); //send data to LEDs
	
	/* establish I2C communication with the mcp23017 I/0 expander */
	mcp23017_init(&lcd, 0b000);
		
	/* Initialize the HD44780 LCD into 4 wire mode through the MCP23017 I/O expander */
	lcd_init(&lcd);
	
	/*  Initialize GPS */
	uart1_puts(PMTK_SET_BAUD_9600); //Set the GPS's baud rate to match our UART1 baud rate
	_delay_ms(50);
	uart1_puts(PMTK_SET_NMEA_OUTPUT_RMCONLY); // request just RMC - Recommended Minimum sentence
	_delay_ms(50);
	uart1_puts(PMTK_SET_NMEA_UPDATE_1HZ); // send NMEA sentence every second
	_delay_ms(50);
	uart1_puts(PMTK_API_SET_FIX_CTL_1HZ); // position fix every second
	_delay_ms(50);
	struct gps_t gps = {0};
	char nmea_buffer[85] = {0};
	uint8_t index = 0;

	/*  structs for BNO055 */
	struct bno055_euler_double_t d_euler_hpr = {0};		// holds converted euler data (double degrees)
	struct bno055_calibration_unsigned_char_t cal = {0};	// hold calibration values (0-3)
	
	
	uint8_t color = 0;
	int8_t direction = 1;
	char character = 0;
	uint8_t destination = 0;
	
	/* initialize destination structs of where we want to go */
	//destination 0 is pointing North
	struct dest_t dest1 = {	44.074827,	103.205799,	"SDSMT Quad"};
	struct dest_t dest2 = {	44.072725,	103.202519,	"Football"};
	struct dest_t dest3 = {	44.070404,	103.202092,	"Windmill"};
	struct dest_t dest4 = {	44.071419,	103.208006,	"Practice"};
	struct dest_t dest5 = {	44.077980,	103.244722,	"Dinosaur"};
	struct dest_t dest6 = {	44.059678,	103.290932,	"Canyon Lake"};
	struct dest_t dest7 = {	44.056691,	103.293772,	"RC House"};
	struct dest_t dest8 = {	43.592239,	96.575253,	"BV High School"};
	struct dest_t dest9 = {	43.577479,	96.605332,	"bridge"};
	struct dest_t dest10 = {43.580836,	96.595614,	"home"};
	struct dest_t dest11 = {43.512491,	96.593946,	"Kaylynn"};
	struct dest_t dest12 = {43.550907,	96.725334,	"SF Raven"};
		
	uint8_t numWaypoints = 12; //make this match the above
	
	/* Initialize the struct to hold the distance (mi)/bearing (deg) to our destination */	
	struct directions_t directions = {0,0}; //{distance,bearing}
	
	/* Set gps status flag so we update the LCD after boot */	
	gps.status_flags |= GPS_NEW_DEST;
	uint8_t buttonStatus;
	
	uart_puts("Initialization Complete\r\n");
	
/*---------------------------------------------------------------------------*
 *********************** START OF LOOP ************************
 *--------------------------------------------------------------------------*/

    while (1) 
    {
/*--------------------------------------------------------------------------*/
		//uart_putc('1');
		/* read in a character from UART1 buffer (GPS)*/
		c1 = uart1_getc();
		if ( c1 & UART_NO_DATA ){}
		else
		{
			if ( c1 & UART_FRAME_ERROR ) {uart_puts_P("FRAME ERROR");}
			if ( c1 & UART_OVERRUN_ERROR ) {uart_puts_P("OVERRUN");}
			if ( c1 & UART_BUFFER_OVERFLOW ) {uart_puts_P("OVERFLOW");}
					
			/* If we see a $, parse what's in the nmea_buffer, then put that $ into a new nmea_buffer*/
			if((unsigned char)c1 == '$') //note c1 is a 16 bit value masked with error flags
			{
				nmea_buffer[index] = '\0';			//null terminate the nmea sentence
				//uart_puts(nmea_buffer);				//print the NMEA sentence
				parse_nmea_rmc(nmea_buffer, &gps);	//parse nmea_buffer and put data into the gps_t struct
				//print_gps_data(&gps);				//print the parsed data to the terminal
				index = 0;							//reset the buffer index
			}
		
			/* Add the character to the NMEA buffer and increment the index*/
			nmea_buffer[index++] = (unsigned char)c1;
		}
		
		
/*--------------------------------------------------------------------------*/
		//uart_putc('2');
		/* read the buttons and set flags accordingly */
		buttonStatus = readButtons();
		if(buttonStatus & BUTTON_LEFT)
		{
			gps.status_flags |= GPS_NEW_DEST;
			destination++;
			while(readButtons() & BUTTON_LEFT){}
		}
		
		if(destination > numWaypoints)
			destination = 0;
			
		
/*--------------------------------------------------------------------------*/
		//uart_putc('3');
		/* Update LCD at 10Hz */
		if(millis0>100)
		{
			millis0 = 0; //reset the interrupt-driven millisecond counter
			
			/* Change color of the directional LED for a heartbeat */
			color += 2*direction;
			if(color == 20 || color == 0)
				direction *= -1;			
			
			/*read the calibration register and print to the screen */
			getCalibration(&cal.sys, &cal.gyro, &cal.accel, &cal.mag);

			lcd_gotoxy(11,1); //(column 0-19,row 0-1)
			//lcd_puts("C:");
			lcd_putc(*itoa(cal.sys,&character,10));
			lcd_putc(*itoa(cal.gyro,&character,10));
			lcd_putc(*itoa(cal.accel,&character,10));
			lcd_putc(*itoa(cal.mag,&character,10));
			lcd_putc(gps.GPSvalidity);
			
			//uart_putc('4');

			/* If we have new GPS data (1Hz), calculate and update distance/gps bearing */
			/* This flag is set in the parse_nmea_rmc() function if we have valid data */
			if(gps.status_flags & GPS_UNREAD)
			{
				gps.status_flags &= ~GPS_UNREAD;  //reset the 'new gps data available' flag
				
				/* calculate the new distance and bearing into the directions_t struct*/
				switch(destination)
				{
					case 1:
						haversine(&gps,&dest1,&directions);
						break;
					case 2:
						haversine(&gps,&dest2,&directions);
						break;
					case 3:
						haversine(&gps,&dest3,&directions);
						break;
					case 4:
						haversine(&gps,&dest4,&directions);
						break;
					case 5:
						haversine(&gps,&dest5,&directions);
						break;
					case 6:
						haversine(&gps,&dest6,&directions);
						break;
					case 7:
						haversine(&gps,&dest7,&directions);
						break;
					case 8:
						haversine(&gps,&dest8,&directions);
						break;
					case 9:
						haversine(&gps,&dest9,&directions);
						break;
					case 10:
						haversine(&gps,&dest10,&directions);
						break;
					case 11:
						haversine(&gps,&dest11,&directions);
						break;
					case 12:
						haversine(&gps,&dest12,&directions);
						break;

					default:
						/* default to pointing North */
						directions.distance = 0;
						directions.bearing = 0;
				}

				/* Print the new distance and bearing */
				lcd_gotoxy(0,1);
				lcd_puts("D:");

				/* if we're less than 400m away, print meters; if we're over 400m away, print miles */
				if(directions.distance < 400)
				{
					lcd_puts(dtostrf(directions.distance,1,2,dtostrfBuffer)); //2 decimal places
					lcd_puts("m");
				}
				else
				{
					lcd_puts(dtostrf(directions.distance*METERS2MILES,1,2,dtostrfBuffer));
					lcd_puts("mi");
				}
		
				lcd_puts("    ");//print spaces in case we need to clear a characters
				
			}

			//uart_putc('5');
			/* If we have a new destination (if a button was pressed), print it on the screen */
			if(gps.status_flags & GPS_NEW_DEST || destination == 0)
			{
				gps.status_flags &= ~GPS_NEW_DEST;
				lcd_gotoxy(0,0);
				lcd_puts("L:");
								
				/* make sure to pad with spaces */
				switch(destination)
				{
					case 1:
						lcd_puts(dest1.name);
						break;
					case 2:
						lcd_puts(dest2.name);
						break;
					case 3:
						lcd_puts(dest3.name);
						break;
					case 4:
						lcd_puts(dest4.name);
						break;
					case 5:
						lcd_puts(dest5.name);
						break;
					case 6:
						lcd_puts(dest6.name);
						break;
					case 7:
						lcd_puts(dest7.name);
						break;
					case 8:
						lcd_puts(dest8.name);
						break;
					case 9:
						lcd_puts(dest9.name);
						break;
					case 10:
						lcd_puts(dest10.name);
						break;
					case 11:
						lcd_puts(dest11.name);
						break;
					case 12:
						lcd_puts(dest12.name);
						break;
					default:
						//lcd_puts("North  ");
						lcd_puts(dtostrf(d_euler_hpr.h,1,1,dtostrfBuffer));
				}
				lcd_puts("        "); //print extra space to clear the old word			
			}
			
		}/* End Update LCD at 10Hz */



		//uart_putc('6');
/*--------------------------------------------------------------------------*/
		/* Update LED at 200Hz (period = 5ms) */
		if( millis1 >= 5 )
		{
			/*
			uart_puts("ms: ");
			uart_puts(itoa(millis1,dtostrfBuffer,10));
			uart_puts("\r\n");
			*/
			
			millis1 = 0; //reset the interrupt-driven millisecond counter
			
			/* get filtered, tilt-compensated heading from bno055 in 0-360 degrees from North */
			comres += bno055_convert_double_euler_h_deg(&d_euler_hpr.h); 
			/*
			uart_puts("d_euler_hpr.h: ");
			uart_puts(dtostrf(d_euler_hpr.h,1,1,dtostrfBuffer));
			uart_puts("\r\n");			
			*/
			/* Add 3.22 degrees for magnetic declination in Sioux Falls, SD */
			d_euler_hpr.h += MAG_DECLINATION;
			
			/* 
			* Due to the physical orientation of the sensor, 0 Euler degrees = West
			* Add a 1/4 (90 degree) offset to make 0 degrees equal North
			*/
			d_euler_hpr.h += 90;

			/* point towards the destination (directions.bearing will be 0 if pointing North) */
			float bearing_to_draw = d_euler_hpr.h - directions.bearing;

			/* make sure the bearing is between 0 and 360 degrees */
			while(bearing_to_draw < 0)
			{bearing_to_draw += 360;}
			while(bearing_to_draw > 360)
			{bearing_to_draw -= 360;}
				
			/* 
			*  now we take the Euler heading in degrees and light the corresponding light on the ring
			*	[0->360]deg/6 = [0->60]pixels.  
			*	Euler Degrees increase clockwise and pixels increase counterclockwise,
			*	so we take the complement: [60->0]pixels = 60-[0->60]pixels 
			*/
			pixel = MAXPIX-(bearing_to_draw/6);
			
			/* Add an offset of 30 since the front of the device is at pixel 30 of 60 */
			pixel += 30;
			
			/* If our offset took us over pixel 60, subtract 60 */
			if(pixel >= 60)
				pixel -= 60;			
				
			/* 
			* If the pixel has moved since the last iteration,
			*	erase the old p                                                                                                                                                                                                                                              ixel and draw the new pixel
			* Else redraw the pixel with a new color (for a heartbeat)
			*/
			if(old_pixel != pixel)
			{
				led[old_pixel].r=0;led[old_pixel].g=0;led[old_pixel].b=0; //erase
				led[pixel].r=color;led[pixel].g=color;led[pixel].b=20-color; //draw
				old_pixel = pixel;
			}
			else
				led[pixel].r=color;led[pixel].g=0;led[pixel].b=20-color; //draw
		
			ws2812_setleds(led,60); //send data to LEDs.
		}/* End Update LED at 200Hz */
		//uart_putc('7');
		
    }/* while(1) */
}/* main() */

/*---------------------------------------------------------------------------*
 *********************** END OF MAIN ************************
 *--------------------------------------------------------------------------*/

/*
 *	Function: I2C_routine
 *	@brief This maps functions to the global bno055_t struct
 *
 *	@param none
 *
 *	@return zero
 */

 s8 I2C_routine(void) {

	bno055.bus_write = BNO055_I2C_bus_write;
	bno055.bus_read = BNO055_I2C_bus_read;
	bno055.delay_msec = BNO055_delay_msek;
	bno055.dev_addr = MY_BNO055_I2C_ADDDRESS;

	return BNO055_ZERO_U8X;
}


/*
 *	ISR: TIMER1_COMPA_vect
 *	@brief This happens whenever OCR1A overflows
 *		This is every millisecond
 *
 */
ISR (TIMER1_COMPA_vect)
{
	millis0++;
	millis1++;
}