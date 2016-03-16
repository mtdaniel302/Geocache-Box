/*
 * ECompass_V1
 *
 * Created: 3/8/2016 12:35:43 PM
 * Author : Riley
 *
 *	Note: This was written for my own project, apologies for bad code/poor flexibility
 *	If I had to do it again, I might have rather used ARM (Arduino Due) instead of AVR
 *
 *	I tried not to edit the libraries except for pin assignments
 *	I wrote main.c, mcp23017_lcd.c and .h, myBNO055.c and .h, myGPS.c and .h
 *	The rest should be mostly unchanged
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
 *	8.) Go into uart.h and incrase the Tx and Rx buffers from 32 bytes to 128 bytes
 *			It needs to be large enough to hold a whole '80' character NMEA sentence/command
 *			We have 8k of SRAM with the ATmega2560, so we can afford this easily
 *				#define UART_RX_BUFFER_SIZE 128
 *				#define UART_TX_BUFFER_SIZE 128
 *
 *	9.) Connect the Arduino Mega to your computer, note the COM port, and create you tool
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
 *	10.) Enjoy!! Once you get everything working, it is fun!
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
 *	-current draw: 180-200mA @4.86V (a lot is LCD backlight?)
 *	-Counter instructions // http://maxembedded.com/2011/07/avr-timers-ctc-mode/
 */ 

//TODO:: get brightness sensor and LED/LCD auto brightness working
//TODO:: save and load BNO055 calibration values in EEPROM
//TODO:: implement a sleep mode?
//TODO:: Properly implement BNO055 I2C functions to return 0 or 1
//TODO:: Get the RGB LCD Backlight control working - hook up to pwm pins
//TODO:: get the input buttons to use interrupts and flags so we don't miss a press