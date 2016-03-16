/****************************************************************************
 Title:     HD44780U LCD library
 Author:    Peter Fleury <pfleury@gmx.ch>  http://tinyurl.com/peterfleury
 File:	    $Id: lcd.c,v 1.15.2.2 2015/01/17 12:16:05 peter Exp $
 Software:  AVR-GCC 3.3 
 Target:    any AVR device, memory mapped mode only for AT90S4414/8515/Mega

 DESCRIPTION
       Basic routines for interfacing a HD44780U-based text lcd display

       Originally based on Volker Oth's lcd library,
       changed lcd_init(), added additional constants for lcd_command(),
       added 4-bit I/O mode, improved and optimized code.

       Library can be operated in memory mapped mode (LCD_IO_MODE=0) or in 
       4-bit IO port mode (LCD_IO_MODE=1). 8-bit IO port mode not supported.
       
       Memory mapped mode compatible with Kanda STK200, but supports also
       generation of R/W signal through A8 address line.

 USAGE
       See the C include lcd.h file for a description of each function
       
*****************************************************************************/
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "mcp23017_lcd.h"
#include "uart.h"

#define GPIOA_OUTPUTS	0x3F	//bits 0-5 inputs, bits 6-7 output
#define GPIOB_OUTPUTS	0x00	//all 8 bits output

#define ENABLE_MASK		0X20	//5th bit is the enable bit
#define FUNCTION_SET	0x18	//defined by alfred state's page



/* 
** constants/macros 
*/
#define DDR(x) (*(&x - 1))      /* address of data direction register of port x */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__)
    /* on ATmega64/128 PINF is on port 0x00 and not 0x60 */
    #define PIN(x) ( &PORTF==&(x) ? _SFR_IO8(0x00) : (*(&x - 2)) )
#else
	#define PIN(x) (*(&x - 2))    /* address of input register of port x          */
#endif


#if LCD_IO_MODE
#define lcd_e_delay()   _delay_us(LCD_DELAY_ENABLE_PULSE)
#define lcd_e_high()    LCD_E_PORT  |=  _BV(LCD_E_PIN);
#define lcd_e_low()     LCD_E_PORT  &= ~_BV(LCD_E_PIN);
#define lcd_e_toggle()  toggle_e()
#define lcd_rw_high()   LCD_RW_PORT |=  _BV(LCD_RW_PIN)
#define lcd_rw_low()    LCD_RW_PORT &= ~_BV(LCD_RW_PIN)
#define lcd_rs_high()   LCD_RS_PORT |=  _BV(LCD_RS_PIN)
#define lcd_rs_low()    LCD_RS_PORT &= ~_BV(LCD_RS_PIN)
#endif

#if LCD_IO_MODE
#if LCD_LINES==1
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_1LINE 
#else
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES 
#endif
#else
#if LCD_LINES==1
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_8BIT_1LINE
#else
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_8BIT_2LINES
#endif
#endif

#if LCD_CONTROLLER_KS0073
#if LCD_LINES==4

#define KS0073_EXTENDED_FUNCTION_REGISTER_ON  0x2C   /* |0|010|1100 4-bit mode, extension-bit RE = 1 */
#define KS0073_EXTENDED_FUNCTION_REGISTER_OFF 0x28   /* |0|010|1000 4-bit mode, extension-bit RE = 0 */
#define KS0073_4LINES_MODE                    0x09   /* |0|000|1001 4 lines mode */

#endif
#endif

/* 
** function prototypes 
*/
#if LCD_IO_MODE
static void toggle_e(void);
#endif

/*
** local functions
*/


/************************************************************************* 
delay for a minimum of <us> microseconds
the number of loops is calculated at compile-time from MCU clock frequency
*************************************************************************/
#define delay(us)  _delay_us(us) 


#if LCD_IO_MODE
/* toggle Enable Pin to initiate write */
static void toggle_e(void)
{
    lcd_e_high();
    lcd_e_delay();
    lcd_e_low();
}
#endif


/*************************************************************************
Low-level function to write byte to LCD controller
Input:    data   byte to write to LCD
          rs     1: write data    
                 0: write instruction
Returns:  none
*************************************************************************/
#if LCD_IO_MODE
static void lcd_write(uint8_t data,uint8_t rs) 
{
    unsigned char dataBits ;


    if (rs) {        /* write data        (RS=1, RW=0) */
       lcd_rs_high();
    } else {         /* write instruction (RS=0, RW=0) */
       lcd_rs_low();
    }
    lcd_rw_low();    /* RW=0  write mode      */

    if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT )
      && (LCD_DATA0_PIN == 0) && (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3) )
    {
        /* configure data pins as output */
        DDR(LCD_DATA0_PORT) |= 0x0F;

        /* output high nibble first */
        dataBits = LCD_DATA0_PORT & 0xF0;
        LCD_DATA0_PORT = dataBits |((data>>4)&0x0F);
        lcd_e_toggle();

        /* output low nibble */
        LCD_DATA0_PORT = dataBits | (data&0x0F);
        lcd_e_toggle();

        /* all data pins high (inactive) */
        LCD_DATA0_PORT = dataBits | 0x0F;
    }
    else
    {
        /* configure data pins as output */
        DDR(LCD_DATA0_PORT) |= _BV(LCD_DATA0_PIN);
        DDR(LCD_DATA1_PORT) |= _BV(LCD_DATA1_PIN);
        DDR(LCD_DATA2_PORT) |= _BV(LCD_DATA2_PIN);
        DDR(LCD_DATA3_PORT) |= _BV(LCD_DATA3_PIN);
        
        /* output high nibble first */
        LCD_DATA3_PORT &= ~_BV(LCD_DATA3_PIN);
        LCD_DATA2_PORT &= ~_BV(LCD_DATA2_PIN);
        LCD_DATA1_PORT &= ~_BV(LCD_DATA1_PIN);
        LCD_DATA0_PORT &= ~_BV(LCD_DATA0_PIN);
    	if(data & 0x80) LCD_DATA3_PORT |= _BV(LCD_DATA3_PIN);
    	if(data & 0x40) LCD_DATA2_PORT |= _BV(LCD_DATA2_PIN);
    	if(data & 0x20) LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
    	if(data & 0x10) LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);   
        lcd_e_toggle();
        
        /* output low nibble */
        LCD_DATA3_PORT &= ~_BV(LCD_DATA3_PIN);
        LCD_DATA2_PORT &= ~_BV(LCD_DATA2_PIN);
        LCD_DATA1_PORT &= ~_BV(LCD_DATA1_PIN);
        LCD_DATA0_PORT &= ~_BV(LCD_DATA0_PIN);
    	if(data & 0x08) LCD_DATA3_PORT |= _BV(LCD_DATA3_PIN);
    	if(data & 0x04) LCD_DATA2_PORT |= _BV(LCD_DATA2_PIN);
    	if(data & 0x02) LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
    	if(data & 0x01) LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);
        lcd_e_toggle();        
        
        /* all data pins high (inactive) */
        LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);
        LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
        LCD_DATA2_PORT |= _BV(LCD_DATA2_PIN);
        LCD_DATA3_PORT |= _BV(LCD_DATA3_PIN);
    }
}
#else
#define lcd_write(d,rs) if (rs) *(volatile uint8_t*)(LCD_IO_DATA) = d; else *(volatile uint8_t*)(LCD_IO_FUNCTION) = d;
/* rs==0 -> write instruction to LCD_IO_FUNCTION */
/* rs==1 -> write data to LCD_IO_DATA */
#endif


/*************************************************************************
Low-level function to read byte from LCD controller
Input:    rs     1: read data    
                 0: read busy flag / address counter
Returns:  byte read from LCD controller
*************************************************************************/
#if LCD_IO_MODE
static uint8_t lcd_read(uint8_t rs) 
{
    uint8_t data;

    if (rs)
        lcd_rs_high();                       /* RS=1: read data      */
    else
        lcd_rs_low();                        /* RS=0: read busy flag */
    lcd_rw_high();                           /* RW=1  read mode      */
    
    if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT )
      && ( LCD_DATA0_PIN == 0 )&& (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3) )
    {
        DDR(LCD_DATA0_PORT) &= 0xF0;         /* configure data pins as input */
        
        lcd_e_high();
        lcd_e_delay();        
        data = PIN(LCD_DATA0_PORT) << 4;     /* read high nibble first */
        lcd_e_low();
        
        lcd_e_delay();                       /* Enable 500ns low       */
        
        lcd_e_high();
        lcd_e_delay();
        data |= PIN(LCD_DATA0_PORT)&0x0F;    /* read low nibble        */
        lcd_e_low();
    }
    else
    {
        /* configure data pins as input */
        DDR(LCD_DATA0_PORT) &= ~_BV(LCD_DATA0_PIN);
        DDR(LCD_DATA1_PORT) &= ~_BV(LCD_DATA1_PIN);
        DDR(LCD_DATA2_PORT) &= ~_BV(LCD_DATA2_PIN);
        DDR(LCD_DATA3_PORT) &= ~_BV(LCD_DATA3_PIN);
                
        /* read high nibble first */
        lcd_e_high();
        lcd_e_delay();        
        data = 0;
        if ( PIN(LCD_DATA0_PORT) & _BV(LCD_DATA0_PIN) ) data |= 0x10;
        if ( PIN(LCD_DATA1_PORT) & _BV(LCD_DATA1_PIN) ) data |= 0x20;
        if ( PIN(LCD_DATA2_PORT) & _BV(LCD_DATA2_PIN) ) data |= 0x40;
        if ( PIN(LCD_DATA3_PORT) & _BV(LCD_DATA3_PIN) ) data |= 0x80;
        lcd_e_low();

        lcd_e_delay();                       /* Enable 500ns low       */
    
        /* read low nibble */    
        lcd_e_high();
        lcd_e_delay();
        if ( PIN(LCD_DATA0_PORT) & _BV(LCD_DATA0_PIN) ) data |= 0x01;
        if ( PIN(LCD_DATA1_PORT) & _BV(LCD_DATA1_PIN) ) data |= 0x02;
        if ( PIN(LCD_DATA2_PORT) & _BV(LCD_DATA2_PIN) ) data |= 0x04;
        if ( PIN(LCD_DATA3_PORT) & _BV(LCD_DATA3_PIN) ) data |= 0x08;        
        lcd_e_low();
    }
    return data;
}
#else
#define lcd_read(rs) (rs) ? *(volatile uint8_t*)(LCD_IO_DATA+LCD_IO_READ) : *(volatile uint8_t*)(LCD_IO_FUNCTION+LCD_IO_READ)
/* rs==0 -> read instruction from LCD_IO_FUNCTION */
/* rs==1 -> read data from LCD_IO_DATA */
#endif


/*************************************************************************
loops while lcd is busy, returns address counter
*************************************************************************/
static uint8_t lcd_waitbusy(void)

{
    register uint8_t c;
    
    /* wait until busy flag is cleared */
    while ( (c=lcd_read(0)) & (1<<LCD_BUSY)) {}
    
    /* the address counter is updated 4us after the busy flag is cleared */
    delay(LCD_DELAY_BUSY_FLAG);

    /* now read the address counter */
    return (lcd_read(0));  // return address counter
    
}/* lcd_waitbusy */


/*************************************************************************
Move cursor to the start of next line or to the first line if the cursor 
is already on the last line.
*************************************************************************/
static inline void lcd_newline(uint8_t pos)
{
    register uint8_t addressCounter;


#if LCD_LINES==1
    addressCounter = 0;
#endif
#if LCD_LINES==2
    if ( pos < (LCD_START_LINE2) )
        addressCounter = LCD_START_LINE2;
    else
        addressCounter = LCD_START_LINE1;
#endif
#if LCD_LINES==4
#if KS0073_4LINES_MODE
    if ( pos < LCD_START_LINE2 )
        addressCounter = LCD_START_LINE2;
    else if ( (pos >= LCD_START_LINE2) && (pos < LCD_START_LINE3) )
        addressCounter = LCD_START_LINE3;
    else if ( (pos >= LCD_START_LINE3) && (pos < LCD_START_LINE4) )
        addressCounter = LCD_START_LINE4;
    else 
        addressCounter = LCD_START_LINE1;
#else
    if ( pos < LCD_START_LINE3 )
        addressCounter = LCD_START_LINE2;
    else if ( (pos >= LCD_START_LINE2) && (pos < LCD_START_LINE4) )
        addressCounter = LCD_START_LINE3;
    else if ( (pos >= LCD_START_LINE3) && (pos < LCD_START_LINE2) )
        addressCounter = LCD_START_LINE4;
    else 
        addressCounter = LCD_START_LINE1;
#endif
#endif
    lcd_command((1<<LCD_DDRAM)+addressCounter);

}/* lcd_newline */


/*
** PUBLIC FUNCTIONS 
*/

/*************************************************************************
Send LCD controller instruction command
Input:   instruction to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_command(uint8_t cmd)
{
    lcd_waitbusy();
    lcd_write(cmd,0);
}


/*************************************************************************
Send data byte to LCD controller 
Input:   data to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_data(uint8_t data)
{
    lcd_waitbusy();
    lcd_write(data,1);
}



/*************************************************************************
Set cursor to specified position
Input:    x  horizontal position  (0: left most position)
          y  vertical position    (0: first line)
Returns:  none
*************************************************************************/
void lcd_gotoxy(uint8_t x, uint8_t y)
{
    if ( y==0 ) 
        lcd_sendcommand((1<<LCD_DDRAM)+LCD_START_LINE1+x);
    else
        lcd_sendcommand((1<<LCD_DDRAM)+LCD_START_LINE2+x);

}/* lcd_gotoxy */


/*************************************************************************
*************************************************************************/
int lcd_getxy(void)
{
    return lcd_waitbusy();
}


/*************************************************************************
Clear display and set cursor to home position
*************************************************************************/
void lcd_clrscr(void)
{
    lcd_sendcommand(1<<LCD_CLR); //CLEAR = 0x01
}


/*************************************************************************
Set cursor to home position
*************************************************************************/
void lcd_home(void)
{
	lcd_sendcommand(1<<LCD_HOME);
}

/*************************************************************************
Display string without auto linefeed 
Input:    string to be displayed
Returns:  none
*************************************************************************/
void lcd_puts(const char *s)
/* print string on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = *s++) ) {
        lcd_putc(c);
    }

}/* lcd_puts */


/*************************************************************************
Display string from program memory without auto linefeed 
Input:     string from program memory be be displayed                                        
Returns:   none
*************************************************************************/
void lcd_puts_p(const char *progmem_s)
/* print string from program memory on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = pgm_read_byte(progmem_s++)) ) {
        lcd_putc(c);
    }

}/* lcd_puts_p */


/*************************************************************************
Initialize display and select type of cursor 
Input:    dispAttr LCD_DISP_OFF            display off
                   LCD_DISP_ON             display on, cursor off
                   LCD_DISP_ON_CURSOR      display on, cursor on
                   LCD_DISP_CURSOR_BLINK   display on, cursor on flashing
Returns:  none
*************************************************************************/

MCP23017 lcdDevice;

void lcd_init(MCP23017 *obj)
{	
	lcdDevice = *obj;
	
	/* configure data direction registers of I/O expander */
	mcp23017_write_register(obj, MCP23017_IODIRA, GPIOA_OUTPUTS); // bits 0-5 input (1), bits 6-7 output (0)
	mcp23017_write_register(obj, MCP23017_GPPUA, GPIOA_OUTPUTS); // activate the pullup resistors on pins 0-5
	mcp23017_write_register(obj, MCP23017_IODIRB, GPIOB_OUTPUTS); //bits 0-7 output (0)

	/* I followed the instructions here */
	// http://web.alfredstate.edu/weimandn/lcd/lcd_initialization/lcd_initialization_index.html
	
	delay(LCD_DELAY_BOOTUP);   /* wait 16ms or more after power-on       */
	
	/* Step 1,2 */
	mcp23017_write_register(obj, MCP23017_GPIOB, FUNCTION_SET);
	mcp23017_write_register(obj, MCP23017_GPIOB, FUNCTION_SET ^ ENABLE_MASK);
	lcd_e_delay();
	mcp23017_write_register(obj, MCP23017_GPIOB, FUNCTION_SET);
	delay(4100); //4.1ms delay   
	//delay(1000000);
	
	
	/* Step 3 */
	mcp23017_write_register(obj, MCP23017_GPIOB, FUNCTION_SET);
	mcp23017_write_register(obj, MCP23017_GPIOB, FUNCTION_SET ^ ENABLE_MASK);
	lcd_e_delay();
	mcp23017_write_register(obj, MCP23017_GPIOB, FUNCTION_SET);
	delay(100); //100us delay
	//delay(1000000);
	
	/* Step 4 */
	mcp23017_write_register(obj, MCP23017_GPIOB, FUNCTION_SET);
	mcp23017_write_register(obj, MCP23017_GPIOB, FUNCTION_SET ^ ENABLE_MASK);
	lcd_e_delay();
	mcp23017_write_register(obj, MCP23017_GPIOB, FUNCTION_SET);
	delay(100); //100us delay
	//delay(1000000);
	
	
	/* Step 5 */
	mcp23017_write_register(obj, MCP23017_GPIOB, 0x08);
	mcp23017_write_register(obj, MCP23017_GPIOB, 0x08 ^ ENABLE_MASK);
	lcd_e_delay();
	mcp23017_write_register(obj, MCP23017_GPIOB, 0x08);
	delay(100); //100us delay
	//delay(1000000);
	
	
	/* Now the LCD is in 4-bit mode and expects 4-bit commands */
	
	/* Step 6 Set lcd to 4 bits 2 lines (rows) */
	lcd_sendcommand(LCD_FUNCTION_4BIT_2LINES);
	//lcd_sendcommand(MCP23017_GPIOB, 0b01000001);
	delay(53); //53us delay	
	
	/* Step 7 Turn off display*/
	lcd_sendcommand(LCD_DISP_OFF);
	//lcd_sendcommand(MCP23017_GPIOB, 0b00000001);
	delay(53); //53us delay
	
	
	/* Step 8 Clear Display*/
	lcd_sendcommand(1<<LCD_CLR); //CLEAR = 0x01
	//lcd_sendcommand(MCP23017_GPIOB, 0b00001000);
	delay(3000); //3ms delay
	
	
	/* Step 9 Set Entry Mode */
	lcd_sendcommand(LCD_ENTRY_INC_);
	//lcd_sendcommand(0x0D);
	//lcd_sendcommand(MCP23017_GPIOB, 0b00001011);
	delay(53); //53us delay
	
	
	/* Step 10,11 Turn display back on */
	lcd_sendcommand(LCD_DISP_ON);
	//lcd_sendcommand(MCP23017_GPIOB, 0b00000011);
	delay(53); //53us delay
	
	
}/* lcd_init */

/*************************************************************************
Write out a string starting at current cursor position, null terminated
Input:    pointer to string to be displayed
Returns:  none
*************************************************************************/

void lcd_sendcommand(uint8_t data)
{
	uint8_t highByte = 0;
	uint8_t lowByte = 0;
	
	/* input char 0b11111111 needs to be formatted 0bRRE4567L */
	/* for the high byte, 0b0123xxxx would be 0bxxx3210x */
	highByte = ((data&0x80)>>6 | (data&0x40)>>4 | (data&0x20)>>2 | (data&0x10));
		
	/* for the low byte, 0bxxxx4567 would be 0bxxx7654x */
	lowByte = ((data&0x01)<<4 | (data&0x02)<<2 | (data&0x04) | (data&0x08)>>2);
		
	/* Send high byte */
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, highByte);
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, highByte ^ ENABLE_MASK);
	lcd_e_delay();
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, highByte);
	
	/* send low byte */
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, lowByte);
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, lowByte ^ ENABLE_MASK);
	lcd_e_delay();
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, lowByte);
	
}

/*************************************************************************
Display character at current cursor position
Input:    character to be displayed
Returns:  none
*************************************************************************/

void lcd_putc(char data)
{
	uint8_t highByte = 0;
	uint8_t lowByte = 0;
	
	/* input char 0b11111111 needs to be formatted 0bRRE4567L */
	/* for the high byte, 0b0123xxxx would be 0bxxx3210x */
	highByte = ((data&0x80)>>6 | (data&0x40)>>4 | (data&0x20)>>2 | (data&0x10));
	/* for the low byte, 0bxxxx4567 would be 0bxxx7654x */
	lowByte = ((data&0x01)<<4 | (data&0x02)<<2 | (data&0x04) | (data&0x08)>>2);
	
	/* Mask the RS bit high to tell LCD we're sending data, not a command */
	highByte = highByte | 0x80;
	lowByte	= lowByte | 0x80;
	
	/* Send high byte */
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, highByte);
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, highByte ^ ENABLE_MASK); //0x20
	lcd_e_delay();
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, highByte);
	
	/* send low byte */
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, lowByte);
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, lowByte ^ ENABLE_MASK);
	lcd_e_delay();
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, lowByte);
}


/* color = 0b00000RGB */
void change_lcd_color(uint8_t color)
{
	uint8_t gpioa = mcp23017_read_register(&lcdDevice, MCP23017_GPIOA);
	uint8_t gpiob = mcp23017_read_register(&lcdDevice, MCP23017_GPIOB);
	
	uart_puts("Before = ");
	//uart_putu8(gpioa, 16);
	gpioa &= ~((color & 0b00000110 ) << 5);
	uart_puts("\r\nAfter = ");
	//uart_putu8(gpioa, 16);
	
	gpiob &= ~(color & 0b00000001 );
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOB, gpiob);
	mcp23017_write_register(&lcdDevice, MCP23017_GPIOA, gpioa);
}



/*************************************************************************
This function scrolls the display without changing the RAM
	There are 16 characters and 40 RAM positions
	Data written to RAM positions > 16 will be what comes on when you scroll
Input:    none
Returns:  none
*************************************************************************/
void scrollDisplayLeft(void) 
{
	lcd_sendcommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}


/*************************************************************************
Read the buttons on the Adafruit I2C LCD shield
	The buttons are configured with pullup resistors
	
	PA0: Button 1: Select	Returns: 0x01
	PA1: Button 2: Right	Returns: 0x02
	PA2: Button 3: Down		Returns: 0x04
	PA3: Button 4: Up		Returns: 0x08
	PA4: Button 5: Left		Returns: 0x10;
	PA5: NC (I think)
	PA6: output for red backlight
	PA7: output for green backlight
	
	More than 1 button can be pressed at a time; the return will be OR'd
	
Input:    none
Returns:  The port value of the buttons
*************************************************************************/

uint8_t readButtons(void)
{
	uint8_t reply = 0x1F; // buttons are pulled up to 5V

	reply &= ~(mcp23017_read_register(&lcdDevice, MCP23017_GPIOA));

	return reply;
}
