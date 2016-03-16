/**
 *  \file       mcp23017.h
 *  \details    This is the header file for interface to the MCP23017
 *  \author     Alan K. Duncan <duncan.alan@mac.com>
 *  \version    1.0
 *  \date       2011-12-23
 *  \warning    Requires Pascal Stang's i2cmaster.h and i2cmaster.S.  The latter
 *              needs to be modified with the specifications of the target 
 *              AVR.
 *  \copyright  MIT license
 */
 
#ifndef MCP23017_H_
#define MCP23017_H_

//#include "global.h"     //  should define F_CPU here
#include <inttypes.h>

#define BUTTON_UP		0x08
#define BUTTON_DOWN		0x04
#define BUTTON_LEFT		0x10
#define BUTTON_RIGHT	0x02
#define BUTTON_SELECT	0x01

/************************************************************************/
/* MCP23017 REGISTER DEFINITIONS                                        */
/************************************************************************/
#define MCP23017_IODIRA 0x00
#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLA 0x02
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENA 0x04
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALA 0x06
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONA 0x08
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCON 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFA 0x0E
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPA 0x10
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOA 0x12
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATA 0x14
#define MCP23017_OLATB 0x15

/** \struct     MCP23017
 *  \brief      Used to define a MCP23017 device on the I2C bus
 *  \details    MCP23017 type is used to hold the device address and the data to *              be written to the device (or read from the device)
 */
typedef struct {
	uint8_t address;	//	the address of the device
	uint16_t data;		//	data to/from device
} MCP23017;

/** \fn         void mcp23017_init(MCP23017 *obj, uint8_t address)
 *  \brief      Initialize the MCP23017 device.
 *  \details    Setup the device to use on the I2C bus.
 *  \param      address The uint8_t address should just be the hardware address
 *              component setup at the circuit level, i.e. at A2-A0 (e.g.
 *              0b000 if all are GND.)
 */
void mcp23017_init(MCP23017 *obj, uint8_t address);

/** \fn         void mcp23017_write(MCP23017 *object)
 *  \brief      Writes 16 bits of data
 *  \details    Write 16 bits of data to the device with the GPIOA register
 *              representing the lower 8 bits and GPIOB the upper 8 bits.
 *  \param      object A pointer to the MCP23017 structure.  Its data
 *              member should hold the data to be written.
 */
void mcp23017_write(MCP23017 *object);

/** \fn         mcp23017_write_register( MCP23017 *obj, uint8_t reg, uint8_t data)
 *  \brief      Writes data to register
 *  \param      obj A pointer to the MCP23017 structure.
 *  \param      reg The register to be written as uint8_t.
 *  \param      data The data to be written as uint8_t.
 */
void mcp23017_write_register( MCP23017 *obj, uint8_t reg, uint8_t data);

/** \fn         mcp23017_read_register( MCP23017 *obj, uint8_t reg)
 *  \brief      Reads a register
 *  \param      obj A pointer to the MCP23017 structure.
 *  \param      reg The register to be read as uint8_t.  
 *  \return     The method returns the contents of the specified register as uint8_t
 */
uint8_t mcp23017_read_register( MCP23017 *obj, uint8_t reg);

/** \fn         mcp23017_write_word( MCP23017 *obj, uint8_t reg )
 *  \brief      Writes 16 bits of data.  Assumes sequential mode.
 *  \param      obj A pointer to the MCP23017 structure.
 *  \param      reg The register to be written as uint8_t.
 */
void mcp23017_write_word( MCP23017 *obj, uint8_t reg );

#endif /* MCP23017_H_ */