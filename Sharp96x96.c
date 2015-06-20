//
// Sharp96x96.c - Tiva interface to the Sharp Memory LCD BoosterPack.
//
// Copyright (c) 2015 Donald Rich.
// Portions derived from TI Sharp96x96.c in EXP430FR5969 Sharp LCD sample code.
//
// This software is supplied solely as a programming example.
//
// This file is part of SharpLcdTivaInterface.
//
// SharpLcdTivaInterface is free software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SharpLcdTivaInterface is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with SharpLcdTivaInterface.  If not, see
// <http://www.gnu.org/licenses/>.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"

#include "grlib/grlib.h"
#include "Sharp96x96.h"

//#include <stdint.h>

// LCD Screen Size
#define VERTICAL_MAX                   96
#define HORIZONTAL_MAX                 96

//*****************************************************************************
//
// Macros for the Display Driver
//
//*****************************************************************************
#define BLACK							0x00
#define WHITE							0xFF
#define SEND_TOGGLE_VCOM_COMMAND		0x01
#define SKIP_TOGGLE_VCOM_COMMAND 		0x00
#define TRAILER_BYTE					0x00
#define VCOM_TOGGLE_BIT 		   		0x40
#define CMD_CHANGE_VCOM					0x00
#define CMD_CLEAR_SCREEN				0x20
#define CMD_WRITE_LINE					0x80

// Ports for TM4C1233H6PM connections to LCD
#define SPI_SI_PORT                     SYSCTL_PERIPH_GPIOB
#define SPI_CLK_PORT                    SYSCTL_PERIPH_GPIOB
#define DISP_PORT                       SYSCTL_PERIPH_GPIOE
#define POWER_PORT                      SYSCTL_PERIPH_GPIOB
#define SPI_CS_PORT     	            SYSCTL_PERIPH_GPIOE

// Port base address for TM4C1233H6PM connections to LCD
#define SPI_SI_PORT_BASE                GPIO_PORTB_BASE
#define SPI_CLK_PORT_BASE               GPIO_PORTB_BASE
#define DISP_PORT_BASE                  GPIO_PORTE_BASE
#define POWER_PORT_BASE                 GPIO_PORTB_BASE
#define SPI_CS_PORT_BASE	            GPIO_PORTE_BASE

// Pins for TM4C1233H6PM connections to LCD
#define SPI_SI_PIN                      GPIO_PIN_7
#define SPI_CLK_PIN                     GPIO_PIN_4
#define DISP_PIN                        GPIO_PIN_4
#define POWER_PIN                       GPIO_PIN_5
#define SPI_CS_PIN                      GPIO_PIN_5

// Definition of SSI base address to be used for SPI communication
#define LCD_SSI_BASE					SSI2_BASE

// General use counter. Incremented each time the VCOM inversion interrupt is
// called. Not used in the Sharp96x96 code. Can be reset by user code.
unsigned int Sharp96x96_VCOM_count = 0;

static uint8_t VCOMbit= 0x40;
static unsigned char currentVCOMbit = 0x40;

// Caution: The following declaration only works if HORIZONTAL_MAX is a
// multiple of 8!
static uint8_t DisplayBuffer[VERTICAL_MAX][HORIZONTAL_MAX/8];

//*****************************************************************************
//
// Clears CS line
//
// This function clears the Chip Select (CS) line
//
// \return None
//
//*****************************************************************************
static void ClearCS(void){
	GPIOPinWrite(SPI_CS_PORT_BASE, SPI_CS_PIN, 0);
}

//*****************************************************************************
//
// Set CS line
//
// This function sets the Chip Select (CS) line
//
// \return None
//
//*****************************************************************************
static void SetCS(void){
	GPIOPinWrite(SPI_CS_PORT_BASE, SPI_CS_PIN, SPI_CS_PIN);
}

//*****************************************************************************
//
// Get CS line
//
// This function gets the Chip Select (CS) line
//
// \return long
//
//*****************************************************************************
static long GetCS(void)
{
	return GPIOPinRead(SPI_CS_PORT_BASE, SPI_CS_PIN);
}

// Delay counts used to enforce SPI timing constraints.
static unsigned long twoUsDelayCount;
static unsigned long sixUsDelayCount;

//*****************************************************************************
//
// Waits until the SPI communication with the LCD is finished a command to
// the LCD Driver
//
// \param None
//
// \return None
//*****************************************************************************
static void WaitUntilLcdWriteDone(void)
{
	while (SSIBusy(LCD_SSI_BASE))
	{
		// Just wait
	}
}

//*****************************************************************************
//
// Writes command or data to the LCD Driver
//
// \param ucCmdData is the 8 or 16 bit command to send to the LCD driver
// Uses the SET_DATA macro
//
// \return None
//
//*****************************************************************************
static void WriteByte(uint8_t byte)
{
	WaitUntilLcdWriteDone();
	SSIDataPut(LCD_SSI_BASE, byte);
}

static void WriteCommand(unsigned char *command, unsigned char length)
{
	int i;
	unsigned char * bytePointer = command;

	// Set the VCOM bit in the command byte
	command[0] &= ~VCOMbit;
	command[0] |= currentVCOMbit;

	SetCS();

	// Ensure a 6us min delay to meet the LCD's tsSCS
	SysCtlDelay(sixUsDelayCount);

	for(i = 0; i < length; i++)
	{
		WriteByte(*bytePointer++);
	}

	// Wait for last byte to be sent
	WaitUntilLcdWriteDone();

	// Ensure a 2us min delay to meet the LCD's thSCS
	SysCtlDelay(twoUsDelayCount);

	ClearCS();

	// Ensure a 2us delay to meet the LCD's twSCSL
	SysCtlDelay(twoUsDelayCount);
}

//*****************************************************************************
//
//! Send toggle VCOM command.
//!
//! This function toggles the state of VCOM which prevents a DC bias from being
//! built up within the panel.
//!
//! \return None.
//
//*****************************************************************************
static unsigned char ToggleVComCommand[] = {
	CMD_CHANGE_VCOM,
	TRAILER_BYTE
};

void SendToggleVCOMCommand(void)
{
	Sharp96x96_VCOM_count++; // Used for secondary timing functions unrelated to the Sharp LCD.

	// Toggle the VCOM bit.
	currentVCOMbit ^= VCOMbit;

	if(!GetCS()) // CS is low, not already sending command
	{
		WriteCommand(ToggleVComCommand, 2);
	}
}

static void VCOMtimerInit(void)
{
    // Enable the timer for periodic interrupts to drive the VCOM inversion
	// and other general timed functions.
	// The clock is configured to timeout every half second to give a 1 hz
	// alternation of VCOM.
	SysTickPeriodSet(SysCtlClockGet() / 2); // must be less than 16,777,216
	SysTickIntRegister(SendToggleVCOMCommand);
	SysTickIntEnable();
	SysTickEnable();
}

//*****************************************************************************
//
//! Initializes the display driver.
//!
//! This function initializes the Sharp96x96 display. This function
//! configures the GPIO pins used to control the LCD display when the basic
//! GPIO interface is in use. On exit, the LCD has been reset and is ready to
//! receive command and data writes.
//!
//! \return None.
//
//*****************************************************************************
static void InitDisplay(void)
{
	// Clock frequency / 3 (cycles per Delay tick) / 500000 (2 uS periods in a second)
	twoUsDelayCount = SysCtlClockGet() / 1500000 + 1;
	sixUsDelayCount = SysCtlClockGet() / 500000 + 1;

	// Configure the GPIO Port B pin 5 for output to power the LCD Display.
    SysCtlPeripheralEnable(POWER_PORT);
    GPIOPinTypeGPIOOutput(POWER_PORT_BASE, POWER_PIN);
    GPIOPinWrite(POWER_PORT_BASE, POWER_PIN, POWER_PIN);

    // Configure the GPIO Port E pin 4 for output to enable the LCD Display.
    SysCtlPeripheralEnable(DISP_PORT);
    GPIOPinTypeGPIOOutput(DISP_PORT_BASE, DISP_PIN);
    GPIOPinWrite(DISP_PORT_BASE, DISP_PIN, DISP_PIN);

    // Configure the GPIO Port E pin 5 for SPI Chip Select for the LCD Display.
    GPIOPinTypeGPIOOutput(SPI_CS_PORT_BASE, SPI_CS_PIN);
    GPIOPinWrite(SPI_CS_PORT_BASE, SPI_CS_PIN, 0); // Output by default.

	ClearCS();

	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	SSIClockSourceSet(LCD_SSI_BASE, SSI_CLOCK_SYSTEM);
	SSIConfigSetExpClk(LCD_SSI_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_MASTER, 1000000, 8);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
	// Split if SPI_SI and SPI_CLK are on different GPIO ports.
    GPIOPinTypeGPIOOutput(SPI_SI_PORT_BASE, SPI_SI_PIN | SPI_CLK_PIN);
	GPIOPinTypeSSI(SPI_SI_PORT_BASE, SPI_SI_PIN | SPI_CLK_PIN);
    SSIEnable(LCD_SSI_BASE);

    VCOMtimerInit();
}

static unsigned char
ClearCommand[] = {
	CMD_CLEAR_SCREEN,
	TRAILER_BYTE
};

static void ClearScreen(void)
{
	unsigned char oldVCOM = currentVCOMbit;

	WriteCommand(ClearCommand, 2);

	// SendToggleVCOMCommand called while sending the command.
	if(oldVCOM != currentVCOMbit)
	{
		WriteCommand(ToggleVComCommand, 2);
	}
}

//*****************************************************************************
//
//! Initializes the display driver.
//!
//! This function initializes the Sharp96x96 display controller preparing it to
//! display data.
//!
//! \return None.
//
//*****************************************************************************
void Sharp96x96_initDisplay(void)
{
	InitDisplay();
	ClearScreen();
}

void Sharp96x96_disable(void)
{
	GPIOPinWrite(POWER_PORT_BASE, POWER_PIN, 0);
	GPIOPinWrite(DISP_PORT_BASE, DISP_PIN, 0);
}

void Sharp96x96_enable(void)
{
	GPIOPinWrite(POWER_PORT_BASE, POWER_PIN, POWER_PIN);
	GPIOPinWrite(DISP_PORT_BASE, DISP_PIN, DISP_PIN);
}

//*****************************************************************************
//
//! Draws a pixel on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the pixel.
//! \param lY is the Y coordinate of the pixel.
//! \param ulValue is the color of the pixel.
//!
//! This function sets the given pixel to a particular color.  The coordinates
//! of the pixel are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void PixelDraw(void *pvDisplayData, int32_t lX, int32_t lY,
	uint32_t ulValue)
{
	if( ClrBlack == ulValue){
		DisplayBuffer[lY][lX>>3] &= ~(0x80 >> (lX & 0x7));
	}else{
		DisplayBuffer[lY][lX>>3] |= (0x80 >> (lX & 0x7));
	}
}

//*****************************************************************************
//
//! Draws a horizontal sequence of pixels on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the first pixel.
//! \param lY is the Y coordinate of the first pixel.
//! \param lX0 is sub-pixel offset within the pixel data, which is valid for 1
//! or 4 bit per pixel formats.
//! \param lCount is the number of pixels to draw.
//! \param lBPP is the number of bits per pixel; must be 1, 4, or 8.
//! \param pucData is a pointer to the pixel data.  For 1 and 4 bit per pixel
//! formats, the most significant bit(s) represent the left-most pixel.
//! \param pucPalette is a pointer to the palette used to draw the pixels.
//!
//! This function draws a horizontal sequence of pixels on the screen, using
//! the supplied palette.  For 1 bit per pixel format, the palette contains
//! pre-translated colors; for 4 and 8 bit per pixel formats, the palette
//! contains 24-bit RGB values that must be translated before being written to
//! the display.
//!
//! \return None.
//
//*****************************************************************************
static void DrawMultiple(void *pvDisplayData, int32_t lX,
		int32_t lY, int32_t lX0, int32_t lCount, int32_t lBPP,
	const uint8_t *pucData, const uint8_t *pucPalette)
{
	uint8_t *pData = &DisplayBuffer[lY][lX>>3];
	uint16_t xj = 0;

	//Write bytes of data to the display buffer
	for(xj=0;xj<(lCount >> 3);xj++){
		*pData++ = *pucData++;
	}

	//Write last data byte to the display buffer
	*pData = (*pData & (0xFF >> (lCount & 0x7))) | *pucData;
}

//*****************************************************************************
//
//! Draws a horizontal line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX1 is the X coordinate of the start of the line.
//! \param lX2 is the X coordinate of the end of the line.
//! \param lY is the Y coordinate of the line.
//! \param ulValue is the color of the line.
//!
//! This function draws a horizontal line on the display.  The coordinates of
//! the line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void LineDrawH(void *pvDisplayData, int32_t lX1, int32_t lX2,
		int32_t lY, uint32_t ulValue)
{
	uint16_t xi = 0;
	uint16_t x_index_min = lX1>>3;
	uint16_t x_index_max = lX2>>3;
	uint8_t *pucData, ucfirst_x_byte, uclast_x_byte;

	//calculate first byte
	//mod by 8 and shift this # bits
	ucfirst_x_byte = (0xFF >> (lX1 & 0x7));    
	//calculate last byte
	//mod by 8 and shift this # bits
	uclast_x_byte = (0xFF << (7-(lX2 & 0x7))); 

	//check if more than one data byte
	if(x_index_min != x_index_max){

		//set buffer to correct location
		pucData = &DisplayBuffer[lY][x_index_min];

		//black pixels (clear bits)
		if(ClrBlack == ulValue)
		{
			//write first byte
			*pucData++ &= ~ucfirst_x_byte;

			//write middle bytes
			for(xi = x_index_min; xi < x_index_max-1; xi++)
			{
				*pucData++ = 0x00;
			}

			//write last byte
			*pucData &= ~uclast_x_byte;
		}
		//white pixels (set bits)
		else
		{
			//write first byte
			*pucData++ |= ucfirst_x_byte;

			//write middle bytes
			for(xi = x_index_min; xi < x_index_max-1; xi++)
			{
				*pucData++ = 0xFF;
			}

			//write last byte
			*pucData |= uclast_x_byte;
		}
	}
	//only one data byte
	else
	{
		//calculate value of single byte
		ucfirst_x_byte &= uclast_x_byte;

		//set buffer to correct location
		pucData = &DisplayBuffer[lY][x_index_min];

		//draw black pixels (clear bits)
		if(ClrBlack == ulValue)
		{
			*pucData++ &= ~ucfirst_x_byte;
		}
		//white pixels (set bits)
		else
		{
			*pucData++ |= ucfirst_x_byte;
		}
	}
}


//*****************************************************************************
//
//! Draws a vertical line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the line.
//! \param lY1 is the Y coordinate of the start of the line.
//! \param lY2 is the Y coordinate of the end of the line.
//! \param ulValue is the color of the line.
//!
//! This function draws a vertical line on the display.  The coordinates of the
//! line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void LineDrawV(void *pvDisplayData, int32_t lX, int32_t lY1,
		int32_t lY2, uint32_t ulValue)
{

	uint16_t yi = 0;
	uint16_t x_index = lX>>3;
	uint8_t data_byte;

	//calculate data byte
	//mod by 8 and shift this # bits
	data_byte = (0x80 >> (lX & 0x7));     

	//write data to the display buffer
	for(yi = lY1; yi <= lY2; yi++){

		//black pixels (clear bits)
		if(ClrBlack == ulValue)
		{
			DisplayBuffer[yi][x_index] &= ~data_byte;
		}
		//white pixels (set bits)
		else
		{
			DisplayBuffer[yi][x_index] |= data_byte;
		}
	}
}

//*****************************************************************************
//
//! Fills a rectangle.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param pRect is a pointer to the structure describing the rectangle.
//! \param ulValue is the color of the rectangle.
//!
//! This function fills a rectangle on the display.  The coordinates of the
//! rectangle are assumed to be within the extents of the display, and the
//! rectangle specification is fully inclusive (in other words, both sXMin and
//! sXMax are drawn, along with sYMin and sYMax).
//!
//! \return None.
//
//*****************************************************************************
static void RectFill(void *pvDisplayData, const tRectangle *pRect,
	uint32_t color)
{
    unsigned int y;

    for(y = pRect->i16YMin; y <= pRect->i16YMax; y++)
    {
    	LineDrawH(0, pRect->i16XMin, pRect->i16XMax, y, color);
    }
}

//*****************************************************************************
//
//! Translates a 24-bit RGB color to a display driver-specific color.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//!   display driver
//! \param ulValue is the 24-bit RGB color.  The least-significant byte is the
//!   blue channel, the next byte is the green channel, and the third byte is
//!   the red channel.
//!
//! This fucntion translates a 24-bit RGB color into a value that can be written
//! into the display's frame buffer in order to reproduce that color, or the
//! closest possible approximation of that color. This particular driver
//! requires the 8-8-8 24 bit RGB color to convert into mono color
//! 1 = White, 0 = Black
//!
//! \return Returns the display-driver specific color
//
//*****************************************************************************
static uint32_t ColorTranslate(void *pvDisplayData,
	uint32_t ulValue)
{
    //
    // Translate from a 24-bit RGB color to mono color.
    //
    return(((ulValue != 0) ? ulValue = 1 : ulValue ));
}

//*******************************************************************************
//
//! Reverses the bit order.- Since the bit reversal function is called
//! frequently by the several driver function this function is implemented
//! to maximize code execution
// Taken from TI Sharp96x96.c in EXP430FR5969 Sharp LCD sample code.
//
//*******************************************************************************
static const uint8_t reverse_data[] = {0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE, 0x1,
		0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF};
static uint8_t Reverse(uint8_t x)
{
  uint8_t b = 0;

  b  = reverse_data[x & 0xF]<<4;
  b |= reverse_data[(x & 0xF0)>>4];
  return b;
}

//*****************************************************************************
//
//! Flushes any cached drawing operations.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//!
//!
//! This functions flushes any cached drawing operations to the display.  This
//! is useful when a local frame buffer is used for drawing operations, and the
//! flush would copy the local frame buffer to the display.
//!
//! \return None.
//
//*****************************************************************************
static void Flush (void *pvDisplayData)
{
	uint8_t *pucData = &(DisplayBuffer[0][0]);
	int32_t xi =0;
	int32_t xj = 0;

	uint8_t command = CMD_WRITE_LINE;

	// Set the VCOM bit in the command
	command &= ~VCOMbit;
	command |= currentVCOMbit;

	unsigned char oldVCOM = currentVCOMbit;

	SetCS();

	// Ensure a 6us min delay to meet the LCD's tsSCS
	SysCtlDelay(sixUsDelayCount);

	WriteByte(command);

	for(xj=0; xj<VERTICAL_MAX; xj++)
	{
		WriteByte(Reverse(xj + 1));

		for(xi=0; xi<(HORIZONTAL_MAX>>3); xi++)
		{
			WriteByte(*(pucData++));
		}
		WriteByte(TRAILER_BYTE);
	}

	WriteByte(TRAILER_BYTE);

	// Wait for last byte to be sent, then drop SCS
	WaitUntilLcdWriteDone();

    // Ensure a 2us min delay to meet the LCD's thSCS
	SysCtlDelay(twoUsDelayCount);

	ClearCS();

	// Ensure a 2us min delay to meet the LCD's twSCSL
	SysCtlDelay(twoUsDelayCount);

	// SendToggleVCOMCommand called while sending the command.
	if(oldVCOM != currentVCOMbit)
	{
		WriteCommand(ToggleVComCommand, 2);
	}
}

//*****************************************************************************
//
//! The display structure that describes the driver for the 
//! sharpLCD panel 
//
//*****************************************************************************
const tDisplay g_sharp96x96LCD =
{
    sizeof(tDisplay),
    DisplayBuffer,
    HORIZONTAL_MAX,
    VERTICAL_MAX,
    PixelDraw,
    DrawMultiple,
    LineDrawH,
    LineDrawV,
    RectFill,
    ColorTranslate,
    Flush
};

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
