//
// main.c - Tiva interface to the Sharp Memory LCD BoosterPack test program.
//
// Copyright (c) 2015 Donald Rich.
//
// This software is supplied solely as a programming example. Modified from
// the grlib_demo.c program supplied as an example of the TivaWare Graphics
// Library. The original license for grlib_demo.c is provided below.
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
//*****************************************************************************
//
// grlib_demo.c - Demonstration of the TivaWare Graphics Library.
//
// Copyright (c) 2008-2015 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.1.71 of the DK-TM4C123G Firmware Package.
//
//*****************************************************************************


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "grlib/grlib.h"
#include "Sharp96x96.h"

tContext g_sContext;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

#define AUTO_STRING_LENGTH	-1
#define TRANSPARENT_TEXT	false
#define OPAQUE_TEXT			true

tRectangle myRectangle1 = { 5, 15, 65, 45};
tRectangle myRectangle2 = { 10, 40, 90, 90};
tRectangle myRectangle3 = { 0, 0, 95, 95};

void clockInit(void)
{
    // Set the clocking to 20Mhz (400Mhz PLL / 2 default divider / 10).
	// Clock must be less than 32Mhz for use with sharplcd_debug code.
	// The limitation is imposed by the use of SysTick for VCOM inversion
	// timing.
	SysCtlClockSet(SYSCTL_SYSDIV_10 |SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
		SYSCTL_XTAL_16MHZ);
}

// Needed since the TivaWare Graphics Library seems to not have a function
// for clearing the display.
void BlankScreen(void)
{
    GrContextForegroundSet(&g_sContext, ClrWhite);
	GrContextBackgroundSet(&g_sContext, ClrBlack);
    GrRectFill(&g_sContext, &(g_sContext.sClipRegion));
    GrFlush(&g_sContext);
	GrContextForegroundSet(&g_sContext, ClrBlack);
	GrContextBackgroundSet(&g_sContext, ClrWhite);
}
//*****************************************************************************
//
// A simple demonstration of the features of the TivaWare Graphics Library.
//
//*****************************************************************************
int
main(void)
{
  	clockInit();

    // Set up the LCD
  	Sharp96x96_initDisplay();

    GrContextInit(&g_sContext, &g_sharp96x96LCD);
  	GrContextFontSet(&g_sContext, &g_sFontFixed6x8);

  	int step = -1;
  	Sharp96x96_VCOM_count = 10;
  	while(1)
  	{
  		if(Sharp96x96_VCOM_count < 4)
  		{
  			continue;
  		}
  		Sharp96x96_VCOM_count = 0;
  		if(++step >= 5)
  		{
  			step = 0;
  		}

  		BlankScreen();
  	  	GrFlush(&g_sContext);
		switch(step) {
		case 0:
			// Intro Screen
			GrStringDrawCentered(&g_sContext,
								 "How to use",
								 AUTO_STRING_LENGTH,
								 48,
								 15,
								 TRANSPARENT_TEXT);
			GrStringDrawCentered(&g_sContext,
								 "the TivaWare",
								 AUTO_STRING_LENGTH,
								 48,
								 35,
								 TRANSPARENT_TEXT);
			GrStringDraw(&g_sContext,
						 "Graphics Library",
						 AUTO_STRING_LENGTH,
						 1,
						 51,
						 TRANSPARENT_TEXT);
			GrStringDrawCentered(&g_sContext,
								 "Primitives",
								 AUTO_STRING_LENGTH,
								 48,
								 75,
								 TRANSPARENT_TEXT);
			break;
		case 1:
			// Draw pixels and lines on the display
			GrStringDrawCentered(&g_sContext,
								 "Draw Pixels",
								 AUTO_STRING_LENGTH,
								 48,
								 5,
								 TRANSPARENT_TEXT);
			GrStringDrawCentered(&g_sContext,
								 "& Lines",
								 AUTO_STRING_LENGTH,
								 48,
								 15,
								 TRANSPARENT_TEXT);
			GrPixelDraw(&g_sContext, 30, 30);
			GrPixelDraw(&g_sContext, 30, 32);
			GrPixelDraw(&g_sContext, 32, 32);
			GrPixelDraw(&g_sContext, 32, 30);
			GrLineDraw(&g_sContext, 35, 35, 90, 90);
			GrLineDraw(&g_sContext, 5, 80, 80, 20);
			GrLineDraw(&g_sContext,
					   0,
					   GrContextDpyHeightGet(&g_sContext) - 1,
					   GrContextDpyWidthGet(&g_sContext) - 1,
					   GrContextDpyHeightGet(&g_sContext) - 1);
			break;
		case 2:
			// Draw circles on the display
			GrStringDraw(&g_sContext,
						 "Draw Circles",
						 AUTO_STRING_LENGTH,
						 10,
						 5,
						 TRANSPARENT_TEXT);
			GrCircleDraw(&g_sContext,
						 30,
						 70,
						 20);
			GrCircleFill(&g_sContext,
						 60,
						 50,
						 30);
			break;
		case 3:
			// Draw rectangles on the display
			GrStringDrawCentered(&g_sContext,
								 "Draw Rectangles",
								 AUTO_STRING_LENGTH,
								 48,
								 5,
								 TRANSPARENT_TEXT);
			GrRectDraw(&g_sContext, &myRectangle1);
			GrRectFill(&g_sContext, &myRectangle2);
			// Text below won't be visible on screen due to transparency
			// (foreground colors match)
			GrStringDrawCentered(&g_sContext,
								 "Normal Text",
								 AUTO_STRING_LENGTH,
								 50,
								 50,
								 TRANSPARENT_TEXT);
			// Text below draws foreground and background for opacity
			GrStringDrawCentered(&g_sContext,
								 "Opaque Text",
								 AUTO_STRING_LENGTH,
								 50,
								 65,
								 OPAQUE_TEXT);
			GrContextForegroundSet(&g_sContext, ClrWhite);
			GrContextBackgroundSet(&g_sContext, ClrBlack);
			GrStringDrawCentered(&g_sContext,
								 "Invert Text",
								 AUTO_STRING_LENGTH,
								 50,
								 80,
								 TRANSPARENT_TEXT);
			break;
		case 4:
			// Invert the foreground and background colors
			GrContextForegroundSet(&g_sContext, ClrBlack);
			GrContextBackgroundSet(&g_sContext, ClrWhite);
			GrRectFill(&g_sContext, &myRectangle3);
			GrContextForegroundSet(&g_sContext, ClrWhite);
			GrContextBackgroundSet(&g_sContext, ClrBlack);
			GrStringDrawCentered(&g_sContext,
								 "Invert Colors",
								 AUTO_STRING_LENGTH,
								 48,
								 5,
								 TRANSPARENT_TEXT);
			GrRectDraw(&g_sContext, &myRectangle1);
			GrRectFill(&g_sContext, &myRectangle2);
			// Text below won't be visible on screen due to
			// transparency (foreground colors match)
			GrStringDrawCentered(&g_sContext,
								 "Normal Text",
								 AUTO_STRING_LENGTH,
								 50,
								 50,
								 TRANSPARENT_TEXT);
			// Text below draws foreground and background for opacity
			GrStringDrawCentered(&g_sContext,
								 "Opaque Text",
								 AUTO_STRING_LENGTH,
								 50,
								 65,
								 OPAQUE_TEXT);
			// Text below draws with inverted foreground color to become visible
			GrContextForegroundSet(&g_sContext, ClrBlack);
			GrContextBackgroundSet(&g_sContext, ClrWhite);
			GrStringDrawCentered(&g_sContext,
								 "Invert Text",
								 AUTO_STRING_LENGTH,
								 50,
								 80,
								 TRANSPARENT_TEXT);
			break;
		default:
			break;
		}
		GrFlush(&g_sContext);
  	}
}
