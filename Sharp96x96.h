//
// Sharp96x96.h - Tiva interface to the Sharp Memory LCD BoosterPack.
//
// Copyright (c) 2015 Donald Rich.
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

#ifndef __SHARPLCD_H__
#define __SHARPLCD_H__

#include "grlib/grlib.h"

// The Graphics_Display object contains pointers to the public
// functions of the interface used in the graphics library.
extern const tDisplay g_sharp96x96LCD;

// A counter driven at 2Hz by the VCOM inversion interrupt. Not otherwise
// used in the LCD code but provided for use in general purpose timing.
extern unsigned int Sharp96x96_VCOM_count;

extern void Sharp96x96_initDisplay(void);
extern void Sharp96x96_disable(void);
extern void Sharp96x96_enable(void);

#endif // __SHARPLCD_H__
