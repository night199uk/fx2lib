/**
 * Copyright (C) 2008 Ubixum, Inc. 
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 **/

#ifndef GPIF_H
#define GPIF_H

#include "fx2types.h"


/**
 * Gpif designer generates a c file with waveform data.
 * Copy the WaveData[128] array
 * and the InitData[7] to your code somewhere
 * Then this function is pretty much the reset of the generated
 * code but ported to sdcc.
 *
 * uses syncdelay of 4 which might not be long enough if peripheral
 * runs slower than 30mhz
 *
 * IFCONFIG is set with IFCFG[1:0] = 10 for GPIF master but you still
 * have to set the ifclk, polarity, and the rest of the bits
 **/

void gpif_init( BYTE* waveform, BYTE* initdata );

/**
 * Uses the correct bytes from your flowstates array.
 * This may or may not be needed depending on whether
 * your waveform data uses flowstates.  If you don't
 * know if you need them or not, you probably don't.
 *
 * flowstates should have 36 elements.
 * bank should be 0-3
 **/
void gpif_setflowstate( BYTE* flowstates, BYTE bank);



// These defines/functions pretty much out of the TRM 10.4
#define GPIFTRGWR 0
#define GPIFTRGRD 4
typedef enum {
    GPIF_EP2 = 0,
    GPIF_EP4 = 1,
    GPIF_EP6 = 2,
    GPIF_EP8 = 3
} GPIF_EP_NUM;

/**
 * Use the gpif to read a single byte at a time.
 * Read len bytes and store in res
 *
 * This is coded to use 16 bit data bus.  Len
 * Should be multiple of 2.  Should this function
 * be coded to be smart enough for 8 bit bus and 16 both
 * or should there be a different function?
 **/
void gpif_single_read( BYTE* res , WORD len) ;

void gpif_fifo_read ( GPIF_EP_NUM ep_num );

void gpif_fifo_write( GPIF_EP_NUM ep_num );

#endif
