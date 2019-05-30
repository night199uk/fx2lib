/*
 * Copyright (C) 2012 Sven Schnelle <svens@stackframe.org>
 * Copyright (C) 2012 Kyösti Mälkki <kyosti.malkki@gmail.com>
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

#include <fx2regs.h>
#include <fx2macros.h>
#include <delay.h>
#include <autovector.h>
#include <gpif.h>
#include <setupdat.h>
#include <eputils.h>
#include <i2c.h>

#define SYNCDELAY SYNCDELAY4

// uncomment the next line to enable debugging via printf() via I2C.
// this can be used to feed an I2C/UART converter e.g. based on an Arduino.
// note: the default I2C target address is 0x10.
// note: ONLY ENABLE THIS IF a consumer is present on the I2C bus, as the write
//       will block until the data is read - i.e. it will hang the device and
//       no output will be produced!
//#define DEBUG_DEBUGSLAVE

#ifdef DEBUG_DEBUGSLAVE
#include <i2c.h>
#include <stdio.h>

#define I2CDEBUG_ADDR (0x10)
void putchar(char c) {
    i2c_write (I2CDEBUG_ADDR, 1, &c, 0, NULL);
}
#else
#define printf(...)
#endif

volatile __bit got_sud;
extern __code WORD debug_dscr;

/* PORTA.3 */
// indicate on D1 that IN  EP6 is full
#define PA_LED1			(1 << 0)
// indicate on D2 that OUT EP2 is empty
#define PA_LED2			(1 << 1)

// PA5/4 (FIFOADR1,0) select the endpoint to communicate with
#define PA_nFIFOADDR    (3 << 4)
#define PA_nPKTEND		(1 << 6)

#define EP2EMPTY        (0x02)
#define EP6FULL         (0x01)

void main()
{
    // Bit7:   reserved
    // Bit6:   reserved
    // Bit5:   PORTCSTB
    // Bit4-3: CLKSPD[1:0] - 0b10: 48MHz clock
    // Bit2:   CLKINV
    // Bit1:   CLKOE - enable CLKOUT pin
    // Bit0:   8051RES
    CPUCS = 0x12;

    // Bit7: 0 = External, 1 = Internal...1 - Internal
    // Bit6: 0 = 30Mhz Clk, 1 = 48 Mhz Clk...1 - 48Mhz
    // Bit5: 0 = No ClkOut, 1 = ClkOut...0 - No ClkOut
    // Bit4: 0 = NormalPol, 1 = InvPol...0 = NormalPol
    // Bit3: 0 = Synchronous, 1 = Asynchronous...1 = Async
    // Bit2: 0 = GStateOff, 1 = GStateOn...1 = GState On
    // Bit1-0: 0 = Ports, 1 = Reserved, 2 = GPIF Master, 3 = Slave FIFO...3 - Slave FIFO
    IFCONFIG = 0x4F;

    // Bit0 = ENH_PKT - Enhanced Packet Handling.
    // Bit1 = DYN_OUT - Disable Auto-Arming at the 0-1 transition of AUTOOUT.
    RENUMERATE_UNCOND();

    USE_USB_INTS();
    ENABLE_SUDAV();
    ENABLE_SOF();
    ENABLE_HISPEED();
    ENABLE_USBRESET();

    /* INT endpoint */
    EP1INCFG = bmVALID | (3 << 4); SYNCDELAY;

    /* disable all other endpoints */
    EP1OUTCFG &= ~bmVALID; SYNCDELAY;

    // Bit1:0 = BUF1:0 = 0, 0 = QUAD
    // Bit1:0 = BUF1:0 = 0, 1 = INVALID
    // Bit1:0 = BUF1:0 = 1, 0 = DOUBLE
    // Bit1:0 = BUF1:0 = 1, 1 = TRIPLE
    // Bit2 = 0
    // Bit3 = SIZE, 0 = 512 bytes, 1 = 1024 bytes
    // Bit5:4 = TYPE1:0 = 0, 0 = INVALID
    // Bit5:4 = TYPE1:0 = 0, 1 = ISO
    // Bit5:4 = TYPE1:0 = 1, 0 = BULK (def)
    // Bit5:4 = TYPE1:0 = 1, 1 = INT
    // Bit6 = DIR, 0 = OUT, 1 = IN
    // Bit7 = VALID
    // EP2: 10100000 - DIR=OUT, TYPE=BULK, SIZE=512, BUF=QUAD
    EP2CFG  = 0xA0;     SYNCDELAY;
    // EP6: 11100000 - DIR=IN, TYPE=BULK, SIZE=512, BUF=QUAD
    EP6CFG  = 0xE0;     SYNCDELAY;
    EP4CFG &= ~bmVALID; SYNCDELAY;
    EP8CFG &= ~bmVALID; SYNCDELAY;

    RESETFIFOS();

    /* BULK OUT endpoint EP2 */
    // Bit0 = WORDWIDE
    // Bit1 = 0
    // Bit2 = ZEROLENIN
    // Bit3 = AUTOIN
    // Bit4 = AUTOOUT
    // Bit5 = OEP2
    // Bit6 = INFM2
    // Bit7 = 0
    /* BULK OUT endpoint EP2 */
    EP2FIFOCFG = 0;         SYNCDELAY;
    EP2FIFOCFG = bmAUTOOUT; SYNCDELAY;
    EP6FIFOCFG = bmAUTOIN;  SYNCDELAY;

    /*
     * !!!!!!!!! NOTE !!!!!!!!!
     * On the FX2LP PCBs I have here, RDY0 and RDY1
     * are labelled incorrectly and need to be swapped.
     */

    // FLAGA = Indexed
    // FLAGB = EP6 FF/CTL1 (Master RDY0)
    PINFLAGSAB = 0xE0; SYNCDELAY;
    // FLAGC = EP2 EF/CTL2 (Master RDY1)
    // FLAGD = Indexed
    PINFLAGSCD = 0x08; SYNCDELAY;

    FIFOPINPOLAR = 0x00; SYNCDELAY;

    // enable LEDs as output and light up LED2
    // note: you can verify that the firmware is reaching this stage once
    //       D2 is lit on the FX2LP board
    OEA = PA_LED1|PA_LED2; SYNCDELAY;
    IOA =         PA_LED2; SYNCDELAY;

    delay(10);

    EA=1; // global __interrupt enable

    printf("entering main loop...\n");
    while(TRUE) {
        if (got_sud) {
            handle_setupdata();
            got_sud=FALSE;
        }
        if (EP24FIFOFLGS & EP2EMPTY)
            IOA &= ~PA_LED2;
        else
            IOA |= PA_LED2;

        if (EP68FIFOFLGS & EP6FULL)
            IOA &= ~PA_LED1;
        else
            IOA |= PA_LED1;
    }
}

/**
 * Handle DEBUG descriptor reporting here.
 * GET_DESCRIPTOR calls for other descriptor types are passed on
 * to _handle_get_descriptor() in setupdat.
 */
BOOL handle_get_descriptor()
{
    BYTE desc_type = SETUPDAT[3]; // wValueH [TRM 2.3.4.1]

    printf("handle_get_descriptor(DT=%d)\n", desc_type);
    if (desc_type != DSCR_DEBUG_TYPE)
        return FALSE;

    // prepare to send the device debug descriptor [TRM 2.3.4]
    SUDPTRH = MSB((WORD)&debug_dscr);
    SUDPTRL = LSB((WORD)&debug_dscr); // load SUDPTRL last to initiate transfer

    return TRUE;
}

// this firmware does not implement any vendor commands
BOOL handle_vendorcommand(BYTE cmd)
{
    printf("handle_vendorcommand(%d)\n", cmd);

    return FALSE;
}

// this firmware only supports 0,0
BOOL handle_get_interface(BYTE *ifc, BYTE *alt_ifc)
{
    printf("handle_get_interface()\n");

    *ifc=0;
    *alt_ifc=0;
    return TRUE;
}
// this firmware only supports 0,0
BOOL handle_set_interface(BYTE ifc, BYTE alt_ifc)
{
    printf("handle_set_interface(%d, %d)\n", ifc, alt_ifc);

    return ((ifc == 0) && (alt_ifc == 0));
}

// this firmware only supports configuration '1'
BYTE handle_get_configuration()
{
    printf("handle_get_configuration()\n");

    return 1;
}
// this firmware only supports configuration '1'
BOOL handle_set_configuration(BYTE cfg)
{
    printf("handle_set_configuration(%d)\n", cfg);

    EP6AUTOINLENH = 0x00; SYNCDELAY;
    EP6AUTOINLENL = 0x08; SYNCDELAY;
    return (cfg == 1);
}

// copied usb jt routines from usbjt.h
void sudav_isr() __interrupt SUDAV_ISR
{
    got_sud = TRUE;
    CLEAR_SUDAV();
}

void sof_isr () __interrupt SOF_ISR __using 1
{
    CLEAR_SOF();
}

void usbreset_isr() __interrupt USBRESET_ISR
{
    handle_hispeed(FALSE);
    CLEAR_USBRESET();
}

void hispeed_isr() __interrupt HISPEED_ISR
{
    handle_hispeed(TRUE);
    CLEAR_HISPEED();
}

