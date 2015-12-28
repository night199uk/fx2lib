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
#include <stdio.h>
#include <stdint.h>

#include <fx2regs.h>
#include <fx2macros.h>
#include <delay.h>
#include <autovector.h>
#include <gpif.h>
#include <setupdat.h>
#include <eputils.h>

#define SYNCDELAY SYNCDELAY4

volatile __bit got_sud;
volatile __bit gpif_interrupted;

extern __code WORD debug_dscr;

#define GLEAR_GPIF()    CLEAR_GPIF();

/* PORTA.3 */
#define PA_LED1			    (1 << 0)
#define PA_LED2			    (1 << 1)
#define PA_MASTER_SELECT	(1 << 3)

// PA5/4 (FIFOADR1,0) select the endpoint to communicate with
#define PA_FIFOMASK		(3 << 4)
#define 	PA_FIFO_EP2	(0 << 4)
#define 	PA_FIFO_EP6	(2 << 4)
#define PA_nPKTEND		(1 << 6)

/* RDY0/1 inputs */
#define RDY_nFF	(1<<0)
#define RDY_nEF	(1<<1)

#define EP2EMPTY 0x02
#define EP6FULL  0x01

static const unsigned char idledata[8] = {
/* GPIFREADYCFG */	0xA0,
/* GPIFCTLCFG */	0x10,
/* GPIFIDLECS */	0x00,
/* GPIFIDLECTL */	0x07,
/* unused */		0xee,
// Bits 7-6: SnglWr WF Index = 1
// Bits 5-4: SnglRd WF Index = 0
// Bits 3-2: FIFOWR WF Index = 2
// Bits 1-0: FIFORD WF Index = 3
/* GPIFWFSELECT */	0x4b,
/* GPIFREADYSTAT */	0x00
};

static const unsigned char __xdata waveforms[128] =     
{                                      
// Wave 0 
/* LenBr */ 0x01,     0x01,     0x01,     0x01,     0x01,     0x01,     0x01,     0x07,
/* Opcode*/ 0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,
/* Output*/ 0x07,     0x07,     0x07,     0x07,     0x07,     0x07,     0x07,     0x07,
/* LFun  */ 0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x3F,
// Wave 1 
/* LenBr */ 0x01,     0x01,     0x01,     0x01,     0x01,     0x01,     0x01,     0x07,
/* Opcode*/ 0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,
/* Output*/ 0x07,     0x07,     0x07,     0x07,     0x07,     0x07,     0x07,     0x07,
/* LFun  */ 0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x00,     0x3F,
// Wave 2 
/* LenBr */ 0x29,     0x03,     0x01,     0x01,     0x3F,     0x2D,     0x36,     0x07,
/* Opcode*/ 0x01,     0x02,     0x02,     0x04,     0x01,     0x11,     0x01,     0x00,
/* Output*/ 0x07,     0x05,     0x07,     0x07,     0x07,     0x07,     0x07,     0x07,
/* LFun  */ 0xC7,     0x00,     0x00,     0x00,     0x3F,     0x3F,     0x3F,     0x3F,
// Wave 3 
/* LenBr */ 0x21,     0x03,     0x01,     0x3F,     0x24,     0x2D,     0x01,     0x07,
/* Opcode*/ 0x01,     0x00,     0x02,     0x01,     0x11,     0x01,     0x00,     0x00,
/* Output*/ 0x07,     0x02,     0x02,     0x07,     0x07,     0x07,     0x07,     0x07,
/* LFun  */ 0xCF,     0x00,     0x00,     0x3F,     0x09,     0x3F,     0x00,     0x3F,
};                     

static void tx_state_master(void)
{
    unsigned int count;

    if (!GPIFDONE)
        return;

    // If EP6 full
	if (EP6FIFOFLGS & EP6FULL)
		return;

	// PA5/4 (FIFOADR1,0) select the endpoint to communicate with
	if (!(GPIFREADYSTAT & RDY_nEF))
		return;

	IOA = (IOA & ~PA_FIFOMASK) | PA_FIFO_EP2; SYNCDELAY;

    // Set transaction count to 1
    gpif_set_tc16(512);
    
    // Read from GPIF and send the output to EP6BUF
    gpif_fifo_read(GPIF_EP6);
    do { } while (!GPIFDONE);
    if (gpif_interrupted)
        gpif_interrupted = FALSE;

    count = (EP6FIFOBCH << 8) | EP6FIFOBCL;
    if (count > 0 && (count % 8) != 0)
    	INPKTEND = 0x06; SYNCDELAY;
}

// Transmit from master to slave
static void rx_state_master(void)
{
    uint16_t count;
    uint16_t remaining;
    uint16_t written;

    if (!GPIFDONE)
        return;

    // If EP2EMPTY 
	if (EP24FIFOFLGS & EP2EMPTY)
		return;

    // Bail if the output buffer is full
	if (!(GPIFREADYSTAT & RDY_nFF))
		return;

	IOA = (IOA & ~PA_FIFOMASK) | PA_FIFO_EP6; SYNCDELAY;

	/* Payload received from USB can be modified in EP2BUF[] and
	   has length of EP2BCH/L.
	 */

    count = (EP2FIFOBCH << 8) | EP2FIFOBCL;
    gpif_set_tc16(count);   SYNCDELAY;
    GPIFTRIG = GPIF_EP2;    SYNCDELAY;
	do { } while (!GPIFDONE);

    /* 
     * GPIF took an interrupt after FF was asserted by remote. 
     */
    if (gpif_interrupted)
        gpif_interrupted = FALSE;

    remaining = (EP2FIFOBCH << 8) | EP2FIFOBCL;
    written = count - remaining;

    if ((written % 8) != 0)
    {
        IOA &= ~PA_nPKTEND; SYNCDELAY;
        IOA |=  PA_nPKTEND; SYNCDELAY;
    }
}

static void mainloop(void)
{
	while(TRUE) {
		if (got_sud) {
			handle_setupdata();
			got_sud=FALSE;
		}
		tx_state_master();
		rx_state_master();

//	    if (!(GPIFREADYSTAT & RDY_nEF))
//        {
//            IOA &= ~PA_LED2; SYNCDELAY;
//        }
//        else
//        {
//            IOA |= PA_LED2; SYNCDELAY;
//        }
//
//	     if (!(GPIFREADYSTAT & RDY_nFF))
//       {
//            IOA &= ~PA_LED1; SYNCDELAY;
//       }
//       else
//       {
//           IOA |= PA_LED1; SYNCDELAY;
//       }

            
	}
}

void main()
{
//	SETCPUFREQ(CLK_48M); SYNCDELAY;
//	SETIF48MHZ();
    CPUCS = 0x10;       SYNCDELAY; SYNCDELAY;

    // GpifInit();
    // Bit7: 0 = External, 1 = Internal...1 - Internal
    // Bit6: 0 = 30Mhz Clk, 1 = 48 Mhz Clk...1 - 48Mhz 
    // Bit5: 0 = No ClkOut, 1 = ClkOut...0 - No ClkOut
    // Bit4: 0 = NormalPol, 1 = InvPol...0 = NormalPol
    // Bit3: 0 = Synchronous, 1 = Asynchronous...1 = Async
    // Bit2: 0 = GStateOff, 1 = GStateOn...1 = GState On
    // Bit1-0: 0 = Ports, 1 = Reserved, 2 = GPIF Master, 3 = Slave FIFO...2 = GPIF Master
    IFCONFIG = 0xEE;
    gpif_init(waveforms, idledata);

    // Bit0 = ENH_PKT - Enhanced Packet Handling.
    // Bit1 = DYN_OUT - Disable Auto-Arming at the 0-1 transition of AUTOOUT.
//	REVCTL=0x03; SYNCDELAY;

    RENUMERATE_UNCOND();
	USE_USB_INTS();
	ENABLE_SUDAV();
	ENABLE_SOF();
	ENABLE_HISPEED();
	ENABLE_USBRESET();
    USE_GPIF_INTS();
    ENABLE_GPIFWF();

	/* INT endpoint */
	EP1INCFG = bmVALID | (3 << 4); SYNCDELAY;

	/* BULK OUT endpoint EP2 */
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
    // 10100010 - DIR=OUT, TYPE=BULK, SIZE=512, BUF=QUAD
	EP2CFG = 0xA0;          SYNCDELAY;
	EP4CFG &= ~bmVALID;     SYNCDELAY;
    // 11100010 - DIR=IN, TYPE=BULK, SIZE=512, BUF=QUAD
	EP6CFG = 0xE0;          SYNCDELAY;
	EP8CFG &= ~bmVALID;     SYNCDELAY;

	/* disable all other endpoints */
	EP1OUTCFG &= ~bmVALID;  SYNCDELAY;

    FIFORESET = 0x80;       SYNCDELAY;
    FIFORESET = 0x02;       SYNCDELAY;
    FIFORESET = 0x06;       SYNCDELAY;
    FIFORESET = 0x00;       SYNCDELAY;

	// arm ep2
//    OUTPKTEND = 0x82;       SYNCDELAY;
//    OUTPKTEND = 0x82;       SYNCDELAY;

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


	// Bit0:1 = FS[0:1] - GPIF Flag Select.
    // 00 - Programmable
    // 01 - Empty
    // 10 - Full
    // 11 - Reserved
	EP2GPIFFLGSEL = 0x02; SYNCDELAY; /* EF */
    EP6GPIFFLGSEL = 0x01; SYNCDELAY; /* FF */

    // Bit1:0 = IFCFG0:1 = 0:0 = Ports
    // Bit1:0 = IFCFG0:1 = 1:0 = GPIF Master
    // Bit1:0 = IFCFG0:1 = 1:1 = Slave FIFO
    // Bit2 = GSTATE
    // Bit3 = ASYNC
    // Bit4 = IFCLKPOL
    // Bit5 = IFCLKOE
    // Bit6 = 3048MHZ
    // Bit7 = IFCLKSRC
    PORTACFG = 0x0;
    OEA = (PA_nPKTEND | PA_FIFOMASK | PA_MASTER_SELECT | PA_LED1 | PA_LED2); SYNCDELAY;
    IOA = (PA_nPKTEND | PA_LED1 | PA_LED2); SYNCDELAY;

	EA=1; // global __interrupt enable

	mainloop();
}

#define VC_EPSTAT 0xB1

BOOL handle_vendorcommand(BYTE cmd)
{
	__xdata BYTE* pep;
	switch ( cmd ) {
	case 6:
		return TRUE;
	case VC_EPSTAT:

		pep = ep_addr(SETUPDAT[2]);
		if (pep) {
			EP0BUF[0] = *pep;
			EP0BCH=0;
			EP0BCL=1;
			return TRUE;
		}
	default:
	}
	return FALSE;
}

// this firmware only supports 0,0
BOOL handle_get_interface(BYTE ifc, BYTE *alt_ifc)
{
	if (ifc)
		return FALSE;
	*alt_ifc=0;
	return TRUE;
}

BOOL handle_set_interface(BYTE ifc, BYTE alt_ifc)
{
	if (ifc==1 && alt_ifc==0) {
		// SEE TRM 2.3.7
		// reset toggles
		RESETTOGGLE(0x02);
		RESETTOGGLE(0x86);

		RESETFIFOS();

        EP2FIFOCFG = 0;         SYNCDELAY;
        EP2FIFOCFG = bmAUTOOUT; SYNCDELAY;
		return TRUE;
	} else
		return FALSE;
}

// get/set configuration
BYTE handle_get_configuration()
{
	return 1;
}

BOOL handle_get_descriptor(BYTE desc)
{
	if (desc != DSCR_DEBUG_TYPE)
		return FALSE;

	SUDPTRH = MSB((WORD)&debug_dscr);
	SUDPTRL = LSB((WORD)&debug_dscr);
	return TRUE;
}

BOOL handle_set_configuration(BYTE cfg)
{

    EP6AUTOINLENH = 0x0;    SYNCDELAY;
    EP6AUTOINLENL = 0x8;    SYNCDELAY;
	return cfg==1 ? TRUE : FALSE; // we only handle cfg 1
}

void gpifwf_isr() __interrupt GPIFWF_ISR {
    CLEAR_GPIFWF();
    gpif_interrupted = TRUE;
    GPIFABORT = 0xFF;   SYNCDELAY;
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
