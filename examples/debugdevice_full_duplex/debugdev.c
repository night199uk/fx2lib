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

#include <fx2regs.h>
#include <fx2macros.h>
#include <delay.h>
#include <autovector.h>
#include <gpif.h>
#include <setupdat.h>
#include <eputils.h>

#define SYNCDELAY SYNCDELAY4

volatile __bit got_sud;
extern __code WORD debug_dscr;
static __bit master_device;

/* PORTA.3 */
#define PA_nSLOE 		(1 << 2)
#define PA_MASTER_SELECT	(1 << 3)
#define PA_FIFOMASK		(3 << 4)
#define 	PA_FIFO_EP2	(0 << 4)
#define 	PA_FIFO_EP4	(1 << 4)
#define 	PA_FIFO_EP6	(2 << 4)
#define		PA_FIFO_EP8	(3 << 4)
#define PA_nPKTEND	(1 << 6)

/* RDY0/1 inputs */
#define RDY_nEF	(1<<0)
#define RDY_nFF	(1<<1)

static const unsigned char __idledata[8] = {
/* GPIFREADYCFG */	0x0,
/* GPIFCTLCFG */	0x0,
/* GPIFIDLECS */	0x0,
/* GPIFIDLECTL */	0xff,
/* unused */		0x0,
/* GPIFWFSELECT */	0xe4,
/* GPIFREADYSTAT */	0x0
};

static const unsigned char __waveforms[4][4][8] = {
	{
	/* length / branch */	{0x0f, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00},
	/* opcode */		{0x01, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00},
	/* output */		{0x07, 0x06, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00},
	/* logic fn */		{0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	},
	{
	/* length / branch */	{0x01, 0x03, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00},
	/* opcode */		{0x02, 0x02, 0x02, 0x05, 0x00, 0x00, 0x00, 0x00},
	/* output */		{0x07, 0x05, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00},
	/* logic fn */		{0x00, 0x00, 0x00, 0xf1, 0x00, 0x00, 0x00, 0x00},
	},
	{{{0}}},
	{
	/* length / branch */	{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00},
	/* opcode */		{0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	/* output */		{0x07, 0x05, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07},
	/* logic fn */		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	},
};

static unsigned char * idledata = &__idledata[0];
static unsigned char * waveforms = &__waveforms[0][0][0];

static void tx_state_master(void)
{
	if (EP2468STAT & bmEP6FULL)
		return;

	do {} while (!GPIFDONE);

	IOA = (IOA & ~PA_FIFOMASK) | PA_FIFO_EP2;
	SYNCDELAY;
	if (!(GPIFREADYSTAT & RDY_nEF))
		return;

	/* As both SLOE and SLRD signals gate FD[0:15] drivers in slave
	   FIFO asynchronous mode, it should be safe to have SLOE active
	   all the time.
	  */
	IOA &= ~PA_nSLOE;

	gpif_set_tc16(1);
	gpif_fifo_read(GPIF_EP6);

	do {} while (!GPIFDONE);
	IOA |= PA_nSLOE;

	/* Payload to send to USB can be modified in EP6FIFOBUF[] and
	   it has length of EP6FIFOBCH/L.
	 */

	INPKTEND = 0x06;
	SYNCDELAY;
}

static void rx_state_master(void)
{
	if (EP2468STAT & bmEP2EMPTY)
		return;

	do {} while (!GPIFDONE);

	IOA = (IOA & ~PA_FIFOMASK) | PA_FIFO_EP6;
	SYNCDELAY;
	if (!(GPIFREADYSTAT & RDY_nFF))
		return;

	/* As both SLOE and SLRD signals gate FD[0:15] drivers in slave
	   FIFO asynchronous mode, it should be safe to have SLOE active
	   all the time.
	  */
	IOA |= PA_nSLOE;

	/* Payload received from USB can be modified in EP2BUF[] and
	   has length of EP2BCH/L.
	 */

	OUTPKTEND = 0x02;
	SYNCDELAY;
	gpif_set_tc16(1);
	gpif_fifo_write(GPIF_EP2);

	do {} while (!GPIFDONE);

	/* No more data on master enpoint out, flush slave endpoint in. */
	IOA &= ~PA_nPKTEND;
	IOA |= PA_nPKTEND;
}


static void mainloop(void)
{
	__bit master = master_device;
	while(TRUE) {
		if (got_sud) {
			handle_setupdata();
			got_sud=FALSE;
		}
		if (master) {
			tx_state_master();
			rx_state_master();
		}
	}
}

static void clock_setup(void)
{
	SETCPUFREQ(CLK_48M);
	SETIF48MHZ();

	if (master_device)
		IFCONFIG |= bmIFCLKOE;
	else
		IFCONFIG &= ~bmIFCLKSRC;
}

static void usb_setup(void)
{
	RENUMERATE_UNCOND();

	USE_USB_INTS();
	ENABLE_SUDAV();
	ENABLE_SOF();
	ENABLE_HISPEED();
	ENABLE_USBRESET();

	/* INT endpoint */
	EP1INCFG = bmVALID | (3 << 4);
	SYNCDELAY;

	/* BULK OUT endpoint EP2 */
	EP2CFG = 0xA2; // 10100010
	SYNCDELAY;

	/* BULK IN endpoint EP6 */
	EP6CFG = 0xE2;
	SYNCDELAY;

	/* disable all other endpoints */

	EP1OUTCFG &= ~bmVALID;
	SYNCDELAY;

	EP4CFG &= ~bmVALID;
	SYNCDELAY;

	EP8CFG &= ~bmVALID;
	SYNCDELAY;

	/* BULK OUT endpoint EP2 */
	EP2FIFOCFG = 0;
	SYNCDELAY;

	RESETFIFOS();

	// arm ep2
	OUTPKTEND = 0x82;
	SYNCDELAY;
	OUTPKTEND = 0x82;
	SYNCDELAY;

	if (master_device) {
		EP2GPIFFLGSEL = 0x01; /* EF */
		SYNCDELAY;

		/* BULK IN endpoint EP6 */
		EP6FIFOCFG = 0;
		SYNCDELAY;
		EP6GPIFFLGSEL = 0x02; /* FF */
		SYNCDELAY;

	} else {
		EP2FIFOCFG = bmAUTOOUT;
		SYNCDELAY;

		/* BULK IN endpoint EP6 */
		EP6FIFOCFG = bmAUTOIN;
		SYNCDELAY;

		/* 512 bytes */
		EP6AUTOINLENH = 0x2;
		SYNCDELAY;
		EP6AUTOINLENL = 0x0;
		SYNCDELAY;
	}
}

static void port_setup(void)
{
	if (master_device) {
		/* GPIF master */
		gpif_init(waveforms, idledata);
		IFCONFIG = (IFCONFIG & ~0x03) | 0x02;
		PORTACFG = 0x0;
		IOA = (PA_nPKTEND | PA_nSLOE);
		OEA = (PA_nPKTEND | PA_nSLOE | PA_FIFOMASK | PA_MASTER_SELECT);
		FIFOPINPOLAR = 0x0;
	} else {
		/* SLAVE FIFO in asynchronous mode */
		IFCONFIG |= bmASYNC | 0x03;
		PORTACFG = 0x0;
		IOA = 0x0;
		OEA = 0x0;
		FIFOPINPOLAR = 0x0;
		PINFLAGSAB = 0x0;
		PINFLAGSCD = 0x0;
	}

	/* Output drive enable is under GPIF or #SLOE control. */
	OEB = 0x00;
	OED = 0x00;
	IOB = 0xff;
	IOD = 0xff;
}

void main()
{
	__bit last_msel = 0;
	__bit this_msel = 0;
	int i = 0;
	REVCTL=0x03;
	SYNCDELAY;

	OEA = 0;
	master_device = FALSE;
	while (i<5) {
		this_msel = !!(IOA & PA_MASTER_SELECT);
		if (this_msel == last_msel)
			i++;
		else
			i=0;
		last_msel = this_msel;
	}
	if (last_msel) {
		IOA &= ~PA_MASTER_SELECT;
		OEA |= PA_MASTER_SELECT;
		master_device = TRUE;
	}

	/* Delay slave so master has time to drive IFCLK and IOA */
	if (! master_device)
		delay(500);

	port_setup();

	clock_setup();

	usb_setup();

	delay(10);

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

		// restore endpoints to default condition
		if (!master_device) {
			EP2FIFOCFG = 0;
			SYNCDELAY;
		}
		RESETFIFOS();
		OUTPKTEND = 0x82;
		SYNCDELAY;
		OUTPKTEND = 0x82;
		SYNCDELAY;
		if (!master_device) {
			EP2FIFOCFG = bmAUTOOUT;
			SYNCDELAY;
		}
		return TRUE;
	} else
		return FALSE;
}

// get/set configuration
BYTE handle_get_configuration()
{
	return 1;
}

BOOL handle_get_descriptor()
{
	BYTE desc = SETUPDAT[3];
	if (desc != DSCR_DEBUG_TYPE)
		return FALSE;

	SUDPTRH = MSB((WORD)&debug_dscr);
	SUDPTRL = LSB((WORD)&debug_dscr);
	return TRUE;
}

BOOL handle_set_configuration(BYTE cfg)
{

	return cfg==1 ? TRUE : FALSE; // we only handle cfg 1
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
