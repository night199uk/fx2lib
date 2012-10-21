/*
 * Copyright (C) 2012 Sven Schnelle <svens@stackframe.org>
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
#include <serial.h>
#include <delay.h>
#include <autovector.h>
#include <lights.h>
#include <setupdat.h>
#include <eputils.h>

#define SYNCDELAY SYNCDELAY4
#define REARMVAL 0x80
#define REARM() EP2BCL=REARMVAL

volatile __bit got_sud;
extern __code WORD debug_dscr;
static __bit master_device;

#define DATA_MISO	IOB
#define DATA_MOSI	IOD

#define DVALID_MISO	(1 << 0)
#define DLAST_MISO	(1 << 1)
#define DRDY_MISO	(1 << 2)
#define MASTER_SELECT	(1 << 3)
#define DVALID_MOSI	(1 << 4)
#define DLAST_MOSI	(1 << 5)
#define DRDY_MOSI	(1 << 6)


typedef enum {
	TX_STATE_IDLE=0,
	TX_STATE_DATA,
	TX_STATE_ACK,
} tx_state_t;

typedef enum {
	RX_STATE_IDLE=0,
	RX_STATE_DATA,
	RX_STATE_ACK,
} rx_state_t;

static void tx_state_master(void)
{
	static int offset, last;
	static tx_state_t state = TX_STATE_IDLE;

	switch(state) {
		case TX_STATE_IDLE:
			if (EP2468STAT & bmEP6FULL)
				break;
			IOA |= DRDY_MOSI;

			if(!(IOA & DVALID_MISO))
				break;
			state = TX_STATE_DATA;
			/* intentional fall through */

		case TX_STATE_DATA:
			last = IOA & DLAST_MISO;
			EP6FIFOBUF[offset] = DATA_MISO;
			IOA &= ~DRDY_MOSI;
			state = TX_STATE_ACK;
			/* intentional fall through */

		case TX_STATE_ACK:
			if (IOA & DVALID_MISO)
				break;
			offset++;
			if (last) {
				EP6BCH = MSB(offset);
				SYNCDELAY;
				EP6BCL = LSB(offset);
				state = TX_STATE_IDLE;
				offset = 0;
				break;
			}
			state = TX_STATE_IDLE;;
			break;
		default:
			state = TX_STATE_IDLE;
	}
}

static void rx_state_master(void)
{
	static int offset, remaining;
	static rx_state_t state = RX_STATE_IDLE;

	switch(state) {
		case RX_STATE_IDLE:
			if (EP2468STAT & bmEP2EMPTY)
				break;

			state = RX_STATE_DATA;
			offset = 0;
			remaining = MAKEWORD(EP2BCH, EP2BCL);
			break;

		case RX_STATE_DATA:
			if (!(IOA & DRDY_MISO))
				break;

			DATA_MOSI = EP2FIFOBUF[offset++];

			if (--remaining)
				IOA &= ~DLAST_MOSI;
			else
				IOA |= DLAST_MOSI;

			IOA |= DVALID_MOSI;

			state = RX_STATE_ACK;
			break;

		case RX_STATE_ACK:
			if (IOA & DRDY_MISO)
				break;

			IOA &= ~DVALID_MOSI;

			if (remaining) {
				state = RX_STATE_DATA;
			} else {
				state = RX_STATE_IDLE;
				REARM();
			}
			break;
		default:
			state = RX_STATE_IDLE;
	}
}

static void tx_state_slave(void)
{
	static int offset, last;
	static tx_state_t state = TX_STATE_IDLE;

	switch(state) {
		case TX_STATE_IDLE:
			if (EP2468STAT & bmEP6FULL)
				break;
			IOA |= DRDY_MISO;

			if(!(IOA & DVALID_MOSI))
				break;
			state = TX_STATE_DATA;
			/* intentional fall through */

		case TX_STATE_DATA:
			last = IOA & DLAST_MOSI;
			EP6FIFOBUF[offset] = DATA_MOSI;
			IOA &= ~DRDY_MISO;
			state = TX_STATE_ACK;
			/* intentional fall through */

		case TX_STATE_ACK:
			if (IOA & DVALID_MOSI)
				break;
			offset++;
			if (last) {
				EP6BCH = MSB(offset);
				SYNCDELAY();
				EP6BCL = LSB(offset);
				state = TX_STATE_IDLE;
				offset = 0;
				break;
			}
			state = TX_STATE_IDLE;;
			break;
		default:
			state = TX_STATE_IDLE;
	}
}

static void rx_state_slave(void)
{
	static int offset, remaining;
	static rx_state_t state = RX_STATE_IDLE;

	switch(state) {
		case RX_STATE_IDLE:
			if (EP2468STAT & bmEP2EMPTY)
				break;

			state = RX_STATE_DATA;
			offset = 0;
			remaining = MAKEWORD(EP2BCH, EP2BCL);
			break;

		case RX_STATE_DATA:
			if (!(IOA & DRDY_MOSI))
				break;

			DATA_MISO = EP2FIFOBUF[offset++];

			if (--remaining)
				IOA &= ~DLAST_MISO;
			else
				IOA |= DLAST_MISO;

			IOA |= DVALID_MISO;

			state = RX_STATE_ACK;
			break;

		case RX_STATE_ACK:
			if (IOA & DRDY_MOSI)
				break;

			IOA &= ~DVALID_MISO;

			if (remaining) {
				state = RX_STATE_DATA;
			} else {
				state = RX_STATE_IDLE;
				REARM();
			}
			break;
		default:
			state = RX_STATE_IDLE;
	}
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
		} else {
			tx_state_slave();
			rx_state_slave();
		}
	}
}

static void clock_setup(void)
{
	SETCPUFREQ(CLK_48M);
	SETIF48MHZ();
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

	/* BULK IN endpoint EP2 */
	EP2CFG = 0xA2; // 10100010
	SYNCDELAY;

	/* BULK OUT endpoint EP6 */
	EP6CFG = 0xE2;
	SYNCDELAY;

	/* disable all other endpoints */

	EP1OUTCFG &= ~bmVALID;
	SYNCDELAY;

	EP4CFG &= ~bmVALID;
	SYNCDELAY;

	EP8CFG &= ~bmVALID;
	SYNCDELAY;

	// arm ep2
	EP2BCL = 0x80; // write once
	SYNCDELAY;
	EP2BCL = 0x80; // do it again
}

static void port_setup(void)
{
	IOA = 0;
	IOB = 0xff;
	IOD = 0xff;

	if (master_device) {
		OEA = (DVALID_MOSI | DLAST_MOSI | DRDY_MOSI);
		OEB = 0;	/* DATA_MISO */
		OED = 0xff;	/* DATA_MOSI */
	} else {
		OEA = (DVALID_MISO | DLAST_MISO | DRDY_MISO);
		OEB = 0xff;	/* DATA_MISO */
		OED = 0;	/* DATA_MOSI */
	}
}

void main()
{
	REVCTL=0; // not using advanced endpoint controls

	master_device = (IOA & MASTER_SELECT);

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
	if (ifc==0&&alt_ifc==0) {
		// SEE TRM 2.3.7
		// reset toggles
		RESETTOGGLE(0x02);
		RESETTOGGLE(0x86);
		// restore endpoints to default condition
		RESETFIFO(0x02);
		EP2BCL=0x80;
		SYNCDELAY;
		EP2BCL=0X80;
		SYNCDELAY;
		RESETFIFO(0x86);
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
