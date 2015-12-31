/**
 * Copyright (C) 2009 Ubixum, Inc.
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
#include <cstdio>
#include <cassert>
#include <libusb-1.0/libusb.h>
#include <cstring>
#include <unistd.h>

#define BUFSIZE 256

int main(int argc __attribute__((unused)),
	char **argv __attribute__((unused)))
{
	int transferred, rv, i, devmask;
	libusb_context* ctx;
	unsigned char buf[BUFSIZE];
	libusb_device_handle *hndl[2];
	ssize_t devcount;
	struct libusb_device_descriptor desc;

	libusb_device **devlist;
	libusb_init(&ctx);

	devcount = libusb_get_device_list(ctx, &devlist);

    devmask = 0;
    printf("looking through: %d devices\n", devcount);
	for(i = 0; i < devcount; i++) {
		if (libusb_get_device_descriptor(devlist[i], &desc) != 0)
			continue;

		if (desc.idVendor == 0x04b4 && desc.idProduct == 0x8619)
        {
            printf("found %04x:%04x - master device\n", desc.idVendor, desc.idProduct);
			if (!(libusb_open(devlist[i], &hndl[0]))) {
				printf("opened device %d\n", 0);
                devmask |= 1;
			}
		}
		if (desc.idVendor == 0x04b4 && desc.idProduct == 0x8620)
        {
            printf("found %04x:%04x - slave device\n", desc.idVendor, desc.idProduct);
			if (!(libusb_open(devlist[i], &hndl[1]))) {
				printf("opened device %d\n", 1);
                devmask |= 2;
			}
		}
	}

	if (devmask != 0x3) {
		fprintf(stderr, "Need two devices, have %d\n", devmask);
		return 1;
	}

	libusb_claim_interface(hndl[0],1);
	libusb_claim_interface(hndl[1],1);

	libusb_set_interface_alt_setting(hndl[0], 1, 0);
	libusb_set_interface_alt_setting(hndl[1], 1, 0);

	for (i = 0; i < (int)sizeof(buf); i++)
		buf[i] = i;

	printf("OUT transfer to master, endpoint 2\n");
	rv = libusb_bulk_transfer(hndl[0], 0x02, buf, sizeof(buf), &transferred, sizeof(buf));

	if(rv)
		printf("OUT Transfer failed: %d, %s\n", rv, libusb_error_name(rv));

    sleep(1);
    printf("IN transfer from slave, endpoint 6\n");

    rv = 0;
    while (!rv)
    {
    	memset(buf, 0, sizeof(buf));
    	transferred = 0;

       	rv=libusb_bulk_transfer(hndl[1], 0x86, (unsigned char*)buf ,sizeof(buf), &transferred, sizeof(buf));
       	if(rv)
       		printf("IN Transfer failed: %d, %s\n", rv, libusb_error_name(rv));
      
       	printf("received %d bytes:\n", transferred);
    
   	    for (i = 0; i < transferred; i++)
	        printf ("%d: %02x ", i, buf[i]);
      	printf("\n");
    }

	for (i = 0; i < (int)sizeof(buf); i++)
		buf[i] = i;

	printf("OUT transfer to slave, endpoint 2\n");
	rv = libusb_bulk_transfer(hndl[1], 0x02, buf, sizeof(buf), &transferred, sizeof(buf));

	if(rv)
		printf("OUT Transfer failed: %d\n", rv);

	printf("IN transfer from master, endpoint 6\n");

	memset(buf, 0, sizeof(buf));
	transferred = 0;

    rv = 0;
    while (!rv)
    {
    	rv=libusb_bulk_transfer(hndl[0], 0x86, (unsigned char*)buf ,sizeof(buf), &transferred, sizeof(buf));
    	if(rv)
    		printf("IN Transfer failed: %d\n", rv);

    	printf("received %d bytes:\n", transferred);

    	for (i = 0; i < transferred; i++)
	        printf ("%d: %02x ", i, buf[i]);
	    printf("\n");
    }

	libusb_free_device_list(devlist, 1);
	libusb_close(hndl[0]);
	libusb_close(hndl[1]);
	return 0;
}
