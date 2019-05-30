/**
 * Copyright (C) 2012 Sven Schnelle <svens@stackframe.rog>
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
#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include <libusb-1.0/libusb.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

void tp_diff(struct timespec *diff, struct timespec *start, struct timespec *end)
{
    if (end->tv_nsec - start->tv_nsec < 0) {
        diff->tv_sec = end->tv_sec - start->tv_sec - 1;
        diff->tv_nsec = 1000000000 + end->tv_nsec - start->tv_nsec;
    } else {
        diff->tv_sec = end->tv_sec - start->tv_sec;
        diff->tv_nsec = end->tv_nsec - start->tv_nsec;
    }
}

static int format_timediff(struct timespec *diff, char *out, int outlen)
{
    if (diff->tv_sec > 1)
        return snprintf(out, outlen, "%d.%02lds", (int)diff->tv_sec,
                        diff->tv_nsec / 10000000);

    if (diff->tv_nsec > 1000000)
        return snprintf(out, outlen, "%ldms", diff->tv_nsec / 1000000);

    if (diff->tv_nsec > 1000)
        return snprintf(out, outlen, "%ldus", diff->tv_nsec / 1000);

    return snprintf(out, outlen, "%ldns", diff->tv_nsec);

}

volatile int exit_terminal = 0;
void handle_sigint(int sig) {
    (void)sig;

    fflush(stdout);
    exit_terminal = 1;
}

static int terminal_loop(libusb_device_handle *hndl, FILE *outfile)
{
    unsigned char buf[512];
    char timebuf[32];

    struct timespec tp, oldtp, diff, lastlinetp, linediff;
    unsigned char c;
    int print_time = 1;
    int len, ret, i;

    if (clock_gettime(CLOCK_MONOTONIC, &oldtp) == -1) {
        fprintf(stderr, "clock_gettime: %s\n", strerror(errno));
        return -1;
    }

    memcpy(&lastlinetp, &oldtp, sizeof(lastlinetp));
    while(!exit_terminal) {
        ret = libusb_bulk_transfer(hndl, 0x86, buf, sizeof(buf), &len, sizeof(buf));

        if (ret == -4) {
            fprintf(stderr, "\nDevice disappeared - will try to reconnect...\n");
            return -1;
        }

        if(ret != -7 && ret != 0) {
            fprintf(stderr, "\nIN Transfer failed: %d\n", ret);
            return -1;
        }

        for (i = 0; i < len; i++) {

            if (print_time) {
                print_time = 0;
                if (clock_gettime(CLOCK_MONOTONIC, &tp) == -1) {
                    fprintf(stderr, "\nclock_gettime: %s\n", strerror(errno));
                    return -1;
                }

                tp_diff(&diff, &oldtp, &tp);
                tp_diff(&linediff, &lastlinetp, &tp);

                format_timediff(&linediff, timebuf, sizeof(timebuf)-1);

                printf("%3d.%03ld [%5s]: ", (int)diff.tv_sec, diff.tv_nsec / 1000000, timebuf);
                fprintf(outfile, "%3d.%03ld [%5s]: ", (int)diff.tv_sec, diff.tv_nsec / 1000000, timebuf);

                memcpy(&lastlinetp, &tp, sizeof(lastlinetp));
            }

            c = buf[i];

            if (c == '\r')
                continue;

            printf ("%c", c);
            print_time = !!(c == '\n');

            if (outfile)
                fputc(buf[i], outfile);
        }
    }
    return 0;
}

int main(int argc __attribute__((unused)),
         char **argv __attribute__((unused)))
{
    FILE *outfile = NULL;
    libusb_context* ctx;
    libusb_device_handle *hndl = NULL;

    libusb_init(&ctx);

    struct sigaction sa;
    sa.sa_handler = handle_sigint;
    sigaction(SIGINT, &sa, NULL);

    printf("Waiting for EHCI Debug Master device...\n");

restart:
    if (exit_terminal) {
        goto out;
    }

    hndl = libusb_open_device_with_vid_pid(ctx, 0x4b4, 0x8619);
    if (!hndl) {
        sleep(1);
        goto restart;
    }

    libusb_detach_kernel_driver(hndl, 1);
    libusb_claim_interface(hndl, 1);

    libusb_set_interface_alt_setting(hndl, 1, 0);

    printf("EHCI Debug Master device opened.\n");

    outfile = fopen("debuglog.txt", "w+");
    if (!outfile) {
        fprintf(stderr, "fopen: debuglog.txt: %s", strerror(errno));
        goto out;
    }

    if (terminal_loop(hndl, outfile) == -1)
        goto restart;

out:
    if (outfile)
        fclose(outfile);
    libusb_close(hndl);
    return 0;
}
