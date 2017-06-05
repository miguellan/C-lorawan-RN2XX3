/* MIT License
//
// Copyright (c) 2017 Beach Cities Software, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// ___________________     _________       _____  __                                 
// \______   \_   ___ \   /   _____/ _____/ ____\/  |___  _  _______ _______   ____  
//  |    |  _/    \  \/   \_____  \ /  _ \   __\\   __\ \/ \/ /\__  \\_  __ \_/ __ \ 
//  |    |   \     \____  /        (  <_> )  |   |  |  \     /  / __ \|  | \/\  ___/ 
//  |______  /\______  / /_______  /\____/|__|   |__|   \/\_/  (____  /__|    \___  >
//         \/        \/          \/                                 \/            \/ 
*/


// RN2XX3.c
// C library for access RN2483 and RN2903 modules over UART on a RPi or other *nix
// TODO: finish building out command functions
// based on command ref: http://ww1.microchip.com/downloads/en/DeviceDoc/40001811A.pdf

// Lora wan MPIDE plugin with C-code generation which is useful for structs etc used by chip internally
// http://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en589115
// blog post to work with MPE IDX and lorawan (requires older versions than are current at microchip.com)
// https://www.thethingsnetwork.org/forum/t/microchip-released-rn2483-library-code/4439/25

#include "RN2XX3.h"

#include <stdlib.h>         // free
#include <stdio.h>          // open, printf, vprintf
#include <unistd.h>         // open on some *nix
#include <fcntl.h>          // open on some *nix
#include <termios.h>        // termios, tcgetattr, tcflush, tcsetattr

#include <string.h>         // strncpy, strlen
#include <strings.h>        // strcasecmp
#include <sys/time.h>       // gettimeofday
#include <sys/select.h>     // select

#include <stdint.h>         // uint64_t
#include <errno.h>          // errno


/*==================
// DEFINES/STATIC
//================*/

#define _BCDBG // debug flag: comment to silence logging


#ifdef _BCDBG
//#define _BCVBS
#include <inttypes.h>       // PRId64
#include <stdarg.h>         // va_list, va_start, va_end
#endif

// can be used to bring chip from sleep sooner
// used to tell UART to auto-detect baud rate if needed
#define AUTO_BAUD_RATE_DETECT_FLAG 0x55

#define TX_BUFFER_SZ 76
#define RX_BUFFER_SZ 256
#define TS_EPOCH 1262304000000 // Jan 1 2010 00:00:00

#define UART_RX_TIMEOUT_MS 500
#define UART_TX_TIMEOUT_MIN 100
#define UART_RX_SLEEP_MS 100

#ifndef NUL
#define NUL '\0'
#endif

#ifndef RN2XX3_UART_PORT
#define RN2XX3_UART_PORT   "/dev/ttyAMA0"
#endif

static int uart0 = 0;
static int uartRxTimeout = UART_RX_TIMEOUT_MS;
static unsigned char tx_buffer[TX_BUFFER_SZ] = {};
static unsigned char rx_buffer[RX_BUFFER_SZ] = {};
static unsigned char **rx_multi_lines = NULL;

typedef enum 
{
    NO_DATA = 0,
    RX_ONE_LINE,
    RX_MUTLI_LINE,
    RX_FAILURE = 21
} RxDataResponse_t;

typedef uint64_t Timestamp_t;

/*==================
// FORWARD DECLARES
//================*/

static void DLog(const char *format, ...);
static bool uart_send();
static RxDataResponse_t uart_receive();
static bool valid_response();

static Timestamp_t getTimestamp();
static Timestamp_t getUnixTime();
static void sleepFor(const int ms);


/*==================
// PUBLIC IFACE
//================*/

void RN2XX3_Init() {
    // uart0 will be gt 0 if opened
    uart0 = open(RN2XX3_UART_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart0 > 0) {
        // Turn off blocking for reads, use (fd, F_SETFL, FNDELAY) if you want that
        //fcntl(uart0, F_SETFL, 0);
        //CONFIGURE THE UART
        // http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
        //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
        //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
        //	CSIZE:- CS5, CS6, CS7, CS8
        //	CLOCAL - Ignore modem status lines
        //	CREAD - Enable receiver
        //	IGNPAR = Ignore characters with parity errors
        //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
        //	PARENB - Parity enable
        //	PARODD - Odd parity (else even)
        struct termios options;
        tcgetattr(uart0, &options);

        // RN2XX3 modules use 57600, 8-bit, no parity
        options.c_cflag = B57600 | CS8 | CREAD | CLOCAL; 
        options.c_iflag = IGNPAR; // no parity
        options.c_oflag = OCRNL; // RN2XX2 modules use \r\n at the end of everything (not sure this matters though)
        options.c_lflag = 0;
        tcflush(uart0, TCIFLUSH);
        tcsetattr(uart0, TCSANOW, &options);
        DLog("UART INITIALIZED");
    } else {
        uart0 = 0;
        DLog("Error: Failed to initialize - Unable to acccess uart");
    }
}

int RN2XX3_GetSerial() {
    return uart0;
}

void RN2XX3_SetSerialRxTimeout(const int timeoutMs) {
    uartRxTimeout = (timeoutMs < UART_TX_TIMEOUT_MIN) ? UART_TX_TIMEOUT_MIN : timeoutMs;
}

int RN2XX3_Close() {
    int result = -1;
    if (uart0 > 0) {
        result = (close(uart0) < 0) ? 1 : 0;
        if (result > 0) {
            DLog("FAILED TO CLOSE UART");
        } else {
            DLog("CLOSED UART");
        }
        uart0 = 0;
    }
    if (rx_multi_lines) for (char *line = *rx_multi_lines; line; line=*++rx_multi_lines) {
        free(line);
    }
    return result;
}

const char *RN2XX3_ExecCmd(const char *cmd) {
    const char *response = NULL;
    if (uart0 > 0) {
        int cmd_len = strlen(cmd),
            bufsz = sizeof(tx_buffer);

        if (cmd_len > 0 && cmd_len < bufsz) {
            memcpy(&tx_buffer, cmd, cmd_len);
            tx_buffer[cmd_len] = NUL; // assumes <CR><LF> part of cmd
        } else {
            perror("Increase buffer size for commands\n");
            printf("Command: %s", cmd);
        }
    }
    if (strlen(tx_buffer) > 0 && uart_send()) {
        response = &rx_buffer[0];
    }
    return response;
}

const char* RN2XX3_GetSysVersion() {
    if (uart0 > 0) {
        return RN2XX3_ExecCmd("sys get ver\r\n");
    }
    return NULL;
}

bool RN2XX3_SysSleep(const int intervalMs) {
    bool valid = false;
    if (uart0 > 0) {
        char cmd[40] = {};
        snprintf(cmd, 39, "sys sleep %i\r\n", intervalMs); // TODO: move this into ExecCmd
        const char *response = RN2XX3_ExecCmd(cmd);
        valid = valid_response();
    }
    return valid;
}

/*==================
// PRIVATE
//================*/

static void DLog(const char *format, ...) {
#ifdef _BCDBG
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
#else
    do { } while(0);
#endif
}

static bool valid_response() {
    return (0 == strcasecmp("ok\r\n", rx_buffer));
}

// TODO: run tx/rx on bg pthread and use callbacks for data readiness
static bool uart_send() {
    bool success = false;
    DLog("Sending command over uart: %s", tx_buffer);
    if (uart0 > 0 && strlen(tx_buffer) > 0) {
        int tx_len = strlen(tx_buffer),
            bytes_sent = write(uart0, &tx_buffer[0], tx_len);
        success = (tx_len == bytes_sent);
        memset(&rx_buffer, 0, RX_BUFFER_SZ);
        if (bytes_sent <= 0) {
            DLog("UART TX error, failed to send: %s\n", tx_buffer);
        } else switch(uart_receive()) { // listen for response
            case RX_ONE_LINE: {
                success = true;
                break;
            }
            case RX_MUTLI_LINE: // TODO: add support
                break;
            case NO_DATA:
                break;
            case RX_FAILURE:
                break;
        }
    }
    return success;
}

static RxDataResponse_t uart_receive() {
    RxDataResponse_t response = NO_DATA;
    if (uart0 > 0) {
        const int buf_len = RX_BUFFER_SZ - 1;

        // TODO: if sys sleep, then we should sleep as long as the sleep cmd
        Timestamp_t ctime = getTimestamp(),
            etime = (ctime + uartRxTimeout);
        while(ctime < etime) {
            int rx_len = read(uart0, (void *)rx_buffer, buf_len);
            if (rx_len < 0) {
                if (EAGAIN != errno) {
                    DLog("UART RX error");
                }
                response = RX_FAILURE;
            } else if (rx_len > 0) {
                response = RX_ONE_LINE;
                // TODO: add support for multi-line command responses
                rx_buffer[rx_len] = NUL;
                DLog("Received: %i bytes\n%s", rx_len, rx_buffer);
            } else { // 0 == rx_len
                // DLog("Received no response");
                response = NO_DATA; // redundant
            }
            ctime = getTimestamp(); // update timestamp
// #ifdef _BCDBG
            // if (ctime > 0) {
                // char fmt[50] = {};
                // snprintf(fmt, 49, "CURRENT MS: %%%s, TIME LEFT: %%s", PRId64);
                // DLog(fmt, ctime, (etime - ctime));
                // int timeLeft = (int)(etime - ctime);
                // DLog("TIME LEFT: %i", (timeLeft > 0 ? timeLeft : 0));
            //  }
// #endif
            if ((etime-ctime) < 0) break;
            else sleepFor(UART_RX_SLEEP_MS);
        }
    }
    return response;
}

static Timestamp_t getTimestamp() {
    Timestamp_t ts = getUnixTime();
    return (ts - TS_EPOCH);
}

static Timestamp_t getUnixTime() {
    struct timeval tv;
    gettimeofday(&tv, NULL); // TODO: switch out to use monotonic time, this would be off by changing date/time on *nix, while this was running
    Timestamp_t ts = ((Timestamp_t) (tv.tv_sec) * 1000) + ((Timestamp_t) (tv.tv_usec) / 1000);
    return ts;
}

static void sleepFor(const int ms) {
    struct timeval tv = {0};
    tv.tv_sec = 0;
    tv.tv_usec = ms * 1000L;
    select(0, NULL, NULL, NULL, &tv);
}


/*==================
// TEST
//================*/

int main(void) {
    RN2XX3_Init();
    const char *version = RN2XX3_GetSysVersion();
    DLog("SYSTEM VERSION IS: %s", version);
    // TODO: isnt it obvious?
    RN2XX3_SysSleep(250); // sleep with module for 3 seconds
    RN2XX3_Close();
}
