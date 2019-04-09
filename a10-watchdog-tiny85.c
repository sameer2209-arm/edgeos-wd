/*
 * Copyright (c) 2018, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * a10-watchdog-tiny85.c
 *
 *  This is a variation of the A10 watchdog for hardware with the tiny85
 *  board with power cutoff watchdog. It does everything that the a10-watchdog
 *  does but also sends keepalives to the tiny85 board. If the kernel freezes
 *  up on reboot on the A10 (known issue with A10) then the tiny85 board will
 *  kick in and power cycle the A10 for us.
 *
 *  Created on: Feb 16, 2017
 *      Author: ed
 */

// implements everything in watchdoglib.h

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>

// for serial port stuff
#include <termios.h>

#include <uv.h>

#include "watchdoglib.h"
#include "error-common.h"

uv_mutex_t WD_lock;


#define TINY85_SPEED B115200

// tiny85 stuff
//const char *ENABLE_TINY85 = "^we:$";
//#define ENABLE_TINY85_LEN 5
const char *RETRIGGER_TINY85 = "^wer,$";
#define RETRIGGER_TINY85_LEN 6
const char *DISABLE_TINY85 = "^wd9$";
#define DISABLE_TINY85_LEN 5
const char *GET_TIME_LEFT_TINY85="^wet.$";
#define GET_TIME_LEFT_TINY85_LEN 5

const char *RETRIGGER_TINY85_RESPONSE = "^werA$";
#define RETRIGGER_TINY85_RESPONSE_LEN 6
const char *DISABLE_TINY85_RESPONSE = "^wdA$";
#define DISABLE_TINY85_RESPONSE_LEN 5
const char *GET_TIME_LEFT_TINY85_RESPONSE = "^wetA%d$"; // ^wetA###$
#define GET_TIME_LEFT_TINY85_RESPONSE_LEN  9


const char *TINY85_DEFAULT_PORT="/dev/ttyS1";
int tiny85_fd = 0;
// tiny85 serial functions

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
			WD_ERROR_OUT ("error %d from tggetattr\n", errno);
			return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        	WD_ERROR_OUT ("error %d setting term attributes\n", errno);
}

int set_interface_attribs (int fd, int speed, int parity)
{
	// http://slackware.cs.utah.edu/pub/slackware/slackware-7.0/docs/Linux-mini-HOWTO/Serial-Port-Programming
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        	WD_ERROR_OUT ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);   // shut off parity
        tty.c_cflag |= parity;               // set 1 above if you want parity (usually "none" - as in 8-N-1)
        tty.c_cflag &= ~CSTOPB;              // 1 (enabled) stop bit please
        tty.c_cflag &= ~CRTSCTS;             // disabled hardware flow control

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
        	WD_ERROR_OUT("serial port set_interface_attribs(): error %d from tcsetattr\n", errno);
            return -1;
        }
        return 0;
}

// used to protect writing to the port
uv_mutex_t port_write_mutex;
uv_mutex_t port_read_mutex;


/**
 * Attempts to read buflen into buf from fd, which is a serial port
 * will timeout in maxms milliseconds, on everyt attempt. will stop after
 * 'attempts' are made
 */
int read_serial(int fd, char *buf, int buflen, uint32_t maxms, int attempts) {
	fd_set set;
	struct timeval timeout;

	FD_ZERO(&set); /* clear the set */
    FD_SET(fd, &set); /* add our file descriptor to the set */

    uint32_t sec = maxms / 1000;
	timeout.tv_sec = sec;
	timeout.tv_usec = (maxms-sec)*1000;
	int e = 0;
	int readn = 0;
	int n;

	uv_mutex_lock(&port_read_mutex);
	while(attempts > 0) {
		e = select(fd + 1, &set, NULL, NULL, &timeout);
		if(e == -1) {
		  WD_ERROR_OUT("read_serial: error on select()"); /* an error accured */
		  return 1;
		} else if(e == 0) {
			// timeout on select
			attempts--;
		} else {
			n = read( fd, buf+readn, buflen-readn ); /* there was data to read */
			if(n > 0) {
				readn += n;
				if(readn >= buflen) {
					break;
				}
			} else {
				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					attempts--;
				} else {
					WD_ERROR_OUT("read_serial: error on read() %d\n",errno);
					uv_mutex_unlock(&port_read_mutex);
					return -1;
				}
			}
		}
	}
	uv_mutex_unlock(&port_read_mutex);
	return 0;

}


int open_tiny85_port(const char *portname, int *fd) {

	*fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (*fd < 0)
	{
		WD_ERROR_OUT("TINY85: error %d opening %s: %s\n", errno, portname, strerror (errno));
		return 1;
	}

	set_interface_attribs (*fd, TINY85_SPEED, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (*fd, 0);
	uv_mutex_init(&port_write_mutex);
	uv_mutex_init(&port_read_mutex);
	return 0;
}

//int tiny85_enable() {
//	if (tiny85_fd) {
//		int ret = write(tiny85_fd,ENABLE_TINY85,ENABLE_TINY85_LEN);
//		if (ret >= ENABLE_TINY85_LEN) {
//			WD_ERROR_OUT("write() to tiny85 watchdog failed - %s\n",strerror(errno))
//			return -1;
//		}
//	}
//	return 0;
//}

int tiny85_disable() {
	if (tiny85_fd) {
		uv_mutex_lock(&port_read_mutex);
		int ret = write(tiny85_fd,DISABLE_TINY85,DISABLE_TINY85_LEN);
		uv_mutex_unlock(&port_read_mutex);
		if (ret >= DISABLE_TINY85_LEN) {
			WD_ERROR_OUT("write() to tiny85 watchdog failed - %s\n",strerror(errno))
			return -1;
		} else {
			char buf[DISABLE_TINY85_RESPONSE_LEN+1];
			int len = DISABLE_TINY85_RESPONSE_LEN;
			memset(buf,DISABLE_TINY85_RESPONSE_LEN+1,0);
			if(read_serial(tiny85_fd, buf, len, 500, 3) != 0) {
				WD_ERROR_OUT("Failed to get response from tiny85!");
			} else {
				if (memcmp(buf,DISABLE_TINY85_RESPONSE,DISABLE_TINY85_RESPONSE_LEN) != 0) {
					WD_ERROR_OUT("tiny85 response not expected string, was: %s\n",buf);
				} else {
					WD_PRINTF_OUT("tiny85: got valid response\n");
				}
			}
		}
	}
	return 0;
}

int tiny85_retrigger() {
	if (tiny85_fd) {
		uv_mutex_lock(&port_read_mutex);
		int ret = write(tiny85_fd,RETRIGGER_TINY85,RETRIGGER_TINY85_LEN);
		uv_mutex_unlock(&port_read_mutex);
		if (ret >= RETRIGGER_TINY85_LEN) {
			WD_ERROR_OUT("write() to tiny85 watchdog failed - %s\n",strerror(errno))
			return -1;
		} else {
			char buf[RETRIGGER_TINY85_RESPONSE_LEN+1];
			int len = RETRIGGER_TINY85_RESPONSE_LEN;
			memset(buf,RETRIGGER_TINY85_RESPONSE_LEN+1,0);
			if(read_serial(tiny85_fd, buf, len, 500, 3) != 0) {
				WD_ERROR_OUT("Failed to get response from tiny85!");
			} else {
				if (memcmp(buf,RETRIGGER_TINY85_RESPONSE,RETRIGGER_TINY85_RESPONSE_LEN) != 0) {
					WD_ERROR_OUT("tiny85 response not expected string, was: %s\n",buf);
				} else {
					WD_PRINTF_OUT("tiny85: got valid response\n");
				}
			}
		}
	}
	return 0;
}



// traditional A10 stuff:

#define MAGIC_NUMBER -1

#define TIMER_BASE 0x01c20000


#define TMR_WDT_KEY 0x0a57
//typedef uint32_t u32;

struct sunxi_timer_reg_t {
    uint32_t dummy1[0xc90 / 4];
//    volatile u32 ctrl_reg;
//    volatile u32 mode_reg;
    // Docs: https://linux-sunxi.org/Timers_Controller_Register_guide#TMR_WDT_CTRL
    volatile uint32_t TMR_WDT_CTRL;
    volatile uint32_t TMR_WDT_MODE;
};

typedef volatile struct sunxi_timer_reg_t *sunxi_timer_regs;

sunxi_timer_regs REGS;


int mem_fd = -1;

int enabled = 0;


volatile unsigned *map_physical_memory(uint32_t addr, size_t len)
{
    volatile unsigned *mem;

    if (mem_fd == -1 && (mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
    {
    	WD_ERROR_OUT("Error opening /dev/mem");
    	perror("opening /dev/mem");
        exit(1);
    }

    mem = (volatile unsigned *) mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, (off_t) addr);

    if (mem == MAP_FAILED)
    {
    	WD_ERROR_OUT("Error with mmap() on /dev/mem");
        perror("mmap");
        exit (1);
    }

    return mem;
}



char *RUNNING_STRING="running tiny85:%d";
char *STOPPED_STRING="stopped tiny85:N/A";

//int watchdog_status(watchdog_status_response_cb success_cb, watchdog_response_failure_cb fail_cb) {
//	char temp_status_buf[50];
//	char *buf;
//	int size;
//
//	int seconds_remain = 0;
//
//	if (tiny85_fd) {
//		uv_mutex_lock();
//		int ret = write(tiny85_fd,GET_TIME_LEFT_TINY85,GET_TIME_LEFT_TINY85_LEN);
//		uv_mutex_unlock();
//		if (ret >= RETRIGGER_TINY85_LEN) {
//			WD_ERROR_OUT("write() to tiny85 watchdog failed - %s\n",strerror(errno))
//			return -1;
//		} else {
//			char buf[GET_TIME_LEFT_TINY85_RESPONSE_LEN+1];
//			int len = GET_TIME_LEFT_TINY85_RESPONSE_LEN;
//			memset(buf,GET_TIME_LEFT_TINY85_RESPONSE_LEN+1,0);
//			if(read_serial(tiny85_fd, buf, len, 500, 3) != 0) {
//				WD_ERROR_OUT("Failed to get response from tiny85!");
//				fail_cb(WD_UNAVAILABLE);
//			} else {
//				if(sscanf(buf,GET_TIME_LEFT_TINY85_RESPONSE,&seconds_remain) != 1) {
//					WD_ERROR_OUT("tiny85 response not expected string, was: %s\n",buf);
//					fail_cb(WD_BAD_RESPONSE);
//				} else {
//					WD_PRINTF_OUT("tiny85: got valid response -> %s\n",buf);
//					char *resp = malloc(50);
//					sprintf(resp,RUNNING_STRING,seconds_remain);
//					success_cb(resp,strlen(resp));
//				}
//			}
//		}
//	}
//
//	return 0;
//}

int watchdog_status(char **buf, int *size) {
	int seconds_remain = 0;
	if(!enabled) {
		*size = strlen(STOPPED_STRING)+1;
		*buf = malloc(*size);
		strcpy(*buf,STOPPED_STRING);
	} else
	if (tiny85_fd) {
		uv_mutex_lock(&port_read_mutex);
		int ret = write(tiny85_fd,GET_TIME_LEFT_TINY85,GET_TIME_LEFT_TINY85_LEN);
		uv_mutex_unlock(&port_read_mutex);
		if (ret >= RETRIGGER_TINY85_LEN) {
			WD_ERROR_OUT("write() to tiny85 watchdog failed - %s\n",strerror(errno))
			return -1;
		} else {
			char tempbuf[GET_TIME_LEFT_TINY85_RESPONSE_LEN+1];
			int len = GET_TIME_LEFT_TINY85_RESPONSE_LEN;
			memset(tempbuf,GET_TIME_LEFT_TINY85_RESPONSE_LEN+1,0);
			if(read_serial(tiny85_fd, tempbuf, len, 500, 3) != 0) {
				WD_ERROR_OUT("Failed to get response from tiny85!");
				return WD_UNAVAILABLE;
			} else {
				if(sscanf(tempbuf,GET_TIME_LEFT_TINY85_RESPONSE,&seconds_remain) != 1) {
					WD_ERROR_OUT("tiny85 response not expected string, was: %s\n",tempbuf);
					return WD_BAD_RESPONSE;
				} else {
					WD_PRINTF_OUT("tiny85: got valid response -> %s\n",tempbuf);
					*buf = (char *) malloc(50);
					sprintf((char *) *buf,RUNNING_STRING,seconds_remain);
					*size = strlen(*buf)+1;
				}
			}
		}
	} else {
		return WD_UNAVAILABLE;
	}

	return 0;
}

int watchdog_start(int wait_time, int *actual_wait) {

	uv_mutex_lock(&WD_lock);

	REGS = (sunxi_timer_regs) map_physical_memory(TIMER_BASE, 4096);

	//REGS->TMR_WDT_MODE = (5 << 3) | 3; // enable watch dog, allow it to be reset, set to timeout - max value - 16 seconds  0010 1011 >> 3 0101 =0x05 ! not 0x0b  
	REGS->TMR_WDT_MODE = (11 << 3) | 3;
	REGS->TMR_WDT_CTRL = (TMR_WDT_KEY << 1) | 1; // 0001 0100 1010 1111
	REGS->TMR_WDT_MODE = (11 << 3) | 3;

	*actual_wait = 16;
	enabled = 1;

//	// ok, now let's enable the tiny85
//	if(tiny85_enable()) {
//		WD_ERROR_OUT("FAILED to enable tiny85!!\n");
//	}

	uv_mutex_unlock(&WD_lock);

	return WATCHDOG_OK;
}


int watchdog_init(uv_loop_t *loop) {
	uv_mutex_init(&WD_lock);

	if (!open_tiny85_port(TINY85_DEFAULT_PORT, &tiny85_fd)) {
		WD_ERROR_OUT("Could not connect to tiny85 watchdog!!\n");
		tiny85_fd = 0;
	}
}

int watchdog_stop() {
	uv_mutex_lock(&WD_lock);
	REGS->TMR_WDT_MODE = 0;
	enabled = 0;
	uv_mutex_unlock(&WD_lock);

	if(mem_fd > 0) {
		close(mem_fd);
	}

	if(tiny85_disable()) {
		WD_ERROR_OUT("FAILED to disable tiny85!!\n");
	}

	return WATCHDOG_OK;
}

int watchdog_keepalive(int *actual_wait) {
	uv_mutex_lock(&WD_lock);
	if(enabled) {
		REGS->TMR_WDT_CTRL = (TMR_WDT_KEY << 1) | 1;
		REGS->TMR_WDT_MODE = (11 << 3) | 3;
	}
	*actual_wait = 16;

	if(tiny85_retrigger()) {

	}
	uv_mutex_unlock(&WD_lock);
}


