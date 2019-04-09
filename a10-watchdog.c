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
 * a10-watchdog.c
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

#include <uv.h>

#include "watchdoglib.h"
#include "error-common.h"

uv_mutex_t WD_lock;


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



char *RUNNING_STRING="running";
char *STOPPED_STRING="stopped";
//int watchdog_status(watchdog_status_response_cb succeed_cb, watchdog_response_failure_cb fail_cb) {
//	char *buf;
//	int size;
//	if(enabled) {
//		size = strlen(RUNNING_STRING)+1;
//		buf = malloc(size);
//		strcpy(buf,RUNNING_STRING);
//		succeed_cb(buf,size);
//	} else {
//		size = strlen(STOPPED_STRING)+1;
//		buf = malloc(size);
//		strcpy(buf,STOPPED_STRING);
//		succeed_cb(buf,size);
//	}
//	return 0;
//}

int watchdog_status(char **buf, int *size) {
	if(enabled) {
		*size = strlen(RUNNING_STRING)+1;
		*buf = malloc(*size);
		strcpy(*buf,RUNNING_STRING);
	} else {
		*size = strlen(STOPPED_STRING)+1;
		*buf = malloc(*size);
		strcpy(*buf,STOPPED_STRING);
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
	uv_mutex_unlock(&WD_lock);

	return WATCHDOG_OK;
}


int watchdog_init(uv_loop_t *loop) {
	uv_mutex_init(&WD_lock);
}

int watchdog_stop() {
	uv_mutex_lock(&WD_lock);
	REGS->TMR_WDT_MODE = 0;
	enabled = 0;
	uv_mutex_unlock(&WD_lock);

	if(mem_fd > 0) {
		close(mem_fd);
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
	uv_mutex_unlock(&WD_lock);
}


