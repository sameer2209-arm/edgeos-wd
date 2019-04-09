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
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>

#include <uv.h>

#include "watchdoglib.h"
#include "error-common.h"

uv_mutex_t WD_lock;
uv_loop_t *myloop;

int enabled = 0;


#define SIMULATED_TIMEOUT 16  // 16 seconds
uv_timer_t simulate_timer;

void timeout_cb(uv_timer_t *timer) {
	WD_ERROR_OUT("**********************!!!!!!! TIMEOUT - HW WILL REBOOT*****************\n");
}

int watchdog_start(int wait_time, int *actual_wait) {



	uv_mutex_lock(&WD_lock);

	WD_DBG_OUT("ENABLE WATCHDOG - 16 seconds");

	*actual_wait = 16;
	enabled = 1;
	uv_mutex_unlock(&WD_lock);

	uv_timer_start(&simulate_timer,timeout_cb,16*1000,16*1000);

	return WATCHDOG_OK;
}

char *RUNNING_STRING="running";
char *STOPPED_STRING="stopped";
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

int watchdog_init(uv_loop_t *loop) {
	myloop = loop;
	uv_mutex_init(&WD_lock);
	uv_timer_init(loop, &simulate_timer);
}

int watchdog_stop() {
	uv_mutex_lock(&WD_lock);
	enabled = 0;
	uv_mutex_unlock(&WD_lock);

	uv_timer_stop(&simulate_timer);

	return WATCHDOG_OK;
}



int watchdog_keepalive(int *actual_wait) {
	uv_mutex_lock(&WD_lock);

	if(enabled) {
		*actual_wait = 16;
		WD_DBG_OUT("HW WATCHDOG -- KEEP ALIVE");
		uv_timer_again(&simulate_timer);
	}
	uv_mutex_unlock(&WD_lock);
}


