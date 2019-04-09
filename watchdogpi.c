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

/* Reference https://embeddedfreak.wordpress.com/2010/08/23/howto-use-linux-watchdog/ */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>
#include <string.h>
#include <errno.h>
#include <linux/watchdog.h>
#include <sys/ioctl.h>
#include "watchdoglib.h"
#include <uv.h>

#define WATCHDOGDEV "/dev/watchdog"
#define Running 1
#define NotRunning 0

uv_mutex_t WD_lock;
char *RUNNING_STRING="running";
char *STOPPED_STRING="stopped";
int enabled = 0;
int status = NotRunning;


int fd=0;
//int isDaemon = 1;
int watchdog_start(int wait_time, int *actual_wait) {
    if(!status) {
        uv_mutex_lock(&WD_lock);
        fd = open(WATCHDOGDEV, O_RDWR);
        if ( fd== -1) {
                WD_ERROR_OUT("Error: %s\n", strerror(errno));
                return errno;
            }
        ioctl(fd,WDIOC_GETTIMELEFT, actual_wait);
        uv_mutex_unlock(&WD_lock);
    }
    else {
        WD_ERROR_OUT("Watchdog is Running\n");
    }
    status=Running;
    return WATCHDOG_OK;
}

int watchdog_init(uv_loop_t *loop) {
	uv_mutex_init(&WD_lock);
}

int watchdog_stop() {
    if(status) {
        uv_mutex_lock(&WD_lock);
        WD_DBG_OUT("Stopping watchdog\n");
        write(fd, "V", 1);
        close(fd);
        uv_mutex_unlock(&WD_lock);
        }
    else {
        WD_ERROR_OUT("Watchdog is not Running\n");
        return -1;
    }
    status=NotRunning;
    return 0;
}

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

int watchdog_heartbeat() {
    int actual_wait=0;
    watchdog_keepalive(&actual_wait);
}

int watchdog_enable() {
    int time_interval=0;
    return watchdog_start(15,&time_interval);
}

int watchdog_disable() {
    return watchdog_stop();
}

int _dog_set(int duration) 
{
   WD_DBG_OUT("Setting Watchdog to duration %d \n", duration);
    if(ioctl(fd, WDIOC_SETTIMEOUT, &duration)!=0) {
        WD_ERROR_OUT("Setting watchdog interval failed %s\n",strerror(errno));
        return errno;
    }
    return 0;
}

int watchdog_set(int duration) {
    _dog_set(duration);
}

int watchdog_keepalive(int *actual_wait) {
	    uv_mutex_lock(&WD_lock);
    	if(ioctl(fd, WDIOC_KEEPALIVE, NULL)!=0) {
            WD_ERROR_OUT("Keep alive watchdog failed %s\n",strerror(errno));
            return errno;
        }
    	WD_DBG_OUT("Keeping watchdog Alive\n");
	    uv_mutex_unlock(&WD_lock);
    	actual_wait=12;
        return 0;
}

int watchdog_expirenow() {
    int interval=2;
    WD_DBG_OUT("Got expire Now Command\n");
    if(ioctl(fd, WDIOC_SETTIMEOUT, &interval)!=0) {
        WD_ERROR_OUT("Error Cannot read status %s\n",strerror(errno));
        return errno;
    }
    return 0;
}
int _dog_timeremaining() 
{
     int interval=0;
     WD_DBG_OUT("Got Time Remaining Command\n");
      if(ioctl(fd, WDIOC_GETTIMELEFT, &interval)!=0) {
        WD_ERROR_OUT("Error Cannot read status %s\n",strerror(errno));
        return errno;
    }
    else {
        WD_DBG_OUT("Time left %d\n", interval);
    }
    return 0;
}

void watchdog_timeremaining()
{
    _dog_timeremaining();
}


/*
 int main(int argc, char **argv) {
    printf("Welcome to Watchdog control for RaspberryPi\n");
        printf("Enter the Following commands\n");
        printf("<e> to enable watchdog\n");
        printf("<d> to disable watchdog\n");
        printf("<t> for time remaining watchdog\n");
        printf("<s> for setting time for watchdog, can't be set more than 15 sec on RPi\n");
        printf("<x> for expire watchdog\n");
        printf("<i> for keepAlive watchdog\n");
        printf("<h> for Heartbeat\n");
        printf("\n------------------------------------------\n");
    while(1) {
        char cmd;
        int duration;
        scanf("%c",&cmd);
        if(cmd == 'e')
        {
            watchdogEnable();
        }
            if(cmd == 'd')
        {
            watchdogDisable();
        }
            if(cmd == 'i')
        {
            watchdogKeepAlive();
        }
        if(cmd == 't')
        {
            watchdogTimeRemaining();
        }
        if(cmd == 'h')
        {
            watchdogHearbeat();
        }
         if(cmd == 'x')
        {
            watchdogExpireNow();
        }
        if (cmd == 's') {
            printf("Enter the duration \n");
            scanf("%d", &duration);
            watchdogSet(duration);
        }
    }
 }*/