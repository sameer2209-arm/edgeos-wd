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
#include "ledlib.h"
#define REDLED "/sys/class/leds/led1/brightness"
#define GREENLED "/sys/class/leds/led0/brightness"
#define HIGH '1'
#define LOW '0'
#include "./../watchdoglib.h"

int RedLedOn() {
    WD_DBG_OUT("Pushing red led ON\n");
    FILE *fpRed;
    fpRed=fopen(REDLED,"w");
    if(fpRed==NULL) {
        return -1;
    }
    putc(HIGH,fpRed);
    fclose(fpRed);
    return 0;

}
int GreenLedOn() {
    WD_DBG_OUT("Pushing green led ON\n");
    FILE *fpGreen;
    fpGreen=fopen(GREENLED,"w");
    if(fpGreen==NULL){
        return -1;
    }
    putc(HIGH,fpGreen);
    fclose(fpGreen);
    return 0;
}

int RedLedOff() {
    WD_DBG_OUT("Pushing red led OFF\n");
    FILE *fpRed;
    fpRed=fopen(REDLED,"w");
    if(fpRed==NULL) {
        return -1;
    }
    putc(LOW,fpRed);
    fclose(fpRed);
    return 0;
}
int GreenLedOff() {
    WD_DBG_OUT("Pushing green led OFF\n");
    FILE *fpGreen;
    fpGreen=fopen(GREENLED,"w");
    if(fpGreen == NULL){
        return -1;
    }
    putc(LOW,fpGreen);
    fclose(fpGreen);
    return 0;
}


int push_ledcontrol_request(unsigned long mode, unsigned long submode, unsigned long R, unsigned long G, unsigned long B,unsigned long t) {
	int err;
    if(mode == 'L') {
        if(R > 0) {
            err=RedLedOn();
        }
        if(G > 0 || B > 0) {
            err=GreenLedOn(); 
        }

        if(R <= 0) {
            err=RedLedOff();
        }
        if(G <= 0 && B <= 0) {
            err=GreenLedOff();
        }
    }
    return err;
}


int push_pwm_led_control_request(unsigned long mode, unsigned long submode, unsigned long Rs, unsigned long Gs, unsigned long Bs,unsigned long Re, unsigned long Ge, unsigned long Be, unsigned long time){
    if(mode == 'L') {
        if(Rs > 0) {
           return RedLedOn();
        }
        if(Gs > 0 || Bs > 0) {
            return GreenLedOn();
        }

        if(Rs <= 0) {
            return RedLedOff();
        }
        if(Gs <= 0 && Bs <= 0) {
            return GreenLedOff();
        }
    }
}

void *play(void *threadid) {}

void getRing(int tone_num) {}

int init_ledcontrol(char *path) {}

int mgmtcontrol_getInfo(void){}
