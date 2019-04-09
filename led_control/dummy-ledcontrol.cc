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
 * dummy-ledcontrol.c
 *
 *  Created on: Jan 13, 2018
 *      Author: yash
 */

// implements everything in ledlib.h

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <queue>
#include <pthread.h>

#include "ledlib.h"
#include "./../error-common.h"
/*
int push_ledcontrol_request(unsigned long R, unsigned long G, unsigned long B)   
{
}
*/
/*void push_to_ATtiny(unsigned int mode,unsigned long submode,unsigned int power1,unsigned int power2, unsigned long time1)
{
}*/

int push_ledcontrol_request(unsigned long mode, unsigned long submode, unsigned long R, unsigned long G, unsigned long B,unsigned long t)                                                                                      
{

}
int push_pwm_led_control_request(unsigned long mode, unsigned long submode, unsigned long Rs, unsigned long Gs, unsigned long Bs,unsigned long Re, unsigned long Ge, unsigned long Be, unsigned long time)                   
{
}
int init_ledcontrol(char *path) 
{
}

int mgmtcontrol_getInfo(){}

int watchdog_enable() {}
int watchdog_disable() {}
int watchdog_heartbeat() {}
int watchdog_expirenow() {}
void watchdog_set(unsigned long t) {}
void watchdog_timeremaining() {}
void push_to_ATtiny_piezo(uint8_t mode,unsigned long submode, uint16_t freq, uint32_t time1){}
void PROCESS_THREAD(char *p){}

void getRing(int tone_num){}