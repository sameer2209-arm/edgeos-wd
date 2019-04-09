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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "./../../watchdoglib.h"
#include "./../../led_control/ledlib.h"

extern int isDaemon = 0;

int main(int argv, char** args) {
    int testvar;
    testvar = watchdog_enable();
    WD_PRINTF_OUT("Testing LED and watchdog \n");
    if(!testvar) {
        WD_PRINTF_OUT("Watchdog starting test successfull \n");
    }
    else {
        WD_ERROR_OUT("Error starting watchdog %d\n", testvar);
        return testvar;
    }
    testvar = watchdog_set(10);
    if(!testvar) {
        WD_PRINTF_OUT("Watchdog set test successfull \n");
    }
    else {
        WD_ERROR_OUT("Watchdog set test failed %d\n",testvar);
        return testvar;
    }
    testvar = watchdog_timeremaining();
    if(!testvar) {
        WD_PRINTF_OUT("Watchdog time remaining test successfull\n");
    }
    else {
        WD_ERROR_OUT("Watchdog time remaining test failed %d\n",testvar);
        return testvar;
    }
    testvar = watchdog_heartbeat();
    if(!testvar) {
        WD_PRINTF_OUT("Watchdog Keep alive test successfull\n");
    }
    else {
        WD_ERROR_OUT("Watchdog Keep alive test fail %d\n",testvar);
   	 return testvar;
    }
    testvar = watchdog_disable();
    if(!testvar){
        WD_PRINTF_OUT("Watchdog disable test successfull \n");
    }
    else {
        WD_ERROR_OUT("Watchdog disable test failed %d", testvar);
        return testvar;
    }
    testvar=push_ledcontrol_request('L','b',10,0,0,1000);
    if(!testvar){
        WD_PRINTF_OUT("Red LED test successfull\n");
    }
    else {
        WD_ERROR_OUT("Red LED test failed %d\n",testvar);
	return testvar;
    }

    testvar=push_ledcontrol_request('L','b',0,10,0,1000);
    if(!testvar){
        WD_PRINTF_OUT("Green LED test successfull\n");
    }
    else {
        WD_ERROR_OUT("Green LED test failed %d\n", testvar);
        return testvar;
    }
    testvar=push_ledcontrol_request('L','b',0,0,0,1000);
    WD_PRINTF_OUT("Turning all leds off\n");
    if(!testvar){
        WD_PRINTF_OUT("Turning all leds off test sucessfull\n");
    }
    else {
        WD_ERROR_OUT("All LED off test failed %d\n", testvar);
        return testvar;
    }


}