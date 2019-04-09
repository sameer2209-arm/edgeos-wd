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
 * x86-ledcontrol.c
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

int SCLK_PIN = 38;
int SDATA_PIN = 37;
char SCLK[100] = "/home/yashgoyal/workspace/deviceOSWD/led_control/test/gpio38_value";
char SDATA[100] = "/home/yashgoyal/workspace/deviceOSWD/led_control/test/gpio37_value";

char SCLK_DIRECTION_PATH[100] = "/home/yashgoyal/workspace/deviceOSWD/led_control/test/gpio38_direction";
char SDATA_DIRECTION_PATH[100] = "/home/yashgoyal/workspace/deviceOSWD/led_control/test/gpio37_direction";

char GPIO_EXPORT_PATH[100] = "/home/yashgoyal/workspace/deviceOSWD/led_control/test/gpio_export";

int GPIO_SCLK, GPIO_SDATA;

int global_request_id = 0; //Goes up to 255

typedef struct {
    int id;
    unsigned long R;
    unsigned long G;
    unsigned long B;
} ledColor;

bool isWorking = false;

static pthread_mutex_t eventqueue_mutex = PTHREAD_MUTEX_INITIALIZER;
static std::queue<ledColor *> eventqueue;

const char *ledSocketPath;

static int export_gpio(char *gpio_path, int pin) 
{
    int fd;
    fd = open(gpio_path, O_WRONLY);
    if(fd == -1) {
        LD_ERROR_OUT("Cannot open gpio export path- %s, err %s", gpio_path, strerror(errno));
        return 1;
    }

    char str_pin[3] = "\0";
    snprintf(str_pin, (3*sizeof(char)), "%d", pin);
    if (write(fd, str_pin, 3 * sizeof(char)) != 3*sizeof(char)) {
        LD_ERROR_OUT("Unable to export GPIO pin %d, err- %s", pin, strerror(errno));
        return 2;
    }
    close(fd);
    return 0;
}

static int set_direction_output(char *gpio_direction_path) 
{
    int fd;
    fd = open(gpio_direction_path, O_WRONLY);
    if(fd == -1) {
        LD_ERROR_OUT("Cannot open gpio export path- %s, err %s", gpio_direction_path, strerror(errno));
        return 1;
    }

    if (write(fd, "out", 3 * sizeof(char)) != 3*sizeof(char)) {
        LD_ERROR_OUT("Unable to set direction to out for GPIO %s, err- %s", gpio_direction_path, strerror(errno));
        return 2;
    }
    close(fd);
    return 0;
}

static int write_gpio(int fd, int data) 
{
    char str_data[2] = "\0";
    snprintf(str_data, (2*sizeof(char)), "%d", data);
    if(write(fd, str_data, 2 * sizeof(char)) != (2 * sizeof(char))) {
        LD_ERROR_OUT("Unable to set value %s", strerror(errno));
        return 1;
    } 

    return 0;
}

static int open_gpio(char *gpio_path) 
{
    int fd;
    fd = open(gpio_path, O_WRONLY);
    if(fd == -1) {
        LD_ERROR_OUT("Cannot open gpio path- %s, err- %s", gpio_path, strerror(errno));
        return 1;
    }

    return fd;
}

static int execute_ledcontrol()
{
    if(!isWorking) {
        isWorking = true;
        if(!eventqueue.empty()) {
            ledColor *req;
            req = eventqueue.front();

            LD_PRINTF_OUT("Executing led request %d", req->id);
            
            unsigned long R = req->R;
            unsigned long G = req->G;
            unsigned long B = req->B;

            int ret = 0;

            unsigned long mask = 16;
            unsigned long maskfilter = 0; 
            int N = 0;

            ret = 2;

            for (N = 0; N < 32; N++) {
                write_gpio(GPIO_SCLK, 1);
                write_gpio(GPIO_SCLK, 0);
            }

            write_gpio(GPIO_SDATA, 1);
            write_gpio(GPIO_SCLK, 1);
            write_gpio(GPIO_SCLK, 0);

            mask = 16;
            for (N = 0; N < 5; N++) {
                maskfilter = mask & R;
                if(maskfilter == 0) 
                    write_gpio(GPIO_SDATA, 0);
                else
                    write_gpio(GPIO_SDATA, 1);
                write_gpio(GPIO_SCLK, 1);
                write_gpio(GPIO_SCLK, 0);
                mask >>= 1;
            }
            mask = 16;
            for (N = 0; N < 5; N++) {
                maskfilter = mask & B;
                if(maskfilter == 0) 
                    write_gpio(GPIO_SDATA, 0);
                else
                    write_gpio(GPIO_SDATA, 1);
                write_gpio(GPIO_SCLK, 1);
                write_gpio(GPIO_SCLK, 0);
                mask >>= 1;
            }
            mask = 16;
            for (N = 0; N < 5; N++) {
                maskfilter = mask & G;
                if(maskfilter == 0) 
                    write_gpio(GPIO_SDATA, 0);
                else
                    write_gpio(GPIO_SDATA, 1);
                write_gpio(GPIO_SCLK, 1);
                write_gpio(GPIO_SCLK, 0);
                mask >>= 1;
            }
            write_gpio(GPIO_SDATA, 0);
            for (N = 0; N < 2; N++) {
                write_gpio(GPIO_SCLK, 1);
                write_gpio(GPIO_SCLK, 0);
            }

            eventqueue.pop();
            isWorking = false;
            //Check if there is something in the queue, if yes then execute it
            if(!eventqueue.empty()) {
                execute_ledcontrol();
            }
        }
    } else {
        printf("LED is busy, request will be executed is sometime...");
    }

    return 0;
}
/*
int push_ledcontrol_request(unsigned long R, unsigned long G, unsigned long B)  
{                                                                               
}                                                                               
                                                                              
void push_to_ATtiny(unsigned int mode,unsigned long submode,unsigned int power1,
{                                                                               
}                                                                               
                                                                                
int push_ledcontrol_request(unsigned long mode, unsigned submode, unsigned long 
{                                                                               
                                                                                
}                                                                               
int push_pwm_led_control_request(unsigned long mode, unsigned submode, unsigned 
{                                                                               
} 
*/

int push_ledcontrol_request(unsigned long mode, unsigned long submode, unsigned long R, unsigned long G, unsigned long B)
{
    LD_PRINTF_OUT("Pushing led request with color- R: %d, G: %d, B: %d", R, G, B);
    ledColor *req = new ledColor();
    req->R = R;
    req->G = G;
    req->B = B;
    req->id = global_request_id++ & 0xFF;

    pthread_mutex_lock(&eventqueue_mutex);
    eventqueue.push(req);
    pthread_mutex_unlock(&eventqueue_mutex);

    execute_ledcontrol();
}

static int test_leds() {
    push_ledcontrol_request('L', 'S', 1, 0, 0); //R
    push_ledcontrol_request('L', 'S', 0, 1, 0); //G
    push_ledcontrol_request('L', 'S', 0, 0, 1); //B
}

int watchdog_enable() {}
int watchdog_disable() {}
int watchdog_heartbeat() {}
int watchdog_expirenow() {}
int mgmtcontrol_getInfo(){}

/*void push_to_ATtiny(unsigned int mode,unsigned long submode,unsigned int power1,unsigned int power2, unsigned long time1)
{
}*/

int push_pwm_led_control_request(unsigned long mode, unsigned submode, unsigned long Rs, unsigned long Gs, unsigned long Bs,unsigned long Re, unsigned long Ge, unsigned long Be, unsigned long time)                   
{
}
int init_ledcontrol(char *path) 
{
}
void watchdog_set(unsigned long t)
{
}
void watchdog_timeremaining()
{
}

void push_to_ATtiny_piezo(uint8_t mode,unsigned long submode, uint16_t freq, uint32_t time1){}
void PROCESS_THREAD(char *p){}

void getRing(int tone_num){}

int init_ledcontrol(char *path) {
    LD_PRINTF_OUT("Initializing LED GPIOs...");
    // ledSocketPath = local_strdup_safe(path);
    if((export_gpio(GPIO_EXPORT_PATH, SCLK_PIN) == 0) && (export_gpio(GPIO_EXPORT_PATH, SDATA_PIN) == 0)) {
        if((set_direction_output(SCLK_DIRECTION_PATH) == 0) && (set_direction_output(SDATA_DIRECTION_PATH) == 0)) {
            GPIO_SCLK = open_gpio(SCLK);
            GPIO_SDATA = open_gpio(SDATA);
            LD_PRINTF_OUT("LED GPIOs setup completed successfully");
            test_leds();
            return 0;
        } else {
            LD_ERROR_OUT("Failed to set gpio direction as output");
            return 2;
        }
    } else {
        LD_ERROR_OUT("Failed to export pins");
        return 1;
    }
}
