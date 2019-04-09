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
 * ledlib.h
 *
 *  Created on: Jan 13, 2018
 *      Author: yash
 */

#ifndef _LED_H_
#define _LED_H_

#include <syslog.h>
#include <inttypes.h>
#include "./../error-common.h"

extern int isDaemon;

#ifdef ERRCMN_DEBUG_BUILD
#define LD_DBG_OUT(s,...) if(isDaemon) { syslog(LOG_DEBUG, "DEBUG: " s, ##__VA_ARGS__ ); } else { DBG_OUT(s,  ##__VA_ARGS__ ); }
#define LD_PRINTF_OUT(s,...) if(isDaemon) { syslog(LOG_DEBUG, "INFO: " s, ##__VA_ARGS__ ); } else { printf(s "\n",  ##__VA_ARGS__ ); }
#define LD_ERROR_OUT(s,...) if(isDaemon) { syslog(LOG_ERR, "ERROR: " s, ##__VA_ARGS__ ); } else { ERROR_OUT(s,  ##__VA_ARGS__ ); }
#else
#define LD_DBG_OUT(s,...) if(isDaemon) { }
#define LD_PRINTF_OUT(s,...) if(isDaemon) { }
#define LD_ERROR_OUT(s,...) if(isDaemon) { syslog(LOG_ERR, "ERROR: " s, ##__VA_ARGS__ ); } else { ERROR_OUT(s,  ##__VA_ARGS__ ); }
#endif

// #define CMD_GET_WD_STATUS 0x0001

#ifdef __cplusplus
extern "C" {
#endif
extern int init_ledcontrol(char *);

extern void push_to_ATtiny_piezo(uint8_t mode,unsigned long submode, uint16_t freq, uint32_t time1);
void PROCESS_THREAD(char *threadpara); 
void getRing(int tone_num);
extern int  push_pwm_led_control_request(unsigned long mode, unsigned long submode, unsigned long Rs, unsigned long Gs, unsigned long Bs,unsigned long Re, unsigned long Ge, unsigned long Be, unsigned long time);
extern int  push_ledcontrol_request(unsigned long mode, unsigned long submode, unsigned long R, unsigned long G, unsigned long B,unsigned long t);
//extern void push_to_ATtiny(unsigned int mode,unsigned long submode,unsigned int power1,unsigned int power2,unsigned long time1);

extern void getRing(int tone_num);

//extern int push_ledcontrol_request(unsigned long R, unsigned long G, unsigned long B);


extern int mgmtcontrol_getInfo();

//adding watchdog calls here as we are keeping a20 watchdog for now 
extern int watchdog_enable();
extern int watchdog_disable();
extern int watchdog_heartbeat();
extern int watchdog_expirenow();
extern void watchdog_set(unsigned long t);
extern void watchdog_timeremaining();

#ifdef __cplusplus
}
#endif


#endif /* _LED_H_ */
