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
 * watchdoglib.h
 *
 *  Created on: Feb 16, 2017
 *      Author: ed
 */

#ifndef WATCHDOGLIB_H_
#define WATCHDOGLIB_H_

#define WATCHDOG_OK 0
#define WATCHDOG_ERROR 1

#include <syslog.h>
#include <uv.h>
#include "error-common.h"

extern int isDaemon;

#ifdef ERRCMN_DEBUG_BUILD
#define WD_DBG_OUT(s,...) if(isDaemon) { syslog(LOG_DEBUG, "DEBUG: " s, ##__VA_ARGS__ ); } else { DBG_OUT(s,  ##__VA_ARGS__ ); }
#define WD_PRINTF_OUT(s,...) if(isDaemon) { syslog(LOG_DEBUG, "INFO: " s, ##__VA_ARGS__ ); } else { printf(s "\n",  ##__VA_ARGS__ ); }
#define WD_ERROR_OUT(s,...) if(isDaemon) { syslog(LOG_ERR, "ERROR: " s, ##__VA_ARGS__ ); } else { ERROR_OUT(s,  ##__VA_ARGS__ ); }
#else
#define WD_DBG_OUT(s,...) if(isDaemon) { }
#define WD_PRINTF_OUT(s,...) if(isDaemon) { }
#define WD_ERROR_OUT(s,...) if(isDaemon) { syslog(LOG_ERR, "ERROR: " s, ##__VA_ARGS__ ); } else { ERROR_OUT(s,  ##__VA_ARGS__ ); }
#endif

#define CMD_GET_WD_STATUS 0x0001

/**
 * Specs:
 * Return WATCHDOG_OK on all success, or a postive number otherwise
 * Functions must be thread-safe
 *
 */


#ifdef __cplusplus
extern "C" {
#endif
// Start the watchdog. If you need a libuv loop, there it is
extern int watchdog_init(uv_loop_t *loop);

// starts watch dog, if no keepalive call is made in wait_time, it should reboot
// wait_time is a best effort. Not all hardware can support it
extern int watchdog_start(int wait_time, int *actual_wait);

extern int watchdog_stop();

#define WD_RESPONSE_SENT 0x1
#define WD_RESPONSE_FAILED 0x2

#define WD_UNAVAILABLE 0x3
#define WD_BAD_RESPONSE 0x4

typedef int (*watchdog_response_failure_cb)(int errnum);
typedef int (*watchdog_status_response_cb)(char *buf, int size);

// gets watchdog status, and sends it via the succeed_cb, or, if failure, calls fail_cb
// **NOTE** the implementation should be reentrant safe
//extern int watchdog_status(watchdog_status_response_cb succeed_cb, watchdog_response_failure_cb fail_cb);

extern int watchdog_status(char **buf, int *len);


// tells the watchdog to stay alive, and resets the interval (usually the same as actual_wait) from
// the watchdog_start() call. The function will fill in actual_wait with the amount of time until next
// reboot
extern int watchdog_keepalive(int *actual_wait);

#ifdef __cplusplus
}
#endif


#endif /* WATCHDOGLIB_H_ */
