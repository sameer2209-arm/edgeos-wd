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
 * error-common.cc
 *
 *  Created on: Sep 3, 2014
 *      Author: ed
 * (c) 2014, WigWag Inc.
 */



#include "error-common.h"



#include <string.h>
//extern char *strdup(const char *s);

extern "C" char *local_strdup_safe(const char *s);

typedef struct {
	const char *label;
	const int code;
} custom_errno;



custom_errno custom_errs[] = {
//		{ "Port is unresponsive.", SIXLBR_PORT_UNRESPONSIVE }
//		{ "Unknown TTY.", GREASE_UNKNOWN_TTY },
//		{ "Unknown failure.", GREASE_UNKNOWN_FAILURE },
//		{ "Call needs path.", GREASE_UNKNOWN_NO_PATH }
};


	char *get_custom_err_str(int _errno) {
		int n = sizeof(custom_errs);
		while(n > 0) {
			if(custom_errs[n-1].code == _errno) {
				return (char *) custom_errs[n-1].label;
			}
			n--;
		}
		return NULL;
	}


	const int max_error_buf = 255;

	char *_errcmn::get_error_str(int _errno) {
		char *ret = (char *) malloc(max_error_buf);
		int r = ERR_STRERROR_R(_errno,ret,max_error_buf);
		if ( r != 0 ) DBG_OUT("strerror_r bad return: %d\n",r);
		return ret;
	}


	void _errcmn::free_error_str(char *b) {
		free(b);
	}

	void _errcmn::err_ev::setError(int e,const char *m)
	{
		_errno = e;
		if(errstr) free(errstr);
		if(!m) {
//			if(_errno < _ERRCMN_CUSTOM_ERROR_CUTOFF) {
				errstr = get_error_str(_errno);
//			} else {
//				char *custom = get_custom_err_str(_errno);
//				if(custom)
//					errstr = ::local_strdup_safe(custom);
//				else
//				    errstr = ::local_strdup_safe("Unknown Error.");
//			}
		} else
			errstr = ::local_strdup_safe(m);
	}




