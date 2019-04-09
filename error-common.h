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
 * error-common.h
 *
 *  Created on: Feb 16, 2017
 *      Author: ed
 */
#ifndef ERROR_COMMON_H_
#define ERROR_COMMON_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <errno.h>
#if !defined(_MSC_VER)
#include <unistd.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif
extern int __xpg_strerror_r (int __errnum, char *__buf, size_t __buflen);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

namespace _errcmn {

	char *get_error_str(int _errno);
	void free_error_str(char *b);
	struct err_ev {
		char *errstr;
		int _errno;
		err_ev(void) : errstr(NULL), _errno(0) {};
		void setError(int e,const char *m=NULL);
		err_ev(int e) : err_ev() {
			setError(e);
		}
		err_ev(const err_ev &o) = delete;
		inline err_ev &operator=(const err_ev &o) = delete;
		inline err_ev &operator=(err_ev &&o) {
			this->errstr = o.errstr;  // transfer string to other guy...
			this->_errno = o._errno;
			o.errstr = NULL; o._errno = 0;
			return *this;
		}
		inline void clear() {
			if(errstr) ::free(errstr); errstr = NULL;
			_errno = 0;
		}
		~err_ev() {
			if(errstr) ::free(errstr);
		}
		bool hasErr() { return (_errno != 0); }
	};
}

#define ERROR_PERROR(s,E,...) { char *__S=_errcmn::get_error_str(E); fprintf(stderr, "**ERROR** [ %s ] " s "\n", __S, ##__VA_ARGS__ ); _errcmn::free_error_str(__S); }

#endif /// __cplusplus

#define ERR_STRERROR_R(ernum,b,len) __xpg_strerror_r(ernum, b, len)



#ifdef ERRCMN_DEBUG_BUILD
#pragma message "Build is Debug"
// confused? here: https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html
#define ERROR_OUT(s,...) fprintf(stderr, "**ERROR** " s "\n", ##__VA_ARGS__ )
//#define ERROR_PERROR(s,...) fprintf(stderr, "*****ERROR***** " s, ##__VA_ARGS__ );
#define DBG_OUT(s,...) fprintf(stderr, "**DEBUG** " s "\n", ##__VA_ARGS__ )
#define IF_DBG( x ) { x }
#else
#define ERROR_OUT(s,...) fprintf(stderr, "**ERROR** " s, ##__VA_ARGS__ )
//#define ERROR_PERROR(s,...) fprintf(stderr, "*****ERROR***** " s, ##__VA_ARGS__ );
#define DBG_OUT(s,...) {}
#define IF_DBG( x ) {}
#endif


#endif /* ERROR_COMMON_H_ */
