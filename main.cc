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
 * main.c
 *
 *  Created on: Feb 16, 2017
 *      Author: ed
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <ctype.h>
#include <syslog.h>

#include <TW/tw_fifo.h>

#include <uv.h>

#include "error-common.h"
#include "watchdoglib.h"
#include "./led_control/ledlib.h"

extern "C" char *local_strdup_safe(const char *s); // local_strdup.c
extern "C" char *local_strcat_safe(const char *s, const char *s2); // local_strdup.c

//const int INIT_WAIT = 60*5*1000; //5 minutes
const int INIT_WAIT = 30*1000; // 30 seconds

const int HEARTBEAT_INTERVAL = 10*1000; // 10 seconds
const int MAX_KEEPALIVE = 45; // in seconds. the largest keep alive command we will accept. (caps here)
const char *DEFAULT_SOCKET_PATH = "/tmp/devOSkeepalive";
const char *DEFAULT_LED_CONTROL_SOCKET_PATH = "/tmp/ledcontrol";

// if we don't hear from anyone in this amount of time
// we don't do a heartbeat
const int TIME_UNTIL_FLATLINE = 30*1000; // 30 second

#define BILLION 1000000000L

struct timespec last_beat_ts;
struct timespec last_keepalive;

uv_loop_t mainLoop;
uv_timer_t keepAliveTimer;

int watchdog_timeout_interval = 0; // filled in by the watchdog_start() function

int until_next_miss; // not really using this

int watchdog_enabled = 0; // if true then watchdog_start() has been called

int wakeup_pipe[2];
char *unixDgramPath = NULL;


int counter = 0;

unsigned long Rs, Gs, Bs, Re, Ge, Be;
unsigned long t;


const char *COMMAND_UP = "up";
const char *COMMAND_STOP = "stop";
const char *COMMAND_START = "start";
const char *COMMAND_STATUS = "status";
const char *COMMAND_LED = "led";
const char *COMMAND_LED_BLINK_ONCE = "led_b";
const char *COMMAND_LED_BLINK = "led_B";
const char *COMMAND_LED_FADE_ONCE = "led_f";
const char *COMMAND_LED_FADE = "led_F";
const char *COMMAND_LED_TOGGLE = "led_T";
const char *COMMAND_PIEZO = "piezo";
const char *COMMAND_PIEZO_TONE = "piezo_tone";
const char *COMMAND_MGMT_GET_INFO="mgmt";
const char *COMMAND_TINY_WATCHDOG_ENABLE = "dog_go";
const char *COMMAND_TINY_WATCHDOG_DISABLE = "dog_stop";
const char *COMMAND_TINY_WATCHDOG_EXPIRE = "dog_die";
const char *COMMAND_TINY_WATCHDOG_HEARTBEAT = "dog_run";

const char *COMMAND_TINY_WATCHDOG_SET = "dog_set";
const char *COMMAND_TINY_WATCHDOG_TIME_STATUS = "dog_status";

const char *WD_ERRORSTR_UNAVAILABLE = "WATCHDOG UNAVAILABLE";
const char *WD_ERRORSTR_BAD_RESPONSE= "WATCHDOG BAD RESPONSE";
const char *WD_UNKNOWN_ERROR_RESPONSE= "WATCHDOG UNKNOWN ERROR";

void doKeepAlive(int seconds);
void resetTimer(int interval);
static void ms2ts(struct timespec *ts, uint64_t ms)
{
	ts->tv_sec = ms / 1000;
	ts->tv_nsec = (ms % 1000) * 1000000;
}

static void ts2ms(struct timespec *ts, uint64_t *ms)
{
	*ms = ts->tv_sec * 1000UL;
	*ms += ts->tv_nsec / 1000000UL;
}

int isDaemon = 0;

#define WD_ERROR_PERROR(s,E,...) if(isDaemon) { char *__S=_errcmn::get_error_str(E); syslog(LOG_ERR, "**ERROR** [ %s ] " s "\n", __S, ##__VA_ARGS__ ); _errcmn::free_error_str(__S);  } else { ERROR_PERROR(s,E,  ##__VA_ARGS__ ); }




class responseWork {
public:
	responseWork() : cmd(0), aux(NULL) {}
	responseWork( uint32_t a ) : aux(NULL) { cmd = a; }; // default constructor (or param)
	responseWork( responseWork &o ) : cmd(o.cmd), aux(o.aux) { };  // copy constructor
	responseWork& operator=(const responseWork &o) { this->cmd = o.cmd; this->aux = o.aux; return *this; }
	uint32_t cmd;
	// future auxillary data
	void *aux;
};


// a queue type used to hold things we need to respond to
class responseWorkQueue : public TWlib::tw_safeFIFO<responseWork, TWlib::Allocator<TWlib::Alloc_Std> > {};

class DgramKeepAliveSocket {
protected:

	uv_thread_t listener_thread;
//	uv_thread_t response_worker_thread;
	char *path;

public:

	uv_loop_t *loop;
//		uv_pipe_t pipe; // on Unix this is a AF_UNIX/SOCK_STREAM, on Windows its a Named Pipe
	uv_mutex_t control_mutex;
	bool stop_thread;
	int socket_fd;
	int wakeup_pipe[2];
	static const int PIPE_WAIT = 1;
	static const int PIPE_WAKEUP = 0;
	static const int DESIRED_SOCKET_SIZE = 65536;
	struct sockaddr_un sink_dgram_addr;
	bool ready;
//	responseWorkQueue responseQueue;
	DgramKeepAliveSocket() = delete;
	DgramKeepAliveSocket(const char *_path, uv_loop_t *l) :
//				buffers(BUFFERS_PER_SINK),
	listener_thread(),
	path(NULL), loop(l),
	stop_thread(false),
	socket_fd(0),
			wakeup_pipe(), // {-1,-1}
			ready(false)//			responseQueue()
			{
//			uv_pipe_init(l,&pipe,0);
//			pipe.data = this;
				wakeup_pipe[0] = -1; wakeup_pipe[1] = -1;
				if(_path && strlen(_path) > 0) {
					path = ::local_strdup_safe(_path);
				}
				else
					WD_DBG_OUT("UnixDgramSink: No path set. Will fail.\n");
//			for (int n=0;n<BUFFERS_PER_SINK;n++) {
//				heapBuf *b = new heapBuf(SINK_BUFFER_SIZE);
////				buffers.add(b);
//			}
			}

			bool bind() {
				if(path) {
					socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
					if(socket_fd < 0) {
						ERROR_PERROR("SyslogDgramSink: Failed to create SOCK_DGRAM socket.\n", errno);
					} else {
						if(pipe(wakeup_pipe) < 0) {
							ERROR_PERROR("SyslogDgramSink: Failed to pipe() for SOCK_DGRAM socket.\n", errno);
						} else {
							fcntl(wakeup_pipe[PIPE_WAIT], F_SETFL, O_NONBLOCK);
							::memset(&sink_dgram_addr,0,sizeof(sink_dgram_addr));
							sink_dgram_addr.sun_family = AF_UNIX;
					unlink(path); // get rid of it if it already exists
					strcpy(sink_dgram_addr.sun_path,path);
					if(::bind(socket_fd, (const struct sockaddr *) &sink_dgram_addr, sizeof(sink_dgram_addr)) < 0) {
						ERROR_PERROR("SyslogDgramSink: Failed to bind() SOCK_DGRAM socket.\n", errno);
						close(socket_fd);
					} else {
						ready = true;
					}
				}
			}
		} else {
			WD_ERROR_OUT("SyslogDgramSink: No path set.");
		}
		return ready;
	}


//	static void response_work(void *self) {
//		DgramKeepAliveSocket *sink = (DgramKeepAliveSocket *) self;
//
//		responseWork work;
//
//		while(sink->responseQueue.removeOrBlock(work)) {
//			switch (work.cmd) {
//			case CMD_GET_WD_STATUS:
//
//				break;
//			default:
//				WD_ERROR_OUT("Unknown command in response worker: %d",work.cmd);
//			}
//
//		}
//	}

	static void listener_work(void *self) {
		DgramKeepAliveSocket *sink = (DgramKeepAliveSocket *) self;

		fd_set readfds;

		char dump[5];

//		const int nbuffers = 10;
		const int nbuffers = 1;
		char *raw_buffer[nbuffers];
		struct iovec iov[nbuffers];
		struct msghdr message;

		socklen_t optsize;

		char *temp_buffer_entry = (char *) malloc(1024);


		// using MSG_DONTWAIT instead
//			int flags = fcntl(socket_fd, F_GETFL, 0);
//			if (flags < 0) {
//				ERROR_PERROR("UnixDgramSink: Error getting socket flags\n",errno);
//			}
//			flags = flags|O_NONBLOCK;
//			if(fcntl(socket_fd, F_SETFL, flags) < 0) {
//				ERROR_PERROR("UnixDgramSink: Error setting socket non-blocking flags\n",errno);
//			}

		// discover socket max recieve size (this will be the max for a non-fragmented log message
		int rcv_buf_size = 65536;
		setsockopt(sink->socket_fd, SOL_SOCKET, SO_RCVBUF, &rcv_buf_size, (unsigned int) sizeof(int));
		getsockopt(sink->socket_fd, SOL_SOCKET, SO_RCVBUF, &rcv_buf_size, &optsize);
		// http://stackoverflow.com/questions/10063497/why-changing-value-of-so-rcvbuf-doesnt-work
		if(rcv_buf_size < 100) {
			WD_ERROR_OUT("SyslogDgramSink: Failed to start reader thread - SO_RCVBUF too small\n");
		} else {
			WD_DBG_OUT("SyslogDgramSink: SO_RCVBUF is %d\n", rcv_buf_size);
		}

		for(int n=0;n<nbuffers;n++) {
			raw_buffer[n] = (char *) ::malloc(rcv_buf_size);
		}

		FD_ZERO(&readfds);
		FD_SET(sink->wakeup_pipe[PIPE_WAIT], &readfds);
		FD_SET(sink->socket_fd, &readfds);

		int n = sink->socket_fd + 1;
		if(sink->wakeup_pipe[PIPE_WAIT] > n)
			n = sink->wakeup_pipe[PIPE_WAIT]+1;

		int recv_cnt = 0;
		int err_cnt = 0;
		struct sockaddr_un client_addr;
		socklen_t client_addr_len = sizeof(sockaddr_un);
		while(sink->ready && !sink->stop_thread) {
			memset(&client_addr,sizeof(struct sockaddr),0);
			client_addr_len = sizeof(sockaddr_un);
			WD_DBG_OUT("at select()");
			int err = select(n,&readfds,NULL,NULL,NULL); // block and wait...
			if(err == -1) {
				ERROR_PERROR("UnixDgramSink: error on select() \n", errno);
			} else if(err > 0) {
				if(FD_ISSET(sink->wakeup_pipe[PIPE_WAIT], &readfds)) {
					while(read(sink->wakeup_pipe[PIPE_WAIT], dump, 1) == 1) {}
				}
			if(FD_ISSET(sink->socket_fd, &readfds)) {

					// ////////////////////////////////
					// Business work of sink.........
					// ////////////////////////////////
				message.msg_name=&sink->sink_dgram_addr;
				message.msg_namelen=sizeof(struct sockaddr_un);
				message.msg_iov=iov;
				message.msg_iovlen=nbuffers;
				message.msg_control=NULL;
				message.msg_controllen=0;
				message.msg_flags = 0;


				for(int n=0;n<nbuffers;n++) {
					memset(raw_buffer[n],0,rcv_buf_size);
					iov[n].iov_base = raw_buffer[n];
					iov[n].iov_len = rcv_buf_size;
				}

//					if((recv_cnt = recvmsg(sink->socket_fd, &message, MSG_DONTWAIT)) < 0) {
//						if(errno != EAGAIN || errno != EWOULDBLOCK) {
//							ERROR_PERROR("SyslogDgramSink: Error on recvfrom() ", errno);
//							err_cnt++;
//						}
//					}
				if((recv_cnt = recvfrom(sink->socket_fd, iov[0].iov_base, rcv_buf_size, MSG_DONTWAIT,(sockaddr *) &client_addr,&client_addr_len)) < 0) {
					if(errno != EAGAIN || errno != EWOULDBLOCK) {
						ERROR_PERROR("SyslogDgramSink: Error on recvfrom() ", errno);
						err_cnt++;
					}
				}
				iov[0].iov_len = recv_cnt;

				WD_DBG_OUT("Got packet from: %s %d\n",client_addr.sun_path,client_addr_len);

#define MAX_MESSAGE 512

				int remain = recv_cnt;
//					int remain_in_buf = (recv_cnt > rcv_buf_size) ? rcv_buf_size : recv_cnt;
				int fill_time = 0;
				char fill_cmd[MAX_MESSAGE];

				int iov_n = 0;
				char *current_buffer = NULL;
				uint32_t entry_size;

#define SD_WAL_BUF_P (current_buffer + (walk_buf))
#define SD_WALK(N) do{walk += N; walk_buf += N; remain = remain - N; remain_in_buf = remain_in_buf - N;}while(0)


				while(iov_n < nbuffers && remain > 0) {
					if(iov[iov_n].iov_len > 0) {


						int amount = iov[iov_n].iov_len;
						if(amount > recv_cnt) amount = recv_cnt;
#ifdef ERRCMN_DEBUG_BUILD
						char *buf = (char *) malloc(iov[iov_n].iov_len+1);
						::memcpy(buf,iov[iov_n].iov_base,amount);
						*(buf+amount) = 0;
						WD_DBG_OUT("socket in %d>>%s<<",iov_n, buf);
						free(buf);
#endif
						if(recv_cnt > MAX_MESSAGE && iov[iov_n].iov_len > MAX_MESSAGE) {
							WD_ERROR_OUT("huge message on socket. ignoring. %lu bytes", iov[iov_n].iov_len);
						} else
						if(amount > 3) {
							fill_time = 0;
							if(sscanf((char *) iov[iov_n].iov_base,"%s %d",fill_cmd,&fill_time) > 0) {
								WD_DBG_OUT("Command: %s  seconds: %d",fill_cmd, fill_time);
								if(strcmp(COMMAND_UP,fill_cmd) == 0 && fill_time > 0) {
									doKeepAlive(fill_time);
								} else if(strcmp(COMMAND_STOP, fill_cmd) == 0) {
									WD_PRINTF_OUT("Got stop command.");
									watchdog_stop();
									watchdog_enabled = 0;
								} else if(strcmp(COMMAND_START, fill_cmd) == 0 && fill_time > 0) {
									watchdog_start(fill_time, &watchdog_timeout_interval);
									resetTimer(watchdog_timeout_interval);
									WD_PRINTF_OUT("Got start command. watchdog interval: %d seconds requested. watchdog will do %d seconds",fill_time,watchdog_timeout_interval);
									watchdog_enabled = 1;
									doKeepAlive(fill_time);
								} else if(strcmp(COMMAND_STATUS, fill_cmd) == 0) {
									WD_PRINTF_OUT("Got status command.");
									char *bufans; int buflen;
									int err = watchdog_status(&bufans,&buflen);
									if(err != 0) {
										switch(err) {
											case WD_UNAVAILABLE:
											sendto(sink->socket_fd, WD_ERRORSTR_UNAVAILABLE, strlen(WD_ERRORSTR_UNAVAILABLE),0,(sockaddr *)&client_addr,client_addr_len);
											WD_PRINTF_OUT("sending: %s",WD_ERRORSTR_UNAVAILABLE);
											break;
											case WD_BAD_RESPONSE:
											sendto(sink->socket_fd, WD_ERRORSTR_BAD_RESPONSE, strlen(WD_ERRORSTR_BAD_RESPONSE),0,(sockaddr *)&client_addr,client_addr_len);
											WD_PRINTF_OUT("sending: %s",WD_ERRORSTR_BAD_RESPONSE);
											break;
											default:
											sendto(sink->socket_fd, WD_UNKNOWN_ERROR_RESPONSE, strlen(WD_UNKNOWN_ERROR_RESPONSE),0,(sockaddr *)&client_addr,client_addr_len);
											WD_PRINTF_OUT("sending: %s",WD_UNKNOWN_ERROR_RESPONSE);
										}
									} else {
										sendto(sink->socket_fd, bufans, buflen,0,(sockaddr *)&client_addr,client_addr_len);
										WD_PRINTF_OUT("sending: %s",bufans);
										free(bufans);
									}

//										watchdog_start(fill_time, &watchdog_timeout_interval);
//										resetTimer(watchdog_timeout_interval);
//										WD_PRINTF_OUT("Got start command. watchdog interval: %d seconds requested. watchdog will do %d seconds",fill_time,watchdog_timeout_interval);
//										watchdog_enabled = 1;
//										doKeepAlive(fill_time);
								}


								else if(strcmp(COMMAND_LED, fill_cmd) == 0)
								{
									if(sscanf((char *) iov[iov_n].iov_base,"%s %d %d %d",fill_cmd, &Rs, &Gs, &Bs) > 0)
									{

										push_ledcontrol_request('L', 'S',Rs, Gs, Bs, t);

									}
								}

								else if(strcmp(COMMAND_LED_BLINK, fill_cmd) == 0)
								{
									if(sscanf((char *) iov[iov_n].iov_base,"%s %d %d %d %ld",fill_cmd, &Rs, &Gs, &Bs,&t) > 0)
									{
								    	push_ledcontrol_request('L', 'B',Rs, Gs, Bs,t);
									}
								}

								else if(strcmp(COMMAND_LED_BLINK_ONCE, fill_cmd) == 0)
								{
									if(sscanf((char *) iov[iov_n].iov_base,"%s %d %d %d %ld",fill_cmd, &Rs, &Gs, &Bs,&t) > 0)
									{

										push_ledcontrol_request('L', 'b',Rs, Gs, Bs,t);

									}
								}

								else if(strcmp(COMMAND_LED_FADE, fill_cmd) == 0)
								{
									if(sscanf((char *) iov[iov_n].iov_base,"%s %d %d %d %d %d %d %ld",fill_cmd, &Rs, &Gs, &Bs, &Re, &Ge, &Be,&t) > 0)
									{

										push_pwm_led_control_request('L', 'F',Rs, Gs, Bs, Re, Ge, Be,t);

									}
								}

								else if(strcmp(COMMAND_LED_FADE_ONCE, fill_cmd) == 0)
								{
									if(sscanf((char *) iov[iov_n].iov_base,"%s %d %d %d %d %d %d %ld",fill_cmd, &Rs, &Gs, &Bs, &Re, &Ge, &Be,&t) > 0)
									{

										push_pwm_led_control_request('L', 'f',Rs, Gs, Bs, Re, Ge, Be,t);

									}
								}

								else if(strcmp(COMMAND_LED_TOGGLE, fill_cmd) == 0)
								{
									if(sscanf((char *) iov[iov_n].iov_base,"%s %d %d %d %d %d %d %ld",fill_cmd, &Rs, &Gs, &Bs, &Re, &Ge, &Be,&t) > 0)
									{
									    	push_pwm_led_control_request('L', 'T',Rs, Gs, Bs, Re, Ge, Be,t);
									}
								}
								else if(strcmp(COMMAND_TINY_WATCHDOG_SET, fill_cmd) == 0)
								{
									if(sscanf((char *) iov[iov_n].iov_base,"%s %d",fill_cmd, &t) > 0)
									{
									    	watchdog_set(t);
									}
								}
								else if(strcmp(COMMAND_PIEZO, fill_cmd) == 0)
								{
									unsigned long power;
									unsigned long time;
									if(sscanf((char *) iov[iov_n].iov_base,"%s %d %ld",fill_cmd, &power,&time) > 0)
									{
										//push_to_ATtiny('P', 0,power,0, time);
									}
								}
								else if(strcmp(COMMAND_PIEZO_TONE, fill_cmd) == 0)
								{
									unsigned long tone_num;
									if(sscanf((char *) iov[iov_n].iov_base,"%s %d",fill_cmd, &tone_num) > 0)
									{
										getRing(tone_num);
									}
								}

								else if(strcmp(COMMAND_MGMT_GET_INFO, fill_cmd) == 0) {
									mgmtcontrol_getInfo();
								}
								else if(strcmp(COMMAND_TINY_WATCHDOG_ENABLE, fill_cmd) == 0) {
									watchdog_enable();
								}
								else if(strcmp(COMMAND_TINY_WATCHDOG_DISABLE, fill_cmd) == 0) {
									push_pwm_led_control_request('L', 'T', 255, 10, 110, 0, 0, 0, 500);
									watchdog_disable();
								}
								else if(strcmp(COMMAND_TINY_WATCHDOG_EXPIRE, fill_cmd) == 0) {
									watchdog_expirenow();
								}
								else if(strcmp(COMMAND_TINY_WATCHDOG_HEARTBEAT, fill_cmd) == 0) {
									watchdog_heartbeat();
								}
								else if(strcmp(COMMAND_TINY_WATCHDOG_TIME_STATUS, fill_cmd) == 0) {
									watchdog_timeremaining();
								}
							} else {
								WD_ERROR_OUT("Malformed message.");
							}
						} else {
							WD_ERROR_OUT("tiny message on socket. ignoring.");
						}


						remain -= iov[iov_n].iov_len;
						iov_n++;

					} else {
						WD_DBG_OUT("socket in %d ZERO length",iov_n );
					}
				}


			}
			} // else timeout
		}

		free(temp_buffer_entry);
	}

	void start() {
		uv_thread_create(&listener_thread,DgramKeepAliveSocket::listener_work,(void *) this);
//		uv_thread_create(&response_worker_thread,DgramKeepAliveSocket::response_work,(void *) this);
	}

	void stop() {
		uv_mutex_lock(&control_mutex);
		stop_thread = true;
		uv_mutex_unlock(&control_mutex);
		wakeup_thread(); // waking up thread, will cause thread to stop since flag is set
	}


	~DgramKeepAliveSocket() {
		if(path) ::free(path);
		if(wakeup_pipe[0] < 0)
			close(wakeup_pipe[0]);
		if(wakeup_pipe[1] < 0)
			close(wakeup_pipe[1]);
	}

protected:
	void wakeup_thread() {
		if(wakeup_pipe[PIPE_WAKEUP] > -1) {
			write(wakeup_pipe[PIPE_WAKEUP], "x", 1);
		}
	}
};


int init_wait = INIT_WAIT;
int heartbeat_interval = HEARTBEAT_INTERVAL;
int time_until_flatline = TIME_UNTIL_FLATLINE;

int freebie_heartbeats; // computed in startTimer. These are the initial heartbeats
                        // the daemon does not need to see a keep alive

uint64_t next_flatline = 0; // in milliseconds
uint64_t last_beat_ms = 0;

uint64_t ms_uptime = 0;
uint64_t heartbeat_interval_ms = 0;



void heartbeat(uv_timer_t* handle) {
	// will send funky stuff to memory to stop reboot

	counter++;

	WD_DBG_OUT("heartbeat %d",counter);
	WD_DBG_OUT("uptime: %lu s",(heartbeat_interval_ms*counter)/1000);

	clock_gettime(CLOCK_MONOTONIC,&last_beat_ts);
	ts2ms(&last_beat_ts, &last_beat_ms);

	if(!watchdog_enabled) {
		WD_DBG_OUT("watchdog disabled. doing nothing\n");
		return;
	}

	if(freebie_heartbeats > 0) {
		WD_DBG_OUT("  Freebie heartbeats %d\n",freebie_heartbeats);
		freebie_heartbeats--;
		watchdog_keepalive(&until_next_miss);
		watchdog_heartbeat();
		return;
	}

	if(last_beat_ms > next_flatline) {
		//Send indication that gateway is flatline
		WD_ERROR_OUT("!!! FLATLINE.\n");
		int i = 0;
		for(i = 0; i < 10; i++) {
			push_ledcontrol_request('L', 'S', 30, 0, 0, 0);
			usleep(100000);
			push_ledcontrol_request('L', 'S', 0, 0, 0, 0);
			usleep(100000);
		}
		
		watchdog_expirenow();

	} else {
		//Send indication that its going to flatline
		if(counter % 2 == 0) {
			push_ledcontrol_request('L', 'S', 30, 30, 30, 0);
		} else {
			push_ledcontrol_request('L', 'S', 15, 15, 15, 0);
		}
		watchdog_keepalive(&until_next_miss);
		watchdog_heartbeat();
		WD_DBG_OUT("  sent keepalive. %lu ms until flat line\n",next_flatline-last_beat_ms);
	}

	//	observationCounter++;
//	//	printf("\nheartbeat.\n\n");
}


void resetTimer(int interval) {

	clock_gettime(CLOCK_MONOTONIC,&last_beat_ts);
	memcpy(&last_keepalive,&last_beat_ts,sizeof(struct timespec));

	heartbeat_interval_ms = interval*1000;
	uv_timer_set_repeat(&keepAliveTimer, heartbeat_interval_ms); // start timer on next turn
}

/**
 * Interval is in seconds
 */
void startTimer(int interval) {
	clock_gettime(CLOCK_MONOTONIC,&last_beat_ts);
	memcpy(&last_keepalive,&last_beat_ts,sizeof(struct timespec));

	freebie_heartbeats = init_wait / (interval*1000);
	heartbeat_interval_ms = interval*1000;
	uv_timer_start(&keepAliveTimer, heartbeat, 0, heartbeat_interval_ms); // start timer on next turn

}

int max_keepalive = MAX_KEEPALIVE;

void doKeepAlive(int seconds) {

	if(seconds > max_keepalive) {
		seconds = max_keepalive;
	}

	WD_DBG_OUT("doKeepAlive %d\n",seconds);

	clock_gettime(CLOCK_MONOTONIC, &last_keepalive);
	uint64_t t;
	ts2ms(&last_keepalive, &t);

	if((next_flatline - t) < heartbeat_interval_ms) {
		WD_DBG_OUT("reset watchdog immediately. almost timedout.");
		watchdog_keepalive(&until_next_miss);
		watchdog_heartbeat();
	}

	t += seconds * 1000;
	if(t > next_flatline) next_flatline = t;

}


void show_usage() {
	printf("deviceOSWD - deviceOS watchdog\n");
	printf("Options:\n");
	printf(" -w [seconds]   Wait [seconds] before starting hardware watchdog\n");
	printf(" -s [path]      Use [path] for keep alive socket\n");
	printf(" -m [seconds]   The max allowed keep alive.\n");
	printf(" -d             Daemon the process. All log output will go to syslog.\n");
	printf(" -p [path]      Place a deviceOSWD.pid file at 'path'\n");
	printf(" -l [path]      Use [path] for led control socket\n");
	printf(" -v             Version number\n");
	printf("\n");
}

pid_t mypid = 0;

const char *PID_FILE_NAME = "/deviceOSWD.pid";

int main(int argc, char **argv) {
	char *str_arg = NULL;
	int index;
	int num;
	int c;
	char *pid_file_path = NULL;

	char *path = local_strdup_safe(DEFAULT_SOCKET_PATH);
	char *ledSocketPath = local_strdup_safe(DEFAULT_LED_CONTROL_SOCKET_PATH);

	opterr = 0;

	while ((c = getopt (argc, argv, "hdvm:w:s:p:")) != -1)
		switch (c)
	{
		case 'p':
		pid_file_path = optarg;
		break;
		case 'd':
		isDaemon = 1;
		break;
		case 'h':
		show_usage();
		exit(0);
		break;
		case 'w':
		str_arg = optarg;
		if(sscanf(str_arg,"%d",&num) > 0) {
			if(num > 0) {
				init_wait = num * 1000;
				printf("Ok, initial wait for %d seconds\n",num);
			}
		} else {
			fprintf(stderr,"Not a valid wait time\n");
			exit(1);
		}
		break;
		case 'm':
		str_arg = optarg;
		if(sscanf(str_arg,"%d",&num) > 0) {
			if(num > 0) {
				max_keepalive = num;
				printf("Ok, max wait is %d seconds\n",num);
			}
		} else {
			fprintf(stderr,"Not a valid wait time\n");
			exit(1);
		}
		break;
		case 's':
		free(path);
		path = local_strdup_safe(optarg);
		break;
	case 'v': //Added something really simple to make sure the build system is getting the latest
	printf("1.2\n");
	exit(0);
	break;
	case 'l':
	free(ledSocketPath);
	ledSocketPath = local_strdup_safe(optarg);
	break;
	case '?':
	if (optopt == 'c')
		fprintf (stderr, "Option -%c requires an argument.\n", optopt);
	else if (isprint (optopt))
		fprintf (stderr, "Unknown option `-%c'.\n", optopt);
	else
		fprintf (stderr,
			"Unknown option character `\\x%x'.\n",
			optopt);
	return 1;
	default:
	abort ();
}



pid_t sid;

if(isDaemon) {
	mypid = fork();
	if(mypid == 0) {
		setlogmask (LOG_UPTO (LOG_DEBUG));
		openlog("deviceOSWD",LOG_CONS | LOG_PID | LOG_NDELAY, LOG_DAEMON );
			// daemonize:
			// http://www.thegeekstuff.com/2012/02/c-daemon-process
			// child is daemon.
		umask(0);
		sid = setsid();
		if(sid < 0) {
			WD_ERROR_OUT("Could not set session ID. Did you run as root? Error: %d\n",errno);
			exit(1);
		}
		chdir("/tmp");
		close(STDIN_FILENO);
		close(STDOUT_FILENO);
		close(STDERR_FILENO);
	} else {
		printf("Started deviceOSWD watchdog as PID %d\n",mypid);
		exit(0);
	}

}

mypid = getpid();

if(pid_file_path) {
	char *fpath = local_strcat_safe(pid_file_path,PID_FILE_NAME);
	if(fpath) {
		int fd = open(fpath,O_WRONLY | O_CREAT | O_TRUNC );
		if(fd == -1) {
			WD_ERROR_OUT("Failed to create PID file. %s Errno: %d",fpath,errno);
		} else {
			char str[128];
			int len = snprintf(str,128,"%d\n",mypid);
			if(write(fd,str,len) == -1) {
				WD_ERROR_OUT("Error writing to PID file. %s Errno: %d",fpath, errno);
			} else {
				close(fd);
			}
		}
	}
}

WD_PRINTF_OUT("deviceOS watchdog starting! PID:%d",mypid);

uv_loop_init(&mainLoop);
	// timer keeps uv_run up
uv_timer_init(&mainLoop, &keepAliveTimer);


DgramKeepAliveSocket keepAliveSocket(path,&mainLoop);
if(keepAliveSocket.bind()) {
	keepAliveSocket.start();

	init_ledcontrol(ledSocketPath);

	watchdog_init(&mainLoop);
	WD_DBG_OUT("watchdog_init");

	watchdog_start(15,&watchdog_timeout_interval);
	watchdog_enabled = 1;
	WD_DBG_OUT("Watchdog says it needs a keep alive every %d seconds\n",watchdog_timeout_interval);
	if(watchdog_timeout_interval > 5) {
			watchdog_timeout_interval -= 4; // give our selves a 2 second buffer;
		}
		WD_DBG_OUT("Daemon will provide keepalive %d seconds\n",watchdog_timeout_interval);

		startTimer(watchdog_timeout_interval);

	} else {
		WD_ERROR_OUT("Could not bind socket listener on %s",path);
		exit(1);
	}
	uv_run(&mainLoop, UV_RUN_DEFAULT);

}



/**

 test with:
 Use this for unix domain (SOCK_DGRAM)
echo -e "up 10" | socat unix-sendto:/tmp/devOSkeepalive STDIO

If you use the 'status' command, try this to see the response:
echo -e "status" | nc -uU /tmp/devOSkeepalive
echo -e "status" | nc.netcat-openbsd -uU /var/deviceOSkeepalive


 */


