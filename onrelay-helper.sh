#!/bin/bash

# Copyright (c) 2018, Arm Limited and affiliates.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

PIDROOT="/var/run"
KEEPALIVE="/var/deviceOSkeepalive"
DISABLE_TINY_WATCHDOG_CMD="echo -e \"dog_stop\" | socat unix-sendto:$KEEPALIVE STDIO"
STOP_DEVICEOSWD_CMD="echo -e \"stop\" | socat unix-sendto:$KEEPALIVE STDIO"
MGMT="echo -e \"mgmt\" | socat unix-sendto:$KEEPALIVE STDIO"
RED="echo -e \"led 10 0 0\" | socat unix-sendto:$KEEPALIVE STDIO"
BLUE="echo -e \"led 0 0 10\" | socat unix-sendto:$KEEPALIVE STDIO"
GREEN="echo -e \"led 0 10 0\" | socat unix-sendto:$KEEPALIVE STDIO"
LED_FADE="echo -e \"led_F 255 0 0 0 255 0 5000\" | socat unix-sendto:$KEEPALIVE STDIO"
LED_FADE_ONCE="echo -e \"led_f 255 0 0 0 255 0 5000\" | socat unix-sendto:$KEEPALIVE STDIO"
LED_TOGGLE="echo -e \"led_T 255 0 0 0 255 0 1000\" | socat unix-sendto:$KEEPALIVE STDIO"
LED_BLINK_ONCE="echo -e \"led_b 255 0 0 5000\" | socat unix-sendto:$KEEPALIVE STDIO"


PIEZO_TONE="echo -e \"piezo_tone 1\" | socat unix-sendto:$KEEPALIVE STDIO"
TINY_WATCHDOG_ENABLE="echo -e \"dog_go\" | socat unix-sendto:$KEEPALIVE STDIO"
TINY_WATCHDOG_DISABLE="echo -e \"dog_stop\" | socat unix-sendto:$KEEPALIVE STDIO"
TINY_WATCHDOG_EXPIRE="echo -e \"dog_die\" | socat unix-sendto:$KEEPALIVE STDIO"
TINY_WATCHDOG_HEARTBEAT="echo -e \"dog_run\" | socat unix-sendto:$KEEPALIVE STDIO"
TINY_WATCHDOG_STATUS="echo -e \"dog_status\" | socat unix-sendto:$KEEPALIVE STDIO"

HB="echo -e \"dog_run\" | socat unix-sendto:$KEEPALIVE STDIO"
init(){
	if [[ ! -e .inited ]]; then
		cd deps
		./install-deps.sh
		cd ..
		touch .inited
	fi
}

build() {
	init
	make deviceOSWD-a10-tiny841-onrelay
}


run(){
	./deviceOSWD -w 300 -m 90 -s $KEEPALIVE -p $PIDROOT/
}

killit(){
	echo "killing that stuff"
	eval "$DISABLE_TINY_WATCHDOG_CMD"
	eval "$STOP_DEVICEOSWD_CMD"
	pkill deviceOSWD
}


if [[ $1 = "buildrun" ]]; then
	killit
	build
	run
elif [[ $1 = "run" ]]; then
	run
elif [[ $1 = "build" ]]; then
	build
elif [[ $1 = "kill" ]]; then
	killit
elif [[ $1 = "red" ]]; then
	eval "$RED"
elif [[ $1 = "green" ]]; then
	eval "$GREEN"
elif [[ $1 = "blue" ]]; then
	eval "$BLUE"
elif [[ $1 = "hb" ]]; then
	eval "$HB"
elif [[ $1 = "piezo" ]]; then
        eval "$PIEZO_TONE"
elif [[ $1 = "test" ]]; then
	eval "$MGMT"
elif [[ $1 = "led_fade" ]]; then
        eval "$LED_FADE"
elif [[ $1 = "led_fade_once" ]]; then
        eval "$LED_FADE_ONCE"
elif [[ $1 = "led_toggle" ]]; then
        eval "$LED_TOGGLE"
elif [[ $1 = "led_blink" ]]; then
        eval "$LED_BLINK_ONCE"
elif [[ $1 = "dog_go" ]]; then
        eval "$TINY_WATCHDOG_ENABLE"
elif [[ $1 = "dog_stop" ]]; then
        eval "$TINY_WATCHDOG_DISABLE"
elif [[ $1 = "dog_die" ]]; then
        eval "$TINY_WATCHDOG_EXPIRE"
elif [[ $1 = "dog_status" ]]; then
        eval "$TINY_WATCHDOG_STATUS"
elif [[ $1 = "dog_run" ]]; then
        eval "$TINY_WATCHDOG_HEARTBEAT"
fi
