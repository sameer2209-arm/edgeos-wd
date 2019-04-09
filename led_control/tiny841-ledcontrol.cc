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

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <linux/serial.h>       /* for struct serial_struct */
#include <sys/ioctl.h>
#include <ctype.h>

#include "ledlib.h"
#include "./../error-common.h"
#include<pthread.h>
#include "notes.h"

#define NUM_THREADS 1
pthread_t threads[NUM_THREADS];
int thread_flag,arrindex;

uint32_t notes_array[100], notes_duration[100];

static char *uart_port_name = "/dev/ttyS3";
static unsigned int uart_baud = B38400;
/* handle returned from open() */
static int uart_handle = -1;
/* serial I/O settings */
static struct termios uart_oldtio;
/* for setting custom divisor */
static struct serial_struct uart_oldserial;

static int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		LD_ERROR_OUT("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		LD_ERROR_OUT("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

static void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		LD_ERROR_OUT("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		LD_ERROR_OUT("Error tcsetattr: %s\n", strerror(errno));
}

// void
// set_blocking (int fd, int should_block)
// {
//         struct termios tty;
//         memset (&tty, 0, sizeof tty);
//         if (tcgetattr (fd, &tty) != 0)
//         {
//                 error_message ("error %d from tggetattr", errno);
//                 return;
//         }

//         tty.c_cc[VMIN]  = should_block ? 1 : 0;
//         tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

//         if (tcsetattr (fd, TCSANOW, &tty) != 0)
//                 error_message ("error %d setting term attributes", errno);
// }
static void Uart_Set_Interface(char *ifname)
{
    /* note: expects a constant char, or char from the heap */
	if (ifname) {
		uart_port_name = ifname;
	}
}

static uint32_t Uart_Get_Baud_Rate(void)
{
	uint32_t baud = 0;

	switch (uart_baud) {
		case B0:
		baud = 0;
		break;
		case B50:
		baud = 50;
		break;
		case B75:
		baud = 75;
		break;
		case B110:
		baud = 110;
		break;
		case B134:
		baud = 134;
		break;
		case B150:
		baud = 150;
		break;
		case B200:
		baud = 200;
		break;
		case B300:
		baud = 300;
		break;
		case B600:
		baud = 600;
		break;
		case B1200:
		baud = 1200;
		break;
		case B1800:
		baud = 1800;
		break;
		case B2400:
		baud = 2400;
		break;
		case B4800:
		baud = 4800;
		break;
		case B9600:
		baud = 9600;
		break;
		case B19200:
		baud = 19200;
		break;
		case B38400:
		baud = 38400;
		break;
		case B57600:
		baud = 57600;
		break;
		case B115200:
		baud = 115200;
		break;
		case B230400:
		baud = 230400;
		break;
		default:
		baud = 9600;
	}

	return baud;
}

static bool Uart_Set_Baud_Rate(uint32_t baud)
{
	bool valid = true;

	switch (baud) {
		case 0:
		uart_baud = B0;
		break;
		case 50:
		uart_baud = B50;
		break;
		case 75:
		uart_baud = B75;
		break;
		case 110:
		uart_baud = B110;
		break;
		case 134:
		uart_baud = B134;
		break;
		case 150:
		uart_baud = B150;
		break;
		case 200:
		uart_baud = B200;
		break;
		case 300:
		uart_baud = B300;
		break;
		case 600:
		uart_baud = B600;
		break;
		case 1200:
		uart_baud = B1200;
		break;
		case 1800:
		uart_baud = B1800;
		break;
		case 2400:
		uart_baud = B2400;
		break;
		case 4800:
		uart_baud = B4800;
		break;
		case 9600:
		uart_baud = B9600;
		break;
		case 19200:
		uart_baud = B19200;
		break;
		case 38400:
		uart_baud = B38400;
		break;
		case 57600:
		uart_baud = B57600;
		break;
		case 115200:
		uart_baud = B115200;
		break;
		case 230400:
		uart_baud = B230400;
		break;
		default:
		valid = false;
		break;
	}

	if (valid) {
        /* FIXME: store the baud rate */
	}

	return valid;
}

static void Uart_Cleanup(void)
{
    /* restore the old port settings */
	tcsetattr(uart_handle, TCSANOW, &uart_oldtio);
	ioctl(uart_handle, TIOCSSERIAL, &uart_oldserial);
	close(uart_handle);
	LD_PRINTF_OUT("Closed attiny serial handle");
}


static uint8_t generate_crc(uint8_t *buffer, int len) 
{
	int i = 0;
	uint8_t crc = 0x00;
	for(i = 0; i < len; i++) {
		crc += buffer[i];
	}
	crc = crc % 64;
	return crc;
}

/*
   Params:
      fd       -  (int) socket file descriptor
      buffer - (char*) buffer to hold data
      len     - (int) maximum number of bytes to recv()
      flags   - (int) flags (as the fourth param to recv() )
      to       - (int) timeout in milliseconds
   Results:
      int      - The same as recv, but -2 == TIMEOUT
   Notes:
      You can only use it on file descriptors that are sockets!
      'to' must be different to 0
      'buffer' must not be NULL and must point to enough memory to hold at least 'len' bytes
      I WILL mix the C and C++ commenting styles...
*/
static int Uart_Receive_Frame(int fd, uint8_t *buffer, int len, int to) 
{
	fd_set readset;
	int result, iof = -1;
	struct timeval tv;

   // Initialize the set
	FD_ZERO(&readset);
	FD_SET(fd, &readset);

   // Initialize time out struct
	tv.tv_sec = 0;
	tv.tv_usec = to * 1000;
   // select()
	result = select(fd+1, &readset, NULL, NULL, &tv);

   // Check status
	if (result < 0) {
		return -1;
	} else if (result > 0 && FD_ISSET(fd, &readset)) {
      // // Set non-blocking mode
      // if ((iof = fcntl(fd, F_GETFL, 0)) != -1) {
      //    fcntl(fd, F_SETFL, iof | O_NONBLOCK);
      // }
      // receive
		result = read(fd, buffer, len);
      // set as before
      // if (iof != -1) {
      //    fcntl(fd, F_SETFL, iof);
      // }
		return result;
	}
	return -2;
}

static void Uart_Send_Frame(uint8_t * buffer, uint16_t nbytes)
{
	ssize_t written = 0;
    // int i = 0;
	LD_PRINTF_OUT("Send %d: \"%s\", ", nbytes, buffer);
    // for(i = 0; i < nbytes; i++) {
    //     printf("%02x ", buffer[i]);
    // }
    // printf("\n");
	written = write(uart_handle, buffer, nbytes);
	if (written <= 0) {
		LD_ERROR_OUT("write error: %s\n", strerror(errno));
	} else {
        /* wait until all output has been transmitted. */
		tcdrain(uart_handle);
	}
	return;
}

// static int Uart_Receive_Frame(uint8_t *expectedResponse) 
// {
//     int rdlen;
//     uint8_t rx_buf[80];
//     for (;;) {
//         rdlen = read(uart_handle, rx_buf, sizeof(rx_buf) - 1);
//         if (rdlen > 0) {
//             rx_buf[rdlen] = 0;
//             fprintf(stdout, "Read %d: \"%s\", ", rdlen, rx_buf);

//             unsigned char *p;
//             for (p = rx_buf; rdlen-- > 0; p++)
//                 printf(" %02x", *p);
//             printf("\n");
//         } else if (rdlen < 0) {
//             LD_ERROR_OUT("Failed to read uart %")
//         }
//     }
// }





/*

int push_ledcontrol_request(unsigned long R, unsigned long G, unsigned long B)
{
	LD_PRINTF_OUT("Pushing led request with color- R: %d, G: %d, B: %d", R, G, B);

	uint8_t tx_buf[20] = {0x4c, 0x53, R, G, B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};
	int rdlen;
	uint8_t rx_buf[80];

	tx_buf[19] = generate_crc(tx_buf, 19);
	Uart_Send_Frame(tx_buf, 20);

	rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
	if(rdlen > 0) {
		rx_buf[rdlen] = 0; 
		LD_PRINTF_OUT("Read %d: \"%s\", ", rdlen, rx_buf);

        // unsigned char *p;
        // for (p = rx_buf; rdlen-- > 0; p++)
        //     printf(" %02x", *p);
        // printf("\n");
	} else {
		LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
	}
}
*/


void push_to_ATtiny(unsigned int mode,unsigned long submode, unsigned int power1,unsigned int power2, unsigned long time1)
{

    uint8_t T4     = (time1 & 0x000000FF);
    uint8_t T3     = (time1 & 0x0000FF00) >> 8;
    uint8_t T2     = (time1 & 0x00FF0000) >> 16;
    uint8_t T1     = (time1 & 0xFF000000) >> 24;

   uint8_t tx_buf[20] = {mode, submode, power1, power2 , T1, T2, T3, T4, 0x00, 0, 5, 29, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};
  

    int rdlen;
    uint8_t rx_buf[80];

    tx_buf[19] = generate_crc(tx_buf, 19);
    Uart_Send_Frame(tx_buf, 20);

    rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
    if(rdlen > 0) {
        rx_buf[rdlen] = 0;
        LD_PRINTF_OUT("Read %d: \"%s\", ", rdlen, rx_buf);

    } else {
        LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
    }
}




int push_ledcontrol_request(unsigned long mode, unsigned long submode, unsigned long R, unsigned long G, unsigned long B,unsigned long t)
{	
    uint8_t t4=(t   & 0x000000ff);
    uint8_t t3=((t  & 0x0000ff00)  >>  8);
    uint8_t t2=((t  & 0x00ff0000)    >> 16);
    uint8_t t1=((t  & 0xff000000)      >> 24) ;


    LD_PRINTF_OUT("Pushing led request with color- R: %lu, G: %lu, B: %lu T: %lu- T1-%d  T2-%d T3-%d T4-%d", R, G, B,t,t1,t2,t3,t4);
    uint8_t tx_buf[20] = {mode, submode, R, G, B, 0x00, 0x00, 0x00, t1, t2, t3, t4, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};
 
//   uint8_t tx_buf[20]  = {'L' ,  'B',     0, 120, 0, 0, 0, 0, 0, 20,0,29,45,45,45,45,45,45,45,50};
   int rdlen;
    uint8_t rx_buf[80];

    tx_buf[19] = generate_crc(tx_buf, 19);
    Uart_Send_Frame(tx_buf, 20);

    rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
    if(rdlen > 0) {
        rx_buf[rdlen] = 0; 
       // printf("Read %d: \"%s\", ", rdlen, rx_buf);

        // unsigned char *p;
        // for (p = rx_buf; rdlen-- > 0; p++)
        //     printf(" %02x", *p);
        // printf("\n");
    } else {
        LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
    }
}

int push_pwm_led_control_request(unsigned long mode, unsigned long submode, unsigned long Rs, unsigned long Gs, unsigned long Bs,unsigned long Re, unsigned long Ge, unsigned long Be, unsigned long time)
{
 //   printf("Pushing led request with color- R: %lu, G: %lu, B: %lu\n", R, G, B);


    uint8_t T4     = (time & 0x000000FF);
    uint8_t T3     = (time & 0x0000FF00) >> 8;
    uint8_t T2     = (time & 0x00FF0000) >> 16;
    uint8_t T1     = (time & 0xFF000000) >> 24;

    uint8_t tx_buf[20] = {mode, submode, Rs, Gs, Bs, Re, Ge, Be, T1, T2, T3, T4, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};

   int rdlen;
    uint8_t rx_buf[80];

    tx_buf[19] = generate_crc(tx_buf, 19);
    Uart_Send_Frame(tx_buf, 20);

    rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
    if(rdlen > 0) {
        rx_buf[rdlen] = 0;
      //  printf("Read %d: \"%s\", ", rdlen, rx_buf);

        // unsigned char *p;
        // for (p = rx_buf; rdlen-- > 0; p++)
        //     printf(" %02x", *p);
        // printf("\n");
    } else {
        LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
    }
}



int mgmtcontrol_getInfo(void)
{
	
	LD_PRINTF_OUT("Pushing mgmt request");

	uint8_t tx_buf[20] = {'O', 'V', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};
	int rdlen;
	uint8_t rx_buf[80];

	tx_buf[19] = generate_crc(tx_buf, 19);
	Uart_Send_Frame(tx_buf, 20);

	rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
	if(rdlen > 0) {
		rx_buf[rdlen] = 0; 
		LD_PRINTF_OUT("Read(2) %d: \"%X %X\", ", rdlen, rx_buf[2],rx_buf[3]);
			FILE * fp;
	int i;
	fp=fopen("/tmp/test.txt", "w");
	for (i=0;i<rdlen;i++) {
		fprintf(fp,"0x%02x ", rx_buf[i]);
	}
	fprintf(fp,"\n");
	//fprintf(fp,"version %X %X",rdlen,rx_buf[2],rx_buf[3]);
	fclose(fp);

        // unsigned char *p;
        // for (p = rx_buf; rdlen-- > 0; p++)
        //     printf(" %02x", *p);
        // printf("\n");
	} else {
		LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
	}
}



int init_ledcontrol(char *path)
{
	struct termios newtio;
	struct serial_struct newserial;
	float baud_error = 0.0;

	LD_DBG_OUT("Initializing %s", uart_port_name);
    /*
       Open device for reading and writing.
       Blocking mode - more CPU effecient
     */
	uart_handle = open(uart_port_name, O_RDWR | O_NOCTTY | O_NDELAY  );
	if (uart_handle < 0) {
		perror(uart_port_name);
		return 0;
	}
// #if 0
    /* non blocking for the read */
	fcntl(uart_handle, F_SETFL, FNDELAY);
// #else
//     /* efficient blocking for the read */
//     fcntl(uart_handle, F_SETFL, 0);
// #endif
    /* save current serial port settings */
	tcgetattr(uart_handle, &uart_oldtio);
    /* we read the old serial setup */
	ioctl(uart_handle, TIOCGSERIAL, &uart_oldserial);
    /* we need a copy of existing settings */
	memcpy(&newserial, &uart_oldserial, sizeof(struct serial_struct));
    /* clear struct for new port settings */
	bzero(&newtio, sizeof(newtio));
    /*
       BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
       CRTSCTS : output hardware flow control (only used if the cable has
       all necessary lines. See sect. 7 of Serial-HOWTO)
       CS8     : 8n1 (8bit,no parity,1 stopbit)
       CLOCAL  : local connection, no modem contol
       CREAD   : enable receiving characters
     */
	newtio.c_cflag = uart_baud | CS8 | CLOCAL | CREAD;
    /* Raw input */
	newtio.c_iflag = 0;
    /* Raw output */
	newtio.c_oflag = 0;
    /* no processing */
	newtio.c_lflag = 0;
    /* activate the settings for the port after flushing I/O */
	tcsetattr(uart_handle, TCSAFLUSH, &newtio);
	LD_DBG_OUT(" at Baud Rate %u", Uart_Get_Baud_Rate());
    /* destructor */
	atexit(Uart_Cleanup);
    /* flush any data waiting */
	usleep(200000);
	tcflush(uart_handle, TCIOFLUSH);
	LD_DBG_OUT("=success!\n");
}

//Watchdog packets
int watchdog_enable()
{    
	LD_PRINTF_OUT("Got enable watchdog command");

	uint8_t tx_buf[20] = {0x57, 0x45, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};
	int rdlen;
	uint8_t rx_buf[80];

	tx_buf[19] = generate_crc(tx_buf, 19);
	Uart_Send_Frame(tx_buf, 20);

	rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
	if(rdlen > 0) {
		rx_buf[rdlen] = 0; 
		LD_PRINTF_OUT("Read %d: \"%s\", ", rdlen, rx_buf);

        // unsigned char *p;
        // for (p = rx_buf; rdlen-- > 0; p++)
        //     printf(" %02x", *p);
        // printf("\n");
	} else {
		LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
	}
}

int watchdog_disable()
{    
	LD_PRINTF_OUT("Got disable watchdog command");

	uint8_t tx_buf[20] = {0x57, 0x44, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};
	int rdlen;
	uint8_t rx_buf[80];

	tx_buf[19] = generate_crc(tx_buf, 19);
	Uart_Send_Frame(tx_buf, 20);

	rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
	if(rdlen > 0) {
		rx_buf[rdlen] = 0; 
		LD_PRINTF_OUT("Read %d: \"%s\", ", rdlen, rx_buf);

        // unsigned char *p;
        // for (p = rx_buf; rdlen-- > 0; p++)
        //     printf(" %02x", *p);
        // printf("\n");
	} else {
		LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
	}

}

int watchdog_heartbeat()
{    
	LD_PRINTF_OUT("Got heartbeat watchdog command");

	uint8_t tx_buf[20] = {0x57, 0x48, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};
	int rdlen;
	uint8_t rx_buf[80];

	tx_buf[19] = generate_crc(tx_buf, 19);
	Uart_Send_Frame(tx_buf, 20);

	rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
	if(rdlen > 0) {
		rx_buf[rdlen] = 0; 
		LD_PRINTF_OUT("Read %d: \"%s\", ", rdlen, rx_buf);

        // unsigned char *p;
        // for (p = rx_buf; rdlen-- > 0; p++)
        //     printf(" %02x", *p);
        // printf("\n");
	} else {
		LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
	}

}

int watchdog_expirenow()
{    
	LD_PRINTF_OUT("Got expirenow watchdog command");

	uint8_t tx_buf[20] = {0x57, 0x58, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};
	int rdlen;
	uint8_t rx_buf[80];

	tx_buf[19] = generate_crc(tx_buf, 19);
	Uart_Send_Frame(tx_buf, 20);

	rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
	if(rdlen > 0) {
		rx_buf[rdlen] = 0; 
		LD_PRINTF_OUT("Read %d: \"%s\", ", rdlen, rx_buf);

        // unsigned char *p;
        // for (p = rx_buf; rdlen-- > 0; p++)
        //     printf(" %02x", *p);
        // printf("\n");
	} else {
		LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
	}

}

void push_to_ATtiny_piezo(uint8_t mode,unsigned long submode, uint16_t freq, uint32_t time1)
{
	LD_PRINTF_OUT("Pushing peizo request with Frequency and Time- F: %lu, T: %l\n",freq,time1);

//    freq_l = freq & 0x00ff;
//    freq_h = freq >>8;
//   printf("l-%lu u-%iu",freq_l,freq_h);
   uint8_t freq_l = freq & 0x00ff;
   uint8_t freq_h = freq >>8;
//    time1 = 1000;
   uint8_t T4     = (time1 & 0x000000FF);
   uint8_t T3     = (time1 & 0x0000FF00) >> 8;
   uint8_t T2     = (time1 & 0x00FF0000) >> 16;
   uint8_t T1     = (time1 & 0xFF000000) >> 24;

//   printf("l-%lu u-%iu",freq_l,freq_h);
  uint8_t tx_buf[20] = {mode, submode, freq_h, freq_l , T1, T2, T3, T4, 0x00, 0, 5, 29, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};

 //   uint8_t tx_buf[20] = {'P', 'b', 20, 0, 1, 255, 0, 0, 0x00, 0, 5, 29, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};

   int rdlen;
   uint8_t rx_buf[80];

   tx_buf[19] = generate_crc(tx_buf, 19);
   Uart_Send_Frame(tx_buf, 20);

   rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
   if(rdlen > 0) {
       rx_buf[rdlen] = 0;
       LD_PRINTF_OUT("Read %d: \"%s\", ", rdlen, rx_buf);

       // unsigned char *p;
       // for (p = rx_buf; rdlen-- > 0; p++)
       //     printf(" %02x", *p);
       // printf("\n");
   } else {
       LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
   }
}


void PROCESS_THREAD(char *p) 
{
	//char *p=(char*)threadpara;

//        static char *p=0;
	static uint8_t *d;
	uint8_t default_dur = 4;
	static uint8_t default_oct = 6;
	int bpm = 63;
	int num;
	static float beat;
	long wholenote;
	static float duration = 0;
	uint8_t note;
	uint8_t scale;
	static char message[7]={0};  
	arrindex=0;

	while(1)
	{

	     /* switch(a)
		{
			case 1:
		            p = "Indiana:d=4,o=5,b=250:e,8p,8f,8g,8p,1c6,8p.,d,8p,8e,1f,p.,g,8p,8a,8b,8p,1f6,p,a,8p,8b,2c6,2d6,2e6,e,8p,8f,8g,8p,1c6,p,d6,8p,8e6,1f.6,g,8p,8g,e.6,8p,d6,8p,8g,e.6,8p,d6,8p,8g,f.6,8p,e6,8p,8d6,2c6";
			break;

			case 2:
			   p = "Entertainer:d=4,o=5,b=140:8d,8d#,8e,c6,8e,c6,8e,2c.6,8c6,8d6,8d#6,8e6,8c6,8d6,e6,8b,d6,2c6,p,8d,8d#,8e,c6,8e,c6,8e,2c.6,8p,8a,8g,8f#,8a,8c6,e6,8d6,8c6,8a,2d6";
			break;			

			case 3:
			  starWars:d=4,o=5,b=45:32p,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#.6,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#6;
			break;
		}*/
	

		// char *p ="Bond:d=4,o=5,b=80:32p,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d#6,16d#6,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d6,16c#6,16c#7,c.7,16g#6,16f#6,g#.6";
		// static char *p = "Flinstones:d=4,o=5,b=40:32p,16f6,16a#,16a#6,32g6,16f6,16a#.,16f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c6,d6,16f6,16a#.,16a#6,32g6,16f6,16a#.,32f6,32f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c6,a#,16a6,16d.6,16a#6,32a6,32a6,32g6,32f#6,32a6,8g6,16g6,16c.6,32a6,32a6,32g6,32g6,32f6,32e6,32g6,8f6,16f6,16a#.,16a#6,32g6,16f6,16a#.,16f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c.6,32d6,32d#6,32f6,16a#,16c.6,32d6,32d#6,32f6,16a#6,16c7,8a#.6";
		// static char *p = "MissionImp:d=16,o=6,b=95:32d,32d#,32d,32d#,32d,32d#,32d,32d#,32d,32d,32d#,32e,32f,32f#,32g,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,a#,g,2d,32p,a#,g,2c#,32p,a#,g,2c,a#5,8c,2p,32p,a#5,g5,2f#,32p,a#5,g5,2f,32p,a#5,g5,2e,d#,8d";
		// static char *p = "A-Team:d=8,o=5,b=125:4d#6,a#,2d#6,16p,g#,4a#,4d#.,p,16g,16a#,d#6,a#,f6,2d#6,16p,c#.6,16c6,16a#,g#.,2a#";
		// static char *p =  "GoodBad:d=4,o=5,b=56:32p,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,4d#,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#


		while(*p != ':') p++;    // ignore name
		p++;                     // skip ':'

		// get default duration
		if(*p == 'd')
		{
			p++; p++;              // skip "d="
			num = 0;
			
			while(isdigit(*p))
			{
				num = (num * 10) + (*p++ - '0');
			}
			if(num > 0)
			{
				default_dur = num;
			}
			p++;                   // skip comma
		}
		// get default octave
		if(*p == 'o')
		{
			p++; p++;              // skip "o="
			num = *p++ - '0';
			if(num >= 3 && num <=7) default_oct = num;
			p++;                   // skip comma
		}
		// get BPM
		if(*p == 'b'){
			p++; p++;              // skip "b="
			num = 0;
			while(isdigit(*p)){
				num = (num * 10) + (*p++ - '0');
			}
			bpm = (num * 8) /60;
			beat = bpm;
			p++;                   // skip colon
		}
		// BPM usually expresses the number of quarter notes per minute
		wholenote = (60 * 1000L / bpm) * 4;  // this is the time for whole note (in milliseconds)
		// now begin note loop
		while(*p){
			// first, get note duration, if available
			num = 0;
			while(isdigit(*p)){
				num = (num * 10) + (*p++ - '0');
			}

			if(num){
				if(num == 32)
				duration =  .5;
				if(num == 16)
				duration =  1;
				if(num == 8)
				duration =  2;
			}
			else duration = default_dur;  // we will need to check if we are a dotted note after

			if(duration > 4)
			duration = 4;

			note = 0;

			switch(*p){
				case 'c':
				note = 1;
				break;
				case 'd':
				note = 3;
				break;
				case 'e':
				note = 5;
				break;
				case 'f':
				note = 6;
				break;
				case 'g':
				note = 8;
				break;
				case 'a':
				note = 10;
				break;
				case 'b':
				note = 12;
				break;
				case 'p':
				default:
				note = 0;
			}
			p++;

			// now, get optional '#' sharp
			if(*p == '#'){
				note++;
				p++;
			}

			// now, get optional '.' dotted note
			if(*p == '.'){
				duration += duration/2;
				p++;
			}

			// now, get scale
			if(isdigit(*p))
			{
				scale = *p - '0';
				p++;
			}
			else
			{
				scale = default_oct;
			}
			scale += OCTAVE_OFFSET;
			if(*p == ',')
			p++;       // skip comma for next note (or we may be at the end)

			//		    	printf("Playing: %i",scale);
			//		    	printf("note  %i\n",note			//		    	printf("end p  %c\n", *p);
			//		    	printf("end p  %d\n", p);
			//		    	printf(" (%i",notess[(scale - 4) * 12 + note]);
			//		    	printf("duration %i\n ",duration);
			//		    	printf("beat %i\n ",beat);

			notes_array[arrindex]=(int)(notess[(scale - 4) * 12 + note]);
			notes_duration[arrindex]=(int)((((64)*duration)*3/beat)*3);
			arrindex++;
			//push_to_ATtiny('P','0',(int)((((64)*duration)*3/beat)*3),(notess[(scale - 4) * 12 + note]));
			//push_to_ATtiny_piezo('P','0',(notess[(scale - 4) * 12 + note]),(int)((((64)*duration)*3/beat)*3));
			//usleep((int)(((64)*duration)*3/beat)*5000);
			
		}
	if(strlen(p)==0)
	{
	arrindex=0;
		
//	pthread_exit(NULL);
	break;	
	}
}}

void *play(void *threadid)
{
	while(1)
	{	
		push_to_ATtiny_piezo('P','0',(notes_array[arrindex]),notes_duration[arrindex]);
		usleep(notes_duration[arrindex]*2500);

	if(notes_array[arrindex]==NULL)
	{
	thread_flag=0;
	pthread_exit(NULL);
	break;
	}
	arrindex++;
}
}


void getRing(int tone_num)
{
	
	FILE * fp;
	char * line = NULL;
    size_t len = 0;
    ssize_t read;
    char count=1;
    char *token;
    char tone[200];

	fp = fopen("/wigwag/wwrelay-utils/conf/rp200_tones.txt", "r");
    if (fp == NULL){
    	LD_ERROR_OUT("Failed to open file rp200_tones %s\n", strerror(errno));
      //  exit(EXIT_FAILURE);
    } else {
        while ((read = getline(&line, &len, fp)) != -1) {
          	token = strtok(line, "\n");
          	if(tone_num==count)
		{
			PROCESS_THREAD(line);
			if(!thread_flag)
			{
				thread_flag = pthread_create(&threads[1], NULL, play,(void*)1);
				thread_flag = 1;
			}
       			
			//PROCESS_THREAD(line);
       			return;
       		}
		

       	    count++;
        }
        fclose(fp);
    }
}

void watchdog_set(unsigned long t)
{
    LD_PRINTF_OUT("Setting watchdog for %d",t);
       uint8_t t4=(t   & 0x000000ff);
    uint8_t t3=((t  & 0x0000ff00)  >>  8);
    uint8_t t2=((t  & 0x00ff0000)    >> 16);
    uint8_t t1=((t  & 0xff000000)      >> 24) ;
    LD_PRINTF_OUT("Time = %d,T1- %d,T2- %d,T3- %d,T4- %d",t,t1,t2,t3,t4); 
    uint8_t tx_buf[20] = {0x57, 0x53, t1, t2,t3, t4, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 54};
    int rdlen;
    uint8_t rx_buf[80];

    tx_buf[19] = generate_crc(tx_buf, 19);
    Uart_Send_Frame(tx_buf, 20);

    rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
    if(rdlen > 0) {
        rx_buf[rdlen] = 0; 
        LD_PRINTF_OUT("Read %d: \"%s\", ", rdlen, rx_buf);

        // unsigned char *p;
        // for (p = rx_buf; rdlen-- > 0; p++)
        //     printf(" %02x", *p);
        // printf("\n");
    } else {
        LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
    }
}

void watchdog_timeremaining()
{
   // printf("\n\nTime remaining ");

    uint8_t tx_buf[20] = {'W', 'A', 0x00, 0x00, 0, 0, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 33};
    int rdlen;
    uint8_t rx_buf[80];

    tx_buf[19] = generate_crc(tx_buf, 19);
    Uart_Send_Frame(tx_buf, 20);

    rdlen = Uart_Receive_Frame(uart_handle, rx_buf, sizeof(rx_buf) - 1, 5);
  
  if(rdlen > 0) {
        rx_buf[rdlen] = 0; 
     //   printf("is %d\n", rx_buf);
       // LD_PRINTF_OUT("\n\nREAD ");
  //     unsigned char *p;
   //     for (p = rx_buf; rdlen-- > 0; p++)
//	{
 //           printf("%d,", *p);
//	}
        
	LD_PRINTF_OUT("\n\nTime remaining ");

	LD_PRINTF_OUT("%d",(((((uint32_t)rx_buf[2] << 24) | ((uint32_t)rx_buf[3] << 16)) | ((uint32_t)rx_buf[4] << 8)) | (uint32_t)rx_buf[5]));

/*	uint32_t r_time = 0;

	for (p = rx_buf+2; rx_buf+6 < p; p++)
        {
 	 r_time             
        }
*/
            LD_PRINTF_OUT("\n");
    } else {
        LD_ERROR_OUT("Error from read: %d: %s\n", rdlen, strerror(errno));
    }
}

#if TEST
int main()
{
	int rdlen;
	uint8_t rx_buf[80];
	uint8_t tx_buf[20] = {0x4c, 0x53, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x3c};
	Uart_Init();
	Uart_Send_Frame(tx_buf, 20);
	for (;;) {
		rdlen = read(uart_handle, rx_buf, sizeof(rx_buf) - 1);
		if (rdlen > 0) {
			rx_buf[rdlen] = 0;
			fprintf(stdout, "Read %d: \"%s\", ", rdlen, rx_buf);

			unsigned char *p;
			for (p = rx_buf; rdlen-- > 0; p++)
				printf(" %02x", *p);
			printf("\n");
		} else if (rdlen < 0) {
			fprintf(stderr, "Error from read: %d: %s\n", rdlen, strerror(errno));
		}
	}

	return 0;
}
#endif
