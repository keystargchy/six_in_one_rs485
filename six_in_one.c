#include <stdio.h>   
#include <error.h>  
#include <stdint.h>  
#include <stdlib.h>
#include <unistd.h>     /*Unix标准函数定义*/
#include <sys/types.h>  /**/
#include <sys/stat.h>   /**/
#include <sys/wait.h>	/*waitpid*/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <getopt.h>
#include <string.h>
#include <signal.h> 

#include "six_in_one.h"

/* Word control code */
#define WD_NONE()           "\033[0m"     /* Close all property */
#define WD_HIGHLIGHT()      "\033[1m"     /* High light */
#define WD_UNDERLINE()      "\033[4m"     /* Under line */
#define WD_FLICKER()        "\033[5m"     /* Flicker */
#define WD_RVSPRN()         "\033[7m"     /* Reverse display */
#define WD_FADE()           "\033[8m"     /* Fade */

/* words colour */
#define FG_BLACK()          "\033[30m"    /* Black */
#define FG_RED()            "\033[31m"    /* Red */
#define FG_GREEN()          "\033[32m"    /* Green */
#define FG_YELLOW()         "\033[33m"    /* Yellow */
#define FG_BLUE()           "\033[34m"    /* Blue */
#define FG_PURPLE()         "\033[35m"    /* Purple */
#define FG_DGREEN()         "\033[36m"    /* Dark Green */
#define FG_WHITE()          "\033[37m"    /* White */

#define FALSE 1
#define TRUE 0

//pthread_mutex_t mutex; 

char *recchr="We received:\"";

int speed_arr[] = { 
	B921600, B460800, B230400, B115200, B57600, B38400, B19200, 
	B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, 
	B4800, B2400, B1200, B300, 
};
int name_arr[] = {
	921600, 460800, 230400, 115200, 57600, 38400,  19200,  
	9600,  4800,  2400,  1200,  300, 38400,  19200,  9600, 
	4800, 2400, 1200,  300, 
};

unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};


unsigned char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};

uint32_t CRC_Compute(uint8_t * pushMsg,uint8_t usDataLen)
{
	uint8_t uchCRCHi = 0xff;//高 CRC 字节初始化
	uint8_t uchCRCLo = 0xff;//低 CRC 字节初始化
	uint8_t uIndex; //CRC 循环中的索引
	while(usDataLen--)
	{
		uIndex = uchCRCLo^ *pushMsg++;//计算 CRC
		uchCRCLo = uchCRCHi^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (((uint8_t)uchCRCHi) << 8 | uchCRCLo);
}


void set_speed(int fd, int speed)
{
	int   i;
	int   status;
	struct termios   Opt;
	tcgetattr(fd, &Opt);

	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
		if  (speed == name_arr[i])	{
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &Opt);
			if  (status != 0)
				perror("tcsetattr fd1");
				return;
		}
		tcflush(fd,TCIOFLUSH);
   }
}

int set_Parity(int fd,int databits,int stopbits,int parity,int flowctrl)
{
	struct termios options;
	if  ( tcgetattr( fd,&options)  !=  0) {
		perror("SetupSerial 1");
		return(FALSE);
	}
	options.c_cflag &= ~CSIZE ;
	switch (databits) /*设置数据位数*/ {
	case 7:
		options.c_cflag |= CS7;
	break;
	case 8:
		options.c_cflag |= CS8;
	break;
	default:
		fprintf(stderr,"Unsupported data size\n");
		return (FALSE);
	}
	
	switch (parity) {
	case 'n':
	case 'N':
		options.c_cflag &= ~PARENB;   /* Clear parity enable */
		options.c_iflag &= ~INPCK;     /* Enable parity checking */
	break;
	case 'o':
	case 'O':
		options.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/
		options.c_iflag |= INPCK;             /* Disnable parity checking */
	break;
	case 'e':
	case 'E':
		options.c_cflag |= PARENB;     /* Enable parity */
		options.c_cflag &= ~PARODD;   /* 转换为偶效验*/ 
		options.c_iflag |= INPCK;       /* Disnable parity checking */
	break;
	case 'S':	
	case 's':  /*as no parity*/
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
	break;
	default:
		fprintf(stderr,"Unsupported parity\n");
		return (FALSE);
	}
 	/* 设置停止位*/  
  	switch (stopbits) {
   	case 1:
    	options.c_cflag &= ~CSTOPB;
  	break;
 	case 2:
  		options.c_cflag |= CSTOPB;
  	break;
 	default:
  		fprintf(stderr,"Unsupported stop bits\n");
  		return (FALSE);
 	}
	/* Set flow control */
	if (flowctrl)
		options.c_cflag |= CRTSCTS;
	else
		options.c_cflag &= ~CRTSCTS;

  	/* Set input parity option */
  	if (parity != 'n')
    	options.c_iflag |= INPCK;
  	options.c_cc[VTIME] = 150; // 15 seconds
    options.c_cc[VMIN] = 0;

	options.c_lflag &= ~(ECHO | ICANON);
	options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);  //发送0x11 0x13

    options.c_oflag &= ~OPOST;  
    options.c_cflag |= CLOCAL | CREAD;  
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 



  	tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
  	if (tcsetattr(fd,TCSANOW,&options) != 0) {
    	perror("SetupSerial 3");
  		return (FALSE);
 	}
	return (TRUE);
}

int OpenDev(char *Dev)
{
	int fd = open( Dev, O_RDWR );         //| O_NOCTTY | O_NDELAY
 	if (-1 == fd) { /*设置数据位数*/
   		perror("Can't Open Serial Port");
   		return -1;
	} else
		return fd;
}

const char * program_name;

void print_usage (FILE *stream, int exit_code)
{
    fprintf(stream, "Usage: %s option [ dev... ] \n", program_name);
    fprintf(stream,
            "\t-h  --help     Display this usage information.\n"
            "\t-d  --device   The device ttyS[0-3] or ttyEXT[0-3]\n"
            "\t-s  --string   Write the device data\n"
	    "\t-f  --flow     Flow control switch\n"
	    "\t-b  --speed    Set speed bit/s\n");
    exit(exit_code);
}

void do_sig(int a)  
{  
    printf("\nexit\n"); 

    exit(0); 
} 


uint16_t crc_send = 0xFFFF; 
uint16_t crc_resp = 0xFFFF; 
uint8_t addrflag = 0x0;
uint8_t dataReadAddr[8] = {0xFE, 0x17, 0x0, 0x0, 0x0, 0x01};
uint8_t dataRespAddr[7];
uint8_t dataReadValue[8]= {0x0, 0x03, 0x0, 0x0, 0x0, 0x07};
uint8_t dataRespValue[19];

uint16_t co_2;    	    //二氧化碳
float tvoc;			    //总挥发性有机物
float ch_2_o;		    //甲醛
uint16_t pm_25;		    //PM2.5
uint16_t pm_10;		    //PM10
uint16_t humidity;		//湿度
float temperature;	    //温度


int main(int argc, char *argv[])  
{  
#define BUF_SIZE  256
	int i = 0;  

    int  fd, next_option, havearg = 0;
    char *device = "/dev/ttySAC2"; /* Default device */
	int speed = 9600;
	int flowctrl = 0;

 	int nread = 0, nwrite = 0;			/* Read the counts of data */
 	uint8_t value_buff[BUF_SIZE] = {0};
 	int n = 0;


    uint8_t *ptrDataValue = &dataRespValue[3];

 	if (signal(SIGINT, do_sig) == SIG_ERR) {  
        perror("signal");  
        exit(1);  
    }

 	fd = OpenDev(device);

 	if (fd > 0) {
      	set_speed(fd,speed);
 	} else {
	  	fprintf(stderr, "Error opening %s: %s\n", device, strerror(errno));
  		exit(1);
 	}
	if (set_Parity(fd,8,1,'N', flowctrl)== FALSE) {
		fprintf(stderr, "Set Parity Error\n");
		close(fd);
      	exit(1);
   	}

   	crc_send = CRC_Compute(dataReadAddr, sizeof(dataReadAddr)/sizeof(uint8_t) - 2);
	*(dataReadAddr+6) = crc_send % 256;  
	*(dataReadAddr+7) = crc_send / 256; 
	nwrite = write(fd, dataReadAddr, sizeof(dataReadAddr));
	
	printf(FG_GREEN()"\nSend Addr:\n");
	for(i = 0; i < sizeof(dataReadAddr); i++)
		printf("%#02x ", dataReadAddr[i]);
	printf("\n"WD_NONE());

	nread = read(fd, dataRespAddr, sizeof(dataRespAddr));
	printf(FG_YELLOW()"\nResp Addr:\n");
	for(i = 0; i < sizeof(dataRespAddr); i++)
		printf("%#02x ", dataRespAddr[i]);
	printf("nread = %d\n"WD_NONE(), nread);
	if(nread != sizeof(dataRespAddr))
	{
		perror(FG_RED()"read invalue Addr1"WD_NONE());
		exit(-1);
	}

	crc_resp = CRC_Compute(dataRespAddr, sizeof(dataRespAddr)/sizeof(uint8_t) -2);
	printf("%#04x  %#04x\n", (((uint16_t)dataRespAddr[6] << 8) | dataRespAddr[5]), crc_resp);
	if((((uint16_t)dataRespAddr[6] << 8) | dataRespAddr[5]) != crc_resp)
	{
		perror(FG_RED()"read invalue Addr2"WD_NONE());
		exit(-1);
	}
	addrflag = dataRespAddr[4];

	
	int times = 0;
	while(1)
	{
		static int flag = 0;
		crc_send = 0xFFFF;
		dataReadValue[0] = addrflag;
		crc_send = CRC_Compute(dataReadValue, sizeof(dataReadValue)/sizeof(uint8_t) - 2);
		*(dataReadValue+6) = crc_send % 256;  
		*(dataReadValue+7) = crc_send / 256; 

		while(flag)
		{
			sleep(1);
			flag = 0;
		}
		flag = 1;
		write(fd, dataReadValue, sizeof(dataReadValue));
		
		printf(FG_GREEN()"\nSend Value:\n");
		for(i = 0; i < sizeof(dataReadValue); i++)
			printf("%#02x ", dataReadValue[i]);
		printf("\n"WD_NONE());
	


		printf("\nPrepare requiring data:\n");
		printf("第 %d 次\n", ++times);

		crc_resp = 0xFFFF;
		nread = read(fd, dataRespValue, sizeof(dataRespValue));
		crc_resp = CRC_Compute(dataRespValue, sizeof(dataRespValue)/sizeof(uint8_t) - 2);
	
		if(nread == 0)
		{
			printf(FG_BLUE()"Valid value: \n");
    		for(i = 3; i < 17; i++)
    			printf("%#02x ", dataRespValue[i]);
    		printf("\n"WD_NONE());
		}
//#define CRC
#ifdef CRC
		if((((uint16_t)dataRespValue[18] << 8) | dataRespValue[17]) != crc_resp)
    	{
    		perror(FG_RED()"\nRead invalue Value"WD_NONE());
    		printf("%#04x  %#04x\n", (((uint16_t)dataRespValue[18] << 8) | dataRespValue[17]), crc_resp);
    		exit(-1);
    	}  
#endif
    	printf(FG_YELLOW()"\nValid value: \n");
    	for(i = 3; i < 17; i++)
    		printf("%#02x ", dataRespValue[i]);
    	printf("\n"WD_NONE());

    	handle_data(ptrDataValue, 7);

    	printf(FG_BLUE());
    	printf("%-15s", "CO2 (ppm)");
		printf(":   %d\n", co_2);
		printf("%-15s", "TVOC(ug/m3)");
		printf(":   %.1f\n", tvoc);
		printf("%-15s", "CH2O(ug/m3)");
		printf(":   %.1f\n", ch_2_o);
		printf("%-15s", "PM2.5(ug/m3)");
		printf(":   %d\n", pm_25);
		printf("%-15s", "HUM(%RH)");
		printf(":   %d\n", humidity);
		printf("%-15s", "TEM(℃)");
		printf(" :   %.2f\n", temperature);
		printf("%-15s", "PM10(ug/m3)");
		printf(":   %d\n", pm_10);
    	printf(WD_NONE());
	}

    return 0;
}  

void handle_data(uint8_t *ptr, int len)
{
	int srh = 0;
	float stem = 0.0;

	co_2 = (uint16_t)CO2_H << 8 | CO2_L;

	tvoc = ((uint16_t)TVOC_H << 8 | TVOC_L)/10.0;

	ch_2_o = ((uint16_t)CH2O_H << 8 | CH2O_L)/10.0;

	pm_25 = (uint16_t)PM25_H << 8 | PM25_L;

	srh = 125 * ((uint16_t)HUMIDITY_H << 8 | HUMIDITY_L);
	humidity = -6 + (srh >> 16);

	stem = 175.72 * ((uint16_t)TEMPERATURE_H << 8 | TEMPERATURE_L);
	temperature = -46.85 + stem / 65536.00;

	pm_10 = (uint16_t)PM10_H << 8 | PM10_L;
}


