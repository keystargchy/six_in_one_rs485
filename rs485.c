#include <fcntl.h>     //文件控制定义  
#include <stdio.h>     //标准输入输出定义  
#include <stdlib.h>     //标准函数库定义  
#include <unistd.h>    //Unix标准函数定义   
#include <errno.h>     //错误好定义  
#include <termios.h>   //POSIX终端控制定义  
#include <sys/ioctl.h>   //ioctl函数定义  
#include <string.h>     //字符操作  
#include <sys/types.h>    
#include <sys/stat.h>   
#include <pthread.h>  
#include <sys/timeb.h>  
  
//时间戳  
long long getSystemTime() {  
    struct timeb t;  
    ftime(&t);  
    return 1000 * t.time + t.millitm;  
}  
  
 long long start;  
 long long end;  
  
  
//定义互斥量  
pthread_mutex_t mutex;  
int fd_gpio;  
  
struct termios newtio, oldtio;  
typedef struct {  
        int  pin_idx;  
        int  pin_dir;  
        int  pin_sta;  
} davinci_gio_arg;  
  
typedef enum {  
        AT91PIO_DIR_OUT = 0,  
        AT91PIO_DIR_INP   
} davinci_gio_dir;  
//驱动判断输入输出模式  
  
davinci_gio_arg arg;  
  
#define DEV_PIO_LED "/dev/pio"  
// 需要手动添加设备号 mknod /dev/pio c 203 0  
#define PIO_NUM 47  
// 47pin 为控制输入输出方向引脚  
#define DEV_UART    "/dev/ttyS1"  
// /dev/ttyS1 为串口设备  
  
#define IOCTL_PIO_SETDIR    1       //set gpio direct  
#define IOCTL_PIO_GETDIR    2       //get gpio direct  
#define IOCTL_PIO_SETSTA    3       //set gpio status  
#define IOCTL_PIO_GETSTA    4       //get gpio status  
  
//保存信息  
int log_init( const char *strFileName )  
{  
    int fdLog = -1;  
  
    if( -1 == (fdLog = open( strFileName,  O_CREAT|O_TRUNC ) ) )  
    {  
    }  
    close( fdLog );  
}  
  
int log_out( const char *strFileName, const char * szLog )  
{  
    int fdLog = -1;  
  
    if( -1 == ( fdLog = open( strFileName,  O_CREAT|O_WRONLY|O_APPEND ) ) )  
    {  
        printf( "LOG (%s) open error!\n", strFileName );  
        return -1;  
    }  
  
    write( fdLog, szLog, strlen( szLog ) );  
  
    close( fdLog );  
  
    return 0;  
}  
  
//配置串口  
/* 参数说明：fd 设备文件描述符，nspeed 波特率，nbits 数据位数（7位或8位）， 
            parity 奇偶校验位（'n'或'N'为无校验位，'o'或'O'为偶校验，'e'或'E'奇校验）， 
            nstop 停止位（1位或2位） 
  成功返回1，失败返回-1。  
*/  
int set_com_opt( int fd, int nspeed, int nbits, char parity, int nstop )  
{  
    char szTmp[128];  
//打印配置信息  
    sprintf( szTmp, "set_com_opt - speed:%d,bits:%d,parity:%c,stop:%d\n",   
                nspeed, nbits, parity, nstop );  
      
    log_out( "./485.log", szTmp );  
    //保存并测试现在有串口参数设置，在这里如果串口号等出错，会有相关的出错信息   
    if( tcgetattr( fd, &oldtio ) != 0 )  
    {  
      
    sprintf( szTmp, "SetupSerial 1" );  
  
    log_out( "./485.log", szTmp );  
  
        perror( "SetupSerial 1" );  
        return -1;  
    }  
  
    //修改输出模式，原始数据输出  
    bzero( &newtio, sizeof( newtio ));  
    newtio.c_cflag &=~(OPOST);  
  
    //屏蔽其他标志位  
    newtio.c_cflag |= (CLOCAL | CREAD );  
    newtio.c_cflag &= ~CSIZE;  
      
    //设置数据位  
    switch( nbits )  
    {  
    case 7:  
        newtio.c_cflag |= CS7;  
        break;  
    case 8:  
        newtio.c_cflag |= CS8;  
        break;  
    default:  
        perror("Unsupported date bit!\n");  
        return -1;  
    }  
      
    //设置校验位  
    switch( parity )  
    {  
    case 'n':  
    case 'N':  //无奇偶校验位  
        newtio.c_cflag &= ~PARENB;  
        newtio.c_iflag &= ~INPCK;  
        break;  
    case 'o':  
    case 'O':    //设置为奇校验  
        newtio.c_cflag |= ( PARODD | PARENB );  
        newtio.c_iflag |= ( INPCK | ISTRIP );  
        break;  
    case 'e':  
    case 'E':  //设置为偶校验  
        newtio.c_iflag |= ( INPCK |ISTRIP );  
        newtio.c_cflag |= PARENB;  
        newtio.c_cflag &= ~PARODD;  
        break;  
    default:  
        perror("unsupported parity\n");  
        return -1;  
    }  
  
    //设置停止位  
    switch( nstop )   
    {  
    case 1:   
        newtio.c_cflag &= ~CSTOPB;  
        break;  
    case 2:  
        newtio.c_cflag |= CSTOPB;  
        break;  
    default :  
        perror("Unsupported stop bit\n");  
        return -1;  
    }  
  
    //设置波特率  
    switch( nspeed )  
    {  
    case 2400:  
        cfsetispeed( &newtio, B2400 );  
        cfsetospeed( &newtio, B2400 );  
        break;  
    case 4800:  
        cfsetispeed( &newtio, B4800 );  
        cfsetospeed( &newtio, B4800 );  
        break;    
    case 9600:  
        cfsetispeed( &newtio, B9600 );  
        cfsetospeed( &newtio, B9600 );  
        break;    
    case 115200:  
        cfsetispeed( &newtio, B115200 );  
        cfsetospeed( &newtio, B115200 );  
        break;  
    case 460800:  
        cfsetispeed( &newtio, B460800 );  
        cfsetospeed( &newtio, B460800 );  
        break;  
    default:      
        cfsetispeed( &newtio, B9600 );  
        cfsetospeed( &newtio, B9600 );  
        break;  
    }  
    newtio.c_cflag |= CRTSCTS;
    //设置等待时间和最小接收字符  
    newtio.c_cc[VTIME] = 0;    
    newtio.c_cc[VMIN] = 0;     
//VTIME=0，VMIN=0，不管能否读取到数据，read都会立即返回。  
  
//输入模式  
    newtio.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);  
//设置数据流控制  
    newtio.c_iflag &= ~(IXON|IXOFF|IXANY); //使用软件流控制  
//如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读  
    tcflush( fd, TCIFLUSH );   
//激活配置 (将修改后的termios数据设置到串口中）  
    if( tcsetattr( fd, TCSANOW, &newtio ) != 0 )  
    {  
    sprintf( szTmp, "serial set error!\n" );  
  
    log_out( "./485.log", szTmp );  
        perror( "serial set error!" );  
        return -1;  
    }  
  
    log_out( "./485.log", "serial set ok!\n" );  
    return 1;  
}  
  
//打开串口并返回串口设备文件描述  
int open_com_dev( char *dev_name )  
{  
    int fd;  
        char szTmp[128];  
  
    log_init( "./485.log" );  
    if(( fd = open( dev_name, O_RDWR|O_NOCTTY|O_NDELAY)) == -1 )  
    {  
  
        perror("open\n");  
        //printf("Can't open Serial %s Port!\n", dev_name );  
        sprintf( szTmp, "Can't open Serial %s Port!\n", dev_name );  
          
        log_out( "./485.log", szTmp );  
  
        return -1;  
    }  
  
    sprintf( szTmp, "open %s ok!\n", dev_name );  
    log_out( "./485.log", szTmp );  
  
    if(fcntl(fd,F_SETFL,0)<0)  
    {  
        printf("fcntl failed!\n");  
    }  
    //printf("Open %s ok\n",dev_name );  
    return fd;  
}  
  
//发送云台数据  
void* task(void* p)  
{  
    char ch;  
    int j = 0, nread = 0;  
    while(scanf ("%s", &ch) ==1)  
    {  
        pthread_mutex_lock (&mutex);  
        arg.pin_sta = 1;     //设为高电平 发送态  
        ioctl(fd_gpio, IOCTL_PIO_SETSTA, &arg);  
          
        int fd = open_com_dev( DEV_UART );  
        if( fd < 0 )  
        {  
            printf( "open UART device error! %s\n", DEV_UART );  
        }  
        else  
        set_com_opt(fd, 2400,8,'n',1);  
        //set_com_opt(fd, 9600,8,'n',1);  
  
        char buff[] = {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};  
        int len = write(fd,buff,sizeof (buff));  
        if (len < 0)  
        {  
            perror ("write err");  
            exit (-1);  
        }  
        //打印发送数据  
        printf ("sead: ");  
        for (j = 0; j < sizeof(buff); j++)  
        {  
            printf ("%02X ", buff[j]);  
        }  
        printf ("\n");   
      
        //清除scanf缓冲区  
        scanf ("%*[^\n]");  
        scanf ("%*c");  
        close (fd);  
        pthread_mutex_unlock (&mutex);  
    }  
}  
  
//单片机数据收发  
void* task1(void* p)  
{  
    char buf[255];  
    int j = 0, res = 0, nread = 0, i = 0;  
    while (1) {  
    pthread_mutex_lock (&mutex);  
  
    arg.pin_sta = 1;     //设为高电平 发送态  
    ioctl(fd_gpio, IOCTL_PIO_SETDIR, &arg);  
    //打开/dev/pio   
    int fd_s = open_com_dev( DEV_UART );  
    if( fd_s < 0 )  
    {  
        printf( "open UART device error! %s\n", DEV_UART );  
    }  
    else  
        set_com_opt(fd_s, 2400,8,'n',1);  
        //set_com_opt(fd_s, 9600,8,'n',1);  
    
    //发送数据  
    char buff[] = {0xaa,0x55,0x05,0x00,0x33,0x44,0x14,0x90,0x00};  
    int len = write(fd_s,buff,sizeof (buff));  
    if (len < 0)  
    {  
        perror ("write err");  
        exit (-1);  
    }  
    printf ("sead: ");  
    for (j = 0; j < sizeof (buff); j++)  
    {  
        printf ("%02X ", buff[j]);  
    }  
    printf ("\n");  
    
    close (fd_s);  
    start=getSystemTime();  
    arg.pin_sta = 0;   //设为低电平 接收态    
    ioctl(fd_gpio, IOCTL_PIO_SETSTA, &arg);  
    int fd_r=open_com_dev( DEV_UART );  
    if( fd_r < 0 )  
    {  
        printf( "open UART device error! %s\n", DEV_UART );  
    }  
    else  
        set_com_opt(fd_r, 2400,8,'n',1);  
        //set_com_opt(fd_r, 9600,8,'n',1);  
  
    //执行select  
    fd_set rd;    
    FD_ZERO(&rd);    
    FD_SET(fd_r, &rd);    
    if ((res = select (fd_r+1,&rd, NULL, NULL, NULL) )< 0)  
    {  
        perror ("read err");  
        exit (-1);  
    }  
  
    memset (buf, 0, sizeof (buf));  
    if (FD_ISSET (fd_r, &rd))  
    {  
        //接收数据 8 8 2   
        int res1 = 0;  
        while ((nread = read(fd_r, buf, 8)) > 0)  
        {  
            //打印接收数据  
            for (i = 0; i < nread; i++)  
            {  
                printf ("%02X ", buf[i]);  
            }  
            //退出循环， 这里有点疑问  
            res1 += nread;  
            if (res1 == 18)  
            {  
                memset (buf, 0, sizeof (buf));  
                printf ("\n");  
                break;  
            }  
        }  
    }  
    close (fd_r);   
    pthread_mutex_unlock (&mutex);  
    end=getSystemTime();  
    printf("time: %lld ms\n", end-start);  
    usleep (200000);  
    }  
}  
  
int main (void)  
{  
  
    int error = 0, error1 = 0;  
    arg.pin_idx = PIO_NUM;          
    arg.pin_dir = AT91PIO_DIR_OUT;  
    //打开/dev/pio设备  
    fd_gpio = open(DEV_PIO_LED, O_RDWR);   
    if(fd_gpio < 0)  
    {  
        perror("fd_gpio open err");  
        exit (-1);  
    }  
  
    //初始化互斥量  
    pthread_mutex_init (&mutex, 0);  
    pthread_t tid, tid1;  
    //创建线程  
    error = pthread_create (&tid, NULL, task, NULL);  
    error1 = pthread_create (&tid1, NULL, task1, NULL);  
    //等待线程结束  
    pthread_join (tid, NULL);  
    pthread_join (tid1, NULL);  
    //销毁互斥量  
    pthread_mutex_destroy(&mutex);  
    //关闭设备    
    close (fd_gpio);  
    return 0;  
}  