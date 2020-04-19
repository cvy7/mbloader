/**
 * Bootloader um dem Mikrocontroller Bootloader von Peter Dannegger anzusteuern
 * Teile des Codes sind vom original Booloader von Peter Dannegger (danni@alice-dsl.net)
 *
 * @author Bernhard Michler, based on linux source of Andreas Butti
 */
//трассировка ввода-вывода
//strace -s9999 -o mbloader.strase -eread,write,ioctl ./mbloader -d /dev/ttyS3 -b 38400 -a 100 -m 10 -p svz.bin
/// Includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <sys/times.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "com.h"
#include "protocol.h"
#include "modbus.h"
/**************************************************************/
/*                          CONSTANTS                         */
/**************************************************************/

#define BIN     1

#define AUX     1
#define CON     2
#define TRUE    1
#define FALSE   0

#define P_COM   'p'

#define PROGRAM      1
#define WREG         2
#define RREG         4
#define PFPGA        8
#define RFPGA       0x10
#define VFPGA       0x20
#define PEEPROM     0x40
#define REEPROM     0x80
#define VEEPROM     0x100
#define RRAM        0x200
#define RREGF       0x400
#define WREGF       0x800
#define RIREG       0x1000
#define RIREGF      0x2000
#define RCOILS      0x4000
#define WCOILS      0x8000
#define RDISCS      0x10000L
#define PROGRAM88   0x20000L
#define WREG1       0x40000L

//#define ACK     0x06
//#define NACK    0x15
#define ESC     0x1b
#define CTRLC   0x03
#define CTRLP   0x10
#define CTRLF   0x06
#define CTRLV   0x16

#define  MAXFLASH 0x400000L       // max flash size (4MB)

// Definitions
#define TIMEOUT   3   // 0.3s
#define TIMEOUTP  24  // 1.2s
#define MAX_LIST 30000


#define ELAPSED_TIME(a) {                   \
    struct tms    time;                     \
    clock_t       stop;                     \
    stop = times (&time);                   \
    (a) = (double) (stop - start) / ticks;  \
}

/**************************************************************/
/*                          GLOBALS                           */
/**************************************************************/
static struct termios   curr_term, old_term;
static FILE             *fp_stdio = NULL; /* filepointer to normal terminal */
static int              running = TRUE;
static int              esc_seq = 0;

static int              bsize = 16;

    // pointer to password...
    // must contain one of
    // 0x0A - LF,  0x0B - VT,  0x0D - CR,  0x0F - SI
    // 0x21 - '!', 0x43 - 'C', 0x61 - 'a', 0x85, 0x87
    // 0xC3 - 'A~',0xE1 - 'aВґ' - ISO8859-1
static char             *password = "Peda";
static char             *device = "/dev/ttyS0";
static int              baud = 38400;
static int              mb_addr= 11;
static int              mb_addr_boot= 100;
static int              mb_areg=0;
static int              mb_nreg=1;
static int              rs232_echo=0;
static int              rs485_manc=0;
static int              mb_set_debug=0;
static int              mb_stop_bits=1;

/* variables for stopwatch */
static clock_t  start  = 0;
static double   ticks = 1;

// Filename of the HEX File
static const char * hexfile = NULL;
static modbus_t *ctx= NULL;
static modbus_mapping_t *mb_mapping;
static uint16_t reg[128];


typedef struct
{
    unsigned long   value;
    speed_t         constval;
} baudInfo_t;
baudInfo_t baudrates[] = { 
    {     50,     B50 },
    {     75,     B75 },
    {    110,    B110 },
    {    134,    B134 },
    {    150,    B150 },
    {    200,    B200 },
    {    300,    B300 },
    {    600,    B600 },
    {   1200,   B1200 },
    {   1800,   B1800 },
    {   2400,   B2400 },
    {   4800,   B4800 },
    {   9600,   B9600 },
    {  19200,  B19200 },
    {  38400,  B38400 },
    {  57600,  B57600 },
    { 115200, B115200 },
    { 230400, B230400 }
};



/*****************************************************************************
 *
 *      Signal handler - reset terminal
 *
 ****************************************************************************/
void sig_handler(int signal)
{
    running = FALSE;
#if 0
    if (fp_stdio != NULL)
    {
        printf("\nSignal %d (%d) resetting terminal !\n", signal, getpid());
        tcsetattr (fileno (fp_stdio), TCSAFLUSH, &old_term);
    }
#endif
}


/*****************************************************************************
 *
 *      Set timeout on tty input to new value, returns old timeout
 *
 ****************************************************************************/
static int set_tty_timeout (int    fd,
                            int    timeout)
{
    struct termios      terminal;
    int                 old_timeout;

    /* get current settings */
    tcgetattr (fd, &terminal);

    /* get old timeout */
    old_timeout = terminal.c_cc[VTIME];

    /* set timeout in 10th of seconds */
    terminal.c_cc[VTIME] = timeout;

    /* set new status */
    tcsetattr (fd, TCSANOW, &terminal);

    return (old_timeout);
}

#ifndef BIN
/**
 * reads hex data from string
 */
int sscanhex (char          *str,
              unsigned int  *hexout,
              int           n)
{
    unsigned int hex = 0, x = 0;

    for(; n; n--)
    {
        x = *str;
        if(x >= 'a')
        {
            x += 10 - 'a';
        }
        else if(x >= 'A')
        {
            x += 10 - 'A';
        }
        else
        {
            x -= '0';
        }

        if(x >= 16)
        {
            break;
        }

        hex = hex * 16 + x;
        str++;
    }

    *hexout = hex;
    return n; // 0 if all digits
}


/**
 * Reads the hex file
 *
 * @return 1 to 255 number of bytes, -1 file end, -2 error or no HEX File
 */
int readhex (FILE           *fp,
             unsigned long  *addr,
             unsigned char  *data)
{
    char hexline[524]; // intel hex: max 255 byte
    char *hp = hexline;
    unsigned int byte;
    int i;
    unsigned int num;
    unsigned int low_addr;

    if(fgets( hexline, 524, fp ) == NULL)
    {
        return -1; // end of file
    }

    if(*hp++ != ':')
    {
        return -2; // no hex record
    }

    if(sscanhex(hp, &num, 2))
    {
        return -2; // no hex number
    }

    hp += 2;

    if(sscanhex(hp, &low_addr, 4))
    {
        return -2;
    }

    *addr &= 0xF0000L;
    *addr += low_addr;
    hp += 4;

    if(sscanhex( hp, &byte, 2))
    {
        return -2;
    }

    if(byte == 2)
    {
        hp += 2;
        if(sscanhex(hp, &low_addr, 4))
        {
            return -2;
        }
        *addr = low_addr * 16L;
        return 0; // segment record
    }

    if(byte == 1)
    {
        return 0; // end record
    }

    if(byte != 0)
    {
        return -2; // error, unknown record
    }

    for(i = num; i--;)
    {
        hp += 2;
        if(sscanhex(hp, &byte, 2))
        {
            return -2;
        }
        *data++ = byte;
    }
    return num;
}
#endif
/**
 * Read a hexfile
 */
char * read_hexfile(const char * filename, uint32_t * lastaddr)
{
    char    *data;
    FILE    *fp;
    int     len;
    int     x;
    unsigned char line[256],c;
    unsigned long addr = 0;
    unsigned long file_length;
    struct stat sbuf;
    // reading file to "data"
#ifdef BIN
    if (access(filename, 0) != 0)
            {
                    printf("Error: can't access file \"%s\"\n", filename);
                    return NULL;
            }
            else
            {
                    /* get length of file */
                    if (stat(filename, &sbuf) == 0) file_length = sbuf.st_size;

                    if ((fp = fopen(filename, "rb")) == NULL)
                    {
                            printf( "Error: can't open file \"%s\"\n", filename);
                            return NULL;
                    }
                    else
                    {
                            /*
                            *	Read entire file into a buffer
                            */
                            data =  (unsigned char *) malloc((size_t) (file_length+1024));

                            if (data == NULL)
                            {
                                    printf("Error: can't allocate memory (%d Kbytes)\n",
                                            (int) (file_length / 1024L));
                                    return NULL;
                            }
                            else
                            {

                                printf("Reading        : %s... ", filename);
                                if (fread(data, 1, (size_t) file_length, fp) !=
                                            (size_t) file_length)
                                    {
                                            printf("Error reading file \"%s\"\n", filename);
                                            return NULL;
                                    }
                            }

                            fclose(fp);
                            *lastaddr = file_length;
                            return data;
                    }
             }


#else
                    data = malloc(MAXFLASH);
                    if (data == NULL)
                    {
                        printf("Memory allocation error, could not get %d bytes for flash-buffer!\n",
                               MAXFLASH);
                        return NULL;
                    }

                    *lastaddr = 0;
                    memset (data, 0xff, MAXFLASH);


                    if(NULL == (fp = fopen(filename, "rb")))
                    {
                        printf("File \"%s\" open failed: %s!\n\n", filename, strerror(errno));
                        free(data);
                        return NULL;
                    }

                    printf("Reading       : %s... ", filename);
    while((len = readhex(fp, &addr, line)) >= 0)

    {
        if(len)
        {
            if( addr + len > MAXFLASH )
            {
                fclose(fp);
                free(data);
                printf("\n  Hex-file too large for target!\n");
                return NULL;
            }
            for(x = 0; x < len; x++)
            {
                data[x + addr] = line[x];
            }

            addr += len;

            if(*lastaddr < (addr-1))
            {
                *lastaddr = addr-1;
            }
            addr++;
        }
    }

    fclose(fp);

    printf("File read.\n");
    return data;
  #endif
}


/**
 * Print percentage line
 */
void print_perc_bar (char          *text,
                     unsigned long full_val,
                     unsigned long cur_val)
{
    int         i;
    int         cur_perc;
    int         cur100p;
    int         txtlen = 8;    // length of the add. text 2 * " [" "100%"
    unsigned short columns = 80;
    struct winsize win_size;

    if (text)
        txtlen += strlen (text);

    if (ioctl (STDIN_FILENO, TIOCGWINSZ, &win_size) >= 0)
    {
        // number of columns in terminal
        columns = win_size.ws_col;
    }
    cur100p  = columns - txtlen;
    cur_perc = (cur_val * cur100p) / full_val;

    printf ("%s [", text ? text : "");

    for (i = 0; i < cur_perc; i++) printf ("#");
    for (     ; i < cur100p;  i++) printf (" ");

    printf ("] %3d%%\r", (int)((cur_val * 100) / full_val));

    fflush(stdout);
}

//********************************************************************************************************************
//******************************************************************************************************************
//Не совсем стандартный MODBUS в бутлоадере- длина пакета 256 байт чистых - страница Flash Atmeg'и 128 !!!!!!!!!!!!!
//Поэтому он тут наряду с libmodbus
//******************************************************************************************************************
/*
Датаграмма модифицированный Modbus RTU
Запрос для основной программы
10   адрес
0x06 функция запись  регистра
00   старший байт адреса
0xff младший байт адреса
0xde старший байт
0xad младший байт
     CRC старший байт
     CRC младший байт
********************************
отвечает основная программа повтором,
или ошибкой
10   адрес
>127 ошибка
код ошибки
CRC
CRC

если нет ошибки переход на бутлоадер
с записью в eeprom по адресу 4095 0xde

иначе загрузчик отваливается
******************************************
Запрос
100  адрес
0x10 функция запись нескольких регистров
XX   старший байт адреса (номер страницы)
XX   младший байт адреса (номер страницы)
0x00 количество реш=гистров старший байт
0x10 кол-во регистров младший байт
00   кол-во байт далее- формально - правильно, байт 256, но не соответствует спецификации modbus
     старший байт буфера[0]
     младший байт буфера[0]
--------------------------
     старший байт буфера[255]
     младший байт буфера[255]
     CRC старший байт
     CRC младший байт
********************************************
пауза 1,75mc (2mc)
Ответ
100  адрес
0x10 функция запись нескольких регистров
XX   старший байт адреса (номер страницы)
XX   младший байт адреса (номер страницы)
0x01 количество реш=гистров старший байт
0x00 кол-во регистров младший байт
 CRC старший байт
 CRC младший байт
*/
static u_int8_t mb_buff[2048];
static int  mb_state;
#define  MB_STATE_ERR       -1
#define  MB_STATE_IDLE       1
#define  MB_STATE_WRITE_PAGE 2
#define  MB_STATE_F          3
#define  MB_STATE_ADR_MSB    4
#define  MB_STATE_ADR_LSB    5
#define  MB_STATE_NREG_MSB   6
#define  MB_STATE_NREG_LSB   7
#define  MB_STATE_NBYTES     8
#define  MB_STATE_DATA_MSB   9
#define  MB_STATE_DATA_LSB   10
#define  MB_STATE_CRC_MSB    11
#define  MB_STATE_CRC_LSB    12
#define  MB_STATE_READ_ANS   13
#define  MB_STATE_TX   14


u_int16_t
crc16_update(u_int16_t crc, u_int8_t a)
{
int i;
crc &=0xffff;
crc ^= a;
for (i = 0; i < 8; ++i)
{
    if (crc & 1)
    crc = (crc >> 1) ^ 0xA001;
    else
    crc = (crc >> 1);
}
return crc;
}

u_int16_t mbcrc(u_int8_t *pucFrame, int usLen){
    u_int16_t crc=0xffff;
    while( usLen-- )
    {
        crc=crc16_update(crc, *( pucFrame++ ));
    }
    return crc;
}
//******************************************************************************************************************
//Не совсем стандартный MODBUS в бутлоадере- длина пакета 256 байт чистых - страница Flash Atmeg'и 128
//Поэтому он тут наряду с libmodbus
//******************************************************************************************************************
int mb_write(int           fd,
             int           debug,
             u_int16_t     mb_addr,
             u_int16_t     mb_reg,
             u_int16_t     nreg,
             u_int8_t      nbytes,
             u_int8_t      *regs)
{
u_int8_t  *buff;
u_int16_t nbytesc=nreg*2;
u_int16_t crc;
int i;
mb_state=MB_STATE_TX;

 for(;;){
     fprintf(stderr,"state==%d==",mb_state);
     switch(mb_state){
    default://MB_STATE_TX
    buff=mb_buff;
    *buff++=mb_addr;
    *buff++=0x10;
    *buff++=mb_reg>>8;
    *buff++=mb_reg&0xff;
    *buff++=(nreg>>8)&0xff;
    *buff++=nreg&0xff;
    *buff++=nbytes&0xff;
    memcpy(buff,regs,(nbytesc+2));
    buff+=nbytesc;
    crc=mbcrc(mb_buff,(7+nbytesc));
    //crc=usMBCRC16(mb_buff,(7+nbytesc));
    *buff++=crc&0xff;
    *buff++=(crc>>8)&0xff;
    *buff++=0;
    *buff++=0;
    *buff++=0;
    *buff++=0;
    *buff++=0;
    *buff++=0;
    *buff++=0;
    *buff++=0;
    *buff++=0;
    *buff=0;
    //**************************************************
    //DBG
    //fputs("Time150us_pre_write\n", stderr);
    //usleep(150UL);
    //**************************************************
    int n;
    com_drain(fd);
   n = write(fd, mb_buff, (7+nbytesc+2+5));
    com_drain(fd);
   /* char *pc=mb_buff;
    for(n=0; n<(7+nbytesc+2);n++){
      com_putc(fd, *pc);
      pc++;
    }*/
    //**************************************************
    //DBG
    //fputs("Time150us_post_write\n", stderr);
    //usleep(150UL);
    //**************************************************
    if(rs232_echo){
    int timeout=(1000000UL/baud)*12*(7+nbytesc+2)+1500;
    usleep(timeout);
    int n_read=read(fd, mb_buff, (7+nbytesc+2));
    fprintf(stderr, "Writed [%d] chars, read [%d] chars with timeout [%d]us...",n,n_read,timeout);
    }
    if (n < 0){
      fputs("write() failed!\n", stderr);
      return 0;
    }
    buff=mb_buff;
    mb_state=MB_STATE_READ_ANS;
    if(debug)fprintf(stderr,"\n->0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X <<\n",*buff,*(buff+1),*(buff+2),*(buff+3),*(buff+4),*(buff+5),*(buff+6),*(buff+7),*(buff+8),*(buff+9),*(buff+10),*(buff+11),*(buff+12));
    break;

    case MB_STATE_CRC_LSB:
        i=com_getc(fd,TIMEOUTP);
        *buff++=i;
        if(debug) fprintf(stderr,"0x%02X:",i);
        if(mbcrc(mb_buff,(buff-mb_buff))) {fputs("CRC!\n", stderr); return 0;}
        else                                                        return 1;
    break;

    case MB_STATE_CRC_MSB:
        i=com_getc(fd,TIMEOUTP);
        *buff++=i;
        if(debug) fprintf(stderr,"0x%02X:",i);
        mb_state=MB_STATE_CRC_LSB;
    break;

    case MB_STATE_NREG_LSB:
        i=com_getc(fd,TIMEOUTP);
        *buff++=i;
        if(debug) fprintf(stderr,"0x%02X:",i);
        mb_state=MB_STATE_CRC_MSB;
    break;

    case MB_STATE_NREG_MSB:
        i=com_getc(fd,TIMEOUTP);
        *buff++=i;
        if(debug) fprintf(stderr,"0x%02X:",i);
        mb_state=MB_STATE_NREG_LSB;
    break;

    case MB_STATE_ADR_LSB:
        i=com_getc(fd,TIMEOUTP);
        *buff++=i;
        if(debug) fprintf(stderr,"0x%02X>>",i);
        mb_state=MB_STATE_NREG_MSB;
    break;

    case MB_STATE_ADR_MSB:
        i=com_getc(fd,TIMEOUTP);
        *buff++=i;
        if(debug) fprintf(stderr,"0x%02X:",i);
        mb_state=MB_STATE_ADR_LSB;
    break;

    case MB_STATE_F:
     i=com_getc(fd,TIMEOUTP);
     *buff++=i;
     if(debug) fprintf(stderr,"0x%02X:",i);
     if(i==0x10) mb_state=MB_STATE_ADR_MSB;
     else        mb_state=MB_STATE_ERR;
    break;

    case MB_STATE_READ_ANS:
    i=com_getc(fd,TIMEOUTP);
    *buff=i;
    if(debug) fprintf(stderr,"0x%02X:",i);

    if(i==mb_addr){
        mb_state=MB_STATE_F;
        buff++;
        //***************
       // sleep(1);
       // return 1;
        //*******************
    }

    if(i==-1) {mb_state=MB_STATE_ERR;
        fputs("Timeout!\n", stderr);
    }
    break;

    case MB_STATE_ERR:
     return 0;

 }
}

}
//********************************************************************************************************************
//********************************************************************************************************************
/**
 * Flashes the controller
 */
int programflash (int           fd,
                  char        * data,
                  unsigned long lastaddr
                                        )
{
    struct tms  timestruct;
    clock_t start_time;       //time
    clock_t end_time;         //time
    float   seconds;
    const char * ANIM_CHARS = "-\\|/";
    int actCode=0xdede;
    u_int16_t mb_reg=254;
    int mb_err=0;
    int nbytes;
    int adr;
    int debug=1;

    start_time = times (&timestruct);
    // Sending command to ativate bootloader
    fprintf(stderr,"Testing bootloader activated...");
    int i=com_getc(fd,TIMEOUTP);
    fprintf(stderr,"0x%02X:\n",i);

    if (i==mb_addr_boot) { // уже активирован и кидается своим адресом
     fprintf(stderr,"Bootloader  activated!\n");
    }
    else if (i!=-1){ // какой то другой пакет?
      fprintf(stderr,"Error boot_device_addres!\n");
      return -1;
    }
    else {
        fprintf(stderr,"Activating bootloader 0x%03X=%d\n",mb_addr,mb_addr);
        if(mb_write(fd,debug,mb_addr,mb_reg,1,2,&actCode))
                fprintf(stderr,"Bootloader activated \n");
        else {
                fprintf(stderr,"Error Bootloader activation,0x%03X=%d\n",mb_err,mb_err);
                return -1;//error
             }
        fprintf(stderr,"Waiting for bootloader activity...");
        int i=com_getc(fd,TIMEOUTP);
        fprintf(stderr,"0x%02X:\n",i);
    }

    u_int8_t zero[5]={0,0,0,0,0};
    write(fd, zero, 5);
    mb_reg=0;
    nbytes=256;//те в пакете будет 0
    lastaddr++;//последний адрес файла длиной 512 байт будет 511

    for(adr=0;adr<=lastaddr;adr+=256){//скармливаем файл по 256 байт, в последнем пакете будет часть мусора
        if((lastaddr-adr) <=256) nbytes=255;//этот пакет последний, даже если он содержит ровно 256 байт nbytes=255
        //if(mb_reg!=0x39){
        if(!mb_write(fd,debug,mb_addr_boot,mb_reg,128,nbytes,data)){
            if(!mb_write(fd,debug,mb_addr_boot,mb_reg,128,nbytes,data)){//еще одна попытка отправить пакет
                fprintf(stderr,"Error sending file");              //
                return -1;
            }
        }
        if(debug) {
            fputs("\n",stderr);
            //debug=0;
        }
        fprintf(stderr,"%x\r",adr);
        //}
        data+=256;
        mb_reg++;


    }
    return 0;//Success
}
//***************************************************************************************************************************
/**
 * prints usage
 */
void usage()
{
    printf("./mbloader [-d /dev/ttyS0] [-b 115200] [-m 11] [-a 100] -[p] file.hex\n");
    printf("-d    Device\n");
    printf("-b    Baudrate\n");
    printf("-stop 2 stop bits\n");
    printf("-p    Program\n");
    printf("-a    mb addres of bootloader\n");
    printf("-m    mb_addres of main programm\n");
    printf("-echo  flush rs232 echo         \n");
    printf("-manc  manchester coded 485 data\n");
    printf("-debug  print debug  data\n");

    printf("Programm mk       : ./mbloader -d /dev/ttyS0 -b 115200 -a 102 -m 12 -p mk_su2.bin \n");
    printf("Programm mega88   : ./mbloader -d /dev/ttyS0 -b 115200 -a 102 -m 12 -p88 mk_su2.bin \n");
    printf("Programm fpga conf: ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -pfpga -vfpga output_file.rpd \n");
    printf("Read fpga conf fl.: ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -rfpga  rfpga.bin \n");
    printf("Programm mk eeprom: ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -peeprom -veeprom eeprom.bin \n");
    printf("Read mk eeprom    : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -reeprom  reeprom.bin \n");
    printf("Read mk ram       : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -rram  ram.bin \n");
    printf("Reading registers : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 0 -nreg 32 -rreg -rregf regs.bin  \n");
    printf("Reading inp. regs : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 0 -nreg 32 -rireg -riregf iregs.bin  \n");
    printf("Writing registers : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 0 -nreg 3 -wreg 1 2 3  \n");
    printf("Writing register  : ./mbloader -d /dev/ttyS1 -b 9600   -m 12 -areg 8193 -wreg1 1000 \n");
    printf("Wr regs from file : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 0 -nreg 20 -wregf regs.bin  \n");
    printf("Reading coils     : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 0 -nreg 32 -rcoils \n");
    printf("with manchester co: ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -manc -areg 0 -nreg 32 -rcoils \n");
    printf("with echo flush   : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -echo -areg 0 -nreg 32 -rcoils \n");
    printf("Writing coils     : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 0 -nreg 32 -wcoils 1 0 1 1 1 \n");
    printf("Reading discs     : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 0 -nreg 32 -rdiscs \n");
    printf("Reading hmi_vars  : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 200 -nreg 120 -rregf hmi_vars.bin  \n");
    printf("Reading tconv     : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 300 -nreg 120 -rregf tconv.bin  \n");
    printf("Reading FPGA RAM  : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 4096 -nreg 12801 -riregf fpga_ram.bin  \n");
    printf("Activate console  : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 233 -nreg 1 -wreg 2 \n");
    printf("PPCH on & heat on : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 17 -nreg 7 -wcoils 1 1 1 0 0 0 1 \n");
    printf("PPCH heat off     : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 17 -nreg 7 -wcoils 1 0 1 0 0 0 1 \n");
    printf("PPCH state 1      : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 0 -nreg 32 -rdiscs \n");
    printf("PPCH state 2      : ./mbloader -d /dev/ttyS0 -b 115200 -m 12 -areg 0 -nreg 32 -rireg -riregf ppch_state2.bin  \n");

    printf("Author: cvy7  (based on code from Bernhard Michler & Andreas Butti)\n");

    exit(1);
}

/**
 * Try to connect a device
 */
int connect_device ( int fd,
                     const char *password )
{
    const char * ANIM_CHARS = "-\\|/";

    int state = 0;
    int in = 0;

    char passtring[32];

    sprintf (passtring, "%s%c", password, 0xff);

    printf("Waiting for device...  ");

    while (running)
    {
        const char *s = passtring; //password;

        usleep (25000);     // just to slow animation...
        printf("\b%c", ANIM_CHARS[state++ & 3]);
        fflush(stdout);

        do 
        {
            if (*s)
                com_putc(fd, *s);
            else
                com_putc(fd, 0x0ff);

            in = com_getc(fd, 0);

            if (/*in == CONNECT*/1)
            {
                printf ("\bconnected");

                // clear buffer from echo...
                while (com_getc(fd, TIMEOUT) != -1);

                //sendcommand( fd, COMMAND );

                while (1)
                {
                    switch(com_getc(fd, TIMEOUT))
                    {

                        case NAK:
                        case -1:
                            printf ("!\n");
                            return 1;
                    }
                }
            }
        } while (*s++);
    }
    printf ("\nTerminated by user.\n");

    return 0;
}//void connect_device()

int prog_verify (int            fd,
                 uint32_t       mode,
                 int            baud,
                 int            block_size,
                 const char     *password,
                 const char     *device,
                 const char     *hexfile)
{
    char        *data = NULL;


    // last address in hexfile
    uint32_t last_addr = 0;
    int rc;



    switch (mode & PROGRAM)
    {
        case PROGRAM:
            printf ("Program device.\n");
            break;       
    }

    printf("Port           : %s\n", device);
    printf("Baudrate       : %d\n", baud);
    printf("Mb address main: %d\n", mb_addr);
    printf("Mb address boot: %d\n", mb_addr_boot);
    printf("File           : %s\n", hexfile);

    // read the file
    data = read_hexfile (hexfile, &last_addr);

    if (data == NULL)
        return -4;

    printf("Size          : %ld Bytes\n", last_addr);

    printf("-------------------------------------------------\n");

    // now start with target...
    if (1/*connect_device (fd, password)*/)
    {

        if (data == NULL)
        {
            printf ("ERROR: no buffer allocated and filled, exiting!\n");
            return (-1);
        }

        if (mode & PROGRAM)
        {
            rc=programflash (fd, data, last_addr);
            if (rc >= 0)
            {
                    printf("\n ++++++++++ Device successfully programmed! ++++++++++\n\n");
                    return 0;
            }
            else
            {
                    printf("\n ---------- Programming failed! ----------\n\n");
                    return (-5);
            }
        }


        printf("...starting application\n\n");
      //  sendcommand(fd, START);         //start application
      //  sendcommand(fd, START);
    }
    return 0;
}
//*****************************************************************************************************************
/**
 * Flashes the controller atmega88
 */
int boot_waiting_for_activity(const char *device,int baudid){
    int fd;
    struct tms  timestruct;
    clock_t     start_time;       //time

    start_time = times (&timestruct);
    fd = com_open(device, baudrates[baudid].constval);
    if (fd < 0)
    {
        printf("\nOpening com port \"%s\" failed (%s)!\n",
               device, strerror (errno));
        return -1;
    }
    fprintf(stderr,"Testing bootloader activated...");
    int i=com_getc(fd,TIMEOUTP);
    fprintf(stderr,"com_getc=0x%02X:\n",i);
    com_close(fd);
    if (i==mb_addr_boot) { // уже активирован и кидается своим адресом
     fprintf(stderr,"Bootloader  activated!\n");
     return 1;
    }
    else if (i!=-1){ // какой то другой пакет?
      fprintf(stderr,"Error boot_device_address!\n");
      return -1;
    }
    return 0;
}

int write_page88(const char *device,int baud,int baudid,int mb_addr_boot,int adr,uint8_t *data,int len){
    int rc;
    uint8_t data2[64];
if(len%2) len++;
if(len>64){
    printf("\nwrite_page88:len>64!\n");
    return -1;
}
for(int j=0;j<len;j+=2){
   data2[j]=data[j+1];
   data2[j+1]=data[j];
}
len/=2;
adr/=2;
for(int i=0;i<3;i++){
    rc=mb_write_regs(adr,len,(uint16_t *)data2);
   if(rc>=0) return rc;
    rc=boot_waiting_for_activity(device,baudid);
    if(rc<1){
       printf("\nwrite_page88:boot_waiting_for_activity(device,baudid); failed!\n");
       return -1;
    }
    rc=mb_open_master(device, baud);
    if(rc < 0) {
        printf("\nwrite_page88:mb_open_master(device, baud); failed!\n");
        return rc;
    }
    rc=mb_slave_addr(mb_addr_boot);
    if(rc < 0) {
        printf("\nwrite_page88:mb_slave_addr(mb_addr_boot); failed!\n");
        return rc;
    }
}
}

int prog_verify88 (
                 int            baud,
                 int            baudid,
                 const char     *device,
                 const char     *hexfile)
{
    uint8_t    *data = NULL;
    int         adr=0;
    int         actCode=0xdede;
    uint16_t    actmb_reg=254;
    // last address in hexfile
    uint32_t    last_addr = 0;
    int         page_len=64;
    int         dop_page=0;
    uint16_t    dop_data=0xffff;
    int         rc;
    printf ("Program atmega88\n");
    printf("Port           : %s\n", device);
    printf("Baudrate       : %d\n", baud);
    printf("Mb address main: %d\n", mb_addr);
    printf("Mb address boot: %d\n", mb_addr_boot);
    printf("File           : %s\n", hexfile);
    // read the file
    data = read_hexfile (hexfile, &last_addr);
    if (data == NULL){
        printf ("ERROR: no buffer allocated and filled, exiting!\n");
        return -1;
    }
    printf("Size          : %ld Bytes\n", last_addr);
    printf("-------------------------------------------------\n");
    rc=boot_waiting_for_activity(device,baudid);
    if(rc<0){
        printf("\nprog_verify88:0:boot_waiting_for_activity(device,baudid); failed!\n");
        return -1;
    }
    if(!rc){//Активируем бутлоадер
        rc=mb_open_master(device, baud);
        if(rc < 0) {
            printf("\nprog_verify88:mb_open_master(device, baud); failed!\n");
            return rc;
        }
        rc=mb_slave_addr(mb_addr);
        if(rc < 0) {
            printf("\nprog_verify88:mb_slave_addr(mb_addr); failed!\n");
            return rc;
        }
        rc=mb_write_regs(actmb_reg, 1,&actCode);
        if(rc < 0) {
            printf("\nprog_verify88:mb_write_regs(actmb_reg, 1,&actCode); failed!\n");
            return rc;
        }
        mb_close();
        rc=boot_waiting_for_activity(device,baudid);
        if(rc<0){
            printf("\nprog_verify88:1:boot_waiting_for_activity(device,baudid) failed!\n");
            return -1;
        }
    }
    rc=mb_open_master(device, baud);
    if(rc < 0) {
        printf("\nprog_verify88:mb_open_master(device, baud); failed!\n");
        return rc;
    }
    rc=mb_slave_addr(mb_addr_boot);
    if(rc < 0) {
        printf("\nprog_verify88:mb_slave_addr(mb_addr_boot); failed!\n");
        return rc;
    }
    for(adr=0;adr<=last_addr;adr+=page_len){
       int len=last_addr-adr;
       if     (len==page_len) dop_page=1;//Признаком конца записи для бутлоадера является неполная страница.
       //Если так совпало, что прошивка имеет динну ровно страниц то добавляется 1 страница из 2 байт 0xff
       //Минусом метода является то, что нельзя использовать последние 2 байта памяти.
       else if(len> page_len) len =page_len;
       rc=write_page88(device,baud,baudid,mb_addr_boot,adr,data,len);
       data+=len;
       if(rc < 0) {
           printf("\nprog_verify88:write_page88(device,baud,baudid,mb_addr_boot,adr,data,len); failed!\n");
           return rc;
       }
       if(dop_page){
           rc=write_page88(device,baud,baudid,mb_addr_boot,(adr+page_len),&dop_data,2);
           if(rc < 0) {
               printf("\nprog_verify88:write_page88(device,baud,baudid,mb_addr_boot,(adr+page_len),(char*)&dop_data,2); failed!\n");
               return rc;
           }
       }
    }
    mb_close();
    printf("\n ++++++++++ Device successfully programmed! ++++++++++\n\n");
    printf("...starting application\n\n");
    return 0;

}

int mb_open_master(char *device, int baud){

    ctx = modbus_new_rtu(device, baud, 'N', 8, mb_stop_bits);
    if (ctx == NULL) {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        return -1;
    }


    modbus_set_debug(ctx,mb_set_debug);
    modbus_set_echo(ctx,rs232_echo);
    modbus_set_manc(ctx,rs485_manc);
    modbus_rtu_set_rts(ctx,1);
    mb_mapping = modbus_mapping_new(/*BITS_ADDRESS + BITS_NB*/256,
                                    /*INPUT_BITS_ADDRESS + INPUT_BITS_NB*/256,
                                    /*REGISTERS_ADDRESS + REGISTERS_NB*/256,
                                    /*INPUT_REGISTERS_ADDRESS + INPUT_REGISTERS_NB*/256);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    printf("MODBUS master started at\n");
    printf("Port           : %s\n", device);
    printf("Baudrate       : %d\n", baud);
    printf("Stop bits      : %d\n", mb_stop_bits);
    printf("Manchester codi: %d\n", rs485_manc);



    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    modbus_set_debug(ctx, mb_set_debug);
return 0;
}

int mb_slave_addr(int slave_id){

    int rc = modbus_set_slave(ctx, slave_id);
    if (rc == -1) {
        fprintf(stderr, "Invalid slave ID\n");
        //modbus_free(ctx);
        return -1;
    }
    printf("Slave ID=      : %d\n", slave_id);
    return rc;
}

int mb_read_regs(int addr, int nb, uint16_t *dst){
int rc;

//do
    rc = modbus_read_registers(ctx, addr, nb, dst);
//while (rc == -1);

if (rc == -1) {
    fprintf(stderr, "Err@ modbus_read_registers(ctx, addr=%d, nb=%d, src) %s\n", addr, nb, modbus_strerror(errno));
    //modbus_free(ctx);
    return -1;
}
//printf("%d register(s) read from %d\n", nb, addr);
return rc;
}

int mb_read_input_regs(int addr, int nb, uint16_t *dst){
int rc;

//do
 rc = modbus_read_input_registers(ctx, addr, nb, dst);
//while (rc == -1);

if (rc == -1) {
    fprintf(stderr, "Err@ modbus_read_input_registers(ctx, addr=%d, nb=%d, src) %s\n", addr, nb, modbus_strerror(errno));
    //modbus_free(ctx);
    return -1;
}
//printf("%d input register(s) read from %d\n", nb, addr);
return rc;
}


int mb_write_regs(int addr, int nb, uint16_t *src){
int rc;

//do
    rc = modbus_write_registers(ctx, addr, nb, src);
//while (rc == -1);

if (rc == -1) {
    fprintf(stderr, "Err@ modbus_write_registers(ctx, addr, nb, src) %s\n" , modbus_strerror(errno));
    //modbus_free(ctx);
    return -1;
}

return rc;
}

int mb_write_reg(int addr, uint16_t *src){
int rc;

//do
    rc = modbus_write_register(ctx, addr, *src);
//while (rc == -1);

if (rc == -1) {
    fprintf(stderr, "Err@ modbus_write_registers(ctx, addr, nb, src) %s\n" , modbus_strerror(errno));
    //modbus_free(ctx);
    return -1;
}
//printf("%d register(s) wrote from %d\n", nb,addr);
return rc;
}


int mb_read_coils(int addr, int nb, uint8_t *dst){

int rc = modbus_read_bits(ctx, addr, nb, dst);
if (rc == -1) {
    fprintf(stderr, "Err@ modbus_read_bits(ctx, addr=%d, nb=%d, src) %s\n", addr, nb, modbus_strerror(errno));
    //modbus_free(ctx);
    return -1;
}
//printf("%d coil(s) read from %d\n", nb, addr);
return rc;
}

int mb_write_coils(int addr, int nb, uint8_t *src){

int rc = modbus_write_bits(ctx, addr, nb, src);
if (rc == -1) {
    fprintf(stderr, "Err@ modbus_write_bits(ctx, addr, nb, src) %s\n" , modbus_strerror(errno));
    //modbus_free(ctx);
    return -1;
}
//printf("%d bit(s) wrote from %d\n", nb,addr);
return rc;
}

int mb_read_discs(int addr, int nb, uint8_t *dst){

int rc = modbus_read_input_bits(ctx, addr, nb, dst);
if (rc == -1) {
    fprintf(stderr, "Err@ modbus_read_input_bits(ctx, addr=%d, nb=%d, src) %s\n", addr, nb, modbus_strerror(errno));
    //modbus_free(ctx);
    return -1;
}
//printf("%d discrete(s) read from %d\n", nb, addr);
return rc;
}

void mb_close(){
    if(ctx){
    modbus_close(ctx);
    modbus_free(ctx);
    printf("MODBUS connection closed\n");
    }
    else
        printf("ctx==0!\n");
}

//*******************************************************************************************************
//
int prog_eeprom   (int            fd,
                 uint32_t         mode,
                 int            baud,
                 int            block_size,
                 const char     *password,
                 const char     *device,
                 const char     *hexfile)
{
    char        *data = NULL;
    char        *data1 = NULL;
    char read_buffer[256];
    FILE * fp;
    // last address in hexfile
    uint32_t adr,last_addr = 0;
    uint16_t cmd[2];
    int i,j;
    int rc;
#define EEP_END 4096
#define RAM_END 4096

    printf("File           : %s\n", hexfile);
    rc=mb_open_master(device, baud);
    if(rc < 0) return rc;
    rc=mb_slave_addr(mb_addr);
    if(rc < 0) return rc;

    if (mode & (PEEPROM))
            {
        data1 = read_hexfile (hexfile, &last_addr);
        data=data1;
        if (data == NULL)
            return -4;

        printf("Size          : %ld Bytes\n", last_addr);
        printf("-------------------------------------------------\n");
        printf("Write eeprom ...\n");
        rc=modbus_set_debug(ctx, mb_set_debug);
        if(rc < 0) return rc;
        for(adr=0; adr<last_addr ;adr+=256){

                rc=mb_write_regs(adr/2+0x1000,    64, (uint16_t *) &data[0]);
                if(rc < 0) return rc;
                //rc=modbus_set_debug(ctx, FALSE);
                //if(rc < 0) return rc;
                rc=mb_write_regs(adr/2+0x1000+64, 64, (uint16_t *) &data[128]);
                if(rc < 0) return rc;
                data+=256;
                fprintf(stderr,"%x\r",adr);
        }

    }
    if(mode & VEEPROM){
    if (data1 == NULL){
        data = read_hexfile (hexfile, &last_addr);

        if (data == NULL)
            return -4;

        printf("Size          : %ld Bytes\n", last_addr );
        printf("-------------------------------------------------\n");
    }
    else data=data1;
        printf("Verifing eeprom ...\n");
        // read the file

    rc=modbus_set_debug(ctx, mb_set_debug);
    if(rc < 0) return rc;
    for(adr=0; adr<last_addr ;adr+=256){

            rc=mb_read_regs(adr/2+0x1000,    64, (uint16_t *) &read_buffer[0]);
            if(rc < 0) return rc;
            //rc=modbus_set_debug(ctx, FALSE);
            //if(rc < 0) return rc;
            rc=mb_read_regs(adr/2+0x1000+64, 64, (uint16_t *) &read_buffer[128]);
            if(rc < 0) return rc;
            char *pread_buffer=read_buffer;
            for(j=0;j<256;j++)
                if(*data++!=*pread_buffer++) {
                    data--;
                    pread_buffer--;
                    fprintf(stderr,"Verifacation error @0x%x read%x file%x",adr,data,*pread_buffer);
                    return -1;
                }
            fprintf(stderr,"%x\r",adr);

    }
        printf("\nVerification succesful...\n");
        return 0;
    }
    //*****************************************************
    if(mode & (REEPROM)) {

        printf("Reading EEPROM ...\n");


        if ((fp = fopen(hexfile, "wb")) == NULL)
        {
                printf( "Error: can't open file \"%s\"\n", hexfile);
                return -1;
        }

    rc=modbus_set_debug(ctx, mb_set_debug);
    if(rc < 0) return rc;
    for(adr=0; adr<EEP_END ;adr+=256){

            rc=mb_read_regs(adr/2+0x1000,    64, (uint16_t *) &read_buffer[0]);
            if(rc < 0) return rc;
            //rc=modbus_set_debug(ctx, FALSE);
            //if(rc < 0) return rc;
            rc=mb_read_regs(adr/2+0x1000+64, 64, (uint16_t *) &read_buffer[128]);
            if(rc < 0) return rc;
            fwrite (read_buffer , sizeof(char), sizeof(read_buffer), fp);
            fprintf(stderr,"%x\r",adr);

    }
        fclose (fp);
        printf("\n");
        return 0;
    }
    if(mode & (RRAM)) {

        printf("Reading RAM ...\n");


        if ((fp = fopen(hexfile, "wb")) == NULL)
        {
                printf( "Error: can't open file \"%s\"\n", hexfile);
                return -1;
        }

    rc=modbus_set_debug(ctx, mb_set_debug);
    if(rc < 0) return rc;
    for(adr=0; adr<RAM_END ;adr+=256){

            rc=mb_read_regs(adr/2+0x2000,    64, (uint16_t *) &read_buffer[0]);
            if(rc < 0) return rc;
            //rc=modbus_set_debug(ctx, FALSE);
            if(rc < 0) return rc;
            rc=mb_read_regs(adr/2+0x2000+64, 64, (uint16_t *) &read_buffer[128]);
            if(rc < 0) return rc;
            fwrite (read_buffer , sizeof(char), sizeof(read_buffer), fp);
            fprintf(stderr,"%x\r",adr);

    }
        fclose (fp);
        printf("\n");
        return 0;
    }
    return 0;

}

//*******************************************************************************************************
int prog_fpga   (int            fd,
                 uint32_t       mode,
                 int            baud,
                 int            block_size,
                 const char     *password,
                 const char     *device,
                 const char     *hexfile)
{
    char        *data = NULL;
    char        *data1 = NULL;
    char read_buffer[256];
    FILE * fp;
    // last address in hexfile
    uint32_t adr,last_addr = 0;
    uint16_t cmd[2];
    int i,j;
    int rc;

    printf("File           : %s\n", hexfile);



#define VAR_COMMAND_REGISTER 30
#define VAR_DATA_REGISTER 32
#define PAGE_SIZP (256)
#define NPAGES 0x800
#define VAR_PAGE_ERASE  1
#define VAR_PAGE_ERASE_TIME 2
#define VAR_PAGE_WRITE  3
#define VAR_PAGE_WRITE1 4
#define VAR_PAGE_SKIP   5
#define VAR_BULK_ERASE  6
    #define VAR_PAGE_READ   7

    rc=mb_open_master(device, baud);
    if(rc < 0) return rc;
    rc=mb_slave_addr(mb_addr);
    if(rc < 0) return rc;
//**************************************************
if (mode & (PFPGA))
        {
            // read the file
            data1 = read_hexfile (hexfile, &last_addr);
            data=data1;
            if (data == NULL)
                return -4;

            printf("Size          : %ld Bytes\n", last_addr);
            printf("-------------------------------------------------\n");

            printf("Erasing the config flash for 24c...\n");
            cmd[0]=VAR_BULK_ERASE;cmd[1]=0;
            rc=modbus_set_debug(ctx,mb_set_debug);
            if(rc < 0) return rc;
            rc=mb_write_regs(VAR_COMMAND_REGISTER, 2, cmd);
            if(rc < 0) return rc;
            sleep (7);
            rc=mb_read_regs(VAR_COMMAND_REGISTER, 1, cmd);
            if(rc < 0) return rc;


            if(cmd[0]) {
                printf ("ERROR: erase, exiting!\n");
                return -1;
            }
            if (data == NULL)
                {
                printf ("ERROR: no buffer allocated and filled, exiting!\n");
                return (-1);
            }

            printf ("Program FPGA conf. flash.\n");

            cmd[0]=VAR_PAGE_WRITE1; cmd[1]=0;
            for(cmd[1]=0;cmd[1]<NPAGES/*last_addr*/;cmd[1]++){//скармливаем файл по 256 байт

                char *pd=data;
                int wr=0;
                for(j=0;j<PAGE_SIZP;j++) {
                   if(pd[j]!=0xff) {
                       wr=1;
                       break;
                   }
                }

                if(wr){
                    rc=mb_write_regs(VAR_DATA_REGISTER, 64, (uint16_t *) &data[0]);
                    if(rc < 0) return rc;
                    rc=mb_write_regs(VAR_DATA_REGISTER+64, 64, (uint16_t *) &data[128]);
                    if(rc < 0) return rc;
                    cmd[0]=VAR_PAGE_WRITE1;
                    rc=mb_write_regs(VAR_COMMAND_REGISTER, 2, cmd);
                    if(rc < 0) return rc;
                    for(j=0;j<100;j++){
                        rc=mb_read_regs (VAR_COMMAND_REGISTER, 1, cmd);
                        if(rc < 0) return rc;
                        if(!cmd[0]) break;
                    }
                    //rc=modbus_set_debug(ctx, FALSE);
                    //if(rc < 0) return rc;
                    fprintf(stderr,"%x\r",cmd[1]);
                }

                data+=PAGE_SIZP;
            }
            printf("\n ++++++++++ FPGA conf. flash successfully programmed! ++++++++++\n\n");
         }
if(mode & VFPGA){
if (data1 == NULL){
    data = read_hexfile (hexfile, &last_addr);

    if (data == NULL)
        return -4;

    printf("Size          : %ld Bytes\n", last_addr );
    printf("-------------------------------------------------\n");
}
else data=data1;
    printf("Verifing conf. flash ...\n");
    // read the file

rc=modbus_set_debug(ctx, mb_set_debug);
if(rc < 0) return rc;
for(cmd[1]=0; cmd[1]<NPAGES; cmd[1]++){
    cmd[0]=VAR_PAGE_READ;
    rc=mb_write_regs(VAR_COMMAND_REGISTER, 2, cmd);
    if(rc < 0) return rc;
    for(j=0;j<100;j++){
        rc=mb_read_regs (VAR_COMMAND_REGISTER, 1, cmd);
        if(rc < 0) return rc;
        if(!cmd[0]) break;
    }
    //rc=modbus_set_debug(ctx, FALSE);
    //if(rc < 0) return rc;
    rc=mb_read_regs(VAR_DATA_REGISTER, 64, (uint16_t *) &read_buffer[0]);
    if(rc < 0) return rc;
    rc=mb_read_regs(VAR_DATA_REGISTER+64, 64, (uint16_t *) &read_buffer[128]);
    if(rc < 0) return rc;
    //fwrite (read_buffer , sizeof(char), sizeof(read_buffer), fp);
    char *pread_buffer=read_buffer;
    for(j=0;j<PAGE_SIZP;j++)
        if(*data++!=*pread_buffer++) {
            data--;
            pread_buffer--;
            fprintf(stderr,"Verifacation error @0x%x 0x%x read%x file%x",cmd[1],j,data,*pread_buffer);
            return -1;
        }
    fprintf(stderr,"%x\r",cmd[1]);
}
    //fclose (fp);
    printf("\nVerification succesful...\n");
    return 0;
}
//*****************************************************
if(mode & (RFPGA)) {
    //cmd[0]=VAR_PAGE_READ;cmd[1]=0;
    printf("Reading conf. flash ...\n");


    if ((fp = fopen(hexfile, "wb")) == NULL)
    {
            printf( "Error: can't open file \"%s\"\n", hexfile);
            return -1;
    }

rc=modbus_set_debug(ctx, mb_set_debug);
if(rc < 0) return rc;
for(cmd[1]=0; cmd[1]<NPAGES; cmd[1]++){
    cmd[0]=VAR_PAGE_READ;
    rc=mb_write_regs(VAR_COMMAND_REGISTER, 2, cmd);
    if(rc < 0) return rc;
    for(j=0;j<100;j++){
        rc=mb_read_regs (VAR_COMMAND_REGISTER, 1, cmd);
        if(rc < 0) return rc;
        if(!cmd[0]) break;
    }
    //rc=modbus_set_debug(ctx, FALSE);
    //if(rc < 0) return rc;
    rc=mb_read_regs(VAR_DATA_REGISTER, 64, (uint16_t *) &read_buffer[0]);
    if(rc < 0) return rc;
    rc=mb_read_regs(VAR_DATA_REGISTER+64, 64, (uint16_t *) &read_buffer[128]);
    if(rc < 0) return rc;
    fwrite (read_buffer , sizeof(char), sizeof(read_buffer), fp);
    fprintf(stderr,"%x\r",cmd[1]);
}
    fclose (fp);
    printf("\n");
    return 0;
}

    return 0;
}
//************************************MODBUS_REGS******************************************************
int prog_mb_regs(int            fd,
                 uint32_t       mode,
                 int            baud,
                 int            block_size,
                 const char     *password,
                 const char     *device,
                 const char     *hexfile)
{
    char        *data = NULL;
    char        *data1 = NULL;
    char read_buffer[256];
    FILE * fp;
    // last address in hexfile
    uint32_t adr,last_addr = 0;
    uint16_t cmd[2];
    int i,j;
    int rc;


    printf("File           : %s\n", hexfile);
    rc=mb_open_master(device, baud);
    if(rc < 0) return rc;
    rc=mb_slave_addr(mb_addr);
    if(rc < 0) return rc;

    if (mode & (WCOILS))
            {
        printf("Starting adress: %d\n", mb_areg);
        printf("Number of flags: %d\n", mb_nreg);
        printf("Mode           : Wr coils\n");

        for(j=0;j<mb_nreg;j++)//    static uint16_t reg[128];
            read_buffer[j]=reg[j];//int modbus_write_bits(modbus_t *ctx, int addr, int nb, const uint8_t *data);

        rc=mb_write_coils(mb_areg, mb_nreg, read_buffer);
        if(rc < 0) return rc;
    }

    if (mode & (RCOILS))
            {
        printf("Starting adress: %d\n", mb_areg);
        printf("Number of flags: %d\n", mb_nreg);
        printf("Mode           : Rd coils\n");
        rc=mb_read_coils(mb_areg, mb_nreg, read_buffer);
        if(rc < 0) return rc;

        printf("\n");
        for(j=0;j<mb_nreg;j++) printf("%1d ",read_buffer[j]);
        printf("\n");

    }

    if (mode & (RDISCS))
            {
        printf("Starting adress: %d\n", mb_areg);
        printf("Number of flags: %d\n", mb_nreg);
        printf("Mode           : Rd discs\n");
        rc=mb_read_discs(mb_areg, mb_nreg, read_buffer);
        if(rc < 0) return rc;

        printf("\n");
        for(j=0;j<mb_nreg;j++) printf("%1d ",read_buffer[j]);
        printf("\n");

    }

    if (mode & (WREG))
            {
        printf("Starting adress: %d\n", mb_areg);
        printf("Number of flags: %d\n", mb_nreg);
        printf("Mode           : Wr regs\n");
        rc=mb_write_regs(mb_areg, mb_nreg, reg);
        if(rc < 0) return rc;
    }

    if (mode & (WREG1))
            {
        printf("Starting adress: %d\n", mb_areg);
        printf("Mode           : Wr reg\n");
        rc=mb_write_reg(mb_areg, reg);
        if(rc < 0) return rc;
    }

    if (mode & (WREGF))
            {
        // read the file
        data = read_hexfile (hexfile, &last_addr);
        if (data == NULL)
            return -4;

        printf("Size          : %ld Bytes\n", last_addr);
        printf("-------------------------------------------------\n");

        printf("Starting adress: %d\n", mb_areg);
        printf("Number of flags: %d\n", mb_nreg);
        printf("Mode           : Wr regs from file\n");
        rc=mb_write_regs(mb_areg, mb_nreg, (uint16_t *) data);
        if(rc < 0) return rc;
    }

    if (mode & (RREG))
            {
        printf("Starting adress: %d\n", mb_areg);
        printf("Number of flags: %d\n", mb_nreg);
        printf("Mode           : Rd regs\n");
        rc=mb_read_regs(mb_areg, mb_nreg, (uint16_t *)read_buffer);
        if(rc < 0) return rc;
        uint16_t *pread_buffer=read_buffer;
        printf("\n");
        for(j=0;j<mb_nreg;j++) printf("%05d ",pread_buffer[j]);
        printf("\n");
        for(j=0;j<mb_nreg;j++) printf("x%04X ",pread_buffer[j]);
        printf("\n");
    }

    if (mode & (RIREG))
            {
        printf("Starting adress: %d\n", mb_areg);
        printf("Number of flags: %d\n", mb_nreg);
        printf("Mode           : Rd input regs\n");
        rc=mb_read_input_regs(mb_areg, mb_nreg, (uint16_t *)read_buffer);
        if(rc < 0) return rc;
        uint16_t *pread_buffer=read_buffer;
        printf("\n");
        for(j=0;j<mb_nreg;j++) printf("%05d ",pread_buffer[j]);
        printf("\n");
        for(j=0;j<mb_nreg;j++) printf("x%04X ",pread_buffer[j]);
        printf("\n");
    }

    if (mode & (RREGF))
            {
        if ((fp = fopen(hexfile, "wb")) == NULL)
        {
                printf( "Error: can't open file \"%s\"\n", hexfile);
                return -1;
        }
        printf("Starting adress: %d\n", mb_areg);
        printf("Number of flags: %d\n", mb_nreg);
        printf("Mode           : Rd regs to file\n");
        rc=mb_read_regs(mb_areg, mb_nreg, (uint16_t *) read_buffer);
        if(rc < 0) {fclose(fp);return rc;}
        //for(j=0;j<mb_nreg;j++) printf("%d ",reg[j]);
        fwrite (read_buffer , sizeof(char), sizeof(read_buffer), fp);
        //printf("\n");
        if(fp) fclose(fp);
    }

    if (mode & (RIREGF))
            {
        if ((fp = fopen(hexfile, "wb")) == NULL)
        {
                printf( "Error: can't open file \"%s\"\n", hexfile);
                return -1;
        }
        printf("Starting adress: %d\n", mb_areg);
        printf("Number of flags: %d\n", mb_nreg);
        printf("Mode           : Rd input regs to file\n");

        int regs=mb_nreg;
        int areg=mb_areg;

        while(regs>0){

        int regs2=regs;

        if(regs2>120) regs2=120;

        rc=mb_read_input_regs(areg, regs2, (uint16_t *) read_buffer);
        if(rc < 0) {if(fp)fclose(fp);return rc;}

        //rc=modbus_set_debug(ctx, FALSE);
        //if(rc < 0) return rc;

        fwrite (read_buffer , sizeof(char), regs2*2, fp);
        fprintf(stderr,"%x\r",mb_areg);
        regs-=120;
        areg+=120;
        }
        if(fp) fclose(fp);
    }
    mb_close();
    return 0;
}

//******************************************************************************************************

/**
 * Main, startup
 */
int main(int argc, char *argv[])
{
    int     fd = 0;
    uint32_t     mode = 0;

    // default values
    int baudid = -1;
    int rc=0;

    struct tms timestruct;
    struct sigaction sa;

    sa.sa_handler = sig_handler;
    sa.sa_flags = SA_NOMASK;

    sigaction (SIGHUP, &sa, NULL);
    sigaction (SIGINT, &sa, NULL);
    sigaction (SIGQUIT, &sa, NULL);
    sigaction (SIGTERM, &sa, NULL);

    /* set start time for stopwatch */
    start  = times (&timestruct);
    ticks = (double) sysconf (_SC_CLK_TCK);

    // print header
    printf("\n");
    printf("=================================================\n");
    printf("|                MODBUS_RTU_LOADER              |\n");
    printf("|            (" __DATE__ " " __TIME__ ")             |\n");
    printf("=================================================\n");

    // Parsing / checking parameter
    int i,j;

    for(i = 1; i < argc; i++)
    {
        if (strcmp (argv[i], "-d") == 0)
        {
            i++;
            if (i < argc)
                device = argv[i];
        }
        else if (strcmp (argv[i], "-b") == 0)
        {
            i++;
            if (i < argc)
                baud = atoi(argv[i]);
        }
  //*******************************************MODBUS**************
        else if (strcmp (argv[i], "-m") == 0)
        {
            i++;
            if (i < argc)
                mb_addr = atoi(argv[i]);
        }
        else if (strcmp (argv[i], "-a") == 0)
        {
            i++;
            if (i < argc)
                mb_addr_boot = atoi(argv[i]);
        }
        else if (strcmp (argv[i], "-stop") == 0)
        {
            i++;
            if (i < argc)
                mb_stop_bits = atoi(argv[i]);
                if(mb_stop_bits>2) mb_stop_bits=2;
                if(mb_stop_bits<1) mb_stop_bits=1;
        }
        else if (strcmp (argv[i], "-echo") == 0)
        {
                rs232_echo=1;
        }
        else if (strcmp (argv[i], "-debug") == 0)
        {
                mb_set_debug=1;
        }
        else if (strcmp (argv[i], "-manc") == 0)
        {
                rs485_manc=1;
        }
        else if (strcmp (argv[i], "-nreg") == 0)
        {
            i++;
            if (i < argc)
                mb_nreg = atoi(argv[i]);
        }
        else if (strcmp (argv[i], "-areg") == 0)
        {
            i++;
            if (i < argc)
                mb_areg = atoi(argv[i]);
        }
        else if (strcmp (argv[i], "-wreg") == 0)
        {

            if (i < argc)
                mode |= WREG;
            for(j=0;j<mb_nreg;j++) {
                i++;
                reg[j]=atoi(argv[i]);
            }
        }

        else if (strcmp (argv[i], "-wregf") == 0)
        {
            //i++;
            if (i < argc)
                mode |= WREGF;
        }

        else if (strcmp (argv[i], "-wreg1") == 0)
        {
            //i++;
            if (i < argc)
                mode |= WREG1;

                i++;
                reg[0]=atoi(argv[i]);

        }

        else if (strcmp (argv[i], "-rreg") == 0)
        {
            //i++;
            if (i < argc)
                mode |= RREG;
        }

        else if (strcmp (argv[i], "-rireg") == 0)
        {
            //i++;
            if (i < argc)
                mode |= RIREG;
        }

        else if (strcmp (argv[i], "-rregf") == 0)
        {
            //i++;
            if (i < argc)
                mode |= RREGF;
        }

        else if (strcmp (argv[i], "-riregf") == 0)
        {
            //i++;
            if (i < argc)
                mode |= RIREGF;
        }

        else if (strcmp (argv[i], "-rcoils") == 0)
        {
            //i++;
            if (i < argc)
                mode |= RCOILS;
        }

        else if (strcmp (argv[i], "-wcoils") == 0)
        {

            if (i < argc)
                mode |= WCOILS;
            for(j=0;j<mb_nreg;j++) {
                i++;
                reg[j]=atoi(argv[i]);
            }
        }

        else if (strcmp (argv[i], "-rdiscs") == 0)
        {
            //i++;
            if (i < argc)
                mode |= RDISCS;
        }

        else if (strcmp (argv[i], "-p") == 0)
        {
            mode |= PROGRAM;
        }
        else if (strcmp (argv[i], "-p88") == 0)
        {
            mode |= PROGRAM88;
        }
   //*********************************FPGA********************
        else if (strcmp (argv[i], "-pfpga") == 0)
        {
            mode |= PFPGA;
        }

        else if (strcmp (argv[i], "-rfpga") == 0)
        {
            mode |= RFPGA;
        }

        else if (strcmp (argv[i], "-vfpga") == 0)
        {
            mode |= VFPGA;
        }
    //*********************************EEPROM********************
        else if (strcmp (argv[i], "-peeprom") == 0)
        {
            mode |= PEEPROM;

        }

        else if (strcmp (argv[i], "-reeprom") == 0)
        {
            mode |= REEPROM;
        }

        else if (strcmp (argv[i], "-veeprom") == 0)
        {
            mode |= VEEPROM;
        }

        else if (strcmp (argv[i], "-rram") == 0)
        {
            mode |= RRAM;
        }

        else
        {
            hexfile = argv[i];
        }
    }
   //**********************************************************************************************
    if ((hexfile == NULL) && (mode & (PROGRAM | PFPGA | RFPGA | VFPGA)))
    {
        printf("No hexfile specified!\n");
        usage();
    }

    if (mode == 0)
    {
        printf("No Verify / Program specified!\n");
        usage();
    }

    // Checking baudrate
    for(i = 0; i < (sizeof (baudrates) / sizeof (baudInfo_t)); i++)
    {
        if (baudrates[i].value == baud)
        {
            baudid = i;
            break;
        }
    }

    if(baudid == -1)
    {
        printf("Unknown baudrate (%i)!\nUse one of:", baud);
        for(i = 0; i < (sizeof (baudrates) / sizeof (baudInfo_t)); i++)
        {
            printf (" %ld", baudrates[i].value);
        }
        printf ("\n");
        usage();
    }



    if (mode & PROGRAM ){
        fd = com_open(device, baudrates[baudid].constval);

        if (fd < 0)
        {
            printf("Opening com port \"%s\" failed (%s)!\n",
                   device, strerror (errno));
            exit(2);
        }

        prog_verify (fd, mode, baud, bsize, password, device, hexfile);

        com_close(fd);                //close open com port
    }

    if (mode & PROGRAM88 ){
        rc=prog_verify88 (baud,baudid,device,hexfile);
        return rc;
    }

    if (mode & (WREG | RREG | RIREG | RREGF  | RIREGF | WREGF | WREG1 | RCOILS | WCOILS | RDISCS ) ){
        rc=prog_mb_regs(fd, mode, baud, bsize, password, device, hexfile);
    }

    if (mode & (PFPGA | RFPGA | VFPGA) )
        rc=prog_fpga (fd, mode, baud, bsize, password, device, hexfile);

    if (mode & (PEEPROM | REEPROM | VEEPROM | RRAM) )
        rc=prog_eeprom (fd, mode, baud, bsize, password, device, hexfile);

    if(rc < 0){
        mb_close();
        return rc;
    }
    return 0;
}

/* end of file */


