#ifndef MODBUSASCII_H
#define MODBUSASCII_H

#include "modbus.h"
#include "modbus-private.h"


#ifndef _MSC_VER
#include <stdint.h>
#else
#include "stdint.h"
#endif

#if defined(_WIN32)
#include <windows.h>
#else
#include <termios.h>
#endif

#define _MODBUS_ASCII_HEADER_LENGTH      1
#define _MODBUS_ASCII_PRESET_REQ_LENGTH  13
#define _MODBUS_ASCII_PRESET_RSP_LENGTH  5

#define _MODBUS_ASCII_CHECKSUM_LENGTH    2

#if defined(_WIN32)
#if !defined(ENOTSUP)
#define ENOTSUP WSAEOPNOTSUPP
#endif

/* WIN32: struct containing serial handle and a receive buffer */
#define PY_BUF_SIZE 512
struct win32_ser {
    /* File handle */
    HANDLE fd;
    /* Receive buffer */
    uint8_t buf[PY_BUF_SIZE];
    /* Received chars */
    DWORD n_bytes;
};
#endif /* _WIN32 */

typedef struct _modbus_ascii {
    /* Device: "/dev/ttyS0", "/dev/ttyUSB0" or "/dev/tty.USA19*" on Mac OS X. */
    char *device;
    /* Bauds: 9600, 19200, 57600, 115200, etc */
    int baud;
    /* Data bit */
    uint8_t data_bit;
    /* Stop bit */
    uint8_t stop_bit;
    /* Parity: 'N', 'O', 'E' */
    char parity;
#if defined(_WIN32)
    struct win32_ser w_ser;
    DCB old_dcb;
#else
    /* Save old termios settings */
    struct termios old_tios;
#endif
#if HAVE_DECL_TIOCSRS485
    int serial_mode;
#endif
#if HAVE_DECL_TIOCM_RTS
    int rts;
    int rts_delay;
    int onebyte_time;
    void (*set_rts) (modbus_t *ctx, int on);
#endif
    /* To handle many slaves on the same link */
    int confirmation_to_ignore;
} modbus_ascii_t;

MODBUS_BEGIN_DECLS

/* Modbus_Application_Protocol_V1_1b.pdf Chapter 4 Section 1 Page 5
 * RS232 / RS485 ADU = 253 bytes + slave (1 byte) + CRC (2 bytes) = 256 bytes
 */
#define MODBUS_ASCII_MAX_ADU_LENGTH  256

MODBUS_API modbus_t* modbus_new_ascii(const char *device, int baud, char parity,
                                    int data_bit, int stop_bit);

#define MODBUS_ASCII_RS232 0
#define MODBUS_ASCII_RS485 1

MODBUS_API int modbus_ascii_set_serial_mode(modbus_t *ctx, int mode);
MODBUS_API int modbus_ascii_get_serial_mode(modbus_t *ctx);

#define MODBUS_ASCII_RTS_NONE   0
#define MODBUS_ASCII_RTS_UP     1
#define MODBUS_ASCII_RTS_DOWN   2

MODBUS_API int modbus_ascii_set_rts(modbus_t *ctx, int mode);
MODBUS_API int modbus_ascii_get_rts(modbus_t *ctx);

MODBUS_API int modbus_ascii_set_custom_rts(modbus_t *ctx, void (*set_rts) (modbus_t *ctx, int on));

MODBUS_API int modbus_ascii_set_rts_delay(modbus_t *ctx, int us);
MODBUS_API int modbus_ascii_get_rts_delay(modbus_t *ctx);

MODBUS_END_DECLS

MODBUS_API int modbus_read_input_registers_ascii(modbus_t *ctx, int addr, int nb, uint16_t *dest);
MODBUS_API int modbus_write_register_ascii(modbus_t *ctx, int reg_addr, int value);

#endif // MODBUSASCII_H
