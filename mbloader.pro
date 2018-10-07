TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += mbloader.c \
           com.c \
    modbus.c \
    modbus-data.c \
    modbus-rtu.c \
    modbus-tcp.c \

    bin2csv.c

HEADERS += \
    com.h \
    protocol.h \
    modbus-version.h \
    modbus-tcp-private.h \
    modbus-tcp.h \
    modbus.h \
    modbus-rtu.h \
    modbus-rtu-private.h \
    modbus-tcp.h \
    modbus-private.h \
    config.h

DISTFILES += \
    libmodbus/docs/libmodbus.txt \
    libmodbus/docs/modbus_close.txt \
    libmodbus/docs/modbus_connect.txt \
    libmodbus/docs/modbus_flush.txt \
    libmodbus/docs/modbus_free.txt \
    libmodbus/docs/modbus_get_byte_from_bits.txt \
    libmodbus/docs/modbus_get_byte_timeout.txt \
    libmodbus/docs/modbus_get_float.txt \
    libmodbus/docs/modbus_get_float_abcd.txt \
    libmodbus/docs/modbus_get_float_badc.txt \
    libmodbus/docs/modbus_get_float_cdab.txt \
    libmodbus/docs/modbus_get_float_dcba.txt \
    libmodbus/docs/modbus_get_header_length.txt \
    libmodbus/docs/modbus_get_response_timeout.txt \
    libmodbus/docs/modbus_get_slave.txt \
    libmodbus/docs/modbus_get_socket.txt \
    libmodbus/docs/modbus_mapping_free.txt \
    libmodbus/docs/modbus_mapping_new.txt \
    libmodbus/docs/modbus_mapping_new_start_address.txt \
    libmodbus/docs/modbus_mask_write_register.txt \
    libmodbus/docs/modbus_new_rtu.txt \
    libmodbus/docs/modbus_new_tcp.txt \
    libmodbus/docs/modbus_new_tcp_pi.txt \
    libmodbus/docs/modbus_read_bits.txt \
    libmodbus/docs/modbus_read_input_bits.txt \
    libmodbus/docs/modbus_read_input_registers.txt \
    libmodbus/docs/modbus_read_registers.txt \
    libmodbus/docs/modbus_receive.txt \
    libmodbus/docs/modbus_receive_confirmation.txt \
    libmodbus/docs/modbus_reply.txt \
    libmodbus/docs/modbus_reply_exception.txt \
    libmodbus/docs/modbus_report_slave_id.txt \
    libmodbus/docs/modbus_rtu_get_rts.txt \
    libmodbus/docs/modbus_rtu_get_rts_delay.txt \
    libmodbus/docs/modbus_rtu_get_serial_mode.txt \
    libmodbus/docs/modbus_rtu_set_custom_rts.txt \
    libmodbus/docs/modbus_rtu_set_rts.txt \
    libmodbus/docs/modbus_rtu_set_rts_delay.txt \
    libmodbus/docs/modbus_rtu_set_serial_mode.txt \
    libmodbus/docs/modbus_send_raw_request.txt \
    libmodbus/docs/modbus_set_bits_from_byte.txt \
    libmodbus/docs/modbus_set_bits_from_bytes.txt \
    libmodbus/docs/modbus_set_byte_timeout.txt \
    libmodbus/docs/modbus_set_debug.txt \
    libmodbus/docs/modbus_set_error_recovery.txt \
    libmodbus/docs/modbus_set_float.txt \
    libmodbus/docs/modbus_set_float_abcd.txt \
    libmodbus/docs/modbus_set_float_badc.txt \
    libmodbus/docs/modbus_set_float_cdab.txt \
    libmodbus/docs/modbus_set_float_dcba.txt \
    libmodbus/docs/modbus_set_response_timeout.txt \
    libmodbus/docs/modbus_set_slave.txt \
    libmodbus/docs/modbus_set_socket.txt \
    libmodbus/docs/modbus_strerror.txt \
    libmodbus/docs/modbus_tcp_accept.txt \
    libmodbus/docs/modbus_tcp_listen.txt \
    libmodbus/docs/modbus_tcp_pi_accept.txt \
    libmodbus/docs/modbus_tcp_pi_listen.txt \
    libmodbus/docs/modbus_write_and_read_registers.txt \
    libmodbus/docs/modbus_write_bit.txt \
    libmodbus/docs/modbus_write_bits.txt \
    libmodbus/docs/modbus_write_register.txt \
    libmodbus/docs/modbus_write_registers.txt \
    mprg \
    pprg \
    readall \
    con \
    on \
    off \
    reset
