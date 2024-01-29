#ifndef UART_H
#define UART_H

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

class Uart {
    public:
        Uart(const char *device);
        ~Uart();
        int uart_write(const char* data, int length);
        int uart_read(char* buffer, int length);

        int fd;
};

#endif // UART_H