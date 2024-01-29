#include "uart.h"

Uart::Uart(const char *device) 
{
    fd = open(device, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        printf("Can't open device.\n");
        usleep(1000000);
        return;
    }
    
    struct termios settings;
    tcgetattr(fd, &settings);
    cfsetospeed(&settings, B115200);
    cfsetispeed(&settings, B115200);
    settings.c_cflag &= ~PARENB;
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= CS8;
    settings.c_cflag &= ~CRTSCTS;
    settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tcsetattr(fd, TCSANOW, &settings);

    return;
}

Uart::~Uart() 
{
    close(fd);
}

int Uart::uart_write(const char* data, int length)
{
    return write(fd, data, length);
}

int Uart::uart_read(char* buffer, int length)
{
    return read(fd, buffer, length);
}