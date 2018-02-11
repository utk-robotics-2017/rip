#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            //TODO: replace with periphery
            class SerialPort
            {
                private:
                    int readBufferSize;
                    int timeout;
                    char terminationChar;
                    bool termination;
                    std::string id;
                    int baudRate;
                    int fd;
                    struct termios tty;
                    int err;

                public:

                SerialPort(int baudRate, std::string id)
                {
                    init(baudRate, id);
                }

                void init(int baudRate, std::string id)
                {
                    int USB = open(id.c_str(), O_RDWR| O_NOCTTY);
                    if(USB < 0)
                    {
                        //TODO: RIP exception handling
                        std::cerr << "Could not open " << id.c_str() << " as a TTY:";
                        perror("");
                        throw std::runtime_error("");
                    }

                    memset(&tty, 0, sizeof(tty));
                    this->baudRate = baudRate;
                    this->id = id;

                    cfsetospeed(&tty,(speed_t)baudRate);
                    cfsetispeed(&tty,(speed_t)baudRate);

                    tty.c_cflag &= ~PARENB;
                    tty.c_cflag &= ~CSTOPB;
                    tty.c_cflag &= ~CSIZE;
                    tty.c_cflag |= CS8;

                    tty.c_cc[VMIN] = 1;
                    tty.c_cc[VTIME] = 10;

                    tty.c_cflag |= CREAD | CLOCAL;

                    cfmakeraw(&tty);
                    this->fd = USB;
                    tcflush(this->fd, TCIOFLUSH);
                    //TODO: exception. handling.
                    if(tcsetattr(USB,TCSANOW,&tty) != 0) std::cout << "Failed to initialize serial." << std::endl;

                }

                void setReadBufferSize(int size)
                {
                    this->readBufferSize = size;
                }

                void setTimeout(int timeout)
                {
                    this->timeout = timeout;
                    tty.c_cc[VTIME] = timeout*10;
                    cfmakeraw(&tty);
                    //TODO: exception handling
                    if(tcsetattr(this->fd,TCSANOW,&tty) != 0) std::cout << "Failed to initialize serial in SetTimeout." << std::endl;;
                }

                void enableTermination(char c)
                {
                    this->termination = true;
                    this->terminationChar = c;
                }

                void flush()
                {
                    tcflush(this->fd, TCOFLUSH);
                }

                void write(char *data, int length)
                {
                    int n_written = 0, spot = 0;
                    do
                    {
                        n_written = ::write(this->fd, &data[spot], length);
                        if(n_written > 0)
                            spot += n_written;
                    } while(data[spot-1] != terminationChar);
                }

                int getBytesReceived()
                {
                    int bytes_avail;
                    ioctl(this->fd, FIONREAD, &bytes_avail);
                    return bytes_avail;
                }

                int read(char *data, int size)
                {
                    int n = 0, loc = 0;
                    char buf = '\0';
                    memset(data, '\0', size);
                    //error tracks number of errors
                    do
                    {
                        n = ::read(this->fd, &buf, 1);
                        sprintf(&data[loc], "%c", buf);
                        loc += n;

                        if(n == 0) err++;
                        //if read fails over 10 times
                        if(err > 10)
                        {
                            err = 0;
                            reset();
                            close();
                            init(this->baudRate, this->id);
                            setTimeout(this->timeout);
                            setReadBufferSize(this->readBufferSize);
                            enableTermination(this->terminationChar);
                            reset();
                            break;
                        }
                    } while(buf != terminationChar && loc < size);

                    if(n < 0)
                    {
                        //TODO: replace this garbage serial library
                        std::cout << "Error reading: " << strerror(errno) << std::endl;
                    }
                    else if(n == 0) {
                        std::cout << "Read nothing!" << std::endl;
                    }
                    else
                    {
                        //std::cout << "Response: " << data  << std::endl;
                    }
                    return loc;
                }

                void waitForData()
                {
                    fd_set readfds;
                    struct timeval tv;
                    FD_ZERO(&readfds);
                    FD_SET(this->fd, &readfds);
                    tv.tv_sec = 0;
                    tv.tv_usec = 100000;
                    select(this->fd + 1, &readfds, NULL, NULL, &tv);
                }

                void reset()
                {
                    tcflush(this->fd, TCIOFLUSH);
                }

                void close()
                {
                    ::close(this->fd);
                }
            };
        }
    }
}
