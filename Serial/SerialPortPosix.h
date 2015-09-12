/* Copyright (c) 2014 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <sys/time.h>
#include <sys/ioctl.h>

#include <stdexcept>

class SerialPort
{
public:
    SerialPort()
        : fd(-1)
    {
    }

    SerialPort(std::string device_pattern, int dev_index = 0)
        : fd(-1)
    {
        try{
            Open(device_pattern, dev_index);
        } catch (std::runtime_error err) {
            std::cerr << "Unable to connect to Meta IMU." << std::endl;
        }
    }


    ~SerialPort()
    {
        Close();
    }

    bool IsOpen()
    {
        return fd >= 0;
    }

    void Open(std::string device_pattern, int dev_index = 0)
    {
        if (IsOpen()) {
            Close();
        }

        const std::string path = FindDevice(device_pattern, dev_index);
        if (path.empty()) {
            throw std::runtime_error("Device not connected");
        }

        fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL ) & ~O_NONBLOCK);

        if (fd == -1) {
            perror("Error opening port");
            throw std::runtime_error("Unable to open com port");
        }

        if (tcgetattr(fd, &defaults) < 0) {
            perror("tcgetattr");  		// Grab snapshot of current settings  for port
        }

        options = defaults;


        //        cfmakeraw(&config);
        //        config.c_oflag = 0; // raw output
        //        config.c_lflag = 0; // raw input

        //set the baud rate to 115200
        int baudRate = B115200;
        cfsetospeed(&options, baudRate);
        cfsetispeed(&options, baudRate);

        //set the number of data bits.
        options.c_cflag &= ~CSIZE;  // Mask the character size bits
        options.c_cflag |= CS8;

        //set the number of stop bits to 1
        options.c_cflag &= ~CSTOPB;

        //Set parity to None
        options.c_cflag &= ~PARENB;

        //set for non-canonical (raw processing, no echo, etc.)
        options.c_iflag = IGNPAR; // ignore parity check close_port(int
        options.c_oflag = 0; // raw output
        options.c_lflag = 0; // raw input

        //Time-Outs -- won't work with NDELAY option in the call to open
        options.c_cc[VMIN] = 0;   // block reading until RX x characers. If x = 0, it is non-blocking.
        options.c_cc[VTIME] = 100;   // Inter-Character Timer -- i.e. timeout= x*.1 s

        //Set local mode and enable the receiver
        options.c_cflag |= (CLOCAL | CREAD);

        Purge();

        if (tcsetattr(fd, TCSANOW, &options) < 0) {
            perror("tcsetattr config");   	// Set options for port
        }

        Purge();
    }

    void Close()
    {
        if (IsOpen()) {
            if (tcsetattr(fd, TCSANOW, &defaults) < 0) {
                perror("tcsetattr default");	// Restore port default before closing
            }

            close(fd);
            fd = -1;
        }
    }

    ssize_t Write(const unsigned char* buffer, size_t num_bytes)
    {
        return write(fd, buffer, num_bytes);
    }

    void FinishWrite()
    {
        if (tcdrain(fd) < 0) perror("Error waiting");
    }

    // Returns true if data is available.
    bool WaitForRead(timeval timeout)
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        // Wait up to timeout until data input available.
        const int res = select(fd + 1, &fds, NULL, NULL, &timeout);

        return res > 0;
    }

    bool WaitForRead(time_t seconds, suseconds_t microseconds = 0)
    {
        struct timeval timeout;
        timeout.tv_sec = seconds;
        timeout.tv_usec = microseconds;

        return WaitForRead(timeout);
    }

    ssize_t Read(unsigned char* buffer, size_t num_bytes)
    {
        ssize_t b_read = read(fd, buffer, num_bytes);

        if (b_read < 0) {
            const int err = errno;
            printf("ERR reading: %s (%i)\n", strerror(err), err);
        }

        return b_read;
    }

    ssize_t ReadCount()
    {
        int bytes_available;
        ioctl(fd, FIONREAD, &bytes_available);
        return bytes_available;
    }

    int Purge()
    {
        if (tcflush(fd, TCIOFLUSH) == -1){
            printf("flush failed\n");
            return 0;
        }
        return 1;
    }

protected:

    std::string FindDevice(std::string device_pattern, int index = 0)
    {
        // device should be something like /dev/serial/ttyUSB* for linux or
        //                                 /dev/cu.usbmodem* for MacOS
        std::string cmd = "find " + device_pattern + " -print 2> /dev/null";
        FILE* instream = popen(cmd.c_str(), "r");

        if (!instream) {
            std::cerr << "Unable to open pipe." << std::endl;
            return 0;
        }

        int devfound;
        char devname[255];
        for (devfound = -1; devfound < index && fgets(devname, sizeof(devname), instream); ++devfound) {
        }

        if (devfound >= index) {
            devname[strlen(devname) - 1] = '\0';
            return std::string(devname);
        }

        return std::string();
    }

private:
    int fd;
    struct termios defaults;
    struct termios options;
};
