#pragma once

#include <thread>
#include <Eigen/Eigen>


#include "Serial/SerialPort.h"

class Machine
{
    virtual void EmergencyStop() = 0;
};

class GerblMachine : public Machine
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GerblMachine()
        : should_run(false)
    {
    }

    ~GerblMachine()
    {
        if(read_thread.joinable()) {
            should_run = false;
            read_thread.join();
        }
    }

    GerblMachine(const std::string& serial_port)
        : should_run(false)
    {
        Open(serial_port);
    }

    void Open(const std::string& serial_port)
    {
        // Open Serial port to Gerbl Arduino program
        serial.Open(serial_port);

        // Start read thread
        should_run = true;
        read_thread = std::thread(&GerblMachine::ReadLoop, this);
        write_thread = std::thread(&GerblMachine::WriteLoop, this);
    }

    void EmergencyStop() override
    {
    }

private:
    void ReadLoop()
    {
        const size_t buffer_size = 10240;
        unsigned char buffer[buffer_size];

        while(should_run)
        {
            int num_read = serial.Read(buffer, buffer_size);
            if(num_read) {

            }
        }
    }

    void WriteLoop()
    {
        // TODO: Send queued data
    }


    volatile bool should_run;
    std::thread read_thread;
    std::thread write_thread;

    SerialPort serial;
};
