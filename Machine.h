#pragma once

#include <thread>
#include <Eigen/Eigen>

#include "GcodeProgram.h"
#include "Serial/SerialPort.h"
#include <string.h>
#include <queue>

namespace cnsee {

    class Machine {
    public:
        virtual ~Machine() { };

        virtual void EmergencyStop() = 0;
    };

    class GerblMachine : public Machine {
    public:
        typedef GCodeCmd<double> GCodeCmdT;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GerblMachine()
                : should_run(false),
                  max_buffer_size(10240),
                  buffer(new char[max_buffer_size]),
                  parse_start(buffer.get()),
                  parse_end(parse_start),
                  grbl_ready(false),
                  cmd_size_outstanding(0)
        {
        }

        ~GerblMachine() {
            if (read_thread.joinable()) {
                should_run = false;
                read_thread.join();
            }
        }

        GerblMachine(const std::string &serial_port)
                : should_run(false) {
            Open(serial_port);
        }

        void Open(const std::string &serial_port) {
            // Open Serial port to Gerbl Arduino program
            serial.Open(serial_port);

            // Start read thread
            should_run = true;
            read_thread = std::thread(&GerblMachine::ReadLoop, this);
//        write_thread = std::thread(&GerblMachine::WriteLoop, this);
        }

        void EmergencyStop() override {
        }

        ssize_t GrblBufferAvailable() const
        {
            return GRBL_RX_BUFFER_SIZE - cmd_size_outstanding;
        }

        void SendLine(const unsigned char *gcode, size_t size) {
            if(size > GRBL_RX_BUFFER_SIZE) {
                throw std::runtime_error("Command is greater than GRBL_RX_BUFFER_SIZE");
            }

            // TODO: Use condition variable.
            // Busy-wait whilst buffer is full.
            while(size > GrblBufferAvailable()) {
//                if(!should_run) return;
            }

            // Add to cmd buffer size 'in transit'
            cmd_size_outstanding += size;
            cmd_size_queue.push(size);

            size_t num_written = 0;
            while (should_run && num_written < size) {
                num_written += serial.Write(gcode, size);
            }
        }

        void SendLine(const std::string &gcode) {
            SendLine((const unsigned char *) gcode.c_str(), gcode.size());
        }

        void SendGCode(GCodeCmdT cmd)
        {
//            SendLine(...)
        }

        void Status()
        {
            SendLine("?");
        }


    private:
        void ClearBuffer() {
            parse_start = buffer.get();
            parse_end = parse_start;
        }

        void QueueResponse(bool success)
        {
            if(cmd_size_queue.size() > 0) {
                size_t size = cmd_size_queue.front();
                cmd_size_queue.pop();
                cmd_size_outstanding -= size;
                if( cmd_size_outstanding < 0) {
                    throw std::runtime_error("Mismatched cmd queue response.");
                }
            }else{
                throw std::runtime_error("Mismatched cmd queue response.");
            }
        }

        void ParseLine(char *start, char *end) {
            size_t size = end - start;
            if (size > 0) {
                if (start[0] == '[') {
                    std::cout << "feedback: " << std::string(start, end) << std::endl;
                } else if (start[0] == '<') {
                    std::cout << "status: " << std::string(start, end) << std::endl;
                    QueueResponse(true);
                } else if (size > 4 && !strncmp(start, "Grbl", 4)) {
                    grbl_ready = true;
                    std::cout << "Grbl connection established." << std::endl;
                } else if (size == 2 && !strncmp(start, "ok", size)) {
                    QueueResponse(true);
                } else if (size > 5 && !strncmp(start, "error", 5)) {
                    QueueResponse(false);
                    std::cout << std::string(start, end) << std::endl;
                } else {
                    std::cout << "other: (" << (int) start[0] << ", len " << end - start << ") " <<
                    std::string(start, end) << std::endl;
                }
            }
        }

        void ReadLoop() {
            while (should_run) {
                if (!BytesLeft()) {
                    // This really should never happen. If it does, we can shift buffer contents a bit.
                    throw std::runtime_error("Read buffer full");
                }

                if (serial.WaitForRead(0, 5000)) {
                    parse_end += serial.Read((unsigned char *) parse_end, BytesLeft());

                    char *newln = std::find(parse_start, parse_end, '\r');
                    while (newln != parse_end) {
                        ParseLine(parse_start, newln);
                        parse_start = newln + 2; // consume '\r\n'
                        newln = std::find(parse_start, parse_end, '\r');
                    }
                    if (parse_start == parse_end) {
                        ClearBuffer();
                    }
                }
            }
        }

        void WriteLoop() {
            // TODO: Send queued data
        }

        size_t BytesLeft() {
            return max_buffer_size - (parse_end - buffer.get());
        }

        const static size_t GRBL_RX_BUFFER_SIZE = 127;

        volatile bool should_run;

        std::thread read_thread;
        std::thread write_thread;
        SerialPort serial;

        size_t max_buffer_size;
        std::unique_ptr<char[]> buffer;
        char *parse_start;
        char *parse_end;

        volatile bool grbl_ready;

        std::queue<size_t> cmd_size_queue;
        volatile ssize_t cmd_size_outstanding;
    };

}