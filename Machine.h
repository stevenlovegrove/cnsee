#pragma once

#include <thread>
#include <Eigen/Eigen>

#include "GcodeProgram.h"
#include "Serial/SerialPort.h"
#include <string.h>
#include <queue>

std::vector<std::string>& Split(const std::string& s, char delim, std::vector<std::string>& elements) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elements.push_back(item);
    }
    return elements;
}

std::vector<std::string> Split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    return Split(s, delim, elems);
}

namespace Eigen
{
    inline std::istream& operator>>( std::istream& is, Eigen::Vector3d& mat)
    {
        size_t rows = mat.rows();
        for( size_t r = 0; r < rows-1; r++ ) {
            is >> mat[r];
            is.get();
        }
        is >> mat[rows-1];
        return is;
    }
}

namespace cnsee {
    enum MachineStatus {
        Disconnected,
        Unknown,
        Idle,
        Running,
        Alarm
    };

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
                  status(Disconnected),
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

            {
                std::unique_lock<std::mutex> lock(cmd_queue_mutex);
                // Add to cmd buffer size 'in transit'
                cmd_size_outstanding += size;
                cmd_size_queue.push(size);
            }

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

        void RequestStatus()
        {
            if(status != Disconnected) {
                SendLine("?");
            }
        }

    private:
        void ClearBuffer() {
            parse_start = buffer.get();
            parse_end = parse_start;
        }

        void QueueResponse(bool success)
        {
            std::unique_lock<std::mutex> lock(cmd_queue_mutex);

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

        bool ParseStatus(char *start, char *end) {
            char* end_status = std::find(start, end, ',');
            if(end_status == end) return false;

            std::string str_status(start+1, end_status);
            if( !str_status.compare(0,4,"Idle") ) {
                status = Idle;
            }else if( !str_status.compare(0,3,"Run") ) {
                status = Running;
            }else if( !str_status.compare(0,4,"Alarm") ) {
                status = Alarm;
            }else{
                status = Unknown;
            }

            char* m_pos = std::find(end_status+1, end, 'M');
            char* w_pos = std::find(end_status+1, end, 'W');
            if(m_pos == end || w_pos == end ) return false;

            std::string str_mpos(m_pos +5, w_pos -1);
            std::string str_wpos(w_pos +5, end-1);

            mpos = pangolin::Convert<Eigen::Vector3d,std::string>::Do(str_mpos);
            wpos = pangolin::Convert<Eigen::Vector3d,std::string>::Do(str_wpos);

            std::cout << mpos.transpose() << std::endl;
            std::cout << wpos.transpose() << std::endl;
            return true;
        }

        void ParseLine(char *start, char *end) {
            ssize_t size = end - start;
            if (size > 0) {
                if (start[0] == '[') {
                    std::cout << "feedback: " << std::string(start, end) << std::endl;
                } else if (start[0] == '<') {
                    ParseStatus(start, end);
//                    std::cout << "status: " << std::string(start, end) << std::endl;
                    QueueResponse(true);
                } else if (size > 4 && !strncmp(start, "Grbl", 4)) {
                    status = Unknown;
                    std::cout << "Grbl connection established." << std::endl;
                } else if (size == 2 && !strncmp(start, "ok", size)) {
                    QueueResponse(true);
                } else if (size > 5 && !strncmp(start, "error", 5)) {
                    QueueResponse(false);
                    std::cout << std::string(start, end) << std::endl;
                } else if (start[0] == '\r' || start[0] == '\n') {
                    ParseLine(start+1, end);
                } else {
                    std::cout << "other: (" << (int) start[0] << ", len " << end - start << ") " <<
                    std::string(start, end) << std::endl;
                    exit(-1);
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
                        parse_start = newln + 1;
                        while(parse_start < parse_end && (parse_start[0] == '\r' || parse_start[0] == '\n') ) {
                            ++parse_start;
                        }
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

        std::mutex cmd_queue_mutex;
        std::thread read_thread;
        std::thread write_thread;
        SerialPort serial;

        size_t max_buffer_size;
        std::unique_ptr<char[]> buffer;
        char *parse_start;
        char *parse_end;

        volatile MachineStatus status;

        std::queue<size_t> cmd_size_queue;
        volatile ssize_t cmd_size_outstanding;

        Eigen::Vector3d wpos;
        Eigen::Vector3d mpos;
    };

}