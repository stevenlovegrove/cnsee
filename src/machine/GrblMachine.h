#pragma once

#include <string>
#include <stack>
#include <cctype>
#include <deque>
#include <thread>
#include <algorithm>
#include <Eigen/Eigen>

#include "Machine.h"
#include "../gcode/GTokenize.h"
#include "../gcode/GLineBuilder.h"
#include "../Serial/SerialPort.h"
#include "../utils/StreamOperatorUtils.h"

namespace cnsee {

    class GrblMachine : public MachineInterface {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GrblMachine()
                : should_run(false),
                  max_read_buffer_size(10240),
                  read_buffer(new char[max_read_buffer_size]),
                  read_buffer_parse_start(read_buffer.get()),
                  read_buffer_parse_end(read_buffer_parse_start),
                  status(MachineStatus::Disconnected),
                  cmd_size_outstanding(0)
        {
        }

        ~GrblMachine() {
            should_run = false;
            queue_changed_cond.notify_all();

            if (write_thread.joinable()) {
                write_thread.join();
            }
            if (read_thread.joinable()) {
                read_thread.join();
            }
        }

        GrblMachine(const std::string &serial_port)
                : should_run(false) {
            Open(serial_port);
        }

        bool IsConnected() const
        {
            return status != MachineStatus::Disconnected;
        }

        void Open(const std::string &serial_port) {
            // Open Serial port to Gerbl Arduino program
            serial.Open(serial_port);

            // Start read thread
            should_run = true;
            read_thread = std::thread(&GrblMachine::ReadLoop, this);
            write_thread = std::thread(&GrblMachine::WriteLoop, this);
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Implement MachineInterface
        ////////////////////////////////////////////////////////////////////////////////

        void EmergencyStop() override {
            //
        }

        std::future<AckStatus> QueueCommand(const std::string& cmd) override
        {
            std::future<AckStatus> f;
            {
                std::unique_lock<std::mutex> l(queue_mutex);
                queued_commands.emplace_back(cmd);
                f = queued_commands.back().promise.get_future();
            }
            queue_changed_cond.notify_all();
            return f;
        }

        const std::shared_ptr<JobProgress> QueueProgram(const GProgram& program) override
        {
            throw std::runtime_error("Not implemented");
        }

        void RequestStatus()
        {
            // Only request status updates when we're connected and there aren't too many requests pending.
            if(IsConnected() && cmd_size_outstanding < 20) {
                QueueCommand("?\n");
            }
        }

        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Vector3d wpos;
        Eigen::Vector3d mpos;
        Eigen::Vector3d wco;
        Eigen::Vector2d feed_speed;

    private:
        struct PromisedCommand{
            PromisedCommand(const std::string& cmd) : cmd(cmd){}
            PromisedCommand(PromisedCommand&&) = default;

            std::string cmd;
            std::promise<AckStatus> promise;
        };

        ssize_t GrblBufferAvailable() const
        {
            return GRBL_RX_BUFFER_SIZE - cmd_size_outstanding;
        }

        // Send gcode directly over wire
        void SendLine(const char *gcode, size_t size) {
            if(size > GRBL_RX_BUFFER_SIZE) {
                throw std::runtime_error("Command is greater than max allowable line size (GRBL_RX_BUFFER_SIZE)");
            }

            // TODO: Use condition variable.
            // Busy-wait whilst buffer is full.
            while(size > GrblBufferAvailable()) {
            }

            cmd_size_outstanding += size;

            size_t total_written = 0;
            while (should_run && total_written < size) {
                const size_t written = serial.Write((unsigned char*) gcode, size);
                gcode += written;
                total_written += written;
            }
        }

        void SendLine(const std::string &gcode) {
            SendLine(gcode.c_str(), gcode.size());
        }

        static std::string SanitizedLine(const std::string& line)
        {
            std::string str(line);
            str.erase(std::remove_if(str.begin(), str.end(), ::isspace), str.end());
            return str;
        }

        void ClearBuffer() {
            read_buffer_parse_start = read_buffer.get();
            read_buffer_parse_end = read_buffer_parse_start;
        }

        bool ParseStatusNameVal(const std::string& name, const std::string& val)
        {
            if(!name.compare("MPos")) {
                mpos = pangolin::Convert<Eigen::Vector3d,std::string>::Do(val);
            }else if(!name.compare("WPos")) {
                wpos = pangolin::Convert<Eigen::Vector3d,std::string>::Do(val);
            }else if(!name.compare("WCO")) {
                wco = pangolin::Convert<Eigen::Vector3d,std::string>::Do(val);
            }else if(!name.compare("FS")) {
                feed_speed = pangolin::Convert<Eigen::Vector2d,std::string>::Do(val);
            }else if(!name.compare("Ov")) {
                // Ignore Overrides status
            }else{
                std::cerr << "Unknown status Name:Value pair: ('" << name << "' : '" << val << "')" << std::endl;
                return false;
            }
            return true;
        }

        bool ParseStatus(char *start, char *end) {

            char* end_status = std::find(start, end, '|');
            if(end_status == end) return false;

            const std::string str_status(start+1, end_status);
            if( !str_status.compare(0,4,"Idle") ) {
                status = MachineStatus::Idle;
            }else if( !str_status.compare(0,3,"Run") ) {
                status = MachineStatus::Running;
            }else if( !str_status.compare(0,5,"Alarm") ) {
                status = MachineStatus::Alarm;
            }else{
                status = MachineStatus::Unknown;
            }

            // Find all name:value pairs
            char* token_start = end_status+1;
            char* name_val_sep = std::find(token_start, end, ':');

            while(name_val_sep != end) {
                char* token_end = std::find(token_start, end, '|');
                if(token_end == end) {
                    token_end =  std::find(token_start, end, '>');
                    if(token_end == end) {
                        // Parse error
                        std::cerr << "Error parsing status: " << std::string(start, end) << std::endl;
                        return false;
                    }
                }

                const std::string name(token_start, name_val_sep);
                const std::string val(name_val_sep+1, token_end);
                ParseStatusNameVal(name,val);

                token_start = token_end+1;
                name_val_sep = std::find(token_start, end, ':');
            }

            return true;
        }

        void MachineConnected()
        {
            std::cout << "Grbl connection established." << std::endl;
            status = MachineStatus::Unknown;

            QueueCommand("$X\n");    // unlock
            QueueCommand("G21\n");   // mm units
            QueueCommand("$21=1\n"); // enable hard-limits
        }

        void ParseLine(char *start, char *end) {
            ssize_t size = end - start;
            if (size > 0) {
                if (start[0] == '[') {
                    std::cout << "feedback: " << std::string(start, end) << std::endl;
                } else if (start[0] == '<') {
                    ParseStatus(start, end);
                } else if (size >= 4 && !strncmp(start, "Grbl", 4)) {
                    MachineConnected();
                } else if (size == 2 && !strncmp(start, "ok", 2)) {
                    ReceivedAck(AckStatus::Ack);
                } else if (size >= 5 && !strncmp(start, "error", 5)) {
                    std::cerr << std::string(start, end) << std::endl;
                    ReceivedAck(AckStatus::Failed);
                } else if (size >= 5 && !strncmp(start, "ALARM", 5)) {
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

        void ReceivedAck(AckStatus status)
        {
            std::unique_lock<std::mutex> l(queue_mutex);
            if(unacked_commands.empty()) {
                std::cerr << "Warning: Received unmatched extra acknowledgement. Ignoring..." << std::endl;
            }else{
                unacked_commands.front().promise.set_value(status);
                cmd_size_outstanding -= unacked_commands.front().cmd.size();
                unacked_commands.pop_front();
            }
        }

        void ReadLoop() {
            while (should_run) {
                if (!ReadBufferBytesLeft()) {
                    // This really should never happen. If it does, we can shift buffer contents a bit.
                    throw std::runtime_error("Read buffer full");
                }

                if (serial.WaitForRead(0, 5000)) {
                    read_buffer_parse_end += serial.Read((unsigned char *) read_buffer_parse_end, ReadBufferBytesLeft());

                    char *newln = std::find(read_buffer_parse_start, read_buffer_parse_end, '\r');
                    while (newln != read_buffer_parse_end) {
                        ParseLine(read_buffer_parse_start, newln);
                        read_buffer_parse_start = newln + 1;
                        while(read_buffer_parse_start < read_buffer_parse_end && (read_buffer_parse_start[0] == '\r' || read_buffer_parse_start[0] == '\n') ) {
                            ++read_buffer_parse_start;
                        }
                        newln = std::find(read_buffer_parse_start, read_buffer_parse_end, '\r');
                    }
                    if (read_buffer_parse_start == read_buffer_parse_end) {
                        ClearBuffer();
                    }
                }
            }
        }

        void WriteLoop() {
            while(should_run) {
                {
                    std::unique_lock<std::mutex> l(queue_mutex);
                    while(queued_commands.empty()) {
                        queue_changed_cond.wait(l);
                        if(!should_run) return;
                    }
                    const std::string cmd = queued_commands.front().cmd;
//                    if(cmd.length() && cmd[0] != '?') {
                        unacked_commands.push_back(std::move(queued_commands.front()));
//                    }else{
//                        queued_commands.front().promise.set_value(AckStatus::NoOp);
//                    }
                    queued_commands.pop_front();
                    SendLine(cmd);
                }
            }
        }

        size_t ReadBufferBytesLeft() {
            return max_read_buffer_size - (read_buffer_parse_end - read_buffer.get());
        }

        ///////////////////////////////////////////////////////////////////////////////

        const static size_t GRBL_RX_BUFFER_SIZE = 127;

        SerialPort serial;
        size_t max_read_buffer_size;
        std::unique_ptr<char[]> read_buffer;
        char *read_buffer_parse_start;
        char *read_buffer_parse_end;

        volatile MachineStatus status;
        volatile bool should_run;
        std::thread read_thread;
        std::thread write_thread;

        std::deque<PromisedCommand> queued_commands;
        std::deque<PromisedCommand> unacked_commands;
        std::mutex queue_mutex;
        std::condition_variable queue_changed_cond;

        volatile ssize_t cmd_size_outstanding;

        // Programs get sent to the machines queue.
        struct ProgramAndProgress {
            GProgram program;
            std::shared_ptr<JobProgress> progrss;
        };
        std::stack<ProgramAndProgress> queued_programs;
    };

}
