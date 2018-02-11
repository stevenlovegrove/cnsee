#pragma once

#include <future>

#include "../gcode/GToken.h"
#include "../gcode/GMachineState.h"

#include <pangolin/utils/signal_slot.h>

namespace cnsee {

enum class AckStatus {
    Ack,
    NoOp,
    Failed
};

enum class JobStatus {
    Queued,
    Running,
    Finished,
    Failed
};

struct JobProgress {
    // Job ID
    size_t job_uid;

    // Progress
    JobStatus status;
    size_t line_index;
    size_t total_lines;
    GLine executing_line;

    // Stats
    float running_time_s;
    float distance_moved;
};

// Pure Virtual Interface
class MachineInterface {
public:
    virtual ~MachineInterface() {};

    // Abort any running operations and halt the machine
    virtual void EmergencyStop() = 0;

    // Queue GCode line
    virtual std::future<AckStatus> QueueCommand(const std::string& line) = 0;

    // Execute the gcode program
    // returns a shared progress object which can be used to query the programs state
    virtual const std::shared_ptr<JobProgress> QueueProgram(const GProgram& program) = 0;

    //        virtual void Pause() = 0;
    //        virtual void Continue() = 0;
    //        virtual void CancelJob(size_t job_id) = 0;

};

}