#pragma once

#include <future>

#include "../gcode/GToken.h"
#include "../gcode/GMachineState.h"

#include <pangolin/utils/signal_slot.h>

namespace cnsee {

enum class AckStatus {
    Ack,
    NoOp,
    Failed,
    Aborted
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
    double running_time_s;
    double distance_moved;
};

struct ProbeResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool contact_made;

    Eigen::Vector3d probe_start;
    Eigen::Vector3d probe_direction;
    Eigen::Vector3d contact_point;
};

// Pure Virtual Interface
class MachineInterface {
public:
    virtual ~MachineInterface() {};

    // Abort any running operations and halt the machine
    virtual void EmergencyStop() = 0;

    // Queue GCode line
    virtual std::future<AckStatus> QueueCommand(const std::string& gcode_line) = 0;

    virtual std::future<ProbeResult> ProbeSurface(const Eigen::Vector3d& probe_direction, double feed_rate) = 0;

    //        virtual void Pause() = 0;
    //        virtual void Continue() = 0;
    //        virtual void CancelJob(size_t job_id) = 0;

};

}
