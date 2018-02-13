#pragma once

#include <Eigen/Core>

#include "GCodeCommand.h"

namespace cnsee {

enum class MachineStatus {
    Disconnected,
    Unknown,
    Idle,
    Running,
    Alarm
};

enum GUnits
{
    GUnits_inch, // G20
    GUnits_mm    // G21
};

enum GCoordinates
{
    GCoordinatesAbsolute, // G90
    GCoordinatesRelative  // G91
};

enum GPlane
{
    GXY,    // G17
    GZX,    // G18
    GYZ,    // G19
};

enum GFeedUnit
{
    GFeedDuration,
    GFeedUnitsPerTime,
    GFeedUnitsPerRevolution
};

struct GMachineState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GMachineState()
        : state(MachineStatus::Running),
          units(GUnits_mm),
          feed_unit(GFeedUnitsPerTime),
          coords(GCoordinatesAbsolute),
          plane(GXY),
          feed_rate(100),
          tool_offset(-1),
          P_w(0.0,0.0,0.0),
          active_cmd(Cmd::None),
          spindle_cw(0),
          spindle_speed(0),
          time_s(0.0f),
          distance_travelled(0.0f)
    {
    }

    MachineStatus state;
    GUnits units;
    GFeedUnit feed_unit;
    GCoordinates coords;
    GPlane plane;
    double feed_rate;
    int tool_offset;
    Eigen::Vector3d P_w;
    GCodeId active_cmd;
    int spindle_cw;
    double spindle_speed;

    // Meta information
    double time_s;
    double distance_travelled;
};

}
