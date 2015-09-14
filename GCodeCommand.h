#pragma once

#include <iomanip>
#include <sstream>
#include "GCodeDescription.h"
#include "StreamOperatorUtils.h"

namespace cnsee
{
    inline GCodeId StringToGCodeId(const char* str, size_t size)
    {
        const GCodeId mask = (1ul << 8*size) - 1;
        return *reinterpret_cast<const GCodeId*>(str) & mask;
    }

    inline GCodeId StringToGCodeId(const std::string& str)
    {
        return StringToGCodeId(str.data(), str.size());
    }

    inline std::string GCodeIdToString(GCodeId id)
    {
        const size_t max_size = sizeof(GCodeId);
        std::string str(max_size,'\0');

        for(int i=0; i<max_size; ++i) {
            const GCodeId b = (id >> max_size*i) & 0xFF;
            if(!b) {
                str.resize(i);
                return str;
            }else{
                str[i] = (unsigned char)b;
            }
        }
        return str;
    }

    inline void OutputCmdCCode() {
        std::cout << "namespace Cmd" << std::endl;
        std::cout << "{" << std::endl;
        for(const GCodeDescription& cmd : gcode_descriptions) {
            std::cout << "    static const GCodeId " << cmd.desc << " = 0x" << std::hex << cnsee::StringToGCodeId(cmd.code) << "ul; // " << cmd.code << std::endl;
        }
        std::cout << "}" << std::endl;
    }

    namespace Cmd
    {
        static const GCodeId None = 0x0000ul;
        static const GCodeId RapidLinearMove = 0x3047ul; // G0
        static const GCodeId LinearMove = 0x3147ul; // G1
        static const GCodeId ArcMotion = 0x3247ul; // G2
        static const GCodeId HelicalMotion = 0x3347ul; // G3
        static const GCodeId Dwell = 0x3447ul; // G4
        static const GCodeId SetCoordData = 0x303147ul; // G10
        static const GCodeId PlaneSelectXY = 0x373147ul; // G17
        static const GCodeId PlaneSelectXZ = 0x383147ul; // G18
        static const GCodeId PlaneSelectYZ = 0x393147ul; // G19
        static const GCodeId UnitsInch = 0x303247ul; // G20
        static const GCodeId Units_mm = 0x313247ul; // G21
        static const GCodeId MoveToHome = 0x383247ul; // G28
        static const GCodeId MoveToHome2 = 0x303347ul; // G30
        static const GCodeId ProbeForContactSignalOnFailure = 0x322e383347ul; // G38.2
        static const GCodeId ProbeForContact = 0x332e383347ul; // G38.3
        static const GCodeId ProbeForBreakSignalOnFailure = 0x342e383347ul; // G38.4
        static const GCodeId ProbeForBreak = 0x352e383347ul; // G38.5
        static const GCodeId CutterRadiusCompensation = 0x303447ul; // G40
        static const GCodeId ToolOffsetSet = 0x312e333447ul; // G43.1
        static const GCodeId ToolOffsetClear = 0x393447ul; // G49
        static const GCodeId MoveAbsolute = 0x333547ul; // G53
        static const GCodeId SelectWorkOffset1 = 0x343547ul; // G54
        static const GCodeId SelectWorkOffset2 = 0x353547ul; // G55
        static const GCodeId SelectWorkOffset3 = 0x363547ul; // G56
        static const GCodeId SelectWorkOffset4 = 0x373547ul; // G57
        static const GCodeId SelectWorkOffset5 = 0x383547ul; // G58
        static const GCodeId SelectWorkOffset = 0x393547ul; // G59
        static const GCodeId PathControlModes = 0x313647ul; // G61
        static const GCodeId MotionModeCancel = 0x303847ul; // G80
        static const GCodeId AbsoluteCoords = 0x303947ul; // G90
        static const GCodeId RelativeCoords = 0x313947ul; // G91
        static const GCodeId CoordinateOffset = 0x323947ul; // G92
        static const GCodeId CoordinateOffsetClear = 0x312e323947ul; // G92.1
        static const GCodeId SetFeedrateUnitInverseTime = 0x333947ul; // G93
        static const GCodeId SetFeedrateUnitPerMinute = 0x343947ul; // G94
        static const GCodeId SetFeedrateUnitPerRevolution = 0x353947ul; // G95
        static const GCodeId ProgramStop = 0x304dul; // M0
        static const GCodeId ProgramEnd = 0x324dul; // M2
        static const GCodeId ProgramEndRewind = 0x30334dul; // M30
        static const GCodeId RotateSpindleClockwise = 0x334dul; // M3
        static const GCodeId RotateSpindleAnticlockwise = 0x344dul; // M4
        static const GCodeId StopSpindle = 0x354dul; // M5
        static const GCodeId FloodCoolantOn = 0x384dul; // M8
        static const GCodeId AllCoolantOff = 0x394dul; // M9
    }

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

    struct MachineState
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MachineState()
            : units(GUnits_mm),
              coords(GCoordinatesAbsolute),
              feed_rate(100),
              P_w(0.0f,0.0f,0.0f,0.0f),
              active_cmd(Cmd::RapidLinearMove)
        {
        }

        GUnits units;
        GCoordinates coords;
        float feed_rate;
        Eigen::Vector4f P_w;
        GCodeId active_cmd;
    };

}