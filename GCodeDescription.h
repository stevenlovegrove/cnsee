#pragma  once

namespace cnsee
{
    typedef uint64_t GCodeId;

    struct GCodeDescription
    {
        std::string code;
        std::string desc;
        std::string args;
    };

    // Description of GCode's at
    // http://www.nist.gov/customcf/get_pdf.cfm?pub_id=823374
    // also, http://www.tormach.com/machine_codes.html
    const static GCodeDescription gcode_descriptions[] = {
            // Motions
            {"G0",  "RapidLinearMove", "XYZA"},
            {"G1",  "LinearMove",      "XYZAF"},
            {"G2",  "ArcMotion",       "XYZARIJKF"},
            {"G3",  "HelicalMotion",   "XYZARIJKF"},
            {"G4",  "Dwell",           "P"},

            {"G10",  "SetCoordData"},

            // Plane Selection
            {"G17",  "PlaneSelectXY"},
            {"G18",  "PlaneSelectXZ"},
            {"G19",  "PlaneSelectYZ"},

            // Units
            {"G20",  "UnitsInch"},
            {"G21",  "Units_mm"},

            // Pre-defined positions
            {"G28",  "MoveToHome"},
            {"G30",  "MoveToHome2"},

//        // Not sure
//        {"G28.1",  ""},
//        {"G30.1",  ""},

            // Probing
            {"G38.2",  "ProbeForContactSignalOnFailure"},
            {"G38.3",  "ProbeForContact"},
            {"G38.4",  "ProbeForBreakSignalOnFailure"},
            {"G38.5",  "ProbeForBreak"},

            {"G40",    "CutterRadiusCompensation"},
            {"G43.1",  "ToolOffsetSet"},
            {"G49",    "ToolOffsetClear"},

            {"G53",    "MoveAbsolute"},

            {"G54",    "SelectWorkOffset1"},
            {"G55",    "SelectWorkOffset2"},
            {"G56",    "SelectWorkOffset3"},
            {"G57",    "SelectWorkOffset4"},
            {"G58",    "SelectWorkOffset5"},
            {"G59",    "SelectWorkOffset"},

            {"G61",    "PathControlModes"},

            {"G80",    "MotionModeCancel"},

            // Coordinate
            {"G90",    "AbsoluteCoords" },
            {"G91",    "RelativeCoords"},
            {"G92",    "CoordinateOffset"},
            {"G92.1",  "CoordinateOffsetClear"},

            // Feedrate modes
            {"G93",    "SetFeedrateUnitInverseTime"},
            {"G94",    "SetFeedrateUnitPerMinute"},
            {"G95",    "SetFeedrateUnitPerRevolution"},

            // Program and Spindle control
            {"M0",     "ProgramStop"},
            {"M2",     "ProgramEnd"},
            {"M30",    "ProgramEndRewind"},
            {"M3",     "RotateSpindleClockwise"},
            {"M4",     "RotateSpindleAnticlockwise"},
            {"M5",     "StopSpindle"},
            {"M8",     "FloodCoolantOn"},
            {"M9",     "AllCoolantOff"},
    };
}