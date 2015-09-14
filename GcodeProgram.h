#pragma once

#include <fstream>
#include <map>

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include "GCodeCommand.h"

namespace cnsee {
    struct GToken
    {
        GToken()
            : letter(0), number(0.0f)
        {
        }

        GToken(char l, float n)
            : letter(l), number(n)
        {
        }

        GToken(const std::string& str)
        {
            letter = str[0];
            number = (float)atof(str.c_str()+1);
        }

        char letter;
        float number;
    };

    std::ostream& operator<<(std::ostream& os, const GToken& token)
    {
        os << token.letter << token.number;
        return os;
    }

    std::istream& operator>>(std::istream& is, GToken& token)
    {
        while(is.good() && std::isspace(is.peek())) is.get();
        token.letter = (char)is.get();
        is >> token.number;
        return is;
    }

    struct GLine
    {
        std::vector<GToken> tokens;
    };

    struct GProgram
    {
        std::vector<GLine> lines;
    };

    inline float ConsumeFloat(std::istream &is)
    {
        float s = 1;
        float v = 0;
        float dec = 0;

        int c = is.peek();
        while(is.good() && (std::isdigit(c) || c == '.' || c == '-' || c == '+' ) )
        {
            if( c=='+') {
                // ignore
            }else if( c=='-') {
                s = -1;
            }else if( c=='.') {
                dec = 10;
            }else{
                const float d = float(c - '0');
                v = 10.0f*v + d;
                if(dec) dec *= 10;
            }
            is.get();
            c = is.peek();
        }

        return dec ? s*v/dec : s*v;
    }

    inline GProgram ParseGProgram(std::istream &is)
    {
        GProgram program;
        GLine line;
        GToken token;

        int bracket = 0;

        while(is.good()) {
            const int c = is.peek();
            if(c=='(') {
                ++bracket;
            }else if(c==')') {
                --bracket;
            }else if(bracket > 0) {
                // ignore input
            }else if(c=='\n') {
                if(token.letter) {
                    // Add token to line
                    line.tokens.push_back(token);
                    token.letter = 0;
                }
                if(line.tokens.size()) {
                    // Add line to program
                    program.lines.push_back(line);
                    line.tokens.clear();
                }
            }else if(c=='%' || c=='#' || c=='*') {
                // Treat checksums '*..' like end of line comments!
                ConsumeUntil(is, '\n');
                continue;
            }else if(std::isspace(c)) {
                if(token.letter) {
                    // Add token to line
                    line.tokens.push_back(token);
                    token.letter = 0;
                }
            }else {
                if(std::isdigit(c) || c == '-' || c == '.') {
                    token.number = ConsumeFloat(is);
                    continue;
                }else if(token.letter){
                    // start new token
                    line.tokens.push_back(token);
                    token.letter = (char)c;
                }else{
                    // Set letter for next token
                    token.letter = (char)c;
                }
            }
            is.get();
        }

        if(!is.eof()) {
            pango_print_warn("Aborted due to error bit on stream.\n");
        }

        return program;
    }

    inline GProgram ParseGProgram(const std::string& filename) {
        std::ifstream file(filename);
        if(file.is_open()) {
            return ParseGProgram(file);
        }else{
            throw std::runtime_error("Unable to open file: " + filename);
        }
    }

    struct GProgramExecution
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct Info {
            Info(size_t line, uint64_t time_us)
                : line(line), time_us(time_us)
            {
            }

            size_t   line;
            uint64_t time_us;
        };

        typedef float Prec;
        typedef Eigen::Matrix<Prec, 3, 1> Vec3;
        typedef std::vector<Info> TrajectoryInfo;
        typedef std::vector<Vec3,Eigen::aligned_allocator<Vec3>> Trajectory;

        GProgramExecution(float samples_per_unit = 100, const MachineState& starting_state = MachineState() )
            : start_state(starting_state), state(starting_state), samples_per_unit(samples_per_unit)
        {
        }

        void ClearTrajectory()
        {
            trajectory.clear();
            trajectory_info.clear();
            bounds_mm.setEmpty();
        }

        void ResetState()
        {
            state = start_state;
        }

        void SetOrdinate(float &ordinate, float val)
        {
            if( state.coords == GCoordinatesAbsolute) {
                ordinate = val;
            }else{
                ordinate += val;
            }
        }

        void ExecuteProgram(const GProgram& program)
        {
            for(size_t l=0; l < program.lines.size(); ++l) {
                size_t line_num = l;
                Eigen::Vector4f axis = state.P_w;
                float feedrate = state.feed_rate;

                for(const GToken& t : program.lines[l].tokens) {
                    if(t.letter == 'N') {
                        line_num = (size_t)t.number;
                    }else if(t.letter == 'G') {
                        if(t.number == 0)       state.active_cmd = Cmd::RapidLinearMove;
                        else if(t.number == 1)  state.active_cmd = Cmd::LinearMove;
                        else if(t.number == 17) state.plane  = GXY;
                        else if(t.number == 18) state.plane  = GZX;
                        else if(t.number == 19) state.plane  = GYZ;
                        else if(t.number == 20) state.units  = GUnits_inch;
                        else if(t.number == 21) state.units  = GUnits_mm;
                        else if(t.number == 40) pango_print_warn("Ignoring 'tool compensation off'.\n");
                        else if(t.number == 43) pango_print_warn("Ignoring 'tool offset'.\n");
                        else if(t.number == 49) state.tool_offset = -1;
                        else if(t.number == 54) pango_print_warn("Ignoring switch to User Offset 1.\n");
                        else if(t.number == 80) state.active_cmd = Cmd::None; // TODO: Check this is correct action
                        else if(t.number == 90) state.coords = GCoordinatesAbsolute;
                        else if(t.number == 91) state.coords = GCoordinatesRelative;
                        else{
                            std::cerr << "Unknown token: " << t << std::endl;
                            state.active_cmd = Cmd::None;
                        }
                    }else if(t.letter == 'M') {
                        if(t.number == 3)      state.spindle_cw = +1;
                        else if(t.number == 4) state.spindle_cw = -1;
                        else if(t.number == 5) state.spindle_cw =  0;
                        else if(t.number == 6) pango_print_warn("Ignoring 'tool change'.\n");
                        else {
                            std::cerr << "Unknown token: " << t << std::endl;
                            state.active_cmd = Cmd::None;
                        }
                    }else if(t.letter == 'X') {
                        SetOrdinate(axis[0], t.number);
                    }else if(t.letter == 'Y') {
                        SetOrdinate(axis[1], t.number);
                    }else if(t.letter == 'Z') {
                        SetOrdinate(axis[2], t.number);
                    }else if(t.letter == 'A') {
                        SetOrdinate(axis[3], t.number);
                    }else if(t.letter == 'F') {
                        feedrate = t.number;
                    }else if(t.letter == 'S') {
                        state.spindle_speed = t.number;
                    }else if(t.letter == 'T') {
                        pango_print_warn("Ignoring 'tool chance'.\n");
                    }else if(t.letter == 'H') {
                        pango_print_warn("Ignoring 'tool offset' parameter.\n");
                    }else{
                        std::cerr << "Unknown token: " << t << std::endl;
                        state.active_cmd = Cmd::None;
                    }
                }

                // Execute line
                if(state.active_cmd == Cmd::RapidLinearMove) {
                    RapidLinearMove( axis);
                }else if(state.active_cmd == Cmd::LinearMove) {
                    LinearMove( axis, feedrate);
                }
            }
        }

        void LinearMove(const Eigen::Vector4f &End_w, float feedrate)
        {
            if(state.plane == GXY) {
                const float dist = (End_w - state.P_w).norm();
                if(dist > 0.0) {
                    const int samples = std::max(2, (int) std::ceil(dist * samples_per_unit));
                    for (int s = 0; s < samples; ++s) {
                        const float lambda = (float)s / (float)samples;
                        const Eigen::Vector4f P_w = (1 - lambda) * state.P_w + lambda * End_w;
                        const Eigen::Vector3f P3_w = P_w.head<3>();
                        bounds_mm.extend(P3_w);
                        trajectory.push_back(P3_w);
                    }
                    state.P_w = End_w;
                    state.feed_rate = feedrate;
                }
            }else{
                pango_print_error("Unsupported Plane\n");
            }
        }

        void RapidLinearMove(const Eigen::Vector4f &End_w) {
            const float dist = (End_w - state.P_w).norm();
            if(dist > 0.0) {
                const int samples = std::max(2, (int) std::ceil(dist * samples_per_unit));
                for (int s = 0; s < samples; ++s) {
                    const float lambda = (float)s / (float)samples;
                    const Eigen::Vector4f P_w = (1 - lambda) * state.P_w + lambda * End_w;
                    trajectory.push_back(P_w.head<3>());
                }
                state.P_w = End_w;
            }
        }

        const MachineState start_state;
        MachineState state;
        Eigen::AlignedBox<Prec,3> bounds_mm;

        Trajectory trajectory;
        TrajectoryInfo trajectory_info;
        float samples_per_unit;
    };

}