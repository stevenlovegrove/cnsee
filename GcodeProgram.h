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
        token.letter = is.get();
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
                    // Add char to token
                    line.tokens.push_back(token);
                    token.letter = 0;
                }
                if(line.tokens.size()) {
                    // Add token to line
                    program.lines.push_back(line);
                    line.tokens.clear();
                }
            }else if(c=='%') {
                ConsumeUntil(is,'\n');
                continue;
            }else if(std::isspace(c)) {
                if(token.letter) {
                    // Add char to token
                    line.tokens.push_back(token);
                    token.letter = 0;
                }
            }else {
                // Add to current token
                if(!token.letter) {
                    token.letter = c;
                }else{
                    is >> token.number;
                    continue;
                }
            }
            is.get();
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
                        else if(t.number == 20) state.units  = GUnits_inch;
                        else if(t.number == 21) state.units  = GUnits_mm;
                        else if(t.number == 90) state.coords = GCoordinatesAbsolute;
                        else if(t.number == 91) state.coords = GCoordinatesRelative;
                        else{
                            std::cerr << "Unknown token: " << t << std::endl;
                            state.active_cmd = Cmd::None;
                        }
                    }else if(t.letter == 'M') {
                        std::cerr << "Unknown token: " << t << std::endl;
                        state.active_cmd = Cmd::None;
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
            const float dist = (End_w - state.P_w).norm();
            const int samples = std::max(2, (int) std::ceil(dist * samples_per_unit));
            for (int s = 0; s < samples; ++s) {
                const float lambda = (float)s / (float)samples;
                const Eigen::Vector4f P_w = (1 - lambda) * state.P_w + lambda * End_w;
                SamplePosition(P_w.head<3>());
            }
            state.P_w = End_w;
        }

        void RapidLinearMove(const Eigen::Vector4f &End_w) {
            // TODO: Get correct feedrate and perform freespace check
            LinearMove(End_w, 100);
        }

        void SamplePosition(const Eigen::Vector3f &P_w)
        {
            bounds_mm.extend(P_w);
            trajectory.push_back(P_w);
//            trajectory_info.push_back(info);
        }

        const MachineState start_state;
        MachineState state;
        Eigen::AlignedBox<Prec,3> bounds_mm;

        Trajectory trajectory;
        TrajectoryInfo trajectory_info;
        float samples_per_unit;
    };

}