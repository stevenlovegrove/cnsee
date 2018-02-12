#pragma once

#include "GMachineState.h"
#include "GCodeCommand.h"
#include "GTokenize.h"
#include "../utils/aligned_vector.h"

namespace cnsee {

void ApplyGCode(GMachineState& state, const GLine& line) {
    auto SetOrdinate = [&](float &ordinate, float val)
    {
        ordinate = ( state.coords == GCoordinatesAbsolute) ? val : ordinate + val;
    };

    size_t line_num = line.line_number;
    Eigen::Vector4f new_P_w = state.P_w;
    float feedrate = state.feed_rate;
    float dwell_ms = 0.0f;

    for(const GToken& t : line.tokens) {
        if(t.letter == 'N') {
            line_num = (size_t)t.number;
        }else if(t.letter == 'G') {
            if(t.number == 0)       state.active_cmd = Cmd::RapidLinearMove;
            else if(t.number == 1)  state.active_cmd = Cmd::LinearMove;
            else if(t.number == 4)  state.active_cmd = Cmd::Dwell;
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
            else if(t.number == 93) state.feed_unit = GFeedDuration;
            else if(t.number == 94) state.feed_unit = GFeedUnitsPerTime;
            else if(t.number == 95) state.feed_unit = GFeedUnitsPerRevolution;
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
            SetOrdinate(new_P_w[0], t.number);
        }else if(t.letter == 'Y') {
            SetOrdinate(new_P_w[1], t.number);
        }else if(t.letter == 'Z') {
            SetOrdinate(new_P_w[2], t.number);
        }else if(t.letter == 'A') {
            SetOrdinate(new_P_w[3], t.number);
        }else if(t.letter == 'F') {
            feedrate = t.number;
        }else if(t.letter == 'S' && state.active_cmd == Cmd::Dwell) {
            dwell_ms = 1000.0f * t.number;
        }else if(t.letter == 'P' && state.active_cmd == Cmd::Dwell) {
            dwell_ms = t.number;
        }else if(t.letter == 'S') {
            state.spindle_speed = t.number;
        }else if(t.letter == 'T') {
            pango_print_warn("Ignoring 'tool change'.\n");
        }else if(t.letter == 'H') {
            pango_print_warn("Ignoring 'tool offset' parameter.\n");
        }else{
            std::cerr << "Unknown token: " << t << std::endl;
            state.active_cmd = Cmd::None;
        }
    }

    // Compute duration of move
    const float distance = (new_P_w - state.P_w).norm();
    float line_time_s = 0;
    if(state.feed_unit == GFeedUnitsPerTime) {
        const float time_m = distance / feedrate;
        line_time_s = time_m * 60;
    }else if(state.feed_unit == GFeedDuration) {
        line_time_s = feedrate;
    }else{
        line_time_s = distance * feedrate;
    }

    // Check that we understand how to perform this command
    if(state.plane != GXY) {
        pango_print_error("Unsupported Plane\n");
    }

    // Update state
    if(state.active_cmd == Cmd::RapidLinearMove) {
        // Rapid linear move
        state.P_w = new_P_w;
        state.feed_rate = feedrate;
        state.time_s += line_time_s;
        state.distance_travelled += distance;
    }else if(state.active_cmd == Cmd::LinearMove) {
        // Linear move
        state.P_w = new_P_w;
        state.feed_rate = feedrate;
        state.time_s += line_time_s;
        state.distance_travelled += distance;
    }else if(state.active_cmd == Cmd::Dwell) {
        state.time_s += dwell_ms;
    }
}

struct GProgramExecution
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GProgramExecution(const GMachineState& starting_state = GMachineState() )
    {
        state_vec.push_back(starting_state);
        Clear();
    }

    void Clear()
    {
        // Keep starting state
        state_vec.resize(1);
        bounds_mm.setEmpty();
        bounds_mm.extend(state_vec[0].P_w.head<3>());
    }

    void ExecuteProgram(const GProgram& program)
    {
        for(const GLine& l : program.lines) {
            // Copy state and evolve
            state_vec.push_back(state_vec.back());
            ApplyGCode(state_vec.back(), l);
            bounds_mm.extend(state_vec.back().P_w.head<3>());
        }
    }

    float TotalDistance() const
    {
        return state_vec.back().distance_travelled;
    }

    float TotalTime_s() const
    {
        return state_vec.back().time_s;
    }

    Eigen::Vector4f GetP_wAtTime(float time_s) const
    {
        GMachineState s;
        s.time_s = time_s;

        const auto ilb = std::upper_bound(state_vec.begin(), state_vec.end(), s, [](const GMachineState& lhs, const GMachineState& rhs){
            return lhs.time_s < rhs.time_s;
        });

        const size_t lb = ilb - state_vec.begin();

        if(lb == 0) {
            return state_vec.front().P_w;
        }else if(lb < state_vec.size()) {
            // lerp between lb-1 and lb
            const GMachineState& p = state_vec[lb-1];
            const GMachineState& n = state_vec[lb];
            const float lambda = (time_s - p.time_s) / (n.time_s - p.time_s);
            return (1.0f - lambda) * p.P_w + lambda * n.P_w;
        }else{
            return state_vec.back().P_w;
        }
    }

    Eigen::Vector4f GetP_wAtDistance(float distance_travelled) const
    {
        GMachineState s;
        s.distance_travelled = distance_travelled;

        const auto ilb = std::upper_bound(state_vec.begin(), state_vec.end(), s, [](const GMachineState& lhs, const GMachineState& rhs){
            return lhs.distance_travelled < rhs.distance_travelled;
        });

        const size_t lb = ilb - state_vec.begin();

        if(lb == 0) {
            return state_vec.front().P_w;
        }else if(lb < state_vec.size()) {
            // lerp between lb-1 and lb
            const GMachineState& p = state_vec[lb-1];
            const GMachineState& n = state_vec[lb];
            const float lambda = (distance_travelled - p.distance_travelled) / (n.distance_travelled - p.distance_travelled);
            return (1.0f - lambda) * p.P_w + lambda * n.P_w;
        }else{
            return state_vec.back().P_w;
        }
    }

    aligned_vector<Eigen::Vector3f> GenerateUpsampledTrajectory(float samples_per_unit)
    {
        aligned_vector<Eigen::Vector3f> trajectory;

        const GMachineState* p = &state_vec[0];
        for(const GMachineState& n : state_vec)
        {
            const float dist = (n.P_w - p->P_w).norm();
            if(dist > 0.0) {
                const int samples = std::max(2, (int) std::ceil(dist * samples_per_unit));
                for (int s = 0; s < samples; ++s) {
                    const float lambda = (float)s / (float)samples;
                    const Eigen::Vector3f P_w = ((1 - lambda) * p->P_w + lambda * n.P_w).head<3>();
                    trajectory.push_back(P_w);
                }
            }

            p = &n;
        }

        return trajectory;
    }

    aligned_vector<GMachineState> state_vec;
    Eigen::AlignedBox<float,3> bounds_mm;
};

}
