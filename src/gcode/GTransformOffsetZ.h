#pragma once

#include "GTransformToAbs.h"
#include "../thinplate/ThinPlateSpline.hpp"
#include "../machine/GrblMachine.h"

namespace cnsee {

GProgram GTransformOffsetZ(const GProgram& prog, const GrblMachine& machine, const ThinPlateSpline& surface_machine)
{
    // surface_machine says how to take 0_machine to new height. But we want 0_work to be new height.
    // We must apply a second offset
    const double zwork_offet = -machine.wco[2];

    // It is easier to adjust a purely absolute program
    GProgram transformed_prog = GTransformToAbs(prog);

    // Original execution
    const GProgramExecution orig_exec(transformed_prog);
    GMachineState new_state = orig_exec.state_vec[0];
    new_state.P_w[2] += surface_machine.SurfaceOffset(machine.MachineFromWorkCoords(new_state.P_w) ) + zwork_offet;

    // Check that there is an execution state to start and then for every line
    assert(orig_exec.state_vec.size() == transformed_prog.lines.size()+1);

    // Transform all lines based on surface relief
    for(size_t i=0; i < transformed_prog.lines.size(); ++i) {
        GLine& line = transformed_prog.lines[i];
        const GMachineState& orig_line_end   = orig_exec.state_vec[i+1];

        // Construct desired state
        GMachineState desired_end = orig_line_end;
        desired_end.P_w[2] += surface_machine.SurfaceOffset(machine.MachineFromWorkCoords(orig_line_end.P_w)) + zwork_offet;

        // Update Line to achieve desired state
        if(orig_line_end.active_cmd == Cmd::LinearMove || orig_line_end.active_cmd == Cmd::RapidLinearMove) {
            // A move is active, so the modal Z ordinate should contain the desired value

            // Check if Z is updated in this line
            auto it = std::find_if(line.tokens.rbegin(), line.tokens.rend(), [](const GToken&o){
                return o.letter == 'Z';
            });

            if(it == line.tokens.rend()) {
                // Z not referenced, add a Z ordinate to line.
                line.tokens.emplace_back('Z', desired_end.P_w[2]);
            }else{
                // Z already referenced
                if( it->number != orig_line_end.P_w[2]) {
                    // Double check the reference matches the old transition state
                    std::cerr << "Original value doesn't make sense" << std::endl;
                }
                // Update the Z value
                it->number = desired_end.P_w[2];
            }
        }

        std::stringstream ss;
        ss << line;
        line.raw_line = ss.str();

        // Check that new_state matches desired_end
        ApplyGCode(new_state, line);
        if( (new_state.P_w - desired_end.P_w).norm() > 1E-3) {
            std::cerr << "Incorrect transform" << std::endl;
        }
    }

    return transformed_prog;
}

}
