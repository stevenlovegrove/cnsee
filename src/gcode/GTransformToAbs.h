#pragma once

#include "GProgramExecution.h"

namespace cnsee {

GProgram GTransformToAbs(const GProgram& prog)
{
    const GProgramExecution exec(prog);
    const bool already_absolute = std::all_of(exec.state_vec.begin(), exec.state_vec.end(), [](const GMachineState& s){
        return s.coords == GCoordinatesAbsolute;
    });
    if(already_absolute) {
        return prog;
    }else{
        throw std::runtime_error("Conversion to absolute coordinatess not implemented yet.");
    }
}

}
