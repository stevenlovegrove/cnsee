#pragma once

#include "GTokenize.h"

namespace cnsee {

GLine MoveRel(const Eigen::Vector3f& v)
{
    char buffer[1024];
    snprintf(buffer,1024,"G91G1X%fY%fZ%f\n", v[0], v[1], v[2]);
    return TokenizeLine(std::string(buffer),-1);
}

GLine MoveRelQuick(const Eigen::Vector3f& v)
{
    char buffer[1024];
    snprintf(buffer,1024,"G91G0X%fY%fZ%f\n", v[0], v[1], v[2]);
    return TokenizeLine(std::string(buffer),-1);
}

GLine MoveTo(const Eigen::Vector3f& v)
{
    char buffer[1024];
    snprintf(buffer,1024,"G90G1X%fY%fZ%f\n", v[0], v[1], v[2]);
    return TokenizeLine(std::string(buffer),-1);
}

GLine MoveToQuick(const Eigen::Vector3f& v)
{
    char buffer[1024];
    snprintf(buffer,1024,"G90G0X%fY%fZ%f\n", v[0], v[1], v[2]);
    return TokenizeLine(std::string(buffer),-1);
}

GLine Unlock()
{
    return TokenizeLine("$X\n",-1);
}

GLine SetHardLimits(bool enable)
{
    if(enable) {
        return TokenizeLine("$21=1\n",-1);
    }else{
        return TokenizeLine("$21=0\n",-1);
    }
}

GLine SetUnits_mm()
{
    return TokenizeLine("G21\n",-1);
}

}
