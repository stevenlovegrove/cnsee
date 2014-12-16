#pragma once

#include <fstream>
#include <map>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace cnsee
{

struct GcodeCmd
{
    unsigned char letter;
    unsigned int  code;
    std::map<unsigned char,double> params;
};

void ConsumeComments(std::istream& is)
{
    // TODO: This wont handle multiple nested brackets.
    if(is.good() && is.peek() == '(') {
        is.get();
        while(is.peek() != ')') is.get();
        is.get();
    }
}

void ConsumeWhitespace(std::istream& is)
{
    while (is.good() && std::isspace(is.peek())) is.get();
}

std::istream& operator>>(std::istream& is, GcodeCmd& cmd)
{
    ConsumeWhitespace(is);
    ConsumeComments(is);

    is >> cmd.letter;
    is >> cmd.code;

    ConsumeWhitespace(is);
    ConsumeComments(is);

    while(is.good()) {
        unsigned char c;
        double v;
        is >> c;
        is >> v;
        if(!is.fail()) {
            cmd.params[c] = v;
            ConsumeComments(is);
        }else{
            is.clear(std::ios_base::eofbit);
            break;
        }
    }
    return is;
}

void FillCoords(Eigen::Vector3d& p_w, const std::map<unsigned char,double>& params)
{
    const int num_axis = 3;
    const unsigned char axis[num_axis] = {'X','Y','Z'};
    for(int i=0; i<num_axis; ++i) {
        std::map<unsigned char,double>::const_iterator it = params.find(axis[i]);
        if(it != params.end()) p_w[i] = it->second;
    }
}

class GcodeProgram
{
public:
    GcodeProgram(const Eigen::Vector3d& start = Eigen::Vector3d::Zero())
        : samples_per_unit(100),
          start_w(start),
          end_w(start)
    {
        trajectory_w.push_back(start);
    }

    void PushCommand(const GcodeCmd& cmd)
    {
        if(cmd.letter == 'G')
        {
            switch (cmd.code) {
            case 0:
            case 1:
                Eigen::Vector3d p_w = end_w;
                FillCoords(p_w, cmd.params);
                AbsLinearMove(p_w);
                return;
            }
        }

        std::cerr << "Ignoring unknown command, " << cmd.letter << cmd.code << std::endl;
    }

//protected:
    void AbsLinearMove(const Eigen::Vector3d& p_w)
    {
        const double dist = (p_w - end_w).norm();
        const int samples = std::max(2, (int)std::ceil(dist * samples_per_unit));
        for(int s=0; s < samples; ++s) {
            const double lambda = (double)s / (double)samples;
            SamplePosition(lambda*p_w + (1-lambda)*end_w);
        }
        end_w = p_w;
    }

    void SamplePosition(const Eigen::Vector3d& p_w)
    {
        bounds_mm.extend(p_w.head<2>());
        trajectory_w.push_back( p_w );
    }

    double samples_per_unit;

    // Start and end positions for the spindle
    Eigen::Vector3d start_w;
    Eigen::Vector3d end_w;

    Eigen::AlignedBox2d bounds_mm;

    // samples on trajectory, absolute coordinates
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > trajectory_w;
    std::vector<GcodeCmd> cmds;
};

GcodeProgram ParseFile(const std::string& filename)
{
    std::ifstream infile(filename);

    GcodeProgram prog;

    std::string line;
    while (std::getline(infile, line)) {
        GcodeCmd cmd;
        std::istringstream iss(line);
        iss >> cmd;
        if(!iss.fail()) {
            prog.PushCommand(cmd);
        }
    }

    return prog;
}

}
