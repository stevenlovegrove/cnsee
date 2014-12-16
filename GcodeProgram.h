#pragma once

#include <fstream>
#include <map>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace cnsee
{

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

template<typename T>
struct GcodeCmd
{
    unsigned char letter;
    unsigned int  code;
    std::map<unsigned char,T> params;
};

template<typename T>
std::istream& operator>>(std::istream& is, GcodeCmd<T>& cmd)
{
    ConsumeWhitespace(is);
    ConsumeComments(is);

    is >> cmd.letter;
    is >> cmd.code;

    ConsumeWhitespace(is);
    ConsumeComments(is);

    while(is.good()) {
        unsigned char c;
        T v;
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

template<typename T>
void FillCoords(Eigen::Matrix<T,3,1>& p_w, const std::map<unsigned char,T>& params)
{
    const int num_axis = 3;
    const unsigned char axis[num_axis] = {'X','Y','Z'};
    for(int i=0; i<num_axis; ++i) {
        typename std::map<unsigned char,T>::const_iterator it = params.find(axis[i]);
        if(it != params.end()) p_w[i] = it->second;
    }
}

template<typename T>
class GcodeProgram
{
public:
    GcodeProgram(const Eigen::Matrix<T,3,1>& start = Eigen::Matrix<T,3,1>::Zero())
        : samples_per_unit(200),
          start_w(start),
          end_w(start)
    {
        trajectory_w.push_back(start);
    }

    void PushCommand(const GcodeCmd<T>& cmd)
    {
        if(cmd.letter == 'G')
        {
            switch (cmd.code) {
            case 0:
            case 1:
                Eigen::Matrix<T,3,1> p_w = end_w;
                FillCoords(p_w, cmd.params);
                AbsLinearMove(p_w);
                return;
            }
        }

        std::cerr << "Ignoring unknown command, " << cmd.letter << cmd.code << std::endl;
    }

//protected:
    void AbsLinearMove(const Eigen::Matrix<T,3,1>& p_w)
    {
        const T dist = (p_w - end_w).norm();
        const int samples = std::max(2, (int)std::ceil(dist * samples_per_unit));
        for(int s=0; s < samples; ++s) {
            const T lambda = (T)s / (T)samples;
            SamplePosition(lambda*p_w + (1-lambda)*end_w);
        }
        end_w = p_w;
    }

    void SamplePosition(const Eigen::Matrix<T,3,1>& p_w)
    {
        bounds_mm.extend(p_w.template head<2>());
        trajectory_w.push_back( p_w );
    }

    double samples_per_unit;

    // Start and end positions for the spindle
    Eigen::Matrix<T,3,1> start_w;
    Eigen::Matrix<T,3,1> end_w;

    Eigen::AlignedBox<T,2> bounds_mm;

    // samples on trajectory, absolute coordinates
    std::vector<Eigen::Matrix<T,3,1>,Eigen::aligned_allocator<Eigen::Matrix<T,3,1> > > trajectory_w;
    std::vector<GcodeCmd<T> > cmds;
};

template<typename T>
GcodeProgram<T> ParseFile(const std::string& filename)
{
    std::ifstream infile(filename);

    GcodeProgram<T> prog;

    std::string line;
    while (std::getline(infile, line)) {
        GcodeCmd<T> cmd;
        std::istringstream iss(line);
        iss >> cmd;
        if(!iss.fail()) {
            prog.PushCommand(cmd);
        }
    }

    return prog;
}

}
