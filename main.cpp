#include <fstream>
#include <pangolin/pangolin.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <map>

struct GcodeCmd
{
    unsigned char letter;
    unsigned int  code;
    std::map<unsigned char,double> params;
};

void ConsumeComments(std::istream& is)
{
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
            const Eigen::Vector3d p = (lambda*p_w + (1-lambda)*end_w);
//            std::cout << p.transpose() << std::endl;
            trajectory_w.push_back( p );
        }
        end_w = p_w;
    }

    double samples_per_unit;

    // Start and end positions for the spindle
    Eigen::Vector3d start_w;
    Eigen::Vector3d end_w;

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

int main( int argc, char** argv )
{  
    if(argc <= 1) {
        std::cout << "Usage: cnsee filename.nc" << std::endl;
        return -1;
    }

    const std::string filename = argv[1];
    GcodeProgram prog = ParseFile(filename);

    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);
    
    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,1,1000),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );
    
    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);
    
    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        
        // Render OpenGL Teapot
        pangolin::glDrawColouredCube();

        glColor3f(1.0f,0.0f,0.0f);
        glVertexPointer(3, GL_DOUBLE, 0, &prog.trajectory_w[0][0]);
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_LINE_STRIP, 0, prog.trajectory_w.size());
        glDisableClientState(GL_VERTEX_ARRAY);

        
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
    
    return 0;
}
