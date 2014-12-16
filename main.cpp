#include <fstream>
#include <map>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>

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

class Heightmap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Heightmap(const Eigen::AlignedBox2d& bounds)
        : res_per_mm(10.0f)
    {
        bounds_mm = bounds;
        bounds_mm.min()[0] -= 100;
        bounds_mm.min()[1] -= 100;
        bounds_mm.max()[0] += 100;
        bounds_mm.max()[1] += 100;
        Eigen::Vector2i size_pix = (res_per_mm*bounds_mm.sizes()).cast<int>();
        surface = Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>::Zero(size_pix[0], size_pix[1]);
    }

    // Assume V tip for now
    void Mill(const Eigen::Vector3d& p_w)
    {
        // Surface strictly z < max_surface_height_mm
        const double max_surface_height_mm = 0;
        const double grad = 1.0;
        const double max_cut_depth_mm = max_surface_height_mm - p_w[2];
        const double max_rad_mm = grad * max_cut_depth_mm;

        if(max_rad_mm > 0.0) {
            const Eigen::Vector2d min_mm = p_w.head<2>().array() - max_rad_mm;
            const Eigen::Vector2d max_mm = p_w.head<2>().array() + max_rad_mm;
            const Eigen::AlignedBox2d update_pixf(res_per_mm*min_mm - bounds_mm.min(), res_per_mm*max_mm - bounds_mm.min());
            const Eigen::AlignedBox2i update_pix = update_pixf.cast<int>();

            // Update height for max_rad patch around p_w.xy
            const Eigen::Vector2d center_pix = (p_w.head<2>() - bounds_mm.min()) * res_per_mm;
            const double grad_pix = grad * res_per_mm;
            Eigen::Vector2i pix;
            for(pix[0] = update_pix.min()[0]; pix[0] <= update_pix.max()[0]; ++pix[0]) {
                for(pix[1] = update_pix.min()[1]; pix[1] <= update_pix.max()[1]; ++pix[1]) {
                    const double rad = (pix.cast<double>() - center_pix).norm();
                    const double tool_height_mm = p_w[2] + rad * grad_pix;
                    if(tool_height_mm < surface(pix[1],pix[0]) ) {
                        surface(pix[1],pix[0]) = tool_height_mm;
                    }
                }
            }
        }
    }

    double res_per_mm;
    Eigen::AlignedBox2d bounds_mm;
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> surface;
};

int main( int argc, char** argv )
{  
    if(argc <= 1) {
        std::cout << "Usage: cnsee filename.nc" << std::endl;
        return -1;
    }

    const std::string filename = argv[1];
    GcodeProgram prog = ParseFile(filename);

    std::cout << prog.bounds_mm.min().transpose() << std::endl;
    std::cout << prog.bounds_mm.max().transpose() << std::endl;

    Heightmap heightmap(prog.bounds_mm);
    for(const Eigen::Vector3d& p_w : prog.trajectory_w)
    {
        heightmap.Mill(p_w);
    }

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
