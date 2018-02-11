#include <thread>

#include <pangolin/pangolin.h>
#include <pangolin/utils/argagg.hpp>

#include "machine/GrblMachine.h"
#include "gcode/GProgramExecution.h"
#include "cut/Heightmap.h"

template<typename T>
void ComputeNormals(
    Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic>& norms,
    const Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic>& verts
) {
    for(int u=0; u < norms.rows()-1; ++u) {
        for(int v=0; v < norms.cols()-1; ++v) {
            const Eigen::Matrix<T,3,1>& Vc = verts(u,v);
            const Eigen::Matrix<T,3,1>& Vr = verts(u+1,v);
            const Eigen::Matrix<T,3,1>& Vu = verts(u,v+1);
            const Eigen::Matrix<T,3,1> a = Vr - Vc;
            const Eigen::Matrix<T,3,1> b = Vu - Vc;

            const Eigen::Matrix<T,3,1> axb(
                a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]
            );

            const T magaxb = axb.norm();
            norms(u,v) = -axb / magaxb;
        }
    }
}

int main( int argc, char** argv )
{
    argagg::parser argparser {{
            { "help",    {"-h", "--help"},    "Shows this help message", 0},
            { "file",    {"-f", "--file"},    "Input gcode program filename", 1},
            { "machine", {"-m", "--machine"}, "CNC Machine serial port device", 1},
            { "samples", {"-s", "--samples"}, "Number of samples per mm for simulation", 1}
    }};

    argagg::parser_results args;
    try {
        args = argparser.parse(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    if (args["help"]) {
        argagg::fmt_ostream(std::cerr)
            << "Usage e.g: cnsee -f model.gcode -m /dev/tty.USB0" << std::endl
            << argparser;
        return EXIT_SUCCESS;
    }

    const std::string gcode_filename = args["file"].as<std::string>("");
    const std::string grbl_serial = args["machine"].as<std::string>("");
    const size_t samples_per_mm = args["samples"].as<size_t>(10);

    typedef float T;

    cnsee::GProgramExecution exec;
    if(!gcode_filename.empty()) {
        exec.ExecuteProgram(cnsee::TokenizeProgram(gcode_filename));
    }
    const cnsee::aligned_vector<Eigen::Vector3f> trajectory = exec.GenerateUpsampledTrajectory(samples_per_mm);

    cnsee::Heightmap<T> heightmap(exec.bounds_mm);

    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
        pangolin::ModelViewLookAt(0,0,100, 0,0,0, pangolin::AxisY)
    );
    
    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::CreatePanel("tool")
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(300));
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(300), 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    // Assume shaders are located in the build directory or in some parent of there.
    const std::string shaders_dir = pangolin::FindPath(".", "/shaders");
    pangolin::GlBuffer trajectory_vbo(pangolin::GlArrayBuffer, trajectory.size(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_vbo(pangolin::GlArrayBuffer, heightmap.surface.rows() * heightmap.surface.cols(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_nbo(pangolin::GlArrayBuffer, heightmap.surface.rows() * heightmap.surface.cols(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_ibo = pangolin::MakeTriangleStripIboForVbo(heightmap.surface.rows(), heightmap.surface.cols());
    pangolin::GlTexture matcaptex;
    matcaptex.LoadFromFile(shaders_dir + "/matcap/thuglee-backlight-01.jpg");

    pangolin::GlSlProgram norm_shader;
    norm_shader.AddShaderFromFile(pangolin::GlSlVertexShader,   shaders_dir + std::string("/matcap.vert"));
    norm_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaders_dir + std::string("/matcap.frag"));
    norm_shader.Link();

    std::cout << "(" << exec.bounds_mm.min().transpose() << ") - (" << exec.bounds_mm.max().transpose() << ") mm." << std::endl;
    std::cout << heightmap.surface.rows() << " x " << heightmap.surface.cols() << " px." << std::endl;

    pangolin::Var<float> cut_time("tool.cut_time", 0.0, 0.0, exec.TotalTime_s());

    pangolin::Var<bool> show_trajectory("tool.show_trajectory", true, true);
    pangolin::Var<bool> show_surface("tool.show_surface", true, true);
    pangolin::Var<bool> show_mesh("tool.show_mesh", true, true);
    pangolin::Var<bool> show_live_endmill("tool.show_endmill", true, true);

    pangolin::Var<float> tool_tip_width_mm("tool.diameter_mm", 0.6, 0.01, 5.0);
    pangolin::Var<float> tool_tip_height_mm("tool.height_mm", 8.0, 0.0, 10.0);
    pangolin::Var<float> tool_v_angle_deg("tool.v_angle_deg", 40, 0.0, 100.0);
    pangolin::Var<float> tool_z_offset_mm("tool.z_offset_mm", 0.0, -1.0, +1.0);

    pangolin::Var<bool> hard_limits("tool.hard_limits", true, true);

    std::thread mill_thread;
    bool mill_changed = false;
    bool mill_abort = false;

    auto mill = [&]() {
        mill_abort = false;
        heightmap.Clear();
        const Eigen::Matrix<T,3,1> offset(0.0, 0.0, tool_z_offset_mm);
        for(const Eigen::Matrix<T,3,1>& p_w : trajectory) {
            heightmap.MillSquare(p_w + offset);
            mill_changed = true;
            if(mill_abort) break;
        }
    };

    auto mill_in_thread = [&]() {
        if(mill_thread.joinable()) {
            mill_abort = true;
            mill_thread.join();
        }
        mill_thread = std::thread(mill);
    };

    // CNC Machine
    cnsee::GrblMachine machine;

    if(!grbl_serial.empty()) {
        try{
            machine.Open(grbl_serial);
        }catch(...) {
            std::cerr << "Unable to connect to machine." << std::endl;
        }
    }

    const double step = 10.0;
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_UP,    [&](){ machine.QueueCommand(cnsee::MoveRel(Eigen::Vector3f(0,+step,0)).raw_line);});
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_DOWN,  [&](){ machine.QueueCommand(cnsee::MoveRel(Eigen::Vector3f(0,-step,0)).raw_line);});
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT, [&](){ machine.QueueCommand(cnsee::MoveRel(Eigen::Vector3f(+step,0,0)).raw_line);});
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_LEFT,  [&](){ machine.QueueCommand(cnsee::MoveRel(Eigen::Vector3f(-step,0,0)).raw_line);});

    pangolin::FlagVarChanged();

    size_t frame = 0;
    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        if( pangolin::GuiVarHasChanged() ) {
            if(tool_v_angle_deg.GuiChanged()) {
                T tool_v_angle_rad = M_PI * tool_v_angle_deg / 180.0;
                tool_tip_height_mm = tool_tip_width_mm / tan(tool_v_angle_rad / 2.0);
            }else if(tool_tip_height_mm.GuiChanged()) {
                T tool_v_angle_rad = 2.0 * atan2(tool_tip_width_mm, tool_tip_height_mm);
                tool_v_angle_deg = 180.0 * tool_v_angle_rad / M_PI;
            }
            heightmap.tool.diameter = tool_tip_width_mm;
            heightmap.tool.height = tool_tip_height_mm;
            mill_in_thread();
        }

        if(mill_changed) {
            // Compute Normals
            mill_changed = false;
            ComputeNormals(heightmap.normals, heightmap.surface);
            trajectory_vbo.Upload(&trajectory[0][0], trajectory.size() * sizeof(T) * 3 );
            surface_vbo.Upload(&heightmap.surface(0,0)[0], heightmap.surface.rows() * heightmap.surface.cols() * sizeof(T) * 3);
            surface_nbo.Upload(&heightmap.normals(0,0)[0], heightmap.normals.rows() * heightmap.normals.cols() * sizeof(T) * 3);
        }
        
        // Trajectory
        if(show_trajectory) {
            glColor3f(1.0f,0.0f,0.0f);
            trajectory_vbo.Bind();
            glVertexPointer(3, GL_FLOAT, 0, 0);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINE_STRIP, 0, trajectory.size());
            glDisableClientState(GL_VERTEX_ARRAY);
            trajectory_vbo.Unbind();
        }

        // Surface
        if(show_surface) {
            norm_shader.Bind();
            matcaptex.Bind();
            pangolin::RenderVboIboNbo(surface_vbo, surface_ibo, surface_nbo, show_mesh, true);
            norm_shader.Unbind();
        }

        if(show_live_endmill) {
            glPushMatrix();
            glTranslated(machine.mpos[0],machine.mpos[1],machine.mpos[2]);
            pangolin::glDrawAxis(10.0);
            glPopMatrix();
        }

        // Show simulated cutting head position
        {
            glPushMatrix();
            const Eigen::Vector3f mpos = exec.GetP_wAtTime(cut_time).head<3>();
            glTranslated(mpos[0],mpos[1],mpos[2]);
            pangolin::glDrawAxis(1.0);
            glPopMatrix();
        }

//        if(frame%6 == 0)
        {
            machine.RequestStatus();
        }

        ++frame;
        pangolin::FinishFrame();
    }

    if(mill_thread.joinable()) {
        mill_abort = true;
        mill_thread.join();
    }
    
    return 0;
}
