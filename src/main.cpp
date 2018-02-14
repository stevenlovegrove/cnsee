#include <thread>

#include <pangolin/pangolin.h>
#include <pangolin/utils/argagg.hpp>

#include "machine/GrblMachine.h"
#include "thinplate/ThinPlateSpline.hpp"
#include "gcode/GProgramExecution.h"
#include "cut/Heightmap.h"
#include "gcode/GTransformOffsetZ.h"

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
            { "samples", {"-s", "--samples"}, "Number of samples per mm for simulation", 1},
            { "wco",     {"--wco"},           "Work Coordinate Offset - specify the origin of the work system", 1},
            { "vsamples",{"--vertex-samples"}, "Maximum number of vertices to use for displaying the surface", 1}
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
    const size_t vertex_samples = args["vsamples"].as<size_t>(2000000);

    typedef float T;

    // CNC Machine
    cnsee::GrblMachine machine;

    // default to these before even connecting so
    machine.wco = pangolin::Convert<Eigen::Vector3d,std::string>::Do(args["wco"].as<std::string>("0,0,0"));
    machine.mpos = machine.MachineFromWorkCoords(machine.wpos);

    if(!grbl_serial.empty()) {
        try{
            machine.Open(grbl_serial);
        }catch(...) {
            std::cerr << "Unable to connect to machine." << std::endl;
        }
    }

    cnsee::GProgram current_prog;
    cnsee::GProgramExecution exec_work;
    cnsee::aligned_vector<Eigen::Matrix<T,3,1>> trajectory_machine;


    cnsee::ThinPlateSpline scanned_surface;
    cnsee::Heightmap<T> heightmap;
    std::mutex surface_update_mutex;

    auto updated_wco = [&](){
        // Recompute bounds in machine coordinates
        const Eigen::AlignedBox2d bounds_machine = Eigen::AlignedBox2d(
                exec_work.bounds_mm.min().head<2>(),
                exec_work.bounds_mm.max().head<2>()
        ).translate(machine.wco.head<2>());
        heightmap.Init(bounds_machine.cast<T>(), vertex_samples);
        trajectory_machine.clear();
        const auto trajectory_work = exec_work.GenerateTrajectory<T>();
        for(const auto& p_w : trajectory_work) {
            trajectory_machine.push_back( machine.MachineFromWorkCoords(p_w) );
        }
    };

    auto use_program = [&](const cnsee::GProgram& prog){
        current_prog = prog;
        exec_work.Clear();
        exec_work.ExecuteProgram(prog);
        updated_wco();
    };

    auto load_surface = [&](const std::string& filename){
        std::ifstream ifs(filename);
        while(ifs.good()) {
            Eigen::Vector3d p;
            if(ifs >> p) scanned_surface.AddSurfacePoint(p);
        }
    };
    auto save_surface = [&](const std::string& filename){
        std::ofstream ofs(filename);
        if(ofs.good()) {
            for(const auto& p : scanned_surface.Samples()) {
                ofs << pangolin::FormatString("%,%,%",p[0],p[1],p[2]) << std::endl;
            }
        }
    };

    load_surface("surface.txt");

    if(!gcode_filename.empty()) {
        const cnsee::GProgram orig_program = cnsee::TokenizeProgram(gcode_filename);
        use_program(orig_program);
    }

    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
        pangolin::ModelViewLookAt(machine.mpos[0],machine.mpos[1],200, machine.mpos[0],machine.mpos[1],0, pangolin::AxisY)
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
    pangolin::GlBuffer trajectory_vbo(pangolin::GlArrayBuffer, trajectory_machine.size(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_vbo(pangolin::GlArrayBuffer, heightmap.surface.rows() * heightmap.surface.cols(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_nbo(pangolin::GlArrayBuffer, heightmap.surface.rows() * heightmap.surface.cols(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_ibo = pangolin::MakeTriangleStripIboForVbo(heightmap.surface.rows(), heightmap.surface.cols());
    pangolin::GlTexture matcaptex;
    matcaptex.LoadFromFile(shaders_dir + "/matcap/thuglee-backlight-01.jpg");

    pangolin::GlSlProgram norm_shader;
    norm_shader.AddShaderFromFile(pangolin::GlSlVertexShader,   shaders_dir + std::string("/matcap.vert"));
    norm_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaders_dir + std::string("/matcap.frag"));
    norm_shader.Link();

    std::thread mill_thread;
    bool mill_changed = false;
    bool mill_abort = false;

    std::deque<std::future<void>> probes;
    auto cleanup_finished_probes = [&](){
        // Try to limit the async futures that build up.
        while(probes.size() && probes.front().wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            probes.pop_front();
        }
    };

    auto mill = [&]() {
        {
            std::unique_lock<std::mutex> l(surface_update_mutex);
            scanned_surface.ApplyToSurface(heightmap);
        }
        mill_abort = false;
        for(size_t i=1; i < trajectory_machine.size(); ++i) {
            Eigen::Matrix<T,3,2> line;
            line.col(0) = trajectory_machine[i-1];
            line.col(1) = trajectory_machine[i];
            heightmap.MillSquare<T>(line, heightmap.tool.diameter/2.0f);

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

    pangolin::Var<double> cut_time("tool.cut_time", 0.0, 0.0, exec_work.TotalTime_s());
    pangolin::Var<int> cmd_queue_size("tool.cmd_queue_size", 0.0);
    pangolin::Var<int> cmd_pending_size("tool.cmd_pending_size", 0.0);

    pangolin::Var<bool> show_trajectory("tool.show_trajectory", true, true);
    pangolin::Var<bool> show_surface("tool.show_surface", true, true);
    pangolin::Var<bool> show_mesh("tool.show_mesh", true, true);
    pangolin::Var<bool> show_live_endmill("tool.show_endmill", true, true);
    pangolin::Var<bool> show_scanned_points("tool.show_scanned", true, true);

    pangolin::Var<double> tool_tip_width_mm("tool.diameter_mm", 0.6, 0.01, 5.0);
    pangolin::Var<double> tool_tip_height_mm("tool.height_mm", 8.0, 0.0, 10.0);
    pangolin::Var<double> tool_v_angle_deg("tool.v_angle_deg", 40, 0.0, 100.0);

    pangolin::Var<double>::Attach("tool.Machine X", machine.mpos[0]);
    pangolin::Var<double>::Attach("tool.Machine Y", machine.mpos[1]);
    pangolin::Var<double>::Attach("tool.Machine Z", machine.mpos[2]);
    pangolin::Var<double>::Attach("tool.Work X", machine.wpos[0]);
    pangolin::Var<double>::Attach("tool.Work Y", machine.wpos[1]);
    pangolin::Var<double>::Attach("tool.Work Z", machine.wpos[2]);

    pangolin::Var<std::function<void(void)>>("tool.HomeMachine", [&](){
        machine.QueueCommand("$H\n");
    });
    pangolin::Var<std::function<void(void)>>("tool.ResetXY", [&](){
        machine.QueueCommand("G10L20P1X0Y0\n").get();
        machine.QueueCommand("?\n").get();
        machine.QueueSync().get();
        updated_wco();
        mill_in_thread();
    });
    pangolin::Var<std::function<void(void)>>("tool.ResetZ", [&](){
        machine.QueueCommand("G10L20P1Z0\n").get();
        machine.QueueCommand("?\n").get();
        machine.QueueSync().get();
        updated_wco();
        mill_in_thread();
    });
    pangolin::Var<std::function<void(void)>>("tool.ProbeAndSetZ", [&](){
        auto a = std::async(std::launch::async, [&](){
            auto promise = machine.ProbeSurface(Eigen::Vector3d(0,0,-10), 50);
            cnsee::ProbeResult probe = promise.get();
            if(probe.contact_made) {
                // Our probe and surface should agree (to account for changed tool, for e.g.
                const double curr_surface = scanned_surface.SurfaceOffset(probe.contact_point);
                const double new_surface = probe.contact_point[2];
                scanned_surface.OffsetEntireSurface(new_surface - curr_surface);
                mill_in_thread();
            }else{
                std::cerr << "Probe failed. " << std::endl;
            }
        });

        probes.push_back(std::move(a));
    });
    pangolin::Var<std::function<void(void)>>("tool.GoToXY0", [&](){
        machine.QueueCommand("$J=G90G21X0Y0F2000\n");
    });
    pangolin::Var<std::function<void(void)>>("tool.ClearProbedSurface", [&](){
        std::unique_lock<std::mutex> l(surface_update_mutex);
        scanned_surface.Clear();
        heightmap.Clear();
    });
    pangolin::Var<std::function<void(void)>>("tool.TransformGCode", [&](){
        const cnsee::GProgram orig_program = cnsee::TokenizeProgram(gcode_filename);
        cnsee::GProgram zprog = cnsee::GTransformOffsetZ(orig_program, machine, scanned_surface);
        use_program(zprog);
        mill_in_thread();
    });
    pangolin::Var<std::function<void(void)>>("tool.ExecuteProgram", [&](){
        for(const auto& line : current_prog.lines) {
            const std::string cmd = pangolin::Convert<std::string,cnsee::GLine>::Do(line);
            machine.QueueCommand(cmd);
        }
    });


    {
        using namespace pangolin;

        const double xystep = 5.0;
        const double zstep  = 1.0;
        const double feedrate = 2000;
        pangolin::RegisterKeyPressCallback(PANGO_SPECIAL+PANGO_KEY_UP,    [&](){ machine.QueueCommand(FormatString("$J=G91G21Y%F%\n",+xystep,feedrate));});
        pangolin::RegisterKeyPressCallback(PANGO_SPECIAL+PANGO_KEY_DOWN,  [&](){ machine.QueueCommand(FormatString("$J=G91G21Y%F%\n",-xystep,feedrate));});
        pangolin::RegisterKeyPressCallback(PANGO_SPECIAL+PANGO_KEY_RIGHT, [&](){ machine.QueueCommand(FormatString("$J=G91G21X%F%\n",+xystep,feedrate));});
        pangolin::RegisterKeyPressCallback(PANGO_SPECIAL+PANGO_KEY_LEFT,  [&](){ machine.QueueCommand(FormatString("$J=G91G21X%F%\n",-xystep,feedrate));});
        pangolin::RegisterKeyPressCallback('o', [&](){ machine.QueueCommand(FormatString("$J=G91G21Z%F%\n",+zstep,feedrate));});
        pangolin::RegisterKeyPressCallback('l', [&](){ machine.QueueCommand(FormatString("$J=G91G21Z%F%\n",-zstep,feedrate));});
        pangolin::RegisterKeyPressCallback('x', [&](){ machine.QueueCommand("$X\n");});
        pangolin::RegisterKeyPressCallback(' ', [&](){
            // Abort commands in command queue and abort those received by the machine.
            machine.ClearCommandQueue();
            machine.QueueCommand("\x85\n");
        });

        pangolin::RegisterKeyPressCallback('p',  [&](){
            cleanup_finished_probes();

            auto a = std::async(std::launch::async, [&](){
                auto promise = machine.ProbeSurface(Eigen::Vector3d(0,0,-10), 50);
//                machine.QueueCommand(FormatString("$J=G91G21Z%F%\n",5,feedrate));
                cnsee::ProbeResult probe = promise.get();
                if(probe.contact_made) {
                    std::cout << "Probe succeeded: " << probe.contact_point.transpose() << std::endl;
                    std::unique_lock<std::mutex> l(surface_update_mutex);
                    scanned_surface.AddSurfacePoint(probe.contact_point);
                    save_surface("surface.txt");
                }else{
                    std::cerr << "Probe failed. " << std::endl;
                }
            });

            probes.push_back(std::move(a));
        });
    }

    pangolin::FlagVarChanged();

    size_t frame = 0;
    while( !pangolin::ShouldQuit() )
    {
        cmd_queue_size = machine.NumberOfCommandsInQueue();
        cmd_pending_size = machine.NumberOfCommandsUnacked();

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

            // create a new buffer as the length of the trajectory may have changed.
            trajectory_vbo = pangolin::GlBuffer(pangolin::GlArrayBuffer, trajectory_machine.size(), GL_FLOAT, 3);
            trajectory_vbo.Upload(&trajectory_machine[0][0], trajectory_machine.size() * sizeof(T) * 3 );

            surface_vbo.Upload(&heightmap.surface(0,0)[0], heightmap.surface.rows() * heightmap.surface.cols() * sizeof(T) * 3);
            surface_nbo.Upload(&heightmap.normals(0,0)[0], heightmap.normals.rows() * heightmap.normals.cols() * sizeof(T) * 3);
        }
        
        // Trajectory
        if(show_trajectory) {
            glColor3f(1.0f,0.0f,0.0f);
            trajectory_vbo.Bind();
            glVertexPointer(3, GL_FLOAT, 0, 0);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINE_STRIP, 0, trajectory_machine.size());
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
            const Eigen::Vector3d mpos = machine.MachineFromWorkCoords(exec_work.GetP_wAtTime(cut_time));
            glTranslated(mpos[0],mpos[1],mpos[2]);
            pangolin::glDrawAxis(1.0);
            glPopMatrix();
        }

        if(show_scanned_points)
        {
            std::unique_lock<std::mutex> l(surface_update_mutex);
            glPointSize(3.0);
            glColor3f(0.0,1.0,0.0);
            pangolin::glDrawPoints(scanned_surface.Samples());
            glPointSize(1.0);
        }

        // Poll the CNC machine for status
        machine.RequestStatus();

        ++frame;
        pangolin::FinishFrame();
    }

    if(mill_thread.joinable()) {
        mill_abort = true;
        mill_thread.join();
    }
    
    return 0;
}
