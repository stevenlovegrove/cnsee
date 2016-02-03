#include <thread>

#include <pangolin/pangolin.h>

#include "GcodeProgram.h"
#include "Heightmap.h"
#include "Machine.h"

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
    if(argc <= 1) {
        std::cout << "Usage: cnsee filename.nc" << std::endl;
        return -1;
    }

    typedef float T;
    const std::string filename = argv[1];
//    const std::string grbl_serial = "/dev/tty.USB0";
    const std::string grbl_serial = "/dev/tty.usbserial-A9ORBH5T";

    cnsee::GProgram prog = cnsee::ParseGProgram(filename);
    cnsee::GProgramExecution exec(1000);
    exec.ExecuteProgram(prog);
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
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    const std::string shaders_dir = pangolin::FindPath(".", "/shaders");
    std::vector<std::string> matcap_files;
#ifdef HAVE_JPEG
    pangolin::FilesMatchingWildcard(shaders_dir + std::string("/matcap/*.jpg"), matcap_files);
#endif // HAVE_JPEG
#ifdef HAVE_PNG
    pangolin::FilesMatchingWildcard(shaders_dir + std::string("/matcap/*.png"), matcap_files);
#endif // HAVE_PNG
    std::sort(matcap_files.begin(), matcap_files.end());

    pangolin::GlBuffer trajectory_vbo(pangolin::GlArrayBuffer, exec.trajectory.size(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_vbo(pangolin::GlArrayBuffer, heightmap.surface.rows() * heightmap.surface.cols(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_nbo(pangolin::GlArrayBuffer, heightmap.surface.rows() * heightmap.surface.cols(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_ibo = pangolin::MakeTriangleStripIboForVbo(heightmap.surface.rows(), heightmap.surface.cols());
    pangolin::GlTexture matcaptex;
    if(matcap_files.size()) {
        std::cout << "Using matcap texture: " << matcap_files[0] << std::endl;
        matcaptex.LoadFromFile(matcap_files[0]);
    }else{
        std::cerr << "No 'matcap' textures found." << std::endl;
        return -1;
    }

    pangolin::GlSlProgram norm_shader;
    norm_shader.AddShaderFromFile(pangolin::GlSlVertexShader,   shaders_dir + std::string("/matcap.vert"));
    norm_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaders_dir + std::string("/matcap.frag"));
    norm_shader.Link();

    std::cout << "(" << exec.bounds_mm.min().transpose() << ") - (" << exec.bounds_mm.max().transpose() << ") mm." << std::endl;
    std::cout << heightmap.surface.rows() << " x " << heightmap.surface.cols() << " px." << std::endl;
    std::cout << matcap_files[0] << std::endl;

    pangolin::Var<bool> show_trajectory("tool.show_trajectory", true, true);
    pangolin::Var<bool> show_surface("tool.show_surface", true, true);
    pangolin::Var<bool> show_mesh("tool.show_mesh", true, true);
    pangolin::Var<bool> show_endmill("tool.show_endmill", true, true);

    pangolin::Var<float> tool_tip_width_mm("tool.diameter_mm", 0.01, 0.01, 5.0);
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
        for(const Eigen::Matrix<T,3,1>& p_w : exec.trajectory) {
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

    // Connect to machine
    cnsee::GerblMachine machine;

    try{
        machine.Open(grbl_serial);
    }catch(...) {
        std::cerr << "Unable to connect to machine." << std::endl;
    }

    const double step = 10.0;
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_UP,    [&](){ machine.MoveRel(Eigen::Vector3d(0,+step,0));});
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_DOWN,  [&](){ machine.MoveRel(Eigen::Vector3d(0,-step,0));});
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT, [&](){ machine.MoveRel(Eigen::Vector3d(+step,0,0));});
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_LEFT,  [&](){ machine.MoveRel(Eigen::Vector3d(-step,0,0));});

    pangolin::Var<std::function<void()> > gcode_x_plus("tool.x_plus", [&](){
        machine.SendLine("G91 G0  Y10\n");
    });
    pangolin::Var<std::function<void()> >("tool.unlock", [&](){
        machine.SendLine("$X\n");
    });
    pangolin::Var<std::function<void()> >("tool.home", [&](){
        machine.SendLine("$H\n");
    });

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

        if(hard_limits.GuiChanged()) {
            machine.SetHardLimits(hard_limits);
        }

        if(mill_changed) {
            // Compute Normals
            mill_changed = false;
            ComputeNormals(heightmap.normals, heightmap.surface);
            trajectory_vbo.Upload(&exec.trajectory[0][0], exec.trajectory.size() * sizeof(T) * 3 );
            surface_vbo.Upload(&heightmap.surface(0,0)[0], heightmap.surface.rows() * heightmap.surface.cols() * sizeof(T) * 3);
            surface_nbo.Upload(&heightmap.normals(0,0)[0], heightmap.normals.rows() * heightmap.normals.cols() * sizeof(T) * 3);
        }
        
        // Trajectory
        if(show_trajectory) {
            glColor3f(1.0f,0.0f,0.0f);
            trajectory_vbo.Bind();
            glVertexPointer(3, GL_FLOAT, 0, 0);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINE_STRIP, 0, exec.trajectory.size());
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

        if(show_endmill) {
            glPushMatrix();
            glTranslated(machine.mpos[0],machine.mpos[1],machine.mpos[2]);
            pangolin::glDrawAxis(10.0);
            glPopMatrix();
        }

        if(frame%6 == 0) {
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
