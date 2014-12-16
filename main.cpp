#include <pangolin/pangolin.h>
#include <pangolin/gl/glvbo.h>

#include "GcodeProgram.h"
#include "Heightmap.h"

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

    typedef float P;
    const std::string filename = argv[1];
    cnsee::GcodeProgram<P> prog = cnsee::ParseFile<P>(filename);

    cnsee::Heightmap<P> heightmap(prog.bounds_mm);
    for(const Eigen::Matrix<P,3,1>& p_w : prog.trajectory_w)
    {
        heightmap.Mill(p_w);
    }

    // Compute Normals
    ComputeNormals(heightmap.normals, heightmap.surface);

    pangolin::CreateWindowAndBind("Main",640,480);
    // Setup default OpenGL parameters
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
    glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
    glEnable (GL_BLEND);
    glEnable (GL_LINE_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(1.5);
    glPixelStorei(GL_PACK_ALIGNMENT,1);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);

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

    const std::string shaders_dir = pangolin::FindPath(argv[0], "/shaders");
    std::vector<std::string> matcap_files;
#ifdef HAVE_JPEG
    pangolin::FilesMatchingWildcard(shaders_dir + std::string("/matcap/*.jpg"), matcap_files);
#endif // HAVE_JPEG
#ifdef HAVE_PNG
    pangolin::FilesMatchingWildcard(shaders_dir + std::string("/matcap/*.png"), matcap_files);
#endif // HAVE_PNG

    pangolin::GlBuffer trajectory_vbo(pangolin::GlArrayBuffer, prog.trajectory_w.size(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_vbo(pangolin::GlArrayBuffer, heightmap.surface.rows() * heightmap.surface.cols(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_nbo(pangolin::GlArrayBuffer, heightmap.surface.rows() * heightmap.surface.cols(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_ibo = pangolin::MakeTriangleStripIboForVbo(heightmap.surface.rows(), heightmap.surface.cols());
    pangolin::GlTexture matcaptex;
    if(matcap_files.size()) {
        matcaptex.LoadFromFile(matcap_files[0]);
    }

    pangolin::GlSlProgram norm_shader;
    norm_shader.AddShaderFromFile(pangolin::GlSlVertexShader,   shaders_dir + std::string("/matcap.vert"));
    norm_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, shaders_dir + std::string("/matcap.frag"));
    norm_shader.Link();

    norm_shader.Bind();
//    norm_shader.SetUniform("uTexMatCap", 0);
    norm_shader.SetUniform("shininess", 0.5f);
    norm_shader.Unbind();

    trajectory_vbo.Upload(&prog.trajectory_w[0][0], prog.trajectory_w.size() * sizeof(P) * 3 );
    surface_vbo.Upload(&heightmap.surface(0,0)[0], heightmap.surface.rows() * heightmap.surface.cols() * sizeof(P) * 3);
    surface_nbo.Upload(&heightmap.normals(0,0)[0], heightmap.normals.rows() * heightmap.normals.cols() * sizeof(P) * 3);

    std::cout << prog.bounds_mm.min().transpose() << " - " << prog.bounds_mm.max().transpose() << " mm." << std::endl;
    std::cout << heightmap.surface.rows() << " x " << heightmap.surface.cols() << " px." << std::endl;
    std::cout << matcap_files[0] << std::endl;
    
    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        
        // Trajectory
        glColor3f(1.0f,0.0f,0.0f);
        trajectory_vbo.Bind();
        glVertexPointer(3, GL_FLOAT, 0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_LINE_STRIP, 0, prog.trajectory_w.size());
        glDisableClientState(GL_VERTEX_ARRAY);
        trajectory_vbo.Unbind();

        // Surface
        glColor3f(0.0f,0.0f,1.0f);
        norm_shader.Bind();
        pangolin::RenderVboIboNbo(surface_vbo, surface_ibo, surface_nbo, true, true);
        norm_shader.Unbind();

        
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
    
    return 0;
}
