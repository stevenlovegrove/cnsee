#include <pangolin/pangolin.h>
#include <pangolin/gl/glvbo.h>

#include "GcodeProgram.h"
#include "Heightmap.h"

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

    pangolin::GlBuffer trajectory_vbo(pangolin::GlArrayBuffer, prog.trajectory_w.size(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_vbo(pangolin::GlArrayBuffer, heightmap.surface.rows() * heightmap.surface.cols(), GL_FLOAT, 3);
    pangolin::GlBuffer surface_ibo = pangolin::MakeTriangleStripIboForVbo(heightmap.surface.rows(), heightmap.surface.cols());

    trajectory_vbo.Upload(&prog.trajectory_w[0][0], prog.trajectory_w.size() * sizeof(P) * 3 );
    surface_vbo.Upload(&heightmap.surface(0,0)[0], heightmap.surface.rows() * heightmap.surface.cols() * sizeof(P) * 3);

    std::cout << prog.bounds_mm.min().transpose() << " - " << prog.bounds_mm.max().transpose() << " mm." << std::endl;
    std::cout << heightmap.surface.rows() << " x " << heightmap.surface.cols() << " px." << std::endl;
    
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
        pangolin::RenderVboIbo(surface_vbo, surface_ibo, true);

        
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
    
    return 0;
}
