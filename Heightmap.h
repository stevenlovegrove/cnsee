#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace cnsee
{

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

}
