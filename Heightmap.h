#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace cnsee
{

template<typename T>
class Heightmap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Heightmap(const Eigen::AlignedBox<T,2>& bounds)
        : res_per_mm(10.0f)
    {
        bounds_mm = bounds;
        bounds_mm.min()[0] -= 100;
        bounds_mm.min()[1] -= 100;
        bounds_mm.max()[0] += 100;
        bounds_mm.max()[1] += 100;
        Eigen::Vector2i size_pix = (res_per_mm*bounds_mm.sizes()).template cast<int>();
        surface = Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>::Zero(size_pix[0], size_pix[1]);
    }

    // Assume V tip for now
    void Mill(const Eigen::Matrix<T,3,1>& p_w)
    {
        // Surface strictly z < max_surface_height_mm
        const T max_surface_height_mm = 0;
        const T grad = 1.0;
        const T max_cut_depth_mm = max_surface_height_mm - p_w[2];
        const T max_rad_mm = grad * max_cut_depth_mm;

        if(max_rad_mm > 0.0) {
            const Eigen::Matrix<T,2,1> min_mm = p_w.template head<2>().template array() - max_rad_mm;
            const Eigen::Matrix<T,2,1> max_mm = p_w.template head<2>().template array() + max_rad_mm;
            const Eigen::AlignedBox<T,2> update_pixf(res_per_mm*min_mm - bounds_mm.min(), res_per_mm*max_mm - bounds_mm.min());
            const Eigen::AlignedBox2i update_pix = update_pixf.template cast<int>();

            // Update height for max_rad patch around p_w.xy
            const Eigen::Matrix<T,2,1> center_pix = (p_w.template head<2>() - bounds_mm.min()) * res_per_mm;
            const T grad_pix = grad * res_per_mm;
            Eigen::Vector2i pix;
            for(pix[0] = update_pix.min()[0]; pix[0] <= update_pix.max()[0]; ++pix[0]) {
                for(pix[1] = update_pix.min()[1]; pix[1] <= update_pix.max()[1]; ++pix[1]) {
                    const T rad = (pix.cast<T>() - center_pix).norm();
                    const T tool_height_mm = p_w[2] + rad * grad_pix;
                    if(tool_height_mm < surface(pix[1],pix[0]) ) {
                        surface(pix[1],pix[0]) = tool_height_mm;
                    }
                }
            }
        }
    }

    double res_per_mm;
    Eigen::AlignedBox<T,2> bounds_mm;
    Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> surface;
};

}
