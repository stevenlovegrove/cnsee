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

    Heightmap(const Eigen::AlignedBox<T,2>& bounds_mm)
        : res_per_mm(10.0f)
    {
        Init(bounds_mm);
    }

    void Init(const Eigen::AlignedBox<T,2>& bounds_mm)
    {
        bbox_mm = bounds_mm;
        bbox_mm.min()[0] -= 10;
        bbox_mm.min()[1] -= 10;
        bbox_mm.max()[0] += 10;
        bbox_mm.max()[1] += 10;

        Eigen::Vector2i size_pix = (res_per_mm*bbox_mm.sizes()).template cast<int>();
        surface = Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic>(size_pix[1], size_pix[0]);
        for(int r = 0; r < surface.rows(); ++r) {
            for(int c = 0; c < surface.cols(); ++c) {
                const T x = (c / res_per_mm) + bbox_mm.min()[0];
                const T y = (r / res_per_mm) + bbox_mm.min()[1];
                surface(r,c) = Eigen::Matrix<T,3,1>(x,y,(T)0.0);
            }
        }
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
            const T max_rad_pix = max_rad_mm * res_per_mm;
            const T grad_pix = grad / res_per_mm;

            const Eigen::Matrix<T,2,1> center_pix = (p_w.template head<2>() - bbox_mm.min()) * res_per_mm;
            const Eigen::Matrix<T,2,1> min_pix = center_pix.array() - max_rad_pix;
            const Eigen::Matrix<T,2,1> max_pix = center_pix.array() + max_rad_pix;

            const Eigen::AlignedBox2i update_pix(min_pix.template cast<int>(), max_pix.template cast<int>() );

            // Update height for max_rad patch around p_w.xy
            Eigen::Vector2i pix;
            for(pix[0] = update_pix.min()[0]; pix[0] <= update_pix.max()[0]; ++pix[0]) {
                for(pix[1] = update_pix.min()[1]; pix[1] <= update_pix.max()[1]; ++pix[1]) {
                    const T rad_pix = (pix.cast<T>() - center_pix).norm();
                    const T tool_height_mm = p_w[2] + rad_pix * grad_pix;
                    if(tool_height_mm < Height(pix) ) {
                        Height(pix) = tool_height_mm;
                    }
                }
            }
        }
    }

    T& Height(const Eigen::Vector2i& p)
    {
        return surface(p[1],p[0])[2];
    }

    T res_per_mm;
    Eigen::AlignedBox<T,2> bbox_mm;
    Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic> surface;
};

}
