#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace cnsee
{

template<typename T>
struct Tool
{
    Tool(T diameter, T height)
        : diameter(diameter), height(height)
    {
    }

    T diameter;
    T height;
};

template<typename T>
class Heightmap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Heightmap(const Eigen::AlignedBox<T,3>& bounds_mm)
        : max_pixels(5000*5000), tool(3.2, 8.0)
    {
        if(bounds_mm.isEmpty()) {
            throw std::invalid_argument("Heightmap initialized with empy bounds");
        }

        Init(bounds_mm);
    }

    void Init(const Eigen::AlignedBox<T,3>& bounds_mm)
    {
        const float border = 0.01;
        bbox_mm = bounds_mm;
        bbox_mm.min()[0] -= border;
        bbox_mm.min()[1] -= border;
        bbox_mm.max()[0] += border;
        bbox_mm.max()[1] += border;

        res_per_mm = std::sqrt((float)max_pixels / bbox_mm.sizes().template head<2>().prod());
        std::cout << max_pixels << std::endl;
        std::cout << bbox_mm.sizes().template head<2>().prod() << std::endl;
        std::cout << res_per_mm << std::endl;

        Eigen::Vector2i size_pix = (res_per_mm*bbox_mm.sizes().template head<2>()).template cast<int>();
        surface_height = Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>(size_pix[1], size_pix[0]);
        surface = Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic>(size_pix[1], size_pix[0]);
        normals = Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic>(size_pix[1], size_pix[0]);
        for(int r = 0; r < surface.rows(); ++r) {
            for(int c = 0; c < surface.cols(); ++c) {
                const T x = (c / res_per_mm) + bbox_mm.min()[0];
                const T y = (r / res_per_mm) + bbox_mm.min()[1];
                surface(r,c) = Eigen::Matrix<T,3,1>(x,y,(T)0.0);
            }
        }
        surface_height.fill(0.0);
    }

    void Clear(T z = 0.0)
    {
        surface_height.fill(0.0);

        for(int r = 0; r < surface.rows(); ++r) {
            for(int c = 0; c < surface.cols(); ++c) {
                surface(r,c)[2] = z;
            }
        }
    }

    void MillSquare(const Eigen::Matrix<T, 3, 1> &p_w)
    {
        const T rad_mm = tool.diameter / 2.0;

        if(rad_mm > 0.0) {
            const T max_rad_pix = rad_mm * res_per_mm;
            const T max_rad_pix_border = max_rad_pix + 1;

            const Eigen::Matrix<T,2,1> center_pix = (p_w.template head<2>() - bbox_mm.min().template head<2>()) * res_per_mm;
            const Eigen::Matrix<T,2,1> min_pix = center_pix.array() - max_rad_pix_border;
            const Eigen::Matrix<T,2,1> max_pix = center_pix.array() + max_rad_pix_border;

            const Eigen::AlignedBox2i update_pix(min_pix.template cast<int>(), max_pix.template cast<int>() );

            // Update height for max_rad patch around p_w.xy
            Eigen::Vector2i pix;
            for(pix[0] = update_pix.min()[0]; pix[0] <= update_pix.max()[0]; ++pix[0]) {
                for(pix[1] = update_pix.min()[1]; pix[1] <= update_pix.max()[1]; ++pix[1]) {
                    const T rad_pix = (pix.cast<T>() - center_pix).norm();
                    const T tool_height_mm = p_w[2];
                    if(rad_pix <= max_rad_pix && tool_height_mm < Height(pix) ) {
                        Height(pix) = tool_height_mm;
                    }
                }
            }
        }
    }

    void MillV(const Eigen::Matrix<T, 3, 1> &p_w)
    {
        // Surface strictly z < max_surface_height_mm
        const T max_surface_height_mm = 0;
        const T rad = tool.diameter / 2.0;
        const T grad = tool.height / tool.diameter;
        const T max_cut_depth_mm = max_surface_height_mm - p_w[2];
        const T max_rad_mm = std::min(max_cut_depth_mm / grad, rad);

        if(max_rad_mm > 0.0 && grad > 0.0) {
            const T max_rad_pix = max_rad_mm * res_per_mm;
            const T max_rad_pix_border = max_rad_pix + 2;
            const T grad_pix = grad / res_per_mm;

            const Eigen::Matrix<T,2,1> center_pix = (p_w.template head<2>() - bbox_mm.min().template head<2>()) * res_per_mm;
            const Eigen::Matrix<T,2,1> min_pix = center_pix.array() - max_rad_pix_border;
            const Eigen::Matrix<T,2,1> max_pix = center_pix.array() + max_rad_pix_border;

            const Eigen::AlignedBox2i update_pix(min_pix.template cast<int>(), max_pix.template cast<int>() );

            // Update height for max_rad patch around p_w.xy
            Eigen::Vector2i pix;
            for(pix[0] = update_pix.min()[0]; pix[0] <= update_pix.max()[0]; ++pix[0]) {
                for(pix[1] = update_pix.min()[1]; pix[1] <= update_pix.max()[1]; ++pix[1]) {
                    const T rad_pix = (pix.cast<T>() - center_pix).norm();
                    const T tool_height_mm = p_w[2] + rad_pix * grad_pix;
                    if(rad_pix <= max_rad_pix && tool_height_mm < Height(pix) ) {
                        Height(pix) = tool_height_mm;
                    }
                }
            }
        }
    }

    T& Height(const Eigen::Vector2i& p)
    {
//        return surface(p[1],p[0])[2];
        return surface_height(p[1],p[0]);
    }

    uint64_t max_pixels;
    T res_per_mm;
    Eigen::AlignedBox<T,3> bbox_mm;
    Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> surface_height;
    Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic> surface;
    Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic> normals;

    Tool<T> tool;
};

}
