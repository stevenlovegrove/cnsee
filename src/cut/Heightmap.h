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

template<typename T, int Dim>
using Line = Eigen::Matrix<T,Dim,2>;

template<typename T>
class Heightmap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Heightmap()
        : max_pixels(0), tool(3.2, 8.0)
    {
    }

    Heightmap(const Eigen::AlignedBox<T,3>& bounds_mm, size_t max_pixels)
        : max_pixels(max_pixels), tool(3.2, 8.0)
    {
        Init(bounds_mm);
    }

    void Init(const Eigen::AlignedBox<T,2>& bounds_mm, size_t max_pixels)
    {
        this->max_pixels = max_pixels;

        const float border_mm = 0.01;
        bbox_mm = bounds_mm;
        if(bbox_mm.isEmpty()) {
            bbox_mm.extend(Eigen::Matrix<T,2,1>::Zero());
        }
        bbox_mm.min().array() -= border_mm;
        bbox_mm.max().array() += border_mm;

        res_per_mm = std::sqrt((float)max_pixels / bbox_mm.sizes().template head<2>().prod());
        Eigen::Vector2i size_pix = (res_per_mm*bbox_mm.sizes()).template cast<int>();
        surface = Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic>(size_pix[1], size_pix[0]);
        normals = Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic>(size_pix[1], size_pix[0]);
        for(int r = 0; r < surface.rows(); ++r) {
            for(int c = 0; c < surface.cols(); ++c) {
                const T x = (c / res_per_mm) + bbox_mm.min()[0];
                const T y = (r / res_per_mm) + bbox_mm.min()[1];
                surface(r,c) = Eigen::Matrix<T,3,1>(x,y,(T)0.0);
            }
        }
    }

    void Clear(T z = 0.0)
    {
        for(int r = 0; r < surface.rows(); ++r) {
            for(int c = 0; c < surface.cols(); ++c) {
                surface(r,c)[2] = z;
            }
        }
    }

    template<typename P>
    Eigen::AlignedBox2i ToPix(const Eigen::AlignedBox<P,2>& box_mm) const
    {
        Eigen::AlignedBox<P,2> box_pix(
            (box_mm.min() - bbox_mm.min()) * res_per_mm,
            (box_mm.max() - bbox_mm.min()) * res_per_mm
        );
        box_pix.min() = box_pix.min().unaryExpr([](P v){return std::floor(v);});
        box_pix.max() = box_pix.max().unaryExpr([](P v){return std::ceil(v);});
        return box_pix.template cast<int>();
    }

    template<typename P>
    void MillSquare(const Line<P,3>& path, const P rad_mm)
    {
        if(rad_mm > 0.0) {
            // Compute bounds in mm
            Eigen::AlignedBox<P,2> box_mm;
            box_mm.extend(path.col(0).template head<2>());
            box_mm.extend(path.col(1).template head<2>());
            box_mm.min().array() -= rad_mm;
            box_mm.max().array() += rad_mm;

            const Eigen::AlignedBox2i bounds_pix(Eigen::Vector2i(0,0), Eigen::Vector2i(surface.cols()-1, surface.rows()-1));
            const Eigen::AlignedBox2i change_pix = ToPix(box_mm);
            const Eigen::AlignedBox2i update_pix = change_pix.intersection(bounds_pix);

            if(!update_pix.isEmpty()) {
                MillSquare(update_pix, path, rad_mm);
            }
        }
    }

    template<typename P>
    void MillSquare(const Eigen::AlignedBox2i& update_pix, const Line<P,3>& path, const P rad_mm)
    {

        // This maths is not obvious. Draw yourself a diagram of the geometry...

        const P rad_sq = rad_mm * rad_mm;

        // Define beginning and end such that beginning is canonically lower in height than end
        const bool pathgood = path(2,0) <= path(2,1);
        const auto& b3 = pathgood ? path.col(0) : path.col(1);
        const auto& e3 = pathgood ? path.col(1) : path.col(0);

        // We'll be using XY coords a lot
        const Eigen::Matrix<P,2,1> b = b3.template head<2>();
        const Eigen::Matrix<P,2,1> e = e3.template head<2>();
        const Eigen::Matrix<P,2,1> be = e-b;
        const P magbe = be.norm();
        const Eigen::Matrix<P,2,1> unitbe = be / magbe;

        // Define rotation matrix to place us in 'line' coordinates
        // such that be runs along the x-axis and the y-axis is orthogonal CCW.
        Eigen::Matrix<P,2,2> R_l_m;
        R_l_m << unitbe[0], unitbe[1],
                -unitbe[1], unitbe[0];

        Eigen::Vector2i pix;
        for(pix[0] = update_pix.min()[0]; pix[0] <= update_pix.max()[0]; ++pix[0]) {
            for(pix[1] = update_pix.min()[1]; pix[1] <= update_pix.max()[1]; ++pix[1]) {
                Eigen::Matrix<T,3,1>& S = surface(pix[1], pix[0]);

                const Eigen::Matrix<T,2,1> p = S.template head<2>();
                const P distb_sq = (p - b).squaredNorm();
                const P diste_sq = (p - e).squaredNorm();

                // Looking top down, are we in the path of the tool (circle -> rect -> circle)?

                // Are we inside of beginning circle?
                if(distb_sq <= rad_sq) {
                    // Special case where we can terminate early (the height is just the endmill height)
                    S[2] = std::min(S[2], b3[2]);
                    continue;
                }

//                {//temp
//                // Are we inside of beginning circle?
//                if(diste_sq <= rad_sq) {
//                    // Special case where we can terminate early (the height is just the endmill height)
//                    S[2] = std::min(S[2], e3[2]);
//                    continue;
//                }
//                continue;
//                }

                // Are we inside of end circle?
                bool inside = diste_sq <= rad_sq;

                const Eigen::Matrix<T,2,1> bp = p-b;

                // If not, lets check if we're inside of the rectangle
                if(!inside) {
                    // XY location of point, and convert into 'line' coordinates
                    const Eigen::Matrix<T,2,1> bp_l = R_l_m * bp;

                    // This shape is the union of two circles and a rectangle
                    inside = (0 <= bp_l[0] && bp_l[0] <= magbe ) && (std::abs(bp_l[1]) < rad_mm);
                }

                if(inside) {
                    // We are in the 'shadow' of the shape. Now need to compute lower height boundary at p

                    // Observation: The lowest the endmill would intersect at p is at the circumference.
                    // Solve for when this would happen.

                    // rad^2 = ||lambda*(e-b) + (b-p)||^2
                    // Solve as quadratic: ax^2 + bx + c = 0;
                    const P a = be.squaredNorm();
                    const P b = -2.0 * be.transpose() * bp;
                    const P c = bp.squaredNorm() - rad_sq;
                    const P fac = b*b - 4*a*c;
                    if(fac >= 0) {
                        const P bb_4ac = std::sqrt(fac);
                        // There are two solutions, but we always want the lowest
                        // which for us is the smallest lambda (since we made sure b <= e)
                        const P l = (-b - bb_4ac) / (2*a);
                        const P h = b3[2] + l * (e3[2] - b3[2]);
                        S[2] = std::min(S[2], h);
                    }else{
                        // It should not be possible for this to happen.
                        assert(false && "Not expecting case to be possible.");
                    }
                }
            }
        }
    }

    void MillSquare(const Eigen::Matrix<T, 3, 1> &p_w)
    {
        // Surface strictly z < max_surface_height_mm
        const T rad_mm = tool.diameter / 2.0;

        if(rad_mm > 0.0) {
            const T max_rad_pix = rad_mm * res_per_mm;
            const T max_rad_pix_border = max_rad_pix + 1;

            const Eigen::Matrix<T,2,1> center_pix = (p_w.template head<2>() - bbox_mm.min()) * res_per_mm;
            const Eigen::Matrix<T,2,1> min_pix = center_pix.array() - max_rad_pix_border;
            const Eigen::Matrix<T,2,1> max_pix = center_pix.array() + max_rad_pix_border;

            const Eigen::AlignedBox2i bounds_pix(Eigen::Vector2i(0,0), Eigen::Vector2i(surface.cols()-1, surface.rows()-1));
            const Eigen::AlignedBox2i change_pix(min_pix.template cast<int>(), max_pix.template cast<int>() );
            const Eigen::AlignedBox2i update_pix = change_pix.intersection(bounds_pix);

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

            const Eigen::AlignedBox2i bounds_pix(Eigen::Vector2i(0,0), Eigen::Vector2i(surface.cols(), surface.rows()));
            const Eigen::AlignedBox2i change_pix(min_pix.template cast<int>(), max_pix.template cast<int>() );
            const Eigen::AlignedBox2i update_pix = change_pix.intersection(bounds_pix);

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
        return surface(p[1],p[0])[2];
    }

    size_t max_pixels;
    T res_per_mm;
    Eigen::AlignedBox<T,2> bbox_mm;
    Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic> surface;
    Eigen::Matrix<Eigen::Matrix<T,3,1>,Eigen::Dynamic,Eigen::Dynamic> normals;

    Tool<T> tool;
};

}
