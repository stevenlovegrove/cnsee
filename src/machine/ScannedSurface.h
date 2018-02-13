#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../thinplate/ThinPlateSpline.hpp"
#include "../cut/Heightmap.h"
#include "../utils/aligned_vector.h"

namespace cnsee {

class ScannedSurface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ScannedSurface()
    {
    }

    ScannedSurface(const Eigen::AlignedBox3d& bounds)
    {
        Clear();
        ResetBounds(bounds);
    }

    void Clear()
    {
        surface_samples.clear();
        tps.Clear();
    }

    void ResetBounds(const Eigen::AlignedBox3d& bounds)
    {
        this->bounds = bounds;
        surface.Init(bounds.cast<float>());
        UpdateSurface();
    }

    void AddSurfacePoint(const Eigen::Vector3d& P)
    {
        surface_samples.push_back(P);
        const Eigen::Vector3d src(P[0], P[1], 0.0);
        const Eigen::Vector3d diff(0.0, 0.0, P[2]);
        tps.AddWarpedPoint(src, diff);
    }

    void UpdateSurface()
    {
        if(surface_samples.size() > 0) {
            tps.Solve();
            for(size_t y=0; y < surface.surface.rows(); ++y) {
                for(size_t x=0; x < surface.surface.cols(); ++x) {
                    Eigen::Vector3f& S = surface.surface(y,x);
                    const Eigen::Vector3d src(S[0],S[1],0.0);
                    const Eigen::Vector3d diff = tps.Interpolate(src);
                    S[2] = 0.0 + diff[2];
                }
            }
        }else{
            surface.Clear();
        }
    }

    Eigen::AlignedBox3d bounds;
    Heightmap<float> surface;
    aligned_vector<Eigen::Vector3d> surface_samples;
    ThinPlateSpline tps;
};

}
