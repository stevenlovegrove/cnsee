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

    ScannedSurface(const Eigen::AlignedBox3f& bounds)
    {
        Clear();
        ResetBounds(bounds);
    }

    void Clear()
    {
        surface_samples.clear();
        tps.Clear();
    }

    void ResetBounds(const Eigen::AlignedBox3f& bounds)
    {
        this->bounds = bounds;
        surface.Init(bounds);
    }

    void AddSurfacePoint(const Eigen::Vector3f& P)
    {
        surface_samples.push_back(P);
        Eigen::Vector3d Q(P[0], P[1], 0.0);
        tps.AddWarpedPoint(Q, P.cast<double>());
    }

    void UpdateSurface()
    {
        if(surface_samples.size() > 0) {
            tps.Solve();
            for(size_t y=0; y < surface.surface.rows(); ++y) {
                for(size_t x=0; x < surface.surface.cols(); ++x) {
                    Eigen::Vector3f& S = surface.surface(y,x);
                    const Eigen::Vector3d Q(S[0],S[1],0.0);
                    const Eigen::Vector3d P = tps.Interpolate(Q);
                    S[2] = P[2];
                }
            }
        }else{
            surface.Clear();
        }
    }

    Eigen::AlignedBox3f bounds;
    Heightmap<float> surface;
    aligned_vector<Eigen::Vector3f> surface_samples;
    ThinPlateSpline tps;
};

}
