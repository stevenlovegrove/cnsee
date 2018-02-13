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

    void Clear()
    {
        surface_samples.clear();
        tps.Clear();
    }

    void AddSurfacePoint(const Eigen::Vector3d& P)
    {
        surface_samples.push_back(P);
        const Eigen::Vector3d src(P[0], P[1], 0.0);
        const Eigen::Vector3d diff(0.0, 0.0, P[2]);
        tps.AddWarpedPoint(src, diff);
    }

    template<typename T>
    void ApplyToSurface(Heightmap<T>& heightmap)
    {
        if(surface_samples.size() > 0) {
            tps.Solve();
            for(size_t y=0; y < heightmap.surface.rows(); ++y) {
                for(size_t x=0; x < heightmap.surface.cols(); ++x) {
                    auto& S = heightmap.surface(y,x);
                    const Eigen::Vector3d src(S[0],S[1],0.0);
                    S[2] = 0.0 + tps.SurfaceOffset(src);
                }
            }
        }else{
            heightmap.Clear();
        }
    }

    aligned_vector<Eigen::Vector3d> surface_samples;
    ThinPlateSpline tps;
};

}
