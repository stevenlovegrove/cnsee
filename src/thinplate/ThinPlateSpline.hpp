#pragma once

// Code refactored version of:
// https://github.com/buresu/ThinPlateSpline

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/StdVector>

namespace cnsee {


class ThinPlateSpline {
public:
    ThinPlateSpline()
        : dirty(false)
    {
    }

    void Solve()
    {
        assert(mSrcPoints.size() == mDstPoints.size());

        if(!dirty) {
            return;
        }

        const size_t num = mSrcPoints.size();
        const size_t rows = num + 3 + 1;

        // Create L Matrix
        Eigen::MatrixXd mL = Eigen::MatrixXd::Zero(rows, rows);

        for (size_t i = 0; i < num; ++i) {

            size_t j = i + 1;

            for (; j < num; ++j)
            {
                const double dist = (mSrcPoints[i] - mSrcPoints[j]).norm();
                mL(i, j) = mL(j, i) = radialBasis(dist);
            }

            mL(j, i) = mL(i, j) = 1.0;
            ++j;

            for (size_t posElm = 0; j < rows; ++posElm, ++j)
            {
                mL(j, i) = mL(i, j) = mSrcPoints[i][posElm];
            }
        }

        // Create Y Matrix
        Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(rows, 3);

        for (size_t i = 0; i < num; ++i)
        {
            Y.row(i) = mDstPoints[i];
        }

        // Solve L W^T = Y as W^T = L^-1 Y
        mW = mL.colPivHouseholderQr().solve(Y);

        dirty = false;
    }

    Eigen::Vector3d Interpolate(const Eigen::Vector3d &q) const
    {
        const Eigen::Vector3d p = TransformUnits(q);

        if(NumPoints() > 0) {
            Eigen::Vector3d res = Eigen::Vector3d::Zero();
            size_t i = 0;

            for (; i < mW.rows() - (3 + 1); ++i)
            {
                const double dist = (mSrcPoints[i] - p).norm();
                const double rb = radialBasis(dist);
                res += mW.row(i) * rb;
            }

            res += mW.row(i);
            i++;

            for (size_t j = 0; j < 3; ++j, ++i) {
                res += mW.row(i) * p[j];
            }

            return res;
        }else{
            return p;
        }
    }

    void AddWarpedPoint(const Eigen::Vector3d& src, const Eigen::Vector3d& dst)
    {
        mSrcPoints.push_back(TransformUnits(src));
        mDstPoints.push_back(dst);
        dirty = true;
    }

    void Clear()
    {
        mSrcPoints.clear();
        mDstPoints.clear();
        dirty = true;
    }


    size_t NumPoints() const
    {
        return mSrcPoints.size();
    }

    bool IsSolutionOutdated() const
    {
        return dirty;
    }

    template<typename Derived>
    typename Derived::Scalar SurfaceOffset(const Eigen::MatrixBase<Derived>& P) const
    {
        const Eigen::Vector3d src(P[0], P[1], 0.0);
        const Eigen::Vector3d diff = Interpolate(src);
        return diff[2];
    }

protected:
    static inline Eigen::Vector3d TransformUnits(const Eigen::Vector3d& P)
    {
        // We're using transformed input points to help with numerical stability.
        // Ideally we would normalise mean and variance properly, but this seems to work.
        return P / 100.0;
    }

    static inline double radialBasis(double r) {
        return r == 0.0 ? r : r * r * log(r);
    }

    bool dirty;
    std::vector<Eigen::Vector3d> mSrcPoints;
    std::vector<Eigen::Vector3d> mDstPoints;
    Eigen::MatrixXd mW;
};

}
