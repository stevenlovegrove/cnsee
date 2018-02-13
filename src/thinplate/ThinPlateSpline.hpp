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

        const int num(int(mSrcPoints.size()));
        const int rows(num + 3 + 1);

        // Create L Matrix
        mL = Eigen::MatrixXd::Zero(rows, rows);

        for (int i(0); i < num; ++i) {

            int j(i + 1);

            for (; j < num; ++j)
            {
                mL(i, j) = mL(j, i) = radialBasis(
                            (mSrcPoints[std::size_t(i)] - mSrcPoints[std::size_t(j)]).norm());
            }

            mL(j, i) = mL(i, j) = 1.0;
            ++j;

            for (int posElm(0); j < rows; ++posElm, ++j)
            {
                mL(j, i) = mL(i, j) = mSrcPoints[std::size_t(i)][posElm];
            }
        }

        // Create Y Matrix
        Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(rows, 3);

        for (int i(0); i < num; ++i)
        {
            Y.row(i) = mDstPoints[std::size_t(i)];
        }

        // Solve L W^T = Y as W^T = L^-1 Y
        mW = mL.colPivHouseholderQr().solve(Y);

        dirty = false;
    }

    Eigen::Vector3d Interpolate(const Eigen::Vector3d &p) const
    {
        if(NumPoints() > 0) {
            Eigen::Vector3d res = Eigen::Vector3d::Zero();
            int i(0);

            for (; i < mW.rows() - (3 + 1); ++i)
            {
                double rb = radialBasis((mSrcPoints[std::size_t(i)] - p).norm());
                res += mW.row(i) * rb;
            }

            res += mW.row(i);
            i++;

            for (int j(0); j < 3; ++j, ++i) {
                res += mW.row(i) * p[j];
            }

            return res;
        }else{
            return p;
        }
    }

    void Clear()
    {
        mSrcPoints.clear();
        mDstPoints.clear();
    }

    void AddWarpedPoint(const Eigen::Vector3d& src, const Eigen::Vector3d& dst)
    {
        mSrcPoints.push_back(src);
        mDstPoints.push_back(dst);
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
    static inline double radialBasis(double r) {
        return r == 0.0 ? r : r * r * log(r);
    }

    bool dirty;
    std::vector<Eigen::Vector3d> mSrcPoints;
    std::vector<Eigen::Vector3d> mDstPoints;
    Eigen::MatrixXd mW;
    Eigen::MatrixXd mL;
};

}
