// SPDX-License-Identifier: AGPL-3.0-only
#pragma once

#include <Eigenvalues>
#include <Cholesky>

namespace align::detail {

// Solve only in directions constrained by the measurements, before ridge
// regularization adds artificial information. The correction is relative to
// the prior, so an unobservable component learned in an earlier block survives.
template<int N>
bool observableUpdate(const Eigen::Matrix<double, N, N>& information,
    const Eigen::Matrix<double, N, 1>& rhs,
    const Eigen::Matrix<double, N, 1>& prior,
    const Eigen::Matrix<double, N, 1>& thresholds,
    const Eigen::Matrix<double, N, 1>& ridge,
    Eigen::Matrix<double, N, 1>& result)
{
    result = prior;
    if (!information.allFinite() || !rhs.allFinite() || !prior.allFinite()) return false;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> eigen(information);
    if (eigen.info() != Eigen::Success) return false;

    Eigen::Matrix<double, N, N> basis = Eigen::Matrix<double, N, N>::Zero();
    Eigen::Matrix<double, N, 1> values = Eigen::Matrix<double, N, 1>::Zero();
    int count = 0;
    for (int i = 0; i < N; ++i)
    {
        const auto direction = eigen.eigenvectors().col(i);
        // Translation and scale have different excitation thresholds. Compare
        // a coupled direction with the corresponding weighted threshold.
        const double threshold = direction.array().square().matrix().dot(thresholds);
        if (eigen.eigenvalues()[i] > threshold)
        {
            basis.col(count) = direction;
            values[count++] = eigen.eigenvalues()[i];
        }
    }
    if (count == 0) return false;

    Eigen::Matrix<double, N, N> system = Eigen::Matrix<double, N, N>::Identity();
    Eigen::Matrix<double, N, 1> projected = Eigen::Matrix<double, N, 1>::Zero();
    const Eigen::Matrix<double, N, 1> residual = rhs - information * prior;
    for (int i = 0; i < count; ++i)
    {
        projected[i] = basis.col(i).dot(residual);
        for (int j = 0; j < count; ++j)
            system(i, j) = (i == j ? values[i] : 0.0)
                + (basis.col(i).array() * ridge.array() * basis.col(j).array()).sum();
    }
    Eigen::LDLT<Eigen::Matrix<double, N, N>> solve(system);
    if (solve.info() != Eigen::Success || !solve.isPositive()) return false;
    const Eigen::Matrix<double, N, 1> step = basis * solve.solve(projected);
    if (!step.allFinite()) return false;
    result += step;
    return true;
}

} // namespace align::detail
