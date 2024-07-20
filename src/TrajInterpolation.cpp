#include "TrajInterpolation.h"
#include <unsupported/Eigen/Splines>
#include <iostream>
Eigen::VectorXd TrajInterpolation::parametrizePath(const std::vector<Eigen::Vector3d> &path) const
{
    std::cout << "parametrizePath" << std::endl;
    Eigen::VectorXd t(path.size());
    t(0) = 0;
    for (int i = 1; i < path.size(); i++)
    {
        t(i) = t(i - 1) + (path[i] - path[i - 1]).norm();
    }
    // Normalize t
    t /= t(t.size() - 1);
    return t;
}

Eigen::Spline3d TrajInterpolation::fitSpline(const Eigen::VectorXd &t, const std::vector<Eigen::Vector3d> &path) const
{
    std::cout << "fitSpline" << std::endl;
    Eigen::MatrixXd points(3, path.size());
    for (int i = 0; i < path.size(); i++)
    {
        points.col(i) = path[i];
    }
    Eigen::Spline3d spline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(points, 3, t);
    return spline;
}

std::vector<Eigen::Vector3d> TrajInterpolation::sampleSpline(const Eigen::Spline3d &spline, const double maxT, const double dt) const
{
    std::cout << "sampleSpline" << std::endl;
    const int numSamples = static_cast<int>(maxT / dt) + 1;
    std::vector<Eigen::Vector3d> samples(numSamples);
    for (int i = 0; i < numSamples; i++)
    {
        const double t = static_cast<double>(i) / (numSamples - 1);
        samples[i] = spline(t);
    }
    return samples;
}

Eigen::MatrixXd TrajInterpolation::interpolateTraj(const std::vector<Eigen::Vector3d> &path, const double maxT, const double advancedTime, const double dt) const
{
    Eigen::VectorXd t = parametrizePath(path);
    Eigen::Spline3d spline = fitSpline(t, path);
    const double maxTNormalized = maxT - advancedTime;
    std::vector<Eigen::Vector3d> samples = sampleSpline(spline, maxTNormalized, dt);
    Eigen::MatrixXd traj(samples.size(), 10);
    for (int i = 0; i < samples.size(); i++)
    {
        const double t = i * dt;
        const Eigen::Vector3d pos = samples[i];
        traj(i, 0) = pos(0);
        traj(i, 1) = 0;
        traj(i, 2) = 0;
        traj(i, 3) = pos(1);
        traj(i, 4) = 0;
        traj(i, 5) = 0;
        traj(i, 6) = pos(2);
        traj(i, 7) = 0;
        traj(i, 8) = 0;
        traj(i, 9) = t + advancedTime;
    }
    return traj;
}

double distance(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
{
    return (p1 - p2).norm();
}

double computeTimeSegment(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const double v_start, const double v_max, const double a_max)
{
    const double d = distance(p1, p2);
    if (d == 0)
    {
        return 0;
    }
    double t_acc = (v_max - v_start) / a_max;
    double d_acc = v_start * t_acc + 0.5 * a_max * t_acc * t_acc;

    if (d_acc >= d / 2)
    {
        t_acc = (-v_start + sqrt(v_start * v_start + 2 * a_max * d / 2)) / a_max;
        return 2 * t_acc;
    }
    else
    {
        t_acc = (v_max - v_start) / a_max;
        double t_dec = (v_max - 0) / a_max;
        double t_const = (d - 2 * d_acc) / v_max;
        return t_acc + t_const + t_dec;
    }
}

Eigen::MatrixXd TrajInterpolation::interpolateTrajMaxVel(const std::vector<Eigen::Vector3d> &path, const double v_start, const double v_max, const double a_max, const double advancedTime, const double dt) const
{
    Eigen::VectorXd t = parametrizePath(path);
    Eigen::Spline3d spline = fitSpline(t, path);
    const double maxTNormalized = 15 - advancedTime;
    std::vector<Eigen::Vector3d> samples = sampleSpline(spline, maxTNormalized, dt);

    std::vector<double> times = {0};
    std::vector<double> velocities = {v_start};

    for (size_t i = 1; i < samples.size(); i++)
    {
        const Eigen::Vector3d p1 = samples[i - 1];
        const Eigen::Vector3d p2 = samples[i];
        const double t = computeTimeSegment(p1, p2, velocities[i - 1], v_max, a_max);
        times.push_back(times[i - 1] + t);
        velocities.push_back(v_max);
    }

    Eigen::MatrixXd traj(samples.size(), 10);
    for (int i = 0; i < samples.size(); i++)
    {
        const Eigen::Vector3d pos = samples[i];
        const double t = times[i];
        traj(i, 0) = pos(0);
        traj(i, 1) = 0;
        traj(i, 2) = 0;
        traj(i, 3) = pos(1);
        traj(i, 4) = 0;
        traj(i, 5) = 0;
        traj(i, 6) = pos(2);
        traj(i, 7) = 0;
        traj(i, 8) = 0;
        traj(i, 9) = t + advancedTime;
    }
    return traj;
}
