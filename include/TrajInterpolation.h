#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

class TrajInterpolation
{
public:
    Eigen::MatrixXd interpolateTraj(const std::vector<Eigen::Vector3d> &path, const double maxT, const double advancedTime, const double dt) const;
    Eigen::MatrixXd interpolateTrajMaxVel(const std::vector<Eigen::Vector3d> &path, const double v_start, const double v_max, const double a_max, const double advancedTime, const double dt) const;

private:
    Eigen::VectorXd parametrizePath(const std::vector<Eigen::Vector3d> &path) const;
    Eigen::Spline3d fitSpline(const Eigen::VectorXd &t, const std::vector<Eigen::Vector3d> &path) const;
    std::vector<Eigen::Vector3d> sampleSpline(const Eigen::Spline3d &spline, const double maxT, const double dt) const;


};