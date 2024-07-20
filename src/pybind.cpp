#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "OnlineTrajGenerator.h" // Include the header of your class

namespace py = pybind11;

PYBIND11_MODULE(online_traj_planner, m)
{
    py::class_<OnlineTrajGenerator>(m, "OnlineTrajGenerator")
        .def(py::init<const Eigen::Vector3d &, const Eigen::Vector3d &, const Eigen::MatrixXd &, const Eigen::MatrixXd &, const std::string &>(),
             py::arg("start"), py::arg("goal"), py::arg("nominalGatePositionAndType"), py::arg("nominalObstaclePosition"), py::arg("configPath"))
        .def("pre_compute_traj", &OnlineTrajGenerator::preComputeTraj, py::arg("takeoffTime"))
        .def("update_gate_pos", &OnlineTrajGenerator::updateGatePos,
             py::arg("gateId"), py::arg("newPose"), py::arg("dronePos"), py::arg("nextGateWithinRange"), py::arg("flightTime"))
        .def("sample_traj", &OnlineTrajGenerator::sampleTraj, py::arg("currentTime"))
        .def("get_traj_end_time", &OnlineTrajGenerator::getTrajEndTime)
        .def("get_planned_traj", &OnlineTrajGenerator::getPlannedTraj);

    py::class_<Eigen::Vector3d>(m, "Vector3d")
        .def(py::init<const Eigen::Ref<const Eigen::Vector3d> &>()); // Allows initialization from a numpy array directly

    py::class_<Eigen::MatrixXd>(m, "MatrixXd")
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd> &>()); // Allows initialization from a numpy array directly
}
