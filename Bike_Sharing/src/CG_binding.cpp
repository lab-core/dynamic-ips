#include <pybind11/pybind11.h>
#include "solver/solver.h" // Adjust the path as necessary

namespace py = pybind11;

PYBIND11_MODULE(CG_binding, m) {
    py::class_<solver>(m, "Solver")
            .def(py::init<>())
            .def("createInstance", &solver::createInstance,
                 "Create instance from JSON strings",
                 py::arg("jsonStrDuration"), py::arg("jsonStrParam"),
                 py::arg("jsonStrTasks"), py::arg("jsonStrVehicles"))
            .def("solveCG", &solver::solveCG,
                 "Solve the epoch instance with CG using ISUD");
}
