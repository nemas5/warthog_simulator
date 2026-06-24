#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <PSO/PSO.hpp>

namespace py = pybind11;

PYBIND11_MODULE(pspso, m) {
    py::class_<PSO>(m, "PSO")
        .def(py::init<int, int, double, double, double, int, int, unsigned>(),
            py::arg("subswormsAmount"),
            py::arg("particlesPerSubswormAmount"),
            py::arg("cognitiveCoeff"),
            py::arg("socialCoeff"),
            py::arg("pertrubationCoeff"),
            py::arg("maxOperations"),
            py::arg("dimensions"),
            py::arg("startDynamicsFunction")
        )
        .def("iterate", &PSO::iterate,
            py::arg("basisFunctions"),
            py::arg("realCurrentVelocity")
        );
}