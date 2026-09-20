#include <pybind11/pybind11.h>

namespace py = pybind11;

int add(int a, int b) { return a + b; }

PYBIND11_MODULE(_gridplan_core, m) {
    m.doc() = "gridplan C++ core — graph-search planners on 2D/3D grids";
    m.def("add", &add, "Smoke-test function: returns a + b", py::arg("a"), py::arg("b"));
}
