#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "gridplan/grid.hpp"
#include "gridplan/planner.hpp"
#include "gridplan/bfs.hpp"
#include "gridplan/dijkstra.hpp"
#include "gridplan/astar.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_gridplan_core, m) {
	m.doc() = "gridplan C++ core — graph-search planners on 2D/3D grids";

	// ---- Enums ----
	py::enum_<gridplan::Connectivity>(m, "Connectivity")
		.value("FOUR", gridplan::Connectivity::FOUR)
		.value("EIGHT", gridplan::Connectivity::EIGHT)
		.value("SIX", gridplan::Connectivity::SIX)
		.value("EIGHTEEN", gridplan::Connectivity::EIGHTEEN)
		.value("TWENTYSIX", gridplan::Connectivity::TWENTYSIX);

	py::enum_<gridplan::Heuristic>(m, "Heuristic")
		.value("MANHATTAN", gridplan::Heuristic::MANHATTAN)
		.value("OCTILE", gridplan::Heuristic::OCTILE)
		.value("EUCLIDEAN", gridplan::Heuristic::EUCLIDEAN)
		.value("ZERO", gridplan::Heuristic::ZERO);

	// ---- Grid ----
	py::class_<gridplan::Grid>(m, "Grid")
		.def(py::init<uint32_t, uint32_t, const std::vector<uint8_t>&, gridplan::Connectivity>(),
			py::arg("rows"), py::arg("cols"), py::arg("grid"), py::arg("connectivity"))
		.def(py::init<uint32_t, uint32_t, uint32_t, const std::vector<uint8_t>&, gridplan::Connectivity>(),
			py::arg("rows"), py::arg("cols"), py::arg("depth"), py::arg("grid"), py::arg("connectivity"))
		.def("get_neighbors", &gridplan::Grid::get_neighbors, py::arg("index"))
		.def("size", &gridplan::Grid::size)
		.def("is_obstacle", &gridplan::Grid::is_obstacle, py::arg("index"))
		.def("index_to_coords", &gridplan::Grid::index_to_coords, py::arg("index"))
		.def("get_rows", &gridplan::Grid::get_rows)
		.def("get_cols", &gridplan::Grid::get_cols)
		.def("get_depth", &gridplan::Grid::get_depth);

	// ---- PlannerConfig ----
	py::class_<gridplan::PlannerConfig>(m, "PlannerConfig")
		.def(py::init([](bool tie_break, bool record_expansions, float weight, gridplan::Heuristic heuristic) {
			return gridplan::PlannerConfig{tie_break, record_expansions, weight, heuristic};
		}), py::arg("tie_break") = false, py::arg("record_expansions") = false,
			py::arg("weight") = 1.0f, py::arg("heuristic") = gridplan::Heuristic::MANHATTAN)
		.def_readwrite("tie_break", &gridplan::PlannerConfig::tie_break)
		.def_readwrite("record_expansions", &gridplan::PlannerConfig::record_expansions)
		.def_readwrite("weight", &gridplan::PlannerConfig::weight)
		.def_readwrite("heuristic", &gridplan::PlannerConfig::heuristic);

	// ---- PlannerResult ----
	py::class_<gridplan::PlannerResult>(m, "PlannerResult")
		.def_readonly("path", &gridplan::PlannerResult::path)
		.def_readonly("cost", &gridplan::PlannerResult::cost)
		.def_readonly("nodes_expanded", &gridplan::PlannerResult::nodes_expanded)
		.def_readonly("expansion_order", &gridplan::PlannerResult::expansion_order)
		.def_property_readonly("time_ms", [](const gridplan::PlannerResult& r) {
			return r.time.count();
		});

	// ---- Planners ----
	py::class_<gridplan::BFS>(m, "BFS")
		.def(py::init<>())
		.def("plan", &gridplan::BFS::plan,
			py::arg("grid"), py::arg("start"), py::arg("goal"), py::arg("config"),
			py::call_guard<py::gil_scoped_release>());

	py::class_<gridplan::Dijkstra>(m, "Dijkstra")
		.def(py::init<>())
		.def("plan", &gridplan::Dijkstra::plan,
			py::arg("grid"), py::arg("start"), py::arg("goal"), py::arg("config"),
			py::call_guard<py::gil_scoped_release>());

	py::class_<gridplan::AStar>(m, "AStar")
		.def(py::init<>())
		.def("plan", &gridplan::AStar::plan,
			py::arg("grid"), py::arg("start"), py::arg("goal"), py::arg("config"),
			py::call_guard<py::gil_scoped_release>());
}
