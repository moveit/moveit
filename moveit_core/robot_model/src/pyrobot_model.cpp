#include <pybind11/pybind11.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>

namespace py = pybind11;

auto load_robot_model(const std::string &urdf_path, const std::string &srdf_path)
{
    auto urdf_model = urdf::parseURDFFile(urdf_path);
    auto srdf_model = std::make_shared<srdf::Model>();
    srdf_model->initFile(*urdf_model, srdf_path);
    auto robot_model = std::make_shared<robot_model::RobotModel>(urdf_model, srdf_model);
    return robot_model;
}

void def_robot_model_bindings(py::module &m)
{
    m.doc() = "Definition of a kinematic model. Not thread safe, however multiple instances can be created.";
    py::class_<robot_model::RobotModel, robot_model::RobotModelPtr>(m, "RobotModel")
            .def(py::init<const urdf::ModelInterfaceSharedPtr &,
                         const srdf::ModelConstSharedPtr &>(),
                 py::arg("urdf_model"),
                 py::arg("srdf_model")
            )
            .def("getURDF", &robot_model::RobotModel::getURDF)
        // This comment prevents clang-format from moving the semicolon
            ;

    // convenience function
    m.def("load_robot_model", &load_robot_model);
}
