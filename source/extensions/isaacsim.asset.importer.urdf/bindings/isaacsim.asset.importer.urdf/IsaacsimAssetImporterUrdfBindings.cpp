// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <carb/BindingsPythonUtils.h>

#include <isaacsim/asset/importer/urdf/IUrdf.h>
#include <isaacsim/core/includes/math/core/Maths.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

CARB_BINDINGS("isaacsim.asset.importer.urdf.python")
PYBIND11_MAKE_OPAQUE(std::map<std::string, isaacsim::asset::importer::urdf::UrdfMaterial>);

namespace isaacsim
{
namespace asset
{
namespace importer
{
namespace urdf
{
}
}
}
}
namespace
{
// Helper function that creates a python type for a std::map with a string key and a custom value type
template <class T>
void declare_map(py::module& m, const std::string typestr)
{
    py::class_<std::map<std::string, T>>(m, typestr.c_str())
        .def(py::init<>())
        .def("__getitem__",
             [](const std::map<std::string, T>& map, std::string key)
             {
                 try
                 {
                     return map.at(key);
                 }
                 catch (const std::out_of_range&)
                 {
                     throw py::key_error("key '" + key + "' does not exist");
                 }
             })
        .def(
            "__iter__", [](std::map<std::string, T>& items) { return py::make_key_iterator(items.begin(), items.end()); },
            py::keep_alive<0, 1>())

        .def(
            "items", [](std::map<std::string, T>& items) { return py::make_iterator(items.begin(), items.end()); },
            py::keep_alive<0, 1>())
        .def("__len__", [](std::map<std::string, T>& items) { return items.size(); });
}

PYBIND11_MODULE(_urdf, m)
{
    using namespace carb;
    using namespace isaacsim::asset::importer::urdf;

    m.doc() = R"pbdoc(
        This extension provides an interface to the URDF importer.

        Example:
            Setup the configuration parameters before importing.
            Files must be parsed before imported.

            .. code-block:: python

                >>> import omni
                >>> import os
                >>> from isaacsim.asset.importer.urdf import _urdf
                >>> urdf_interface = _urdf.acquire_urdf_interface()
                >>>
                >>> # setup config params
                >>> import_config = _urdf.ImportConfig()
                >>> import_config.set_merge_fixed_joints(False)
                >>> import_config.set_fix_base(True)
                >>>
                >>> # parse and import file
                >>> ext_manager = omni.kit.app.get_app().get_extension_manager()
                >>> ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
                >>> extension_path = ext_manager.get_extension_path(ext_id)
                >>> urdf_path = os.path.abspath(
                ...     extension_path + "/data/urdf/robots/franka_description/robots/"
                ... )
                >>> filename = "panda_arm_hand.urdf"
                >>> imported_robot = urdf_interface.parse_urdf(urdf_path, filename, import_config)
                >>> urdf_interface.import_robot(urdf_path, filename, imported_robot, import_config, "")
                '/panda'


        Refer to the sample documentation for more examples and usage
                )pbdoc";


    py::class_<ImportConfig>(m, "ImportConfig")
        .def(py::init<>())
        .def_readwrite("merge_fixed_joints", &ImportConfig::mergeFixedJoints,
                       "Consolidating links that are connected by fixed joints")
        .def_readwrite("convex_decomp", &ImportConfig::convexDecomp,
                       "Decompose a convex mesh into smaller pieces for a closer fit")
        .def_readwrite("import_inertia_tensor", &ImportConfig::importInertiaTensor,
                       "Import inertia tensor from urdf, if not specified in urdf it will import as identity")
        .def_readwrite("fix_base", &ImportConfig::fixBase, "Create fix joint for base link")
        // .def_readwrite("flip_visuals", &ImportConfig::flipVisuals, "Flip visuals from Y up to Z up")
        .def_readwrite("self_collision", &ImportConfig::selfCollision, "Self collisions between links in the articulation")
        .def_readwrite("density", &ImportConfig::density, "default density used for links, use 0 to autocompute")
        .def_readwrite("default_drive_type", &ImportConfig::defaultDriveType, "default drive type used for joints")
        .def_readwrite(
            "subdivision_scheme", &ImportConfig::subdivisionScheme, "Subdivision scheme to be used for mesh normals")
        .def_readwrite(
            "default_drive_strength", &ImportConfig::defaultDriveStrength,
            "default drive stiffness used for joints if drive type is position or velocity and none is authored")
        .def_readwrite("default_position_drive_damping", &ImportConfig::defaultPositionDriveDamping,
                       "default drive damping used if drive type is set to position and no damping is authored")
        .def_readwrite("distance_scale", &ImportConfig::distanceScale,
                       "Set the unit scaling factor, 1.0 means meters, 100.0 means cm")
        .def_readwrite("up_vector", &ImportConfig::upVector, "Up vector used for import")
        .def_readwrite("create_physics_scene", &ImportConfig::createPhysicsScene,
                       "add a physics scene to the stage on import if none exists")
        .def_readwrite("make_default_prim", &ImportConfig::makeDefaultPrim, "set imported robot as default prim")
        .def_readwrite("collision_from_visuals", &ImportConfig::collisionFromVisuals,
                       "Generate convex collision from the visual meshes.")
        .def_readwrite("replace_cylinders_with_capsules", &ImportConfig::replaceCylindersWithCapsules,
                       "Replace all cylinder bodies in the URDF with capsules.")
        .def_readwrite("parse_mimic", &ImportConfig::parseMimic, "Parse Mimic Joint flag using PhysX Tendons")
        .def_readwrite(
            "override_joint_dynamics", &ImportConfig::overrideJointDynamics, "Use default values for all joints")
        // setters for each property
        .def("set_merge_fixed_joints", [](ImportConfig& config, const bool value) { config.mergeFixedJoints = value; })
        .def("set_replace_cylinders_with_capsules",
             [](ImportConfig& config, const bool value) { config.replaceCylindersWithCapsules = value; })
        .def("set_convex_decomp", [](ImportConfig& config, const bool value) { config.convexDecomp = value; })
        .def("set_import_inertia_tensor",
             [](ImportConfig& config, const bool value) { config.importInertiaTensor = value; })
        .def("set_fix_base", [](ImportConfig& config, const bool value) { config.fixBase = value; })
        // .def("set_flip_visuals", [](ImportConfig& config, const bool value) { config.flipVisuals = value; })
        .def("set_self_collision", [](ImportConfig& config, const bool value) { config.selfCollision = value; })
        .def("set_density", [](ImportConfig& config, const float value) { config.density = value; })
        .def("set_default_drive_type", [](ImportConfig& config, const int value)
             { config.defaultDriveType = static_cast<UrdfJointTargetType>(value); })
        .def("set_subdivision_scheme", [](ImportConfig& config, const int value)
             { config.subdivisionScheme = static_cast<UrdfNormalSubdivisionScheme>(value); })
        .def("set_default_drive_strength",
             [](ImportConfig& config, const float value) { config.defaultDriveStrength = value; })
        .def("set_default_position_drive_damping",
             [](ImportConfig& config, const float value) { config.defaultPositionDriveDamping = value; })
        .def("set_distance_scale", [](ImportConfig& config, const float value) { config.distanceScale = value; })
        .def("set_up_vector",
             [](ImportConfig& config, const float x, const float y, const float z) {
                 config.upVector = { x, y, z };
             })
        .def("set_create_physics_scene",
             [](ImportConfig& config, const bool value) { config.createPhysicsScene = value; })
        .def("set_make_default_prim", [](ImportConfig& config, const bool value) { config.makeDefaultPrim = value; })
        .def("set_collision_from_visuals",
             [](ImportConfig& config, const bool value) { config.collisionFromVisuals = value; })
        .def("set_parse_mimic", [](ImportConfig& config, const bool value) { config.parseMimic = value; })
        .def("set_override_joint_dynamics",
             [](ImportConfig& config, const bool value) { config.overrideJointDynamics = value; });

    py::class_<Vec3>(m, "Position", "")
        .def_readwrite("x", &Vec3::x, "")
        .def_readwrite("y", &Vec3::y, "")
        .def_readwrite("z", &Vec3::z, "")
        .def(py::init<>());

    py::class_<Quat>(m, "Orientation", "")
        .def_readwrite("w", &Quat::w, "")
        .def_readwrite("x", &Quat::x, "")
        .def_readwrite("y", &Quat::y, "")
        .def_readwrite("z", &Quat::z, "")
        .def(py::init<>());

    py::class_<Transform>(m, "UrdfOrigin", "")
        .def_readwrite("p", &Transform::p, "")
        .def_readwrite("q", &Transform::q, "")
        .def(py::init<>());

    py::class_<UrdfInertia>(m, "UrdfInertia", "")
        .def_readwrite("ixx", &UrdfInertia::ixx, "")
        .def_readwrite("ixy", &UrdfInertia::ixy, "")
        .def_readwrite("ixz", &UrdfInertia::ixz, "")
        .def_readwrite("iyy", &UrdfInertia::iyy, "")
        .def_readwrite("iyz", &UrdfInertia::iyz, "")
        .def_readwrite("izz", &UrdfInertia::izz, "")
        .def(py::init<>());

    py::class_<UrdfInertial>(m, "UrdfInertial", "")
        .def_readwrite("origin", &UrdfInertial::origin, "")
        .def_readwrite("mass", &UrdfInertial::mass, "")
        .def_readwrite("inertia", &UrdfInertial::inertia, "")
        .def_readwrite("has_origin", &UrdfInertial::hasOrigin, "")
        .def_readwrite("has_mass", &UrdfInertial::hasMass, "")
        .def_readwrite("has_inertia", &UrdfInertial::hasInertia, "")
        .def(py::init<>());

    py::class_<UrdfAxis>(m, "UrdfAxis", "")
        .def_readwrite("x", &UrdfAxis::x, "")
        .def_readwrite("y", &UrdfAxis::y, "")
        .def_readwrite("z", &UrdfAxis::z, "")
        .def(py::init<>());

    py::class_<UrdfColor>(m, "UrdfColor", "")
        .def_readwrite("r", &UrdfColor::r, "")
        .def_readwrite("g", &UrdfColor::g, "")
        .def_readwrite("b", &UrdfColor::b, "")
        .def_readwrite("a", &UrdfColor::a, "")
        .def(py::init<>());

    py::enum_<UrdfJointType>(m, "UrdfJointType", py::arithmetic(), "")
        .value("JOINT_REVOLUTE", UrdfJointType::REVOLUTE)
        .value("JOINT_CONTINUOUS", UrdfJointType::CONTINUOUS)
        .value("JOINT_PRISMATIC", UrdfJointType::PRISMATIC)
        .value("JOINT_FIXED", UrdfJointType::FIXED)
        .value("JOINT_FLOATING", UrdfJointType::FLOATING)
        .value("JOINT_PLANAR", UrdfJointType::PLANAR)
        .export_values();

    py::enum_<UrdfJointTargetType>(m, "UrdfJointTargetType", py::arithmetic(), "")
        .value("JOINT_DRIVE_NONE", UrdfJointTargetType::NONE)
        .value("JOINT_DRIVE_POSITION", UrdfJointTargetType::POSITION)
        .value("JOINT_DRIVE_VELOCITY", UrdfJointTargetType::VELOCITY)
        .export_values();

    py::enum_<UrdfJointDriveType>(m, "UrdfJointDriveType", py::arithmetic(), "")
        .value("JOINT_DRIVE_ACCELERATION", UrdfJointDriveType::ACCELERATION)
        .value("JOINT_DRIVE_FORCE", UrdfJointDriveType::FORCE)
        .export_values();

    py::class_<UrdfDynamics>(m, "UrdfDynamics", "")
        .def_readwrite("damping", &UrdfDynamics::damping, "")
        .def_readwrite("friction", &UrdfDynamics::friction, "")
        .def_readwrite("stiffness", &UrdfDynamics::stiffness, "")
        .def("set_damping", [](UrdfDynamics& drive, const float value) { drive.damping = value; })
        .def("set_friction", [](UrdfDynamics& drive, const float value) { drive.friction = value; })
        .def("set_stiffness", [](UrdfDynamics& drive, const float value) { drive.stiffness = value; })
        .def(py::init<>());

    py::class_<UrdfJointDrive>(m, "UrdfJointDrive", "")
        .def_readwrite("target", &UrdfJointDrive::target, "")
        .def_readwrite("strength", &UrdfJointDrive::strength, "")
        .def_readwrite("damping", &UrdfJointDrive::damping, "")
        .def_readwrite("target_type", &UrdfJointDrive::targetType, "")
        .def_readwrite("drive_type", &UrdfJointDrive::driveType, "")
        .def_readwrite("natural_frequency", &UrdfJointDrive::naturalFrequency, "")
        .def_readwrite("damping_ratio", &UrdfJointDrive::dampingRatio, "")
        .def("set_target", [](UrdfJointDrive& drive, const float value) { drive.target = value; })
        .def("set_strength", [](UrdfJointDrive& drive, const float value) { drive.strength = value; })
        .def("set_damping", [](UrdfJointDrive& drive, const float value) { drive.damping = value; })
        .def("set_target_type",
             [](UrdfJointDrive& drive, const int value) { drive.targetType = static_cast<UrdfJointTargetType>(value); })
        .def("set_drive_type",
             [](UrdfJointDrive& drive, const int value) { drive.driveType = static_cast<UrdfJointDriveType>(value); })
        .def(py::init<>());

    py::class_<UrdfJointMimic>(m, "UrdfJointMimic", "")
        .def_readwrite("joint", &UrdfJointMimic::joint, "")
        .def_readwrite("multiplier", &UrdfJointMimic::multiplier, "")
        .def_readwrite("offset", &UrdfJointMimic::offset, "")
        .def(py::init<>());

    py::class_<UrdfLimit>(m, "UrdfLimit", "")
        .def_readwrite("lower", &UrdfLimit::lower, "")
        .def_readwrite("upper", &UrdfLimit::upper, "")
        .def_readwrite("effort", &UrdfLimit::effort, "")
        .def_readwrite("velocity", &UrdfLimit::velocity, "")
        .def("set_lower", [](UrdfLimit& limit, const float value) { limit.lower = value; })
        .def("set_upper", [](UrdfLimit& limit, const float value) { limit.upper = value; })
        .def("set_effort", [](UrdfLimit& limit, const float value) { limit.effort = value; })
        .def("set_velocity", [](UrdfLimit& limit, const float value) { limit.velocity = value; })
        .def(py::init<>());

    py::enum_<UrdfGeometryType>(m, "UrdfGeometryType", py::arithmetic(), "")
        .value("GEOMETRY_BOX", UrdfGeometryType::BOX)
        .value("GEOMETRY_CYLINDER", UrdfGeometryType::CYLINDER)
        .value("GEOMETRY_CAPSULE", UrdfGeometryType::CAPSULE)
        .value("GEOMETRY_SPHERE", UrdfGeometryType::SPHERE)
        .value("GEOMETRY_MESH", UrdfGeometryType::MESH)
        .export_values();

    py::class_<UrdfGeometry>(m, "UrdfGeometry", "")
        .def_readwrite("type", &UrdfGeometry::type, "")
        .def_readwrite("size_x", &UrdfGeometry::size_x, "")
        .def_readwrite("size_y", &UrdfGeometry::size_y, "")
        .def_readwrite("size_z", &UrdfGeometry::size_z, "")
        .def_readwrite("radius", &UrdfGeometry::radius, "")
        .def_readwrite("length", &UrdfGeometry::length, "")
        .def_readwrite("scale_x", &UrdfGeometry::scale_x, "")
        .def_readwrite("scale_y", &UrdfGeometry::scale_y, "")
        .def_readwrite("scale_z", &UrdfGeometry::scale_z, "")
        .def_readwrite("mesh_file_path", &UrdfGeometry::meshFilePath, "")

        .def(py::init<>());


    py::class_<UrdfMaterial>(m, "UrdfMaterial", "")
        .def_readwrite("name", &UrdfMaterial::name, "")
        .def_readwrite("color", &UrdfMaterial::color, "")
        .def_readwrite("texture_file_path", &UrdfMaterial::textureFilePath, "")
        .def(py::init<>());


    py::class_<UrdfVisual>(m, "UrdfVisual", "")
        .def_readwrite("name", &UrdfVisual::name, "")
        .def_readwrite("origin", &UrdfVisual::origin, "")
        .def_readwrite("geometry", &UrdfVisual::geometry, "")
        .def_readwrite("material", &UrdfVisual::material, "")
        .def(py::init<>());

    py::class_<UrdfCollision>(m, "UrdfCollision", "")
        .def_readwrite("name", &UrdfCollision::name, "")
        .def_readwrite("origin", &UrdfCollision::origin, "")
        .def_readwrite("geometry", &UrdfCollision::geometry, "")
        .def(py::init<>());

    py::class_<UrdfCamera>(m, "UrdfCamera", "")
        .def_readwrite("name", &UrdfCamera::name, "")
        .def_readwrite("origin", &UrdfCamera::origin, "")
        .def_readwrite("update_rate", &UrdfCamera::updateRate, "")
        .def_readwrite("width", &UrdfCamera::width, "")
        .def_readwrite("height", &UrdfCamera::height, "")
        .def_readwrite("format", &UrdfCamera::format, "")
        .def_readwrite("h_fov", &UrdfCamera::hfov, "")
        .def_readwrite("clip_near", &UrdfCamera::clipNear, "")
        .def_readwrite("clip_far", &UrdfCamera::clipFar, "")
        .def(py::init<>());

    py::class_<UrdfRayDim>(m, "UrdfRayDim", "")
        .def_readwrite("samples", &UrdfRayDim::samples)
        .def_readwrite("resolution", &UrdfRayDim::resolution)
        .def_readwrite("min_angle", &UrdfRayDim::minAngle)
        .def_readwrite("max_angle", &UrdfRayDim::maxAngle)
        .def(py::init<>());

    py::class_<UrdfRay>(m, "UrdfRay", "")
        .def_readwrite("name", &UrdfRay::name, "")
        .def_readwrite("origin", &UrdfRay::origin, "")
        .def_readwrite("update_rate", &UrdfRay::updateRate, "")
        .def_readwrite("has_horizontal", &UrdfRay::hasHorizontal)
        .def_readwrite("has_vertical", &UrdfRay::hasVertical)
        .def_readwrite("horizontal", &UrdfRay::horizontal)
        .def_readwrite("vertical", &UrdfRay::vertical)
        // .def_readwrite("min", &UrdfRay::min)
        // .def_readwrite("max", &UrdfRay::max)
        // .def_readwrite("resolution", &UrdfRay::resolution)
        .def(py::init<>());

    py::class_<UrdfLink>(m, "UrdfLink", "")
        .def_readwrite("name", &UrdfLink::name, "")
        .def_readwrite("inertial", &UrdfLink::inertial, "")
        .def_readwrite("visuals", &UrdfLink::visuals, "")
        .def_readwrite("collisions", &UrdfLink::collisions, "")
        .def_readwrite("cameras", &UrdfLink::cameras, "")
        .def_readwrite("lidars", &UrdfLink::lidars, "")
        .def(py::init<>());

    py::class_<UrdfJoint>(m, "UrdfJoint", "")
        .def_readwrite("name", &UrdfJoint::name, "")
        .def_readwrite("type", &UrdfJoint::type, "")
        .def_readwrite("origin", &UrdfJoint::origin, "")
        .def_readwrite("parent_link_name", &UrdfJoint::parentLinkName, "")
        .def_readwrite("child_link_name", &UrdfJoint::childLinkName, "")
        .def_readwrite("axis", &UrdfJoint::axis, "")
        .def_readwrite("dynamics", &UrdfJoint::dynamics, "")
        .def_readwrite("limit", &UrdfJoint::limit, "")
        .def_readwrite("drive", &UrdfJoint::drive, "")
        .def_readwrite("mimic", &UrdfJoint::mimic, "")
        .def_readwrite("parent_joint", &UrdfJoint::parentJoint, "")
        .def_readwrite("children_joints", &UrdfJoint::childrenJoints, "")
        .def_readwrite("inertia", &UrdfJoint::jointInertia, "")
        .def(py::init<>());

    py::class_<UrdfRobot>(m, "UrdfRobot", "")
        .def_readwrite("name", &UrdfRobot::name, "")
        .def_readwrite("root_link", &UrdfRobot::rootLink, "")
        .def_readwrite("urdf_path", &UrdfRobot::urdfPath, "")
        .def_readwrite("asset_root", &UrdfRobot::assetRoot, "")
        .def_readwrite("links", &UrdfRobot::links, "")
        .def_readwrite("joints", &UrdfRobot::joints, "")
        .def_readwrite("materials", &UrdfRobot::materials, "")
        .def(py::init<>());

    declare_map<UrdfLink>(m, std::string("UrdfLinkMap"));
    declare_map<UrdfJoint>(m, std::string("UrdfJointMap"));
    declare_map<UrdfMaterial>(m, std::string("UrdfMaterialMap"));


    defineInterfaceClass<Urdf>(m, "Urdf", "acquire_urdf_interface", "release_urdf_interface")
        .def("parse_string_urdf", wrapInterfaceFunction(&Urdf::parseUrdfString),
             R"pbdoc(
                Parse URDF file into the internal data structure, which is displayed in the importer window for inspection.

                Args:
                    arg0 (:obj:`str`): The urdf in text format

                    arg2 (:obj:`isaacsim.asset.importer.urdf._urdf.ImportConfig`): Import configuration parameters

                Returns:
                    :obj:`isaacsim.asset.importer.urdf._urdf.UrdfRobot`: Parsed URDF stored in an internal structure.

                )pbdoc")
        .def("parse_urdf", wrapInterfaceFunction(&Urdf::parseUrdf),
             R"pbdoc(
                Parse URDF file into the internal data structure, which is displayed in the importer window for inspection.

                Args:
                    arg0 (:obj:`str`): The absolute path to where the urdf file is

                    arg1 (:obj:`str`): The name of the urdf file

                    arg2 (:obj:`isaacsim.asset.importer.urdf._urdf.ImportConfig`): Import configuration parameters

                Returns:
                    :obj:`isaacsim.asset.importer.urdf._urdf.UrdfRobot`: Parsed URDF stored in an internal structure.

                )pbdoc")

        .def("import_robot", wrapInterfaceFunction(&Urdf::importRobot), py::arg("assetRoot"), py::arg("assetName"),
             py::arg("robot"), py::arg("importConfig"), py::arg("stage") = std::string(""),
             py::arg("getArticulationRoot") = false,
             R"pbdoc(
                Importing the robot, from the already parsed URDF file.

                Args:
                    arg0 (:obj:`str`): The absolute path to where the urdf file is

                    arg1 (:obj:`str`): The name of the urdf file

                    arg2 (:obj:`isaacsim.asset.importer.urdf._urdf.UrdfRobot`): The parsed URDF file, the output from :obj:`parse_urdf`

                    arg3 (:obj:`isaacsim.asset.importer.urdf._urdf.ImportConfig`): Import configuration parameters

                    arg4 (:obj:`str`): optional: path to stage to use for importing. leaving it empty will import on open stage. If the open stage is a new stage, textures will not load.

                Returns:
                    :obj:`str`: Path to the robot on the USD stage.

                )pbdoc")

        .def("get_kinematic_chain", wrapInterfaceFunction(&Urdf::getKinematicChain),
             R"pbdoc(
                Get the kinematic chain of the robot. Mostly used for graphic display of the kinematic tree.

                Args:
                    arg0 (:obj:`isaacsim.asset.importer.urdf._urdf.UrdfRobot`): The parsed URDF, the output from :obj:`parse_urdf`

                Returns:
                    :obj:`dict`: A dictionary with information regarding the parent-child relationship between all the links and joints

                )pbdoc")
        .def("compute_natural_stiffness", wrapInterfaceFunction(&Urdf::computeJointNaturalStiffess),
             R"pbdoc(
                Compute the natural stiffness of a joint.

                Args:
                    arg0 (:obj:`isaacsim.asset.importer.urdf._urdf.UrdfRobot`): The parsed URDF, the output from :obj:`parse_urdf`

                    arg1 (:obj:`str`): The name of the joint

                    arg2 (:obj:`float`): The natural frequency of the joint

                Returns:
                    :obj:`float`: The natural stiffness of the joint

                )pbdoc");
}
}
