// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/asset/importer/mjcf/IMjcf.h>

CARB_BINDINGS("isaacsim.asset.importer.mjcf.python")

namespace isaacsim
{
namespace asset
{
namespace importer
{
namespace mjcf
{
}
} // namespace importer
} // namespace omni
}
namespace
{
PYBIND11_MODULE(_mjcf, m)
{
    using namespace carb;
    using namespace isaacsim::asset::importer::mjcf;

    m.doc() = R"pbdoc(
        This extension provides an interface to the MJCF importer.

        Example:
            Setup the configuration parameters before importing.
            Files must be parsed before imported.

            .. code-block:: python

                >>> import omni
                >>> import os
                >>> from isaacsim.asset.importer.mjcf import _mjcf
                >>> mjcf_interface = _mjcf.acquire_mjcf_interface()
                >>>
                >>> # setup config params
                >>> import_config = _mjcf.ImportConfig()
                >>> import_config.fix_base = True
                >>>
                >>> # parse and import file

                >>> ext_manager = omni.kit.app.get_app().get_extension_manager()
                >>> ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.mjcf")
                >>> extension_path = ext_manager.get_extension_path(ext_id)
                >>> mjcf_path = os.path.abspath(extension_path + "/data/mjcf/nv_ant.xml")
                >>> mjcf_interface.create_asset_mjcf(mjcf_path, "", import_config)


        Refer to the sample documentation for more examples and usage
                )pbdoc";

    py::class_<ImportConfig>(m, "ImportConfig")
        .def(py::init<>())
        .def_readwrite("merge_fixed_joints", &ImportConfig::mergeFixedJoints,
                       "Consolidating links that are connected by fixed joints")
        .def_readwrite("convex_decomp", &ImportConfig::convexDecomp,
                       "Decompose a convex mesh into smaller pieces for a closer fit")
        .def_readwrite("import_inertia_tensor", &ImportConfig::importInertiaTensor,
                       "Import inertia tensor from mjcf, if not specified in "
                       "mjcf it will import as identity")
        .def_readwrite("fix_base", &ImportConfig::fixBase, "Create fix joint for base link")
        .def_readwrite("self_collision", &ImportConfig::selfCollision, "Self collisions between links in the articulation")
        .def_readwrite("density", &ImportConfig::density, "default density used for links")
        //.def_readwrite("default_drive_type", &ImportConfig::defaultDriveType,
        //"default drive type used for joints")
        .def_readwrite(
            "default_drive_strength", &ImportConfig::defaultDriveStrength, "default drive stiffness used for joints")
        .def_readwrite("distance_scale", &ImportConfig::distanceScale,
                       "Set the unit scaling factor, 1.0 means meters, 100.0 means cm")
        //.def_readwrite("up_vector", &ImportConfig::upVector, "Up vector used for
        // import")
        .def_readwrite(
            "create_physics_scene", &ImportConfig::createPhysicsScene, "add a physics scene to the stage on import")
        .def_readwrite("make_default_prim", &ImportConfig::makeDefaultPrim, "set imported robot as default prim")
        .def_readwrite(
            "create_body_for_fixed_joint", &ImportConfig::createBodyForFixedJoint, "creates body for fixed joint")
        .def_readwrite("override_com", &ImportConfig::overrideCoM,
                       "whether to compute the center of mass from geometry and "
                       "override values given in the original asset")
        .def_readwrite("override_inertia_tensor", &ImportConfig::overrideInertia,
                       "Whether to compute the inertia tensor from geometry and "
                       "override values given in the original asset")
        .def_readwrite("make_instanceable", &ImportConfig::makeInstanceable,
                       "Creates an instanceable version of the asset. All meshes "
                       "will be placed in a separate USD file")
        .def_readwrite(
            "instanceable_usd_path", &ImportConfig::instanceableMeshUsdPath, "USD file to store instanceable mehses in")

        // setters for each property
        .def("set_merge_fixed_joints", [](ImportConfig& config, const bool value) { config.mergeFixedJoints = value; })
        .def("set_convex_decomp", [](ImportConfig& config, const bool value) { config.convexDecomp = value; })
        .def("set_import_inertia_tensor",
             [](ImportConfig& config, const bool value) { config.importInertiaTensor = value; })
        .def("set_fix_base", [](ImportConfig& config, const bool value) { config.fixBase = value; })
        .def("set_self_collision", [](ImportConfig& config, const bool value) { config.selfCollision = value; })
        .def("set_density", [](ImportConfig& config, const float value) { config.density = value; })
        /*        .def("set_default_drive_type",
                     [](ImportConfig& config, const int value) {
                         config.defaultDriveType =
           static_cast<UrdfJointTargetType>(value);
                     })*/
        .def("set_default_drive_strength",
             [](ImportConfig& config, const float value) { config.defaultDriveStrength = value; })
        .def("set_distance_scale", [](ImportConfig& config, const float value) { config.distanceScale = value; })
        /*        .def("set_up_vector",
                     [](ImportConfig& config, const float x, const float y, const
           float z) { config.upVector = { x, y, z };
                     })*/
        .def("set_create_physics_scene",
             [](ImportConfig& config, const bool value) { config.createPhysicsScene = value; })
        .def("set_make_default_prim", [](ImportConfig& config, const bool value) { config.makeDefaultPrim = value; })
        .def("set_create_body_for_fixed_joint",
             [](ImportConfig& config, const bool value) { config.createBodyForFixedJoint = value; })
        .def("set_override_com", [](ImportConfig& config, const bool value) { config.overrideCoM = value; })
        .def("set_override_inertia", [](ImportConfig& config, const bool value) { config.overrideInertia = value; })
        .def("set_make_instanceable", [](ImportConfig& config, const bool value) { config.makeInstanceable = value; })
        .def("set_instanceable_usd_path",
             [](ImportConfig& config, const std::string value) { config.instanceableMeshUsdPath = value; })
        .def("set_visualize_collision_geoms",
             [](ImportConfig& config, const bool value) { config.visualizeCollisionGeoms = value; })
        .def("set_import_sites", [](ImportConfig& config, const bool value) { config.importSites = value; });

    defineInterfaceClass<Mjcf>(m, "Mjcf", "acquire_mjcf_interface", "release_mjcf_interface")
        .def("create_asset_mjcf", wrapInterfaceFunction(&Mjcf::createAssetFromMJCF), py::arg("fileName"),
             py::arg("primName"), py::arg("config"), py::arg("stage_identifier") = std::string(""),
             R"pbdoc(
                Parse and import MJCF file.

                Args:
                    arg0 (:obj:`str`): The absolute path to the mjcf

                    arg1 (:obj:`str`): Path to the robot on the USD stage

                    arg2 (:obj:`isaacsim.asset.importer.mjcf._mjcf.ImportConfig`): Import configuration

                    arg3 (:obj:`str`): optional: path to stage to use for importing. leaving it empty will import on open stage. If the open stage is a new stage, textures will not load.

                )pbdoc");
}
} // namespace
