-- SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
-- http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- Function to define test startup experience with app name, kit file, and optional extra arguments
function define_test_startup_experience(app_name, kit_file, extra_args)
    local script_dir_token = (os.target() == "windows") and "%~dp0" or "$SCRIPT_DIR"
    local extra_args = extra_args or ""
    local kit_file = kit_file or app_name
    define_test_experience(app_name, {
        config_path = "../apps/" .. kit_file .. ".kit",
        extra_args = '--ext-folder "' .. script_dir_token .. '/../exts" ' ..
                     '--ext-folder "' .. script_dir_token .. '/../extscache" ' ..
                     '--ext-folder "' .. script_dir_token .. '/../extsDeprecated" ' ..
                     '--ext-folder "' .. script_dir_token .. '/../apps" ' .. extra_args,
    })
end

-- Helper function to define a group of related tests
function define_test_group(group_name, tests)
    group(group_name)
    for _, test in ipairs(tests) do
        define_test_startup_experience(test.name, test.kit_file, test.extra_args)
    end
end

-- Helper to register a list of python sample tests
-- Test format: { name, script_path, args, pythonpath_dirs, env_vars }
-- pythonpath_dirs is optional and can be a table of paths to add to PYTHONPATH
-- env_vars is optional and can be a table of environment variable key-value pairs
local function register_python_sample_tests(tests)
    for _, test in ipairs(tests) do
        local pythonpath_dirs = test[4] or {}
        local env_vars = test[5] or {}
        python_sample_test(test[1], test[2], test[3], pythonpath_dirs, env_vars)
    end
end

local function get_startup_tests()
    return {
        {
            name = "tests-startup.main",
            kit_file = "isaacsim.exp.full",
            extra_args = "--/app/quitAfter=1000 --/app/file/ignoreUnsavedStage=1",
        },
        {
            name = "tests-startup.streaming",
            kit_file = "isaacsim.exp.full.streaming",
            extra_args = "--no-window --/app/quitAfter=1000 --/app/file/ignoreUnsavedStage=1",
        },
        {
            name = "tests-startup.extscache",
            kit_file = "isaacsim.exp.full",
            extra_args = "--no-window --/app/quitAfter=1000 --/app/extensions/registryEnabled=0 --/app/file/ignoreUnsavedStage=1",
        },
        {
            name = "tests-startup.xr.vr",
            kit_file = "isaacsim.exp.base.xr.vr",
            extra_args = "--no-window --/app/quitAfter=1000 --/app/file/ignoreUnsavedStage=1",
        },
    }
end

local function get_simulation_app_tests()
    return {
        {
            "tests-nativepython-isaacsim.simulation_app.hello_world",
            "standalone_examples/api/isaacsim.simulation_app/hello_world.py",
        },
        {
            "tests-nativepython-isaacsim.simulation_app.change_resolution",
            "standalone_examples/api/isaacsim.simulation_app/change_resolution.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.simulation_app.load_stage",
            "standalone_examples/api/isaacsim.simulation_app/load_stage.py",
            "--usd_path /Isaac/Environments/Simple_Room/simple_room.usd --test --headless",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_config",
            "standalone_examples/testing/isaacsim.simulation_app/test_config.py",
            "--/persistent/isaac/asset_root/default=omniverse://ov-test-this-is-working",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_multiprocess",
            "standalone_examples/testing/isaacsim.simulation_app/test_multiprocess.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_viewport_ready",
            "standalone_examples/testing/isaacsim.simulation_app/test_viewport_ready.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_headless_no_rendering",
            "standalone_examples/testing/isaacsim.simulation_app/test_headless_no_rendering.py",
        },
        -- Additional simulation app tests
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_frame_delay_basic",
            "standalone_examples/testing/isaacsim.simulation_app/test_frame_delay.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_fabric_frame_delay",
            "standalone_examples/testing/isaacsim.simulation_app/test_fabric_frame_delay.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_ogn",
            "standalone_examples/testing/isaacsim.simulation_app/test_ogn.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_syntheticdata",
            "standalone_examples/testing/isaacsim.simulation_app/test_syntheticdata.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_fetch_results",
            "standalone_examples/testing/isaacsim.simulation_app/test_fetch_results.py",
        },
        {
            "tests-nativepython-testing-isaacsim.benchmark.services.test_no_rendering",
            "standalone_examples/testing/isaacsim.benchmark.services/test_no_rendering.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_test_runner",
            "standalone_examples/testing/isaacsim.simulation_app/test_test_runner.py",
        },
        -- Additional simulation_app standalone scripts
        {
            "tests-nativepython-isaacsim.simulation_app.constant_fps",
            "standalone_examples/api/isaacsim.simulation_app/constant_fps.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.simulation_app.livestream",
            "standalone_examples/api/isaacsim.simulation_app/livestream.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.simulation_app.async_call",
            "standalone_examples/api/isaacsim.simulation_app/async_call.py",
        },
    }
end

local function get_core_tests()
    return {
        -- Cloner
        {
            "tests-nativepython-isaacsim.core.cloner.clone_ants",
            "standalone_examples/api/isaacsim.core.cloner/clone_ants.py",
        },
        -- Core Experimental API
        {
            "tests-nativepython-isaacsim.core.experimental.api.deformable_stress_visualization",
            "standalone_examples/api/isaacsim.core.experimental.api/deformable_stress_visualization.py",
            "--test",
        },
        -- Deprecated Core API
        {
            "tests-nativepython-deprecated-isaacsim.core.api.add_cubes",
            "standalone_examples/deprecated/api/isaacsim.core.api/add_cubes.py",
        },
        {
            "tests-nativepython-deprecated-isaacsim.core.api.add_frankas",
            "standalone_examples/deprecated/api/isaacsim.core.api/add_frankas.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.core.api.data_logging",
            "standalone_examples/deprecated/api/isaacsim.core.api/data_logging.py",
        },
        {
            "tests-nativepython-deprecated-isaacsim.core.api.control_robot",
            "standalone_examples/deprecated/api/isaacsim.core.api/control_robot.py",
        },
        {
            "tests-nativepython-deprecated-isaacsim.core.api.simulate_robot",
            "standalone_examples/deprecated/api/isaacsim.core.api/simulate_robot.py",
        },
        {
            "tests-nativepython-deprecated-isaacsim.core.api.simulation_callbacks",
            "standalone_examples/deprecated/api/isaacsim.core.api/simulation_callbacks.py",
        },
        {
            "tests-nativepython-deprecated-isaacsim.core.api.time_stepping",
            "standalone_examples/deprecated/api/isaacsim.core.api/time_stepping.py",
        },
        {
            "tests-nativepython-deprecated-isaacsim.core.api.visual_materials",
            "standalone_examples/deprecated/api/isaacsim.core.api/visual_materials.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.core.api.omnigraph_triggers",
            "standalone_examples/deprecated/api/isaacsim.core.api/omnigraph_triggers.py",
        },
        {
            "tests-nativepython-deprecated-isaacsim.core.api.rigid_contact_view",
            "standalone_examples/deprecated/api/isaacsim.core.api/rigid_contact_view.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.core.api.detailed_contact_data",
            "standalone_examples/deprecated/api/isaacsim.core.api/detailed_contact_data.py",
            "--test",
        },
        -- Core Experimental API (additional)
        {
            "tests-nativepython-isaacsim.core.experimental.api.add_cubes",
            "standalone_examples/api/isaacsim.core.experimental.api/add_cubes.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.api.control_frankas",
            "standalone_examples/api/isaacsim.core.experimental.api/control_frankas.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.api.omnigraph_triggers",
            "standalone_examples/api/isaacsim.core.experimental.api/omnigraph_triggers.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.api.simulation_callbacks",
            "standalone_examples/api/isaacsim.core.experimental.api/simulation_callbacks.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.api.visual_materials",
            "standalone_examples/api/isaacsim.core.experimental.api/visual_materials.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.api.control_robot_numpy",
            "standalone_examples/api/isaacsim.core.experimental.api/control_robot_numpy.py",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.api.control_robot_torch",
            "standalone_examples/api/isaacsim.core.experimental.api/control_robot_torch.py",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.api.control_robot_warp",
            "standalone_examples/api/isaacsim.core.experimental.api/control_robot_warp.py",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.api.control_robot_jax",
            "standalone_examples/api/isaacsim.core.experimental.api/control_robot_jax.py",
        },
        -- From Misc
        {
            "tests-nativepython-testing-omni.syntheticdata.test_basic",
            "standalone_examples/testing/omni.syntheticdata/test_basic.py",
        },
        -- Cortex
        {
            "tests-nativepython-testing-isaacsim.cortex.framework.bringup",
            "standalone_examples/testing/isaacsim.cortex.framework/cortex_bringup_test.py",
        },
    }
end

local function get_sensor_tests()
    local tests = {
        -- RTX Sensors (Experimental)
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.create_camera_basic",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/create_camera_basic.py",
            "--test --disable-output",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.create_camera_depth_sensor",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/create_camera_depth_sensor.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.camera_opencv_pinhole",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/camera_opencv_pinhole.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.camera_opencv_fisheye",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/camera_opencv_fisheye.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.camera_stereoscopic_depth",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/camera_stereoscopic_depth.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.camera_tiled",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/camera_tiled.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.camera_annotator_devices",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/camera_annotator_devices.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.camera_ros",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/camera_ros.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.create_lidar_basic",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/create_lidar_basic.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.create_lidar_with_config_and_variants",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/create_lidar_with_config_and_variants.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.inspect_lidar_gmo",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/inspect_lidar_gmo.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.resolve_lidar_object_ids",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/resolve_lidar_object_ids.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.lidar_robot_integration",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/lidar_robot_integration.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.create_radar_basic",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/create_radar_basic.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.inspect_radar_gmo",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/inspect_radar_gmo.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.create_acoustic_basic",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/create_acoustic_basic.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.inspect_acoustic_gmo",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/inspect_acoustic_gmo.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.apply_nonvisual_materials",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/apply_nonvisual_materials.py",
            "--test",
        },
        -- From Misc Physics
        {
            "tests-nativepython-testing-isaacsim.sensors.physics.contact_sensor",
            "standalone_examples/testing/isaacsim.sensors.physics/contact_sensor_test.py",
        },
        -- Experimental Physics Sensors
        {
            "tests-nativepython-isaacsim.sensors.experimental.physics.contact_sensor",
            "standalone_examples/api/isaacsim.sensors.experimental.physics/contact_sensor.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.physics.effort_sensor",
            "standalone_examples/api/isaacsim.sensors.experimental.physics/effort_sensor.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.experimental.physics.imu_sensor",
            "standalone_examples/api/isaacsim.sensors.experimental.physics/imu_sensor.py",
            "--test",
        },
        -- Deprecated Physics Sensors (using extra_args for extension loading)
        {
            "tests-nativepython-deprecated-isaacsim.sensors.physics.contact_sensor",
            "standalone_examples/deprecated/api/isaacsim.sensors.physics/contact_sensor.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.physics.effort_sensor",
            "standalone_examples/deprecated/api/isaacsim.sensors.physics/effort_sensor.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.physics.imu_sensor",
            "standalone_examples/deprecated/api/isaacsim.sensors.physics/imu_sensor.py",
            "--test",
        },
        -- Deprecated RTX Sensors
        {
            "tests-nativepython-deprecated-isaacsim.sensors.rtx.create_lidar_basic",
            "standalone_examples/deprecated/api/isaacsim.sensors.rtx/create_lidar_basic.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.rtx.create_radar_basic",
            "standalone_examples/deprecated/api/isaacsim.sensors.rtx/create_radar_basic.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.rtx.create_lidar_with_config_and_variants",
            "standalone_examples/deprecated/api/isaacsim.sensors.rtx/create_lidar_with_config_and_variants.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.rtx.inspect_lidar_gmo",
            "standalone_examples/deprecated/api/isaacsim.sensors.rtx/inspect_lidar_gmo.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.rtx.inspect_radar_gmo",
            "standalone_examples/deprecated/api/isaacsim.sensors.rtx/inspect_radar_gmo.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.rtx.resolve_lidar_object_ids",
            "standalone_examples/deprecated/api/isaacsim.sensors.rtx/resolve_lidar_object_ids.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.rtx.lidar_robot_integration",
            "standalone_examples/deprecated/api/isaacsim.sensors.rtx/lidar_robot_integration.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.rtx.apply_nonvisual_materials",
            "standalone_examples/deprecated/api/isaacsim.sensors.rtx/apply_nonvisual_materials.py",
            "--test",
        },
        -- Deprecated PhysX Sensor
        {
            "tests-nativepython-deprecated-isaacsim.sensors.physx.rotating_lidar_physX",
            "standalone_examples/deprecated/api/isaacsim.sensors.physx/rotating_lidar_physX.py",
            "--test",
        },
        -- Deprecated Camera Sensors
        {
            "tests-nativepython-deprecated-isaacsim.sensors.camera.camera",
            "standalone_examples/deprecated/api/isaacsim.sensors.camera/camera.py",
            "--test --disable-output",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.camera.camera_stereoscopic_depth",
            "standalone_examples/deprecated/api/isaacsim.sensors.camera/camera_stereoscopic_depth.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.camera.camera_add_depth_sensor",
            "standalone_examples/deprecated/api/isaacsim.sensors.camera/camera_add_depth_sensor.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.camera.camera_annotator_device",
            "standalone_examples/deprecated/api/isaacsim.sensors.camera/camera_annotator_device.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.camera.camera_opencv_fisheye",
            "standalone_examples/deprecated/api/isaacsim.sensors.camera/camera_opencv_fisheye.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.camera.camera_opencv_pinhole",
            "standalone_examples/deprecated/api/isaacsim.sensors.camera/camera_opencv_pinhole.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.camera.camera_pre_isp_pipeline",
            "standalone_examples/deprecated/api/isaacsim.sensors.camera/camera_pre_isp_pipeline.py",
            "--draw-output --test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.camera.camera_ros",
            "standalone_examples/deprecated/api/isaacsim.sensors.camera/camera_ros.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.sensors.camera.camera_view",
            "standalone_examples/deprecated/api/isaacsim.sensors.camera/camera_view.py",
            "--test",
        },
    }

    local platform_target = _OPTIONS["platform-target"] or "linux-x86_64"
    if os.target() == "linux" and platform_target == "linux-x86_64" then
        table.insert(tests, {
            "tests-nativepython-isaacsim.sensors.experimental.rtx.camera_isp_pipeline",
            "standalone_examples/api/isaacsim.sensors.experimental.rtx/camera_isp_pipeline.py",
            "--test",
        })
    end

    return tests
end

local function get_robot_tests()
    return {
        -- Manipulators (experimental): one --test per standalone; extra rows exercise non-default CLI (SVD IK on pick_place and UR10 IK;
        -- --with-obstacle on both RmpFlow follow-target scripts).
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.franka.pick_place",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/franka/pick_place.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.franka.pick_place.ik_singular_value_decomposition",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/franka/pick_place.py",
            "--test --ik-method singular-value-decomposition",
        },
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.franka.stacking",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/franka/stacking.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.franka.multiple_tasks",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/franka/multiple_tasks.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.franka.follow_target_with_rmpflow",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/franka/follow_target_with_rmpflow.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.franka.follow_target_with_rmpflow.with_obstacle",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/franka/follow_target_with_rmpflow.py",
            "--test --with-obstacle",
        },
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.ur10.follow_target_with_ik",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/universal_robots/follow_target_with_ik.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.ur10.follow_target_with_ik.ik_singular_value_decomposition",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/universal_robots/follow_target_with_ik.py",
            "--test --ik-method singular-value-decomposition",
        },
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.ur10.follow_target_with_rmpflow",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/universal_robots/follow_target_with_rmpflow.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.ur10.follow_target_with_rmpflow.with_obstacle",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/universal_robots/follow_target_with_rmpflow.py",
            "--test --with-obstacle",
        },
        {
            "tests-nativepython-isaacsim.robot.experimental.manipulators.ur10.stacking",
            "standalone_examples/api/isaacsim.robot.experimental.manipulators/universal_robots/stacking.py",
            "--test",
        },
        -- Wheeled Robots
        {
            "tests-nativepython-isaacsim.robot.wheeled_robots.examples.jetbot_differential_move",
            "standalone_examples/api/isaacsim.robot.wheeled_robots.examples/jetbot_differential_move.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.wheeled_robots.examples.kaya_holonomic_move",
            "standalone_examples/api/isaacsim.robot.wheeled_robots.examples/kaya_holonomic_move.py",
            "--test",
        },
        -- Robot Policy Examples
        {
            "tests-nativepython-isaacsim.robot.policy.examples.anymal_standalone",
            "standalone_examples/api/isaacsim.robot.policy.examples/anymal_standalone.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.policy.examples.h1_standalone",
            "standalone_examples/api/isaacsim.robot.policy.examples/h1_standalone.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.policy.examples.spot_standalone",
            "standalone_examples/api/isaacsim.robot.policy.examples/spot_standalone.py",
            "--test",
        },
        -- Deprecated Manipulators (Franka)
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.franka.follow_target_with_ik",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/franka/follow_target_with_ik.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.franka.follow_target_with_rmpflow",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/franka/follow_target_with_rmpflow.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.franka.franka_gripper",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/franka/franka_gripper.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.franka.multiple_tasks",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/franka/multiple_tasks.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.franka.pick_place",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/franka/pick_place.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.franka.stacking",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/franka/stacking.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.franka_pick_up",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/franka_pick_up.py",
            "--test",
        },
        -- Deprecated Manipulators (Universal Robots)
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur.follow_target_with_ik",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/universal_robots/follow_target_with_ik.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur.follow_target_with_ik_experimental",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/universal_robots/follow_target_with_ik_experimental.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur.follow_target_with_rmpflow",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/universal_robots/follow_target_with_rmpflow.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur.multiple_tasks",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/universal_robots/multiple_tasks.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur.pick_place",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/universal_robots/pick_place.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur.pick_place2",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/universal_robots/pick_place2.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur.stacking",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/universal_robots/stacking.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur.bin_filling",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/universal_robots/bin_filling.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur10_pick_up",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/ur10_pick_up.py",
            "--test",
        },
        -- Deprecated Manipulators (UR10e)
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur10e.follow_target_example",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/ur10e/follow_target_example.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur10e.follow_target_example_rmpflow",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/ur10e/follow_target_example_rmpflow.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur10e.pick_up_example",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/ur10e/pick_up_example.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.ur10e.gripper_control",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/ur10e/gripper_control.py",
            "--test",
        },
        -- Deprecated Manipulators (Cobotta)
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.cobotta_900.follow_target_example",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/cobotta_900/follow_target_example.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.cobotta_900.gripper_control",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/cobotta_900/gripper_control.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.cobotta_900.pick_up_example",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/cobotta_900/pick_up_example.py",
            "--test",
        },
        -- Deprecated Manipulators (RMPFlow supported robots)
        {
            "tests-nativepython-deprecated-isaacsim.robot.manipulators.rmpflow.supported_robot_follow_target",
            "standalone_examples/deprecated/api/isaacsim.robot.manipulators/rmpflow_supported_robots/supported_robot_follow_target_example.py",
            "--test",
        },
        -- Deprecated Cortex Framework
        {
            "tests-nativepython-deprecated-isaacsim.cortex.framework.demo_ur10_conveyor",
            "standalone_examples/deprecated/api/isaacsim.cortex.framework/demo_ur10_conveyor_main.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.cortex.framework.example_command_api",
            "standalone_examples/deprecated/api/isaacsim.cortex.framework/example_command_api_main.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.cortex.framework.follow_example",
            "standalone_examples/deprecated/api/isaacsim.cortex.framework/follow_example_main.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.cortex.framework.follow_example_modified",
            "standalone_examples/deprecated/api/isaacsim.cortex.framework/follow_example_modified_main.py",
            "--test",
        },
        {
            "tests-nativepython-deprecated-isaacsim.cortex.framework.franka_examples",
            "standalone_examples/deprecated/api/isaacsim.cortex.framework/franka_examples_main.py",
            "--test",
        },
        -- Motion Generation (Experimental)
        {
            "tests-nativepython-isaacsim.robot_motion.experimental.motion_generation.mobile_robot_control",
            "standalone_examples/api/isaacsim.robot_motion.experimental.motion_generation/mobile_robot_control_example.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot_motion.experimental.motion_generation.mobile_robot_control.noise",
            "standalone_examples/api/isaacsim.robot_motion.experimental.motion_generation/mobile_robot_control_example.py",
            "--noise --test",
        },
        {
            "tests-nativepython-isaacsim.robot_motion.experimental.motion_generation.mobile_robot_control.filter",
            "standalone_examples/api/isaacsim.robot_motion.experimental.motion_generation/mobile_robot_control_example.py",
            "--filter --test",
        },
        {
            "tests-nativepython-isaacsim.robot_motion.experimental.motion_generation.mobile_robot_control.filter_and_noise",
            "standalone_examples/api/isaacsim.robot_motion.experimental.motion_generation/mobile_robot_control_example.py",
            "--filter --noise --test",
        },
        {
            "tests-nativepython-isaacsim.robot_motion.experimental.motion_generation.trajectory",
            "standalone_examples/api/isaacsim.robot_motion.experimental.motion_generation/trajectory_example.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot_motion.experimental.motion_generation.trajectory.linear",
            "standalone_examples/api/isaacsim.robot_motion.experimental.motion_generation/trajectory_example.py",
            "--linear --test",
        },
        {
            "tests-nativepython-isaacsim.robot_motion.experimental.motion_generation.scene_interaction",
            "standalone_examples/api/isaacsim.robot_motion.experimental.motion_generation/scene_interaction_example.py",
        },
        -- Newton Actuators (Experimental)
        {
            "tests-nativepython-isaacsim.core.experimental.actuators.newton_actuators_python",
            "standalone_examples/api/isaacsim.core.experimental.actuators/newton_actuators_python_example.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.actuators.newton_actuators_python.non_ideal",
            "standalone_examples/api/isaacsim.core.experimental.actuators/newton_actuators_python_example.py",
            "--non-ideal --test",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.actuators.newton_actuators_usd",
            "standalone_examples/api/isaacsim.core.experimental.actuators/newton_actuators_usd_example.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.experimental.actuators.newton_actuators_omnigraph",
            "standalone_examples/api/isaacsim.core.experimental.actuators/newton_actuators_omnigraph_example.py",
            "--test",
        },
    }
end

local function get_asset_tests()
    return {
        -- URDF
        {
            "tests-nativepython-isaacsim.asset.importer.urdf.urdf_import",
            "standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py",
            "--test",
        },
        -- URDF exporter
        {
            "tests-nativepython-isaacsim.asset.exporter.urdf.urdf_export",
            "standalone_examples/api/isaacsim.asset.exporter.urdf/urdf_export.py",
            "--test",
        },
        -- MJCF
        {
            "tests-nativepython-isaacsim.asset.importer.mjcf.mjcf_import",
            "standalone_examples/api/isaacsim.asset.importer.mjcf/mjcf_import.py",
            "--test --usd-path standalone_examples/api/isaacsim.asset.importer.mjcf/nv_humanoid",
        },
        -- Asset Transformer
        {
            "tests-nativepython-isaacsim.asset.transformer.run_asset_transformer",
            "standalone_examples/api/isaacsim.asset.transformer/run_asset_transformer.py",
            "--test",
        },
        -- Asset USD Converter
        {
            "tests-nativepython-omni.kit.asset_converter.asset_usd_converter",
            "standalone_examples/api/omni.kit.asset_converter/asset_usd_converter.py",
            "--folders standalone_examples/data/cube standalone_examples/data/torus",
        },
    }
end

local function get_replicator_tests()
    local tests = {
        {
            "tests-nativepython-replicator.infinigen_sdg_default",
            "standalone_examples/replicator/infinigen/infinigen_sdg.py",
            "--close-on-completion",
        },
        {
            "tests-nativepython-replicator.infinigen_sdg_config",
            "standalone_examples/replicator/infinigen/infinigen_sdg.py",
            [[--close-on-completion --config "$SAMPLE_DIR/standalone_examples/replicator/infinigen/config/infinigen_multi_writers_pt.yaml"]],
        },
        {
            "tests-nativepython-replicator.scene_based_sdg",
            "standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py",
        },
        {
            "tests-nativepython-replicator.scene_based_sdg_basic_writer",
            "standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py",
            [[--config "$SAMPLE_DIR/standalone_examples/replicator/scene_based_sdg/config/config_basic_writer.yaml"]],
        },
        {
            "tests-nativepython-replicator.scene_based_sdg_default_writer",
            "standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py",
            [[--config "$SAMPLE_DIR/standalone_examples/replicator/scene_based_sdg/config/config_default_writer.json"]],
        },
        {
            "tests-nativepython-replicator.scene_based_sdg_kitti_writer",
            "standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py",
            [[--config "$SAMPLE_DIR/standalone_examples/replicator/scene_based_sdg/config/config_kitti_writer.yaml"]],
        },
        {
            "tests-nativepython-replicator.scene_based_sdg_coco_writer",
            "standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py",
            [[--config "$SAMPLE_DIR/standalone_examples/replicator/scene_based_sdg/config/config_coco_writer.yaml"]],
        },
        {
            "tests-nativepython-replicator.object_based_sdg",
            "standalone_examples/replicator/object_based_sdg/object_based_sdg.py",
        },
        {
            "tests-nativepython-replicator.object_based_sdg_config",
            "standalone_examples/replicator/object_based_sdg/object_based_sdg.py",
            [[--config "$SAMPLE_DIR/standalone_examples/replicator/object_based_sdg/config/object_based_sdg_config.yaml"]],
        },
        {
            "tests-nativepython-replicator.object_based_sdg_config_dope",
            "standalone_examples/replicator/object_based_sdg/object_based_sdg.py",
            [[--config "$SAMPLE_DIR/standalone_examples/replicator/object_based_sdg/config/object_based_sdg_dope_config.yaml"]],
        },
        {
            "tests-nativepython-replicator.object_based_sdg_config_centerpose",
            "standalone_examples/replicator/object_based_sdg/object_based_sdg.py",
            [[--config "$SAMPLE_DIR/standalone_examples/replicator/object_based_sdg/config/object_based_sdg_centerpose_config.yaml"]],
        },
        {
            "tests-nativepython-replicator.writer_augmentation_numpy",
            "standalone_examples/replicator/augmentation/writer_augmentation.py",
            "--num_frames 1",
        },
        {
            "tests-nativepython-replicator.writer_augmentation_warp",
            "standalone_examples/replicator/augmentation/writer_augmentation.py",
            "--num_frames 1 --use_warp",
        },
        {
            "tests-nativepython-replicator.annotator_augmentation_numpy",
            "standalone_examples/replicator/augmentation/annotator_augmentation.py",
            "--num_frames 1",
        },
        {
            "tests-nativepython-replicator.annotator_augmentation_warp",
            "standalone_examples/replicator/augmentation/annotator_augmentation.py",
            "--num_frames 1 --use_warp",
        },
        {
            "tests-nativepython-replicator.amr_navigation",
            "standalone_examples/replicator/amr_navigation.py",
            "--num_frames 3 --env_interval 1 --env_urls None",
        },
        {
            "tests-nativepython-replicator.amr_navigation_use_temp_rp",
            "standalone_examples/replicator/amr_navigation.py",
            "--num_frames 3 --env_interval 1 --use_temp_rp --env_urls None",
        },
        {
            "tests-nativepython-replicator.cosmos_writer_warehouse",
            "standalone_examples/replicator/cosmos_writer_warehouse.py",
        },
        -- From Misc Replicator
        {
            "tests-nativepython-isaacsim.replicator.behavior.behaviors",
            "/standalone_examples/api/isaacsim.replicator.behavior/behaviors.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.cosmos_writer_simple",
            "standalone_examples/api/isaacsim.replicator.examples/cosmos_writer_simple.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_deformables",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_deformables.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_geomsubset_per_subset_true",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_geomsubset.py",
            "--/syntheticdata/sensors/perSubsetSegmentation=true --test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_geomsubset_per_subset_false",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_geomsubset.py",
            "--/syntheticdata/sensors/perSubsetSegmentation=false --test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.custom_event_and_write",
            "/standalone_examples/api/isaacsim.replicator.examples/custom_event_and_write.py",
            "--test",
        },
        {
            "tests-nativepython-testing-isaacsim.replicator.examples.ar_capture_pipeline",
            "/standalone_examples/testing/isaacsim.replicator.examples/ar_capture_pipeline.py",
        },
        {
            "tests-nativepython-testing-isaacsim.replicator.examples.ar_capture_pipeline_gpu",
            "/standalone_examples/testing/isaacsim.replicator.examples/ar_capture_pipeline.py",
            "--gpu_dynamics",
        },
        {
            "tests-nativepython-testing-isaacsim.replicator.examples.motion_blur_short",
            "/standalone_examples/api/isaacsim.replicator.examples/motion_blur.py",
            "--delta_times None 0.00416666666 --samples_per_pixel 32 --motion_blur_subsamples 4 --test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.subscribers_and_events",
            "/standalone_examples/api/isaacsim.replicator.examples/subscribers_and_events.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.custom_fps_writer_annotator",
            "/standalone_examples/api/isaacsim.replicator.examples/custom_fps_writer_annotator.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_getting_started_01",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_01.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_getting_started_02",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_02.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_getting_started_03",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_03.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_getting_started_04",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_04.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_getting_started_05",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_05.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_workflow_01",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_workflow_01.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_workflow_02",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_workflow_02.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.simready_assets_sdg",
            "standalone_examples/api/isaacsim.replicator.examples/simready_assets_sdg.py",
            "--num_scenarios 2 --test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.multi_camera",
            "standalone_examples/api/isaacsim.replicator.examples/multi_camera.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.simulation_get_data",
            "standalone_examples/api/isaacsim.replicator.examples/simulation_get_data.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.grasping.grasping_workflow_sdg",
            "standalone_examples/api/isaacsim.replicator.grasping/grasping_workflow_sdg.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.replicator.experimental.domain_randomization",
            "standalone_examples/api/isaacsim.replicator.experimental.domain_randomization/randomization_demo.py",
            "--env-url none --num-envs 2 --reset-interval 20 --max-frames 50",
        },
        {
            "tests-nativepython-testing-omni.replicator.agent.test_scripting",
            "standalone_examples/testing/omni.replicator.agent/test_scripting.py",
        },
        -- Episode Recorder
        {
            "tests-nativepython-isaacsim.replicator.episode_recorder.episode_record_replay",
            "standalone_examples/api/isaacsim.replicator.episode_recorder/episode_record_replay.py",
            "--replay --test",
        },
        -- Deprecated Domain Randomization
        {
            "tests-nativepython-deprecated-isaacsim.replicator.domain_randomization.randomization_demo",
            "standalone_examples/deprecated/api/isaacsim.replicator.domain_randomization/randomization_demo.py",
            "--test",
        },
    }

    local platform_target = _OPTIONS["platform-target"] or "linux-x86_64"
    if os.target() == "linux" and platform_target == "linux-x86_64" then
        table.insert(tests, {
            "tests-nativepython-isaacsim.replicator.teleop.sdg_teleop_replay",
            "standalone_examples/api/isaacsim.replicator.teleop/sdg_teleop_replay.py",
            "--test",
        })
    end

    return tests
end

local function get_ros_tests()
    return {
        {
            "tests-nativepython-testing-isaacsim.ros2.bridge.enable_extension",
            "standalone_examples/testing/isaacsim.ros2.bridge/enable_extension.py",
        },
        {
            "tests-nativepython-testing-isaacsim.ros2.bridge.test_carter_camera_multi_robot_nav",
            "standalone_examples/testing/isaacsim.ros2.bridge/test_carter_camera_multi_robot_nav.py",
        },
        {
            "tests-nativepython-testing-isaacsim.ros2.bridge.test_camera_tf_delay",
            "standalone_examples/testing/isaacsim.ros2.bridge/test_camera_tf_delay.py",
            "--test-steps=50 --/rtx/hydra/supportMultiTickRate=0 --/rtx/rendering/perSensorTickTlas=0",
        },
        {
            "tests-nativepython-testing-isaacsim.ros2.bridge.test_publish_camera_data",
            "standalone_examples/testing/isaacsim.ros2.bridge/test_publish_camera_data.py",
            "--test-steps=5",
        },
        -- ROS2 Bridge Standalone Scripts
        {
            "tests-nativepython-isaacsim.ros2.bridge.camera_manual",
            "standalone_examples/api/isaacsim.ros2.bridge/camera_manual.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.ros2.bridge.camera_periodic",
            "standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.ros2.bridge.camera_noise",
            "standalone_examples/api/isaacsim.ros2.bridge/camera_noise.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.ros2.bridge.carter_multiple_robot_navigation",
            "standalone_examples/api/isaacsim.ros2.bridge/carter_multiple_robot_navigation.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.ros2.bridge.carter_stereo",
            "standalone_examples/api/isaacsim.ros2.bridge/carter_stereo.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.ros2.bridge.clock",
            "standalone_examples/api/isaacsim.ros2.bridge/clock.py",
        },
        {
            "tests-nativepython-isaacsim.ros2.bridge.moveit",
            "standalone_examples/api/isaacsim.ros2.bridge/moveit.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.ros2.bridge.rtx_lidar",
            "standalone_examples/api/isaacsim.ros2.bridge/rtx_lidar.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.ros2.bridge.rtx_radar",
            "standalone_examples/api/isaacsim.ros2.bridge/rtx_radar.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.ros2.bridge.subscriber",
            "standalone_examples/api/isaacsim.ros2.bridge/subscriber.py",
            "--test",
        },
    }
end

local function get_doc_snippets_tests()
    return {
        -- assets
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.assets.usd_assets_nurec.nurec_carter",
            "../../../docs/isaacsim/snippets/assets/usd_assets_nurec/nurec_carter.py",
            "--test",
        },
        -- core_api_tutorials
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.core_api_tutorials.tutorial_core_hello_world.open_a_new_my_applicationpy_file_and_add_the_follo",
            "../../../docs/isaacsim/snippets/core_api_tutorials/tutorial_core_hello_world/open_a_new_my_applicationpy_file_and_add_the_follo.py",
            "--test",
        },
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.core_api_tutorials.tutorial_core_adding_manipulator.creating_the_scene",
            "../../../docs/isaacsim/snippets/core_api_tutorials/tutorial_core_adding_manipulator/creating_the_scene.py",
            "--test",
        },
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.core_api_tutorials.tutorial_core_adding_manipulator.understanding_the_state_machine",
            "../../../docs/isaacsim/snippets/core_api_tutorials/tutorial_core_adding_manipulator/understanding_the_state_machine.py",
            "--test",
        },
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.core_api_tutorials.tutorial_core_adding_manipulator.using_the_pickandplace_controller",
            "../../../docs/isaacsim/snippets/core_api_tutorials/tutorial_core_adding_manipulator/using_the_pickandplace_controller.py",
            "--test",
        },
        -- cortex_tutorials
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.cortex_tutorials.tutorial_cortex_2_decider_networks.decision_framework_tooling",
            "../../../docs/isaacsim/snippets/cortex_tutorials/tutorial_cortex_2_decider_networks/decision_framework_tooling.py",
            "--test",
        },
        -- development_tools
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.development_tools.jupyter_notebook.configuration_files_2",
            "../../../docs/isaacsim/snippets/development_tools/jupyter_notebook/configuration_files_2.py",
            "--test",
        },
        -- installation
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.installation.install_python.perform_any_isaac_sim_omniverse_imports_after_inst",
            "../../../docs/isaacsim/snippets/installation/install_python/perform_any_isaac_sim_omniverse_imports_after_inst.py",
            "--test",
        },
        -- introduction
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.introduction.quickstart_isaacsim_robot.standalone_start_and_scene",
            "../../../docs/isaacsim/snippets/introduction/quickstart_isaacsim_robot/standalone_start_and_scene.py",
            "--test",
        },
        -- python_scripting/manual_standalone_python
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.python_scripting.manual_standalone_python.from_python_code",
            "../../../docs/isaacsim/snippets/python_scripting/manual_standalone_python/from_python_code.py",
            "--test",
        },
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.python_scripting.manual_standalone_python.usage_example",
            "../../../docs/isaacsim/snippets/python_scripting/manual_standalone_python/usage_example.py",
            "--test",
        },
        -- python_scripting/util_snippets
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.python_scripting.util_snippets.rendering_frame_delay",
            "../../../docs/isaacsim/snippets/python_scripting/util_snippets/rendering_frame_delay.py",
            "--test",
        },
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.python_scripting.util_snippets.rendering_frame_delay_1",
            "../../../docs/isaacsim/snippets/python_scripting/util_snippets/rendering_frame_delay_1.py",
            "--test",
        },
        -- reference_material
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.reference_material.sim_performance_optimization_handbook.cpu_thread_count_optimizations",
            "../../../docs/isaacsim/snippets/reference_material/sim_performance_optimization_handbook/cpu_thread_count_optimizations.py",
            "--test",
        },
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.reference_material.sim_performance_optimization_handbook.scene_and_rendering_optimizations",
            "../../../docs/isaacsim/snippets/reference_material/sim_performance_optimization_handbook/scene_and_rendering_optimizations.py",
            "--test",
        },
        -- action_and_event_data_generation/tutorial_replicator_incident
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.action_and_event_data_generation.tutorial_replicator_incident.tutorial_replicator_incident",
            "../../../docs/isaacsim/snippets/action_and_event_data_generation/tutorial_replicator_incident.py",
            "--test",
        },
        -- action_and_event_data_generation/tutorial_replicator_agent
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.action_and_event_data_generation.tutorial_replicator_agent.tutorial_replicator_agent",
            "../../../docs/isaacsim/snippets/action_and_event_data_generation/tutorial_replicator_agent.py",
            "--test",
        },
        -- replicator_tutorials
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.replicator_tutorials.tutorial_replicator_isaac_randomizers.simready_assets_sdg_example",
            "../../../docs/isaacsim/snippets/replicator_tutorials/tutorial_replicator_isaac_randomizers/simready_assets_sdg_example.py",
            "--test",
        },
        -- robot_setup/asset_transformer_api
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.robot_setup.asset_transformer_api",
            "../../../docs/isaacsim/snippets/robot_setup/asset_transformer_api.py",
            "--test",
        },
        -- robot_setup/asset_transformer_tutorials
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.robot_setup.asset_transformer_tutorials",
            "../../../docs/isaacsim/snippets/robot_setup/asset_transformer_tutorials.py",
            "--test",
        },
        -- robot_setup/merge_mesh
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.robot_setup.merge_mesh",
            "../../../docs/isaacsim/snippets/robot_setup/merge_mesh.py",
            "--test",
        },
        -- robot_simulation/mobile_robot_controllers
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.robot_simulation.mobile_robot_controllers.ackemann_controller",
            "../../../docs/isaacsim/snippets/robot_simulation/mobile_robot_controllers/ackemann_controller.py",
            "--test",
        },
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.robot_simulation.mobile_robot_controllers.differential_controller",
            "../../../docs/isaacsim/snippets/robot_simulation/mobile_robot_controllers/differential_controller.py",
            "--test",
        },
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.robot_simulation.mobile_robot_controllers.holonomic_controller",
            "../../../docs/isaacsim/snippets/robot_simulation/mobile_robot_controllers/holonomic_controller.py",
            "--test",
        },
        -- robot_simulation/grasp_editor
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.robot_simulation.grasp_editor.using_authored_grasps_in_isaac_sim",
            "../../../docs/isaacsim/snippets/robot_simulation/grasp_editor/using_authored_grasps_in_isaac_sim.py",
            "--test",
        },
        -- ros2_tutorials
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.ros2_tutorials.tutorial_ros2_camera_publishing.camera_publishing",
            "../../../docs/isaacsim/snippets/ros2_tutorials/tutorial_ros2_camera_publishing/camera_publishing.py",
            "--test",
        },
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.ros2_tutorials.tutorial_ros2_rtx_lidar.create_a_separate_writer_for_the_objectid_mapping",
            "../../../docs/isaacsim/snippets/ros2_tutorials/tutorial_ros2_rtx_lidar/create_a_separate_writer_for_the_objectid_mapping.py",
            "--test",
        },
        -- sensors/isaacsim_sensors_physics_contact
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.sensors.isaacsim_sensors_physics_contact.creating_and_modifying_the_contact_sensor",
            "../../../docs/isaacsim/snippets/sensors/isaacsim_sensors_physics_contact/creating_and_modifying_the_contact_sensor.py",
            "--test",
        },
        -- sensors/isaacsim_sensors_physics_imu
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.sensors.isaacsim_sensors_physics_imu.creating_and_modifying_the_imu",
            "../../../docs/isaacsim/snippets/sensors/isaacsim_sensors_physics_imu/creating_and_modifying_the_imu.py",
            "--test",
        },
        -- sensors/isaacsim_sensors_physx_proximity
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.sensors.isaacsim_sensors_physx_proximity.standalone_python",
            "../../../docs/isaacsim/snippets/sensors/isaacsim_sensors_physx_proximity/standalone_python.py",
            "--test",
        },
        -- sensors/isaacsim_sensors_rtx
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.sensors.isaacsim_sensors_rtx.how_to_enable_motion_bvh",
            "../../../docs/isaacsim/snippets/sensors/isaacsim_sensors_rtx/how_to_enable_motion_bvh.py",
            "--test",
        },
        -- sensors/isaacsim_sensors_rtx_annotators
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.sensors.isaacsim_sensors_rtx_annotators.collect_data_with_lidar_sensor",
            "../../../docs/isaacsim/snippets/sensors/isaacsim_sensors_rtx_annotators/collect_data_with_lidar_sensor.py",
            "--test",
        },
        -- sensors/isaacsim_sensors_multitick_rendering
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.sensors.isaacsim_sensors_multitick_rendering.defer_radar_after_lidar_warmup",
            "../../../docs/isaacsim/snippets/sensors/isaacsim_sensors_multitick_rendering/defer_radar_after_lidar_warmup.py",
            "--test",
        },
        -- cumotion/trajectory_optimizer
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.cumotion.trajectory_optimizer_example",
            "../../../docs/isaacsim/snippets/cumotion/trajectory_optimizer_example.py",
            "--test",
        },
        -- cumotion/graph_planner
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.cumotion.graph_planner_example",
            "../../../docs/isaacsim/snippets/cumotion/graph_planner_example.py",
            "--test",
        },
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.cumotion.robot_configuration_example",
            "../../../docs/isaacsim/snippets/cumotion/robot_configuration_example.py",
            "--test",
        },
        -- cumotion/rmpflow
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.cumotion.rmpflow_example",
            "../../../docs/isaacsim/snippets/cumotion/rmpflow_example.py",
            "--test",
        },
        -- cumotion/trajectory_generator
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.cumotion.trajectory_generator_example",
            "../../../docs/isaacsim/snippets/cumotion/trajectory_generator_example.py",
            "--test",
        },
        -- cumotion/world_interface
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.cumotion.world_interface_example",
            "../../../docs/isaacsim/snippets/cumotion/world_interface_example.py",
            "--test",
        },
        -- utilities/debugging/profiling_performance
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.utilities.debugging.profiling_performance.standalone_workflow",
            "../../../docs/isaacsim/snippets/utilities/debugging/profiling_performance/standalone_workflow.py",
            "--test",
        },
        -- utilities/debugging/tutorial_advanced_python_debugging
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.utilities.debugging.tutorial_advanced_python_debugging.add_the_following_lines_to_hello_worldpy_and_place",
            "../../../docs/isaacsim/snippets/utilities/debugging/tutorial_advanced_python_debugging/add_the_following_lines_to_hello_worldpy_and_place.py",
            "--test",
        },
        -- async snippets
        {
            "doc_snippets/tests-nativepython-testing-doc_snippets.test_snippets_async",
            "standalone_examples/testing/doc_snippets/test_snippets_async.py",
            "--expected-failures-csv expected_failures.csv --experience-csv experiences.csv --excluded-snippets-csv excluded_snippets.csv --platform-constraints-csv platform_constraints.csv",
        },
    }
end

local function get_testing_misc_tests()
    return {
        -- Core API Testing Samples
        {
            "tests-nativepython-testing-isaacsim.core.api.test_hello_world",
            "standalone_examples/testing/isaacsim.core.api/test_hello_world.py",
            "--test",
        },
        {
            "tests-nativepython-testing-isaacsim.core.api.test_articulation",
            "standalone_examples/testing/isaacsim.core.api/test_articulation.py",
        },
        {
            "tests-nativepython-testing-isaacsim.core.api.test_time_stepping",
            "standalone_examples/testing/isaacsim.core.api/test_time_stepping.py",
        },
        {
            "tests-nativepython-testing-isaacsim.core.api.test_save_stage",
            "standalone_examples/testing/isaacsim.core.api/test_save_stage.py",
        },
        {
            "tests-nativepython-testing-isaacsim.core.api.test_delete_in_contact",
            "standalone_examples/testing/isaacsim.core.api/test_delete_in_contact.py",
        },
        {
            "tests-nativepython-testing-isaacsim.core.api.test_xform_prim_view",
            "standalone_examples/testing/isaacsim.core.api/test_xform_prim_view.py",
        },
        -- Python Shell Tests
        {
            "tests-nativepython-testing-python_sh.import_torch",
            "standalone_examples/testing/python_sh/import_torch.py",
        },
        {
            "tests-nativepython-testing-python_sh.import_scipy",
            "standalone_examples/testing/python_sh/import_scipy.py",
        },
        {
            "tests-nativepython-testing-python_sh.path_length",
            "standalone_examples/testing/python_sh/path_length.py",
        },
        {
            "tests-nativepython-testing-python_sh.import_sys",
            "standalone_examples/testing/python_sh/import_sys.py",
        },
        -- Tutorials
        {
            "tests-nativepython-testing-tutorials-getting_started",
            "standalone_examples/tutorials/getting_started/getting_started.py",
        },
        {
            "tests-nativepython-testing-tutorials-getting_started_robot",
            "standalone_examples/tutorials/getting_started/getting_started_robot.py",
        },
        {
            "tests-nativepython-testing-tutorials.manipulation.tutorial_9_gripper_control",
            "standalone_examples/tutorials/manipulation/tutorial_9_gripper_control.py",
            "--test",
        },
        {
            "tests-nativepython-testing-tutorials.manipulation.tutorial_9_arm_trajectory",
            "standalone_examples/tutorials/manipulation/tutorial_9_arm_trajectory.py",
            "--test",
        },
        {
            "tests-nativepython-testing-tutorials.manipulation.tutorial_9_follow_target",
            "standalone_examples/tutorials/manipulation/tutorial_9_follow_target.py",
            "--test",
        },
        {
            "tests-nativepython-testing-tutorials.manipulation.tutorial_9_pick_place_cumotion",
            "standalone_examples/tutorials/manipulation/tutorial_9_pick_place_cumotion.py",
            "--test",
        },
        {
            "tests-nativepython-testing-tutorials.manipulation.tutorial_9_pick_place_pink",
            "standalone_examples/tutorials/manipulation/tutorial_9_pick_place_pink.py",
            "--test",
        },
    }
end

local function get_benchmark_tests()
    return {
        {
            "tests-standalone_benchmarks-benchmark_robot_motion_cumotion_rmpflow",
            "standalone_examples/benchmarks/benchmark_robot_motion_cumotion_rmpflow.py",
            "--num-frames 10",
        },
        {
            "tests-standalone_benchmarks-benchmark_robot_motion_lula_rmpflow",
            "standalone_examples/benchmarks/benchmark_robot_motion_lula_rmpflow.py",
            "--num-frames 10",
        },
        {
            "tests-standalone_benchmarks-benchmark_camera",
            "standalone_examples/benchmarks/benchmark_camera.py",
            "--num-frames 10 --num-cameras 2",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_nova_carter_ros2",
            "standalone_examples/benchmarks/benchmark_robots_nova_carter_ros2.py",
            "--num-frames 10 --num-robots 2 --enable-3d-lidar 1 --enable-2d-lidar 2 --enable-hawks 1 --non-headless",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_nova_carter_ros2_async",
            "standalone_examples/benchmarks/benchmark_robots_nova_carter_ros2.py",
            "--num-frames 10 --num-robots 2 --enable-3d-lidar 1 --enable-2d-lidar 2 --enable-hawks 1 --non-headless --async-render-handshake --/rtx/hydra/supportMultiTickRate=false --/rtx/rendering/perSensorTickTlas=false",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_nova_carter",
            "standalone_examples/benchmarks/benchmark_robots_nova_carter.py",
            "--num-frames 10 --num-robots 2",
        },
        {
            "tests-standalone_benchmarks-benchmark_rtx_lidar_rotary",
            "standalone_examples/benchmarks/benchmark_rtx_lidar.py",
            "--num-frames 10 --num-sensors 8 --lidar-type Rotary",
        },
        {
            "tests-standalone_benchmarks-benchmark_rtx_lidar_solid_state",
            "standalone_examples/benchmarks/benchmark_rtx_lidar.py",
            "--num-frames 10 --num-sensors 8 --lidar-type Solid_State",
        },
        {
            "tests-standalone_benchmarks-benchmark_rtx_lidar_ros2_pcl_metadata",
            "standalone_examples/benchmarks/benchmark_rtx_lidar_ros2_pcl_metadata.py",
            "--num-frames 10 --num-sensors 2 --metadata Intensity ObjectId Timestamp --non-headless",
        },
        {
            "tests-standalone_benchmarks-benchmark_sdg_simple",
            "standalone_examples/benchmarks/benchmark_sdg.py",
            "--num-frames 10 --num-cameras 2 --resolution 1280 720 --asset-count 10 --annotators rgb distance_to_camera --disable-viewport-rendering --delete-data-when-done --print-results",
        },
        {
            "tests-standalone_benchmarks-benchmark_sdg_advanced",
            "standalone_examples/benchmarks/benchmark_sdg.py",
            "--num-frames 10 --num-cameras 2 --resolution 1280 720 --asset-count 10 --annotators all --disable-viewport-rendering --delete-data-when-done --print-results",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_ur10",
            "standalone_examples/benchmarks/benchmark_robots_ur10.py",
            "--num-frames 10 --num-robots 10",
        },
        {
            "tests-standalone_benchmarks-benchmark_physx_lidar",
            "standalone_examples/benchmarks/benchmark_physx_lidar.py",
            "--num-frames 10 --num-sensors 4",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_o3dyn",
            "standalone_examples/benchmarks/benchmark_robots_o3dyn.py",
            "--num-frames 10 --num-robots 2",
        },
        {
            "tests-standalone_benchmarks-benchmark_scene_loading",
            "standalone_examples/benchmarks/benchmark_scene_loading.py",
            "--num-frames 10 --env-url /Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_evobot",
            "standalone_examples/benchmarks/benchmark_robots_evobot.py",
            "--num-frames 10 --num-robots 1 1 1",
        },
        {
            "tests-standalone_benchmarks-benchmark_single_view_depth_sensor",
            "standalone_examples/benchmarks/benchmark_single_view_depth_sensor.py",
            "--num-frames 10 --num-cameras 2",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_humanoid",
            "standalone_examples/benchmarks/benchmark_robots_humanoid.py",
            "--num-frames 10 --num-robots 2",
        },
        {
            "tests-standalone_benchmarks-benchmark_async_handshake_validation",
            "standalone_examples/benchmarks/validation/benchmark_async_handshake_validation.py",
            "--num-frames 50",
        },
        {
            "tests-standalone_benchmarks-benchmark_rtx_radar",
            "standalone_examples/benchmarks/benchmark_rtx_radar.py",
            "--num-frames 10 --num-sensors 2",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_nova_carter_ros2_validation",
            "standalone_examples/benchmarks/validation/benchmark_robots_nova_carter_ros2_validation.py",
	},
        {
            "tests-standalone_benchmarks-benchmark_mobility_gen_recording",
            "standalone_examples/benchmarks/benchmark_mobility_gen_recording.py",
            "--num-steps 10 --warmup-steps 5",
        },
    }
end

function create_tests()
    -- Startup test group
    define_test_group("startup_tests", get_startup_tests())

    -- Python samples
    group("python_samples")

    -- smoke tests for python.sh itself
    python_script_test("tests-nativepython-pip_list", "-m pip list --")
    python_script_test(
        "tests-nativepython-pycocotools",
        "-m pip install --force pycocotools --no-cache-dir --no-dependencies --"
    ) -- this test makes sure that pip packages that need Python.h can be installed.

    -- Omni Kit App
    python_sample_test(
        "tests-nativepython-omni.kit.app.app_framework",
        "standalone_examples/api/omni.kit.app/app_framework.py"
    )

    register_python_sample_tests(get_simulation_app_tests())

    if os.target() == "linux" then
        python_sample_test(
            "tests-nativepython-testing-isaacsim.simulation_app.test_ovd",
            "standalone_examples/testing/isaacsim.simulation_app/test_ovd.py",
            '--ovd="/tmp/"'
        )
    end

    register_python_sample_tests(get_core_tests())
    register_python_sample_tests(get_sensor_tests())
    register_python_sample_tests(get_robot_tests())
    register_python_sample_tests(get_asset_tests())
    register_python_sample_tests(get_replicator_tests())
    register_python_sample_tests(get_ros_tests())
    register_python_sample_tests(get_doc_snippets_tests())
    register_python_sample_tests(get_testing_misc_tests())

    -- Benchmarks
    group("benchmarks")
    register_python_sample_tests(get_benchmark_tests())

end
