# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio
import threading

import carb
import isaacsim.core.utils.stage as stage_utils
import nest_asyncio
import omni
import omni.timeline
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from pxr import Sdf
from usdrt import Usd

from .entity_utils import create_empty_entity_state, get_entity_state, get_filtered_entities


# Define the Singleton decorator
def Singleton(class_):
    """A singleton decorator to ensure only one instance of a class exists"""
    instances = {}

    def getinstance(*args, **kwargs):
        if class_ not in instances:
            instances[class_] = class_(*args, **kwargs)
        return instances[class_]

    return getinstance


@Singleton
class ROS2ServiceManager:
    """Manager for ROS2 services that can control the Isaac Sim simulation.

    This class is a singleton to ensure there's only one ROS2 node running
    that handles all simulation control services.
    """

    def __init__(self):
        self.node_name = "isaac_sim_control"
        self.node = None
        self.services = {}
        self.action_servers = {}  # New dictionary to store action servers
        self.is_initialized = False
        self.running = False
        self.spin_thread = None
        self.loop = None

    def initialize(self):
        """Initialize the ROS2 node for simulation control services"""

        if self.is_initialized:
            return

        try:
            import rclpy
            from rclpy.node import Node

            # Initialize ROS2 if it's not already initialized
            try:
                rclpy.init()
            except RuntimeError:
                # ROS2 is already initialized
                pass

            # Create a ROS2 node
            self.node = rclpy.create_node(self.node_name)
            self.is_initialized = True

            # Create event loop in main thread
            self.loop = asyncio.get_event_loop()
            # Allow nested event loops
            nest_asyncio.apply(self.loop)

            # Start a separate thread for ROS2 spinning
            self.running = True
            self.spin_thread = threading.Thread(target=self._spin)
            self.spin_thread.daemon = True
            self.spin_thread.start()

            carb.log_info(f"ROS2 ServiceManager initialized with node '{self.node_name}'")

        except ImportError as e:
            carb.log_error(f"Failed to import ROS2 Python libraries: {e}")
            self.is_initialized = False
        except Exception as e:
            carb.log_error(f"Error initializing ROS2 ServiceManager: {e}")
            self.is_initialized = False

    def shutdown(self):
        """Shutdown the ROS2 node and clean up resources"""
        if not self.is_initialized:
            return

        import rclpy

        self.running = False

        # Wait for the spin thread to finish
        if self.spin_thread and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)

        # Destroy all services
        for service_name, service in self.services.items():
            self.node.destroy_service(service)
        self.services.clear()

        # Destroy all action servers
        for action_name, action_server in self.action_servers.items():
            action_server.destroy()
        self.action_servers.clear()

        # Destroy the node
        if self.node:
            self.node.destroy_node()
            self.node = None

        # Try to shut down ROS2
        try:
            rclpy.shutdown()
        except Exception:
            pass

        self.is_initialized = False
        carb.log_info("ROS2 ServiceManager shutdown completed")

    def register_service(self, service_name, service_type, callback):
        """Register a new ROS2 service

        Args:
            service_name (str): Name of the service
            service_type: ROS2 service type
            callback: Async callback function to handle service requests

        Returns:
            bool: True if registration was successful, False otherwise
        """
        if not self.is_initialized:
            carb.log_error("Cannot register service: ROS2 ServiceManager not initialized")
            return False

        if service_name in self.services:
            carb.log_warn(f"Service '{service_name}' is already registered")
            return False

        try:
            # Wrap async callback to work with ROS2 sync service
            def sync_wrapper(request, response):
                if not self.loop:
                    self.loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(self.loop)
                future = asyncio.run_coroutine_threadsafe(callback(request, response), self.loop)
                return future.result()

            service = self.node.create_service(service_type, service_name, sync_wrapper)
            self.services[service_name] = service
            carb.log_info(f"Registered ROS2 service: {service_name}")
            return True
        except Exception as e:
            carb.log_error(f"Failed to register service '{service_name}': {e}")
            return False

    def unregister_service(self, service_name):
        """Unregister a ROS2 service

        Args:
            service_name (str): Name of the service to unregister

        Returns:
            bool: True if unregistration was successful, False otherwise
        """
        if not self.is_initialized or service_name not in self.services:
            return False

        try:
            service = self.services.pop(service_name)
            self.node.destroy_service(service)
            carb.log_info(f"Unregistered ROS2 service: {service_name}")
            return True
        except Exception as e:
            carb.log_error(f"Failed to unregister service '{service_name}': {e}")
            return False

    def register_action_server(
        self, action_name, action_type, execute_callback, goal_callback=None, cancel_callback=None
    ):
        """Register a new ROS2 action server

        Args:
            action_name (str): Name of the action
            action_type: ROS2 action type
            execute_callback: Async callback function to handle action execution
            goal_callback: Callback to accept/reject goals (optional)
            cancel_callback: Callback to handle cancellation (optional)

        Returns:
            bool: True if registration was successful, False otherwise
        """
        if not self.is_initialized:
            carb.log_error("Cannot register action server: ROS2 ServiceManager not initialized")
            return False

        if action_name in self.action_servers:
            carb.log_warn(f"Action server '{action_name}' is already registered")
            return False

        try:
            import rclpy
            from rclpy.action import ActionServer

            # Create the action server
            action_server = ActionServer(
                node=self.node,
                action_type=action_type,
                action_name=action_name,
                execute_callback=self._wrap_execute_callback(execute_callback),
                goal_callback=goal_callback,
                cancel_callback=cancel_callback,
            )

            self.action_servers[action_name] = action_server
            carb.log_info(f"Registered ROS2 action server: {action_name}")
            return True

        except Exception as e:
            carb.log_error(f"Failed to register action server '{action_name}': {e}")
            return False

    def _wrap_execute_callback(self, execute_callback):
        """Wrap async execute callback to work with ROS2 action server

        Args:
            execute_callback: Async callback function

        Returns:
            function: Wrapped callback that handles the event loop
        """

        def wrapped_execute(goal_handle):
            if not self.loop:
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)
            future = asyncio.run_coroutine_threadsafe(execute_callback(goal_handle), self.loop)
            return future.result()

        return wrapped_execute

    def unregister_action_server(self, action_name):
        """Unregister a ROS2 action server

        Args:
            action_name (str): Name of the action server to unregister

        Returns:
            bool: True if unregistration was successful, False otherwise
        """
        if not self.is_initialized or action_name not in self.action_servers:
            return False

        try:
            action_server = self.action_servers.pop(action_name)
            action_server.destroy()
            carb.log_info(f"Unregistered ROS2 action server: {action_name}")
            return True
        except Exception as e:
            carb.log_error(f"Failed to unregister action server '{action_name}': {e}")
            return False

    def _spin(self):
        """Spin the ROS2 node in a separate thread"""
        import rclpy

        while self.running and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except rclpy.executors.ExternalShutdownException:
                break


class SimulationControl:
    SERVICE_PREFIX = "isaacsim"  # New constant for service prefix

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.service_manager = ROS2ServiceManager()
        self.is_initialized = False

        # Import service types at class level
        try:
            from simulation_interfaces.srv import (
                DeleteEntity,
                GetEntities,
                GetEntitiesStates,
                GetEntityInfo,
                GetEntityState,
                GetSimulationState,
                GetSimulatorFeatures,
                ResetSimulation,
                SetEntityState,
                SetSimulationState,
                SpawnEntity,
                StepSimulation,
            )

            self.GetSimulationState = GetSimulationState
            self.SetSimulationState = SetSimulationState
            self.GetEntities = GetEntities
            self.DeleteEntity = DeleteEntity
            self.GetEntityInfo = GetEntityInfo
            self.SpawnEntity = SpawnEntity
            self.ResetSimulation = ResetSimulation
            self.StepSimulation = StepSimulation
            self.GetEntityState = GetEntityState
            self.GetEntitiesStates = GetEntitiesStates
            self.SetEntityState = SetEntityState
            self.GetSimulatorFeatures = GetSimulatorFeatures

            # Import action types
            from simulation_interfaces.action import SimulateSteps

            self.SimulateSteps = SimulateSteps
        except ImportError as e:
            carb.log_error(f"Failed to import simulation_interfaces services: {e}")
            self.GetSimulationState = None
            self.SetSimulationState = None
            self.GetEntities = None
            self.DeleteEntity = None
            self.GetEntityInfo = None
            self.SpawnEntity = None
            self.ResetSimulation = None
            self.StepSimulation = None
            self.GetEntityState = None
            self.GetEntitiesStates = None
            self.SetEntityState = None
            self.SimulateSteps = None
            self.GetSimulatorFeatures = None

        # Initialize and register services
        self._initialize_ros2_services()

    def _initialize_ros2_services(self):
        """Initialize ROS2 services for simulation control"""
        try:
            # Import ROS2 message and service types
            from simulation_interfaces.msg import Result, SimulationState

            # Initialize the ROS2 service manager
            self.service_manager.initialize()

            if not self.service_manager.is_initialized:
                carb.log_error("Failed to initialize ROS2 service manager")
                return

            # Register the basic simulation control services
            service_prefix = SimulationControl.SERVICE_PREFIX

            # Register the simulation state services
            if self.GetSimulationState:
                self.service_manager.register_service(
                    f"{service_prefix}/GetSimulationState", self.GetSimulationState, self._handle_get_simulation_state
                )
            else:
                carb.log_error("GetSimulationState service type not available")

            if self.SetSimulationState:
                self.service_manager.register_service(
                    f"{service_prefix}/SetSimulationState", self.SetSimulationState, self._handle_set_simulation_state
                )
            else:
                carb.log_error("SetSimulationState service type not available")

            # Register the GetEntities service
            if self.GetEntities:
                self.service_manager.register_service(
                    f"{service_prefix}/GetEntities", self.GetEntities, self._handle_get_entities
                )
            else:
                carb.log_error("GetEntities service type not available")

            # Register the DeleteEntity service
            if self.DeleteEntity:
                self.service_manager.register_service(
                    f"{service_prefix}/DeleteEntity", self.DeleteEntity, self._handle_delete_entity
                )
            else:
                carb.log_error("DeleteEntity service type not available")

            # Register the GetEntityInfo service
            if self.GetEntityInfo:
                self.service_manager.register_service(
                    f"{service_prefix}/GetEntityInfo", self.GetEntityInfo, self._handle_get_entity_info
                )
            else:
                carb.log_error("GetEntityInfo service type not available")

            # Register the SpawnEntity service
            if self.SpawnEntity:
                self.service_manager.register_service(
                    f"{service_prefix}/SpawnEntity", self.SpawnEntity, self._handle_spawn_entity
                )
            else:
                carb.log_error("SpawnEntity service type not available")

            # Register the ResetSimulation service
            if self.ResetSimulation:
                self.service_manager.register_service(
                    f"{service_prefix}/ResetSimulation", self.ResetSimulation, self._handle_reset_simulation
                )
            else:
                carb.log_error("ResetSimulation service type not available")

            # Register the StepSimulation service
            if self.StepSimulation:
                self.service_manager.register_service(
                    f"{service_prefix}/StepSimulation", self.StepSimulation, self._handle_step_simulation
                )
            else:
                carb.log_error("StepSimulation service type not available")

            # Register the GetEntityState service
            if self.GetEntityState:
                self.service_manager.register_service(
                    f"{service_prefix}/GetEntityState", self.GetEntityState, self._handle_get_entity_state
                )
            else:
                carb.log_error("GetEntityState service type not available")

            # Register the GetEntitiesStates service
            if self.GetEntitiesStates:
                self.service_manager.register_service(
                    f"{service_prefix}/GetEntitiesStates", self.GetEntitiesStates, self._handle_get_entities_states
                )
            else:
                carb.log_error("GetEntitiesStates service type not available")

            # Register the SetEntityState service
            if hasattr(self, "SetEntityState") and self.SetEntityState:
                self.service_manager.register_service(
                    f"{service_prefix}/SetEntityState", self.SetEntityState, self._handle_set_entity_state
                )
            else:
                carb.log_error("SetEntityState service type not available")

            # Register the SimulateSteps action server
            if hasattr(self, "SimulateSteps") and self.SimulateSteps:
                self.service_manager.register_action_server(
                    f"{service_prefix}/SimulateSteps", self.SimulateSteps, self._handle_simulate_steps_action
                )
            else:
                carb.log_error("SimulateSteps action type not available")

            # Register the GetSimulatorFeatures service
            if hasattr(self, "GetSimulatorFeatures") and self.GetSimulatorFeatures:
                self.service_manager.register_service(
                    f"{service_prefix}/GetSimulatorFeatures",
                    self.GetSimulatorFeatures,
                    self._handle_get_simulator_features,
                )
            else:
                carb.log_error("GetSimulatorFeatures service type not available")

            self.is_initialized = True
            carb.log_info("ROS 2 Simulation Control services initialized")

        except ImportError as e:
            carb.log_error(f"Failed to import required ROS2 types: {e}")
            self.is_initialized = False
        except Exception as e:
            carb.log_error(f"Error initializing ROS2 services: {e}")
            self.is_initialized = False

    async def _handle_get_simulation_state(self, request, response):
        """Handle simulation state query request"""
        try:
            from simulation_interfaces.msg import Result, SimulationState

            # Get current simulation state with correct constant mapping
            if self.timeline.is_playing():
                response.state.state = SimulationState.STATE_PLAYING  # 1
            elif self.timeline.is_stopped():
                response.state.state = SimulationState.STATE_STOPPED  # 0
            else:  # Paused state
                response.state.state = SimulationState.STATE_PAUSED  # 2

            # Create Result with correct fields
            response.result = Result(result=Result.RESULT_OK, error_message="")

        except Exception as e:
            response.result = Result(
                result=Result.RESULT_OPERATION_FAILED, error_message=f"Failed to get simulation state: {str(e)}"
            )
            carb.log_error(f"Error in get_simulation_state service: {e}")

        return response

    async def _handle_set_simulation_state(self, request, response):
        """Handle request to set simulation state"""
        try:
            from simulation_interfaces.msg import Result, SimulationState

            target_state = request.state.state

            if target_state == SimulationState.STATE_PLAYING:
                if not self.timeline.is_playing():
                    self.timeline.play()
                response.result = Result(result=Result.RESULT_OK, error_message="")
            elif target_state == SimulationState.STATE_PAUSED:
                if self.timeline.is_playing():
                    self.timeline.pause()
                response.result = Result(result=Result.RESULT_OK, error_message="")
            elif target_state == SimulationState.STATE_STOPPED:
                self.timeline.stop()
                response.result = Result(result=Result.RESULT_OK, error_message="")
            elif target_state == SimulationState.STATE_QUITTING:
                # First stop the simulation timeline
                self.timeline.stop()

                # Schedule application quit
                carb.log_warn("Shutting down Isaac Sim via STATE_QUITTING request")
                response.result = Result(result=Result.RESULT_OK, error_message="Initiating simulator shutdown")
                omni.kit.app.get_app().post_uncancellable_quit(0)
            else:
                response.result = Result(
                    result=Result.RESULT_FEATURE_UNSUPPORTED,
                    error_message=f"Unsupported state transition to {target_state}",
                )

        except Exception as e:
            response.result = Result(
                result=Result.RESULT_OPERATION_FAILED, error_message=f"Failed to set simulation state: {str(e)}"
            )
            carb.log_error(f"Error in SetSimulationState service: {e}")

        return response

    async def _handle_get_entities(self, request, response):
        """Handle GetEntities service request

        This service returns a list of entities (prims) in the simulation,
        optionally filtered by name using the partial prim path.

        Args:
            request: GetEntities request with optional filters
            response: GetEntities response with entities list and result

        Returns:
            response: Completed GetEntities response
        """

        try:
            from simulation_interfaces.msg import Result

            # Get the results ready
            response.entities = []
            response.result.result = Result.RESULT_OK
            response.result.error_message = ""

            # Get usdrt stage for traversing
            usdrt_stage = stage_utils.get_current_stage(fabric=True)
            if not usdrt_stage:
                response.result.result = Result.RESULT_OPERATION_FAILED
                response.result.error_message = "usdrt Stage not available for traversing"
                return response

            # Get filtered entities using external helper function
            filter_pattern = (
                request.filters.filter if hasattr(request, "filters") and hasattr(request.filters, "filter") else None
            )
            filtered_entities, error = get_filtered_entities(usdrt_stage, filter_pattern)

            if error:
                response.result.result = Result.RESULT_OPERATION_FAILED
                response.result.error_message = error
                return response

            response.entities = filtered_entities

            # Log the number of entities found
            carb.log_info(f"GetEntities found {len(response.entities)} entities")

        except Exception as e:
            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error getting entities: {e}"
            carb.log_error(f"Error in GetEntities service handler: {e}")

        return response

    async def _handle_delete_entity(self, request, response):
        """Handle DeleteEntity service request

        This service deletes a specified entity (prim) from the simulation if it exists
        and is not protected from deletion.

        Args:
            request: DeleteEntity request with entity prim path
            response: DeleteEntity response with result status

        Returns:
            response: Completed DeleteEntity response
        """
        try:
            import isaacsim.core.utils.prims as prim_utils
            from simulation_interfaces.msg import Result

            # First check if the entity exists
            if not prim_utils.is_prim_path_valid(request.entity):
                response.result = Result(
                    result=Result.RESULT_NOT_FOUND,
                    error_message=f"Entity '{request.entity}' does not exist",
                )
                return response

            # Check if prim can be deleted
            if prim_utils.is_prim_no_delete(request.entity):
                response.result = Result(
                    result=Result.RESULT_OPERATION_FAILED,
                    error_message=f"Entity '{request.entity}' is protected and cannot be deleted",
                )
                return response

            # Delete the prim - Note: delete_prim returns True if prim was protected, False if successfully deleted
            if not prim_utils.delete_prim(request.entity):
                response.result = Result(result=Result.RESULT_OK, error_message="")
                carb.log_info(f"Successfully deleted entity: {request.entity}")
            else:
                response.result = Result(
                    result=Result.RESULT_OPERATION_FAILED,
                    error_message=f"Entity '{request.entity}' could not be deleted (protected)",
                )

        except Exception as e:
            response.result = Result(result=Result.RESULT_OPERATION_FAILED, error_message=f"Error deleting entity: {e}")
            carb.log_error(f"Error in DeleteEntity service handler: {e}")

        return response

    async def _handle_get_entity_info(self, request, response):
        """Handle GetEntityInfo service request

        This service provides detailed information about a specific entity in the simulation.
        Currently, all entities are classified as OBJECT category this is a placeholder for future use.

        Args:
            request: GetEntityInfo request containing the entity prim path
            response: GetEntityInfo response with entity information

        Returns:
            response: Completed GetEntityInfo response
        """

        try:
            import isaacsim.core.utils.prims as prims_utils
            from simulation_interfaces.msg import EntityCategory, EntityInfo, Result

            if not prims_utils.is_prim_path_valid(request.entity):
                response.result = Result(
                    result=Result.RESULT_NOT_FOUND, error_message=f"Entity '{request.entity}' does not exist"
                )
                return response

            # Set entity info with default OBJECT category. this is a placeholder for future use
            response.info = EntityInfo(
                category=EntityCategory(category=EntityCategory.CATEGORY_OBJECT), description="", tags=[]
            )
            response.result = Result(result=Result.RESULT_OK, error_message="")
            carb.log_info(f"Successfully retrieved info for entity: {request.entity}")

        except Exception as e:
            response.result = Result(
                result=Result.RESULT_OPERATION_FAILED, error_message=f"Error getting entity info: {e}"
            )
            carb.log_error(f"Error in GetEntityInfo service handler: {e}")

        return response

    async def _handle_spawn_entity(self, request, response):
        """Handle SpawnEntity service request

        This service spawns a new entity in the simulation.
        If URI is provided, it loads the valid USD file as a reference in the given prim path.
        If URI is not provided, a Xform will be created in the given prim path.
        Any spawned prims using this service will be tracked.

        Args:
            request: SpawnEntity request with entity name, URI, and initial pose
            response: SpawnEntity response with result status

        Returns:
            response: Completed SpawnEntity response
        """

        try:
            from simulation_interfaces.msg import Result

            # Get regular stage for prim operations
            stage = stage_utils.get_current_stage()
            if not stage:
                response.result.result = Result.RESULT_OPERATION_FAILED
                response.result.error_message = "Stage not available"
                return response

            # Check name validity and try to get default prim name from URI if possible
            entity_name = request.name

            # If name is empty, try to get default prim name from URI
            if not entity_name and request.uri:
                try:
                    # Try to open the stage and get its default prim
                    temp_stage = Usd.Stage.Open(request.uri)
                    if temp_stage:
                        default_prim = temp_stage.GetDefaultPrim()
                        if default_prim:
                            # Use the default prim name
                            default_prim_name = default_prim.GetName()
                            entity_name = f"{default_prim_name}"
                            carb.log_info(f"Using default prim name from USD: {entity_name}")
                except Exception as e:
                    carb.log_warn(f"Could not extract default prim name from USD: {e}")

            # Check if name is still empty
            if not entity_name:
                if not request.allow_renaming:
                    response.result.result = response.NAME_INVALID
                    response.result.error_message = (
                        "Entity name is empty, no default prim found, and allow_renaming is false"
                    )
                    return response
                # Generate a unique name by counting existing spawned entities
                spawned_count = 0
                # Get usdrt stage for traversing to count spawned entities
                usdrt_stage = stage_utils.get_current_stage(fabric=True)
                if usdrt_stage:
                    for prim in usdrt_stage.Traverse():
                        if prim.HasAttribute("simulationInterfacesSpawned"):
                            attr = prim.GetAttribute("simulationInterfacesSpawned")
                            if attr and attr.Get():
                                spawned_count += 1
                else:
                    carb.log_warn("usdrt stage not available for counting spawned entities, using 0 as count")
                entity_name = f"SpawnedEntity_{spawned_count}"
            elif not entity_name.startswith("/"):
                # Name provided
                # The stage will handle the proper path creation based on where this is added
                entity_name = f"/{entity_name}"
                carb.log_info(f"Using entity name as is: /{entity_name}")

            # Check if name already exists
            if stage.GetPrimAtPath(entity_name):
                if not request.allow_renaming:
                    response.result.result = response.NAME_NOT_UNIQUE
                    response.result.error_message = f"Entity '{entity_name}' already exists and allow_renaming is false"
                    return response
                # Generate a unique name
                base_name = entity_name
                suffix = 1
                while stage.GetPrimAtPath(f"{base_name}_{suffix}"):
                    suffix += 1
                entity_name = f"{base_name}_{suffix}"

            # Extract initial pose
            position = [0, 0, 0]
            orientation = [0, 0, 0, 1]  # w, x, y, z (quaternion)

            if hasattr(request, "initial_pose") and hasattr(request.initial_pose, "pose"):
                if hasattr(request.initial_pose.pose, "position"):
                    position = [
                        request.initial_pose.pose.position.x,
                        request.initial_pose.pose.position.y,
                        request.initial_pose.pose.position.z,
                    ]

                if hasattr(request.initial_pose.pose, "orientation"):
                    orientation = [
                        request.initial_pose.pose.orientation.w,
                        request.initial_pose.pose.orientation.x,
                        request.initial_pose.pose.orientation.y,
                        request.initial_pose.pose.orientation.z,
                    ]

            # Create the entity based on URI or create a new Xform
            if request.uri:
                # Check if the URI is accessible without opening the stage
                try:
                    result, _ = omni.client.stat(request.uri)
                    if result != omni.client.Result.OK:
                        response.result.result = response.RESOURCE_PARSE_ERROR
                        response.result.error_message = f"Cannot access USD file: {request.uri}"
                        return response

                except Exception as e:
                    response.result.result = response.RESOURCE_PARSE_ERROR
                    response.result.error_message = f"Failed to validate USD file: {e}"
                    return response

                # Load USD file as reference
                try:
                    # Create a reference
                    prim = stage.DefinePrim(entity_name)
                    prim.GetReferences().AddReference(request.uri)

                    try:
                        # Create XformPrim wrapper for the spawned entity
                        xform_prim = XformPrim(entity_name, reset_xform_op_properties=True)

                        # Set position and orientation
                        xform_prim.set_world_poses(positions=position, orientations=orientation)

                        carb.log_info(f"Set transform for {entity_name}")

                    except Exception as e:
                        carb.log_error(f"Error setting transform for {entity_name}: {e}")

                    carb.log_info(
                        f"Successfully spawned entity from URI: {entity_name} with reference to {request.uri}"
                    )
                except Exception as e:
                    response.result.result = response.RESOURCE_PARSE_ERROR
                    response.result.error_message = f"Failed to parse or load USD file: {e}"
                    return response
            else:
                # Create a new Xform prim
                stage.DefinePrim(entity_name, "Xform")

                # Use XformPrim to set the transform
                xform_prim = XformPrim(entity_name, reset_xform_op_properties=True)

                xform_prim.set_world_poses(positions=position, orientations=orientation)

                carb.log_info(f"Successfully spawned empty Xform entity: {entity_name}")

            # Track the spawned entities by adding an attribute to mark it as spawned via this service
            prim = stage.GetPrimAtPath(entity_name)
            attr1 = prim.CreateAttribute("simulationInterfacesSpawned", Sdf.ValueTypeNames.Bool, custom=True)
            attr1.Set(True)

            # Add namespace attribute if specified in the request
            if hasattr(request, "entity_namespace") and request.entity_namespace:
                try:
                    # Create and set the namespace attribute
                    attr = prim.CreateAttribute("isaac:namespace", Sdf.ValueTypeNames.String, custom=True)
                    result = attr.Set(request.entity_namespace)
                except Exception as e:
                    carb.log_error(f"Error setting namespace attribute: {e}")

            await omni.kit.app.get_app().next_update_async()

            # Set the response
            response.entity_name = entity_name
            response.result.result = Result.RESULT_OK
            response.result.error_message = ""

        except Exception as e:
            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error spawning entity: {e}"
            carb.log_error(f"Error in SpawnEntity service handler: {e}")

        return response

    async def _handle_reset_simulation(self, request, response):
        """Handle ResetSimulation service request

        This service resets the simulation environment to its initial state.
        Any dynamically spawned entities will be de-spawned after simulation is reset.

        Args:
            request: ResetSimulation request with scope of reset
            response: ResetSimulation response with result status

        Returns:
            response: Completed ResetSimulation response
        """

        try:
            import isaacsim.core.utils.prims as prim_utils
            from simulation_interfaces.msg import Result

            # Set the response result
            response.result = Result(result=Result.RESULT_OK, error_message="")

            # Ignore specific scope values - always use SCOPE_DEFAULT (full reset)
            carb.log_info("Resetting simulation with SCOPE_DEFAULT (full reset)")

            # Get usdrt stage for traversing to find spawned entities
            usdrt_stage = stage_utils.get_current_stage(fabric=True)
            if not usdrt_stage:
                response.result.result = Result.RESULT_OPERATION_FAILED
                response.result.error_message = "usdrt Stage not available for traversing"
                return response

            # First stop the simulation
            carb.log_info("Stopping simulation")
            self.timeline.stop()

            # Wait for one app update cycle to ensure stop is fully processed
            await omni.kit.app.get_app().next_update_async()

            # Find and remove all dynamically spawned entities
            carb.log_info("Removing dynamically spawned entities")

            # Find all prims with the simulationInterfacesSpawned attribute
            spawned_entities = []

            # Find spawned entities using usdrt stage traversal
            for prim in usdrt_stage.Traverse():
                if prim.HasAttribute("simulationInterfacesSpawned"):
                    attr = prim.GetAttribute("simulationInterfacesSpawned")
                    if attr and attr.Get():  # Check if the attribute value is True
                        spawned_entities.append(str(prim.GetPath()))

            # Delete the spawned entities
            for entity_path in spawned_entities:
                prim_utils.delete_prim(entity_path)
                carb.log_info(f"Removed spawned entity: {entity_path}")

            await omni.kit.app.get_app().next_update_async()

            # Start the timeline again
            self.timeline.play()
            carb.log_info("Simulation reset completed successfully")

        except Exception as e:
            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error resetting simulation: {e}"
            carb.log_error(f"Error in ResetSimulation service handler: {e}")

        return response

    async def _handle_step_simulation(self, request, response):
        """Handle StepSimulation service request

        This service steps the simulation forward by a specific number of frames,
        and then returns to a paused state. The simulation must be paused before
        stepping can be performed.

        Note: Single step (steps=1) is not currently supported. Please use 2 or more steps.

        Args:
            request: StepSimulation request with number of steps
            response: StepSimulation response with result

        Returns:
            response: Completed StepSimulation response
        """

        try:
            from simulation_interfaces.msg import Result

            # Initialize response
            response.result = Result()
            steps = request.steps

            # Check that steps is a positive integer
            if steps <= 0:
                response.result.result = Result.RESULT_OPERATION_FAILED
                response.result.error_message = "Steps must be a positive integer"
                return response

            # Check for single step request and warn that it's not supported
            if steps == 1:
                carb.log_warn(
                    "Single step (steps=1) is not supported in StepSimulation. Using steps=2 instead. Only step values greater than 1 are available."
                )
                steps = 2  # Override steps to 2

            # Ensure simulation is in paused state before stepping
            if self.timeline.is_playing():
                response.result.result = Result.RESULT_INCORRECT_STATE
                response.result.error_message = (
                    "Cannot step simulation while it is playing. Pause the simulation first."
                )
                return response

            # Get application instance
            app = omni.kit.app.get_app()
            self.timeline.play()
            self.timeline.commit()

            # Step through the requested number of frames
            for i in range(steps):

                # Wait for the frame to process
                await app.next_update_async()

                # Set successful response
                response.result.result = Result.RESULT_OK
                if request.steps == 1:
                    response.result.error_message = f"Successfully stepped simulation by {steps} frames. Note: steps=1 was automatically changed to steps=2 as only step values greater than 1 are available."
                else:
                    response.result.error_message = f"Successfully stepped simulation by {steps} frames."

            # Pause the simulation when done
            self.timeline.pause()
            self.timeline.commit()
            # Ensure the pause takes effect
            await app.next_update_async()
            await app.next_update_async()

        except Exception as e:
            # Ensure simulation is paused if an error occurs
            try:
                self.timeline.pause()
                self.timeline.commit()
                await app.next_update_async()
            except Exception:
                pass

            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error stepping simulation: {e}"

        return response

    async def _handle_get_entity_state(self, request, response):
        """Handle GetEntityState service request

        This service retrieves the state (pose, twist, acceleration) of a specific entity.
        If the entity has a rigid body API, only the pose is returned (REQ 6).
        If the entity doesn't have a rigid body API, both pose and velocity are returned.

        Args:
            request: GetEntityState request with entity path
            response: GetEntityState response with state information

        Returns:
            response: Completed GetEntityState response
        """

        try:
            from simulation_interfaces.msg import Result

            # Get entity state using external helper function
            entity_state, error, status_code = await get_entity_state(request.entity)

            if error:
                response.result = Result(result=status_code, error_message=error)
                return response

            # Set the state in the response
            response.state = entity_state

            # Set success response
            response.result = Result(result=Result.RESULT_OK, error_message="")
            carb.log_info(f"Successfully retrieved state for entity: {request.entity}")

        except Exception as e:
            response.result = Result(
                result=Result.RESULT_OPERATION_FAILED, error_message=f"Error getting entity state: {e}"
            )
            carb.log_error(f"Error in GetEntityState service handler: {e}")

        return response

    async def _handle_get_entities_states(self, request, response):
        """Handle GetEntitiesStates service request

        This service retrieves the states (pose, twist, acceleration) of multiple entities
        in the simulation, filtered by name using the partial prim path.
        It combines the functionality of GetEntities and GetEntityState.

        Args:
            request: GetEntitiesStates request with optional filters
            response: GetEntitiesStates response with entities and their states

        Returns:
            response: Completed GetEntitiesStates response
        """

        try:
            from simulation_interfaces.msg import Result

            # Initialize response lists
            response.entities = []
            response.states = []

            # Get usdrt stage for traversing
            usdrt_stage = stage_utils.get_current_stage(fabric=True)
            if not usdrt_stage:
                response.result = Result(
                    result=Result.RESULT_OPERATION_FAILED, error_message="usdrt Stage not available for traversing"
                )
                return response

            # Get filtered entities using external helper function
            filter_pattern = (
                request.filters.filter if hasattr(request, "filters") and hasattr(request.filters, "filter") else None
            )
            filtered_entities, error = get_filtered_entities(usdrt_stage, filter_pattern)

            if error:
                response.result = Result(result=Result.RESULT_OPERATION_FAILED, error_message=error)
                return response

            # Process each filtered entity to get its state
            for entity_path in filtered_entities:
                # Get entity state using external helper function
                entity_state, error, _ = await get_entity_state(entity_path)

                # Add to entities list regardless of state retrieval success
                response.entities.append(entity_path)

                # Add the entity state to the response (which might be None or a default state if there was an error)
                if entity_state:
                    response.states.append(entity_state)
                else:
                    # Create a default empty state if there was an error using the utility function
                    response.states.append(create_empty_entity_state())

            # Set success result
            response.result = Result(result=Result.RESULT_OK, error_message="")
            carb.log_info(f"GetEntitiesStates found and processed {len(response.entities)} entities")

        except Exception as e:
            response.result = Result(
                result=Result.RESULT_OPERATION_FAILED, error_message=f"Error getting entities states: {e}"
            )
            carb.log_error(f"Error in GetEntitiesStates service handler: {e}")

        return response

    async def _handle_set_entity_state(self, request, response):
        """Handle SetEntityState service request

        This service sets the state (pose, twist) of a specific entity in the simulation.
        It updates the position, orientation, and velocities (if applicable) of the specified prim.
        Note that velocity and acceleration features may have limited support depending on entity type.

        Args:
            request: SetEntityState request with entity name and desired state
            response: SetEntityState response with result status

        Returns:
            response: Completed SetEntityState response
        """

        try:
            import isaacsim.core.utils.prims as prim_utils
            import numpy as np
            from simulation_interfaces.msg import Result

            # Check if the entity exists
            if not prim_utils.is_prim_path_valid(request.entity):
                response.result = Result(
                    result=Result.RESULT_NOT_FOUND, error_message=f"Entity '{request.entity}' does not exist"
                )
                return response

            # Get the state from the request
            entity_state = request.state
            position = entity_state.pose.position
            orientation = entity_state.pose.orientation
            linear_velocity = entity_state.twist.linear
            angular_velocity = entity_state.twist.angular

            try:
                # Get regular stage for prim operations
                stage = stage_utils.get_current_stage()
                if not stage:
                    response.result = Result(result=Result.RESULT_OPERATION_FAILED, error_message="Stage not available")
                    return response

                prim = stage.GetPrimAtPath(request.entity)
                if not prim:
                    response.result = Result(
                        result=Result.RESULT_NOT_FOUND, error_message=f"Entity '{request.entity}' not found in stage"
                    )
                    return response

                # Check for PhysicsRigidBodyAPI
                applied_apis = prim.GetAppliedSchemas()
                has_rigid_body = "PhysicsRigidBodyAPI" in applied_apis

                velocity_message = ""

                if has_rigid_body:
                    # Use RigidPrim for rigid bodies - can set both pose and velocities
                    try:
                        # Initialize RigidPrim for the entity
                        rigid_prim = RigidPrim(paths=request.entity, reset_xform_op_properties=True)

                        # Set position and orientation
                        rigid_prim.set_world_poses(
                            positions=np.array([position.x, position.y, position.z]),
                            orientations=np.array([orientation.w, orientation.x, orientation.y, orientation.z]),
                        )

                        # Set linear and angular velocities using the combined method
                        linear_vel = np.array([linear_velocity.x, linear_velocity.y, linear_velocity.z])
                        angular_vel = np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z])
                        rigid_prim.set_velocities(linear_velocities=linear_vel, angular_velocities=angular_vel)

                        # Log based on timeline state
                        if not self.timeline.is_playing():
                            velocity_message = "Position, orientation, and velocities set while simulation is paused. Velocities will take effect when simulation resumes."
                            carb.log_info(
                                f"Set pose and velocities for rigid body '{request.entity}' while simulation is paused"
                            )
                        else:
                            velocity_message = "Position, orientation, and velocities set successfully."
                            carb.log_info(f"Set pose and velocities for rigid body '{request.entity}'")

                    except Exception as rigid_error:
                        response.result = Result(
                            result=Result.RESULT_OPERATION_FAILED,
                            error_message=f"Error setting rigid body state: {rigid_error}",
                        )
                        carb.log_error(f"Error setting rigid body state for '{request.entity}': {rigid_error}")
                        return response

                else:
                    # Non-rigid bodies - can only set pose
                    try:
                        xform_prim = XformPrim(request.entity, resolve_paths=False, reset_xform_op_properties=True)

                        # Set position and orientation
                        xform_prim.set_world_poses(
                            positions=[position.x, position.y, position.z],
                            orientations=[orientation.w, orientation.x, orientation.y, orientation.z],
                        )

                        velocity_message = "Entity doesn't have rigid body API, only position and orientation were set."
                        carb.log_info(f"Set pose for '{request.entity}'")

                    except Exception as xform_error:
                        response.result = Result(
                            result=Result.RESULT_OPERATION_FAILED,
                            error_message=f"Error setting transform: {xform_error}",
                        )
                        carb.log_error(f"Error setting transform for '{request.entity}': {xform_error}")
                        return response

                # Check acceleration values and add message if they are non-zero
                acceleration_message = ""
                if hasattr(entity_state, "acceleration"):
                    accel = entity_state.acceleration
                    if (
                        abs(accel.linear.x) > 0.001
                        or abs(accel.linear.y) > 0.001
                        or abs(accel.linear.z) > 0.001
                        or abs(accel.angular.x) > 0.001
                        or abs(accel.angular.y) > 0.001
                        or abs(accel.angular.z) > 0.001
                    ):
                        acceleration_message = " Setting accelerations is not supported in the current implementation."
                        carb.log_warn("Setting accelerations is not supported in SetEntityState service.")

                # Success - include appropriate messages
                response_message = f"Successfully set state for entity: {request.entity}."
                if velocity_message:
                    response_message += f" {velocity_message}"
                if acceleration_message:
                    response_message += acceleration_message

                response.result = Result(result=Result.RESULT_OK, error_message=response_message)
                carb.log_info(f"Successfully set state for entity: {request.entity}")

            except Exception as e:
                response.result = Result(
                    result=Result.RESULT_OPERATION_FAILED, error_message=f"Error setting entity state: {e}"
                )
                carb.log_error(f"Error in SetEntityState internal operation: {e}")

        except Exception as e:
            response.result = Result(
                result=Result.RESULT_OPERATION_FAILED, error_message=f"Error in SetEntityState service: {e}"
            )
            carb.log_error(f"Error in SetEntityState service handler: {e}")

        return response

    async def _handle_simulate_steps_action(self, goal_handle):
        """Handle SimulateSteps action request

        This action steps the simulation forward by a specific number of frames,
        and then returns to a paused state. The simulation must be paused before
        stepping can be performed. The action provides feedback after each step.

        Args:
            goal_handle: ROS2 action goal handle that contains the goal and methods
                         to publish feedback and set result

        Returns:
            result: Result message for the SimulateSteps action
        """

        try:
            from simulation_interfaces.action import SimulateSteps
            from simulation_interfaces.msg import Result

            # Create the feedback and result messages
            feedback_msg = SimulateSteps.Feedback()
            result_msg = SimulateSteps.Result()
            result_msg.result = Result()

            # Get the number of steps from the goal
            steps = goal_handle.request.steps

            # Check that steps is a positive integer
            if steps <= 0:
                result_msg.result.result = Result.RESULT_OPERATION_FAILED
                result_msg.result.error_message = "Steps must be a positive integer"
                goal_handle.abort()
                return result_msg

            # Check for single step request and warn that it's not supported
            if steps == 1:
                carb.log_warn(
                    "Single step (steps=1) is not supported in MultiStepSimulation. Using steps=2 instead. Only step values greater than 1 are available."
                )
                steps = 2  # Override steps to 2

            # Ensure simulation is in paused state before stepping
            if self.timeline.is_playing():
                result_msg.result.result = Result.RESULT_INCORRECT_STATE
                result_msg.result.error_message = (
                    "Cannot step simulation while it is playing. Pause the simulation first."
                )
                goal_handle.abort()
                return result_msg

            # Get application instance
            app = omni.kit.app.get_app()
            self.timeline.play()

            # Initialize feedback
            feedback_msg.completed_steps = 0
            feedback_msg.remaining_steps = steps

            # Step through the requested number of frames
            for i in range(steps):
                # Check if goal has been canceled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    # Pause the simulation when canceled
                    self.timeline.pause()
                    self.timeline.commit()
                    await app.next_update_async()
                    await app.next_update_async()

                    result_msg.result.result = Result.RESULT_OPERATION_FAILED
                    result_msg.result.error_message = "Simulation stepping was canceled"
                    return result_msg

                # Commit the timeline to ensure changes are applied
                self.timeline.commit()

                # Wait for the frame to process
                await app.next_update_async()

                # Update feedback
                feedback_msg.completed_steps = i + 1
                feedback_msg.remaining_steps = steps - (i + 1)

                # Publish feedback
                goal_handle.publish_feedback(feedback_msg)

                carb.log_info(f"Completed step {i+1}/{steps}")

            # Pause the simulation when done
            self.timeline.pause()
            self.timeline.commit()

            # Ensure the pause takes effect
            await app.next_update_async()
            await app.next_update_async()

            # Set successful result
            result_msg.result.result = Result.RESULT_OK
            if goal_handle.request.steps == 1:
                result_msg.result.error_message = f"Successfully stepped simulation by {steps} frames. Note: steps=1 was automatically changed to steps=2 as only step values greater than 1 are available."
            else:
                result_msg.result.error_message = f"Successfully stepped simulation by {steps} frames"
            goal_handle.succeed()

        except Exception as e:
            # Ensure simulation is paused if an error occurs
            try:
                self.timeline.pause()
                self.timeline.commit()
                await app.next_update_async()
                await app.next_update_async()
            except Exception:
                pass

            result_msg.result.result = Result.RESULT_OPERATION_FAILED
            result_msg.result.error_message = f"Error stepping simulation: {e}"
            goal_handle.abort()

        return result_msg

    async def _handle_get_simulator_features(self, request, response):
        """Handle GetSimulatorFeatures service request

        This service lists the subset of services and actions supported by Isaac Sim
        from simulation_interfaces, including brief descriptions of workflows for each interface.

        Args:
            request: GetSimulatorFeatures request (empty)
            response: GetSimulatorFeatures response with list of supported features

        Returns:
            response: Completed GetSimulatorFeatures response
        """
        try:
            from simulation_interfaces.msg import SimulatorFeatures

            # Define the features supported by our implementation
            features = [
                SimulatorFeatures.SPAWNING,  # Supports SpawnEntity
                SimulatorFeatures.DELETING,  # Supports DeleteEntity
                SimulatorFeatures.ENTITY_STATE_GETTING,  # Supports GetEntityState
                SimulatorFeatures.ENTITY_STATE_SETTING,  # Supports SetEntityState
                SimulatorFeatures.ENTITY_INFO_GETTING,  # Supports GetEntityInfo
                SimulatorFeatures.SIMULATION_RESET,  # Supports ResetSimulation
                SimulatorFeatures.SIMULATION_RESET_SPAWNED,  # Supports SCOPE_SPAWNED reset
                SimulatorFeatures.SIMULATION_STATE_GETTING,  # Supports GetSimulationState
                SimulatorFeatures.SIMULATION_STATE_SETTING,  # Supports SetSimulationState
                SimulatorFeatures.SIMULATION_STATE_PAUSE,  # Supports pausing simulation
                # SimulatorFeatures.STEP_SIMULATION_SINGLE,    # Supports single stepping
                SimulatorFeatures.STEP_SIMULATION_MULTIPLE,  # Supports multi-stepping
                SimulatorFeatures.STEP_SIMULATION_ACTION,  # Supports SimulateSteps action
            ]

            # Set the features in the response
            response.features.features = features

            # Set supported spawn formats
            response.features.spawn_formats = ["usd"]

            # Set custom info with version and description
            response.features.custom_info = "Control Isaac Sim via ROS2 Simulation Interfaces."

            carb.log_info("Successfully responded to GetSimulatorFeatures request")

        except Exception as e:
            carb.log_error(f"Error in GetSimulatorFeatures service handler: {e}")
            # There's no result field in GetSimulatorFeatures response, so we can only log the error

        return response

    def shutdown(self):
        """Shutdown the simulation control services"""
        if self.service_manager:
            self.service_manager.shutdown()


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.sim_control = SimulationControl()

    def on_shutdown(self):
        if self.sim_control:
            self.sim_control.shutdown()
