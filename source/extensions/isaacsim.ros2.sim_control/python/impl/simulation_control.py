# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""ROS 2 simulation control services and action servers."""

import asyncio
import os
import threading

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import nest_asyncio
import numpy as np
import omni
import omni.timeline
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from isaacsim.storage.native import (
    find_filtered_files_async,
    get_assets_root_path_async,
    is_local_path,
    is_valid_usd_file,
    resolve_asset_path_async,
)
from pxr import Sdf
from pxr import Usd as PxrUsd
from pxr import UsdGeom
from usdrt import Usd

from .entity_utils import create_empty_entity_state, get_entity_state, get_filtered_entities, resolve_source_path

# Service prefix constant
SERVICE_PREFIX = ""  # Prefix for all ROS2 services (empty by default)

# ROS2 Interface Configuration
# Format: (module_name, class_name, endpoint_name)
# Example: ("simulation_interfaces.srv", "GetSimulationState", "get_simulation_state")
#          imports GetSimulationState from simulation_interfaces.srv
#          and registers it as ROS2 service "/get_simulation_state"
SERVICE_TYPES = [
    ("simulation_interfaces.srv", "GetSimulationState", "get_simulation_state"),
    ("simulation_interfaces.srv", "SetSimulationState", "set_simulation_state"),
    ("simulation_interfaces.srv", "GetEntities", "get_entities"),
    ("simulation_interfaces.srv", "DeleteEntity", "delete_entity"),
    ("simulation_interfaces.srv", "GetEntityInfo", "get_entity_info"),
    ("simulation_interfaces.srv", "SpawnEntity", "spawn_entity"),
    ("simulation_interfaces.srv", "SpawnEntities", "spawn_entities"),
    ("simulation_interfaces.srv", "ResetSimulation", "reset_simulation"),
    ("simulation_interfaces.srv", "StepSimulation", "step_simulation"),
    ("simulation_interfaces.srv", "GetEntityState", "get_entity_state"),
    ("simulation_interfaces.srv", "GetEntitiesStates", "get_entities_states"),
    ("simulation_interfaces.srv", "SetEntityState", "set_entity_state"),
    ("simulation_interfaces.srv", "GetEntityBounds", "get_entity_bounds"),
    ("simulation_interfaces.srv", "GetSimulatorFeatures", "get_simulator_features"),
    ("simulation_interfaces.srv", "GetSpawnables", "get_spawnables"),
    ("simulation_interfaces.srv", "LoadWorld", "load_world"),
    ("simulation_interfaces.srv", "UnloadWorld", "unload_world"),
    ("simulation_interfaces.srv", "GetCurrentWorld", "get_current_world"),
    ("simulation_interfaces.srv", "GetAvailableWorlds", "get_available_worlds"),
]

ACTION_TYPES = [
    ("simulation_interfaces.action", "SimulateSteps", "simulate_steps"),
]


# Define the Singleton decorator
def Singleton(class_: type) -> callable:  # noqa: N802 - decorator named after the pattern it implements
    """Decorate a class to implement the singleton pattern."""
    instances = {}

    def getinstance(*args: object, **kwargs: object) -> object:
        if class_ not in instances:
            instances[class_] = class_(*args, **kwargs)
        return instances[class_]

    return getinstance


@Singleton
class ROS2ServiceManager:
    """Manager for ROS2 services that can control the Isaac Sim simulation.

    This class is a singleton to ensure there's only one ROS2 node running
    that handles all simulation control services.

    Sets up the initial state for managing ROS2 services and action servers
    for Isaac Sim simulation control.
    """

    def __init__(self) -> None:
        """Initialize the ROS2 service manager."""
        self.node_name = "isaac_sim_control"
        self.node = None
        self.services = {}
        self.action_servers = {}
        self.is_initialized = False
        self.executor = None
        self.executor_thread = None
        self.loop = None
        # Single callback group for parallel execution of both services and actions
        self.callback_group = None

    def initialize(self) -> None:
        """Initialize the ROS2 node for simulation control services.

        Creates a ROS2 node and starts a separate thread for spinning the node.
        This method is idempotent - calling it multiple times has no effect
        if already initialized.

        Raises:
            ImportError: If ROS2 Python libraries are not available.

        """
        if self.is_initialized:
            return

        try:
            import rclpy
            from rclpy.callback_groups import ReentrantCallbackGroup
            from rclpy.executors import MultiThreadedExecutor

            # Initialize ROS2 if it's not already initialized
            try:
                rclpy.init()
            except RuntimeError:
                # ROS2 is already initialized
                pass

            self.node = rclpy.create_node(self.node_name)

            self.callback_group = ReentrantCallbackGroup()

            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)

            self.is_initialized = True

            self.loop = asyncio.get_event_loop()

            nest_asyncio.apply(self.loop)

            # Start a separate thread for ROS2 spinning with the multithreaded executor
            self.executor_thread = threading.Thread(target=self._spin)
            self.executor_thread.daemon = True
            self.executor_thread.start()

            carb.log_info(
                f"ROS2 ServiceManager initialized with node '{self.node_name}' using single callback group for parallel execution"
            )

        except ImportError as e:
            carb.log_error(f"Failed to import ROS2 Python libraries: {e}")
            self.is_initialized = False
        except Exception as e:
            carb.log_error(f"Error initializing ROS2 ServiceManager: {e}")
            self.is_initialized = False

    def shutdown(self) -> None:
        """Shutdown the ROS2 node and clean up resources.

        Stops the spinning thread, destroys all registered services and action servers,
        and shuts down the ROS2 node. This method is safe to call multiple times.
        """
        if not self.is_initialized:
            return

        import rclpy

        if self.executor:
            self.executor.shutdown()

        # Wait for the executor thread to finish
        if self.executor_thread and self.executor_thread.is_alive():
            self.executor_thread.join(timeout=1.0)

        # Unregister all services using the proper unregister methods
        for service_name in self.services:
            self.unregister_service(service_name, remove_from_dict=False)
        self.services.clear()

        # Unregister all action servers using the proper unregister methods
        for action_name in self.action_servers:
            self.unregister_action_server(action_name, remove_from_dict=False)
        self.action_servers.clear()

        if self.node:
            self.node.destroy_node()
            self.node = None

        self.callback_group = None
        self.executor = None

        try:
            rclpy.shutdown()
        except Exception:
            pass

        self.is_initialized = False
        carb.log_info("ROS2 ServiceManager shutdown completed")

    def register_service(self, service_name: str, service_type: object, callback: callable) -> bool:
        """Register a new ROS2 service.

        Args:
            service_name: Name of the service
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
            # Create service with callback group for parallel execution
            service = self.node.create_service(
                service_type, service_name, self._wrap_async_callback(callback), callback_group=self.callback_group
            )
            self.services[service_name] = service
            carb.log_info(f"Registered ROS2 service: {service_name} with parallel callback execution")
            return True
        except Exception as e:
            carb.log_error(f"Failed to register service '{service_name}': {e}")
            return False

    def unregister_service(self, service_name: str, remove_from_dict: bool = True) -> bool:
        """Unregister a ROS2 service.

        Cleanly destroys a ROS2 service and optionally removes it from the internal
        services registry. This method provides proper error handling and logging
        for service cleanup operations.

        Args:
            service_name: Name of the service to unregister.
            remove_from_dict: Whether to remove the service from the internal services
                dictionary. Set to False when iterating over services during bulk
                cleanup operations like shutdown to avoid modifying dictionary during
                iteration. Defaults to True for individual service cleanup.

        Returns:
            bool: True if unregistration was successful, False otherwise.
            Returns False if the service manager is not initialized or the service
            does not exist.

        Example:

        .. code-block:: python

            # Individual service cleanup (normal usage)
            >>> success = service_manager.unregister_service("my_service")
            >>> success
            True

            # Bulk cleanup during iteration (shutdown scenario)
            >>> for name in service_manager.services:
            ...     service_manager.unregister_service(name, remove_from_dict=False)
            >>> service_manager.services.clear()

        """
        if not self.is_initialized or service_name not in self.services:
            return False

        try:
            if remove_from_dict:
                service = self.services.pop(service_name)
            else:
                service = self.services[service_name]
            self.node.destroy_service(service)
            carb.log_info(f"Unregistered ROS2 service: {service_name}")
            return True
        except Exception as e:
            carb.log_error(f"Failed to unregister service '{service_name}': {e}")
            return False

    def register_action_server(
        self,
        action_name: str,
        action_type: object,
        execute_callback: callable,
        goal_callback: callable = None,
        cancel_callback: callable = None,
    ) -> bool:
        """Register a new ROS2 action server.

        Args:
            action_name: Name of the action
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
            from rclpy.action import ActionServer

            # Create the action server with callback group for parallel execution
            action_server = ActionServer(
                node=self.node,
                action_type=action_type,
                action_name=action_name,
                execute_callback=self._wrap_async_callback(execute_callback),
                goal_callback=goal_callback,
                cancel_callback=cancel_callback,
                callback_group=self.callback_group,
            )

            self.action_servers[action_name] = action_server
            carb.log_info(f"Registered ROS2 action server: {action_name} with parallel callback execution")
            return True

        except Exception as e:
            carb.log_error(f"Failed to register action server '{action_name}': {e}")
            return False

    def _wrap_async_callback(self, async_callback: callable) -> callable:
        """Wrap any async callback to work with ROS2 services and actions.

        Args:
            async_callback: Async callback function

        Returns:
            callable: Wrapped callback that handles the event loop

        """

        def wrapper(*args: object, **kwargs: object) -> object:
            if not self.loop:
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)
            future = asyncio.run_coroutine_threadsafe(async_callback(*args, **kwargs), self.loop)
            return future.result()

        return wrapper

    def unregister_action_server(self, action_name: str, remove_from_dict: bool = True) -> bool:
        """Unregister a ROS2 action server.

        Cleanly destroys a ROS2 action server and optionally removes it from the internal
        action servers registry. This method provides proper error handling and logging
        for action server cleanup operations.

        Args:
            action_name: Name of the action server to unregister.
            remove_from_dict: Whether to remove the action server from the internal
                action_servers dictionary. Set to False when iterating over action servers
                during bulk cleanup operations like shutdown to avoid modifying dictionary
                during iteration. Defaults to True for individual action server cleanup.

        Returns:
            bool: True if unregistration was successful, False otherwise.
            Returns False if the service manager is not initialized or the action server
            does not exist.

        Example:

        .. code-block:: python

            # Individual action server cleanup (normal usage)
            >>> success = service_manager.unregister_action_server("my_action")
            >>> success
            True

            # Bulk cleanup during iteration (shutdown scenario)
            >>> for name in service_manager.action_servers:
            ...     service_manager.unregister_action_server(name, remove_from_dict=False)
            >>> service_manager.action_servers.clear()

        """
        if not self.is_initialized or action_name not in self.action_servers:
            return False

        try:
            if remove_from_dict:
                action_server = self.action_servers.pop(action_name)
            else:
                action_server = self.action_servers[action_name]
            action_server.destroy()
            carb.log_info(f"Unregistered ROS2 action server: {action_name}")
            return True
        except Exception as e:
            carb.log_error(f"Failed to unregister action server '{action_name}': {e}")
            return False

    def _spin(self) -> None:
        """Spin the ROS2 executor in a separate thread.

        This method runs the multithreaded executor which enables parallel
        callback execution. The executor will run until shutdown() is called.
        """
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            carb.log_error(f"Error in executor thread: {e}")
        finally:
            carb.log_info("ROS2 executor thread stopping")


class SimulationControl:
    """Controller for Isaac Sim simulation via ROS2 services.

    Sets up the timeline interface, ROS2 service manager, and imports
    all available simulation interface service and action types.
    Services are registered automatically during initialization.
    """

    def _import_interfaces(self, interface_types: list, interface_kind: str) -> None:
        """Import ROS2 interfaces with graceful fallback.

        Args:
            interface_types: List of tuples (module_name, class_name, endpoint_name)
            interface_kind: String describing interface type ("service" or "action")

        """
        for module_name, class_name, endpoint_name in interface_types:
            try:
                module = __import__(module_name, fromlist=[class_name])
                interface_class = getattr(module, class_name)
                setattr(self, class_name, interface_class)
                setattr(self, f"{class_name}_{interface_kind}_name", endpoint_name)
                carb.log_info(f"Successfully imported {interface_kind} {class_name}")
            except (ImportError, AttributeError) as e:
                carb.log_warn(
                    f"{interface_kind.capitalize()} {class_name} not available in this "
                    f"simulation_interfaces release; skipping registration ({e})"
                )
                setattr(self, class_name, None)
                setattr(self, f"{class_name}_{interface_kind}_name", None)

    def __init__(self) -> None:
        """Initialize the simulation control with ROS2 services and actions."""
        self.timeline = omni.timeline.get_timeline_interface()
        self.service_manager = ROS2ServiceManager()
        self.is_initialized = False

        # Import interfaces using helper method
        self._import_interfaces(SERVICE_TYPES, "service")
        self._import_interfaces(ACTION_TYPES, "action")

        # Initialize and register services
        self._initialize_ros2_services()

    def _register_service_if_available(self, service_class_name: str, handler: callable) -> None:
        """Register a service if its type is available.

        Args:
            service_class_name: Name of the service class attribute (e.g., 'GetSimulationState').
            handler: The handler function for the service.

        """
        if hasattr(self, service_class_name):
            service_class = getattr(self, service_class_name)
            if service_class:
                service_name = getattr(self, f"{service_class_name}_service_name")
                self.service_manager.register_service(f"{SERVICE_PREFIX}{service_name}", service_class, handler)
            else:
                carb.log_warn(
                    f"{service_class_name} service type not available in this "
                    "simulation_interfaces release; skipping registration"
                )

    def _register_action_server_if_available(
        self,
        action_class_name: str,
        execute_handler: callable,
        goal_callback: callable = None,
        cancel_callback: callable = None,
    ) -> None:
        """Register an action server if its type is available.

        Args:
            action_class_name: Name of the action class attribute (e.g., 'SimulateSteps').
            execute_handler: The execute callback function for the action server.
            goal_callback: Callback to accept/reject goals (optional).
            cancel_callback: Callback to handle cancellation (optional).

        """
        if hasattr(self, action_class_name):
            action_class = getattr(self, action_class_name)
            if action_class:
                action_name = getattr(self, f"{action_class_name}_action_name")
                self.service_manager.register_action_server(
                    f"{SERVICE_PREFIX}{action_name}",
                    action_class,
                    execute_handler,
                    goal_callback=goal_callback,
                    cancel_callback=cancel_callback,
                )
            else:
                carb.log_warn(
                    f"{action_class_name} action type not available in this "
                    "simulation_interfaces release; skipping registration"
                )

    def _initialize_ros2_services(self) -> None:
        """Initialize ROS2 services for simulation control.

        Registers all available simulation interface services and action servers
        with the ROS2 service manager. Only services with successfully imported
        types will be registered.

        Raises:
            ImportError: If required ROS2 message types cannot be imported.

        """
        try:
            # Initialize the ROS2 service manager
            self.service_manager.initialize()

            if not self.service_manager.is_initialized:
                carb.log_error("Failed to initialize ROS2 service manager")
                return

            # Register services using helper methods
            self._register_service_if_available("GetSimulationState", self._handle_get_simulation_state)
            self._register_service_if_available("SetSimulationState", self._handle_set_simulation_state)
            self._register_service_if_available("GetEntities", self._handle_get_entities)
            self._register_service_if_available("DeleteEntity", self._handle_delete_entity)
            self._register_service_if_available("GetEntityInfo", self._handle_get_entity_info)
            self._register_service_if_available("SpawnEntity", self._handle_spawn_entity)
            self._register_service_if_available("SpawnEntities", self._handle_spawn_entities)
            self._register_service_if_available("ResetSimulation", self._handle_reset_simulation)
            self._register_service_if_available("StepSimulation", self._handle_step_simulation)
            self._register_service_if_available("GetEntityState", self._handle_get_entity_state)
            self._register_service_if_available("GetEntitiesStates", self._handle_get_entities_states)
            self._register_service_if_available("SetEntityState", self._handle_set_entity_state)
            self._register_service_if_available("GetEntityBounds", self._handle_get_entity_bounds)
            self._register_service_if_available("GetSimulatorFeatures", self._handle_get_simulator_features)
            self._register_service_if_available("GetSpawnables", self._handle_get_spawnables)
            self._register_service_if_available("LoadWorld", self._handle_load_world)
            self._register_service_if_available("UnloadWorld", self._handle_unload_world)
            self._register_service_if_available("GetCurrentWorld", self._handle_get_current_world)
            self._register_service_if_available("GetAvailableWorlds", self._handle_get_available_worlds)

            # Register action servers using helper method
            self._register_action_server_if_available(
                "SimulateSteps",
                self._handle_simulate_steps_action,
                goal_callback=None,
                cancel_callback=self._handle_simulate_steps_cancel_callback,
            )

            self.is_initialized = True
            carb.log_info("ROS 2 Simulation Control services initialized")

        except ImportError as e:
            carb.log_error(f"Failed to import required ROS2 types: {e}")
            self.is_initialized = False
        except Exception as e:
            carb.log_error(f"Error initializing ROS2 services: {e}")
            self.is_initialized = False

    async def _handle_get_simulation_state(self, request: object, response: object) -> object:
        """Handle simulation state query request.

        Args:
            request: GetSimulationState request.
            response: GetSimulationState response.

        Returns:
            object: Completed GetSimulationState response.

        """
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

    async def _handle_set_simulation_state(self, request: object, response: object) -> object:
        """Handle request to set simulation state.

        Args:
            request: SetSimulationState request with target state.
            response: SetSimulationState response.

        Returns:
            object: Completed SetSimulationState response.

        """
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

    async def _handle_get_entities(self, request: object, response: object) -> object:
        """Handle GetEntities service request.

        This service returns a list of entities (prims) in the simulation,
        optionally filtered by name using the partial prim path.

        Args:
            request: GetEntities request with optional filters
            response: GetEntities response with entities list and result

        Returns:
            object: Completed GetEntities response

        """
        try:
            from simulation_interfaces.msg import Result

            # Get the results ready
            response.entities = []
            response.result.result = Result.RESULT_OK
            response.result.error_message = ""

            # Get usdrt stage for traversing
            usdrt_stage = stage_utils.get_current_stage(backend="fabric")
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

    async def _handle_delete_entity(self, request: object, response: object) -> object:
        """Handle DeleteEntity service request.

        This service deletes a specified entity (prim) from the simulation if it exists
        and is not protected from deletion.

        Args:
            request: DeleteEntity request with entity prim path
            response: DeleteEntity response with result status

        Returns:
            object: Completed DeleteEntity response

        """
        try:
            from simulation_interfaces.msg import Result

            # First check if the entity exists
            prim = prim_utils.get_prim_at_path(request.entity)
            if not prim.IsValid():
                response.result = Result(
                    result=Result.RESULT_NOT_FOUND,
                    error_message=f"Entity '{request.entity}' does not exist",
                )
                return response

            if prim.GetMetadata("no_delete"):
                response.result = Result(
                    result=Result.RESULT_OPERATION_FAILED,
                    error_message=f"Entity '{request.entity}' is protected and cannot be deleted",
                )
                return response

            if stage_utils.delete_prim(request.entity):
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

    async def _handle_get_entity_info(self, request: object, response: object) -> object:
        """Handle GetEntityInfo service request.

        This service provides detailed information about a specific entity in the simulation.
        Currently, all entities are classified as OBJECT category this is a placeholder for future use.

        Args:
            request: GetEntityInfo request containing the entity prim path
            response: GetEntityInfo response with entity information

        Returns:
            object: Completed GetEntityInfo response

        """
        try:
            from simulation_interfaces.msg import EntityCategory, EntityInfo, Result

            if not prim_utils.get_prim_at_path(request.entity).IsValid():
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

    async def _handle_spawn_entity(self, request: object, response: object) -> object:
        """Handle SpawnEntity service request.

        Deprecated in favour of SpawnEntities but kept for backwards compatibility.
        Delegates to _spawn_single_entity_core.

        Args:
            request: SpawnEntity request with entity name, URI, and initial pose
            response: SpawnEntity response with result status

        Returns:
            object: Completed SpawnEntity response

        """
        try:
            from simulation_interfaces.msg import Result

            result_code, error_msg, entity_name = await self._spawn_single_entity_core(
                request.name,
                request.allow_renaming,
                request.uri,
                getattr(request, "resource_string", ""),
                getattr(request, "entity_namespace", ""),
                request.initial_pose,
            )

            response.result.result = result_code
            response.result.error_message = error_msg
            if result_code == Result.RESULT_OK:
                response.entity_name = entity_name

        except Exception as e:
            from simulation_interfaces.msg import Result

            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error spawning entity: {e}"
            carb.log_error(f"Error in SpawnEntity service handler: {e}")

        return response

    async def _spawn_single_entity_core(
        self,
        name: str,
        allow_renaming: bool,
        uri: str,
        resource_string: str,
        entity_namespace: str,
        initial_pose: object,
    ) -> tuple:
        """Spawn a single entity and return (result_code, error_message, entity_name).

        Shared spawn logic used by both SpawnEntity and SpawnEntities handlers.

        Args:
            name: Desired entity name.
            allow_renaming: Whether to allow auto-renaming when name conflicts.
            uri: URI of the USD asset to load. Empty string creates an empty Xform.
            resource_string: Inline resource string (not supported by Isaac Sim, ignored).
            entity_namespace: Optional namespace to attach to the spawned entity.
            initial_pose: geometry_msgs/PoseStamped specifying the initial pose.

        Returns:
            tuple: (result_code, error_message, entity_name).
                   result_code is Result.RESULT_OK (1) on success, or a service-specific
                   error code (from SpawnResult) / Result error code on failure.

        """
        from simulation_interfaces.msg import Result, SpawnResult

        # Validate and resolve URI when provided
        path_to_load = None
        if uri:
            if not is_valid_usd_file(uri, []):
                return (
                    SpawnResult.UNSUPPORTED_FORMAT,
                    f"Unsupported format. Only USD files (.usd, .usda, .usdc, .usdz) are supported. Got: {uri}",
                    "",
                )
            path_to_load = await resolve_asset_path_async(uri)
            if not path_to_load:
                return (
                    SpawnResult.RESOURCE_PARSE_ERROR,
                    f"Could not find path '{uri}' or default asset root based path",
                    "",
                )

        stage = stage_utils.get_current_stage()
        if not stage:
            return (Result.RESULT_OPERATION_FAILED, "Stage not available", "")

        # Determine entity name, optionally reading USD default prim
        entity_name = name
        if not entity_name and path_to_load:
            try:
                temp_stage = Usd.Stage.Open(path_to_load)
                if temp_stage:
                    default_prim = temp_stage.GetDefaultPrim()
                    if default_prim:
                        entity_name = default_prim.GetName()
                        carb.log_info(f"Using default prim name from USD: {entity_name}")
            except Exception as e:
                carb.log_warn(f"Could not extract default prim name from USD: {e}")

        if not entity_name:
            if not allow_renaming:
                return (
                    SpawnResult.NAME_INVALID,
                    "Entity name is empty, no default prim found, and allow_renaming is false",
                    "",
                )
            spawned_count = 0
            usdrt_stage = stage_utils.get_current_stage(backend="fabric")
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
            entity_name = f"/{entity_name}"
            carb.log_info(f"Using entity name as is: {entity_name}")

        # Enforce name uniqueness
        if prim_utils.get_prim_at_path(entity_name).IsValid():
            if not allow_renaming:
                return (
                    SpawnResult.NAME_NOT_UNIQUE,
                    f"Entity '{entity_name}' already exists and allow_renaming is false",
                    "",
                )
            base_name = entity_name
            suffix = 1
            while prim_utils.get_prim_at_path(f"{base_name}_{suffix}").IsValid():
                suffix += 1
            entity_name = f"{base_name}_{suffix}"

        # Extract initial pose
        position = [0, 0, 0]
        orientation = [0, 0, 0, 1]  # w, x, y, z
        if hasattr(initial_pose, "pose"):
            if hasattr(initial_pose.pose, "position"):
                position = [
                    initial_pose.pose.position.x,
                    initial_pose.pose.position.y,
                    initial_pose.pose.position.z,
                ]
            if hasattr(initial_pose.pose, "orientation"):
                orientation = [
                    initial_pose.pose.orientation.w,
                    initial_pose.pose.orientation.x,
                    initial_pose.pose.orientation.y,
                    initial_pose.pose.orientation.z,
                ]

        # Create the entity from URI reference or as an empty Xform
        if path_to_load:
            try:
                prim = stage.DefinePrim(entity_name)
                prim.GetReferences().AddReference(path_to_load)
                try:
                    xform_prim = XformPrim(entity_name, reset_xform_op_properties=True)
                    xform_prim.set_world_poses(positions=position, orientations=orientation)
                    carb.log_info(f"Set transform for {entity_name}")
                except Exception as e:
                    carb.log_error(f"Error setting transform for {entity_name}: {e}")
                carb.log_info(f"Successfully spawned entity from URI: {entity_name} with reference to {path_to_load}")
            except Exception as e:
                return (SpawnResult.RESOURCE_PARSE_ERROR, f"Failed to parse or load USD file: {e}", "")
        else:
            stage.DefinePrim(entity_name, "Xform")
            xform_prim = XformPrim(entity_name, reset_xform_op_properties=True)
            xform_prim.set_world_poses(positions=position, orientations=orientation)
            carb.log_info(f"Successfully spawned empty Xform entity: {entity_name}")

        # Mark the prim as simulation-interfaces-spawned for reset tracking
        prim = prim_utils.get_prim_at_path(entity_name)
        attr1 = prim.CreateAttribute("simulationInterfacesSpawned", Sdf.ValueTypeNames.Bool, custom=True)
        attr1.Set(True)

        if entity_namespace:
            try:
                attr = prim.CreateAttribute("isaac:namespace", Sdf.ValueTypeNames.String, custom=True)
                attr.Set(entity_namespace)
            except Exception as e:
                carb.log_error(f"Error setting namespace attribute: {e}")

        await omni.kit.app.get_app().next_update_async()
        return (Result.RESULT_OK, "", entity_name)

    async def _handle_spawn_entities(self, request: object, response: object) -> object:
        """Handle SpawnEntities service request.

        Spawns multiple entities in a single call. Each request in spawn_requests is
        processed independently using the SpawnEntity.msg format, which carries the
        resource as an entity_resource (Resource) field instead of flat uri/resource_string
        fields. Individual failures are reported per-entry in the results list.

        Available in simulation_interfaces >= 1.4.0 (humble) and >= 1.5.0 (jazzy).

        Args:
            request: SpawnEntities request with list of SpawnEntity msg spawn requests.
            response: SpawnEntities response with aggregate result and per-entity SpawnResult list.

        Returns:
            object: Completed SpawnEntities response.

        """
        try:
            from simulation_interfaces.msg import Result, SpawnResult

            response.results = []
            any_failed = False

            for spawn_req in request.spawn_requests:
                uri = spawn_req.entity_resource.uri if hasattr(spawn_req, "entity_resource") else ""
                resource_string = (
                    spawn_req.entity_resource.resource_string if hasattr(spawn_req, "entity_resource") else ""
                )

                try:
                    result_code, error_msg, entity_name = await self._spawn_single_entity_core(
                        spawn_req.name,
                        spawn_req.allow_renaming,
                        uri,
                        resource_string,
                        getattr(spawn_req, "entity_namespace", ""),
                        spawn_req.initial_pose,
                    )
                except Exception as inner_e:
                    result_code = Result.RESULT_OPERATION_FAILED
                    error_msg = f"Unexpected error spawning entity: {inner_e}"
                    entity_name = ""
                    carb.log_error(f"Error spawning entity '{spawn_req.name}': {inner_e}")

                spawn_result = SpawnResult()
                spawn_result.result.result = result_code
                spawn_result.result.error_message = error_msg
                spawn_result.entity_name = entity_name
                response.results.append(spawn_result)

                if result_code != Result.RESULT_OK:
                    any_failed = True

            if any_failed:
                from simulation_interfaces.srv import SpawnEntities

                response.result.result = SpawnEntities.Response.ENTITIES_SPAWN_FAILED
                response.result.error_message = "One or more entity spawns failed; check individual results"
            else:
                response.result.result = Result.RESULT_OK
                response.result.error_message = ""

        except Exception as e:
            from simulation_interfaces.msg import Result

            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error in SpawnEntities service: {e}"
            carb.log_error(f"Error in SpawnEntities service handler: {e}")

        return response

    async def _handle_get_entity_bounds(self, request: object, response: object) -> object:
        """Handle GetEntityBounds service request.

        Computes the axis-aligned world bounding box for an entity using
        UsdGeom.BBoxCache and returns it as a TYPE_BOX Bounds message with
        [min_corner, max_corner] points.

        Args:
            request: GetEntityBounds request with entity name.
            response: GetEntityBounds response with Bounds and result.

        Returns:
            object: Completed GetEntityBounds response.

        """
        try:
            from geometry_msgs.msg import Vector3
            from simulation_interfaces.msg import Bounds, Result

            # prim_utils.get_prim_at_path returns a pxr.Usd.Prim, compatible with BBoxCache
            prim = prim_utils.get_prim_at_path(request.entity)
            if not prim.IsValid():
                response.result = Result(
                    result=Result.RESULT_NOT_FOUND,
                    error_message=f"Entity '{request.entity}' does not exist",
                )
                return response

            bbox_cache = UsdGeom.BBoxCache(
                time=PxrUsd.TimeCode.Default(),
                includedPurposes=[UsdGeom.Tokens.default_],
                useExtentsHint=True,
            )
            # ComputeAlignedRange() returns a Gf.Range3d with GetMin()/GetMax()
            aligned_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            min_pt = aligned_range.GetMin()
            max_pt = aligned_range.GetMax()

            bounds = Bounds()
            bounds.type = Bounds.TYPE_BOX
            bounds.points = [
                Vector3(x=float(min_pt[0]), y=float(min_pt[1]), z=float(min_pt[2])),
                Vector3(x=float(max_pt[0]), y=float(max_pt[1]), z=float(max_pt[2])),
            ]

            response.bounds = bounds
            response.result = Result(result=Result.RESULT_OK, error_message="")
            carb.log_info(f"Successfully retrieved bounds for entity: {request.entity}")

        except Exception as e:
            from simulation_interfaces.msg import Result

            response.result = Result(
                result=Result.RESULT_OPERATION_FAILED,
                error_message=f"Error getting entity bounds: {e}",
            )
            carb.log_error(f"Error in GetEntityBounds service handler: {e}")

        return response

    async def _handle_get_spawnables(self, request: object, response: object) -> object:
        """Handle GetSpawnables service request.

        Returns a list of USD assets available for spawning, discovered by searching
        the Isaac Sim assets root (Robots and Props directories) and any caller-supplied
        additional sources. Each result is a Spawnable with uri, description, and empty bounds.

        Args:
            request: GetSpawnables request with optional additional source paths.
            response: GetSpawnables response with list of Spawnable objects.

        Returns:
            object: Completed GetSpawnables response.

        """
        try:
            from simulation_interfaces.msg import Bounds, Result, Spawnable

            response.result = Result()
            response.spawnables = []

            try:
                assets_root_path = await get_assets_root_path_async()
            except Exception as e:
                assets_root_path = None
                carb.log_error(f"{e}")

            # Default search paths (only if assets_root_path is available)
            default_paths = []
            if assets_root_path is not None:
                default_paths = [
                    assets_root_path + "/Isaac/Samples/ROS2/Robots",
                ]

            if not default_paths and not request.sources:
                response.result.result = Result.RESULT_OPERATION_FAILED
                response.result.error_message = "No asset sources available"
                return response

            usd_files = set()

            # 1. Search default paths with depth=2
            for path in default_paths:
                try:
                    found = await find_filtered_files_async(
                        path, None, False, filepath_excludes=[".thumbs"], max_depth=2
                    )
                    usd_files.update(found)
                except Exception as e:
                    carb.log_warn(f"Error searching default spawnables path {path}: {e}")

            # 2. Search additional sources with unlimited depth
            if request.sources:
                for src in request.sources:
                    resolved = resolve_source_path(src, assets_root_path)
                    try:
                        found = await find_filtered_files_async(
                            resolved, None, False, filepath_excludes=[".thumbs"], max_depth=None
                        )
                        usd_files.update(found)
                    except Exception as e:
                        carb.log_warn(f"Error searching spawnables source {resolved}: {e}")

            empty_bounds = Bounds()
            empty_bounds.type = Bounds.TYPE_EMPTY
            for uri in usd_files:
                spawnable = Spawnable()
                spawnable.uri = uri
                spawnable.description = os.path.splitext(os.path.basename(uri))[0]
                spawnable.spawn_bounds = empty_bounds
                response.spawnables.append(spawnable)

            response.result.result = Result.RESULT_OK
            response.result.error_message = f"Found {len(response.spawnables)} default ROS 2 assets"
            carb.log_info(f"GetSpawnables found {len(response.spawnables)} default ROS 2 assets")

        except Exception as e:
            from simulation_interfaces.msg import Result

            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error in GetSpawnables service: {e}"
            carb.log_error(f"Error in GetSpawnables service handler: {e}")

        return response

    async def _handle_reset_simulation(self, request: object, response: object) -> object:
        """Handle ResetSimulation service request.

        This service resets the simulation environment to its initial state.
        Any dynamically spawned entities will be de-spawned after simulation is reset.

        Args:
            request: ResetSimulation request with scope of reset
            response: ResetSimulation response with result status

        Returns:
            object: Completed ResetSimulation response

        """
        try:
            from simulation_interfaces.msg import Result

            # Set the response result
            response.result = Result(result=Result.RESULT_OK, error_message="")

            # Ignore specific scope values - always use SCOPE_DEFAULT (full reset)
            carb.log_info("Resetting simulation with SCOPE_DEFAULT (full reset)")

            # Get usdrt stage for traversing to find spawned entities
            usdrt_stage = stage_utils.get_current_stage(backend="fabric")
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
                stage_utils.delete_prim(entity_path)
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

    async def _handle_step_simulation(self, request: object, response: object) -> object:
        """Handle StepSimulation service request.

        This service steps the simulation forward by a specific number of frames,
        and then returns to a paused state. The simulation must be paused before
        stepping can be performed.

        Note: Single step (steps=1) is not currently supported. Please use 2 or more steps.

        Args:
            request: StepSimulation request with number of steps
            response: StepSimulation response with result

        Returns:
            object: Completed StepSimulation response

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

    async def _handle_get_entity_state(self, request: object, response: object) -> object:
        """Handle GetEntityState service request.

        This service retrieves the state (pose, twist, acceleration) of a specific entity.
        If the entity has a rigid body API, only the pose is returned (REQ 6).
        If the entity doesn't have a rigid body API, both pose and velocity are returned.

        Args:
            request: GetEntityState request with entity path
            response: GetEntityState response with state information

        Returns:
            object: Completed GetEntityState response

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

    async def _handle_get_entities_states(self, request: object, response: object) -> object:
        """Handle GetEntitiesStates service request.

        This service retrieves the states (pose, twist, acceleration) of multiple entities
        in the simulation, filtered by name using the partial prim path.
        It combines the functionality of GetEntities and GetEntityState.

        Args:
            request: GetEntitiesStates request with optional filters
            response: GetEntitiesStates response with entities and their states

        Returns:
            object: Completed GetEntitiesStates response

        """
        try:
            from simulation_interfaces.msg import Result

            # Initialize response lists
            response.entities = []
            response.states = []

            # Get usdrt stage for traversing
            usdrt_stage = stage_utils.get_current_stage(backend="fabric")
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

    async def _handle_set_entity_state(self, request: object, response: object) -> object:
        """Handle SetEntityState service request.

        This service sets the state (pose, twist) of a specific entity in the simulation.
        It updates the position, orientation, and velocities (if applicable) of the specified prim.
        Note that velocity and acceleration features may have limited support depending on entity type.

        Args:
            request: SetEntityState request with entity name and desired state
            response: SetEntityState response with result status

        Returns:
            object: Completed SetEntityState response

        """
        try:

            from simulation_interfaces.msg import Result

            stage = stage_utils.get_current_stage()
            if not stage:
                response.result = Result(result=Result.RESULT_OPERATION_FAILED, error_message="Stage not available")
                return response

            # Check if the entity exists
            prim = prim_utils.get_prim_at_path(request.entity)
            if not prim.IsValid():
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

    async def _handle_simulate_steps_action(self, goal_handle: object) -> object:
        """Handle SimulateSteps action request.

        This action steps the simulation forward by a specific number of frames,
        and then returns to a paused state. The simulation must be paused before
        stepping can be performed. The action provides feedback after each step.

        Args:
            goal_handle: ROS2 action goal handle that contains the goal and methods
                         to publish feedback and set result

        Returns:
            object: Result message for the SimulateSteps action

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

    def _handle_simulate_steps_cancel_callback(self, goal_handle: object) -> object:
        """Handle cancellation requests for SimulateSteps action.

        This callback is invoked when a client requests to cancel the SimulateSteps action.
        It immediately pauses the simulation and returns the appropriate response.

        Args:
            goal_handle: ROS2 action goal handle for the cancellation request.

        Returns:
            object: Response indicating if cancellation is accepted.

        """
        try:
            from rclpy.action import CancelResponse

            carb.log_info("SimulateSteps action cancellation requested")

            # Pause the simulation immediately when cancellation is requested
            try:
                self.timeline.pause()
                self.timeline.commit()
                carb.log_info("Simulation paused due to SimulateSteps cancellation")
            except Exception as pause_error:
                carb.log_error(f"Error pausing simulation during cancellation: {pause_error}")

            carb.log_info("SimulateSteps action cancellation accepted")
            return CancelResponse.ACCEPT

        except Exception as e:
            carb.log_error(f"Error in SimulateSteps cancel callback: {e}")
            # Default to accepting cancellation even if there's an error to prevent
            # the action from being stuck in an unresponsive state
            from rclpy.action import CancelResponse

            return CancelResponse.ACCEPT

    async def _handle_load_world(self, request: object, response: object) -> object:
        """Handle LoadWorld service request.

        This service loads a world or environment file into the simulation,
        clearing the current scene and setting the simulation to stopped state.
        Currently supports USD format worlds. If the given path fails to load, the default asset root prefix is attempted with the given path.

        Args:
            request: LoadWorld request with world file path and parameters.
            response: LoadWorld response with result status and world info.

        Returns:
            object: Completed LoadWorld response.

        """
        try:
            from simulation_interfaces.msg import Result, WorldResource

            # Initialize response
            response.result = Result()
            response.world = WorldResource()

            # Check if simulation is playing - load world is only allowed when simulation is not playing
            if self.timeline.is_playing():
                response.result.result = Result.RESULT_OPERATION_FAILED
                response.result.error_message = (
                    "Load world operation requires simulation to be stopped or paused. Current state: playing"
                )
                return response

            # Validate USD format
            if not is_valid_usd_file(request.uri, []):
                response.result.result = response.UNSUPPORTED_FORMAT
                response.result.error_message = (
                    f"Unsupported format. Only USD files (.usd, .usda, .usdc, .usdz) are supported. Got: {request.uri}"
                )
                return response

            # Use the new utility function to resolve the asset path
            path_to_load = await resolve_asset_path_async(request.uri)

            # Load the path that exists, or report error if neither exists
            if path_to_load:

                # Stop the simulation first
                carb.log_info("Stopping simulation before loading world")
                self.timeline.stop()
                await omni.kit.app.get_app().next_update_async()

                carb.log_info(f"Loading world from USD file: {path_to_load}")
                success, error = await stage_utils.open_stage_async(path_to_load)
                if not success:
                    response.result.result = response.RESOURCE_PARSE_ERROR
                    response.result.error_message = f"Failed to load world: {error}"
                    return response
            else:
                response.result.result = response.RESOURCE_PARSE_ERROR
                response.result.error_message = f"Could not find path '{request.uri}' or default asset root based path"
                return response

            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

            stage = stage_utils.get_current_stage(backend="fabric")
            if not stage:
                response.result.result = Result.RESULT_OPERATION_FAILED
                response.result.error_message = "Failed to get loaded stage"
                return response

            response.world.world_resource.uri = path_to_load
            response.world.name = os.path.splitext(os.path.basename(path_to_load))[0]
            response.result.result = Result.RESULT_OK
            if path_to_load != request.uri:
                response.result.error_message = (
                    f"Successfully loaded world: {response.world.name} (using default asset root path)"
                )
                carb.log_info(f"Successfully loaded world: {response.world.name} using default asset root path")
            else:
                response.result.error_message = f"Successfully loaded world: {response.world.name}"
                carb.log_info(f"Successfully loaded world: {response.world.name}")

        except Exception as e:
            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error in LoadWorld service: {e}"
            carb.log_error(f"Error in LoadWorld service handler: {e}")

        return response

    async def _handle_unload_world(self, request: object, response: object) -> object:
        """Handle UnloadWorld service request.

        This service unloads the current world from the simulation,
        clearing the current scene and creating a new empty stage.
        Any previously spawned entities will be removed.

        Args:
            request: UnloadWorld request (empty)
            response: UnloadWorld response with result status

        Returns:
            object: Completed UnloadWorld response

        """
        try:
            from simulation_interfaces.msg import Result

            # Initialize response
            response.result = Result()

            # Check if simulation is playing - unload world is only allowed when simulation is not playing
            if self.timeline.is_playing():
                response.result.result = Result.RESULT_OPERATION_FAILED
                response.result.error_message = (
                    "Unload world operation requires simulation to be stopped or paused. Current state: playing"
                )
                return response

            usdrt_stage = stage_utils.get_current_stage(backend="fabric")
            if not usdrt_stage:
                carb.log_warn("No stage currently loaded")
                response.result.result = response.NO_WORLD_LOADED
                response.result.error_message = "No world is currently loaded"
                return response

            # Stop the simulation first
            self.timeline.stop()
            await omni.kit.app.get_app().next_update_async()

            # Create a new empty stage
            carb.log_info("Creating new empty stage")
            await stage_utils.create_new_stage_async()

            # Wait for the new stage to be fully initialized
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

            # Success
            response.result.result = Result.RESULT_OK
            response.result.error_message = "Successfully unloaded world and created empty stage"

        except Exception as e:
            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error in UnloadWorld service: {e}"

        return response

    async def _handle_get_current_world(self, request: object, response: object) -> object:
        """Handle GetCurrentWorld service request.

        This service returns information about the currently loaded world,
        including its URI, name, and format.

        Args:
            request: GetCurrentWorld request (empty).
            response: GetCurrentWorld response with world information.

        Returns:
            object: Completed GetCurrentWorld response.

        """
        try:

            from simulation_interfaces.msg import Result, WorldResource

            response.result = Result()
            response.world = WorldResource()

            usdrt_stage = stage_utils.get_current_stage()
            if not usdrt_stage:
                carb.log_warn("No stage currently loaded")
                response.result.result = response.NO_WORLD_LOADED
                response.result.error_message = "No world is currently loaded"
                return response

            # Get stage information
            root_layer = usdrt_stage.GetRootLayer()
            if not root_layer:
                response.result.result = response.NO_WORLD_LOADED
                response.result.error_message = "No world is currently loaded (no root layer)"
                return response

            # Extract world information
            stage_uri = root_layer.identifier
            if stage_uri and not stage_uri.startswith("anon:"):
                # World was loaded from a file
                response.world.world_resource.uri = stage_uri
                response.world.name = os.path.splitext(os.path.basename(stage_uri))[0]
            else:
                # World was created in memory (new stage or from string)
                response.world.world_resource.uri = ""
                response.world.name = "untitled_world"

            # Success
            response.result.result = Result.RESULT_OK

        except Exception as e:
            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error in GetCurrentWorld service: {e}"
            carb.log_error(f"Error in GetCurrentWorld service handler: {e}")

        return response

    async def _handle_get_available_worlds(self, request: object, response: object) -> object:
        """Handle GetAvailableWorlds service request.

        This service returns a list of available world files that can be loaded into the simulation.
        It searches paths for USD world files, with support for TagsFilter-based filtering.

        Args:
            request: GetAvailableWorlds request with optional filters and search parameters.
            response: GetAvailableWorlds response with list of available worlds.

        Returns:
            object: Completed GetAvailableWorlds response.

        """
        try:
            from simulation_interfaces.msg import Result, TagsFilter, WorldResource

            # Initialize response
            response.result = Result()
            response.worlds = []

            # Get Isaac assets root path
            try:
                assets_root_path = await get_assets_root_path_async()
            except Exception as e:
                assets_root_path = None
                carb.log_error(f"{e}")

            if assets_root_path is None:
                # Only continue if continue_on_error=True AND additional_sources exist
                if not (request.continue_on_error and request.additional_sources):
                    response.result.result = Result.RESULT_OPERATION_FAILED
                    response.result.error_message = "Default assets root path not accessible"
                    return response

            # Default search paths (only if assets_root_path is available)
            default_paths = []
            if assets_root_path is not None:
                default_paths = [
                    assets_root_path + "/Isaac/Environments",
                    assets_root_path + "/Isaac/Samples/ROS2/Scenario",
                ]

            # Check if we have a tags filter with actual tags
            has_tags_filter = bool(request.filter.tags)

            filter_patterns = None
            match_all = False

            # Parse TagsFilter to extract patterns and mode if provided
            if has_tags_filter:
                filter_patterns = list(request.filter.tags)
                # Get filter mode: 0 = FILTER_MODE_ANY (default), 1 = FILTER_MODE_ALL
                match_all = request.filter.filter_mode == TagsFilter.FILTER_MODE_ALL

            # Search for USD files
            usd_files = set()

            # 1. Always search default paths first with depth=1
            for path in default_paths:

                if request.offline_only and not is_local_path(path):
                    continue

                try:
                    found_files = await find_filtered_files_async(
                        path,
                        filter_patterns,
                        match_all,
                        filepath_excludes=[".thumbs", "Props", "Materials"],
                        max_depth=1,
                    )
                    usd_files.update(found_files)

                except Exception as e:
                    carb.log_warn(f"Error searching default path {path}: {e}")
                    if not request.continue_on_error:
                        raise

            # 2. Search additional sources with unlimited depth
            if request.additional_sources:
                for src in request.additional_sources:
                    # Skip Nucleus-relative resolution for offline paths to avoid mangling local paths.
                    resolved = src if request.offline_only else resolve_source_path(src, assets_root_path)

                    if request.offline_only and not is_local_path(resolved):
                        continue

                    try:
                        found_files = await find_filtered_files_async(
                            resolved, filter_patterns, match_all, max_depth=None
                        )
                        usd_files.update(found_files)

                    except Exception as e:
                        carb.log_warn(f"Error searching additional source {resolved}: {e}")
                        if not request.continue_on_error:
                            raise

            # Convert found worlds to WorldResource objects
            for world_path in usd_files:
                world_resource = WorldResource()
                world_resource.world_resource.uri = world_path

                # Extract world name from path (filename without extension)
                world_name = os.path.splitext(os.path.basename(world_path))[0]
                world_resource.name = world_name

                response.worlds.append(world_resource)

            # Set success result
            response.result.result = Result.RESULT_OK
            response.result.error_message = f"Found {len(response.worlds)} available worlds"

        except Exception as e:
            response.result.result = Result.RESULT_OPERATION_FAILED
            response.result.error_message = f"Error in GetAvailableWorlds service: {e}"
            carb.log_error(f"Error in GetAvailableWorlds service handler: {e}")

        return response

    async def _handle_get_simulator_features(self, request: object, response: object) -> object:
        """Handle GetSimulatorFeatures service request.

        This service lists the subset of services and actions supported by Isaac Sim
        from simulation_interfaces, including brief descriptions of workflows for each interface.

        Args:
            request: GetSimulatorFeatures request (empty)
            response: GetSimulatorFeatures response with list of supported features

        Returns:
            object: Completed GetSimulatorFeatures response

        """
        try:
            from simulation_interfaces.msg import SimulatorFeatures

            # Resolve via getattr so older simulation_interfaces releases
            # advertise the subset they define instead of crashing on a
            # missing constant.
            feature_names = [
                "SPAWNING",
                "DELETING",
                "ENTITY_BOUNDS",
                "ENTITY_BOUNDS_BOX",
                "ENTITY_STATE_GETTING",
                "ENTITY_STATE_SETTING",
                "ENTITY_INFO_GETTING",
                "SPAWNABLES",
                "SIMULATION_RESET",
                "SIMULATION_RESET_SPAWNED",
                "SIMULATION_STATE_GETTING",
                "SIMULATION_STATE_SETTING",
                "SIMULATION_STATE_PAUSE",
                "STEP_SIMULATION_SINGLE",
                "STEP_SIMULATION_MULTIPLE",
                "STEP_SIMULATION_ACTION",
                "WORLD_LOADING",
                "WORLD_TAGS",
                "WORLD_UNLOADING",
                "WORLD_INFO_GETTING",
                "AVAILABLE_WORLDS",
                "SPAWNING_ENTITIES",
            ]
            features = []
            unavailable = []
            for name in feature_names:
                if hasattr(SimulatorFeatures, name):
                    features.append(getattr(SimulatorFeatures, name))
                else:
                    unavailable.append(name)
            if unavailable:
                carb.log_warn(
                    "SimulatorFeatures constants not present in this simulation_interfaces "
                    f"release; omitting from response: {', '.join(unavailable)}"
                )

            # Set the features in the response
            response.features.features = features

            # Set supported spawn formats
            response.features.spawn_formats = ["usd"]

            # Set custom info with version and description
            response.features.custom_info = "Control Isaac Sim via ROS2 Simulation Interfaces."

            carb.log_info("Successfully responded to GetSimulatorFeatures request")

        except Exception as e:
            # There's no result field in GetSimulatorFeatures response, so we can only log the error
            carb.log_error(f"Error in GetSimulatorFeatures service handler: {e}")

        return response

    def shutdown(self) -> None:
        """Shutdown the simulation control services.

        Cleanly shuts down the ROS2 service manager and all registered
        services and action servers.
        """
        if self.service_manager:
            self.service_manager.shutdown()


class Extension(omni.ext.IExt):
    """Extension that starts and stops the ROS 2 simulation control services."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension."""
        self.sim_control = SimulationControl()

    def on_shutdown(self) -> None:
        """Clean up resources when the extension shuts down."""
        if self.sim_control:
            self.sim_control.shutdown()
