# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Core buffer and module primitives for the MobilityGen state machine."""

from __future__ import annotations

from collections import OrderedDict

from pxr import Sdf

__all__ = ["Buffer", "Module"]


def _join_sdf_paths(*subpaths: str) -> str:
    """Join one or more USD path segments into a single SdfPath string."""
    p = Sdf.Path(subpaths[0])
    for subpath in subpaths[1:]:
        subpath = subpath.strip("/")
        if subpath:
            p = p.AppendPath(subpath)
    return str(p)


class Buffer:
    """A tagged container for storing and managing data values within modules.

    The Buffer class provides a flexible storage mechanism that associates arbitrary data with optional tags for
    filtering and categorization. Buffers are commonly used within Module hierarchies to organize state data,
    sensor outputs, or configuration parameters that can be selectively accessed based on their tags.

    Tags enable powerful filtering capabilities, allowing buffers to be grouped by type (e.g., "rgb",
    "segmentation", "depth") or other categorical attributes. This makes it easy to retrieve specific subsets
    of data from complex module hierarchies.

    Args:
        value: The initial value of the buffer. Defaults to None.
        tags: Optional list of string tags for filtering. Defaults to None.
    """

    def __init__(self, value: object | None = None, tags: list[str] | None = None) -> None:
        self.value = value
        if tags is None:
            tags = []
        self.tags = tags

    def get_value(self) -> object:
        """Get the buffer value.

        Returns:
            The value of the buffer.
        """
        return self.value

    def set_value(self, value: object) -> None:
        """Set the buffer value.

        Args:
            value: The new value of the buffer.
        """
        self.value = value

    def includes_tags(self, tags: list[str]) -> bool:
        """Check if the buffer includes a set of tags.

        Args:
            tags: The set of tags the buffer must include.

        Returns:
            True if the buffer includes all tags.
        """
        tags_a = set(self.tags)
        tags_b = set(tags)
        return tags_a.issuperset(tags_b)

    def excludes_tags(self, tags: list[str]) -> bool:
        """Check if the buffer excludes a set of tags.

        Args:
            tags: The set of tags the buffer must exclude.

        Returns:
            True if the buffer excludes all tags.
        """
        tags_a = set(self.tags)
        tags_b = set(tags)
        return tags_a.isdisjoint(tags_b)


class Module:
    """Base class for hierarchical state management and simulation data handling.

    This class provides a framework for organizing simulation components in a tree structure with automatic
    state management capabilities. It supports nested modules and buffers, enabling efficient data collection,
    serialization, and replay functionality for simulation scenarios.

    The Module class manages two types of objects:
    - Child modules: Other Module instances that form a hierarchical structure
    - Buffers: Data containers (Buffer instances) that store simulation state with optional tagging

    Key features include:
    - Automatic discovery of nested modules and buffers
    - State dictionary generation with tag-based filtering
    - Specialized methods for different rendering types (RGB, segmentation, depth, normals)
    - State loading and saving for simulation replay
    - Non-strict state loading that handles missing keys gracefully
    """

    def children(self) -> dict[str, "Module"]:
        """Get the immediate children attached to the module.

        Returns:
            A dictionary of all immediate children.
        """
        children = OrderedDict()
        for k, v in self.__dict__.items():
            if issubclass(v.__class__, Module):
                children[k] = v
        return children

    def buffers(self) -> dict[str, Buffer]:
        """Get the buffers directly attached to the module.

        Returns:
            A dictionary of all directly attached buffers.
        """
        buffers = OrderedDict()
        for k, v in self.__dict__.items():
            if issubclass(v.__class__, Buffer):
                buffers[k] = v
        return buffers

    def named_modules(self, prefix: str = "") -> dict[str, "Module"]:
        """Get a dictionary of all nested modules.

        Returns:
            The dictionary of all nested modules with
                expanded names as keys.
        """
        named_modules = OrderedDict()

        named_modules[prefix] = self

        for k, child in self.children().items():
            if prefix:
                child_prefix = prefix + "." + k
            else:
                child_prefix = k

            named_modules.update(child.named_modules(child_prefix))

        return named_modules

    def named_buffers(
        self, prefix: str = "", include_tags: list[str] | None = None, exclude_tags: list[str] | None = None
    ) -> dict[str, "Buffer"]:
        """Get a dictionary of all nested buffers.

        Returns:
            The dictionary of all nested buffers with
                expanded names as keys.
        """
        named_buffers = OrderedDict()

        for name, module in self.named_modules(prefix).items():
            for buffer_name, buffer in module.buffers().items():
                if name:
                    full_state_name = name + "." + buffer_name
                else:
                    full_state_name = buffer_name

                if include_tags is not None and not buffer.includes_tags(include_tags):
                    continue
                if exclude_tags is not None and not buffer.excludes_tags(exclude_tags):
                    continue

                named_buffers[full_state_name] = buffer

        return named_buffers

    def state_dict(
        self, prefix: str = "", include_tags: list[str] | None = None, exclude_tags: list[str] | None = None
    ) -> dict[str, object]:
        """Get the state dictionary of the module.

        Args:
            prefix: A prefix for state value names. Defaults to "".
            include_tags: A set of tags that each buffer must include. Defaults to None.
            exclude_tags: A set of tags that each buffer must exclude. Defaults to None.

        Returns:
            The module's state dictionary.
        """
        named_values = OrderedDict()
        for name, buffer in self.named_buffers(prefix, include_tags, exclude_tags).items():
            named_values[name] = buffer.value
        return named_values

    def state_dict_common(self, prefix: str = "") -> dict[str, object]:
        """Get the state dictionary, including only common types (no images).

        This method gets the state dictionary, but excludes any state values
        that are tagged with "segmentation" or "rgb" or "depth.  This state is intended to be small
        (in number of bytes), so it can be efficiently saved with a single call
        to np.save(...).

        The buffer list is cached after the first call because the module tree
        and tag assignments are fixed at build time.  Subsequent calls only
        iterate the cached list and read current buffer values, avoiding the
        repeated ``__dict__`` traversal.

        Args:
            prefix: A prefix for state value names. Defaults to "".

        Returns:
            The module's state dictionary, including only common types.
        """
        cache_attr = f"_state_dict_common_cache_{prefix}"
        cached = self.__dict__.get(cache_attr)
        if cached is None:
            cached = list(self.named_buffers(prefix, exclude_tags=["rgb", "segmentation", "depth", "normals"]).items())
            self.__dict__[cache_attr] = cached
        return {name: buf.value for name, buf in cached}

    def state_dict_rgb(self, prefix: str = "") -> dict[str, object]:
        """Get the state dictionary, including only values tagged "rgb".

        Args:
            prefix: A prefix for state value names. Defaults to "".

        Returns:
            The module's state dictionary, including only values tagged "rgb".
        """
        return self.state_dict(prefix, include_tags=["rgb"])

    def state_dict_segmentation(self, prefix: str = "") -> dict[str, object]:
        """Get the state dictionary, including only values tagged "segmentation".

        Args:
            prefix: A prefix for state value names. Defaults to "".

        Returns:
            The module's state dictionary, including only values tagged "segmentation".
        """
        return self.state_dict(prefix, include_tags=["segmentation"])

    def state_dict_depth(self, prefix: str = "") -> dict[str, object]:
        """Get the state dictionary, including only values tagged "depth".

        Args:
            prefix: A prefix for state value names. Defaults to "".

        Returns:
            The module's state dictionary, including only values tagged "depth".
        """
        return self.state_dict(prefix, include_tags=["depth"])

    def state_dict_normals(self, prefix: str = "") -> dict[str, object]:
        """Get the state dictionary, including only values tagged "normals".

        Args:
            prefix: A prefix for state value names. Defaults to "".

        Returns:
            The module's state dictionary, including only values tagged "normals".
        """
        return self.state_dict(prefix, include_tags=["normals"])

    def enable_rgb_rendering(self) -> None:
        """Enable RGB rendering for this module.

        This class only needs to be overwritten for Camera implementations, which
        perform the logic of enabling rendering.  By default, this method
        traverses all child modules to enable rendering.
        """
        for child in self.children().values():
            child.enable_rgb_rendering()

    def enable_segmentation_rendering(self) -> None:
        """Enable segmentation rendering for this module.

        This class only needs to be overwritten for Camera implementations, which
        perform the logic of enabling rendering.  By default, this method
        traverses all child modules to enable rendering.
        """
        for child in self.children().values():
            child.enable_segmentation_rendering()

    def enable_depth_rendering(self) -> None:
        """Enable depth rendering for this module.

        This class only needs to be overwritten for Camera implementations, which
        perform the logic of enabling rendering.  By default, this method
        traverses all child modules to enable rendering.
        """
        for child in self.children().values():
            child.enable_depth_rendering()

    def enable_instance_id_segmentation_rendering(self) -> None:
        """Enable instance ID segmentation rendering for this module.

        This class only needs to be overwritten for Camera implementations, which
        perform the logic of enabling rendering.  By default, this method
        traverses all child modules to enable rendering.
        """
        for child in self.children().values():
            child.enable_instance_id_segmentation_rendering()

    def enable_normals_rendering(self) -> None:
        """Enable normals rendering for this module.

        This class only needs to be overwritten for Camera implementations, which
        perform the logic of enabling rendering.  By default, this method
        traverses all child modules to enable rendering.
        """
        for child in self.children().values():
            child.enable_normals_rendering()

    def finalize_rendering(self) -> None:
        """Re-enable hydra texture updates after all annotators have been attached.

        Call this once after all enable_*_rendering() calls to unblock the render
        pipeline.  Render products have their hydra texture updates disabled during
        annotator attachment (to avoid evaluating partially-constructed OmniGraph
        nodes), and this method re-enables them.  By default, traverses all children.
        """
        for child in self.children().values():
            child.finalize_rendering()

    def disable_rendering(self) -> None:
        """Disable rendering and release all render products and annotators.

        This class only needs to be overwritten for Camera implementations, which
        perform the logic of disabling rendering.  By default, this method
        traverses all child modules to disable rendering.
        """
        for child in self.children().values():
            child.disable_rendering()

    def write_replay_data(self) -> None:
        """Write module state to Isaac Sim for replay.

        This method writes the module state to Isaac Sim for replaying.
        It is intended to be used along with module.load_state_dict(...)

        Example usage:

        ```python
        reader = Reader(recording_path="...")

        # Read state from disk
        state_dict = reader.read_state_dict_common(index=20)

        # Update module state buffers
        scenario.load_state_dict(state_dict)

        # Send module replay-related state to Isaac Sim
        scenario.write_replay_data()
        ```

        This method is overwritten by some classes to perform the logic
        of updating Isaac Sim.  For example, the Robot class uses this
        method to update it's pose and joint positions in the simulation.

        By default, this method traverses all children recursively to
        write the child's replay data.
        """
        for child in self.children().values():
            child.write_replay_data()

    def update_state(self) -> None:
        """Update the module state by reading data from Isaac Sim.

        This method reads data from Isaac Sim to update the module state.

        This is intended to be overwritten by certain classes to ensure
        the module state reflects the simulation.

        For example, for the camera class this method will update
        image state buffers.  For the robot class, this method will read
        the position, orientation, joint positions and joint velocities
        and update the corresponding state buffers.  By default, this
        method traverses all children to update their state.
        """
        for child in self.children().values():
            child.update_state()

    def load_state_dict(self, state_dict: dict) -> None:
        """Load a state dictionary.

        This method updates all state buffers by reading the state dictionary.

        This method is "non-strict", it will load all matching keys, but
        doesn't fail if a key is not present in either the module or the
        state dictionary to be loaded.

        This method only updates the state buffer values, and does not modify the
        simulation.  This is accomplished using other methods.
        """
        for k, v in self.named_buffers().items():
            if k in state_dict:
                v.set_value(state_dict[k])
