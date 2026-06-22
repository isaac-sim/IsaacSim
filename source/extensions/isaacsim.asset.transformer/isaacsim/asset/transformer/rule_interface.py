# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Abstract base interface for asset transformer rules."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from pxr import Usd

from .models import RuleConfigurationParam


class RuleInterface(ABC):
    """Abstract base class for asset transformation rules.

    Implementations operate on a source :class:`pxr.Usd.Stage` and may write
    opinions to a destination :class:`pxr.Usd.Stage`. Subclasses should record
    human-readable log messages and any identifiers for stages or layers they
    affect so the manager can produce comprehensive reports.

    Rules may request a stage replacement by returning a stage identifier (file
    path) from :meth:`process_rule`. The manager will open the new stage and
    use it for subsequent rules in the pipeline.

    Stage mutation contract:
        ``args["input_stage"]`` is informational only and must be treated
        as **read-only in every respect**, including its session layer.
        Rules that need to author overrides while reading from the
        original input must open a private :class:`Usd.Stage` from
        ``args["input_stage_path"]`` and author into that stage's session
        layer; they must not write to either layer of any caller-owned
        Stage.

        Two reasons for this:

        1. ``args["input_stage"]`` may be a caller-owned Stage (e.g. the
           editor's active Stage). Its session layer commonly carries
           user-driven overrides such as visibility toggles, purpose
           settings, and camera opinions; rule authoring or cleanup there
           would corrupt user state.
        2. The root :class:`pxr.Sdf.Layer` of an input opened by path is
           shared via USD's process-wide layer cache with every Stage
           observing the same file -- including the editor's active Stage
           when the user runs the transformer on the open stage. Authoring
           to that root layer (or calling ``Reload()`` on it) fires change
           notifications on the editor's Stage that have been observed to
           invalidate Hydra render product prims mid-frame and crash the
           renderer.

    Args:
        source_stage: Input stage providing opinions to read from.
        package_root: Root directory for output files.
        destination_path: Relative path for rule outputs.
        args: Mapping of parameters including keys such as ``destination`` and
            ``params``.

    """

    def __init__(self, source_stage: Usd.Stage, package_root: str, destination_path: str, args: dict[str, Any]) -> None:
        self.source_stage: Usd.Stage = source_stage
        self.package_root: str = package_root
        self.destination_path: str = destination_path
        self.args: dict[str, Any] = args or {}
        self._log: list[str] = []
        self._affected_stages: list[str] = []
        self._pending_deletions: list[str] = []

    @abstractmethod
    def process_rule(self) -> str | None:
        """Execute the rule logic.

        This method must be implemented by subclasses. Implementations should
        emit log messages via :meth:`log_operation` and record any affected
        stage or layer identifiers via :meth:`add_affected_stage`.

        Returns:
            The file path of the stage to be used by subsequent rules. Return
            ``None`` if the current working stage should continue to be used.
            If a path is returned and differs from the current working stage,
            the manager will open the new stage for subsequent rules.

        Example:

        .. code-block:: python

            from pxr import Usd
            from isaacsim.asset.transformer import RuleInterface

            class NoOpRule(RuleInterface):
                def process_rule(self) -> None:
                    self.log_operation("noop")

            stage = Usd.Stage.CreateInMemory()
            NoOpRule(stage, "/tmp", "", {"destination": "out.usda"}).process_rule()

        """
        raise NotImplementedError

    def log_operation(self, message: str) -> None:
        """Append a human-readable message to the operation log.

        Args:
            message: Message to record in the rule execution log.

        Example:

        .. code-block:: python

            rule.log_operation("Copied prim /World")

        """
        self._log.append(message)

    def get_operation_log(self) -> list[str]:
        """Return the accumulated operation log messages.

        Returns:
            List of log message strings in chronological order.

        Example:

        .. code-block:: python

            messages = rule.get_operation_log()

        """
        return list(self._log)

    def add_affected_stage(self, stage_identifier: str) -> None:
        """Record an identifier for a stage or layer affected by this rule.

        Args:
            stage_identifier: Logical label, file path, or layer id that was
                created, modified, or otherwise affected by the rule.

        Example:

        .. code-block:: python

            rule.add_affected_stage("/tmp/output.usda")

        """
        if stage_identifier and stage_identifier not in self._affected_stages:
            self._affected_stages.append(stage_identifier)

    def get_affected_stages(self) -> list[str]:
        """Return identifiers for stages or layers affected by this rule.

        Returns:
            List of unique identifiers provided via :meth:`add_affected_stage`.

        Example:

        .. code-block:: python

            affected = rule.get_affected_stages()

        """
        return list(self._affected_stages)

    def request_deletion(self, path: str) -> None:
        """Ask the manager to delete *path* once it releases the working stage.

        A rule must use this instead of calling :func:`os.remove` directly when
        the file it wants to delete backs the working stage. ``self.source_stage``
        is the manager-owned working stage; the manager keeps it (and its root
        :class:`pxr.Sdf.Layer`) open for the duration of the rule, so the rule
        cannot free the underlying file. On Windows an open USD layer is an
        OS-level file lock, so an in-rule ``os.remove`` fails with
        ``PermissionError: [WinError 5] Access is denied``. The manager drains
        the requests returned by :meth:`get_pending_deletions` only after it has
        dropped the working stage, guaranteeing the file handle is gone before
        the unlink.

        The rule should drop its own references to the file (e.g. set
        ``self.source_stage = None``) before returning so the manager holds the
        last handle.

        Args:
            path: Filesystem path the manager should delete after releasing the
                working stage. Duplicate and empty paths are ignored.

        Example:

        .. code-block:: python

            rule.request_deletion("/tmp/pkg/payloads/base.usd")

        """
        if path and path not in self._pending_deletions:
            self._pending_deletions.append(path)

    def get_pending_deletions(self) -> list[str]:
        """Return paths the rule deferred to the manager for deletion.

        Returns:
            List of unique filesystem paths registered via
            :meth:`request_deletion`, in registration order.

        Example:

        .. code-block:: python

            for path in rule.get_pending_deletions():
                ...

        """
        return list(self._pending_deletions)

    @abstractmethod
    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            List of configuration parameters.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        raise NotImplementedError
