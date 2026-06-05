# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Base class for nodes that manage replicator writers with automatic reset capabilities."""

import copy

import carb.eventdispatcher
import carb.events
import omni.replicator.core as rep
import omni.usd
from pxr import Usd

from .base_reset_node import BaseResetNode


class WriterRequest:
    """A request to manage writer attachment and detachment operations for render products.

    This class encapsulates the information needed to attach or detach a replicator writer to specific
    render product paths. It is used internally by writer nodes to queue and process writer operations
    asynchronously.

    Args:
        writer: The replicator writer to be managed.
        render_product_path: Path or paths to render products where the writer will be attached.
        activate: Whether to activate (attach) or deactivate (detach) the writer.
    """

    def __init__(self, writer: rep.Writer, render_product_path: str | list[str], activate: bool = True) -> None:
        self.writer = writer
        self.render_product_path = render_product_path
        self.activate = activate

    def __repr__(self) -> str:
        """String representation of the writer request.

        Returns:
            A formatted string showing the writer node type, configuration, render product path, and annotators.
        """
        output = f"{self.writer.node_type_id}\n\t{self.writer._kwargs}\n\t{self.render_product_path}\n\tAnnotators:\n"
        for a in self.writer._annotators:
            output = output + f"\t\t{a}\n"
        return output


class BaseWriterNode(BaseResetNode):
    """Base class for nodes that automatically reset when stop is pressed.

    Args:
        initialize: Whether to initialize the node immediately.
    """

    def __init__(self, initialize: bool = False) -> None:
        self._writers = []
        self._requests = []
        self._event_stream = None
        super().__init__(initialize=False)

    def custom_reset(self) -> None:
        """Resets the writer node by deactivating all writers and clearing the internal writer list."""
        for w in self._writers:
            self._append_request(WriterRequest(w, None, False))
        self._writers = []
        self.initialized = False

    def append_writer(self, writer: rep.Writer) -> None:
        """Appends deepcopy of provided writer to internal writer list.

        Args:
            writer: The writer to add to the internal list.
        """
        self._writers.append(copy.deepcopy(writer))

    def attach_writers(self, render_product_path: str | list[str]) -> None:
        """Create writer requests for all stored writers using the provided render product and activate them.

        Args:
            render_product_path: The render product path to attach writers to.
        """
        for w in self._writers:
            self._append_request(WriterRequest(w, render_product_path, True))

    def attach_writer(self, writer: rep.Writer, render_product_path: str | list[str]) -> None:
        """Create writer request for deepcopy of provided writer to provided render_product_path, and activates it.

        Args:
            writer: The writer to attach.
            render_product_path: The render product path to attach the writer to.
        """
        # Appending provided writer to member list
        self._writers.append(copy.deepcopy(writer))

        # Ensure previously appended writer is referenced for the WriterRequest
        self._append_request(WriterRequest(self._writers[-1], render_product_path, True))

    def _append_request(self, request: WriterRequest) -> None:
        """Appends a writer request to the internal queue and sets up event processing if needed.

        Args:
            request: The WriterRequest to add to the processing queue.
        """
        self._requests.append(request)
        if self._event_stream is None:
            self._event_stream = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self._process_activation_requests,
                observer_name="BaseWriterNode._process_activation_requests",
            )

    def _process_activation_requests(self, event: carb.eventdispatcher.Event) -> None:
        """Process all pending writer activation requests by attaching or detaching writers.

        Args:
            event: The event that triggered the processing.
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            self._event_stream = None
            return
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for request in self._requests:
                try:
                    if request.activate:
                        request.writer.attach(request.render_product_path)
                        self.post_attach(request.writer, request.render_product_path)
                        ### WAR to make sure the graph is not deleted on stop
                        noop = rep.AnnotatorRegistry.get_annotator(
                            "IsaacNoop",
                        )
                        noop.attach([request.render_product_path])
                        carb.log_info(f"Attaching:\n{request}")
                    else:
                        request.writer.detach()
                except Exception as e:
                    carb.log_error(
                        f"Could not process writer attach request {request.writer, request.render_product_path}, {e}"
                    )
            # Stop processing additional requests until another one is appended
            self._requests = []
            self._event_stream = None

    # Defined by subclass
    def post_attach(self, writer: rep.Writer, render_product: str | list[str]) -> None:
        """Hook method called after a writer is attached to a render product.

        Args:
            writer: The writer that was attached.
            render_product: The render product the writer was attached to.
        """
