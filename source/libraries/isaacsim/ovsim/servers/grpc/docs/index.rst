..
   SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
   SPDX-License-Identifier: Apache-2.0

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

.. _isaacsim-ovsim-servers-grpc:

====================
OV SIM gRPC server
====================

.. contents:: On this page
   :local:
   :depth: 1

.. _isaacsim-ovsim-servers-grpc-overview:

Behavior
========

The server hosts one local OV SIM physics simulation and exposes the minimal
lifecycle, latest-value read, single-attribute write, and step operations
used by the remote client. Attribute writes use the shared protocol value codec
and delegate capability validation to the local data implementation.

.. _isaacsim-ovsim-servers-grpc-run:

Run the server
==============

Start the server before connecting a remote client::

   isaacsim-ovsim-grpc-server --listen-address 127.0.0.1:50051
