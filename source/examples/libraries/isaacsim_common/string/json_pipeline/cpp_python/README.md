<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# C++ with Python JSON Pipeline

`main.py` calls an example-local C++ extension that uppercases a value with `isaacsim.common.string`, serializes it
with `nlohmann_json`, and reads it with `jsonpointer`.
