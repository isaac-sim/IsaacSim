<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# C++ with Python JSON pipeline

`main.py` calls a C++ extension built for this example. The extension uppercases a value with
`isaacsim.common.string`, serializes it
with `nlohmann_json`, and reads it with `jsonpointer`.
