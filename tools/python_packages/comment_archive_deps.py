# SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import argparse
import logging
import os
from typing import Callable, Dict

import omni.repo.man

logger = logging.getLogger(__name__)


def _comment_archives(file_path, archives, remove: bool = True):
    # remove
    if remove:
        with open(file_path, "r") as f:
            lines = f.readlines()
        content = []
        for line in lines:
            append_line = True
            for archive in archives:
                if archive in line:
                    append_line = False
                    omni.repo.man.print_log(f"{file_path} -> {archive}", logging.INFO)
            if append_line:
                content.append(line)
        with open(file_path, "w") as f:
            f.writelines(content)
    # comment
    else:
        with open(file_path, "r") as f:
            content = f.read()
        for archive in archives:
            if archive in content:
                content = content.replace(f'"{archive}"', f'## "{archive}"')
                content = content.replace(f"'{archive}'", f'## "{archive}"')
                omni.repo.man.print_log(f"{file_path} -> {archive}", logging.INFO)
        with open(file_path, "w") as f:
            f.write(content)


def setup_repo_tool(parser: argparse.ArgumentParser, config: Dict) -> Callable:
    parser.description = "Comment archives extension dependencies in *.kit and config/extension.toml files"

    def run_repo_tool(options: Dict, config: Dict):
        tool_config = config["repo_comment_archive_deps"]
        search_folder = tool_config["search_folder"]
        archives = tool_config["archives"]

        if not os.path.isdir(search_folder):
            omni.repo.man.print_log(f"The search folder doesn't exist: {search_folder}", logging.WARN)
            return

        for root, dirs, files in os.walk(search_folder, followlinks=True):
            for file in files:
                if file == "extension.toml" or file.endswith(".kit"):
                    file_path = os.path.join(root, file)
                    if os.path.isfile(file_path):
                        _comment_archives(file_path, archives)
                    else:
                        omni.repo.man.print_log(f"Skipping {file_path} because it doesn't exist", logging.WARN)
                        continue

    return run_repo_tool
