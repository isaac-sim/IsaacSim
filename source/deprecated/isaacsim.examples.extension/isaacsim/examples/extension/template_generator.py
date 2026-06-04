# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides template generation functionality for creating Isaac Sim extension templates from predefined source files."""

import os
import shutil
from datetime import datetime
from pathlib import Path

import omni.kit.app


class TemplateGenerator:
    """Generates Isaac Sim extension templates from predefined source files.

    This class provides methods to create different types of extension templates by copying template source files,
    replacing placeholder keywords with user-specified values, and organizing the generated files into proper
    extension directory structures. It supports creating configuration tooling, loaded scenario, scripting, and
    component library extension templates.

    The generator automatically handles file system operations including directory creation, file copying,
    and keyword replacement in template files. It converts extension titles to valid Python package names
    and maintains consistent extension metadata across all generated templates.
    """

    def __init__(self) -> None:
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_extension_id_by_module("isaacsim.examples.extension")
        self._extension_path = ext_manager.get_extension_path(ext_id)

    def _write_string_to_file(self, file_path: str, file_string: str) -> None:
        """Writes a string to a file, creating directories as needed.

        Args:
            file_path: Path to the target file.
            file_string: Content to write to the file.
        """
        Path(os.path.dirname(file_path)).mkdir(parents=True, exist_ok=True)

        with open(file_path, "w+") as f:
            f.write(file_string)

    def _copy_directory_contents(self, source_dir: str, target_dir: str) -> list[str]:
        """Recursively copies all files and directories from source to target directory.

        Args:
            source_dir: Path to the source directory.
            target_dir: Path to the target directory.

        Returns:
            List of paths to all copied files.
        """
        new_paths = []

        for file_name in os.listdir(source_dir):
            source = os.path.join(source_dir, file_name)
            target = os.path.join(target_dir, file_name)

            if os.path.isfile(source):
                Path(os.path.dirname(target)).mkdir(parents=True, exist_ok=True)
                shutil.copy(source, target)
                new_paths.append(target)
            elif os.path.isdir(source):
                new_paths.extend(
                    self._copy_directory_contents(source, os.path.join(target_dir, os.path.basename(source)))
                )

        return new_paths

    def _replace_keywords(self, replace_dict: dict[str, str], file_paths: list[str]) -> None:
        """Replaces keywords in template files with specified values.

        Args:
            replace_dict: Dictionary mapping keywords to replacement values.
            file_paths: List of file paths to process for keyword replacement.
        """
        for file_path in file_paths:
            if file_path[-4:] == ".png":
                continue
            with open(file_path) as template:
                file_string = template.read()

            for k, v in replace_dict.items():
                file_string = file_string.replace(k, v)

            self._write_string_to_file(file_path, file_string)

    def _write_common_data(self, file_path: str, extension_title: str, extension_description: str) -> None:
        """Copies common template files and replaces standard keywords.

        Args:
            file_path: Target directory path for the template files.
            extension_title: Title of the extension.
            extension_description: Description of the extension.
        """
        source_dir = os.path.join(self._extension_path, "template_source_files", "common")
        new_paths = self._copy_directory_contents(source_dir, file_path)

        python_package_name = self._get_python_package_name(extension_title)

        replace_keywords = {
            "{EXTENSION_TITLE}": extension_title,
            "{EXTENSION_DESCRIPTION}": extension_description,
            "{CURRENT_DATE}": datetime.now().strftime("%Y-%m-%d"),
            "{EXTENSION_REPOSITORY}": "",
            "{PYTHON_PACKAGE_NAME}": python_package_name,
        }
        self._replace_keywords(replace_keywords, new_paths)

    def _get_python_package_name(self, extension_name: str) -> str:
        """Converts extension name to a valid Python package name.

        Args:
            extension_name: Name of the extension.

        Returns:
            Valid Python package name with special characters replaced by underscores and '_python' suffix.
        """
        # Convert all special characters in extension_name to underscores to make a valid python package name
        package_name = ""
        for c in extension_name:
            if c.isalnum():
                package_name += c
            else:
                package_name += "_"

        # Add the tag _python to the end of the path to make it more likely to be a unique module name
        package_name += "_python"

        return package_name

    def generate_configuration_tooling_template(
        self, file_path: str, extension_title: str, extension_description: str
    ) -> None:
        """Generates a configuration tooling workflow template.

        Args:
            file_path: Target directory path for the generated template.
            extension_title: Title of the extension.
            extension_description: Description of the extension.
        """
        self._write_common_data(file_path, extension_title, extension_description)

        python_package_name = self._get_python_package_name(extension_title)

        source_dir = os.path.join(self._extension_path, "template_source_files", "configuration_tooling_workflow")
        target_dir = os.path.join(file_path, python_package_name)

        new_paths = self._copy_directory_contents(source_dir, target_dir)

        replace_keywords = {
            "{EXTENSION_TITLE}": '"' + extension_title + '"',
            "{EXTENSION_DESCRIPTION}": '"' + extension_description + '"',
        }

        self._replace_keywords(replace_keywords, [os.path.join(target_dir, "global_variables.py")])

    def generate_loaded_scenario_template(
        self, file_path: str, extension_title: str, extension_description: str
    ) -> None:
        """Generates a loaded scenario workflow template.

        Args:
            file_path: Target directory path for the generated template.
            extension_title: Title of the extension.
            extension_description: Description of the extension.
        """
        self._write_common_data(file_path, extension_title, extension_description)

        python_package_name = self._get_python_package_name(extension_title)

        source_dir = os.path.join(self._extension_path, "template_source_files", "loaded_scenario_workflow")
        target_dir = os.path.join(file_path, python_package_name)

        new_paths = self._copy_directory_contents(source_dir, target_dir)

        replace_keywords = {
            "{EXTENSION_TITLE}": '"' + extension_title + '"',
            "{EXTENSION_DESCRIPTION}": '"' + extension_description + '"',
        }

        self._replace_keywords(replace_keywords, [os.path.join(target_dir, "global_variables.py")])

    def generate_scripting_template(self, file_path: str, extension_title: str, extension_description: str) -> None:
        """Generates a scripting workflow template.

        Args:
            file_path: Target directory path for the generated template.
            extension_title: Title of the extension.
            extension_description: Description of the extension.
        """
        self._write_common_data(file_path, extension_title, extension_description)

        python_package_name = self._get_python_package_name(extension_title)

        source_dir = os.path.join(self._extension_path, "template_source_files", "scripting_workflow")
        target_dir = os.path.join(file_path, python_package_name)

        new_paths = self._copy_directory_contents(source_dir, target_dir)

        replace_keywords = {
            "{EXTENSION_TITLE}": '"' + extension_title + '"',
            "{EXTENSION_DESCRIPTION}": '"' + extension_description + '"',
        }

        self._replace_keywords(replace_keywords, [os.path.join(target_dir, "global_variables.py")])

    def generate_component_library_template(
        self, file_path: str, extension_title: str, extension_description: str
    ) -> None:
        """Generates a UI component library template.

        Args:
            file_path: Target directory path for the generated template.
            extension_title: Title of the extension.
            extension_description: Description of the extension.
        """
        self._write_common_data(file_path, extension_title, extension_description)

        python_package_name = self._get_python_package_name(extension_title)

        source_dir = os.path.join(self._extension_path, "template_source_files", "ui_component_library")
        target_dir = os.path.join(file_path, python_package_name)

        new_paths = self._copy_directory_contents(source_dir, target_dir)

        replace_keywords = {
            "{EXTENSION_TITLE}": '"' + extension_title + '"',
            "{EXTENSION_DESCRIPTION}": '"' + extension_description + '"',
        }

        self._replace_keywords(replace_keywords, [os.path.join(target_dir, "global_variables.py")])
