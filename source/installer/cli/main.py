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

"""Isaac Sim installer commands and optional development frontend."""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

from isaacsim_installer import __version__
from isaacsim_installer.core.backend import BackendError, LocalBackend, execute_terminal, find_checkout_root


def main(argv: list[str] | None = None) -> int:
    """Dispatch installer commands without loading optional frontend dependencies early."""
    parser = argparse.ArgumentParser(prog="isaacsim-installer", description=__doc__, allow_abbrev=False)
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("list", "build", "run", "test", "tui"):
        command_parser = subparsers.add_parser(
            command,
            help=f"{command.capitalize()} source examples." if command != "tui" else "Open the examples frontend.",
            allow_abbrev=False,
        )
        command_parser.add_argument(
            "--source-root", type=Path, help="Isaac Sim checkout (detected for source/editable installs)"
        )
        command_parser.add_argument("--config", choices=("Release", "Debug"), default="Release")
        if command != "tui":
            command_parser.epilog = "Place --source-root and --config before forwarded command arguments."
            command_parser.add_argument(
                "arguments",
                nargs=argparse.REMAINDER,
                metavar="COMMAND_ARGUMENT",
                help=(
                    "Example IDs and command options forwarded to the examples runner; "
                    "use -- after a run selector for example arguments."
                ),
            )
        else:
            command_parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
            command_parser.add_argument(
                "--python", default=sys.executable, help="Python interpreter for the examples runner"
            )
    options, forwarded = parser.parse_known_args(argv)
    if options.command == "tui" and forwarded:
        parser.error(f"Unrecognized arguments: {' '.join(forwarded)}")
    if options.command == "tui" and importlib.util.find_spec("textual") is None:
        parser.error("The TUI requires optional dependencies: pip install 'isaacsim-installer[tui]'.")
    source_root = options.source_root or find_checkout_root(Path(__file__).resolve().parents[1])
    if source_root is None:
        parser.error("Specify --source-root PATH to an Isaac Sim checkout.")
    source_root = source_root.expanduser().resolve()
    if options.command != "tui":
        try:
            return execute_terminal(source_root, options.command, [*forwarded, *options.arguments], options.config)
        except BackendError as error:
            parser.error(str(error))
    try:
        backend = LocalBackend(source_root, options.python, options.config)
    except BackendError as error:
        parser.error(str(error))
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        parser.error("The frontend requires an interactive terminal.")
    from isaacsim_installer.tui.app import ExamplesApp

    ExamplesApp(backend).run()
    return 0
