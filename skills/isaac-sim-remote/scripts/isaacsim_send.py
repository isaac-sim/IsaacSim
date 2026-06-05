#!/usr/bin/env python3
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

"""Send Python code to a running Isaac Sim instance via the python_server TCP socket.

Usage:
    # Inline code
    python isaacsim_send.py 'print("hello")'

    # From stdin (pipe or heredoc)
    echo 'print("hello")' | python isaacsim_send.py

    # Send a .py file
    python isaacsim_send.py --file path/to/script.py

    # Send a file with injected variables
    python isaacsim_send.py --file script.py --arg output_path=/tmp/shot.png --arg width=1920

    # Custom host/port/timeout
    python isaacsim_send.py --host 127.0.0.1 --port 8226 --timeout 120 'print("hello")'

    # Raw JSON output (default is formatted human-readable)
    python isaacsim_send.py --raw 'print("hello")'

    # --file scripts are isolated by default (wrapped in a function scope).
    # Use --context for persistent named namespaces:
    python isaacsim_send.py --context recording --file setup.py
    python isaacsim_send.py --context recording 'print(my_var)'

    # JSON envelope mode: named context, injected args, per-request timeout
    python isaacsim_send.py --context my_session --execution-timeout 30 'x = 1'
    python isaacsim_send.py --args-json '{"x": 42}' 'print(x)'

    # Fire-and-forget: get immediate ACK and task_id
    python isaacsim_send.py --fire-and-forget 'import time; time.sleep(5)'

    # Introspection queries
    python isaacsim_send.py --introspect status
    python isaacsim_send.py --introspect contexts
    python isaacsim_send.py --introspect tasks

Exit codes:
    0 - Execution succeeded (status: "ok")
    1 - Execution failed (status: "error") or connection error
"""

import argparse
import asyncio
import json
import sys


async def send_and_receive(host: str, port: int, source: str, timeout: float = 60.0) -> dict:
    """Send Python source code or a JSON envelope and return the parsed JSON response."""
    reader, writer = await asyncio.wait_for(asyncio.open_connection(host, port), timeout=timeout)
    writer.write(source.encode())
    writer.write_eof()
    data = await asyncio.wait_for(reader.read(), timeout=timeout)
    writer.close()
    return json.loads(data.decode())


def _inject_args(source: str, args: list[str]) -> str:
    """Prepend variable assignments for --arg key=value pairs."""
    if not args:
        return source
    lines = []
    for arg in args:
        key, _, value = arg.partition("=")
        key = key.strip()
        value = value.strip()
        try:
            parsed = eval(value, {"__builtins__": {}})  # noqa: S307
            lines.append(f"{key} = {repr(parsed)}")
        except Exception:
            lines.append(f'{key} = "{value}"')
    return "\n".join(lines) + "\n" + source


def _wrap_isolated(source: str, args: list[str]) -> str:
    """Wrap script in an async function scope to isolate from executor state.

    The python_server executor shares global state between connections.
    Wrapping in a function prevents variable leakage between calls.
    Top-level `await` expressions are supported inside the wrapper.
    """
    # Build argument assignments
    arg_lines = []
    for arg in args:
        key, _, value = arg.partition("=")
        key = key.strip()
        value = value.strip()
        try:
            parsed = eval(value, {"__builtins__": {}})  # noqa: S307
            arg_lines.append(f"    {key} = {repr(parsed)}")
        except Exception:
            arg_lines.append(f'    {key} = "{value}"')

    # Indent the source
    indented = "\n".join("    " + line for line in source.splitlines())

    parts = ["async def _isolated_script():"]
    if arg_lines:
        parts.extend(arg_lines)
    parts.append(indented)
    parts.append("")
    parts.append("await _isolated_script()")

    return "\n".join(parts)


def _parse_args_kv(arg_list: list[str]) -> dict:
    """Convert a list of ``key=value`` strings to a dict with type inference."""
    result = {}
    for arg in arg_list:
        key, _, value = arg.partition("=")
        key = key.strip()
        value = value.strip()
        try:
            result[key] = eval(value, {"__builtins__": {}})  # noqa: S307
        except Exception:
            result[key] = value
    return result


def _needs_envelope(args: argparse.Namespace) -> bool:
    """Return whether any flag requires the JSON envelope format."""
    return bool(
        args.json_envelope
        or args.context
        or args.fire_and_forget
        or args.execution_timeout is not None
        or args.args_json
        or args.introspect
    )


async def main() -> None:
    parser = argparse.ArgumentParser(description="Send Python code to Isaac Sim python_server")
    parser.add_argument("code", nargs="?", help="Python code to execute (reads stdin if omitted)")
    parser.add_argument("--file", "-f", help="Path to a Python file to send instead of inline code")
    parser.add_argument(
        "--arg", "-a", action="append", default=[], help="Inject key=value as a global variable (use with --file)"
    )
    parser.add_argument("--host", default="127.0.0.1", help="Server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8226, help="Server port (default: 8226)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Client TCP timeout in seconds (default: 60)")
    parser.add_argument("--raw", action="store_true", help="Print raw JSON instead of formatted output")
    parser.add_argument(
        "--no-isolate",
        action="store_true",
        help="Don't wrap --file scripts in isolated scope (use for state persistence)",
    )

    # JSON envelope options
    envelope_group = parser.add_argument_group("JSON envelope options")
    envelope_group.add_argument("--json-envelope", "-j", action="store_true", help="Force JSON envelope mode")
    envelope_group.add_argument(
        "--context", "-c", help="Named execution context (creates or reuses a persistent namespace)"
    )
    envelope_group.add_argument(
        "--fire-and-forget",
        "--ff",
        action="store_true",
        dest="fire_and_forget",
        help="Fire-and-forget mode: receive immediate ACK with task_id, code runs in background",
    )
    envelope_group.add_argument(
        "--execution-timeout",
        "-t",
        type=float,
        metavar="SECONDS",
        dest="execution_timeout",
        help="Per-request server-side execution timeout (0 = no limit; different from --timeout)",
    )
    envelope_group.add_argument(
        "--args-json",
        metavar="JSON",
        help="Inject variables as a JSON object string, e.g. '{\"x\": 42}'",
    )
    envelope_group.add_argument(
        "--introspect",
        metavar="COMMAND",
        help=(
            "Run an introspection query instead of executing code. "
            "Commands: status, contexts, context, tasks, task, delete_context"
        ),
    )
    args = parser.parse_args()

    # Handle introspection queries
    if args.introspect:
        envelope: dict = {"introspect": args.introspect}
        if args.context:
            envelope["context"] = args.context
        if args.code:
            # Allow passing a task_id as a positional argument for 'task' queries
            envelope["task_id"] = args.code
        source = json.dumps(envelope)
        try:
            result = await send_and_receive(args.host, args.port, source, args.timeout)
        except ConnectionRefusedError:
            print(f"Error: Cannot connect to Isaac Sim at {args.host}:{args.port}", file=sys.stderr)
            sys.exit(1)
        except asyncio.TimeoutError:
            print(f"Error: Timeout after {args.timeout}s waiting for response", file=sys.stderr)
            sys.exit(1)
        if args.raw:
            print(json.dumps(result, indent=2))
        else:
            if result.get("status") == "ok":
                print(json.dumps(result.get("result"), indent=2))
            else:
                print(f"ERROR: {result.get('result', result)}", file=sys.stderr)
                sys.exit(1)
        return

    # Build code string
    if args.file:
        with open(args.file) as f:
            source_code = f.read()
        if args.no_isolate or _needs_envelope(args):
            source_code = _inject_args(source_code, args.arg)
        else:
            source_code = _wrap_isolated(source_code, args.arg)
    elif args.code:
        source_code = args.code
        if not _needs_envelope(args):
            source_code = _inject_args(source_code, args.arg)
    else:
        source_code = sys.stdin.read()

    if not source_code.strip():
        print("Error: No code provided", file=sys.stderr)
        sys.exit(1)

    # Build the final request: JSON envelope or raw Python
    if _needs_envelope(args):
        envelope = {"code": source_code}

        if args.context:
            envelope["context"] = args.context
        if args.fire_and_forget:
            envelope["fire_and_forget"] = True
        if args.execution_timeout is not None:
            envelope["timeout"] = args.execution_timeout

        # Merge args from --arg key=value pairs and --args-json
        merged_args: dict = _parse_args_kv(args.arg)
        if args.args_json:
            try:
                json_args = json.loads(args.args_json)
                if isinstance(json_args, dict):
                    merged_args.update(json_args)
                else:
                    print("Error: --args-json must be a JSON object", file=sys.stderr)
                    sys.exit(1)
            except json.JSONDecodeError as exc:
                print(f"Error: Invalid --args-json: {exc}", file=sys.stderr)
                sys.exit(1)
        if merged_args:
            envelope["args"] = merged_args

        source = json.dumps(envelope)
    else:
        source = source_code

    try:
        result = await send_and_receive(args.host, args.port, source, args.timeout)
    except ConnectionRefusedError:
        print(f"Error: Cannot connect to Isaac Sim at {args.host}:{args.port}", file=sys.stderr)
        print("Make sure Isaac Sim is running with isaacsim.code_editor.python_server enabled.", file=sys.stderr)
        sys.exit(1)
    except asyncio.TimeoutError:
        print(f"Error: Timeout after {args.timeout}s waiting for response", file=sys.stderr)
        sys.exit(1)

    if args.raw:
        print(json.dumps(result, indent=2))
    else:
        # Fire-and-forget: show task_id prominently
        if result.get("fire_and_forget"):
            print(f"Task submitted. task_id: {result.get('task_id')}")
            sys.exit(0)

        output = result.get("output", "")
        if output:
            print(output)
        if "result" in result and result["result"] is not None:
            print(f"=> {result['result']}")
        if result.get("status") == "error":
            print(f"\nERROR [{result.get('ename', '?')}]: {result.get('evalue', '?')}", file=sys.stderr)
            for tb in result.get("traceback", []):
                print(tb, file=sys.stderr)

        if result.get("elapsed_seconds") is not None:
            print(f"(elapsed: {result['elapsed_seconds']:.2f}s)", file=sys.stderr)

    sys.exit(0 if result.get("status") == "ok" else 1)


if __name__ == "__main__":
    asyncio.run(main())
