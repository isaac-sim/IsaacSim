# Overview

The isaacsim.code_editor.python_server extension provides a TCP socket server that enables remote Python code execution within a running Isaac Sim instance. Any client (VS Code, LLM agents, custom scripts) can connect, send Python source code, and receive structured JSON results.

## Quick Start

Launch Isaac Sim with the extension enabled:

```bash
bash isaac-sim.sh --no-window --enable isaacsim.code_editor.python_server
```

Wait for `app ready` in the output, then connect from any TCP client on port 8226. If `auth_token` setting is left empty while `require_auth` is true, the extension will generate a random token upon startup. It will then write the token to the `auth_token` setting and print it to the standard output (e.g., *Python server authentication token: TOKEN*).

## Functionality

### Code Execution Server

The extension creates an async TCP server that listens for incoming Python source code. When code is received, it executes within Isaac Sim's Python environment using the Executor class, which handles both statements and expressions. Results, including output, evaluated expression values, errors, and tracebacks, are transmitted back as JSON.

### Wire Protocol

- **Request**: Raw UTF-8 Python source code sent over TCP with a token header, or a JSON envelope with an `auth_token` field (see below). The client must signal end-of-input by calling ``write_eof()`` (TCP half-close) after sending all data. The server buffers incoming data until EOF is received before executing, ensuring that TCP-fragmented payloads are fully reassembled.
- **Response**: JSON object with the following fields:
  - `status`: `"ok"` or `"error"`
  - `output`: Captured standard output (includes output from both synchronous and async code)
  - `result`: Evaluated expression value (present only for expression evaluation)
  - `traceback`: List of traceback strings (present only on error)
  - `ename`: Exception class name (present only on error)
  - `evalue`: Exception message (present only on error)
  - `elapsed_seconds`: Total execution time in seconds (present when `keepalive_interval` is set and elapsed time exceeds it)

### Async Code Support

The server supports top-level ``await`` expressions. When submitted code produces a coroutine, the server drives it to completion without creating an asyncio Task. This means user code never runs as the "current task", so operations that pump the event loop (e.g. ``create_new_stage_async``, ``update_app_async``) work correctly without causing task reentrancy errors.

Standard output from ``print()`` calls inside awaited coroutines is captured and included in the JSON response ``output`` field.

```python
# Top-level await is supported — including event-loop-pumping operations
import isaacsim.core.experimental.utils.stage as stage_utils
await stage_utils.create_new_stage_async(template="empty")
print("this appears in the output field")
```

### State Persistence

The server maintains globals dictionaries (one per named context) across all connections within a session. Variables defined in one request are available in subsequent requests to the same context. The default context (no ``"context"`` field) uses a single shared namespace; named contexts provide isolated namespaces for multi-tool workflows. See "Named Execution Contexts" below.

### Carbonite Log Broadcasting

An optional UDP-based logging feature broadcasts Isaac Sim's Carbonite logging messages to connected clients, providing real-time access to internal logging output.

### JSON Envelope Protocol

Instead of raw Python source, a client may send a JSON-encoded envelope object. The server auto-detects the envelope when the request starts with ``{``. All fields except ``code`` are optional.

**Envelope format:**

```json
{
  "auth_token": "TOKEN",
  "code": "print('hello')",
  "context": "my_context",
  "args": {"x": 42, "name": "robot"},
  "timeout": 30,
  "fire_and_forget": false
}
```

**Fields:**

- `code` *(required)*: Python source to execute.
- `auth_token`: Authentication token. Required when `require_auth` is true.
- `context`: Named execution context (see below). Defaults to `""` (shared default context).
- `args`: Dict of values injected into the execution namespace before running the code.
- `timeout`: Per-request timeout in seconds (overrides the global `execution_timeout` setting; `0` = no timeout).
- `fire_and_forget`: If `true`, acknowledge immediately and execute in background (see below).

Raw Python source (not starting with ``{``) is also accepted. If the request starts with ``{`` but fails JSON parsing, it is treated as raw Python code.
If authentication is required, this can be indicated via the authentication header comment:

```python
# isaacsim-python-server-token: TOKEN
print("hello")
```


**Example:**

```python
import asyncio, json

async def send_envelope(envelope: dict, host="SERVER_HOST", port=8226, token="TOKEN") -> dict:
    reader, writer = await asyncio.open_connection(host, port)
    envelope = {**envelope, "auth_token": token}
    writer.write(json.dumps(envelope).encode())
    writer.write_eof()
    data = await asyncio.wait_for(reader.read(), timeout=30.0)
    writer.close()
    return json.loads(data.decode())

# Inject args and use a named context
result = asyncio.run(send_envelope({
    "code": "print(f'x={x}, name={name}')",
    "args": {"x": 42, "name": "robot"},
    "context": "my_session",
}))
print(result)  # {"status": "ok", "output": "x=42, name=robot"}
```

### Named Execution Contexts

By default all requests share a single globals dict (`""` context). Named contexts provide fully isolated namespaces that persist for the lifetime of the extension.

```python
# Context A: define a variable
asyncio.run(send_envelope({"code": "x = 10", "context": "session_A"}))

# Context B: x is NOT visible here
asyncio.run(send_envelope({"code": "print(x)", "context": "session_B"}))
# => {"status": "error", "ename": "NameError", ...}

# Back in context A: x persists
result = asyncio.run(send_envelope({"code": "x", "context": "session_A"}))
# => {"status": "ok", "result": 10}
```

Contexts can be inspected and deleted via the introspection endpoint (see below).

### Execution Timeouts

Set a per-request timeout via the JSON envelope ``timeout`` field (seconds), or configure a global default with the ``execution_timeout`` setting.

```python
# This will be cancelled after 2 seconds
result = asyncio.run(send_envelope({
    "code": "import asyncio\nawait asyncio.sleep(60)",
    "timeout": 2,
}))
print(result)
# {"status": "error", "ename": "TimeoutError", "evalue": "Execution timed out after 2s", ...}
```

**Async code:** Uses `asyncio.wait_for()` for clean cancellation — the request completes in exactly `timeout` seconds.

**Sync code:** Uses a background watchdog timer. The sync code continues running (it cannot be interrupted), but the client receives the timeout error as soon as the event loop is free after the code finishes. Set `timeout=0` to disable.

### Fire-and-Forget Mode

When ``fire_and_forget: true`` is set in the envelope, the server immediately acknowledges the request and executes the code asynchronously in the background.

**Acknowledgement response:**

```json
{"status": "ok", "output": "", "fire_and_forget": true, "task_id": "<uuid>"}
```

The `task_id` can be used to retrieve the result via introspection after the code finishes.

```python
import asyncio, json, time

async def demo():
    # Uses send_envelope() defined in the JSON Envelope example above
    # Launch background task
    ack = await send_envelope({
        "code": "import time; time.sleep(0.5); result = 42",
        "fire_and_forget": True,
        "context": "bg",
    })
    task_id = ack["task_id"]

    # Poll for result
    for _ in range(20):
        await asyncio.sleep(0.1)
        data = await send_envelope({"introspect": "task", "task_id": task_id})
        if data["result"] is not None:
            print("Task result:", data["result"])
            break

asyncio.run(demo())
```

The server retains up to 100 completed task results (FIFO eviction).

### Introspection

Send an envelope with ``"introspect"`` instead of ``"code"`` to query server state without executing code.

| Command | Additional fields | Description |
|---------|------------------|-------------|
| `"contexts"` | — | List all context names and variable counts |
| `"context"` | `"context": "name"` | List variable names and types in a context |
| `"tasks"` | — | List all completed background tasks |
| `"task"` | `"task_id": "..."` | Retrieve the result of a specific completed task |
| `"delete_context"` | `"context": "name"` | Delete a named context |
| `"status"` | — | Server uptime, active connections, task counts |

**Example — server status:**

```python
data = asyncio.run(send_envelope({"introspect": "status"}))
print(data["result"])
# {"uptime_seconds": 42.3, "active_connections": 1, "completed_tasks": 5, "contexts": ["", "session_A"]}
```

**Example — list context variables:**

```python
asyncio.run(send_envelope({"code": "x = 1; y = 'hello'", "context": "demo"}))
data = asyncio.run(send_envelope({"introspect": "context", "context": "demo"}))
print(data["result"])
# {"x": "int", "y": "str", ...}
```

## Client Example

### Python (asyncio)

```python
import asyncio
import json

async def send_code(code: str, host: str = "SERVER_HOST", port: int = 8226, token: str = "TOKEN") -> dict:
    """Send Python code to Isaac Sim and return the JSON response."""
    reader, writer = await asyncio.open_connection(host, port)
    if token:
        code = f"# isaacsim-python-server-token: {token}\n{code}"
    writer.write(code.encode())
    writer.write_eof()  # Signal end-of-input (required)
    data = await asyncio.wait_for(reader.read(), timeout=30.0)
    writer.close()
    return json.loads(data.decode())

# Usage
result = asyncio.run(send_code('print("Hello from Isaac Sim")'))
print(result)
# {"status": "ok", "output": "Hello from Isaac Sim", "result": null}
```

**Important**: Always call ``writer.write_eof()`` after sending code. The server buffers incoming data and only executes once it receives EOF. Without it, the connection will hang until the timeout.

### netcat (quick test)

```bash
printf '# isaacsim-python-server-token: TOKEN\nprint("hello")\n' | nc -q 0 SERVER_HOST 8226
```

## Key Components

### Executor

The Executor class manages Python code execution within Isaac Sim's environment. It accepts source code as strings and executes them asynchronously, capturing standard output, expression results, exceptions, and tracebacks. It distinguishes between expressions and statements, returning the evaluated value for expressions.

## Configuration

The extension provides the following configuration settings:

- `host`: Configures the IP address where the socket server listens for connections (default: 127.0.0.1)
- `port`: Sets the port number for socket communication (default: 8226)
- `require_auth`: Requires clients to provide an authentication token before commands are processed (default: false)
- `auth_token`: Authentication token for TCP clients. If empty and `require_auth` is true, a random token is generated on startup.
- `carb_logs`: Controls whether Carbonite logging messages are broadcast via UDP (default: false, with warnings about potential application freezing)
- `execution_timeout`: Global default execution timeout in seconds; `0` disables the timeout (default: 0)
- `keepalive_interval`: When non-zero, includes `elapsed_seconds` in responses that took longer than this value in seconds (default: 0)

## Integration

The extension depends on isaacsim.core.deprecation_manager for compatibility management within the Isaac Sim ecosystem.
