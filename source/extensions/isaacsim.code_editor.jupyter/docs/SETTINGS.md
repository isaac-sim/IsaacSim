```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

## Settings Provided by the Extension

## exts."isaacsim.code_editor.jupyter".host
   - **Default Value**: "127.0.0.1"
   - **Description**: IP address where the extension server will listen for connections.

## exts."isaacsim.code_editor.jupyter".port
   - **Default Value**: 8227
   - **Description**: Port number where the extension server will listen for connections.

## exts."isaacsim.code_editor.jupyter".kill_processes_with_port_in_use
   - **Default Value**: false
   - **Description**: Whether to kill applications/processes that use the same ports before enabling the extension.

## exts."isaacsim.code_editor.jupyter".notebook_ip
   - **Default Value**: "127.0.0.1"
   - **Description**: IP address where the Jupyter server is being started.

## exts."isaacsim.code_editor.jupyter".notebook_port
   - **Default Value**: 8228
   - **Description**: Port number where the Jupyter server is being started.

## exts."isaacsim.code_editor.jupyter".notebook_token
   - **Default Value**: ""
   - **Description**: Jupyter server token for token-based authentication.

## exts."isaacsim.code_editor.jupyter".notebook_dir
   - **Default Value**: ""
   - **Description**: The directory to use for notebooks.

## exts."isaacsim.code_editor.jupyter".command_line_options
   - **Default Value**: "--allow-root --no-browser --JupyterApp.answer_yes=True"
   - **Description**: Jupyter server command line options other than --ip, --port, --token and --notebook-dir.
