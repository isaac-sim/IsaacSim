# Developing extensions in Docker

Isaac Sim extensions can be hot reloaded while the application is running. Kit's extension system watches enabled extension files and reloads a reloadable extension when a watched file changes.

For Docker development, keep the extension source outside the image and bind-mount it into the container. This lets the filesystem watcher observe edits without rebuilding the image or copying the extension again.

## Bind-mount the extension source

Assume the host has an extension at:

```text
$PWD/exts/my.company.extension/
├── config/extension.toml
└── my/company/extension/...
```

Mount the parent extension directory into the container and add it as an extension search path:

```bash
docker run --name isaac-sim --rm -it --gpus all --network=host \
  -e ACCEPT_EULA=Y \
  -v "$PWD/exts:/workspace/exts:rw" \
  isaac-sim-docker:latest \
  --ext-folder /workspace/exts \
  --enable my.company.extension \
  --/app/extensions/fsWatcherEnabled=true
```

The Docker image uses `/isaac-sim/runheadless.sh` as its entrypoint, so arguments after the image name are forwarded to Isaac Sim.

Do not bind-mount a development directory over `/isaac-sim/exts` itself. Doing so hides the extensions already provided by the image. Mount the development extensions at a separate path and add that path with `--ext-folder`.

## Make the extension reloadable

Extensions are reloadable by default. If `config/extension.toml` explicitly disables reloading, change it for development:

```toml
[core]
reloadable = true
```

An extension can also become non-reloadable when it depends on a non-reloadable extension. Check the extension state in the Extension Manager if edits are detected but the extension is not reloaded.

Kit's default filesystem watcher monitors Python and TOML files. If the extension must reload when additional file types change, add them in `config/extension.toml`:

```toml
[fswatcher]
patterns.include = ["*.toml", "*.py", "*.ogn"]
patterns.exclude = []
paths.include = ["*"]
paths.exclude = ["*/__pycache__*", "*/.git*"]
```

The global `/app/extensions/fsWatcherEnabled` setting defaults to `true`. The launch example sets it explicitly so the expected development behavior is visible in the command line.

## Verify hot reload

After Isaac Sim starts:

1. Confirm that `my.company.extension` is enabled.
2. Confirm that the file you edit on the host changes at the mounted path inside the running container:

   ```bash
   docker exec isaac-sim stat /workspace/exts/my.company.extension/config/extension.toml
   ```

3. Edit a watched `.py` or `.toml` file on the host.
4. Check the Isaac Sim log for the extension disable/enable cycle.

If the file changes inside the container but no reload occurs, check that the extension is reloadable and that `/app/extensions/fsWatcherEnabled` is enabled. Kit uses operating-system filesystem notifications for extension watching, so the backing filesystem must deliver change notifications to the container. When that is not available, move the development checkout to a local filesystem that supports native file events or reload the extension manually.

## Prebuilt images

The same workflow works with the prebuilt Isaac Sim image. Replace the local image name in the example with the desired NGC image, for example:

```bash
docker run --name isaac-sim --rm -it --gpus all --network=host \
  -e ACCEPT_EULA=Y \
  -v "$PWD/exts:/workspace/exts:rw" \
  nvcr.io/nvidia/isaac-sim:6.0.1 \
  --ext-folder /workspace/exts \
  --enable my.company.extension \
  --/app/extensions/fsWatcherEnabled=true
```

This development layout keeps the container image immutable while allowing the extension source to be edited and reloaded repeatedly.