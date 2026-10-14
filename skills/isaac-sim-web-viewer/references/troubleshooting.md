# Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| `pull access denied` for `nvcr.io/nvidia/isaac-sim` | Not logged in to NGC | Operator must `docker login nvcr.io` locally (do not paste the key into chat), then retry compose |
| `ASK_USER_IMAGE_TAG` / `manifest unknown` | NGC tag does not exist | Stop. Ask the operator for a published tag. Do not compose up. Public NGC latest is `nvcr.io/nvidia/isaac-sim:6.0.1`. |
| Web viewer never becomes healthy | Isaac Sim still starting or GPU/EULA issue | `docker compose -p isim logs isaac-sim`; wait for `Isaac Sim Full Streaming App is loaded` |
| URL given before stream is ready | Agent printed the URL on web-viewer healthy / AppReady | Wait for `Isaac Sim Full Streaming App is loaded`, then intimate the operator with the URL |
| Browser connects but black screen / WAITING FOR STREAM | Browser URL host does not match baked `ISAACSIM_HOST` | Same-machine browser → `http://127.0.0.1:8210`. Other computer → `http://<this-machine-IP>:8210` with `--host auto` (or that IP) and `--build`. Close extra tabs. |
| `NVST_R_BUSY` in isaac-sim logs | A second browser tab tried to connect | Close the extra tab. Only one stream client is allowed. |
| Hub warnings / compose Hub errors after a prior test | Stale Hub container | Restart the Hub container from Hub Workstation Cache, then retry Docker Compose |
| Permission errors on `~/docker/isaac-sim` | Mounts not writable by container uid 1234 | `setfacl` on `~/docker/isaac-sim` (see workflow). If `setfacl` is missing, install the `acl` package — do not `chmod a+rwX`. Do not follow `ISAAC_SIM_DATA` symlinks. Use `chown -R 1234:1234` only if that is not enough. Do not chmod `~/docker` or `~/.cache/ov`. |
| Clipboard (Ctrl+C/V) not working | HTTP from a non-localhost origin is not a secure context | Open Chromium on the Isaac Sim host at `http://127.0.0.1:8210`. Do not enable Chrome insecure-origin flags. Do not TLS-proxy 8210 only (mixed-content signaling). See `references/browser.md`. |
