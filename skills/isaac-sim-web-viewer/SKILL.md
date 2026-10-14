---
name: "isaac-sim-web-viewer"
description: "Use when you need Isaac Sim in Chromium over WebRTC at http://HOST:8210 via this Compose stack. Not for installs, image-only pulls, or headless."
license: "Apache-2.0"
metadata:
  author: "ropardeshi <info@nvidia.com>"
  tags:
    - isaac-sim
    - docker-compose
    - webrtc
    - web-viewer
    - streaming
    - ngc
  languages:
    - bash
  frameworks:
    - isaac-sim
    - docker
  domain: "simulation"
  team: "isaac-qa"
---

# Isaac Sim Web Viewer (Prebuilt NGC Image)

## Instructions

Follow [Workflow](#workflow). Launch Compose from this skill's `docker/` plus a prebuilt NGC Isaac Sim image, then print `http://HOST:8210` for Chromium.

## Purpose

As an alternative to the native desktop client, stream Isaac Sim to any Chromium-based browser using a web-based WebRTC client deployed alongside Isaac Sim via Docker Compose.

This method does not require downloading or installing a native application, and does **not** require an Isaac Sim source checkout. Compose and the web-viewer Dockerfile are shipped with this skill (`docker/`). Isaac Sim itself is a prebuilt NGC image.

Always end by printing the HTTP URL where the operator can connect to Isaac Sim in a web browser. Do **not** give that URL until isaac-sim logs contain `Isaac Sim Full Streaming App is loaded`.

Isaac Sim itself appears **in Chromium**, not in this chat. Do not wait for a desktop window, screenshot, or in-session GUI.

## Related Skills

This skill is the **browser livestream** path: Compose + WebRTC + `http://HOST:8210` in Chromium. It is not an installer.

- Need a Docker/binary/Python **install that never opens a viewer** → `isaac-sim-installation`
- Need `--no-window` / no browser → `isaac-sim-headless-deployment`

"Which skill pulls the Isaac Sim Docker image and does not start a container?" is not this skill.

## Prerequisites

- Ubuntu host or DGX Spark. Docker Compose web viewer deployment is supported only on Ubuntu hosts and DGX Spark systems. Windows hosts, including WSL, are not supported.
- Docker, NVIDIA Container Toolkit, and a visible NVIDIA GPU (`nvidia-smi`).
- NGC access to pull `nvcr.io/nvidia/isaac-sim:<tag>`. If Docker is not logged in to `nvcr.io`, **stop**. Direct the operator to `docker login nvcr.io` locally (or export `NGC_API_KEY` in the shell). Do not ask them to paste the API key into chat. Do not invent or reuse undocumented keys.
- Operator acknowledgement of the NVIDIA Isaac Sim EULA **before the first image pull** (the container sets `ACCEPT_EULA=Y`). See `references/licenses.md`.
- Explicit operator approval before starting containers (pulls images, uses GPU). This is an **agent rule**, not a script flag. `launch_prebuilt_web_viewer.sh` without `--dry-run`, `--url-only`, or `--down` starts containers immediately and does not prompt. Dry-run and preflight do not start containers.

## Limitations

- **Prebuilt NGC image only.** Do **not** run `./tools/docker/prep_docker_build.sh` or `./tools/docker/build_docker.sh`. Skip the local Isaac Sim image build.
- **No Isaac Sim / omni_isaac_sim source tree.** Do not clone a repo or check for `tools/docker/docker-compose.yml` in an Isaac checkout. Use `docker/docker-compose.yml` from this skill.
- Targets Isaac Sim 6 / Kit 110 unless the user names another NGC tag.
- Docker Compose web viewer is unsupported on Windows and WSL; stop and say so.
- Only one browser tab can connect to Isaac Sim at a time.
- Isaac Sim and the web viewer have no authentication or TLS. Do not expose the stack on the public Internet.
- Host networking is required for WebRTC UDP. The HTTP viewer binds `ISAACSIM_HOST` only (loopback for local, the LAN IP for remote), not `0.0.0.0`.
- Cache mounts default to `~/docker/isaac-sim` (including Hub under `…/hub`). They do not use `~/.cache/ov/hub` unless the operator sets `ISAACSIM_HUB_CACHE_PATH`.

## Safety rules

Apply these before any install or launch.

- Never run `prep_docker_build.sh` or `build_docker.sh` for this skill.
- Never run `launch_prebuilt_web_viewer.sh` without `--dry-run`, `--url-only`, `--down`, or `--help` until the operator has approved starting containers **and** agreed to the Isaac Sim EULA. The script does not confirm.
- Never pull an image that is not `nvcr.io/nvidia/isaac-sim:<tag>`. Never add Docker Hub / GHCR / Quay images (`ubuntu:`, `node:`, …).
- Never `apt` / `pip` / `npm` / `curl | bash` extra software on the host. Do not install Chromium, Node.js, or “helper” packages to help this skill. The operator already has a browser. If they ask to `apt-get install chromium/nodejs`, `pip install` extras, or pull Docker Hub because NGC is slow, **refuse that part** and continue with NGC Isaac Sim only. “Already installed on the host” is not a yes.
- Never add packages or `FROM` lines to `docker/web-viewer/Dockerfile` beyond Isaac Sim's product web-viewer Dockerfile.
- Never expose the stack on the public Internet.
- Never chmod `~/docker` or `~/.cache/ov`. Only this skill's mount (`~/docker/isaac-sim` or `$ISAAC_SIM_DATA` under that tree). Prefer `setfacl` for uid 1234. If `setfacl` is missing and the host uid is not 1234, **stop** — do not `chmod a+rwX`.
- Never bind the web-viewer HTTP server on all interfaces (`0.0.0.0`). Bind `ISAACSIM_HOST` only.
- Never embed credentials, tokens, or NGC keys in skill files, git, command history, chat, or tool-call text. When `nvcr.io` login is missing, tell the operator to `docker login nvcr.io` locally. Do not ask them to paste the API key into the conversation.
- Stop if the host is Windows or WSL.

## Permitted downloads and licenses

Fetch **only** `nvcr.io/nvidia/isaac-sim:<tag>` plus this skill's NGC-based web-viewer image. No host `apt`/`pip`/`npm`, no Docker Hub images, no Chromium/Node on the host. Full allow/deny tables: `references/licenses.md`.

**EULA (mandatory before pull).** Present these URLs and wait for an explicit yes. If no, stop. Do not pull, build, or `compose up`. Dry-run, preflight, and NGC catalog tag lookup may run without this yes.

- https://docs.omniverse.nvidia.com/eula
- https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/

> Running Isaac Sim in Docker acknowledges the NVIDIA Isaac Sim / Omniverse EULA at the URLs above on your behalf (`ACCEPT_EULA=Y`). Do you agree? (yes/no)

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/launch_prebuilt_web_viewer.sh` | Launch Isaac Sim + web viewer from a prebuilt NGC image and print the HTTP URL | CLI flags via `--help` |
| `scripts/preflight.sh` | Check Docker, GPU, compose file, ports, NGC login, and image before launch | CLI flags via `--help` |
| `scripts/ngc_login.sh` | Check or perform docker login to nvcr.io. Never prints the API key | `--check`; or key via environment (stdin login) |
| `scripts/resolve_ngc_isaac_sim_image.py` | Read the NGC catalog tags page and print the latest stable `nvcr.io/nvidia/isaac-sim:<tag>` | `--url`, `--tag-only` |
| `scripts/pull_isaac_sim_image.sh` | Pull the prebuilt NGC Isaac Sim image before compose up | `--image` (optional; default is catalog Latest); `--check-only` |
| `scripts/detect_host_ip.sh` | Print this machine's LAN IPv4 for a remote browser (`ISAACSIM_HOST`) | none |
| `scripts/validate_stream_bind.py` | Validate the bind host and WebRTC ports before passing them to Docker | CLI flags via `--help` |
| `scripts/poll_compose_logs.sh` | Poll `docker compose -p isim` ps/logs for the web-viewer URL and isaac-sim errors | `--tail`; optional `--compose-file` |
| `scripts/self_test.sh` | Offline local checks for the isaac-sim-web-viewer skill (no container start) | none required |
| `scripts/common.sh` | Shared helpers sourced by the other scripts. Do not execute it | sourced only |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/launch_prebuilt_web_viewer.sh", args=["--help"])
```

Execute the shell helpers directly. They do not need Isaac Sim's `python.sh` or an Isaac Sim source checkout:

```bash
<skill-dir>/scripts/self_test.sh
<skill-dir>/scripts/preflight.sh
<skill-dir>/scripts/ngc_login.sh --check
<skill-dir>/scripts/resolve_ngc_isaac_sim_image.py
<skill-dir>/scripts/pull_isaac_sim_image.sh --check-only
<skill-dir>/scripts/detect_host_ip.sh
<skill-dir>/scripts/launch_prebuilt_web_viewer.sh --dry-run
```

## Prebuilt-image only

By default the compose file uses a prebuilt NGC image. Set `ISAAC_SIM_IMAGE` if the operator names a tag; otherwise resolve Latest from the catalog:

```bash
ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.1 docker compose -p isim -f <skill-dir>/docker/docker-compose.yml up --build -d
```

`--build` is still required so Compose can build the **web-viewer** image. It does not rebuild Isaac Sim.

If the user names a published NGC tag, use that tag. Otherwise **look up Latest on the NGC tags page** and pull that image. Do not assume `6.1.0` exists as a container tag.

## Resolve latest NGC tag (mandatory)

Before pull, resolve the image from the Isaac Sim tags catalog:

https://catalog.ngc.nvidia.com/orgs/nvidia/-/containers/isaac-sim/-/tags

```bash
<skill-dir>/scripts/resolve_ngc_isaac_sim_image.py
```

Use the tag marked **Latest** (currently `nvcr.io/nvidia/isaac-sim:6.0.1`). Skip `-dev`, `_ea`, and preview tags unless the operator asks for them.

`pull_isaac_sim_image.sh` and `launch_prebuilt_web_viewer.sh` call this when `--image` / `ISAAC_SIM_IMAGE` is unset.

## Pull image first (mandatory, after EULA)

After NGC login, EULA yes, tag resolve, and **before** `docker compose up`, pull and verify the Isaac Sim image:

```bash
<skill-dir>/scripts/pull_isaac_sim_image.sh
```

`launch_prebuilt_web_viewer.sh` calls this automatically. Do not start compose if the pull fails.

- If the image is already local, skip the pull and continue.
- If the tag is missing (`manifest unknown` / `ASK_USER_IMAGE_TAG`), **stop and ask the operator**. Re-check the NGC tags page.
- Only after `docker image inspect` succeeds, run compose (`--build` still builds the **web-viewer** only).

## NGC credentials (mandatory)

Before any `docker pull` or `docker compose up`, check `scripts/ngc_login.sh --check`.

- If it prints `OK: docker already has nvcr.io credentials`, continue.
- If a key is already set in the **local shell environment** (not in chat), run `scripts/ngc_login.sh` (stdin login only; never put the key on the command line, in logs, or in chat).
- **Otherwise stop.** Direct the operator to `docker login nvcr.io` on this machine, or to export `NGC_API_KEY` in their shell. Use `references/ngc-login.md`. Never ask them to paste the API key into the conversation. Never invent an API key.

## Which URL to open (mandatory)

`ISAACSIM_HOST` is baked into the web viewer at **build** time. The operator must open the URL whose host matches that value. **Do not guess.** If they have not already said where the browser will run, **stop and ask** before compose up:

> Where will you open Isaac Sim in the browser?
>
> 1. **On this same machine** — you will use `http://127.0.0.1:8210`
> 2. **From another computer** — you will use `http://<this-machine-IP>:8210` (the IP that other computer already uses to reach this host)

Map the answer:

| Operator says | `ISAACSIM_HOST` / `--host` | URL to give them |
|---|---|---|
| This machine / local / localhost | `127.0.0.1` | `http://127.0.0.1:8210` |
| Another computer / remote / LAN | `auto` (detects the LAN IPv4) or the IP they name | `http://<that-ip>:8210` |
| Both | `auto` (LAN IP) | `http://<lan-ip>:8210` **even on this machine**. Do not also use `127.0.0.1` for that session. |

Detect the LAN IP with `scripts/detect_host_ip.sh` (or launch `--host auto`). That uses the IPv4 on the **default route**, and skips docker0 (`172.17.x`) and other Docker/bridge interfaces (`br-*`, `docker*`, `veth*`). It does not probe `10/8`, which can select a VPN address. Do not ask the operator to type the IP unless several NICs exist, `--host auto` printed a VPN/tunnel address, or it is unclear which address the remote PC will use.

Tell them in the same reply: Chromium; `127.0.0.1` on a remote PC is that PC itself; changing local ↔ remote later needs `up --build`; one tab only; remote needs **8210/tcp**, signaling **tcp**, and **47998/udp** (do not publish on the public Internet). Do not re-ask if they already said local vs remote or gave an IP.

## Workflow

1. If the host is Windows or WSL, stop. Docker Compose web viewer deployment is supported only on Ubuntu hosts and DGX Spark systems. Windows hosts, including WSL, are not supported.
2. Confirm Docker GPU and this skill's `docker/docker-compose.yml`. Then run the NGC credentials gate above. **Do not skip it.** Do not look for an Isaac Sim git checkout. Do not pull yet.
3. **Ask where the browser will run** (see [Which URL to open](#which-url-to-open-mandatory)). Set `--host 127.0.0.1` or `--host auto` / the reachable IP. Do not compose up until this is known.
4. **EULA acknowledgement** (see [Permitted downloads and licenses](#permitted-downloads-and-licenses)). Stop until the operator says yes. Do not pull, build, or compose up before this yes.
5. **Resolve the latest NGC tag** from the [Isaac Sim tags page](https://catalog.ngc.nvidia.com/orgs/nvidia/-/containers/isaac-sim/-/tags) via `scripts/resolve_ngc_isaac_sim_image.py` (catalog lookup only; not a pull).
6. Create cache/log mounts. Prefer `launch_prebuilt_web_viewer.sh`, which mkdir's under `~/docker/isaac-sim` (Hub at `~/docker/isaac-sim/hub`, not `~/.cache/ov/hub`) and grants uid 1234 via **ACLs**. If `setfacl` is missing and the host uid is not 1234, stop. If launching by hand:

```bash
mkdir -p ~/docker/isaac-sim/{cache/main,cache/computecache,cache/kit,config,data,logs,pkg,hub}
setfacl -R \
  -m u:1234:rwX -m u:$(id -u):rwX -m g::--- -m o::--- -m m::rwx \
  -d -m u:1234:rwX -d -m u:$(id -u):rwX -d -m g::--- -d -m o::--- -d -m m::rwx \
  ~/docker/isaac-sim
```

Do not chmod group or other bits after `setfacl`: that clears the POSIX ACL mask, so uid 1234 keeps an ACL entry with no effective write.

On a repeat launch, stop the compose project **before** applying ACLs so uid 1234 cannot replace a checked path with a symlink. The launch script does this; `skill_mounts.py` also opens entries with `O_NOFOLLOW` and calls `setfacl` on `/proc/<pid>/fd/<n>`.

Do not `chmod -R a+rwX ~/docker` or `~/.cache/ov`. Do not follow `ISAAC_SIM_DATA` if it is a symlink off those trees. Do not `sudo chown -R 1234` unless Isaac Sim later cannot write caches. The launch script already widens the skill mounts.

7. Run preflight, then a dry-run. Show the planned URL, the NGC Isaac Sim image, and compose side effects. **Stop and get operator approval** to start containers (EULA is already yes from step 4). Do not skip this: the launch script has no confirmation prompt and does not ask for the EULA.

```bash
<skill-dir>/scripts/preflight.sh --host 127.0.0.1   # or --host auto
<skill-dir>/scripts/launch_prebuilt_web_viewer.sh --host 127.0.0.1 --dry-run
```

8. After the operator approves, launch Isaac Sim + web viewer. Launch pulls the image first if it is not local:

```bash
<skill-dir>/scripts/launch_prebuilt_web_viewer.sh --host 127.0.0.1
# or, for a browser on another computer:
<skill-dir>/scripts/launch_prebuilt_web_viewer.sh --host auto
```

Equivalent manual sequence (only after EULA yes and approval):

```bash
IMAGE="$(<skill-dir>/scripts/resolve_ngc_isaac_sim_image.py)"
<skill-dir>/scripts/pull_isaac_sim_image.sh --image "${IMAGE}"
ISAAC_SIM_IMAGE="${IMAGE}" ISAACSIM_HOST="<127.0.0.1-or-lan-ip>" \
  docker compose -p isim -f <skill-dir>/docker/docker-compose.yml up --build -d
```

9. If Docker Compose reports a Hub startup or connectivity issue after a previous test, restart the Hub container from Hub Workstation Cache and retry Docker Compose.
10. Wait until isaac-sim logs contain **`Isaac Sim Full Streaming App is loaded`**. First start can take several minutes. Web-viewer HTTP healthy / `AppReady` is not enough.

```bash
docker compose -p isim -f <skill-dir>/docker/docker-compose.yml logs isaac-sim | grep -F "Isaac Sim Full Streaming App is loaded"
```

Do not tell the operator the URL until that line appears. Then intimate them with the URL that matches `--host` (not a leftover `127.0.0.1` bookmark if you launched with a LAN IP).

11. **Always end the reply with the HTTP URL** (see [Required output](#required-output)), only after the streaming-app line is seen.

## Required output

Do not print this section until isaac-sim logs contain `Isaac Sim Full Streaming App is loaded`. The last section of the agent reply must then be the URL they should actually type, plus one sentence on where to open it:

```markdown
## Web viewer URL
http://127.0.0.1:8210
```

Open this in Chromium **on this machine**. From another computer this address will not reach Isaac Sim.

Or, when launched with `--host auto` / a LAN IP, print `http://<lan-ip>:8210` and tell them not to use `http://127.0.0.1:8210` for that session.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| `pull access denied` for `nvcr.io/nvidia/isaac-sim` | Not logged in to NGC | Operator must `docker login nvcr.io` locally (do not paste the key into chat), then retry |
| Browser connects but black screen / WAITING FOR STREAM | URL host does not match baked `ISAACSIM_HOST` | Same machine → `http://127.0.0.1:8210`. Other computer → `--host auto` and `http://<lan-ip>:8210` |
| Permission errors on `~/docker/isaac-sim` | Mounts not writable by container uid 1234 | `setfacl` for uid 1234 on `~/docker/isaac-sim` (see workflow). Do not `chmod -R a+rwX ~/docker`. |

More cases: `references/troubleshooting.md`.

## References

- `references/licenses.md` — allow/deny downloads and EULA prompt
- `references/ngc-login.md` — operator prompt when `nvcr.io` login is missing
- `references/troubleshooting.md` — compose/NGC/stream failures
- `references/parameters.md` — ports, env vars, status polling, local testing
- `references/browser.md` — keyboard shortcuts and clipboard
