# Build stage
# Keep this file aligned with Isaac Sim tools/docker/web-viewer/Dockerfile.
# FROM lines must stay on nvcr.io. Do not add apt/pip/npm packages, Docker Hub
# images, or extra NodeSource/npm registries beyond that product Dockerfile.
FROM nvcr.io/nvidia/base/ubuntu:noble-20251013 AS builder

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl ca-certificates \
    && curl -fsSL --proto '=https' https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

# Scope the NVIDIA npm registry, scaffold the app, then clean up the credential
RUN echo '@nvidia:registry=https://edge.urm.nvidia.com/artifactory/api/npm/omniverse-client-npm/' >> /root/.npmrc \
    && npm_config_ignore_scripts=true npx --ignore-scripts @nvidia/create-ov-web-rtc-app@2.1.5 --name=web-viewer --sample=local-sample --external --outputDir=/app sample-app --framework=react \
    && rm -f /root/.npmrc

WORKDIR /app/web-viewer

COPY patch-stream-config.mjs /tmp/patch-stream-config.mjs

# Bind address and ports arrive as ARG/ENV data. Do not interpolate them into sed.
ARG ISAACSIM_HOST=127.0.0.1
ARG ISAACSIM_SIGNAL_PORT=49100
ARG ISAACSIM_STREAM_PORT=47998
ENV ISAACSIM_HOST=${ISAACSIM_HOST} \
    ISAACSIM_SIGNAL_PORT=${ISAACSIM_SIGNAL_PORT} \
    ISAACSIM_STREAM_PORT=${ISAACSIM_STREAM_PORT}

RUN node /tmp/patch-stream-config.mjs src/main.tsx \
    && grep -q "signalingPort:" src/main.tsx \
    && grep -q "mediaPort:" src/main.tsx \
    && grep -q "forceWSS: false" src/main.tsx

# Patch: preserve browser shortcuts (DevTools, refresh) from being swallowed by the stream.
# F11: fullscreen toggle via Fullscreen API. On Mac, Fn+F11 sends F11; we also check e.code.
# F5 / Cmd+R or Ctrl+R: explicitly reload (capture phase stopPropagation would block browser default).
# F12: stopPropagation so stream does not see it (DevTools may need focus outside stream).
# F7: prevent Chrome caret-browsing dialog only.
# Clipboard shortcuts (Ctrl+C/V/X) are intentionally left for the streaming library to handle.
RUN grep -qF 'void app.initialize();' src/main.tsx \
    && sed -i '/void app.initialize/i\
const sc = document.getElementById("stream-container");\
if (sc) {\
    sc.addEventListener("keydown", (e: KeyboardEvent) => {\
        if (e.key === "F11" || e.code === "F11") { e.preventDefault(); e.stopPropagation(); if (document.fullscreenElement) { document.exitFullscreen(); } else { document.documentElement.requestFullscreen(); } return; }\
        if (e.key === "F7") { e.preventDefault(); return; }\
        if (["F5"].includes(e.key) || e.code === "F5") { e.preventDefault(); e.stopPropagation(); window.location.reload(); return; }\
        if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === "r") { e.preventDefault(); e.stopPropagation(); window.location.reload(); return; }\
        if (["F12"].includes(e.key) || e.code === "F12") { e.stopPropagation(); return; }\
        if ((e.ctrlKey || e.metaKey) && e.shiftKey && ["i","j","c"].includes(e.key.toLowerCase())) { e.stopPropagation(); return; }\
        if (e.metaKey && e.altKey && ["i","j","c"].includes(e.key.toLowerCase())) { e.stopPropagation(); return; }\
    }, true);\
}' src/main.tsx \
    && grep -qF 'const sc = document.getElementById("stream-container");' src/main.tsx

RUN npm install --ignore-scripts
RUN npm run build

# Production stage
FROM nvcr.io/nvidia/base/ubuntu:noble-20251013

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl ca-certificates \
    && curl -fsSL --proto '=https' https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -r appuser --gid 1234 && useradd --no-log-init -r -g appuser --uid 1234 -d /app -s /usr/sbin/nologin appuser

WORKDIR /app

COPY --from=builder --chmod=755 /app/web-viewer/package.json ./

RUN echo '@nvidia:registry=https://edge.urm.nvidia.com/artifactory/api/npm/omniverse-client-npm/' >> /root/.npmrc \
    && npm install --ignore-scripts \
    && npm cache clean --force \
    && rm -f /root/.npmrc \
    && chown -R appuser:appuser node_modules

COPY --from=builder --chmod=755 /app/web-viewer/dist ./dist

USER appuser

EXPOSE 8210

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
    CMD curl -f http://localhost:8210/ || exit 1

CMD ["./node_modules/.bin/vite", "preview", "--host", "127.0.0.1", "--port", "8210"]
