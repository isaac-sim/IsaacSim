FROM nvcr.io/nvidia/omniverse/hub_workstation_cache:2.0.0 AS hub-cache
FROM nvcr.io/nvidia/base/ubuntu:noble-20251013

ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

# Disable hardware-specific library optimizations in dynamic linker for wider compatibility (legacy Debian/Ubuntu)
RUN touch /etc/ld.so.nohwcap

# Install dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        curl \
        gpgv \
        libatomic1 \
        libegl1 \
        libgl1 \
        libglu1-mesa \
        libglx0 \
        libgomp1 \
        libsm6 \
        libxi6 \
        libxrandr2 \
        libxt6 \
        unzip \
        ca-certificates \
        libglib2.0-0 \
        libnghttp2-14 \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/*

# Install runtime configuration files
RUN mkdir -p /usr/share/glvnd/egl_vendor.d /etc/vulkan/icd.d /etc/vulkan/implicit_layer.d
RUN printf '{\n    "file_format_version" : "1.0.0",\n    "ICD" : {\n        "library_path" : "libEGL_nvidia.so.0"\n    }\n}\n' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json
RUN printf '{\n    "file_format_version" : "1.0.0",\n    "ICD" : {\n        "library_path" : "libEGL_mesa.so.0"\n    }\n}\n' > /usr/share/glvnd/egl_vendor.d/50_mesa.json
RUN printf '{\n    "file_format_version" : "1.0.0",\n    "ICD" : {\n        "library_path" : "libGLX_nvidia.so.0",\n        "api_version" : "1.3.194"\n    }\n}\n' > /etc/vulkan/icd.d/nvidia_icd.json
RUN mkdir -p /etc/vulkan/implicit_layer.d && printf '{\n    "file_format_version" : "1.0.0",\n    "layer": {\n        "name": "VK_LAYER_NV_optimus",\n        "type": "INSTANCE",\n        "library_path": "libGLX_nvidia.so.0",\n        "api_version" : "1.3.194",\n        "implementation_version" : "1",\n        "description" : "NVIDIA Optimus layer",\n        "functions": {\n            "vkGetInstanceProcAddr": "vk_optimusGetInstanceProcAddr",\n            "vkGetDeviceProcAddr": "vk_optimusGetDeviceProcAddr"\n        },\n        "enable_environment": {\n            "__NV_PRIME_RENDER_OFFLOAD": "1"\n        },\n        "disable_environment": {\n            "DISABLE_LAYER_NV_OPTIMUS_1": ""\n        }\n    }\n}\n' > /etc/vulkan/implicit_layer.d/nvidia_layers.json
ENV VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json

# Create non-root user
USER root
RUN useradd -ms /bin/bash -l --uid 1234 isaac-sim -d /isaac-sim
RUN chown -R isaac-sim /tmp
RUN mkdir -p /var/cache/hub && chown isaac-sim:isaac-sim /var/cache/hub
RUN touch /tmp/hub-isaac-sim.lock && chown isaac-sim:isaac-sim /tmp/hub-isaac-sim.lock
USER isaac-sim

# Copy package
RUN mkdir -p /isaac-sim
WORKDIR /isaac-sim
COPY --chown=isaac-sim:isaac-sim . /isaac-sim/
# Set group write and execute permissions so any user in isaac-sim group can
# create, modify or execute files
RUN chmod -R g+wsx /isaac-sim /tmp
RUN mkdir -p /isaac-sim/.nvidia-omniverse/config

# Copy /app from hub_workstation_cache
COPY --from=hub-cache --chmod=755 /app/hub /usr/local/bin/hub

ENV MIN_DRIVER_VERSION=570.169
ENV HUB__CACHE__PATH=/var/cache/hub
ENV HUB__ARGS__DETECT_ONLY=true
ENV OMNICLIENT_HUB_EXE=/usr/local/bin/hub

# Add symlink
RUN ln -s exts/isaacsim.examples.interactive/isaacsim/examples/interactive extension_examples

# Update .bashrc
RUN echo 'alias runapp=/isaac-sim/runapp.sh' >> ~/.bashrc && \
    echo 'alias runheadless=/isaac-sim/runheadless.sh' >> ~/.bashrc

# Health check: consider ready when AppReady appears in the Isaac Sim log (Kit/<app>/<version>/kit_*.log)
HEALTHCHECK --interval=30s --timeout=10s --start-period=180s --retries=3 \
  CMD bash -c 'logdir="/isaac-sim/.nvidia-omniverse/logs/Kit"; [ -d "$logdir" ] && latest=$(ls -t "$logdir"/*/*/kit_*.log 2>/dev/null | head -1) && [ -n "$latest" ] && grep -q "AppReady" "$latest" && exit 0; exit 1'

# Default entrypoint to launch headless with streaming
ENTRYPOINT ["/isaac-sim/runheadless.sh"]
