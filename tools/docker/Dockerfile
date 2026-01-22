FROM nvcr.io/nvidia/base/ubuntu:noble-20250619

ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

# Disable hardware-specific library optimizations in dynamic linker for wider compatibility (legacy Debian/Ubuntu)
RUN touch /etc/ld.so.nohwcap

# Install dependencies
# Note: GCC 11 is included to support containerized builds on hosts with incompatible
# compilers (e.g., Ubuntu 24.04 with GCC 13). Isaac Sim requires GCC 11 for compilation.
# This enables: ./tools/docker/prep_docker_build.sh --docker-build
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        curl \
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
        # Build and prep tools
        build-essential \
        gcc-11 \
        g++-11 \
        python3-pip \
        rsync \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100 \
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

# Switch to root user
USER root
RUN useradd -ms /bin/bash -l --uid 1234 isaac-sim -d /isaac-sim
RUN chown -R isaac-sim /tmp
USER isaac-sim

# Copy package
RUN mkdir -p /isaac-sim
WORKDIR /isaac-sim
COPY --chown=isaac-sim:isaac-sim . /isaac-sim/
RUN mkdir -p /isaac-sim/.nvidia-omniverse/config

# Open ports for live streaming
EXPOSE 47998/udp \
       49100/tcp

ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

ENV OMNI_SERVER=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1

ENV MIN_DRIVER_VERSION=570.169

WORKDIR /isaac-sim

# Add symlink
RUN ln -s exts/isaacsim.examples.interactive/isaacsim/examples/interactive extension_examples

# Update .bashrc
RUN echo 'alias runapp=/isaac-sim/runapp.sh' >> ~/.bashrc && \
    echo 'alias runheadless=/isaac-sim/runheadless.sh' >> ~/.bashrc

# Default entrypoint to launch headless with streaming
ENTRYPOINT ["/isaac-sim/runheadless.sh"]
