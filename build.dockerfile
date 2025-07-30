FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update
RUN apt upgrade -y
RUN apt install -y curl git git-lfs build-essential gcc-11 g++-11 ffmpeg libsm6 libxext6 libglu1 libvulkan1 nvidia-driver-570
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200
RUN git clone https://github.com/isaac-sim/IsaacSim.git /workspace

WORKDIR /workspace

RUN git lfs install && git lfs pull
RUN echo "yes" | ./build.sh

WORKDIR /isaac-sim
RUN mv /workspace/_build/linux-x86_64/release/* /isaac-sim
ENTRYPOINT [ "/isaac-sim/isaac-sim.sh" ]
