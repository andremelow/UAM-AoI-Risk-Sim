# Dev container for OCUDU scheduler work: toolchain + UHD (B200) + ZeroMQ.
# Source and build dirs are bind-mounted from the host; nothing is compiled into the image.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates cmake make ninja-build ccache gcc g++ pkg-config git \
      libfftw3-dev libmbedtls-dev libsctp-dev libyaml-cpp-dev libgtest-dev \
      libzmq3-dev libuhd-dev uhd-host libboost-program-options-dev \
      libdw-dev binutils-dev \
      gdb iproute2 iputils-ping iperf3 netcat-openbsd python3 usbutils \
    && rm -rf /var/lib/apt/lists/*
# FPGA/firmware images for the B200 family only.
RUN uhd_images_downloader -t b2xx
WORKDIR /work
