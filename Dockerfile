# Minimal ns-3 build environment (source build) to avoid host distro packaging issues.
#
# Builds ns-3 from upstream Git into /opt/ns-3-dev.
# Override the ref with: docker build --build-arg NS3_REF=ns-3.46 -t ns3-local .

FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG NS3_REF=master

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
     ca-certificates \
     git \
     python3 \
     python3-pip \
    python3-dev \
    python3-venv \
    python3-setuptools \
    python3-wheel \
     g++ \
     clang \
     cmake \
     ninja-build \
     pkg-config \
     ccache \
    gdb \
    valgrind \
    \
    # Useful networking utilities for debugging/emulation scenarios
    iproute2 \
    iputils-ping \
    tcpdump \
    \
    # Packet capture support (some examples/features)
    libpcap-dev \
     \
     # Common optional deps that improve feature coverage
     libboost-all-dev \
     libsqlite3-dev \
     libxml2-dev \
     libgsl-dev \
    libgtk-3-dev \
    \
    # Parallel/distributed simulation (optional)
    openmpi-bin \
    libopenmpi-dev \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN git clone https://gitlab.com/nsnam/ns-3-dev.git ns-3-dev \
  && cd ns-3-dev \
  && git checkout "${NS3_REF}" \
  && ./ns3 configure \
       --enable-examples \
       --enable-tests \
       --enable-logs \
       --enable-asserts \
       --disable-werror \
  && ./ns3 build

WORKDIR /opt/ns-3-dev

CMD ["./ns3", "--help"]