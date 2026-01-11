# Minimal ns-3 build environment (source build) to avoid host distro packaging issues.
#
# Builds ns-3 from upstream Git into /opt/ns-3-dev.
# Override the ref with: docker build --build-arg NS3_REF=ns-3.46 -t ns3-local .

FROM ubuntu:24.04

ARG NS3_REF=ns-3.46

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
  build-essential \
     g++ \
     clang \
     cmake \
     ninja-build \
     pkg-config \
     ccache \
    gdb \
    valgrind \
     \
     # Crazyflie USB / debugging support
     libusb-1.0-0 \
     usbutils \
     udev \
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
    \
    # File editing utilities
    nano \
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

# Copy the entire project into the image so it can be built/run from inside.
WORKDIR /workspace/Autonomous-Network-Self-Healing
COPY . /workspace/Autonomous-Network-Self-Healing

# Link the scratch program from the workspace at container start.
RUN mkdir -p /usr/local/bin \
  && printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -euo pipefail' \
    '' \
    'WORKSPACE_DIR="/workspace/Autonomous-Network-Self-Healing"' \
    'NS3_DIR="/opt/ns-3-dev"' \
    '' \
    'shopt -s nullglob' \
    'for SRC in "${WORKSPACE_DIR}"/apps/ns3_flooding/*.cc; do' \
    '  if [[ ! -s "${SRC}" ]]; then' \
    '    continue' \
    '  fi' \
    '  BASENAME="$(basename "${SRC}")"' \
    '  ln -sf "${SRC}" "${NS3_DIR}/scratch/${BASENAME}"' \
    'done' \
    '' \
    'mkdir -p "${NS3_DIR}/scratch/interfaces" "${NS3_DIR}/scratch/modules/flooding"' \
    'ln -sf "${WORKSPACE_DIR}/interfaces/communication_manager.h" "${NS3_DIR}/scratch/interfaces/communication_manager.h"' \
    'ln -sf "${WORKSPACE_DIR}/modules/flooding/messages.h" "${NS3_DIR}/scratch/modules/flooding/messages.h"' \
    'ln -sf "${WORKSPACE_DIR}/modules/flooding/flood.h" "${NS3_DIR}/scratch/modules/flooding/flood.h"' \
    'ln -sf "${WORKSPACE_DIR}/modules/flooding/flood.cpp" "${NS3_DIR}/scratch/modules/flooding/flood.cpp"' \
    '' \
    '# Convenience: allow running "./ns3 ..." from the bind-mounted workspace.' \
    'if [[ "${1:-}" == "./ns3" || "${1:-}" == "ns3" ]]; then' \
    '  shift' \
    '  exec "${NS3_DIR}/ns3" "$@"' \
    'fi' \
    '' \
    'exec "$@"' \
  > /usr/local/bin/ns3-entrypoint
RUN chmod +x /usr/local/bin/ns3-entrypoint
ENTRYPOINT ["/usr/local/bin/ns3-entrypoint"]

RUN python3 -m venv /opt/cf-venv \
  && /opt/cf-venv/bin/python -m pip install --upgrade pip \
  && /opt/cf-venv/bin/pip install --no-cache-dir cflib

ENV PATH="/opt/cf-venv/bin:${PATH}"

CMD ["bash"]