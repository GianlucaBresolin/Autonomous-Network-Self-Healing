FROM ubuntu:22.04

ARG NS3_VERSION=3.46.1

RUN apt-get update

RUN apt-get install -y --no-install-recommends \
    ca-certificates \
    g++ \
    python3 \
    cmake \
    ninja-build \
    git \
    build-essential \
    pkg-config \
    ccache \
    tar \
    bzip2 \
    wget

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

RUN mkdir workspace && \
    cd workspace && \
    wget https://www.nsnam.org/releases/ns-${NS3_VERSION}.tar.bz2 && \
    tar xjf ns-${NS3_VERSION}.tar.bz2 && \
    rm ns-${NS3_VERSION}.tar.bz2

WORKDIR /workspace/ns-${NS3_VERSION}

RUN ./ns3 configure --enable-examples --enable-tests --prefix=/opt/ns3 && \
    ./ns3 build && \
    ./ns3 install

WORKDIR /project

COPY . /project

RUN cmake -S /project/simulator -B /project/build-docker -DCMAKE_PREFIX_PATH=/opt/ns3 && \
    cmake --build /project/build-docker -j

ENTRYPOINT ["/project/build-docker/swarm_sim"]