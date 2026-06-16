FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libceres-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    python3 \
    python3-numpy \
    python3-yaml \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /work
