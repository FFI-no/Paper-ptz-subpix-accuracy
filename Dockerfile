# Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

FROM ubuntu:24.04
ARG DEBIAN_FRONTEND=noninteractive
ARG PTCEE_REPO=https://github.com/ffi-no/ptcee.git
ARG PAPER_REPO=https://github.com/ffi-no/Paper-ptz-subpix-accuracy.git
RUN apt-get update \
  && apt-get -qqy install \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    python3-venv \
    python-is-python3 \
    git \
    texlive-full \
  && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN python -m venv /opt/conan-venv \
  && /opt/conan-venv/bin/pip install --upgrade pip setuptools wheel \
  && /opt/conan-venv/bin/pip install conan
ENV PATH="/opt/conan-venv/bin:$PATH"
RUN conan profile detect
RUN git clone $PTCEE_REPO --depth=1 /tmp/ptcee \
  && conan export /tmp/ptcee \
  && rm -rf /tmp/ptcee
WORKDIR /root
RUN git clone $PAPER_REPO --depth=1
#ADD . Paper-ptz-subpix-accuracy
RUN pip install -r Paper-ptz-subpix-accuracy/python/requirements.txt
RUN conan build Paper-ptz-subpix-accuracy -bmissing
