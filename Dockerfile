# Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
      && apt-get -qqy install \
        build-essential \
        cmake \
        python-is-python3 \
        python3-pip \
        git \
      && pip install conan \
      && conan profile new $HOME/.conan/profiles/default --detect \
      && conan profile update settings.compiler.libcxx=libstdc++11 default
ARG CONAN_GTSAM_REPO=https://github.com/marvonlar/conan-gtsam.git
ARG PTCEE_REPO=https://github.com/ffi-no/ptcee.git
ARG PAPER_REPO=https://github.com/ffi-no/Paper-ptz-subpix-accuracy.git
ADD . /git/Paper-ptz-subpix-accuracy.git
RUN git clone $CONAN_GTSAM_REPO --depth=1 /tmp/conan-gtsam \
      && conan export /tmp/conan-gtsam gtsam/4.1.1@ \
      && rm -rf /tmp/conan-gtsam
RUN git clone $PTCEE_REPO --depth=1 /tmp/ptcee \
      && conan export /tmp/ptcee \
      && rm -rf /tmp/ptcee
WORKDIR /root
RUN git clone $PAPER_REPO --depth=1 \
      && pip install -r Paper-ptz-subpix-accuracy/python/requirements.txt \
      && conan install Paper-ptz-subpix-accuracy -if=build -bmissing \
      && cd /root/build \
      && cmake /root/Paper-ptz-subpix-accuracy -DCMAKE_BUILD_TYPE=RELEASE \
      && cmake --build . -- -j$(nproc) \
      && cd ..
