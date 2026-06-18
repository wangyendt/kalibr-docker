ARG BASE_IMAGE_PLATFORM=linux/amd64
FROM --platform=${BASE_IMAGE_PLATFORM} osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=/catkin_ws
ENV VIO_COMMON_ROOT=/opt/vio_common
ENV VIO_COMMON_PYTHON=/opt/vio_common/python
ENV MPLBACKEND=Agg

ARG VIO_COMMON_REPO=https://github.com/JzHuai0108/vio_common.git
ARG VIO_COMMON_REF=

RUN apt-get update && apt-get install -y --no-install-recommends \
    autoconf \
    automake \
    build-essential \
    cmake \
    doxygen \
    ffmpeg \
    git \
    libblas-dev \
    libboost-all-dev \
    libeigen3-dev \
    liblapack-dev \
    libopencv-dev \
    libpoco-dev \
    libsuitesparse-dev \
    libtbb-dev \
    libv4l-dev \
    nano \
    pybind11-dev \
    python3-catkin-tools \
    python3-dev \
    python3-h5py \
    python3-igraph \
    python3-matplotlib \
    python3-numpy \
    python3-opencv \
    python3-pandas \
    python3-pip \
    python3-pyx \
    python3-scipy \
    python3-tk \
    python3-wxgtk4.0 \
    wget \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    dt-apriltags \
    natsort \
    PyYAML \
    tqdm

RUN git clone --depth 1 "${VIO_COMMON_REPO}" "${VIO_COMMON_ROOT}" \
  && if [[ -n "${VIO_COMMON_REF}" ]]; then \
       cd "${VIO_COMMON_ROOT}" \
       && git fetch --depth 1 origin "${VIO_COMMON_REF}" \
       && git checkout --detach FETCH_HEAD; \
     fi \
  && cmake -S "${VIO_COMMON_ROOT}/timestamp_corrector" -B "${VIO_COMMON_ROOT}/timestamp_corrector/build" \
       -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
       -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG -fno-lto" \
  && cmake --build "${VIO_COMMON_ROOT}/timestamp_corrector/build" -- -j1 \
  && cp "${VIO_COMMON_ROOT}"/timestamp_corrector/build/TimestampCorrector*.so "${VIO_COMMON_PYTHON}/"

RUN mkdir -p "${WORKSPACE}/src/kalibr"
COPY Schweizer-Messer "${WORKSPACE}/src/kalibr/Schweizer-Messer"
COPY aslam_cv "${WORKSPACE}/src/kalibr/aslam_cv"
COPY aslam_incremental_calibration "${WORKSPACE}/src/kalibr/aslam_incremental_calibration"
COPY aslam_nonparametric_estimation "${WORKSPACE}/src/kalibr/aslam_nonparametric_estimation"
COPY aslam_offline_calibration "${WORKSPACE}/src/kalibr/aslam_offline_calibration"
COPY aslam_optimizer "${WORKSPACE}/src/kalibr/aslam_optimizer"
COPY catkin_simple "${WORKSPACE}/src/kalibr/catkin_simple"
COPY opencv2_catkin "${WORKSPACE}/src/kalibr/opencv2_catkin"

RUN source /opt/ros/noetic/setup.bash \
  && cd "${WORKSPACE}" \
  && catkin init \
  && catkin config --extend /opt/ros/noetic \
  && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && catkin build kalibr -j"$(nproc)"

COPY tools "${WORKSPACE}/src/kalibr/tools"

ENV PYTHONPATH=${WORKSPACE}/src/kalibr/tools/camera_calibration:${VIO_COMMON_PYTHON}:${VIO_COMMON_ROOT}/timestamp_corrector/build

COPY docker/camera-calibration/entrypoint.sh /usr/local/bin/kalibr-camera-calibration
RUN chmod +x /usr/local/bin/kalibr-camera-calibration

WORKDIR /work
ENTRYPOINT ["/usr/local/bin/kalibr-camera-calibration"]
CMD ["--help"]
