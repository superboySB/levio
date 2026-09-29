FROM python:3.10-slim-bookworm@sha256:2559be987fd64d61badbdafd303ea58a9ccab36d6c3c08bce219e762177d2eca

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    MPLBACKEND=Agg \
    MPLCONFIGDIR=/tmp/levio-matplotlib

RUN apt-get update && apt-get install -y --no-install-recommends \
    libglib2.0-0 libgl1 libsm6 libxext6 libxrender1 git fonts-wqy-microhei \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --upgrade pip && python -m pip install \
    'numpy==1.26.4' 'scipy==1.15.3' 'opencv-contrib-python-headless==4.11.0.86' \
    'matplotlib==3.10.9' 'gtsam==4.2' 'rosbag-standalone==1.17.4.1' \
    'pyyaml==6.0.3'

WORKDIR /workspace/levio
