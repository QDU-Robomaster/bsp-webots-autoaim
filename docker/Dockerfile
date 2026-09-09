FROM ghcr.io/xrobot-org/docker-image-webots:main

RUN apt-get update && \
    apt-get install -y --no-install-recommends xvfb xauth libxcb-cursor0 && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir xrobot==0.3.1

WORKDIR /workspace

ENV WEBOTS_HOME=/usr/local/webots
ENV PYTHONUNBUFFERED=1
