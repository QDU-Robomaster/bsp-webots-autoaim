FROM ghcr.io/xrobot-org/docker-image-webots:main

RUN apt-get update && \
    apt-get install -y --no-install-recommends xvfb && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir xrobot

WORKDIR /workspace

ENV WEBOTS_HOME=/usr/local/webots
ENV QT_QPA_PLATFORM=offscreen
ENV PYTHONUNBUFFERED=1
