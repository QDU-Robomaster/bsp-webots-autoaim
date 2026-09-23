FROM ghcr.io/xrobot-org/docker-image-webots:main

RUN apt-get update && \
    apt-get install -y --no-install-recommends xvfb xauth libxcb-cursor0 && \
    rm -rf /var/lib/apt/lists/*

ARG XROBOT_REF=feat/static-assembly-20260915
RUN python3 -m pip install --no-cache-dir "git+https://github.com/xrobot-org/XRobot.git@${XROBOT_REF}"

WORKDIR /workspace

ENV WEBOTS_HOME=/usr/local/webots
ENV PYTHONUNBUFFERED=1
