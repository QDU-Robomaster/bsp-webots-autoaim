FROM ghcr.io/xrobot-org/docker-image-webots:main

RUN apt-get update && \
    apt-get install -y --no-install-recommends xvfb xauth libxcb-cursor0 && \
    rm -rf /var/lib/apt/lists/*

# Install the XRobot version pinned in Modules/modules.yaml (a release version
# or a 40-hex commit), the same one CI uses.
COPY Modules/modules.yaml /tmp/xrobot-pin/modules.yaml
RUN pin="$(python3 -c 'import yaml; print((yaml.safe_load(open("/tmp/xrobot-pin/modules.yaml")) or {}).get("xrobot") or "")')" && \
    if [ -z "$pin" ]; then echo "Pin xrobot in Modules/modules.yaml" >&2; exit 1; fi && \
    if echo "$pin" | grep -Eq '^[0-9a-f]{40}$'; then \
      python3 -m pip install --no-cache-dir "git+https://github.com/xrobot-org/XRobot.git@$pin"; \
    else \
      python3 -m pip install --no-cache-dir "xrobot==$pin"; \
    fi

WORKDIR /workspace

ENV WEBOTS_HOME=/usr/local/webots
ENV PYTHONUNBUFFERED=1
