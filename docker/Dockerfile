FROM docker.io/ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y cmake ninja-build gcc-arm-none-eabi build-essential ssh git tzdata bash curl jq sudo && rm -rf /var/lib/apt/lists/*
RUN curl -1sLf 'https://dl.cloudsmith.io/public/evilmartians/lefthook/setup.deb.sh' | bash && apt install lefthook

# Create vscode user with sudo privileges
ARG USERNAME=builder
ARG UID=1000
ARG GID=1000
RUN groupadd --gid ${GID} ${USERNAME} \
    && useradd --uid ${UID} --gid ${GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME}

COPY build.sh /usr/local/bin/build
WORKDIR /builds
USER $USERNAME
CMD ["build"]