FROM docker.io/ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y cmake ninja-build gcc-arm-none-eabi build-essential ssh git tzdata bash curl jq graphviz && rm -rf /var/lib/apt/lists/*
RUN LATEST_TAG=$(curl -s https://api.github.com/repos/doxygen/doxygen/releases/latest | jq -r .tag_name) \
    && VERSION=$(echo $LATEST_TAG | sed 's/Release_//' | tr '_' '.') \
    && wget https://github.com/doxygen/doxygen/releases/download/${LATEST_TAG}/doxygen-${VERSION}.linux.bin.tar.gz \
    && tar -xzf doxygen-${VERSION}.linux.bin.tar.gz -C /usr/local --strip-components=1 \
    && rm doxygen-${VERSION}.linux.bin.tar.gz
COPY *.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/*.sh
WORKDIR /builds