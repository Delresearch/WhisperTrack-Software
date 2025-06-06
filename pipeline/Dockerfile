FROM docker.io/ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y cmake ninja-build gcc-arm-none-eabi build-essential ssh git tzdata bash && rm -rf /var/lib/apt/lists/*

COPY *.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/*.sh
WORKDIR /builds