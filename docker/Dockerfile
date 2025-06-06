ARG TARGETPLAFORM=linux/amd64
FROM --platform=${TARGETPLAFORM} docker.io/ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y cmake ninja-build gcc-arm-none-eabi build-essential ssh git tzdata bash && rm -rf /var/lib/apt/lists/*

WORKDIR /app
CMD ["bash", "-c", "while true; do sleep 30; done;"]