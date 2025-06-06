# WhisperTrack-Sotfware

## Getting started
### Pre-requisites
- Docker

### Build
1. Clone the repository:
   ```bash
   git clone https://github.com/Delresearch/WhisperTrack-Software.git
    cd WhisperTrack-Software
    ```
2. Build the Docker image:
    ```bash
    cd docker
    docker build -t whispertrack .
    ```
3. Run the Docker container:
    ```bash
    cd ../
    docker run -it --rm -v $(pwd):/app whispertrack bash
    ```
4. Inside the container, you can run the application:
    ```bash
    cd /app
    mkdir -p build
    cd build
    cmake .. -G Ninja -DSTM32=ON -DCMAKE_TOOLCHAIN_FILE=stm32/gcc-arm-none-eabi.cmake
    ninja
    ```
The application will be built in build/stm32/. You can then flash it to your STM32 device using your preferred method.

## License
This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.