# WhisperTrack-Software

## Getting Started
### Prerequisites
- Docker

### Build
1. Clone the repository:

   ```bash
   git clone https://github.com/Delresearch/WhisperTrack-Software.git
   cd WhisperTrack-Software
   git submodule update --init --recursive
   ```
2. Build the Docker image:

    ```bash
    cd docker
    docker build -t whispertrack .
    ```
3. Build the application:

    ```bash
    cd ../
    docker run -it --rm -v $(pwd):/builds whispertrack
    ```
After the build completes, you will find the built application in the `build_stm32/stm32` directory. You can then flash it to your STM32 device using your preferred method.
## Development
To help with development, we provide a Dockerfile and a .devcontainer that set up a development environment with all the necessary dependencies. This allows you to work on the project without needing to install all dependencies on your local machine.
### Pre-requisites
- Docker
- VSCode
### Development Environment Setup
1. Clone the repository:

   ```bash
    git clone https://github.com/Delresearch/WhisperTrack-Software.git
    cd WhisperTrack-Software
    git submodule update --init --recursive
    ```
2. Open the project in VSCode.
3. You will be prompted to reopen the project in a container. Click "Reopen in Container".
    - If you are not prompted, you can manually open the command palette (Ctrl+Shift+P on Windows or Command+Shift+P on MacOS) and select "Dev Containers: Rebuild and Reopen in Container".

## License
This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.