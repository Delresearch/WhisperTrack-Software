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
    
After the build completes, you will find the built application in the `build_stm/stm32` directory. You can then flash it to your STM32 device using your preferred method.

![whisper-terminal](docs/assets/whisper-terminal.gif)
## Development
To help with development, we provide a Dockerfile and a .devcontainer that set up a development environment with all the necessary dependencies. This allows you to work on the project without needing to install all dependencies on your local machine.
### Prerequisites
- Docker
- VSCode
- [Dev Containers extension for VSCode](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- J-Link software installed on your host machine (only needed for debugging in vscode).
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

![whisper-devcontainer](docs/assets/whisper-devcontainer.gif)
### Debugging
To debug the application, you need to run a gdb server using a Segger j-link device and software. [Download](https://www.segger.com/downloads/jlink/) the J-Link software and install it on your host machine. Then, run the following command to start the gdb server:
```bash
JLinkGDBServer -if swd -port 50000 -swoport 50001 -telnetport 50002 -device STM32L496VG
```
You should see at the end of the output:
```bash
Waiting for GDB connection...
```
Once the GDB server is started, you can use the VSCode debugger "Cortex Debug (Remote J-Link Dev Container)" to connect to the gdb server.
##### Note
If your device is not flashing, you may need to . You can do this using the `JLinkSTM32` utility in the cli then selecting `18` from the list or use the JFlashLite gui application and select `Erase Chip`. 
## Modulation
The modulation is based on the [Fontus spec](./docs/fontus_bit_definitions_v0.1.pdf)

- Samples at 64kHz,
- 2 second acoustic message is 2kBytes long?
- Framebuffer is 8kBytes large, center frequency is 25kHz.

## YAML based C++ code generation
For unpacking bitfields in Fontus Acoustic Messages (FAM) we use a YAML based code generation approach. This allows us to define the bitfields in a human-readable format and automatically generate the corresponding C++ code for handling these bitfields.
To ensure the bit field `get` and `set` functions are accurate according to the latest version of the [Fontus bit definitions](./docs/fontus_bit_definitions_v0.1.pdf) we make updating the bitfield defined in a Fontus Acoustic Message (FAM) as easy as possible. 

By generating C++ code from a [YAML](./codegen/bitstream.yml) file of FAM bitfields upon every build, adding a new Message to the bitfield handling code is as straight forward as updating the YAML

The following snippet is an example of some YAML file configurations
```YAML
ReleaseRequest:
  - Type:
      - size: 6  # The Type bitfield is always six bits, this is true for all fontus acoustic message
        value: 2 # This FAM message has default value of 2, this value must be unique to each FAM
  - Device_ID:
      - size: 24
  - Reserved_bits_22:
      - size: 22
  - CRC:
      - size: 12

ReleaseReport:
  - Type:
      - size: 6
        value: 3 # This FAM message has default value of 3, this value must be unique to each FAM
  - Device_ID:
      - size: 12
  - Activation_status:
      - size: 6
  - Reserved_bits_28:
      - size: 28
  - CRC:
      - size: 12
```
```h++
// ReleaseRequest Table
const BitstreamParameter ReleaseRequest[] = {
    {"Type", 58, 6},
    {"Device_ID", 34, 24},
    {"Reserved_bits_22", 12, 22},
    {"CRC", 0, 12},
};

// ReleaseReport Table
const BitstreamParameter ReleaseReport[] = {
    {"Type", 58, 6},
    {"Device_ID", 46, 12},
    {"Activation_status", 40, 6},
    {"Reserved_bits_28", 12, 28},
    {"CRC", 0, 12},
};
```
This is gettable and settable with the `get("releasereport",uint64 *status)`

## More information
For more information on how to use the WhisperTrack software, please refer to the [WhisperTrack website](https://whispertrackdocs.popotomodem.com).
## License
This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.