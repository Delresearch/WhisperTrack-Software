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

![whisper-terminal](docs/assets/whisper-terminal.gif)
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

## Modulation
The modulation is based on the [Fontus spec](./docs/fontus_bit_definitions_v0.1.pdf)

samples at 64kHz,
each 2 second acoustic message is 2kBytes long (TODO: uncompressed or compressed?)
framebuffer is 8kbytes large,
center frequency is 25kHz,

## YAML based C++ code generation for unpacking bitfields in Fontus Acoustic Messages (FAM):
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