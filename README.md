# Flight-Controller

This project requires the following software for Windows 11:

1. [Arm GCC Compiler](https://developer.arm.com/downloads/-/gnu-rm)
2. [CMake](https://cmake.org/download/)
3. [Build Tools for Visual Studio 2022](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2019)
4. [Python 3](https://www.python.org/downloads/windows/)
5. [Git](https://git-scm.com/download/win)

## Downloading the Pico SDK and Example Projects

Use the following commands to download the Pico SDK and example projects:
Create a Pico named folder and type 
```bash
cd Pico 
git clone -b master https://github.com/raspberrypi/pico-sdk
git submodule update --init
git clone -b master https://github.com/raspberrypi/pico-examples
```

## Building the Example Projects
To build the example projects from the command line, use the following commands:
```bash
mkdir build
cd build 
cmake -G "NMake Makefiles" ..
nmake
```