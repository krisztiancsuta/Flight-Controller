# 🚀 Flight-Controller

This project requires the following software for Windows:

**Source :https://github.com/raspberrypi/pico-setup-windows**
## 🛠️ Pico Setup for Windows

### Installing the tools

Download [the latest release](https://github.com/raspberrypi/pico-setup-windows/releases/latest/download/pico-setup-windows-x64-standalone.exe) and run it.

### 🚀 Starting Visual Studio Code

In your Start Menu, look for the *Pico - Visual Studio Code* shortcut, in the *Raspberry Pi Pico SDK \<version\>* folder. The shortcut sets up the needed environment variables and then launches Visual Studio Code.

### 📚 Opening the examples

The first time you launch Visual Studio Code using the Start Menu shortcut, it will open the [pico-examples](https://github.com/raspberrypi/pico-examples) repository.

To re-open the examples repository later, you can open the copy installed at `C:\Users\<user>\Documents\Pico-<version>\pico-examples`.

### 🏗️ Building an example

Visual Studio Code will ask if you want to configure the pico-examples project when it is first opened; click *Yes* on that prompt to proceed. You will then be prompted to select a kit -- select the *Pico ARM GCC - Pico SDK Toolchain with GCC arm-none-eabi* entry. If the *Pico ARM GCC* entry is not present, select *Unspecified* to have the SDK auto-detect the compiler.

### 🐞 Debugging an example

The `pico-examples` repository comes with `.vscode\*.json` files configured for debugging with Visual Studio Code. You can copy these files into your own projects as well.

### 📌 Wiring up SWD and UART to Picoprobe

Picoprobe wiring is explained in the [Getting started document](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf), *Appendix A: Using Picoprobe*, under the heading *Picoprobe Wiring*.

### 📟 Open serial monitor in VSCode

The SDK installer adds the *Serial Monitor* extension to Visual Studio Code. Picoprobe includes a USB-serial bridge as well; assuming that you have wired up the TX and RX pins of the target to Picoprobe as described previously, you should have an option to select *COMn - USB Serial Device (COMn)* in the *Serial Monitor* tab in the bottom panel.

### 💻 Command-line usage

To build and debug projects using command-line tools, you can open a terminal window using the *Pico - Developer Command Prompt* or *Pico - Developer PowerShell* shortcuts.

### 🆕 Creating a new project

The commands below are for PowerShell, and will need to be adjusted slightly if you're using Command Prompt instead.

### 🗑️ Uninstalling

Open the *Apps and Features* section in Windows Settings, then select *Raspberry Pi Pico SDK \<version\>*. Click the *Uninstall* button and follow the prompts.

    # initialize the Raspberry Pi Pico SDK
    pico_sdk_init()

    # rest of your project
    ```

4.  Write your code (see
    [pico-examples](https://github.com/raspberrypi/pico-examples) or the
    [Raspberry Pi Pico C/C++ SDK](https://rptl.io/pico-c-sdk)
    documentation for more information)

    About the simplest you can do is a single source file (e.g.
    hello_world.c)

    ``` c
    #include <stdio.h>
    #include "pico/stdlib.h"

    int main() {
        setup_default_uart();
        printf("Hello, world!\n");
        return 0;
    }
    ```

    And add the following to your `CMakeLists.txt`:

    ``` cmake
    add_executable(hello_world
        hello_world.c
    )

    # Add pico_stdlib library which aggregates commonly used features
    target_link_libraries(hello_world pico_stdlib)

    # create map/bin/hex/uf2 file in addition to ELF.
    pico_add_extra_outputs(hello_world)
    ```

    Note this example uses the default UART for *stdout*; if you want to
    use the default USB see the
    [hello-usb](https://github.com/raspberrypi/pico-examples/tree/master/hello_world/usb)
    example.

5.  Launch VS Code from the *Pico - Visual Studio Code* shortcut in the
    Start Menu, and then open your new project folder.

6.  Configure the project by running the *CMake: Configure* command from
    VS Code's command palette.

7.  Build and debug the project as described in previous sections.

## Uninstalling

Open the *Apps and Features* section in Windows Settings, then select
*Raspberry Pi Pico SDK \<version\>*. Click the *Uninstall* button and
follow the prompts.
