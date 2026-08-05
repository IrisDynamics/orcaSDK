# orcaSDK

## Contents
- [Introduction](#introduction) 
- [Prerequisites](#prerequisites) 
    - [Software](#software) 
    - [Hardware](#hardware) 
    - [Knowledge](#knowledge) 
- [Setting Up Hardware and Testing Your Motor](#setting-up-hardware-and-testing-your-motor) 
    - [Windows](#windows) 
    - [Linux](#linux) 
- [Building the SDK](#building-the-sdk) 
    - [Create a CMakeLists.txt file](#create-a-cmakeliststxt-file) 
    - [Create main.cpp](#create-maincpp)
- [Compile and Run Your Application](#compile-and-run-your-application)
    - [Visual Studio](#visual-studio)
    - [Command Line](#command-line)
- [(Optional) Download the Documentation](#optional-download-the-documentation)
- [Whats Next?](#whats-next)

## Introduction
Welcome to the orcaSDK repo! The goal of the SDK is to help users who want to create custom software for controlling their ORCA series linear motor. 

Not all use cases require custom software for controlling an ORCA motor. It's possible that a more appropriate tool exists for your use case. See a list of available options on the [Getting Started](https://irisdynamics.com/getting-started-resources) page of our website.

The benefits of using the SDK include:
- High speed serial communication with the motor
- Programmatic control over the motor using every operational mode available

If after considering your options custom software seems like the appropriate solution, then this repo is for you! The SDK is a C++ library designed to provide an easy to use and understand abstraction for your motor.

> If you prefer Python to C++, we have [a version of this library available in Python](https://pypi.org/project/pyorcasdk/).

## Prerequisites
To use the SDK you will need the following tools:

### Software
- A C++ build system. 
    - For Windows the most appropriate option is likely MSBuild, which comes packaged with [Microsoft Visual Studio](https://visualstudio.microsoft.com/).
    - For Linux the most appropriate option is likely GCC.
- [CMake](https://cmake.org/)
    - If using Visual Studio, this is installed as part of the 'Desktop development for C++' workload.
- Installation of git client is necessary to help CMake build succesfully. Git can be downloaded at the following [link](https://git-scm.com/).
- A IDE or text editor of your choice
    
### Hardware
- An Orca series linear motor, and [any additional required components](https://irisdynamics.com/hubfs/Website/Downloads/Orca/Approved/UG220206_Orca_Series_Quickstart_Guide.pdf). A quick checklist of the dependencies includes:
    - An appropriate power supply
    - An ORCA-USB or cable splitter
    - An RS422 interface, used with Modbus and the SDK (port A of ORCA-USB, or a yellow cable in port 2 of cable splitter)
    - An RS485 interface, used with Iris Controls (port B of ORCA-USB, or a blue cable in port 1 of cable splitter)

### Knowledge
- Some basic programming knowledge, while not expressly required, will be very helpful. The tutorials will assume that you have written a (possibly very simple) program before. Experience with C++ is particularly useful.

## Setting Up Hardware and Testing Your Motor

Before continuing onto any software development, make sure that you have read through and followed the steps in the Orca Series Quickstart Guide, which can be found on [our downloads page](https://irisdynamics.com/downloads). We recommend testing your motor's functionality through IrisControls, also hosted on our downloads page, before beginning to operate your motor through the SDK.

### Windows

On Windows, each [interface](#hardware) has a built in latency of 16ms between messages by default. For the RS485 interface, this is fine. But to enable high speed communication with your motor, this latency setting should be reduced as much as possible on the RS422 interface. To update this setting, follow these steps:
 - Ensure your ORCA motor is connected to your computer and powered on
 - Open Device Manager
 - Navigate to "Ports (COM & LPT)" and expand the dropdown menu
 - Right click on the COM port for your RS422 interface and select "Properties"
   - If using an ORCA-USB, the RS422 interface can be identified by the 'Location' property of the general tab reading "on USB Serial Converter A"
   - If using a cable splitter, the RS422 interface can be identified by unplugging and plugging back in your cable while monitoring the "Ports (COM & LPT)" dropdown. The COM port you're looking for should disappear and reappear in the dropdown options
 - Under the "Port Settings" tab of the properties window, select "Advanced"
 - Set the "Latency Timer (msec)" option to a value of 1
 - Select Ok to confirm your selections

Keep a note of what the COM port number for your RS422 interface is. This is the port to use when interacting with your motor through the SDK.

### Linux

When you connect an [RS422 or RS485 interface](#hardware) to your Linux device, it will appear as a file matching the pattern /dev/ttyUSB{x} with x being an arbitrary number incrementing from zero (if using ORCA-USB it will be two files). When this file is created, access to it will be restricted to the superuser, and port latency will default to 16ms. We recommend configuring these values by setting up [udev .rules files](https://www.freedesktop.org/software/systemd/man/latest/udev.html). The following is a set of sample udev rules which give read write access to all users and set port latencies to 1ms for all supported RS422 interfaces:

```
SUBSYSTEM=="usb-serial",DRIVER=="ftdi_sio",ATTR{latency_timer}="1"
SUBSYSTEM=="tty",SUBSYSTEMS=="usb-serial",DRIVERS=="ftdi_sio",MODE="0666"
```

> Note that these settings will apply to all FTDI USB-to-serial devices. If you're using other FTDI USB-to-serial devices which cannot have these settings, you will need to modify these rules.

To set this up, simply place these sample rules in a .rules file (E.g. '99-ORCA.rules') inside an appropriate directory (E.g. /etc/udev/rules.d/). Then reload the rules using:

```
udevadm control --reload-rules
udevadm trigger
```

> These steps will likely require root privilege to perform.

## Building the SDK

The goal for this section is to illustrate how to create and compile a basic project that uses the orcaSDK. The goal is to get to the point of successful compilation, not for detailed use of the SDK. For tutorials and example projects with such use cases, we have created a separate repo, which we link to at the end of this README. Before going through those tutorials, however, we will assume you have followed the steps listed here for how to create a basic application.

We strongly recommend using CMake to build the SDK. For the vast majority of users we recommend using CMake's FetchContent features which we describe in this section.

> If our recommended build solution is insufficient for your needs, orcaSDK-CMake-Details.md contains relevant details for custom installations.

### Create a CMakeLists.txt file

The basic unit of projects using CMake is the CMakeLists.txt file. To begin, create a file called CMakeLists.txt in the folder in which you'd like to create your application. Open the new file with a text editor of your choice and add the following text:

```CMakeLists.txt
cmake_minimum_required(VERSION 3.23)

project(basicOrcaSDKProj)
```

These two lines are required for any C++ project built using CMake. 

cmake_minimum_required() indicates the minimum CMake version that can be used to build the application. It also enables CMake features up to the version listed and changes some default behaviours. 

project() creates a CMake project and assigns it a name. For the purpose of building an application not meant to be shared, the details of this command isn't of much concern.

Now let's update the CMakeLists.txt file to add an application.

```CMakeLists.txt
...

add_executable(basicOrcaSDKApp
    main.cpp
)
```

This command defines an application target which will result in an executable (.exe on Windows) when the project is built. This command depends on the source file main.cpp. We will describe this file in the next step.

Next we add the commands which download the orcaSDK and its dependencies and prepare it for use.

```CMakeLists.txt
...

include(FetchContent)
FetchContent_Declare(orcaSDK
    GIT_REPOSITORY https://github.com/IrisDynamics/orcaSDK.git
    GIT_TAG main # Or 'v1.1.0' or some commit hash from the SDK
)
FetchContent_MakeAvailable(orcaSDK)
```

The command `include(FetchContent)` makes the CMake FetchContent features available for use in this project. The next command `FetchContent_Declare(orcaSDK ...)` describes where to download the SDK from and what version to use. The `FetchContent_MakeAvailable(orcaSDK)` command downloads and builds the SDK, making it available for use.

Finally, we add one more command to associate the SDK with your executable.

```CMakeLists.txt
...

target_link_libraries(basicOrcaSDKApp PUBLIC orcaSDK::core)
```

`target_link_libraries(basicOrcaSDKApp ...)` indicates to CMake that your application is a client of the SDK, and requires access to the SDK's include paths and compiled object files.

At this point we have a simple CMakeLists.txt file describing an application which makes use of the SDK. Your CMakeLists.txt file should look something like this:

```CMakeLists.txt
cmake_minimum_required(VERSION 3.23)

project(basicOrcaSDKProj)

add_executable(basicOrcaSDKApp
    main.cpp
)

include(FetchContent)
FetchContent_Declare(orcaSDK
    GIT_REPOSITORY https://github.com/IrisDynamics/orcaSDK.git
    GIT_TAG main # Or 'v1.1.0' or some commit hash from the SDK
)
FetchContent_MakeAvailable(orcaSDK)

target_link_libraries(basicOrcaSDKApp PUBLIC orcaSDK::core)
```

### Create main.cpp

Next let's create a very simple main.cpp which makes use of the SDK, and build it to test if our system is working.

```main.cpp
#include <iostream>
#include "actuator.h"

int main()
{
    orcaSDK::Actuator motor{ "MotorName" };
    std::cout << "Hello World\n";
    return 0;
}
```

For now don't worry about the contents of this source file.

## Compile and Run Your Application

Building the application can be done in a few ways. For Windows users we recommend interacting with your project through Visual Studio. 

If on Linux, or if you don't want to use Visual Studio, you can also build your app using the command line.

### Visual Studio

To build a CMake app though Visual Studio follow these steps:
 - Open Visual Studio, and on the project selection window, select the option "Open a local folder". 
 - Select the folder that contains your CMakeLists.txt file. Visual Studio should recognize the project as a CMake project upon opening and configure itself appropriately. 
 - To interact with your project, right click on any item in the Solution Explorer, and select "Switch to CMake Targets View". 
 - To build your application, expand the dropdown menu for your project and right click on your application. Select "Set as Startup Item" then click the play button in the top.

For further information regarding CMake projects in Visual Studio, [CMake Project in Visual Studio](https://learn.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=msvc-170) is a helpful resource.

### Command Line

If building your app through the command line, simply use the following commands.

```
mkdir build
cd build
cmake ..
cmake --build .
```

If the cmake commands complete without displaying error messages, then you're done! The resulting executable should be placed in either the Debug or Release directory generated within your build directory, or within the build directory itself.

## (Optional) Download the Documentation

We recommend downloading the documentation package, found in the releases section of this Github repo. To open the documentation, unzip the package and open the documentation.html file with any browser, located within the file's top-level directory.

## What's Next?

Take a look at our [Tutorial Repo](https://github.com/IrisDynamics/orcaSDK_tutorials) for tutorials on how to handle some of the most common use cases for Orca motors.
