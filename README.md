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
    - A cable splitter
    - An RS422 cable, used with Modbus and the SDK (yellow)
    - An RS485 cable, used with Iris Controls (blue)

### Knowledge
- Some basic programming knowledge, while not expressly required, will be very helpful. The tutorials will assume that you have written a (possibly very simple) program before. Experience with C++ is particularly useful.

## Setting Up Hardware and Testing Your Motor

Before continuing onto any software development, make sure that you have read through and followed the steps in the Orca Series Quickstart Guide, which can be found on [our downloads page](https://irisdynamics.com/downloads). We recommend testing your motor's functionality through IrisControls, also hosted on our downloads page, before beginning to operate your motor through the SDK.

### Windows

On Windows, each cable has a built in latency of 16ms between messages by default. For the blue cable, this is fine. But to enable high speed communication with your motor, this latency setting should be reduced as much as possible on the yellow cable. To update this setting, follow these steps:
 - Ensure your cables are connected to your computer
 - Open Device Manager
 - Navigate to "Ports (COM & LPT)" and expand the dropdown menu
 - Right click on the COM port for your yellow cable and select "Properties"
    - If you do not know which COM port number corresponds to your yellow cable, try unplugging and plugging back in your cable. The COM port you're looking for should disappear and reappear in the dropdown options
 - Under the "Port Settings" tab of the properties window, select "Advanced"
 - Set the "Latency Timer (msec)" option to a value of 1
 - Select Ok to confirm your selections

Keep a note of what the COM port number for your yellow cable is. You will need it for each time you want to connect to your motor through the SDK.

### Linux

When you plug in a yellow or blue cable to your Linux device, it will appear as a file matching the pattern /dev/ttyUSB{x} with x being an arbitrary number incrementing from zero. When this file is created, access to it will be restricted to the superuser. The file permissions can be adjusted using the chmod command. An example command (we'll assume it receives the name ttyUSB0) exposing this file for reading and writing to all users is:

```
sudo chmod 666 /dev/ttyUSB0
```

Additionally, your yellow cable should have its serial parameters updated to minimize port latency. This can be done using the [setserial](https://linux.die.net/man/8/setserial) command, which should be available to your package manager. Once installed the command (once again assuming the name ttyUSB0) to reduce your serial port latency is:

```
setserial /dev/ttyUSB0 low_latency
```

Linux doesn't save parameters for these devices by default, which means that each time you turn on your computer or plug in your device, your cables will be assigned a new ttyUSB file, which may not have the same number as the last time you used it. In addition, because this is a new file, you will need to repeat the previous two commands again. If you don't want to locate your device and repeat these commands regularly, you can set up automatic configuration by writing up [udev .rules files](https://www.freedesktop.org/software/systemd/man/latest/udev.html) for your device.

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
