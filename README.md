# orcaSDK
## Introduction
Welcome to the orcaSDK repo! The goal of the SDK is to help users who want to create custom software for controlling their Orca series linear motor. 

Not all use cases require custom software for controlling an Orca motor. It's possible that a tool already exists that handles your use case. Other options include:

- The Orca motor's built in GUI accessible through [IrisControls](https://irisdynamics.com/products/software).
- Our [MATLAB](https://irisdynamics.com/hubfs/Website/Downloads/Orca/Approved/UG230704_Orca_Series_Modbus_RTU_with_MATLAB.pdf) and [LabVIEW](https://irisdynamics.com/hubfs/Website/Downloads/Orca/Approved/UG230713_Orca_Series_Modbus_RTU_with_LabVIEW.pdf) packages
- One of our SmartHubs.

The benefits of using the SDK include:
- High speed serial communication with the motor
- Programmatic control over the motor using every operational mode available

 If after considering your options custom software seems like the appropriate solution, then this repo is for you! The SDK includes a set of C++ libraries to help with setting up and controlling the motor, as well as tools for common use cases. This repo contains tutorials and examples to help with understanding how to use these libraries to get your software up and running quickly!

Currently, these tools only support applications running on the Windows operating system. 

## Prerequisites
To follow these tutorials you will need the following tools:

### Software
- [A Git client](https://git-scm.com/downloads). We will be using the Windows command line client Git BASH.
- A C++ build system. For windows your most appropriate option is likely [CMake](https://cmake.org/) and/or [Microsoft Visual Studio](https://visualstudio.microsoft.com/).
    
### Hardware
- An Orca series linear motor, and [any additional required components](https://irisdynamics.com/hubfs/Website/Downloads/Orca/Approved/UG220206_Orca_Series_Quickstart_Guide.pdf). A quick checklist of the dependencies includes:
    - An appropriate power supply
    - A cable splitter
    - An rs422 cable
    - An rs485 cable

### Knowledge
- Some basic programming knowledge, while not expressly required, will be very helpful. These tutorials will assume that you have written a (possibly very simple) program before. Experience with C++ is particularly useful.

## Setup
To set up this project, first clone this repo using whatever git client you have:

```
git clone git@github.com:IrisDynamics/orcaSDK.git
```
or
```
git clone https://github.com/IrisDynamics/orcaSDK.git
```



## Quick start
If you're starting out with our motors for the first time, take a look at the [QuickStartTutorial] document and follow along with the steps. If you've completed this tutorial but need more examples for your use case, browse our [other tutorials](#whats-next), or [contact us](https://irisdynamics.com/support) for more assistance.


## What's next?
Once you've worked through the Quick Start tutorial, there are a series of more advanced tutorials that demonstrate simple solutions to common use cases. Please browse the list below and see if any of these tutorials apply to you!

| Tutorial Name | Description |
| --- | --- |
| [Advanced tutorial 1]() | In this tutorial you will ...
| [Advanced tutorial 2]() | In this tutorial you will ...

## Appendices
[Iris Dynamics Ltd. documentation and resources](https://irisdynamics.com/downloads)