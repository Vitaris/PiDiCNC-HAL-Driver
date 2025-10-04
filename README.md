# PiDiCNC LinuxCNC Driver

A lightweight LinuxCNC HAL userspace driver for PiDiCNC motion/IO boards, designed specifically for LinuxCNC integration on Raspberry Pi systems.

## Overview

This driver provides an affordable and reliable solution for hobby CNC enthusiasts and retrofitting older CNC machines. The compact Raspberry Pi form factor makes it ideal for smaller electrical cabinets where traditional PC-based systems won't fit, offering a space-efficient alternative to classic industrial controllers.

## Supported Hardware

The driver automatically detects and configures three types of PiDiCNC boards:

- **Board 3805** - Stepper motor, digital I/O, and analog output for the spindle controler
- **Board 3809** - Digital I/O expansion board  
- **Board 3811** - Multi-axis encoder and analog I/O board

## Key Features

### Board 3805 Capabilities
- **Stepper Motors**: 4 native axes plus up to 3 additional time-multiplexed axes
- **Digital Outputs**: 11 channels with multiple modes (static, PWM, RC servo, or stepper reassignment)
- **Digital Inputs**: Up to 5 configurable input channels
- **Analog Output**: 1 DAC channel (0-10V with scaling and limits)
- **Status LEDs**: 9 software-controlled indicators

### Board 3809 Capabilities  
- **Digital I/O**: 16 outputs and 16 inputs with inversion support
- **Status LEDs**: 4 indicator channels

### Board 3811 Capabilities
- **Servo Control**: 4 groups, each with ±5V analog output for speed control and encoder feedback for position
- **Plasma Cutting Control**: 1 DAC for current control and 1 ADC for arc length sensing (optional)
- **Status LEDs**: 9 software-controlled indicators

## Platform Support

Compatible with Raspberry Pi generations 1-4 (tested on 3b+, 4) using direct memory-mapped SPI access for optimal performance.

## Safety Features

Global emergency stop functionality gates all motion and analog outputs, with automatic reconfiguration when safety conditions change.

## Installation

Copy the compiled pidi.so file to the LinuxCNC modules directory. The pidi.so file is located in the Releases section.
```
cp pidi.so /usr/lib/linuxcnc/modules/
```
## License

GNU GPL v2 (see source file headers for full details)
