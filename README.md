# PiDiCNC-HAL-Driver
Driver for communication with PiDiCNC boards over SPI interface

Currently it supports Raspberry Pi 2, 3, 3B versions

Todo:
* Platform check have to be changed, checking thru cpuinfo is not right because all last versions has same label BCM2835. Change it to check info via cat /sys/firmware/devicetree/base/model
2220: platform_t check_platform(void)
