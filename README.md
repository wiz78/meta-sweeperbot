# meta-sweeperbot

Yocto layer for the Sweeperbot project.

## Dependencies

This layer depends on:

* URI: git://git.yoctoproject.org/poky
    * branch: master
    * revision: HEAD

* URI: git://git.openembedded.org/meta-openembedded
    * branch: master
    * revision: HEAD

* URI: https://github.com/agherzan/meta-raspberrypi
    * branch: master
    * revision: HEAD

## Quick Start

1. source poky/oe-init-build-env build
2. Add this layer to bblayers.conf and the dependencies above
3. Set DISTRO in local.conf to sweeperbot
4. Set MACHINE in local.conf to one of the supported boards (presumably raspberrypi3-64)
5. Edit recipes-connectivity/wpa-supplicant/files/wpa_supplicant-wlan0.conf.dist as required
6. bitbake sweeperbot-image
7. Use Balena Etcher (for instance) to copy the generated .wic file to the SD card
8. Boot your RPI


## Remote control

When the bot is connected to your WLAN, it'll serve a web page at http://sweeperbot.local:8000/ which you
can use to control the robot and to update the ESP32 firmware.
