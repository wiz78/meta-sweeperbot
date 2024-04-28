#!/bin/bash

/usr/sbin/swapoff -a

echo "1-1" > /sys/bus/usb/drivers/usb/unbind