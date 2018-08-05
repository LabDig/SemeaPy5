#!/bin/bash

echo 2 >/sys/class/gpio/export
echo in >  /sys/class/gpio/gpio2/direction

#P8_7