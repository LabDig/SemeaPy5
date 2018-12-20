#!/bin/bash

echo 49 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio49/direction

#P9_23 (1_17 =49)
