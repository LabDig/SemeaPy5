#!/bin/bash
echo 0 > /sys/class/pwm/pwmchip2/export
echo 1 > /sys/class/pwm/pwmchip2/export
echo "OK"
#0 is P8.19 and 1 is P8.13
