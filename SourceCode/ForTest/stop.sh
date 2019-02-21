###Script to stop In, Out and PWM
#!/bin/bash
echo 47 >/sys/class/gpio/unexport
echo 66 > /sys/class/gpio/unexport
echo 67 > /sys/class/gpio/unexport
echo 69 > /sys/class/gpio/unexport
echo 68 > /sys/class/gpio/unexport
echo 45 > /sys/class/gpio/unexport
echo 44 > /sys/class/gpio/unexport
echo 0 > /sys/class/pwm/pwmchip2/unexport
echo 1 > /sys/class/pwm/pwmchip2/unexport


