###Enable uart 4 in P9.11 and P9.13 pin and
### PWM2 P8.13 e P8.19
### In : P9_23 (49), P8_7 (66), P8_8 (67) , P8_9 (69), P8_10 (68)
### Out : P8_11 (45)  P8_12 (44)

#/!bin/bash

echo BB-UART4 > /sys/devices/platform/bone_capemgr/slots
echo BB-PWM2 > /sys/devices/platform/bone_capemgr/slots
echo 49 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio49/direction
echo 66 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio66/direction
echo 67 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio67/direction
echo 69 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio69/direction
echo 68 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio68/direction
echo 45 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio45/direction
echo 44 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio44/direction
echo 0 > /sys/class/pwm/pwmchip2/export
echo 1 > /sys/class/pwm/pwmchip2/export
echo 1000000 > /sys/class/pwm/pwmchip2/pwm0/period
echo 1000000 > /sys/class/pwm/pwmchip2/pwm1/period
echo 1 > /sys/class/pwm/pwmchip2/pwm0/enable
echo 1 > /sys/class/pwm/pwmchip2/pwm1/enable
