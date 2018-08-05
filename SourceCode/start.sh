###enable uart 4 in P9.11 and P9.13 pin and
### PWM2 P8.13 e P8.19
#/!bin/bash

echo BB-UART4 > /sys/devices/platform/bone_capemgr/slots
echo BB-PWM2 > /sys/devices/platform/bone_capemgr/slots
echo "UART Started"