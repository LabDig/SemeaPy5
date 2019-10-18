******************************
Open source software developed by seed variable rate application controller.
Developed in Python 3.6 language, and using PyQt5 tools to build the Graphical User Interface(GUI)

SemeaPy5 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

SemeaPy5 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with SemeaPy5.  If not, see <https://www.gnu.org/licenses/>6.
******************************
Authors of software

M.Sc Andre Luiz de Freitas Coelho (Federal University of Vicosa).Email:andre.coelho@ufv.br

PhD Daniel Marcal de Queiroz (Federal University of Vicosa). Email:queiroz@ufv.br

PhD Francisco de Assis de Carvalho Pinto (Federal University of Vicosa).

D. Sc Domingos Sarvio Magalhes Valente (Federal University of Vicosa).

******************************
The controller was developed using the BeableBone Black single board computer
The Adafruit BBIO module was used to control GPIO and PWM pin
A Arduino Pro Mini 3.3 V 8 Mhz was used to calculate planter speed and angular velocity of electric motor
******************************
The componentes used was:
BeableBone Black Revision C, with oficial Linux Debian 8.6 operational system
Touch Sensitive 7 inch HDMI LCD Screen
Arduino ProMini 3.3V hMhz
Low Cost GNSS Module Ublox NEO6M
BTS 7690 motor controller
******************************
Directory and Files Description:
******************************
Directory SemeaPy 5 must be past in /root

arduino/arduino.ino

This file it is the software running in Arduino ProMini for read optical switch signal and calculate planter spped and angular velocity of electric motor.

Mapas/sowing_map.*

This files it is the sowing map used in field test. The shapefile was build in QGIS 2.18.


SourceCode/*.png

The png files it is the icons used in Graphical User Interface (GUI)

ClickableLineEdit.py

This script it is used to add a mouse event in Line Edit. In the software, a virtual numeric keyboard openned when it is clicked over Line Edit. 
The numeric keyboard it is used to configure operation parameters, such as row spacing and seed germination percentage.

conf.txt

This file it is used to store the configuration used in operation. When the software it's openned the configuration contained in conf.txt it's loaded. 
When the software it's closed, the actual configuration it is saved in conf.txt. So, the software always open with the last configuration.

keyboard.py

This file contain the keyboard interface. It was build by keyboard.ui, using pyuic5 command of PyQt5 tool

operation.py
This file contain functions that run calculations for the sowing operation.


seeder_ui.py

This file contain the sofware interface. It was build by seeder_ui.ui, using pyuic5 command of PyQt5 tool

SemeaPy.Py

It is the main file of software. This script contain functions to add action for all GUI elements's. besides to control the sowing operation.

