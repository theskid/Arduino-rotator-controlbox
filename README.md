# Arduino Rotator Controlbox

This project is using third party open-source software, specifically [**UTFT**](http://www.rinkydinkelectronics.com/library.php?id=51), [**UTFT_Geometry**](http://www.rinkydinkelectronics.com/library.php?id=59) and [**default fonts**](http://www.rinkydinkelectronics.com/r_fonts.php) by [**Henning Karlsen**](http://www.rinkydinkelectronics.com/), [**ResponsiveAnalogRead**](https://github.com/dxinteractive/ResponsiveAnalogRead) by [**Damien Clarke**](http://damienclarke.me/code/posts/writing-a-better-noise-reducing-analogread).

## Library dependencies

[**UTFT**](http://www.rinkydinkelectronics.com/library.php?id=51) – A universal library that supports a large number of display modules and controllers

[**UTFT_Geometry**](http://www.rinkydinkelectronics.com/library.php?id=59) – Adds some geometric functions to UTFT library

[**ResponsiveAnalogRead**](https://github.com/dxinteractive/ResponsiveAnalogRead) – Analog read error correction through exponential moving average

These libraries need to be installed in your Arduino compiler library directory.

## Compiling

In order to compile this project, you need to create and edit a configuration file (**Settings.h**). You can find a sample configuration provided with the project ([**Settings_sample.h**](https://github.com/theskid/Arduino-rotator-controlbox/blob/master/Settings_sample.h)).

ARC is still a WIP and currently developed and tested exclusively on **ATMega 2560**.
