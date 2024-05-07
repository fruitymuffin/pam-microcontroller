# pam-microcontroller
The AVR controller is based on the AVR64DA32 chip which is programmed via the UPDI interface.

## 1. Compiling
The C/C++ code is compiled using avr-gcc/avr-g++ and makes use of avr-libc. [Microchip Studio](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio) was used as the IDE as it integrates the avr-g++ toolchain so building is easy.

## 2. Uploading
The serialUPDI interface was used to upload .hex or .elf files to the chip with the help of [avrdude](https://github.com/avrdudes/avrdude). An avrdude upload command can be added to Microchip Studio as a external tool to make uploading executables easier.
Go to Tools->External Tools and add a new tool. The tool was configured as:

Command:
```
C:\Program Files\avrdude-v7.3-windows-x64\avrdude.exe
```
Arguments:
```
-p avr64da32 -P COM5 -b 115200 -c serialupdi -C "C:\Program Files\avrdude-v7.3-windows-x64\avrdude.conf" -U flash:w:$(BinDir)\led-controller.hex:i
```

Initial Directory:
```
$(BinDir)
```

In order to set fuses with avrdude on the newer DA series chips, it was necessary to manually write it. Luckily the corrent fuse locations are specified in avrdude.conf under the DX family information. To enable the reset pin, the fuse was written by:
```
avrdude -c serialupdi -p avr64da32 -P com5 -U syscfg0:w:0xC8:m
```

## 3. Serial communication
The avr controller has a FTDI 230XS usb to serial chip. Which will typically show up as /dev/ttyUSBn on linux. To write to the port, we can open it as a file using fopen, however to do so we need to know which tty port our device is on. The device is not guaranteed to always be /dev/ttyUSB0.
To be able to access it consistently we can create a symlink to a known filename by creating a udev rule.

Create a file caled "N-my-rule.rules" in ```/etc/udev/rules.d/``` with the contents:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", ATTRS{serial}=="DU0D13LG", SYMLINK+="ttypam-serial"
```

This creates a symlink called ```ttypam-serial``` to the tty port which matches the properties given (idVendor, idProduct, serial), so that we can now use ```/dev/ttypam-serial``` to access our device.

To get the required USB device info use:
```
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}\|{idProduct}\|{idVendor}' | head -n3
```
Where ttyUSB0 is replaced with the tty port of the device. See ```lsusb```.

It might also be necessary to add the user to the dialout group to get permissions to usb devices.