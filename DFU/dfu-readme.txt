## General Instructions ##

Make sure BOOT0 jumper/switch is on while powering up the mcu.
BOOT0 -> HIGH
BOOT1 -> LOW

'''LINUX'''


#Add udev rules

sudo gedit /etc/udev/rules.d/stm32.rules

Copy paste following content and save:

#STM32F4 Dsicovery in USB Serial Mode (CN5)
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{MTP_NO_PROBE}="1"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666"
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666"
# 0483:df11 - STM32F4 Discovery in DFU mode (CN5)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE:="0666"

sudo restart udev




'''Eclipse Instructions'''

*Run > External Tool > External Tool Config > New Program
*Select location of dfu-util at 'Location'
*select 'build' directory location at 'Working Directory'
*Put following lines in Argument tab

-d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D nayan_slave_processor.bin

*Disable AutoBuild from Build tab.

Once done, launch this from external tool menu to flash.


