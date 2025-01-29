## Adding New Board Configuration to boards.txt

To use the custom board settings for "Arduino Mini (Prescale /16 - 1MHz)", you need to update your `boards.txt` file with the following configuration:

1. Locate the `boards.txt` file in your Arduino installation directory. On most systems, this will be in:

   - Windows: `C:\Program Files (x86)\Arduino\hardware\arduino\avr\boards.txt`
   - macOS: `/Users/{yourname}/Library/Arduino15/packages/arduino/hardware/avr/{version}/boards.txt`
   - Linux: `/usr/share/arduino/hardware/arduino/avr/boards.txt`

2. Open `boards.txt` in a text editor with administrative privileges.

3. Add the following lines at the end of the file:

   ```plaintext
   ##############################################################

   mini1.name=Arduino Mini (Prescale /16 - 1MHz)

   mini1.upload_port.0.board=mini

   mini1.upload.tool=avrdude
   mini1.upload.tool.default=avrdude
   mini1.upload.tool.network=arduino_ota
   mini1.upload.protocol=arduino

   mini1.bootloader.tool=avrdude
   mini1.bootloader.tool.default=avrdude
   mini1.bootloader.low_fuses=0xff
   mini1.bootloader.unlock_bits=0x3F
   mini1.bootloader.lock_bits=0x0F

   mini1.build.f_cpu=1000000L
   mini1.build.board=AVR_MINI
   mini1.build.core=arduino
   mini1.build.variant=eightanaloginputs

   ## Arduino Mini w/ ATmega328P
   ## --------------------------
   mini1.menu.cpu.atmega328=ATmega328P

   mini1.menu.cpu.atmega328.upload.maximum_size=28672
   mini1.menu.cpu.atmega328.upload.maximum_data_size=2048
   mini1.menu.cpu.atmega328.upload.speed=115200

   mini1.menu.cpu.atmega328.bootloader.high_fuses=0xd8
   mini1.menu.cpu.atmega328.bootloader.extended_fuses=0xFD
   mini1.menu.cpu.atmega328.bootloader.file=optiboot/optiboot_atmega328-Mini.hex

   mini1.menu.cpu.atmega328.build.mcu=atmega328p
   ```
