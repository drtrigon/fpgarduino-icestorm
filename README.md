## Setup the Arduino IDE for picorv32

This hardware and code was build and tested using icestudio (icestorm) and 
an Alhambra II board. It should work out of the box or be fairly easy to
adopt to other similar boards (e.g. TinyFPGA, iCEBreaker, etc.).

![gpio_adc_hardware.png](https://raw.githubusercontent.com/drtrigon/fpgarduino-icestorm/master/AlhambraII/picorv32/variants/AlhambraII/gpio_adc_hardware.png)


### 1.) unpack (clone) this archive (repo) to ~/sketchbook/hardware/

    $ cd ~/sketchbook/hardware/
    $ wget https://github.com/drtrigon/fpgarduino-icestorm/archive/master.zip
    $ unzip master.zip
    $ mv fpgarduino-icestorm-master/AlhambraII .
    $ rm -rf fpgarduino-icestorm-master

or clone

    $ cd ~/sketchbook/hardware/
    $ git clone https://github.com/drtrigon/fpgarduino-icestorm
    $ mv fpgarduino-icestorm/AlhambraII .
    $ rm -rf fpgarduino-icestorm


### 2.) follow https://github.com/cliffordwolf/picorv32#building-a-pure-rv32i-toolchain

    # Ubuntu packages needed:
    sudo apt-get install autoconf automake autotools-dev curl libmpc-dev \
        libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo \
        gperf libtool patchutils bc zlib1g-dev git libexpat1-dev

    sudo mkdir ~/sketchbook/hardware/AlhambraII/picorv32/tools/riscv32i
    sudo chown $USER ~/sketchbook/hardware/AlhambraII/picorv32/tools/riscv32i

    git clone https://github.com/riscv/riscv-gnu-toolchain riscv-gnu-toolchain-rv32i
    cd riscv-gnu-toolchain-rv32i
    git checkout c3ad555
    git submodule update --init --recursive

    mkdir build; cd build
    ../configure --with-arch=rv32i --prefix=$HOME/sketchbook/hardware/AlhambraII/picorv32/tools/riscv32i
    make -j$(nproc)

Now the Arduino IDE replaces the Makefile used before to compile and upload
the firmware.

This DOES NOT use a bootloader, the compiled code is uploaded and executed
directly. If you want to use a bootloader for uploading consider [9,10,5]
(and [7] too).

In other words: In the Arduino IDE this is like uploading a "bootloader" to
Arduino/AVR (does not use a bootloader but ISP e.g.). Uploading a "sketch" as
the Arduino IDE usualy does (via bootloader) is done within the projects
referred before (FPGArduino and "Programming the TinyFPGA BX with Arduino").

In Arduino IDE "Tools" pulldown menu, select:

    Board: Alhambra II Generic picorv32
    Port: (of the 2 available it's typically the second one, e.g. /dev/ttyUSB1)
    Programmer: iceprog (Icestudio/Icestorm)

Note: The port needs to be set if using the Serial Montor only. For the programming it has no effect.
Note: Forgetting to set the correct programmer results in a java error.


### 3.) OPTIONAL: use iceprog directly from installed icestudio and its toolchain:

    cd ~/sketchbook/hardware/AlhambraII/picorv32/tools/iceprog/
    rm iceprog
    ln -s ~/.icestudio/apio/packages/toolchain-icestorm/bin/iceprog iceprog

or alternatively you can set the following line in platform.txt:

    tools.iceprog.path={runtime.platform.path}/../../../../.icestudio/apio/packages/toolchain-icestorm/bin


### 4.) INFORMATION: The bitstream to use is stored in $HOME/sketchbook/hardware/AlhambraII/picorv32/variants/AlhambraII/

The bitstream can be programmed using: $ iceprog hardware.bin

This bitstream was derived from GPIO_ADC-recent-picorv32.zip using icestudio 0.3.3 (linux64.AppImage).

What does work:
* Verify (building)
* Upload
* Serial Monitor using any baudrate, default is 115200 (needs setting the correct port)
* Arduino IDE examples:
  * 01.Basics
    * BareMinimum: (compiles)
    * Blink: pinMode dummy for leds, digitalWrite on leds also using full 8-bit uint8_t, delay
    * DigitalReadSerial: Serial.println, Serial.begin all baudrates, pinMode (0-7), digitalRead (0-15)
    * AnalogReadSerial: analogRead (A0, 4Hz, bits increased from 8 to 10 by multiplying with 4)
    * ReadAnalogVoltage: (voltage calculation wrong 5.0->3.3)
    * Fade: analogWrite (PWM0 shares pin through OR with LED0, a digitalWrite(LED_BUILTIN, HIGH) "disables" the PWM0 - check hardware - currently pin is ignored as there is only one - also fixed digital output pins are shared and thus disabled or debug when using PWM0)
  * 02.Digital
    * BlinkWithoutDelay: millis, micros, (delay refactored to use micros)
    * Button: (works after changing ledPin 13->LED_BUILTIN)
    * Debounce: (works after changing ledPin 13->LED_BUILTIN)
    * StateChangeDetection: (works after changing ledPin 13->LED_BUILTIN)
    * (TODO: DigitalInputPullup, tone...)
  * 03.Analog
    * AnalogInput: (works after changing ledPin 13->LED_BUILTIN)
    * Smoothing: (works)
    * AnalogInOutSerial: map
    * Calibration: (works)
    * Fading: (works)
    * (AnalogWriteMega could also be implemented)
  * 04.Communication
    * ASCIITable: (works)
    * Graph: (works)
    * VirtualColorMixer: (works with A0 only, A1 and A2 always read 0)
    * Dimmer: Serial.available, Serial.read, Serial._rx_complete_irq
    * PhysicalPixel: (works)
    * SerialCallResponse: (works but the function "establishContact* needs to be declared before "setup" and needs Serial._rx_complete_irq(); within the loop)
    * SerialCallResponseASCII: (works but the function "establishContact* needs to be declared before "setup" and needs Serial._rx_complete_irq(); within the loop)
    * (TODO: ReadASCIIString - parseInt from Stream class, SerialEvent)
  * 05.Control
    * IfStatementConditional: (works after changing ledPin 13->LED_BUILTIN)
    * switchCase: (works)
    * WhileStatementConditional: (works after changing indicatorLedPin 13->LED_BUILTIN and buttonPin 2->14, also the function "calibrate" needs to be declared before "loop")
    * Array: digitalWrite (for gpio and fixed pins)
    * ForLoopIteration: (works)
    * switchCase2: (works)
  * 11.ArduinoISP
    * (TODO: BitBanged/software SPI, USE_OLD_STYLE_WIRING, RESET/PIN_MOSI/PIN_MISO/PIN_SCK/etc. to use gpio (0-7), needs delayMicroseconds - see comment below ...)
  * Other Libraries:
    * SSD1306Ascii: SoftSpi128x64 (compiles, not tested - size big; 157988 - need hardware for testing)
    * (TODO: PJON ???)


### TODO:
    TODO: spi with external hardware like oled, etc.

    (TODO: add support for String class)
    (TODO: serial input drops/misses chars when e.g. transmitting "abcde" - not interrupt based)
    (TODO: add travis integration in order to automatically check whether all examples compile and link properly - functional check has to be done manually)
    (TODO: virtual functions around Print(::write) do not work...)
    (TODO: remove "picorv32: work-a-round" and adopt to arduino template where possible)
    (TODO: gpio with switchable/dynamic pull-up only possible with SB_IO_I3C (rare on some pins only)
          or 2 pins
          https://stackoverflow.com/questions/56517923/ice40-icestorm-fpga-switchable-pullup-on-bi-directional-io-pins/ )
    (TODO: code loading and execution way to slow for delayMicroseconds - a single line of assembler
          code takes around 200 cycles or 16-21 us - reason is slow spi flash as memory - solution is
          faster memory, "[...] create a cache. Or you can just copy all performance-critical code to 
          RAM, or execute from a ROM."
          https://github.com/cliffordwolf/picorv32/issues/126 )
    (TODO: bitbang I2C w/o using interrupts needs delayMicroseconds - solution add I2C hardware
          https://github.com/felias-fogg/SlowSoftWire, https://github.com/felias-fogg/SlowSoftI2CMaster )


### Further info:
* [1] https://github.com/riscv/riscv-tools
* [2] https://github.com/riscv/riscv-wiki/wiki/RISC-V-Software-Status
* [3] https://github.com/FPGAwars/Alhambra-II-FPGA/tree/master/examples/picorv32/picosoc (old, including icestudio project)
* [4] https://github.com/cliffordwolf/picorv32 (recent)
* [5] http://www.nxlab.fer.hr/fpgarduino/
* [6] https://github.com/arduino/Arduino/wiki/Arduino-IDE-1.5-3rd-party-Hardware-specification
* [7] https://github.com/f32c/arduino/issues/32
* [8] https://github.com/FPGAwars/icestudio/issues/321
* [9] https://discourse.tinyfpga.com/t/programming-the-tinyfpga-bx-with-arduino/898
* [10] https://github.com/emard/prjtrellis-picorv32 (mentioned in [7,9])
