# owi-slave

Implementation of an One Wire Interface slave device on an ATTiny 2313A without
external clock. Work-in-progress.

Though the schematic suggests bus-powered operation, the AVR's power
consumption may be too high for this to work. If connecting the AVR causes the
bus voltage to drop below 3V, consider replacing the capacitors with a ~4.5V
battery. Do not use a mains power supply -- it may not have the same GND as
the 1-Wire bus.

# Building

    make && sudo make flash

## supported features

Successfully tested on a DS2482-100 Single Channel 1-Wire Master.
Tests with a commercially available iButton reader will follow soon.

* Read ROM (0x33) with 64bit identifier
* Search ROM (0xf0) with 64bit identifier

## TODO

* Overdrive Read ROM / Search ROM
