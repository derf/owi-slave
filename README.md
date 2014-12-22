# owi-slave

Implementation of an One Wire Interface slave device on an ATTiny 2313A without
external clock.

At the moment, the code counts time using a busyloop. It is therefore not
suitable for bus-powered applications and requires a powe source. Do NOT use a
mains power supply -- it may not have the same ground level as the 1-Wire bus.

# Building

set the desired onewire address in main.S. then run

    make && sudo make flash

## supported features

### READ ROM (0x33)

Works fine. Successfully tested on:

* DS2482-100 Single Channel 1-Wire Master
* IBL USB iButton reader

### SEARCH ROM (0xf0)

the code in main.c works on one-device buses, the code in main.S was not tested
yet. SEARCH ROM on a multi-device bus does not work.

Successfully tested on:

* DS2482-100 Single Channel 1-Wire Master

## TODO

* test SEARCH ROM
* implement readout of master direction bit in SEARCH ROM
