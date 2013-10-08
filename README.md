# owi-slave

Implementation of an One Wire Interface slave device on an ATTiny 2313A without
external clock.

This code is a work in progress. The Command readout is not working realibly.
A schematic will follow, until then: PD3 is onewire data.

# Building

    make && sudo make flash

## supported features

Successfully tested on a DS2482-100 Single Channel 1-Wire Master.
Tests with a commercially available iButton reader will follow soon.

* Read ROM (0x34) with 64bit identifier

## work in progress

* Search ROM (0xf0) with 64bit identifier

## TODO

* Overdrive Read ROM / Search ROM
