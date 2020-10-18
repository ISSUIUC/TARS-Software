The data logging code was taken from the hybrid repo. We will be working on it here in preparation for the static pressure test.

### Note!

The `chFifoDataLogger_ver2` directory is a
[Platformio IDE](https://platformio.org/) project configured for a Teensy 3.6.

Platformio is a package that you can install to Atom or VSCode that provides
compilers, debuggers, and other tools for embedded platforms.

It's essentially a MUCH nicer way to write code than the Arduino IDE.

- Teensy source code is found in `chFifoDataLogger_ver2/src/main.cpp`
- ChibiOS/RT Real Time Operating System library found in `lib/ChRt/`
- `daq_gui.py` is repurposed interface with a dynamic plot
(its buggy but it works ¯\_(ツ)_/¯)
