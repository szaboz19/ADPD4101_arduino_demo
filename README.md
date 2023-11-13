# ADPD4101_arduino_demo
Analog Devices ADPD4101 I2C optical measurement chip demo codes modified for arduino

Several bugs and faults from the original Analog Devices codes are corrected, but the driver files (adpd4101.cpp and adpd4101.h) are basically the original.


## FILES

* adpd4101.cpp & ~.h - driver files (register read/write functions, type declarations of configuration-variables, etc.)
* adpd4101_registers.h - register name definitions
* adpd4101_app.cpp & ~.h - higher level application functions (setup function, start/stop measurement, calibration routins etc.)
* adpd4101_app_config.h - default Timeslot parameters, sampling frequency
* ADPD41001_DEMO.ino - Arduino main file: serial bridge between PC and ADPD4101 sensor with several serial commands:
    * COMMAND\r\n - command name & new line & carriage return
    * HELP - list of commands
    * INIT - initializaion from adpd4101_app_config.h
    * MEAS - single measurement
    * CONT - continous measurement
    * STOP - stop continous measurement
    * REGR *addr* - read the value of from *addr* register address
    * REGW *addr* *data* *mask* - write *data* value into the *masked* region of *addr* register address

