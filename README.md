# ADPD4101_arduino_demo
Analog Devices ADPD4101 I2C optical measurement chip demo codes modified for arduino

Several bugs and faults from the original Analog Devices codes are corrected, but the driver files (adpd4101.cpp and adpd4101.h) are basically the original.


## FILES

* adpd4101.cpp & ~.h - driver files (register read/write functions, type declarations of configuration-variables, etc.)
* adpd4101_registers.h - register name definitions
* adpd4101_app.cpp & ~.h - higher level application functions (setup function, start/stop measurement, calibration routins etc.)
* adpd4101_app_config.h - default Timeslot parameters, sampling frequency
* ADPD41001_DEMO.ino - Arduino main file: serial bridge between PC and ADPD4101 sensor with several serial commands:
    * COMMAND param\r\n - command name & new line & carriage return
    * HELP - list of commands
    * INIT - initializaion from adpd4101_app_config.h
    * MEAS - single measurement
    * CONT - continous measurement
    * STOP - stop continous measurement
    * REGR *addr* - read the value of from *addr* register address
    * REGW *addr* *data* *mask* - write *data* value into the *masked* region of *addr* register address



## Current configuration

8 Timeslots are active from 12
* Timeslot A - LED1 24 mA, In2 connected to TIA (gain 12k5) & ADC, 1 repeats (1 pulse)
* Timeslot B - LED2 50 mA, In2 connected to TIA (gain 200k) & ADC, 1 repeats (1 pulse)
* Timeslot C - LED1 1.5 mA, In2 connected to TIA (gain 12k5) & BPF & Integrator & ADC, 6 repeats
* Timeslot D - LED2 50 mA, In2 connected to TIA (gain 12k5) & BPF & Integrator & ADC, 16 repeats
* Timeslot E - LED1 3 mA, In2 connected to TIA (gain 12k5) & BPF & Integrator & ADC, 16 repeats
* Timeslot F - LED2 50 mA, In2 connected to TIA (gain 50k) & BPF & Integrator & ADC, 16 repeats
* Timeslot G - LED1 1.5 mA, In2 connected to TIA (gain 12k5) & BPF & Integrator & ADC, 16 repeats
* Timeslot H - LED2 24 mA, In2 connected to TIA (gain 100k) & BPF & Integrator & ADC, 16 repeats
* Timeslot I - inactive
* Timeslot J - inactive
* Timeslot K - inactive
* Timeslot L - inactive

## common settings:
* LED pulse - 2 us
* Integration width - 3 us
* LED offset - 32 us
* Integration offset - 32 us
* chopper pattern: 0xA = 0101
* TIA Vref - 1.265 V
* data bytes - 4


