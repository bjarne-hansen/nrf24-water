# NRF24 Water Controller

This is a simple Arduino based water controller for 2 solenoid valves and 2 corresponding water flow sensors.

The water controller reports relay status, water flow per minute, litres during the last duty cycle, and the total number of litres since last reboot.

Data is reported via NRF24L01 communication modules, and the solenoid valves can be opened and closed by sending commands via NRF24L01.

