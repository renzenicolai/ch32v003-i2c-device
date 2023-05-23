# I2C peripheral

The code in this repository is my attempt at using the I2C peripheral in the CH32V003 as an I2C slave.

The amount of registers can be changed to any amount desired (1-256), the amount is set to 16 for now.

# Status

Has some issues that need to be addressed, I've tested using a Micropython device as master and the device doesn't always respond to requests yet. Try scanning the bus a couple of times or read from the device and eventually it will work.
