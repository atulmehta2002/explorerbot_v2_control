This Package will only work with Raspberry Pi Hardware
Need to build WiringPi Package from source

Use these 2 commands to change the access of I2C in RPI:

sudo chmod g+rw /dev/gpiomem
sudo chown root.gpio /dev/gpiomem
