sudo cp 20-termite-devices.rules /etc/udev/rules.d/

sudo udevadm control --reload-rules
sudo udevadm trigger
