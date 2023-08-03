echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="yacyac/lidar"' >/etc/udev/rules.d/yacyac_lidar.rules


echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0666", GROUP:="dialout",  SYMLINK+="yacyac/motor"' >/etc/udev/rules.d/yacyac_motor.rules

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT4TCVL4", GROUP:="dialout",  SYMLINK+="yacyac/servo"' >/etc/udev/rules.d/yacyac_servo.rules

service udev reload
sleep 2
service udev restart