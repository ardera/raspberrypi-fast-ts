all:
	cc ./raspberrypi-fast-ts.c -O2 -o ./raspberrypi-fast-ts

clean:
	rm ./raspberrypi-fast-ts

install: raspberrypi-fast-ts
	install ./raspberrypi-fast-ts /usr/local/bin
	install ./raspberrypi-fast-ts.service /etc/systemd/system/
	systemctl daemon-reload

enable:
	echo 'uinput' >> /etc/modules
	echo '[all]' >> /boot/config.txt
	echo 'disable_touchscreen=1' >> /boot/config.txt
	echo 'dtparam=i2c_vc=on' >> /boot/config.txt
	echo 'dtparam=i2c_vc_baudrate=400000' >> /boot/config.txt
	systemctl enable raspberrypi-fast-ts.service
