# raspberrypi-fast-ts
A faster, lower-latency, userspace touchscreen input driver for the official Raspberry Pi 7" screen.

#### Compatibility
Only tested on Pi 4. It's possible it works on Pi 3 too. Will not work on Pi 2 or 1.

#### Building
```bash
$ git clone https://github.com/ardera/raspberrypi-fast-ts.git
$ cd raspberrypi-fast-ts
$ gcc -O2 ./raspberrypi-fast-ts.c -o ./raspberrypi-fast-ts
```

#### Configuring your Pi
1. `$ sudo nano /boot/config.txt`
2. add the following lines:
```
disable_touchscreen=1
dtparam=i2c_vc=on
dtparam=i2c_vc_baudrate=400000
```
3. save & reboot

#### Running
```
$ sudo modprobe uinput
$ sudo ./raspberrypi-fast-ts
```
