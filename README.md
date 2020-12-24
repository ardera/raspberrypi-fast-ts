# raspberrypi-fast-ts
A faster, lower-latency, userspace touchscreen input driver for the official Raspberry Pi 7" screen.

#### Compatibility
Only tested on Pi 4. It's possible it works on Pi 3 too. Will not work on Pi 2 or 1.

#### Installation Instructions
```bash
$ git clone https://github.com/ardera/raspberrypi-fast-ts.git
$ cd raspberrypi-fast-ts
$ make
$ sudo make install
```

After that, the driver is installed, but not yet enabled. To enable it and disable the old driver, see the following steps. *Be aware this makes changes to `/boot/config.txt`.* The installation script is rather dumb. If your `config.txt` is heavily modified, you can apply the changes manually, just look at how the Makefile does it.
```bash
$ sudo make enable
```

If you specified `lcd_rotate=2` inside `/boot/config.txt` to rotate your display by 180 degrees, you also need to invoke `raspberrypi-fast-ts` with the `-f` option.
1. Open `/etc/systemd/system/raspberrypi-fast-ts.service` using your favorite text editor
2. find the line where it says `ExecStart=/usr/local/bin/raspberrypi-fast-ts`
3. change it to `ExecStart=/usr/local/bin/raspberrypi-fast-ts -f`

After a reboot, the new driver will be used.

#### Switching back to the old driver
To switch back to the old driver:
- remove the 5 last lines of `/boot/config.txt`
- remove the `uinput` line from `/etc/modules`
- run `sudo systemctl disable raspberrypi-fast-ts.service`

#### Backlight interface
This driver now has a backlight interface similiar to the standard sysfs one.
It's located inside `/var/tmp/raspberrypi-fast-ts`. Currently, `bl_power` and `brightness` are supported.

You can use it just like the sysfs interface, for example to change the brightness to 50%:
```
$ sudo bash
# echo 127 > /var/tmp/raspberrypi-fast-ts/brightness
# exit
```

to change it to 100% again:
```
$ sudo bash
# echo 255 > /var/tmp/raspberrypi-fast-ts/brightness
# exit 
```
