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

After a reboot, the new driver will be used.

#### Switching back to the old driver
To switch back to the old driver:
- remove the 5 last lines of `/boot/config.txt`
- remove the `uinput` line from `/etc/modules`
- run `sudo systemctl disable raspberrypi-fast-ts.service`
