#define _GNU_SOURCE
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/fcntl.h>
#include <time.h>
#include <errno.h>
#include <memory.h>
#include <linux/uinput.h>
#include <i2c/smbus.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

struct ft5406_regs {
    uint8_t device_mode;
    uint8_t gesture_id;
    uint8_t num_points;
    struct ft5406_touch {
        uint8_t xh;
        uint8_t xl;
        uint8_t yh;
        uint8_t yl;
        uint8_t pressure;
        uint8_t area;
    } point[10];
};

struct ts {
    int max_x;
    int max_y;
    bool hflip;
    bool vflip;
    bool xyswap;

    int uinput_fd;

    int i2c_fd;
    uint8_t slave_addr;
    struct ft5406_regs regs;
    int32_t tracking_ids[10];
    int32_t next_tracking_id;
    int known_ids;
};

static int read_bytes(int fd, uint8_t slave_addr, uint8_t reg, uint8_t *dest, int num_bytes) {
    struct i2c_msg msgs[2] = {
        {
            .addr = slave_addr,
            .flags = 0,
            .buf = &reg,
            .len = 1
        },
        {
            .addr = slave_addr,
            .flags = I2C_M_RD,
            .buf = dest,
            .len = num_bytes
        }
    };


    struct i2c_rdwr_ioctl_data data = {
        .msgs = msgs,
        .nmsgs = 2
    };

    return ioctl(fd, I2C_RDWR, &data);
}

#define read_into_regs(fd, slave_addr, field, p_regs, num_bytes) \
    read_bytes(fd, slave_addr, offsetof(struct ft5406_regs, field), ((uint8_t*) p_regs) + offsetof(struct ft5406_regs, field), num_bytes)

static int poll(struct ts *ts) {
    struct input_event events[64];
    struct timeval time;
    int n_events = 0;
    int modified_ids;
    int released_ids;
    int ok;

    ok = read_into_regs(ts->i2c_fd, ts->slave_addr, num_points, &ts->regs, 1);
    if (ok < 0) {
        ok = errno;
        perror("Could not execute I2C transfer. ioctl");
        return ok;
    }
    
    {   
        struct timespec timespec;
        clock_gettime(CLOCK_MONOTONIC, &timespec);
        time.tv_sec = timespec.tv_sec;
        time.tv_usec = timespec.tv_nsec / 1000;
    }

    modified_ids = 0;

    for (int i = 0; i < ts->regs.num_points; i++) {
        ok = read_into_regs(ts->i2c_fd, ts->slave_addr, point[i].xh, &ts->regs, 4);
        if (ok < 0) {
            ok = errno;
            perror("Could not execute I2C transfer. ioctl");
            return ok;
        }

        int x = (((int) ts->regs.point[i].xh & 0xf) << 8) + ts->regs.point[i].xl;
        int y = (((int) ts->regs.point[i].yh & 0xf) << 8) + ts->regs.point[i].yl;
        int touchid = (ts->regs.point[i].yh >> 4) & 0xF;
        int event_type = (ts->regs.point[i].xh >> 6) & 0x03;
        
        modified_ids |= 1 << touchid;

        if (event_type == 0 || event_type == 2) {
            if (ts->hflip) {
                x = ts->max_x - 1 - x;
            }

            if (ts->vflip) {
                y = ts->max_y - 1 - y;
            }

            if (ts->xyswap) {
                int z = x;
                x = y;
                y = z;
            }

            if (ts->tracking_ids[touchid] == -1) {
                ts->tracking_ids[touchid] = ts->next_tracking_id++;
            }

            events[n_events++] = (struct input_event) {
                .time = time,
                .type = EV_ABS,
                .code = ABS_MT_SLOT,
                .value = touchid
            };

            events[n_events++] = (struct input_event) {
                .time = time,
                .type = EV_ABS,
                .code = ABS_MT_TRACKING_ID,
                .value = ts->tracking_ids[touchid]
            };

            events[n_events++] = (struct input_event) {
                .time = time,
                .type = EV_ABS,
                .code = ABS_MT_TOOL_TYPE,
                .value = MT_TOOL_FINGER
            };

            events[n_events++] = (struct input_event) {
                .time = time,
                .type = EV_ABS,
                .code = ABS_MT_POSITION_X,
                .value = x
            };

            events[n_events++] = (struct input_event) {
                .time = time,
                .type = EV_ABS,
                .code = ABS_MT_POSITION_Y,
                .value = y
            };
        }
    }

    released_ids = ts->known_ids & ~modified_ids;
    for (int i = 0; released_ids && i < 10; i++) {
        if (released_ids & (1 << i)) {
            ts->tracking_ids[i] = -1;
            
            events[n_events++] = (struct input_event) {
                .time = time,
                .type = EV_ABS,
                .code = ABS_MT_SLOT,
                .value = i
            };

            events[n_events++] = (struct input_event) {
                .time = time,
                .type = EV_ABS,
                .code = ABS_MT_TRACKING_ID,
                .value = ts->tracking_ids[i]
            };

            modified_ids &= ~(1 << i);
            released_ids &= ~(1 << i);
        }
    }

    ts->known_ids = modified_ids;

    events[n_events++] = (struct input_event) {
        .time = time,
        .type = EV_SYN,
        .code = SYN_REPORT,
        .value = 0
    };

    if (n_events) {
        ok = write(ts->uinput_fd, events, n_events * sizeof(struct input_event));
        if (ok < 0) {
            ok = errno;
            perror("Could not add input events to uinput device. write");
            return ok;
        }
    }

    return 0;
}


static int setup_abs(int fd, uint16_t code, int32_t value, int32_t minimum, int32_t maximum, int32_t fuzz, int32_t flat, int32_t resolution) {
    struct uinput_abs_setup setup;

    memset(&setup, 0, sizeof(setup));

    setup.code = code;
    setup.absinfo.value = value;
    setup.absinfo.minimum = minimum;
    setup.absinfo.maximum = maximum;
    setup.absinfo.fuzz = fuzz;
    setup.absinfo.flat = flat;
    setup.absinfo.resolution = resolution;

    return ioctl(fd, UI_ABS_SETUP, &setup);
}

static int setup_dev(int fd, uint16_t bustype, uint16_t vendor, uint16_t product, uint16_t version, const char name[80]) {
    struct uinput_setup setup;
    
    memset(&setup, 0, sizeof(setup));

    setup.id.bustype = bustype;
    setup.id.vendor = vendor;
    setup.id.product = product;
    setup.id.version = version;
    strncpy(setup.name, name, 80);

    return ioctl(fd, UI_DEV_SETUP, &setup);
}

int main(int argc, char **argv) {
    struct timespec time;
    struct ts *ts;
    char *uinput_path, *i2c_path;
    int uinput_fd, i2c_fd;
    int ok;

    uinput_path = "/dev/uinput";
    ok = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (ok < 0) {
        perror("Could not open uinput device. Make sure the uinput module is loaded using \"sudo modprobe uinput\". open");
        goto fail;
    }

    uinput_fd = ok;

    i2c_path = "/dev/i2c-11";
    ok = open(i2c_path, O_RDWR);
    if (ok < 0) {
        perror("Could not open the I2C bus. open");
        goto fail;
    }

    i2c_fd = ok;

    ts = malloc(sizeof(struct ts));
    if (ts == NULL) {
        fprintf(stderr, "Out of memory\n");
        goto fail;
    }

    ts->max_x = 800;
    ts->max_y = 480;
    ts->vflip = true;
    ts->hflip = true;
    ts->xyswap = false;
    ts->uinput_fd = uinput_fd;
    ts->i2c_fd = i2c_fd;
    ts->slave_addr = 0x38;
    memset(&ts->regs, 0, sizeof(ts->regs));
    for (int i = 0; i < 10; i++) ts->tracking_ids[i] = -1;
    ts->next_tracking_id = 0;
    ts->known_ids = 0;
    
    ok = ioctl(uinput_fd, UI_SET_EVBIT, EV_SYN);
    if (ok < 0) {
        perror("Could not EV_SYN evbit on uinput device. ioctl");
        goto fail;
    }

    ok = ioctl(uinput_fd, UI_SET_EVBIT, EV_KEY);
    if (ok < 0) {
        perror("Could not EV_KEY evbit on uinput device. ioctl");
        goto fail;
    }

    ok = ioctl(uinput_fd, UI_SET_KEYBIT, BTN_TOUCH);
    if (ok < 0) {
        perror("Could not BTN_TOUCH keybit on uinput device. ioctl");
        goto fail;
    }

    ok = ioctl(uinput_fd, UI_SET_EVBIT, EV_ABS);
    if (ok < 0) {
        perror("Could not EV_ABS evbit on uinput device. ioctl");
        goto fail;
    }

    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_X);
    if (ok < 0) {
        perror("Could not ABS_X absbit on uinput device. ioctl");
        goto fail;
    }

    ok = setup_abs(uinput_fd, ABS_X, 0, 0, ts->max_x-1, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_X absdata on uinput device. ioctl");
        goto fail;
    }

    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_Y);
    if (ok < 0) {
        perror("Could not ABS_Y absbit on uinput device. ioctl");
        goto fail;
    }

    ok = setup_abs(uinput_fd, ABS_Y, 0, 0, ts->max_y - 1, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_X absdata on uinput device. ioctl");
        goto fail;
    }

    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_MT_SLOT);
    if (ok < 0) {
        perror("Could not ABS_MT_SLOT absbit on uinput device. ioctl");
        goto fail;
    }
    
    ok = setup_abs(uinput_fd, ABS_MT_SLOT, 0, 0, 9, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_MT_SLOT absdata on uinput device. ioctl");
        goto fail;
    }
    
    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_MT_POSITION_X);
    if (ok < 0) {
        perror("Could not ABS_MT_POSITION_X absbit on uinput device. ioctl");
        goto fail;
    }

    ok = setup_abs(uinput_fd, ABS_MT_POSITION_X, 0, 0, ts->max_x - 1, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_MT_POSITION_X absdata on uinput device. ioctl");
        goto fail;
    }
    
    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_MT_POSITION_Y);
    if (ok < 0) {
        perror("Could not ABS_MT_POSITION_Y absbit on uinput device. ioctl");
        goto fail;
    }

    ok = setup_abs(uinput_fd, ABS_MT_POSITION_Y, 0, 0, ts->max_y - 1, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_MT_POSITION_Y absdata on uinput device. ioctl");
        goto fail;
    }

    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_MT_TRACKING_ID);
    if (ok < 0) {
        perror("Could not ABS_MT_TRACKING_ID absbit on uinput device. ioctl");
        goto fail;
    }

    ok = setup_abs(uinput_fd, ABS_MT_TRACKING_ID, 0, 0, 65535, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_MT_TRACKING_ID absdata on uinput device. ioctl");
        goto fail;
    }

    ok = ioctl(uinput_fd, UI_SET_PROPBIT, INPUT_PROP_DIRECT);
    if (ok < 0) {
        perror("Could not INPUT_PROP_DIRECT propbit on uinput device. ioctl");
        goto fail;
    }

    ok = setup_dev(uinput_fd, BUS_HOST, 0x1234, 0x5678, 0x0000, "raspberrypi-fast-ts");
    if (ok < 0) {
        perror("Could not setup uinput device. ioctl");
        goto fail;
    }

    ok = ioctl(uinput_fd, UI_DEV_CREATE);
    if (ok < 0) {
        perror("Could not create uinput device. ioctl");
        goto fail;
    }

    ok = ioctl(i2c_fd, I2C_SLAVE, ts->slave_addr);
    if (ok < 0) {
        perror("Could not set I2C slave address. ioctl");
        goto fail;
    }
    
    while (1) {
        time = (struct timespec) {.tv_sec = 0, .tv_nsec = 5000000};
        while (1) {
            ok = nanosleep(&time, &time);
            if ((ok < 0) && errno != EINTR) {
                perror("Error while waiting for next poll. nanosleep");
                goto fail;
            } else if (ok == 0) {
                break;
            }
        }

        ok = poll(ts);
        if (ok != 0 && ok != EREMOTEIO) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;

    fail:
    return EXIT_FAILURE;
}