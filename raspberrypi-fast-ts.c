#define _GNU_SOURCE
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <memory.h>
#include <ctype.h>

#include <sys/timerfd.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/stat.h>
#include <i2c/smbus.h>
#include <linux/uinput.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

const char *const usage = "\
USAGE:\n\
  raspberry-pi-fast-ts [options]\n\
\n\
OPTIONS:\n\
  -f    Flips the touchscreen coordinates, as if the screen\n\
        were rotated by 180 degrees.\n\
        Necessary when 'lcd_rotate=2' in /boot/config.txt\n\
        is also specified.\n\
\n\
EXAMPLES:\n\
  sudo raspberrypi-fast-ts\n\
  sudo raspberrypi-fast-ts -f\n\
\n\
SEE ALSO:\n\
  Author:  Hannes Winkler, a.k.a ardera\n\
  Source:  https://github.com/ardera/raspberrypi-fast-ts\n\
  License: MIT\n\
";

#define FIFO_DIR "/var/tmp/raspberrypi-fast-ts"
#define BRIGHTNESS_FIFO_PATH FIFO_DIR "/brightness"
#define BL_POWER_FIFO_PATH FIFO_DIR "/bl_power"

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

struct fifo_data {
    char *path;
    int fd;
    char buffer[16];
    size_t offset;
    bool discard;
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

static int write_byte(int fd, uint8_t slave_addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {
        reg,
        value
    };

    struct i2c_msg msg = {
        .addr = slave_addr,
        .flags = 0,
        .buf = buf,
        .len = 2
    };


    struct i2c_rdwr_ioctl_data data = {
        .msgs = &msg,
        .nmsgs = 1
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

    if (ts->regs.num_points == 0xFF) {
        ts->regs.num_points = 0;
    }

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

    if (n_events) {
        events[n_events++] = (struct input_event) {
            .time = time,
            .type = EV_SYN,
            .code = SYN_REPORT,
            .value = 0
        };

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

static int fifo_read_ul(struct fifo_data *data, bool *has_value_out, unsigned long *value_out) {
    unsigned long value;
    static char discard_buffer[256];
    size_t to_read;
    char *cursor; 
    int ok;

    *has_value_out = false;
    
    cursor = data->buffer + data->offset;
    to_read = sizeof(data->buffer) - data->offset - 1;

    if (to_read == 0) {
        ok = EOVERFLOW;
        fprintf(stderr, "FIFO overflow. All data will be discarded until the next EOF.\n");
        data->discard = true;
        data->offset = 0;
        memset(data->buffer, 0, sizeof(data->buffer));
    }

    if (data->discard) {
        ok = read(data->fd, discard_buffer, sizeof(discard_buffer));
        if (ok < 0) {
            ok = errno;
            perror("Couldn't read FIFO. read");
            goto fail_return_ok;
        } else if (ok == 0) {
            // we need to reopen hear to clear the EOF.
            ok = open(data->path, O_RDONLY | O_NONBLOCK);
            if (ok < 0) {
                ok = errno;
                perror("Couldn't reopen FIFO. open");
                goto fail_return_ok;
            }

            close(data->fd);
            data->fd = ok;

            data->discard = false;
        }
    } else {
        ok = read(data->fd, cursor, to_read);
        if (ok < 0) {
            ok = errno;
            perror("Couldn't read FIFO. read");
            goto fail_discard_and_reset_buffer;
        } else if (ok == 0) {
            // received EOF.
            // we need to reopen the FIFO since otherwise the select in main
            // will continously return because of the EOF on the FIFO fd.
            // (We can't clear the EOF, so we need to reopen)
            ok = open(data->path, O_RDONLY | O_NONBLOCK);
            if (ok < 0) {
                ok = errno;
                perror("Couldn't reopen FIFO. open");
                goto fail_discard_and_reset_buffer;
            }

            close(data->fd);
            data->fd = ok;
            
            // parse the value
            errno = 0;
            char *end;
            value = strtoul(data->buffer, &end, 0);
            if (errno != 0) {
                ok = errno;
                perror("Couldn't convert received FIFO value to a number. strtoul");
                goto fail_reset_buffer;
            } else if ((end == '\0') || (end == data->buffer)) {
                ok = EINVAL;
                fprintf(stderr, "Invalid FIFO value received\n");
                goto fail_reset_buffer;
            }

            // reset the buffer
            data->offset = 0;
            memset(data->buffer, 0, sizeof(data->buffer));

            *has_value_out = true;
            *value_out = value;
        } else if (ok > 0) {
            data->offset += ok;
        }
    }

    return 0;


    fail_discard_and_reset_buffer:
    data->discard = true;

    fail_reset_buffer:
    data->offset = 0;
    memset(data->buffer, 0, sizeof(data->buffer));

    fail_return_ok:
    return ok;
}

void print_usage(void) {
    printf("%s", usage);
}

int parse_cmdline_args(int argc, char **argv, bool *vflip_out, bool *hflip_out, bool *print_usage_only_out) {
    unsigned long rotation;
    int c;

    *vflip_out = true;
    *hflip_out = true;

    while ((c = getopt(argc, argv, "fh")) != -1) {
        switch (c) {
            case 'f':;
                *vflip_out = false;
                *hflip_out = false;
                break;
            case 'h':
                print_usage();
                *print_usage_only_out = true;
                return 0;
            case '?':
                if (optopt == 'r') {
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                } else if (isprint(optopt)) {
                    fprintf(stderr, "Unknown option `-%c'.\n", optopt);
                } else {
                    fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
                }
                return EINVAL;
            default:
                abort();
        }
    }

    if (optind < argc) {
        fprintf(stderr, "Commandline arguments that aren't options were specified.\n");
        return EINVAL;
    }

    return 0;
}

int main(int argc, char **argv) {
    struct fifo_data brightness_fifo, bl_power_fifo;
    struct timespec time;
    struct ts *ts;
    char *uinput_path, *i2c_path;
    bool fifo_dir_existed_already, print_usage_only, vflip, hflip;
    uint8_t brightness;
    int uinput_fd, i2c_fd, timer_fd;
    int ok;

    brightness = 255;

    ok = parse_cmdline_args(argc, argv, &vflip, &hflip, &print_usage_only);
    if (ok != 0) {
        return EXIT_FAILURE;
    } else if (print_usage_only) {
        return EXIT_SUCCESS;
    }

    ok = mkdir(FIFO_DIR, 0777);
    if (ok < 0 && errno == EEXIST) {
        fifo_dir_existed_already = true;
    } else if (ok < 0) {
        perror("Could not create \"" FIFO_DIR "\" directory for backlight control. mkdir");
        goto fail;
    } else {
        fifo_dir_existed_already = false;
    }

    ok = mkfifo(BRIGHTNESS_FIFO_PATH, 0777);
    if ((ok < 0) && (errno != EEXIST)) {
        perror("Could not create \"" BRIGHTNESS_FIFO_PATH "\" FIFO for backlight control. mkfifo");
        goto fail_remove_bl_dir;
    }
    
    ok = mkfifo(BL_POWER_FIFO_PATH, 0777);
    if ((ok < 0) && (errno != EEXIST)) {
        perror("Could not create \"" BL_POWER_FIFO_PATH "\" FIFO for backlight control. mkfifo");
        goto fail_remove_brightness_fifo;
    }

    ok = open(BRIGHTNESS_FIFO_PATH, O_RDONLY | O_NONBLOCK);
    if (ok < 0) {
        perror("Could not open \"" BRIGHTNESS_FIFO_PATH "\" FIFO. open");
        goto fail_remove_bl_power_fifo;
    }

    memset(&brightness_fifo, 0, sizeof(brightness_fifo));
    brightness_fifo.fd = ok;
    brightness_fifo.path = BRIGHTNESS_FIFO_PATH;
    
    ok = open(BL_POWER_FIFO_PATH, O_RDONLY | O_NONBLOCK);
    if (ok < 0) {
        perror("Could not open \"" BL_POWER_FIFO_PATH "\" FIFO. open");
        goto fail_close_brightness_fifo;
    }
    
    memset(&bl_power_fifo, 0, sizeof(bl_power_fifo));
    bl_power_fifo.fd = ok;
    bl_power_fifo.path = BL_POWER_FIFO_PATH;

    uinput_path = "/dev/uinput";
    ok = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (ok < 0) {
        perror("Could not open uinput device. Make sure the uinput module is loaded using \"sudo modprobe uinput\" and that you're executing `raspberrypi-fast-ts` as root, using `sudo`. open");
        goto fail_close_bl_power_fifo;
    }

    uinput_fd = ok;

    i2c_path = "/dev/i2c-11";
    ok = open(i2c_path, O_RDWR);
    if (ok < 0) {
        perror("Could not open the I2C bus. open");
        goto fail_close_uinput;
    }

    i2c_fd = ok;

    ts = malloc(sizeof(struct ts));
    if (ts == NULL) {
        fprintf(stderr, "Out of memory\n");
        goto fail_close_i2c;
    }

    ts->max_x = 800;
    ts->max_y = 480;
    ts->vflip = vflip;
    ts->hflip = hflip;
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
        goto fail_remove_bl_power_fifo;
    }

    ok = ioctl(uinput_fd, UI_SET_EVBIT, EV_KEY);
    if (ok < 0) {
        perror("Could not EV_KEY evbit on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = ioctl(uinput_fd, UI_SET_KEYBIT, BTN_TOUCH);
    if (ok < 0) {
        perror("Could not BTN_TOUCH keybit on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = ioctl(uinput_fd, UI_SET_EVBIT, EV_ABS);
    if (ok < 0) {
        perror("Could not EV_ABS evbit on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_X);
    if (ok < 0) {
        perror("Could not ABS_X absbit on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = setup_abs(uinput_fd, ABS_X, 0, 0, ts->max_x-1, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_X absdata on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_Y);
    if (ok < 0) {
        perror("Could not ABS_Y absbit on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = setup_abs(uinput_fd, ABS_Y, 0, 0, ts->max_y - 1, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_X absdata on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_MT_SLOT);
    if (ok < 0) {
        perror("Could not ABS_MT_SLOT absbit on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }
    
    ok = setup_abs(uinput_fd, ABS_MT_SLOT, 0, 0, 9, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_MT_SLOT absdata on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }
    
    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_MT_POSITION_X);
    if (ok < 0) {
        perror("Could not ABS_MT_POSITION_X absbit on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = setup_abs(uinput_fd, ABS_MT_POSITION_X, 0, 0, ts->max_x - 1, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_MT_POSITION_X absdata on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }
    
    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_MT_POSITION_Y);
    if (ok < 0) {
        perror("Could not ABS_MT_POSITION_Y absbit on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = setup_abs(uinput_fd, ABS_MT_POSITION_Y, 0, 0, ts->max_y - 1, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_MT_POSITION_Y absdata on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = ioctl(uinput_fd, UI_SET_ABSBIT, ABS_MT_TRACKING_ID);
    if (ok < 0) {
        perror("Could not ABS_MT_TRACKING_ID absbit on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = setup_abs(uinput_fd, ABS_MT_TRACKING_ID, 0, 0, 65535, 0, 0, 0);
    if (ok < 0) {
        perror("Could setup ABS_MT_TRACKING_ID absdata on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = ioctl(uinput_fd, UI_SET_PROPBIT, INPUT_PROP_DIRECT);
    if (ok < 0) {
        perror("Could not INPUT_PROP_DIRECT propbit on uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = setup_dev(uinput_fd, BUS_HOST, 0x1234, 0x5678, 0x0000, "raspberrypi-fast-ts");
    if (ok < 0) {
        perror("Could not setup uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = ioctl(uinput_fd, UI_DEV_CREATE);
    if (ok < 0) {
        perror("Could not create uinput device. ioctl");
        goto fail_remove_bl_power_fifo;
    }

    ok = ioctl(i2c_fd, I2C_SLAVE, ts->slave_addr);
    if (ok < 0) {
        perror("Could not set I2C slave address. ioctl");
        goto fail_remove_bl_power_fifo;
    }
    
    timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (timer_fd < 0) {
        perror("Couldn't create timer. timerfd_create");
        goto fail_remove_bl_power_fifo;
    }

    ok = timerfd_settime(
        timer_fd,
        0,
        &(struct itimerspec) {
            .it_interval = {.tv_sec = 0, .tv_nsec = 1000000000ull / (60*3)},
            .it_value = {.tv_sec = 0, .tv_nsec = 1000000000ull / (60*3)}
        },
        NULL
    );
    if (ok < 0) {
        perror("Couldn't arm timer. timerfd_settime");
        goto fail_close_timer;
    }

    while (1) {
        fd_set ready_rfds;

        FD_ZERO(&ready_rfds);

        FD_SET(timer_fd, &ready_rfds);
        FD_SET(brightness_fifo.fd, &ready_rfds);
        FD_SET(bl_power_fifo.fd, &ready_rfds);

        int n_fds = bl_power_fifo.fd;
        if (brightness_fifo.fd > n_fds) n_fds = brightness_fifo.fd;
        if (timer_fd > n_fds) n_fds = timer_fd;

        n_fds++;

        ok = select(n_fds, &ready_rfds, NULL, NULL, NULL);
        if (ok < 0) {
            perror("Couldn't wait for brightness / bl_power events or timer expiration. select");
            goto fail_close_timer;
        }

        if (FD_ISSET(brightness_fifo.fd, &ready_rfds)) {
            bool has_value;
            unsigned long brightness_value;

            ok = fifo_read_ul(
                &brightness_fifo,
                &has_value,
                &brightness_value
            );
            if ((ok != 0) && (ok != EINVAL) && (ok != ERANGE)) {
                goto fail_close_timer;
            }

            if (has_value && brightness_value >= 0 && brightness_value <= 255) {
                ok = write_byte(i2c_fd, 0x45, 0x86, (uint8_t) brightness_value);
                if ((ok < 0) && (errno != EREMOTEIO) && (errno != ETIMEDOUT)) {
                    goto fail_close_timer;
                }
                brightness = brightness_value;
            }
        }
        
        if (FD_ISSET(bl_power_fifo.fd, &ready_rfds)) {
            bool has_value;
            unsigned long bl_power_value;

            ok = fifo_read_ul(
                &bl_power_fifo,
                &has_value,
                &bl_power_value
            );
            if ((ok != 0) && (ok != EINVAL) && (ok != ERANGE)) {
                goto fail_close_timer;
            }

            if (has_value) {
                ok = write_byte(i2c_fd, 0x45, 0x86, bl_power_value? 0 : brightness);
                if ((ok < 0) && (errno != EREMOTEIO) && (errno != ETIMEDOUT)) {
                    goto fail_close_timer;
                }
            }
        }

        if (FD_ISSET(timer_fd, &ready_rfds)) {
            static char timer_buf[8];
            ok = read(timer_fd, timer_buf, 8);
            if (ok < 0) {
                perror("Couldn't rearm timerfd. read");
                goto fail_remove_bl_power_fifo;
            }

            ok = poll(ts);
            if ((ok != 0) && (ok != EREMOTEIO) && (ok != ETIMEDOUT)) {
                // ignore remote I/O errors and timeouts since they ocurr spuriously all the time
                goto fail_close_timer;
            }
        }
    }

    close(timer_fd);
    ioctl(uinput_fd, UI_DEV_DESTROY);
    free(ts);
    close(i2c_fd);
    close(uinput_fd);
    close(bl_power_fifo.fd);
    close(brightness_fifo.fd);
    remove(BL_POWER_FIFO_PATH);
    remove(BRIGHTNESS_FIFO_PATH);
    if (fifo_dir_existed_already == false) {
        rmdir(FIFO_DIR);
    }
    return EXIT_SUCCESS;


    fail_close_timer:
    close(timer_fd);

    fail_destroy_uinput_device:
    ioctl(uinput_fd, UI_DEV_DESTROY);

    fail_free_ts:
    free(ts);

    fail_close_i2c:
    close(i2c_fd);

    fail_close_uinput:
    close(uinput_fd);

    fail_close_bl_power_fifo:
    close(bl_power_fifo.fd);

    fail_close_brightness_fifo:
    close(brightness_fifo.fd);

    fail_remove_bl_power_fifo:
    remove(BL_POWER_FIFO_PATH);

    fail_remove_brightness_fifo:
    remove(BRIGHTNESS_FIFO_PATH);

    fail_remove_bl_dir:
    if (fifo_dir_existed_already == false) {
        rmdir(FIFO_DIR);
    }

    fail:
    return EXIT_FAILURE;
}
