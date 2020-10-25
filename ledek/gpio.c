#include <stdio.h>
#include "gpio.h"
#include "hardware.h"

const uint8_t rev1_p1pin2gpio_map[] = {
        DMY,	// P1-1   3v3
        DMY,	// P1-2   5v
        0,	    // P1-3   GPIO 0 (SDA)
        DMY,	// P1-4   5v
        1,	    // P1-5   GPIO 1 (SCL)
        DMY,	// P1-6   Ground
        4,	    // P1-7   GPIO 4 (GPCLK0)
        14,	    // P1-8   GPIO 14 (TXD)
        DMY,	// P1-9   Ground
        15,	    // P1-10  GPIO 15 (RXD)
        17,	    // P1-11  GPIO 17
        18,	    // P1-12  GPIO 18 (PCM_CLK)
        21,	    // P1-13  GPIO 21
        DMY,	// P1-14  Ground
        22,	    // P1-15  GPIO 22
        23,	    // P1-16  GPIO 23
        DMY,	// P1-17  3v3
        24,	    // P1-18  GPIO 24
        10,	    // P1-19  GPIO 10 (MOSI)
        DMY,	// P1-20  Ground
        9,	    // P1-21  GPIO 9 (MISO)
        25,	    // P1-22  GPIO 25
        11,	    // P1-23  GPIO 11 (SCLK)
        8,	    // P1-24  GPIO 8 (CE0)
        DMY,	// P1-25  Ground
        7,	    // P1-26  GPIO 7 (CE1)
};

static uint8_t rev1_p5pin2gpio_map[] = {
        DMY,	// (P5-1 on rev 2 boards)
        DMY,	// (P5-2 on rev 2 boards)
        DMY,	// (P5-3 on rev 2 boards)
        DMY,	// (P5-4 on rev 2 boards)
        DMY,	// (P5-5 on rev 2 boards)
        DMY,	// (P5-6 on rev 2 boards)
        DMY,	// (P5-7 on rev 2 boards)
        DMY,	// (P5-8 on rev 2 boards)
};

static uint8_t rev2_p1pin2gpio_map[] = {
        DMY,	// P1-1   3v3
        DMY,	// P1-2   5v
        2,	    // P1-3   GPIO 2 (SDA)
        DMY,	// P1-4   5v
        3,	    // P1-5   GPIO 3 (SCL)
        DMY,	// P1-6   Ground
        4,	    // P1-7   GPIO 4 (GPCLK0)
        14,	    // P1-8   GPIO 14 (TXD)
        DMY,	// P1-9   Ground
        15,	    // P1-10  GPIO 15 (RXD)
        17,	    // P1-11  GPIO 17
        18,	    // P1-12  GPIO 18 (PCM_CLK)
        27, 	// P1-13  GPIO 27
        DMY,	// P1-14  Ground
        22,	    // P1-15  GPIO 22
        23,	    // P1-16  GPIO 23
        DMY,	// P1-17  3v3
        24,	    // P1-18  GPIO 24
        10,	    // P1-19  GPIO 10 (MOSI)
        DMY,	// P1-20  Ground
        9,	    // P1-21  GPIO 9 (MISO)
        25,	    // P1-22  GPIO 25
        11,	    // P1-23  GPIO 11 (SCLK)
        8,	    // P1-24  GPIO 8 (CE0)
        DMY,	// P1-25  Ground
        7,	    // P1-26  GPIO 7 (CE1)
};

static uint8_t rev2_p5pin2gpio_map[] = {
        DMY,	// P5-1   5v0
        DMY,	// P5-2   3v3
        28,	    // P5-3   GPIO 28 (I2C0_SDA)
        29,	    // P5-4   GPIO 29 (I2C0_SCL)
        30,	    // P5-5   GPIO 30
        31,	    // P5-6   GPIO 31
        DMY,	// P5-7   Ground
        DMY,	// P5-8   Ground
};

static uint8_t bplus_p1pin2gpio_map[] = {
        DMY,	// P1-1   3v3
        DMY,	// P1-2   5v
        2,	    // P1-3   GPIO 2 (SDA)
        DMY,	// P1-4   5v
        3,	    // P1-5   GPIO 3 (SCL)
        DMY,	// P1-6   Ground
        4,	    // P1-7   GPIO 4 (GPCLK0)
        14,	    // P1-8   GPIO 14 (TXD)
        DMY,	// P1-9   Ground
        15,	    // P1-10  GPIO 15 (RXD)
        17,	    // P1-11  GPIO 17
        18,	    // P1-12  GPIO 18 (PCM_CLK)
        27,	    // P1-13  GPIO 27
        DMY,	// P1-14  Ground
        22,	    // P1-15  GPIO 22
        23,	    // P1-16  GPIO 23
        DMY,	// P1-17  3v3
        24,	    // P1-18  GPIO 24
        10,	    // P1-19  GPIO 10 (MOSI)
        DMY,	// P1-20  Ground
        9,	    // P1-21  GPIO 9 (MISO)
        25,	    // P1-22  GPIO 25
        11,	    // P1-23  GPIO 11 (SCLK)
        8,	    // P1-24  GPIO 8 (CE0)
        DMY,	// P1-25  Ground
        7,	    // P1-26  GPIO 7 (CE1)
        DMY,	// P1-27  ID_SD
        DMY,	// P1-28  ID_SC
        5,	    // P1-29  GPIO 5
        DMY,	// P1-30  Ground
        6,	    // P1-31  GPIO 5
        12,	    // P1-32  GPIO 12
        13,	    // P1-33  GPIO 13
        DMY,	// P1-34  Ground
        19,	    // P1-35  GPIO 19
        16,	    // P1-36  GPIO 16
        26, 	// P1-37  GPIO 26
        20, 	// P1-38  GPIO 20
        DMY,	// P1-39  Ground
        21, 	// P1-40  GPIO 21
};

char *gpio_desc[] = {
        "Unknown",
        "P1 (26 pins)",
        "P1 (26 pins), P5 (8 pins)",
        "P1 (40 pins)"
};

uint32_t gpio_get_mode(uint32_t gpio) {
    uint32_t fsel = gpio_reg[GPIO_FSEL0 + gpio/10];

    return (fsel >> ((gpio % 10) * 3)) & 7;
}

void gpio_set_mode(uint32_t gpio, uint32_t mode) {
    uint32_t fsel = gpio_reg[GPIO_FSEL0 + gpio/10];

    fsel &= ~(7 << ((gpio % 10) * 3));
    fsel |= mode << ((gpio % 10) * 3);
    gpio_reg[GPIO_FSEL0 + gpio/10] = fsel;
}

void gpio_set(int gpio, int level) {
    if (level)
        gpio_reg[GPIO_SET0] = 1 << gpio;
    else
        gpio_reg[GPIO_CLR0] = 1 << gpio;
}

void parse_pin_lists(int p1first, char *p1pins, char*p5pins) {
    char *name, *pins;
    int i, mapcnt;
    uint8_t *map, *pNpin2servo;
    int lst, servo = 0;
    FILE *fp;

    memset(servo2gpio, DMY, sizeof(servo2gpio));
    memset(p1pin2servo, DMY, sizeof(p1pin2servo));
    memset(p5pin2servo, DMY, sizeof(p5pin2servo));
    for (lst = 0; lst < 2; lst++) {
        if (lst == 0 && p1first) {
            name = "P1";
            pins = p1pins;
            if (board_model == 1 && gpio_cfg == 1) {
                map = rev1_p1pin2gpio_map;
                mapcnt = sizeof(rev1_p1pin2gpio_map);
            } else if (board_model == 1 && gpio_cfg == 2) {
                map = rev2_p1pin2gpio_map;
                mapcnt = sizeof(rev2_p1pin2gpio_map);
            } else {
                map = bplus_p1pin2gpio_map;
                mapcnt = sizeof(bplus_p1pin2gpio_map);
            }
            pNpin2servo = p1pin2servo;
        } else {
            name = "P5";
            pins = p5pins;
            if (board_model == 1 && gpio_cfg == 1) {
                map = rev1_p5pin2gpio_map;
                mapcnt = sizeof(rev1_p5pin2gpio_map);
            } else if (board_model == 1 && gpio_cfg == 2) {
                map = rev2_p5pin2gpio_map;
                mapcnt = sizeof(rev2_p5pin2gpio_map);
            } else {
                map = NULL;
                mapcnt = 0;
            }
            pNpin2servo = p5pin2servo;
        }
        while (*pins) {
            char *end;
            long pin = strtol(pins, &end, 0);

            if (*end && (end == pins || *end != ','))
                fatal("Invalid character '%c' in %s pin list\n", *end, name);
            if (pin < 0 || pin > mapcnt)
                fatal("Invalid pin number %d in %s pin list\n", pin, name);
            if (servo == MAX_SERVOS)
                fatal("Too many servos specified\n");
            if (pin == 0) {
                servo++;
            } else {
                if (map[pin-1] == DMY)
                    fatal("Pin %d on header %s cannot be used for a servo output\n", pin, name);
                pNpin2servo[pin] = servo;
                servo2gpio[servo++] = map[pin-1];
                num_servos++;
            }
            pins = end;
            if (*pins == ',')
                pins++;
        }
    }
    /* Write a cfg file so can tell which pins are used for servos */
    fp = fopen(CFGFILE, "w");
    if (fp) {
        if (p1first)
            fprintf(fp, "p1pins=%s\np5pins=%s\n", p1pins, p5pins);
        else
            fprintf(fp, "p5pins=%s\np1pins=%s\n", p5pins, p1pins);
        fprintf(fp, "\nServo mapping:\n");
        for (i = 0; i < MAX_SERVOS; i++) {
            if (servo2gpio[i] == DMY)
                continue;
            fprintf(fp, "    %2d on %-5s          GPIO-%d\n", i, gpio2pinname(servo2gpio[i]), servo2gpio[i]);
        }
        fclose(fp);
    }
}

uint8_t gpiosearch(uint8_t gpio, uint8_t *map, int len) {
    while (--len) {
        if (map[len] == gpio)
            return len+1;
    }
    return 0;
}

char * gpio2pinname(uint8_t gpio) {
    static char res[16];
    uint8_t pin;

    if (board_model == 1 && gpio_cfg == 1) {
        if ((pin = gpiosearch(gpio, rev1_p1pin2gpio_map, sizeof(rev1_p1pin2gpio_map))))
            sprintf(res, "P1-%d", pin);
        else if ((pin = gpiosearch(gpio, rev1_p5pin2gpio_map, sizeof(rev1_p5pin2gpio_map))))
            sprintf(res, "P5-%d", pin);
        else
            fatal("Cannot map GPIO %d to a header pin\n", gpio);
    } else if (board_model == 1 && gpio_cfg == 2) {
        if ((pin = gpiosearch(gpio, rev2_p1pin2gpio_map, sizeof(rev2_p1pin2gpio_map))))
            sprintf(res, "P1-%d", pin);
        else if ((pin = gpiosearch(gpio, rev2_p5pin2gpio_map, sizeof(rev2_p5pin2gpio_map))))
            sprintf(res, "P5-%d", pin);
        else
            fatal("Cannot map GPIO %d to a header pin\n", gpio);
    } else {
        if ((pin = gpiosearch(gpio, bplus_p1pin2gpio_map, sizeof(bplus_p1pin2gpio_map))))
            sprintf(res, "P1-%d", pin);
        else
            fatal("Cannot map GPIO %d to a header pin\n", gpio);
    }

    return res;
}