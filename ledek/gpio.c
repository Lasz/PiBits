#include "gpio.h"

static uint8_t rev1_p1pin2gpio_map[] = {
        DMY,	// P1-1   3v3
        DMY,	// P1-2   5v
        0,	// P1-3   GPIO 0 (SDA)
        DMY,	// P1-4   5v
        1,	// P1-5   GPIO 1 (SCL)
        DMY,	// P1-6   Ground
        4,	// P1-7   GPIO 4 (GPCLK0)
        14,	// P1-8   GPIO 14 (TXD)
        DMY,	// P1-9   Ground
        15,	// P1-10  GPIO 15 (RXD)
        17,	// P1-11  GPIO 17
        18,	// P1-12  GPIO 18 (PCM_CLK)
        21,	// P1-13  GPIO 21
        DMY,	// P1-14  Ground
        22,	// P1-15  GPIO 22
        23,	// P1-16  GPIO 23
        DMY,	// P1-17  3v3
        24,	// P1-18  GPIO 24
        10,	// P1-19  GPIO 10 (MOSI)
        DMY,	// P1-20  Ground
        9,	// P1-21  GPIO 9 (MISO)
        25,	// P1-22  GPIO 25
        11,	// P1-23  GPIO 11 (SCLK)
        8,	// P1-24  GPIO 8 (CE0)
        DMY,	// P1-25  Ground
        7,	// P1-26  GPIO 7 (CE1)
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
        2,	// P1-3   GPIO 2 (SDA)
        DMY,	// P1-4   5v
        3,	// P1-5   GPIO 3 (SCL)
        DMY,	// P1-6   Ground
        4,	// P1-7   GPIO 4 (GPCLK0)
        14,	// P1-8   GPIO 14 (TXD)
        DMY,	// P1-9   Ground
        15,	// P1-10  GPIO 15 (RXD)
        17,	// P1-11  GPIO 17
        18,	// P1-12  GPIO 18 (PCM_CLK)
        27,	// P1-13  GPIO 27
        DMY,	// P1-14  Ground
        22,	// P1-15  GPIO 22
        23,	// P1-16  GPIO 23
        DMY,	// P1-17  3v3
        24,	// P1-18  GPIO 24
        10,	// P1-19  GPIO 10 (MOSI)
        DMY,	// P1-20  Ground
        9,	// P1-21  GPIO 9 (MISO)
        25,	// P1-22  GPIO 25
        11,	// P1-23  GPIO 11 (SCLK)
        8,	// P1-24  GPIO 8 (CE0)
        DMY,	// P1-25  Ground
        7,	// P1-26  GPIO 7 (CE1)
};

static uint8_t rev2_p5pin2gpio_map[] = {
        DMY,	// P5-1   5v0
        DMY,	// P5-2   3v3
        28,	// P5-3   GPIO 28 (I2C0_SDA)
        29,	// P5-4   GPIO 29 (I2C0_SCL)
        30,	// P5-5   GPIO 30
        31,	// P5-6   GPIO 31
        DMY,	// P5-7   Ground
        DMY,	// P5-8   Ground
};

static uint8_t bplus_p1pin2gpio_map[] = {
        DMY,	// P1-1   3v3
        DMY,	// P1-2   5v
        2,	// P1-3   GPIO 2 (SDA)
        DMY,	// P1-4   5v
        3,	// P1-5   GPIO 3 (SCL)
        DMY,	// P1-6   Ground
        4,	// P1-7   GPIO 4 (GPCLK0)
        14,	// P1-8   GPIO 14 (TXD)
        DMY,	// P1-9   Ground
        15,	// P1-10  GPIO 15 (RXD)
        17,	// P1-11  GPIO 17
        18,	// P1-12  GPIO 18 (PCM_CLK)
        27,	// P1-13  GPIO 27
        DMY,	// P1-14  Ground
        22,	// P1-15  GPIO 22
        23,	// P1-16  GPIO 23
        DMY,	// P1-17  3v3
        24,	// P1-18  GPIO 24
        10,	// P1-19  GPIO 10 (MOSI)
        DMY,	// P1-20  Ground
        9,	// P1-21  GPIO 9 (MISO)
        25,	// P1-22  GPIO 25
        11,	// P1-23  GPIO 11 (SCLK)
        8,	// P1-24  GPIO 8 (CE0)
        DMY,	// P1-25  Ground
        7,	// P1-26  GPIO 7 (CE1)
        DMY,	// P1-27  ID_SD
        DMY,	// P1-28  ID_SC
        5,	// P1-29  GPIO 5
        DMY,	// P1-30  Ground
        6,	// P1-31  GPIO 5
        12,	// P1-32  GPIO 12
        13,	// P1-33  GPIO 13
        DMY,	// P1-34  Ground
        19,	// P1-35  GPIO 19
        16,	// P1-36  GPIO 16
        26,	// P1-37  GPIO 26
        20,	// P1-38  GPIO 20
        DMY,	// P1-39  Ground
        21,	// P1-40  GPIO 21
};