#ifndef LEDEK_GPIO
#define LEDEK_GPIO

#include <stdint.h>

#define DMY	255	// Used to represent an invalid P1 pin, or unmapped servo

#define NUM_P1PINS	40
#define NUM_P5PINS	8

#define GPIO_BASE_OFFSET	0x00200000
#define GPIO_LEN		0x100

#define GPIO_VIRT_BASE		(periph_virt_base + GPIO_BASE_OFFSET)
#define GPIO_PHYS_BASE		(periph_phys_base + GPIO_BASE_OFFSET)

#define GPIO_FSEL0		(0x00/4)
#define GPIO_SET0		(0x1c/4)
#define GPIO_CLR0		(0x28/4)
#define GPIO_LEV0		(0x34/4)
#define GPIO_PULLEN		(0x94/4)
#define GPIO_PULLCLK		(0x98/4)

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1

#endif //LEDEK_GPIO
