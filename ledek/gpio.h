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

uint32_t gpio_get_mode(uint32_t gpio);
void gpio_set_mode(uint32_t gpio, uint32_t mode);
void gpio_set(int gpio, int level);
void parse_pin_lists(int p1first, char *p1pins, char*p5pins);
uint8_t gpiosearch(uint8_t gpio, uint8_t *map, int len);
char * gpio2pinname(uint8_t gpio);

#endif //LEDEK_GPIO
