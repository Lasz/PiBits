#ifndef LEDEK_CLK
#define LEDEK_CLK

#include <time.h>

#define CLK_BASE_OFFSET	        0x00101000
#define CLK_LEN			0xA8

#define CLK_VIRT_BASE		(periph_virt_base + CLK_BASE_OFFSET)

#define PLLDFREQ_MHZ_DEFAULT	500
#define PLLDFREQ_MHZ_PI4	750

void udelay(int us);

#endif //LEDEK_CLK
