#ifndef LEDEK_HARDWARE
#define LEDEK_HARDWARE

#include <stdint.h>

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

const char *model_names[];
void terminate(int dummy);
void fatal(char *fmt, ...);
void setup_sighandlers(void);
void init_hardware(void);
void get_model_and_revision(void);

#endif //LEDEK_HARDWARE
