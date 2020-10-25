#include <stdarg.h>
#include <stdio.h>

#include "clk.h"
#include "dma.h"
#include "gpio.h"
#include "hardware.h"
#include "pwm.h"

// bcm_host_get_model_type() return values to name mapping
const char *model_names[] = {
        "A", "B", "A+", "B+", "2B", "Alpha", "CM", "CM2", "3B", "Zero", "CM3",
        "Custom", "ZeroW", "3B+", "3A+", "FPGA", "CM3+", "4B"
};
#define NUM_MODELS	(sizeof(model_names)/sizeof(*model_names))

volatile uint32_t *pwm_reg;
volatile uint32_t *pcm_reg;
volatile uint32_t *clk_reg;
volatile uint32_t *dma_reg;
volatile uint32_t *gpio_reg;

int delay_hw = DELAY_VIA_PWM;

uint32_t periph_phys_base;
uint32_t periph_virt_base;
uint32_t dram_phys_base;
uint32_t mem_flag;

void terminate(int dummy) {
    int i;

    if (dma_reg && mbox.virt_addr) {
        for (i = 0; i < MAX_SERVOS; i++) {
            if (servo2gpio[i] != DMY)
                set_servo(i, 0);
        }
        udelay(cycle_time_us);
        dma_reg[DMA_CS] = DMA_RESET;
        udelay(10);
    }
    if (restore_gpio_modes) {
        for (i = 0; i < MAX_SERVOS; i++) {
            if (servo2gpio[i] != DMY)
                gpio_set_mode(servo2gpio[i], gpiomode[i]);
        }
    }
    if (mbox.virt_addr != NULL) {
        unmapmem(mbox.virt_addr, mbox.size);
        mem_unlock(mbox.handle, mbox.mem_ref);
        mem_free(mbox.handle, mbox.mem_ref);
        if (mbox.handle >= 0)
            mbox_close(mbox.handle);
    }

    unlink(DEVFILE);
    unlink(CFGFILE);
    exit(1);
}

void fatal(char *fmt, ...) {
    va_list ap;

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    terminate(0);
}

void setup_sighandlers(void) {
    int i;

    // Catch all signals possible - it is vital we kill the DMA engine
    // on process exit!
    for (i = 0; i < 64; i++) {
        struct sigaction sa;

        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = terminate;
        sigaction(i, &sa, NULL);
    }
}

void init_hardware(void) {
    if (delay_hw == DELAY_VIA_PWM) {
        // Initialise PWM
        pwm_reg[PWM_CTL] = 0;
        udelay(10);
        clk_reg[PWMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz or 750MHz on Pi4)
        udelay(100);
        clk_reg[PWMCLK_DIV] = 0x5A000000 | (plldfreq_mhz<<12);	// set pwm div to give 1MHz
        udelay(100);
        clk_reg[PWMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
        udelay(100);
        pwm_reg[PWM_RNG1] = step_time_us;
        udelay(10);
        pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
        udelay(10);
        pwm_reg[PWM_CTL] = PWMCTL_CLRF;
        udelay(10);
        pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
        udelay(10);
    } else {
        // Initialise PCM
        pcm_reg[PCM_CS_A] = 1;				// Disable Rx+Tx, Enable PCM block
        udelay(100);
        clk_reg[PCMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz or 750MHz on Pi4)
        udelay(100);
        clk_reg[PCMCLK_DIV] = 0x5A000000 | (plldfreq_mhz<<12);	// Set pcm div to give 1MHz
        udelay(100);
        clk_reg[PCMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
        udelay(100);
        pcm_reg[PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16; // 1 channel, 8 bits
        udelay(100);
        pcm_reg[PCM_MODE_A] = (step_time_us - 1) << 10;
        udelay(100);
        pcm_reg[PCM_CS_A] |= 1<<4 | 1<<3;		// Clear FIFOs
        udelay(100);
        pcm_reg[PCM_DREQ_A] = 64<<24 | 64<<8;		// DMA Req when one slot is free?
        udelay(100);
        pcm_reg[PCM_CS_A] |= 1<<9;			// Enable DMA
        udelay(100);
    }

    // Initialise the DMA
    dma_reg[DMA_CS] = DMA_RESET;
    udelay(10);
    dma_reg[DMA_CS] = DMA_INT | DMA_END;
    dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(cb_base);
    dma_reg[DMA_DEBUG] = 7; // clear debug error flags
    dma_reg[DMA_CS] = 0x10880001;	// go, mid priority, wait for outstanding writes

    if (delay_hw == DELAY_VIA_PCM) {
        pcm_reg[PCM_CS_A] |= 1<<2;			// Enable Tx
    }
}

void get_model_and_revision(void) {
    char buf[128], revstr[128], modelstr[128];
    char *ptr, *end, *res;
    int board_revision;
    FILE *fp;

    revstr[0] = modelstr[0] = '\0';

    fp = fopen("/proc/cpuinfo", "r");

    if (!fp)
        fatal("Unable to open /proc/cpuinfo: %m\n");

    while ((res = fgets(buf, 128, fp))) {
        if (!strncasecmp("hardware", buf, 8))
            memcpy(modelstr, buf, 128);
        else if (!strncasecmp(buf, "revision", 8))
            memcpy(revstr, buf, 128);
    }
    fclose(fp);

    if (modelstr[0] == '\0')
        fatal("servod: No 'Hardware' record in /proc/cpuinfo\n");
    if (revstr[0] == '\0')
        fatal("servod: No 'Revision' record in /proc/cpuinfo\n");

    if (strstr(modelstr, "BCM2708"))
        board_model = 1;
    else if (strstr(modelstr, "BCM2709") || strstr(modelstr, "BCM2835"))
        board_model = 2;
    else
        fatal("servod: Cannot parse the hardware name string\n");

    /* Revisions documented at http://elinux.org/RPi_HardwareHistory */
    ptr = revstr + strlen(revstr) - 3;
    board_revision = strtol(ptr, &end, 16);
    if (end != ptr + 2)
        fatal("servod: Failed to parse Revision string\n");
    if (board_revision < 1)
        fatal("servod: Invalid board Revision\n");
    else if (board_revision < 4)
        gpio_cfg = 1;
    else if (board_revision < 16)
        gpio_cfg = 2;
    else
        gpio_cfg = 3;

    if (bcm_host_is_model_pi4()) {
        plldfreq_mhz = PLLDFREQ_MHZ_PI4;
        dma_chan = DMA_CHAN_PI4;
    } else {
        plldfreq_mhz = PLLDFREQ_MHZ_DEFAULT;
        dma_chan = DMA_CHAN_DEFAULT;
    }

    periph_virt_base = bcm_host_get_peripheral_address();
    dram_phys_base = bcm_host_get_sdram_address();
    periph_phys_base = 0x7e000000;

    /*
     * See https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
     *
     * 1:  MEM_FLAG_DISCARDABLE = 1 << 0	// can be resized to 0 at any time. Use for cached data
     *     MEM_FLAG_NORMAL = 0 << 2		// normal allocating alias. Don't use from ARM
     * 4:  MEM_FLAG_DIRECT = 1 << 2		// 0xC alias uncached
     * 8:  MEM_FLAG_COHERENT = 2 << 2	// 0x8 alias. Non-allocating in L2 but coherent
     *     MEM_FLAG_L1_NONALLOCATING =	// Allocating in L2
     *       (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)
     * 16: MEM_FLAG_ZERO = 1 << 4		// initialise buffer to all zeros
     * 32: MEM_FLAG_NO_INIT = 1 << 5	// don't initialise (default is initialise to all ones
     * 64: MEM_FLAG_HINT_PERMALOCK = 1 << 6	// Likely to be locked for long periods of time
     *
     */
    if (board_model == 1) {
        mem_flag         = 0x0c;	/* MEM_FLAG_DIRECT | MEM_FLAG_COHERENT */
    } else {
        mem_flag         = 0x04;	/* MEM_FLAG_DIRECT */
    }
}