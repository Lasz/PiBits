#ifndef LEDEK_PWM
#define LEDEK_PWM

#define PWM_BASE_OFFSET		0x0020C000
#define PWM_LEN			0x28

#define PWM_VIRT_BASE		(periph_virt_base + PWM_BASE_OFFSET)
#define PWM_PHYS_BASE		(periph_phys_base + PWM_BASE_OFFSET)

#define PWM_CTL			(0x00/4)
#define PWM_DMAC		(0x08/4)
#define PWM_RNG1		(0x10/4)
#define PWM_FIFO		(0x18/4)

#define PWMCLK_CNTL		40
#define PWMCLK_DIV		41

#define PWMCTL_MODE1		(1<<1)
#define PWMCTL_PWEN1		(1<<0)
#define PWMCTL_CLRF		(1<<6)
#define PWMCTL_USEF1		(1<<5)

#define PWMDMAC_ENAB		(1<<31)
#define PWMDMAC_THRSHLD		((15<<8)|(15<<0))

#define DELAY_VIA_PWM		0

#endif //LEDEK_PWM
