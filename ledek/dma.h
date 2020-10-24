#ifndef LEDEK_DMA
#define LEDEK_DMA

#include <stdint.h>


#define DMA_CHAN_SIZE		0x100
#define DMA_CHAN_MIN		0
#define DMA_CHAN_MAX		14
#define DMA_CHAN_DEFAULT	14
#define DMA_CHAN_PI4		7

#define DMA_BASE_OFFSET		0x00007000
#define DMA_LEN			DMA_CHAN_SIZE * (DMA_CHAN_MAX+1)

#define DMA_VIRT_BASE		(periph_virt_base + DMA_BASE_OFFSET)

#define DMA_NO_WIDE_BURSTS	(1<<26)
#define DMA_WAIT_RESP		(1<<3)
#define DMA_D_DREQ		(1<<6)
#define DMA_PER_MAP(x)		((x)<<16)
#define DMA_END			(1<<1)
#define DMA_RESET		(1<<31)
#define DMA_INT			(1<<2)

#define DMA_CS			(0x00/4)
#define DMA_CONBLK_AD		(0x04/4)
#define DMA_SOURCE_AD		(0x0c/4)
#define DMA_DEBUG		(0x20/4)

typedef struct {
    uint32_t info, src, dst, length,
            stride, next, pad[2];
} dma_cb_t;

#endif //LEDEK_DMA
