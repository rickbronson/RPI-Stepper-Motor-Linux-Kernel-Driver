/*
 * Stepper motor driver for Raspberry PI 4
 *
 * Copyright (C) 2020 Rick Bronson
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Send feedback to <rick@efn.org>
 *
 * Description : Stepper motor driver for Raspberry PI 4
 *
 * Based in some part upon drivers/char/tlclk.c (Copyright (C) 2005 Kontron Canada)
 * and drivers/pwm/pwm-bcm2835.c Copyright 2014 Bart Tanghe
 * dma based on drivers/mmc/host/bcm2835-mmc.c

To build, install headers:

apt-cache search linux-headers-$(uname -r)
# pick one from the above output, then do (for example):
sudo apt install linux-headers-4.19.0-8-amd64 

Makefile:
-----------
all:	driver

obj-m += kpwm3.o

driver:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

cleandriver:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
-----------
NOTE: need to blacklist pwm-bcm2835
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/fs.h>  /* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_data/dma-bcm2708.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/of_address.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/of_dma.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/pwm.h>
#include "rpi4-stepper.h"

MODULE_AUTHOR("Rick Bronson <rick@efn.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom BCM2835 PWM for stepper motor driver");

#define PERI_DMA_MASK 0x7fffffff  /* DMA seems to have a different view of addresses */
#define PERI_BASE   0xfe000000  /* 0xfe000000 for BCM2711,
																	 0x20000000 for BCM2835,
																	 0x3f000000 for BCM2836/7 */
#define GPIO_BASE (PERI_BASE + 0x00200000)  /* GPIO registers base address. */
#define PWM_BASE (PERI_BASE + 0x0020c000)  /* PWM registers base address */
#define PCM_BASE (PERI_BASE + 0x00203000)
#define PWM_CLK_BASE (PERI_BASE + 0x00101000)
#define DMA_BASE   (PERI_BASE + 0x00007000)
#define DMA15_BASE (PERI_BASE + 0x00e05000)
#define SYSTEM_TIMER_CLO (PERI_BASE + 0x00003004)  /* based on 1 MHz */
#define PCMCLK_CTL 38
#define PCMCLK_DIV 39
#define PWMCLK_CTL 40
#define PWMCLK_DIV 41

#define GPSET0 7  /* offset for gpio set/clr */
#define GPCLR0 10
#define GPIO_CLR (GPIO_BASE + GPCLR0 * 4)
#define GPIO_SET (GPIO_BASE + GPSET0 * 4)

#define PWM_RNG1     4  /* offsets for PWM reg's */
#define PWM_FIFO1    6

#define PWM_CTL_MSEN2 (1 << 15)
#define PWM_CTL_PWEN2 (1 << 8)
#define PWM_CTL_MSEN1 (1 << 7)
#define PWM_CTL_CLRF1 (1 << 6)
#define PWM_CTL_USEF1 (1 << 5)
#define PWM_CTL_MODE1 (1 << 1)
#define PWM_CTL_PWEN1 (1 << 0)

#define PWM_DMAC_ENAB      (1 << 31)
#define PWM_DMAC_PANIC(x) ((x) << 8)
#define PWM_DMAC_DREQ_VAL 10
#define PWM_DMAC_DREQ(x)   (x)

#define CLK_PASS 0x5a000000  /* clock password */
#define CLK_CTL_MASH(x) ((x) << 9)
#define CLK_CTL_BUSY    (1 << 7)
#define CLK_CTL_KILL    (1 << 5)
#define CLK_CTL_ENAB    (1 << 4)
#define CLK_CTL_SRC(x)  ((x) << 0)

#define TO_PHYS_KLUDGE(x) (__pa(x) | 0xc0000000)  // be nice to get rid of this

#define DEBUG
#ifdef DEBUG
#define PRINTI(fmt, args...) printk(KERN_INFO fmt, ## args)
#else
#define PRINTI(fmt, args...)
#endif	/* ATA_VERBOSE_DEBUG */

typedef enum {  /* for build_dma_thread() */
	USE_DMA_BUF,
	USE_BUILD_BUF,} BUILDTYPE;

/* GPIO registers */
struct S_GPIO_REGS
	{
	uint32_t gpfsel[6]; uint32_t reserved0;
	uint32_t gpset[2]; uint32_t reserved1;
	uint32_t gpclr[2]; uint32_t reserved2;
	uint32_t gplev[2]; uint32_t reserved3;
	uint32_t gpeds[2]; uint32_t reserved4;
	uint32_t gpren[2]; uint32_t reserved5;
	uint32_t gpfen[2]; uint32_t reserved6;
	uint32_t gphen[2]; uint32_t reserved7;
	uint32_t gplen[2]; uint32_t reserved8;
	uint32_t gparen[2]; uint32_t reserved9;
	uint32_t gpafen[2]; uint32_t reserved10;
	uint32_t gppud;
	uint32_t gppudclk[2]; uint32_t reserved11[4];
};

/* PWM reg's */
struct S_PWM_CTL {
	unsigned pwen1 : 1;
	unsigned mode1 : 1;
	unsigned rptl1 : 1;
	unsigned sbit1 : 1;
	unsigned pola1 : 1;
	unsigned usef1 : 1;
	unsigned clrf : 1;
	unsigned msen1 : 1;
	unsigned pwen2 : 1;
	unsigned mode2 : 1;
	unsigned rptl2 : 1;
	unsigned sbit2 : 1;
	unsigned pola2 : 1;
	unsigned usef2 : 1;
	unsigned reserved1 : 1;
	unsigned msen2 : 1;
	unsigned reserved2 : 16;
};

/* PCM regs */

struct S_PCM_REGS
	{
	uint32_t cs;
	uint32_t fifo;
	uint32_t mode;
	uint32_t rxc;
	uint32_t txc;
	uint32_t dreq;
	uint32_t inten;
	uint32_t intstc;
	uint32_t gray;
	};

/* PWM registers */
struct S_PWM_REGS
	{
	union {
		uint32_t ctl;
		struct S_PWM_CTL ctl_bits;
		} control;

	uint32_t sta;
	uint32_t dmac;
#define DMA_ENAB (1 << 31)
	uint32_t reserved0;
	uint32_t rng1;
	uint32_t dat1;
	uint32_t fif1;
	uint32_t reserved1;
	uint32_t rng2;
	uint32_t dat2;
	};

/* DMA control block "info" field bits */
#define DMA_NO_WIDE_BURSTS          (1 << 26)
#define DMA_PERIPHERAL_MAPPING(x) ((x) << 16)
#define DMA_BURST_LENGTH(x)       ((x) << 12)
#define DMA_SRC_IGNORE              (1 << 11)
#define DMA_SRC_DREQ                (1 << 10)
#define DMA_SRC_WIDTH               (1 <<  9)
#define DMA_SRC_INC                 (1 <<  8)
#define DMA_DEST_IGNORE             (1 <<  7)
#define DMA_DEST_DREQ               (1 <<  6)
#define DMA_DEST_WIDTH              (1 <<  5)
#define DMA_DEST_INC                (1 <<  4)
#define DMA_WAIT_RESP               (1 <<  3)
#define DMA_TDMODE                  (1 <<  1)
#define DMA_INTEN                  (1 <<  0)

#define NORMAL_DMA (DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP)
#define TIMED_DMA(x) (DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(x))

	struct S_DMA_REGS
	{
	uint32_t cs; /* DMA Control and Status */
/* DMA CS Control and Status bits */
#define DMA_CHANNEL_RESET       (1 << 31)
#define DMA_CHANNEL_ABORT       (1 << 30)
#define DMA_WAIT_ON_WRITES      (1 << 28)
#define DMA_PANIC_PRIORITY(x) ((x) << 20)
#define DMA_PRIORITY(x)       ((x) << 16)
#define DMA_INTERRUPT_STATUS    (1 <<  2)
#define DMA_END_FLAG            (1 <<  1)
#define DMA_ACTIVE              (1 <<  0)
	uint32_t conblk_ad; /* DMA Control Block Address */
	uint32_t pad1[6];
	uint32_t debug; /* DMA Channel Debug */
#define DMA_DEBUG_READ_ERR           (1 << 2)
#define DMA_DEBUG_FIFO_ERR           (1 << 1)
#define DMA_DEBUG_RD_LST_NOT_SET_ERR (1 << 0)
	uint32_t pad2[64 - 9];  /* pad out to 0x100 */
	};
/**
 * Private data
 */
struct dma_cb1 {  /* One DMA Control Block (CB), borrowed from include/linux/platform_data/dma-bcm2708.h */
	u32 info;
	u32 src;
	u32 dst;
	u32 length;
	u32 stride;
	u32 next;
	u16 index;  /* DMA unused, we use as index */
	u16 motor;  /* DMA unused, we use as motor */
	u32 range;  /* DMA unused, we use as range, NOTE: range is used in PWM sense, ie. it's a delay */
};

struct dma_cb3 {  /* to make it easier to deal with 3 control blocks that entail one time delay */
	u32 info1;
	u32 *src1;  /* pointer to range value */
	u32 dst1;  /* GPIO perif set/clr reg */
	u32 length1;
	u32 stride1;
	u32 next1;
#define STEP_INDEX_MOTOR 0  /* index in upper 16 bits, motor in lower */
#define STEP_RANGE 1
	u16 index;  /* DMA unused, we use as index */
	u16 motor;  /* DMA unused, we use as motor */
	u32 range;  /* DMA unused, we use as range */
	u32 info2;
	u32 *src2;  /* any valid pointer */
	u32 dst2;  /* PWM perif range reg */
	u32 length2;
	u32 stride2;
	u32 next2;
	u32 pad2[2];
	u32 info3;
	u32 src3;  /* pointer to GPIO value */
	u32 dst3;  /* PWM perif FIFO reg */
	u32 length3;
	u32 stride3;
	u32 next3;
	u32 pad3[2];
};

#define PWM_CBS_PER_STEP 6  /* PWM control blocks per step */
#define PWM_CBS_DUMMY (PWM_CBS_PER_STEP + 1)  /* Dummy so we don't match when searching */
#define PWM_MAX_CBS (MAX_STEPS * PWM_CBS_PER_STEP)
struct pwm_dma_data {  /* everything got with alloc coherent */
	struct dma_cb1 run_cbs[PWM_MAX_CBS];  /* DMA run cbs's, must be on 8 word boundry */
	u32 range[MAX_STEPS * 2];  /* * 2 because we won't always have the same delay for each half cycle */
	u32 gpio_set_mask[MAX_MOTORS];  /* mask for setting microstep/dir GPIO's */
	u32 gpio_clr_mask[MAX_MOTORS];  /* mask for clearing microstep/dir GPIO's */
	};

struct stepper_priv {
	struct completion dma_cmpl;
	struct STEPPER_SETUP step_cmd;
	struct STEPPER_SETUP step_cmd_read;
	volatile unsigned int *pwm_clk_regs; /* Holds the address of PWM CLK registers */
	volatile struct S_DMA_REGS *dma_regs;
#define BCM2711_DMA_CHANNELS   15  /* don't use chan 16 */
	volatile unsigned int *system_timer_regs;
	struct platform_device *pdev;
	struct S_PCM_REGS *pcm_regs;
	struct S_PWM_REGS *pwm_regs;
	struct S_GPIO_REGS *gpio_regs;
	spinlock_t lock;
	struct dma_chan	*dma_chan_tx;  /* DMA channel for writes */
	int irq_number;
	struct dma_slave_config dma_cfg;
	dma_addr_t dma_handle;
	struct pwm_dma_data *dma_send_buf;  /* dma data to send to motor step pin via pwm */
	int dma_size;
	void __iomem *base;  /* base addr of PWM */
	struct clk *clk;
	int motors;  /* number of motors */
	struct dma_cb1 build_cbs[PWM_MAX_CBS];  /* place to build the Control Blocks from new command */
	struct dma_cb3 copy_cbs[PWM_MAX_CBS / 3];  /* place to copy the Control Blocks */
	struct dma_cb1 *cur_buf_end; /* current dma end (from build or combine) */
	struct dma_cb1 *dma_buf_end; /* for end check */
#define GPIO_RPI4_MAX 28
	unsigned char gpio_requested[GPIO_RPI4_MAX];  /* marks if these gpio's are already requested */
#define NO_MOTOR 255
	unsigned char gpio2motor[GPIO_RPI4_MAX];  /* Keeps track of motor index */
	int timer_save;
	};

static int request_set_gpio(struct stepper_priv *priv, GPIO pin, int value)
	{
	int ret = 0;

	if (!priv->gpio_requested[pin]) {  /* only request if we don't already have it */
		ret = gpio_request(pin, NULL);
		if (ret)
			printk(KERN_ERR "pwm-stepper Can't request gpio pin %d\n", pin);
		else {
			gpio_direction_output(pin, value & 1);
			priv->gpio_requested[pin] = 1;
			}
		}
	return ret;
	}

/* fast copy 3 CB's, way faster than memcpy, roughly the same as "inline" and no reg pushes/pops */
static noinline __naked void memcpy_cb3(struct dma_cb3 *dst, struct dma_cb3 *src, int size)
	{
	asm volatile (
    "    mov	ip, sp     \n"
    "    push	{r3-r9, ip, lr, pc}\n"
		"    ldmia  r1!, {r2-r9}     \n"
		"    stmia  r0!, {r2-r9}     \n"
		"    ldmia  r1!, {r2-r9}     \n"
		"    stmia  r0!, {r2-r9}     \n"
		"    ldmia  r1!, {r2-r9}     \n"
		"    stmia  r0!, {r2-r9}     \n"
    "    ldm	sp, {r3-r9, sp, pc}   \n"); 
	}

/* copy control blocks until source end, deleting motor, pass -1 (for del_motor) if no delete */
#define MIN_RANGE_TIME 23  /* in 1 / PWM_FREQ units, 22 or below causes whole step at this period */
/* retun values
copied  deleted  action return  do after return
0       0        Err    MIN     wait/build
0       >MIN            0       build/combine (no copy)
0       <MIN     Mark   deleted wait/build
<MIN    0        Mark   copied  wait/build
<MIN    >0       Mark   copied  wait/build
>MIN    n/a      Mark   copied  build/combine
*/
static int copy_del_cbs(struct stepper_priv *priv, struct dma_cb3 *pCbs_dest, struct dma_cb3 *pCbs_src, int del_motor)
	{
	struct dma_cb3 *pCbs_last_copied = pCbs_src;
	int next, half_steps = 0, range = 0, deleted = 0;

	do {  /* just copy until end */
		next = pCbs_src->next3;  /* next pointer in linked list */
		if (pCbs_src->motor != del_motor) {
			if (half_steps) {
				(pCbs_dest - 1)->range = max(range, MIN_RANGE_TIME);  /* set range on last one */
				range = 0;
				}
			memcpy_cb3(pCbs_dest, pCbs_src, sizeof (*pCbs_dest));
			pCbs_last_copied = pCbs_src;  /* keep track of last one copied so we can mark it as end */
			range = pCbs_src->range;
			pCbs_dest++;
			half_steps++;
			}
		else {  /* skip this one */
			deleted++;
			range += pCbs_src->range;  /* accumulate range */
			}
		pCbs_src++;
		} while (next);
	if (half_steps) {  /* anthing copied? */
		pCbs_last_copied->next3 = 0;  /* mark last src copied the last (in case we wait) */
		(pCbs_dest - 1)->next3 = 0;  /* mark last dest as the last */
		}
	else  /* nothing copied */
		if (deleted < PWM_CBS_PER_STEP * priv->motors)  /* minimum deleted? */
			return (PWM_CBS_PER_STEP - 1);  /* make sure caller waits */
		else
			return 0;
	return half_steps / 2;  /* return full steps */
	}

#define CB_RANGE_OFFSET 0
#define range_addr src1  /* this get's set according to CB_RANGE_OFFSET above ie, 0 is src1, 1 is src2, etc */
/* combine from priv->build_cbs and pCbs_src to run_cbs, return number of blocks combined, takes about 1800us for total of 800+214 steps */
static int combine_dma_threads(struct stepper_priv *priv, struct dma_cb3 *pCbs_dest, struct dma_cb3 *pCbs_src1, struct dma_cb3 *pCbs_src2)
	{
	struct pwm_dma_data *dma_send_buf = priv->dma_send_buf;
	u32 *p_range;
	u32 range, next1 = (u32) pCbs_src1, next2 = (u32) pCbs_src2;  /* NOTE: either could be NULL */
	int total1 = 0, total2 = 0, half_steps = 0, half_step_offset = pCbs_dest - (struct dma_cb3 *) priv->dma_send_buf->run_cbs;

	while (next1 || next2) {  /* while either thread is active */
		if (next1 && (total2 >= total1 || !next2)) {  /* do thread1 */
			next1 = pCbs_src1->next3;  /* next pointer in linked list */
			range = pCbs_src1->range;  /* save range */
			if ((next2 && total1 + range > total2) || !next1) {
				if (total2 >= total1)
					pCbs_src1->range = max(total2 - total1, MIN_RANGE_TIME);  /* set for minimum if needed */
				}
			total1 += range;  /* keep track of total for thread1 */
			memcpy_cb3(pCbs_dest, pCbs_src1, sizeof (*pCbs_dest));  /* do thread1 */
			p_range = &dma_send_buf->range[half_step_offset];
			pCbs_dest->range_addr = p_range;  /* set range addr and range */
			*p_range = pCbs_src1->range;
			pCbs_src1++;  /* move to next step */
			pCbs_dest->next1 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 1);
			pCbs_dest->next2 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 2);
			pCbs_dest->next3 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 3);
			pCbs_dest->info3 &= ~DMA_INTEN;  /* remove interupt on this one */
			half_steps++;
			half_step_offset++;
			pCbs_dest++;
			}
		else
			if (next2 && (total1 > total2 || !next1)) {  /* do thread2 */
				next2 = pCbs_src2->next3;
				range = pCbs_src2->range;
				if ((next1 && total2 + range > total1) || !next2) {
					if (total1 > total2)
						pCbs_src2->range = max(total1 - total2, MIN_RANGE_TIME);  /* set for minimum if needed */
					}
				total2 += range;
				memcpy_cb3(pCbs_dest, pCbs_src2, sizeof (*pCbs_dest));
				p_range = &dma_send_buf->range[half_step_offset];
				pCbs_dest->range_addr = p_range;  /* set range addr and range */
				*p_range = pCbs_src2->range;
				pCbs_src2++;  /* move to next step */
				pCbs_dest->next1 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 1);
				pCbs_dest->next2 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 2);
				pCbs_dest->next3 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 3);
				pCbs_dest->info3 &= ~DMA_INTEN;  /* remove interupt on this one */
				half_steps++;
				half_step_offset++;
				pCbs_dest++;
				}
		if (pCbs_dest >= (struct dma_cb3 *) priv->dma_buf_end) {
			printk(KERN_ERR "pwm-stepper Exceeding size of max steps, pCbs_dest=0x%x dma_buf_end=0x%x\n", (int) pCbs_dest, (int) priv->dma_buf_end);
			return -1;
			}
//		PRINTI("pwm-stepper debug total1 = %d, total2 = %d, range = %d, next1 = 0x%x, next2 = 0x%x\n", total1, total2, range, next1, next2);
		}
	(--pCbs_dest)->info3 |= DMA_INTEN;  /* enable interupt on last one */
	pCbs_dest->next3 = 0;  /* mark last one as the last */
	priv->cur_buf_end = (struct dma_cb1 *) pCbs_dest;  /* save end */
	return half_steps / 2;  /* return steps */
	}

static void setup_pwm(struct stepper_priv *priv)
	{
	struct S_PWM_REGS *pwm_regs = priv->pwm_regs;

	/* Set up PWM */
	pwm_regs->control.ctl = 0;  /* reset PWM */
	udelay(10);

	pwm_regs->sta = -1;
	udelay(10);

	/* enable PWM DMA, raise panic and dreq thresholds to 1 NOTE: setting DREQ(X) will causse
(X+3)/2 glitch cycles (~650 KHz) to come out before our waveform starts */
	pwm_regs->dmac = PWM_DMAC_ENAB | PWM_DMAC_PANIC(15) | PWM_DMAC_DREQ(PWM_DMAC_DREQ_VAL);
	udelay(10);

	pwm_regs->control.ctl = PWM_CTL_CLRF1;  /* clear PWM fifo */
	udelay(10);

	/* enable PWM channel 1 and use fifo */
	pwm_regs->control.ctl = PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;

	}

/*
 * set PWM frequency function
 *  Parameters:
 *   freq   - frequency in Hz
 */
static void pwm_frequency(struct stepper_priv *priv, u32 freq)
	{
	u32 divi, divf, pwen1;

	pwen1 = priv->pwm_regs->control.ctl_bits.pwen1;  /* save state */
	priv->pwm_regs->control.ctl_bits.pwen1 = 0;  /* Disable PWM */

	divi = RPI4_CRYSTAL_FREQ / freq;
	divf = RPI4_CRYSTAL_FREQ % freq;

	*(priv->pwm_clk_regs + PWMCLK_CTL) = CLK_PASS | CLK_CTL_KILL;  /* stop the clock */
	*(priv->pwm_clk_regs + PWMCLK_DIV) = CLK_PASS | (divi << 12) | divf;  /* Set the integer divisor */	*(priv->pwm_clk_regs + PWMCLK_CTL) = CLK_PASS | CLK_CTL_SRC(1);  /* Set source to oscillator */
	*(priv->pwm_clk_regs + PWMCLK_CTL) = CLK_PASS | CLK_CTL_ENAB | CLK_CTL_SRC(1);  /* enable clock */
	udelay(10);
	priv->pwm_regs->control.ctl_bits.pwen1 = pwen1;  /* restore PWM */
	}

static void start_dma(struct stepper_priv *priv)
	{
	int index;

	setup_pwm(priv);
	/* we need to prime the range or we end up with the previous range from the last transfer for the 1st step */
//	PRINTI("pwm-stepper debug 1st Range = 0x%x\n", priv->dma_send_buf->run_cbs[CB_RANGE_OFFSET].range);
	priv->pwm_regs->rng1 = priv->dma_send_buf->run_cbs[CB_RANGE_OFFSET].range;
/* to get rid of glitch cycles, prime the PWM FIFO with some cycles */

	for (index = 0; index < PWM_DMAC_DREQ_VAL + 2; index++)  /* -1: 2 glitches,
				+ 0: 1 glitch, + 1 gives one glitch but correct timing,
				+ 2 is perfect, + 3: 1st step 2x long, + 5: 4x long */
		priv->pwm_regs->fif1 = 0;

	priv->dma_regs->conblk_ad = (int) (priv->dma_send_buf);  /* set start of CB */
	priv->dma_regs->cs = DMA_WAIT_ON_WRITES | DMA_PANIC_PRIORITY(8) |
		DMA_PRIORITY(8) | DMA_ACTIVE;  /* start DMA */
		priv->timer_save = *priv->system_timer_regs;  /* save current timer */
//	PRINTI("pwm-stepper debug step_cmd_write end %d us\n", timer);
	}

/* build control block, see page 42 of rpi_DATA_2711_1p0.pdf */
#define BUILD_CB(inf, source, dest, indx, mtr) \
	pCbs->info = inf;	\
	pCbs->src = (int) source;	\
	pCbs->dst = (dest) & PERI_DMA_MASK;	\
	pCbs->length = 4;	\
	pCbs->stride = 0;	\
	pCbs->next = TO_PHYS_KLUDGE(pCbs + 1);	\
	pCbs->motor = mtr;  /* use unused spot to keep track of motor */ \
	pCbs->index = indx;  /* use unused spot to keep track of index */ \
	pCbs++

/* build DMA control block, NOTE: the ordering of the steps below is really important */
static inline struct dma_cb1 *build1step(struct stepper_priv *priv, struct dma_cb1 *pCbs, int step, u32 half_period, int motor, BUILDTYPE flag)
	{  /* half the period for high and low half-cycles */
	struct pwm_dma_data *dma_send_buf = priv->dma_send_buf;
	int *p_gpio_set_mask = &dma_send_buf->gpio_set_mask[motor];
	int *p_gpio_clr_mask = &dma_send_buf->gpio_clr_mask[motor];
	u32 *p_range = &dma_send_buf->range[step * 2];
	int cb_dx = 0;  /* DMA block index */

//	PRINTI("pwm-stepper debug build1step dx=%d period=%d\n", index, half_period * 2);
	if (flag == USE_DMA_BUF)  /* don't write to range if not a DMA buf build, it will get written during combine */
		*p_range = half_period;
	/* NOTE: CB_RANGE_OFFSET gets set based on the ordering below, set to the relative positioning of the Range write */
	pCbs->range = half_period;  /* save our delay in a secret spot (->src1 in cb3) */
  /* set range on PWM for half_period */
	BUILD_CB(NORMAL_DMA, p_range++, PWM_BASE + PWM_RNG1 * 4, cb_dx++, motor);
  /* delay, half_period range amount via PWM */
	BUILD_CB(NORMAL_DMA | TIMED_DMA(5), p_gpio_set_mask, PWM_BASE + PWM_FIFO1 * 4, cb_dx++, motor);
	/* Set GPIO high */
	BUILD_CB(NORMAL_DMA, p_gpio_set_mask, GPIO_SET, cb_dx++, motor);
	/* Same as above 3 except Set GPIO low */
	if (flag == USE_DMA_BUF)
		*p_range = half_period;
	pCbs->range = half_period;
	BUILD_CB(NORMAL_DMA, p_range, PWM_BASE + PWM_RNG1 * 4, cb_dx++, motor);
	BUILD_CB(NORMAL_DMA | TIMED_DMA(5), p_gpio_clr_mask, PWM_BASE + PWM_FIFO1 * 4, cb_dx++, motor);
	BUILD_CB(NORMAL_DMA, p_gpio_clr_mask, GPIO_CLR, cb_dx++, motor);

	return (pCbs);
	}

/* build DMA control blocks, pass in destination pCB, returns number of steps built or 0 */
static int build_dma_thread(struct stepper_priv *priv, BUILDTYPE flag)
	{
	struct STEPPER_SETUP *p_cmd = &priv->step_cmd;
	struct dma_cb1 *pCbs, *pCbs_reverse;
	struct pwm_dma_data *dma_send_buf = priv->dma_send_buf;
	int motor = priv->gpio2motor[p_cmd->gpios[GPIO_STEP]];
	int *p_gpio_set_mask = &dma_send_buf->gpio_set_mask[motor];
	int *p_gpio_clr_mask = &dma_send_buf->gpio_clr_mask[motor];
	int distance = abs(p_cmd->distance);
	int limit, cntr, up_index, steps = 0, freq, diff, set_mask, clr_mask;

	if (flag == USE_DMA_BUF)
		pCbs = priv->dma_send_buf->run_cbs;
	else
		pCbs = priv->build_cbs;
		
  /* first, setup GPIO set/clears */
	set_mask = clr_mask = 1 << p_cmd->gpios[GPIO_STEP];  /* set our step GPIO */
	for (cntr = GPIO_MICROSTEP0; cntr <= GPIO_MICROSTEP2; cntr++) {
		if (p_cmd->microstep_control & (1 << cntr))
			set_mask |= 1 << p_cmd->gpios[cntr];
		else
			clr_mask |= 1 << p_cmd->gpios[cntr];
		}
	if (p_cmd->distance >= 0)
		set_mask |= 1 << p_cmd->gpios[GPIO_DIRECTION];
	else
		clr_mask |= 1 << p_cmd->gpios[GPIO_DIRECTION];
	*p_gpio_set_mask = set_mask;
	*p_gpio_clr_mask = clr_mask;

	/* do UP ramp, First go 1/2 the UP ramp: can't exceed 1/4
		 the distance and can't go over 1/2 max speed */
	freq = p_cmd->min_speed;  /* start off freq */
	diff = p_cmd->ramp_aggressiveness;  /* base 1st diff on ramp_aggressiveness */
	limit = p_cmd->min_speed + (p_cmd->max_speed - p_cmd->min_speed) / 2;
	do {
		pCbs = build1step(priv, pCbs, steps++, PWM_FREQ / freq / 2, motor, flag);
		freq += diff / DIFF_SCALE;
		diff += p_cmd->ramp_aggressiveness;
		}
	while (freq <= limit && steps < distance / 4);
	
	diff -= p_cmd->ramp_aggressiveness;  /* roll back from last time through above loop */
	freq -= diff / DIFF_SCALE;
	diff -= p_cmd->ramp_aggressiveness + 2;  /* prepare for next loop */
	freq += diff / DIFF_SCALE;

//	PRINTI("pwm-stepper debug 1/4 UP steps = %d, freq = %d, distance = %d speed = %d, aggr = %d\n", steps, freq, distance, p_cmd->max_speed, p_cmd->ramp_aggressiveness);
	/* Do 2nd half of UP ramp */
	do {
		pCbs = build1step(priv, pCbs, steps++, PWM_FREQ / freq / 2, motor, flag);
		freq += diff / DIFF_SCALE;
		diff -= p_cmd->ramp_aggressiveness;
		if (diff < DIFF_SCALE)
			diff += DIFF_SCALE;  /* can't let diff go below scale */
//		PRINTI("pwm-stepper debug 2nd half of UP steps = %d, diff = %d, freq = %d\n", steps, diff, freq);
		}
	while (freq <= p_cmd->max_speed && steps < distance / 2);
		
	up_index = steps;  /* save steps for ramp down */
//	PRINTI("pwm-stepper debug UP steps = %d, freq = %d, distance = %d speed = %d, aggr = %d\n", steps, freq, distance, p_cmd->max_speed, p_cmd->ramp_aggressiveness);

	/* hold steady at max speed */
	if (freq > p_cmd->max_speed)
		freq = p_cmd->max_speed;
	for (cntr = distance - steps * 2; cntr > 0; cntr--) {
		pCbs = build1step(priv, pCbs, steps++, PWM_FREQ / freq / 2, motor, flag);
		}

//	PRINTI("pwm-stepper debug SP steps = %d\n", steps);
	/* ramp down, just feed ramp up in reverse */
	for (pCbs_reverse = pCbs - PWM_CBS_PER_STEP; up_index--; pCbs_reverse -= PWM_CBS_PER_STEP) {
		pCbs = build1step(priv, pCbs, steps++, pCbs_reverse->range, motor, flag);
		//			PRINTI("pwm-stepper debug DN 0x%08x\n");
		}
	(--pCbs)->next = 0;  /* mark last one as the last */
	pCbs->info |= DMA_INTEN;  /* enable interupt on last one */
//	PRINTI("pwm-stepper debug DN steps = %d\n", steps);

//	PRINTI("pwm-stepper debug dma_send_buf = 0x%x __pa 0x%x dma = 0x%x\n", (int) priv->dma_send_buf, (int) __pa(priv->dma_send_buf), priv->dma_regs->conblk_ad);
	return steps;
	}

static irqreturn_t bcm2835_dma_callback(int irq, void *data)
	{
	struct stepper_priv *priv = data;

	if (priv->pwm_regs->sta & 0x1fc)
		printk(KERN_ERR "pwm-stepper Error PWM STA = 0x%x\n", priv->pwm_regs->sta);

	priv->dma_regs->cs |= DMA_INTERRUPT_STATUS;  /* Clear the INT flag */
	PRINTI("pwm-stepper debug dma int complete %dms\n", (*priv->system_timer_regs - priv->timer_save) / 1000);
	complete(&priv->dma_cmpl);

	return IRQ_HANDLED;
	}

static ssize_t step_cmd_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buffer, loff_t pos, size_t count)
	{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stepper_priv *priv = dev_get_drvdata(dev);

	priv->step_cmd_read.status = priv->dma_regs->cs;  /* get status */
	memcpy(buffer, &priv->step_cmd_read, sizeof(priv->step_cmd_read));
//	PRINTI("pwm-stepper debug status = 0x%x, size = %d\n", priv->dma_regs->cs, sizeof(priv->step_cmd_read));
	return sizeof(priv->step_cmd_read);
	}

#define DMA_CMPL_TIMEOUT 2000  /* in ms */
/* check if are still doing a DMA, wait else build and start DMA */
static size_t wait_build(struct stepper_priv *priv, size_t count)
	{
	volatile struct S_DMA_REGS *dma_regs = priv->dma_regs;
	int steps;

	if (priv->step_cmd.wait_timeout && dma_regs->cs & DMA_ACTIVE &&
		dma_regs->conblk_ad) {  /* still doing a DMA? */
//		PRINTI("pwm-stepper waiting for DMA interrupt\n");
		reinit_completion(&priv->dma_cmpl);
		if (!wait_for_completion_timeout(&priv->dma_cmpl,
				msecs_to_jiffies(priv->step_cmd.wait_timeout))) {
			printk(KERN_ERR "pwm-stepper timeout waiting for DMA interrupt, aborting\n");
			priv->dma_regs->cs = DMA_CHANNEL_ABORT;
			return -ETIMEDOUT;
			}
//		PRINTI("pwm-stepper done waiting for DMA interrupt\n");
		}
	if ((steps = build_dma_thread(priv, USE_DMA_BUF))) { /* build to real DMA area */
		priv->cur_buf_end = priv->dma_send_buf->run_cbs + steps * PWM_CBS_PER_STEP;  /* save end, NOTE: the build to non DMA buffer won't be used */
		start_dma(priv);
		}
	return count;
	}

int rick_debug3 = 0; /* keep track of time for debug */
#define report_debug(num) \
	PRINTI("pwm-stepper debug " num " cur_buf_end = 0x%x pCbs3 = 0x%x conblk = 0x%x cs = 0x%x index = %d\n", (int) priv->cur_buf_end, (int) pCbs3, (int) dma_regs->conblk_ad, (int) dma_regs->cs, pCbs->index); \
	PRINTI("pwm-stepper debug copy/build/combine steps = %d %d %d delay CB's =%d\n", (int) copy_steps, (int) build_steps, (int) combine_steps, (int) cntr); \
	PRINTI("pwm-stepper debug timing actual: copy+build %dus combine %dus, est: combine = %dus (%d percent)\n", \
    timer_save, \
	  *priv->system_timer_regs - rick_debug3, \
	  (int) range_ticks / (PWM_FREQ / 1000000), \
	  (int) (range_ticks / (PWM_FREQ / 1000000)) * 100 / (*priv->system_timer_regs - rick_debug3))

/* take a command from user space, if we are not currently doing a DMA to a motor, just start a new DMA.  If we are already doing a DMA, find out where our current trasfer is (via conblk_ad) and calcuate how far we should move ahead, knowing how long it takes us to copy/build/combine, then start the combine at that point. */
static ssize_t step_cmd_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buffer, loff_t pos, size_t count)
	{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stepper_priv *priv = dev_get_drvdata(dev);
	volatile struct S_DMA_REGS *dma_regs = priv->dma_regs;
	struct dma_cb1 *pCbs;  /* pointer to DMA control block */
	struct dma_cb3 *pCbs3, *pCbs3_start, *pCbs3_build;  /* pointer to DMA control block triplet */
	struct STEPPER_SETUP *p_cmd = &priv->step_cmd;
	int timer_save, range_ticks, ticks, ret, cntr, copy_steps = 0, build_steps = 0, combine_steps = 0;

	memcpy(p_cmd, buffer, count);  /* insert this command */
	if (!p_cmd->combine_ticks_per_step)  /* sanity check */
		p_cmd->combine_ticks_per_step = COMBINE_TICKS_PER_STEP;

	if (priv->gpio2motor[p_cmd->gpios[GPIO_STEP]] == NO_MOTOR)
		{  /* this is a new motor request gpio's */
		priv->gpio2motor[p_cmd->gpios[GPIO_STEP]] = priv->motors++;  /* set this motor */
		/* Set the GPIO pins */
//		PRINTI("pwm-stepper debug microstep_control = %d, step = %d, motor = %d\n", p_cmd->microstep_control, p_cmd->gpios[GPIO_STEP], priv->gpio2motor[p_cmd->gpios[GPIO_STEP]]);
		ret = request_set_gpio(priv, p_cmd->gpios[GPIO_MICROSTEP0], p_cmd->microstep_control >> 0);
		ret |= request_set_gpio(priv, p_cmd->gpios[GPIO_MICROSTEP1], p_cmd->microstep_control >> 1);
		ret |= request_set_gpio(priv, p_cmd->gpios[GPIO_MICROSTEP2], p_cmd->microstep_control >> 2);
		ret |= request_set_gpio(priv, p_cmd->gpios[GPIO_DIRECTION], p_cmd->distance >= 0 ? 1 : 0);
		ret |= request_set_gpio(priv, p_cmd->gpios[GPIO_STEP], 0);
		if (ret) {
			printk(KERN_ERR "pwm-stepper Requesting GPIO's that are already used\n");
			return -EBUSY;
			}
		}
	if (abs(p_cmd->distance) > MAX_STEPS) {
		printk(KERN_ERR "pwm-stepper Exceeding size of max steps\n");
		return -EINVAL;
		}
	pCbs = (struct dma_cb1 *) dma_regs->conblk_ad;  /* pointer to DMA control block */
	if (dma_regs->cs & DMA_ACTIVE && pCbs) {  /* still doing a DMA? */
		timer_save = *priv->system_timer_regs;  /* save current timer */
		for (; pCbs->index != 0 && pCbs->next &&  /* first find 1st of 6 CB's */
					 pCbs < priv->cur_buf_end; pCbs++)
			;
		if (pCbs->index || pCbs >= priv->cur_buf_end || !pCbs->next) {  /* are we not at the 1st control block? */
			report_debug("2");
			printk(KERN_ERR "pwm-stepper error: didn't first find 1st of 6 CB's\n");
			return (wait_build(priv, count));
			}
		pCbs3_start = pCbs3 = (struct dma_cb3 *) pCbs;  /* switch to looking at half steps */
		copy_steps = copy_del_cbs(priv, priv->copy_cbs, pCbs3,
			priv->gpio2motor[p_cmd->gpios[GPIO_STEP]]);
		if (copy_steps && copy_steps < PWM_CBS_PER_STEP * priv->motors) {  /* needs to be a minimum amount */
			report_debug("0");
			printk(KERN_ERR "pwm-stepper didn't pass minimum amount test copy_steps=%d\n", copy_steps);
			return (wait_build(priv, count));
			}
		build_steps = build_dma_thread(priv, USE_BUILD_BUF);
		/* calc range offset by adding the real time it took to copy/build plus a guess at combine */
		rick_debug3 = *priv->system_timer_regs;  /* save current timer for real combine time */
		timer_save = *priv->system_timer_regs - timer_save;  /* save time for copy/build */
		range_ticks = timer_save * (PWM_FREQ / 1000000) +
			p_cmd->combine_ticks_per_step * (copy_steps + build_steps);
		for (ticks = 0, cntr = 0; pCbs3->next3 &&  /* find our delay point, at least one CB */
					 pCbs3 < (struct dma_cb3 *) priv->cur_buf_end; cntr++) {
			pCbs3++;
			if (ticks >= range_ticks)
				break;
			ticks += pCbs3->range;
			}
		if (ticks < range_ticks || !pCbs3->next3 ||  /* didn't reach our delay point? */
			pCbs3 >= (struct dma_cb3 *) priv->cur_buf_end) {
			printk(KERN_ERR "pwm-stepper didn't reach our delay point ticks=%d range_ticks=%d pCbs3->next3=0x%x increase COMBINE_TICKS_PER_STEP\n",
				(int) ticks, (int) range_ticks, (int) pCbs3->next3);
			report_debug("3");
			return (wait_build(priv, count));
			}
		if (copy_steps)  /* anything copied? */
			pCbs3_start = (struct dma_cb3 *)priv->copy_cbs + (pCbs3 - pCbs3_start);
		else
			pCbs3_start = NULL;
		if (build_steps)  /* anything built? */
			pCbs3_build = (struct dma_cb3 *)priv->build_cbs;
		else
			pCbs3_build = NULL;
		combine_steps = combine_dma_threads(priv, pCbs3, pCbs3_start, pCbs3_build);
		if (combine_steps > 0) {  /* did combine work? */
			if (pCbs3 < (struct dma_cb3 *) dma_regs->conblk_ad ||  /* has DMA advanced passed where we did combine? */
				  !(dma_regs->cs & DMA_ACTIVE && dma_regs->conblk_ad)) {  /* not still doing a DMA? */
				report_debug("6");
				printk(KERN_ERR "pwm-stepper took too long to copy/build/combine\n");
				return -EINVAL;  /* error */
				}
			}
		else {
			report_debug("5");
			printk(KERN_ERR "pwm-stepper combine error\n");
			return -EINVAL;  /* error */
			}
		report_debug("G");
		}
	else {  /* not currently doing a DMA */
		return (wait_build(priv, count));
		}
	return count;
	}

/* shows up in /sys/devices/platform/stepper_plat/cmd */
static const struct bin_attribute step_cmd_attr = {
	.attr = {.name = "cmd", .mode = 0666},
	.size = sizeof(struct STEPPER_SETUP),	/* Limit image size */
	.write = step_cmd_write,
	.read = step_cmd_read,
};

static int bcm2835_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stepper_priv *priv;
	struct resource *res;
	struct irq_desc *desc;
	int virq, ret = 0;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		printk(KERN_ERR "pwm-stepper: priv alloc failed, decrease MAX_STEPS\n");
		return -ENOMEM;
		}
	init_completion(&priv->dma_cmpl);
	memset(priv->gpio2motor, NO_MOTOR, sizeof(priv->gpio2motor));  /* set to No Motor */
	platform_set_drvdata(pdev, priv);
	priv->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "clock not found: %d\n", ret);

		return ret;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	ret = sysfs_create_bin_file(&pdev->dev.kobj, &step_cmd_attr);
	if (ret) {
		printk(KERN_ERR "pwm-stepper: Failed to create sysfs files\n");
		goto out5;
	}

	/* Map the from PHYSICAL address space to VIRTUAL address space */
	priv->pwm_clk_regs = ioremap(PWM_CLK_BASE, PAGE_SIZE);
	priv->system_timer_regs = ioremap(SYSTEM_TIMER_CLO, PAGE_SIZE);
	priv->dma_regs = ioremap(DMA_BASE, BCM2711_DMA_CHANNELS * 256);
	priv->pcm_regs = ioremap(PCM_BASE, PAGE_SIZE);
	priv->pwm_regs = (struct S_PWM_REGS *)priv->base;
	priv->gpio_regs = (struct S_GPIO_REGS *)ioremap(GPIO_BASE, sizeof(struct S_GPIO_REGS));
	if (!priv->gpio_regs || !priv->pwm_regs || !priv->pwm_clk_regs || !priv->dma_regs)
		{
		printk(KERN_ERR "pwm-stepper: failed to ioremap\n");
		ret = -ENOMEM;
		goto out4;
		}
	priv->dma_chan_tx = dma_request_chan(dev, "rx-tx");
#define DMA_CHANNEL 0  /* The channel we get from the last call */
#define DMA_START_VIRQ_NUM (80 + 32)  /* Gotten from "GIC_SPI 80..." in bcm2711.dtsi */
	for (virq = 0; virq < 64; virq++) {  /* find our interrupt */
		desc = irq_to_desc(virq);
		if (desc->irq_data.hwirq == DMA_START_VIRQ_NUM)
			break;
		}
	if (desc->irq_data.hwirq != DMA_START_VIRQ_NUM) {
		printk(KERN_ERR "pwm-stepper can't find DMA interrupt 112\n");
		goto out4;
		}
	priv->irq_number = virq + DMA_CHANNEL;
	priv->dma_regs = &priv->dma_regs[DMA_CHANNEL];  /* move to our reg's */
	if (request_irq(priv->irq_number, bcm2835_dma_callback, 0, "Stepper DMA IRQ", priv)) {
		printk(KERN_ERR "pwm-stepper request_irq %d failed\n", priv->irq_number);
		goto out4;
		}
	/* preallocate dma buffers */
	priv->dma_size = sizeof(struct pwm_dma_data);
	priv->dma_send_buf = dma_alloc_coherent(dev, priv->dma_size,
		&priv->dma_handle, GFP_KERNEL | GFP_DMA);
	priv->dma_buf_end = &priv->dma_send_buf->run_cbs[PWM_MAX_CBS];  /* end for check */
	PRINTI("pwm-stepper debug Inserting stepper_driver module dma_handle = 0x%x, priv alloc = %d, dma buf alloc = %d\n", (int) priv->dma_handle, sizeof(*priv), priv->dma_size);
	if (!priv->dma_send_buf) {
		printk(KERN_ERR "pwm-stepper: dma buf alloc failed, decrease MAX_STEPS\n");
		return -ENOMEM;
		}
	pwm_frequency(priv, PWM_FREQ);
	return 0;

out4:
	sysfs_remove_bin_file(&pdev->dev.kobj, &step_cmd_attr);

out5:
	platform_device_unregister(pdev);
	kfree(priv);
	return ret;

}

static int bcm2835_pwm_remove(struct platform_device *pdev)
{
	struct stepper_priv *priv = platform_get_drvdata(pdev);
	int ret = 0;

	dmaengine_terminate_all(priv->dma_chan_tx);
	sysfs_remove_bin_file(&pdev->dev.kobj, &step_cmd_attr);
	free_irq(priv->irq_number, priv);  /* free our int and put his back */
	dma_release_channel(priv->dma_chan_tx);
	clk_disable_unprepare(priv->clk);
	dma_free_coherent(&pdev->dev, priv->dma_size, priv->dma_send_buf, priv->dma_handle); 
	kfree(priv);
	PRINTI("pwm-stepper debug bcm2835_pwm_remove 3\n");
 	platform_device_unregister(pdev); // hangs, never returns from <mutex_lock>
	PRINTI("pwm-stepper debug bcm2835_pwm_remove 4\n");
	return ret;
}

static const struct of_device_id bcm2835_pwm_of_match[] = {
	{ .compatible = "brcm,bcm2835-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bcm2835_pwm_of_match);

static struct platform_driver bcm2835_pwm_driver = {
	.driver = {
		.name = "bcm2835-pwm",
		.of_match_table = bcm2835_pwm_of_match,
	},
	.probe = bcm2835_pwm_probe,
	.remove = bcm2835_pwm_remove,
};
module_platform_driver(bcm2835_pwm_driver);
