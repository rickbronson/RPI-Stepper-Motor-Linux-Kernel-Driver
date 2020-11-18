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
#include <../drivers/dma/virt-dma.h>
#include "bcm2835-dma.h"

MODULE_AUTHOR("Rick Bronson <rick@efn.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom BCM2835 PWM for stepper motor driver");

int rick_debug = 0;

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
#define PWM_DMAC_DREQ_VAL 3
#define PWM_DMAC_DREQ(x)   (x)

#define CLK_PASS 0x5a000000  /* clock password */
#define CLK_CTL_MASH(x) ((x) << 9)
#define CLK_CTL_BUSY    (1 << 7)
#define CLK_CTL_KILL    (1 << 5)
#define CLK_CTL_ENAB    (1 << 4)
#define CLK_CTL_SRC(x)  ((x) << 0)

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
#define PWM_CBS_PER_PULSE 6  /* PWM control blocks per pulse */
#define PWM_MAX_CBS (MAX_STEPS * PWM_CBS_PER_PULSE)

struct pwm_dma_data {  /* everything got with alloc coherent */
	struct bcm2708_dma_cb cbs[PWM_MAX_CBS];  /* must be on 8 word boundry */
	u32 range[MAX_STEPS];
	u32 gpio_val;
	};

struct stepper_priv {
	struct STEPPER_SETUP step_cmd;
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
	struct bcm2835_chan *bcm2835_chan;
	struct dma_slave_config dma_cfg;
	dma_addr_t dma_handle;
	struct pwm_dma_data *dma_send_buf;  /* dma data to send to motor step pin via pwm */
	struct pwm_dma_data *dma_send_buf_phys;  /* physical addr of above */
	int dma_size;
	void __iomem *base;  /* base addr of PWM */
	struct clk *clk;
};

static int request_set_gpio(GPIO pin, int value)
	{
	int ret = 0;

	ret = gpio_request(pin, NULL);
	if (ret)
		printk(KERN_INFO "Can't request gpio pin %d\n", pin);
	else
		gpio_direction_output(pin, value & 1);
	return ret;
	}

static irqreturn_t bcm2835_dma_callback(int irq, void *data)
	{
	struct stepper_priv *priv = data;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->dma_regs->cs = DMA_INTERRUPT_STATUS;  /* Clear the INT flag */
	spin_unlock_irqrestore(&priv->lock, flags);

	gpio_free(priv->step_cmd.gpio_microstep0);  /* give up GPIO pins */
	gpio_free(priv->step_cmd.gpio_microstep1);
	gpio_free(priv->step_cmd.gpio_microstep2);
	gpio_free(priv->step_cmd.gpio_direction);
	gpio_free(priv->step_cmd.gpio_step);
	printk(KERN_INFO "Rick debug dma int complete %dms\n", (*priv->system_timer_regs - rick_debug) / 1000);
	rick_debug = *priv->system_timer_regs;

	return IRQ_HANDLED;
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

#define TO_PHYS_KLUDGE(x) (__pa(x) | 0xc0000000)  // be nice to get rid of this
/* build control block, see page 42 of rpi_DATA_2711_1p0.pdf */
#define BUILD_CB(inf, source, dest)							\
	pCbs->info = inf;	\
	pCbs->src = (int) source;	\
	pCbs->dst = (dest) & PERI_DMA_MASK;	\
	pCbs->length = 4;	\
	pCbs->stride = 0;	\
	pCbs->next = TO_PHYS_KLUDGE(pCbs + 1);	\
	pCbs++

/* build DMA control block */
static struct bcm2708_dma_cb *build1pulse(struct stepper_priv *priv, struct bcm2708_dma_cb *pCbs, int index, int half_period)
	{  /* half the period for high and low half-cycles */
	struct pwm_dma_data *dma_send_buf_phys = priv->dma_send_buf_phys;
//	printk(KERN_INFO "Rick debug build1pulse dx=%d period=%d\n", index, half_period * 2);
	priv->dma_send_buf->range[index] = half_period;

	/* Set GPIO high */
	BUILD_CB(NORMAL_DMA, &dma_send_buf_phys->gpio_val, GPIO_BASE + GPSET0 * 4);
  /* set range on PWM for half_period */
	BUILD_CB(NORMAL_DMA, &dma_send_buf_phys->range[index], PWM_BASE + PWM_RNG1 * 4);
  /* delay, half_period range amount via PWM */
	BUILD_CB(NORMAL_DMA | TIMED_DMA(5), &dma_send_buf_phys->gpio_val, PWM_BASE + PWM_FIFO1 * 4);
	/* Same as above 3 except Set GPIO low */
	BUILD_CB(NORMAL_DMA, &dma_send_buf_phys->gpio_val, GPIO_BASE + GPCLR0 * 4);
	BUILD_CB(NORMAL_DMA, &dma_send_buf_phys->range[index], PWM_BASE + PWM_RNG1 * 4);
	BUILD_CB(NORMAL_DMA | TIMED_DMA(5), &dma_send_buf_phys->gpio_val, PWM_BASE + PWM_FIFO1 * 4);

	return (pCbs);
	}

/* build DMA control blocks and start DMA */
static int stepper_transfer_dma(struct stepper_priv *priv)
	{
	int limit, cntr, up_index, index = 0, freq, diff;
	struct bcm2708_dma_cb *pCbs = priv->dma_send_buf->cbs;  /* pointer to DMA control block */

	pCbs = priv->dma_send_buf->cbs;  /* pointer to DMA control block */

	/* do UP ramp, First go 1/2 the UP ramp: can't exceed 1/4
		 the distance and can't go over 1/2 max speed */
	freq = priv->step_cmd.min_speed;  /* start off freq */
	diff = priv->step_cmd.ramp_aggressiveness;  /* base 1st diff on ramp_aggressiveness */
	limit = priv->step_cmd.min_speed + (priv->step_cmd.max_speed - priv->step_cmd.min_speed) / 2;
	do {
		pCbs = build1pulse(priv, pCbs, index++, PWM_FREQ / freq / 2);
		freq += diff / DIFF_SCALE;
		diff += priv->step_cmd.ramp_aggressiveness;
		}
	while (freq <= limit && index < priv->step_cmd.distance / 4);
	
	diff -= priv->step_cmd.ramp_aggressiveness;  /* roll back from last time through above loop */
	freq -= diff / DIFF_SCALE;
	diff -= priv->step_cmd.ramp_aggressiveness + 2;  /* prepare for next loop */
	freq += diff / DIFF_SCALE;

	printk(KERN_INFO "Rick debug 1/4 UP index = %d, freq = %d, distance = %d speed = %d, aggr = %d\n", index, freq, priv->step_cmd.distance, priv->step_cmd.max_speed, priv->step_cmd.ramp_aggressiveness);
	/* Do 2nd half of UP ramp */
	do {
		pCbs = build1pulse(priv, pCbs, index++, PWM_FREQ / freq / 2);
		freq += diff / DIFF_SCALE;
		diff -= priv->step_cmd.ramp_aggressiveness;
		if (diff < DIFF_SCALE)
			diff += DIFF_SCALE;  /* can't let diff go below scale */
//		printk(KERN_INFO "Rick debug 2nd half of UP index = %d, diff = %d, freq = %d\n", index, diff, freq);
		}
	while (freq <= priv->step_cmd.max_speed && index < priv->step_cmd.distance / 2);
		
	up_index = index;  /* save index for ramp down */
	printk(KERN_INFO "Rick debug UP index = %d, freq = %d, distance = %d speed = %d, aggr = %d\n", index, freq, priv->step_cmd.distance, priv->step_cmd.max_speed, priv->step_cmd.ramp_aggressiveness);

	/* hold steady at max speed */
	if (freq > priv->step_cmd.max_speed)
		freq = priv->step_cmd.max_speed;
	for (cntr = priv->step_cmd.distance - index * 2; cntr > 0; cntr--) {
		pCbs = build1pulse(priv, pCbs, index++, PWM_FREQ / freq / 2);
		}

	printk(KERN_INFO "Rick debug SP index = %d\n", index);
	/* ramp down, just feed ramp up in reverse */
	while (up_index--) {
		pCbs = build1pulse(priv, pCbs, index++, priv->dma_send_buf->range[up_index]);
		//			printk(KERN_INFO "Rick debug DN 0x%08x\n");
		}
	(--pCbs)->next = 0;  /* mark last one as the last */
	pCbs->info |= DMA_INTEN;  /* enable interupt on last one */
	printk(KERN_INFO "Rick debug DN index = %d\n", index);

//	printk(KERN_INFO "Rick debug stepper_dma_start %d\n", *priv->system_timer_regs - rick_debug);
	rick_debug = *priv->system_timer_regs;

	priv->dma_regs->conblk_ad = (int) (priv->dma_send_buf_phys);  /* set start of CB */
//	printk(KERN_INFO "Rick debug dma_send_buf = 0x%x __pa 0x%x dma = 0x%x\n", (int) priv->dma_send_buf, (int) __pa(priv->dma_send_buf), priv->dma_regs->conblk_ad);

	/* to get rid of glitch cycles, prime the PWM FIFO with some cycles */
	for (index = 0; index < PWM_DMAC_DREQ_VAL + 2; index++)  /* -1: 2 glitches,
				+ 0: 1 glitch, + 1 gives one glitch but correct timing,
				+ 2 is perfect, + 3: 1st pulse 2x long, + 5: 4x long */
		priv->pwm_regs->fif1 = 0;

	priv->dma_regs->cs = DMA_WAIT_ON_WRITES | DMA_PANIC_PRIORITY(8) |
		DMA_PRIORITY(8) | DMA_ACTIVE;  /* start DMA */
	return 0;
	}

static ssize_t step_cmd_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buffer, loff_t pos, size_t count)
	{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stepper_priv *priv = dev_get_drvdata(dev);

	priv->step_cmd.status = priv->dma_regs->cs;  /* get status */
	memcpy(buffer, &priv->step_cmd, sizeof(priv->step_cmd));
	printk(KERN_INFO "Rick debug statu = 0x%x, size = %d\n", priv->dma_regs->cs, sizeof(priv->step_cmd));
	return sizeof(priv->step_cmd);
	}

/* get command from user */
static ssize_t step_cmd_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buffer, loff_t pos, size_t count)
	{

	int ret;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stepper_priv *priv = dev_get_drvdata(dev);
	struct S_PWM_REGS *pwm_regs = priv->pwm_regs;

	memcpy(&priv->step_cmd, buffer, count);
	printk(KERN_INFO "Rick debug microstep_control = %d, step = %d\n", priv->step_cmd.microstep_control, priv->step_cmd.gpio_step);

	/* Setting the GPIO pins */
	ret = request_set_gpio(priv->step_cmd.gpio_microstep0, priv->step_cmd.microstep_control >> 0);
	ret |= request_set_gpio(priv->step_cmd.gpio_microstep1, priv->step_cmd.microstep_control >> 1);
	ret |= request_set_gpio(priv->step_cmd.gpio_microstep2, priv->step_cmd.microstep_control >> 2);
	ret |= request_set_gpio(priv->step_cmd.gpio_direction, priv->step_cmd.distance >= 0 ? 1 : 0);
	priv->step_cmd.distance = abs(priv->step_cmd.distance);
	ret |= request_set_gpio(priv->step_cmd.gpio_step, 0);
	if (ret)
		return -EBUSY;

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

	if (priv->dma_regs->conblk_ad)  { // busy? debug only, won't need if abort works below
		printk(KERN_ERR "Rick pwm-stepper: DMA busy, aborting\n");
		return -EBUSY;
		}
	rick_debug = *priv->system_timer_regs;  /* dave current timer */

	printk(KERN_INFO "Rick debug chan = %d: dma ADDR = 0x%x, handle = 0x%x\n", priv->dma_chan_tx->chan_id, (int) priv->dma_regs, (int) priv->dma_handle);

	priv->dma_send_buf->gpio_val = 1 << priv->step_cmd.gpio_step;  /* set our step GPIO */
	pwm_frequency(priv, PWM_FREQ);
	stepper_transfer_dma(priv);
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
	int ret = 0;

	printk(KERN_INFO "Rick debug Inserting stepper_driver module\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	printk(KERN_INFO "Rick debug PWM base from DT = 0x%x\n", (int) res);
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
		printk(KERN_ERR "Rick pwm-stepper: Failed to create sysfs files\n");
		goto out5;
	}

	/* Map the from PHYSICAL address space to VIRTUAL address space */
	priv->pwm_clk_regs = ioremap(PWM_CLK_BASE, PAGE_SIZE);
	priv->system_timer_regs = ioremap(SYSTEM_TIMER_CLO, PAGE_SIZE);
	priv->dma_regs = ioremap(DMA_BASE, BCM2711_DMA_CHANNELS * 256);
	priv->pcm_regs = ioremap(PCM_BASE, PAGE_SIZE);
	priv->pwm_regs = (struct S_PWM_REGS *)priv->base;
	priv->gpio_regs = (struct S_GPIO_REGS *)ioremap(GPIO_BASE, sizeof(struct S_GPIO_REGS));
	if(!priv->gpio_regs || !priv->pwm_regs || !priv->pwm_clk_regs || !priv->dma_regs)
		{
		printk(KERN_ERR "Rick pwm-stepper: failed to ioremap.\n");
		ret = -ENOMEM;
		goto out4;
		}
	priv->dma_chan_tx = dma_request_chan(dev, "rx-tx");
	priv->bcm2835_chan = to_bcm2835_dma_chan(priv->dma_chan_tx);
	priv->irq_number = priv->bcm2835_chan->irq_number;  /* save our int number */
	free_irq(priv->irq_number, priv->bcm2835_chan);  /* free his interrupt! */
	priv->dma_regs = &priv->dma_regs[priv->bcm2835_chan->ch];  /* move to our reg's */
	/* steal the interrupt for this channel */
	if (request_irq(priv->irq_number, bcm2835_dma_callback, 0, "Stepper DMA IRQ", priv)) {
		printk(KERN_ERR "Rick request_irq failed\n");
		goto out4;
		}
	/* preallocate dma buffers */
	priv->dma_size = sizeof(struct pwm_dma_data);
	priv->dma_send_buf = dma_alloc_coherent(dev, priv->dma_size,
		&priv->dma_handle, GFP_KERNEL | GFP_DMA);
	if (!priv->dma_send_buf) {
		printk(KERN_ERR "Rick stepper: kmalloc failed\n");
		return -ENOMEM;
		}
	priv->dma_send_buf_phys = (struct pwm_dma_data *) (int) priv->dma_handle;

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
	ret = request_irq(priv->irq_number, bcm2835_dma_callback, 0, "Stepper DMA IRQ", priv->bcm2835_chan);
	dma_release_channel(priv->dma_chan_tx);
	clk_disable_unprepare(priv->clk);
	dma_free_coherent(&pdev->dev, priv->dma_size, priv->dma_send_buf, priv->dma_handle); 
	kfree(priv);
	printk(KERN_INFO "Rick debug bcm2835_pwm_remove 3\n");
// 	platform_device_unregister(pdev); hangs, never returns from:  c078d00c:	eb0c0ad8 	bl	c0a8fb74 <mutex_lock>
	printk(KERN_INFO "Rick debug bcm2835_pwm_remove 4\n");
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
