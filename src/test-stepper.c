/*********************************************************************
 * test-stepper.c - PWM RPI4 test program

To make:
make

To run:
./test-stepper -r 1 -d 1024 -g 1024 | less
./test-stepper -r 1 -d 1024 -g 1024 | grep otal
./test-stepper -r 1 -d 1024 -g 1024 | grep otal
 generate some waveform graphs:
./test-stepper2 -d 200 -n 400 -s 1000 -r 1 -m 7 -g -v 
./test-stepper2 -d 112 -n 400 -s 1000 -r 10 -m 7 -g -v 

 *********************************************************************/

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
typedef unsigned char		u8;
typedef unsigned short		u16;
typedef unsigned int		u32;
typedef unsigned long long	u64;
typedef signed char		s8;
typedef short			s16;
typedef int			s32;
typedef long long		s64;
#include "rpi4-stepper.h"


#define STEP_CMD_FILE "/sys/devices/platform/soc/fe20c000.pwm/cmd"

/* A string listing valid short options letters.  */
const char* program_name;  /* The name of this program.  */
const char* const short_options = "d:s:n:m:p:r:gtv";
  /* An array describing valid long options.  */
const struct option long_options[] = {
    { "distance",      1, NULL, 'd' },
    { "max_speed",     1, NULL, 's' },
    { "min_speed",     1, NULL, 'n' },
    { "microstep",     1, NULL, 'm' },
    { "step_gpio",     1, NULL, 'p' },
    { "ramp",     1, NULL, 'r' },
    { "generate",     0, NULL, 'g' },
    { "status",     0, NULL, 't' },
    { "verbose",   0, NULL, 'v' },
    { NULL,        0, NULL, 0   }   /* Required at end of array.  */
  };

void print_usage (FILE* stream, int exit_code)
{
  fprintf (stream, "Usage:  %s options\n", program_name);
  fprintf (stream,
           "  -d  --distance     Distance in steps >=0 sets direction pin to 0, <0 sets direction pin to 1\n"
           "  -s  --max_speed    Max speed in steps/second\n"
           "  -n  --min_speed    Min speed in steps/second\n"
           "  -m  --microstep    Microstep [7]\n"
           "  -p  --step_gpio    Step GPIO [13]\n"
           "  -r  --ramp         Ramp_aggressiveness 1-X (lower 4 bits are treated as a fraction)\n"
           "  -g  --generate     Generate waveform graph\n"
           "  -t  --status       Get DMA status register\n"
           "  -v  --verbose      Print verbose messages.\n");
  exit (exit_code);
}

#define CBS_PER_PULSE 6  /* control blocks per pulse */
#define MAX_CBS (MAX_STEPS * CBS_PER_PULSE)

struct bcm2708_dma_cb {
	u32 info;
	u32 src;
	u32 dst;
	u32 length;
	u32 stride;
	u32 next;
	u32 pad[2];
};

struct dma_data {  /* everything got with alloc coherent */
	struct bcm2708_dma_cb cbs[MAX_CBS];  /* must be on 8 word boundry */
	u32 range[MAX_STEPS];
	u32 gpio_val;
	};

struct stepper_priv {
	struct STEPPER_SETUP step_cmd;
	struct dma_data *dma_send_buf;  /* dma data to send to motor step pin via pwm */
	int verbose;
	} priv_data = {0};

struct STEPPER_SETUP setup =
	{
	.distance = 100,  /* in steps NOTE: signed, 
										pos = DIR pin high, neg = DIR pin low NOTE: if = 0 then stop */
	.min_speed = 1,  /* max speed in steps/second NOTE: if = 0 then stop */
	.max_speed = 200,  /* max speed in steps/second NOTE: if = 0 then stop */
	.microstep_control = 7,  /* bit 0 is value for gpio_microstep0, bit 1 = microstep1, etc */
#define PERIOD_INC 10  /* in tenth's */
	.ramp_aggressiveness = PERIOD_INC,
	.gpio_step = GPIO_13,
	.gpio_direction = GPIO_06,
	.gpio_microstep0 = GPIO_19,
	.gpio_microstep1 = GPIO_20,
	.gpio_microstep2 = GPIO_21,
	};

#define printk printf
#define KERN_INFO
static int last_freq = 0;
static struct bcm2708_dma_cb *build1pulse(struct stepper_priv *priv, struct bcm2708_dma_cb *pCbs, int index, int half_period)
	{  /* half the period for high and low half-cycles */
	if (priv->verbose) {
		int freq = PWM_FREQ / (half_period * 2);

		printf("pulse dx=%d diff=%d freq=%d per=%d\n", index, freq - last_freq, freq, half_period);
		last_freq = freq;
		}
	priv->dma_send_buf->range[index] = half_period;

	pCbs += 6;  /* move 6 struct's */

	return (pCbs);
	}

/*
 * Main program:
 */
int main(int argc,char **argv) {
  int next_option;
	int steps, fd, generate = 0, get_status = 0;
	struct stepper_priv *priv = &priv_data;
	memcpy(&priv->step_cmd, &setup, sizeof(struct STEPPER_SETUP));
  do
    {
    next_option = getopt_long (argc, argv, short_options,
                               long_options, NULL);
    switch (next_option)
      {
      case 'd':   /* -d or --distance */
        priv->step_cmd.distance = strtol (optarg, NULL, 0);
				if (priv->step_cmd.distance > MAX_STEPS) {
					printf("Error: Distance can't exceed %d\n", MAX_STEPS);
					exit(1);
					}
        break;
      case 's':   /* -s or --max_speed */
        priv->step_cmd.max_speed = strtoul (optarg, NULL, 0);
				if (priv->step_cmd.max_speed > 250000)
					printf("Warning: Speed of over 250000 is outside the spec of most motor drivers\n");
        break;
      case 'n':   /* -n or --min_speed */
        priv->step_cmd.min_speed = strtoul (optarg, NULL, 0);
        break;
      case 'm':   /* -m or --microstep */
				priv->step_cmd.microstep_control = strtoul (optarg, NULL, 0);
        break;
      case 'p':   /* -p or --step_gpio */
				priv->step_cmd.gpio_step = strtoul (optarg, NULL, 0);
        break;
      case 'r':   /* -r or --ramp */
        priv->step_cmd.ramp_aggressiveness = strtoul (optarg, NULL, 0);
				if (priv->step_cmd.ramp_aggressiveness <= 0) {
					printf("Error: aggressiveness must be a non-zero positive number\n");
					exit(1);
					}
        break;
      case 'g':   /* -g or --generate */
        generate = 1;
        break;
      case 't':   /* -t or --status */
        get_status = 1;
        break;
      case 'v':   /* -v or --verbose */
        priv->verbose = 1;
        break;
      case '?':   /* The user specified an invalid option.  */
        /* Print usage information to standard error, and exit with exit
           code one (indicating abonormal termination).  */
        print_usage (stderr, 1);
        break;
      case -1:    /* Done with options.  */
        break;
      default:    /* Something else: unexpected.  */
				break;
			}
    }
  while (next_option != -1);

  if (priv->verbose)
    printf ("RPI4 PWM motor driver tester\n");

	if (generate) {  /* generate sample waveform via gnuplot */
		int limit, cntr, up_index, index = 0, freq, diff;
		struct bcm2708_dma_cb *pCbs;

		priv->dma_send_buf = (struct dma_data *) calloc(1024 * 1024 * 64, sizeof(int));  /* get a lot of memory */

		/* ramp up */
		pCbs = priv->dma_send_buf->cbs;  /* pointer to DMA control block */

		/* do UP ramp, First go 1/2 the UP ramp: can't exceed 1/4
			 the distance and can't go over 1/2 max speed */
		freq = priv->step_cmd.min_speed;  /* start off freq */
		diff = priv->step_cmd.ramp_aggressiveness;  /* base 1st diff on ramp_aggressiveness */
		limit = priv->step_cmd.min_speed + (priv->step_cmd.max_speed - priv->step_cmd.min_speed) / 2;
		do {
			if (priv->verbose)
				printk(KERN_INFO "1/4 UP freq = %d, diff = %d   ", freq, diff);
			pCbs = build1pulse(priv, pCbs, index++, PWM_FREQ / freq / 2);
			freq += diff / DIFF_SCALE;
			diff += priv->step_cmd.ramp_aggressiveness;
			}
		while (freq <= limit && index < priv->step_cmd.distance / 4);
	
		diff -= priv->step_cmd.ramp_aggressiveness;  /* roll back from last time through above loop */
		freq -= diff / DIFF_SCALE;
		diff -= priv->step_cmd.ramp_aggressiveness + 2;  /* prepare for next loop */
		freq += diff / DIFF_SCALE;

		printk(KERN_INFO "Rick debug 1/4 UP index = %d, distance = %d speed = %d, aggr = %d\n", index, priv->step_cmd.distance, priv->step_cmd.max_speed, priv->step_cmd.ramp_aggressiveness);
		/* Do 2nd half of UP ramp */
		do {
			pCbs = build1pulse(priv, pCbs, index++, PWM_FREQ / freq / 2);
			freq += diff / DIFF_SCALE;
			diff -= priv->step_cmd.ramp_aggressiveness;
			if (diff < DIFF_SCALE)
				diff += DIFF_SCALE;  /* can't let diff go below scale */
//			printk(KERN_INFO "Rick debug 2nd half of UP index = %d, diff = %d, freq = %d\n", index, diff, freq);
			}
		while (freq <= priv->step_cmd.max_speed && index < priv->step_cmd.distance / 2);
		
		up_index = index;
		printk(KERN_INFO "Rick debug UP index = %d, distance = %d speed = %d, aggr = %d\n", index, priv->step_cmd.distance, priv->step_cmd.max_speed, priv->step_cmd.ramp_aggressiveness);

		/* hold steady at max speed */
		if (freq > priv->step_cmd.max_speed)
			freq = priv->step_cmd.max_speed;
		for (cntr = priv->step_cmd.distance - index * 2; cntr > 0; cntr--) {  /* run at max speed */
			pCbs = build1pulse(priv, pCbs, index++, PWM_FREQ / freq / 2);
			}

		printk(KERN_INFO "Rick debug SP index = %d\n", index);
		/* ramp down, just feed ramp up in reverse */
		while (up_index--) {
			pCbs = build1pulse(priv, pCbs, index++, priv->dma_send_buf->range[up_index]);
			//			printk(KERN_INFO "Rick debug DN 0x%08x\n");
			}
		(pCbs - 1)->next = 0;  /* mark last one as the last */
		printk(KERN_INFO "Rick debug DN index = %d\n", index);

//	printk(KERN_INFO "Rick debug stepper_dma_start %d\n", *priv->system_timer_regs - rick_debug);

//	printk(KERN_INFO "Rick debug dma_send_buf = 0x%x __pa 0x%x dma = 0x%x\n", (int) priv->dma_send_buf, (int) __pa(priv->dma_send_buf), priv->dma_regs->conblk_ad);

			{
			int cntr;
			FILE *gnuplot = popen("gnuplot", "w");

			fprintf(gnuplot, "plot '-'\n");
			for (cntr = 0; cntr < index; cntr++)
				fprintf(gnuplot, "%d %d\n", cntr, PWM_FREQ / 2 / priv->dma_send_buf->range[cntr]);
			fprintf(gnuplot, "e\n");
			fflush(gnuplot);
			getchar();
			}
		free(priv->dma_send_buf);
		exit(0);	
		}
	fd = open(STEP_CMD_FILE, O_RDWR | O_SYNC);  /* might need root access */
	if ( fd < 0 ) {
		perror(STEP_CMD_FILE);
		exit(1);
	}
	if (get_status) {
		if (read(fd, &priv->step_cmd, sizeof(priv->step_cmd)) != sizeof(priv->step_cmd)) {
			perror(STEP_CMD_FILE);
			close(fd);
			exit(1);
			}
		printf("DMA status reg = 0x%08x\n", priv->step_cmd.status);
		close(fd);
		exit(0);
		}

	if (write(fd, &priv->step_cmd, sizeof(priv->step_cmd)) != sizeof(priv->step_cmd)) {
		perror(STEP_CMD_FILE);
		exit(1);
	}
	// fflush(fd);
	close(fd);
	return 0;
	}
