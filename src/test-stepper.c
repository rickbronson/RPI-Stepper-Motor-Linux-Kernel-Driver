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
#include <time.h>
typedef unsigned char		u8;
typedef unsigned short		u16;
typedef unsigned int		u32;
typedef unsigned long long	u64;
typedef signed char		s8;
typedef short			s16;
typedef int			s32;
typedef long long		s64;
#include "rpi4-stepper.h"

#define PERI_BASE   0xfe000000
#define SYSTEM_TIMER_CLO (PERI_BASE + 0x00003004)  /* based on 1 MHz */

#define STEP_CMD_FILE "/sys/devices/platform/soc/fe20c000.pwm/cmd"

/* A string listing valid short options letters.  */
const char* program_name;  /* The name of this program.  */
const char* const short_options = "d:s:n:m:p:r:l:y:tv";
  /* An array describing valid long options.  */
const struct option long_options[] = {
    { "distance",      1, NULL, 'd' },
    { "max_speed",     1, NULL, 's' },
    { "min_speed",     1, NULL, 'n' },
    { "microstep",     1, NULL, 'm' },
    { "step_gpio",     1, NULL, 'p' },
    { "ramp",     1, NULL, 'r' },
    { "loop",     1, NULL, 'l' },
    { "delay",     1, NULL, 'y' },
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
           "  -l  --loop         Loop count\n"
           "  -y  --delay        Delay between loops in milliseconds\n"
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
	.gpios[GPIO_STEP] = GPIO_13,
	.gpios[GPIO_DIRECTION] = GPIO_06,
	.gpios[GPIO_MICROSTEP0] = GPIO_19,
	.gpios[GPIO_MICROSTEP1] = GPIO_20,
	.gpios[GPIO_MICROSTEP2] = GPIO_21,
	};

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
static int map_read_mem(off_t addr)
  {
  void *map_base, *virt_addr; 
  int fd, retval;

  if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    return (fd);
  /* Map one page */
  map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, addr & ~MAP_MASK);
  if (map_base == (void *) -1)
    return -1;
    
  virt_addr = map_base + (addr & MAP_MASK);
  retval = *((unsigned long *) virt_addr);
  if (munmap(map_base, MAP_SIZE) == -1)
    return -1;
  return (retval);
  }

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
	int steps, fd, msdelay = 0, loop = 1, get_status = 0;
	struct stepper_priv *priv = &priv_data;
	int system_timer_regs;
	struct timespec ts = { 0, 5000000 };
	
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
				priv->step_cmd.gpios[GPIO_STEP] = strtoul (optarg, NULL, 0);
        break;
      case 'r':   /* -r or --ramp */
        priv->step_cmd.ramp_aggressiveness = strtoul (optarg, NULL, 0);
				if (priv->step_cmd.ramp_aggressiveness <= 0) {
					printf("Error: aggressiveness must be a non-zero positive number\n");
					exit(1);
					}
        break;
      case 'l':   /* -l or --loop */
        loop = strtoul (optarg, NULL, 0);
        break;
      case 'y':   /* -y or --delay */
        msdelay = strtol (optarg, NULL, 0);
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

	fd = open(STEP_CMD_FILE, O_RDWR | O_SYNC);  /* might need root access */
	if ( fd < 0 ) {
		perror(STEP_CMD_FILE);
		exit(1);
	}
	for (; loop; loop--) {
		lseek(fd, 0, SEEK_SET);
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

		system_timer_regs = map_read_mem(SYSTEM_TIMER_CLO);
		if (write(fd, &priv->step_cmd, sizeof(priv->step_cmd)) != sizeof(priv->step_cmd)) {
			perror(STEP_CMD_FILE);
			exit(1);
			}
		/* delay in milliseconds */
		ts.tv_nsec = (msdelay % 1000) * 1000 * 1000;
		ts.tv_sec = msdelay / 1000;
		if (loop > 1) {
			printf("tv_nsec = %d, tv_sec = %d\n", ts.tv_nsec, ts.tv_sec);
			nanosleep(&ts, NULL);
			}
		}
	system_timer_regs = map_read_mem(SYSTEM_TIMER_CLO) - system_timer_regs;
	printf("write time = %d us\n", system_timer_regs);
	// fflush(fd);
	close(fd);
	return 0;
	}
