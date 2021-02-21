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
 * Send feedback to <rick@efn.org> and the current
 *
 * Description : Stepper motor driver for Raspberry PI 4

 Stepper API

  - user space uses sys/kernel/blah/file1 to set three parameters:
    - (signed) distance to move in step increments, + = CW, - = CCW
    - with a maximum speed in steps/second.  NOTE: Max step freq of DRV8825 is 250 KHz, 17PM-J204-G1ST has 200 steps/rotation and 1500 RPM max (just a wild guess) 
    - direction of rotation (selected with sign of distance)
    - microsteps to use (full, half, 1/4, ... 1/128)
    - ramp aggressiveness (not sure what units)

  - driver uses a "S" curve to ramp motor up and down over the desired length, speed, direction, and aggressiveness
  - user can poll /sys/kernel/blah/file2 to get status of motor movement
  - user can write sys/kernel/blah/file3 to abort motor movement

  use a PWM (with DMA) to do the actual waveform to the motor.  Using the DMA it's possible to setup a table of pulses with variable periods between each pulse.

  - Steps/pulse output GPIO pin
  - Direction GPIO pin
  - Mode/microsteps GPIO pins, mode needs 3 pins
  _ Maybe even other pins like sleep, reset, fault. Not sure if I would ever use those pins. Personally I would not worry about them at all.
- Motor steps per revolution. All motors are not the same. Mine is 200 steps which seems common but it can be different.
- Not sure about direction since it really is based on how the motor is wired as well as how you wire up the motor. But also the orientation where you define direction. Facing the motor shaft side or back side. This is from experience which I will explain later.

 NOTE that: PWM and audio share the same clock and thus changing the clock divisor immediately distorts the audio
*/

typedef enum {
	GPIO_MICROSTEP0,
	GPIO_MICROSTEP1,
	GPIO_MICROSTEP2,
	GPIO_DIRECTION,
	GPIO_STEP,
	GPIO_CONTROL_MAX,} GPIO_CONTROL;

/* RAMP pins available on connector p1 */
typedef enum {RAMP_LOW = 0,
	RAMP_MODERATE = 1,
	RAMP_HIGH = 2} RAMP;

/* GPIO pins available on connector p1 */
typedef enum {GPIO_02 = 2,
              GPIO_03 = 3,
              GPIO_04 = 4,
              GPIO_05 = 5,
              GPIO_06 = 6,
              GPIO_07 = 7,
              GPIO_08 = 8,
              GPIO_09 = 9,
              GPIO_10 = 10,
              GPIO_11 = 11,
              GPIO_12 = 12,
              GPIO_13 = 13,
              GPIO_14 = 14,
              GPIO_15 = 15,
              GPIO_16 = 16,
              GPIO_17 = 17,
              GPIO_18 = 18,
              GPIO_19 = 19,
              GPIO_20 = 20,
              GPIO_21 = 21,
              GPIO_22 = 22,
              GPIO_23 = 23,
              GPIO_24 = 24,
              GPIO_25 = 25,
              GPIO_26 = 26,
              GPIO_27 = 27} GPIO;

/* struct to send to /sys/kernel/stepper */
struct STEPPER_SETUP
	{
	s32 distance;  /* in step degree increments (see microstep_control below) NOTE: signed, 
												pos = DIR pin high, neg = DIR pin low NOTE: if = 0 then stop */
	u32 max_speed;  /* max speed in steps/second NOTE: if = 0 then stop */
	u32 min_speed;  /* max speed in steps/second */
	u8 microstep_control;  /* bit 0 is value for gpio_microstep0, bit 1 = microstep1, set the step mode - 0=full, 1=1/2, 2=1/4, 3=1/8/ 4=1/16, etc step. Internal pulldown.
 */
	RAMP ramp_aggressiveness;  /* 0 = 1/2 ramp up/down 1=1/3, 2=1/4, 3=1/5, etc */
	GPIO gpios[GPIO_CONTROL_MAX];  /* any GPIO */
	u32 status;  /* upon read, shows the control status register of the DMA, see page 52 of rpi_DATA_2711_1p0.pdf  */
	};

#define MAX_STEPS (128 * 1024)
#define DIFF_SCALE (1 << 8)  /* divisor used for aggressiveness, lower bits treated as fraction */
#define RPI4_CRYSTAL_FREQ 54000000
#define PWM_FREQ (RPI4_CRYSTAL_FREQ / 2)  /* set for max granularity */
