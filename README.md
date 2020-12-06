  RPI Stepper Motor Linux Kernel Driver
==========================================

This is everything you need to build a stepper motor driver that is a real Linux kernel module.

This code consists of a Makefile, one driver C file, one test C file and a common shared header file

Features

- Driver runs in kernel and not in user space.
- Uses PWM or PCM to provide delays between motor STEP cycles
- Uses DMA to stream pulses
- Only uses 6 DMA's per motor STEP cycle (when using PWM)
- Pulse width granularity based on 27 MHz
- Can use any unused GPIO's for STEP, DIRECTION, and MICROSTEP control pins
- Driver smoothly ramps motor up to max speed and back down again based on a (approximate) sinusoidal curve
- Drive multiple motors (future)

Untested
- Only tested on RPI-4
- Multiple motors untested

1. Get and build everything. We have to tweak the device tree and need a header inside the kernel so we have to build.  We will keep your old kernel so you can go back to that if need be.  This kernel and module directory will have "motor" in the name.

```
git clone --recursive https://github.com/rickbronson/RPI-Stepper-Motor-Linux-Kernel-Driver.git
```

  At this point we need to get a kernel running that we can get the linux-headers for.

```
sudo cp -a /boot /boot.sav  # save old kernel and device tree (optional)
sudo apt update
sudo rpi-update stable  # get stable kernel
sudo reboot # after which you should be running new kernel
```
  Now get the kernel headers for the updated kernel and other programs we need to build the device tree.

```
sudo apt install raspberrypi-kernel-headers git bc bison flex libssl-dev make
git clone --depth=1 https://github.com/raspberrypi/linux
cd linux
make bcm2711_defconfig

```

  Now we need to insert our tweaked device tree file

```
cp -f ../RPI-Stepper-Motor-Linux-Kernel-Driver/src/bcm270x.dtsi arch/arm/boot/dts/bcm270x.dtsi
```

  Then continue building the kernel device tree

```
make -j4 dtbs
sudo cp arch/arm/boot/dts/*.dtb /boot/
sudo reboot # to load the new device tree
```

  Lastly build this driver using the kernel headers installed above.

```
cd ../RPI-Stepper-Motor-Linux-Kernel-Driver/src/
make
```

2. Hookup
--------------

Simplified hookup of the motor with a DRV8825 driver board and serial debug
![Motor hookup](https://github.com/rickbronson/RPI-Stepper-Motor-Linux-Kernel-Driver/blob/master/docs/hardware/schematic10.png "Motor hookup")

3. Example speed profiles
--------------

Here are some example speed profiles:
./test-stepper -d 110 -n 400 -s 1000 -r 10 -m 7 -g 10 -v
![Motor hookup](https://github.com/rickbronson/RPI-Stepper-Motor-Linux-Kernel-Driver/blob/master/docs/plot1.png "plot 1")
./test-stepper -d 200 -n 400 -s 1000 -r 10 -m 7 -g 10 -v
![Motor hookup](https://github.com/rickbronson/RPI-Stepper-Motor-Linux-Kernel-Driver/blob/master/docs/plot2.png "plot 2")

4. Run
--------------

  do "./test-stepper --help" for test program options

```
cd ~/RPI-Stepper-Motor-Linux-Kernel-Driver/src
sudo rmmod pwm-bcm2835  # remove existing driver if any
sudo rmmod pwm-stepper-bcm2835 # remove our driver if it's already installed
sudo insmod ./pwm-stepper-bcm2835.ko
sudo test-stepper -d 500 -n 400 -s 4000 -r 1 -m 7  # move 500 microsteps, start at 400Hz, max 4000Hz
sudo test-stepper -d 500 -n 400 -s 4000 -r 3 -m 7  # same, more aggressive curve
```

  I test my motor up to 215000 Hz (without load at "-m 7" and it worked well.

5. Debug

  See pwm-stepper-bcm2835.c for printk statements.

  You can generate the speed profile using the test-stepper program with the "-g" option as below:

```
./test-stepper -d 25600 -n 1280 -s 128000 -m 7 -g
```

6. Comments/suggestions

  Please contact me at rick AT efn DOT org
