# Makefile for the sjw branch of Certyflie.

export RUNTIME=/Users/simon/cortex-gnat-rts/local/stm32f4
export PLATFORM_BUILD=Production
export ADL_BUILD=Production
export ADL_BUILD_CHECKS=Enabled

all: cflie.bin

cflie.elf: force
	gprbuild -p -P crazyflie.gpr

%.bin: %.elf
	arm-eabi-objcopy -O binary $< $@

clean:
	gprclean -P crazyflie.gpr -r

.PHONY: all clean force
