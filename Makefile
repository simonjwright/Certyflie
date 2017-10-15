# Makefile for the sjw branch of Certyflie.

all: cflie.bin

cflie.elf: force
	gprbuild -p -P crazyflie.gpr

%.bin: %.elf
	arm-eabi-objcopy -O binary $< $@

clean:
	gprclean -P crazyflie.gpr -r

.PHONY: all clean force
