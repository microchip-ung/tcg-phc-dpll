obj-m = microchip-dpll-i2c.o microchip-dpll-spi.o
ccflags-y += -I$(PWD)/../include

KVERSION = $(shell uname -r)
all:
	        make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	        make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
