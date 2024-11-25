obj-m = ptp_zl3073x.o
ccflags-y += -I$(PWD)/../include

KVERSION = $(shell uname -r)
all:
	        make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	        make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
