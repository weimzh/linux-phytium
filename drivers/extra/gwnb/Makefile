ifeq ($(KERNELRELEASE), )
KERNELDIR := /lib/modules/$(shell uname -r)/build
PWD :=$(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR)  M=$(PWD)  
clean:
	rm -rf *.mk .tmp_versions Module.symvers *.mod.c *.o *.ko .*.cmd Module.markers modules.order
load:
	insmod gwnb_power.ko
unload:
	rmmod gwnb_power
install: default
	mkdir -p /lib/modules/$(shell uname -r)/kernel/drivers/misc/
	cp -f ./gwnb_power.ko /lib/modules/$(shell uname -r)/kernel/drivers/misc/
	depmod -a
	echo "gwnb_power" >> /etc/modules
uninstall:
	rm -rf /lib/modules/$(shell uname -r)/kernel/drivers/misc/gwnb_power.ko
	depmod -a
else
	obj-m := gwnb_power.o
endif