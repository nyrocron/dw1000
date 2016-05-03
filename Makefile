MODNAME := dw1000

# Invoked from kernel build system
ifneq ($(KERNELRELEASE),)

	obj-m := $(MODNAME).o
#	$(MODNAME)-objs := dw1000.o


# Invoked from command line
else
	
	RHOST ?= pi2-a
	RANR ?= 1
	KERNELDIR ?= /home/florian/uni/rpi2/linux
	PWD := $(shell pwd)

default: $(MODNAME).ko

dw1000.ko: $(MODNAME).c
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

load: $(MODNAME).ko
	sudo insmod $(MODNAME).ko

rload: $(MODNAME).ko
	scp $< $(RHOST):
	ssh $(RHOST) 'sudo insmod $<'

start: load
	sudo ip link set up wpan0
	sudo ip link add link wpan0 name lowpan0 type lowpan
	sudo ip link set up lowpan0

rstart: rload
	ssh $(RHOST) 'sudo ip link set addr 0a:00:00:0$(RANR) dev dw0'
	ssh $(RHOST) 'sudo ip addr add 10.0.0.$(RANR)/16 dev dw0'
	ssh $(RHOST) 'sudo ip link set up dw0'

unload:
	sudo rmmod $(MODNAME).ko

runload:
	ssh $(RHOST) 'sudo rmmod $(MODNAME).ko'

clean:
	rm -rf *.mod.c *.o *.ko .*.cmd Module.symvers modules.order .tmp_versions

endif

.PHONY: load rload unload runload start rstart clean
