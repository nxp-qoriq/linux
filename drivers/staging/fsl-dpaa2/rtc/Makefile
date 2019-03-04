
obj-$(CONFIG_FSL_DPAA2_PTP_CLOCK) += dpaa2-rtc.o

dpaa2-rtc-objs := rtc.o dprtc.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
