
obj-$(CONFIG_PTP_1588_CLOCK_DPAA2) += dpaa2-rtc.o

dpaa2-rtc-objs := rtc.o dprtc.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
