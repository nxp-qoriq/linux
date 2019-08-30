
obj-$(CONFIG_FSL_DPAA2_PTP_CLOCK) += fsl-dpaa2-ptp.o

fsl-dpaa2-ptp-objs	:= dpaa2-ptp.o dprtc.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
