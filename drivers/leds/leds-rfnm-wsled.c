// SPDX-License-Identifier: GPL-2.0+

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>

#define MAX_CHAIN_ID   2
#define MAX_LED_ID     4

uint32_t chain[MAX_CHAIN_ID][MAX_LED_ID] = {
	// first is the arm cycle prefetch
	// 4th is the first LED (motherboard)
	// 3rd is the first LED on daughterboard
	// 2nd is the second LED on daughterboard
	// color order is (left to right) GRB
	{0xaaaaaaaa, 0x30ff00, 0, 0},
	{0xaaaaaaaa, 0x30ff00, 0, 0}
};

void __iomem *rfnm_wsled_io;

void rfnm_wsled_send_chain(uint8_t chain_id) {

	uint16_t bitmap;
	
	if(chain_id == 0) {
		bitmap = 0x80;
	} else {
		bitmap = 0x100;
	}
	
	volatile unsigned int *addr;
	addr = (volatile unsigned int *) rfnm_wsled_io;

	uint32_t initial = *addr;
	uint32_t cond_high = initial | bitmap;
	uint32_t cond_low = initial & ~(bitmap);

	// 1000 = 1.06us
	// 200 = 160ns

	int z;

	// do not trigger reset condition here, assume application layer takes care of it
	//*addr = cond_low; for(z = 0; z < 250000; z++) asm volatile ("nop");

	uint32_t send[4];
	for(z = 0; z < 4; z++) {
		send[z] = chain[chain_id][z];
	}
	int8_t current_bit = 0;
	uint8_t current_led = 0;

	uint8_t bit = (send[current_led] & (1 << current_bit)) >> current_bit;

	for(current_led = 0; current_led < 4; current_led++) {
		for(current_bit = 23; current_bit >= 0; current_bit--) {

			if(current_led != 0) {
				*addr = cond_high; // reset condition while prefetching the loop
			}

			bit = (send[current_led] & (1 << current_bit)) >> current_bit;

#if 0
			// reset time -> 280us+
			// send one   -> high for 790 +- 210 ns; then low for 790 +- 210 ns
			// send zero  -> high for 300 +- 80 ns; then low for 790 +- 210 ns


			if(bit) {
				for(z = 0; z < 625; z++) asm volatile ("nop");
			} else {
				for(z = 0; z < 230; z++) asm volatile ("nop");
			}

			*addr = cond_low;

			if(bit) {
				for(z = 0; z < 625; z++) asm volatile ("nop");
			} else {
				for(z = 0; z < 625; z++) asm volatile ("nop");
			}
#endif
#if 0
			// total bit length = 1.25us +- 600ns
			// send one  -> high for 800 +- 150 ns; then low for 450 +- 150ns
			// send zero -> high for 400 +- 150 ns; then low for 850 +- 150ns


			if(bit) {
				for(z = 0; z < 625; z++) asm volatile ("nop");
			} else {
				for(z = 0; z < 310; z++) asm volatile ("nop");
			}

			*addr = cond_low;

			if(bit) {
				for(z = 0; z < 360; z++) asm volatile ("nop");
			} else {
				for(z = 0; z < 675; z++) asm volatile ("nop");
			}
#endif
#if 1
			// datasheet is all lies...

			// reset time -> 280us+
			// send one  -> high for 620 ns; then low for 930 ns
			// send zero -> high for 300 ns; then low for 930 ns


			if(bit) {
				for(z = 0; z < 495; z++) asm volatile ("nop");
			} else {
				for(z = 0; z < 235; z++) asm volatile ("nop");
			}

			*addr = cond_low;

			if(bit) {
				for(z = 0; z < 730; z++) asm volatile ("nop");
			} else {
				for(z = 0; z < 730; z++) asm volatile ("nop");
			}
#endif
		}
	}

	*addr = cond_low;
}

EXPORT_SYMBOL(rfnm_wsled_send_chain);

int rfnm_wsled_generic_parse(int chain_id, int led_id, int color_idx,  const char *buf, size_t count) {
	
	long long var; 
	uint32_t c;

	kstrtoll(buf, 10, &var);
	c = 0xff & var;

	// (g << 16) | (r << 8) | b

	if(color_idx == 0) {
		color_idx = 8;
	} else if(color_idx == 1) {
		color_idx = 16;
	} else if(color_idx == 2) {
		color_idx = 0;
	}

	//led_id = 3 - led_id;
	led_id = 1 + led_id;

	chain[chain_id][led_id] &= ~(0xff << color_idx);
	chain[chain_id][led_id] |= c << color_idx;

	//printk("chain %d led %d is now %x\n", chain_id, led_id, chain[chain_id][led_id]);

	return count;
}

static ssize_t chain0_led0_r_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(0, 0, 0, buf, count); }
static ssize_t chain0_led0_g_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(0, 0, 1, buf, count); }
static ssize_t chain0_led0_b_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(0, 0, 2, buf, count); }

static ssize_t chain0_led1_r_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(0, 1, 0, buf, count); }
static ssize_t chain0_led1_g_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(0, 1, 1, buf, count); }
static ssize_t chain0_led1_b_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(0, 1, 2, buf, count); }

static ssize_t chain0_led2_r_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(0, 2, 0, buf, count); }
static ssize_t chain0_led2_g_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(0, 2, 1, buf, count); }
static ssize_t chain0_led2_b_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(0, 2, 2, buf, count); }

static ssize_t chain1_led0_r_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(1, 0, 0, buf, count); }
static ssize_t chain1_led0_g_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(1, 0, 1, buf, count); }
static ssize_t chain1_led0_b_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(1, 0, 2, buf, count); }

static ssize_t chain1_led1_r_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(1, 1, 0, buf, count); }
static ssize_t chain1_led1_g_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(1, 1, 1, buf, count); }
static ssize_t chain1_led1_b_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(1, 1, 2, buf, count); }

static ssize_t chain1_led2_r_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(1, 2, 0, buf, count); }
static ssize_t chain1_led2_g_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(1, 2, 1, buf, count); }
static ssize_t chain1_led2_b_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { return rfnm_wsled_generic_parse(1, 2, 2, buf, count); }


static ssize_t chain0_apply_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { rfnm_wsled_send_chain(0); return count; }
static ssize_t chain1_apply_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { rfnm_wsled_send_chain(1); return count; }


static DEVICE_ATTR_WO(chain0_led0_r);
static DEVICE_ATTR_WO(chain0_led0_g);
static DEVICE_ATTR_WO(chain0_led0_b);

static DEVICE_ATTR_WO(chain0_led1_r);
static DEVICE_ATTR_WO(chain0_led1_g);
static DEVICE_ATTR_WO(chain0_led1_b);

static DEVICE_ATTR_WO(chain0_led2_r);
static DEVICE_ATTR_WO(chain0_led2_g);
static DEVICE_ATTR_WO(chain0_led2_b);

static DEVICE_ATTR_WO(chain1_led0_r);
static DEVICE_ATTR_WO(chain1_led0_g);
static DEVICE_ATTR_WO(chain1_led0_b);

static DEVICE_ATTR_WO(chain1_led1_r);
static DEVICE_ATTR_WO(chain1_led1_g);
static DEVICE_ATTR_WO(chain1_led1_b);

static DEVICE_ATTR_WO(chain1_led2_r);
static DEVICE_ATTR_WO(chain1_led2_g);
static DEVICE_ATTR_WO(chain1_led2_b);

static DEVICE_ATTR_WO(chain0_apply);
static DEVICE_ATTR_WO(chain1_apply);



struct gpio_desc *rfnm_wsled1_gpio;
struct gpio_desc *rfnm_wsled2_gpio;

static struct led_classdev rfnm_wsled = {
	.name = "rfnm_wsled",
};

static int rfnm_wsled_probe(struct platform_device *pdev)
{
	int error, z;

	rfnm_wsled1_gpio = devm_gpiod_get(&pdev->dev, "wsled1", GPIOD_OUT_LOW);
	if (IS_ERR(rfnm_wsled1_gpio)) {
		error = PTR_ERR(rfnm_wsled1_gpio);
		printk("Failed to get enable gpio: %d\n", error);
		return error;
	}

	rfnm_wsled2_gpio = devm_gpiod_get(&pdev->dev, "wsled2", GPIOD_OUT_LOW);
	if (IS_ERR(rfnm_wsled2_gpio)) {
		error = PTR_ERR(rfnm_wsled2_gpio);
		printk("Failed to get enable gpio: %d\n", error);
		return error;
	}

	rfnm_wsled_io = ioremap(0x30220000, SZ_4K);

	for(z = 0; z < 450000; z++) asm volatile ("nop");

	rfnm_wsled_send_chain(0);
	rfnm_wsled_send_chain(1);

	led_classdev_register(&pdev->dev, &rfnm_wsled);

	error = 0;
	error |= device_create_file(&pdev->dev, &dev_attr_chain0_led0_r);
	error |= device_create_file(&pdev->dev, &dev_attr_chain0_led0_g);
	error |= device_create_file(&pdev->dev, &dev_attr_chain0_led0_b);

	error |= device_create_file(&pdev->dev, &dev_attr_chain0_led1_r);
	error |= device_create_file(&pdev->dev, &dev_attr_chain0_led1_g);
	error |= device_create_file(&pdev->dev, &dev_attr_chain0_led1_b);

	error |= device_create_file(&pdev->dev, &dev_attr_chain0_led2_r);
	error |= device_create_file(&pdev->dev, &dev_attr_chain0_led2_g);
	error |= device_create_file(&pdev->dev, &dev_attr_chain0_led2_b);

	error |= device_create_file(&pdev->dev, &dev_attr_chain1_led0_r);
	error |= device_create_file(&pdev->dev, &dev_attr_chain1_led0_g);
	error |= device_create_file(&pdev->dev, &dev_attr_chain1_led0_b);

	error |= device_create_file(&pdev->dev, &dev_attr_chain1_led1_r);
	error |= device_create_file(&pdev->dev, &dev_attr_chain1_led1_g);
	error |= device_create_file(&pdev->dev, &dev_attr_chain1_led1_b);

	error |= device_create_file(&pdev->dev, &dev_attr_chain1_led2_r);
	error |= device_create_file(&pdev->dev, &dev_attr_chain1_led2_g);
	error |= device_create_file(&pdev->dev, &dev_attr_chain1_led2_b);

	error |= device_create_file(&pdev->dev, &dev_attr_chain0_apply);
	error |= device_create_file(&pdev->dev, &dev_attr_chain1_apply);
	if(error) {
		printk("RFNM: WSLED failed to create all dev attributes\n");
	}

	printk("RFNM: WSLED driver");

	return 0;
}

void rfnm_wsled_set(uint8_t chain_id, uint8_t led_id, uint8_t r, uint8_t g, uint8_t b) {
	if(led_id >= MAX_LED_ID) {
		printk("wrong led_id");
		return;
	}
	if(chain_id >= MAX_CHAIN_ID) {
		printk("wrong chain_id");
		return;
	}
	chain[chain_id][led_id] = b | (r << 8) | (g << 16);
}

EXPORT_SYMBOL(rfnm_wsled_set);

static const struct of_device_id rfnm_wsled_match_table[] = {
	{ .compatible = "rfnm,wsled", },
	{}
};

MODULE_DEVICE_TABLE(of, rfnm_wsled_match_table);

static struct platform_driver rfnm_wsled_driver = {
	.probe		= rfnm_wsled_probe,
	.driver		= {
		.name		= "rfnm-wsled-led",
		.of_match_table = rfnm_wsled_match_table,
	},
};

module_platform_driver(rfnm_wsled_driver);
