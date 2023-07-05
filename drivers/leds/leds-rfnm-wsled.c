#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>


uint32_t rfnm_wsled_chain[2][4] = {
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
		send[z] = rfnm_wsled_chain[chain_id][z];
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

struct gpio_desc *rfnm_wsled1_gpio;
struct gpio_desc *rfnm_wsled2_gpio;

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

	printk("RFNM: WSLED driver");

	return 0;
}

void rfnm_wsled_set(uint8_t chain_id, uint8_t led_id, uint8_t r, uint8_t g, uint8_t b) {
	if(led_id > 3) {
		printk("wrong led_id");
		return;
	}
	if(chain_id > 1) {
		printk("wrong chain_id");
		return;
	}
	rfnm_wsled_chain[chain_id][1 + led_id] = b | (r << 8) | (g << 16);
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
