/* Copyright 2015 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the above-listed copyright holders nor the
 * names of any contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/compat.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>

/* MC and IOP character device to read from RAM */

#define MC_BASE_ADDR 0x83e0000000

#define MC_BUFFER_OFFSET 0x01000000
#define MC_BUFFER_SIZE (1024*1024*3)

#define AIOP_BUFFER_OFFSET  0x06000000
#define AIOP_BUFFER_SIZE (1024*1024*16)

#define invalidate(p) { asm volatile("dc ivac, %0" : : "r" (p) : "memory"); }

struct console_data {
	char *start_addr; /* Start of buffer */
	char *end_addr; /* End of buffer */
	char *end_of_data; /* Current end of data */
	char *last_to_console; /* Last data sent to console */
};

static void adjust_end(struct console_data *cd);

static int fsl_ls2_generic_console_open(struct inode *node, struct file *fp,
					u64 offset, u64 size)
{
	struct console_data *cd;

	cd = kmalloc(sizeof(*cd), GFP_KERNEL);
	if (cd == NULL)
		return -ENOMEM;
	fp->private_data = cd;
	cd->start_addr = ioremap(MC_BASE_ADDR + offset, size);
	cd->end_addr = cd->start_addr + size;
	if (strncmp(cd->start_addr, "START", 5) == 0) {
		/* Data has not wrapped yet */
		cd->end_of_data = cd->start_addr + 5;
		cd->last_to_console  = cd->start_addr + 4;
	} else {
		cd->end_of_data = cd->start_addr;
		cd->last_to_console  = cd->start_addr;
		adjust_end(cd);
		cd->end_of_data += 3;
		cd->last_to_console += 2;
	}
	return 0;
}

static int fsl_ls2_mc_console_open(struct inode *node, struct file *fp)
{
	return fsl_ls2_generic_console_open(node, fp, MC_BUFFER_OFFSET,
					    MC_BUFFER_SIZE);
}

static int fsl_ls2_aiop_console_open(struct inode *node, struct file *fp)
{
	return fsl_ls2_generic_console_open(node, fp, AIOP_BUFFER_OFFSET,
					    AIOP_BUFFER_SIZE);
}

static int fsl_ls2_console_close(struct inode *node, struct file *fp)
{
	struct console_data *cd = fp->private_data;

	iounmap(cd->start_addr);
	kfree(cd);
	return 0;
}

static void adjust_end(struct console_data *cd)
{
	/* Search for the END marker, but being careful of
	   wraparound */
	char last3[3] = { 0, 0, 0 };
	int i = 0;
	char *ptr = cd->end_of_data;

	invalidate(ptr);

	while (i < 3) {
		last3[i] = *ptr;
		i++;
		ptr++;
		if (ptr >= cd->end_addr)
			ptr = cd->start_addr;

		if (((u64)ptr) % 64 == 0)
			invalidate(ptr);

	}
	while (last3[0] != 'E' || last3[1] != 'N' ||
	       last3[2] != 'D') {
		last3[0] = last3[1];
		last3[1] = last3[2];
		last3[2] = *ptr;
		ptr++;
		if (ptr == cd->end_addr)
			ptr = cd->start_addr;
		if (((u64)ptr) % 64 == 0)
			invalidate(ptr);
	}
	cd->end_of_data = ptr - 3;
}

/* Read one past the end of the buffer regardless of end */
static char consume_next_char(struct console_data *cd)
{
	++cd->last_to_console;
	if (cd->last_to_console == cd->end_addr)
		cd->last_to_console = cd->start_addr;

	/* Sadly we need to invalidate all tthe time here as the data
	   may have changed as we go */
	invalidate(cd->last_to_console);

	return *(cd->last_to_console);
}

ssize_t fsl_ls2_console_read(struct file *fp, char __user *buf, size_t count,
			     loff_t *f_pos)
{
	struct console_data *cd = fp->private_data;
	size_t bytes = 0;
	char data;

	/* Check if we need to adjust the end of data addr */
	adjust_end(cd);

	while (count != bytes && ((cd->end_of_data-1) != cd->last_to_console)) {
		data = consume_next_char(cd);
		if (copy_to_user(&buf[bytes], &data, 1))
			return -EFAULT;
		++bytes;
	}
	return bytes;
}

static const struct file_operations fsl_ls2_mc_console_fops = {
	.owner          = THIS_MODULE,
	.open           = fsl_ls2_mc_console_open,
	.release        = fsl_ls2_console_close,
	.read           = fsl_ls2_console_read,
};

static struct miscdevice fsl_ls2_mc_console_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "fsl_mc_console",
	.fops = &fsl_ls2_mc_console_fops
};

static const struct file_operations fsl_ls2_aiop_console_fops = {
	.owner          = THIS_MODULE,
	.open           = fsl_ls2_aiop_console_open,
	.release        = fsl_ls2_console_close,
	.read           = fsl_ls2_console_read,
};

static struct miscdevice fsl_ls2_aiop_console_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "fsl_aiop_console",
	.fops = &fsl_ls2_aiop_console_fops
};

static int __init fsl_ls2_console_init(void)
{
	int err = 0;

	pr_info("Freescale LS2 console driver\n");
	err = misc_register(&fsl_ls2_mc_console_dev);
	if (err) {
		pr_err("fsl_mc_console: cannot register device\n");
		return err;
	}
	pr_info("fsl-ls2-console: device %s registered\n",
		fsl_ls2_mc_console_dev.name);

	err = misc_register(&fsl_ls2_aiop_console_dev);
	if (err) {
		pr_err("fsl_aiop_console: cannot register device\n");
		return err;
	}
	pr_info("fsl-ls2-console: device %s registered\n",
		fsl_ls2_aiop_console_dev.name);

	return 0;
}

static void __exit fsl_ls2_console_exit(void)
{
	int err = misc_deregister(&fsl_ls2_mc_console_dev);
	if (err)
		pr_err("Failed to deregister device %s code %d\n",
		       fsl_ls2_mc_console_dev.name, err);
	else
		pr_info("device %s deregistered\n",
			fsl_ls2_mc_console_dev.name);

	err = misc_deregister(&fsl_ls2_aiop_console_dev);
	if (err)
		pr_err("Failed to deregister device %s code %d\n",
		       fsl_ls2_aiop_console_dev.name, err);
	else
		pr_info("device %s deregistered\n",
			fsl_ls2_aiop_console_dev.name);
}

module_init(fsl_ls2_console_init);
module_exit(fsl_ls2_console_exit);

MODULE_AUTHOR("Roy Pledge <roy.pledge@freescale.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Freescale LS2 console driver");
