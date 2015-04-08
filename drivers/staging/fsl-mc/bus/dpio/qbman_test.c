/* Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/module.h>

#include "qbman_private.h"
#include "fsl_qbman_portal.h"
#include "qbman_debug.h"
#include "../../include/fsl_dpaa2_fd.h"

#define QBMAN_SWP_CENA_BASE 0x818000000
#define QBMAN_SWP_CINH_BASE 0x81c000000

#define QBMAN_PORTAL_IDX 2
#define QBMAN_TEST_FQID 19
#define QBMAN_TEST_BPID 23
#define QBMAN_USE_QD
#ifdef QBMAN_USE_QD
#define QBMAN_TEST_QDID 1
#endif
#define QBMAN_TEST_LFQID 0xf00010

#define NUM_EQ_FRAME 10
#define NUM_DQ_FRAME 10
#define NUM_DQ_IN_DQRR 5
#define NUM_DQ_IN_MEM   (NUM_DQ_FRAME - NUM_DQ_IN_DQRR)

static struct qbman_swp *swp;
static struct qbman_eq_desc eqdesc;
static struct qbman_pull_desc pulldesc;
static struct qbman_release_desc releasedesc;
static struct qbman_eq_response eq_storage[1];
static struct dpaa2_dq dq_storage[NUM_DQ_IN_MEM] __aligned(64);
static dma_addr_t eq_storage_phys;
static dma_addr_t dq_storage_phys;

/* FQ ctx attribute values for the test code. */
#define FQCTX_HI 0xabbaf00d
#define FQCTX_LO 0x98765432
#define FQ_VFQID 0x123456

/* Sample frame descriptor */
static struct qbman_fd_simple fd = {
	.addr_lo = 0xbabaf33d,
	.addr_hi = 0x01234567,
	.len = 0x7777,
	.frc = 0xdeadbeef,
	.flc_lo = 0xcafecafe,
	.flc_hi = 0xbeadabba
};

static void fd_inc(struct qbman_fd_simple *_fd)
{
	_fd->addr_lo += _fd->len;
	_fd->flc_lo += 0x100;
	_fd->frc += 0x10;
}

static int fd_cmp(struct qbman_fd *fda, struct qbman_fd *fdb)
{
	int i;

	for (i = 0; i < 8; i++)
		if (fda->words[i] - fdb->words[i])
			return 1;
	return 0;
}

struct qbman_fd fd_eq[NUM_EQ_FRAME];
struct qbman_fd fd_dq[NUM_DQ_FRAME];

/* "Buffers" to be released (and storage for buffers to be acquired) */
static uint64_t rbufs[320];
static uint64_t abufs[320];

static void do_enqueue(struct qbman_swp *swp)
{
	int i, j, ret;

#ifdef QBMAN_USE_QD
	pr_info("*****QBMan_test: Enqueue %d frames to QD %d\n",
					NUM_EQ_FRAME, QBMAN_TEST_QDID);
#else
	pr_info("*****QBMan_test: Enqueue %d frames to FQ %d\n",
					NUM_EQ_FRAME, QBMAN_TEST_FQID);
#endif
	for (i = 0; i < NUM_EQ_FRAME; i++) {
		/*********************************/
		/* Prepare a enqueue descriptor */
		/*********************************/
		memset(eq_storage, 0, sizeof(eq_storage));
		eq_storage_phys = virt_to_phys(eq_storage);
		qbman_eq_desc_clear(&eqdesc);
		qbman_eq_desc_set_no_orp(&eqdesc, 0);
		qbman_eq_desc_set_response(&eqdesc, eq_storage_phys, 0);
		qbman_eq_desc_set_token(&eqdesc, 0x99);
#ifdef QBMAN_USE_QD
		/**********************************/
		/* Prepare a Queueing Destination */
		/**********************************/
		qbman_eq_desc_set_qd(&eqdesc, QBMAN_TEST_QDID, 0, 3);
#else
		qbman_eq_desc_set_fq(&eqdesc, QBMAN_TEST_FQID);
#endif

		/******************/
		/* Try an enqueue */
		/******************/
		ret = qbman_swp_enqueue(swp, &eqdesc,
					(const struct qbman_fd *)&fd);
		BUG_ON(ret);
		for (j = 0; j < 8; j++)
			fd_eq[i].words[j] = *((uint32_t *)&fd + j);
		fd_inc(&fd);
	}
}

static void do_push_dequeue(struct qbman_swp *swp)
{
	int i, j;
	const struct dpaa2_dq *dq_storage1;
	const struct qbman_fd *__fd;
	int loopvar;

	pr_info("*****QBMan_test: Start push dequeue\n");
	for (i = 0; i < NUM_DQ_FRAME; i++) {
		DBG_POLL_START(loopvar);
		do {
			DBG_POLL_CHECK(loopvar);
			dq_storage1 = qbman_swp_dqrr_next(swp);
		} while (!dq_storage1);
		if (dq_storage1) {
			__fd = (const struct qbman_fd *)
					dpaa2_dq_fd(dq_storage1);
			for (j = 0; j < 8; j++)
				fd_dq[i].words[j] = __fd->words[j];
			if (fd_cmp(&fd_eq[i], &fd_dq[i])) {
				pr_info("enqueue FD is\n");
				hexdump(&fd_eq[i], 32);
				pr_info("dequeue FD is\n");
				hexdump(&fd_dq[i], 32);
			}
			qbman_swp_dqrr_consume(swp, dq_storage1);
		} else {
			pr_info("The push dequeue fails\n");
		}
	}
}

static void do_pull_dequeue(struct qbman_swp *swp)
{
	int i, j, ret;
	const struct dpaa2_dq *dq_storage1;
	const struct qbman_fd *__fd;
	int loopvar;

	pr_info("*****QBMan_test: Dequeue %d frames with dq entry in DQRR\n",
							NUM_DQ_IN_DQRR);
	for (i = 0; i < NUM_DQ_IN_DQRR; i++) {
		qbman_pull_desc_clear(&pulldesc);
		qbman_pull_desc_set_storage(&pulldesc, NULL, 0, 0);
		qbman_pull_desc_set_numframes(&pulldesc, 1);
		qbman_pull_desc_set_fq(&pulldesc, QBMAN_TEST_FQID);

		ret = qbman_swp_pull(swp, &pulldesc);
		BUG_ON(ret);
		DBG_POLL_START(loopvar);
		do {
			DBG_POLL_CHECK(loopvar);
			dq_storage1 = qbman_swp_dqrr_next(swp);
		} while (!dq_storage1);

		if (dq_storage1) {
			__fd = (const struct qbman_fd *)
					dpaa2_dq_fd(dq_storage1);
			for (j = 0; j < 8; j++)
				fd_dq[i].words[j] = __fd->words[j];
			if (fd_cmp(&fd_eq[i], &fd_dq[i])) {
				pr_info("enqueue FD is\n");
				hexdump(&fd_eq[i], 32);
				pr_info("dequeue FD is\n");
				hexdump(&fd_dq[i], 32);
			}
			qbman_swp_dqrr_consume(swp, dq_storage1);
		} else {
			pr_info("Dequeue with dq entry in DQRR fails\n");
		}
	}

	pr_info("*****QBMan_test: Dequeue %d frames with dq entry in memory\n",
								NUM_DQ_IN_MEM);
	for (i = 0; i < NUM_DQ_IN_MEM; i++) {
		dq_storage_phys = virt_to_phys(&dq_storage[i]);
		qbman_pull_desc_clear(&pulldesc);
		qbman_pull_desc_set_storage(&pulldesc, &dq_storage[i],
						dq_storage_phys, 1);
		qbman_pull_desc_set_numframes(&pulldesc, 1);
		qbman_pull_desc_set_fq(&pulldesc, QBMAN_TEST_FQID);
		ret = qbman_swp_pull(swp, &pulldesc);
		BUG_ON(ret);

		DBG_POLL_START(loopvar);
		do {
			DBG_POLL_CHECK(loopvar);
			ret = qbman_result_has_new_result(swp,
							    &dq_storage[i]);
		} while (!ret);

		if (ret) {
			for (j = 0; j < 8; j++)
				fd_dq[i + NUM_DQ_IN_DQRR].words[j] =
				dq_storage[i].dont_manipulate_directly[j + 8];
			j = i + NUM_DQ_IN_DQRR;
			if (fd_cmp(&fd_eq[j], &fd_dq[j])) {
				pr_info("enqueue FD is\n");
				hexdump(&fd_eq[i + NUM_DQ_IN_DQRR], 32);
				pr_info("dequeue FD is\n");
				hexdump(&fd_dq[i + NUM_DQ_IN_DQRR], 32);
				hexdump(&dq_storage[i], 64);
			}
		} else {
			pr_info("Dequeue with dq entry in memory fails\n");
		}
	}
}

static void release_buffer(struct qbman_swp *swp, unsigned int num)
{
	int ret;
	unsigned int i, j;

	qbman_release_desc_clear(&releasedesc);
	qbman_release_desc_set_bpid(&releasedesc, QBMAN_TEST_BPID);
	pr_info("*****QBMan_test: Release %d buffers to BP %d\n",
					num, QBMAN_TEST_BPID);
	for (i = 0; i < (num / 7 + 1); i++) {
		j = ((num - i * 7) > 7) ? 7 : (num - i * 7);
		ret = qbman_swp_release(swp, &releasedesc, &rbufs[i * 7], j);
		BUG_ON(ret);
	}
}

static void acquire_buffer(struct qbman_swp *swp, unsigned int num)
{
	int ret;
	unsigned int i, j;

	pr_info("*****QBMan_test: Acquire %d buffers from BP %d\n",
					num, QBMAN_TEST_BPID);

	for (i = 0; i < (num / 7 + 1); i++) {
		j = ((num - i * 7) > 7) ? 7 : (num - i * 7);
		ret = qbman_swp_acquire(swp, QBMAN_TEST_BPID, &abufs[i * 7], j);
		BUG_ON(ret != j);
	}
}

static void buffer_pool_test(struct qbman_swp *swp)
{
	struct qbman_attr info;
	struct dpaa2_dq *bpscn_message;
	dma_addr_t bpscn_phys;
	uint64_t bpscn_ctx;
	uint64_t ctx = 0xbbccddaadeadbeefull;
	int i, ret;
	uint32_t hw_targ;

	pr_info("*****QBMan_test: test buffer pool management\n");
	ret = qbman_bp_query(swp, QBMAN_TEST_BPID, &info);
	qbman_bp_attr_get_bpscn_addr(&info, &bpscn_phys);
	pr_info("The bpscn is %llx, info_phys is %llx\n", bpscn_phys,
			virt_to_phys(&info));
	bpscn_message = phys_to_virt(bpscn_phys);

	for (i = 0; i < 320; i++)
		rbufs[i] = 0xf00dabba01234567ull + i * 0x40;

	release_buffer(swp, 320);

	pr_info("QBMan_test: query the buffer pool\n");
	qbman_bp_query(swp, QBMAN_TEST_BPID, &info);
	hexdump(&info, 64);
	qbman_bp_attr_get_hw_targ(&info, &hw_targ);
	pr_info("hw_targ is %d\n", hw_targ);

	/* Acquire buffers to trigger BPSCN */
	acquire_buffer(swp, 300);
	/* BPSCN should be written to the memory */
	qbman_bp_query(swp, QBMAN_TEST_BPID, &info);
	hexdump(&info, 64);
	hexdump(bpscn_message, 64);
	BUG_ON(!qbman_result_is_BPSCN(bpscn_message));
	/* There should be free buffers in the pool */
	BUG_ON(!(qbman_result_bpscn_has_free_bufs(bpscn_message)));
	/* Buffer pool is depleted */
	BUG_ON(!qbman_result_bpscn_is_depleted(bpscn_message));
	/* The ctx should match */
	bpscn_ctx = qbman_result_bpscn_ctx(bpscn_message);
	pr_info("BPSCN test: ctx %llx, bpscn_ctx %llx\n", ctx, bpscn_ctx);
	BUG_ON(ctx != bpscn_ctx);
	memset(bpscn_message, 0, sizeof(struct dpaa2_dq));

	/* Re-seed the buffer pool to trigger BPSCN */
	release_buffer(swp, 240);
	/* BPSCN should be written to the memory */
	BUG_ON(!qbman_result_is_BPSCN(bpscn_message));
	/* There should be free buffers in the pool */
	BUG_ON(!(qbman_result_bpscn_has_free_bufs(bpscn_message)));
	/* Buffer pool is not depleted */
	BUG_ON(qbman_result_bpscn_is_depleted(bpscn_message));
	memset(bpscn_message, 0, sizeof(struct dpaa2_dq));

	acquire_buffer(swp, 260);
	/* BPSCN should be written to the memory */
	BUG_ON(!qbman_result_is_BPSCN(bpscn_message));
	/* There should be free buffers in the pool while BPSCN generated */
	BUG_ON(!(qbman_result_bpscn_has_free_bufs(bpscn_message)));
	/* Buffer pool is depletion */
	BUG_ON(!qbman_result_bpscn_is_depleted(bpscn_message));
}

static void ceetm_test(struct qbman_swp *swp)
{
	int i, j, ret;

	qbman_eq_desc_clear(&eqdesc);
	qbman_eq_desc_set_no_orp(&eqdesc, 0);
	qbman_eq_desc_set_fq(&eqdesc, QBMAN_TEST_LFQID);
	pr_info("*****QBMan_test: Enqueue to LFQID %x\n",
						QBMAN_TEST_LFQID);
	for (i = 0; i < NUM_EQ_FRAME; i++) {
		ret = qbman_swp_enqueue(swp, &eqdesc,
					(const struct qbman_fd *)&fd);
		BUG_ON(ret);
		for (j = 0; j < 8; j++)
			fd_eq[i].words[j] = *((uint32_t *)&fd + j);
		fd_inc(&fd);
	}
}

int qbman_test(void)
{
	struct qbman_swp_desc pd;
	uint32_t reg;

	pd.cena_bar = ioremap_cache_ns(QBMAN_SWP_CENA_BASE +
				QBMAN_PORTAL_IDX * 0x10000, 0x10000);
	pd.cinh_bar = ioremap(QBMAN_SWP_CINH_BASE +
				QBMAN_PORTAL_IDX * 0x10000, 0x10000);

	/* Detect whether the mc image is the test image with GPP setup */
	reg = readl_relaxed(pd.cena_bar + 0x4);
	if (reg != 0xdeadbeef) {
		pr_err("The MC image doesn't have GPP test setup, stop!\n");
		iounmap(pd.cena_bar);
		iounmap(pd.cinh_bar);
		return -1;
	}

	pr_info("*****QBMan_test: Init QBMan SWP %d\n", QBMAN_PORTAL_IDX);
	swp = qbman_swp_init(&pd);
	if (!swp) {
		iounmap(pd.cena_bar);
		iounmap(pd.cinh_bar);
		return -1;
	}

	/*******************/
	/* Enqueue frames  */
	/*******************/
	do_enqueue(swp);

	/*******************/
	/* Do pull dequeue */
	/*******************/
	do_pull_dequeue(swp);

	/*******************/
	/* Enqueue frames  */
	/*******************/
	qbman_swp_push_set(swp, 0, 1);
	qbman_swp_fq_schedule(swp, QBMAN_TEST_FQID);
	do_enqueue(swp);

	/*******************/
	/* Do push dequeue */
	/*******************/
	do_push_dequeue(swp);

	/**************************/
	/* Test buffer pool funcs */
	/**************************/
	buffer_pool_test(swp);

	/******************/
	/* CEETM test     */
	/******************/
	ceetm_test(swp);

	qbman_swp_finish(swp);
	pr_info("*****QBMan_test: Kernel test Passed\n");
	return 0;
}

/* user-space test-case, definitions:
 *
 * 1 portal only, using portal index 3.
 */

#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/mman.h>

#define QBMAN_TEST_US_SWP 3 /* portal index for user space */

#define QBMAN_TEST_MAGIC 'q'
struct qbman_test_swp_ioctl {
	unsigned long portal1_cinh;
	unsigned long portal1_cena;
};
struct qbman_test_dma_ioctl {
	unsigned long ptr;
	uint64_t phys_addr;
};

struct qbman_test_priv {
	int has_swp_map;
	int has_dma_map;
	unsigned long pgoff;
};

#define QBMAN_TEST_SWP_MAP \
	_IOR(QBMAN_TEST_MAGIC, 0x01, struct qbman_test_swp_ioctl)
#define QBMAN_TEST_SWP_UNMAP \
	_IOR(QBMAN_TEST_MAGIC, 0x02, struct qbman_test_swp_ioctl)
#define QBMAN_TEST_DMA_MAP \
	_IOR(QBMAN_TEST_MAGIC, 0x03, struct qbman_test_dma_ioctl)
#define QBMAN_TEST_DMA_UNMAP \
	_IOR(QBMAN_TEST_MAGIC, 0x04, struct qbman_test_dma_ioctl)

#define TEST_PORTAL1_CENA_PGOFF ((QBMAN_SWP_CENA_BASE + QBMAN_TEST_US_SWP * \
						0x10000) >> PAGE_SHIFT)
#define TEST_PORTAL1_CINH_PGOFF ((QBMAN_SWP_CINH_BASE + QBMAN_TEST_US_SWP * \
						0x10000) >> PAGE_SHIFT)

static int qbman_test_open(struct inode *inode, struct file *filp)
{
	struct qbman_test_priv *priv;

	priv = kmalloc(sizeof(struct qbman_test_priv), GFP_KERNEL);
	if (!priv)
		return -EIO;
	filp->private_data = priv;
	priv->has_swp_map = 0;
	priv->has_dma_map = 0;
	priv->pgoff = 0;
	return 0;
}

static int qbman_test_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	struct qbman_test_priv *priv = filp->private_data;

	BUG_ON(!priv);

	if (vma->vm_pgoff == TEST_PORTAL1_CINH_PGOFF)
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	else if (vma->vm_pgoff == TEST_PORTAL1_CENA_PGOFF)
		vma->vm_page_prot = pgprot_cached_ns(vma->vm_page_prot);
	else if (vma->vm_pgoff == priv->pgoff)
		vma->vm_page_prot = pgprot_cached(vma->vm_page_prot);
	else {
		pr_err("Damn, unrecognised pg_off!!\n");
		return -EINVAL;
	}
	ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				      vma->vm_end - vma->vm_start,
				      vma->vm_page_prot);
	return ret;
}

static long qbman_test_ioctl(struct file *fp, unsigned int cmd,
				unsigned long arg)
{
	void __user *a = (void __user *)arg;
	unsigned long longret, populate;
	int ret = 0;
	struct qbman_test_priv *priv = fp->private_data;

	BUG_ON(!priv);

	switch (cmd) {
	case QBMAN_TEST_SWP_MAP:
	{
		struct qbman_test_swp_ioctl params;

		if (priv->has_swp_map)
			return -EINVAL;
		down_write(&current->mm->mmap_sem);
		/* Map portal1 CINH */
		longret = do_mmap_pgoff(fp, PAGE_SIZE, 0x10000,
				PROT_READ | PROT_WRITE, MAP_SHARED,
				TEST_PORTAL1_CINH_PGOFF, &populate);
		if (longret & ~PAGE_MASK) {
			ret = (int)longret;
			goto out;
		}
		params.portal1_cinh = longret;
		/* Map portal1 CENA */
		longret = do_mmap_pgoff(fp, PAGE_SIZE, 0x10000,
				PROT_READ | PROT_WRITE, MAP_SHARED,
				TEST_PORTAL1_CENA_PGOFF, &populate);
		if (longret & ~PAGE_MASK) {
			ret = (int)longret;
			goto out;
		}
		params.portal1_cena = longret;
		priv->has_swp_map = 1;
out:
		up_write(&current->mm->mmap_sem);
		if (!ret && copy_to_user(a, &params, sizeof(params)))
			return -EFAULT;
		return ret;
	}
	case QBMAN_TEST_SWP_UNMAP:
	{
		struct qbman_test_swp_ioctl params;

		if (!priv->has_swp_map)
			return -EINVAL;

		if (copy_from_user(&params, a, sizeof(params)))
			return -EFAULT;
		down_write(&current->mm->mmap_sem);
		do_munmap(current->mm, params.portal1_cena, 0x10000);
		do_munmap(current->mm, params.portal1_cinh, 0x10000);
		up_write(&current->mm->mmap_sem);
		priv->has_swp_map = 0;
		return 0;
	}
	case QBMAN_TEST_DMA_MAP:
	{
		struct qbman_test_dma_ioctl params;
		void *vaddr;

		if (priv->has_dma_map)
			return -EINVAL;
		vaddr = (void *)get_zeroed_page(GFP_KERNEL);
		params.phys_addr = virt_to_phys(vaddr);
		priv->pgoff = (unsigned long)params.phys_addr >> PAGE_SHIFT;
		down_write(&current->mm->mmap_sem);
		longret = do_mmap_pgoff(fp, PAGE_SIZE, PAGE_SIZE,
				PROT_READ | PROT_WRITE, MAP_SHARED,
				priv->pgoff, &populate);
		if (longret & ~PAGE_MASK) {
			ret = (int)longret;
			return ret;
		}
		params.ptr = longret;
		priv->has_dma_map = 1;
		up_write(&current->mm->mmap_sem);
		if (copy_to_user(a, &params, sizeof(params)))
			return -EFAULT;
		return 0;
	}
	case QBMAN_TEST_DMA_UNMAP:
	{
		struct qbman_test_dma_ioctl params;

		if (!priv->has_dma_map)
			return -EINVAL;
		if (copy_from_user(&params, a, sizeof(params)))
			return -EFAULT;
		down_write(&current->mm->mmap_sem);
		do_munmap(current->mm, params.ptr, PAGE_SIZE);
		up_write(&current->mm->mmap_sem);
		free_page((unsigned long)phys_to_virt(params.phys_addr));
		priv->has_dma_map = 0;
		return 0;
	}
	default:
		pr_err("Bad ioctl cmd!\n");
	}
	return -EINVAL;
}

static const struct file_operations qbman_fops = {
	.open		   = qbman_test_open,
	.mmap		   = qbman_test_mmap,
	.unlocked_ioctl	   = qbman_test_ioctl
};

static struct miscdevice qbman_miscdev = {
	.name = "qbman-test",
	.fops = &qbman_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

static int qbman_miscdev_init;

static int test_init(void)
{
	int ret = qbman_test();

	if (!ret) {
		/* MC image supports the test cases, so instantiate the
		 * character devic that the user-space test case will use to do
		 * its memory mappings. */
		ret = misc_register(&qbman_miscdev);
		if (ret) {
			pr_err("qbman-test: failed to register misc device\n");
			return ret;
		}
		pr_info("qbman-test: misc device registered!\n");
		qbman_miscdev_init = 1;
	}
	return 0;
}

static void test_exit(void)
{
	if (qbman_miscdev_init) {
		misc_deregister(&qbman_miscdev);
		qbman_miscdev_init = 0;
	}
}

module_init(test_init);
module_exit(test_exit);
