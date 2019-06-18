/* Copyright 2015 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
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

#ifndef DPAA2_ETH_DEBUGFS_H
#define DPAA2_ETH_DEBUGFS_H

#include <linux/dcache.h>

struct dpaa2_eth_priv;

struct dpaa2_debugfs {
	struct dentry *dir;
	struct dentry *fq_stats;
	struct dentry *ch_stats;
	struct dentry *cpu_stats;
	struct dentry *reset_stats;
	struct dentry *reset_mc_stats;
};

#ifdef CONFIG_DEBUG_FS
void dpaa2_eth_dbg_init(void);
void dpaa2_eth_dbg_exit(void);
void dpaa2_dbg_add(struct dpaa2_eth_priv *priv);
void dpaa2_dbg_remove(struct dpaa2_eth_priv *priv);
#else
static inline void dpaa2_eth_dbg_init(void) {}
static inline void dpaa2_eth_dbg_exit(void) {}
static inline void dpaa2_dbg_add(struct dpaa2_eth_priv *priv) {}
static inline void dpaa2_dbg_remove(struct dpaa2_eth_priv *priv) {}
#endif /* CONFIG_DEBUG_FS */

#endif /* DPAA2_ETH_DEBUGFS_H */
