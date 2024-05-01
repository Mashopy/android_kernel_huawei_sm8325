/*
 * dsp_trace_utils.h
 *
 * dsp trace
 *
 * Copyright (c) 2021-2021 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef _DSP_TRACE_UTILS_H_
#define _DSP_TRACE_UTILS_H_

#include <linux/types.h>
#include <trace/audio_trace.h>
#include <dsp/audio_cal_utils.h>
#include <dsp/q6asm-v2.h>

static inline void dsp_trace_map_cal(const char *tag,
	int32_t cal_index, struct cal_block_data *cal_block)
{
	struct mem_trace_info trace;

	if (!cal_block)
		return;

	trace.mem_bytes = cal_block->map_data.map_size;
	trace.phy_addr = cal_block->cal_data.paddr;
	trace.mmap_hdl = cal_block->map_data.q6map_handle;
	trace.tag = (char *)tag;
	trace.extra_size = sizeof(cal_index);
	trace.extra = &cal_index;
	atrace_report(NULL, MEM_TRACE_MAP, &trace, sizeof(trace));
}

static inline void dsp_trace_unmap_cal(const char *tag,
	struct cal_block_data *cal_block)
{
	struct mem_trace_info trace;

	if (!cal_block)
		return;

	trace.mem_bytes = cal_block->map_data.map_size;
	trace.phy_addr = cal_block->cal_data.paddr;
	trace.mmap_hdl = cal_block->map_data.q6map_handle;
	trace.tag = (char *)tag;
	trace.extra_size = 0;
	trace.extra = NULL;
	atrace_report(NULL, MEM_TRACE_UNMAP, &trace, sizeof(trace));
}

#define ADM_TRACE_MAP_CAL(cal_index, cal_block) \
	dsp_trace_map_cal("adm cal", cal_index, cal_block);
#define ADM_TRACE_UNMAP_CAL(cal_block) \
	dsp_trace_unmap_cal("adm cal", cal_block);
#define AFE_TRACE_MAP_CAL(cal_index, cal_block) \
	dsp_trace_map_cal("afe cal", cal_index, cal_block);
#define AFE_TRACE_UNMAP_CAL(cal_block) \
	dsp_trace_unmap_cal("afe cal", cal_block);
#define ASM_TRACE_MAP_CAL(cal_index, cal_block) \
	dsp_trace_map_cal("asm cal", cal_index, cal_block);
#define ASM_TRACE_UNMAP_CAL(cal_block) \
	dsp_trace_unmap_cal("asm cal", cal_block);

static inline void dsp_trace_map_rtac(const char *tag,
	struct rtac_cal_block_data *cal_block)
{
	struct mem_trace_info trace;

	if (!cal_block)
		return;

	trace.mem_bytes = cal_block->map_data.map_size;
	trace.phy_addr = cal_block->cal_data.paddr;
	trace.mmap_hdl = cal_block->map_data.map_handle;
	trace.tag = (char *)tag;
	trace.extra_size = 0;
	trace.extra = NULL;
	atrace_report(NULL, MEM_TRACE_MAP, &trace, sizeof(trace));
}

static inline void dsp_trace_unmap(const char *tag, int32_t mmap_hdl)
{
	struct mem_trace_info trace;

	trace.mem_bytes = 0;
	trace.phy_addr = 0;
	trace.mmap_hdl = mmap_hdl;
	trace.tag = (char *)tag;
	trace.extra_size = 0;
	trace.extra = NULL;
	atrace_report(NULL, MEM_TRACE_UNMAP, &trace, sizeof(trace));
}

static inline void dsp_trace_map(const char *tag, uint32_t cmd,
	phys_addr_t phy_addr, int32_t mmap_hdl, size_t mem_bytes)
{
	struct mem_trace_info trace;

	trace.mem_bytes = mem_bytes;
	trace.phy_addr = phy_addr;
	trace.mmap_hdl = mmap_hdl;
	trace.tag = (char *)tag;
	trace.extra_size = 0;
	trace.extra = NULL;
	atrace_report(NULL, cmd, &trace, sizeof(trace));
}

#define ADM_TRACE_MAP(phy_addr, mmap_hdl, mem_bytes) \
	dsp_trace_map("adm port", MEM_TRACE_MAP, phy_addr, \
		mmap_hdl, mem_bytes);
#define ADM_TRACE_UNMAP(phy_addr, mmap_hdl, mem_bytes) \
	dsp_trace_map("adm port", MEM_TRACE_UNMAP, phy_addr, \
		mmap_hdl, mem_bytes);
#define ADM_TRACE_ST_MAP(phy_addr, mmap_hdl, mem_bytes) \
	dsp_trace_map("adm source tracking", MEM_TRACE_MAP, phy_addr, \
		mmap_hdl, mem_bytes);
#define ADM_TRACE_ST_UNMAP(phy_addr, mmap_hdl, mem_bytes) \
	dsp_trace_map("adm source tracking", MEM_TRACE_UNMAP, phy_addr, \
		mmap_hdl, mem_bytes);

#define ASM_TRACE_AIO_MAP(phy_addr, mmap_hdl, mem_bytes) \
	dsp_trace_map("asm aio", MEM_TRACE_MAP, phy_addr, \
		mmap_hdl, mem_bytes);
#define ASM_TRACE_AIO_UNMAP(phy_addr, mmap_hdl, mem_bytes) \
	dsp_trace_map("asm aio", MEM_TRACE_UNMAP, phy_addr, \
		mmap_hdl, mem_bytes);

static inline void dsp_trace_audio_client(const char *tag, uint32_t cmd,
	struct audio_client *ac, int dir)
{
	struct mem_trace_info trace;
	struct asm_buffer_node *buf_node = NULL;
	struct list_head *ptr = NULL;
	struct list_head *next = NULL;

	if (!ac || !ac->mmap_apr)
		return;

	trace.tag = (char *)tag;
	trace.phy_addr = ac->port[dir].buf->phys;
	trace.mem_bytes  = ac->port[dir].buf->size;
	trace.extra_size = 0;
	trace.extra = NULL;
	list_for_each_safe(ptr, next, &ac->port[dir].mem_map_handle) {
		buf_node = list_entry(ptr, struct asm_buffer_node, list);
		if (buf_node->buf_phys_addr == trace.phy_addr) {
			trace.mmap_hdl = buf_node->mmap_hdl;
			atrace_report(NULL, cmd, &trace, sizeof(trace));
			break;
		}
	}
}

#define ASM_TRACE_CLIENT_BUF_MAP(ac, dir) \
	dsp_trace_audio_client("asm client buf", MEM_TRACE_MAP, ac, dir)
#define ASM_TRACE_CLIENT_BUF_UNMAP(ac, dir) \
	dsp_trace_audio_client("asm client buf", MEM_TRACE_UNMAP, ac, dir)
#define ASM_TRACE_CLIENT_CBUF_MAP(ac, dir) \
	dsp_trace_audio_client("asm client contiguous buf", MEM_TRACE_MAP, \
		ac, dir)
#define ASM_TRACE_CLIENT_CBUF_UNMAP(ac, dir) \
	dsp_trace_audio_client("asm client contiguous buf", MEM_TRACE_UNMAP, \
		ac, dir)

#endif /* _DSP_TRACE_UTILS_H_ */
