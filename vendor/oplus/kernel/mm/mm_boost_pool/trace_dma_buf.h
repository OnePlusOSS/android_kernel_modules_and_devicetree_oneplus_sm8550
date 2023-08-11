/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) oplus add dmabuf tracepoint
 */

#if !defined(_TRACE_DMA_BUF_ALLOC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_DMA_BUF_ALLOC_H

#undef TRACE_SYSTEM
#define TRACE_SYSTEM dma_buf_alloc

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(dma_buf_alloc,

	TP_PROTO(size_t len,
		 unsigned long flags),

	TP_ARGS(len, flags),

	TP_STRUCT__entry(
		__field(size_t,	len)
		__field(unsigned long, flags)
	),

	TP_fast_assign(
		__entry->len = len;
		__entry->flags = flags;
	),

	TP_printk("len=%zu flags=0x%x",
		  __entry->len,
		  __entry->flags)
);

DEFINE_EVENT(dma_buf_alloc, dma_buf_alloc_start,

	TP_PROTO(size_t len,
		 unsigned long flags),

	TP_ARGS(len, flags)
);

DEFINE_EVENT(dma_buf_alloc, dma_buf_alloc_end,

	TP_PROTO(size_t len,
		 unsigned long flags),

	TP_ARGS(len, flags)
);

#endif /* _TRACE_DMA_BUF_ALLOC_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../../mm/oplus_mm/mm_boost_pool/

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace_dma_buf

/* This part must be outside protection */
#include <trace/define_trace.h>
