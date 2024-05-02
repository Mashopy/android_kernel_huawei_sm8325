	.text
	.file	"mas_bkops_core.c"
	.globl	bkops_idle_work_func    // -- Begin function bkops_idle_work_func
	.p2align	2
	.type	bkops_idle_work_func,@function
bkops_idle_work_func:                   // @bkops_idle_work_func
.Lbkops_idle_work_func$local:
// %bb.0:
	sub	sp, sp, #80             // =80
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	add	x29, sp, #32            // =32
	str	x21, [sp, #48]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stur	wzr, [x29, #-12]
	mov	x19, x0
	ldur	x0, [x0, #-8]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB0_3
// %bb.1:
	ldr	x8, [x19, #136]
	str	x8, [sp, #8]
	ldr	x8, [sp, #8]
	tbz	w8, #8, .LBB0_3
// %bb.2:
	add	x8, x19, #136           // =136
	mov	w9, #256
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tbnz	w10, #8, .LBB0_19
.LBB0_3:
	ldur	x20, [x19, #-24]
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_lock
	adrp	x21, bkops_enabled
	ldrb	w8, [x21, :lo12:bkops_enabled]
	cmp	w8, #1                  // =1
	b.ne	.LBB0_5
// %bb.4:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	b	.LBB0_9
.LBB0_5:
	ldr	x8, [x20, #8]
	ldur	x0, [x19, #-16]
	sub	x1, x29, #12            // =12
	blr	x8
	adrp	x8, user_manual_gc
	ldrb	w9, [x8, :lo12:user_manual_gc]
	mov	w20, w0
	cbz	w9, .LBB0_8
// %bb.6:
	ldur	w9, [x29, #-12]
	cbnz	w9, .LBB0_8
// %bb.7:
	strb	wzr, [x8, :lo12:user_manual_gc]
.LBB0_8:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	cbnz	w20, .LBB0_21
.LBB0_9:
	ldr	x8, [x19, #136]
	ldur	x0, [x19, #-8]
	and	x8, x8, #0xfffffffffffffffd
	str	x8, [x19, #136]
	ldur	w8, [x29, #-12]
	str	w8, [x19, #208]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB0_13
// %bb.10:
	ldur	w8, [x19, #-32]
	cmp	w8, #2                  // =2
	b.ne	.LBB0_13
// %bb.11:
	ldur	w8, [x29, #-12]
	cbnz	w8, .LBB0_14
// %bb.12:
	add	x8, x19, #136           // =136
	mov	w9, #128
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
.LBB0_13:
	ldur	w8, [x29, #-12]
	cbz	w8, .LBB0_19
.LBB0_14:
	ldr	x8, [x19, #136]
	tbnz	w8, #0, .LBB0_19
// %bb.15:
	ldur	x20, [x19, #-24]
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_lock
	ldrb	w8, [x21, :lo12:bkops_enabled]
	cmp	w8, #1                  // =1
	b.ne	.LBB0_17
// %bb.16:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	b	.LBB0_18
.LBB0_17:
	ldr	x8, [x20]
	ldur	x0, [x19, #-16]
	mov	w1, #1
	blr	x8
	mov	w20, w0
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	cbnz	w20, .LBB0_22
.LBB0_18:
	add	x8, x19, #136           // =136
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 1
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB0_19:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB0_24
// %bb.20:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldr	x21, [sp, #48]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #80             // =80
	ret
.LBB0_21:
	adrp	x0, .L.str
	add	x0, x0, :lo12:.L.str
	b	.LBB0_23
.LBB0_22:
	adrp	x0, .L.str.1
	add	x0, x0, :lo12:.L.str.1
.LBB0_23:
	bl	printk
	b	.LBB0_19
.LBB0_24:
	bl	__stack_chk_fail
.Lfunc_end0:
	.size	bkops_idle_work_func, .Lfunc_end0-bkops_idle_work_func
                                        // -- End function
	.globl	bkops_io_busyidle_notify_handler // -- Begin function bkops_io_busyidle_notify_handler
	.p2align	2
	.type	bkops_io_busyidle_notify_handler,@function
bkops_io_busyidle_notify_handler:       // @bkops_io_busyidle_notify_handler
.Lbkops_io_busyidle_notify_handler$local:
// %bb.0:
	sub	sp, sp, #48             // =48
	sub	w8, w1, #2              // =2
	cmp	w8, #2                  // =2
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	add	x29, sp, #16            // =16
	b.lo	.LBB1_23
// %bb.1:
	ldr	x19, [x0, #40]
	mov	w20, w1
	cbz	w1, .LBB1_12
// %bb.2:
	cmp	w20, #1                 // =1
	b.ne	.LBB1_21
// %bb.3:
	ldr	x0, [x19, #24]
	bl	blk_queue_query_unistore_enable
	tbnz	w0, #0, .LBB1_5
// %bb.4:
	mov	x0, x19
	bl	bkops_should_query
	cbz	w0, .LBB1_24
.LBB1_5:
	ldr	x0, [x19, #24]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB1_11
// %bb.6:
	ldr	w8, [x19]
	cmp	w8, #2                  // =2
	b.ne	.LBB1_9
// %bb.7:
	add	x8, x19, #168           // =168
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 64
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x19, #168]
	tbz	w8, #7, .LBB1_23
// %bb.8:
	ldr	w8, [x19]
	cmp	w8, #2                  // =2
	b.eq	.LBB1_10
.LBB1_9:
	mov	x0, x19
	bl	bkops_should_query
	cbz	w0, .LBB1_24
.LBB1_10:
	ldr	x8, [x19, #168]
	tbnz	w8, #5, .LBB1_23
.LBB1_11:
	add	x8, x19, #168           // =168
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x11, [x8]
	orr	x11, x11, 32
	stxr	w12, x11, [x8]
	cbnz	w12, 1b
	//NO_APP
	mov	w9, #256
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x11, [x8]
	bic	x11, x11, x9
	stxr	w12, x11, [x8]
	cbnz	w12, 1b
	//NO_APP
	adrp	x10, bkops_workqueue_g
	ldr	w0, [x19, #176]
	ldr	x20, [x10, :lo12:bkops_workqueue_g]
	add	x19, x19, #32           // =32
	bl	__msecs_to_jiffies
	mov	x3, x0
	mov	w0, #8
	mov	x1, x20
	mov	x2, x19
	bl	queue_delayed_work_on
	mov	w0, #1
	b	.LBB1_24
.LBB1_12:
	ldr	x0, [x19, #24]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB1_15
// %bb.13:
	ldr	w8, [x19]
	cmp	w8, #2                  // =2
	b.ne	.LBB1_15
// %bb.14:
	add	x8, x19, #168           // =168
	mov	w9, #64
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
.LBB1_15:
	ldr	x8, [x19, #168]
	str	x8, [sp, #8]
	ldr	x8, [sp, #8]
	tbz	w8, #5, .LBB1_23
// %bb.16:
	add	x8, x19, #168           // =168
	mov	w9, #32
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tbz	w10, #5, .LBB1_23
// %bb.17:
	mov	w9, #256
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	add	x0, x19, #32            // =32
	bl	cancel_delayed_work_sync
	ldr	x8, [x19, #168]
	tbz	w8, #0, .LBB1_23
// %bb.18:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_lock
	adrp	x8, bkops_enabled
	ldrb	w8, [x8, :lo12:bkops_enabled]
	tbnz	w8, #0, .LBB1_20
// %bb.19:
	ldp	x8, x0, [x19, #8]
	mov	w1, wzr
	ldr	x8, [x8]
	blr	x8
.LBB1_20:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	add	x8, x19, #168           // =168
	mov	w9, #1
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	b	.LBB1_23
.LBB1_21:
	adrp	x0, bkops_io_busyidle_notify_handler._rs
	adrp	x1, .L__func__.bkops_io_busyidle_notify_handler
	add	x0, x0, :lo12:bkops_io_busyidle_notify_handler._rs
	add	x1, x1, :lo12:.L__func__.bkops_io_busyidle_notify_handler
	bl	___ratelimit
	cbz	w0, .LBB1_24
// %bb.22:
	adrp	x0, .L.str.2
	adrp	x1, .L__func__.bkops_io_busyidle_notify_handler
	add	x0, x0, :lo12:.L.str.2
	add	x1, x1, :lo12:.L__func__.bkops_io_busyidle_notify_handler
	mov	w2, w20
	bl	printk
.LBB1_23:
	mov	w0, wzr
.LBB1_24:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #48             // =48
	ret
.Lfunc_end1:
	.size	bkops_io_busyidle_notify_handler, .Lfunc_end1-bkops_io_busyidle_notify_handler
                                        // -- End function
	.globl	mas_bkops_work_query    // -- Begin function mas_bkops_work_query
	.p2align	2
	.type	mas_bkops_work_query,@function
mas_bkops_work_query:                   // @mas_bkops_work_query
.Lmas_bkops_work_query$local:
// %bb.0:
	sub	sp, sp, #64             // =64
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	add	x29, sp, #32            // =32
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stur	wzr, [x29, #-12]
	cbz	x0, .LBB2_8
// %bb.1:
	ldr	x0, [x0, #136]
	bl	mas_blk_get_lld
	cbz	x0, .LBB2_8
// %bb.2:
	ldrb	w8, [x0, #25]
	tbz	w8, #1, .LBB2_7
// %bb.3:
	ldr	x20, [x0, #312]
	cbz	x20, .LBB2_26
// %bb.4:
	ldr	w8, [x20]
	cmp	w8, #2                  // =2
	b.ne	.LBB2_7
// %bb.5:
	ldr	x8, [x20, #168]
	tbnz	w8, #6, .LBB2_11
// %bb.6:
	mov	w0, #4
	b	.LBB2_9
.LBB2_7:
	mov	w0, #1
	b	.LBB2_9
.LBB2_8:
	mov	w0, #2
.LBB2_9:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB2_27
// %bb.10:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #64             // =64
	ret
.LBB2_11:
	ldr	x8, [x20, #168]
	tbnz	w8, #0, .LBB2_17
// %bb.12:
	ldr	x8, [x20, #168]
	str	x8, [sp, #8]
	ldr	x8, [sp, #8]
	tbz	w8, #5, .LBB2_15
// %bb.13:
	add	x8, x20, #168           // =168
	mov	w9, #32
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tbz	w10, #5, .LBB2_15
// %bb.14:
	mov	w9, #256
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	add	x0, x20, #32            // =32
	bl	cancel_delayed_work_sync
.LBB2_15:
	ldr	x19, [x20, #8]
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_lock
	adrp	x8, bkops_enabled
	ldrb	w8, [x8, :lo12:bkops_enabled]
	cmp	w8, #1                  // =1
	b.ne	.LBB2_18
// %bb.16:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	b	.LBB2_22
.LBB2_17:
	mov	w0, #5
	b	.LBB2_9
.LBB2_18:
	ldr	x8, [x19, #8]
	ldr	x0, [x20, #16]
	sub	x1, x29, #12            // =12
	blr	x8
	adrp	x8, user_manual_gc
	ldrb	w9, [x8, :lo12:user_manual_gc]
	mov	w19, w0
	cbz	w9, .LBB2_21
// %bb.19:
	ldur	w9, [x29, #-12]
	cbnz	w9, .LBB2_21
// %bb.20:
	strb	wzr, [x8, :lo12:user_manual_gc]
.LBB2_21:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	cbnz	w19, .LBB2_25
.LBB2_22:
	ldur	w8, [x29, #-12]
	cbz	w8, .LBB2_24
// %bb.23:
	mov	w0, #8
	b	.LBB2_9
.LBB2_24:
	add	x8, x20, #168           // =168
	mov	w9, #128
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w0, #7
	b	.LBB2_9
.LBB2_25:
	adrp	x0, .L.str.3
	adrp	x1, .L__func__.mas_bkops_work_query
	add	x0, x0, :lo12:.L.str.3
	add	x1, x1, :lo12:.L__func__.mas_bkops_work_query
	mov	w2, w19
	bl	printk
	mov	w0, #6
	b	.LBB2_9
.LBB2_26:
	mov	w0, #3
	b	.LBB2_9
.LBB2_27:
	bl	__stack_chk_fail
.Lfunc_end2:
	.size	mas_bkops_work_query, .Lfunc_end2-mas_bkops_work_query
                                        // -- End function
	.globl	mas_bkops_work_start    // -- Begin function mas_bkops_work_start
	.p2align	2
	.type	mas_bkops_work_start,@function
mas_bkops_work_start:                   // @mas_bkops_work_start
.Lmas_bkops_work_start$local:
// %bb.0:
	sub	sp, sp, #80             // =80
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	add	x29, sp, #32            // =32
	str	x21, [sp, #48]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stur	wzr, [x29, #-12]
	cbz	x0, .LBB3_8
// %bb.1:
	ldr	x0, [x0, #136]
	bl	mas_blk_get_lld
	cbz	x0, .LBB3_8
// %bb.2:
	ldrb	w8, [x0, #25]
	tbz	w8, #1, .LBB3_7
// %bb.3:
	ldr	x20, [x0, #312]
	cbz	x20, .LBB3_30
// %bb.4:
	ldr	w8, [x20]
	cmp	w8, #2                  // =2
	b.ne	.LBB3_7
// %bb.5:
	ldr	x8, [x20, #168]
	tbnz	w8, #6, .LBB3_11
// %bb.6:
	mov	w0, #4
	b	.LBB3_9
.LBB3_7:
	mov	w0, #1
	b	.LBB3_9
.LBB3_8:
	mov	w0, #2
.LBB3_9:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB3_31
// %bb.10:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldr	x21, [sp, #48]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #80             // =80
	ret
.LBB3_11:
	ldr	x8, [x20, #168]
	tbnz	w8, #0, .LBB3_17
// %bb.12:
	ldr	x8, [x20, #168]
	str	x8, [sp, #8]
	ldr	x8, [sp, #8]
	tbz	w8, #5, .LBB3_15
// %bb.13:
	add	x8, x20, #168           // =168
	mov	w9, #32
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tbz	w10, #5, .LBB3_15
// %bb.14:
	mov	w9, #256
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	add	x0, x20, #32            // =32
	bl	cancel_delayed_work_sync
.LBB3_15:
	ldr	x19, [x20, #8]
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_lock
	adrp	x21, bkops_enabled
	ldrb	w8, [x21, :lo12:bkops_enabled]
	cmp	w8, #1                  // =1
	b.ne	.LBB3_18
// %bb.16:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	b	.LBB3_22
.LBB3_17:
	mov	w0, #5
	b	.LBB3_9
.LBB3_18:
	ldr	x8, [x19, #8]
	ldr	x0, [x20, #16]
	sub	x1, x29, #12            // =12
	blr	x8
	adrp	x8, user_manual_gc
	ldrb	w9, [x8, :lo12:user_manual_gc]
	mov	w19, w0
	cbz	w9, .LBB3_21
// %bb.19:
	ldur	w9, [x29, #-12]
	cbnz	w9, .LBB3_21
// %bb.20:
	strb	wzr, [x8, :lo12:user_manual_gc]
.LBB3_21:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	cbnz	w19, .LBB3_28
.LBB3_22:
	ldur	w8, [x29, #-12]
	cbz	w8, .LBB3_25
// %bb.23:
	ldr	x19, [x20, #8]
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_lock
	ldrb	w8, [x21, :lo12:bkops_enabled]
	cmp	w8, #1                  // =1
	b.ne	.LBB3_26
// %bb.24:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	b	.LBB3_27
.LBB3_25:
	add	x8, x20, #168           // =168
	mov	w9, #128
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w0, #7
	b	.LBB3_9
.LBB3_26:
	ldr	x8, [x19]
	ldr	x0, [x20, #16]
	mov	w1, #1
	blr	x8
	mov	w19, w0
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	cbnz	w19, .LBB3_29
.LBB3_27:
	add	x8, x20, #168           // =168
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 128
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 1
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w0, wzr
	b	.LBB3_9
.LBB3_28:
	adrp	x0, .L.str.3
	adrp	x1, .L__func__.mas_bkops_work_start
	add	x0, x0, :lo12:.L.str.3
	add	x1, x1, :lo12:.L__func__.mas_bkops_work_start
	mov	w2, w19
	bl	printk
	mov	w0, #6
	b	.LBB3_9
.LBB3_29:
	adrp	x0, .L.str.14
	adrp	x1, .L__func__.mas_bkops_work_start_ret
	add	x0, x0, :lo12:.L.str.14
	add	x1, x1, :lo12:.L__func__.mas_bkops_work_start_ret
	mov	w2, w19
	bl	printk
	mov	w0, #9
	b	.LBB3_9
.LBB3_30:
	mov	w0, #3
	b	.LBB3_9
.LBB3_31:
	bl	__stack_chk_fail
.Lfunc_end3:
	.size	mas_bkops_work_start, .Lfunc_end3-mas_bkops_work_start
                                        // -- End function
	.globl	mas_bkops_work_stop     // -- Begin function mas_bkops_work_stop
	.p2align	2
	.type	mas_bkops_work_stop,@function
mas_bkops_work_stop:                    // @mas_bkops_work_stop
.Lmas_bkops_work_stop$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB4_14
// %bb.1:
	ldr	x0, [x0, #136]
	bl	mas_blk_get_lld
	cbz	x0, .LBB4_14
// %bb.2:
	ldr	x19, [x0, #312]
	cbz	x19, .LBB4_14
// %bb.3:
	ldr	x8, [x0, #24]
	tbz	w8, #9, .LBB4_14
// %bb.4:
	ldr	w8, [x19]
	cmp	w8, #2                  // =2
	b.ne	.LBB4_14
// %bb.5:
	ldr	x8, [x19, #168]
	str	x8, [x29, #24]
	ldr	x8, [x29, #24]
	tbz	w8, #7, .LBB4_14
// %bb.6:
	add	x8, x19, #168           // =168
	mov	w9, #128
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tbz	w10, #7, .LBB4_14
// %bb.7:
	ldr	x8, [x19, #168]
	str	x8, [x29, #24]
	ldr	x8, [x29, #24]
	tbz	w8, #5, .LBB4_10
// %bb.8:
	add	x8, x19, #168           // =168
	mov	w9, #32
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tbz	w10, #5, .LBB4_10
// %bb.9:
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 256
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB4_10:
	ldr	x8, [x19, #168]
	tbz	w8, #0, .LBB4_14
// %bb.11:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_lock
	adrp	x8, bkops_enabled
	ldrb	w8, [x8, :lo12:bkops_enabled]
	tbnz	w8, #0, .LBB4_13
// %bb.12:
	ldp	x8, x0, [x19, #8]
	mov	w1, wzr
	ldr	x8, [x8]
	blr	x8
.LBB4_13:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	add	x8, x19, #168           // =168
	mov	w9, #1
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
.LBB4_14:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end4:
	.size	mas_bkops_work_stop, .Lfunc_end4-mas_bkops_work_stop
                                        // -- End function
	.globl	bkops_pm_callback       // -- Begin function bkops_pm_callback
	.p2align	2
	.type	bkops_pm_callback,@function
bkops_pm_callback:                      // @bkops_pm_callback
.Lbkops_pm_callback$local:
// %bb.0:
	sub	x8, x1, #1              // =1
	cmp	x8, #3                  // =3
	b.hi	.LBB5_2
// %bb.1:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	adrp	x19, bkops_lock
	add	x19, x19, :lo12:bkops_lock
	tst	x8, #0x1
	mov	x0, x19
	mov	x29, sp
	cset	w20, eq
	bl	mutex_lock
	adrp	x8, bkops_enabled
	mov	x0, x19
	strb	w20, [x8, :lo12:bkops_enabled]
	bl	mutex_unlock
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	mov	w0, #1
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB5_2:
	mov	w0, wzr
	ret
.Lfunc_end5:
	.size	bkops_pm_callback, .Lfunc_end5-bkops_pm_callback
                                        // -- End function
	.globl	bkops_notify_reboot     // -- Begin function bkops_notify_reboot
	.p2align	2
	.type	bkops_notify_reboot,@function
bkops_notify_reboot:                    // @bkops_notify_reboot
.Lbkops_notify_reboot$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	adrp	x19, bkops_lock
	add	x19, x19, :lo12:bkops_lock
	mov	x0, x19
	mov	x29, sp
	bl	mutex_lock
	adrp	x8, bkops_enabled
	mov	w9, #1
	mov	x0, x19
	strb	w9, [x8, :lo12:bkops_enabled]
	bl	mutex_unlock
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	mov	w0, wzr
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end6:
	.size	bkops_notify_reboot, .Lfunc_end6-bkops_notify_reboot
                                        // -- End function
	.globl	mas_bkops_alloc         // -- Begin function mas_bkops_alloc
	.p2align	2
	.type	mas_bkops_alloc,@function
mas_bkops_alloc:                        // @mas_bkops_alloc
.Lmas_bkops_alloc$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	adrp	x8, kmalloc_caches+72
	ldr	x0, [x8, :lo12:kmalloc_caches+72]
	mov	w1, #3520
	mov	w2, #296
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	bl	kmem_cache_alloc_trace
	mov	x19, x0
	cbz	x0, .LBB7_3
// %bb.1:
	adrp	x10, __ufs_bkops_idle_work_func
	adrp	x1, delayed_work_timer_fn
	mov	x8, #68719476704
	add	x9, x19, #40            // =40
	add	x10, x10, :lo12:__ufs_bkops_idle_work_func
	add	x0, x19, #80            // =80
	add	x1, x1, :lo12:delayed_work_timer_fn
	mov	w2, #2097152
	mov	x3, xzr
	mov	x4, xzr
	str	x8, [x19, #32]
	str	x9, [x19, #40]
	stp	x9, x10, [x19, #48]
	bl	init_timer_key
	mov	w8, #1000
	mov	w9, #3600
	mov	w10, #536870912
	str	x8, [x19, #176]
	str	x9, [x19, #248]
	str	x10, [x19, #264]
	str	x10, [x19, #280]
.LBB7_2:
	mov	x0, x19
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB7_3:
	adrp	x0, .L.str.4
	add	x0, x0, :lo12:.L.str.4
	bl	printk
	b	.LBB7_2
.Lfunc_end7:
	.size	mas_bkops_alloc, .Lfunc_end7-mas_bkops_alloc
                                        // -- End function
	.globl	mas_bkops_manual_gc_proc_show // -- Begin function mas_bkops_manual_gc_proc_show
	.p2align	2
	.type	mas_bkops_manual_gc_proc_show,@function
mas_bkops_manual_gc_proc_show:          // @mas_bkops_manual_gc_proc_show
.Lmas_bkops_manual_gc_proc_show$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x8, user_manual_gc
	ldrb	w2, [x8, :lo12:user_manual_gc]
	adrp	x1, .L.str.5
	add	x1, x1, :lo12:.L.str.5
	mov	x29, sp
	bl	seq_printf
	mov	w0, wzr
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end8:
	.size	mas_bkops_manual_gc_proc_show, .Lfunc_end8-mas_bkops_manual_gc_proc_show
                                        // -- End function
	.globl	mas_bkops_manual_gc_proc_open // -- Begin function mas_bkops_manual_gc_proc_open
	.p2align	2
	.type	mas_bkops_manual_gc_proc_open,@function
mas_bkops_manual_gc_proc_open:          // @mas_bkops_manual_gc_proc_open
.Lmas_bkops_manual_gc_proc_open$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x0, x1
	adrp	x1, __cfi_mas_bkops_manual_gc_proc_show
	add	x1, x1, :lo12:__cfi_mas_bkops_manual_gc_proc_show
	mov	x2, xzr
	mov	x29, sp
	bl	single_open
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end9:
	.size	mas_bkops_manual_gc_proc_open, .Lfunc_end9-mas_bkops_manual_gc_proc_open
                                        // -- End function
	.globl	mas_bkops_manual_gc_proc_write // -- Begin function mas_bkops_manual_gc_proc_write
	.p2align	2
	.type	mas_bkops_manual_gc_proc_write,@function
mas_bkops_manual_gc_proc_write:         // @mas_bkops_manual_gc_proc_write
.Lmas_bkops_manual_gc_proc_write$local:
// %bb.0:
	sub	sp, sp, #64             // =64
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	mov	x0, #-22
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	str	x21, [sp, #32]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	add	x29, sp, #16            // =16
	str	x8, [sp, #8]
	strb	wzr, [sp, #4]
	cbz	x1, .LBB10_19
// %bb.1:
	mov	x19, x2
	cbz	x2, .LBB10_19
// %bb.2:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w11, [x8, #62]
	ldr	x10, [x8, #8]
	lsl	x9, x1, #8
	tbnz	w11, #5, .LBB10_4
// %bb.3:
	ldr	x12, [x8]
	mov	x11, x1
	tbz	w12, #26, .LBB10_5
.LBB10_4:
	and	x11, x1, x9, asr #8
.LBB10_5:
	//APP
		adds	x11, x11, 1
	csel	x10, xzr, x10, hi
	csinv	x11, x11, xzr, cc
	sbcs	xzr, x11, x10
	cset	x11, ls

	//NO_APP
	cbz	x11, .LBB10_9
// %bb.6:
	adrp	x21, arm64_const_caps_ready
	ldr	w10, [x21, :lo12:arm64_const_caps_ready]
	adrp	x20, cpu_hwcap_keys+16
	cmp	w10, #1                 // =1
	b.lt	.LBB10_10
// %bb.7:
	ldr	w10, [x20, :lo12:cpu_hwcap_keys+16]
	cmp	w10, #0                 // =0
	b.le	.LBB10_11
.LBB10_8:
	//APP
	.if 1 == 1
661:
	nop
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 10
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	.inst (0xd500401f | ((0) << 16 | (4) << 5) | ((!!0) << 8))
	
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	b	.LBB10_14
.LBB10_9:
	mov	w2, #1
	b	.LBB10_26
.LBB10_10:
	adrp	x10, cpu_hwcaps
	ldr	x10, [x10, :lo12:cpu_hwcaps]
	tbnz	w10, #4, .LBB10_8
.LBB10_11:
	//APP
	.if 1 == 1
661:
	mrs	x10, daif
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 42
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
		.irp	num,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30
	.equ	.L__reg_num_x\num, \num
	.endr
	.equ	.L__reg_num_xzr, 31
	.macro	mrs_s, rt, sreg
.inst (0xd5200000|(\sreg)|(.L__reg_num_\rt))
		.endm
	mrs_s x10, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
	.purgem	mrs_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	//APP
	.if 1 == 1
661:
	and	w11, w10, #0x00000080
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 42
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	eor	w11, w10, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w11, .LBB10_13
// %bb.12:
	mov	w11, #96
	//APP
	.if 1 == 1
661:
	msr	daifset, #2		// arch_local_irq_disable
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 42
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
		.irp	num,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30
	.equ	.L__reg_num_x\num, \num
	.endr
	.equ	.L__reg_num_xzr, 31
	.macro	msr_s, sreg, rt
.inst (0xd5000000|(\sreg)|(.L__reg_num_\rt))
		.endm
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x11
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB10_13:
	ldr	x11, [x8, #16]
	//APP
	mrs x12, ttbr1_el1
	//NO_APP
	mov	x13, x11
	bfxil	x13, x12, #0, #48
	//APP
	msr ttbr1_el1, x13
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x10
nop
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 42
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
		.irp	num,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30
	.equ	.L__reg_num_x\num, \num
	.endr
	.equ	.L__reg_num_xzr, 31
	.macro	msr_s, sreg, rt
.inst (0xd5000000|(\sreg)|(.L__reg_num_\rt))
		.endm
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB10_14:
	ldr	x10, [x8, #8]
	and	x9, x1, x9, asr #8
	//APP
		bics	xzr, x9, x10
	csel	x8, x1, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #4              // =4
	mov	w2, #1
	mov	x1, x8
	bl	__arch_copy_from_user
	ldr	w8, [x21, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB10_22
// %bb.15:
	ldr	w8, [x20, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB10_23
.LBB10_16:
	//APP
	.if 1 == 1
661:
	nop
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 10
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	.inst (0xd500401f | ((0) << 16 | (4) << 5) | ((!!1) << 8))
	
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	x2, .LBB10_26
.LBB10_17:
	ldrb	w8, [sp, #4]
	sub	w8, w8, #48             // =48
	cmp	w8, #2                  // =2
	b.hs	.LBB10_21
// %bb.18:
	adrp	x9, user_manual_gc
	mov	x0, x19
	strb	w8, [x9, :lo12:user_manual_gc]
.LBB10_19:
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB10_27
// %bb.20:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldr	x21, [sp, #32]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #64             // =64
	ret
.LBB10_21:
	adrp	x0, .L.str.6
	adrp	x1, .L__func__.mas_bkops_manual_gc_proc_write
	add	x0, x0, :lo12:.L.str.6
	add	x1, x1, :lo12:.L__func__.mas_bkops_manual_gc_proc_write
	mov	w2, #745
	bl	printk
	mov	x0, #-22
	b	.LBB10_19
.LBB10_22:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB10_16
.LBB10_23:
	//APP
	.if 1 == 1
661:
	mrs	x8, daif
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 42
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
		.irp	num,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30
	.equ	.L__reg_num_x\num, \num
	.endr
	.equ	.L__reg_num_xzr, 31
	.macro	mrs_s, rt, sreg
.inst (0xd5200000|(\sreg)|(.L__reg_num_\rt))
		.endm
	mrs_s x8, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
	.purgem	mrs_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	//APP
	.if 1 == 1
661:
	and	w9, w8, #0x00000080
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 42
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	eor	w9, w8, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w9, .LBB10_25
// %bb.24:
	mov	w9, #96
	//APP
	.if 1 == 1
661:
	msr	daifset, #2		// arch_local_irq_disable
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 42
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
		.irp	num,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30
	.equ	.L__reg_num_x\num, \num
	.endr
	.equ	.L__reg_num_xzr, 31
	.macro	msr_s, sreg, rt
.inst (0xd5000000|(\sreg)|(.L__reg_num_\rt))
		.endm
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB10_25:
	//APP
	mrs x9, ttbr1_el1
	//NO_APP
	and	x9, x9, #0xffffffffffff
	sub	x10, x9, #1, lsl #12    // =4096
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x9
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x8
nop
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 42
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
		.irp	num,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30
	.equ	.L__reg_num_x\num, \num
	.endr
	.equ	.L__reg_num_xzr, 31
	.macro	msr_s, sreg, rt
.inst (0xd5000000|(\sreg)|(.L__reg_num_\rt))
		.endm
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x8
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbz	x2, .LBB10_17
.LBB10_26:
	add	x8, sp, #4              // =4
	sub	x8, x8, x2
	add	x0, x8, #1              // =1
	mov	w1, wzr
	bl	memset
	mov	x0, #-14
	b	.LBB10_19
.LBB10_27:
	bl	__stack_chk_fail
.Lfunc_end10:
	.size	mas_bkops_manual_gc_proc_write, .Lfunc_end10-mas_bkops_manual_gc_proc_write
                                        // -- End function
	.globl	mas_bkops_status_proc_show // -- Begin function mas_bkops_status_proc_show
	.p2align	2
	.type	mas_bkops_status_proc_show,@function
mas_bkops_status_proc_show:             // @mas_bkops_status_proc_show
.Lmas_bkops_status_proc_show$local:
// %bb.0:
	sub	sp, sp, #64             // =64
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	str	x21, [sp, #32]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	str	x8, [sp, #8]
	str	wzr, [sp, #4]
	ldr	x20, [x0, #120]
	mov	x19, x0
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	ldr	x21, [x20, #8]
	add	x29, sp, #16            // =16
	bl	mutex_lock
	adrp	x8, bkops_enabled
	ldrb	w8, [x8, :lo12:bkops_enabled]
	tbnz	w8, #0, .LBB11_4
// %bb.1:
	ldr	x8, [x21, #8]
	ldr	x0, [x20, #16]
	add	x1, sp, #4              // =4
	blr	x8
	adrp	x8, user_manual_gc
	ldrb	w9, [x8, :lo12:user_manual_gc]
	cbz	w9, .LBB11_4
// %bb.2:
	ldr	w9, [sp, #4]
	cbnz	w9, .LBB11_4
// %bb.3:
	strb	wzr, [x8, :lo12:user_manual_gc]
.LBB11_4:
	adrp	x0, bkops_lock
	add	x0, x0, :lo12:bkops_lock
	bl	mutex_unlock
	ldr	w2, [sp, #4]
	adrp	x1, .L.str.7
	add	x1, x1, :lo12:.L.str.7
	mov	x0, x19
	bl	seq_printf
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB11_6
// %bb.5:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldr	x21, [sp, #32]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	mov	w0, wzr
	add	sp, sp, #64             // =64
	ret
.LBB11_6:
	bl	__stack_chk_fail
.Lfunc_end11:
	.size	mas_bkops_status_proc_show, .Lfunc_end11-mas_bkops_status_proc_show
                                        // -- End function
	.globl	mas_bkops_status_proc_open // -- Begin function mas_bkops_status_proc_open
	.p2align	2
	.type	mas_bkops_status_proc_open,@function
mas_bkops_status_proc_open:             // @mas_bkops_status_proc_open
.Lmas_bkops_status_proc_open$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x19, x1
	bl	PDE_DATA
	adrp	x1, __cfi_mas_bkops_status_proc_show
	mov	x2, x0
	add	x1, x1, :lo12:__cfi_mas_bkops_status_proc_show
	mov	x0, x19
	bl	single_open
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end12:
	.size	mas_bkops_status_proc_open, .Lfunc_end12-mas_bkops_status_proc_open
                                        // -- End function
	.globl	mas_bkops_enable        // -- Begin function mas_bkops_enable
	.p2align	2
	.type	mas_bkops_enable,@function
mas_bkops_enable:                       // @mas_bkops_enable
.Lmas_bkops_enable$local:
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	mov	x21, x0
	mov	w0, #-22
	str	x25, [sp, #16]          // 8-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x21, .LBB13_17
// %bb.1:
	mov	x19, x1
	cbz	x1, .LBB13_17
// %bb.2:
	mov	x22, x2
	cbz	x2, .LBB13_17
// %bb.3:
	mov	x0, x21
	bl	mas_blk_get_lld
	ldrb	w8, [x0, #24]
	tbz	w8, #3, .LBB13_23
// %bb.4:
	ldr	w8, [x19]
	sub	w8, w8, #1              // =1
	cmp	w8, #3                  // =3
	b.hs	.LBB13_18
// %bb.5:
	ldr	x8, [x19, #8]
	cbz	x8, .LBB13_19
// %bb.6:
	adrp	x25, mas_bkops_inited
	ldrb	w8, [x25, :lo12:mas_bkops_inited]
	mov	x20, x0
	adrp	x24, bkops_workqueue_g
	tbnz	w8, #0, .LBB13_9
// %bb.7:
	adrp	x0, .L.str.16
	add	x0, x0, :lo12:.L.str.16
	mov	w1, #24
	mov	w2, wzr
	bl	alloc_workqueue
	str	x0, [x24, :lo12:bkops_workqueue_g]
	cbz	x0, .LBB13_21
// %bb.8:
	adrp	x0, bkops_pm_notifier
	add	x0, x0, :lo12:bkops_pm_notifier
	bl	register_pm_notifier
	adrp	x0, bkops_reboot_notifier
	add	x0, x0, :lo12:bkops_reboot_notifier
	bl	register_reboot_notifier
	mov	w8, #1
	strb	w8, [x25, :lo12:mas_bkops_inited]
.LBB13_9:
	mov	x0, x19
	mov	x1, x22
	add	x23, x19, #184          // =184
	bl	mas_bkops_add_debugfs
	cbnz	w0, .LBB13_20
.LBB13_10:
	adrp	x1, .L.str.13
	add	x1, x1, :lo12:.L.str.13
	mov	w2, #32
	mov	x0, x23
	str	x21, [x19, #24]
	bl	strncpy
	adrp	x8, __cfi_bkops_io_busyidle_notify_handler
	add	x8, x8, :lo12:__cfi_bkops_io_busyidle_notify_handler
	mov	x0, x20
	mov	x1, x23
	strb	wzr, [x19, #215]
	stp	x8, x19, [x19, #216]
	bl	blk_lld_busyidle_event_subscribe
	cbz	w0, .LBB13_14
// %bb.11:
	ldr	x8, [x24, :lo12:bkops_workqueue_g]
	cbz	x8, .LBB13_13
// %bb.12:
	adrp	x0, bkops_reboot_notifier
	add	x0, x0, :lo12:bkops_reboot_notifier
	strb	wzr, [x25, :lo12:mas_bkops_inited]
	bl	unregister_reboot_notifier
	adrp	x0, bkops_pm_notifier
	add	x0, x0, :lo12:bkops_pm_notifier
	bl	unregister_pm_notifier
	ldr	x0, [x24, :lo12:bkops_workqueue_g]
	bl	destroy_workqueue
	str	xzr, [x24, :lo12:bkops_workqueue_g]
.LBB13_13:
	mov	w0, #-22
	b	.LBB13_17
.LBB13_14:
	adrp	x0, .L.str.17
	add	x0, x0, :lo12:.L.str.17
	mov	x1, xzr
	bl	proc_mkdir
	cbz	x0, .LBB13_22
// %bb.15:
	mov	x20, x0
	adrp	x0, .L.str.19
	adrp	x3, mas_bkops_manual_gc_fops
	add	x0, x0, :lo12:.L.str.19
	add	x3, x3, :lo12:mas_bkops_manual_gc_fops
	mov	w1, #432
	mov	x2, x20
	bl	proc_create
	adrp	x0, .L.str.20
	adrp	x3, mas_bkops_status_fops
	add	x0, x0, :lo12:.L.str.20
	add	x3, x3, :lo12:mas_bkops_status_fops
	mov	w1, #288
	mov	x2, x20
	mov	x4, x19
	bl	proc_create_data
.LBB13_16:
	mov	w0, wzr
.LBB13_17:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB13_18:
	adrp	x0, .L.str.9
	add	x0, x0, :lo12:.L.str.9
	bl	printk
	b	.LBB13_13
.LBB13_19:
	adrp	x0, .L.str.10
	add	x0, x0, :lo12:.L.str.10
	bl	printk
	b	.LBB13_13
.LBB13_20:
	adrp	x0, .L.str.12
	add	x0, x0, :lo12:.L.str.12
	bl	printk
	b	.LBB13_10
.LBB13_21:
	adrp	x0, .L.str.11
	adrp	x1, .L__func__.mas_bkops_enable
	add	x0, x0, :lo12:.L.str.11
	add	x1, x1, :lo12:.L__func__.mas_bkops_enable
	mov	w2, #830
	bl	printk
	b	.LBB13_13
.LBB13_22:
	adrp	x0, .L.str.18
	adrp	x1, .L__func__.mas_bkops_manual_gc_proc_init
	add	x0, x0, :lo12:.L.str.18
	add	x1, x1, :lo12:.L__func__.mas_bkops_manual_gc_proc_init
	mov	w2, #792
	bl	printk
	b	.LBB13_16
.LBB13_23:
	adrp	x0, .L.str.8
	add	x0, x0, :lo12:.L.str.8
	bl	printk
	b	.LBB13_13
.Lfunc_end13:
	.size	mas_bkops_enable, .Lfunc_end13-mas_bkops_enable
                                        // -- End function
	.globl	blk_mq_tagset_tz_query_register // -- Begin function blk_mq_tagset_tz_query_register
	.p2align	2
	.type	blk_mq_tagset_tz_query_register,@function
blk_mq_tagset_tz_query_register:        // @blk_mq_tagset_tz_query_register
.Lblk_mq_tagset_tz_query_register$local:
// %bb.0:
	cbz	x0, .LBB14_2
// %bb.1:
	str	x1, [x0, #128]
.LBB14_2:
	ret
.Lfunc_end14:
	.size	blk_mq_tagset_tz_query_register, .Lfunc_end14-blk_mq_tagset_tz_query_register
                                        // -- End function
	.globl	blk_lld_tz_query        // -- Begin function blk_lld_tz_query
	.p2align	2
	.type	blk_lld_tz_query,@function
blk_lld_tz_query:                       // @blk_lld_tz_query
.Lblk_lld_tz_query$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	mov	x8, x0
	mov	w0, #-1
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x8, .LBB15_7
// %bb.1:
	mov	x20, x2
	cbz	x2, .LBB15_7
// %bb.2:
	ldr	x8, [x8, #128]
	ldr	x22, [x8, #1264]
	cbz	x22, .LBB15_6
// %bb.3:
	mov	x0, x22
	mov	w19, w3
	mov	w21, w1
	bl	mas_blk_get_lld
	cbz	x0, .LBB15_6
// %bb.4:
	ldr	x8, [x0, #64]
	cbz	x8, .LBB15_6
// %bb.5:
	mov	x0, x22
	mov	w1, w21
	mov	x2, x20
	mov	w3, w19
	blr	x8
	b	.LBB15_7
.LBB15_6:
	mov	w0, #-1
.LBB15_7:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end15:
	.size	blk_lld_tz_query, .Lfunc_end15-blk_lld_tz_query
                                        // -- End function
	.globl	blk_mq_tagset_tz_ctrl_register // -- Begin function blk_mq_tagset_tz_ctrl_register
	.p2align	2
	.type	blk_mq_tagset_tz_ctrl_register,@function
blk_mq_tagset_tz_ctrl_register:         // @blk_mq_tagset_tz_ctrl_register
.Lblk_mq_tagset_tz_ctrl_register$local:
// %bb.0:
	cbz	x0, .LBB16_2
// %bb.1:
	str	x1, [x0, #136]
.LBB16_2:
	ret
.Lfunc_end16:
	.size	blk_mq_tagset_tz_ctrl_register, .Lfunc_end16-blk_mq_tagset_tz_ctrl_register
                                        // -- End function
	.globl	blk_lld_tz_ctrl         // -- Begin function blk_lld_tz_ctrl
	.p2align	2
	.type	blk_lld_tz_ctrl,@function
blk_lld_tz_ctrl:                        // @blk_lld_tz_ctrl
.Lblk_lld_tz_ctrl$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB17_5
// %bb.1:
	ldr	x8, [x0, #128]
	ldr	x21, [x8, #1264]
	cbz	x21, .LBB17_5
// %bb.2:
	mov	x0, x21
	mov	w19, w2
	mov	w20, w1
	bl	mas_blk_get_lld
	cbz	x0, .LBB17_5
// %bb.3:
	ldr	x8, [x0, #72]
	cbz	x8, .LBB17_5
// %bb.4:
	mov	x0, x21
	mov	w1, w20
	mov	w2, w19
	blr	x8
	b	.LBB17_6
.LBB17_5:
	mov	w0, #-1
.LBB17_6:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end17:
	.size	blk_lld_tz_ctrl, .Lfunc_end17-blk_lld_tz_ctrl
                                        // -- End function
	.globl	blk_mq_tagset_health_query_register // -- Begin function blk_mq_tagset_health_query_register
	.p2align	2
	.type	blk_mq_tagset_health_query_register,@function
blk_mq_tagset_health_query_register:    // @blk_mq_tagset_health_query_register
.Lblk_mq_tagset_health_query_register$local:
// %bb.0:
	cbz	x0, .LBB18_2
// %bb.1:
	str	x1, [x0, #144]
.LBB18_2:
	ret
.Lfunc_end18:
	.size	blk_mq_tagset_health_query_register, .Lfunc_end18-blk_mq_tagset_health_query_register
                                        // -- End function
	.globl	blk_lld_health_query    // -- Begin function blk_lld_health_query
	.p2align	2
	.type	blk_lld_health_query,@function
blk_lld_health_query:                   // @blk_lld_health_query
.Lblk_lld_health_query$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	mov	x8, x0
	mov	w0, #-1
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x8, .LBB19_8
// %bb.1:
	mov	x21, x1
	cbz	x1, .LBB19_8
// %bb.2:
	mov	x20, x2
	cbz	x2, .LBB19_8
// %bb.3:
	mov	x19, x3
	cbz	x3, .LBB19_8
// %bb.4:
	ldr	x8, [x8, #128]
	ldr	x22, [x8, #1264]
	mov	x0, x22
	bl	mas_blk_get_lld
	cbz	x0, .LBB19_7
// %bb.5:
	ldr	x8, [x0, #80]
	cbz	x8, .LBB19_7
// %bb.6:
	mov	x0, x22
	mov	x1, x21
	mov	x2, x20
	mov	x3, x19
	blr	x8
	b	.LBB19_8
.LBB19_7:
	mov	w0, #-1
.LBB19_8:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end19:
	.size	blk_lld_health_query, .Lfunc_end19-blk_lld_health_query
                                        // -- End function
	.p2align	2               // -- Begin function bkops_should_query
	.type	bkops_should_query,@function
bkops_should_query:                     // @bkops_should_query
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0, #168]
	mov	x19, x0
	mov	x29, sp
	tbnz	w8, #1, .LBB20_15
// %bb.1:
	ldr	w9, [x19, #244]
	cbz	w9, .LBB20_3
// %bb.2:
	ldr	w9, [x19, #240]
	cbnz	w9, .LBB20_15
.LBB20_3:
	ldr	x0, [x19, #24]
	bl	mas_blk_get_lld
	ldr	x8, [x19, #168]
	mov	x20, x0
	tbz	w8, #4, .LBB20_7
// %bb.4:
	mov	w0, #1
	bl	ktime_get_with_offset
	bl	ns_to_timespec64
	ldr	x8, [x19, #256]
	cbz	x8, .LBB20_17
// %bb.5:
	ldr	x9, [x19, #248]
	sub	x8, x0, x8
	cmp	x8, x9
	b.ge	.LBB20_17
// %bb.6:
	ldr	x8, [x19, #168]
.LBB20_7:
	tbz	w8, #3, .LBB20_9
// %bb.8:
	ldr	x9, [x20, #1384]
	ldp	x11, x10, [x19, #264]
	sub	x10, x9, x10
	cmp	x10, x11
	b.hs	.LBB20_13
.LBB20_9:
	tbz	w8, #2, .LBB20_11
// %bb.10:
	ldr	x9, [x20, #1376]
	ldp	x11, x10, [x19, #280]
	sub	x10, x9, x10
	cmp	x10, x11
	b.hs	.LBB20_14
.LBB20_11:
	adrp	x9, user_manual_gc
	ldrb	w9, [x9, :lo12:user_manual_gc]
	cbnz	w9, .LBB20_15
// %bb.12:
	mov	w0, wzr
	and	x8, x8, #0xfffffffffffffffd
	b	.LBB20_16
.LBB20_13:
	str	x9, [x19, #272]
	b	.LBB20_15
.LBB20_14:
	str	x9, [x19, #288]
.LBB20_15:
	orr	x8, x8, #0x2
	mov	w0, #1
.LBB20_16:
	str	x8, [x19, #168]
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB20_17:
	ldr	x8, [x19, #168]
	str	x0, [x19, #256]
	b	.LBB20_15
.Lfunc_end20:
	.size	bkops_should_query, .Lfunc_end20-bkops_should_query
                                        // -- End function
	.type	.L.str,@object          // @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"\0013[BKOPS]mas_bkops_query failed!\n"
	.size	.L.str, 34

	.type	.L.str.1,@object        // @.str.1
.L.str.1:
	.asciz	"\0013[BKOPS]mas_bkops_start failed!\n"
	.size	.L.str.1, 34

	.type	bkops_io_busyidle_notify_handler._rs,@object // @bkops_io_busyidle_notify_handler._rs
	.data
	.p2align	3
bkops_io_busyidle_notify_handler._rs:
	.zero	4
	.word	1250                    // 0x4e2
	.word	10                      // 0xa
	.word	0                       // 0x0
	.word	0                       // 0x0
	.zero	4
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	bkops_io_busyidle_notify_handler._rs, 40

	.type	.L__func__.bkops_io_busyidle_notify_handler,@object // @__func__.bkops_io_busyidle_notify_handler
	.section	.rodata.str1.1,"aMS",@progbits,1
.L__func__.bkops_io_busyidle_notify_handler:
	.asciz	"bkops_io_busyidle_notify_handler"
	.size	.L__func__.bkops_io_busyidle_notify_handler, 33

	.type	.L.str.2,@object        // @.str.2
.L.str.2:
	.asciz	"\0013[BKOPS]%s unknown state: %d\n"
	.size	.L.str.2, 31

	.type	.L.str.3,@object        // @.str.3
.L.str.3:
	.asciz	"\0013[BKOPS]%s: mas_bkops_query failed, ret = %d!\n"
	.size	.L.str.3, 48

	.type	.L__func__.mas_bkops_work_query,@object // @__func__.mas_bkops_work_query
.L__func__.mas_bkops_work_query:
	.asciz	"mas_bkops_work_query"
	.size	.L__func__.mas_bkops_work_query, 21

	.type	.L__func__.mas_bkops_work_start,@object // @__func__.mas_bkops_work_start
.L__func__.mas_bkops_work_start:
	.asciz	"mas_bkops_work_start"
	.size	.L__func__.mas_bkops_work_start, 21

	.type	.L.str.4,@object        // @.str.4
.L.str.4:
	.asciz	"\0013[BKOPS]Failed to alloc bkops_func\n"
	.size	.L.str.4, 37

	.type	.L.str.5,@object        // @.str.5
.L.str.5:
	.asciz	"%d\n"
	.size	.L.str.5, 4

	.type	user_manual_gc,@object  // @user_manual_gc
	.local	user_manual_gc
	.comm	user_manual_gc,1,4
	.type	.L.str.6,@object        // @.str.6
.L.str.6:
	.asciz	"\0013[BKOPS]%s %d: input error.\n"
	.size	.L.str.6, 30

	.type	.L__func__.mas_bkops_manual_gc_proc_write,@object // @__func__.mas_bkops_manual_gc_proc_write
.L__func__.mas_bkops_manual_gc_proc_write:
	.asciz	"mas_bkops_manual_gc_proc_write"
	.size	.L__func__.mas_bkops_manual_gc_proc_write, 31

	.type	.L.str.7,@object        // @.str.7
.L.str.7:
	.asciz	"%u\n"
	.size	.L.str.7, 4

	.type	.L.str.8,@object        // @.str.8
.L.str.8:
	.asciz	"\0013[BKOPS]Busy Idle was not enabled!\n"
	.size	.L.str.8, 37

	.type	.L.str.9,@object        // @.str.9
.L.str.9:
	.asciz	"\0013[BKOPS]bkops dev type not specified!\n"
	.size	.L.str.9, 40

	.type	.L.str.10,@object       // @.str.10
.L.str.10:
	.asciz	"\0013[BKOPS]bkops_ops is NULL!\n"
	.size	.L.str.10, 29

	.type	.L.str.11,@object       // @.str.11
.L.str.11:
	.asciz	"\0013[BKOPS]%s %d mas_bkops_init failed!\n"
	.size	.L.str.11, 39

	.type	.L__func__.mas_bkops_enable,@object // @__func__.mas_bkops_enable
.L__func__.mas_bkops_enable:
	.asciz	"mas_bkops_enable"
	.size	.L__func__.mas_bkops_enable, 17

	.type	.L.str.12,@object       // @.str.12
.L.str.12:
	.asciz	"\0013[BKOPS]Failed to create mmc bkops debugfs!\n"
	.size	.L.str.12, 46

	.type	.L.str.13,@object       // @.str.13
.L.str.13:
	.asciz	"mas_bkops"
	.size	.L.str.13, 10

	.type	bkops_lock,@object      // @bkops_lock
	.data
	.p2align	3
bkops_lock:
	.zero	8
	.zero	4
	.zero	4
	.xword	bkops_lock+16
	.xword	bkops_lock+16
	.size	bkops_lock, 32

	.type	bkops_enabled,@object   // @bkops_enabled
	.local	bkops_enabled
	.comm	bkops_enabled,1,1
	.type	bkops_workqueue_g,@object // @bkops_workqueue_g
	.local	bkops_workqueue_g
	.comm	bkops_workqueue_g,8,8
	.type	.L.str.14,@object       // @.str.14
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.14:
	.asciz	"\0013[BKOPS]%s: mas_bkops_start failed, ret = %d!\n"
	.size	.L.str.14, 48

	.type	.L__func__.mas_bkops_work_start_ret,@object // @__func__.mas_bkops_work_start_ret
.L__func__.mas_bkops_work_start_ret:
	.asciz	"mas_bkops_work_start_ret"
	.size	.L__func__.mas_bkops_work_start_ret, 25

	.type	mas_bkops_inited,@object // @mas_bkops_inited
	.local	mas_bkops_inited
	.comm	mas_bkops_inited,1,4
	.type	.L.str.16,@object       // @.str.16
.L.str.16:
	.asciz	"bkops_workqueue"
	.size	.L.str.16, 16

	.type	bkops_pm_notifier,@object // @bkops_pm_notifier
	.data
	.p2align	3
bkops_pm_notifier:
	.xword	__cfi_bkops_pm_callback
	.xword	0
	.word	0                       // 0x0
	.zero	4
	.size	bkops_pm_notifier, 24

	.type	bkops_reboot_notifier,@object // @bkops_reboot_notifier
	.p2align	3
bkops_reboot_notifier:
	.xword	__cfi_bkops_notify_reboot
	.xword	0
	.word	2147483647              // 0x7fffffff
	.zero	4
	.size	bkops_reboot_notifier, 24

	.type	.L.str.17,@object       // @.str.17
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.17:
	.asciz	"block"
	.size	.L.str.17, 6

	.type	.L.str.18,@object       // @.str.18
.L.str.18:
	.asciz	"\0013[BKOPS]%s %d: proc mk error.\n"
	.size	.L.str.18, 32

	.type	.L__func__.mas_bkops_manual_gc_proc_init,@object // @__func__.mas_bkops_manual_gc_proc_init
.L__func__.mas_bkops_manual_gc_proc_init:
	.asciz	"mas_bkops_manual_gc_proc_init"
	.size	.L__func__.mas_bkops_manual_gc_proc_init, 30

	.type	.L.str.19,@object       // @.str.19
.L.str.19:
	.asciz	"manual_gc"
	.size	.L.str.19, 10

	.type	mas_bkops_manual_gc_fops,@object // @mas_bkops_manual_gc_fops
	.section	.rodata,"a",@progbits
	.p2align	3
mas_bkops_manual_gc_fops:
	.xword	0
	.xword	seq_lseek
	.xword	seq_read
	.xword	__cfi_mas_bkops_manual_gc_proc_write
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0                       // 0x0
	.xword	__cfi_mas_bkops_manual_gc_proc_open
	.xword	0
	.xword	single_release
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	mas_bkops_manual_gc_fops, 288

	.type	.L.str.20,@object       // @.str.20
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.20:
	.asciz	"status"
	.size	.L.str.20, 7

	.type	mas_bkops_status_fops,@object // @mas_bkops_status_fops
	.section	.rodata,"a",@progbits
	.p2align	3
mas_bkops_status_fops:
	.xword	0
	.xword	seq_lseek
	.xword	seq_read
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0                       // 0x0
	.xword	__cfi_mas_bkops_status_proc_open
	.xword	0
	.xword	single_release
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	mas_bkops_status_fops, 288

	.ident	"Android (6443078 based on r383902) clang version 11.0.1 (https://android.googlesource.com/toolchain/llvm-project b397f81060ce6d701042b782172ed13bee898b79)"
	.section	".note.GNU-stack","",@progbits
