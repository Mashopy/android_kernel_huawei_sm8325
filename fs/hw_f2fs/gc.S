	.text
	.file	"gc.c"
	.globl	f2fs_init_gc_thread     // -- Begin function f2fs_init_gc_thread
	.p2align	2
	.type	f2fs_init_gc_thread,@function
f2fs_init_gc_thread:                    // @f2fs_init_gc_thread
.Lf2fs_init_gc_thread$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	adrp	x1, .L.str
	adrp	x2, f2fs_init_gc_thread.__key
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	add	x0, x0, #1720           // =1720
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:f2fs_init_gc_thread.__key
	mov	x29, sp
	bl	__init_waitqueue_head
	adrp	x1, .L.str.2
	adrp	x2, f2fs_init_gc_thread.__key.1
	add	x0, x19, #1776          // =1776
	add	x1, x1, :lo12:.L.str.2
	add	x2, x2, :lo12:f2fs_init_gc_thread.__key.1
	bl	__init_waitqueue_head
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end0:
	.size	f2fs_init_gc_thread, .Lfunc_end0-f2fs_init_gc_thread
                                        // -- End function
	.globl	f2fs_start_gc_thread    // -- Begin function f2fs_start_gc_thread
	.p2align	2
	.type	f2fs_start_gc_thread,@function
f2fs_start_gc_thread:                   // @f2fs_start_gc_thread
.Lf2fs_start_gc_thread$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0]
	mov	x11, #60000
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x10, #500
	ldr	x8, [x8, #248]
	movk	x11, #37856, lsl #32
	movk	x10, #10000, lsl #32
	movk	x11, #4, lsl #48
	ldr	w21, [x8]
	adrp	x1, set_block_idle
	mov	x19, x0
	add	x9, x0, #1712           // =1712
	str	xzr, [x0, #1712]
	str	x10, [x0, #1744]
	str	x11, [x0, #1752]
	mov	w8, #1
	add	x0, x0, #1856           // =1856
	add	x1, x1, :lo12:set_block_idle
	mov	w2, wzr
	mov	x3, xzr
	mov	x4, xzr
	mov	x29, sp
	str	wzr, [x9, #60]
	str	w8, [x9, #52]
	bl	init_timer_key
	add	x20, x19, #1800         // =1800
	adrp	x1, .L.str.3
	add	x1, x1, :lo12:.L.str.3
	mov	w2, #32
	mov	x0, x20
	strb	wzr, [x19, #1912]
	bl	strncpy
	ldr	x8, [x19]
	adrp	x9, gc_io_busyidle_notify_handler
	add	x9, x9, :lo12:gc_io_busyidle_notify_handler
	str	x9, [x19, #1832]
	str	x19, [x19, #1840]
	ldr	x0, [x8, #248]
	mov	x1, x20
	bl	blk_busyidle_event_subscribe
	mov	w20, w0
	adrp	x0, gc_thread_func
	adrp	x3, .L.str.4
	lsr	w4, w21, #20
	and	w5, w21, #0xfffff
	add	x0, x0, :lo12:gc_thread_func
	add	x3, x3, :lo12:.L.str.4
	mov	w2, #-1
	mov	x1, x19
	bl	kthread_create_on_node
	mov	x21, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB1_2
// %bb.1:
	mov	x0, x21
	bl	wake_up_process
	str	x21, [x19, #1712]
	mov	w21, w20
.LBB1_2:
	mov	w0, w21
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end1:
	.size	f2fs_start_gc_thread, .Lfunc_end1-f2fs_start_gc_thread
                                        // -- End function
	.p2align	2               // -- Begin function set_block_idle
	.type	set_block_idle,@function
set_block_idle:                         // @set_block_idle
// %bb.0:
	mov	w8, #1
	strb	w8, [x0, #56]
	ret
.Lfunc_end2:
	.size	set_block_idle, .Lfunc_end2-set_block_idle
                                        // -- End function
	.p2align	2               // -- Begin function gc_io_busyidle_notify_handler
	.type	gc_io_busyidle_notify_handler,@function
gc_io_busyidle_notify_handler:          // @gc_io_busyidle_notify_handler
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	ldr	x19, [x0, #40]
	mov	x29, sp
	ldr	x8, [x19, #1712]
	cbz	x8, .LBB3_5
// %bb.1:
	cbz	w1, .LBB3_4
// %bb.2:
	cmp	w1, #1                  // =1
	b.ne	.LBB3_5
// %bb.3:
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	add	x0, x19, #1856          // =1856
	add	x1, x8, #250            // =250
	bl	mod_timer
	b	.LBB3_5
.LBB3_4:
	add	x0, x19, #1856          // =1856
	bl	del_timer_sync
	strb	wzr, [x19, #1912]
.LBB3_5:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	mov	w0, wzr
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end3:
	.size	gc_io_busyidle_notify_handler, .Lfunc_end3-gc_io_busyidle_notify_handler
                                        // -- End function
	.p2align	2               // -- Begin function gc_thread_func
	.type	gc_thread_func,@function
gc_thread_func:                         // @gc_thread_func
// %bb.0:
	sub	sp, sp, #224            // =224
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #128]    // 16-byte Folded Spill
	add	x29, sp, #128           // =128
	stp	x28, x27, [sp, #144]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #160]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #176]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #192]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #208]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	//APP
	mrs x25, sp_el0
	//NO_APP
	ldr	w8, [x25, #60]
	ldr	w24, [x0, #1748]
	mov	x20, x0
	orr	w8, w8, #0x20000000
	str	w8, [x25, #60]
	bl	set_freezable
	add	x8, x20, #3408          // =3408
	str	x8, [sp, #16]           // 8-byte Folded Spill
	add	x8, x20, #1720          // =1720
	str	x8, [sp, #40]           // 8-byte Folded Spill
	add	x8, x20, #1664          // =1664
	add	x28, x20, #1784         // =1784
	str	x8, [sp, #56]           // 8-byte Folded Spill
	add	x8, x20, #1776          // =1776
	adrp	x19, jiffies
	str	x8, [sp, #48]           // 8-byte Folded Spill
	str	x28, [sp, #32]          // 8-byte Folded Spill
	b	.LBB4_4
.LBB4_1:                                //   in Loop: Header=BB4_4 Depth=1
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	mov	w1, #3
	mov	w2, wzr
	mov	x3, xzr
	bl	__wake_up
.LBB4_2:                                //   in Loop: Header=BB4_4 Depth=1
	ldr	x0, [x20]
	mov	w1, #1
	bl	__sb_end_write
.LBB4_3:                                //   in Loop: Header=BB4_4 Depth=1
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB4_132
.LBB4_4:                                // =>This Loop Header: Depth=1
                                        //     Child Loop BB4_21 Depth 2
                                        //       Child Loop BB4_27 Depth 3
                                        //     Child Loop BB4_97 Depth 2
                                        //     Child Loop BB4_120 Depth 2
	ldr	x8, [x20, #136]
	ldr	w22, [x20, #1236]
	ldp	x10, x9, [x8]
	ldr	w8, [x8, #104]
	ldr	w26, [x9, #4]
	ldr	w23, [x10, #16]
	add	w27, w8, w8, lsl #1
	cmp	w26, w27
	b.hs	.LBB4_39
// %bb.5:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	w9, [x20, #1232]
	tbnz	w9, #2, .LBB4_39
// %bb.6:                               //   in Loop: Header=BB4_4 Depth=1
	subs	w9, w26, w8
	sub	w8, w22, w23
	b.hs	.LBB4_8
// %bb.7:                               //   in Loop: Header=BB4_4 Depth=1
	mov	x9, xzr
	b	.LBB4_9
.LBB4_8:                                //   in Loop: Header=BB4_4 Depth=1
	ldr	w10, [x20, #1184]
	lsl	w9, w9, w10
.LBB4_9:                                //   in Loop: Header=BB4_4 Depth=1
	sub	x9, x8, x9
	cmp	x9, #1                  // =1
	b.lt	.LBB4_39
// %bb.10:                              //   in Loop: Header=BB4_4 Depth=1
	add	x9, x9, x9, lsl #2
	lsl	x9, x9, #1
	cmp	x9, x8
	b.le	.LBB4_39
// %bb.11:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	x8, [x20, #3384]
	ldr	x9, [x19, :lo12:jiffies]
	sub	x8, x8, x9
	mov	w9, #18928
	movk	w9, #2, lsl #16
	cmn	x8, x9
	b.mi	.LBB4_14
// %bb.12:                              //   in Loop: Header=BB4_4 Depth=1
	ldrb	w8, [x20, #3377]
	cbz	w8, .LBB4_39
.LBB4_13:                               //   in Loop: Header=BB4_4 Depth=1
	mov	w8, #3
	str	w8, [x20, #1764]
	b	.LBB4_55
.LBB4_14:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	x8, [x19, :lo12:jiffies]
	ldr	x9, [x20, #16]
	str	x8, [x20, #3384]
	ldr	w8, [x9, #68]
	stp	xzr, xzr, [x29, #-24]
	stp	xzr, xzr, [x29, #-40]
	stur	xzr, [x29, #-48]
	str	x8, [sp, #64]           // 8-byte Folded Spill
	ldur	x8, [x9, #36]
	cmp	x8, #2048, lsl #12      // =8388608
	b.lo	.LBB4_16
// %bb.15:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w9, [x20, #1240]
	sub	x8, x8, x9
	cmp	x8, #640, lsl #12       // =2621440
	b.ls	.LBB4_17
.LBB4_16:                               //   in Loop: Header=BB4_4 Depth=1
	mov	w8, wzr
	b	.LBB4_38
.LBB4_17:                               //   in Loop: Header=BB4_4 Depth=1
	sub	x0, x29, #48            // =48
	mov	w2, #40
	mov	w1, wzr
	str	x25, [sp, #24]          // 8-byte Folded Spill
	bl	memset
	ldr	x8, [sp, #64]           // 8-byte Folded Reload
	cbz	w8, .LBB4_36
// %bb.18:                              //   in Loop: Header=BB4_4 Depth=1
	mov	x28, xzr
	mov	w25, wzr
	b	.LBB4_21
.LBB4_19:                               //   in Loop: Header=BB4_21 Depth=2
	ldur	w8, [x29, #-12]
	add	w8, w8, w1
	stur	w8, [x29, #-12]
.LBB4_20:                               //   in Loop: Header=BB4_21 Depth=2
	ldr	x8, [sp, #64]           // 8-byte Folded Reload
	add	x28, x28, #1            // =1
	adrp	x19, jiffies
	cmp	x28, x8
	b.eq	.LBB4_37
.LBB4_21:                               //   Parent Loop BB4_4 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB4_27 Depth 3
	ldr	x8, [x20, #136]
	mov	w9, #40
	mul	x9, x28, x9
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	stur	xzr, [x29, #-56]
	ldr	w1, [x20, #1188]
	ldr	w9, [x8, x9]
	ubfx	w9, w9, #6, #10
	cbz	w9, .LBB4_19
// %bb.22:                              //   in Loop: Header=BB4_21 Depth=2
	cmp	w9, w1
	b.eq	.LBB4_20
// %bb.23:                              //   in Loop: Header=BB4_21 Depth=2
	cbz	w1, .LBB4_20
// %bb.24:                              //   in Loop: Header=BB4_21 Depth=2
	mov	w9, #40
	madd	x8, x28, x9, x8
	add	x19, x8, #8             // =8
	b	.LBB4_27
.LBB4_25:                               //   in Loop: Header=BB4_27 Depth=3
	mov	w8, #-1
.LBB4_26:                               //   in Loop: Header=BB4_27 Depth=3
	sbfiz	x8, x8, #2, #32
	sub	x10, x29, #48           // =48
	ldr	w9, [x10, x8]
	add	w25, w21, w25
	add	w9, w9, w21
	str	w9, [x10, x8]
	ldur	x8, [x29, #-56]
	ldr	w1, [x20, #1188]
	cmp	x8, x1
	b.hs	.LBB4_20
.LBB4_27:                               //   Parent Loop BB4_4 Depth=1
                                        //     Parent Loop BB4_21 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	ldr	x0, [x19]
	sub	x2, x29, #56            // =56
	bl	find_next_free_extent
	tbnz	w0, #31, .LBB4_20
// %bb.28:                              //   in Loop: Header=BB4_27 Depth=3
	mov	w21, w0
	cbz	w0, .LBB4_25
// %bb.29:                              //   in Loop: Header=BB4_27 Depth=3
	clz	w8, w21
	eor	w8, w8, #0x1f
	cmp	w21, #1024              // =1024
	mov	w9, #9
	csel	w8, w8, w9, lo
	cbz	x20, .LBB4_26
// %bb.30:                              //   in Loop: Header=BB4_27 Depth=3
	b.lo	.LBB4_26
// %bb.31:                              //   in Loop: Header=BB4_27 Depth=3
	add	x8, x20, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x20, #3376]
	cbnz	w8, .LBB4_35
// %bb.32:                              //   in Loop: Header=BB4_27 Depth=3
	ldr	x1, [x20, #3392]
	mov	w8, #9
	cbz	x1, .LBB4_26
// %bb.33:                              //   in Loop: Header=BB4_27 Depth=3
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB4_26
// %bb.34:                              //   in Loop: Header=BB4_27 Depth=3
	ldr	x2, [sp, #16]           // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
.LBB4_35:                               //   in Loop: Header=BB4_27 Depth=3
	mov	w8, #9
	b	.LBB4_26
.LBB4_36:                               //   in Loop: Header=BB4_4 Depth=1
	mov	w25, wzr
.LBB4_37:                               //   in Loop: Header=BB4_4 Depth=1
	ldur	w3, [x29, #-48]
	adrp	x21, .L.str.13
	add	x21, x21, :lo12:.L.str.13
	mov	x0, x20
	mov	x1, x21
	mov	w2, wzr
	bl	f2fs_printk
	ldur	w3, [x29, #-44]
	mov	w2, #1
	mov	x0, x20
	mov	x1, x21
	bl	f2fs_printk
	ldur	w3, [x29, #-40]
	mov	w2, #2
	mov	x0, x20
	mov	x1, x21
	bl	f2fs_printk
	ldur	w3, [x29, #-36]
	mov	w2, #3
	mov	x0, x20
	mov	x1, x21
	bl	f2fs_printk
	ldur	w3, [x29, #-32]
	mov	w2, #4
	mov	x0, x20
	mov	x1, x21
	bl	f2fs_printk
	ldur	w3, [x29, #-28]
	mov	w2, #5
	mov	x0, x20
	mov	x1, x21
	bl	f2fs_printk
	ldur	w3, [x29, #-24]
	mov	w2, #6
	mov	x0, x20
	mov	x1, x21
	bl	f2fs_printk
	ldur	w3, [x29, #-20]
	mov	w2, #7
	mov	x0, x20
	mov	x1, x21
	bl	f2fs_printk
	ldur	w3, [x29, #-16]
	mov	w2, #8
	mov	x0, x20
	mov	x1, x21
	bl	f2fs_printk
	ldur	w3, [x29, #-12]
	mov	w2, #9
	mov	x0, x20
	mov	x1, x21
	bl	f2fs_printk
	ldp	w8, w9, [x29, #-48]
	add	w9, w9, w8
	cmp	w9, w25, lsr #1
	cset	w9, hs
	cmp	w8, w25, lsr #2
	ldr	x25, [sp, #24]          // 8-byte Folded Reload
	cset	w8, hs
	orr	w8, w8, w9
.LBB4_38:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	x28, [sp, #32]          // 8-byte Folded Reload
	strb	w8, [x20, #3377]
	cbnz	w8, .LBB4_13
.LBB4_39:                               //   in Loop: Header=BB4_4 Depth=1
	mov	w8, #100
	mul	x8, x23, x8
	udiv	x8, x8, x22
	cmp	x8, #90                 // =90
	b.ls	.LBB4_41
// %bb.40:                              //   in Loop: Header=BB4_4 Depth=1
	mov	w8, #30000
	str	wzr, [x20, #1764]
	b	.LBB4_50
.LBB4_41:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	x8, [x20, #136]
	ldr	w9, [x20, #1236]
	ldr	x10, [x8]
	ldr	w10, [x10, #16]
	subs	w10, w9, w10
	b.eq	.LBB4_49
// %bb.42:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	x11, [x8, #8]
	ldr	w12, [x8, #104]
	ldr	w11, [x11, #4]
	subs	w11, w11, w12
	b.hs	.LBB4_44
// %bb.43:                              //   in Loop: Header=BB4_4 Depth=1
	mov	x11, xzr
	cmp	w26, w27
	b.lo	.LBB4_45
	b	.LBB4_49
.LBB4_44:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	w12, [x20, #1184]
	lsl	w11, w11, w12
	mov	w12, #100
	umull	x11, w11, w12
	cmp	w26, w27
	b.hs	.LBB4_49
.LBB4_45:                               //   in Loop: Header=BB4_4 Depth=1
	mov	w10, w10
	udiv	x10, x11, x10
	cmp	w10, #9                 // =9
	b.gt	.LBB4_49
// %bb.46:                              //   in Loop: Header=BB4_4 Depth=1
	mov	w10, #2
	str	w10, [x20, #1764]
	ldr	x10, [x8]
	ldr	w10, [x10, #16]
	subs	w9, w9, w10
	b.eq	.LBB4_51
// %bb.47:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	x10, [x8, #8]
	ldr	w8, [x8, #104]
	ldr	w10, [x10, #4]
	subs	w8, w10, w8
	b.hs	.LBB4_52
// %bb.48:                              //   in Loop: Header=BB4_4 Depth=1
	mov	x8, xzr
	b	.LBB4_53
.LBB4_49:                               //   in Loop: Header=BB4_4 Depth=1
	mov	w8, #1
	str	w8, [x20, #1764]
	mov	w8, #10000
.LBB4_50:                               //   in Loop: Header=BB4_4 Depth=1
	str	w8, [x20, #1748]
	b	.LBB4_56
.LBB4_51:                               //   in Loop: Header=BB4_4 Depth=1
	mov	w24, #10000
	b	.LBB4_56
.LBB4_52:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	w10, [x20, #1184]
	lsl	w8, w8, w10
	mov	w10, #100
	umull	x8, w8, w10
.LBB4_53:                               //   in Loop: Header=BB4_4 Depth=1
	mov	w9, w9
	udiv	x8, x8, x9
	mov	w9, #10000
	mul	w8, w8, w9
	mov	w9, #34564
	movk	w9, #1, lsl #16
	cmp	w8, w9
	b.lt	.LBB4_55
// %bb.54:                              //   in Loop: Header=BB4_4 Depth=1
	mov	w9, #34079
	movk	w9, #20971, lsl #16
	smull	x8, w8, w9
	lsr	x9, x8, #63
	asr	x8, x8, #37
	add	w24, w8, w9
	b	.LBB4_56
.LBB4_55:                               //   in Loop: Header=BB4_4 Depth=1
	mov	w24, #1000
.LBB4_56:                               //   in Loop: Header=BB4_4 Depth=1
	mov	w0, w24
	bl	__msecs_to_jiffies
	ldr	w8, [x20, #1772]
	mov	x26, x0
	sxtw	x21, w0
	cbnz	w8, .LBB4_60
// %bb.57:                              //   in Loop: Header=BB4_4 Depth=1
	adrp	x8, system_freezing_cnt
	ldr	w8, [x8, :lo12:system_freezing_cnt]
	cbnz	w8, .LBB4_129
.LBB4_58:                               //   in Loop: Header=BB4_4 Depth=1
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB4_60
// %bb.59:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	x8, [x28]
	cmp	x28, x8
	b.eq	.LBB4_82
.LBB4_60:                               //   in Loop: Header=BB4_4 Depth=1
	cmp	w26, #0                 // =0
	csinc	x26, x21, xzr, ne
.LBB4_61:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #1772]
	cbz	w8, .LBB4_63
// %bb.62:                              //   in Loop: Header=BB4_4 Depth=1
	str	wzr, [x20, #1772]
.LBB4_63:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	x0, [x20]
	mov	w1, #1
	mov	w2, wzr
	bl	__sb_start_write
	cbz	w0, .LBB4_72
// %bb.64:                              //   in Loop: Header=BB4_4 Depth=1
	cbz	w26, .LBB4_74
// %bb.65:                              //   in Loop: Header=BB4_4 Depth=1
	adrp	x8, system_freezing_cnt
	ldr	w8, [x8, :lo12:system_freezing_cnt]
	cbnz	w8, .LBB4_124
.LBB4_66:                               //   in Loop: Header=BB4_4 Depth=1
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB4_131
// %bb.67:                              //   in Loop: Header=BB4_4 Depth=1
	tbnz	w26, #31, .LBB4_77
// %bb.68:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w19, [x20, #2336]
	cbz	w19, .LBB4_70
// %bb.69:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	down_write
	mov	w1, #1
	mov	w4, #-1
	mov	x0, x20
	mov	w2, wzr
	mov	w3, wzr
	bl	f2fs_gc
	add	x8, x20, #2336          // =2336
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w19
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB4_70:                               //   in Loop: Header=BB4_4 Depth=1
	mov	x0, x20
	mov	w1, wzr
	bl	has_not_enough_free_secs
	adrp	x19, jiffies
	tbz	w0, #0, .LBB4_1
// %bb.71:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	down_write
	mov	w4, #-1
	mov	x0, x20
	mov	w1, wzr
	mov	w2, wzr
	mov	w3, wzr
	bl	f2fs_gc
	b	.LBB4_1
.LBB4_72:                               //   in Loop: Header=BB4_4 Depth=1
	ldrb	w8, [x20, #1232]
	tbz	w8, #4, .LBB4_3
// %bb.73:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #2248]
	add	w8, w8, #1              // =1
	str	w8, [x20, #2248]
	b	.LBB4_3
.LBB4_74:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	x8, [x20]
	ldr	w8, [x8, #616]
	cmp	w8, #1                  // =1
	b.lt	.LBB4_78
// %bb.75:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #1748]
	ldr	w9, [x20, #1752]
	ldr	w10, [x20, #1756]
	ldrb	w11, [x20, #1232]
	add	x12, x8, w24, uxtw
	add	w8, w8, w24
	cmp	x12, x9
	csel	w8, w9, w8, hi
	cmp	w24, w10
	csel	w24, w24, w8, eq
	tbz	w11, #4, .LBB4_2
.LBB4_76:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #2248]
	add	w8, w8, #1              // =1
	str	w8, [x20, #2248]
	b	.LBB4_2
.LBB4_77:                               //   in Loop: Header=BB4_4 Depth=1
	adrp	x1, .L.str.10
	mov	x0, x20
	add	x1, x1, :lo12:.L.str.10
	bl	f2fs_printk
	b	.LBB4_2
.LBB4_78:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	down_write_trylock
	cbz	w0, .LBB4_95
// %bb.79:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #1988]
	cmp	w8, #5                  // =5
	b.ne	.LBB4_87
// %bb.80:                              //   in Loop: Header=BB4_4 Depth=1
	ldrb	w8, [x20, #1232]
	tbnz	w8, #3, .LBB4_87
// %bb.81:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w24, [x20, #1744]
	b	.LBB4_112
.LBB4_82:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #2336]
	cmp	w8, #0                  // =0
	ccmp	w26, #0, #0, ne
	csinc	x26, x21, xzr, ne
	cbnz	w8, .LBB4_61
// %bb.83:                              //   in Loop: Header=BB4_4 Depth=1
	cbz	x26, .LBB4_61
// %bb.84:                              //   in Loop: Header=BB4_4 Depth=1
	sub	x0, x29, #48            // =48
	mov	w1, wzr
	stp	xzr, xzr, [x29, #-24]
	stp	xzr, xzr, [x29, #-40]
	stur	xzr, [x29, #-48]
	bl	init_wait_entry
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	sub	x1, x29, #48            // =48
	mov	w2, #1
	bl	prepare_to_wait_event
	ldr	w8, [x20, #1772]
	cbz	w8, .LBB4_96
.LBB4_85:                               //   in Loop: Header=BB4_4 Depth=1
	cmp	x21, #0                 // =0
	csinc	x21, x21, xzr, ne
.LBB4_86:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	sub	x1, x29, #48            // =48
	bl	finish_wait
	mov	x26, x21
	b	.LBB4_61
.LBB4_87:                               //   in Loop: Header=BB4_4 Depth=1
	ldrb	w8, [x20, #1912]
	cbnz	w8, .LBB4_92
// %bb.88:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #1232]
	tbnz	w8, #0, .LBB4_92
// %bb.89:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w9, [x20, #1748]
	ldr	w10, [x20, #1752]
	ldr	w11, [x20, #1756]
	add	x12, x9, w24, uxtw
	add	w9, w9, w24
	cmp	x12, x10
	csel	w9, w10, w9, hi
	cmp	w24, w11
	csel	w24, w24, w9, eq
	tbz	w8, #4, .LBB4_91
// %bb.90:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #2244]
	add	w8, w8, #1              // =1
	str	w8, [x20, #2244]
.LBB4_91:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	up_write
	b	.LBB4_2
.LBB4_92:                               //   in Loop: Header=BB4_4 Depth=1
	ldr	x9, [x20, #136]
	ldr	w10, [x20, #1236]
	mov	w12, #34079
	movk	w12, #20971, lsl #16
	ldr	x8, [x9]
	add	w11, w10, w10, lsl #2
	lsl	w11, w11, #3
	umull	x11, w11, w12
	ldr	w8, [x8, #16]
	sub	w8, w10, w8
	lsr	x10, x11, #37
	cmp	w8, w10
	b.ls	.LBB4_110
// %bb.93:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	x10, [x9, #8]
	ldr	w9, [x9, #104]
	ldr	w10, [x10, #4]
	subs	w9, w10, w9
	b.hs	.LBB4_105
// %bb.94:                              //   in Loop: Header=BB4_4 Depth=1
	mov	w9, wzr
	b	.LBB4_106
.LBB4_95:                               //   in Loop: Header=BB4_4 Depth=1
	ldrb	w8, [x20, #1232]
	tbz	w8, #4, .LBB4_2
	b	.LBB4_76
.LBB4_96:                               //   in Loop: Header=BB4_4 Depth=1
	mov	x26, x0
.LBB4_97:                               //   Parent Loop BB4_4 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	adrp	x8, system_freezing_cnt
	ldr	w8, [x8, :lo12:system_freezing_cnt]
	cbnz	w8, .LBB4_104
.LBB4_98:                               //   in Loop: Header=BB4_97 Depth=2
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB4_85
// %bb.99:                              //   in Loop: Header=BB4_97 Depth=2
	ldr	x8, [x28]
	cmp	x28, x8
	b.ne	.LBB4_85
// %bb.100:                             //   in Loop: Header=BB4_97 Depth=2
	ldr	w8, [x20, #2336]
	cmp	w8, #0                  // =0
	ccmp	x21, #0, #0, ne
	csinc	x21, x21, xzr, ne
	cbnz	w8, .LBB4_86
// %bb.101:                             //   in Loop: Header=BB4_97 Depth=2
	cbz	x21, .LBB4_86
// %bb.102:                             //   in Loop: Header=BB4_97 Depth=2
	cbnz	x26, .LBB4_61
// %bb.103:                             //   in Loop: Header=BB4_97 Depth=2
	mov	x0, x21
	bl	schedule_timeout
	mov	x21, x0
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	sub	x1, x29, #48            // =48
	mov	w2, #1
	bl	prepare_to_wait_event
	ldr	w8, [x20, #1772]
	mov	x26, x0
	cbz	w8, .LBB4_97
	b	.LBB4_85
.LBB4_104:                              //   in Loop: Header=BB4_97 Depth=2
	mov	x0, x25
	bl	freezing_slow_path
	tbz	w0, #0, .LBB4_98
	b	.LBB4_85
.LBB4_105:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w10, [x20, #1184]
	lsl	w9, w9, w10
.LBB4_106:                              //   in Loop: Header=BB4_4 Depth=1
	add	w8, w8, w8, lsl #2
	mov	w10, #34079
	lsl	w8, w8, #3
	movk	w10, #20971, lsl #16
	umull	x8, w8, w10
	lsr	x8, x8, #37
	cmp	w9, w8
	b.hs	.LBB4_110
// %bb.107:                             //   in Loop: Header=BB4_4 Depth=1
	ldr	w9, [x20, #1756]
	ldr	w8, [x20, #1748]
	cmp	w24, w9
	b.ne	.LBB4_109
// %bb.108:                             //   in Loop: Header=BB4_4 Depth=1
	ldr	w24, [x20, #1752]
.LBB4_109:                              //   in Loop: Header=BB4_4 Depth=1
	mov	w9, w24
	sub	x9, x9, x8
	sub	w10, w24, w8
	cmp	x9, x8
	csel	w24, w8, w10, lt
	b	.LBB4_112
.LBB4_110:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #1756]
	cmp	w24, w8
	b.eq	.LBB4_112
// %bb.111:                             //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #1748]
	ldr	w9, [x20, #1752]
	add	x10, x8, w24, uxtw
	add	w8, w8, w24
	cmp	x10, x9
	csel	w24, w9, w8, hi
.LBB4_112:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	x8, [x20, #2120]
	ldr	w9, [x8, #188]
	add	w9, w9, #1              // =1
	str	w9, [x8, #188]
	ldr	x12, [x20, #136]
	ldr	w9, [x20, #1236]
	ldr	w11, [x20, #1168]
	ldr	w10, [x20, #1240]
	ldr	x13, [x20, #2120]
	ldr	x15, [x12, #8]
	mul	x14, x11, x9
	sub	w9, w9, w10
	ldr	w10, [x12, #100]
	ldr	w8, [x20, #1508]
	ldr	w4, [x13, #188]
	mul	x11, x9, x11
	ldr	w5, [x15, #8]
	ldr	w9, [x20, #1192]
	lsr	x2, x14, #20
	cmn	w10, #1                 // =1
	lsr	x3, x11, #20
	b.eq	.LBB4_114
// %bb.113:                             //   in Loop: Header=BB4_4 Depth=1
	udiv	w6, w10, w9
	b	.LBB4_115
.LBB4_114:                              //   in Loop: Header=BB4_4 Depth=1
	mov	w6, #-1
.LBB4_115:                              //   in Loop: Header=BB4_4 Depth=1
	cmp	w8, #2                  // =2
	ldr	w8, [x20, #1188]
	ldrsw	x10, [x20, #1284]
	ldr	w11, [x20, #1184]
	ldrsw	x12, [x20, #1272]
	mul	w8, w8, w9
	sub	x8, x8, #1              // =1
	add	x10, x8, x10
	add	x8, x8, x12
	asr	x10, x10, x11
	asr	x8, x8, x11
	adrp	x1, .L.str.11
	udiv	w7, w10, w9
	udiv	w8, w8, w9
	mov	x0, x20
	add	x1, x1, :lo12:.L.str.11
	cset	w21, eq
	str	w8, [sp]
	bl	f2fs_printk
	mov	w2, #1
	mov	w4, #-1
	mov	x0, x20
	mov	w1, w21
	mov	w3, wzr
	bl	f2fs_gc
	cbz	w0, .LBB4_117
// %bb.116:                             //   in Loop: Header=BB4_4 Depth=1
	ldr	w24, [x20, #1756]
.LBB4_117:                              //   in Loop: Header=BB4_4 Depth=1
	ldr	x8, [x20, #136]
	ldr	x21, [x20]
	ldp	x8, x9, [x8, #8]
	ldr	w27, [x8, #4]
	adrp	x8, __tracepoint_f2fs_background_gc+8
	ldr	w26, [x9, #140]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_background_gc+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB4_123
// %bb.118:                             //   in Loop: Header=BB4_4 Depth=1
	adrp	x9, cpu_number
	//APP
	.if 1 == 1
661:
	mrs x8, tpidr_el1
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 11
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	mrs x8, tpidr_el2
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	add	x9, x9, :lo12:cpu_number
	ldr	w8, [x8, x9]
	adrp	x10, __cpu_online_mask
	add	x10, x10, :lo12:__cpu_online_mask
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x10, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB4_123
// %bb.119:                             //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x25, #24]
	add	w8, w8, #1              // =1
	str	w8, [x25, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_background_gc+32
	ldr	x19, [x8, :lo12:__tracepoint_f2fs_background_gc+32]
	cbz	x19, .LBB4_121
.LBB4_120:                              //   Parent Loop BB4_4 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldp	x8, x0, [x19]
	mov	x1, x21
	mov	w2, w24
	mov	w3, w26
	mov	w4, w27
	blr	x8
	ldr	x8, [x19, #24]!
	cbnz	x8, .LBB4_120
.LBB4_121:                              //   in Loop: Header=BB4_4 Depth=1
	//APP
	//NO_APP
	ldr	x8, [x25, #24]
	adrp	x19, jiffies
	subs	x8, x8, #1              // =1
	str	w8, [x25, #24]
	b.eq	.LBB4_130
// %bb.122:                             //   in Loop: Header=BB4_4 Depth=1
	ldr	x8, [x25, #24]
	cbz	x8, .LBB4_130
.LBB4_123:                              //   in Loop: Header=BB4_4 Depth=1
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_balance_fs_bg
	b	.LBB4_2
.LBB4_124:                              //   in Loop: Header=BB4_4 Depth=1
	mov	x0, x25
	bl	freezing_slow_path
	tbz	w0, #0, .LBB4_66
// %bb.125:                             //   in Loop: Header=BB4_4 Depth=1
	mov	w0, wzr
	bl	__refrigerator
	tbz	w0, #0, .LBB4_66
// %bb.126:                             //   in Loop: Header=BB4_4 Depth=1
	ldrb	w8, [x20, #1232]
	mov	w22, w24
	tbz	w8, #4, .LBB4_128
// %bb.127:                             //   in Loop: Header=BB4_4 Depth=1
	ldr	w8, [x20, #2248]
	add	w8, w8, #1              // =1
	str	w8, [x20, #2248]
.LBB4_128:                              //   in Loop: Header=BB4_4 Depth=1
	adrp	x19, jiffies
	mov	w24, w22
	b	.LBB4_2
.LBB4_129:                              //   in Loop: Header=BB4_4 Depth=1
	mov	x0, x25
	bl	freezing_slow_path
	tbz	w0, #0, .LBB4_58
	b	.LBB4_60
.LBB4_130:                              //   in Loop: Header=BB4_4 Depth=1
	bl	preempt_schedule_notrace
	b	.LBB4_123
.LBB4_131:
	ldr	x0, [x20]
	mov	w1, #1
	bl	__sb_end_write
.LBB4_132:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB4_134
// %bb.133:
	ldp	x20, x19, [sp, #208]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #192]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #176]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #160]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #144]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #128]    // 16-byte Folded Reload
	mov	w0, wzr
	add	sp, sp, #224            // =224
	ret
.LBB4_134:
	bl	__stack_chk_fail
.Lfunc_end4:
	.size	gc_thread_func, .Lfunc_end4-gc_thread_func
                                        // -- End function
	.globl	f2fs_stop_gc_thread     // -- Begin function f2fs_stop_gc_thread
	.p2align	2
	.type	f2fs_stop_gc_thread,@function
f2fs_stop_gc_thread:                    // @f2fs_stop_gc_thread
.Lf2fs_stop_gc_thread$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x19, x0
	ldr	x0, [x0, #1712]
	mov	x29, sp
	cbz	x0, .LBB5_4
// %bb.1:
	bl	kthread_stop
	add	x0, x19, #1776          // =1776
	mov	w1, #3
	mov	w2, wzr
	mov	x3, xzr
	str	xzr, [x19, #1712]
	bl	__wake_up
	add	x20, x19, #1800         // =1800
.LBB5_2:                                // =>This Inner Loop Header: Depth=1
	mov	x0, x20
	bl	blk_busyidle_event_unsubscribe
	cbnz	w0, .LBB5_2
// %bb.3:
	add	x0, x19, #1856          // =1856
	bl	del_timer_sync
.LBB5_4:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end5:
	.size	f2fs_stop_gc_thread, .Lfunc_end5-f2fs_stop_gc_thread
                                        // -- End function
	.globl	f2fs_start_bidx_of_node // -- Begin function f2fs_start_bidx_of_node
	.p2align	2
	.type	f2fs_start_bidx_of_node,@function
f2fs_start_bidx_of_node:                // @f2fs_start_bidx_of_node
.Lf2fs_start_bidx_of_node$local:
// %bb.0:
	cbz	w0, .LBB6_14
// %bb.1:
	cmp	w0, #2                  // =2
	b.hi	.LBB6_3
// %bb.2:
	sub	w8, w0, #1              // =1
	b	.LBB6_6
.LBB6_3:
	cmp	w0, #2040               // =2040
	b.hi	.LBB6_5
// %bb.4:
	mov	w9, #37367
	sub	w8, w0, #4              // =4
	movk	w9, #321, lsl #16
	umull	x9, w8, w9
	lsr	x9, x9, #32
	sub	w8, w8, w9
	add	w8, w9, w8, lsr #1
	sub	w8, w0, w8, lsr #9
	sub	w8, w8, #2              // =2
	b	.LBB6_6
.LBB6_5:
	mov	w9, #37367
	sub	w8, w0, #2043           // =2043
	movk	w9, #321, lsl #16
	umull	x9, w8, w9
	lsr	x9, x9, #32
	sub	w8, w8, w9
	add	w8, w9, w8, lsr #1
	sub	w8, w0, w8, lsr #9
	sub	w8, w8, #5              // =5
.LBB6_6:
	ldrh	w9, [x1]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB6_9
// %bb.7:
	ldr	x11, [x1, #736]
	mov	w10, #1018
	tbz	w11, #31, .LBB6_10
// %bb.8:
	ldr	w11, [x1, #1292]
	neg	w11, w11
	and	w10, w11, w10
	b	.LBB6_10
.LBB6_9:
	mov	w10, #1018
.LBB6_10:
	ldr	w11, [x1, #1184]
	ldr	w12, [x1, #1192]
	mul	w8, w10, w8
	mov	w10, #923
	cmp	w9, #8, lsl #12         // =32768
	add	w11, w12, w11, asr #2
	sub	w9, w10, w11
	b.ne	.LBB6_13
// %bb.11:
	ldr	x10, [x1, #736]
	tbz	w10, #31, .LBB6_13
// %bb.12:
	ldr	w10, [x1, #1292]
	neg	w10, w10
	and	w9, w9, w10
.LBB6_13:
	add	w0, w9, w8
.LBB6_14:
	ret
.Lfunc_end6:
	.size	f2fs_start_bidx_of_node, .Lfunc_end6-f2fs_start_bidx_of_node
                                        // -- End function
	.globl	f2fs_gc                 // -- Begin function f2fs_gc
	.p2align	2
	.type	f2fs_gc,@function
f2fs_gc:                                // @f2fs_gc
.Lf2fs_gc$local:
// %bb.0:
	sub	sp, sp, #288            // =288
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #192]    // 16-byte Folded Spill
	add	x29, sp, #192           // =192
	stp	x28, x27, [sp, #208]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #224]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #240]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #256]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #272]    // 16-byte Folded Spill
	str	w2, [sp, #96]           // 4-byte Folded Spill
	sub	x9, x29, #88            // =88
	stur	x8, [x29, #-16]
	mov	x8, #13469017440256
	stur	w4, [x29, #-20]
	stp	xzr, xzr, [x29, #-40]
	stp	xzr, xzr, [x29, #-56]
	stp	x9, x9, [x29, #-88]
	stp	x8, xzr, [x29, #-72]
	ldr	x8, [x0, #2016]
	mov	w23, w3
	mov	x19, x0
	str	w4, [sp, #56]           // 4-byte Folded Spill
	str	x8, [sp, #72]           // 8-byte Folded Spill
	str	w1, [sp, #92]           // 4-byte Folded Spill
	and	w22, w1, #0x1
	bl	sched_clock
	str	x0, [sp, #48]           // 8-byte Folded Spill
	ldr	x28, [x19]
	ldrsw	x21, [x19, #1284]
	ldrsw	x20, [x19, #1272]
	ldrsw	x26, [x19, #1296]
	ldr	x8, [x19, #136]
	adrp	x10, __tracepoint_f2fs_gc_begin+8
	ldp	x9, x11, [x8, #8]
	ldr	w24, [x8, #100]
	ldp	w13, w27, [x9, #4]
	ldr	w11, [x11, #140]
	ldr	w8, [x10, :lo12:__tracepoint_f2fs_gc_begin+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB7_6
// %bb.1:
	adrp	x9, cpu_number
	//APP
	.if 1 == 1
661:
	mrs x8, tpidr_el1
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 11
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	mrs x8, tpidr_el2
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	add	x9, x9, :lo12:cpu_number
	ldr	w8, [x8, x9]
	adrp	x10, __cpu_online_mask
	add	x10, x10, :lo12:__cpu_online_mask
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x10, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB7_6
// %bb.2:
	str	x19, [sp, #40]          // 8-byte Folded Spill
	stur	w23, [x29, #-92]        // 4-byte Folded Spill
	str	w22, [sp, #68]          // 4-byte Folded Spill
	//APP
	mrs x9, sp_el0
	//NO_APP
	ldr	w8, [x9, #24]
	str	x9, [sp, #80]           // 8-byte Folded Spill
	add	w8, w8, #1              // =1
	str	w8, [x9, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_gc_begin+32
	ldp	w25, w10, [sp, #92]     // 8-byte Folded Reload
	ldr	x23, [x8, :lo12:__tracepoint_f2fs_gc_begin+32]
	cbz	x23, .LBB7_4
.LBB7_3:                                // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	and	w2, w25, #0x1
	and	w3, w10, #0x1
	mov	x1, x28
	mov	x4, x21
	mov	x5, x20
	mov	x6, x26
	mov	w7, w27
	str	w11, [sp, #16]
	str	w24, [sp, #8]
	str	w13, [sp]
	mov	w19, w13
	mov	w22, w11
	blr	x8
	ldr	w10, [sp, #96]          // 4-byte Folded Reload
	ldr	x8, [x23, #24]!
	mov	w11, w22
	mov	w13, w19
	cbnz	x8, .LBB7_3
.LBB7_4:
	//APP
	//NO_APP
	ldr	x9, [sp, #80]           // 8-byte Folded Reload
	ldr	x8, [x9, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x9, #24]
	ldr	w22, [sp, #68]          // 4-byte Folded Reload
	ldur	w23, [x29, #-92]        // 4-byte Folded Reload
	ldr	x19, [sp, #40]          // 8-byte Folded Reload
	b.eq	.LBB7_112
// %bb.5:
	ldr	x8, [x9, #24]
	cbz	x8, .LBB7_112
.LBB7_6:
	ldr	w8, [x19, #1424]
	ldr	x10, [x19, #80]
	mov	w9, #2
	mov	w11, #4
	tst	w8, #0x1000
	csel	w8, w11, w9, eq
	tst	x10, #0x2
	csinc	w8, w8, wzr, eq
	stur	w8, [x29, #-56]
	ldr	x8, [x19]
	str	xzr, [x19, #2024]
	ldrb	w8, [x8, #83]
	tbz	w8, #6, .LBB7_109
// %bb.7:
	ldr	x26, [sp, #72]          // 8-byte Folded Reload
	mov	w24, wzr
	mov	w27, wzr
	mov	w21, wzr
	mov	w25, wzr
	mov	w20, wzr
	and	w8, w23, #0x1
	str	w8, [sp, #80]           // 4-byte Folded Spill
	b	.LBB7_10
.LBB7_8:                                //   in Loop: Header=BB7_10 Depth=1
	mov	w20, wzr
.LBB7_9:                                //   in Loop: Header=BB7_10 Depth=1
	ldr	x8, [x19]
	mov	w21, #1
	ldrb	w8, [x8, #83]
	tbz	w8, #6, .LBB7_108
.LBB7_10:                               // =>This Inner Loop Header: Depth=1
	ldr	x8, [x19, #224]
	stur	w20, [x29, #-92]        // 4-byte Folded Spill
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB7_107
// %bb.11:                              //   in Loop: Header=BB7_10 Depth=1
	cbz	w22, .LBB7_20
.LBB7_12:                               //   in Loop: Header=BB7_10 Depth=1
	ldr	x8, [x19, #136]
	mov	w23, w21
.LBB7_13:                               //   in Loop: Header=BB7_10 Depth=1
	ldr	x8, [x8]
	add	x21, x8, #72            // =72
	mov	x0, x21
	bl	down_write
	ldr	x8, [x19, #136]
	sub	x1, x29, #20            // =20
	mov	w3, #8
	mov	x0, x19
	ldr	x8, [x8, #16]
	mov	w2, w22
	mov	w4, wzr
	mov	x5, xzr
	ldr	x8, [x8]
	ldr	x8, [x8]
	blr	x8
	mov	w28, w0
	mov	x0, x21
	bl	up_write
	cbnz	w28, .LBB7_69
// %bb.14:                              //   in Loop: Header=BB7_10 Depth=1
	ldrb	w8, [x19, #2100]
	ldur	w1, [x29, #-20]
	cbz	w8, .LBB7_16
// %bb.15:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	w8, [x19, #2096]
	cmp	w1, w8
	b.ne	.LBB7_50
.LBB7_16:                               //   in Loop: Header=BB7_10 Depth=1
	ldr	w4, [sp, #80]           // 4-byte Folded Reload
	sub	x2, x29, #88            // =88
	mov	x0, x19
	mov	w3, w22
	bl	do_garbage_collect
	ldur	w20, [x29, #-92]        // 4-byte Folded Reload
	cmp	w22, #1                 // =1
	mov	w28, w0
	b.ne	.LBB7_19
// %bb.17:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	w9, [x19, #1192]
	cmp	w28, w9
	b.ne	.LBB7_31
// %bb.18:                              //   in Loop: Header=BB7_10 Depth=1
	add	w20, w20, #1            // =1
	b	.LBB7_32
.LBB7_19:                               //   in Loop: Header=BB7_10 Depth=1
	add	w25, w28, w25
	ldr	w8, [sp, #92]           // 4-byte Folded Reload
	tbz	w8, #0, .LBB7_38
	b	.LBB7_65
.LBB7_20:                               //   in Loop: Header=BB7_10 Depth=1
	mov	x0, x19
	mov	w1, wzr
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB7_25
// %bb.21:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	x8, [x19, #136]
	ldr	x8, [x8, #16]
	ldr	w8, [x8, #140]
	cbz	w8, .LBB7_24
// %bb.22:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	x8, [x19, #80]
	tbnz	w8, #8, .LBB7_24
// %bb.23:                              //   in Loop: Header=BB7_10 Depth=1
	sub	x1, x29, #56            // =56
	mov	x0, x19
	bl	f2fs_write_checkpoint
	cbnz	w0, .LBB7_106
.LBB7_24:                               //   in Loop: Header=BB7_10 Depth=1
	mov	x0, x19
	mov	w1, wzr
	bl	has_not_enough_free_secs
	and	w22, w0, #0x1
.LBB7_25:                               //   in Loop: Header=BB7_10 Depth=1
	ldr	w8, [sp, #96]           // 4-byte Folded Reload
	cbnz	w22, .LBB7_27
// %bb.26:                              //   in Loop: Header=BB7_10 Depth=1
	tbz	w8, #0, .LBB7_101
.LBB7_27:                               //   in Loop: Header=BB7_10 Depth=1
	cbnz	w22, .LBB7_12
// %bb.28:                              //   in Loop: Header=BB7_10 Depth=1
	adrp	x8, write_opt
	ldr	w8, [x8, :lo12:write_opt]
	cbz	w8, .LBB7_12
// %bb.29:                              //   in Loop: Header=BB7_10 Depth=1
	mov	x0, x19
	bl	get_max_reclaimable_segments
	ldr	x8, [x19, #136]
	mov	w10, #100
	mul	w10, w0, w10
	ldr	x9, [x8, #8]
	ldr	w9, [x9, #4]
	udiv	w9, w10, w9
	cmp	w9, #29                 // =29
	b.ls	.LBB7_105
// %bb.30:                              //   in Loop: Header=BB7_10 Depth=1
	mov	w23, w21
	mov	w22, wzr
	b	.LBB7_13
.LBB7_31:                               //   in Loop: Header=BB7_10 Depth=1
	cbz	w28, .LBB7_51
.LBB7_32:                               //   in Loop: Header=BB7_10 Depth=1
	ldr	x8, [x19, #2016]
	add	w25, w28, w25
	cmp	x8, x26
	b.hi	.LBB7_34
// %bb.33:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	x9, [x19, #2024]
	cbz	x9, .LBB7_35
.LBB7_34:                               //   in Loop: Header=BB7_10 Depth=1
	add	w27, w27, #1            // =1
.LBB7_35:                               //   in Loop: Header=BB7_10 Depth=1
	add	w24, w24, #1            // =1
	cbz	w28, .LBB7_37
// %bb.36:                              //   in Loop: Header=BB7_10 Depth=1
	mov	w9, #-1
	str	w9, [x19, #1984]
.LBB7_37:                               //   in Loop: Header=BB7_10 Depth=1
	mov	x26, x8
	ldr	w8, [sp, #92]           // 4-byte Folded Reload
	tbnz	w8, #0, .LBB7_65
.LBB7_38:                               //   in Loop: Header=BB7_10 Depth=1
	mov	x0, x19
	mov	w1, w20
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB7_65
// %bb.39:                              //   in Loop: Header=BB7_10 Depth=1
	cmp	w27, #17                // =17
	b.lo	.LBB7_44
// %bb.40:                              //   in Loop: Header=BB7_10 Depth=1
	lsl	w8, w27, #1
	cmp	w8, w24
	b.lo	.LBB7_44
// %bb.41:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	x8, [sp, #72]           // 8-byte Folded Reload
	subs	x8, x26, x8
	b.ls	.LBB7_97
// %bb.42:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	x9, [x19, #2024]
	cmp	x8, x9
	b.ls	.LBB7_97
// %bb.43:                              //   in Loop: Header=BB7_10 Depth=1
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_drop_inmem_pages_all
.LBB7_44:                               //   in Loop: Header=BB7_10 Depth=1
	mov	w8, #-1
	stur	w8, [x29, #-20]
	ldr	x8, [x19, #136]
	ldr	x9, [x8, #16]
	ldr	w9, [x9, #140]
	cbz	w9, .LBB7_9
// %bb.45:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	w8, [x8, #100]
	cmn	w8, #1                  // =1
	b.eq	.LBB7_47
// %bb.46:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	w9, [x19, #1192]
	udiv	w1, w8, w9
	b	.LBB7_48
.LBB7_47:                               //   in Loop: Header=BB7_10 Depth=1
	mov	w1, #-1
.LBB7_48:                               //   in Loop: Header=BB7_10 Depth=1
	mov	x0, x19
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB7_9
// %bb.49:                              //   in Loop: Header=BB7_10 Depth=1
	sub	x1, x29, #56            // =56
	mov	x0, x19
	bl	f2fs_write_checkpoint
	cbz	w0, .LBB7_8
	b	.LBB7_102
.LBB7_50:                               //   in Loop: Header=BB7_10 Depth=1
	mov	w8, #-1
	strb	wzr, [x19, #2100]
	str	xzr, [x19, #2088]
	str	w8, [x19, #2096]
	b	.LBB7_16
.LBB7_51:                               //   in Loop: Header=BB7_10 Depth=1
	ldrb	w8, [x19, #2100]
	cbz	w8, .LBB7_55
// %bb.52:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	x2, [x19, #2088]
	mov	x8, #23593
	movk	x8, #49807, lsl #16
	movk	x8, #10485, lsl #32
	mov	x10, #36700
	movk	x8, #36700, lsl #48
	movk	x10, #62914, lsl #16
	mul	x8, x2, x8
	movk	x10, #23592, lsl #32
	ror	x8, x8, #2
	movk	x10, #655, lsl #48
	cmp	x8, x10
	b.hi	.LBB7_61
// %bb.53:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	x8, [x19, #136]
	ldur	w5, [x29, #-20]
	mov	w12, #40
	cmp	w9, #2                  // =2
	ldr	x10, [x8]
	mul	x12, x5, x12
	ldr	x11, [x10, #120]
	ldr	w11, [x11, x12]
	and	w6, w11, #0x3f
	b.hs	.LBB7_56
// %bb.54:                              //   in Loop: Header=BB7_10 Depth=1
	ubfx	w7, w11, #6, #10
	b	.LBB7_60
.LBB7_55:                               //   in Loop: Header=BB7_10 Depth=1
	mov	w8, #1
	strb	w8, [x19, #2100]
	ldur	w8, [x29, #-20]
	mov	w9, #2
	str	x9, [x19, #2088]
	str	w8, [x19, #2096]
	b	.LBB7_32
.LBB7_56:                               //   in Loop: Header=BB7_10 Depth=1
	ldr	x10, [x10, #128]
	cmn	w5, #1                  // =1
	b.eq	.LBB7_58
// %bb.57:                              //   in Loop: Header=BB7_10 Depth=1
	udiv	w9, w5, w9
	b	.LBB7_59
.LBB7_58:                               //   in Loop: Header=BB7_10 Depth=1
	mov	w9, #-1
.LBB7_59:                               //   in Loop: Header=BB7_10 Depth=1
	ldr	w7, [x10, x9, lsl #2]
.LBB7_60:                               //   in Loop: Header=BB7_10 Depth=1
	ldp	x9, x10, [x8, #8]
	ldr	w8, [x8, #100]
	adrp	x1, .L.str.5
	adrp	x3, .L.str.3
	ldr	w9, [x9, #4]
	ldr	w10, [x10, #140]
	mov	w4, #3378
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.5
	add	x3, x3, :lo12:.L.str.3
	str	w8, [sp, #16]
	str	w10, [sp, #8]
	str	w9, [sp]
                                        // kill: def $w5 killed $w5 killed $x5
	bl	f2fs_printk
	ldr	x2, [x19, #2088]
.LBB7_61:                               //   in Loop: Header=BB7_10 Depth=1
	add	x8, x2, #1              // =1
	mov	w9, #10001
	cmp	x8, x9
	str	x8, [x19, #2088]
	b.lo	.LBB7_32
// %bb.62:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	x0, [x19, #2104]
	cbnz	x0, .LBB7_64
// %bb.63:                              //   in Loop: Header=BB7_10 Depth=1
	ldr	x8, [x19, #136]
	mov	w1, #3520
	mov	w2, #-1
	ldr	w8, [x8, #96]
	add	x8, x8, #63             // =63
	lsr	x8, x8, #3
	and	x0, x8, #0x3ffffff8
	bl	kvmalloc_node
	str	x0, [x19, #2104]
	cbz	x0, .LBB7_32
.LBB7_64:                               //   in Loop: Header=BB7_10 Depth=1
	ldur	w8, [x29, #-20]
	mov	w10, #1
	lsr	x9, x8, #3
	and	x9, x9, #0x1ffffff8
	add	x9, x0, x9
	lsl	x8, x10, x8
	//APP
	// atomic64_or
	prfm	pstl1strm, [x9]
1:	ldxr	x10, [x9]
	orr	x10, x10, x8
	stxr	w11, x10, [x9]
	cbnz	w11, 1b
	//NO_APP
	b	.LBB7_32
.LBB7_65:
	mov	w26, w25
	str	w22, [sp, #68]          // 4-byte Folded Spill
	stur	w20, [x29, #-92]        // 4-byte Folded Spill
.LBB7_66:
	mov	w28, wzr
.LBB7_67:
	mov	w21, #1
.LBB7_68:
	ldr	w9, [sp, #56]           // 4-byte Folded Reload
	b	.LBB7_70
.LBB7_69:
	ldr	w9, [sp, #56]           // 4-byte Folded Reload
	mov	w26, w25
	str	w22, [sp, #68]          // 4-byte Folded Spill
	mov	w21, w23
.LBB7_70:
	ldr	x8, [x19, #136]
	adrp	x10, __tracepoint_f2fs_gc_end+8
	ldr	x8, [x8]
	str	wzr, [x8, #196]
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
	str	w9, [x8, #200]
	ldr	x27, [x19]
	ldrsw	x22, [x19, #1284]
	ldrsw	x25, [x19, #1272]
	ldrsw	x23, [x19, #1296]
	ldr	x8, [x19, #136]
	ldp	x9, x11, [x8, #8]
	ldr	w12, [x8, #100]
	ldp	w14, w13, [x9, #4]
	ldr	w11, [x11, #140]
	ldr	w8, [x10, :lo12:__tracepoint_f2fs_gc_end+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB7_77
// %bb.71:
	adrp	x9, cpu_number
	//APP
	.if 1 == 1
661:
	mrs x8, tpidr_el1
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 11
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	mrs x8, tpidr_el2
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	add	x9, x9, :lo12:cpu_number
	ldr	w8, [x8, x9]
	adrp	x10, __cpu_online_mask
	add	x10, x10, :lo12:__cpu_online_mask
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x10, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB7_77
// %bb.72:
	str	x19, [sp, #40]          // 8-byte Folded Spill
	str	w21, [sp, #72]          // 4-byte Folded Spill
	//APP
	mrs x9, sp_el0
	//NO_APP
	ldr	w8, [x9, #24]
	str	x9, [sp, #56]           // 8-byte Folded Spill
	add	w8, w8, #1              // =1
	str	w8, [x9, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_gc_end+32
	ldr	x21, [x8, :lo12:__tracepoint_f2fs_gc_end+32]
	ldur	w24, [x29, #-92]        // 4-byte Folded Reload
	cbz	x21, .LBB7_75
// %bb.73:
	mov	w20, w26
	str	w28, [sp, #96]          // 4-byte Folded Spill
	str	w12, [sp, #80]          // 4-byte Folded Spill
.LBB7_74:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	mov	x1, x27
	mov	w2, w28
	mov	w3, w20
	mov	w4, w24
	mov	x5, x22
	mov	x6, x25
	mov	x7, x23
	str	w11, [sp, #24]
	str	w12, [sp, #16]
	str	w14, [sp, #8]
	str	w13, [sp]
	mov	w19, w13
	mov	w28, w14
	mov	w26, w11
	blr	x8
	mov	w14, w28
	ldr	w28, [sp, #96]          // 4-byte Folded Reload
	ldr	w12, [sp, #80]          // 4-byte Folded Reload
	ldr	x8, [x21, #24]!
	mov	w11, w26
	mov	w13, w19
	cbnz	x8, .LBB7_74
.LBB7_75:
	//APP
	//NO_APP
	ldr	x9, [sp, #56]           // 8-byte Folded Reload
	ldr	x8, [x9, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x9, #24]
	ldr	w21, [sp, #72]          // 4-byte Folded Reload
	ldr	x19, [sp, #40]          // 8-byte Folded Reload
	b.eq	.LBB7_113
// %bb.76:
	ldr	x8, [x9, #24]
	cbz	x8, .LBB7_113
.LBB7_77:
	add	x0, x19, #1664          // =1664
	bl	up_write
	cbz	w21, .LBB7_88
// %bb.78:
	add	x21, x19, #2296         // =2296
	mov	x0, x21
	bl	mutex_lock
	ldr	x20, [sp, #48]          // 8-byte Folded Reload
	ldr	w22, [sp, #68]          // 4-byte Folded Reload
	cbz	x20, .LBB7_82
// %bb.79:
	cmp	w22, #1                 // =1
	b.ne	.LBB7_82
// %bb.80:
	bl	sched_clock
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB7_82
// %bb.81:
	ldr	x9, [x8, #144]
	sub	x10, x0, x20
	add	x9, x10, x9
	str	x9, [x8, #144]
.LBB7_82:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB7_84
// %bb.83:
	add	x8, x8, w22, sxtw #2
	ldr	w9, [x8, #80]
	add	w9, w9, #1              // =1
	str	w9, [x8, #80]
.LBB7_84:
	cbz	w28, .LBB7_87
// %bb.85:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB7_87
// %bb.86:
	add	x8, x8, w22, sxtw #2
	ldr	w9, [x8, #88]
	add	w9, w9, #1              // =1
	str	w9, [x8, #88]
.LBB7_87:
	mov	x0, x21
	bl	mutex_unlock
.LBB7_88:
	ldur	x21, [x29, #-88]
	sub	x20, x29, #88           // =88
	cmp	x21, x20
	b.eq	.LBB7_93
// %bb.89:
	mov	x22, #256
	movk	x22, #57005, lsl #48
	add	x23, x20, #16           // =16
	adrp	x24, f2fs_inode_entry_slab
	b	.LBB7_91
.LBB7_90:                               //   in Loop: Header=BB7_91 Depth=1
	add	x8, x22, #34            // =34
	stp	x22, x8, [x21]
	ldr	x0, [x24, :lo12:f2fs_inode_entry_slab]
	mov	x1, x21
	bl	kmem_cache_free
	cmp	x25, x20
	mov	x21, x25
	b.eq	.LBB7_93
.LBB7_91:                               // =>This Inner Loop Header: Depth=1
	ldr	x8, [x21, #16]
	ldr	x25, [x21]
	mov	x0, x23
	ldr	x1, [x8, #64]
	bl	radix_tree_delete
	ldr	x0, [x21, #16]
	bl	iput
	mov	x0, x21
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB7_90
// %bb.92:                              //   in Loop: Header=BB7_91 Depth=1
	ldp	x9, x8, [x21]
	str	x8, [x9, #8]
	str	x9, [x8]
	b	.LBB7_90
.LBB7_93:
	ldur	w10, [x29, #-92]        // 4-byte Folded Reload
	cmp	w28, #0                 // =0
	cset	w9, ne
	ldr	x0, [x19, #2104]
	cmp	w10, #0                 // =0
	ldr	w10, [sp, #92]          // 4-byte Folded Reload
	mov	w8, #-11
	csel	w8, w8, wzr, eq
	orn	w9, w9, w10
	tst	w9, #0x1
	csel	w20, w28, w8, ne
	cbnz	x0, .LBB7_110
// %bb.94:
	ldrb	w8, [x19, #2100]
	cbnz	w8, .LBB7_111
.LBB7_95:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB7_114
// %bb.96:
	mov	w0, w20
	ldp	x20, x19, [sp, #272]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #256]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #240]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #224]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #208]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #192]    // 16-byte Folded Reload
	add	sp, sp, #288            // =288
	ret
.LBB7_97:
	mov	w26, w25
	cmp	w22, #1                 // =1
	stur	w20, [x29, #-92]        // 4-byte Folded Spill
	b.ne	.LBB7_100
// %bb.98:
	ldr	x8, [x19, #80]
	tbnz	w8, #8, .LBB7_103
// %bb.99:
	sub	x1, x29, #56            // =56
	mov	x0, x19
	bl	f2fs_write_checkpoint
	mov	w28, w0
	b	.LBB7_104
.LBB7_100:
	str	w22, [sp, #68]          // 4-byte Folded Spill
	b	.LBB7_66
.LBB7_101:
	mov	w26, w25
	str	wzr, [sp, #68]          // 4-byte Folded Spill
	mov	w28, #-22
	b	.LBB7_68
.LBB7_102:
	mov	w28, w0
	mov	w26, w25
	str	w22, [sp, #68]          // 4-byte Folded Spill
	stur	w20, [x29, #-92]        // 4-byte Folded Spill
	b	.LBB7_67
.LBB7_103:
	mov	w28, wzr
.LBB7_104:
	mov	w8, #1
	str	w8, [sp, #68]           // 4-byte Folded Spill
	b	.LBB7_67
.LBB7_105:
	ldr	x8, [x19, #2120]
	mov	w26, w25
	str	wzr, [sp, #68]          // 4-byte Folded Spill
	mov	w28, #-22
	ldr	w9, [x8, #188]
	sub	w9, w9, #1              // =1
	str	w9, [x8, #188]
	b	.LBB7_68
.LBB7_106:
	mov	w28, w0
	mov	w26, w25
	str	wzr, [sp, #68]          // 4-byte Folded Spill
	b	.LBB7_68
.LBB7_107:
	mov	w26, w25
	str	w22, [sp, #68]          // 4-byte Folded Spill
	mov	w28, #-5
	b	.LBB7_68
.LBB7_108:
	mov	w26, w25
	str	w22, [sp, #68]          // 4-byte Folded Spill
	stur	w20, [x29, #-92]        // 4-byte Folded Spill
	mov	w28, #-22
	b	.LBB7_68
.LBB7_109:
	str	w22, [sp, #68]          // 4-byte Folded Spill
	stur	wzr, [x29, #-92]        // 4-byte Folded Spill
	mov	w26, wzr
	mov	w21, wzr
	mov	w28, #-22
	b	.LBB7_68
.LBB7_110:
	bl	kvfree
	str	xzr, [x19, #2104]
	ldrb	w8, [x19, #2100]
	cbz	w8, .LBB7_95
.LBB7_111:
	mov	w8, #-1
	strb	wzr, [x19, #2100]
	str	xzr, [x19, #2088]
	str	w8, [x19, #2096]
	b	.LBB7_95
.LBB7_112:
	bl	preempt_schedule_notrace
	b	.LBB7_6
.LBB7_113:
	bl	preempt_schedule_notrace
	b	.LBB7_77
.LBB7_114:
	bl	__stack_chk_fail
.Lfunc_end7:
	.size	f2fs_gc, .Lfunc_end7-f2fs_gc
                                        // -- End function
	.p2align	2               // -- Begin function has_not_enough_free_secs
	.type	has_not_enough_free_secs,@function
has_not_enough_free_secs:               // @has_not_enough_free_secs
// %bb.0:
	ldr	w9, [x0, #1192]
	ldr	w16, [x0, #1188]
	ldrsw	x13, [x0, #1284]
	ldr	w11, [x0, #1184]
	ldrsw	x14, [x0, #1272]
	ldrsw	x15, [x0, #1296]
	ldr	x8, [x0, #80]
	tbnz	w8, #3, .LBB8_11
// %bb.1:
	ldr	x2, [x0, #136]
	ldr	x8, [x2, #8]
	ldr	w12, [x2, #100]
	ldr	w8, [x8, #8]
	cmn	w12, #1                 // =1
	udiv	w10, w12, w9
	add	w8, w8, w1
	b.eq	.LBB8_3
// %bb.2:
	mov	w17, w10
	cmp	w8, w17
	b.eq	.LBB8_4
	b	.LBB8_8
.LBB8_3:
	mov	w17, #-1
	cmp	w8, w17
	b.ne	.LBB8_8
.LBB8_4:
	ldr	w3, [x0, #1284]
	ldr	w4, [x0, #1272]
	ldr	w17, [x0, #1272]
	ldr	x0, [x2, #24]
	ldr	x1, [x2]
	mov	w5, #40
	ldr	w2, [x0, #460]
	ldr	x1, [x1, #120]
	madd	x2, x2, x5, x1
	ldrh	w5, [x2, #2]
	add	w2, w4, w3
	and	w3, w5, #0x3ff
	sub	w3, w16, w3
	cmp	w3, w2
	b.lo	.LBB8_8
// %bb.5:
	ldr	w3, [x0, #580]
	mov	w4, #40
	madd	x3, x3, x4, x1
	ldrh	w3, [x3, #2]
	and	w3, w3, #0x3ff
	sub	w3, w16, w3
	cmp	w3, w2
	b.lo	.LBB8_8
// %bb.6:
	ldr	w3, [x0, #700]
	mov	w4, #40
	madd	x3, x3, x4, x1
	ldrh	w3, [x3, #2]
	and	w3, w3, #0x3ff
	sub	w3, w16, w3
	cmp	w3, w2
	b.lo	.LBB8_8
// %bb.7:
	ldr	w0, [x0, #100]
	mov	w2, #40
	madd	x0, x0, x2, x1
	ldrh	w0, [x0, #2]
	and	w0, w0, #0x3ff
	sub	w0, w16, w0
	cmp	w0, w17
	b.hs	.LBB8_11
.LBB8_8:
	mul	w16, w16, w9
	sub	x16, x16, #1            // =1
	add	x13, x16, x13
	add	x14, x16, x14
	add	x15, x16, x15
	asr	x13, x13, x11
	asr	x14, x14, x11
	asr	x11, x15, x11
	udiv	w13, w13, w9
	udiv	w14, w14, w9
	udiv	w9, w11, w9
	add	w11, w13, w14, lsl #1
	cmn	w12, #1                 // =1
	add	w9, w11, w9
	b.ne	.LBB8_10
// %bb.9:
	mov	w10, #-1
.LBB8_10:
	add	w9, w9, w10
	cmp	w8, w9
	cset	w0, ls
	ret
.LBB8_11:
	mov	w0, wzr
	ret
.Lfunc_end8:
	.size	has_not_enough_free_secs, .Lfunc_end8-has_not_enough_free_secs
                                        // -- End function
	.p2align	2               // -- Begin function do_garbage_collect
	.type	do_garbage_collect,@function
do_garbage_collect:                     // @do_garbage_collect
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	mov	x29, sp
	sub	sp, sp, #576            // =576
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	mov	w10, #40
	umull	x10, w1, w10
	mov	w23, w4
	stur	x8, [x29, #-16]
	stp	xzr, xzr, [sp, #312]
	stp	xzr, xzr, [sp, #296]
	stp	xzr, xzr, [sp, #280]
	stp	xzr, xzr, [sp, #264]
	ldr	x8, [x0, #136]
	mov	x24, x2
	mov	w25, w1
	mov	x19, x0
	ldr	x8, [x8]
                                        // kill: def $w3 killed $w3 def $x3
	ldr	x9, [x8, #120]
	ldr	w8, [x0, #1192]
	ldr	w21, [x9, x10]
	cmp	w8, #2                  // =2
	add	w20, w8, w1
	b.lo	.LBB9_2
// %bb.1:
	udiv	w9, w20, w8
	mul	w20, w9, w8
.LBB9_2:
	and	w8, w21, #0x3e
	cmp	w8, #6                  // =6
	str	x3, [sp, #240]          // 8-byte Folded Spill
	b.hs	.LBB9_413
.LBB9_3:
	ldr	w8, [x19, #1192]
	cmp	w8, #2                  // =2
	b.lo	.LBB9_5
// %bb.4:
	ldr	x8, [x19, #136]
	sub	w2, w20, w25
	mov	w3, #3
	mov	w4, #1
	ldr	w8, [x8, #88]
	mov	x0, x19
	add	w1, w8, w25
	bl	f2fs_ra_meta_pages
.LBB9_5:
	cmp	w20, w25
	b.ls	.LBB9_398
// %bb.6:
	str	x24, [sp, #104]         // 8-byte Folded Spill
	mov	w26, wzr
	mov	w24, w25
	and	x8, x21, #0x3f
	sub	w21, w25, w20
	str	x8, [sp, #224]          // 8-byte Folded Spill
.LBB9_7:                                // =>This Inner Loop Header: Depth=1
	add	w22, w25, w26
	mov	x0, x19
	mov	w1, w22
	bl	f2fs_get_sum_page
	mov	x27, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB9_403
// %bb.8:                               //   in Loop: Header=BB9_7 Depth=1
	mov	x0, x27
	bl	unlock_page
	add	w26, w26, #1            // =1
	cmn	w21, w26
	b.ne	.LBB9_7
// %bb.9:
	add	x0, sp, #264            // =264
	bl	blk_start_plug
	cmp	w20, w25
	b.ls	.LBB9_399
// %bb.10:
	add	x8, x19, #3408          // =3408
	str	x8, [sp, #80]           // 8-byte Folded Spill
	ldr	x8, [sp, #104]          // 8-byte Folded Reload
	ldr	x9, [sp, #224]          // 8-byte Folded Reload
	ldr	x22, [sp, #240]         // 8-byte Folded Reload
	add	x10, x19, #1380         // =1380
	add	x8, x8, #16             // =16
	cmp	w9, #2                  // =2
	str	x8, [sp, #72]           // 8-byte Folded Spill
	sxtw	x8, w22
	str	x8, [sp, #144]          // 8-byte Folded Spill
	sub	x8, x29, #160           // =160
	str	x10, [sp, #128]         // 8-byte Folded Spill
	add	x9, x9, #1              // =1
	cset	w10, hi
	str	x9, [sp, #160]          // 8-byte Folded Spill
	add	x9, x19, w22, sxtw #2
	str	w10, [sp, #152]         // 4-byte Folded Spill
	add	x10, x8, #64            // =64
	add	x8, x8, #24             // =24
	cmp	w22, #0                 // =0
	stp	x10, x8, [sp, #56]      // 16-byte Folded Spill
	add	x8, x9, #1992           // =1992
	mov	w27, wzr
	add	x21, x19, #2296         // =2296
	mov	w20, w20
	mov	w25, #1
	str	x8, [sp, #112]          // 8-byte Folded Spill
	cset	w8, eq
	str	wzr, [sp, #156]         // 4-byte Folded Spill
	str	wzr, [sp, #176]         // 4-byte Folded Spill
	str	w8, [sp, #140]          // 4-byte Folded Spill
	str	w23, [sp, #180]         // 4-byte Folded Spill
	stp	x20, x21, [sp, #192]    // 16-byte Folded Spill
	str	x19, [sp, #248]         // 8-byte Folded Spill
	b	.LBB9_12
.LBB9_11:                               //   in Loop: Header=BB9_12 Depth=1
	add	x24, x24, #1            // =1
	cmp	x24, x20
	b.eq	.LBB9_411
.LBB9_12:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB9_68 Depth 2
                                        //       Child Loop BB9_70 Depth 3
                                        //         Child Loop BB9_179 Depth 4
                                        //         Child Loop BB9_300 Depth 4
                                        //         Child Loop BB9_275 Depth 4
                                        //         Child Loop BB9_267 Depth 4
                                        //     Child Loop BB9_39 Depth 2
                                        //       Child Loop BB9_47 Depth 3
	ldr	x8, [x19, #136]
	ldr	x9, [x19, #248]
	mov	w2, wzr
	mov	w3, wzr
	ldr	w8, [x8, #88]
	ldr	x0, [x9, #48]
	add	w1, w8, w24
	bl	pagecache_get_page
	mov	x12, x0
	cbz	x0, .LBB9_15
// %bb.13:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x12, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x12, x9, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w25
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB9_15
// %bb.14:                              //   in Loop: Header=BB9_12 Depth=1
	mov	x26, x12
	bl	__put_page
	mov	x12, x26
.LBB9_15:                               //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x19, #136]
	mov	w9, #40
	mul	x9, x24, x9
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	ldrh	w8, [x8, x9]
	tst	w8, #0xffc0
	b.eq	.LBB9_365
// %bb.16:                              //   in Loop: Header=BB9_12 Depth=1
	cbnz	w22, .LBB9_19
// %bb.17:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	w8, [x19, #1192]
	cmp	w8, #2                  // =2
	b.lo	.LBB9_19
// %bb.18:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	w8, [x19, #2116]
	ldr	w9, [sp, #176]          // 4-byte Folded Reload
	cmp	w9, w8
	b.hs	.LBB9_387
.LBB9_19:                               //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x12, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x12, x9, eq
	ldr	x8, [x8]
	tbz	w8, #2, .LBB9_387
// %bb.20:                              //   in Loop: Header=BB9_12 Depth=1
	//APP
	dmb ishld
	//NO_APP
	ldr	x8, [x19, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB9_387
// %bb.21:                              //   in Loop: Header=BB9_12 Depth=1
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x8, x12, x8
	lsl	x8, x8, #6
	mov	x9, #-549755813888
	and	x8, x8, #0xfffffffffffff000
	movk	x9, #4091
	ldrb	w4, [x8, x9]
	ldr	w3, [sp, #152]          // 4-byte Folded Reload
	str	x12, [sp, #208]         // 8-byte Folded Spill
	cmp	w3, w4
	b.ne	.LBB9_25
// %bb.22:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [sp, #224]          // 8-byte Folded Reload
	str	x27, [sp, #216]         // 8-byte Folded Spill
	cmp	w9, #3                  // =3
	mov	x9, #-549755813888
	add	x8, x8, x9
	str	x8, [sp, #184]          // 8-byte Folded Spill
	b.lo	.LBB9_33
// %bb.23:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x19, #136]
	cbz	x8, .LBB9_36
// %bb.24:                              //   in Loop: Header=BB9_12 Depth=1
	add	x9, x8, #80             // =80
	b	.LBB9_37
.LBB9_25:                               //   in Loop: Header=BB9_12 Depth=1
	adrp	x1, .L.str.14
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.14
	mov	w2, w24
	bl	f2fs_printk
	cbz	x19, .LBB9_30
// %bb.26:                              //   in Loop: Header=BB9_12 Depth=1
	add	x8, x19, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x19, #3376]
	cbnz	w8, .LBB9_30
// %bb.27:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB9_30
// %bb.28:                              //   in Loop: Header=BB9_12 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB9_30
// %bb.29:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x2, [sp, #80]           // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
.LBB9_30:                               //   in Loop: Header=BB9_12 Depth=1
	mov	x0, x19
	mov	w1, wzr
	bl	f2fs_stop_checkpoint
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB9_32
// %bb.31:                              //   in Loop: Header=BB9_12 Depth=1
	bl	dsm_client_ocuppy
	cbz	w0, .LBB9_35
.LBB9_32:                               //   in Loop: Header=BB9_12 Depth=1
	ldr	x12, [sp, #208]         // 8-byte Folded Reload
	b	.LBB9_387
.LBB9_33:                               //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x19, #136]
	ldr	x9, [x19]
	str	x9, [sp, #88]           // 8-byte Folded Spill
	cbz	x8, .LBB9_64
// %bb.34:                              //   in Loop: Header=BB9_12 Depth=1
	add	x9, x8, #80             // =80
	b	.LBB9_65
.LBB9_35:                               //   in Loop: Header=BB9_12 Depth=1
	adrp	x26, f2fs_dclient
	ldr	x0, [x26, :lo12:f2fs_dclient]
	adrp	x1, .L.str.15
	adrp	x2, .L__func__.do_garbage_collect
	mov	w3, #3123
	add	x1, x1, :lo12:.L.str.15
	add	x2, x2, :lo12:.L__func__.do_garbage_collect
	bl	dsm_client_record
	ldr	x0, [x26, :lo12:f2fs_dclient]
	mov	w1, #15244
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
	ldr	x12, [sp, #208]         // 8-byte Folded Reload
	b	.LBB9_387
.LBB9_36:                               //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB9_37:                               //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x8, #8]
	ldr	w10, [x19, #1184]
	ldr	w9, [x9]
	mov	w25, wzr
	ldr	w8, [x8]
	str	wzr, [sp, #236]         // 4-byte Folded Spill
	str	wzr, [sp, #172]         // 4-byte Folded Spill
	add	w8, w8, w24
	lsl	w8, w8, w10
	add	w8, w9, w8
	str	w8, [sp, #124]          // 4-byte Folded Spill
	b	.LBB9_39
.LBB9_38:                               //   in Loop: Header=BB9_39 Depth=2
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
	ldr	x27, [sp, #216]         // 8-byte Folded Reload
	ldp	x20, x21, [sp, #192]    // 16-byte Folded Reload
	add	w25, w25, #1            // =1
	cmp	w25, #3                 // =3
	b.eq	.LBB9_390
.LBB9_39:                               //   Parent Loop BB9_12 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB9_47 Depth 3
	cmp	w22, #1                 // =1
	b.ne	.LBB9_42
// %bb.40:                              //   in Loop: Header=BB9_39 Depth=2
	cmp	w25, #2                 // =2
	b.ne	.LBB9_42
// %bb.41:                              //   in Loop: Header=BB9_39 Depth=2
	ldr	x10, [sp, #128]         // 8-byte Folded Reload
	//APP
	// atomic_add
	prfm	pstl1strm, [x10]
1:	ldxr	w8, [x10]
	add	w8, w8, 1
	stxr	w9, w8, [x10]
	cbnz	w9, 1b

	//NO_APP
.LBB9_42:                               //   in Loop: Header=BB9_39 Depth=2
	ldr	x8, [sp, #248]          // 8-byte Folded Reload
	ldr	w8, [x8, #1188]
	cbz	w8, .LBB9_38
// %bb.43:                              //   in Loop: Header=BB9_39 Depth=2
	ldr	x20, [sp, #184]         // 8-byte Folded Reload
	mov	w27, wzr
	b	.LBB9_47
.LBB9_44:                               //   in Loop: Header=BB9_47 Depth=3
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
.LBB9_45:                               //   in Loop: Header=BB9_47 Depth=3
	ldp	x22, x19, [sp, #240]    // 16-byte Folded Reload
.LBB9_46:                               //   in Loop: Header=BB9_47 Depth=3
	ldr	w8, [x19, #1188]
	add	w27, w27, #1            // =1
	add	x20, x20, #7            // =7
	cmp	w27, w8
	b.hs	.LBB9_38
.LBB9_47:                               //   Parent Loop BB9_12 Depth=1
                                        //     Parent Loop BB9_39 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	ldr	w26, [x20]
	stp	xzr, xzr, [x29, #-160]
	cbnz	w22, .LBB9_49
// %bb.48:                              //   in Loop: Header=BB9_47 Depth=3
	ldr	x0, [sp, #248]          // 8-byte Folded Reload
	mov	w1, wzr
	bl	has_not_enough_free_secs
	tbnz	w0, #0, .LBB9_61
.LBB9_49:                               //   in Loop: Header=BB9_47 Depth=3
	ldr	x19, [sp, #248]         // 8-byte Folded Reload
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
	add	x22, x8, #72            // =72
	mov	x0, x22
	bl	down_read
	ldr	x8, [x19, #136]
	mov	w9, #40
	mov	w10, #1
	mov	x0, x22
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	madd	x8, x24, x9, x8
	ldr	x8, [x8, #8]
	mov	w9, w27
	lsr	x21, x9, #3
	mvn	w9, w27
	ldrb	w8, [x8, x21]
	and	w9, w9, #0x7
	lsl	w23, w10, w9
	and	w28, w23, w8
	bl	up_read
	cbz	w28, .LBB9_45
// %bb.50:                              //   in Loop: Header=BB9_47 Depth=3
	cmp	w25, #1                 // =1
	b.eq	.LBB9_53
// %bb.51:                              //   in Loop: Header=BB9_47 Depth=3
	cbnz	w25, .LBB9_54
// %bb.52:                              //   in Loop: Header=BB9_47 Depth=3
	mov	w8, #289
	movk	w8, #8210, lsl #16
	mul	x8, x26, x8
	ldr	x19, [sp, #248]         // 8-byte Folded Reload
	lsr	x8, x8, #32
	sub	w9, w26, w8
	add	w8, w8, w9, lsr #1
	lsr	w1, w8, #8
	mov	w2, #1
	mov	w3, #1
	mov	w4, #1
	mov	x0, x19
	bl	f2fs_ra_meta_pages
	b	.LBB9_60
.LBB9_53:                               //   in Loop: Header=BB9_47 Depth=3
	ldr	x19, [sp, #248]         // 8-byte Folded Reload
	mov	w1, w26
	mov	x0, x19
	bl	f2fs_ra_node_page
	b	.LBB9_60
.LBB9_54:                               //   in Loop: Header=BB9_47 Depth=3
	ldr	x19, [sp, #248]         // 8-byte Folded Reload
	mov	x1, x26
	mov	x0, x19
	bl	f2fs_get_node_page
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB9_60
// %bb.55:                              //   in Loop: Header=BB9_47 Depth=3
	ldr	x8, [x19, #136]
	mov	x22, x0
	ldr	x8, [x8]
	add	x28, x8, #72            // =72
	mov	x0, x28
	bl	down_read
	ldr	x8, [x19, #136]
	mov	w9, #40
	mov	x0, x28
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	madd	x8, x24, x9, x8
	ldr	x8, [x8, #8]
	ldrb	w8, [x8, x21]
	and	w21, w23, w8
	bl	up_read
	cbz	w21, .LBB9_59
// %bb.56:                              //   in Loop: Header=BB9_47 Depth=3
	sub	x2, x29, #160           // =160
	mov	x0, x19
	mov	w1, w26
	mov	w3, wzr
	bl	get_node_info_ex
	cbnz	w0, .LBB9_44
// %bb.57:                              //   in Loop: Header=BB9_47 Depth=3
	ldr	w9, [sp, #124]          // 4-byte Folded Reload
	ldur	w8, [x29, #-152]
	add	w9, w9, w27
	cmp	w9, w8
	b.ne	.LBB9_44
// %bb.58:                              //   in Loop: Header=BB9_47 Depth=3
	ldr	x8, [sp, #240]          // 8-byte Folded Reload
	mov	x0, x22
	mov	x22, x8
	cmp	w8, #1                  // =1
	mov	w1, w22
	cset	w21, eq
	bl	f2fs_move_node_page
	ldr	x19, [sp, #248]         // 8-byte Folded Reload
	ldr	w11, [sp, #172]         // 4-byte Folded Reload
	cmp	w0, #0                  // =0
	cset	w9, eq
	ldr	x8, [x19, #2120]
	and	w9, w21, w9
	add	w11, w11, w9
	str	w11, [sp, #172]         // 4-byte Folded Spill
	ldr	w9, [x8, #452]
	ldr	w10, [x8, #444]
	add	w9, w9, #1              // =1
	str	w9, [x8, #452]
	ldr	w9, [sp, #236]          // 4-byte Folded Reload
	add	w10, w10, #1            // =1
	str	w10, [x8, #444]
	ldr	w10, [x8, #460]
	cinc	w9, w9, eq
	str	w9, [sp, #236]          // 4-byte Folded Spill
	ldr	w9, [sp, #140]          // 4-byte Folded Reload
	add	w9, w10, w9
	str	w9, [x8, #460]
	b	.LBB9_46
.LBB9_59:                               //   in Loop: Header=BB9_47 Depth=3
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
.LBB9_60:                               //   in Loop: Header=BB9_47 Depth=3
	ldr	x22, [sp, #240]         // 8-byte Folded Reload
	b	.LBB9_46
.LBB9_61:                               //   in Loop: Header=BB9_12 Depth=1
	ldr	x21, [sp, #200]         // 8-byte Folded Reload
	mov	x0, x21
	bl	mutex_lock
	ldr	x19, [sp, #248]         // 8-byte Folded Reload
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB9_63
// %bb.62:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	w9, [x8, #136]
	ldr	w10, [sp, #236]         // 4-byte Folded Reload
	add	w9, w9, w10
	str	w9, [x8, #136]
.LBB9_63:                               //   in Loop: Header=BB9_12 Depth=1
	mov	x0, x21
	bl	mutex_unlock
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
	ldr	x27, [sp, #216]         // 8-byte Folded Reload
	ldr	x20, [sp, #192]         // 8-byte Folded Reload
	mov	w25, #1
	b	.LBB9_361
.LBB9_64:                               //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB9_65:                               //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x8, #8]
	ldr	w11, [x19, #1184]
	ldr	w9, [x9]
	str	wzr, [sp, #236]         // 4-byte Folded Spill
	ldr	w10, [x8]
	ldr	w8, [x19, #1188]
	str	wzr, [sp, #100]         // 4-byte Folded Spill
	str	wzr, [sp, #172]         // 4-byte Folded Spill
	add	w10, w10, w24
	lsl	w10, w10, w11
	add	w9, w10, w9
	str	w9, [sp, #124]          // 4-byte Folded Spill
	b	.LBB9_68
.LBB9_66:                               //   in Loop: Header=BB9_68 Depth=2
	ldr	x27, [sp, #216]         // 8-byte Folded Reload
	ldp	x20, x21, [sp, #192]    // 16-byte Folded Reload
.LBB9_67:                               //   in Loop: Header=BB9_68 Depth=2
	ldr	w9, [sp, #236]          // 4-byte Folded Reload
	add	w9, w9, #1              // =1
	cmp	w9, #5                  // =5
	str	w9, [sp, #236]          // 4-byte Folded Spill
	b.eq	.LBB9_394
.LBB9_68:                               //   Parent Loop BB9_12 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB9_70 Depth 3
                                        //         Child Loop BB9_179 Depth 4
                                        //         Child Loop BB9_300 Depth 4
                                        //         Child Loop BB9_275 Depth 4
                                        //         Child Loop BB9_267 Depth 4
	cbz	w8, .LBB9_67
// %bb.69:                              //   in Loop: Header=BB9_68 Depth=2
	ldr	x28, [sp, #184]         // 8-byte Folded Reload
	mov	w20, wzr
.LBB9_70:                               //   Parent Loop BB9_12 Depth=1
                                        //     Parent Loop BB9_68 Depth=2
                                        // =>    This Loop Header: Depth=3
                                        //         Child Loop BB9_179 Depth 4
                                        //         Child Loop BB9_300 Depth 4
                                        //         Child Loop BB9_275 Depth 4
                                        //         Child Loop BB9_267 Depth 4
	stp	xzr, xzr, [x29, #-248]
	ldr	w26, [x28]
	cbnz	w22, .LBB9_72
// %bb.71:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x0, x19
	mov	w1, wzr
	bl	has_not_enough_free_secs
	tbnz	w0, #0, .LBB9_357
.LBB9_72:                               //   in Loop: Header=BB9_70 Depth=3
	tbz	w23, #0, .LBB9_74
// %bb.73:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
	b	.LBB9_78
.LBB9_74:                               //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x19, #136]
	ldr	w9, [x19, #1192]
	ldr	x8, [x8]
	cmp	w9, #2                  // =2
	b.lo	.LBB9_76
// %bb.75:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x10, [x8, #128]
	udiv	w11, w24, w9
	ldr	w10, [x10, w11, uxtw #2]
	b	.LBB9_77
.LBB9_76:                               //   in Loop: Header=BB9_70 Depth=3
	ldr	x10, [x8, #120]
	mov	w11, #40
	mul	x11, x24, x11
	ldr	w10, [x10, x11]
	ubfx	w10, w10, #6, #10
.LBB9_77:                               //   in Loop: Header=BB9_70 Depth=3
	ldr	w11, [x19, #1188]
	mul	w9, w11, w9
	cmp	w10, w9
	b.eq	.LBB9_357
.LBB9_78:                               //   in Loop: Header=BB9_70 Depth=3
	add	x22, x8, #72            // =72
	mov	x0, x22
	bl	down_read
	ldr	x8, [x19, #136]
	mov	w9, #40
	mov	w10, #1
	mov	x0, x22
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	madd	x8, x24, x9, x8
	ldr	x8, [x8, #8]
	mov	w9, w20
	lsr	x27, x9, #3
	mvn	w9, w20
	ldrb	w8, [x8, x27]
	and	w9, w9, #0x7
	lsl	w25, w10, w9
	and	w21, w25, w8
	bl	up_read
	cbz	w21, .LBB9_103
// %bb.79:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w8, [sp, #236]          // 4-byte Folded Reload
	cmp	w8, #1                  // =1
	b.eq	.LBB9_82
// %bb.80:                              //   in Loop: Header=BB9_70 Depth=3
	cbnz	w8, .LBB9_83
// %bb.81:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w8, #289
	movk	w8, #8210, lsl #16
	umull	x8, w26, w8
	lsr	x8, x8, #32
	sub	w9, w26, w8
	add	w8, w8, w9, lsr #1
	lsr	w1, w8, #8
	mov	w2, #1
	mov	w3, #1
	mov	w4, #1
	mov	x0, x19
	bl	f2fs_ra_meta_pages
	b	.LBB9_103
.LBB9_82:                               //   in Loop: Header=BB9_70 Depth=3
	mov	x0, x19
	mov	w1, w26
	bl	f2fs_ra_node_page
	b	.LBB9_103
.LBB9_83:                               //   in Loop: Header=BB9_70 Depth=3
	ldr	w22, [x28]
	ldurh	w21, [x28, #5]
	mov	x0, x19
	mov	x1, x22
	bl	f2fs_get_node_page
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB9_103
// %bb.84:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x26, x0
	sub	x2, x29, #248           // =248
	mov	x0, x19
	mov	w1, w22
	mov	w3, wzr
	bl	get_node_info_ex
	cbz	w0, .LBB9_86
// %bb.85:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	mov	x0, x26
	bl	f2fs_put_page
	b	.LBB9_103
.LBB9_86:                               //   in Loop: Header=BB9_70 Depth=3
	ldrb	w8, [x28, #4]
	ldurb	w9, [x29, #-236]
	cmp	w8, w9
	b.eq	.LBB9_95
// %bb.87:                              //   in Loop: Header=BB9_70 Depth=3
	adrp	x1, .L.str.17
	adrp	x2, .L__func__.is_alive
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.17
	add	x2, x2, :lo12:.L__func__.is_alive
	bl	f2fs_printk
	cbz	x19, .LBB9_92
// %bb.88:                              //   in Loop: Header=BB9_70 Depth=3
	add	x8, x19, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x19, #3376]
	cbnz	w8, .LBB9_92
// %bb.89:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB9_92
// %bb.90:                              //   in Loop: Header=BB9_70 Depth=3
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB9_92
// %bb.91:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x2, [sp, #80]           // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
.LBB9_92:                               //   in Loop: Header=BB9_70 Depth=3
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB9_95
// %bb.93:                              //   in Loop: Header=BB9_70 Depth=3
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB9_95
// %bb.94:                              //   in Loop: Header=BB9_70 Depth=3
	adrp	x22, f2fs_dclient
	ldr	x0, [x22, :lo12:f2fs_dclient]
	adrp	x1, .L.str.15
	adrp	x2, .L__func__.is_alive
	mov	w3, #2131
	add	x1, x1, :lo12:.L.str.15
	add	x2, x2, :lo12:.L__func__.is_alive
	bl	dsm_client_record
	ldr	x0, [x22, :lo12:f2fs_dclient]
	mov	w1, #15244
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB9_95:                               //   in Loop: Header=BB9_70 Depth=3
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x8, x26, x8
	lsl	x8, x8, #6
	mov	x11, #-549755813888
	and	x10, x8, #0xfffffffffffff000
	mov	x8, #-549755813888
	mov	x9, #-549755813888
	movk	x11, #4080
	movk	x8, #4072
	movk	x9, #4076
	ldr	w22, [x10, x11]
	ldr	w11, [sp, #124]         // 4-byte Folded Reload
	ldr	w8, [x10, x8]
	ldr	w9, [x10, x9]
	add	w23, w11, w20
	mov	x11, #-549755813888
	cmp	w8, w9
	add	x10, x10, x11
	b.ne	.LBB9_97
// %bb.96:                              //   in Loop: Header=BB9_70 Depth=3
	ldrb	w11, [x10, #3]
	tbnz	w11, #5, .LBB9_98
.LBB9_97:                               //   in Loop: Header=BB9_70 Depth=3
	mov	x11, xzr
	b	.LBB9_99
.LBB9_98:                               //   in Loop: Header=BB9_70 Depth=3
	ldrh	w11, [x10, #360]
	lsr	x11, x11, #2
.LBB9_99:                               //   in Loop: Header=BB9_70 Depth=3
	add	x12, x10, #360          // =360
	cmp	w8, w9
	csel	x8, x12, x10, eq
	add	x9, x11, x21
	ldr	w21, [x8, x9, lsl #2]
	mov	w1, #1
	mov	x0, x26
	bl	f2fs_put_page
	cmp	w21, w23
	b.ne	.LBB9_102
// %bb.100:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	w8, [sp, #236]          // 4-byte Folded Reload
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
	cmp	w8, #2                  // =2
	b.ne	.LBB9_106
// %bb.101:                             //   in Loop: Header=BB9_70 Depth=3
	ldur	w1, [x29, #-244]
	mov	x0, x19
	bl	f2fs_ra_node_page
	b	.LBB9_103
.LBB9_102:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
.LBB9_103:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x22, [sp, #240]         // 8-byte Folded Reload
.LBB9_104:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w25, #1
.LBB9_105:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w8, [x19, #1188]
	add	w20, w20, #1            // =1
	add	x28, x28, #7            // =7
	cmp	w20, w8
	b.lo	.LBB9_70
	b	.LBB9_66
.LBB9_106:                              //   in Loop: Header=BB9_70 Depth=3
	ldurh	w21, [x28, #5]
	ldur	w1, [x29, #-244]
	cmp	w8, #3                  // =3
	lsr	w26, w22, #3
	b.ne	.LBB9_112
// %bb.107:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x0, [sp, #88]           // 8-byte Folded Reload
	bl	f2fs_iget
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB9_103
// %bb.108:                             //   in Loop: Header=BB9_70 Depth=3
	mov	x25, x0
	bl	is_bad_inode
	tbnz	w0, #0, .LBB9_103
// %bb.109:                             //   in Loop: Header=BB9_70 Depth=3
	ldrh	w8, [x25]
	and	w8, w8, #0xf000
	cmp	w8, #6, lsl #12         // =24576
	b.ge	.LBB9_122
// %bb.110:                             //   in Loop: Header=BB9_70 Depth=3
	cmp	w8, #1, lsl #12         // =4096
	b.eq	.LBB9_103
// %bb.111:                             //   in Loop: Header=BB9_70 Depth=3
	cmp	w8, #2, lsl #12         // =8192
	b.eq	.LBB9_103
	b	.LBB9_124
.LBB9_112:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	radix_tree_lookup
	cbz	x0, .LBB9_103
// %bb.113:                             //   in Loop: Header=BB9_70 Depth=3
	str	w21, [sp, #48]          // 4-byte Folded Spill
	ldr	x21, [x0, #16]
	cbz	x21, .LBB9_103
// %bb.114:                             //   in Loop: Header=BB9_70 Depth=3
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB9_118
// %bb.115:                             //   in Loop: Header=BB9_70 Depth=3
	add	x0, x21, #992           // =992
	str	x0, [sp, #40]           // 8-byte Folded Spill
	bl	down_write_trylock
	cbz	w0, .LBB9_103
// %bb.116:                             //   in Loop: Header=BB9_70 Depth=3
	add	x0, x21, #1040          // =1040
	bl	down_write_trylock
	cbz	w0, .LBB9_131
// %bb.117:                             //   in Loop: Header=BB9_70 Depth=3
	mov	x0, x21
	bl	inode_dio_wait
	mov	w13, #1
	b	.LBB9_119
.LBB9_118:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w13, wzr
.LBB9_119:                              //   in Loop: Header=BB9_70 Depth=3
	cbz	w26, .LBB9_128
// %bb.120:                             //   in Loop: Header=BB9_70 Depth=3
	cmp	w22, #23                // =23
	b.hi	.LBB9_129
// %bb.121:                             //   in Loop: Header=BB9_70 Depth=3
	sub	w8, w26, #1             // =1
	b	.LBB9_133
.LBB9_122:                              //   in Loop: Header=BB9_70 Depth=3
	b.eq	.LBB9_103
// %bb.123:                             //   in Loop: Header=BB9_70 Depth=3
	cmp	w8, #12, lsl #12        // =49152
	b.eq	.LBB9_103
.LBB9_124:                              //   in Loop: Header=BB9_70 Depth=3
	add	x27, x25, #1040         // =1040
	mov	x0, x27
	bl	down_write_trylock
	cbz	w0, .LBB9_172
// %bb.125:                             //   in Loop: Header=BB9_70 Depth=3
	cbz	w26, .LBB9_198
// %bb.126:                             //   in Loop: Header=BB9_70 Depth=3
	cmp	w22, #23                // =23
	b.hi	.LBB9_199
// %bb.127:                             //   in Loop: Header=BB9_70 Depth=3
	sub	w8, w26, #1             // =1
	b	.LBB9_203
.LBB9_128:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x22, [sp, #240]         // 8-byte Folded Reload
	mov	w8, wzr
	b	.LBB9_140
.LBB9_129:                              //   in Loop: Header=BB9_70 Depth=3
	cmp	w26, #2040              // =2040
	b.hi	.LBB9_132
// %bb.130:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w9, #37367
	sub	w8, w26, #4             // =4
	movk	w9, #321, lsl #16
	umull	x9, w8, w9
	lsr	x9, x9, #32
	sub	w8, w8, w9
	add	w8, w9, w8, lsr #1
	sub	w8, w26, w8, lsr #9
	sub	w8, w8, #2              // =2
	b	.LBB9_133
.LBB9_131:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x19, #2024]
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	add	x8, x8, #1              // =1
	str	x8, [x19, #2024]
	bl	up_write
	b	.LBB9_103
.LBB9_132:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w9, #37367
	sub	w8, w26, #2043          // =2043
	movk	w9, #321, lsl #16
	umull	x9, w8, w9
	lsr	x9, x9, #32
	sub	w8, w8, w9
	add	w8, w9, w8, lsr #1
	sub	w8, w26, w8, lsr #9
	sub	w8, w8, #5              // =5
.LBB9_133:                              //   in Loop: Header=BB9_70 Depth=3
	ldrh	w9, [x21]
	ldr	x22, [sp, #240]         // 8-byte Folded Reload
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB9_135
// %bb.134:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x10, [x21, #736]
	tbnz	w10, #31, .LBB9_166
.LBB9_135:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w10, #1018
.LBB9_136:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w11, [x21, #1184]
	ldr	w12, [x21, #1192]
	mul	w8, w10, w8
	cmp	w9, #8, lsl #12         // =32768
	mov	w9, #923
	add	w10, w12, w11, asr #2
	sub	w9, w9, w10
	b.ne	.LBB9_139
// %bb.137:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x10, [x21, #736]
	tbz	w10, #31, .LBB9_139
// %bb.138:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	w10, [x21, #1292]
	neg	w10, w10
	and	w9, w9, w10
.LBB9_139:                              //   in Loop: Header=BB9_70 Depth=3
	add	w8, w9, w8
.LBB9_140:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w10, [sp, #48]          // 4-byte Folded Reload
	ldrb	w9, [x21, #13]
	str	w13, [sp, #40]          // 4-byte Folded Spill
	add	w8, w8, w10
	tbz	w9, #6, .LBB9_142
// %bb.141:                             //   in Loop: Header=BB9_70 Depth=3
	ldrh	w9, [x21]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.eq	.LBB9_143
.LBB9_142:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x9, [x21, #664]
	cbz	x9, .LBB9_160
.LBB9_143:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x9, [sp, #64]           // 8-byte Folded Reload
	sub	x11, x29, #160          // =160
	stp	xzr, xzr, [x9, #96]
	stp	xzr, xzr, [x9, #80]
	stp	xzr, xzr, [x9, #64]
	stp	xzr, xzr, [x9, #48]
	stp	xzr, xzr, [x9, #32]
	stp	xzr, xzr, [x9, #16]
	stp	xzr, xzr, [x9]
	ldr	x9, [x21, #40]
	ldr	x9, [x9, #1008]
	stp	xzr, x9, [x29, #-168]
	ldr	x10, [x21, #64]
	stp	xzr, xzr, [x29, #-184]
	stp	xzr, xzr, [x29, #-200]
	stp	xzr, xzr, [x29, #-216]
	stur	w10, [x29, #-152]
	mov	x10, #8589934592
	stur	wzr, [x11, #139]
	stur	wzr, [x29, #-24]
	stur	xzr, [x29, #-224]
	stur	x10, [x11, #12]
	stur	wzr, [x29, #-140]
	sturb	wzr, [x29, #-72]
	sturb	wzr, [x29, #-70]
	stur	wzr, [x29, #-228]
	ldr	w26, [x9, #1504]
	mov	w11, #2
	cbnz	w22, .LBB9_146
// %bb.144:                             //   in Loop: Header=BB9_70 Depth=3
	ldrb	w10, [x9, #1920]
	cbz	w10, .LBB9_146
// %bb.145:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	w9, [x9, #1988]
	mov	w10, #7
	cmp	w9, #5                  // =5
	mov	w9, #2
	csel	w11, w9, w10, eq
.LBB9_146:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x0, [x21, #48]
	mov	w1, w8
	mov	w2, #7
	str	w11, [sp, #24]          // 4-byte Folded Spill
	ldr	w3, [x0, #24]
	str	x1, [sp, #32]           // 8-byte Folded Spill
	bl	pagecache_get_page
	cbz	x0, .LBB9_159
// %bb.147:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x21, #40]
	str	w26, [sp, #16]          // 4-byte Folded Spill
	mov	x26, x21
	str	x0, [sp, #48]           // 8-byte Folded Spill
	ldr	x21, [x8, #1008]
	ldr	x8, [x21, #136]
	ldr	x8, [x8]
	add	x22, x8, #72            // =72
	mov	x0, x22
	bl	down_read
	ldr	x8, [x21, #136]
	mov	w9, #40
	mov	x0, x22
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	madd	x8, x24, x9, x8
	ldr	x8, [x8, #8]
	ldrb	w8, [x8, x27]
	and	w21, w25, w8
	bl	up_read
	cbz	w21, .LBB9_164
// %bb.148:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x26, #736]
	ldr	x22, [sp, #240]         // 8-byte Folded Reload
	mov	w25, #1
	mov	x21, x26
	tbnz	w8, #15, .LBB9_165
// %bb.149:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x21, #736]
	ldr	x26, [sp, #48]          // 8-byte Folded Reload
	tbnz	w8, #28, .LBB9_171
// %bb.150:                             //   in Loop: Header=BB9_70 Depth=3
	sub	x0, x29, #208           // =208
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	ldr	x1, [sp, #32]           // 8-byte Folded Reload
	sub	x0, x29, #208           // =208
	mov	w2, #1
	stp	x21, xzr, [x29, #-208]
	stur	xzr, [x29, #-192]
	stur	wzr, [x29, #-184]
	bl	f2fs_get_dnode_of_data
	mov	w27, w0
	cbnz	w0, .LBB9_338
// %bb.151:                             //   in Loop: Header=BB9_70 Depth=3
	ldur	w8, [x29, #-172]
	cbz	w8, .LBB9_331
// %bb.152:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w2, #1
	mov	w3, #1
	mov	x0, x26
	mov	w1, wzr
	bl	f2fs_wait_on_page_writeback
	ldur	w1, [x29, #-172]
	mov	x0, x21
	bl	f2fs_wait_on_block_writeback
	ldur	x0, [x29, #-160]
	ldur	w1, [x29, #-184]
	sub	x2, x29, #224           // =224
	mov	w3, wzr
	bl	get_node_info_ex
	mov	w27, w0
	cbnz	w0, .LBB9_328
// %bb.153:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	w9, [sp, #16]           // 4-byte Folded Reload
	ldur	w8, [x29, #-172]
	stur	x26, [x29, #-120]
	cmp	w9, #1                  // =1
	stp	w8, w8, [x29, #-132]
	b.ne	.LBB9_155
// %bb.154:                             //   in Loop: Header=BB9_70 Depth=3
	ldur	x8, [x29, #-160]
	add	x0, x8, #168            // =168
	bl	down_write
	ldur	w8, [x29, #-128]
.LBB9_155:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	x9, [x29, #-160]
	mov	w1, w8
	mov	w2, #7
	ldr	x9, [x9, #248]
	ldr	x0, [x9, #48]
	ldr	w3, [x0, #24]
	bl	pagecache_get_page
	cbz	x0, .LBB9_249
// %bb.156:                             //   in Loop: Header=BB9_70 Depth=3
	stur	x0, [x29, #-112]
	ldr	x8, [x0, #8]
	mov	w19, #1
	mov	x23, x0
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x0, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB9_280
// %bb.157:                             //   in Loop: Header=BB9_70 Depth=3
	sub	x0, x29, #160           // =160
	bl	f2fs_submit_page_bio
	cbz	w0, .LBB9_310
// %bb.158:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w27, w0
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_put_page
	b	.LBB9_325
.LBB9_159:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w27, #-12
	mov	w25, #1
	b	.LBB9_340
.LBB9_160:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x9, [x21, #40]
	ldr	x10, [x9, #1008]
	ldr	w9, [x10, #1532]
	cbz	w9, .LBB9_167
// %bb.161:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	w9, [x21, #1280]
	cbnz	w9, .LBB9_143
// %bb.162:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x9, [x21, #736]
	tbnz	x9, #35, .LBB9_143
// %bb.163:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x9, [x21, #736]
	tbnz	x9, #34, .LBB9_143
	b	.LBB9_169
.LBB9_164:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x22, [sp, #240]         // 8-byte Folded Reload
	mov	x21, x26
	ldr	x26, [sp, #48]          // 8-byte Folded Reload
	mov	w27, #-2
	mov	w25, #1
	b	.LBB9_338
.LBB9_165:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w8, [x21, #724]
	ldr	x9, [x21, #40]
	ldr	x26, [sp, #48]          // 8-byte Folded Reload
	mov	w27, #-11
	add	w8, w8, #1              // =1
	str	w8, [x21, #724]
	ldr	x8, [x9, #1008]
	ldr	x9, [sp, #144]          // 8-byte Folded Reload
	add	x8, x8, x9, lsl #3
	ldr	x9, [x8, #2008]
	add	x9, x9, #1              // =1
	str	x9, [x8, #2008]
	b	.LBB9_338
.LBB9_166:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w10, [x21, #1292]
	mov	w11, #1018
	neg	w10, w10
	and	w10, w10, w11
	b	.LBB9_136
.LBB9_167:                              //   in Loop: Header=BB9_70 Depth=3
	ldrh	w9, [x21]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB9_169
// %bb.168:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x9, [x21, #736]
	tbnz	w9, #31, .LBB9_143
.LBB9_169:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, w8
	mov	w2, #1
	mov	x0, x21
	str	x10, [sp, #32]          // 8-byte Folded Spill
	bl	f2fs_get_lock_data_page
	cmn	x0, #4095               // =4095
	b.lo	.LBB9_173
// %bb.170:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w27, w0
	mov	w25, #1
	b	.LBB9_340
.LBB9_171:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_pin_file_control
	mov	w27, #-11
	b	.LBB9_338
.LBB9_172:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x0, x25
	bl	iput
	ldr	x8, [x19, #2024]
	add	x8, x8, #1              // =1
	str	x8, [x19, #2024]
	b	.LBB9_103
.LBB9_173:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x21, #40]
	mov	x26, x21
	str	x0, [sp, #48]           // 8-byte Folded Spill
	ldr	x21, [x8, #1008]
	ldr	x8, [x21, #136]
	ldr	x8, [x8]
	add	x22, x8, #72            // =72
	mov	x0, x22
	bl	down_read
	ldr	x8, [x21, #136]
	mov	w9, #40
	mov	x0, x22
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	madd	x8, x24, x9, x8
	ldr	x8, [x8, #8]
	ldrb	w8, [x8, x27]
	and	w21, w25, w8
	bl	up_read
	cbz	w21, .LBB9_201
// %bb.174:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x26, #736]
	ldr	x22, [sp, #240]         // 8-byte Folded Reload
	mov	x21, x26
	tbnz	w8, #15, .LBB9_228
// %bb.175:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x21, #736]
	tbnz	w8, #28, .LBB9_240
// %bb.176:                             //   in Loop: Header=BB9_70 Depth=3
	str	x24, [sp, #24]          // 8-byte Folded Spill
	cbz	w22, .LBB9_288
// %bb.177:                             //   in Loop: Header=BB9_70 Depth=3
	stp	xzr, xzr, [x29, #-40]
	stp	xzr, xzr, [x29, #-56]
	stp	xzr, xzr, [x29, #-72]
	stp	xzr, xzr, [x29, #-88]
	stp	xzr, xzr, [x29, #-104]
	stp	xzr, xzr, [x29, #-120]
	stp	xzr, xzr, [x29, #-136]
	stp	xzr, xzr, [x29, #-152]
	stur	xzr, [x29, #-160]
	ldr	x8, [x21, #40]
	mov	x9, #8589934592
	mov	x24, #63439
	movk	x24, #58195, lsl #16
	ldr	x8, [x8, #1008]
	movk	x24, #39845, lsl #32
	mov	x22, xzr
	mov	w23, #1
	stur	x8, [x29, #-160]
	ldr	x8, [x21, #64]
	mov	w19, #8
	movk	x24, #8388, lsl #48
	mov	w25, #500
	stur	w8, [x29, #-152]
	sub	x8, x29, #160           // =160
	stur	x9, [x8, #12]
	mov	x9, #1
	movk	x9, #2048, lsl #32
	stur	x9, [x8, #20]
	stur	xzr, [x8, #28]
	ldp	x10, x8, [sp, #48]      // 16-byte Folded Reload
	stur	xzr, [x29, #-104]
	stp	x10, xzr, [x29, #-120]
	stp	xzr, xzr, [x8]
	mov	w8, #7
	sturb	wzr, [x29, #-80]
	stur	wzr, [x29, #-76]
	sturh	wzr, [x29, #-72]
	sturb	wzr, [x29, #-70]
	stur	wzr, [x29, #-68]
	sturb	wzr, [x29, #-64]
	stur	w8, [x29, #-60]
	stp	xzr, xzr, [x29, #-56]
	stur	xzr, [x29, #-40]
	sturb	wzr, [x29, #-32]
	stur	wzr, [x29, #-28]
	ldr	x8, [x10, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x10, x9, eq
	ldr	x8, [x8]
	stp	x21, x8, [sp, #8]       // 16-byte Folded Spill
	b	.LBB9_179
.LBB9_178:                              //   in Loop: Header=BB9_179 Depth=4
	mov	w1, #5
	mov	w0, wzr
	bl	congestion_wait
	add	x23, x23, #1            // =1
.LBB9_179:                              //   Parent Loop BB9_12 Depth=1
                                        //     Parent Loop BB9_68 Depth=2
                                        //       Parent Loop BB9_70 Depth=3
                                        // =>      This Inner Loop Header: Depth=4
	ldr	x26, [sp, #48]          // 8-byte Folded Reload
	mov	w2, #1
	mov	w3, #1
	mov	w1, wzr
	mov	x0, x26
	bl	f2fs_wait_on_page_writeback
	mov	x0, x26
	bl	set_page_dirty
	mov	x0, x26
	bl	clear_page_dirty_for_io
	cbz	w0, .LBB9_188
// %bb.180:                             //   in Loop: Header=BB9_179 Depth=4
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB9_183
// %bb.181:                             //   in Loop: Header=BB9_179 Depth=4
	cmp	w8, #10, lsl #12        // =40960
	b.eq	.LBB9_183
// %bb.182:                             //   in Loop: Header=BB9_179 Depth=4
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB9_187
.LBB9_183:                              //   in Loop: Header=BB9_179 Depth=4
	add	x8, x21, #792           // =792
	mov	w11, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w11
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x21, #736]
	tbz	x8, #37, .LBB9_185
// %bb.184:                             //   in Loop: Header=BB9_179 Depth=4
	ldr	x8, [x21, #40]
	ldr	x8, [x8, #1008]
	add	x8, x8, #1328           // =1328
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w11
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB9_185:                              //   in Loop: Header=BB9_179 Depth=4
	ldr	x8, [x21, #40]
	ldrh	w9, [x21]
	ldr	x8, [x8, #1008]
	and	w9, w9, #0xf000
	cmp	w9, #4, lsl #12         // =16384
	cset	w9, ne
	add	x8, x8, w9, uxtw #2
	add	x8, x8, #1272           // =1272
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w11
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldrb	w8, [x21, #12]
	tbz	w8, #5, .LBB9_187
// %bb.186:                             //   in Loop: Header=BB9_179 Depth=4
	ldr	x8, [x21, #40]
	mov	w11, #1
	ldr	x8, [x8, #1008]
	add	x8, x8, #1280           // =1280
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w11
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB9_187:                              //   in Loop: Header=BB9_179 Depth=4
	mov	x0, x21
	bl	f2fs_remove_dirty_inode
.LBB9_188:                              //   in Loop: Header=BB9_179 Depth=4
	ldr	x26, [sp, #48]          // 8-byte Folded Reload
	ldr	x8, [x26]
	tbnz	w8, #13, .LBB9_190
// %bb.189:                             //   in Loop: Header=BB9_179 Depth=4
	ldr	x8, [x26, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x26, x9, eq
	add	x8, x8, #52             // =52
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x26]
1:	ldxr	x8, [x26]
	orr	x8, x8, 8192
	stxr	w9, x8, [x26]
	cbnz	w9, 1b
	//NO_APP
	str	xzr, [x26, #40]
.LBB9_190:                              //   in Loop: Header=BB9_179 Depth=4
	add	x21, x26, #40           // =40
	//APP
	// atomic64_or
	prfm	pstl1strm, [x21]
1:	ldxr	x8, [x21]
	orr	x8, x8, 1
	stxr	w9, x8, [x21]
	cbnz	w9, 1b
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x21]
1:	ldxr	x8, [x21]
	orr	x8, x8, 8
	stxr	w9, x8, [x21]
	cbnz	w9, 1b
	//NO_APP
	sub	x0, x29, #160           // =160
	bl	f2fs_do_write_data_page
	mov	w27, w0
	cbz	w0, .LBB9_294
// %bb.191:                             //   in Loop: Header=BB9_179 Depth=4
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x21]
1:	ldxr	x8, [x21]
	bic	x8, x8, x19
	stxr	w9, x8, [x21]
	cbnz	w9, 1b
	//NO_APP
	ldr	x8, [x26, #40]
	cmp	x8, #1                  // =1
	b.ne	.LBB9_195
// %bb.192:                             //   in Loop: Header=BB9_179 Depth=4
	str	xzr, [x26, #40]
	ldr	x8, [x26]
	tbz	w8, #13, .LBB9_195
// %bb.193:                             //   in Loop: Header=BB9_179 Depth=4
	mov	w10, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x26]
1:	ldxr	x8, [x26]
	bic	x8, x8, x10
	stxr	w9, x8, [x26]
	cbnz	w9, 1b
	//NO_APP
	ldr	x8, [x26, #8]
	mov	w11, #1
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x26, x9, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w11
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB9_195
// %bb.194:                             //   in Loop: Header=BB9_179 Depth=4
	bl	__put_page
.LBB9_195:                              //   in Loop: Header=BB9_179 Depth=4
	ldr	x21, [sp, #8]           // 8-byte Folded Reload
	cmn	w27, #12                // =12
	mov	w8, w27
	b.ne	.LBB9_292
// %bb.196:                             //   in Loop: Header=BB9_179 Depth=4
	lsr	x8, x23, #2
	umulh	x8, x8, x24
	lsr	x8, x8, #4
	mul	x8, x8, x25
	sub	x8, x8, #1              // =1
	cmp	x8, x22
	add	x22, x22, #1            // =1
	b.ne	.LBB9_178
// %bb.197:                             //   in Loop: Header=BB9_179 Depth=4
	ldr	x0, [sp, #32]           // 8-byte Folded Reload
	adrp	x1, .L.str.19
	adrp	x3, .L__func__.move_data_page
	mov	w4, #2698
	add	x1, x1, :lo12:.L.str.19
	mov	x2, x22
	add	x3, x3, :lo12:.L__func__.move_data_page
	bl	f2fs_printk
	b	.LBB9_178
.LBB9_198:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w8, wzr
	b	.LBB9_210
.LBB9_199:                              //   in Loop: Header=BB9_70 Depth=3
	cmp	w26, #2040              // =2040
	b.hi	.LBB9_202
// %bb.200:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w9, #37367
	sub	w8, w26, #4             // =4
	movk	w9, #321, lsl #16
	umull	x9, w8, w9
	lsr	x9, x9, #32
	sub	w8, w8, w9
	add	w8, w9, w8, lsr #1
	sub	w8, w26, w8, lsr #9
	sub	w8, w8, #2              // =2
	b	.LBB9_203
.LBB9_201:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x22, [sp, #240]         // 8-byte Folded Reload
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	mov	w27, #-2
	mov	w25, #1
	mov	x21, x26
	b	.LBB9_309
.LBB9_202:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w9, #37367
	sub	w8, w26, #2043          // =2043
	movk	w9, #321, lsl #16
	umull	x9, w8, w9
	lsr	x9, x9, #32
	sub	w8, w8, w9
	add	w8, w9, w8, lsr #1
	sub	w8, w26, w8, lsr #9
	sub	w8, w8, #5              // =5
.LBB9_203:                              //   in Loop: Header=BB9_70 Depth=3
	ldrh	w9, [x25]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB9_205
// %bb.204:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x10, [x25, #736]
	tbnz	w10, #31, .LBB9_230
.LBB9_205:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w10, #1018
.LBB9_206:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w11, [x25, #1184]
	ldr	w12, [x25, #1192]
	mul	w8, w10, w8
	cmp	w9, #8, lsl #12         // =32768
	mov	w9, #923
	add	w10, w12, w11, asr #2
	sub	w9, w9, w10
	b.ne	.LBB9_209
// %bb.207:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x10, [x25, #736]
	tbz	w10, #31, .LBB9_209
// %bb.208:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	w10, [x25, #1292]
	neg	w10, w10
	and	w9, w9, w10
.LBB9_209:                              //   in Loop: Header=BB9_70 Depth=3
	add	w8, w9, w8
.LBB9_210:                              //   in Loop: Header=BB9_70 Depth=3
	ldrb	w9, [x25, #13]
	add	w21, w8, w21
	tbz	w9, #6, .LBB9_212
// %bb.211:                             //   in Loop: Header=BB9_70 Depth=3
	ldrh	w8, [x25]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.eq	.LBB9_213
.LBB9_212:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x25, #664]
	cbz	x8, .LBB9_221
.LBB9_213:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x25, #40]
	mov	w1, w21
	ldr	x26, [x8, #1008]
.LBB9_214:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [sp, #64]           // 8-byte Folded Reload
	ldr	x0, [x25, #48]
	stp	xzr, xzr, [x29, #-176]
	stp	xzr, xzr, [x29, #-192]
	stp	xzr, xzr, [x29, #-208]
	stur	wzr, [x29, #-216]
	stur	xzr, [x29, #-224]
	stp	xzr, xzr, [x8, #96]
	stp	xzr, xzr, [x8, #80]
	stp	xzr, xzr, [x8, #64]
	stp	xzr, xzr, [x8, #48]
	stp	xzr, xzr, [x8, #32]
	stp	xzr, xzr, [x8, #16]
	stp	xzr, xzr, [x8]
	stur	x26, [x29, #-160]
	ldr	x8, [x25, #64]
	mov	x9, #8589934592
	mov	w2, #2
	str	x27, [sp, #48]          // 8-byte Folded Spill
	stur	w8, [x29, #-152]
	sub	x8, x29, #160           // =160
	stur	x9, [x8, #12]
	stur	wzr, [x29, #-140]
	sturb	wzr, [x29, #-72]
	sturb	wzr, [x29, #-70]
	mov	x27, x1
	bl	grab_cache_page_write_begin
	cbz	x0, .LBB9_220
// %bb.215:                             //   in Loop: Header=BB9_70 Depth=3
	mov	x22, x0
	sub	x2, x29, #224           // =224
	mov	x0, x25
	mov	x1, x27
	bl	f2fs_lookup_extent_cache
	tbz	w0, #0, .LBB9_225
// %bb.216:                             //   in Loop: Header=BB9_70 Depth=3
	ldur	w8, [x29, #-216]
	ldur	w9, [x29, #-224]
	mov	w2, #8
	mov	x0, x26
	add	w8, w8, w21
	sub	w1, w8, w9
	stur	w1, [x29, #-172]
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB9_355
.LBB9_217:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	w8, [x29, #-172]
	mov	w2, #1
	mov	w3, #1
	mov	x0, x22
	mov	w1, wzr
	stur	x22, [x29, #-120]
	stp	w8, w8, [x29, #-132]
	mov	x21, x22
	bl	f2fs_wait_on_page_writeback
	ldur	w1, [x29, #-172]
	mov	x0, x25
	bl	f2fs_wait_on_block_writeback
	ldr	x8, [x26, #248]
	ldur	w1, [x29, #-172]
	mov	w2, #6
	mov	w3, #3136
	ldr	x0, [x8, #48]
	bl	pagecache_get_page
	ldr	x27, [sp, #48]          // 8-byte Folded Reload
	stur	x0, [x29, #-112]
	cbz	x0, .LBB9_229
// %bb.218:                             //   in Loop: Header=BB9_70 Depth=3
	sub	x0, x29, #160           // =160
	bl	f2fs_submit_page_bio
	ldur	x8, [x29, #-112]
	cbz	w0, .LBB9_233
// %bb.219:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w22, w0
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_put_page
	b	.LBB9_257
.LBB9_220:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x27, [sp, #48]          // 8-byte Folded Reload
	mov	w22, #-12
	b	.LBB9_258
.LBB9_221:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x25, #40]
	ldr	x26, [x8, #1008]
	ldr	w8, [x26, #1532]
	cbz	w8, .LBB9_231
// %bb.222:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	w8, [x25, #1280]
	cbnz	w8, .LBB9_213
// %bb.223:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x25, #736]
	tbnz	x8, #35, .LBB9_213
// %bb.224:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x25, #736]
	mov	w1, w21
	tbnz	x8, #34, .LBB9_214
	b	.LBB9_243
.LBB9_225:                              //   in Loop: Header=BB9_70 Depth=3
	sub	x0, x29, #208           // =208
	mov	w2, #48
	mov	w1, wzr
	mov	x21, x22
	bl	memset
	sub	x0, x29, #208           // =208
	mov	w2, #1
	mov	x1, x27
	stp	x25, xzr, [x29, #-208]
	stur	xzr, [x29, #-192]
	stur	wzr, [x29, #-184]
	bl	f2fs_get_dnode_of_data
	ldr	x27, [sp, #48]          // 8-byte Folded Reload
	mov	w22, w0
	cbnz	w0, .LBB9_257
// %bb.226:                             //   in Loop: Header=BB9_70 Depth=3
	ldur	x0, [x29, #-192]
	cbz	x0, .LBB9_250
// %bb.227:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-192]
	b	.LBB9_251
.LBB9_228:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w8, [x21, #724]
	ldr	x9, [x21, #40]
	add	w8, w8, #1              // =1
	str	w8, [x21, #724]
	ldr	x8, [x9, #1008]
	ldr	x9, [sp, #144]          // 8-byte Folded Reload
	add	x8, x8, x9, lsl #3
	ldr	x9, [x8, #2008]
	add	x9, x9, #1              // =1
	str	x9, [x8, #2008]
	b	.LBB9_308
.LBB9_229:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w22, #-12
	b	.LBB9_257
.LBB9_230:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w10, [x25, #1292]
	mov	w11, #1018
	neg	w10, w10
	and	w10, w10, w11
	b	.LBB9_206
.LBB9_231:                              //   in Loop: Header=BB9_70 Depth=3
	ldrh	w8, [x25]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB9_242
// %bb.232:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x25, #736]
	mov	w1, w21
	tbz	w8, #31, .LBB9_243
	b	.LBB9_214
.LBB9_233:                              //   in Loop: Header=BB9_70 Depth=3
	cbz	x8, .LBB9_236
// %bb.234:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x9, [x8, #8]
	mov	w11, #1
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w11
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB9_236
// %bb.235:                             //   in Loop: Header=BB9_70 Depth=3
	bl	__put_page
.LBB9_236:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_put_page
	ldrb	w8, [x26, #2704]
	cbz	w8, .LBB9_239
// %bb.237:                             //   in Loop: Header=BB9_70 Depth=3
	add	x22, x26, #2344         // =2344
	mov	x0, x22
	bl	_raw_spin_lock
	ldr	x8, [x26, #2480]
	mov	x0, x22
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x26, #2480]
	bl	_raw_spin_unlock
	mov	x0, x26
	bl	f2fs_record_iostat
	ldrb	w8, [x26, #2704]
	cbz	w8, .LBB9_239
// %bb.238:                             //   in Loop: Header=BB9_70 Depth=3
	mov	x0, x22
	bl	_raw_spin_lock
	ldr	x8, [x26, #2488]
	mov	x0, x22
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x26, #2488]
	bl	_raw_spin_unlock
	mov	x0, x26
	bl	f2fs_record_iostat
.LBB9_239:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
	ldr	x19, [sp, #248]         // 8-byte Folded Reload
	ldr	x27, [sp, #48]          // 8-byte Folded Reload
	mov	w22, wzr
	b	.LBB9_258
.LBB9_240:                              //   in Loop: Header=BB9_70 Depth=3
	cmp	w22, #1                 // =1
	b.ne	.LBB9_308
// %bb.241:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_pin_file_control
	b	.LBB9_308
.LBB9_242:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, w21
.LBB9_243:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w2, #1048576
	mov	w3, #1
	mov	x0, x25
	bl	f2fs_get_read_data_page
	mov	x22, x0
	mov	x0, x27
	bl	up_write
	cmn	x22, #4095              // =4095
	b.hs	.LBB9_259
// %bb.244:                             //   in Loop: Header=BB9_70 Depth=3
	cbz	x22, .LBB9_247
// %bb.245:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x22, #8]
	mov	w11, #1
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x22, x9, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w11
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB9_247
// %bb.246:                             //   in Loop: Header=BB9_70 Depth=3
	bl	__put_page
.LBB9_247:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w1, [x25, #64]
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	radix_tree_lookup
	cbz	x0, .LBB9_295
// %bb.248:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x0, #16]
	b	.LBB9_296
.LBB9_249:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w27, #-12
	b	.LBB9_326
.LBB9_250:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x9, xzr
.LBB9_251:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	x8, [x29, #-200]
	cbz	x8, .LBB9_255
// %bb.252:                             //   in Loop: Header=BB9_70 Depth=3
	cmp	x9, x8
	b.eq	.LBB9_255
// %bb.253:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x9, [x8, #8]
	mov	w11, #1
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w11
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB9_255
// %bb.254:                             //   in Loop: Header=BB9_70 Depth=3
	bl	__put_page
.LBB9_255:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	w1, [x29, #-172]
	stp	xzr, xzr, [x29, #-200]
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB9_271
.LBB9_256:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w22, #-2
.LBB9_257:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_put_page
.LBB9_258:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x0, x27
	bl	up_write
	cbz	w22, .LBB9_260
.LBB9_259:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x0, x25
	bl	iput
	b	.LBB9_103
.LBB9_260:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w1, [x25, #64]
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	radix_tree_lookup
	cbz	x0, .LBB9_262
// %bb.261:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x0, #16]
	b	.LBB9_263
.LBB9_262:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x8, xzr
.LBB9_263:                              //   in Loop: Header=BB9_70 Depth=3
	cmp	x8, x25
	b.eq	.LBB9_259
// %bb.264:                             //   in Loop: Header=BB9_70 Depth=3
	adrp	x8, f2fs_inode_entry_slab
	ldr	x21, [x8, :lo12:f2fs_inode_entry_slab]
	mov	w1, #3136
	mov	x0, x21
	bl	kmem_cache_alloc
	mov	x22, x0
	cbnz	x0, .LBB9_266
// %bb.265:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #35904
	mov	x0, x21
	bl	kmem_cache_alloc
	mov	x22, x0
.LBB9_266:                              //   in Loop: Header=BB9_70 Depth=3
	str	x25, [x22, #16]
	ldr	x26, [x25, #64]
	ldr	x21, [sp, #72]          // 8-byte Folded Reload
.LBB9_267:                              //   Parent Loop BB9_12 Depth=1
                                        //     Parent Loop BB9_68 Depth=2
                                        //       Parent Loop BB9_70 Depth=3
                                        // =>      This Inner Loop Header: Depth=4
	mov	x0, x21
	mov	x1, x26
	mov	x2, x22
	bl	radix_tree_insert
	cbnz	w0, .LBB9_267
// %bb.268:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x21, [sp, #104]         // 8-byte Folded Reload
	mov	x0, x22
	ldr	x26, [x21, #8]
	mov	x2, x21
	mov	x1, x26
	bl	__list_add_valid
	mov	w25, #1
	tbz	w0, #0, .LBB9_270
// %bb.269:                             //   in Loop: Header=BB9_70 Depth=3
	str	x22, [x21, #8]
	stp	x21, x26, [x22]
	str	x22, [x26]
.LBB9_270:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x22, [sp, #240]         // 8-byte Folded Reload
	b	.LBB9_105
.LBB9_271:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w9, [x26, #3128]
	cbz	w9, .LBB9_279
// %bb.272:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x26]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB9_278
// %bb.273:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x10, [x26, #3136]
	add	x10, x10, #80           // =80
	b	.LBB9_275
.LBB9_274:                              //   in Loop: Header=BB9_275 Depth=4
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB9_278
.LBB9_275:                              //   Parent Loop BB9_12 Depth=1
                                        //     Parent Loop BB9_68 Depth=2
                                        //       Parent Loop BB9_70 Depth=3
                                        // =>      This Inner Loop Header: Depth=4
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB9_274
// %bb.276:                             //   in Loop: Header=BB9_275 Depth=4
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB9_274
// %bb.277:                             //   in Loop: Header=BB9_70 Depth=3
	ldur	x8, [x10, #-80]
.LBB9_278:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
	ldr	x19, [sp, #248]         // 8-byte Folded Reload
	cbz	x8, .LBB9_319
.LBB9_279:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w2, #7
	mov	x0, x26
	bl	f2fs_is_valid_blkaddr
	mov	x22, x21
	tbnz	w0, #0, .LBB9_217
	b	.LBB9_356
.LBB9_280:                              //   in Loop: Header=BB9_70 Depth=3
	//APP
	dmb ishld
	//NO_APP
.LBB9_281:                              //   in Loop: Header=BB9_70 Depth=3
	ldp	w8, w9, [x29, #-184]
	ldurb	w10, [x29, #-212]
	ldur	x0, [x29, #-160]
	ldur	w2, [x29, #-128]
	ldr	w5, [sp, #24]           // 4-byte Folded Reload
	stur	w8, [x29, #-24]
	sub	x8, x29, #160           // =160
	sub	x3, x29, #228           // =228
	sub	x4, x29, #24            // =24
	mov	x1, xzr
	mov	x6, xzr
	mov	w7, wzr
	sturh	w9, [x8, #141]
	sturb	w10, [x29, #-20]
	str	wzr, [sp]
	bl	f2fs_allocate_data_block
	ldur	x8, [x29, #-160]
	ldur	w1, [x29, #-228]
	mov	w2, #6
	mov	w3, #3136
	ldr	x8, [x8, #248]
	ldr	x0, [x8, #48]
	bl	pagecache_get_page
	stur	x0, [x29, #-112]
	cbz	x0, .LBB9_305
// %bb.282:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w2, #1
	mov	w3, #1
	mov	w1, wzr
	bl	f2fs_wait_on_page_writeback
	ldur	x8, [x29, #-112]
	mov	x10, #2097152
	movk	x10, #1, lsl #32
	add	x9, x23, x10
	add	x8, x8, x10
	lsl	x9, x9, #6
	lsl	x8, x8, #6
	and	x9, x9, #0xfffffffffffff000
	and	x8, x8, #0xfffffffffffff000
	mov	x10, #-549755813888
	add	x0, x8, x10
	add	x1, x9, x10
	mov	w2, #4096
	bl	memcpy
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_put_page
	ldur	x8, [x29, #-160]
	ldur	w1, [x29, #-128]
	ldr	x8, [x8, #248]
	mov	x2, x1
	ldr	x0, [x8, #48]
	bl	invalidate_mapping_pages
	ldur	x0, [x29, #-160]
	ldur	w1, [x29, #-128]
	bl	f2fs_invalidate_compress_page
	ldur	x0, [x29, #-112]
	bl	set_page_dirty
	ldur	x0, [x29, #-112]
	bl	clear_page_dirty_for_io
	cbz	w0, .LBB9_284
// %bb.283:                             //   in Loop: Header=BB9_70 Depth=3
	ldur	x8, [x29, #-160]
	add	x8, x8, #1288           // =1288
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w19
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB9_284:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	x0, [x29, #-112]
	mov	w1, wzr
	bl	__test_set_page_writeback
	ldr	x10, [sp, #48]          // 8-byte Folded Reload
	mov	w11, #256
	sub	x0, x29, #160           // =160
	ldr	x8, [x10, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x10, x9, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x11
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldur	w8, [x29, #-228]
	mov	x10, #1
	sub	x9, x29, #160           // =160
	movk	x10, #2048, lsl #32
	stur	x10, [x9, #20]
	stur	w8, [x29, #-132]
	bl	f2fs_submit_page_write
	ldurb	w8, [x29, #-70]
	cbz	w8, .LBB9_320
// %bb.285:                             //   in Loop: Header=BB9_70 Depth=3
	ldur	x0, [x29, #-112]
	ldr	x8, [x0, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x0, x9, eq
	ldr	x8, [x8]
	tbz	w8, #15, .LBB9_287
// %bb.286:                             //   in Loop: Header=BB9_70 Depth=3
	bl	end_page_writeback
	ldur	x0, [x29, #-112]
.LBB9_287:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	bl	f2fs_put_page
	mov	w27, #-11
	b	.LBB9_306
.LBB9_288:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x10, [sp, #48]          // 8-byte Folded Reload
	ldr	x8, [x10, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x10, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #15, .LBB9_307
// %bb.289:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x19, [sp, #48]          // 8-byte Folded Reload
	mov	x0, x19
	bl	set_page_dirty
	ldr	x8, [x19]
	tbnz	w8, #13, .LBB9_291
// %bb.290:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x11, [sp, #48]          // 8-byte Folded Reload
	ldr	x8, [x11, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x11, x9, eq
	add	x8, x8, #52             // =52
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x11]
1:	ldxr	x8, [x11]
	orr	x8, x8, 8192
	stxr	w9, x8, [x11]
	cbnz	w9, 1b
	//NO_APP
	str	xzr, [x11, #40]
.LBB9_291:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	mov	w27, wzr
	mov	w25, #1
	add	x8, x0, #40             // =40
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 1
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 8
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
	ldp	x22, x19, [sp, #240]    // 16-byte Folded Reload
	ldr	x24, [sp, #24]          // 8-byte Folded Reload
	b	.LBB9_309
.LBB9_292:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x26, [sp, #48]          // 8-byte Folded Reload
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	tbz	w8, #3, .LBB9_294
// %bb.293:                             //   in Loop: Header=BB9_70 Depth=3
	mov	x0, x26
	bl	set_page_dirty
.LBB9_294:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
	ldp	x22, x19, [sp, #240]    // 16-byte Folded Reload
	ldr	x24, [sp, #24]          // 8-byte Folded Reload
	ldr	x21, [sp, #8]           // 8-byte Folded Reload
	mov	w25, #1
	mov	x0, x26
	b	.LBB9_309
.LBB9_295:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x8, xzr
.LBB9_296:                              //   in Loop: Header=BB9_70 Depth=3
	cmp	x8, x25
	b.eq	.LBB9_303
// %bb.297:                             //   in Loop: Header=BB9_70 Depth=3
	adrp	x8, f2fs_inode_entry_slab
	ldr	x19, [x8, :lo12:f2fs_inode_entry_slab]
	mov	w1, #3136
	mov	x0, x19
	bl	kmem_cache_alloc
	mov	x22, x0
	cbnz	x0, .LBB9_299
// %bb.298:                             //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #35904
	mov	x0, x19
	bl	kmem_cache_alloc
	mov	x22, x0
.LBB9_299:                              //   in Loop: Header=BB9_70 Depth=3
	str	x25, [x22, #16]
	ldr	x26, [x25, #64]
	ldr	x21, [sp, #72]          // 8-byte Folded Reload
.LBB9_300:                              //   Parent Loop BB9_12 Depth=1
                                        //     Parent Loop BB9_68 Depth=2
                                        //       Parent Loop BB9_70 Depth=3
                                        // =>      This Inner Loop Header: Depth=4
	mov	x0, x21
	mov	x1, x26
	mov	x2, x22
	bl	radix_tree_insert
	cbnz	w0, .LBB9_300
// %bb.301:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x2, [sp, #104]          // 8-byte Folded Reload
	mov	x0, x22
	ldr	x26, [x2, #8]
	mov	x1, x26
	bl	__list_add_valid
	tbz	w0, #0, .LBB9_304
// %bb.302:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [sp, #104]          // 8-byte Folded Reload
	str	x22, [x8, #8]
	stp	x8, x26, [x22]
	str	x22, [x26]
	b	.LBB9_304
.LBB9_303:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x0, x25
	bl	iput
.LBB9_304:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
	ldp	x22, x19, [sp, #240]    // 16-byte Folded Reload
	b	.LBB9_104
.LBB9_305:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_put_page
	mov	w27, #-12
.LBB9_306:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	x0, [x29, #-160]
	ldur	w2, [x29, #-228]
	ldur	w3, [x29, #-128]
	sub	x1, x29, #24            // =24
	mov	w4, #1
	mov	w5, #1
	mov	w6, #1
	bl	f2fs_do_replace_block
	b	.LBB9_325
.LBB9_307:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
	ldp	x22, x19, [sp, #240]    // 16-byte Folded Reload
	ldr	x24, [sp, #24]          // 8-byte Folded Reload
.LBB9_308:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	mov	w27, #-11
	mov	w25, #1
.LBB9_309:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	b	.LBB9_339
.LBB9_310:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	x22, [x29, #-160]
	ldrb	w8, [x22, #2704]
	cbz	w8, .LBB9_313
// %bb.311:                             //   in Loop: Header=BB9_70 Depth=3
	add	x26, x22, #2344         // =2344
	mov	x0, x26
	bl	_raw_spin_lock
	ldr	x8, [x22, #2480]
	mov	x0, x26
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x22, #2480]
	bl	_raw_spin_unlock
	mov	x0, x22
	bl	f2fs_record_iostat
	ldur	x22, [x29, #-160]
	ldrb	w8, [x22, #2704]
	cbz	w8, .LBB9_313
// %bb.312:                             //   in Loop: Header=BB9_70 Depth=3
	add	x26, x22, #2344         // =2344
	mov	x0, x26
	bl	_raw_spin_lock
	ldr	x8, [x22, #2488]
	mov	x0, x26
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x22, #2488]
	bl	_raw_spin_unlock
	mov	x0, x22
	bl	f2fs_record_iostat
.LBB9_313:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x23, #8]
	mov	x10, x23
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x23, x9, eq
	ldr	x9, [x8]
	str	x9, [sp, #256]
	ldr	x9, [sp, #256]
	tbnz	w9, #0, .LBB9_315
// %bb.314:                             //   in Loop: Header=BB9_70 Depth=3
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbz	w9, #0, .LBB9_316
.LBB9_315:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x0, x23
	bl	__lock_page
.LBB9_316:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	x8, [x29, #-160]
	ldr	x9, [x23, #24]
	ldr	x8, [x8, #248]
	ldr	x8, [x8, #48]
	cmp	x9, x8
	b.ne	.LBB9_318
// %bb.317:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x23, #8]
	mov	x10, x23
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x23, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB9_330
.LBB9_318:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_put_page
	mov	w27, #-5
	b	.LBB9_325
.LBB9_319:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x27, [sp, #48]          // 8-byte Folded Reload
	b	.LBB9_256
.LBB9_320:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	x22, [x29, #-160]
	ldrb	w8, [x22, #2704]
	cbz	w8, .LBB9_322
// %bb.321:                             //   in Loop: Header=BB9_70 Depth=3
	add	x26, x22, #2344         // =2344
	mov	x0, x26
	bl	_raw_spin_lock
	ldr	x8, [x22, #2408]
	mov	x0, x26
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x22, #2408]
	bl	_raw_spin_unlock
	mov	x0, x22
	bl	f2fs_record_iostat
.LBB9_322:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	w1, [x29, #-228]
	sub	x0, x29, #208           // =208
	bl	f2fs_update_data_blkaddr
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4096
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x9, [sp, #48]           // 8-byte Folded Reload
	ldr	x9, [x9, #32]
	cbnz	x9, .LBB9_324
// %bb.323:                             //   in Loop: Header=BB9_70 Depth=3
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 262144
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB9_324:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	x0, [x29, #-112]
	mov	w1, #1
	bl	f2fs_put_page
	mov	w27, wzr
.LBB9_325:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w23, [sp, #180]         // 4-byte Folded Reload
	ldp	x22, x19, [sp, #240]    // 16-byte Folded Reload
	ldr	x26, [sp, #48]          // 8-byte Folded Reload
	mov	w25, #1
.LBB9_326:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w8, [sp, #16]           // 4-byte Folded Reload
	cmp	w8, #1                  // =1
	b.ne	.LBB9_328
// %bb.327:                             //   in Loop: Header=BB9_70 Depth=3
	ldur	x8, [x29, #-160]
	add	x0, x8, #168            // =168
	bl	up_write
.LBB9_328:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	x0, [x29, #-192]
	cbz	x0, .LBB9_332
.LBB9_329:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-192]
	b	.LBB9_333
.LBB9_330:                              //   in Loop: Header=BB9_70 Depth=3
	//APP
	dmb ishld
	//NO_APP
	b	.LBB9_281
.LBB9_331:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x26, #8]
	mov	w11, #4
	mov	w27, #-2
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x26, x9, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x11
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldur	x0, [x29, #-192]
	cbnz	x0, .LBB9_329
.LBB9_332:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x9, xzr
.LBB9_333:                              //   in Loop: Header=BB9_70 Depth=3
	ldur	x8, [x29, #-200]
	cbz	x8, .LBB9_337
// %bb.334:                             //   in Loop: Header=BB9_70 Depth=3
	cmp	x9, x8
	b.eq	.LBB9_337
// %bb.335:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w25
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB9_337
// %bb.336:                             //   in Loop: Header=BB9_70 Depth=3
	bl	__put_page
.LBB9_337:                              //   in Loop: Header=BB9_70 Depth=3
	stp	xzr, xzr, [x29, #-200]
.LBB9_338:                              //   in Loop: Header=BB9_70 Depth=3
	mov	w1, #1
	mov	x0, x26
.LBB9_339:                              //   in Loop: Header=BB9_70 Depth=3
	bl	f2fs_put_page
.LBB9_340:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w9, [sp, #40]           // 4-byte Folded Reload
	cbnz	w27, .LBB9_346
// %bb.341:                             //   in Loop: Header=BB9_70 Depth=3
	cmp	w22, #1                 // =1
	b.eq	.LBB9_345
// %bb.342:                             //   in Loop: Header=BB9_70 Depth=3
	ldrb	w8, [x21, #13]
	tbz	w8, #6, .LBB9_344
// %bb.343:                             //   in Loop: Header=BB9_70 Depth=3
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.eq	.LBB9_345
.LBB9_344:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x21, #664]
	cbz	x8, .LBB9_349
.LBB9_345:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	w8, [sp, #172]          // 4-byte Folded Reload
	add	w8, w8, #1              // =1
	str	w8, [sp, #172]          // 4-byte Folded Spill
.LBB9_346:                              //   in Loop: Header=BB9_70 Depth=3
	cbz	w9, .LBB9_348
// %bb.347:                             //   in Loop: Header=BB9_70 Depth=3
	add	x0, x21, #1040          // =1040
	bl	up_write
	add	x0, x21, #992           // =992
	bl	up_write
.LBB9_348:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x19, #2120]
	ldr	w12, [sp, #140]         // 4-byte Folded Reload
	cmp	w27, #0                 // =0
	ldr	w9, [x8, #444]
	ldr	w10, [x8, #448]
	ldr	w11, [x8, #456]
	add	w9, w9, #1              // =1
	add	w10, w10, #1            // =1
	add	w11, w11, w12
	str	w9, [x8, #444]
	str	w10, [x8, #448]
	str	w11, [x8, #456]
	ldr	w8, [sp, #100]          // 4-byte Folded Reload
	cinc	w8, w8, eq
	str	w8, [sp, #100]          // 4-byte Folded Spill
	b	.LBB9_105
.LBB9_349:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x21, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB9_353
// %bb.350:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	w8, [x21, #1280]
	cbnz	w8, .LBB9_345
// %bb.351:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x21, #736]
	tbnz	x8, #35, .LBB9_345
// %bb.352:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x21, #736]
	tbnz	x8, #34, .LBB9_345
	b	.LBB9_346
.LBB9_353:                              //   in Loop: Header=BB9_70 Depth=3
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB9_346
// %bb.354:                             //   in Loop: Header=BB9_70 Depth=3
	ldr	x8, [x21, #736]
	tbz	w8, #31, .LBB9_346
	b	.LBB9_345
.LBB9_355:                              //   in Loop: Header=BB9_70 Depth=3
	mov	x21, x22
.LBB9_356:                              //   in Loop: Header=BB9_70 Depth=3
	ldr	x27, [sp, #48]          // 8-byte Folded Reload
	mov	w22, #-117
	b	.LBB9_257
.LBB9_357:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x21, [sp, #200]         // 8-byte Folded Reload
	mov	x0, x21
	bl	mutex_lock
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB9_360
// %bb.358:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [sp, #144]          // 8-byte Folded Reload
	ldr	w10, [sp, #100]         // 4-byte Folded Reload
	add	x8, x8, x9, lsl #2
	ldr	w9, [x8, #120]
	add	w9, w9, w10
	str	w9, [x8, #120]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB9_360
// %bb.359:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [x8, #416]
	ldr	w10, [sp, #100]         // 4-byte Folded Reload
	add	x9, x9, w10, sxtw
	str	x9, [x8, #416]
.LBB9_360:                              //   in Loop: Header=BB9_12 Depth=1
	mov	x0, x21
	bl	mutex_unlock
	ldr	x27, [sp, #216]         // 8-byte Folded Reload
	ldr	x20, [sp, #192]         // 8-byte Folded Reload
.LBB9_361:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x19, #2120]
	ldr	w11, [sp, #156]         // 4-byte Folded Reload
	ldr	w10, [sp, #172]         // 4-byte Folded Reload
	ldr	x12, [sp, #208]         // 8-byte Folded Reload
	ldr	w9, [x8, #416]
	add	w11, w10, w11
	ldr	x10, [sp, #224]         // 8-byte Folded Reload
	add	w9, w9, #1              // =1
	str	w9, [x8, #416]
	str	w11, [sp, #156]         // 4-byte Folded Spill
	cmp	w10, #2                 // =2
	b.ls	.LBB9_363
// %bb.362:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	w9, [x8, #420]
	ldr	w10, [x8, #436]
	ldr	w11, [sp, #140]         // 4-byte Folded Reload
	add	w9, w9, #1              // =1
	str	w9, [x8, #420]
	add	w10, w10, w11
	str	w10, [x8, #436]
	b	.LBB9_364
.LBB9_363:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	w9, [x8, #424]
	ldr	w10, [x8, #440]
	ldr	w11, [sp, #140]         // 4-byte Folded Reload
	add	w9, w9, #1              // =1
	str	w9, [x8, #424]
	add	w10, w10, w11
	str	w10, [x8, #440]
.LBB9_364:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	w8, [x19, #1988]
	add	x8, x19, x8, lsl #2
	ldr	w9, [x8, #3248]
	add	w9, w9, #1              // =1
	str	w9, [x8, #3248]
	ldr	w8, [sp, #176]          // 4-byte Folded Reload
	add	w8, w8, #1              // =1
	str	w8, [sp, #176]          // 4-byte Folded Spill
.LBB9_365:                              //   in Loop: Header=BB9_12 Depth=1
	mov	x26, x12
	cmp	w22, #1                 // =1
	b.ne	.LBB9_367
// %bb.366:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x19, #136]
	mov	w9, #40
	mul	x9, x24, x9
	mov	x0, x21
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	ldr	w8, [x8, x9]
	tst	w8, #0xffc0
	cinc	w27, w27, eq
	bl	mutex_lock
	b	.LBB9_368
.LBB9_367:                              //   in Loop: Header=BB9_12 Depth=1
	mov	x0, x21
	bl	mutex_lock
	cbz	w22, .LBB9_372
.LBB9_368:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [x19, #136]
	ldr	w8, [x19, #1192]
	ldr	x9, [x9]
	cmp	w8, #2                  // =2
	b.lo	.LBB9_371
// %bb.369:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [x9, #128]
	udiv	w8, w24, w8
	ldr	w8, [x9, w8, uxtw #2]
	cbz	w8, .LBB9_372
.LBB9_370:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x19, #2328]
	cbnz	x8, .LBB9_379
	b	.LBB9_383
.LBB9_371:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x9, #120]
	mov	w9, #40
	mul	x9, x24, x9
	ldr	w8, [x8, x9]
	ubfx	w8, w8, #6, #10
	cbnz	w8, .LBB9_370
.LBB9_372:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [sp, #224]          // 8-byte Folded Reload
	ldr	x8, [x19, #2328]
	cmp	w9, #3                  // =3
	b.lo	.LBB9_375
// %bb.373:                             //   in Loop: Header=BB9_12 Depth=1
	cbz	x8, .LBB9_383
// %bb.374:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [sp, #144]          // 8-byte Folded Reload
	add	x8, x8, x9, lsl #2
	add	x8, x8, #128            // =128
	b	.LBB9_377
.LBB9_375:                              //   in Loop: Header=BB9_12 Depth=1
	cbz	x8, .LBB9_383
// %bb.376:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [sp, #144]          // 8-byte Folded Reload
	add	x8, x8, x9, lsl #2
	add	x8, x8, #112            // =112
.LBB9_377:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	w9, [x8]
	add	w9, w9, #1              // =1
	str	w9, [x8]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB9_383
// %bb.378:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [sp, #160]          // 8-byte Folded Reload
	add	x8, x8, x9, lsl #3
	ldr	x9, [x8, #440]
	add	x9, x9, #1              // =1
	str	x9, [x8, #440]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB9_383
.LBB9_379:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x10, [x19, #136]
	ldr	w9, [x19, #1192]
	ldr	x10, [x10]
	cmp	w9, #2                  // =2
	b.lo	.LBB9_381
// %bb.380:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x10, [x10, #128]
	udiv	w9, w24, w9
	ldr	w9, [x10, w9, uxtw #2]
	b	.LBB9_382
.LBB9_381:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [x10, #120]
	mov	w10, #40
	mul	x10, x24, x10
	ldr	w9, [x9, x10]
	ubfx	w9, w9, #6, #10
.LBB9_382:                              //   in Loop: Header=BB9_12 Depth=1
	ldr	x10, [sp, #160]         // 8-byte Folded Reload
	add	x8, x8, x10, lsl #3
	ldr	x10, [x8, #496]
	add	x9, x10, w9, uxtw
	str	x9, [x8, #496]
.LBB9_383:                              //   in Loop: Header=BB9_12 Depth=1
	mov	x0, x21
	bl	mutex_unlock
	ldr	w8, [x19, #1192]
	cmp	w8, #2                  // =2
	b.lo	.LBB9_386
// %bb.384:                             //   in Loop: Header=BB9_12 Depth=1
	add	x8, x24, #1             // =1
	cmp	x8, x20
	mov	x12, x26
	b.hs	.LBB9_387
// %bb.385:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [sp, #112]          // 8-byte Folded Reload
	str	w8, [x9]
	b	.LBB9_387
.LBB9_386:                              //   in Loop: Header=BB9_12 Depth=1
	mov	x12, x26
.LBB9_387:                              //   in Loop: Header=BB9_12 Depth=1
	cbz	x12, .LBB9_11
// %bb.388:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x8, [x12, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x12, x9, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w25
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB9_11
// %bb.389:                             //   in Loop: Header=BB9_12 Depth=1
	bl	__put_page
	b	.LBB9_11
.LBB9_390:                              //   in Loop: Header=BB9_12 Depth=1
	cmp	w22, #1                 // =1
	mov	w25, #1
	b.ne	.LBB9_392
// %bb.391:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x10, [sp, #128]         // 8-byte Folded Reload
	//APP
	// atomic_sub
	prfm	pstl1strm, [x10]
1:	ldxr	w8, [x10]
	sub	w8, w8, w25
	stxr	w9, w8, [x10]
	cbnz	w9, 1b

	//NO_APP
.LBB9_392:                              //   in Loop: Header=BB9_12 Depth=1
	mov	x0, x21
	bl	mutex_lock
	ldr	x19, [sp, #248]         // 8-byte Folded Reload
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB9_397
// %bb.393:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [sp, #144]          // 8-byte Folded Reload
	ldr	w10, [sp, #236]         // 4-byte Folded Reload
	add	x8, x8, x9, lsl #2
	ldr	w9, [x8, #136]
	add	w9, w9, w10
	str	w9, [x8, #136]
	b	.LBB9_397
.LBB9_394:                              //   in Loop: Header=BB9_12 Depth=1
	mov	x0, x21
	bl	mutex_lock
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB9_397
// %bb.395:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [sp, #144]          // 8-byte Folded Reload
	ldr	w10, [sp, #100]         // 4-byte Folded Reload
	add	x8, x8, x9, lsl #2
	ldr	w9, [x8, #120]
	add	w9, w9, w10
	str	w9, [x8, #120]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB9_397
// %bb.396:                             //   in Loop: Header=BB9_12 Depth=1
	ldr	x9, [x8, #416]
	ldr	w10, [sp, #100]         // 4-byte Folded Reload
	add	x9, x9, w10, sxtw
	str	x9, [x8, #416]
.LBB9_397:                              //   in Loop: Header=BB9_12 Depth=1
	mov	x0, x21
	bl	mutex_unlock
	b	.LBB9_361
.LBB9_398:
	add	x0, sp, #264            // =264
	bl	blk_start_plug
.LBB9_399:
	mov	w27, wzr
.LBB9_400:
	add	x0, sp, #264            // =264
	bl	blk_finish_plug
	ldr	x8, [x19, #2120]
	ldr	w9, [x8, #404]
	add	w9, w9, #1              // =1
	str	w9, [x8, #404]
.LBB9_401:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB9_421
// %bb.402:
	mov	w0, w27
	add	sp, sp, #576            // =576
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.LBB9_403:
	cmp	w22, w25
	b.ls	.LBB9_401
// %bb.404:
	mov	w21, #1
	b	.LBB9_406
.LBB9_405:                              //   in Loop: Header=BB9_406 Depth=1
	subs	w26, w26, #1            // =1
	add	w25, w25, #1            // =1
	b.eq	.LBB9_401
.LBB9_406:                              // =>This Inner Loop Header: Depth=1
	ldr	x8, [x19, #136]
	ldr	x9, [x19, #248]
	mov	w2, wzr
	mov	w3, wzr
	ldr	w8, [x8, #88]
	ldr	x0, [x9, #48]
	add	w1, w25, w8
	bl	pagecache_get_page
	cbz	x0, .LBB9_405
// %bb.407:                             //   in Loop: Header=BB9_406 Depth=1
	ldr	x8, [x0, #8]
	mov	x20, x0
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x0, x9, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w21
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB9_409
// %bb.408:                             //   in Loop: Header=BB9_406 Depth=1
	bl	__put_page
.LBB9_409:                              //   in Loop: Header=BB9_406 Depth=1
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x20, x9, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w21
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB9_405
// %bb.410:                             //   in Loop: Header=BB9_406 Depth=1
	bl	__put_page
	b	.LBB9_405
.LBB9_411:
	ldr	w8, [sp, #156]          // 4-byte Folded Reload
	cbz	w8, .LBB9_400
// %bb.412:
	ldr	x8, [sp, #224]          // 8-byte Folded Reload
	mov	x0, x19
	cmp	w8, #2                  // =2
	cset	w1, hi
	bl	f2fs_submit_merged_write
	b	.LBB9_400
.LBB9_413:
	bl	print_bdev_access_info
	adrp	x22, f2fs_dclient
	ldr	x0, [x22, :lo12:f2fs_dclient]
	cbz	x0, .LBB9_416
// %bb.414:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB9_416
// %bb.415:
	ldr	x0, [x22, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.sanity_check_seg_type
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.sanity_check_seg_type
	mov	w3, #46
	bl	dsm_client_record
	ldr	x0, [x22, :lo12:f2fs_dclient]
	mov	w8, #15244
	movk	w8, #14160, lsl #16
	sub	w1, w8, #3              // =3
	bl	dsm_client_notify
.LBB9_416:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.h"; .popsection; .long 14472b - 14470b; .short 46; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB9_3
// %bb.417:
	add	x8, x19, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x19, #3376]
	cbnz	w8, .LBB9_3
// %bb.418:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB9_3
// %bb.419:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB9_3
// %bb.420:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB9_3
.LBB9_421:
	bl	__stack_chk_fail
.Lfunc_end9:
	.size	do_garbage_collect, .Lfunc_end9-do_garbage_collect
                                        // -- End function
	.section	.init.text,"ax",@progbits
	.globl	f2fs_create_garbage_collection_cache // -- Begin function f2fs_create_garbage_collection_cache
	.p2align	2
	.type	f2fs_create_garbage_collection_cache,@function
f2fs_create_garbage_collection_cache:   // @f2fs_create_garbage_collection_cache
.Lf2fs_create_garbage_collection_cache$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x0, .L.str.6
	add	x0, x0, :lo12:.L.str.6
	mov	w1, #56
	mov	w3, #131072
	mov	w2, wzr
	mov	x4, xzr
	mov	x29, sp
	bl	kmem_cache_create
	cmp	x0, #0                  // =0
	mov	w8, #-12
	adrp	x9, victim_entry_slab
	csel	w8, w8, wzr, eq
	str	x0, [x9, :lo12:victim_entry_slab]
	mov	w0, w8
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end10:
	.size	f2fs_create_garbage_collection_cache, .Lfunc_end10-f2fs_create_garbage_collection_cache
                                        // -- End function
	.text
	.globl	f2fs_destroy_garbage_collection_cache // -- Begin function f2fs_destroy_garbage_collection_cache
	.p2align	2
	.type	f2fs_destroy_garbage_collection_cache,@function
f2fs_destroy_garbage_collection_cache:  // @f2fs_destroy_garbage_collection_cache
.Lf2fs_destroy_garbage_collection_cache$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x8, victim_entry_slab
	ldr	x0, [x8, :lo12:victim_entry_slab]
	mov	x29, sp
	bl	kmem_cache_destroy
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end11:
	.size	f2fs_destroy_garbage_collection_cache, .Lfunc_end11-f2fs_destroy_garbage_collection_cache
                                        // -- End function
	.globl	f2fs_build_gc_manager   // -- Begin function f2fs_build_gc_manager
	.p2align	2
	.type	f2fs_build_gc_manager,@function
f2fs_build_gc_manager:                  // @f2fs_build_gc_manager
.Lf2fs_build_gc_manager$local:
// %bb.0:
	ldr	x8, [x0, #136]
	adrp	x9, default_v_ops
	add	x9, x9, :lo12:default_v_ops
	ldr	x8, [x8, #16]
	str	x9, [x8]
	ldr	w9, [x0, #3128]
	mov	w8, #2048
	str	x8, [x0, #2032]
	cmp	w9, #2                  // =2
	b.lt	.LBB12_15
// %bb.1:
	ldr	w8, [x0, #1192]
	cmp	w8, #1                  // =1
	b.ne	.LBB12_15
// %bb.2:
	ldr	x11, [x0, #3136]
	ldr	w8, [x11, #80]
	add	w10, w8, #3             // =3
	cmp	w10, #4                 // =4
	b.lo	.LBB12_11
// %bb.3:
	ldr	x10, [x0]
	add	x11, x11, #80           // =80
	ldr	x10, [x10, #248]
	b	.LBB12_5
.LBB12_4:                               //   in Loop: Header=BB12_5 Depth=1
	subs	x9, x9, #1              // =1
	add	x11, x11, #88           // =88
	b.eq	.LBB12_8
.LBB12_5:                               // =>This Inner Loop Header: Depth=1
	ldur	w12, [x11, #-4]
	cmp	w12, w8
	b.hi	.LBB12_4
// %bb.6:                               //   in Loop: Header=BB12_5 Depth=1
	ldr	w12, [x11]
	cmp	w12, w8
	b.lo	.LBB12_4
// %bb.7:
	ldur	x10, [x11, #-80]
.LBB12_8:
	cbz	x10, .LBB12_11
// %bb.9:
	ldr	x9, [x0, #136]
	cbz	x9, .LBB12_12
// %bb.10:
	add	x10, x9, #80            // =80
	b	.LBB12_13
.LBB12_11:
	mov	w8, wzr
	b	.LBB12_14
.LBB12_12:
	ldr	x10, [x0, #16]
	add	x10, x10, #72           // =72
.LBB12_13:
	ldr	x9, [x9, #8]
	ldr	w10, [x10]
	ldr	w11, [x0, #1184]
	ldr	w9, [x9]
	sub	w8, w8, w10
	lsr	w8, w8, w11
	sub	w8, w8, w9
	add	w8, w8, #1              // =1
.LBB12_14:
	ldr	x9, [x0, #136]
	ldr	x9, [x9]
	str	w8, [x9, #196]
.LBB12_15:
	ldrb	w9, [x0, #1427]
	mov	w8, #14975
	movk	w8, #9, lsl #16
	tbz	w9, #3, .LBB12_18
// %bb.16:
	ldr	x9, [x0, #136]
	ldr	x9, [x9]
	ldr	x9, [x9, #136]
	cmp	x9, x8
	b.ls	.LBB12_18
// %bb.17:
	mov	w9, #1
	strb	w9, [x0, #1920]
.LBB12_18:
	mov	x11, #10
	add	x9, x0, #1944           // =1944
	mov	x10, #85899345920
	movk	x11, #60, lsl #32
	add	x8, x8, #1              // =1
	str	xzr, [x0, #1928]
	str	xzr, [x0, #1936]
	str	x9, [x0, #1944]
	str	x9, [x0, #1952]
	str	x10, [x0, #1960]
	str	x11, [x0, #1968]
	str	x8, [x0, #1976]
	ret
.Lfunc_end12:
	.size	f2fs_build_gc_manager, .Lfunc_end12-f2fs_build_gc_manager
                                        // -- End function
	.globl	f2fs_resize_fs          // -- Begin function f2fs_resize_fs
	.p2align	2
	.type	f2fs_resize_fs,@function
f2fs_resize_fs:                         // @f2fs_resize_fs
.Lf2fs_resize_fs$local:
// %bb.0:
	sub	sp, sp, #144            // =144
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	adrp	x9, .L__const.free_segment_range.cpc
	add	x9, x9, :lo12:.L__const.free_segment_range.cpc
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	add	x29, sp, #48            // =48
	ldp	x10, x11, [x9]
	ldp	x12, x9, [x9, #16]
	stp	x28, x27, [sp, #64]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #80]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #96]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #112]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #128]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	w13, [x0, #1192]
	ldr	w8, [x0, #1188]
	stp	x10, x11, [sp, #8]
	stp	x12, x9, [sp, #24]
	ldr	x9, [x0, #16]
	mul	w10, w8, w13
	lsl	w11, w10, #12
	add	x12, x1, x11
	sub	x12, x12, #1            // =1
	ldur	x9, [x9, #36]
	udiv	x22, x12, x11
	and	x11, x22, #0xffffffff
	mul	x26, x11, x10
	cmp	x26, x9
	b.hi	.LBB13_9
// %bb.1:
	ldr	w9, [x0, #3128]
	mov	x19, x0
	cmp	w9, #2                  // =2
	b.lt	.LBB13_3
// %bb.2:
	ldr	x10, [x19, #3136]
	sub	w9, w9, #1              // =1
	mov	w11, #88
	smaddl	x9, w9, w11, x10
	ldr	w9, [x9, #72]
	mul	x8, x9, x8
	cmp	x8, x26
	b.ls	.LBB13_9
.LBB13_3:
	cbz	x26, .LBB13_12
// %bb.4:
	ldr	x8, [x19, #80]
	tbnz	w8, #2, .LBB13_13
// %bb.5:
	ldrb	w8, [x19, #1427]
	tbnz	w8, #1, .LBB13_14
// %bb.6:
	add	x20, x19, #1664         // =1664
	mov	x0, x20
	bl	down_write_trylock
	cbz	w0, .LBB13_15
// %bb.7:
	add	x1, sp, #8              // =8
	mov	x0, x19
	bl	f2fs_write_checkpoint
	cbz	w0, .LBB13_16
// %bb.8:
	mov	x0, x20
	bl	up_write
	mov	w24, #-11
	b	.LBB13_10
.LBB13_9:
	mov	w24, #-22
.LBB13_10:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB13_53
// %bb.11:
	mov	w0, w24
	ldp	x20, x19, [sp, #128]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #112]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #96]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #80]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #64]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #144            // =144
	ret
.LBB13_12:
	mov	w24, wzr
	b	.LBB13_10
.LBB13_13:
	adrp	x1, .L.str.7
	add	x1, x1, :lo12:.L.str.7
	mov	x0, x19
	bl	f2fs_printk
	mov	w24, #-117
	b	.LBB13_10
.LBB13_14:
	adrp	x1, .L.str.8
	add	x1, x1, :lo12:.L.str.8
	mov	x0, x19
	bl	f2fs_printk
	mov	w24, #-22
	b	.LBB13_10
.LBB13_15:
	mov	w24, #-11
	b	.LBB13_10
.LBB13_16:
	add	x21, x19, #368          // =368
	mov	w8, #1
	mov	x0, x21
	strb	w8, [x19, #3456]
	bl	_raw_spin_lock
	ldr	x8, [x19, #352]
	cbz	x8, .LBB13_18
// %bb.17:
	adrp	x23, jiffies
	ldr	x8, [x19, #360]
	ldr	x9, [x23, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB13_33
.LBB13_18:
	mov	x0, x21
	bl	_raw_spin_unlock
	add	x23, x19, #304          // =304
	mov	x0, x23
	bl	down_read
	add	x21, x19, #2340         // =2340
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	w8, [x19, #1240]
	ldr	w9, [x19, #1256]
	ldr	w10, [x19, #1264]
	ldr	w11, [x19, #1436]
	add	x8, x26, x8
	ldr	w24, [x19, #1236]
	add	x8, x8, x9
	add	x8, x8, x10
	mov	x0, x21
	add	x25, x8, x11
	bl	_raw_spin_unlock
	cmp	x25, x24
	b.ls	.LBB13_20
// %bb.19:
	mov	w24, #-28
	b	.LBB13_21
.LBB13_20:
	mov	w2, #1
	mov	x0, x19
	mov	w1, w22
	bl	free_segment_range
	mov	w24, w0
.LBB13_21:
	mov	x0, x23
	bl	up_read
	mov	x0, x20
	bl	up_write
	cbnz	w24, .LBB13_52
// %bb.22:
	cbz	x19, .LBB13_24
// %bb.23:
	add	x8, x19, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 8192
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB13_24:
	ldr	x0, [x19]
	bl	freeze_super
	mov	x0, x20
	bl	down_write
	add	x23, x19, #256          // =256
	mov	x0, x23
	bl	down_write
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	w8, [x19, #1240]
	ldr	w9, [x19, #1256]
	ldr	w10, [x19, #1264]
	ldr	w11, [x19, #1436]
	add	x12, x26, x8
	ldr	w8, [x19, #1236]
	add	x9, x12, x9
	add	x9, x9, x10
	add	x9, x9, x11
	cmp	x9, x8
	b.ls	.LBB13_26
// %bb.25:
	mov	w24, #-28
	b	.LBB13_50
.LBB13_26:
	sub	w8, w8, w26
	mov	x0, x21
	str	w8, [x19, #1236]
	bl	_raw_spin_unlock
	mov	x0, x19
	mov	w1, w22
	mov	w2, wzr
	bl	free_segment_range
	mov	w24, w0
	cbnz	w0, .LBB13_44
// %bb.27:
	ldr	w8, [x19, #1192]
	ldr	x27, [x19, #16]
	add	x25, x19, #24           // =24
	mov	x0, x25
	mneg	w24, w22, w8
	bl	down_write
	ldp	w8, w9, [x27, #44]
	ldr	w10, [x27, #68]
	ldur	x11, [x27, #36]
	sub	w8, w8, w22
	add	w9, w9, w24
	add	w10, w10, w24
	stp	w8, w9, [x27, #44]
	str	w10, [x27, #68]
	ldr	w8, [x19, #1188]
	sxtw	x9, w24
	madd	x8, x8, x9, x11
	stur	x8, [x27, #36]
	ldr	w8, [x19, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB13_29
// %bb.28:
	sub	w8, w8, #1              // =1
	mov	w9, #68
	smaddl	x8, w8, w9, x27
	add	x8, x8, #2265           // =2265
	ldr	w9, [x8]
	add	w9, w9, w24
	str	w9, [x8]
.LBB13_29:
	mov	x0, x25
	bl	up_write
	mov	x0, x19
	mov	w1, wzr
	bl	f2fs_commit_super
	cbz	w0, .LBB13_34
// %bb.30:
	ldr	w8, [x19, #1192]
	ldr	x28, [x19, #16]
	mov	w24, w0
	mov	x0, x25
	mul	w27, w8, w22
	bl	down_write
	ldp	w8, w9, [x28, #44]
	ldr	w10, [x28, #68]
	ldur	x11, [x28, #36]
	add	w8, w8, w22
	add	w9, w9, w27
	add	w10, w10, w27
	stp	w8, w9, [x28, #44]
	str	w10, [x28, #68]
	ldr	w8, [x19, #1188]
	sxtw	x9, w27
	madd	x8, x8, x9, x11
	stur	x8, [x28, #36]
	ldr	w8, [x19, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB13_32
// %bb.31:
	sub	w8, w8, #1              // =1
	mov	w9, #68
	smaddl	x8, w8, w9, x28
	add	x8, x8, #2265           // =2265
	ldr	w9, [x8]
	add	w9, w9, w27
	str	w9, [x8]
.LBB13_32:
	mov	x0, x25
	bl	up_write
	b	.LBB13_44
.LBB13_33:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x19, #360]
	ldr	x2, [x23, :lo12:jiffies]
	adrp	x0, .L.str.20
	add	x0, x0, :lo12:.L.str.20
	bl	printk
	ldr	x0, [x19, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB13_18
.LBB13_34:
	ldr	x9, [x19, #136]
	ldr	w8, [x19, #1192]
	ldr	x11, [x19, #224]
	neg	w10, w22
	ldr	w12, [x9, #92]
	mul	w8, w8, w10
	ldr	w10, [x19, #1188]
	ldr	x11, [x11, #8]
	add	w12, w12, w8
	str	w12, [x9, #92]
	ldr	x9, [x19, #136]
	ldr	w12, [x9, #96]
	add	w12, w12, w8
	str	w12, [x9, #96]
	ldr	w9, [x19, #1200]
	ldr	x12, [x19, #136]
	sub	w9, w9, w22
	str	w9, [x19, #1200]
	ldr	x9, [x12, #8]
	ldr	w12, [x9, #8]
	sub	w12, w12, w22
	str	w12, [x9, #8]
	ldr	x9, [x19, #136]
	ldr	x9, [x9, #8]
	ldr	w12, [x9, #4]
	add	w12, w12, w8
	str	w12, [x9, #4]
	ldr	x12, [x19, #224]
	sxtw	x9, w8
	mul	x9, x9, x10
	add	x10, x11, x9
	str	x10, [x12, #8]
	ldr	w10, [x19, #3128]
	cmp	w10, #2                 // =2
	b.lt	.LBB13_36
// %bb.35:
	ldr	x11, [x19, #3136]
	sub	w10, w10, #1            // =1
	mov	w12, #88
	smaddl	x11, w10, w12, x11
	ldr	w13, [x11, #72]
	add	w8, w13, w8
	str	w8, [x11, #72]
	ldr	x8, [x19, #3136]
	smaddl	x8, w10, w12, x8
	ldr	w10, [x8, #80]
	add	w9, w10, w9
	str	w9, [x8, #80]
.LBB13_36:
	add	x8, x19, #80            // =80
	mov	w9, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	cbz	x19, .LBB13_38
// %bb.37:
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 1
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB13_38:
	add	x1, sp, #8              // =8
	mov	x0, x19
	bl	f2fs_write_checkpoint
	mov	w24, w0
	cbz	w0, .LBB13_51
// %bb.39:
	ldr	x9, [x19, #136]
	ldr	w8, [x19, #1192]
	ldr	x10, [x19, #224]
	ldr	w12, [x19, #1188]
	ldr	w11, [x9, #92]
	mul	w8, w8, w22
	ldr	x10, [x10, #8]
	add	w11, w11, w8
	str	w11, [x9, #92]
	ldr	x9, [x19, #136]
	ldr	w11, [x9, #96]
	add	w11, w11, w8
	str	w11, [x9, #96]
	ldr	w9, [x19, #1200]
	ldr	x11, [x19, #136]
	add	w9, w9, w22
	str	w9, [x19, #1200]
	ldr	x9, [x11, #8]
	ldr	w11, [x9, #8]
	add	w11, w11, w22
	str	w11, [x9, #8]
	ldr	x9, [x19, #136]
	ldr	x9, [x9, #8]
	ldr	w11, [x9, #4]
	add	w11, w11, w8
	str	w11, [x9, #4]
	ldr	x11, [x19, #224]
	sxtw	x9, w8
	mul	x9, x9, x12
	add	x10, x10, x9
	str	x10, [x11, #8]
	ldr	w10, [x19, #3128]
	cmp	w10, #2                 // =2
	b.lt	.LBB13_41
// %bb.40:
	ldr	x11, [x19, #3136]
	sub	w10, w10, #1            // =1
	mov	w12, #88
	smaddl	x11, w10, w12, x11
	ldr	w13, [x11, #72]
	add	w8, w13, w8
	str	w8, [x11, #72]
	ldr	x8, [x19, #3136]
	smaddl	x8, w10, w12, x8
	ldr	w10, [x8, #80]
	add	w9, w10, w9
	str	w9, [x8, #80]
.LBB13_41:
	ldr	w8, [x19, #1192]
	ldr	x28, [x19, #16]
	mov	x0, x25
	mul	w27, w8, w22
	bl	down_write
	ldp	w8, w9, [x28, #44]
	ldr	w10, [x28, #68]
	ldur	x11, [x28, #36]
	add	w8, w8, w22
	add	w9, w9, w27
	add	w10, w10, w27
	stp	w8, w9, [x28, #44]
	str	w10, [x28, #68]
	ldr	w8, [x19, #1188]
	sxtw	x9, w27
	madd	x8, x8, x9, x11
	stur	x8, [x28, #36]
	ldr	w8, [x19, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB13_43
// %bb.42:
	sub	w8, w8, #1              // =1
	mov	w9, #68
	smaddl	x8, w8, w9, x28
	add	x8, x8, #2265           // =2265
	ldr	w9, [x8]
	add	w9, w9, w27
	str	w9, [x8]
.LBB13_43:
	mov	x0, x25
	bl	up_write
	mov	x0, x19
	mov	w1, wzr
	bl	f2fs_commit_super
.LBB13_44:
	cbz	x19, .LBB13_49
// %bb.45:
	add	x8, x19, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x19, #3376]
	cbnz	w8, .LBB13_49
// %bb.46:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB13_49
// %bb.47:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB13_49
// %bb.48:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB13_49:
	adrp	x1, .L.str.9
	add	x1, x1, :lo12:.L.str.9
	mov	x0, x19
	bl	f2fs_printk
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	w8, [x19, #1236]
	add	w8, w8, w26
	str	w8, [x19, #1236]
.LBB13_50:
	mov	x0, x21
	bl	_raw_spin_unlock
.LBB13_51:
	mov	x0, x23
	bl	up_write
	mov	x0, x20
	bl	up_write
	mov	x20, x19
	ldr	x0, [x20], #80
	bl	thaw_super
	mov	w8, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x20]
1:	ldxr	x9, [x20]
	bic	x9, x9, x8
	stxr	w10, x9, [x20]
	cbnz	w10, 1b
	//NO_APP
	cbz	w24, .LBB13_10
.LBB13_52:
	strb	wzr, [x19, #3456]
	b	.LBB13_10
.LBB13_53:
	bl	__stack_chk_fail
.Lfunc_end13:
	.size	f2fs_resize_fs, .Lfunc_end13-f2fs_resize_fs
                                        // -- End function
	.p2align	2               // -- Begin function free_segment_range
	.type	free_segment_range,@function
free_segment_range:                     // @free_segment_range
// %bb.0:
	sub	sp, sp, #192            // =192
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	adrp	x9, .L__const.free_segment_range.cpc
	stp	x29, x30, [sp, #96]     // 16-byte Folded Spill
	add	x29, sp, #96            // =96
	add	x9, x9, :lo12:.L__const.free_segment_range.cpc
	stp	x28, x27, [sp, #112]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #128]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #144]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #160]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #176]    // 16-byte Folded Spill
	str	w2, [sp, #20]           // 4-byte Folded Spill
	ldp	x10, x11, [x9]
	stur	x8, [x29, #-8]
	ldp	x8, x9, [x9, #16]
	mov	x19, x0
	stp	x10, x11, [x29, #-40]
	mov	w20, w1
	stp	x8, x9, [x29, #-24]
	ldr	w8, [x0, #1200]
	ldr	x9, [x0, #136]
	ldr	w10, [x0, #1192]
	sub	w8, w8, w1
	str	w8, [x0, #1200]
	ldr	x11, [x9, #16]
	ldr	w21, [x9, #96]
	mul	w22, w10, w8
	add	x0, x11, #80            // =80
	bl	mutex_lock
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
	ldr	w9, [x8, #184]
	cmp	w9, w22
	b.lo	.LBB14_2
// %bb.1:
	str	wzr, [x8, #184]
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
.LBB14_2:
	ldr	w9, [x8, #188]
	cmp	w9, w22
	b.lo	.LBB14_4
// %bb.3:
	str	wzr, [x8, #188]
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
.LBB14_4:
	ldr	w9, [x8, #192]
	cmp	w9, w22
	b.lo	.LBB14_6
// %bb.5:
	str	wzr, [x8, #192]
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
.LBB14_6:
	ldr	w9, [x8, #196]
	cmp	w9, w22
	b.lo	.LBB14_8
// %bb.7:
	str	wzr, [x8, #196]
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
.LBB14_8:
	ldr	w9, [x8, #200]
	cmp	w9, w22
	b.lo	.LBB14_10
// %bb.9:
	str	wzr, [x8, #200]
.LBB14_10:
	ldr	w8, [x19, #1992]
	sub	w24, w21, #1            // =1
	str	w20, [sp, #16]          // 4-byte Folded Spill
	cmp	w8, w22
	b.lo	.LBB14_12
// %bb.11:
	mov	w8, #-1
	str	w8, [x19, #1992]
.LBB14_12:
	ldr	w8, [x19, #1996]
	str	x21, [sp]               // 8-byte Folded Spill
	cmp	w8, w22
	b.lo	.LBB14_14
// %bb.13:
	mov	w8, #-1
	str	w8, [x19, #1996]
.LBB14_14:
	ldr	x8, [x19, #136]
	ldr	x8, [x8, #16]
	add	x0, x8, #80             // =80
	bl	mutex_unlock
	mov	x0, x19
	mov	w1, wzr
	mov	w2, w22
	mov	w3, w24
	bl	f2fs_allocate_segment_for_resize
	mov	w1, #1
	mov	x0, x19
	mov	w2, w22
	mov	w3, w24
	bl	f2fs_allocate_segment_for_resize
	mov	w1, #2
	mov	x0, x19
	mov	w2, w22
	mov	w3, w24
	bl	f2fs_allocate_segment_for_resize
	mov	w1, #3
	mov	x0, x19
	mov	w2, w22
	mov	w3, w24
	bl	f2fs_allocate_segment_for_resize
	mov	w1, #4
	mov	x0, x19
	mov	w2, w22
	mov	w3, w24
	bl	f2fs_allocate_segment_for_resize
	mov	w1, #5
	mov	x0, x19
	mov	w2, w22
	mov	w3, w24
	bl	f2fs_allocate_segment_for_resize
	cmp	w22, w24
	str	w22, [sp, #12]          // 4-byte Folded Spill
	b.ls	.LBB14_17
.LBB14_15:
	ldr	w8, [sp, #20]           // 4-byte Folded Reload
	tbz	w8, #0, .LBB14_32
// %bb.16:
	ldr	w23, [sp, #16]          // 4-byte Folded Reload
	mov	w0, wzr
	b	.LBB14_44
.LBB14_17:
	ldr	w26, [sp, #12]          // 4-byte Folded Reload
	mov	x28, #256
	add	x20, sp, #24            // =24
	movk	x28, #57005, lsl #48
	add	x25, x20, #16           // =16
	adrp	x22, f2fs_inode_entry_slab
	//APP
	mrs x21, sp_el0
	//NO_APP
	b	.LBB14_19
.LBB14_18:                              //   in Loop: Header=BB14_19 Depth=1
	ldr	w8, [x19, #1192]
	add	w26, w8, w26
	cmp	w26, w24
	b.hi	.LBB14_15
.LBB14_19:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB14_25 Depth 2
	mov	x8, #13469017440256
	add	x2, sp, #24             // =24
	mov	w3, #1
	mov	w4, #1
	mov	x0, x19
	mov	w1, w26
	stp	x20, x20, [sp, #24]
	stp	x8, xzr, [sp, #40]
	bl	do_garbage_collect
	ldr	x27, [sp, #24]
	cmp	x27, x20
	b.ne	.LBB14_25
.LBB14_20:                              //   in Loop: Header=BB14_19 Depth=1
	ldr	w8, [sp, #20]           // 4-byte Folded Reload
	tbnz	w8, #0, .LBB14_29
// %bb.21:                              //   in Loop: Header=BB14_19 Depth=1
	ldr	x8, [x19, #136]
	ldr	w9, [x19, #1192]
	ldr	x8, [x8]
	cmp	w9, #2                  // =2
	b.lo	.LBB14_27
// %bb.22:                              //   in Loop: Header=BB14_19 Depth=1
	ldr	x8, [x8, #128]
	cmn	w26, #1                 // =1
	b.eq	.LBB14_28
// %bb.23:                              //   in Loop: Header=BB14_19 Depth=1
	udiv	w9, w26, w9
	ldr	w8, [x8, x9, lsl #2]
	cbz	w8, .LBB14_29
	b	.LBB14_43
.LBB14_24:                              //   in Loop: Header=BB14_25 Depth=2
	add	x8, x28, #34            // =34
	stp	x28, x8, [x27]
	ldr	x0, [x22, :lo12:f2fs_inode_entry_slab]
	mov	x1, x27
	bl	kmem_cache_free
	cmp	x23, x20
	mov	x27, x23
	b.eq	.LBB14_20
.LBB14_25:                              //   Parent Loop BB14_19 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	x8, [x27, #16]
	ldr	x23, [x27]
	mov	x0, x25
	ldr	x1, [x8, #64]
	bl	radix_tree_delete
	ldr	x0, [x27, #16]
	bl	iput
	mov	x0, x27
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB14_24
// %bb.26:                              //   in Loop: Header=BB14_25 Depth=2
	ldp	x9, x8, [x27]
	str	x8, [x9, #8]
	str	x9, [x8]
	b	.LBB14_24
.LBB14_27:                              //   in Loop: Header=BB14_19 Depth=1
	ldr	x8, [x8, #120]
	mov	w9, #40
	umull	x9, w26, w9
	ldr	w8, [x8, x9]
	ubfx	w8, w8, #6, #10
	cbz	w8, .LBB14_29
	b	.LBB14_43
.LBB14_28:                              //   in Loop: Header=BB14_19 Depth=1
	mov	w9, #-1
	ldr	w8, [x8, x9, lsl #2]
	cbnz	w8, .LBB14_43
.LBB14_29:                              //   in Loop: Header=BB14_19 Depth=1
	ldr	x8, [x21]
	tbz	w8, #0, .LBB14_18
// %bb.30:                              //   in Loop: Header=BB14_19 Depth=1
	ldrb	w8, [x21, #2129]
	tbz	w8, #0, .LBB14_18
// %bb.31:
	ldr	w23, [sp, #16]          // 4-byte Folded Reload
	mov	w0, #-512
	b	.LBB14_44
.LBB14_32:
	sub	x1, x29, #40            // =40
	mov	x0, x19
	bl	f2fs_write_checkpoint
	ldr	w23, [sp, #16]          // 4-byte Folded Reload
	cbnz	w0, .LBB14_44
// %bb.33:
	ldr	x8, [x19, #136]
	ldr	x20, [x8, #8]
	add	x22, x20, #12           // =12
	mov	x0, x22
	bl	_raw_spin_lock
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	ldr	x0, [x20, #16]
	ldr	x1, [sp]                // 8-byte Folded Reload
	mov	w2, w8
	bl	find_next_bit
	mov	x21, x0
	mov	x0, x22
	bl	_raw_spin_unlock
	cmp	w24, w21
	b.lo	.LBB14_42
// %bb.34:
	adrp	x1, .L.str.21
	add	x1, x1, :lo12:.L.str.21
	mov	x0, x19
	mov	w2, w21
	bl	f2fs_printk
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB14_37
// %bb.35:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB14_37
// %bb.36:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.free_segment_range
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.free_segment_range
	mov	w3, #3629
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB14_37:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/gc.c"; .popsection; .long 14472b - 14470b; .short 3629; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB14_42
// %bb.38:
	add	x8, x19, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x19, #3376]
	cbnz	w8, .LBB14_42
// %bb.39:
	ldr	x1, [x19, #3392]
	mov	w0, wzr
	cbz	x1, .LBB14_44
// %bb.40:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB14_44
// %bb.41:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB14_42:
	mov	w0, wzr
	b	.LBB14_44
.LBB14_43:
	ldr	w23, [sp, #16]          // 4-byte Folded Reload
	mov	w0, #-11
.LBB14_44:
	ldr	w8, [x19, #1200]
	adrp	x9, __stack_chk_guard
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	add	w8, w8, w23
	str	w8, [x19, #1200]
	ldur	x8, [x29, #-8]
	cmp	x9, x8
	b.ne	.LBB14_46
// %bb.45:
	ldp	x20, x19, [sp, #176]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #160]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #144]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #128]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #112]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #96]     // 16-byte Folded Reload
	add	sp, sp, #192            // =192
	ret
.LBB14_46:
	bl	__stack_chk_fail
.Lfunc_end14:
	.size	free_segment_range, .Lfunc_end14-free_segment_range
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_put_page
	.type	f2fs_put_page,@function
f2fs_put_page:                          // @f2fs_put_page
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB15_32
// %bb.1:
	mov	x19, x0
	cbz	w1, .LBB15_30
// %bb.2:
	ldr	x8, [x19, #24]
	ldr	x8, [x8]
	ldr	x8, [x8, #40]
	ldr	x9, [x8, #96]
	ldr	x8, [x19, #8]
	sub	x10, x8, #1             // =1
	tst	x8, #0x1
	csel	x8, x19, x10, eq
	ldr	x8, [x8]
	mov	w10, #8208
	movk	w10, #62197, lsl #16
	cmp	x9, x10
	b.ne	.LBB15_24
// %bb.3:
	tbnz	w8, #0, .LBB15_29
// %bb.4:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB15_7
// %bb.5:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB15_7
// %bb.6:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.f2fs_put_page
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.f2fs_put_page
	mov	w3, #3801
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB15_7:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB15_9
// %bb.8:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB15_33
.LBB15_9:
	ldr	x0, [x19, #24]
.LBB15_10:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_raw_sb_info
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB15_12
// %bb.11:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB15_34
.LBB15_12:
	ldr	x0, [x19, #24]
.LBB15_13:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_ckpt_info
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB15_15
// %bb.14:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB15_35
.LBB15_15:
	ldr	x0, [x19, #24]
.LBB15_16:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/f2fs.h"; .popsection; .long 14472b - 14470b; .short 3801; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB15_18
// %bb.17:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB15_36
.LBB15_18:
	ldr	x0, [x19, #24]
.LBB15_19:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB15_29
// %bb.20:
	add	x9, x8, #80             // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x9]
1:	ldxr	x10, [x9]
	orr	x10, x10, 4
	stxr	w11, x10, [x9]
	cbnz	w11, 1b
	//NO_APP
	ldrb	w9, [x8, #3376]
	cbnz	w9, .LBB15_29
// %bb.21:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB15_29
// %bb.22:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB15_29
// %bb.23:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB15_29
.LBB15_24:
	tbnz	w8, #0, .LBB15_29
// %bb.25:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB15_28
// %bb.26:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB15_28
// %bb.27:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.f2fs_put_page
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.f2fs_put_page
	mov	w3, #3803
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB15_28:
	mov	x0, xzr
	bl	f2fs_print_raw_sb_info
	mov	x0, xzr
	bl	f2fs_print_ckpt_info
	mov	x0, xzr
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/f2fs.h"; .popsection; .long 14472b - 14470b; .short 3803; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
.LBB15_29:
	mov	x0, x19
	bl	unlock_page
.LBB15_30:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x19, x9, eq
	add	x8, x0, #52             // =52
	mov	w9, #1
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stlxr	w11, w10, [x8]
	cbnz	w11, 1b
	dmb ish
	//NO_APP
	cbnz	w10, .LBB15_32
// %bb.31:
	bl	__put_page
.LBB15_32:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB15_33:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB15_10
.LBB15_34:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB15_13
.LBB15_35:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB15_16
.LBB15_36:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB15_19
.Lfunc_end15:
	.size	f2fs_put_page, .Lfunc_end15-f2fs_put_page
                                        // -- End function
	.p2align	2               // -- Begin function get_victim_by_default
	.type	get_victim_by_default,@function
get_victim_by_default:                  // @get_victim_by_default
// %bb.0:
	sub	sp, sp, #336            // =336
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #240]    // 16-byte Folded Spill
	add	x29, sp, #240           // =240
	stp	x28, x27, [sp, #256]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #272]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #288]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #304]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #320]    // 16-byte Folded Spill
	str	x1, [sp, #80]           // 8-byte Folded Spill
	stur	x8, [x29, #-16]
	ldr	x8, [x0, #136]
	mov	x22, x0
	mov	x19, x5
	mov	w24, w4
	ldr	x9, [x8, #16]
	ldr	x23, [x8]
	mov	w26, w3
	mov	w27, w2
	add	x0, x9, #80             // =80
	stp	xzr, xzr, [x29, #-48]
	stp	xzr, xzr, [x29, #-64]
	stp	xzr, xzr, [x29, #-80]
	stp	xzr, xzr, [x29, #-96]
	stp	x9, x0, [sp, #16]       // 16-byte Folded Spill
	bl	mutex_lock
	add	x9, x22, #3408          // =3408
	str	x9, [sp, #8]            // 8-byte Folded Spill
	add	x9, x22, #1928          // =1928
	ldr	w10, [x22, #1200]
	ldr	w11, [x22, #1192]
	stur	x19, [x29, #-48]
	sxtw	x12, w26
	stur	x9, [x29, #-112]        // 8-byte Folded Spill
	mov	w9, #57
	str	x12, [sp, #32]          // 8-byte Folded Spill
	lsr	w12, w9, w12
	ldr	x9, [x22, #1976]
	mov	w28, wzr
	and	w8, w24, #0xff
	add	x25, x22, #1944         // =1944
	and	w12, w12, #0x1
	mul	w10, w11, w10
	str	w12, [sp, #44]          // 4-byte Folded Spill
	stp	w10, w8, [x29, #-100]   // 4-byte Folded Spill
	str	x26, [sp, #72]          // 8-byte Folded Spill
	str	w27, [sp, #68]          // 4-byte Folded Spill
	stp	x25, x23, [sp, #48]     // 16-byte Folded Spill
.LBB16_1:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB16_62 Depth 2
                                        //     Child Loop BB16_39 Depth 2
                                        //     Child Loop BB16_68 Depth 2
                                        //       Child Loop BB16_153 Depth 3
                                        //       Child Loop BB16_117 Depth 3
                                        //       Child Loop BB16_89 Depth 3
                                        //     Child Loop BB16_203 Depth 2
                                        //       Child Loop BB16_206 Depth 3
                                        //     Child Loop BB16_166 Depth 2
                                        //     Child Loop BB16_236 Depth 2
	stur	x9, [x29, #-40]
	ldr	x9, [x22, #136]
	cmp	w8, #2                  // =2
	ldr	x11, [x9, #16]
	b.eq	.LBB16_3
// %bb.2:                               //   in Loop: Header=BB16_1 Depth=1
	cmp	w8, #1                  // =1
	b.ne	.LBB16_18
.LBB16_3:                               //   in Loop: Header=BB16_1 Depth=1
	ldr	x13, [sp, #32]          // 8-byte Folded Reload
	mov	w12, #1
	stur	w12, [x29, #-92]
	add	x10, x11, x13, lsl #3
	ldr	x10, [x10, #8]
	add	x11, x11, x13, lsl #2
	stur	x10, [x29, #-88]
	ldr	w11, [x11, #112]
	stur	w12, [x29, #-72]
	mov	w10, #1
.LBB16_4:                               //   in Loop: Header=BB16_1 Depth=1
	stur	w11, [x29, #-80]
	cmp	w27, #1                 // =1
	b.eq	.LBB16_10
// %bb.5:                               //   in Loop: Header=BB16_1 Depth=1
	ldr	w13, [x22, #1988]
	cmp	w13, #5                 // =5
	b.eq	.LBB16_10
// %bb.6:                               //   in Loop: Header=BB16_1 Depth=1
	cmp	w8, #2                  // =2
	b.eq	.LBB16_10
// %bb.7:                               //   in Loop: Header=BB16_1 Depth=1
	cmp	w10, #2                 // =2
	b.eq	.LBB16_10
// %bb.8:                               //   in Loop: Header=BB16_1 Depth=1
	ldr	w13, [x22, #2112]
	cmp	w11, w13
	b.ls	.LBB16_10
// %bb.9:                               //   in Loop: Header=BB16_1 Depth=1
	mov	w11, w13
	stur	w13, [x29, #-80]
.LBB16_10:                              //   in Loop: Header=BB16_1 Depth=1
	ldrb	w13, [x22, #1424]
	tbz	w13, #3, .LBB16_14
// %bb.11:                              //   in Loop: Header=BB16_1 Depth=1
	cmp	w26, #5                 // =5
	b.hi	.LBB16_14
// %bb.12:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	w13, [sp, #44]          // 4-byte Folded Reload
	cbz	w13, .LBB16_14
// %bb.13:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w13, wzr
	b	.LBB16_15
.LBB16_14:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	x13, [x9]
	add	x13, x13, w10, sxtw #2
	ldr	w13, [x13, #184]
.LBB16_15:                              //   in Loop: Header=BB16_1 Depth=1
	stur	w13, [x29, #-76]
	mov	w13, #-1
	cmp	w8, #1                  // =1
	stur	w13, [x29, #-56]
	stur	xzr, [x29, #-64]
	b.eq	.LBB16_20
// %bb.16:                              //   in Loop: Header=BB16_1 Depth=1
	cmp	w8, #2                  // =2
	b.ne	.LBB16_23
// %bb.17:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w12, #-1
	stur	w12, [x29, #-68]
	b	.LBB16_22
.LBB16_18:                              //   in Loop: Header=BB16_1 Depth=1
	cbz	w27, .LBB16_55
// %bb.19:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w10, #1
	b	.LBB16_56
.LBB16_20:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	w13, [x22, #1188]
.LBB16_21:                              //   in Loop: Header=BB16_1 Depth=1
	cmp	w10, #2                 // =2
	stur	w13, [x29, #-68]
	b.ne	.LBB16_27
.LBB16_22:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	x9, [x9]
	mov	x12, #-1
	mov	w26, #1
	str	x12, [x9, #168]
	b	.LBB16_28
.LBB16_23:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w13, #-1
	cbz	w10, .LBB16_21
// %bb.24:                              //   in Loop: Header=BB16_1 Depth=1
	cmp	w10, #2                 // =2
	b.eq	.LBB16_21
// %bb.25:                              //   in Loop: Header=BB16_1 Depth=1
	cmp	w10, #1                 // =1
	b.ne	.LBB16_64
// %bb.26:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	w13, [x22, #1188]
	mul	w12, w13, w12
	lsl	w13, w12, #1
	b	.LBB16_21
.LBB16_27:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w26, wzr
.LBB16_28:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	x9, [sp, #80]           // 8-byte Folded Reload
	ldr	w9, [x9]
	cmn	w9, #1                  // =1
	b.ne	.LBB16_269
// %bb.29:                              //   in Loop: Header=BB16_1 Depth=1
	cbz	w11, .LBB16_282
// %bb.30:                              //   in Loop: Header=BB16_1 Depth=1
	cbnz	w8, .LBB16_35
// %bb.31:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	w9, [x22, #1192]
	cmp	w9, #2                  // =2
	b.lo	.LBB16_35
// %bb.32:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	w9, [x22, #1992]
	cmn	w9, #1                  // =1
	b.ne	.LBB16_283
// %bb.33:                              //   in Loop: Header=BB16_1 Depth=1
	cmp	w27, #1                 // =1
	b.ne	.LBB16_53
// %bb.34:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	w9, [x22, #1996]
	cmn	w9, #1                  // =1
	b.ne	.LBB16_287
.LBB16_35:                              //   in Loop: Header=BB16_1 Depth=1
	add	x9, x23, w10, sxtw #2
	ldr	w19, [x9, #184]
	cmp	w27, #1                 // =1
	b.ne	.LBB16_54
// %bb.36:                              //   in Loop: Header=BB16_1 Depth=1
	cbnz	w8, .LBB16_54
// %bb.37:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	x8, [x22, #136]
	ldr	w1, [x22, #1200]
	mov	x2, xzr
	ldr	x21, [x8, #16]
	ldr	x0, [x21, #144]
	b	.LBB16_39
.LBB16_38:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	x8, [x21, #144]
	mov	w1, w9
	add	w2, w0, #1              // =1
	mov	x0, x8
.LBB16_39:                              //   Parent Loop BB16_1 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	bl	find_next_bit
	ldr	w9, [x22, #1200]
	cmp	w9, w0
	b.ls	.LBB16_52
// %bb.40:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	x8, [x22, #136]
	ldr	x10, [x8, #24]
	ldr	w8, [x22, #1192]
	ldr	w11, [x10, #100]
	udiv	w11, w11, w8
	cmp	w11, w0
	b.eq	.LBB16_38
// %bb.41:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	w11, [x10, #220]
	udiv	w11, w11, w8
	cmp	w11, w0
	b.eq	.LBB16_38
// %bb.42:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	w11, [x10, #340]
	udiv	w11, w11, w8
	cmp	w11, w0
	b.eq	.LBB16_38
// %bb.43:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	w11, [x10, #460]
	udiv	w11, w11, w8
	cmp	w11, w0
	b.eq	.LBB16_38
// %bb.44:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	w11, [x10, #580]
	udiv	w11, w11, w8
	cmp	w11, w0
	b.eq	.LBB16_38
// %bb.45:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	w11, [x10, #700]
	udiv	w11, w11, w8
	cmp	w11, w0
	b.eq	.LBB16_38
// %bb.46:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	w11, [x10, #820]
	udiv	w11, w11, w8
	cmp	w11, w0
	b.eq	.LBB16_38
// %bb.47:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	w10, [x10, #940]
	udiv	w10, w10, w8
	cmp	w10, w0
	b.eq	.LBB16_38
// %bb.48:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	w10, [x22, #1984]
	cmp	w10, w0
	b.eq	.LBB16_38
// %bb.49:                              //   in Loop: Header=BB16_39 Depth=2
	ldr	x10, [x22, #2104]
	cbz	x10, .LBB16_51
// %bb.50:                              //   in Loop: Header=BB16_39 Depth=2
	mul	w8, w8, w0
	add	w11, w8, #63            // =63
	cmp	w8, #0                  // =0
	csel	w11, w11, w8, lt
	asr	w11, w11, #6
	ldr	x10, [x10, w11, sxtw #3]
	lsr	x8, x10, x8
	tbnz	w8, #0, .LBB16_38
.LBB16_51:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	x8, [x21, #144]
	ubfx	x9, x0, #6, #26
	add	x8, x8, x9, lsl #3
	mov	w9, #1
	lsl	x9, x9, x0
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	w8, [x22, #1192]
	mul	w9, w8, w0
	cmn	w9, #1                  // =1
	stur	w9, [x29, #-56]
	b.eq	.LBB16_54
	b	.LBB16_301
.LBB16_52:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w8, #-1
	stur	w8, [x29, #-56]
	b	.LBB16_54
.LBB16_53:                              //   in Loop: Header=BB16_1 Depth=1
	add	x8, x23, w10, sxtw #2
	ldr	w19, [x8, #184]
.LBB16_54:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w24, wzr
	b	.LBB16_68
.LBB16_55:                              //   in Loop: Header=BB16_1 Depth=1
	ldrb	w10, [x22, #1920]
	lsl	w10, w10, #1
.LBB16_56:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	w12, [x22, #1988]
	sub	w12, w12, #1            // =1
	cmp	w12, #4                 // =4
	b.hi	.LBB16_59
// %bb.57:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w13, #27
	lsr	w13, w13, w12
	tbz	w13, #0, .LBB16_59
// %bb.58:                              //   in Loop: Header=BB16_1 Depth=1
	adrp	x10, .Lswitch.table.get_victim_by_default
	add	x10, x10, :lo12:.Lswitch.table.get_victim_by_default
	ldr	w10, [x10, w12, sxtw #2]
.LBB16_59:                              //   in Loop: Header=BB16_1 Depth=1
	stur	w10, [x29, #-92]
	ldr	w12, [x22, #1192]
	cmp	w12, #2                 // =2
	stur	w12, [x29, #-72]
	b.lo	.LBB16_63
// %bb.60:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	x13, [x11, #72]
	stur	x13, [x29, #-88]
	ldr	w14, [x22, #1200]
	cbz	w14, .LBB16_65
// %bb.61:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w11, wzr
	mov	w15, wzr
.LBB16_62:                              //   Parent Loop BB16_1 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	add	w16, w15, #63           // =63
	cmp	w15, #0                 // =0
	csel	w16, w16, w15, lt
	asr	w16, w16, #6
	ldr	x16, [x13, w16, sxtw #3]
	lsr	x16, x16, x15
	add	w15, w15, #1            // =1
	and	w16, w16, #0x1
	cmp	w14, w15
	add	w11, w11, w16
                                        // kill: def $w15 killed $w15 def $x15
	b.ne	.LBB16_62
	b	.LBB16_4
.LBB16_63:                              //   in Loop: Header=BB16_1 Depth=1
	ldr	x13, [x11, #56]
	stur	x13, [x29, #-88]
	ldr	w11, [x11, #136]
	b	.LBB16_4
.LBB16_64:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w13, wzr
	b	.LBB16_21
.LBB16_65:                              //   in Loop: Header=BB16_1 Depth=1
	mov	w11, wzr
	b	.LBB16_4
.LBB16_66:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	w8, [x22, #1960]
	add	w8, w8, #1              // =1
	str	w8, [x22, #1960]
.LBB16_67:                              //   in Loop: Header=BB16_68 Depth=2
	ldur	w8, [x29, #-80]
	cmp	w24, w8
	b.hs	.LBB16_160
.LBB16_68:                              //   Parent Loop BB16_1 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB16_153 Depth 3
                                        //       Child Loop BB16_117 Depth 3
                                        //       Child Loop BB16_89 Depth 3
	ldp	w9, w8, [x29, #-76]
	ldur	w20, [x29, #-100]       // 4-byte Folded Reload
	ldur	x0, [x29, #-88]
	udiv	w2, w9, w8
	udiv	w1, w20, w8
	bl	find_next_bit
	ldur	w8, [x29, #-72]
	mul	w28, w8, w0
	cmp	w28, w20
	b.hs	.LBB16_90
// %bb.69:                              //   in Loop: Header=BB16_68 Depth=2
	add	w8, w28, w8
	stur	w8, [x29, #-76]
	ldr	x9, [x22, #136]
	ldr	w8, [x22, #1192]
	add	w24, w24, #1            // =1
	ldr	x10, [x9, #24]
	udiv	w21, w28, w8
	ldr	w11, [x10, #100]
	udiv	w11, w11, w8
	cmp	w11, w21
	b.eq	.LBB16_67
// %bb.70:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	w11, [x10, #220]
	udiv	w11, w11, w8
	cmp	w11, w21
	b.eq	.LBB16_67
// %bb.71:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	w11, [x10, #340]
	udiv	w11, w11, w8
	cmp	w11, w21
	b.eq	.LBB16_67
// %bb.72:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	w11, [x10, #460]
	udiv	w11, w11, w8
	cmp	w11, w21
	b.eq	.LBB16_67
// %bb.73:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	w11, [x10, #580]
	udiv	w11, w11, w8
	cmp	w11, w21
	b.eq	.LBB16_67
// %bb.74:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	w11, [x10, #700]
	udiv	w11, w11, w8
	cmp	w11, w21
	b.eq	.LBB16_67
// %bb.75:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	w11, [x10, #820]
	udiv	w11, w11, w8
	cmp	w11, w21
	b.eq	.LBB16_67
// %bb.76:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	w10, [x10, #940]
	udiv	w10, w10, w8
	cmp	w10, w21
	b.eq	.LBB16_67
// %bb.77:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	w10, [x22, #1984]
	cmp	w10, w21
	b.eq	.LBB16_67
// %bb.78:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	x10, [x22, #80]
	tbnz	w10, #8, .LBB16_149
.LBB16_79:                              //   in Loop: Header=BB16_68 Depth=2
	cbz	w27, .LBB16_92
.LBB16_80:                              //   in Loop: Header=BB16_68 Depth=2
	ldur	w8, [x29, #-96]
	cmp	w27, #1                 // =1
	b.ne	.LBB16_84
// %bb.81:                              //   in Loop: Header=BB16_68 Depth=2
	cbnz	w8, .LBB16_84
// %bb.82:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	x9, [x22, #2104]
	cbz	x9, .LBB16_84
// %bb.83:                              //   in Loop: Header=BB16_68 Depth=2
	add	w10, w28, #63           // =63
	cmp	w28, #0                 // =0
	csel	w10, w10, w28, lt
	asr	w10, w10, #6
	ldr	x9, [x9, w10, sxtw #3]
	lsr	x9, x9, x28
	tbnz	w9, #0, .LBB16_67
.LBB16_84:                              //   in Loop: Header=BB16_68 Depth=2
	cbz	w26, .LBB16_93
// %bb.85:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	x8, [x22, #136]
	ldr	w9, [x22, #1192]
	ldr	x8, [x8]
	ldr	x11, [x22, #80]
	udiv	w10, w28, w9
	tbz	w11, #8, .LBB16_87
// %bb.86:                              //   in Loop: Header=BB16_68 Depth=2
	ldur	w11, [x29, #-92]
	cmp	w11, #2                 // =2
	b.eq	.LBB16_156
.LBB16_87:                              //   in Loop: Header=BB16_68 Depth=2
	cbz	w9, .LBB16_99
// %bb.88:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	x13, [x8, #120]
	mov	x12, xzr
	mov	x11, xzr
	mul	w10, w10, w9
	mov	x14, x9
	mov	w16, #40
.LBB16_89:                              //   Parent Loop BB16_1 Depth=1
                                        //     Parent Loop BB16_68 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	add	w15, w10, w12
	umaddl	x15, w15, w16, x13
	ldr	x15, [x15, #32]
	subs	x14, x14, #1            // =1
	add	x12, x12, #1            // =1
	add	x11, x15, x11
	b.ne	.LBB16_89
	b	.LBB16_100
.LBB16_90:                              //   in Loop: Header=BB16_68 Depth=2
	ldursw	x8, [x29, #-92]
	add	x9, x23, x8, lsl #2
	ldr	w8, [x9, #184]!
	cbz	w8, .LBB16_244
// %bb.91:                              //   in Loop: Header=BB16_68 Depth=2
	str	wzr, [x9]
	stur	wzr, [x29, #-76]
	stur	w8, [x29, #-100]        // 4-byte Folded Spill
	b	.LBB16_68
.LBB16_92:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	add	w9, w21, #63            // =63
	cmp	w21, #0                 // =0
	csel	w9, w9, w21, lt
	ldr	x8, [x8, #144]
	asr	w9, w9, #6
	ldr	x8, [x8, w9, sxtw #3]
	lsr	x8, x8, x21
	tbnz	w8, #0, .LBB16_67
	b	.LBB16_80
.LBB16_93:                              //   in Loop: Header=BB16_68 Depth=2
	cmp	w8, #1                  // =1
	b.ne	.LBB16_95
// %bb.94:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	x8, [x22, #136]
	mov	w9, #40
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	umaddl	x8, w28, w9, x8
	ldrh	w8, [x8, #2]
	and	w8, w8, #0x3ff
	b	.LBB16_147
.LBB16_95:                              //   in Loop: Header=BB16_68 Depth=2
	ldur	w8, [x29, #-92]
	cbz	w8, .LBB16_115
// %bb.96:                              //   in Loop: Header=BB16_68 Depth=2
	cmp	w8, #1                  // =1
	b.ne	.LBB16_120
// %bb.97:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	x9, [x22, #136]
	ldr	w8, [x22, #1192]
	ldr	x9, [x9]
	cmp	w8, #2                  // =2
	b.lo	.LBB16_129
// %bb.98:                              //   in Loop: Header=BB16_68 Depth=2
	ldr	x9, [x9, #128]
	udiv	w8, w28, w8
	ldr	w8, [x9, w8, uxtw #2]
	b	.LBB16_147
.LBB16_99:                              //   in Loop: Header=BB16_68 Depth=2
	mov	x9, xzr
	mov	x11, xzr
.LBB16_100:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x10, [x8, #152]
	udiv	x20, x11, x9
	cmp	x20, x10
	b.hs	.LBB16_102
// %bb.101:                             //   in Loop: Header=BB16_68 Depth=2
	str	x20, [x8, #152]
.LBB16_102:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x9, [x8, #160]
	cmp	x20, x9
	b.ls	.LBB16_104
// %bb.103:                             //   in Loop: Header=BB16_68 Depth=2
	str	x20, [x8, #160]
.LBB16_104:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x9, [x8, #168]
	cmp	x20, x9
	b.hs	.LBB16_106
// %bb.105:                             //   in Loop: Header=BB16_68 Depth=2
	str	x20, [x8, #168]
.LBB16_106:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x9, [x8, #176]
	cmp	x20, x9
	b.ls	.LBB16_108
// %bb.107:                             //   in Loop: Header=BB16_68 Depth=2
	mov	x9, x20
	str	x20, [x8, #176]
.LBB16_108:                             //   in Loop: Header=BB16_68 Depth=2
	ldur	x8, [x29, #-40]
	sub	x9, x9, x20
	cmp	x9, x8
	b.lo	.LBB16_67
// %bb.109:                             //   in Loop: Header=BB16_68 Depth=2
	ldur	x1, [x29, #-112]        // 8-byte Folded Reload
	mov	w8, #1
	sub	x2, x29, #24            // =24
	sub	x4, x29, #28            // =28
	mov	x0, x22
	mov	x3, x20
	stur	xzr, [x29, #-24]
	sturb	w8, [x29, #-28]
	bl	f2fs_lookup_rb_tree_ext
	ldurb	w8, [x29, #-28]
	ldur	x21, [x29, #-24]
	mov	w1, #3136
	str	w8, [sp, #120]          // 4-byte Folded Spill
	adrp	x8, victim_entry_slab
	ldr	x8, [x8, :lo12:victim_entry_slab]
	stp	x8, x0, [sp, #104]      // 16-byte Folded Spill
	mov	x0, x8
	bl	kmem_cache_alloc
	cbnz	x0, .LBB16_111
// %bb.110:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x0, [sp, #104]          // 8-byte Folded Reload
	mov	w1, #35904
	bl	kmem_cache_alloc
.LBB16_111:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x8, [sp, #112]          // 8-byte Folded Reload
	str	w28, [x0, #32]
	stp	x21, xzr, [x0]
	stp	xzr, x20, [x0, #16]
	str	x0, [x8]
	ldr	w8, [sp, #120]          // 4-byte Folded Reload
	cbz	w8, .LBB16_113
// %bb.112:                             //   in Loop: Header=BB16_68 Depth=2
	str	x0, [x22, #1936]
.LBB16_113:                             //   in Loop: Header=BB16_68 Depth=2
	ldur	x1, [x29, #-112]        // 8-byte Folded Reload
	mov	x20, x0
	bl	rb_insert_color
	ldr	x8, [x22, #1952]
	add	x0, x20, #40            // =40
	mov	x2, x25
	str	x20, [sp, #120]         // 8-byte Folded Spill
	mov	x1, x8
	mov	x20, x8
	mov	x21, x0
	bl	__list_add_valid
	tbz	w0, #0, .LBB16_66
// %bb.114:                             //   in Loop: Header=BB16_68 Depth=2
	mov	x8, x21
	str	x21, [x22, #1952]
	ldr	x21, [sp, #120]         // 8-byte Folded Reload
	stp	x25, x20, [x21, #40]
	str	x8, [x20]
	b	.LBB16_66
.LBB16_115:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x8, [x22, #136]
	ldr	w12, [x22, #1192]
	mov	x13, xzr
	mov	x9, xzr
	ldr	x8, [x8]
	ldr	x10, [x8, #120]
	cbz	w12, .LBB16_131
// %bb.116:                             //   in Loop: Header=BB16_68 Depth=2
	udiv	w11, w28, w12
	mul	w14, w11, w12
	mov	x15, x12
	mov	w17, #40
.LBB16_117:                             //   Parent Loop BB16_1 Depth=1
                                        //     Parent Loop BB16_68 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	add	w16, w14, w13
	umaddl	x16, w16, w17, x10
	ldr	x16, [x16, #32]
	subs	x15, x15, #1            // =1
	add	x13, x13, #1            // =1
	add	x9, x16, x9
	b.ne	.LBB16_117
// %bb.118:                             //   in Loop: Header=BB16_68 Depth=2
	cmp	w12, #2                 // =2
	b.lo	.LBB16_130
// %bb.119:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x10, [x8, #128]
	mov	x13, x12
	ldr	w14, [x10, w11, uxtw #2]
	b	.LBB16_132
.LBB16_120:                             //   in Loop: Header=BB16_68 Depth=2
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_123
// %bb.121:                             //   in Loop: Header=BB16_68 Depth=2
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_123
// %bb.122:                             //   in Loop: Header=BB16_68 Depth=2
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.get_gc_cost
	mov	w3, #1239
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.get_gc_cost
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_123:                             //   in Loop: Header=BB16_68 Depth=2
	mov	x0, x22
	bl	f2fs_print_raw_sb_info
	mov	x0, x22
	bl	f2fs_print_ckpt_info
	mov	x0, x22
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/gc.c"; .popsection; .long 14472b - 14470b; .short 1239; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x22, .LBB16_128
// %bb.124:                             //   in Loop: Header=BB16_68 Depth=2
	add	x8, x22, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x22, #3376]
	cbnz	w8, .LBB16_128
// %bb.125:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x1, [x22, #3392]
	cbz	x1, .LBB16_147
// %bb.126:                             //   in Loop: Header=BB16_68 Depth=2
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB16_147
// %bb.127:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
.LBB16_128:                             //   in Loop: Header=BB16_68 Depth=2
	mov	w8, wzr
	b	.LBB16_147
.LBB16_129:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x8, [x9, #120]
	mov	w9, #40
	umull	x9, w28, w9
	ldr	w8, [x8, x9]
	ubfx	w8, w8, #6, #10
	b	.LBB16_147
.LBB16_130:                             //   in Loop: Header=BB16_68 Depth=2
	mov	x13, x12
.LBB16_131:                             //   in Loop: Header=BB16_68 Depth=2
	mov	w11, #40
	umull	x11, w28, w11
	ldr	w10, [x10, x11]
	ubfx	w14, w10, #6, #10
.LBB16_132:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x10, [x8, #152]
	ldr	w11, [x22, #1184]
	udiv	x9, x9, x13
	cmp	x9, x10
	b.hs	.LBB16_134
// %bb.133:                             //   in Loop: Header=BB16_68 Depth=2
	mov	x10, x9
	str	x9, [x8, #152]
.LBB16_134:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x13, [x8, #160]
	udiv	w12, w14, w12
	cmp	x9, x13
	b.ls	.LBB16_136
// %bb.135:                             //   in Loop: Header=BB16_68 Depth=2
	mov	x13, x9
	str	x9, [x8, #160]
.LBB16_136:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	w14, [x22, #1764]
	mov	w8, #100
	mul	w8, w12, w8
	cbz	w14, .LBB16_142
// %bb.137:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x12, [x22, #136]
	ldr	w15, [x22, #1236]
	ldr	x14, [x12]
	ldr	w14, [x14, #16]
	subs	w14, w15, w14
	b.eq	.LBB16_142
// %bb.138:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x15, [x12, #8]
	ldr	w12, [x12, #104]
	ldr	w15, [x15, #4]
	subs	w12, w15, w12
	b.hs	.LBB16_140
// %bb.139:                             //   in Loop: Header=BB16_68 Depth=2
	mov	x12, xzr
	mov	w14, w14
	b	.LBB16_141
.LBB16_140:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	w15, [x22, #1184]
	mov	w14, w14
	lsl	w12, w12, w15
	mov	w15, #100
	umull	x12, w12, w15
	udiv	x15, x12, x14
	cmp	w15, #9                 // =9
	b.gt	.LBB16_142
.LBB16_141:                             //   in Loop: Header=BB16_68 Depth=2
	udiv	x12, x12, x14
	add	w12, w12, w12, lsl #2
	lsl	w12, w12, #1
	cmp	w12, #1                 // =1
	csinc	w12, w12, wzr, gt
	b	.LBB16_143
.LBB16_142:                             //   in Loop: Header=BB16_68 Depth=2
	mov	w12, #100
.LBB16_143:                             //   in Loop: Header=BB16_68 Depth=2
	subs	x13, x13, x10
	lsr	w8, w8, w11
	b.ne	.LBB16_145
// %bb.144:                             //   in Loop: Header=BB16_68 Depth=2
	mov	w9, wzr
	b	.LBB16_146
.LBB16_145:                             //   in Loop: Header=BB16_68 Depth=2
	sub	x9, x9, x10
	mul	x9, x12, x9
	udiv	x9, x9, x13
	sub	w9, w12, w9
	and	w9, w9, #0xff
.LBB16_146:                             //   in Loop: Header=BB16_68 Depth=2
	and	w8, w8, #0xff
	mov	w11, #100
	sub	w10, w11, w8
	mul	w9, w10, w9
	mul	w9, w9, w11
	add	w8, w8, #100            // =100
	sdiv	w8, w9, w8
	mvn	w8, w8
.LBB16_147:                             //   in Loop: Header=BB16_68 Depth=2
	ldur	w9, [x29, #-68]
	cmp	w9, w8
	b.ls	.LBB16_67
// %bb.148:                             //   in Loop: Header=BB16_68 Depth=2
	stur	w28, [x29, #-56]
	stur	w8, [x29, #-68]
	b	.LBB16_67
.LBB16_149:                             //   in Loop: Header=BB16_68 Depth=2
	ldur	w10, [x29, #-96]
	cbz	w10, .LBB16_151
// %bb.150:                             //   in Loop: Header=BB16_68 Depth=2
	mov	x0, x22
	mov	w1, w28
	bl	f2fs_segment_has_free_slot
	tbz	w0, #0, .LBB16_67
	b	.LBB16_79
.LBB16_151:                             //   in Loop: Header=BB16_68 Depth=2
	cmp	w8, #2                  // =2
	b.lo	.LBB16_154
// %bb.152:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x9, [x9]
	mov	w11, #4767
	movk	w11, #10724, lsl #16
	umull	x11, w28, w11
	lsr	x11, x11, #32
	sub	w12, w28, w11
	ldr	x9, [x9, #120]
	add	w11, w11, w12, lsr #1
	lsr	w11, w11, #5
	mov	w12, #55
	mov	w10, wzr
	mul	w11, w11, w12
	mov	w13, #40
.LBB16_153:                             //   Parent Loop BB16_1 Depth=1
                                        //     Parent Loop BB16_68 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	umaddl	x12, w11, w13, x9
	ldrh	w12, [x12, #2]
	subs	w8, w8, #1              // =1
	add	w11, w11, #1            // =1
	and	w12, w12, #0x3ff
	add	w10, w12, w10
	b.ne	.LBB16_153
	b	.LBB16_155
.LBB16_154:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x8, [x9]
	mov	w9, #40
	ldr	x8, [x8, #120]
	umaddl	x8, w28, w9, x8
	ldrh	w8, [x8, #2]
	and	w10, w8, #0x3ff
.LBB16_155:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	w27, [sp, #68]          // 4-byte Folded Reload
	ldp	x25, x23, [sp, #48]     // 16-byte Folded Reload
	cbnz	w10, .LBB16_67
	b	.LBB16_79
.LBB16_156:                             //   in Loop: Header=BB16_68 Depth=2
	cmp	w9, #2                  // =2
	b.lo	.LBB16_158
// %bb.157:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x11, [x8, #128]
	ldr	w11, [x11, w10, uxtw #2]
	b	.LBB16_159
.LBB16_158:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	x11, [x8, #120]
	mov	w12, #40
	umull	x12, w28, w12
	ldr	w11, [x11, x12]
	ubfx	w11, w11, #6, #10
.LBB16_159:                             //   in Loop: Header=BB16_68 Depth=2
	ldr	w27, [sp, #68]          // 4-byte Folded Reload
	ldp	x25, x23, [sp, #48]     // 16-byte Folded Reload
	cbnz	w11, .LBB16_87
	b	.LBB16_67
.LBB16_160:                             //   in Loop: Header=BB16_1 Depth=1
	ldursw	x8, [x29, #-92]
	add	x10, x23, #184          // =184
	ldur	w9, [x29, #-72]
	lsl	x8, x8, #2
	ldr	w11, [x10, x8]
	cmp	w11, #0                 // =0
	ccmp	w28, w19, #2, eq
	csel	w11, w28, w19, hi
	add	w9, w11, w9
	str	w9, [x10, x8]
	ldursw	x8, [x29, #-92]
	ldr	w9, [x22, #1200]
	ldr	w11, [x22, #1192]
	lsl	x8, x8, #2
	ldr	w12, [x10, x8]
	mul	w9, w11, w9
	udiv	w11, w12, w9
	msub	w9, w11, w9, w12
	str	w9, [x10, x8]
	tbz	w26, #0, .LBB16_280
.LBB16_161:                             //   in Loop: Header=BB16_1 Depth=1
	ldur	x1, [x29, #-112]        // 8-byte Folded Reload
	mov	w2, #1
	mov	x0, x22
	bl	f2fs_check_rb_tree_consistence
	tbz	w0, #0, .LBB16_245
.LBB16_162:                             //   in Loop: Header=BB16_1 Depth=1
	ldur	w8, [x29, #-92]
	cmp	w8, #2                  // =2
	b.ne	.LBB16_199
// %bb.163:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x8, [x22, #136]
	ldr	w9, [x22, #1964]
	ldr	w10, [x22, #1960]
	ldr	w12, [x22, #1968]
	ldr	x11, [x8]
	mul	w9, w10, w9
	mov	w10, #34079
	ldp	x19, x8, [x11, #168]
	movk	w10, #20971, lsl #16
	umull	x9, w9, w10
	lsr	x9, x9, #37
	cmp	w12, w9
	csel	w24, w12, w9, hi
	cmp	x8, x19
	b.lo	.LBB16_233
// %bb.164:                             //   in Loop: Header=BB16_1 Depth=1
	add	x25, x8, #1             // =1
	sub	x9, x25, x19
	mov	x8, #-1
	str	x9, [sp, #120]          // 8-byte Folded Spill
	udiv	x8, x8, x9
	mov	x9, #62915
	movk	x9, #23592, lsl #16
	movk	x9, #49807, lsl #32
	lsr	x8, x8, #2
	movk	x9, #10485, lsl #48
	ldr	x28, [x22, #1936]
	umulh	x8, x8, x9
	lsr	x9, x8, #2
	lsr	x8, x8, #6
	cmp	x8, #625                // =625
	mov	w8, #10000
	csel	x26, x9, x8, lo
	cbz	x28, .LBB16_233
// %bb.165:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	w8, [x22, #1192]
	ldr	w9, [x22, #1188]
	ldr	w10, [x22, #1972]
	mov	w27, wzr
	str	x26, [sp, #112]         // 8-byte Folded Spill
	mul	w23, w9, w8
	mov	w8, w23
	stp	x8, x10, [sp, #96]      // 16-byte Folded Spill
	mov	w8, #100
	sub	w8, w8, w10
	str	x8, [sp, #88]           // 8-byte Folded Spill
.LBB16_166:                             //   Parent Loop BB16_1 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	x9, [x28, #24]
	subs	x8, x25, x9
	b.ls	.LBB16_181
// %bb.167:                             //   in Loop: Header=BB16_166 Depth=2
	cmp	x9, x19
	b.lo	.LBB16_181
// %bb.168:                             //   in Loop: Header=BB16_166 Depth=2
	ldr	x10, [x22, #136]
	ldr	w11, [x22, #1192]
	ldr	x12, [sp, #120]         // 8-byte Folded Reload
	ldr	w9, [x28, #32]
	ldr	x10, [x10]
	mul	x8, x8, x26
	cmp	w11, #2                 // =2
	udiv	x8, x8, x12
	b.lo	.LBB16_171
// %bb.169:                             //   in Loop: Header=BB16_166 Depth=2
	ldr	x10, [x10, #128]
	cmn	w9, #1                  // =1
	b.eq	.LBB16_172
// %bb.170:                             //   in Loop: Header=BB16_166 Depth=2
	udiv	w9, w9, w11
	b	.LBB16_173
.LBB16_171:                             //   in Loop: Header=BB16_166 Depth=2
	ldr	x10, [x10, #120]
	mov	w11, #40
	mul	x9, x9, x11
	ldr	w9, [x10, x9]
	ubfx	w26, w9, #6, #10
	b	.LBB16_174
.LBB16_172:                             //   in Loop: Header=BB16_166 Depth=2
	mov	w9, #-1
.LBB16_173:                             //   in Loop: Header=BB16_166 Depth=2
	ldr	w26, [x10, x9, lsl #2]
.LBB16_174:                             //   in Loop: Header=BB16_166 Depth=2
	ldr	x9, [sp, #104]          // 8-byte Folded Reload
	mul	x21, x8, x9
	cbz	w26, .LBB16_183
// %bb.175:                             //   in Loop: Header=BB16_166 Depth=2
	cmp	w23, w26
	b.eq	.LBB16_183
.LBB16_176:                             //   in Loop: Header=BB16_166 Depth=2
	sub	w8, w23, w26
	ldr	x26, [sp, #112]         // 8-byte Folded Reload
	ldr	x9, [sp, #96]           // 8-byte Folded Reload
	mul	x8, x26, x8
	udiv	x8, x8, x9
	ldr	x9, [sp, #88]           // 8-byte Folded Reload
	madd	x20, x8, x9, x21
	mov	w8, #-1
	cmp	x20, x8
	b.hs	.LBB16_191
.LBB16_177:                             //   in Loop: Header=BB16_166 Depth=2
	ldur	w9, [x29, #-68]
	mvn	w8, w20
	add	w27, w27, #1            // =1
	cmp	w9, w8
	b.hi	.LBB16_180
// %bb.178:                             //   in Loop: Header=BB16_166 Depth=2
	b.ne	.LBB16_181
// %bb.179:                             //   in Loop: Header=BB16_166 Depth=2
	ldur	x9, [x29, #-64]
	cmp	x21, x9
	b.ls	.LBB16_181
.LBB16_180:                             //   in Loop: Header=BB16_166 Depth=2
	stur	w8, [x29, #-68]
	stur	x21, [x29, #-64]
	ldr	w8, [x28, #32]
	stur	w8, [x29, #-56]
.LBB16_181:                             //   in Loop: Header=BB16_166 Depth=2
	cmp	w27, w24
	b.hs	.LBB16_233
// %bb.182:                             //   in Loop: Header=BB16_166 Depth=2
	mov	x0, x28
	bl	rb_next
	mov	x28, x0
	cbnz	x0, .LBB16_166
	b	.LBB16_233
.LBB16_183:                             //   in Loop: Header=BB16_166 Depth=2
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_186
// %bb.184:                             //   in Loop: Header=BB16_166 Depth=2
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_186
// %bb.185:                             //   in Loop: Header=BB16_166 Depth=2
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.atgc_lookup_victim
	mov	w3, #1391
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.atgc_lookup_victim
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_186:                             //   in Loop: Header=BB16_166 Depth=2
	mov	x0, x22
	bl	f2fs_print_raw_sb_info
	mov	x0, x22
	bl	f2fs_print_ckpt_info
	mov	x0, x22
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/gc.c"; .popsection; .long 14472b - 14470b; .short 1391; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x22, .LBB16_176
// %bb.187:                             //   in Loop: Header=BB16_166 Depth=2
	add	x8, x22, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x22, #3376]
	cbnz	w8, .LBB16_176
// %bb.188:                             //   in Loop: Header=BB16_166 Depth=2
	ldr	x1, [x22, #3392]
	cbz	x1, .LBB16_176
// %bb.189:                             //   in Loop: Header=BB16_166 Depth=2
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB16_176
// %bb.190:                             //   in Loop: Header=BB16_166 Depth=2
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB16_176
.LBB16_191:                             //   in Loop: Header=BB16_166 Depth=2
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_194
// %bb.192:                             //   in Loop: Header=BB16_166 Depth=2
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_194
// %bb.193:                             //   in Loop: Header=BB16_166 Depth=2
	adrp	x26, f2fs_dclient
	ldr	x0, [x26, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.atgc_lookup_victim
	mov	w3, #1397
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.atgc_lookup_victim
	bl	dsm_client_record
	ldr	x0, [x26, :lo12:f2fs_dclient]
	ldr	x26, [sp, #112]         // 8-byte Folded Reload
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_194:                             //   in Loop: Header=BB16_166 Depth=2
	mov	x0, x22
	bl	f2fs_print_raw_sb_info
	mov	x0, x22
	bl	f2fs_print_ckpt_info
	mov	x0, x22
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/gc.c"; .popsection; .long 14472b - 14470b; .short 1397; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x22, .LBB16_177
// %bb.195:                             //   in Loop: Header=BB16_166 Depth=2
	add	x8, x22, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x22, #3376]
	cbnz	w8, .LBB16_177
// %bb.196:                             //   in Loop: Header=BB16_166 Depth=2
	ldr	x1, [x22, #3392]
	cbz	x1, .LBB16_177
// %bb.197:                             //   in Loop: Header=BB16_166 Depth=2
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB16_177
// %bb.198:                             //   in Loop: Header=BB16_166 Depth=2
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB16_177
.LBB16_199:                             //   in Loop: Header=BB16_1 Depth=1
	ldur	w8, [x29, #-96]
	cmp	w8, #2                  // =2
	b.ne	.LBB16_225
// %bb.200:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x8, [x22, #136]
	ldr	w9, [x22, #1964]
	ldr	w10, [x22, #1960]
	ldr	w11, [x22, #1968]
	ldr	x8, [x8]
	mul	w10, w10, w9
	ldp	x19, x9, [x8, #168]
	mov	w8, #34079
	movk	w8, #20971, lsl #16
	umull	x8, w10, w8
	lsr	x8, x8, #37
	cmp	w11, w8
	csel	w21, w11, w8, hi
	cmp	x9, x19
	b.lo	.LBB16_233
// %bb.201:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	w23, [x22, #1188]
	mov	w8, wzr
	add	x24, x9, #1             // =1
	b	.LBB16_203
.LBB16_202:                             //   in Loop: Header=BB16_203 Depth=2
	mov	w8, #1
	cbnz	w25, .LBB16_233
.LBB16_203:                             //   Parent Loop BB16_1 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB16_206 Depth 3
	ldur	x3, [x29, #-48]
	ldur	x1, [x29, #-112]        // 8-byte Folded Reload
	sub	x2, x29, #24            // =24
	sub	x4, x29, #28            // =28
	mov	x0, x22
	mov	w25, w8
	stur	xzr, [x29, #-24]
	sturb	wzr, [x29, #-28]
	bl	f2fs_lookup_rb_tree_ext
	ldur	x28, [x29, #-24]
	cbz	x28, .LBB16_202
// %bb.204:                             //   in Loop: Header=BB16_203 Depth=2
	mov	w27, wzr
	b	.LBB16_206
.LBB16_205:                             //   in Loop: Header=BB16_206 Depth=3
	bl	rb_prev
	mov	x28, x0
	cbz	x0, .LBB16_202
.LBB16_206:                             //   Parent Loop BB16_1 Depth=1
                                        //     Parent Loop BB16_203 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	ldr	x8, [x28, #24]
	subs	x26, x8, x24
	b.hs	.LBB16_214
// %bb.207:                             //   in Loop: Header=BB16_206 Depth=3
	cmp	x8, x19
	b.lo	.LBB16_214
// %bb.208:                             //   in Loop: Header=BB16_206 Depth=3
	ldr	x8, [x22, #136]
	ldr	w9, [x28, #32]
	mov	w10, #40
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	madd	x8, x9, x10, x8
	ldrh	w8, [x8, #2]
	ands	w20, w8, #0x3ff
	b.eq	.LBB16_217
.LBB16_209:                             //   in Loop: Header=BB16_206 Depth=3
	cmp	w20, w23
	b.eq	.LBB16_214
// %bb.210:                             //   in Loop: Header=BB16_206 Depth=3
	ldur	x9, [x29, #-48]
	ldur	w10, [x29, #-68]
	mvn	w8, w20
	add	w27, w27, #1            // =1
	add	x9, x9, x26
	cmp	x9, #0                  // =0
	cneg	x9, x9, mi
	cmp	w10, w8
	sub	x9, x24, x9
	b.hi	.LBB16_213
// %bb.211:                             //   in Loop: Header=BB16_206 Depth=3
	b.ne	.LBB16_214
// %bb.212:                             //   in Loop: Header=BB16_206 Depth=3
	ldur	x10, [x29, #-64]
	cmp	x9, x10
	b.ls	.LBB16_214
.LBB16_213:                             //   in Loop: Header=BB16_206 Depth=3
	stur	w8, [x29, #-68]
	stur	x9, [x29, #-64]
	ldr	w8, [x28, #32]
	stur	w8, [x29, #-56]
.LBB16_214:                             //   in Loop: Header=BB16_206 Depth=3
	cmp	w27, w21
	b.hs	.LBB16_202
// %bb.215:                             //   in Loop: Header=BB16_206 Depth=3
	mov	x0, x28
	cmp	w25, #1                 // =1
	b.ne	.LBB16_205
// %bb.216:                             //   in Loop: Header=BB16_206 Depth=3
	bl	rb_next
	mov	x28, x0
	cbnz	x0, .LBB16_206
	b	.LBB16_202
.LBB16_217:                             //   in Loop: Header=BB16_206 Depth=3
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_220
// %bb.218:                             //   in Loop: Header=BB16_206 Depth=3
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_220
// %bb.219:                             //   in Loop: Header=BB16_206 Depth=3
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.atssr_lookup_victim
	mov	w3, #1460
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.atssr_lookup_victim
	bl	dsm_client_record
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_220:                             //   in Loop: Header=BB16_206 Depth=3
	mov	x0, x22
	bl	f2fs_print_raw_sb_info
	mov	x0, x22
	bl	f2fs_print_ckpt_info
	mov	x0, x22
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/gc.c"; .popsection; .long 14472b - 14470b; .short 1460; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x22, .LBB16_209
// %bb.221:                             //   in Loop: Header=BB16_206 Depth=3
	add	x8, x22, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x22, #3376]
	cbnz	w8, .LBB16_209
// %bb.222:                             //   in Loop: Header=BB16_206 Depth=3
	ldr	x1, [x22, #3392]
	cbz	x1, .LBB16_209
// %bb.223:                             //   in Loop: Header=BB16_206 Depth=3
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB16_209
// %bb.224:                             //   in Loop: Header=BB16_206 Depth=3
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB16_209
.LBB16_225:                             //   in Loop: Header=BB16_1 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_228
// %bb.226:                             //   in Loop: Header=BB16_1 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_228
// %bb.227:                             //   in Loop: Header=BB16_1 Depth=1
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.lookup_victim_by_age
	mov	w3, #1503
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.lookup_victim_by_age
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_228:                             //   in Loop: Header=BB16_1 Depth=1
	mov	x0, x22
	bl	f2fs_print_raw_sb_info
	mov	x0, x22
	bl	f2fs_print_ckpt_info
	mov	x0, x22
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/gc.c"; .popsection; .long 14472b - 14470b; .short 1503; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x22, .LBB16_233
// %bb.229:                             //   in Loop: Header=BB16_1 Depth=1
	add	x8, x22, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x22, #3376]
	cbnz	w8, .LBB16_233
// %bb.230:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x1, [x22, #3392]
	cbz	x1, .LBB16_233
// %bb.231:                             //   in Loop: Header=BB16_1 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB16_233
// %bb.232:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
.LBB16_233:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x25, [sp, #48]          // 8-byte Folded Reload
	ldr	x24, [x25]
	cmp	x25, x24
	b.eq	.LBB16_238
// %bb.234:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x26, [sp, #72]          // 8-byte Folded Reload
	ldr	w27, [sp, #68]          // 4-byte Folded Reload
	ldr	x23, [sp, #56]          // 8-byte Folded Reload
	mov	x21, #256
	movk	x21, #57005, lsl #48
	adrp	x28, victim_entry_slab
	b	.LBB16_236
.LBB16_235:                             //   in Loop: Header=BB16_236 Depth=2
	add	x8, x21, #34            // =34
	stp	x21, x8, [x24]
	ldr	x0, [x28, :lo12:victim_entry_slab]
	mov	x1, x19
	bl	kmem_cache_free
	ldr	w8, [x22, #1960]
	cmp	x25, x20
	mov	x24, x20
	sub	w8, w8, #1              // =1
	str	w8, [x22, #1960]
	b.eq	.LBB16_239
.LBB16_236:                             //   Parent Loop BB16_1 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	mov	x19, x24
	ldr	x20, [x19], #-40
	mov	x0, x24
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB16_235
// %bb.237:                             //   in Loop: Header=BB16_236 Depth=2
	ldp	x9, x8, [x24]
	str	x8, [x9, #8]
	str	x9, [x8]
	b	.LBB16_235
.LBB16_238:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	w8, [x22, #1960]
	ldr	x26, [sp, #72]          // 8-byte Folded Reload
	ldr	w27, [sp, #68]          // 4-byte Folded Reload
	ldr	x23, [sp, #56]          // 8-byte Folded Reload
.LBB16_239:                             //   in Loop: Header=BB16_1 Depth=1
	str	xzr, [x22, #1928]
	str	xzr, [x22, #1936]
	cbnz	w8, .LBB16_253
.LBB16_240:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x8, [x25]
	cmp	x25, x8
	b.ne	.LBB16_261
.LBB16_241:                             //   in Loop: Header=BB16_1 Depth=1
	ldur	w9, [x29, #-56]
	cmn	w9, #1                  // =1
	b.ne	.LBB16_281
// %bb.242:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x8, [x23, #136]
	ldur	x9, [x29, #-40]
	cmp	x8, x9
	b.hs	.LBB16_282
// %bb.243:                             //   in Loop: Header=BB16_1 Depth=1
	ldur	w8, [x29, #-96]
	mov	x9, xzr
	mov	w28, #-61
	b	.LBB16_1
.LBB16_244:                             //   in Loop: Header=BB16_1 Depth=1
	cbnz	w26, .LBB16_161
	b	.LBB16_280
.LBB16_245:                             //   in Loop: Header=BB16_1 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_248
// %bb.246:                             //   in Loop: Header=BB16_1 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_248
// %bb.247:                             //   in Loop: Header=BB16_1 Depth=1
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.lookup_victim_by_age
	mov	w3, #1496
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.lookup_victim_by_age
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_248:                             //   in Loop: Header=BB16_1 Depth=1
	mov	x0, x22
	bl	f2fs_print_raw_sb_info
	mov	x0, x22
	bl	f2fs_print_ckpt_info
	mov	x0, x22
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/gc.c"; .popsection; .long 14472b - 14470b; .short 1496; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x22, .LBB16_162
// %bb.249:                             //   in Loop: Header=BB16_1 Depth=1
	add	x8, x22, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x22, #3376]
	cbnz	w8, .LBB16_162
// %bb.250:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x1, [x22, #3392]
	cbz	x1, .LBB16_162
// %bb.251:                             //   in Loop: Header=BB16_1 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB16_162
// %bb.252:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB16_162
.LBB16_253:                             //   in Loop: Header=BB16_1 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_256
// %bb.254:                             //   in Loop: Header=BB16_1 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_256
// %bb.255:                             //   in Loop: Header=BB16_1 Depth=1
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.release_victim_entry
	mov	w3, #1519
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.release_victim_entry
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_256:                             //   in Loop: Header=BB16_1 Depth=1
	mov	x0, x22
	bl	f2fs_print_raw_sb_info
	mov	x0, x22
	bl	f2fs_print_ckpt_info
	mov	x0, x22
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/gc.c"; .popsection; .long 14472b - 14470b; .short 1519; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x22, .LBB16_240
// %bb.257:                             //   in Loop: Header=BB16_1 Depth=1
	add	x8, x22, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x22, #3376]
	cbnz	w8, .LBB16_240
// %bb.258:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x1, [x22, #3392]
	cbz	x1, .LBB16_240
// %bb.259:                             //   in Loop: Header=BB16_1 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB16_240
// %bb.260:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB16_240
.LBB16_261:                             //   in Loop: Header=BB16_1 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_264
// %bb.262:                             //   in Loop: Header=BB16_1 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_264
// %bb.263:                             //   in Loop: Header=BB16_1 Depth=1
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.release_victim_entry
	mov	w3, #1520
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.release_victim_entry
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_264:                             //   in Loop: Header=BB16_1 Depth=1
	mov	x0, x22
	bl	f2fs_print_raw_sb_info
	mov	x0, x22
	bl	f2fs_print_ckpt_info
	mov	x0, x22
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/gc.c"; .popsection; .long 14472b - 14470b; .short 1520; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x22, .LBB16_241
// %bb.265:                             //   in Loop: Header=BB16_1 Depth=1
	add	x8, x22, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x22, #3376]
	cbnz	w8, .LBB16_241
// %bb.266:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x1, [x22, #3392]
	cbz	x1, .LBB16_241
// %bb.267:                             //   in Loop: Header=BB16_1 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB16_241
// %bb.268:                             //   in Loop: Header=BB16_1 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB16_241
.LBB16_269:
	ldr	x8, [x22, #136]
	mov	w11, #40
	mul	x11, x9, x11
	ldr	x10, [x8]
	ldr	x10, [x10, #120]
	ldrh	w10, [x10, x11]
	tst	w10, #0xffc0
	b.eq	.LBB16_282
// %bb.270:
	ldr	x12, [x8, #24]
	ldr	w11, [x22, #1192]
	ldr	w13, [x12, #100]
	udiv	w10, w9, w11
	udiv	w13, w13, w11
	cmp	w13, w10
	b.eq	.LBB16_279
// %bb.271:
	ldr	w13, [x12, #220]
	udiv	w13, w13, w11
	cmp	w13, w10
	b.eq	.LBB16_279
// %bb.272:
	ldr	w13, [x12, #340]
	udiv	w13, w13, w11
	cmp	w13, w10
	b.eq	.LBB16_279
// %bb.273:
	ldr	w13, [x12, #460]
	udiv	w13, w13, w11
	cmp	w13, w10
	b.eq	.LBB16_279
// %bb.274:
	ldr	w13, [x12, #580]
	udiv	w13, w13, w11
	cmp	w13, w10
	b.eq	.LBB16_279
// %bb.275:
	ldr	w13, [x12, #700]
	udiv	w13, w13, w11
	cmp	w13, w10
	b.eq	.LBB16_279
// %bb.276:
	ldr	w13, [x12, #820]
	udiv	w13, w13, w11
	cmp	w13, w10
	b.eq	.LBB16_279
// %bb.277:
	ldr	w12, [x12, #940]
	udiv	w11, w12, w11
	cmp	w11, w10
	b.eq	.LBB16_279
// %bb.278:
	ldr	w23, [x22, #1984]
	cmp	w23, w10
	b.ne	.LBB16_300
.LBB16_279:
	ldr	x20, [sp, #24]          // 8-byte Folded Reload
	mov	w28, #-16
	b	.LBB16_298
.LBB16_280:
	ldur	w9, [x29, #-56]
	ldr	x26, [sp, #72]          // 8-byte Folded Reload
	cmn	w9, #1                  // =1
	b.eq	.LBB16_282
.LBB16_281:
	ldur	w8, [x29, #-72]
	udiv	w10, w9, w8
	mul	w8, w10, w8
	ldr	x10, [sp, #80]          // 8-byte Folded Reload
	str	w8, [x10]
	ldur	w8, [x29, #-96]
	cbnz	w8, .LBB16_289
	b	.LBB16_284
.LBB16_282:
	ldr	x20, [sp, #24]          // 8-byte Folded Reload
	mov	w28, #-61
	b	.LBB16_298
.LBB16_283:
	ldp	x26, x8, [sp, #72]      // 16-byte Folded Reload
	stur	w9, [x29, #-56]
	str	w9, [x8]
	mov	w8, #-1
	str	w8, [x22, #1992]
.LBB16_284:
	ldr	w8, [x22, #1192]
	cmp	w27, #1                 // =1
	udiv	w8, w9, w8
	b.eq	.LBB16_288
// %bb.285:
	ldr	x9, [sp, #16]           // 8-byte Folded Reload
	lsr	w10, w8, #6
	mov	w11, #1
	lsl	x8, x11, x8
	ldr	x9, [x9, #144]
	add	x9, x9, w10, uxtw #3
	//APP
	// atomic64_or
	prfm	pstl1strm, [x9]
1:	ldxr	x10, [x9]
	orr	x10, x10, x8
	stxr	w11, x10, [x9]
	cbnz	w11, 1b
	//NO_APP
	ldur	w8, [x29, #-56]
	cmn	w8, #1                  // =1
	b.ne	.LBB16_289
// %bb.286:
	ldr	x20, [sp, #24]          // 8-byte Folded Reload
	mov	w28, wzr
	b	.LBB16_298
.LBB16_287:
	ldp	x26, x8, [sp, #72]      // 16-byte Folded Reload
	stur	w9, [x29, #-56]
	mov	w10, #-1
	str	w9, [x8]
	ldr	w8, [x22, #1192]
	str	w10, [x22, #1996]
	udiv	w8, w9, w8
.LBB16_288:
	str	w8, [x22, #1984]
.LBB16_289:
	ldr	w23, [x22, #1984]
	ldr	x8, [x22, #136]
	mov	w28, wzr
.LBB16_290:
	ldp	x8, x9, [x8, #8]
	adrp	x10, __tracepoint_f2fs_get_victim+8
	ldr	x22, [x22]
	ldr	w24, [x9, #140]
	ldr	w25, [x8, #4]
	ldr	w8, [x10, :lo12:__tracepoint_f2fs_get_victim+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB16_297
// %bb.291:
	adrp	x9, cpu_number
	//APP
	.if 1 == 1
661:
	mrs x8, tpidr_el1
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 11
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	mrs x8, tpidr_el2
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	add	x9, x9, :lo12:cpu_number
	ldr	w8, [x8, x9]
	adrp	x10, __cpu_online_mask
	add	x10, x10, :lo12:__cpu_online_mask
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x10, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB16_297
// %bb.292:
	//APP
	mrs x19, sp_el0
	//NO_APP
	ldr	w8, [x19, #24]
	add	w8, w8, #1              // =1
	str	w8, [x19, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_get_victim+32
	ldr	x20, [x8, :lo12:__tracepoint_f2fs_get_victim+32]
	cbz	x20, .LBB16_294
.LBB16_293:                             // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x20]
	sub	x4, x29, #96            // =96
	mov	x1, x22
	mov	w2, w26
	mov	w3, w27
	mov	w5, w23
	mov	w6, w24
	mov	w7, w25
	blr	x8
	ldr	x8, [x20, #24]!
	cbnz	x8, .LBB16_293
.LBB16_294:
	//APP
	//NO_APP
	ldr	x8, [x19, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x19, #24]
	ldr	x20, [sp, #24]          // 8-byte Folded Reload
	b.eq	.LBB16_296
// %bb.295:
	ldr	x8, [x19, #24]
	cbnz	x8, .LBB16_298
.LBB16_296:
	bl	preempt_schedule_notrace
	b	.LBB16_298
.LBB16_297:
	ldr	x20, [sp, #24]          // 8-byte Folded Reload
.LBB16_298:
	mov	x0, x20
	bl	mutex_unlock
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB16_302
// %bb.299:
	mov	w0, w28
	ldp	x20, x19, [sp, #320]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #304]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #288]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #272]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #256]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #240]    // 16-byte Folded Reload
	add	sp, sp, #336            // =336
	ret
.LBB16_300:
	ldr	x26, [sp, #72]          // 8-byte Folded Reload
	stur	w9, [x29, #-56]
	b	.LBB16_290
.LBB16_301:
	ldr	x26, [sp, #72]          // 8-byte Folded Reload
	b	.LBB16_281
.LBB16_302:
	bl	__stack_chk_fail
.Lfunc_end16:
	.size	get_victim_by_default, .Lfunc_end16-get_victim_by_default
                                        // -- End function
	.type	f2fs_init_gc_thread.__key,@object // @f2fs_init_gc_thread.__key
	.local	f2fs_init_gc_thread.__key
	.comm	f2fs_init_gc_thread.__key,1,1
	.type	.L.str,@object          // @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"&gc_th->gc_wait_queue_head"
	.size	.L.str, 27

	.type	f2fs_init_gc_thread.__key.1,@object // @f2fs_init_gc_thread.__key.1
	.local	f2fs_init_gc_thread.__key.1
	.comm	f2fs_init_gc_thread.__key.1,1,1
	.type	.L.str.2,@object        // @.str.2
.L.str.2:
	.asciz	"&gc_th->fg_gc_wait"
	.size	.L.str.2, 19

	.type	.L.str.3,@object        // @.str.3
.L.str.3:
	.asciz	"f2fs_gc"
	.size	.L.str.3, 8

	.type	.L.str.4,@object        // @.str.4
.L.str.4:
	.asciz	"f2fs_gc-%u:%u"
	.size	.L.str.4, 14

	.type	.L.str.5,@object        // @.str.5
.L.str.5:
	.asciz	"\0013f2fs_gc_loop same victim retry:%lu in %s:%d segno:%u type:%d blocks:%u free:%u prefree:%u rsvd:%u"
	.size	.L.str.5, 100

	.type	.L.str.6,@object        // @.str.6
.L.str.6:
	.asciz	"f2fs_victim_entry"
	.size	.L.str.6, 18

	.type	victim_entry_slab,@object // @victim_entry_slab
	.local	victim_entry_slab
	.comm	victim_entry_slab,8,8
	.type	default_v_ops,@object   // @default_v_ops
	.section	.rodata,"a",@progbits
	.p2align	3
default_v_ops:
	.xword	get_victim_by_default
	.size	default_v_ops, 8

	.type	.L.str.7,@object        // @.str.7
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.7:
	.asciz	"\0013Should run fsck to repair first."
	.size	.L.str.7, 35

	.type	.L.str.8,@object        // @.str.8
.L.str.8:
	.asciz	"\0013Checkpoint should be enabled."
	.size	.L.str.8, 32

	.type	.L.str.9,@object        // @.str.9
.L.str.9:
	.asciz	"\0013resize_fs failed, should run fsck to repair!"
	.size	.L.str.9, 47

	.type	.L.str.10,@object       // @.str.10
.L.str.10:
	.asciz	"\0013f2fs-gc: some signals have been received..."
	.size	.L.str.10, 46

	.type	.L.str.11,@object       // @.str.11
.L.str.11:
	.asciz	"\0015BG_GC: Size=%lluMB,Free=%lluMB,count=%d,free_sec=%u,reserved_sec=%u,node_secs=%d,dent_secs=%d"
	.size	.L.str.11, 96

	.type	.L.str.13,@object       // @.str.13
.L.str.13:
	.asciz	"\0015block_cnt[%d]: %d"
	.size	.L.str.13, 20

	.type	.L.str.14,@object       // @.str.14
.L.str.14:
	.asciz	"\0013Inconsistent segment (%u) type [%d, %d] in SSA and SIT"
	.size	.L.str.14, 57

	.type	.L.str.15,@object       // @.str.15
.L.str.15:
	.asciz	"SBI_NEED_FSCK:%s:%d\n"
	.size	.L.str.15, 21

	.type	.L__func__.do_garbage_collect,@object // @__func__.do_garbage_collect
.L__func__.do_garbage_collect:
	.asciz	"do_garbage_collect"
	.size	.L__func__.do_garbage_collect, 19

	.type	.L.str.16,@object       // @.str.16
.L.str.16:
	.asciz	"F2FS bug: %s:%d\n"
	.size	.L.str.16, 17

	.type	.L__func__.sanity_check_seg_type,@object // @__func__.sanity_check_seg_type
.L__func__.sanity_check_seg_type:
	.asciz	"sanity_check_seg_type"
	.size	.L__func__.sanity_check_seg_type, 22

	.type	.L__func__.f2fs_put_page,@object // @__func__.f2fs_put_page
.L__func__.f2fs_put_page:
	.asciz	"f2fs_put_page"
	.size	.L__func__.f2fs_put_page, 14

	.type	.L.str.17,@object       // @.str.17
.L.str.17:
	.asciz	"\0014%s: valid data with mismatched node version."
	.size	.L.str.17, 47

	.type	.L__func__.is_alive,@object // @__func__.is_alive
.L__func__.is_alive:
	.asciz	"is_alive"
	.size	.L__func__.is_alive, 9

	.type	.L.str.19,@object       // @.str.19
.L.str.19:
	.asciz	"\0013f2fs_gc_loop nomem retry:%lu in %s:%d"
	.size	.L.str.19, 40

	.type	.L__func__.move_data_page,@object // @__func__.move_data_page
.L__func__.move_data_page:
	.asciz	"move_data_page"
	.size	.L__func__.move_data_page, 15

	.type	.L__func__.get_gc_cost,@object // @__func__.get_gc_cost
.L__func__.get_gc_cost:
	.asciz	"get_gc_cost"
	.size	.L__func__.get_gc_cost, 12

	.type	.L__func__.lookup_victim_by_age,@object // @__func__.lookup_victim_by_age
.L__func__.lookup_victim_by_age:
	.asciz	"lookup_victim_by_age"
	.size	.L__func__.lookup_victim_by_age, 21

	.type	.L__func__.atgc_lookup_victim,@object // @__func__.atgc_lookup_victim
.L__func__.atgc_lookup_victim:
	.asciz	"atgc_lookup_victim"
	.size	.L__func__.atgc_lookup_victim, 19

	.type	.L__func__.atssr_lookup_victim,@object // @__func__.atssr_lookup_victim
.L__func__.atssr_lookup_victim:
	.asciz	"atssr_lookup_victim"
	.size	.L__func__.atssr_lookup_victim, 20

	.type	.L__func__.release_victim_entry,@object // @__func__.release_victim_entry
.L__func__.release_victim_entry:
	.asciz	"release_victim_entry"
	.size	.L__func__.release_victim_entry, 21

	.type	.L.str.20,@object       // @.str.20
.L.str.20:
	.asciz	"\0013cp_rwsem_timeout: start: %lu now: %lu\n"
	.size	.L.str.20, 41

	.type	.L__const.free_segment_range.cpc,@object // @__const.free_segment_range.cpc
	.section	.rodata.cst32,"aM",@progbits,32
	.p2align	3
.L__const.free_segment_range.cpc:
	.word	128                     // 0x80
	.zero	4
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	.L__const.free_segment_range.cpc, 32

	.type	.L.str.21,@object       // @.str.21
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.21:
	.asciz	"\0013segno %u should be free but still inuse!"
	.size	.L.str.21, 43

	.type	.L__func__.free_segment_range,@object // @__func__.free_segment_range
.L__func__.free_segment_range:
	.asciz	"free_segment_range"
	.size	.L__func__.free_segment_range, 19

	.type	.Lswitch.table.get_victim_by_default,@object // @switch.table.get_victim_by_default
	.section	.rodata,"a",@progbits
	.p2align	2
.Lswitch.table.get_victim_by_default:
	.word	0                       // 0x0
	.word	1                       // 0x1
	.word	0                       // 0x0
	.word	2                       // 0x2
	.word	1                       // 0x1
	.size	.Lswitch.table.get_victim_by_default, 20

	.ident	"Android (6443078 based on r383902) clang version 11.0.1 (https://android.googlesource.com/toolchain/llvm-project b397f81060ce6d701042b782172ed13bee898b79)"
	.section	".note.GNU-stack","",@progbits
