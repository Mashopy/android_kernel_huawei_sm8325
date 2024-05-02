	.text
	.file	"mas_blk_busy_idle_notify.c"
	.globl	mas_blk_busyidle_handler_latency_check_timer_expire // -- Begin function mas_blk_busyidle_handler_latency_check_timer_expire
	.p2align	2
	.type	mas_blk_busyidle_handler_latency_check_timer_expire,@function
mas_blk_busyidle_handler_latency_check_timer_expire: // @mas_blk_busyidle_handler_latency_check_timer_expire
.Lmas_blk_busyidle_handler_latency_check_timer_expire$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	add	x2, x0, #80             // =80
	adrp	x0, .L.str
	adrp	x1, .L__func__.mas_blk_busyidle_handler_latency_check_timer_expire
	add	x0, x0, :lo12:.L.str
	add	x1, x1, :lo12:.L__func__.mas_blk_busyidle_handler_latency_check_timer_expire
	mov	w3, #1000
	mov	x29, sp
	bl	printk
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end0:
	.size	mas_blk_busyidle_handler_latency_check_timer_expire, .Lfunc_end0-mas_blk_busyidle_handler_latency_check_timer_expire
                                        // -- End function
	.globl	mas_blk_busyidle_notify_handler // -- Begin function mas_blk_busyidle_notify_handler
	.p2align	2
	.type	mas_blk_busyidle_notify_handler,@function
mas_blk_busyidle_notify_handler:        // @mas_blk_busyidle_notify_handler
.Lmas_blk_busyidle_notify_handler$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #56]
	add	x19, x0, #24            // =24
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	cbz	x8, .LBB1_19
// %bb.1:
	mov	x23, x1
	mov	x22, x0
	bl	ktime_get
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	sub	x21, x22, #56           // =56
	mov	x20, x0
	mov	x0, x21
	add	x1, x8, #250            // =250
	bl	mod_timer
	cmp	x23, #3                 // =3
	b.hi	.LBB1_15
// %bb.2:
	adrp	x8, .LJTI1_0
	add	x8, x8, :lo12:.LJTI1_0
	adr	x9, .LBB1_3
	ldrb	w10, [x8, x23]
	add	x9, x9, x10, lsl #2
	br	x9
.LBB1_3:
	ldr	w8, [x22, #80]
	cmp	w8, #1                  // =1
	b.ne	.LBB1_15
// %bb.4:
	ldr	x8, [x22, #56]
	mov	x0, x19
	mov	w1, wzr
	blr	x8
	str	wzr, [x22, #80]
	b	.LBB1_14
.LBB1_5:
	ldr	w8, [x22, #80]
	cbnz	w8, .LBB1_15
// %bb.6:
	ldr	x8, [x22, #56]
	mov	w1, #1
	mov	x0, x19
	mov	w23, #1
	blr	x8
	cmp	w0, #1                  // =1
	str	w23, [x22, #80]
	b.ne	.LBB1_14
.LBB1_7:
	mov	w22, #32769
	b	.LBB1_16
.LBB1_8:
	ldr	x8, [x22, #72]
	ldrb	w8, [x8, #25]
	tbz	w8, #1, .LBB1_15
// %bb.9:
	ldr	w8, [x22, #84]
	cmp	w8, #1                  // =1
	b.ne	.LBB1_15
// %bb.10:
	ldr	x8, [x22, #56]
	mov	w1, #2
	mov	x0, x19
	blr	x8
	str	wzr, [x22, #84]
	b	.LBB1_14
.LBB1_11:
	ldr	x8, [x22, #72]
	ldrb	w8, [x8, #25]
	tbz	w8, #1, .LBB1_15
// %bb.12:
	ldr	w8, [x22, #84]
	cbnz	w8, .LBB1_15
// %bb.13:
	ldr	x8, [x22, #56]
	mov	w1, #3
	mov	x0, x19
	blr	x8
	mov	w8, #1
	cmp	w0, #1                  // =1
	str	w8, [x22, #84]
	b.eq	.LBB1_7
.LBB1_14:
	cmp	w0, #2                  // =2
	b.eq	.LBB1_18
.LBB1_15:
	mov	w22, wzr
.LBB1_16:
	mov	x0, x21
	bl	del_timer_sync
	bl	ktime_get
	mov	w8, #38528
	movk	w8, #152, lsl #16
	add	x8, x20, x8
	cmp	x0, x8
	b.gt	.LBB1_20
.LBB1_17:
	mov	w0, w22
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB1_18:
	adrp	x0, .L.str.2
	adrp	x1, .L__func__.mas_blk_busyidle_notify_handler
	add	x0, x0, :lo12:.L.str.2
	add	x1, x1, :lo12:.L__func__.mas_blk_busyidle_notify_handler
	mov	x2, x19
	bl	printk
	b	.LBB1_15
.LBB1_19:
	adrp	x0, .L.str.1
	adrp	x1, .L__func__.mas_blk_busyidle_notify_handler
	add	x0, x0, :lo12:.L.str.1
	add	x1, x1, :lo12:.L__func__.mas_blk_busyidle_notify_handler
	mov	x2, x19
	bl	printk
	mov	w22, wzr
	b	.LBB1_17
.LBB1_20:
	bl	ktime_get
	mov	x9, #13531
	movk	x9, #55222, lsl #16
	movk	x9, #56962, lsl #32
	sub	x8, x0, x20
	movk	x9, #17179, lsl #48
	smulh	x8, x8, x9
	asr	x9, x8, #18
	adrp	x0, .L.str.3
	adrp	x1, .L__func__.mas_blk_busyidle_notify_handler
	add	x3, x9, x8, lsr #63
	add	x0, x0, :lo12:.L.str.3
	add	x1, x1, :lo12:.L__func__.mas_blk_busyidle_notify_handler
	mov	x2, x19
	bl	printk
	b	.LBB1_17
.Lfunc_end1:
	.size	mas_blk_busyidle_notify_handler, .Lfunc_end1-mas_blk_busyidle_notify_handler
	.section	.rodata,"a",@progbits
.LJTI1_0:
	.byte	(.LBB1_3-.LBB1_3)>>2
	.byte	(.LBB1_5-.LBB1_3)>>2
	.byte	(.LBB1_8-.LBB1_3)>>2
	.byte	(.LBB1_11-.LBB1_3)>>2
                                        // -- End function
	.text
	.globl	blk_lld_fg_io_idle_notify // -- Begin function blk_lld_fg_io_idle_notify
	.p2align	2
	.type	blk_lld_fg_io_idle_notify,@function
blk_lld_fg_io_idle_notify:              // @blk_lld_fg_io_idle_notify
.Lblk_lld_fg_io_idle_notify$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	w8, [x0]
	mov	x29, sp
	cmp	w8, #122                // =122
	b.ne	.LBB2_2
// %bb.1:
	adrp	x8, blk_fg_io_idle_notify_workqueue
	ldr	x19, [x8, :lo12:blk_fg_io_idle_notify_workqueue]
	ldr	w8, [x0, #1272]
	add	x20, x0, #1136          // =1136
	mov	w0, w8
	bl	__msecs_to_jiffies
	mov	x3, x0
	mov	w0, #8
	mov	x1, x19
	mov	x2, x20
	bl	queue_delayed_work_on
.LBB2_2:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end2:
	.size	blk_lld_fg_io_idle_notify, .Lfunc_end2-blk_lld_fg_io_idle_notify
                                        // -- End function
	.globl	mas_blk_idle_notify_work // -- Begin function mas_blk_idle_notify_work
	.p2align	2
	.type	mas_blk_idle_notify_work,@function
mas_blk_idle_notify_work:               // @mas_blk_idle_notify_work
.Lmas_blk_idle_notify_work$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x20, x0
	ldrb	w8, [x20, #-8]!
	mov	x29, sp
	cbz	w8, .LBB3_2
// %bb.1:
	str	wzr, [x20, #224]
.LBB3_2:
	add	x19, x20, #232          // =232
	mov	x0, x19
	bl	mutex_lock
	ldr	w8, [x20, #224]
	cbnz	w8, .LBB3_5
// %bb.3:
	ldr	w8, [x20, #264]
	cmp	w8, #1                  // =1
	b.eq	.LBB3_5
// %bb.4:
	mov	w8, #1
	add	x0, x20, #168           // =168
	mov	w1, #1
	mov	x2, xzr
	str	w8, [x20, #264]
	bl	blocking_notifier_call_chain
.LBB3_5:
	mov	x0, x19
	bl	mutex_unlock
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end3:
	.size	mas_blk_idle_notify_work, .Lfunc_end3-mas_blk_idle_notify_work
                                        // -- End function
	.globl	blk_lld_idle_notify     // -- Begin function blk_lld_idle_notify
	.p2align	2
	.type	blk_lld_idle_notify,@function
blk_lld_idle_notify:                    // @blk_lld_idle_notify
.Lblk_lld_idle_notify$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	w8, [x0]
	mov	x29, sp
	cmp	w8, #122                // =122
	b.ne	.LBB4_2
// %bb.1:
	adrp	x8, blk_idle_notify_workqueue
	ldr	x19, [x8, :lo12:blk_idle_notify_workqueue]
	ldr	w8, [x0, #1008]
	add	x20, x0, #872           // =872
	mov	w0, w8
	bl	__msecs_to_jiffies
	mov	x3, x0
	mov	w0, #8
	mov	x1, x19
	mov	x2, x20
	bl	queue_delayed_work_on
.LBB4_2:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end4:
	.size	blk_lld_idle_notify, .Lfunc_end4-blk_lld_idle_notify
                                        // -- End function
	.globl	mas_blk_busyidle_check_bio // -- Begin function mas_blk_busyidle_check_bio
	.p2align	2
	.type	mas_blk_busyidle_check_bio,@function
mas_blk_busyidle_check_bio:             // @mas_blk_busyidle_check_bio
.Lmas_blk_busyidle_check_bio$local:
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x26, x25, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	mov	x29, sp
	mov	x21, x1
	mov	x19, x0
	bl	mas_blk_get_lld
	ldrb	w8, [x21, #136]
	str	x19, [x21, #88]
	tbnz	w8, #0, .LBB5_12
// %bb.1:
	add	x22, x0, #1368          // =1368
	mov	x20, x0
	orr	w8, w8, #0x1
	mov	x0, x22
	strb	w8, [x21, #136]
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x20, #1320]
	ldr	x25, [x20, #1344]
	add	x24, x21, #328          // =328
	add	x26, x20, #1336         // =1336
	mov	x23, x0
	add	x8, x8, #1              // =1
	mov	x0, x24
	mov	x1, x25
	mov	x2, x26
	str	x8, [x20, #1320]
	bl	__list_add_valid
	tbz	w0, #0, .LBB5_3
// %bb.2:
	str	x24, [x20, #1344]
	stp	x26, x25, [x21, #328]
	str	x24, [x25]
.LBB5_3:
	mov	x0, x22
	mov	x1, x23
	bl	_raw_spin_unlock_irqrestore
	ldr	x8, [x20, #24]
	tbz	w8, #3, .LBB5_6
// %bb.4:
	ldr	w8, [x20, #1088]
	cbz	w8, .LBB5_13
.LBB5_5:
	add	x8, x20, #1088          // =1088
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x20, #24]
.LBB5_6:
	tbz	w8, #9, .LBB5_12
// %bb.7:
	ldrb	w8, [x21, #290]
	cbz	w8, .LBB5_12
// %bb.8:
	ldrb	w8, [x21, #136]
	tbnz	w8, #5, .LBB5_12
// %bb.9:
	orr	w8, w8, #0x20
	strb	w8, [x21, #136]
	ldr	w8, [x20, #24]
	mov	w9, #520
	mvn	w8, w8
	tst	x8, x9
	b.ne	.LBB5_12
// %bb.10:
	ldr	w8, [x20, #1276]
	add	x22, x20, #1276         // =1276
	cbz	w8, .LBB5_16
.LBB5_11:
	//APP
	// atomic_add
	prfm	pstl1strm, [x22]
1:	ldxr	w8, [x22]
	add	w8, w8, 1
	stxr	w9, w8, [x22]
	cbnz	w9, 1b

	//NO_APP
.LBB5_12:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB5_13:
	add	x22, x20, #1096         // =1096
	mov	x0, x22
	bl	mutex_lock
	add	x0, x20, #872           // =872
	bl	cancel_delayed_work
	ldr	w8, [x20, #1128]
	cmp	w8, #1                  // =1
	b.ne	.LBB5_15
// %bb.14:
	add	x0, x20, #1032          // =1032
	mov	x1, xzr
	mov	x2, xzr
	str	wzr, [x20, #1128]
	bl	blocking_notifier_call_chain
.LBB5_15:
	mov	x0, x22
	bl	mutex_unlock
	b	.LBB5_5
.LBB5_16:
	add	x21, x20, #1280         // =1280
	mov	x0, x21
	bl	mutex_lock
	add	x0, x20, #1136          // =1136
	bl	cancel_delayed_work
	ldr	w8, [x20, #1312]
	cmp	w8, #1                  // =1
	b.ne	.LBB5_18
// %bb.17:
	add	x0, x20, #1032          // =1032
	mov	w1, #2
	mov	x2, x19
	str	wzr, [x20, #1312]
	bl	blocking_notifier_call_chain
.LBB5_18:
	mov	x0, x21
	bl	mutex_unlock
	b	.LBB5_11
.Lfunc_end5:
	.size	mas_blk_busyidle_check_bio, .Lfunc_end5-mas_blk_busyidle_check_bio
                                        // -- End function
	.globl	mas_blk_busyidle_check_request_bio // -- Begin function mas_blk_busyidle_check_request_bio
	.p2align	2
	.type	mas_blk_busyidle_check_request_bio,@function
mas_blk_busyidle_check_request_bio:     // @mas_blk_busyidle_check_request_bio
.Lmas_blk_busyidle_check_request_bio$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x19, [x1, #64]
	mov	x29, sp
	cbz	x19, .LBB6_4
// %bb.1:
	mov	x20, x0
.LBB6_2:                                // =>This Inner Loop Header: Depth=1
	mov	x0, x20
	mov	x1, x19
	bl	mas_blk_busyidle_check_bio
	ldr	x19, [x19]
	cbnz	x19, .LBB6_2
// %bb.3:
	mov	w0, #1
	b	.LBB6_5
.LBB6_4:
	mov	w0, wzr
.LBB6_5:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end6:
	.size	mas_blk_busyidle_check_request_bio, .Lfunc_end6-mas_blk_busyidle_check_request_bio
                                        // -- End function
	.globl	mas_blk_busyidle_end_rq // -- Begin function mas_blk_busyidle_end_rq
	.p2align	2
	.type	mas_blk_busyidle_end_rq,@function
mas_blk_busyidle_end_rq:                // @mas_blk_busyidle_end_rq
.Lmas_blk_busyidle_end_rq$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	mov	x19, x0
	ldr	x0, [x0]
	mov	x29, sp
	mov	w20, w1
	bl	mas_blk_get_lld
	ldr	x8, [x0, #1328]
	mov	x21, x0
	cbz	x8, .LBB7_4
// %bb.1:
	add	x22, x21, #1368         // =1368
	mov	x0, x22
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x21, #1328]
	add	x24, x19, #720          // =720
	mov	x23, x0
	mov	x0, x24
	sub	x8, x8, #1              // =1
	str	x8, [x21, #1328]
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB7_3
// %bb.2:
	ldr	x8, [x19, #728]
	ldr	x9, [x19, #720]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB7_3:
	mov	x0, x22
	mov	x1, x23
	str	x24, [x19, #720]
	str	x24, [x19, #728]
	bl	_raw_spin_unlock_irqrestore
.LBB7_4:
	mov	x0, x21
	bl	blk_idle_count
	ldr	x8, [x19, #280]
	cbz	x8, .LBB7_6
// %bb.5:
	mov	x0, x19
	mov	w1, w20
	blr	x8
.LBB7_6:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end7:
	.size	mas_blk_busyidle_end_rq, .Lfunc_end7-mas_blk_busyidle_end_rq
                                        // -- End function
	.p2align	2               // -- Begin function blk_idle_count
	.type	blk_idle_count,@function
blk_idle_count:                         // @blk_idle_count
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldrb	w8, [x0, #24]
	mov	x29, sp
	tbz	w8, #3, .LBB8_4
// %bb.1:
	ldrb	w8, [x0, #864]
	cbnz	w8, .LBB8_4
// %bb.2:
	ldr	w8, [x0, #1088]
	cbz	w8, .LBB8_5
// %bb.3:
	add	x8, x0, #1088           // =1088
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
	cbz	w10, .LBB8_6
.LBB8_4:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB8_5:
	adrp	x0, .L.str.10
	adrp	x1, .L__func__.blk_idle_count
	add	x0, x0, :lo12:.L.str.10
	add	x1, x1, :lo12:.L__func__.blk_idle_count
	bl	printk
	b	.LBB8_4
.LBB8_6:
	ldr	w8, [x0]
	cmp	w8, #122                // =122
	b.ne	.LBB8_4
// %bb.7:
	adrp	x8, blk_idle_notify_workqueue
	ldr	x19, [x8, :lo12:blk_idle_notify_workqueue]
	ldr	w8, [x0, #1008]
	add	x20, x0, #872           // =872
	mov	w0, w8
	bl	__msecs_to_jiffies
	mov	x3, x0
	mov	w0, #8
	mov	x1, x19
	mov	x2, x20
	bl	queue_delayed_work_on
	b	.LBB8_4
.Lfunc_end8:
	.size	blk_idle_count, .Lfunc_end8-blk_idle_count
                                        // -- End function
	.globl	mas_blk_busyidle_check_execute_request // -- Begin function mas_blk_busyidle_check_execute_request
	.p2align	2
	.type	mas_blk_busyidle_check_execute_request,@function
mas_blk_busyidle_check_execute_request: // @mas_blk_busyidle_check_execute_request
.Lmas_blk_busyidle_check_execute_request$local:
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x26, x25, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x21, [x1, #64]
	mov	x19, x2
	mov	x20, x1
	mov	x29, sp
	cbz	x21, .LBB9_4
// %bb.1:
	mov	x22, x0
.LBB9_2:                                // =>This Inner Loop Header: Depth=1
	mov	x0, x22
	mov	x1, x21
	bl	mas_blk_busyidle_check_bio
	ldr	x21, [x21]
	cbnz	x21, .LBB9_2
// %bb.3:
	add	x8, x20, #816           // =816
	b	.LBB9_10
.LBB9_4:
	ldr	x0, [x20]
	bl	mas_blk_get_lld
	add	x22, x0, #1368          // =1368
	mov	x21, x0
	mov	x0, x22
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x21, #1328]
	ldr	x25, [x21, #1360]
	add	x24, x20, #720          // =720
	add	x26, x21, #1352         // =1352
	mov	x23, x0
	add	x8, x8, #1              // =1
	mov	x0, x24
	mov	x1, x25
	mov	x2, x26
	str	x8, [x21, #1328]
	bl	__list_add_valid
	tbz	w0, #0, .LBB9_6
// %bb.5:
	str	x24, [x21, #1360]
	str	x26, [x20, #720]
	str	x25, [x20, #728]
	str	x24, [x25]
.LBB9_6:
	mov	x0, x22
	mov	x1, x23
	bl	_raw_spin_unlock_irqrestore
	ldrb	w8, [x21, #24]
	tbz	w8, #3, .LBB9_9
// %bb.7:
	ldr	w8, [x21, #1088]
	cbz	w8, .LBB9_11
.LBB9_8:
	add	x8, x21, #1088          // =1088
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB9_9:
	adrp	x8, __cfi_mas_blk_busyidle_end_rq
	add	x8, x8, :lo12:__cfi_mas_blk_busyidle_end_rq
	str	x8, [x20, #816]
	add	x8, x20, #280           // =280
.LBB9_10:
	str	x19, [x8]
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB9_11:
	add	x22, x21, #1096         // =1096
	mov	x0, x22
	bl	mutex_lock
	add	x0, x21, #872           // =872
	bl	cancel_delayed_work
	ldr	w8, [x21, #1128]
	cmp	w8, #1                  // =1
	b.ne	.LBB9_13
// %bb.12:
	add	x0, x21, #1032          // =1032
	mov	x1, xzr
	mov	x2, xzr
	str	wzr, [x21, #1128]
	bl	blocking_notifier_call_chain
.LBB9_13:
	mov	x0, x22
	bl	mutex_unlock
	b	.LBB9_8
.Lfunc_end9:
	.size	mas_blk_busyidle_check_execute_request, .Lfunc_end9-mas_blk_busyidle_check_execute_request
                                        // -- End function
	.globl	mas_blk_busyidle_check_bio_endio // -- Begin function mas_blk_busyidle_check_bio_endio
	.p2align	2
	.type	mas_blk_busyidle_check_bio_endio,@function
mas_blk_busyidle_check_bio_endio:       // @mas_blk_busyidle_check_bio_endio
.Lmas_blk_busyidle_check_bio_endio$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #8]
	add	x9, x0, #88             // =88
	mov	x19, x0
	str	x23, [sp, #16]          // 8-byte Folded Spill
	add	x10, x8, #1264          // =1264
	cmp	x8, #0                  // =0
	csel	x8, x9, x10, eq
	ldr	x0, [x8]
	mov	x29, sp
	bl	mas_blk_get_lld
	ldrb	w8, [x0, #25]
	mov	x20, x0
	tbz	w8, #1, .LBB10_6
// %bb.1:
	ldrb	w8, [x19, #290]
	cbz	w8, .LBB10_6
// %bb.2:
	ldrb	w8, [x19, #136]
	tbz	w8, #5, .LBB10_6
// %bb.3:
	and	w8, w8, #0xffffffdf
	strb	w8, [x19, #136]
	ldr	w8, [x20, #24]
	mov	w9, #520
	mvn	w8, w8
	tst	x8, x9
	b.ne	.LBB10_6
// %bb.4:
	ldr	w8, [x20, #1276]
	cbz	w8, .LBB10_13
// %bb.5:
	add	x8, x20, #1276          // =1276
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
	cbz	w10, .LBB10_14
.LBB10_6:
	ldrb	w8, [x19, #136]
	tbz	w8, #0, .LBB10_12
// %bb.7:
	ldr	x8, [x20, #1320]
	cbz	x8, .LBB10_11
// %bb.8:
	add	x21, x20, #1368         // =1368
	mov	x0, x21
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x20, #1320]
	add	x23, x19, #328          // =328
	mov	x22, x0
	mov	x0, x23
	sub	x8, x8, #1              // =1
	str	x8, [x20, #1320]
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB10_10
// %bb.9:
	ldp	x9, x8, [x19, #328]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB10_10:
	mov	x0, x21
	mov	x1, x22
	str	x23, [x19, #328]
	str	x23, [x19, #336]
	bl	_raw_spin_unlock_irqrestore
.LBB10_11:
	mov	x0, x20
	strb	wzr, [x19, #136]
	bl	blk_idle_count
.LBB10_12:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB10_13:
	adrp	x0, .L.str.11
	adrp	x1, .L__func__.blk_fg_io_idle_count
	add	x0, x0, :lo12:.L.str.11
	add	x1, x1, :lo12:.L__func__.blk_fg_io_idle_count
	bl	printk
	b	.LBB10_6
.LBB10_14:
	ldr	w8, [x20]
	cmp	w8, #122                // =122
	b.ne	.LBB10_6
// %bb.15:
	adrp	x8, blk_fg_io_idle_notify_workqueue
	ldr	w0, [x20, #1272]
	ldr	x21, [x8, :lo12:blk_fg_io_idle_notify_workqueue]
	add	x22, x20, #1136         // =1136
	bl	__msecs_to_jiffies
	mov	x3, x0
	mov	w0, #8
	mov	x1, x21
	mov	x2, x22
	bl	queue_delayed_work_on
	b	.LBB10_6
.Lfunc_end10:
	.size	mas_blk_busyidle_check_bio_endio, .Lfunc_end10-mas_blk_busyidle_check_bio_endio
                                        // -- End function
	.globl	blk_busyidle_event_subscribe // -- Begin function blk_busyidle_event_subscribe
	.p2align	2
	.type	blk_busyidle_event_subscribe,@function
blk_busyidle_event_subscribe:           // @blk_busyidle_event_subscribe
.Lblk_busyidle_event_subscribe$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	mov	x8, x0
	mov	w0, #-22
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	cbz	x8, .LBB11_5
// %bb.1:
	mov	x19, x1
	cbz	x1, .LBB11_5
// %bb.2:
	ldr	x8, [x8, #128]
	ldr	x0, [x8, #1264]
	cbz	x0, .LBB11_4
// %bb.3:
	bl	mas_blk_get_lld
	mov	x1, x19
	bl	blk_lld_busyidle_event_subscribe
	b	.LBB11_5
.LBB11_4:
	mov	w0, #-22
.LBB11_5:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end11:
	.size	blk_busyidle_event_subscribe, .Lfunc_end11-blk_busyidle_event_subscribe
                                        // -- End function
	.globl	blk_queue_busyidle_event_subscribe // -- Begin function blk_queue_busyidle_event_subscribe
	.p2align	2
	.type	blk_queue_busyidle_event_subscribe,@function
blk_queue_busyidle_event_subscribe:     // @blk_queue_busyidle_event_subscribe
.Lblk_queue_busyidle_event_subscribe$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	mov	x8, x0
	mov	w0, #-22
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	cbz	x8, .LBB12_3
// %bb.1:
	mov	x19, x1
	cbz	x1, .LBB12_3
// %bb.2:
	mov	x0, x8
	bl	mas_blk_get_lld
	mov	x1, x19
	bl	blk_lld_busyidle_event_subscribe
.LBB12_3:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end12:
	.size	blk_queue_busyidle_event_subscribe, .Lfunc_end12-blk_queue_busyidle_event_subscribe
                                        // -- End function
	.globl	blk_lld_busyidle_event_subscribe // -- Begin function blk_lld_busyidle_event_subscribe
	.p2align	2
	.type	blk_lld_busyidle_event_subscribe,@function
blk_lld_busyidle_event_subscribe:       // @blk_lld_busyidle_event_subscribe
.Lblk_lld_busyidle_event_subscribe$local:
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	mov	w22, #-22
	str	x25, [sp, #16]          // 8-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB13_26
// %bb.1:
	mov	x21, x1
	cbz	x1, .LBB13_26
// %bb.2:
	str	x0, [x21, #48]
	adrp	x8, kmalloc_caches+64
	mov	x20, x0
	ldr	x0, [x8, :lo12:kmalloc_caches+64]
	mov	w1, #3520
	mov	w2, #160
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB13_20
// %bb.3:
	add	x22, x0, #96            // =96
	mov	x19, x0
	mov	w2, #56
	mov	x0, x22
	mov	x1, x21
	bl	memcpy
	ldr	x8, [x19, #128]
	cbz	x8, .LBB13_23
// %bb.4:
	mov	x0, x22
	bl	strlen
	cbz	x0, .LBB13_21
// %bb.5:
	mov	x0, x22
	bl	strlen
	cmp	x0, #32                 // =32
	b.hs	.LBB13_21
// %bb.6:
	add	x21, x20, #1096         // =1096
	mov	x0, x21
	bl	mutex_lock
	ldrb	w8, [x20, #25]
	tbz	w8, #1, .LBB13_8
// %bb.7:
	add	x0, x20, #1280          // =1280
	bl	mutex_lock
.LBB13_8:
	ldr	x24, [x20, #1016]
	add	x23, x20, #1016         // =1016
	b	.LBB13_10
.LBB13_9:                               //   in Loop: Header=BB13_10 Depth=1
	ldr	x24, [x24]
.LBB13_10:                              // =>This Inner Loop Header: Depth=1
	cmp	x24, x23
	b.eq	.LBB13_14
// %bb.11:                              //   in Loop: Header=BB13_10 Depth=1
	add	x1, x24, #96            // =96
	mov	x0, x22
	bl	strcmp
	cbnz	w0, .LBB13_9
// %bb.12:                              //   in Loop: Header=BB13_10 Depth=1
	ldr	x8, [x19, #128]
	ldr	x9, [x24, #128]
	cmp	x8, x9
	b.ne	.LBB13_9
// %bb.13:
	cbnz	x24, .LBB13_22
.LBB13_14:
	adrp	x1, __cfi_mas_blk_busyidle_handler_latency_check_timer_expire
	add	x0, x19, #16            // =16
	add	x1, x1, :lo12:__cfi_mas_blk_busyidle_handler_latency_check_timer_expire
	mov	w2, wzr
	mov	x3, xzr
	mov	x4, xzr
	bl	init_timer_key
	adrp	x8, __cfi_mas_blk_busyidle_notify_handler
	add	x8, x8, :lo12:__cfi_mas_blk_busyidle_notify_handler
	mov	x24, x19
	mov	x9, #4294967297
	str	x8, [x24, #72]!
	str	wzr, [x19, #88]
	str	x9, [x19, #152]
	ldr	x25, [x20, #1024]
	mov	x0, x19
	mov	x2, x23
	mov	x1, x25
	bl	__list_add_valid
	tbz	w0, #0, .LBB13_16
// %bb.15:
	str	x19, [x20, #1024]
	stp	x23, x25, [x19]
	str	x19, [x25]
.LBB13_16:
	add	x0, x20, #1032          // =1032
	mov	x1, x24
	bl	blocking_notifier_chain_register
	adrp	x0, .L.str.13
	add	x0, x0, :lo12:.L.str.13
	mov	x1, x22
	bl	printk
	mov	w22, wzr
	ldrb	w8, [x20, #25]
	tbz	w8, #1, .LBB13_18
.LBB13_17:
	add	x0, x20, #1280          // =1280
	bl	mutex_unlock
.LBB13_18:
	mov	x0, x21
	bl	mutex_unlock
	cbz	w22, .LBB13_26
// %bb.19:
	mov	w22, #-1
	b	.LBB13_25
.LBB13_20:
	mov	w22, #-12
	b	.LBB13_26
.LBB13_21:
	adrp	x0, .L.str.15
	add	x0, x0, :lo12:.L.str.15
	b	.LBB13_24
.LBB13_22:
	adrp	x0, .L.str.12
	adrp	x1, .L__func__.mas_blk_busyidle_event_register
	add	x0, x0, :lo12:.L.str.12
	add	x1, x1, :lo12:.L__func__.mas_blk_busyidle_event_register
	mov	x2, x22
	bl	printk
	bl	dump_stack
	mov	w22, #-1
	ldrb	w8, [x20, #25]
	tbnz	w8, #1, .LBB13_17
	b	.LBB13_18
.LBB13_23:
	adrp	x0, .L.str.14
	add	x0, x0, :lo12:.L.str.14
.LBB13_24:
	adrp	x1, .L__func__.is_valid_event_node
	add	x1, x1, :lo12:.L__func__.is_valid_event_node
	bl	printk
	bl	dump_stack
	mov	w22, #-22
.LBB13_25:
	mov	x0, x19
	bl	kfree
.LBB13_26:
	mov	w0, w22
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.Lfunc_end13:
	.size	blk_lld_busyidle_event_subscribe, .Lfunc_end13-blk_lld_busyidle_event_subscribe
                                        // -- End function
	.globl	blk_busyidle_event_unsubscribe // -- Begin function blk_busyidle_event_unsubscribe
	.p2align	2
	.type	blk_busyidle_event_unsubscribe,@function
blk_busyidle_event_unsubscribe:         // @blk_busyidle_event_unsubscribe
.Lblk_busyidle_event_unsubscribe$local:
// %bb.0:
	cbz	x0, .LBB14_2
// %bb.1:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x29, sp
	bl	mas_blk_busyidle_event_unregister
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB14_2:
	mov	w0, #-22
	ret
.Lfunc_end14:
	.size	blk_busyidle_event_unsubscribe, .Lfunc_end14-blk_busyidle_event_unsubscribe
                                        // -- End function
	.p2align	2               // -- Begin function mas_blk_busyidle_event_unregister
	.type	mas_blk_busyidle_event_unregister,@function
mas_blk_busyidle_event_unregister:      // @mas_blk_busyidle_event_unregister
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x23, [x0, #48]
	mov	x21, x0
	mov	x29, sp
	add	x19, x23, #1096         // =1096
	mov	x0, x19
	bl	mutex_lock
	ldrb	w8, [x23, #25]
	tbz	w8, #1, .LBB15_2
// %bb.1:
	add	x0, x23, #1280          // =1280
	bl	mutex_lock
.LBB15_2:
	ldr	x20, [x23, #1016]
	add	x24, x23, #1016         // =1016
	b	.LBB15_4
.LBB15_3:                               //   in Loop: Header=BB15_4 Depth=1
	ldr	x20, [x20]
.LBB15_4:                               // =>This Inner Loop Header: Depth=1
	cmp	x20, x24
	b.eq	.LBB15_8
// %bb.5:                               //   in Loop: Header=BB15_4 Depth=1
	add	x22, x20, #96           // =96
	mov	x0, x21
	mov	x1, x22
	bl	strcmp
	cbnz	w0, .LBB15_3
// %bb.6:                               //   in Loop: Header=BB15_4 Depth=1
	ldr	x8, [x21, #32]
	ldr	x9, [x20, #128]
	cmp	x8, x9
	b.ne	.LBB15_3
// %bb.7:
	cbnz	x20, .LBB15_11
.LBB15_8:
	adrp	x0, .L.str.16
	add	x0, x0, :lo12:.L.str.16
	bl	printk
	bl	dump_stack
	mov	w20, #-22
	ldrb	w8, [x23, #25]
	tbz	w8, #1, .LBB15_10
.LBB15_9:
	add	x0, x23, #1280          // =1280
	bl	mutex_unlock
.LBB15_10:
	mov	x0, x19
	bl	mutex_unlock
	mov	w0, w20
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB15_11:
	add	x21, x20, #72           // =72
	add	x0, x23, #1032          // =1032
	mov	x1, x21
	bl	blocking_notifier_chain_unregister
	adrp	x0, .L.str.17
	add	x0, x0, :lo12:.L.str.17
	mov	x1, x22
	mov	x2, x21
	bl	printk
	mov	x0, x20
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB15_13
// %bb.12:
	ldp	x9, x8, [x20]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB15_13:
	mov	x0, x20
	str	x20, [x20]
	str	x20, [x20, #8]
	bl	kfree
	mov	w20, wzr
	ldrb	w8, [x23, #25]
	tbnz	w8, #1, .LBB15_9
	b	.LBB15_10
.Lfunc_end15:
	.size	mas_blk_busyidle_event_unregister, .Lfunc_end15-mas_blk_busyidle_event_unregister
                                        // -- End function
	.globl	blk_queue_busyidle_event_unsubscribe // -- Begin function blk_queue_busyidle_event_unsubscribe
	.p2align	2
	.type	blk_queue_busyidle_event_unsubscribe,@function
blk_queue_busyidle_event_unsubscribe:   // @blk_queue_busyidle_event_unsubscribe
.Lblk_queue_busyidle_event_unsubscribe$local:
// %bb.0:
	cbz	x0, .LBB16_2
// %bb.1:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x29, sp
	bl	mas_blk_busyidle_event_unregister
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB16_2:
	mov	w0, #-22
	ret
.Lfunc_end16:
	.size	blk_queue_busyidle_event_unsubscribe, .Lfunc_end16-blk_queue_busyidle_event_unsubscribe
                                        // -- End function
	.globl	blk_queue_busyidle_enable // -- Begin function blk_queue_busyidle_enable
	.p2align	2
	.type	blk_queue_busyidle_enable,@function
blk_queue_busyidle_enable:              // @blk_queue_busyidle_enable
.Lblk_queue_busyidle_enable$local:
// %bb.0:
	cbz	x0, .LBB17_2
// %bb.1:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	w19, w1
	bl	mas_blk_get_lld
	ldr	x8, [x0, #24]
	cmp	w19, #0                 // =0
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	and	x9, x8, #0xfffffffffffffff7
	orr	x8, x8, #0x8
	csel	x8, x9, x8, eq
	str	x8, [x0, #24]
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
.LBB17_2:
	ret
.Lfunc_end17:
	.size	blk_queue_busyidle_enable, .Lfunc_end17-blk_queue_busyidle_enable
                                        // -- End function
	.globl	blk_mq_tagset_busyidle_enable // -- Begin function blk_mq_tagset_busyidle_enable
	.p2align	2
	.type	blk_mq_tagset_busyidle_enable,@function
blk_mq_tagset_busyidle_enable:          // @blk_mq_tagset_busyidle_enable
.Lblk_mq_tagset_busyidle_enable$local:
// %bb.0:
	cbz	x0, .LBB18_2
// %bb.1:
	ldr	x8, [x0, #88]
	cmp	w1, #0                  // =0
	and	x9, x8, #0xfffffffffffffff7
	orr	x8, x8, #0x8
	csel	x8, x9, x8, eq
	str	x8, [x0, #88]
.LBB18_2:
	ret
.Lfunc_end18:
	.size	blk_mq_tagset_busyidle_enable, .Lfunc_end18-blk_mq_tagset_busyidle_enable
                                        // -- End function
	.globl	blk_mq_tagset_hw_idle_notify_enable // -- Begin function blk_mq_tagset_hw_idle_notify_enable
	.p2align	2
	.type	blk_mq_tagset_hw_idle_notify_enable,@function
blk_mq_tagset_hw_idle_notify_enable:    // @blk_mq_tagset_hw_idle_notify_enable
.Lblk_mq_tagset_hw_idle_notify_enable$local:
// %bb.0:
	cbz	x0, .LBB19_2
// %bb.1:
	cmp	w1, #0                  // =0
	cset	w8, ne
	strb	w8, [x0, #928]
.LBB19_2:
	ret
.Lfunc_end19:
	.size	blk_mq_tagset_hw_idle_notify_enable, .Lfunc_end19-blk_mq_tagset_hw_idle_notify_enable
                                        // -- End function
	.globl	mas_blk_busyidle_state_init // -- Begin function mas_blk_busyidle_state_init
	.p2align	2
	.type	mas_blk_busyidle_state_init,@function
mas_blk_busyidle_state_init:            // @mas_blk_busyidle_state_init
.Lmas_blk_busyidle_state_init$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	adrp	x20, blk_idle_notify_workqueue
	ldr	x8, [x20, :lo12:blk_idle_notify_workqueue]
	mov	x19, x0
	str	x23, [sp, #16]          // 8-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	cbnz	x8, .LBB20_2
// %bb.1:
	adrp	x0, .L.str.4
	add	x0, x0, :lo12:.L.str.4
	mov	w1, #24
	mov	w2, wzr
	bl	alloc_workqueue
	str	x0, [x20, :lo12:blk_idle_notify_workqueue]
	cbz	x0, .LBB20_6
.LBB20_2:
	adrp	x20, blk_fg_io_idle_notify_workqueue
	ldr	x8, [x20, :lo12:blk_fg_io_idle_notify_workqueue]
	cbnz	x8, .LBB20_4
// %bb.3:
	adrp	x0, .L.str.18
	add	x0, x0, :lo12:.L.str.18
	mov	w1, #24
	mov	w2, wzr
	bl	alloc_workqueue
	str	x0, [x20, :lo12:blk_fg_io_idle_notify_workqueue]
	cbz	x0, .LBB20_7
.LBB20_4:
	adrp	x20, delayed_work_timer_fn
	adrp	x9, __cfi_mas_blk_idle_notify_work
	add	x20, x20, :lo12:delayed_work_timer_fn
	mov	w21, #5
	mov	x22, #68719476704
	add	x8, x19, #16            // =16
	add	x9, x9, :lo12:__cfi_mas_blk_idle_notify_work
	add	x0, x19, #56            // =56
	mov	w2, #2097152
	mov	x1, x20
	mov	x3, xzr
	mov	x4, xzr
	str	w21, [x19, #144]
	str	x22, [x19, #8]
	str	x8, [x19, #16]
	stp	x8, x9, [x19, #24]
	bl	init_timer_key
	adrp	x1, .L.str.6
	adrp	x2, mas_blk_busyidle_state_init.__key
	add	x0, x19, #232           // =232
	add	x1, x1, :lo12:.L.str.6
	add	x2, x2, :lo12:mas_blk_busyidle_state_init.__key
	str	wzr, [x19, #224]
	bl	__mutex_init
	adrp	x1, .L.str.8
	adrp	x2, mas_blk_busyidle_state_init.__key.7
	add	x8, x19, #152           // =152
	add	x0, x19, #168           // =168
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:mas_blk_busyidle_state_init.__key.7
	str	x8, [x19, #152]
	str	x8, [x19, #160]
	bl	__init_rwsem
	adrp	x9, mas_blk_fg_io_idle_notify_work
	mov	w23, #1
	add	x8, x19, #280           // =280
	add	x9, x9, :lo12:mas_blk_fg_io_idle_notify_work
	add	x0, x19, #320           // =320
	mov	w2, #2097152
	mov	x1, x20
	mov	x3, xzr
	mov	x4, xzr
	str	xzr, [x19, #216]
	str	w21, [x19, #408]
	str	x22, [x19, #272]
	str	w23, [x19, #264]
	str	x8, [x19, #280]
	stp	x8, x9, [x19, #288]
	bl	init_timer_key
	adrp	x1, .L.str.19
	adrp	x2, mas_blk_fg_io_busyidle_state_init.__key
	add	x0, x19, #416           // =416
	add	x1, x1, :lo12:.L.str.19
	add	x2, x2, :lo12:mas_blk_fg_io_busyidle_state_init.__key
	str	wzr, [x19, #412]
	bl	__mutex_init
	mov	w0, wzr
	add	x8, x19, #472           // =472
	add	x9, x19, #488           // =488
	str	w23, [x19, #448]
	stp	xzr, xzr, [x19, #456]
	str	x8, [x19, #472]
	str	x8, [x19, #480]
	str	x9, [x19, #488]
	str	x9, [x19, #496]
	str	wzr, [x19, #504]
.LBB20_5:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB20_6:
	adrp	x0, .L.str.5
	adrp	x1, .L__func__.mas_blk_busyidle_state_init
	add	x0, x0, :lo12:.L.str.5
	add	x1, x1, :lo12:.L__func__.mas_blk_busyidle_state_init
	b	.LBB20_8
.LBB20_7:
	adrp	x0, .L.str.5
	adrp	x1, .L__func__.mas_blk_alloc_fg_io_workqueue
	add	x0, x0, :lo12:.L.str.5
	add	x1, x1, :lo12:.L__func__.mas_blk_alloc_fg_io_workqueue
.LBB20_8:
	bl	printk
	mov	w0, #-12
	b	.LBB20_5
.Lfunc_end20:
	.size	mas_blk_busyidle_state_init, .Lfunc_end20-mas_blk_busyidle_state_init
                                        // -- End function
	.p2align	2               // -- Begin function mas_blk_fg_io_idle_notify_work
	.type	mas_blk_fg_io_idle_notify_work,@function
mas_blk_fg_io_idle_notify_work:         // @mas_blk_fg_io_idle_notify_work
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x21, x0
	sub	x0, x0, #1136           // =1136
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	bl	mas_blk_get_queue_by_lld
	add	x19, x21, #144          // =144
	mov	x20, x0
	mov	x0, x19
	bl	mutex_lock
	ldr	w8, [x21, #140]
	cbnz	w8, .LBB21_3
// %bb.1:
	sub	x8, x21, #272           // =272
	ldr	w9, [x8, #448]
	cmp	w9, #1                  // =1
	b.eq	.LBB21_3
// %bb.2:
	mov	w9, #1
	add	x0, x8, #168            // =168
	mov	w1, #3
	mov	x2, x20
	str	w9, [x8, #448]
	bl	blocking_notifier_call_chain
.LBB21_3:
	mov	x0, x19
	bl	mutex_unlock
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end21:
	.size	mas_blk_fg_io_idle_notify_work, .Lfunc_end21-mas_blk_fg_io_idle_notify_work
                                        // -- End function
	.type	.L.str,@object          // @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"\0013[BLK-IO]%s: %s process time is more than %u ms\n"
	.size	.L.str, 50

	.type	.L__func__.mas_blk_busyidle_handler_latency_check_timer_expire,@object // @__func__.mas_blk_busyidle_handler_latency_check_timer_expire
.L__func__.mas_blk_busyidle_handler_latency_check_timer_expire:
	.asciz	"mas_blk_busyidle_handler_latency_check_timer_expire"
	.size	.L__func__.mas_blk_busyidle_handler_latency_check_timer_expire, 52

	.type	.L.str.1,@object        // @.str.1
.L.str.1:
	.asciz	"\0013[BLK-IO]%s: %s NULL callback\n"
	.size	.L.str.1, 32

	.type	.L__func__.mas_blk_busyidle_notify_handler,@object // @__func__.mas_blk_busyidle_notify_handler
.L__func__.mas_blk_busyidle_notify_handler:
	.asciz	"mas_blk_busyidle_notify_handler"
	.size	.L__func__.mas_blk_busyidle_notify_handler, 32

	.type	.L.str.2,@object        // @.str.2
.L.str.2:
	.asciz	"\0013[BLK-IO]%s: %s error!\n"
	.size	.L.str.2, 25

	.type	.L.str.3,@object        // @.str.3
.L.str.3:
	.asciz	"\0013[BLK-IO]%s: %s busy idle callback cost %lld ms\n"
	.size	.L.str.3, 50

	.type	blk_fg_io_idle_notify_workqueue,@object // @blk_fg_io_idle_notify_workqueue
	.local	blk_fg_io_idle_notify_workqueue
	.comm	blk_fg_io_idle_notify_workqueue,8,8
	.type	blk_idle_notify_workqueue,@object // @blk_idle_notify_workqueue
	.local	blk_idle_notify_workqueue
	.comm	blk_idle_notify_workqueue,8,8
	.type	.L.str.4,@object        // @.str.4
.L.str.4:
	.asciz	"busyidle_notify"
	.size	.L.str.4, 16

	.type	.L.str.5,@object        // @.str.5
.L.str.5:
	.asciz	"\0013[BLK-IO]%s: Failed to alloc workqueue!\n"
	.size	.L.str.5, 42

	.type	.L__func__.mas_blk_busyidle_state_init,@object // @__func__.mas_blk_busyidle_state_init
.L__func__.mas_blk_busyidle_state_init:
	.asciz	"mas_blk_busyidle_state_init"
	.size	.L__func__.mas_blk_busyidle_state_init, 28

	.type	mas_blk_busyidle_state_init.__key,@object // @mas_blk_busyidle_state_init.__key
	.local	mas_blk_busyidle_state_init.__key
	.comm	mas_blk_busyidle_state_init.__key,1,1
	.type	.L.str.6,@object        // @.str.6
.L.str.6:
	.asciz	"&blk_idle->io_count_mutex"
	.size	.L.str.6, 26

	.type	mas_blk_busyidle_state_init.__key.7,@object // @mas_blk_busyidle_state_init.__key.7
	.local	mas_blk_busyidle_state_init.__key.7
	.comm	mas_blk_busyidle_state_init.__key.7,1,1
	.type	.L.str.8,@object        // @.str.8
.L.str.8:
	.asciz	"&(&blk_idle->nh)->rwsem"
	.size	.L.str.8, 24

	.type	.L.str.10,@object       // @.str.10
.L.str.10:
	.asciz	"\0013[BLK-IO]%s: io_count has been zero\n"
	.size	.L.str.10, 38

	.type	.L__func__.blk_idle_count,@object // @__func__.blk_idle_count
.L__func__.blk_idle_count:
	.asciz	"blk_idle_count"
	.size	.L__func__.blk_idle_count, 15

	.type	.L.str.11,@object       // @.str.11
.L.str.11:
	.asciz	"\0013[BLK-IO]<%s> io_count has been zero\n"
	.size	.L.str.11, 39

	.type	.L__func__.blk_fg_io_idle_count,@object // @__func__.blk_fg_io_idle_count
.L__func__.blk_fg_io_idle_count:
	.asciz	"blk_fg_io_idle_count"
	.size	.L__func__.blk_fg_io_idle_count, 21

	.type	.L.str.12,@object       // @.str.12
.L.str.12:
	.asciz	"\0013[BLK-IO]%s: %s has been registered already!\n"
	.size	.L.str.12, 47

	.type	.L__func__.mas_blk_busyidle_event_register,@object // @__func__.mas_blk_busyidle_event_register
.L__func__.mas_blk_busyidle_event_register:
	.asciz	"mas_blk_busyidle_event_register"
	.size	.L__func__.mas_blk_busyidle_event_register, 32

	.type	.L.str.13,@object       // @.str.13
.L.str.13:
	.asciz	"\0013[BLK-IO]%s registered to busy idle\n"
	.size	.L.str.13, 38

	.type	.L.str.14,@object       // @.str.14
.L.str.14:
	.asciz	"\0010[BLK-IO]%s: notifier_callback is NULL\n"
	.size	.L.str.14, 41

	.type	.L__func__.is_valid_event_node,@object // @__func__.is_valid_event_node
.L__func__.is_valid_event_node:
	.asciz	"is_valid_event_node"
	.size	.L__func__.is_valid_event_node, 20

	.type	.L.str.15,@object       // @.str.15
.L.str.15:
	.asciz	"\0010[BLK-IO]%s: Invalid subscriber\n"
	.size	.L.str.15, 34

	.type	.L.str.16,@object       // @.str.16
.L.str.16:
	.asciz	"\0013[BLK-IO]Trying to unregister an unregistered event_node!\n"
	.size	.L.str.16, 60

	.type	.L.str.17,@object       // @.str.17
.L.str.17:
	.asciz	"\0013[BLK-IO]%s unregistered from busy idle module 0x%pK\n"
	.size	.L.str.17, 55

	.type	.L.str.18,@object       // @.str.18
.L.str.18:
	.asciz	"fg_busy_idle_notify"
	.size	.L.str.18, 20

	.type	.L__func__.mas_blk_alloc_fg_io_workqueue,@object // @__func__.mas_blk_alloc_fg_io_workqueue
.L__func__.mas_blk_alloc_fg_io_workqueue:
	.asciz	"mas_blk_alloc_fg_io_workqueue"
	.size	.L__func__.mas_blk_alloc_fg_io_workqueue, 30

	.type	mas_blk_fg_io_busyidle_state_init.__key,@object // @mas_blk_fg_io_busyidle_state_init.__key
	.local	mas_blk_fg_io_busyidle_state_init.__key
	.comm	mas_blk_fg_io_busyidle_state_init.__key,1,1
	.type	.L.str.19,@object       // @.str.19
.L.str.19:
	.asciz	"&blk_idle->fg_io_count_mutex"
	.size	.L.str.19, 29

	.ident	"Android (6443078 based on r383902) clang version 11.0.1 (https://android.googlesource.com/toolchain/llvm-project b397f81060ce6d701042b782172ed13bee898b79)"
	.section	".note.GNU-stack","",@progbits
