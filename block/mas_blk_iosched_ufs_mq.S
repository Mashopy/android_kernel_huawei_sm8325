	.text
	.file	"mas_blk_iosched_ufs_mq.c"
	.globl	ufs_mq_inc_vip_wait_cnt // -- Begin function ufs_mq_inc_vip_wait_cnt
	.p2align	2
	.type	ufs_mq_inc_vip_wait_cnt,@function
ufs_mq_inc_vip_wait_cnt:                // @ufs_mq_inc_vip_wait_cnt
.Lufs_mq_inc_vip_wait_cnt$local:
// %bb.0:
	ldr	x8, [x0]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB0_3
// %bb.1:
	ldr	x8, [x8, #536]
	cbz	x8, .LBB0_3
// %bb.2:
	add	x8, x8, #28             // =28
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB0_3:
	ret
.Lfunc_end0:
	.size	ufs_mq_inc_vip_wait_cnt, .Lfunc_end0-ufs_mq_inc_vip_wait_cnt
                                        // -- End function
	.globl	ufs_mq_dec_vip_wait_cnt // -- Begin function ufs_mq_dec_vip_wait_cnt
	.p2align	2
	.type	ufs_mq_dec_vip_wait_cnt,@function
ufs_mq_dec_vip_wait_cnt:                // @ufs_mq_dec_vip_wait_cnt
.Lufs_mq_dec_vip_wait_cnt$local:
// %bb.0:
	ldr	x8, [x0]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB1_2
// %bb.1:
	ldr	x8, [x8, #536]
.LBB1_2:
	ldr	w9, [x8, #28]
.LBB1_3:                                // =>This Inner Loop Header: Depth=1
	subs	w10, w9, #1             // =1
	b.mi	.LBB1_5
// %bb.4:                               //   in Loop: Header=BB1_3 Depth=1
	mov	w11, w9
	add	x12, x8, #28            // =28
	//APP
		prfm	pstl1strm, [x12]
1:	ldxr	w13, [x12]
	eor	w14, w13, w11
	cbnz	w14, 2f
	stlxr	w14, w10, [x12]
	cbnz	w14, 1b
	dmb ish
2:
	//NO_APP
	cmp	w13, w9
	mov	w9, w13
	b.ne	.LBB1_3
.LBB1_5:
	ret
.Lfunc_end1:
	.size	ufs_mq_dec_vip_wait_cnt, .Lfunc_end1-ufs_mq_dec_vip_wait_cnt
                                        // -- End function
	.globl	reset_vip_wait_cnt      // -- Begin function reset_vip_wait_cnt
	.p2align	2
	.type	reset_vip_wait_cnt,@function
reset_vip_wait_cnt:                     // @reset_vip_wait_cnt
.Lreset_vip_wait_cnt$local:
// %bb.0:
	ldr	x8, [x0]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB2_2
// %bb.1:
	ldr	x8, [x8, #536]
.LBB2_2:
	ldr	w9, [x8, #24]
	cbz	w9, .LBB2_4
// %bb.3:
	ret
.LBB2_4:
	str	wzr, [x8, #28]
	ret
.Lfunc_end2:
	.size	reset_vip_wait_cnt, .Lfunc_end2-reset_vip_wait_cnt
                                        // -- End function
	.globl	ufs_mq_vip_tag_wait_cnt // -- Begin function ufs_mq_vip_tag_wait_cnt
	.p2align	2
	.type	ufs_mq_vip_tag_wait_cnt,@function
ufs_mq_vip_tag_wait_cnt:                // @ufs_mq_vip_tag_wait_cnt
.Lufs_mq_vip_tag_wait_cnt$local:
// %bb.0:
	ldr	x8, [x0]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB3_2
// %bb.1:
	ldr	x8, [x8, #536]
.LBB3_2:
	ldr	w0, [x8, #28]
	ret
.Lfunc_end3:
	.size	ufs_mq_vip_tag_wait_cnt, .Lfunc_end3-ufs_mq_vip_tag_wait_cnt
                                        // -- End function
	.globl	mas_blk_latency_req_check_ufs // -- Begin function mas_blk_latency_req_check_ufs
	.p2align	2
	.type	mas_blk_latency_req_check_ufs,@function
mas_blk_latency_req_check_ufs:          // @mas_blk_latency_req_check_ufs
.Lmas_blk_latency_req_check_ufs$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x29, sp
	mov	w20, w1
	mov	x19, x0
	bl	mas_blk_latency_req_check
	cmp	w20, #19                // =19
	b.ne	.LBB4_2
// %bb.1:
	ldr	x8, [x19, #736]
	add	x8, x8, #1              // =1
	str	x8, [x19, #736]
.LBB4_2:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end4:
	.size	mas_blk_latency_req_check_ufs, .Lfunc_end4-mas_blk_latency_req_check_ufs
                                        // -- End function
	.globl	mas_blk_queue_get_budget // -- Begin function mas_blk_queue_get_budget
	.p2align	2
	.type	mas_blk_queue_get_budget,@function
mas_blk_queue_get_budget:               // @mas_blk_queue_get_budget
.Lmas_blk_queue_get_budget$local:
// %bb.0:
	add	x8, x0, #1232           // =1232
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ret
.Lfunc_end5:
	.size	mas_blk_queue_get_budget, .Lfunc_end5-mas_blk_queue_get_budget
                                        // -- End function
	.globl	mas_blk_queue_put_budget // -- Begin function mas_blk_queue_put_budget
	.p2align	2
	.type	mas_blk_queue_put_budget,@function
mas_blk_queue_put_budget:               // @mas_blk_queue_put_budget
.Lmas_blk_queue_put_budget$local:
// %bb.0:
	add	x8, x0, #1236           // =1236
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ret
.Lfunc_end6:
	.size	mas_blk_queue_put_budget, .Lfunc_end6-mas_blk_queue_put_budget
                                        // -- End function
	.globl	get_mq_all_tag_used     // -- Begin function get_mq_all_tag_used
	.p2align	2
	.type	get_mq_all_tag_used,@function
get_mq_all_tag_used:                    // @get_mq_all_tag_used
.Lget_mq_all_tag_used$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB7_4
// %bb.1:
	bl	mas_blk_get_lld
	cbz	x0, .LBB7_4
// %bb.2:
	ldr	x8, [x0, #1440]
	cbz	x8, .LBB7_5
// %bb.3:
	ldr	w9, [x8, #24]
	ldr	w10, [x8, #16]
	ldr	w8, [x8, #20]
	add	w9, w10, w9
	add	w0, w9, w8
.LBB7_4:
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB7_5:
	mov	w0, wzr
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end7:
	.size	get_mq_all_tag_used, .Lfunc_end7-get_mq_all_tag_used
                                        // -- End function
	.globl	get_mq_prio_tag_used    // -- Begin function get_mq_prio_tag_used
	.p2align	2
	.type	get_mq_prio_tag_used,@function
get_mq_prio_tag_used:                   // @get_mq_prio_tag_used
.Lget_mq_prio_tag_used$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB8_4
// %bb.1:
	bl	mas_blk_get_lld
	cbz	x0, .LBB8_4
// %bb.2:
	ldr	x8, [x0, #1440]
	cbz	x8, .LBB8_5
// %bb.3:
	ldr	w0, [x8, #24]
.LBB8_4:
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB8_5:
	mov	w0, wzr
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end8:
	.size	get_mq_prio_tag_used, .Lfunc_end8-get_mq_prio_tag_used
                                        // -- End function
	.globl	mas_blk_is_disorder_stuck // -- Begin function mas_blk_is_disorder_stuck
	.p2align	2
	.type	mas_blk_is_disorder_stuck,@function
mas_blk_is_disorder_stuck:              // @mas_blk_is_disorder_stuck
.Lmas_blk_is_disorder_stuck$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #8]
	mov	x20, x0
	mov	x29, sp
	ldr	x0, [x8, #1264]
	bl	mas_blk_get_lld
	cbz	x0, .LBB9_10
// %bb.1:
	ldr	x23, [x0, #1440]
	mov	x19, x0
	cbz	x23, .LBB9_11
// %bb.2:
	ldr	x8, [x20, #8]
	ldr	x0, [x8, #1264]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB9_8
// %bb.3:
	ldrb	w8, [x20, #16]
	tbz	w8, #0, .LBB9_8
// %bb.4:
	ldrb	w2, [x20, #264]
	sub	w8, w2, #1              // =1
	cmp	w8, #4                  // =4
	b.hs	.LBB9_15
// %bb.5:
	bl	__rcu_read_lock
	ldr	x0, [x20, #8]
	ldrb	w1, [x20, #27]
	bl	__disk_get_part
	cbz	x0, .LBB9_16
// %bb.6:
	ldr	x8, [x20, #32]
	ldr	x9, [x0]
	add	x8, x9, x8
	lsr	x21, x8, #3
	bl	__rcu_read_unlock
	ldrb	w8, [x20, #264]
	add	x24, x19, #320          // =320
	add	x8, x24, x8, lsl #2
	sub	x0, x8, #4              // =4
	bl	_raw_spin_lock_irqsave
	ldrb	w1, [x20, #264]
	mov	x22, x0
	mov	x0, x19
	mov	x2, x21
	bl	mas_blk_expected_lba_pu
	ldrb	w8, [x20, #264]
	mov	w21, w0
	mov	x1, x22
	add	x8, x24, x8, lsl #2
	sub	x0, x8, #4              // =4
	bl	_raw_spin_unlock_irqrestore
	tbnz	w21, #0, .LBB9_8
// %bb.7:
	ldr	w8, [x23, #16]
	cmp	w8, #120                // =120
	b.ge	.LBB9_13
.LBB9_8:
	mov	w0, wzr
.LBB9_9:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB9_10:
	adrp	x0, .L.str
	add	x0, x0, :lo12:.L.str
	b	.LBB9_12
.LBB9_11:
	adrp	x0, .L.str.1
	add	x0, x0, :lo12:.L.str.1
.LBB9_12:
	adrp	x1, .L__func__.mas_blk_is_disorder_stuck
	add	x1, x1, :lo12:.L__func__.mas_blk_is_disorder_stuck
	bl	printk
	b	.LBB9_14
.LBB9_13:
	add	x8, x19, #856           // =856
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB9_14:
	mov	w0, #1
	b	.LBB9_9
.LBB9_15:
	adrp	x0, .L.str.2
	adrp	x1, .L__func__.mas_blk_is_disorder_stuck
	add	x0, x0, :lo12:.L.str.2
	add	x1, x1, :lo12:.L__func__.mas_blk_is_disorder_stuck
	bl	printk
	b	.LBB9_14
.LBB9_16:
	adrp	x0, .L.str.3
	adrp	x1, .L__func__.mas_blk_is_disorder_stuck
	add	x0, x0, :lo12:.L.str.3
	add	x1, x1, :lo12:.L__func__.mas_blk_is_disorder_stuck
	bl	printk
	bl	__rcu_read_unlock
	b	.LBB9_14
.Lfunc_end9:
	.size	mas_blk_is_disorder_stuck, .Lfunc_end9-mas_blk_is_disorder_stuck
                                        // -- End function
	.globl	mas_blk_is_reserved_empty // -- Begin function mas_blk_is_reserved_empty
	.p2align	2
	.type	mas_blk_is_reserved_empty,@function
mas_blk_is_reserved_empty:              // @mas_blk_is_reserved_empty
.Lmas_blk_is_reserved_empty$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	ldr	x0, [x0]
	mov	x29, sp
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB10_2
// %bb.1:
	ldrb	w8, [x19, #13]
	tst	w8, #0x3
	b.eq	.LBB10_4
.LBB10_2:
	mov	w0, wzr
.LBB10_3:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB10_4:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB10_6
// %bb.5:
	ldr	x8, [x8, #536]
.LBB10_6:
	ldr	w8, [x8, #20]
	ldr	x9, [x19, #48]
	ldr	x9, [x9, #360]
	ldr	w9, [x9, #4]
	sub	w9, w9, #4              // =4
	cmp	w8, w9
	cset	w0, hs
	b	.LBB10_3
.Lfunc_end10:
	.size	mas_blk_is_reserved_empty, .Lfunc_end10-mas_blk_is_reserved_empty
                                        // -- End function
	.globl	ufs_mq_tag_get          // -- Begin function ufs_mq_tag_get
	.p2align	2
	.type	ufs_mq_tag_get,@function
ufs_mq_tag_get:                         // @ufs_mq_tag_get
.Lufs_mq_tag_get$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	ldr	w21, [x0, #36]
	ldr	x0, [x0]
	mov	w20, #4096
	mov	x29, sp
	movk	w20, #64, lsl #16
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB11_11
// %bb.1:
	ldr	w8, [x19, #12]
	tbnz	w8, #8, .LBB11_13
// %bb.2:
	tbnz	w8, #9, .LBB11_15
// %bb.3:
	ldr	w9, [x19, #36]
	tst	w9, w20
	b.ne	.LBB11_35
// %bb.4:
	tbnz	w8, #7, .LBB11_35
// %bb.5:
	add	w8, w20, #2048          // =2048
	and	w8, w9, w8
	cmp	w8, #2048               // =2048
	b.eq	.LBB11_12
// %bb.6:
	ldr	x2, [x19, #48]
	ldr	x8, [x2, #360]
	ldr	w9, [x8, #4]
	cbz	w9, .LBB11_67
// %bb.7:
	add	x1, x8, #104            // =104
	mov	x0, x19
	bl	ufs_tagset_bt_get
	cmn	w0, #1                  // =1
	b.eq	.LBB11_68
// %bb.8:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB11_10
// %bb.9:
	ldr	x8, [x8, #536]
.LBB11_10:
	add	x8, x8, #20             // =20
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB11_34
.LBB11_11:
	add	w8, w20, #2048          // =2048
	and	w8, w21, w8
	cmp	w8, #2048               // =2048
	b.ne	.LBB11_56
.LBB11_12:
	mov	x0, x19
	bl	ufs_tagset_get_tag
	b	.LBB11_68
.LBB11_13:
	ldr	x0, [x19]
	ldr	x8, [x0, #1376]
	cbz	x8, .LBB11_25
// %bb.14:
	ldr	x20, [x8, #536]
	b	.LBB11_26
.LBB11_15:
	ldr	x2, [x19, #48]
	ldr	x8, [x2, #360]
	ldr	w9, [x8, #4]
	cbz	w9, .LBB11_20
// %bb.16:
	add	x1, x8, #104            // =104
	mov	x0, x19
	bl	ufs_tagset_bt_get
	cmn	w0, #1                  // =1
	b.eq	.LBB11_69
// %bb.17:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB11_19
// %bb.18:
	ldr	x8, [x8, #536]
.LBB11_19:
	add	x8, x8, #20             // =20
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x2, [x19, #48]
	ldr	x8, [x2, #360]
	ldr	w9, [x8, #16]
	add	w0, w9, w0
	cmn	w0, #1                  // =1
	b.ne	.LBB11_68
.LBB11_20:
	ldr	w9, [x8, #8]
	cbz	w9, .LBB11_53
.LBB11_21:
	add	x1, x8, #168            // =168
	mov	x0, x19
	bl	ufs_tagset_bt_get
	cmn	w0, #1                  // =1
	b.eq	.LBB11_53
// %bb.22:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB11_24
// %bb.23:
	ldr	x8, [x8, #536]
.LBB11_24:
	add	x8, x8, #24             // =24
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB11_52
.LBB11_25:
	mov	x20, xzr
.LBB11_26:
	ldr	x1, [x19, #16]
	bl	mas_blk_is_section_ready
	tbz	w0, #0, .LBB11_67
// %bb.27:
	ldrb	w8, [x19, #12]
	tbnz	w8, #7, .LBB11_42
// %bb.28:
	ldr	w10, [x20, #20]
	ldr	x2, [x19, #48]
	ldr	x8, [x2, #360]
	ldr	w9, [x8, #4]
	sub	w11, w9, #1             // =1
	cmp	w10, w11
	b.hs	.LBB11_55
// %bb.29:
	cbz	w9, .LBB11_67
// %bb.30:
	add	x1, x8, #104            // =104
	mov	x0, x19
	bl	ufs_tagset_bt_get
	cmn	w0, #1                  // =1
	b.eq	.LBB11_68
// %bb.31:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB11_33
// %bb.32:
	ldr	x8, [x8, #536]
.LBB11_33:
	add	x8, x8, #20             // =20
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB11_34:
	ldr	x8, [x19, #48]
	ldr	x8, [x8, #360]
	ldr	w8, [x8, #16]
	b	.LBB11_41
.LBB11_35:
	ldr	x2, [x19, #48]
	ldr	x8, [x2, #360]
	ldr	w9, [x8, #8]
	cbz	w9, .LBB11_67
// %bb.36:
	add	x1, x8, #168            // =168
	mov	x0, x19
	bl	ufs_tagset_bt_get
	cmn	w0, #1                  // =1
	b.eq	.LBB11_68
// %bb.37:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB11_39
// %bb.38:
	ldr	x8, [x8, #536]
.LBB11_39:
	add	x8, x8, #24             // =24
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB11_40:
	ldr	x8, [x19, #48]
	ldr	x8, [x8, #360]
	ldr	w8, [x8, #20]
.LBB11_41:
	add	w0, w8, w0
	b	.LBB11_68
.LBB11_42:
	ldr	x2, [x19, #48]
	ldr	x8, [x2, #360]
	ldr	w9, [x8, #4]
	cbz	w9, .LBB11_47
// %bb.43:
	add	x1, x8, #104            // =104
	mov	x0, x19
	bl	ufs_tagset_bt_get
	cmn	w0, #1                  // =1
	b.eq	.LBB11_70
// %bb.44:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB11_46
// %bb.45:
	ldr	x8, [x8, #536]
.LBB11_46:
	add	x8, x8, #20             // =20
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x2, [x19, #48]
	ldr	x8, [x2, #360]
	ldr	w9, [x8, #16]
	add	w0, w9, w0
	cmn	w0, #1                  // =1
	b.ne	.LBB11_68
.LBB11_47:
	ldr	w9, [x8, #8]
	cbz	w9, .LBB11_53
.LBB11_48:
	add	x1, x8, #168            // =168
	mov	x0, x19
	bl	ufs_tagset_bt_get
	cmn	w0, #1                  // =1
	b.eq	.LBB11_53
// %bb.49:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB11_51
// %bb.50:
	ldr	x8, [x8, #536]
.LBB11_51:
	add	x8, x8, #24             // =24
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB11_52:
	ldr	x8, [x19, #48]
	ldr	x8, [x8, #360]
	ldr	w8, [x8, #20]
	add	w0, w8, w0
	cmn	w0, #1                  // =1
	b.ne	.LBB11_68
.LBB11_53:
	mov	x0, x19
	bl	ufs_tagset_get_tag
	cmn	w0, #1                  // =1
	b.ne	.LBB11_68
// %bb.54:
	adrp	x0, .L.str.17
	adrp	x1, .L__func__.ufs_mq_get_all_tag
	add	x0, x0, :lo12:.L.str.17
	add	x1, x1, :lo12:.L__func__.ufs_mq_get_all_tag
	bl	printk
	mov	w0, #-1
	b	.LBB11_68
.LBB11_55:
	ldr	w2, [x20, #20]
	adrp	x0, .L.str.16
	adrp	x1, .L__func__.ufs_mq_get_recovery_tag
	add	x0, x0, :lo12:.L.str.16
	add	x1, x1, :lo12:.L__func__.ufs_mq_get_recovery_tag
	bl	printk
	mov	w0, #-1
	b	.LBB11_68
.LBB11_56:
	ldr	x2, [x19, #48]
	tst	w21, w20
	ldr	x8, [x2, #360]
	b.eq	.LBB11_62
// %bb.57:
	ldr	w9, [x8, #8]
	cbz	w9, .LBB11_67
// %bb.58:
	add	x1, x8, #168            // =168
	mov	x0, x19
	bl	ufs_tagset_bt_get
	cmn	w0, #1                  // =1
	b.eq	.LBB11_68
// %bb.59:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB11_61
// %bb.60:
	ldr	x8, [x8, #536]
.LBB11_61:
	add	x8, x8, #24             // =24
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB11_40
.LBB11_62:
	ldr	w9, [x8, #4]
	cbz	w9, .LBB11_67
// %bb.63:
	add	x1, x8, #104            // =104
	mov	x0, x19
	bl	ufs_tagset_bt_get
	cmn	w0, #1                  // =1
	b.eq	.LBB11_68
// %bb.64:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB11_66
// %bb.65:
	ldr	x8, [x8, #536]
.LBB11_66:
	add	x8, x8, #20             // =20
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB11_34
.LBB11_67:
	mov	w0, #-1
.LBB11_68:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB11_69:
	ldr	x2, [x19, #48]
	ldr	x8, [x2, #360]
	ldr	w9, [x8, #8]
	cbnz	w9, .LBB11_21
	b	.LBB11_53
.LBB11_70:
	ldr	x2, [x19, #48]
	ldr	x8, [x2, #360]
	ldr	w9, [x8, #8]
	cbnz	w9, .LBB11_48
	b	.LBB11_53
.Lfunc_end11:
	.size	ufs_mq_tag_get, .Lfunc_end11-ufs_mq_tag_get
                                        // -- End function
	.p2align	2               // -- Begin function ufs_tagset_get_tag
	.type	ufs_tagset_get_tag,@function
ufs_tagset_get_tag:                     // @ufs_tagset_get_tag
// %bb.0:
	sub	sp, sp, #48             // =48
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	str	x19, [sp, #32]          // 8-byte Folded Spill
	mov	x19, x0
	str	x8, [sp, #8]
	ldr	x0, [x0]
	add	x29, sp, #16            // =16
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB12_8
// %bb.1:
	ldr	x8, [x19, #16]
	cbz	x8, .LBB12_8
// %bb.2:
	ldrb	w8, [x8, #16]
	cmp	w8, #1                  // =1
	b.ne	.LBB12_8
// %bb.3:
	add	x1, sp, #4              // =4
	mov	x0, x19
	strb	wzr, [sp, #4]
	bl	ufs_tagset_bt_get_unistore
	cmn	w0, #1                  // =1
	b.eq	.LBB12_12
// %bb.4:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB12_14
// %bb.5:
	ldr	x8, [x8, #536]
	cbz	x8, .LBB12_14
// %bb.6:
	ldrb	w9, [sp, #4]
	cbz	w9, .LBB12_15
// %bb.7:
	add	x8, x8, #24             // =24
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x19, #48]
	ldr	x8, [x8, #360]
	ldr	w8, [x8, #20]
	add	w0, w8, w0
	b	.LBB12_12
.LBB12_8:
	ldr	x2, [x19, #48]
	mov	x0, x19
	ldr	x8, [x2, #360]
	add	x1, x8, #40             // =40
	bl	ufs_tagset_bt_get
	cmn	w0, #1                  // =1
	b.eq	.LBB12_12
// %bb.9:
	ldr	x8, [x19]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB12_11
// %bb.10:
	ldr	x8, [x8, #536]
.LBB12_11:
	add	x8, x8, #16             // =16
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB12_12:
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB12_16
// %bb.13:
	ldr	x19, [sp, #32]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #48             // =48
	ret
.LBB12_14:
	mov	w0, #-1
	b	.LBB12_12
.LBB12_15:
	add	x8, x8, #16             // =16
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB12_12
.LBB12_16:
	bl	__stack_chk_fail
.Lfunc_end12:
	.size	ufs_tagset_get_tag, .Lfunc_end12-ufs_tagset_get_tag
                                        // -- End function
	.globl	ufs_mq_tag_put          // -- Begin function ufs_mq_tag_put
	.p2align	2
	.type	ufs_mq_tag_put,@function
ufs_mq_tag_put:                         // @ufs_mq_tag_put
.Lufs_mq_tag_put$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	ldr	x8, [x0, #360]
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	mov	x29, sp
	ldr	w9, [x8, #20]
	cmp	w9, w1
	b.ls	.LBB13_6
// %bb.1:
	ldr	w9, [x8, #16]
	cmp	w9, w1
	b.ls	.LBB13_9
// %bb.2:
	ldr	x9, [x2, #272]
	add	x0, x8, #40             // =40
	ldr	w2, [x9, #64]
	bl	sbitmap_queue_clear
	ldr	x8, [x19, #232]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB13_4
// %bb.3:
	ldr	x8, [x8, #536]
.LBB13_4:
	add	x8, x8, #16             // =16
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
.LBB13_5:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	mov	w0, wzr
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB13_6:
	ldr	x10, [x2, #272]
	add	x0, x8, #168            // =168
	sub	w1, w1, w9
	ldr	w2, [x10, #64]
	bl	sbitmap_queue_clear
	ldr	x8, [x19, #232]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB13_8
// %bb.7:
	ldr	x8, [x8, #536]
.LBB13_8:
	add	x8, x8, #24             // =24
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	b	.LBB13_5
.LBB13_9:
	ldr	x10, [x2, #272]
	add	x0, x8, #104            // =104
	sub	w1, w1, w9
	ldr	w2, [x10, #64]
	bl	sbitmap_queue_clear
	ldr	x8, [x19, #232]
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB13_11
// %bb.10:
	ldr	x8, [x8, #536]
.LBB13_11:
	add	x8, x8, #20             // =20
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	b	.LBB13_5
.Lfunc_end13:
	.size	ufs_mq_tag_put, .Lfunc_end13-ufs_mq_tag_put
                                        // -- End function
	.globl	ufs_mq_sync_burst_check_timer_expire // -- Begin function ufs_mq_sync_burst_check_timer_expire
	.p2align	2
	.type	ufs_mq_sync_burst_check_timer_expire,@function
ufs_mq_sync_burst_check_timer_expire:   // @ufs_mq_sync_burst_check_timer_expire
.Lufs_mq_sync_burst_check_timer_expire$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x29, sp
	mov	x19, x0
	bl	ktime_get
	ldr	w8, [x19, #264]
	sub	w8, w8, #1              // =1
	cmp	w8, #1                  // =1
	b.hi	.LBB14_3
// %bb.1:
	ldr	x8, [x19, #176]
	mov	w9, #25856
	movk	w9, #7629, lsl #16
	add	x8, x8, x9
	cmp	x8, x0
	b.ge	.LBB14_3
// %bb.2:
	mov	w8, #3
	str	w8, [x19, #264]
.LBB14_3:
	ldr	x8, [x19, #168]
	mov	w20, #19264
	movk	w20, #76, lsl #16
	add	x8, x8, x20
	cmp	x8, x0
	b.le	.LBB14_7
// %bb.4:
	ldr	w8, [x19, #264]
	cmp	w8, #3                  // =3
	b.eq	.LBB14_12
// %bb.5:
	cmp	w8, #1                  // =1
	b.ne	.LBB14_16
// %bb.6:
	mov	w8, #2
	b	.LBB14_15
.LBB14_7:
	bl	ktime_get
	ldr	w8, [x19, #264]
	cmp	w8, #3                  // =3
	b.eq	.LBB14_10
// %bb.8:
	cmp	w8, #2                  // =2
	b.eq	.LBB14_13
// %bb.9:
	cmp	w8, #1                  // =1
	b.ne	.LBB14_16
.LBB14_10:
	ldr	x8, [x19, #184]
	add	x8, x8, x20
	cmp	x8, x0
	b.ge	.LBB14_16
// %bb.11:
	str	wzr, [x19, #264]
	b	.LBB14_17
.LBB14_12:
	ldr	w8, [x19, #20]
	cbnz	w8, .LBB14_16
	b	.LBB14_14
.LBB14_13:
	ldr	x8, [x19, #168]
	add	x8, x8, x20
	cmp	x8, x0
	b.ge	.LBB14_16
.LBB14_14:
	mov	w8, #1
.LBB14_15:
	str	w8, [x19, #264]
.LBB14_16:
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	add	x0, x19, #208           // =208
	add	x1, x8, #2              // =2
	bl	mod_timer
.LBB14_17:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end14:
	.size	ufs_mq_sync_burst_check_timer_expire, .Lfunc_end14-ufs_mq_sync_burst_check_timer_expire
                                        // -- End function
	.globl	ufs_mq_dump_request     // -- Begin function ufs_mq_dump_request
	.p2align	2
	.type	ufs_mq_dump_request,@function
ufs_mq_dump_request:                    // @ufs_mq_dump_request
.Lufs_mq_dump_request$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x22, [x0, #1376]
	mov	x29, sp
	cbz	x22, .LBB15_2
// %bb.1:
	ldr	x23, [x22, #536]
	cbnz	x23, .LBB15_3
.LBB15_2:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB15_3:
	mov	w20, w1
	ldr	w1, [x0, #1232]
	mov	x19, x0
	adrp	x0, .L.str.4
	add	x0, x0, :lo12:.L.str.4
	bl	printk
	ldr	w1, [x19, #1236]
	adrp	x0, .L.str.5
	add	x0, x0, :lo12:.L.str.5
	bl	printk
	mov	x21, x23
	ldr	x8, [x21, #56]!
	cmp	x21, x8
	b.ne	.LBB15_11
.LBB15_4:
	mov	x21, x23
	ldr	x8, [x21, #80]!
	cmp	x21, x8
	b.ne	.LBB15_16
.LBB15_5:
	ldr	x8, [x23, #112]!
	cmp	x23, x8
	b.eq	.LBB15_2
// %bb.6:
	adrp	x0, .L.str.10
	add	x0, x0, :lo12:.L.str.10
	bl	printk
	mov	x0, x19
	bl	blk_queue_query_unistore_enable
	ldr	x21, [x23]
	cmp	x23, x21
	tbz	w0, #0, .LBB15_21
// %bb.7:
	b.eq	.LBB15_25
.LBB15_8:                               // =>This Inner Loop Header: Depth=1
	mov	x0, x21
	ldr	x8, [x0, #-80]!
	cmp	x8, x19
	b.ne	.LBB15_10
// %bb.9:                               //   in Loop: Header=BB15_8 Depth=1
	mov	w1, w20
	bl	mas_blk_dump_request
.LBB15_10:                              //   in Loop: Header=BB15_8 Depth=1
	ldr	x21, [x21]
	cmp	x23, x21
	b.ne	.LBB15_8
	b	.LBB15_25
.LBB15_11:
	adrp	x0, .L.str.6
	add	x0, x0, :lo12:.L.str.6
	bl	printk
	ldr	x24, [x21]
	b	.LBB15_13
.LBB15_12:                              //   in Loop: Header=BB15_13 Depth=1
	ldr	x24, [x24]
.LBB15_13:                              // =>This Inner Loop Header: Depth=1
	cmp	x21, x24
	b.eq	.LBB15_4
// %bb.14:                              //   in Loop: Header=BB15_13 Depth=1
	mov	x0, x24
	ldr	x8, [x0, #-80]!
	cmp	x8, x19
	b.ne	.LBB15_12
// %bb.15:                              //   in Loop: Header=BB15_13 Depth=1
	mov	w1, w20
	bl	mas_blk_dump_request
	b	.LBB15_12
.LBB15_16:
	adrp	x0, .L.str.7
	add	x0, x0, :lo12:.L.str.7
	bl	printk
	ldr	x24, [x21]
	cmp	x21, x24
	b.eq	.LBB15_20
.LBB15_17:                              // =>This Inner Loop Header: Depth=1
	mov	x0, x24
	ldr	x8, [x0, #-80]!
	cmp	x8, x19
	b.ne	.LBB15_19
// %bb.18:                              //   in Loop: Header=BB15_17 Depth=1
	mov	w1, w20
	bl	mas_blk_dump_request
.LBB15_19:                              //   in Loop: Header=BB15_17 Depth=1
	ldr	x24, [x24]
	cmp	x21, x24
	b.ne	.LBB15_17
.LBB15_20:
	ldp	x1, x2, [x22, #96]
	ldp	w3, w4, [x22, #112]
	adrp	x0, .L.str.18
	add	x0, x0, :lo12:.L.str.18
	bl	printk
	ldr	w1, [x22, #88]
	adrp	x0, .L.str.8
	add	x0, x0, :lo12:.L.str.8
	bl	printk
	ldr	x2, [x22, #8]
	adrp	x21, .L.str.9
	add	x21, x21, :lo12:.L.str.9
	mov	x0, x21
	mov	w1, wzr
	bl	printk
	ldr	x2, [x22, #16]
	mov	w1, #1
	mov	x0, x21
	bl	printk
	ldr	x2, [x22, #24]
	mov	w1, #2
	mov	x0, x21
	bl	printk
	ldr	x2, [x22, #32]
	mov	w1, #3
	mov	x0, x21
	bl	printk
	ldr	x2, [x22, #40]
	mov	w1, #4
	mov	x0, x21
	bl	printk
	ldr	x2, [x22, #48]
	mov	w1, #5
	mov	x0, x21
	bl	printk
	ldr	x2, [x22, #56]
	mov	w1, #6
	mov	x0, x21
	bl	printk
	ldr	x2, [x22, #64]
	mov	w1, #7
	mov	x0, x21
	bl	printk
	ldr	x2, [x22, #72]
	mov	w1, #8
	mov	x0, x21
	bl	printk
	ldr	x2, [x22, #80]
	mov	w1, #9
	mov	x0, x21
	bl	printk
	b	.LBB15_5
.LBB15_21:
	b.eq	.LBB15_25
.LBB15_22:                              // =>This Inner Loop Header: Depth=1
	sub	x0, x21, #704           // =704
	ldr	x8, [x0]
	cmp	x8, x19
	b.ne	.LBB15_24
// %bb.23:                              //   in Loop: Header=BB15_22 Depth=1
	mov	w1, w20
	bl	mas_blk_dump_request
.LBB15_24:                              //   in Loop: Header=BB15_22 Depth=1
	ldr	x21, [x21]
	cmp	x23, x21
	b.ne	.LBB15_22
.LBB15_25:
	ldp	x1, x2, [x22, #352]
	ldr	w3, [x22, #368]
	ldr	w4, [x22, #372]
	adrp	x0, .L.str.18
	add	x0, x0, :lo12:.L.str.18
	bl	printk
	b	.LBB15_2
.Lfunc_end15:
	.size	ufs_mq_dump_request, .Lfunc_end15-ufs_mq_dump_request
                                        // -- End function
	.globl	ufs_mq_flush_plug_list  // -- Begin function ufs_mq_flush_plug_list
	.p2align	2
	.type	ufs_mq_flush_plug_list,@function
ufs_mq_flush_plug_list:                 // @ufs_mq_flush_plug_list
.Lufs_mq_flush_plug_list$local:
// %bb.0:
	sub	sp, sp, #144            // =144
	adrp	x9, .L__const.ufs_mq_flush_plug_list.bd
	adrp	x8, __stack_chk_guard
	add	x9, x9, :lo12:.L__const.ufs_mq_flush_plug_list.bd
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	ldp	x10, x9, [x9]
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #112]    // 16-byte Folded Spill
	add	x29, sp, #48            // =48
	add	x22, sp, #8             // =8
	stp	x28, x27, [sp, #64]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #80]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #96]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #128]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stp	x10, x9, [sp, #24]
	stp	x22, x22, [sp, #8]
	mov	x8, x0
	ldr	x9, [x8, #40]!
	cmp	x8, x9
	b.eq	.LBB16_2
// %bb.1:
	ldr	x10, [x0, #48]
	str	x22, [x9, #8]
	str	x9, [sp, #8]
	str	x22, [x10]
	str	x10, [sp, #16]
	str	x8, [x0, #40]
	str	x8, [x0, #48]
	ldr	x21, [sp, #8]
	b	.LBB16_3
.LBB16_2:
	add	x21, sp, #8             // =8
.LBB16_3:
	adrp	x23, cpu_number
	adrp	x24, __per_cpu_offset
	mov	w27, #26215
	add	x23, x23, :lo12:cpu_number
	add	x24, x24, :lo12:__per_cpu_offset
	mov	w25, #2
	movk	w27, #26214, lsl #16
	mov	w28, #10
.LBB16_4:                               // =>This Inner Loop Header: Depth=1
	mov	x19, x21
	ldr	x20, [x19, #-80]!
	mov	x0, x21
	ldr	x26, [x20, #1376]
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB16_6
// %bb.5:                               //   in Loop: Header=BB16_4 Depth=1
	ldp	x9, x8, [x21]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB16_6:                               //   in Loop: Header=BB16_4 Depth=1
	str	x21, [x21]
	str	x21, [x21, #8]
	str	x19, [sp, #24]
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
	ldr	w8, [x8, x23]
	ldur	w9, [x21, #-56]
	ldr	x10, [x20, #56]
	mov	w1, #7
	ldr	x8, [x24, x8, lsl #3]
	tst	w9, #0xff
	cset	w11, eq
	tst	w9, #0x8000000
	add	x8, x8, x10
	csel	x9, x11, x25, eq
	stur	x8, [x21, #-72]
	add	x8, x8, x9, lsl #3
	ldr	x21, [x8, #80]
	mov	x0, x19
	bl	mas_blk_latency_req_check
	ldr	x2, [x26, #536]
	add	x3, sp, #24             // =24
	mov	x0, x19
	mov	x1, x21
	mov	x4, x20
	bl	__ufs_mq_queue_rq
	cbnz	w0, .LBB16_8
.LBB16_7:                               //   in Loop: Header=BB16_4 Depth=1
	ldr	x21, [sp, #8]
	cmp	x22, x21
	b.ne	.LBB16_4
	b	.LBB16_11
.LBB16_8:                               //   in Loop: Header=BB16_4 Depth=1
	cmp	w0, #9                  // =9
	b.ne	.LBB16_10
// %bb.9:                               //   in Loop: Header=BB16_4 Depth=1
	mov	x0, x19
	bl	__blk_mq_requeue_request
	mov	x0, x19
	mov	x1, x20
	bl	ufs_mq_insert_sync_list
	ldr	x19, [x20, #1376]
	adrp	x8, mas_blk_mq_sync_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_sync_disp_wq]
	mov	w0, #8
	add	x2, x19, #120           // =120
	mov	x3, xzr
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x19, #344]
	mul	x9, x8, x27
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	msub	w8, w9, w28, w8
	add	x8, x19, w8, sxtw #3
	str	x0, [x8, #8]
	ldr	w8, [x19, #88]
	add	w8, w8, #1              // =1
	str	w8, [x19, #88]
	b	.LBB16_7
.LBB16_10:                              //   in Loop: Header=BB16_4 Depth=1
	mov	w1, #10
	mov	x0, x19
	bl	blk_mq_end_request
	b	.LBB16_7
.LBB16_11:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB16_13
// %bb.12:
	ldp	x20, x19, [sp, #128]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #112]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #96]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #80]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #64]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #144            // =144
	ret
.LBB16_13:
	bl	__stack_chk_fail
.Lfunc_end16:
	.size	ufs_mq_flush_plug_list, .Lfunc_end16-ufs_mq_flush_plug_list
                                        // -- End function
	.p2align	2               // -- Begin function __ufs_mq_queue_rq
	.type	__ufs_mq_queue_rq,@function
__ufs_mq_queue_rq:                      // @__ufs_mq_queue_rq
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	mov	x20, x0
	mov	x0, x4
	stp	x26, x25, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	mov	x29, sp
	mov	x23, x4
	mov	x21, x3
	mov	x19, x2
	mov	x22, x1
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB17_3
// %bb.1:
	ldrb	w8, [x20, #240]
	tbz	w8, #2, .LBB17_3
// %bb.2:
	ldr	w8, [x20, #24]
	mov	w9, #61439
	movk	w9, #65471, lsl #16
	and	w8, w8, w9
	str	w8, [x20, #24]
.LBB17_3:
	ldr	x1, [x19]
	mov	x0, x20
	bl	mas_blk_set_lba_flag
	ldr	w8, [x20, #24]
	tbz	w8, #11, .LBB17_45
// %bb.4:
	mov	w25, #4096
	movk	w25, #64, lsl #16
	tst	w8, w25
	b.eq	.LBB17_7
.LBB17_5:
	ldrb	w8, [x20, #200]
	tbnz	w8, #1, .LBB17_10
// %bb.6:
	add	x8, x19, #44            // =44
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	add	x8, x20, #744           // =744
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 1
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB17_11
.LBB17_7:
	ldrb	w8, [x20, #200]
	tbz	w8, #7, .LBB17_18
// %bb.8:
	ldr	x0, [x20]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB17_18
// %bb.9:
	ldr	w8, [x20, #24]
	tst	w8, w25
	b.ne	.LBB17_5
.LBB17_10:
	add	x8, x19, #40            // =40
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	add	x8, x20, #744           // =744
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 2
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB17_11:
	ldr	x0, [x20]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB17_13
// %bb.12:
	ldrb	w8, [x20, #200]
	tbnz	w8, #7, .LBB17_36
.LBB17_13:
	ldr	x0, [x20]
	bl	mas_blk_get_lld
	ldrb	w8, [x0, #24]
	tbz	w8, #7, .LBB17_36
// %bb.14:
	ldrb	w8, [x19, #536]
	cbnz	w8, .LBB17_36
// %bb.15:
	ldr	x0, [x20]
	bl	mas_blk_get_lld
	ldrb	w9, [x20, #200]
	ldr	w8, [x20, #24]
	mov	x24, x0
	tbnz	w9, #1, .LBB17_24
// %bb.16:
	tst	w8, w25
	b.eq	.LBB17_25
// %bb.17:
	ldr	w8, [x19, #40]
	cmp	w8, #2                  // =2
	b.le	.LBB17_25
	b	.LBB17_36
.LBB17_18:
	add	x8, x20, #744           // =744
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	add	x8, x19, #32            // =32
	//APP
	// atomic_add_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	ldr	w8, [x19, #156]
	cmp	w9, w8
	b.le	.LBB17_20
// %bb.19:
	ldrb	w8, [x19, #536]
	cbz	w8, .LBB17_47
.LBB17_20:
	ldrb	w8, [x19, #332]
	cbnz	w8, .LBB17_36
// %bb.21:
	ldr	x0, [x20]
	bl	mas_blk_get_lld
	ldrb	w8, [x0, #24]
	tbz	w8, #7, .LBB17_36
// %bb.22:
	ldrb	w8, [x20, #24]
	cbnz	w8, .LBB17_36
// %bb.23:
	ldr	x8, [x20, #240]
	ldr	x9, [x20, #200]
	orr	x8, x8, #0x2
	orr	x9, x9, #0x1
	str	x8, [x20, #240]
	str	x9, [x20, #200]
	add	x8, x19, #12            // =12
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB17_36
.LBB17_24:
	and	w8, w8, #0xff
	cmp	w8, #1                  // =1
	b.ne	.LBB17_35
.LBB17_25:
	ldr	w8, [x19, #32]
	cbnz	w8, .LBB17_27
// %bb.26:
	ldr	w8, [x19, #36]
	cbz	w8, .LBB17_35
.LBB17_27:
	ldr	w8, [x19, #12]
	cbz	w8, .LBB17_35
// %bb.28:
	ldr	x8, [x24, #1336]
	add	x26, x24, #1336         // =1336
	cmp	x26, x8
	b.eq	.LBB17_35
// %bb.29:
	add	x25, x24, #1368         // =1368
	mov	x0, x25
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x24, #1336]
	mov	x1, x0
	cmp	x26, x8
	b.eq	.LBB17_31
// %bb.30:
	ldur	x24, [x8, #-184]
	b	.LBB17_32
.LBB17_31:
	mov	x24, xzr
.LBB17_32:
	mov	x0, x25
	bl	_raw_spin_unlock_irqrestore
	bl	ktime_get
	mov	w8, #57600
	movk	w8, #1525, lsl #16
	add	x8, x24, x8
	cmp	x0, x8
	b.lt	.LBB17_35
// %bb.33:
	mov	w8, #25856
	movk	w8, #7629, lsl #16
	add	x8, x24, x8
	cmp	x0, x8
	b.ge	.LBB17_36
// %bb.34:
	ldr	w8, [x19, #12]
	ldr	w9, [x19, #8]
	cmp	w8, w9
	b.hs	.LBB17_36
.LBB17_35:
	ldr	x8, [x20, #240]
	ldr	x9, [x20, #200]
	orr	x8, x8, #0x2
	orr	x9, x9, #0x1
	str	x8, [x20, #240]
	str	x9, [x20, #200]
	add	x8, x19, #12            // =12
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB17_36:
	ldr	x8, [x22, #232]
	ldr	x8, [x8, #48]
	ldr	x8, [x8, #16]
	cbz	x8, .LBB17_38
// %bb.37:
	mov	x0, x22
	blr	x8
	tbz	w0, #0, .LBB17_39
.LBB17_38:
	ldr	x8, [x23, #48]
	mov	x0, x22
	mov	x1, x21
	ldr	x8, [x8]
	blr	x8
	ldr	x8, [x21]
	mov	w21, w0
	mov	w1, #10
	mov	x0, x8
	bl	mas_blk_latency_req_check
	ands	w21, w21, #0xff
	b.eq	.LBB17_44
	b	.LBB17_40
.LBB17_39:
	ldr	x0, [x21]
	mov	w1, #11
	bl	mas_blk_latency_req_check
	mov	w21, #9
.LBB17_40:
	ldr	w8, [x20, #256]
	mov	w9, #1
	add	w8, w8, #1              // =1
	stp	w9, w8, [x20, #252]
	ldr	w8, [x19, #44]
	ldr	w9, [x19, #40]
	ldr	w10, [x19, #32]
	ldr	w11, [x19, #36]
	add	w8, w9, w8
	add	w8, w8, w10
	add	w8, w8, w11
	cmp	w8, #8                  // =8
	b.gt	.LBB17_43
// %bb.41:
	mov	w8, #4
	str	w8, [x20, #252]
	add	x8, x20, #268           // =268
.LBB17_42:
	ldr	w9, [x8]
	add	w9, w9, #1              // =1
	str	w9, [x8]
.LBB17_43:
	mov	x0, x20
	mov	x1, x19
	mov	w2, wzr
	bl	ufs_mq_rq_inflt_update
.LBB17_44:
	mov	w0, w21
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB17_45:
	add	x8, x20, #744           // =744
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 8
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	add	x8, x19, #36            // =36
	//APP
	// atomic_add_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	ldr	w8, [x19, #160]
	cmp	w9, w8
	b.le	.LBB17_36
// %bb.46:
	ldrb	w8, [x19, #536]
	cbnz	w8, .LBB17_36
.LBB17_47:
	mov	w9, #2
	add	x8, x20, #260           // =260
	str	w9, [x20, #252]
	mov	w21, #9
	b	.LBB17_42
.Lfunc_end17:
	.size	__ufs_mq_queue_rq, .Lfunc_end17-__ufs_mq_queue_rq
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_insert_sync_list
	.type	ufs_mq_insert_sync_list,@function
ufs_mq_insert_sync_list:                // @ufs_mq_insert_sync_list
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x26, x25, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x1, #1376]
	mov	w1, #8
	mov	x29, sp
	mov	x21, x0
	ldr	x26, [x8, #536]
	bl	mas_blk_latency_req_check
	add	x19, x26, #48           // =48
	mov	x0, x19
	bl	_raw_spin_lock_irqsave
	ldrb	w8, [x21, #200]
	mov	x20, x0
	tbnz	w8, #1, .LBB18_5
// %bb.1:
	ldr	w8, [x21, #24]
	mov	w9, #4096
	movk	w9, #64, lsl #16
	tst	w8, w9
	b.eq	.LBB18_11
// %bb.2:
	ldr	x22, [x26, #64]
	add	x23, x21, #80           // =80
	add	x24, x26, #56           // =56
	mov	x0, x23
	mov	x1, x22
	mov	x2, x24
	bl	__list_add_valid
	tbz	w0, #0, .LBB18_4
// %bb.3:
	str	x23, [x26, #64]
	stp	x24, x22, [x21, #80]
	str	x23, [x22]
.LBB18_4:
	add	x8, x26, #72            // =72
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB18_27
.LBB18_5:
	mov	x23, x26
	ldr	x22, [x23, #56]!
	cmp	x23, x22
	b.eq	.LBB18_20
// %bb.6:
	ldrb	w8, [x22, #120]
	tbz	w8, #1, .LBB18_9
.LBB18_7:                               // =>This Inner Loop Header: Depth=1
	ldr	x8, [x22]
	cmp	x8, x23
	b.eq	.LBB18_21
// %bb.8:                               //   in Loop: Header=BB18_7 Depth=1
	mov	x22, x8
	ldrb	w8, [x8, #120]
	tbnz	w8, #1, .LBB18_7
.LBB18_9:
	ldr	x23, [x22, #8]
	add	x24, x21, #80           // =80
	mov	x0, x24
	mov	x2, x22
	mov	x1, x23
	bl	__list_add_valid
	tbz	w0, #0, .LBB18_23
// %bb.10:
	str	x24, [x22, #8]
	stp	x22, x23, [x21, #80]
	str	x24, [x23]
	b	.LBB18_23
.LBB18_11:
	mov	x23, x26
	ldr	x22, [x23, #80]!
	cmp	x23, x22
	b.eq	.LBB18_13
// %bb.12:
	ldr	w8, [x21, #680]
	cbnz	w8, .LBB18_16
.LBB18_13:
	ldr	x22, [x26, #88]
	add	x24, x21, #80           // =80
	mov	x0, x24
	mov	x2, x23
	mov	x1, x22
	bl	__list_add_valid
	tbz	w0, #0, .LBB18_26
// %bb.14:
	str	x24, [x26, #88]
	stp	x23, x22, [x21, #80]
	str	x24, [x22]
	b	.LBB18_26
.LBB18_15:                              //   in Loop: Header=BB18_16 Depth=1
	ldr	x22, [x22]
	cmp	x23, x22
	b.eq	.LBB18_24
.LBB18_16:                              // =>This Inner Loop Header: Depth=1
	ldr	w9, [x22, #600]
	cbz	w9, .LBB18_15
// %bb.17:                              //   in Loop: Header=BB18_16 Depth=1
	sub	w9, w8, w9
	tbz	w9, #31, .LBB18_15
// %bb.18:
	ldr	x23, [x22, #8]
	add	x24, x21, #80           // =80
	mov	x0, x24
	mov	x2, x22
	mov	x1, x23
	bl	__list_add_valid
	tbz	w0, #0, .LBB18_26
// %bb.19:
	str	x24, [x22, #8]
	stp	x22, x23, [x21, #80]
	str	x24, [x23]
	b	.LBB18_26
.LBB18_20:
	ldr	x22, [x26, #64]
.LBB18_21:
	add	x24, x21, #80           // =80
	mov	x0, x24
	mov	x1, x22
	mov	x2, x23
	bl	__list_add_valid
	tbz	w0, #0, .LBB18_23
// %bb.22:
	str	x24, [x26, #64]
	stp	x23, x22, [x21, #80]
	str	x24, [x22]
.LBB18_23:
	add	x8, x26, #72            // =72
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB18_27
.LBB18_24:
	ldr	x24, [x22, #8]
	add	x25, x21, #80           // =80
	mov	x0, x25
	mov	x2, x23
	mov	x1, x24
	bl	__list_add_valid
	tbz	w0, #0, .LBB18_26
// %bb.25:
	str	x25, [x22, #8]
	stp	x22, x24, [x21, #80]
	str	x25, [x24]
.LBB18_26:
	add	x8, x26, #96            // =96
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB18_27:
	mov	x0, x19
	mov	x1, x20
	bl	_raw_spin_unlock_irqrestore
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.Lfunc_end18:
	.size	ufs_mq_insert_sync_list, .Lfunc_end18-ufs_mq_insert_sync_list
                                        // -- End function
	.globl	ufs_mq_make_request     // -- Begin function ufs_mq_make_request
	.p2align	2
	.type	ufs_mq_make_request,@function
ufs_mq_make_request:                    // @ufs_mq_make_request
.Lufs_mq_make_request$local:
// %bb.0:
	sub	sp, sp, #192            // =192
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #96]     // 16-byte Folded Spill
	add	x29, sp, #96            // =96
	str	x27, [sp, #112]         // 8-byte Folded Spill
	stp	x26, x25, [sp, #128]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #144]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #160]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #176]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stur	x1, [x29, #-32]
	stp	xzr, xzr, [sp, #48]
	stp	xzr, xzr, [sp, #32]
	stp	xzr, xzr, [sp, #16]
	str	xzr, [sp, #8]
	ldr	w22, [x1, #16]
	mov	w8, #253
	mov	x21, x1
	mov	x19, x0
	tst	w22, w8
	b.eq	.LBB19_3
// %bb.1:
	mov	w8, #6144
	movk	w8, #68, lsl #16
	and	w8, w22, w8
	cbnz	w8, .LBB19_3
// %bb.2:
	mov	w20, #1
	ldr	x8, [x21, #8]
	cbnz	x8, .LBB19_10
	b	.LBB19_17
.LBB19_3:
	ldr	x8, [x19, #1376]
	orr	w9, w22, #0x800
	str	w9, [x21, #16]
	ldr	x21, [x8, #536]
	bl	ktime_get
	ldr	w8, [x21, #264]
	mov	x20, x0
	cmp	w8, #1                  // =1
	b.eq	.LBB19_7
// %bb.4:
	cbnz	w8, .LBB19_9
// %bb.5:
	ldr	x8, [x21, #168]
	mov	w9, #19264
	movk	w9, #76, lsl #16
	add	x8, x8, x9
	cmp	x8, x20
	b.le	.LBB19_9
// %bb.6:
	mov	w8, #1
	str	x20, [x21, #176]
	str	w8, [x21, #264]
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	add	x0, x21, #208           // =208
	add	x1, x8, #2              // =2
	bl	mod_timer
	b	.LBB19_9
.LBB19_7:
	ldr	x8, [x21, #168]
	mov	w9, #19264
	movk	w9, #76, lsl #16
	add	x8, x8, x9
	cmp	x8, x20
	b.le	.LBB19_9
// %bb.8:
	mov	w8, #2
	str	w8, [x21, #264]
.LBB19_9:
	str	x20, [x21, #168]
	ldur	x21, [x29, #-32]
	mov	w20, wzr
	ldr	w22, [x21, #16]
	ldr	x8, [x21, #8]
	cbz	x8, .LBB19_17
.LBB19_10:
	ldr	x0, [x8, #1264]
	cbz	x0, .LBB19_17
// %bb.11:
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB19_17
// %bb.12:
	ldrb	w8, [x21, #16]
	cbnz	w8, .LBB19_20
// %bb.13:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldr	x9, [x8, #2280]
	cbz	x9, .LBB19_20
// %bb.14:
	ldr	w10, [x21, #40]
	cmp	w10, #128, lsl #12      // =524288
	b.hi	.LBB19_20
// %bb.15:
	ldr	x11, [x9, #56]
	adrp	x10, __cfi_ufs_mq_flush_plug_list
	add	x10, x10, :lo12:__cfi_ufs_mq_flush_plug_list
	cbz	x11, .LBB19_24
// %bb.16:
	cmp	x11, x10
	b.eq	.LBB19_24
	b	.LBB19_20
.LBB19_17:
	tst	w22, #0xc0000
	cset	w8, ne
	orr	w8, w20, w8
	tbnz	w8, #0, .LBB19_20
// %bb.18:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldr	x9, [x8, #2280]
	cbz	x9, .LBB19_20
// %bb.19:
	ldr	w10, [x21, #40]
	cmp	w10, #128, lsl #12      // =524288
	b.ls	.LBB19_21
.LBB19_20:
	mov	x24, xzr
	b	.LBB19_25
.LBB19_21:
	ldr	x10, [x9, #56]
	cbz	x10, .LBB19_23
// %bb.22:
	adrp	x11, __cfi_ufs_mq_flush_plug_list
	add	x11, x11, :lo12:__cfi_ufs_mq_flush_plug_list
	cmp	x10, x11
	b.ne	.LBB19_20
.LBB19_23:
	adrp	x10, __cfi_ufs_mq_flush_plug_list
	add	x10, x10, :lo12:__cfi_ufs_mq_flush_plug_list
.LBB19_24:
	str	x10, [x9, #56]
	ldr	x24, [x8, #2280]
.LBB19_25:
	sub	x1, x29, #32            // =32
	add	x2, sp, #4              // =4
	mov	x0, x19
	str	wzr, [sp, #4]
	bl	__blk_queue_split
	cbz	x24, .LBB19_36
// %bb.26:
	ldr	x26, [x24, #48]
	add	x27, x24, #40           // =40
	cmp	x27, x26
	b.eq	.LBB19_36
// %bb.27:
	ldur	x21, [x29, #-32]
	ldr	w22, [sp, #4]
	mov	x25, xzr
	b	.LBB19_30
.LBB19_28:                              //   in Loop: Header=BB19_30 Depth=1
	mov	x0, x23
	mov	x1, x21
	mov	w2, w22
	bl	bio_attempt_front_merge
	mov	x25, x23
	tbnz	w0, #0, .LBB19_35
.LBB19_29:                              //   in Loop: Header=BB19_30 Depth=1
	ldr	x26, [x26, #8]
	cmp	x27, x26
	b.eq	.LBB19_37
.LBB19_30:                              // =>This Inner Loop Header: Depth=1
	mov	x23, x26
	ldr	x8, [x23, #-80]!
	cmp	x8, x19
	b.ne	.LBB19_29
// %bb.31:                              //   in Loop: Header=BB19_30 Depth=1
	mov	x0, x23
	mov	x1, x21
	bl	blk_rq_merge_ok
	mov	x25, x23
	tbz	w0, #0, .LBB19_29
// %bb.32:                              //   in Loop: Header=BB19_30 Depth=1
	mov	x0, x23
	mov	x1, x21
	bl	blk_try_merge
	cmp	w0, #1                  // =1
	b.eq	.LBB19_28
// %bb.33:                              //   in Loop: Header=BB19_30 Depth=1
	cmp	w0, #2                  // =2
	mov	x25, x23
	b.ne	.LBB19_29
// %bb.34:                              //   in Loop: Header=BB19_30 Depth=1
	mov	x0, x23
	mov	x1, x21
	mov	w2, w22
	bl	bio_attempt_back_merge
	mov	x25, x23
	tbz	w0, #0, .LBB19_29
.LBB19_35:
	mov	w19, #-1
	b	.LBB19_79
.LBB19_36:
	mov	x25, xzr
.LBB19_37:
	ldur	x8, [x29, #-32]
	ldrh	w9, [x8, #20]
	orr	w9, w9, #0x1000
	strh	w9, [x8, #20]
	ldr	x0, [x19, #24]
	cbz	x0, .LBB19_39
// %bb.38:
	mov	x1, x8
	bl	__rq_qos_throttle
	ldur	x8, [x29, #-32]
.LBB19_39:
	mov	w1, #3
	mov	x0, x8
	bl	mas_blk_latency_bio_check
	ldur	x1, [x29, #-32]
	add	x2, sp, #8              // =8
	mov	x0, x19
	ldr	w8, [x1, #16]
	str	w8, [sp, #44]
	ldr	x8, [x1, #80]
	str	w8, [sp, #20]
	bl	blk_mq_get_request
	ldur	x1, [x29, #-32]
	cbz	x0, .LBB19_83
// %bb.40:
	ldrb	w8, [x0, #24]
	ldr	x9, [x1, #80]
	mov	x21, x0
	cmp	w8, #1                  // =1
	str	x9, [x0, #200]
	b.eq	.LBB19_44
// %bb.41:
	cmp	w8, #3                  // =3
	b.eq	.LBB19_44
// %bb.42:
	cmp	w8, #2                  // =2
	b.ne	.LBB19_45
// %bb.43:
	ldr	x0, [x21]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB19_45
.LBB19_44:
	mov	x0, x21
	bl	blk_req_set_make_req_nr
.LBB19_45:
	ldr	x0, [x19, #24]
	cbz	x0, .LBB19_47
// %bb.46:
	ldur	x2, [x29, #-32]
	mov	x1, x21
	bl	__rq_qos_track
.LBB19_47:
	ldr	w8, [x21, #32]
	ldr	x26, [sp, #56]
	cmn	w8, #1                  // =1
	b.eq	.LBB19_49
// %bb.48:
	ldr	w9, [x26, #452]
	orr	w19, w8, w9, lsl #16
	b	.LBB19_50
.LBB19_49:
	ldr	w8, [x21, #36]
	ldr	w9, [x26, #452]
	orr	w8, w8, w9, lsl #16
	orr	w19, w8, #0x80000000
.LBB19_50:
	ldur	x22, [x29, #-32]
	ldr	w8, [x22, #16]
	and	w9, w8, #0xff
	cmp	w9, #2                  // =2
	b.eq	.LBB19_82
// %bb.51:
	tbnz	w8, #19, .LBB19_82
// %bb.52:
	ldr	w9, [x21, #24]
	tbz	w9, #11, .LBB19_87
// %bb.53:
	ldr	x8, [x21]
	ldr	w2, [sp, #4]
	mov	x0, x21
	mov	x1, x22
	ldr	x26, [x8, #1376]
	stp	xzr, xzr, [x29, #-24]
	bl	blk_mq_bio_to_request
	mov	x20, x21
	cbz	x24, .LBB19_69
// %bb.54:
	ldr	x0, [x21]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB19_61
// %bb.55:
	adrp	x8, __tracepoint_block_plug+8
	ldr	x20, [x21]
	ldr	w8, [x8, :lo12:__tracepoint_block_plug+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB19_61
// %bb.56:
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
	tbz	w8, #0, .LBB19_61
// %bb.57:
	//APP
	mrs x22, sp_el0
	//NO_APP
	ldr	w8, [x22, #24]
	add	w8, w8, #1              // =1
	str	w8, [x22, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_block_plug+32
	ldr	x23, [x8, :lo12:__tracepoint_block_plug+32]
	cbz	x23, .LBB19_59
.LBB19_58:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	mov	x1, x20
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB19_58
.LBB19_59:
	//APP
	//NO_APP
	ldr	x8, [x22, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x22, #24]
	b.eq	.LBB19_95
// %bb.60:
	ldr	x8, [x22, #24]
	cbz	x8, .LBB19_95
.LBB19_61:
	cbz	x25, .LBB19_66
.LBB19_62:
	mov	x8, x24
	ldr	x9, [x8, #40]!
	cmp	x8, x9
	b.eq	.LBB19_92
// %bb.63:
	add	x20, x25, #80           // =80
	mov	x0, x20
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB19_65
// %bb.64:
	ldp	x9, x8, [x25, #80]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB19_65:
	str	x20, [x25, #80]
	str	x20, [x25, #88]
.LBB19_66:
	ldr	x20, [x24, #48]
	add	x22, x21, #80           // =80
	add	x23, x24, #40           // =40
	mov	x0, x22
	mov	x1, x20
	mov	x2, x23
	bl	__list_add_valid
	tbz	w0, #0, .LBB19_68
// %bb.67:
	str	x22, [x24, #48]
	stp	x23, x20, [x21, #80]
	str	x22, [x20]
.LBB19_68:
	mov	w1, #6
	mov	x0, x21
	bl	mas_blk_latency_req_check
	mov	x20, x25
.LBB19_69:
	stur	x20, [x29, #-24]
	ldr	x0, [x21]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB19_76
// %bb.70:
	cbz	x20, .LBB19_79
// %bb.71:
	ldr	x4, [x21]
	ldrb	w9, [x20, #24]
	ldr	x8, [x4, #1376]
	ldr	x8, [x8, #536]
	cbz	w9, .LBB19_74
// %bb.72:
	mov	x9, x8
	ldr	x10, [x9, #56]!
	cmp	x9, x10
	b.ne	.LBB19_75
// %bb.73:
	mov	x9, x8
	ldr	x10, [x9, #80]!
	cmp	x9, x10
	b.ne	.LBB19_75
.LBB19_74:
	ldr	x9, [x8, #136]!
	cmp	x8, x9
	b.eq	.LBB19_81
.LBB19_75:
	mov	x0, x20
	bl	__blk_mq_requeue_request
	ldr	x20, [x21]
	ldur	x0, [x29, #-24]
	mov	x1, x20
	bl	ufs_mq_insert_sync_list
	ldr	x20, [x20, #1376]
	adrp	x8, mas_blk_mq_sync_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_sync_disp_wq]
	mov	w0, #8
	add	x2, x20, #120           // =120
	mov	x3, xzr
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x20, #344]
	mov	w9, #26215
	movk	w9, #26214, lsl #16
	mul	x9, x8, x9
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	mov	w10, #10
	msub	w8, w9, w10, w8
	add	x8, x20, w8, sxtw #3
	str	x0, [x8, #8]
	ldr	w8, [x20, #88]
	add	w8, w8, #1              // =1
	str	w8, [x20, #88]
	b	.LBB19_79
.LBB19_76:
	cbz	x20, .LBB19_79
// %bb.77:
	ldr	x1, [sp, #56]
	ldr	x2, [x26, #536]
	ldr	x4, [x21]
.LBB19_78:
	sub	x3, x29, #24            // =24
	mov	x0, x20
	bl	__ufs_mq_queue_rq
	cbnz	w0, .LBB19_93
.LBB19_79:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB19_103
// %bb.80:
	mov	w0, w19
	ldp	x20, x19, [sp, #176]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #160]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #144]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #128]    // 16-byte Folded Reload
	ldr	x27, [sp, #112]         // 8-byte Folded Reload
	ldp	x29, x30, [sp, #96]     // 16-byte Folded Reload
	add	sp, sp, #192            // =192
	ret
.LBB19_81:
	ldr	x1, [sp, #56]
	ldr	x2, [x26, #536]
	b	.LBB19_78
.LBB19_82:
	ldr	w2, [sp, #4]
	mov	x0, x21
	mov	x1, x22
	bl	blk_mq_bio_to_request
	mov	x0, x21
	bl	blk_insert_flush
	b	.LBB19_91
.LBB19_83:
	ldr	x0, [x19, #24]
	cbz	x0, .LBB19_85
// %bb.84:
	bl	__rq_qos_cleanup
	ldur	x1, [x29, #-32]
.LBB19_85:
	ldrb	w8, [x1, #18]
	tbz	w8, #7, .LBB19_35
// %bb.86:
	mov	w8, #12
	mov	x0, x1
	strb	w8, [x1, #26]
	bl	bio_endio
	mov	w19, #-1
	b	.LBB19_79
.LBB19_87:
	ldr	x23, [sp, #48]
	ldr	w2, [sp, #4]
	mov	w10, #16384
	movk	w10, #12, lsl #16
	tst	w8, w10
	b.ne	.LBB19_89
// %bb.88:
	and	w8, w9, #0xfe
	orr	w8, w8, #0x1
	cmp	w8, #33                 // =33
	b.ne	.LBB19_96
.LBB19_89:
	mov	x0, x21
	mov	x1, x22
	bl	blk_mq_bio_to_request
	mov	x0, x23
	bl	_raw_spin_lock
.LBB19_90:
	ldr	x1, [x26, #232]
	mov	x0, x21
	bl	ufs_mq_req_insert
	mov	x0, x23
	bl	_raw_spin_unlock
.LBB19_91:
	ldr	x0, [sp, #56]
	mov	w1, w20
	bl	blk_mq_run_hw_queue
	b	.LBB19_79
.LBB19_92:
	mov	x25, xzr
	b	.LBB19_66
.LBB19_93:
	ldur	x20, [x29, #-24]
	cmp	w0, #9                  // =9
	b.eq	.LBB19_75
// %bb.94:
	mov	w1, #10
	mov	x0, x20
	bl	blk_mq_end_request
	b	.LBB19_79
.LBB19_95:
	bl	preempt_schedule_notrace
	cbnz	x25, .LBB19_62
	b	.LBB19_66
.LBB19_96:
	ldr	x25, [x26, #232]
	mov	x0, x23
	str	w2, [sp]                // 4-byte Folded Spill
	mov	x27, x26
	bl	_raw_spin_lock
	ldrb	w8, [x22, #17]
	tbnz	w8, #3, .LBB19_102
// %bb.97:
	ldr	x8, [x25, #1384]
	cbz	x8, .LBB19_102
// %bb.98:
	ldr	x26, [x8, #176]
	cbz	x26, .LBB19_102
// %bb.99:
	ldr	x8, [x26]
	ldr	x8, [x8, #40]
	cbz	x8, .LBB19_102
// %bb.100:
	ldr	x8, [x25, #1376]
	ldr	x8, [x8, #536]
	add	x24, x8, #100           // =100
	mov	x0, x24
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x26]
	ldr	w2, [sp]                // 4-byte Folded Reload
	mov	x26, x0
	mov	x0, x22
	ldr	x8, [x8, #40]
	mov	x1, x25
	blr	x8
	mov	w25, w0
	mov	x0, x24
	mov	x1, x26
	bl	_raw_spin_unlock_irqrestore
	tbz	w25, #0, .LBB19_102
// %bb.101:
	mov	x0, x23
	bl	_raw_spin_unlock
	mov	x0, x21
	bl	blk_mq_free_request
	b	.LBB19_79
.LBB19_102:
	ldr	w2, [sp]                // 4-byte Folded Reload
	mov	x0, x21
	mov	x1, x22
	bl	blk_mq_bio_to_request
	mov	x26, x27
	b	.LBB19_90
.LBB19_103:
	bl	__stack_chk_fail
.Lfunc_end19:
	.size	ufs_mq_make_request, .Lfunc_end19-ufs_mq_make_request
                                        // -- End function
	.globl	ufs_mq_sync_io_dispatch_work_fn // -- Begin function ufs_mq_sync_io_dispatch_work_fn
	.p2align	2
	.type	ufs_mq_sync_io_dispatch_work_fn,@function
ufs_mq_sync_io_dispatch_work_fn:        // @ufs_mq_sync_io_dispatch_work_fn
.Lufs_mq_sync_io_dispatch_work_fn$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x0, [x0, #136]
	mov	x29, sp
	bl	ufs_mq_sync_dispatch
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end20:
	.size	ufs_mq_sync_io_dispatch_work_fn, .Lfunc_end20-ufs_mq_sync_io_dispatch_work_fn
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_sync_dispatch
	.type	ufs_mq_sync_dispatch,@function
ufs_mq_sync_dispatch:                   // @ufs_mq_sync_dispatch
// %bb.0:
	sub	sp, sp, #320            // =320
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #224]    // 16-byte Folded Spill
	add	x29, sp, #224           // =224
	stp	x28, x27, [sp, #240]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #256]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #272]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #288]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #304]    // 16-byte Folded Spill
	stur	x8, [x29, #-16]
	stp	xzr, xzr, [sp, #112]
	ldr	x28, [x0, #1376]
	mov	x20, x0
	ldr	x19, [x28, #536]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB21_93
// %bb.1:
	mov	x0, x20
	bl	mas_blk_get_lld
	mov	x27, x0
	cbz	x0, .LBB21_3
// %bb.2:
	ldr	w8, [x27, #564]
	cbz	w8, .LBB21_93
.LBB21_3:
	mov	x0, x20
	bl	mas_blk_get_lld
	cbz	x0, .LBB21_92
// %bb.4:
	ldr	x8, [x0, #88]
	mov	x22, x0
	cbz	x8, .LBB21_92
// %bb.5:
	add	x23, x22, #624          // =624
	mov	x0, x23
	bl	mutex_lock
	ldr	w8, [x22, #568]
	cbnz	w8, .LBB21_9
// %bb.6:
	ldr	w21, [x22, #572]
	mov	x0, x23
	bl	mutex_unlock
	cbz	w21, .LBB21_40
// %bb.7:
	adrp	x26, g_recovery_pwron_info
	add	x26, x26, :lo12:g_recovery_pwron_info
	ldr	w24, [x26, #4]
	mov	x0, x23
	cmp	w24, #0                 // =0
	cset	w21, ne
	bl	mutex_lock
	ldr	w8, [x26]
	cbnz	w8, .LBB21_12
// %bb.8:
	mov	x0, x23
	bl	mutex_unlock
	b	.LBB21_92
.LBB21_9:
	adrp	x19, g_recovery_pwron_info
	mov	w21, #1
	add	x19, x19, :lo12:g_recovery_pwron_info
	mov	x0, x23
	str	wzr, [x22, #568]
	str	w21, [x22, #572]
	str	wzr, [x19]
	bl	mutex_unlock
	adrp	x8, mas_blk_recovery_pwron_info_done
	add	x8, x8, :lo12:mas_blk_recovery_pwron_info_done
	mov	w9, #2
	str	xzr, [x19, #32]
	str	wzr, [x19, #4]
	str	x8, [x19, #144]
	strb	w9, [x19, #152]
	ldr	x8, [x22, #88]
	add	x1, x19, #8             // =8
	mov	x0, x20
	mov	w2, wzr
	blr	x8
	mov	w19, w0
	cbz	w0, .LBB21_11
// %bb.10:
	mov	x0, x23
	bl	mutex_lock
	mov	x0, x23
	str	w21, [x22, #568]
	str	wzr, [x22, #572]
	bl	mutex_unlock
.LBB21_11:
	adrp	x0, .L.str.19
	adrp	x1, .L__func__.mas_blk_recovery_pwron_info_sync
	add	x0, x0, :lo12:.L.str.19
	add	x1, x1, :lo12:.L__func__.mas_blk_recovery_pwron_info_sync
	mov	w2, w19
	bl	printk
	b	.LBB21_92
.LBB21_12:
	adrp	x8, g_recovery_pwron_info
	mov	x0, x23
	str	wzr, [x8, :lo12:g_recovery_pwron_info]
	bl	mutex_unlock
	cbnz	w24, .LBB21_39
// %bb.13:
	mov	x0, x20
	bl	mas_blk_get_lld
	cbz	x0, .LBB21_39
// %bb.14:
	ldr	w8, [x0, #232]
	mov	x25, x0
	cbz	w8, .LBB21_39
// %bb.15:
	ldr	w8, [x25, #236]
	cbz	w8, .LBB21_39
// %bb.16:
	str	x27, [sp, #88]          // 8-byte Folded Spill
	str	x28, [sp, #104]         // 8-byte Folded Spill
	mov	x28, xzr
	add	x27, x25, #336          // =336
	add	x0, x25, #320           // =320
.LBB21_17:                              // =>This Inner Loop Header: Depth=1
	str	x0, [sp, #80]           // 8-byte Folded Spill
	bl	_raw_spin_lock_irqsave
	mov	x11, x28
	ldr	x2, [x27, #32]
	ldp	w9, w8, [x25, #232]
	ldr	x5, [x27, #64]
	ldr	x10, [x27]
	stp	x0, x11, [sp, #64]      // 16-byte Folded Spill
	add	x11, x26, x11, lsl #2
	ldr	w11, [x11, #12]
	mul	x8, x2, x8
	udiv	x6, x10, x9
	add	x28, x28, #1            // =1
	udiv	x3, x8, x9
	msub	x7, x6, x9, x10
	udiv	w10, w11, w9
	adrp	x0, .L.str.21
	msub	x4, x3, x9, x8
	msub	w8, w10, w9, w11
	add	x0, x0, :lo12:.L.str.21
	mov	w1, w28
	str	w10, [sp]
	str	w8, [sp, #8]
	bl	printk
	mov	w0, w28
	str	x28, [sp, #96]          // 8-byte Folded Spill
	bl	mas_blk_enable_disorder_stream
	tbz	w0, #0, .LBB21_19
// %bb.18:                              //   in Loop: Header=BB21_17 Depth=1
	ldr	x8, [x27, #32]
	ldr	w9, [x25, #236]
	ldr	x10, [x27, #64]
	madd	x8, x8, x9, x10
	b	.LBB21_20
.LBB21_19:                              //   in Loop: Header=BB21_17 Depth=1
	ldr	x8, [x27]
.LBB21_20:                              //   in Loop: Header=BB21_17 Depth=1
	str	x8, [sp, #56]           // 8-byte Folded Spill
	ldr	x8, [sp, #72]           // 8-byte Folded Reload
	ldr	x0, [sp, #96]           // 8-byte Folded Reload
                                        // kill: def $w0 killed $w0 killed $x0
	add	x28, x26, x8, lsl #2
	ldr	w8, [x28, #12]
	str	x8, [x27]
	bl	mas_blk_enable_disorder_stream
	tbz	w0, #0, .LBB21_22
// %bb.21:                              //   in Loop: Header=BB21_17 Depth=1
	ldr	w8, [x28, #12]
	ldr	w9, [x25, #236]
	udiv	w8, w8, w9
	str	x8, [x27, #32]
	ldr	w8, [x28, #12]
	udiv	w10, w8, w9
	msub	w8, w10, w9, w8
	str	x8, [x27, #64]
.LBB21_22:                              //   in Loop: Header=BB21_17 Depth=1
	bl	ktime_get
	ldr	x28, [sp, #96]          // 8-byte Folded Reload
	ldr	x1, [x27]
	str	x0, [x27, #96]
	mov	x0, x25
	mov	w2, w28
	bl	mas_blk_recovery_update_section_list
	mov	w0, w28
	bl	mas_blk_enable_disorder_stream
	tbz	w0, #0, .LBB21_24
// %bb.23:                              //   in Loop: Header=BB21_17 Depth=1
	ldr	x8, [x27, #32]
	ldr	w9, [x25, #236]
	ldr	x10, [x27, #64]
	madd	x8, x8, x9, x10
	b	.LBB21_25
.LBB21_24:                              //   in Loop: Header=BB21_17 Depth=1
	ldr	x8, [x27]
.LBB21_25:                              //   in Loop: Header=BB21_17 Depth=1
	ldp	x10, x1, [sp, #56]      // 16-byte Folded Reload
	ldr	x9, [x25, #840]
	ldr	x28, [sp, #80]          // 8-byte Folded Reload
	sub	x8, x10, x8
	cmp	x9, x8
	csel	x8, x9, x8, hi
	mov	x0, x28
	str	x8, [x25, #840]
	bl	_raw_spin_unlock_irqrestore
	ldr	x8, [sp, #96]           // 8-byte Folded Reload
	mov	x0, x28
	add	x27, x27, #8            // =8
	add	x0, x28, #4             // =4
	cmp	x8, #4                  // =4
	mov	x28, x8
	b.ne	.LBB21_17
// %bb.26:
	ldr	x26, [x25, #1440]
	add	x0, x26, #152           // =152
	str	x0, [sp, #56]           // 8-byte Folded Spill
	bl	_raw_spin_lock_irqsave
	add	x8, x26, #136           // =136
	mov	w28, #1
	str	x0, [sp, #48]           // 8-byte Folded Spill
	str	x8, [sp, #64]           // 8-byte Folded Spill
.LBB21_27:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB21_28 Depth 2
                                        //       Child Loop BB21_30 Depth 3
	add	x8, x25, x28, lsl #2
	add	x0, x8, #736            // =736
	str	x0, [sp, #96]           // 8-byte Folded Spill
	bl	_raw_spin_lock_irqsave
	add	x8, x25, x28, lsl #4
	ldr	x27, [x8, #656]
	add	x8, x8, #656            // =656
	stp	x8, x0, [sp, #72]       // 16-byte Folded Spill
	cmp	x27, x8
	b.eq	.LBB21_37
.LBB21_28:                              //   Parent Loop BB21_27 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB21_30 Depth 3
	sub	x26, x27, #384          // =384
	adrp	x2, g_recovery_pwron_info+8
	mov	x0, x26
	mov	x1, x20
	add	x2, x2, :lo12:g_recovery_pwron_info+8
	bl	mas_blk_bio_need_dispatch
	tbz	w0, #0, .LBB21_36
// %bb.29:                              //   in Loop: Header=BB21_28 Depth=2
	ldr	x9, [sp, #64]           // 8-byte Folded Reload
	ldr	x8, [x9]
	cmp	x8, x9
	b.eq	.LBB21_32
.LBB21_30:                              //   Parent Loop BB21_27 Depth=1
                                        //     Parent Loop BB21_28 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	sub	x9, x8, #400            // =400
	cmp	x9, x26
	b.eq	.LBB21_36
// %bb.31:                              //   in Loop: Header=BB21_30 Depth=3
	ldr	x8, [x8]
	ldr	x9, [sp, #64]           // 8-byte Folded Reload
	cmp	x9, x8
	b.ne	.LBB21_30
.LBB21_32:                              //   in Loop: Header=BB21_28 Depth=2
	mov	x0, x20
	bl	mas_blk_get_lld
	cbz	x0, .LBB21_36
// %bb.33:                              //   in Loop: Header=BB21_28 Depth=2
	ldr	x26, [x0, #1440]
	cbz	x26, .LBB21_36
// %bb.34:                              //   in Loop: Header=BB21_28 Depth=2
	ldr	x1, [x26, #144]
	add	x0, x27, #16            // =16
	add	x2, x26, #136           // =136
	str	x2, [sp, #24]           // 8-byte Folded Spill
	stp	x0, x1, [sp, #32]       // 16-byte Folded Spill
	bl	__list_add_valid
	tbz	w0, #0, .LBB21_36
// %bb.35:                              //   in Loop: Header=BB21_28 Depth=2
	ldp	x8, x9, [sp, #24]       // 16-byte Folded Reload
	str	x9, [x26, #144]
	str	x8, [x27, #16]
	ldr	x8, [sp, #40]           // 8-byte Folded Reload
	str	x8, [x27, #24]
	str	x9, [x8]
.LBB21_36:                              //   in Loop: Header=BB21_28 Depth=2
	ldr	x27, [x27]
	ldr	x8, [sp, #72]           // 8-byte Folded Reload
	cmp	x8, x27
	b.ne	.LBB21_28
.LBB21_37:                              //   in Loop: Header=BB21_27 Depth=1
	ldr	x0, [sp, #96]           // 8-byte Folded Reload
	ldr	x1, [sp, #80]           // 8-byte Folded Reload
	bl	_raw_spin_unlock_irqrestore
	add	x28, x28, #1            // =1
	cmp	x28, #5                 // =5
	b.ne	.LBB21_27
// %bb.38:
	ldp	x1, x0, [sp, #48]       // 16-byte Folded Reload
	bl	_raw_spin_unlock_irqrestore
	ldr	x28, [sp, #104]         // 8-byte Folded Reload
	ldr	x27, [sp, #88]          // 8-byte Folded Reload
.LBB21_39:
	mov	x0, x23
	bl	mutex_lock
	mov	x0, x23
	str	w21, [x22, #568]
	str	wzr, [x22, #572]
	bl	mutex_unlock
	adrp	x0, .L.str.20
	adrp	x1, .L__func__.mas_blk_recovery_done_proc
	add	x0, x0, :lo12:.L.str.20
	add	x1, x1, :lo12:.L__func__.mas_blk_recovery_done_proc
	mov	w2, w24
	bl	printk
	cbnz	w24, .LBB21_92
.LBB21_40:
	mov	x0, x20
	stur	xzr, [x29, #-96]
	bl	mas_blk_get_lld
	cbz	x0, .LBB21_91
// %bb.41:
	ldr	x8, [x20, #1376]
	ldr	x8, [x8, #536]
	ldr	x9, [x8, #136]!
	cmp	x8, x9
	b.eq	.LBB21_77
// %bb.42:
	mov	x0, x20
	bl	mas_blk_get_lld
	cbz	x0, .LBB21_91
// %bb.43:
	ldr	w8, [x0, #236]
	cbz	w8, .LBB21_91
// %bb.44:
	ldr	x22, [x0, #1440]
	cbz	x22, .LBB21_91
// %bb.45:
	str	x0, [sp, #64]           // 8-byte Folded Spill
	add	x0, x22, #152           // =152
	stp	x0, x27, [sp, #80]      // 16-byte Folded Spill
	bl	_raw_spin_lock_irqsave
	add	x8, x22, #136           // =136
	mov	w24, #4096
	mov	w2, #1
	mov	w21, #680
	str	x0, [sp, #72]           // 8-byte Folded Spill
	stp	x22, x8, [sp, #96]      // 16-byte Folded Spill
	b	.LBB21_47
.LBB21_46:                              //   in Loop: Header=BB21_47 Depth=1
	str	x26, [x26]
	ldr	x22, [sp, #96]          // 8-byte Folded Reload
	str	x26, [x26, #8]
	mov	w2, #1
.LBB21_47:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB21_54 Depth 2
                                        //       Child Loop BB21_60 Depth 3
	ldr	x8, [sp, #104]          // 8-byte Folded Reload
	ldr	x26, [x8]
	cmp	x26, x8
	b.ne	.LBB21_49
// %bb.48:                              //   in Loop: Header=BB21_47 Depth=1
	ldr	x8, [x22, #144]
	ldr	x9, [sp, #104]          // 8-byte Folded Reload
	cmp	x8, x9
	b.eq	.LBB21_90
.LBB21_49:                              //   in Loop: Header=BB21_47 Depth=1
	add	x8, x22, #32            // =32
	sub	x9, x26, #392           // =392
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	add	w10, w10, 1
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	ldr	x8, [x9]
	sub	x1, x26, #400           // =400
	ldr	x0, [x8, #1264]
	mov	x8, #1099511627776
	stp	xzr, xzr, [x29, #-32]
	stp	xzr, xzr, [x29, #-48]
	stp	xzr, xzr, [x29, #-64]
	stur	xzr, [x29, #-88]
	stp	x8, xzr, [x29, #-80]
	sub	x8, x26, #384           // =384
	ldr	w8, [x8]
	sub	w9, w8, #3              // =3
	and	w10, w9, #0xff
	cmp	w10, #7                 // =7
	b.hs	.LBB21_51
// %bb.50:                              //   in Loop: Header=BB21_47 Depth=1
	mov	w10, #85
	lsr	w10, w10, w9
	tbnz	w10, #0, .LBB21_63
.LBB21_51:                              //   in Loop: Header=BB21_47 Depth=1
	sub	x9, x26, #360           // =360
	ldr	w9, [x9]
	cbz	w9, .LBB21_62
// %bb.52:                              //   in Loop: Header=BB21_47 Depth=1
	sub	x10, x26, #352          // =352
	sub	x12, x26, #356          // =356
	ldr	w11, [x10]
	ldr	w12, [x12]
	ldr	x10, [x26, #72]
	mov	w23, wzr
	b	.LBB21_54
.LBB21_53:                              //   in Loop: Header=BB21_54 Depth=2
	cbz	w9, .LBB21_64
.LBB21_54:                              //   Parent Loop BB21_47 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB21_60 Depth 3
	add	x13, x10, w12, uxtw #4
	ldp	w14, w15, [x13, #8]
	and	w16, w8, #0xff
	add	w23, w23, #1            // =1
	add	w15, w15, w11
	sub	w13, w14, w11
	cmp	w9, w13
	and	w15, w15, #0xfff
	csel	w17, w9, w13, lo
	sub	w15, w24, w15
	cmp	w17, w15
	csel	w15, w17, w15, lo
	cmp	w16, #9                 // =9
	b.hi	.LBB21_57
// %bb.55:                              //   in Loop: Header=BB21_54 Depth=2
	lsl	w16, w2, w16
	tst	w16, w21
	b.eq	.LBB21_57
// %bb.56:                              //   in Loop: Header=BB21_54 Depth=2
	sub	w9, w9, w15
	cbnz	w9, .LBB21_54
	b	.LBB21_64
.LBB21_57:                              //   in Loop: Header=BB21_54 Depth=2
	cbz	w15, .LBB21_53
// %bb.58:                              //   in Loop: Header=BB21_54 Depth=2
	cmp	w9, w15
	csel	w16, w15, w9, hi
	cmp	w16, w13
	csel	w16, w16, w13, lo
	add	w11, w16, w11
	cmp	w11, w14
	sub	w13, w15, w16
	sub	w9, w9, w16
	cinc	w14, w12, eq
	csel	w11, wzr, w11, eq
	cbz	w13, .LBB21_61
// %bb.59:                              //   in Loop: Header=BB21_54 Depth=2
	mov	w12, w14
.LBB21_60:                              //   Parent Loop BB21_47 Depth=1
                                        //     Parent Loop BB21_54 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	add	x15, x10, w14, uxtw #4
	ldr	w15, [x15, #8]
	cmp	w13, w9
	csel	w16, w13, w9, lo
	sub	w17, w15, w11
	cmp	w16, w17
	csel	w16, w16, w17, lo
	add	w11, w16, w11
	cmp	w11, w15
	sub	w13, w13, w16
	sub	w9, w9, w16
	csinc	w12, w12, w14, ne
	csel	w11, wzr, w11, eq
	cinc	w14, w14, eq
	cbnz	w13, .LBB21_60
	b	.LBB21_53
.LBB21_61:                              //   in Loop: Header=BB21_54 Depth=2
	mov	w12, w14
	cbnz	w9, .LBB21_54
	b	.LBB21_64
.LBB21_62:                              //   in Loop: Header=BB21_47 Depth=1
	mov	w23, wzr
	b	.LBB21_64
.LBB21_63:                              //   in Loop: Header=BB21_47 Depth=1
	adrp	x10, .Lswitch.table.ufs_mq_sync_dispatch
	sxtb	x9, w9
	add	x10, x10, :lo12:.Lswitch.table.ufs_mq_sync_dispatch
	ldr	w23, [x10, x9, lsl #2]
.LBB21_64:                              //   in Loop: Header=BB21_47 Depth=1
	sub	x2, x29, #88            // =88
	stur	w8, [x29, #-52]
	mov	x25, x0
	mov	x27, x1
	bl	blk_mq_get_request
	ldr	x9, [x0, #200]
	ldr	w8, [x0, #24]
	mov	w10, #61439
	movk	w10, #65471, lsl #16
	and	x9, x9, #0xffffffffffffff7f
	and	w10, w8, w10
	str	x9, [x0, #200]
	str	w10, [x0, #24]
	cbz	x0, .LBB21_81
// %bb.65:                              //   in Loop: Header=BB21_47 Depth=1
	and	w8, w8, #0xff
	mov	x22, x0
	cmp	w8, #1                  // =1
	b.eq	.LBB21_69
// %bb.66:                              //   in Loop: Header=BB21_47 Depth=1
	cmp	w8, #3                  // =3
	b.eq	.LBB21_69
// %bb.67:                              //   in Loop: Header=BB21_47 Depth=1
	cmp	w8, #2                  // =2
	b.ne	.LBB21_70
// %bb.68:                              //   in Loop: Header=BB21_47 Depth=1
	ldr	x0, [x22]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB21_70
.LBB21_69:                              //   in Loop: Header=BB21_47 Depth=1
	mov	x0, x22
	bl	blk_req_set_make_req_nr
.LBB21_70:                              //   in Loop: Header=BB21_47 Depth=1
	mov	x0, x22
	mov	x1, x27
	mov	w2, w23
	bl	blk_mq_bio_to_request
	ldr	w8, [x22, #24]
	ldr	x9, [x22, #8]
	tst	w8, #0xff
	cset	w10, eq
	tst	w8, #0x8000000
	mov	w8, #2
	csel	x8, x10, x8, eq
	add	x8, x9, x8, lsl #3
	ldr	x23, [x8, #80]
	stur	x22, [x29, #-32]
	ldr	x8, [x23, #232]
	ldr	x8, [x8, #48]
	ldr	x8, [x8, #16]
	cbz	x8, .LBB21_72
// %bb.71:                              //   in Loop: Header=BB21_47 Depth=1
	mov	x0, x23
	blr	x8
	tbz	w0, #0, .LBB21_73
.LBB21_72:                              //   in Loop: Header=BB21_47 Depth=1
	ldr	x8, [x25, #48]
	sub	x1, x29, #32            // =32
	mov	x0, x23
	ldr	x8, [x8]
	blr	x8
	ldur	x8, [x29, #-32]
	mov	w23, w0
	mov	w1, #10
	mov	x0, x8
	bl	mas_blk_latency_req_check
	ands	w23, w23, #0xff
	b.ne	.LBB21_74
	b	.LBB21_75
.LBB21_73:                              //   in Loop: Header=BB21_47 Depth=1
	ldur	x0, [x29, #-32]
	mov	w1, #11
	bl	mas_blk_latency_req_check
	mov	w23, #9
.LBB21_74:                              //   in Loop: Header=BB21_47 Depth=1
	mov	x0, x22
	bl	blk_mq_free_request
	cbnz	w23, .LBB21_81
.LBB21_75:                              //   in Loop: Header=BB21_47 Depth=1
	mov	x0, x26
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB21_46
// %bb.76:                              //   in Loop: Header=BB21_47 Depth=1
	ldp	x9, x8, [x26]
	str	x8, [x9, #8]
	str	x9, [x8]
	b	.LBB21_46
.LBB21_77:
	ldr	x8, [x0, #216]
	mov	x22, x0
	sub	x1, x29, #96            // =96
	mov	x0, x20
	blr	x8
	ldur	x8, [x29, #-96]
	cbnz	x8, .LBB21_91
// %bb.78:
	add	x23, x22, #624          // =624
	mov	x0, x23
	bl	mutex_lock
	ldr	w8, [x22, #564]
	cbz	w8, .LBB21_80
// %bb.79:
	add	x0, x22, #576           // =576
	str	wzr, [x22, #564]
	bl	up_write
.LBB21_80:
	mov	x0, x23
	bl	mutex_unlock
	b	.LBB21_91
.LBB21_81:
	ldr	x12, [sp, #96]          // 8-byte Folded Reload
	mov	w9, #1
	add	x8, x12, #32            // =32
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	ldr	x22, [x12, #136]
	ldr	x8, [sp, #104]          // 8-byte Folded Reload
	ldr	x27, [sp, #88]          // 8-byte Folded Reload
	ldr	x21, [sp, #64]          // 8-byte Folded Reload
	cmp	x8, x22
	b.ne	.LBB21_83
	b	.LBB21_89
.LBB21_82:                              //   in Loop: Header=BB21_83 Depth=1
	ldr	x22, [x22]
	ldr	x8, [sp, #104]          // 8-byte Folded Reload
	cmp	x8, x22
	b.eq	.LBB21_89
.LBB21_83:                              // =>This Inner Loop Header: Depth=1
	ldurb	w23, [x22, #-136]
	sub	w8, w23, #1             // =1
	and	w8, w8, #0xff
	cmp	w8, #3                  // =3
	b.hi	.LBB21_82
// %bb.84:                              //   in Loop: Header=BB21_83 Depth=1
	sub	x8, x22, #368           // =368
	ldr	x8, [x8]
	add	x9, x21, x23, lsl #2
	add	x27, x9, #316           // =316
	mov	x0, x27
	lsr	x26, x8, #3
	bl	_raw_spin_lock_irqsave
	mov	x25, x0
	mov	x0, x21
	mov	w1, w23
	mov	x2, x26
	bl	mas_blk_expected_lba_pu
	mov	w23, w0
	mov	x0, x27
	ldr	x27, [sp, #88]          // 8-byte Folded Reload
	mov	x1, x25
	bl	_raw_spin_unlock_irqrestore
	tbz	w23, #0, .LBB21_82
// %bb.85:
	mov	x0, x22
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB21_87
// %bb.86:
	ldp	x9, x8, [x22]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB21_87:
	str	x22, [x22]
	ldr	x1, [sp, #104]          // 8-byte Folded Reload
	str	x22, [x22, #8]
	mov	x0, x22
	ldr	x23, [x1]
	mov	x2, x23
	bl	__list_add_valid
	tbz	w0, #0, .LBB21_89
// %bb.88:
	ldr	x8, [sp, #104]          // 8-byte Folded Reload
	str	x22, [x23, #8]
	stp	x23, x8, [x22]
	str	x22, [x8]
.LBB21_89:
	ldp	x1, x0, [sp, #72]       // 16-byte Folded Reload
	bl	_raw_spin_unlock_irqrestore
	b	.LBB21_91
.LBB21_90:
	ldp	x1, x0, [sp, #72]       // 16-byte Folded Reload
	bl	_raw_spin_unlock_irqrestore
	ldr	x27, [sp, #88]          // 8-byte Folded Reload
.LBB21_91:
	ldr	w8, [x27, #564]
	cbz	w8, .LBB21_93
.LBB21_92:
	ldr	x19, [x20, #1376]
	adrp	x8, mas_blk_mq_sync_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_sync_disp_wq]
	mov	w0, #8
	add	x2, x19, #120           // =120
	mov	w3, #1
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x19, #344]
	mov	w9, #26215
	movk	w9, #26214, lsl #16
	mul	x9, x8, x9
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	mov	w10, #10
	msub	w8, w9, w10, w8
	add	x8, x19, w8, sxtw #3
	str	x0, [x8, #8]
	ldr	w8, [x19, #88]
	add	w8, w8, #1              // =1
	str	w8, [x19, #88]
	b	.LBB21_112
.LBB21_93:
	bl	ktime_get
	str	x28, [sp, #104]         // 8-byte Folded Spill
	str	x0, [x28, #96]
	add	x21, x19, #48           // =48
	add	x25, x19, #56           // =56
	add	x26, x19, #80           // =80
	mov	w27, #1
	mov	w28, #2
.LBB21_94:                              // =>This Inner Loop Header: Depth=1
	mov	x0, x21
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x25]
	mov	x23, x0
	cmp	x8, x25
	b.eq	.LBB21_96
.LBB21_95:                              //   in Loop: Header=BB21_94 Depth=1
	add	x8, x19, #72            // =72
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w27
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	mov	x8, x25
	b	.LBB21_100
.LBB21_96:                              //   in Loop: Header=BB21_94 Depth=1
	ldr	x8, [x19, #64]
	cmp	x8, x25
	b.ne	.LBB21_95
// %bb.97:                              //   in Loop: Header=BB21_94 Depth=1
	ldr	x8, [x26]
	cmp	x8, x26
	b.ne	.LBB21_99
// %bb.98:                              //   in Loop: Header=BB21_94 Depth=1
	ldr	x8, [x19, #88]
	cmp	x8, x26
	b.eq	.LBB21_110
.LBB21_99:                              //   in Loop: Header=BB21_94 Depth=1
	add	x8, x19, #96            // =96
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w27
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	mov	x8, x26
.LBB21_100:                             //   in Loop: Header=BB21_94 Depth=1
	ldr	x24, [x8]
	subs	x22, x24, #80           // =80
	b.eq	.LBB21_110
// %bb.101:                             //   in Loop: Header=BB21_94 Depth=1
	ldur	x20, [x24, #-80]
	mov	x0, x24
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB21_103
// %bb.102:                             //   in Loop: Header=BB21_94 Depth=1
	ldp	x9, x8, [x24]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB21_103:                             //   in Loop: Header=BB21_94 Depth=1
	mov	x0, x21
	mov	x1, x23
	str	x24, [x24]
	str	x24, [x24, #8]
	bl	_raw_spin_unlock_irqrestore
	ldur	w8, [x24, #-56]
	ldur	x9, [x24, #-72]
	tst	w8, #0xff
	cset	w10, eq
	tst	w8, #0x8000000
	csel	x10, x10, x28, eq
	add	x9, x9, x10, lsl #3
	ldr	x23, [x9, #80]
	and	w8, w8, #0xfc
	orr	w8, w8, #0x3
	cmp	w8, #35                 // =35
	str	x22, [sp, #112]
	b.eq	.LBB21_107
// %bb.104:                             //   in Loop: Header=BB21_94 Depth=1
	add	x3, sp, #112            // =112
	mov	x0, x22
	mov	x1, x23
	mov	x2, x19
	mov	x4, x20
	bl	__ufs_mq_queue_rq
	cbz	w0, .LBB21_94
.LBB21_105:                             //   in Loop: Header=BB21_94 Depth=1
	cmp	w0, #9                  // =9
	b.eq	.LBB21_115
// %bb.106:                             //   in Loop: Header=BB21_94 Depth=1
	mov	w1, #10
	mov	x0, x22
	bl	blk_mq_end_request
	b	.LBB21_94
.LBB21_107:                             //   in Loop: Header=BB21_94 Depth=1
	ldr	x8, [x23, #232]
	ldr	x8, [x8, #48]
	ldr	x8, [x8, #16]
	cbz	x8, .LBB21_109
// %bb.108:                             //   in Loop: Header=BB21_94 Depth=1
	mov	x0, x23
	blr	x8
	tbz	w0, #0, .LBB21_114
.LBB21_109:                             //   in Loop: Header=BB21_94 Depth=1
	ldr	x8, [x20, #48]
	add	x1, sp, #112            // =112
	mov	x0, x23
	ldr	x8, [x8]
	blr	x8
	ldr	x8, [sp, #112]
	mov	w23, w0
	mov	w1, #10
	mov	x0, x8
	bl	mas_blk_latency_req_check
	and	w0, w23, #0xff
	cbz	w0, .LBB21_94
	b	.LBB21_105
.LBB21_110:
	mov	x0, x21
	mov	x1, x23
	bl	_raw_spin_unlock_irqrestore
	ldr	x28, [sp, #104]         // 8-byte Folded Reload
.LBB21_111:
	ldr	w8, [x28, #116]
	add	w8, w8, #1              // =1
	str	w8, [x28, #116]
	bl	ktime_get
	str	x0, [x28, #104]
.LBB21_112:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB21_143
// %bb.113:
	ldp	x20, x19, [sp, #304]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #288]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #272]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #256]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #240]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #224]    // 16-byte Folded Reload
	add	sp, sp, #320            // =320
	ret
.LBB21_114:
	ldr	x0, [sp, #112]
	mov	w1, #11
	bl	mas_blk_latency_req_check
.LBB21_115:
	mov	x0, x22
	bl	__blk_mq_requeue_request
	mov	x0, x22
	mov	x1, x20
	bl	ufs_mq_insert_sync_list
	ldr	x8, [x19]
	mov	x0, x20
	str	x8, [sp, #88]           // 8-byte Folded Spill
	bl	blk_queue_query_unistore_enable
	ldr	x28, [sp, #104]         // 8-byte Folded Reload
	tbz	w0, #0, .LBB21_137
// %bb.116:
	ldr	x8, [x26]
	cmp	x8, x26
	b.ne	.LBB21_118
// %bb.117:
	ldr	x8, [x19, #88]
	cmp	x8, x26
	b.eq	.LBB21_137
.LBB21_118:
	//APP
	.if 1 == 1
661:
	mrs	x22, daif
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
	mrs_s x22, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w8, w22, #0x00000080
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
	eor	w8, w22, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w8, .LBB21_120
// %bb.119:
	mov	w8, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x8
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB21_120:
	mov	x0, x21
	bl	_raw_spin_trylock
	cbz	w0, .LBB21_136
// %bb.121:
	ldr	x8, [sp, #88]           // 8-byte Folded Reload
	ldp	w2, w3, [x8, #232]
	cbz	w2, .LBB21_142
// %bb.122:
	cbz	w3, .LBB21_142
// %bb.123:
	ldr	x24, [x26]
	str	x22, [sp, #80]          // 8-byte Folded Spill
	cmp	x26, x24
	b.eq	.LBB21_134
// %bb.124:
	ldr	x22, [sp, #88]          // 8-byte Folded Reload
	b	.LBB21_126
.LBB21_125:                             //   in Loop: Header=BB21_126 Depth=1
	ldr	x24, [x24]
	cmp	x26, x24
	b.eq	.LBB21_134
.LBB21_126:                             // =>This Inner Loop Header: Depth=1
	ldrb	w25, [x24, #568]
	sub	w8, w25, #1             // =1
	and	w8, w8, #0xff
	cmp	w8, #3                  // =3
	b.hi	.LBB21_125
// %bb.127:                             //   in Loop: Header=BB21_126 Depth=1
	ldurb	w8, [x24, #-56]
	cmp	w8, #1                  // =1
	b.ne	.LBB21_125
// %bb.128:                             //   in Loop: Header=BB21_126 Depth=1
	ldur	x8, [x24, #-24]
	add	x9, x22, x25, lsl #2
	add	x27, x9, #316           // =316
	mov	x0, x27
	lsr	x23, x8, #3
	bl	_raw_spin_lock_irqsave
	str	x0, [sp, #96]           // 8-byte Folded Spill
	mov	x0, x22
	mov	w1, w25
	mov	x2, x23
	bl	mas_blk_expected_lba_pu
	ldr	x1, [sp, #96]           // 8-byte Folded Reload
	mov	w25, w0
	mov	x0, x27
	bl	_raw_spin_unlock_irqrestore
	tbz	w25, #0, .LBB21_125
// %bb.129:
	mov	x0, x24
	sub	x23, x24, #80           // =80
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB21_131
// %bb.130:
	ldp	x9, x8, [x24]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB21_131:
	str	x24, [x24]
	ldr	x1, [sp, #80]           // 8-byte Folded Reload
	mov	x0, x21
	str	x24, [x24, #8]
	bl	_raw_spin_unlock_irqrestore
	stp	xzr, xzr, [x29, #-88]
	ldur	w8, [x24, #-56]
	ldur	x10, [x24, #-72]
	mov	w9, #2
	tst	w8, #0xff
	cset	w11, eq
	tst	w8, #0x8000000
	csel	x9, x11, x9, eq
	add	x9, x10, x9, lsl #3
	ldr	x21, [x9, #80]
	and	w8, w8, #0xfc
	orr	w8, w8, #0x3
	cmp	w8, #35                 // =35
	stur	x23, [x29, #-88]
	b.eq	.LBB21_144
// %bb.132:
	sub	x3, x29, #88            // =88
	mov	x0, x23
	mov	x1, x21
	mov	x2, x19
	mov	x4, x20
	bl	__ufs_mq_queue_rq
	cmp	w0, #9                  // =9
	b.ne	.LBB21_137
.LBB21_133:
	mov	x0, x23
	bl	__blk_mq_requeue_request
	mov	x0, x23
	mov	x1, x20
	bl	ufs_mq_insert_sync_list
	b	.LBB21_137
.LBB21_134:
	ldr	x1, [sp, #80]           // 8-byte Folded Reload
	mov	x0, x21
.LBB21_135:
	bl	_raw_spin_unlock_irqrestore
	b	.LBB21_137
.LBB21_136:
	//APP
	.if 1 == 1
661:
	msr	daif, x22
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x22
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB21_137:
	mov	w21, #37888
	movk	w21, #30517, lsl #16
	bl	ktime_get
	ldr	x8, [x19, #184]
	add	x8, x8, x21
	cmp	x8, x0
	b.ge	.LBB21_140
// %bb.138:
	bl	ktime_get
	ldr	x8, [x19, #192]
	add	x8, x8, x21
	cmp	x8, x0
	b.ge	.LBB21_140
// %bb.139:
	mov	w3, #1
	b	.LBB21_141
.LBB21_140:
	mov	x3, xzr
.LBB21_141:
	ldr	x19, [x20, #1376]
	adrp	x8, mas_blk_mq_sync_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_sync_disp_wq]
	mov	w0, #8
	add	x2, x19, #120           // =120
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x19, #344]
	mov	w9, #26215
	movk	w9, #26214, lsl #16
	mul	x9, x8, x9
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	mov	w10, #10
	msub	w8, w9, w10, w8
	add	x8, x19, w8, sxtw #3
	str	x0, [x8, #8]
	ldr	w8, [x19, #88]
	add	w8, w8, #1              // =1
	str	w8, [x19, #88]
	ldr	w8, [x28, #112]
	add	w8, w8, #1              // =1
	str	w8, [x28, #112]
	b	.LBB21_111
.LBB21_142:
	adrp	x0, .L.str.24
	adrp	x1, .L__func__.ufs_mq_dispatch_match_expected_lba
	add	x0, x0, :lo12:.L.str.24
	add	x1, x1, :lo12:.L__func__.ufs_mq_dispatch_match_expected_lba
	bl	printk
	mov	x0, x21
	mov	x1, x22
	b	.LBB21_135
.LBB21_143:
	bl	__stack_chk_fail
.LBB21_144:
	ldr	x8, [x21, #232]
	ldr	x8, [x8, #48]
	ldr	x8, [x8, #16]
	cbz	x8, .LBB21_146
// %bb.145:
	mov	x0, x21
	blr	x8
	tbz	w0, #0, .LBB21_147
.LBB21_146:
	ldr	x8, [x20, #48]
	sub	x1, x29, #88            // =88
	mov	x0, x21
	ldr	x8, [x8]
	blr	x8
	ldur	x8, [x29, #-88]
	mov	w21, w0
	mov	w1, #10
	mov	x0, x8
	bl	mas_blk_latency_req_check
	ldr	x28, [sp, #104]         // 8-byte Folded Reload
	and	w0, w21, #0xff
	cmp	w0, #9                  // =9
	b.eq	.LBB21_133
	b	.LBB21_137
.LBB21_147:
	ldur	x0, [x29, #-88]
	mov	w1, #11
	bl	mas_blk_latency_req_check
	ldr	x28, [sp, #104]         // 8-byte Folded Reload
	b	.LBB21_133
.Lfunc_end21:
	.size	ufs_mq_sync_dispatch, .Lfunc_end21-ufs_mq_sync_dispatch
                                        // -- End function
	.globl	ufs_mq_write_throttle_check_timer_expire // -- Begin function ufs_mq_write_throttle_check_timer_expire
	.p2align	2
	.type	ufs_mq_write_throttle_check_timer_expire,@function
ufs_mq_write_throttle_check_timer_expire: // @ufs_mq_write_throttle_check_timer_expire
.Lufs_mq_write_throttle_check_timer_expire$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	add	x20, x0, #328           // =328
	mov	x19, x0
	mov	x0, x20
	mov	x29, sp
	bl	_raw_spin_lock_irqsave
	mov	x1, x0
	mov	w8, #1
	mov	x0, x20
	strb	w8, [x19, #332]
	bl	_raw_spin_unlock_irqrestore
	mov	x0, x19
	bl	ufs_mq_async_disp_inflt_lmt_decision
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end22:
	.size	ufs_mq_write_throttle_check_timer_expire, .Lfunc_end22-ufs_mq_write_throttle_check_timer_expire
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_async_disp_inflt_lmt_decision
	.type	ufs_mq_async_disp_inflt_lmt_decision,@function
ufs_mq_async_disp_inflt_lmt_decision:   // @ufs_mq_async_disp_inflt_lmt_decision
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	str	wzr, [x0, #268]
	mov	x19, x0
	ldr	w8, [x8, #1476]
	ldr	w10, [x0, #20]
	ldr	w9, [x0, #264]
	mov	x29, sp
	sub	w8, w8, w10
	cmp	w9, #3                  // =3
	b.hi	.LBB23_5
// %bb.1:
	adrp	x10, .LJTI23_0
	add	x10, x10, :lo12:.LJTI23_0
	adr	x11, .LBB23_2
	ldrb	w12, [x10, x9]
	add	x11, x11, x12, lsl #2
	br	x11
.LBB23_2:
	ldr	w8, [x19, #160]
	cmp	w8, #14                 // =14
	b.eq	.LBB23_13
// %bb.3:
	add	x20, x19, #328          // =328
	mov	x0, x20
	bl	_raw_spin_lock_irqsave
	ldrb	w8, [x19, #332]
	mov	x1, x0
	cbz	w8, .LBB23_12
// %bb.4:
	mov	w8, #14
	b	.LBB23_11
.LBB23_5:
	cmp	w8, #4                  // =4
	ldr	w8, [x19, #160]
	mov	w9, #14
	mov	w10, #16
	csel	w21, w10, w9, lo
	cmp	w8, w21
	b.eq	.LBB23_13
// %bb.6:
	add	x20, x19, #328          // =328
	mov	x0, x20
	bl	_raw_spin_lock_irqsave
	ldrb	w8, [x19, #332]
	mov	x1, x0
	cbz	w8, .LBB23_12
// %bb.7:
	str	w21, [x19, #160]
	b	.LBB23_12
.LBB23_8:
	ldr	w8, [x19, #160]
	cmp	w8, #10                 // =10
	b.eq	.LBB23_13
// %bb.9:
	add	x20, x19, #328          // =328
	mov	x0, x20
	bl	_raw_spin_lock_irqsave
	ldrb	w8, [x19, #332]
	mov	x1, x0
	cbz	w8, .LBB23_12
// %bb.10:
	mov	w8, #10
.LBB23_11:
	str	w8, [x19, #160]
.LBB23_12:
	mov	x0, x20
	bl	_raw_spin_unlock_irqrestore
	bl	ktime_get
	str	x0, [x19, #200]
.LBB23_13:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end23:
	.size	ufs_mq_async_disp_inflt_lmt_decision, .Lfunc_end23-ufs_mq_async_disp_inflt_lmt_decision
	.section	.rodata,"a",@progbits
.LJTI23_0:
	.byte	(.LBB23_5-.LBB23_2)>>2
	.byte	(.LBB23_2-.LBB23_2)>>2
	.byte	(.LBB23_8-.LBB23_2)>>2
	.byte	(.LBB23_2-.LBB23_2)>>2
                                        // -- End function
	.text
	.globl	ufs_mq_write_throttle_handler // -- Begin function ufs_mq_write_throttle_handler
	.p2align	2
	.type	ufs_mq_write_throttle_handler,@function
ufs_mq_write_throttle_handler:          // @ufs_mq_write_throttle_handler
.Lufs_mq_write_throttle_handler$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #1376]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	w19, w1
	ldr	x21, [x8, #536]
	add	x20, x21, #328          // =328
	mov	x0, x20
	bl	_raw_spin_lock_irqsave
	tst	w19, #0x1
	mov	w8, #5
	mov	w9, #20
	mov	x1, x0
	csel	w8, w9, w8, ne
	mov	x0, x20
	strb	wzr, [x21, #332]
	str	w8, [x21, #160]
	bl	_raw_spin_unlock_irqrestore
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	add	x0, x21, #272           // =272
	add	x1, x8, #250            // =250
	bl	mod_timer
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end24:
	.size	ufs_mq_write_throttle_handler, .Lfunc_end24-ufs_mq_write_throttle_handler
                                        // -- End function
	.globl	ufs_mq_async_io_dispatch_work_fn // -- Begin function ufs_mq_async_io_dispatch_work_fn
	.p2align	2
	.type	ufs_mq_async_io_dispatch_work_fn,@function
ufs_mq_async_io_dispatch_work_fn:       // @ufs_mq_async_io_dispatch_work_fn
.Lufs_mq_async_io_dispatch_work_fn$local:
// %bb.0:
	sub	sp, sp, #128            // =128
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	add	x29, sp, #32            // =32
	str	x27, [sp, #48]          // 8-byte Folded Spill
	stp	x26, x25, [sp, #64]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #80]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #96]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #112]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x25, [x0, #136]
	stp	xzr, xzr, [sp, #8]
	ldr	x24, [x25, #1376]
	ldr	x19, [x24, #536]
	bl	ktime_get
	str	x0, [x24, #352]
	mov	x0, x19
	bl	ufs_mq_async_disp_inflt_lmt_decision
	ldr	x8, [x25, #1384]
	cbz	x8, .LBB25_14
// %bb.1:
	mov	w26, #1
	mov	w27, #2
.LBB25_2:                               // =>This Inner Loop Header: Depth=1
	ldr	x20, [x8, #176]
	cbz	x20, .LBB25_14
// %bb.3:                               //   in Loop: Header=BB25_2 Depth=1
	ldr	x8, [x20]
	ldr	x8, [x8, #24]
	cbz	x8, .LBB25_14
// %bb.4:                               //   in Loop: Header=BB25_2 Depth=1
	ldr	x8, [x25, #1376]
	ldr	x22, [x8, #536]
	add	x21, x22, #100          // =100
	mov	x0, x21
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x20]
	mov	x23, x0
	mov	x0, x22
	ldr	x8, [x8, #24]
	blr	x8
	cbz	x0, .LBB25_13
// %bb.5:                               //   in Loop: Header=BB25_2 Depth=1
	add	x8, x22, #128           // =128
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w26
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	mov	x20, x0
	mov	x0, x21
	mov	x1, x23
	bl	_raw_spin_unlock_irqrestore
	ldr	w8, [x20, #24]
	ldp	x21, x9, [x20]
	tst	w8, #0xff
	cset	w10, eq
	tst	w8, #0x8000000
	csel	x10, x10, x27, eq
	add	x9, x9, x10, lsl #3
	ldr	x22, [x9, #80]
	and	w8, w8, #0xfc
	orr	w8, w8, #0x3
	cmp	w8, #35                 // =35
	str	x20, [sp, #8]
	b.eq	.LBB25_8
// %bb.6:                               //   in Loop: Header=BB25_2 Depth=1
	add	x3, sp, #8              // =8
	mov	x0, x20
	mov	x1, x22
	mov	x2, x19
	mov	x4, x21
	bl	__ufs_mq_queue_rq
	cbnz	w0, .LBB25_11
.LBB25_7:                               //   in Loop: Header=BB25_2 Depth=1
	mov	x0, x19
	bl	ufs_mq_async_disp_inflt_lmt_decision
	ldr	x8, [x25, #1384]
	cbnz	x8, .LBB25_2
	b	.LBB25_14
.LBB25_8:                               //   in Loop: Header=BB25_2 Depth=1
	ldr	x8, [x22, #232]
	ldr	x8, [x8, #48]
	ldr	x8, [x8, #16]
	cbz	x8, .LBB25_10
// %bb.9:                               //   in Loop: Header=BB25_2 Depth=1
	mov	x0, x22
	blr	x8
	tbz	w0, #0, .LBB25_16
.LBB25_10:                              //   in Loop: Header=BB25_2 Depth=1
	ldr	x8, [x21, #48]
	add	x1, sp, #8              // =8
	mov	x0, x22
	ldr	x8, [x8]
	blr	x8
	ldr	x8, [sp, #8]
	mov	w22, w0
	mov	w1, #10
	mov	x0, x8
	bl	mas_blk_latency_req_check
	and	w0, w22, #0xff
	cbz	w0, .LBB25_7
.LBB25_11:                              //   in Loop: Header=BB25_2 Depth=1
	cmp	w0, #9                  // =9
	b.eq	.LBB25_17
// %bb.12:                              //   in Loop: Header=BB25_2 Depth=1
	mov	w1, #10
	mov	x0, x20
	bl	blk_mq_end_request
	b	.LBB25_7
.LBB25_13:
	mov	x0, x21
	mov	x1, x23
	bl	_raw_spin_unlock_irqrestore
.LBB25_14:
	bl	ktime_get
	str	x0, [x24, #360]
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB25_22
// %bb.15:
	ldp	x20, x19, [sp, #112]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #96]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #80]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #64]     // 16-byte Folded Reload
	ldr	x27, [sp, #48]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #128            // =128
	ret
.LBB25_16:
	ldr	x0, [sp, #8]
	mov	w1, #11
	bl	mas_blk_latency_req_check
.LBB25_17:
	mov	x0, x20
	bl	__blk_mq_requeue_request
	ldr	x8, [x21, #1384]
	ldr	x19, [x21, #1376]
	cbz	x8, .LBB25_21
// %bb.18:
	ldr	x23, [x8, #176]
	cbz	x23, .LBB25_21
// %bb.19:
	ldr	x8, [x23]
	ldr	x8, [x8, #32]
	cbz	x8, .LBB25_21
// %bb.20:
	ldr	x19, [x19, #536]
	mov	w1, #9
	mov	x0, x20
	bl	mas_blk_latency_req_check
	add	x22, x19, #100          // =100
	mov	x0, x22
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x23]
	mov	x23, x0
	mov	x0, x20
	mov	x1, x19
	ldr	x8, [x8, #32]
	blr	x8
	add	x8, x19, #128           // =128
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	mov	x0, x22
	mov	x1, x23
	bl	_raw_spin_unlock_irqrestore
	ldr	x19, [x21, #1376]
.LBB25_21:
	adrp	x8, mas_blk_mq_async_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_async_disp_wq]
	add	x2, x19, #376           // =376
	mov	w0, #8
	mov	w3, #1
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x19, #344]
	mov	w9, #26215
	movk	w9, #26214, lsl #16
	mul	x9, x8, x9
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	mov	w10, #10
	msub	w8, w9, w10, w8
	add	x8, x19, w8, sxtw #3
	str	x0, [x8, #264]
	ldr	w8, [x19, #344]
	add	w8, w8, #1              // =1
	str	w8, [x19, #344]
	b	.LBB25_14
.LBB25_22:
	bl	__stack_chk_fail
.Lfunc_end25:
	.size	ufs_mq_async_io_dispatch_work_fn, .Lfunc_end25-ufs_mq_async_io_dispatch_work_fn
                                        // -- End function
	.globl	__ufs_mq_complete_request_remote // -- Begin function __ufs_mq_complete_request_remote
	.p2align	2
	.type	__ufs_mq_complete_request_remote,@function
__ufs_mq_complete_request_remote:       // @__ufs_mq_complete_request_remote
.L__ufs_mq_complete_request_remote$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	mov	w1, #26
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x19, x0
	bl	mas_blk_latency_req_check
	ldr	x8, [x19]
	mov	x0, x19
	ldr	x8, [x8, #48]
	ldr	x8, [x8, #48]
	blr	x8
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end26:
	.size	__ufs_mq_complete_request_remote, .Lfunc_end26-__ufs_mq_complete_request_remote
                                        // -- End function
	.globl	ufs_mq_io_guard_work_fn // -- Begin function ufs_mq_io_guard_work_fn
	.p2align	2
	.type	ufs_mq_io_guard_work_fn,@function
ufs_mq_io_guard_work_fn:                // @ufs_mq_io_guard_work_fn
.Lufs_mq_io_guard_work_fn$local:
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	adrp	x19, io_guard_queue_list_lock
	add	x19, x19, :lo12:io_guard_queue_list_lock
	mov	x0, x19
	stp	x26, x25, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	mov	x29, sp
	bl	_raw_spin_lock
	adrp	x20, mas_io_guard_queue_list
	add	x20, x20, :lo12:mas_io_guard_queue_list
	ldr	x21, [x20]
	mov	x0, x19
	bl	_raw_spin_unlock
	cmp	x21, x20
	b.eq	.LBB27_9
// %bb.1:
	adrp	x19, io_guard_queue_list_lock
	mov	w23, #26215
	add	x19, x19, :lo12:io_guard_queue_list_lock
	adrp	x22, mas_blk_mq_async_disp_wq
	movk	w23, #26214, lsl #16
	mov	w24, #10
	b	.LBB27_3
.LBB27_2:                               //   in Loop: Header=BB27_3 Depth=1
	mov	x0, x19
	bl	_raw_spin_lock
	ldr	x21, [x21]
	mov	x0, x19
	bl	_raw_spin_unlock
	cmp	x21, x20
	b.eq	.LBB27_9
.LBB27_3:                               // =>This Inner Loop Header: Depth=1
	ldr	x25, [x21, #16]
	sub	x26, x21, #520          // =520
	ldr	w8, [x25, #24]
	ldr	w9, [x25, #16]
	cmn	w8, w9
	b.eq	.LBB27_6
// %bb.4:                               //   in Loop: Header=BB27_3 Depth=1
	ldr	w8, [x25, #44]
	ldr	w9, [x25, #40]
	ldr	w10, [x25, #32]
	add	w8, w9, w8
	cmn	w8, w10
	b.ne	.LBB27_6
// %bb.5:                               //   in Loop: Header=BB27_3 Depth=1
	ldr	x0, [x26]
	bl	ufs_mq_sync_dispatch
.LBB27_6:                               //   in Loop: Header=BB27_3 Depth=1
	ldr	w8, [x25, #20]
	cbz	w8, .LBB27_2
// %bb.7:                               //   in Loop: Header=BB27_3 Depth=1
	ldr	w8, [x25, #36]
	cbnz	w8, .LBB27_2
// %bb.8:                               //   in Loop: Header=BB27_3 Depth=1
	ldr	x8, [x26]
	ldr	x1, [x22, :lo12:mas_blk_mq_async_disp_wq]
	mov	w0, #8
	mov	x3, xzr
	ldr	x25, [x8, #1376]
	add	x2, x25, #376           // =376
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x25, #344]
	mul	x9, x8, x23
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	msub	w8, w9, w24, w8
	add	x8, x25, w8, sxtw #3
	str	x0, [x8, #264]
	ldr	w8, [x25, #344]
	add	w8, w8, #1              // =1
	str	w8, [x25, #344]
	b	.LBB27_2
.LBB27_9:
	adrp	x8, mas_blk_io_guard_wq
	ldr	x1, [x8, :lo12:mas_blk_io_guard_wq]
	adrp	x2, mas_io_guard_work
	add	x2, x2, :lo12:mas_io_guard_work
	mov	w0, #8
	mov	w3, #1250
	bl	queue_delayed_work_on
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.Lfunc_end27:
	.size	ufs_mq_io_guard_work_fn, .Lfunc_end27-ufs_mq_io_guard_work_fn
                                        // -- End function
	.globl	ufs_mq_req_alloc_prep   // -- Begin function ufs_mq_req_alloc_prep
	.p2align	2
	.type	ufs_mq_req_alloc_prep,@function
ufs_mq_req_alloc_prep:                  // @ufs_mq_req_alloc_prep
.Lufs_mq_req_alloc_prep$local:
// %bb.0:
	orr	w8, w1, #0x800
	tst	w2, #0x1
	csel	x8, x1, x8, ne
	str	w8, [x0, #36]
	ret
.Lfunc_end28:
	.size	ufs_mq_req_alloc_prep, .Lfunc_end28-ufs_mq_req_alloc_prep
                                        // -- End function
	.globl	ufs_mq_req_init         // -- Begin function ufs_mq_req_init
	.p2align	2
	.type	ufs_mq_req_init,@function
ufs_mq_req_init:                        // @ufs_mq_req_init
.Lufs_mq_req_init$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	str	x0, [x1, #272]
	mov	x0, x1
	mov	x29, sp
	str	xzr, [x1, #240]
	bl	mas_blk_request_init_unistore
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end29:
	.size	ufs_mq_req_init, .Lfunc_end29-ufs_mq_req_init
                                        // -- End function
	.globl	ufs_mq_req_complete     // -- Begin function ufs_mq_req_complete
	.p2align	2
	.type	ufs_mq_req_complete,@function
ufs_mq_req_complete:                    // @ufs_mq_req_complete
.Lufs_mq_req_complete$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	mov	x29, sp
	tbz	w2, #0, .LBB30_3
// %bb.1:
	ldr	w8, [x19, #24]
	and	w8, w8, #0xfc
	orr	w8, w8, #0x3
	cmp	w8, #35                 // =35
	b.eq	.LBB30_3
// %bb.2:
	ldr	x8, [x1, #1376]
	mov	w2, #1
	mov	x0, x19
	ldr	x1, [x8, #536]
	bl	ufs_mq_rq_inflt_update
	mov	w1, #23
	mov	x0, x19
	bl	mas_blk_latency_req_check
.LBB30_3:
	ldr	x8, [x19, #8]
	cbz	x8, .LBB30_7
// %bb.4:
	adrp	x10, cpu_number
	ldr	w8, [x8, #64]
	//APP
	.if 1 == 1
661:
	mrs x9, tpidr_el1
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
	mrs x9, tpidr_el2
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	add	x10, x10, :lo12:cpu_number
	ldr	w9, [x9, x10]
	cmp	w8, w9
	b.eq	.LBB30_7
// %bb.5:
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	adrp	x10, __cpu_online_mask
	asr	w9, w9, #6
	add	x10, x10, :lo12:__cpu_online_mask
	ldr	x9, [x10, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB30_7
// %bb.6:
	mov	w1, #24
	mov	x0, x19
	bl	mas_blk_latency_req_check
	ldr	x9, [x19, #8]
	adrp	x8, __cfi__ufs_mq_complete_request_remote
	add	x8, x8, :lo12:__cfi__ufs_mq_complete_request_remote
	str	x8, [x19, #792]
	str	x19, [x19, #800]
	str	wzr, [x19, #808]
	ldr	w0, [x9, #64]
	add	x1, x19, #784           // =784
	bl	smp_call_function_single_async
	b	.LBB30_8
.LBB30_7:
	mov	w1, #25
	mov	x0, x19
	bl	mas_blk_latency_req_check
	ldr	x8, [x19]
	mov	x0, x19
	ldr	x8, [x8, #48]
	ldr	x8, [x8, #48]
	blr	x8
.LBB30_8:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end30:
	.size	ufs_mq_req_complete, .Lfunc_end30-ufs_mq_req_complete
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_rq_inflt_update
	.type	ufs_mq_rq_inflt_update,@function
ufs_mq_rq_inflt_update:                 // @ufs_mq_rq_inflt_update
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	w8, [x0, #24]
	mov	w20, w2
	mov	x21, x0
	mov	x19, x1
	mov	x29, sp
	tbz	w8, #11, .LBB31_15
// %bb.1:
	mov	w22, #4096
	movk	w22, #64, lsl #16
	tst	w8, w22
	b.eq	.LBB31_4
.LBB31_2:
	ldrb	w8, [x21, #200]
	tbnz	w8, #1, .LBB31_7
// %bb.3:
	add	x8, x19, #44            // =44
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	add	x8, x21, #744           // =744
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	x8, [x21, #240]
	tbnz	w8, #1, .LBB31_8
	b	.LBB31_9
.LBB31_4:
	ldrb	w8, [x21, #200]
	tbz	w8, #7, .LBB31_10
// %bb.5:
	ldr	x0, [x21]
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB31_10
// %bb.6:
	ldr	w8, [x21, #24]
	tst	w8, w22
	b.ne	.LBB31_2
.LBB31_7:
	add	x8, x19, #40            // =40
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w11, [x8]
	sub	w11, w11, w9
	stxr	w12, w11, [x8]
	cbnz	w12, 1b

	//NO_APP
	add	x10, x21, #744          // =744
	mov	w8, #2
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x10]
1:	ldxr	x9, [x10]
	bic	x9, x9, x8
	stxr	w11, x9, [x10]
	cbnz	w11, 1b
	//NO_APP
	ldr	x8, [x21, #240]
	tbz	w8, #1, .LBB31_9
.LBB31_8:
	and	x8, x8, #0xfffffffffffffffd
	str	x8, [x21, #240]
	add	x9, x19, #12            // =12
	mov	w8, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x9]
1:	ldxr	w10, [x9]
	sub	w10, w10, w8
	stxr	w11, w10, [x9]
	cbnz	w11, 1b

	//NO_APP
.LBB31_9:
	tbnz	w20, #0, .LBB31_13
	b	.LBB31_14
.LBB31_10:
	ldr	x8, [x21, #240]
	tbz	w8, #1, .LBB31_12
// %bb.11:
	and	x8, x8, #0xfffffffffffffffd
	str	x8, [x21, #240]
	add	x9, x19, #12            // =12
	mov	w8, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x9]
1:	ldxr	w10, [x9]
	sub	w10, w10, w8
	stxr	w11, w10, [x9]
	cbnz	w11, 1b

	//NO_APP
.LBB31_12:
	add	x8, x19, #32            // =32
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w11, [x8]
	sub	w11, w11, w9
	stxr	w12, w11, [x8]
	cbnz	w12, 1b

	//NO_APP
	add	x10, x21, #744          // =744
	mov	w8, #4
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x10]
1:	ldxr	x9, [x10]
	bic	x9, x9, x8
	stxr	w11, x9, [x10]
	cbnz	w11, 1b
	//NO_APP
	tbz	w20, #0, .LBB31_14
.LBB31_13:
	bl	ktime_get
	str	x0, [x19, #184]
.LBB31_14:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB31_15:
	add	x8, x19, #36            // =36
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w11, [x8]
	sub	w11, w11, w9
	stxr	w12, w11, [x8]
	cbnz	w12, 1b

	//NO_APP
	add	x10, x21, #744          // =744
	mov	w8, #8
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x10]
1:	ldxr	x9, [x10]
	bic	x9, x9, x8
	stxr	w11, x9, [x10]
	cbnz	w11, 1b
	//NO_APP
	tbz	w20, #0, .LBB31_14
// %bb.16:
	bl	ktime_get
	str	x0, [x19, #192]
	b	.LBB31_14
.Lfunc_end31:
	.size	ufs_mq_rq_inflt_update, .Lfunc_end31-ufs_mq_rq_inflt_update
                                        // -- End function
	.globl	ufs_mq_req_deinit       // -- Begin function ufs_mq_req_deinit
	.p2align	2
	.type	ufs_mq_req_deinit,@function
ufs_mq_req_deinit:                      // @ufs_mq_req_deinit
.Lufs_mq_req_deinit$local:
// %bb.0:
	ret
.Lfunc_end32:
	.size	ufs_mq_req_deinit, .Lfunc_end32-ufs_mq_req_deinit
                                        // -- End function
	.globl	ufs_mq_req_insert       // -- Begin function ufs_mq_req_insert
	.p2align	2
	.type	ufs_mq_req_insert,@function
ufs_mq_req_insert:                      // @ufs_mq_req_insert
.Lufs_mq_req_insert$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x20, x0
	mov	x0, x1
	mov	x29, sp
	mov	x19, x1
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB33_2
// %bb.1:
	ldrb	w8, [x20, #201]
	tbnz	w8, #1, .LBB33_8
.LBB33_2:
	ldr	x8, [x20]
	cmp	x8, x19
	b.ne	.LBB33_10
// %bb.3:
	ldr	w8, [x20, #24]
	tbnz	w8, #11, .LBB33_5
.LBB33_4:
	and	w8, w8, #0xfc
	orr	w8, w8, #0x3
	cmp	w8, #35                 // =35
	b.ne	.LBB33_11
.LBB33_5:
	mov	x0, x20
	mov	x1, x19
	bl	ufs_mq_insert_sync_list
.LBB33_6:
	ldr	x8, [x20]
	cmp	x8, x19
	b.eq	.LBB33_9
// %bb.7:
	bl	dump_stack
	b	.LBB33_9
.LBB33_8:
	mov	x0, x20
	mov	x1, x19
	bl	ufs_mq_insert_async_list
.LBB33_9:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB33_10:
	bl	dump_stack
	ldr	w8, [x20, #24]
	tbz	w8, #11, .LBB33_4
	b	.LBB33_5
.LBB33_11:
	mov	x0, x20
	mov	x1, x19
	bl	ufs_mq_insert_async_list
	b	.LBB33_6
.Lfunc_end33:
	.size	ufs_mq_req_insert, .Lfunc_end33-ufs_mq_req_insert
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_insert_async_list
	.type	ufs_mq_insert_async_list,@function
ufs_mq_insert_async_list:               // @ufs_mq_insert_async_list
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x1, #1376]
	mov	x20, x0
	mov	x0, x1
	mov	x29, sp
	ldr	x19, [x8, #536]
	mov	x21, x1
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB34_3
// %bb.1:
	ldrb	w8, [x20, #201]
	tbnz	w8, #1, .LBB34_3
// %bb.2:
	mov	x0, x20
	mov	x1, x21
	bl	ufs_mq_insert_sync_list
	b	.LBB34_7
.LBB34_3:
	ldr	x8, [x21, #1384]
	cbz	x8, .LBB34_7
// %bb.4:
	ldr	x22, [x8, #176]
	cbz	x22, .LBB34_7
// %bb.5:
	ldr	x8, [x22]
	ldr	x8, [x8, #16]
	cbz	x8, .LBB34_7
// %bb.6:
	mov	w1, #9
	mov	x0, x20
	bl	mas_blk_latency_req_check
	add	x21, x19, #100          // =100
	mov	x0, x21
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x22]
	mov	x22, x0
	mov	x0, x20
	mov	x1, x19
	ldr	x8, [x8, #16]
	blr	x8
	add	x8, x19, #128           // =128
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	mov	x0, x21
	mov	x1, x22
	bl	_raw_spin_unlock_irqrestore
.LBB34_7:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end34:
	.size	ufs_mq_insert_async_list, .Lfunc_end34-ufs_mq_insert_async_list
                                        // -- End function
	.globl	ufs_mq_req_requeue      // -- Begin function ufs_mq_req_requeue
	.p2align	2
	.type	ufs_mq_req_requeue,@function
ufs_mq_req_requeue:                     // @ufs_mq_req_requeue
.Lufs_mq_req_requeue$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x19, x0
	str	wzr, [x0, #684]
	mov	x0, x1
	mov	x29, sp
	mov	x20, x1
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB35_8
// %bb.1:
	ldrb	w8, [x19, #201]
	tbnz	w8, #1, .LBB35_9
// %bb.2:
	ldr	x8, [x19, #64]
	cbz	x8, .LBB35_5
// %bb.3:
	ldrb	w9, [x8, #16]
	tbz	w9, #0, .LBB35_5
// %bb.4:
	ldrb	w8, [x8, #352]
	cbnz	w8, .LBB35_10
.LBB35_5:
	ldr	w8, [x19, #24]
	tbnz	w8, #11, .LBB35_7
// %bb.6:
	and	w8, w8, #0xfc
	orr	w8, w8, #0x3
	cmp	w8, #35                 // =35
	b.ne	.LBB35_11
.LBB35_7:
	mov	x0, x19
	mov	x1, x20
	bl	ufs_mq_insert_sync_list
	b	.LBB35_10
.LBB35_8:
	mov	x0, x19
	mov	x1, x20
	bl	ufs_mq_req_insert
	b	.LBB35_10
.LBB35_9:
	mov	x0, x19
	mov	x1, x20
	bl	ufs_mq_insert_async_list
.LBB35_10:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB35_11:
	ldr	x8, [x20, #1384]
	cbz	x8, .LBB35_10
// %bb.12:
	ldr	x22, [x8, #176]
	cbz	x22, .LBB35_10
// %bb.13:
	ldr	x8, [x22]
	ldr	x8, [x8, #32]
	cbz	x8, .LBB35_10
// %bb.14:
	ldr	x8, [x20, #1376]
	mov	w1, #9
	mov	x0, x19
	ldr	x20, [x8, #536]
	bl	mas_blk_latency_req_check
	add	x21, x20, #100          // =100
	mov	x0, x21
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x22]
	mov	x22, x0
	mov	x0, x19
	mov	x1, x20
	ldr	x8, [x8, #32]
	blr	x8
	add	x8, x20, #128           // =128
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	mov	x0, x21
	mov	x1, x22
	bl	_raw_spin_unlock_irqrestore
	b	.LBB35_10
.Lfunc_end35:
	.size	ufs_mq_req_requeue, .Lfunc_end35-ufs_mq_req_requeue
                                        // -- End function
	.globl	ufs_mq_req_timeout_handler // -- Begin function ufs_mq_req_timeout_handler
	.p2align	2
	.type	ufs_mq_req_timeout_handler,@function
ufs_mq_req_timeout_handler:             // @ufs_mq_req_timeout_handler
.Lufs_mq_req_timeout_handler$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0]
	ldr	w9, [x0, #28]
	mov	x19, x0
	mov	x29, sp
	ldr	x10, [x8, #1376]
	ldr	x8, [x8, #48]
	orr	w9, w9, #0x200000
	ldr	x20, [x10, #536]
	str	w9, [x0, #28]
	ldr	x8, [x8, #32]
	cbz	x8, .LBB36_5
// %bb.1:
	mov	x0, x19
	mov	w1, wzr
	blr	x8
	cmp	w0, #1                  // =1
	b.eq	.LBB36_5
// %bb.2:
	cbnz	w0, .LBB36_6
// %bb.3:
	ldr	w8, [x19, #24]
	and	w8, w8, #0xfc
	orr	w8, w8, #0x3
	cmp	w8, #35                 // =35
	b.eq	.LBB36_7
// %bb.4:
	mov	w2, #1
	mov	x0, x19
	mov	x1, x20
	bl	ufs_mq_rq_inflt_update
	b	.LBB36_7
.LBB36_5:
	mov	x0, x19
	bl	blk_add_timer
	b	.LBB36_7
.LBB36_6:
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "block/mas_blk_iosched_ufs_mq.c"; .popsection; .long 14472b - 14470b; .short 3324; .short (1 << 0)|((1 << 1) | ((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
.LBB36_7:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end36:
	.size	ufs_mq_req_timeout_handler, .Lfunc_end36-ufs_mq_req_timeout_handler
                                        // -- End function
	.globl	ufs_mq_ctx_put          // -- Begin function ufs_mq_ctx_put
	.p2align	2
	.type	ufs_mq_ctx_put,@function
ufs_mq_ctx_put:                         // @ufs_mq_ctx_put
.Lufs_mq_ctx_put$local:
// %bb.0:
	ret
.Lfunc_end37:
	.size	ufs_mq_ctx_put, .Lfunc_end37-ufs_mq_ctx_put
                                        // -- End function
	.globl	ufs_mq_hctx_get_by_req  // -- Begin function ufs_mq_hctx_get_by_req
	.p2align	2
	.type	ufs_mq_hctx_get_by_req,@function
ufs_mq_hctx_get_by_req:                 // @ufs_mq_hctx_get_by_req
.Lufs_mq_hctx_get_by_req$local:
// %bb.0:
	ldr	w8, [x0, #24]
	ldr	x9, [x0, #272]
	tst	w8, #0xff
	cset	w10, eq
	tst	w8, #0x8000000
	mov	w8, #2
	csel	x8, x10, x8, eq
	add	x8, x9, x8, lsl #3
	ldr	x8, [x8, #80]
	str	x8, [x1]
	ret
.Lfunc_end38:
	.size	ufs_mq_hctx_get_by_req, .Lfunc_end38-ufs_mq_hctx_get_by_req
                                        // -- End function
	.globl	ufs_mq_exec_queue       // -- Begin function ufs_mq_exec_queue
	.p2align	2
	.type	ufs_mq_exec_queue,@function
ufs_mq_exec_queue:                      // @ufs_mq_exec_queue
.Lufs_mq_exec_queue$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	mov	x19, x0
	bl	ufs_mq_sync_dispatch
	ldr	x8, [x19, #1384]
	ldr	x20, [x19, #1376]
	cbz	x8, .LBB39_5
// %bb.1:
	ldr	x22, [x8, #176]
	cbz	x22, .LBB39_5
// %bb.2:
	ldr	x8, [x22]
	ldr	x8, [x8, #48]
	cbz	x8, .LBB39_5
// %bb.3:
	ldr	x20, [x20, #536]
	add	x21, x20, #100          // =100
	mov	x0, x21
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x22]
	mov	x22, x0
	mov	x0, x20
	ldr	x8, [x8, #48]
	blr	x8
	mov	w20, w0
	mov	x0, x21
	mov	x1, x22
	bl	_raw_spin_unlock_irqrestore
	tbnz	w20, #0, .LBB39_6
// %bb.4:
	ldr	x20, [x19, #1376]
.LBB39_5:
	adrp	x8, mas_blk_mq_async_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_async_disp_wq]
	add	x2, x20, #376           // =376
	mov	w0, #8
	mov	x3, xzr
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x20, #344]
	mov	w9, #26215
	movk	w9, #26214, lsl #16
	mul	x9, x8, x9
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	mov	w10, #10
	msub	w8, w9, w10, w8
	add	x8, x20, w8, sxtw #3
	str	x0, [x8, #264]
	ldr	w8, [x20, #344]
	add	w8, w8, #1              // =1
	str	w8, [x20, #344]
.LBB39_6:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end39:
	.size	ufs_mq_exec_queue, .Lfunc_end39-ufs_mq_exec_queue
                                        // -- End function
	.globl	ufs_mq_run_hw_queue     // -- Begin function ufs_mq_run_hw_queue
	.p2align	2
	.type	ufs_mq_run_hw_queue,@function
ufs_mq_run_hw_queue:                    // @ufs_mq_run_hw_queue
.Lufs_mq_run_hw_queue$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x20, [x0, #1376]
	adrp	x8, mas_blk_mq_sync_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_sync_disp_wq]
	mov	x19, x0
	add	x2, x20, #120           // =120
	mov	w0, #8
	mov	x3, xzr
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x20, #344]
	mov	w9, #26215
	movk	w9, #26214, lsl #16
	mov	w23, #10
	mul	x9, x8, x9
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	msub	w8, w9, w23, w8
	add	x8, x20, w8, sxtw #3
	str	x0, [x8, #8]
	ldr	w8, [x20, #88]
	add	w8, w8, #1              // =1
	str	w8, [x20, #88]
	ldr	x8, [x19, #1384]
	ldr	x20, [x19, #1376]
	cbz	x8, .LBB40_5
// %bb.1:
	ldr	x22, [x8, #176]
	cbz	x22, .LBB40_5
// %bb.2:
	ldr	x8, [x22]
	ldr	x8, [x8, #48]
	cbz	x8, .LBB40_5
// %bb.3:
	ldr	x20, [x20, #536]
	add	x21, x20, #100          // =100
	mov	x0, x21
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x22]
	mov	x22, x0
	mov	x0, x20
	ldr	x8, [x8, #48]
	blr	x8
	mov	w20, w0
	mov	x0, x21
	mov	x1, x22
	bl	_raw_spin_unlock_irqrestore
	tbnz	w20, #0, .LBB40_6
// %bb.4:
	ldr	x20, [x19, #1376]
.LBB40_5:
	adrp	x8, mas_blk_mq_async_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_async_disp_wq]
	add	x2, x20, #376           // =376
	mov	w0, #8
	mov	x3, xzr
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x20, #344]
	mov	w9, #26215
	movk	w9, #26214, lsl #16
	mul	x9, x8, x9
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	msub	w8, w9, w23, w8
	add	x8, x20, w8, sxtw #3
	str	x0, [x8, #264]
	ldr	w8, [x20, #344]
	add	w8, w8, #1              // =1
	str	w8, [x20, #344]
.LBB40_6:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end40:
	.size	ufs_mq_run_hw_queue, .Lfunc_end40-ufs_mq_run_hw_queue
                                        // -- End function
	.globl	ufs_mq_run_requeue      // -- Begin function ufs_mq_run_requeue
	.p2align	2
	.type	ufs_mq_run_requeue,@function
ufs_mq_run_requeue:                     // @ufs_mq_run_requeue
.Lufs_mq_run_requeue$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x20, [x0, #1376]
	adrp	x8, mas_blk_mq_sync_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_sync_disp_wq]
	mov	x19, x0
	add	x2, x20, #120           // =120
	mov	w0, #8
	mov	x3, xzr
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x20, #344]
	mov	w9, #26215
	movk	w9, #26214, lsl #16
	mov	w23, #10
	mul	x9, x8, x9
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	msub	w8, w9, w23, w8
	add	x8, x20, w8, sxtw #3
	str	x0, [x8, #8]
	ldr	w8, [x20, #88]
	add	w8, w8, #1              // =1
	str	w8, [x20, #88]
	ldr	x8, [x19, #1384]
	ldr	x20, [x19, #1376]
	cbz	x8, .LBB41_5
// %bb.1:
	ldr	x22, [x8, #176]
	cbz	x22, .LBB41_5
// %bb.2:
	ldr	x8, [x22]
	ldr	x8, [x8, #48]
	cbz	x8, .LBB41_5
// %bb.3:
	ldr	x20, [x20, #536]
	add	x21, x20, #100          // =100
	mov	x0, x21
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x22]
	mov	x22, x0
	mov	x0, x20
	ldr	x8, [x8, #48]
	blr	x8
	mov	w20, w0
	mov	x0, x21
	mov	x1, x22
	bl	_raw_spin_unlock_irqrestore
	tbnz	w20, #0, .LBB41_6
// %bb.4:
	ldr	x20, [x19, #1376]
.LBB41_5:
	adrp	x8, mas_blk_mq_async_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_async_disp_wq]
	add	x2, x20, #376           // =376
	mov	w0, #8
	mov	x3, xzr
	bl	queue_delayed_work_on
	bl	ktime_get
	ldrsw	x8, [x20, #344]
	mov	w9, #26215
	movk	w9, #26214, lsl #16
	mul	x9, x8, x9
	lsr	x10, x9, #63
	asr	x9, x9, #34
	add	w9, w9, w10
	msub	w8, w9, w23, w8
	add	x8, x20, w8, sxtw #3
	str	x0, [x8, #264]
	ldr	w8, [x20, #344]
	add	w8, w8, #1              // =1
	str	w8, [x20, #344]
.LBB41_6:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end41:
	.size	ufs_mq_run_requeue, .Lfunc_end41-ufs_mq_run_requeue
                                        // -- End function
	.globl	ufs_mq_poll_enable      // -- Begin function ufs_mq_poll_enable
	.p2align	2
	.type	ufs_mq_poll_enable,@function
ufs_mq_poll_enable:                     // @ufs_mq_poll_enable
.Lufs_mq_poll_enable$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	str	x23, [sp, #16]          // 8-byte Folded Spill
	adrp	x23, __cpu_online_mask
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	mov	x19, x0
	mov	w21, wzr
	mov	w20, wzr
	adrp	x22, __cpu_possible_mask
	add	x23, x23, :lo12:__cpu_online_mask
	mov	x29, sp
.LBB42_1:                               // =>This Inner Loop Header: Depth=1
	ldrb	w0, [x22, :lo12:__cpu_possible_mask]
	bl	__sw_hweight64
	cmp	w21, w0
	b.hs	.LBB42_3
// %bb.2:                               //   in Loop: Header=BB42_1 Depth=1
	add	w8, w21, #63            // =63
	cmp	w21, #0                 // =0
	csel	w8, w8, w21, lt
	asr	w8, w8, #6
	ldr	x8, [x23, w8, sxtw #3]
	lsr	x8, x8, x21
	and	w8, w8, #0x1
	add	w20, w20, w8
	add	w21, w21, #1            // =1
	b	.LBB42_1
.LBB42_3:
	cmp	w20, #1                 // =1
	cset	w8, hi
	strb	w8, [x19]
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end42:
	.size	ufs_mq_poll_enable, .Lfunc_end42-ufs_mq_poll_enable
                                        // -- End function
	.globl	ufs_order_panic_wait_datasync_handle // -- Begin function ufs_order_panic_wait_datasync_handle
	.p2align	2
	.type	ufs_order_panic_wait_datasync_handle,@function
ufs_order_panic_wait_datasync_handle:   // @ufs_order_panic_wait_datasync_handle
.Lufs_order_panic_wait_datasync_handle$local:
// %bb.0:
	ldr	x8, [x0, #1440]
	ldr	w9, [x8, #24]
	ldr	w10, [x8, #16]
	add	w9, w10, w9
	cmp	w9, #1                  // =1
	cset	w0, lt
	cmp	w9, #0                  // =0
	cset	w9, gt
	strb	w9, [x8, #536]
	ret
.Lfunc_end43:
	.size	ufs_order_panic_wait_datasync_handle, .Lfunc_end43-ufs_order_panic_wait_datasync_handle
                                        // -- End function
	.globl	ufs_order_panic_datasync_handle // -- Begin function ufs_order_panic_datasync_handle
	.p2align	2
	.type	ufs_order_panic_datasync_handle,@function
ufs_order_panic_datasync_handle:        // @ufs_order_panic_datasync_handle
.Lufs_order_panic_datasync_handle$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x8, [x0, #1440]
	mov	w9, #1
	mov	w0, #8
	mov	x3, xzr
	strb	w9, [x8, #536]
	adrp	x9, mas_blk_mq_sync_disp_wq
	ldr	x1, [x9, :lo12:mas_blk_mq_sync_disp_wq]
	add	x2, x8, #400            // =400
	mov	x29, sp
	bl	queue_delayed_work_on
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end44:
	.size	ufs_order_panic_datasync_handle, .Lfunc_end44-ufs_order_panic_datasync_handle
                                        // -- End function
	.globl	ufs_mq_write_throttle_check_timer_prepare // -- Begin function ufs_mq_write_throttle_check_timer_prepare
	.p2align	2
	.type	ufs_mq_write_throttle_check_timer_prepare,@function
ufs_mq_write_throttle_check_timer_prepare: // @ufs_mq_write_throttle_check_timer_prepare
.Lufs_mq_write_throttle_check_timer_prepare$local:
// %bb.0:
	sub	x0, x0, #272            // =272
	ret
.Lfunc_end45:
	.size	ufs_mq_write_throttle_check_timer_prepare, .Lfunc_end45-ufs_mq_write_throttle_check_timer_prepare
                                        // -- End function
	.globl	ufs_mq_sync_burst_check_timer_prepare // -- Begin function ufs_mq_sync_burst_check_timer_prepare
	.p2align	2
	.type	ufs_mq_sync_burst_check_timer_prepare,@function
ufs_mq_sync_burst_check_timer_prepare:  // @ufs_mq_sync_burst_check_timer_prepare
.Lufs_mq_sync_burst_check_timer_prepare$local:
// %bb.0:
	sub	x0, x0, #208            // =208
	ret
.Lfunc_end46:
	.size	ufs_mq_sync_burst_check_timer_prepare, .Lfunc_end46-ufs_mq_sync_burst_check_timer_prepare
                                        // -- End function
	.globl	ufs_tagset_power_off_proc // -- Begin function ufs_tagset_power_off_proc
	.p2align	2
	.type	ufs_tagset_power_off_proc,@function
ufs_tagset_power_off_proc:              // @ufs_tagset_power_off_proc
.Lufs_tagset_power_off_proc$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	ldr	x19, [x0, #1440]
	mov	w8, #1
	mov	w0, #8
	mov	x3, xzr
	strb	w8, [x19, #536]
	adrp	x8, mas_blk_mq_sync_disp_wq
	ldr	x1, [x8, :lo12:mas_blk_mq_sync_disp_wq]
	add	x2, x19, #400           // =400
	mov	x29, sp
	bl	queue_delayed_work_on
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	add	x0, x19, #344           // =344
	add	x1, x8, #125            // =125
	bl	mod_timer
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end47:
	.size	ufs_tagset_power_off_proc, .Lfunc_end47-ufs_tagset_power_off_proc
                                        // -- End function
	.globl	ufs_mq_status_dump      // -- Begin function ufs_mq_status_dump
	.p2align	2
	.type	ufs_mq_status_dump,@function
ufs_mq_status_dump:                     // @ufs_mq_status_dump
.Lufs_mq_status_dump$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0, #1376]
	adrp	x9, .L.str.27
	adrp	x10, .L.str.26
	add	x9, x9, :lo12:.L.str.27
	add	x10, x10, :lo12:.L.str.26
	cmp	w1, #1                  // =1
	csel	x19, x10, x9, eq
	mov	x29, sp
	cbz	x8, .LBB48_2
// %bb.1:
	ldr	x20, [x8, #536]
	cbnz	x20, .LBB48_3
.LBB48_2:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB48_3:
	ldr	w2, [x20, #28]
	adrp	x0, .L.str.12
	add	x0, x0, :lo12:.L.str.12
	mov	x1, x19
	bl	printk
	ldr	w2, [x20, #24]
	ldr	w3, [x20, #16]
	ldr	w4, [x20, #20]
	adrp	x0, .L.str.13
	add	x0, x0, :lo12:.L.str.13
	mov	x1, x19
	bl	printk
	ldr	w2, [x20, #44]
	ldr	w3, [x20, #40]
	ldr	w4, [x20, #32]
	ldr	w5, [x20, #36]
	ldr	w6, [x20, #12]
	adrp	x0, .L.str.14
	add	x0, x0, :lo12:.L.str.14
	mov	x1, x19
	bl	printk
	b	.LBB48_2
.Lfunc_end48:
	.size	ufs_mq_status_dump, .Lfunc_end48-ufs_mq_status_dump
                                        // -- End function
	.globl	ufs_mq_iosched_init     // -- Begin function ufs_mq_iosched_init
	.p2align	2
	.type	ufs_mq_iosched_init,@function
ufs_mq_iosched_init:                    // @ufs_mq_iosched_init
.Lufs_mq_iosched_init$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	adrp	x21, kmalloc_caches+80
	ldr	x8, [x21, :lo12:kmalloc_caches+80]
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	mov	x20, x0
	mov	w1, #3520
	mov	w2, #544
	mov	x0, x8
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	mov	x29, sp
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB49_21
// %bb.1:
	mov	x19, x0
	mov	x0, x20
	bl	mas_blk_get_lld
	ldrb	w8, [x0, #1432]
	mov	x22, x0
	cbz	w8, .LBB49_11
// %bb.2:
	ldr	x8, [x22, #1440]
	add	x8, x8, #336            // =336
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x21, [x22, #1440]
.LBB49_3:
	cbz	x21, .LBB49_18
// %bb.4:
	ldr	x8, [x20, #1384]
	cbz	x8, .LBB49_17
// %bb.5:
	ldr	x23, [x8, #176]
	cbz	x23, .LBB49_17
// %bb.6:
	ldr	x8, [x23]
	ldr	x8, [x8, #8]
	cbz	x8, .LBB49_8
// %bb.7:
	add	x22, x21, #100          // =100
	mov	x0, x22
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x23]
	mov	x23, x0
	mov	x0, x21
	ldr	x8, [x8, #8]
	blr	x8
	mov	x0, x22
	mov	x1, x23
	bl	_raw_spin_unlock_irqrestore
.LBB49_8:
	adrp	x22, delayed_work_timer_fn
	adrp	x9, __cfi_ufs_mq_sync_io_dispatch_work_fn
	add	x22, x22, :lo12:delayed_work_timer_fn
	mov	x23, #68719476704
	add	x8, x19, #128           // =128
	add	x9, x9, :lo12:__cfi_ufs_mq_sync_io_dispatch_work_fn
	add	x0, x19, #168           // =168
	mov	w2, #2097152
	mov	x1, x22
	mov	x3, xzr
	mov	x4, xzr
	str	x23, [x19, #120]
	str	x8, [x19, #128]
	stp	x8, x9, [x19, #136]
	bl	init_timer_key
	adrp	x9, __cfi_ufs_mq_async_io_dispatch_work_fn
	add	x8, x19, #384           // =384
	add	x9, x9, :lo12:__cfi_ufs_mq_async_io_dispatch_work_fn
	add	x0, x19, #424           // =424
	mov	w2, #2097152
	mov	x1, x22
	mov	x3, xzr
	mov	x4, xzr
	str	x20, [x19, #256]
	str	x23, [x19, #376]
	str	x8, [x19, #384]
	stp	x8, x9, [x19, #392]
	bl	init_timer_key
	adrp	x0, io_guard_queue_list_lock
	add	x0, x0, :lo12:io_guard_queue_list_lock
	str	x20, [x19, #512]
	str	wzr, [x19, #88]
	str	wzr, [x19, #344]
	str	wzr, [x20, #1232]
	str	wzr, [x20, #1236]
	str	x20, [x19]
	str	x21, [x19, #536]
	str	x19, [x20, #1376]
	bl	_raw_spin_lock
	adrp	x23, mas_io_guard_queue_list
	add	x23, x23, :lo12:mas_io_guard_queue_list
	ldr	x21, [x23, #8]
	add	x22, x19, #520          // =520
	mov	x0, x22
	mov	x2, x23
	mov	x1, x21
	bl	__list_add_valid
	tbz	w0, #0, .LBB49_10
// %bb.9:
	str	x22, [x23, #8]
	str	x23, [x19, #520]
	str	x21, [x19, #528]
	str	x22, [x21]
.LBB49_10:
	adrp	x0, io_guard_queue_list_lock
	add	x0, x0, :lo12:io_guard_queue_list_lock
	bl	_raw_spin_unlock
	adrp	x8, mas_blk_io_guard_wq
	ldr	x1, [x8, :lo12:mas_blk_io_guard_wq]
	adrp	x2, mas_io_guard_work
	add	x2, x2, :lo12:mas_io_guard_work
	mov	w0, #8
	mov	w3, #1250
	bl	queue_delayed_work_on
	adrp	x1, __cfi_ufs_mq_make_request
	add	x1, x1, :lo12:__cfi_ufs_mq_make_request
	mov	x0, x20
	bl	blk_queue_make_request
	mov	w0, wzr
	b	.LBB49_20
.LBB49_11:
	ldr	x0, [x21, :lo12:kmalloc_caches+80]
	mov	w1, #3520
	mov	w2, #544
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB49_18
// %bb.12:
	mov	w23, #1
	adrp	x1, __ufs_turbo_check_timer_expire
	mov	x21, x0
	str	w23, [x0, #336]
	add	x0, x0, #344            // =344
	add	x1, x1, :lo12:__ufs_turbo_check_timer_expire
	mov	w2, wzr
	mov	x3, xzr
	mov	x4, xzr
	bl	init_timer_key
	adrp	x10, ufs_datasync_work
	adrp	x1, delayed_work_timer_fn
	mov	x8, #68719476704
	add	x9, x21, #408           // =408
	add	x10, x10, :lo12:ufs_datasync_work
	add	x0, x21, #448           // =448
	add	x1, x1, :lo12:delayed_work_timer_fn
	mov	w2, #2097152
	mov	x3, xzr
	mov	x4, xzr
	strb	wzr, [x21, #536]
	str	x8, [x21, #400]
	str	x9, [x21, #408]
	stp	x9, x10, [x21, #416]
	bl	init_timer_key
	add	x8, x21, #56            // =56
	str	wzr, [x21, #16]
	str	wzr, [x21, #20]
	str	wzr, [x21, #24]
	str	wzr, [x21, #28]
	str	x8, [x21, #56]
	str	x8, [x21, #64]
	add	x8, x21, #80            // =80
	str	x8, [x21, #80]
	str	x8, [x21, #88]
	add	x8, x21, #136           // =136
	str	wzr, [x21, #48]
	str	wzr, [x21, #100]
	str	x8, [x21, #136]
	str	x8, [x21, #144]
	mov	w8, #5
	str	wzr, [x21, #152]
	str	wzr, [x21, #72]
	str	wzr, [x21, #96]
	str	wzr, [x21, #128]
	str	w8, [x21, #8]
	mov	x8, #28
	movk	x8, #14, lsl #32
	stur	x8, [x21, #156]
	str	wzr, [x21, #12]
	str	wzr, [x21, #36]
	str	wzr, [x21, #44]
	str	wzr, [x21, #40]
	str	wzr, [x21, #32]
	bl	ktime_get
	str	x0, [x21, #168]
	bl	ktime_get
	str	x0, [x21, #176]
	bl	ktime_get
	str	x0, [x21, #184]
	bl	ktime_get
	str	x0, [x21, #192]
	bl	ktime_get
	adrp	x1, __cfi_ufs_mq_sync_burst_check_timer_expire
	str	x0, [x21, #200]
	add	x0, x21, #208           // =208
	add	x1, x1, :lo12:__cfi_ufs_mq_sync_burst_check_timer_expire
	mov	w2, wzr
	mov	x3, xzr
	mov	x4, xzr
	bl	init_timer_key
	adrp	x1, __cfi_ufs_mq_write_throttle_check_timer_expire
	add	x0, x21, #272           // =272
	add	x1, x1, :lo12:__cfi_ufs_mq_write_throttle_check_timer_expire
	mov	w2, wzr
	mov	x3, xzr
	mov	x4, xzr
	bl	init_timer_key
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	mov	w1, #16
	mov	w2, wzr
	str	wzr, [x21, #328]
	strb	w23, [x21, #332]
	str	wzr, [x21, #264]
	str	wzr, [x21, #268]
	bl	alloc_workqueue
	adrp	x23, mas_blk_mq_sync_disp_wq
	str	x0, [x23, :lo12:mas_blk_mq_sync_disp_wq]
	cbz	x0, .LBB49_22
// %bb.13:
	adrp	x0, .L.str.33
	add	x0, x0, :lo12:.L.str.33
	mov	w1, #2
	mov	w2, wzr
	bl	alloc_workqueue
	adrp	x24, mas_blk_mq_async_disp_wq
	str	x0, [x24, :lo12:mas_blk_mq_async_disp_wq]
	cbz	x0, .LBB49_24
// %bb.14:
	adrp	x0, .L.str.34
	add	x0, x0, :lo12:.L.str.34
	mov	w1, #6
	mov	w2, wzr
	bl	alloc_workqueue
	adrp	x8, mas_blk_io_guard_wq
	str	x0, [x8, :lo12:mas_blk_io_guard_wq]
	cbz	x0, .LBB49_23
// %bb.15:
	adrp	x8, mas_io_guard_work
	add	x8, x8, :lo12:mas_io_guard_work
	mov	x9, #68719476704
	adrp	x10, __cfi_ufs_mq_io_guard_work_fn
	adrp	x1, delayed_work_timer_fn
	add	x10, x10, :lo12:__cfi_ufs_mq_io_guard_work_fn
	str	x9, [x8]
	add	x9, x8, #8              // =8
	add	x0, x8, #48             // =48
	add	x1, x1, :lo12:delayed_work_timer_fn
	mov	w2, #2097152
	mov	x3, xzr
	mov	x4, xzr
	str	x9, [x8, #8]
	stp	x9, x10, [x8, #16]
	bl	init_timer_key
	str	x22, [x21]
	ldr	w8, [x22, #4]
	mov	w9, #1
	str	x21, [x22, #1440]
	strb	w9, [x22, #1432]
	cmp	w8, #2                  // =2
	b.ne	.LBB49_3
// %bb.16:
	add	x0, x22, #1416          // =1416
	bl	mas_blk_flush_list_register
	b	.LBB49_3
.LBB49_17:
	mov	x0, x20
	bl	ufs_mq_sched_ds_lld_exit
.LBB49_18:
	mov	x0, x19
	bl	kfree
.LBB49_19:
	mov	w0, #-12
.LBB49_20:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB49_21:
	adrp	x0, .L.str.15
	adrp	x1, .L__func__.ufs_mq_iosched_init
	add	x0, x0, :lo12:.L.str.15
	add	x1, x1, :lo12:.L__func__.ufs_mq_iosched_init
	mov	w2, #3717
	bl	printk
	b	.LBB49_19
.LBB49_22:
	adrp	x0, .L.str.32
	adrp	x1, .L__func__.ufs_mq_workqueue_init
	add	x0, x0, :lo12:.L.str.32
	add	x1, x1, :lo12:.L__func__.ufs_mq_workqueue_init
	mov	w2, #3560
	bl	printk
	b	.LBB49_25
.LBB49_23:
	ldr	x0, [x24, :lo12:mas_blk_mq_async_disp_wq]
	bl	destroy_workqueue
.LBB49_24:
	ldr	x0, [x23, :lo12:mas_blk_mq_sync_disp_wq]
	bl	destroy_workqueue
	adrp	x0, .L.str.35
	adrp	x1, .L__func__.ufs_mq_workqueue_init
	add	x0, x0, :lo12:.L.str.35
	add	x1, x1, :lo12:.L__func__.ufs_mq_workqueue_init
	bl	printk
.LBB49_25:
	mov	x0, x21
	bl	kfree
	b	.LBB49_18
.Lfunc_end49:
	.size	ufs_mq_iosched_init, .Lfunc_end49-ufs_mq_iosched_init
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_sched_ds_lld_exit
	.type	ufs_mq_sched_ds_lld_exit,@function
ufs_mq_sched_ds_lld_exit:               // @ufs_mq_sched_ds_lld_exit
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x29, sp
	bl	mas_blk_get_lld
	ldr	w8, [x0, #4]
	ldr	x20, [x0, #1440]
	mov	x19, x0
	cmp	w8, #2                  // =2
	b.ne	.LBB50_2
// %bb.1:
	add	x0, x19, #1416          // =1416
	bl	mas_blk_flush_list_unregister
.LBB50_2:
	ldrb	w8, [x19, #1432]
	cbz	w8, .LBB50_5
// %bb.3:
	add	x8, x20, #336           // =336
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
	cbnz	w10, .LBB50_5
// %bb.4:
	strb	wzr, [x19, #1432]
	adrp	x8, mas_blk_mq_async_disp_wq
	ldr	x0, [x8, :lo12:mas_blk_mq_async_disp_wq]
	bl	destroy_workqueue
	adrp	x8, mas_blk_mq_sync_disp_wq
	ldr	x0, [x8, :lo12:mas_blk_mq_sync_disp_wq]
	bl	destroy_workqueue
	adrp	x8, mas_blk_io_guard_wq
	ldr	x0, [x8, :lo12:mas_blk_io_guard_wq]
	bl	destroy_workqueue
	ldr	x0, [x19, #1440]
	bl	kfree
	str	xzr, [x19, #1440]
.LBB50_5:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end50:
	.size	ufs_mq_sched_ds_lld_exit, .Lfunc_end50-ufs_mq_sched_ds_lld_exit
                                        // -- End function
	.globl	ufs_mq_iosched_exit     // -- Begin function ufs_mq_iosched_exit
	.p2align	2
	.type	ufs_mq_iosched_exit,@function
ufs_mq_iosched_exit:                    // @ufs_mq_iosched_exit
.Lufs_mq_iosched_exit$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	ldr	x21, [x0, #1376]
	mov	x29, sp
	cbz	x21, .LBB51_4
// %bb.1:
	mov	x19, x0
	adrp	x0, mas_io_guard_work
	add	x0, x0, :lo12:mas_io_guard_work
	bl	cancel_delayed_work_sync
	adrp	x0, io_guard_queue_list_lock
	add	x0, x0, :lo12:io_guard_queue_list_lock
	bl	_raw_spin_lock
	add	x20, x21, #520          // =520
	mov	x0, x20
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB51_3
// %bb.2:
	ldr	x8, [x21, #528]
	ldr	x9, [x21, #520]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB51_3:
	adrp	x0, io_guard_queue_list_lock
	add	x0, x0, :lo12:io_guard_queue_list_lock
	str	x20, [x21, #520]
	str	x20, [x21, #528]
	bl	_raw_spin_unlock
	mov	x0, x19
	bl	ufs_mq_sched_ds_lld_exit
	ldr	x0, [x19, #1376]
	bl	kfree
	str	xzr, [x19, #1376]
.LBB51_4:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end51:
	.size	ufs_mq_iosched_exit, .Lfunc_end51-ufs_mq_iosched_exit
                                        // -- End function
	.globl	blk_mq_tagset_ufs_mq_iosched_enable // -- Begin function blk_mq_tagset_ufs_mq_iosched_enable
	.p2align	2
	.type	blk_mq_tagset_ufs_mq_iosched_enable,@function
blk_mq_tagset_ufs_mq_iosched_enable:    // @blk_mq_tagset_ufs_mq_iosched_enable
.Lblk_mq_tagset_ufs_mq_iosched_enable$local:
// %bb.0:
	ldr	x8, [x0, #88]
	cmp	w1, #0                  // =0
	and	x9, x8, #0xffffffffffffffdf
	orr	x8, x8, #0x20
	csel	x8, x9, x8, eq
	str	x8, [x0, #88]
	ret
.Lfunc_end52:
	.size	blk_mq_tagset_ufs_mq_iosched_enable, .Lfunc_end52-blk_mq_tagset_ufs_mq_iosched_enable
                                        // -- End function
	.globl	blk_mq_get_io_in_list_count // -- Begin function blk_mq_get_io_in_list_count
	.p2align	2
	.type	blk_mq_get_io_in_list_count,@function
blk_mq_get_io_in_list_count:            // @blk_mq_get_io_in_list_count
.Lblk_mq_get_io_in_list_count$local:
// %bb.0:
	cbz	x0, .LBB53_5
// %bb.1:
	ldr	x8, [x0, #136]
	cbz	x8, .LBB53_5
// %bb.2:
	ldr	x8, [x8, #1376]
	cbz	x8, .LBB53_5
// %bb.3:
	ldr	x8, [x8, #536]
	cbz	x8, .LBB53_5
// %bb.4:
	ldr	w9, [x8, #72]
	ldr	w10, [x8, #96]
	ldr	w8, [x8, #128]
	add	w9, w10, w9
	add	w0, w9, w8
	ret
.LBB53_5:
	mov	w0, #-1
	ret
.Lfunc_end53:
	.size	blk_mq_get_io_in_list_count, .Lfunc_end53-blk_mq_get_io_in_list_count
                                        // -- End function
	.p2align	2               // -- Begin function mas_blk_recovery_pwron_info_done
	.type	mas_blk_recovery_pwron_info_done,@function
mas_blk_recovery_pwron_info_done:       // @mas_blk_recovery_pwron_info_done
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	and	w2, w1, #0xff
	cmp	w2, #2                  // =2
	mov	x29, sp
	b.ne	.LBB54_2
// %bb.1:
	adrp	x8, g_recovery_pwron_info
	add	x8, x8, :lo12:g_recovery_pwron_info
	mov	w9, #1
	str	w0, [x8, #4]
	str	w9, [x8]
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB54_2:
	adrp	x0, .L.str.22
	adrp	x1, .L__func__.mas_blk_recovery_pwron_info_done
	add	x0, x0, :lo12:.L.str.22
	add	x1, x1, :lo12:.L__func__.mas_blk_recovery_pwron_info_done
	bl	printk
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end54:
	.size	mas_blk_recovery_pwron_info_done, .Lfunc_end54-mas_blk_recovery_pwron_info_done
                                        // -- End function
	.p2align	2               // -- Begin function __ufs_turbo_check_timer_expire
	.type	__ufs_turbo_check_timer_expire,@function
__ufs_turbo_check_timer_expire:         // @__ufs_turbo_check_timer_expire
// %bb.0:
	sub	sp, sp, #64             // =64
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	str	x21, [sp, #32]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	adrp	x19, ufs_turbo_check_timer_expire.check_count
	sub	x8, x0, #320            // =320
	sub	x9, x0, #328            // =328
	sub	x10, x0, #300           // =300
	sub	x11, x0, #304           // =304
	sub	x12, x0, #312           // =312
	sub	x13, x0, #308           // =308
	ldrb	w3, [x0, #192]
	ldr	w2, [x19, :lo12:ufs_turbo_check_timer_expire.check_count]
	ldr	w4, [x8]
	ldr	w5, [x9]
	ldr	w6, [x10]
	ldr	w7, [x11]
	ldr	w8, [x12]
	ldr	w9, [x13]
	sub	x20, x0, #344           // =344
	adrp	x0, .L.str.28
	adrp	x1, .L__func__.ufs_turbo_check_timer_expire
	add	x0, x0, :lo12:.L.str.28
	add	x1, x1, :lo12:.L__func__.ufs_turbo_check_timer_expire
	add	x29, sp, #16            // =16
	str	w9, [sp, #8]
	str	w8, [sp]
	bl	printk
	ldr	x0, [x20]
	ldr	x21, [x0, #40]
	bl	mas_blk_get_queue_by_lld
	mov	w1, wzr
	blr	x21
	ldr	w8, [x19, :lo12:ufs_turbo_check_timer_expire.check_count]
	cbz	w8, .LBB55_3
// %bb.1:
	ldr	w9, [x20, #24]
	ldr	w10, [x20, #16]
	add	w9, w10, w9
	cmp	w9, #0                  // =0
	b.le	.LBB55_4
// %bb.2:
	mov	w9, #1
	b	.LBB55_5
.LBB55_3:
	mov	w8, #18
	strb	wzr, [x20, #536]
	str	w8, [x19, :lo12:ufs_turbo_check_timer_expire.check_count]
	b	.LBB55_6
.LBB55_4:
	mov	w0, #1
	bl	blk_power_off_flush
	ldr	w8, [x19, :lo12:ufs_turbo_check_timer_expire.check_count]
	mov	w9, wzr
.LBB55_5:
	sub	w8, w8, #1              // =1
	strb	w9, [x20, #536]
	str	w8, [x19, :lo12:ufs_turbo_check_timer_expire.check_count]
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	add	x0, x20, #344           // =344
	add	x1, x8, #125            // =125
	bl	mod_timer
.LBB55_6:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldr	x21, [sp, #32]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #64             // =64
	ret
.Lfunc_end55:
	.size	__ufs_turbo_check_timer_expire, .Lfunc_end55-__ufs_turbo_check_timer_expire
                                        // -- End function
	.p2align	2               // -- Begin function ufs_datasync_work
	.type	ufs_datasync_work,@function
ufs_datasync_work:                      // @ufs_datasync_work
// %bb.0:
	sub	sp, sp, #32             // =32
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	adrp	x0, .L.str.29
	mov	w9, #1
	add	x0, x0, :lo12:.L.str.29
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	add	x29, sp, #16            // =16
	str	x8, [sp, #8]
	str	w9, [sp, #4]
	bl	printk
	adrp	x0, ufs_sync_fs
	add	x0, x0, :lo12:ufs_sync_fs
	add	x1, sp, #4              // =4
	bl	iterate_supers
	adrp	x0, .L.str.30
	add	x0, x0, :lo12:.L.str.30
	bl	printk
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB56_2
// %bb.1:
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #32             // =32
	ret
.LBB56_2:
	bl	__stack_chk_fail
.Lfunc_end56:
	.size	ufs_datasync_work, .Lfunc_end56-ufs_datasync_work
                                        // -- End function
	.p2align	2               // -- Begin function ufs_sync_fs
	.type	ufs_sync_fs,@function
ufs_sync_fs:                            // @ufs_sync_fs
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldrb	w8, [x0, #80]
	mov	x29, sp
	tbnz	w8, #0, .LBB57_3
// %bb.1:
	ldr	x8, [x0, #48]
	ldr	x8, [x8, #64]
	cbz	x8, .LBB57_3
// %bb.2:
	ldr	w1, [x1]
	blr	x8
.LBB57_3:
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end57:
	.size	ufs_sync_fs, .Lfunc_end57-ufs_sync_fs
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_async_sched_fifo_init
	.type	ufs_mq_async_sched_fifo_init,@function
ufs_mq_async_sched_fifo_init:           // @ufs_mq_async_sched_fifo_init
// %bb.0:
	ldrb	w8, [x0, #104]
	cbz	w8, .LBB58_2
// %bb.1:
	ret
.LBB58_2:
	add	x8, x0, #112            // =112
	mov	w9, #1
	str	x8, [x0, #112]
	str	x8, [x0, #120]
	strb	w9, [x0, #104]
	ret
.Lfunc_end58:
	.size	ufs_mq_async_sched_fifo_init, .Lfunc_end58-ufs_mq_async_sched_fifo_init
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_async_sched_fifo_insert
	.type	ufs_mq_async_sched_fifo_insert,@function
ufs_mq_async_sched_fifo_insert:         // @ufs_mq_async_sched_fifo_insert
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	mov	x19, x0
	ldr	x0, [x0]
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x20, x1
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB59_3
// %bb.1:
	ldr	x21, [x20, #120]
	add	x22, x19, #80           // =80
	add	x23, x20, #112          // =112
	mov	x0, x22
	mov	x1, x21
	mov	x2, x23
	bl	__list_add_valid
	tbz	w0, #0, .LBB59_6
// %bb.2:
	str	x22, [x20, #120]
	stp	x23, x21, [x19, #80]
	b	.LBB59_5
.LBB59_3:
	ldr	x21, [x20, #120]
	add	x22, x19, #704          // =704
	add	x23, x20, #112          // =112
	mov	x0, x22
	mov	x1, x21
	mov	x2, x23
	bl	__list_add_valid
	tbz	w0, #0, .LBB59_6
// %bb.4:
	str	x22, [x20, #120]
	str	x23, [x19, #704]
	str	x21, [x19, #712]
.LBB59_5:
	str	x22, [x21]
.LBB59_6:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end59:
	.size	ufs_mq_async_sched_fifo_insert, .Lfunc_end59-ufs_mq_async_sched_fifo_insert
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_async_sched_fifo_seek
	.type	ufs_mq_async_sched_fifo_seek,@function
ufs_mq_async_sched_fifo_seek:           // @ufs_mq_async_sched_fifo_seek
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x8, x0
	ldr	x19, [x8, #112]!
	mov	x29, sp
	cmp	x19, x8
	b.ne	.LBB60_2
// %bb.1:
	ldr	x9, [x0, #120]
	cmp	x9, x8
	b.eq	.LBB60_9
.LBB60_2:
	ldr	x8, [x0]
	ldrb	w8, [x8, #25]
	tbnz	w8, #1, .LBB60_4
// %bb.3:
	sub	x20, x19, #704          // =704
	b	.LBB60_5
.LBB60_4:
	sub	x20, x19, #80           // =80
.LBB60_5:
	mov	x0, x19
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB60_7
// %bb.6:
	ldp	x9, x8, [x19]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB60_7:
	str	x19, [x19]
	str	x19, [x19, #8]
.LBB60_8:
	mov	x0, x20
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB60_9:
	mov	x20, xzr
	b	.LBB60_8
.Lfunc_end60:
	.size	ufs_mq_async_sched_fifo_seek, .Lfunc_end60-ufs_mq_async_sched_fifo_seek
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_async_sched_fifo_requeue
	.type	ufs_mq_async_sched_fifo_requeue,@function
ufs_mq_async_sched_fifo_requeue:        // @ufs_mq_async_sched_fifo_requeue
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x20, x0
	ldr	x0, [x0]
	mov	x29, sp
	mov	x19, x1
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB61_3
// %bb.1:
	ldr	x22, [x19, #112]!
	add	x21, x20, #80           // =80
	mov	x0, x21
	mov	x1, x19
	mov	x2, x22
	bl	__list_add_valid
	tbz	w0, #0, .LBB61_6
// %bb.2:
	str	x21, [x22, #8]
	stp	x22, x19, [x20, #80]
	b	.LBB61_5
.LBB61_3:
	ldr	x22, [x19, #112]!
	add	x21, x20, #704          // =704
	mov	x0, x21
	mov	x1, x19
	mov	x2, x22
	bl	__list_add_valid
	tbz	w0, #0, .LBB61_6
// %bb.4:
	str	x21, [x22, #8]
	str	x22, [x20, #704]
	str	x19, [x20, #712]
.LBB61_5:
	str	x21, [x19]
.LBB61_6:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end61:
	.size	ufs_mq_async_sched_fifo_requeue, .Lfunc_end61-ufs_mq_async_sched_fifo_requeue
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_async_sched_fifo_attempt_merge_bio
	.type	ufs_mq_async_sched_fifo_attempt_merge_bio,@function
ufs_mq_async_sched_fifo_attempt_merge_bio: // @ufs_mq_async_sched_fifo_attempt_merge_bio
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x1, #1376]
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x20, x0
	mov	x0, x1
	ldr	x23, [x8, #536]
	mov	x29, sp
	mov	w19, w2
	mov	x21, x1
	bl	blk_queue_query_unistore_enable
	tbz	w0, #0, .LBB62_9
// %bb.1:
	ldr	x8, [x21, #1376]
	ldr	x8, [x8, #536]
	ldr	x22, [x8, #120]
	add	x23, x8, #112           // =112
	b	.LBB62_4
.LBB62_2:                               //   in Loop: Header=BB62_4 Depth=1
	mov	x0, x21
	mov	x1, x20
	mov	w2, w19
	bl	bio_attempt_front_merge
	tbnz	w0, #0, .LBB62_17
.LBB62_3:                               //   in Loop: Header=BB62_4 Depth=1
	ldr	x22, [x22, #8]
.LBB62_4:                               // =>This Inner Loop Header: Depth=1
	cmp	x23, x22
	b.eq	.LBB62_18
// %bb.5:                               //   in Loop: Header=BB62_4 Depth=1
	sub	x21, x22, #80           // =80
	mov	x0, x21
	mov	x1, x20
	bl	blk_rq_merge_ok
	tbz	w0, #0, .LBB62_3
// %bb.6:                               //   in Loop: Header=BB62_4 Depth=1
	mov	x0, x21
	mov	x1, x20
	bl	blk_try_merge
	cmp	w0, #1                  // =1
	b.eq	.LBB62_2
// %bb.7:                               //   in Loop: Header=BB62_4 Depth=1
	cmp	w0, #2                  // =2
	b.ne	.LBB62_3
// %bb.8:                               //   in Loop: Header=BB62_4 Depth=1
	mov	x0, x21
	mov	x1, x20
	mov	w2, w19
	bl	bio_attempt_back_merge
	tbz	w0, #0, .LBB62_3
	b	.LBB62_17
.LBB62_9:
	ldr	x22, [x23, #120]
	add	x23, x23, #112          // =112
	b	.LBB62_12
.LBB62_10:                              //   in Loop: Header=BB62_12 Depth=1
	mov	x0, x21
	mov	x1, x20
	mov	w2, w19
	bl	bio_attempt_front_merge
	tbnz	w0, #0, .LBB62_17
.LBB62_11:                              //   in Loop: Header=BB62_12 Depth=1
	ldr	x22, [x22, #8]
.LBB62_12:                              // =>This Inner Loop Header: Depth=1
	cmp	x23, x22
	b.eq	.LBB62_18
// %bb.13:                              //   in Loop: Header=BB62_12 Depth=1
	sub	x21, x22, #704          // =704
	mov	x0, x21
	mov	x1, x20
	bl	blk_rq_merge_ok
	tbz	w0, #0, .LBB62_11
// %bb.14:                              //   in Loop: Header=BB62_12 Depth=1
	mov	x0, x21
	mov	x1, x20
	bl	blk_try_merge
	cmp	w0, #1                  // =1
	b.eq	.LBB62_10
// %bb.15:                              //   in Loop: Header=BB62_12 Depth=1
	cmp	w0, #2                  // =2
	b.ne	.LBB62_11
// %bb.16:                              //   in Loop: Header=BB62_12 Depth=1
	mov	x0, x21
	mov	x1, x20
	mov	w2, w19
	bl	bio_attempt_back_merge
	tbz	w0, #0, .LBB62_11
.LBB62_17:
	mov	w0, #1
	b	.LBB62_19
.LBB62_18:
	mov	w0, wzr
.LBB62_19:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end62:
	.size	ufs_mq_async_sched_fifo_attempt_merge_bio, .Lfunc_end62-ufs_mq_async_sched_fifo_attempt_merge_bio
                                        // -- End function
	.p2align	2               // -- Begin function ufs_mq_async_sched_fifo_empty
	.type	ufs_mq_async_sched_fifo_empty,@function
ufs_mq_async_sched_fifo_empty:          // @ufs_mq_async_sched_fifo_empty
// %bb.0:
	ldr	x8, [x0, #112]!
	cmp	x0, x8
	cset	w0, eq
	ret
.Lfunc_end63:
	.size	ufs_mq_async_sched_fifo_empty, .Lfunc_end63-ufs_mq_async_sched_fifo_empty
                                        // -- End function
	.type	.L.str,@object          // @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"\0013[BLK-IO]%s, blk_dev_lld get failed\n"
	.size	.L.str, 38

	.type	.L__func__.mas_blk_is_disorder_stuck,@object // @__func__.mas_blk_is_disorder_stuck
.L__func__.mas_blk_is_disorder_stuck:
	.asciz	"mas_blk_is_disorder_stuck"
	.size	.L__func__.mas_blk_is_disorder_stuck, 26

	.type	.L.str.1,@object        // @.str.1
.L.str.1:
	.asciz	"\0013[BLK-IO]%s, mas_ufs_sched_ds_lld get failed\n"
	.size	.L.str.1, 47

	.type	.L.str.2,@object        // @.str.2
.L.str.2:
	.asciz	"\0013[BLK-IO]%s, stream_type error: %u\n"
	.size	.L.str.2, 37

	.type	.L.str.3,@object        // @.str.3
.L.str.3:
	.asciz	"\0013[BLK-IO]%s: get hd_struct failed%d\n"
	.size	.L.str.3, 38

	.type	.L.str.4,@object        // @.str.4
.L.str.4:
	.asciz	"\0013[BLK-IO]q get budget %d"
	.size	.L.str.4, 26

	.type	.L.str.5,@object        // @.str.5
.L.str.5:
	.asciz	"\0013[BLK-IO]q put budget %d"
	.size	.L.str.5, 26

	.type	.L.str.6,@object        // @.str.6
.L.str.6:
	.asciz	"\0013[BLK-IO]hp_sync_disp_list:\n"
	.size	.L.str.6, 30

	.type	.L.str.7,@object        // @.str.7
.L.str.7:
	.asciz	"\0013[BLK-IO]sync_disp_list:\n"
	.size	.L.str.7, 27

	.type	.L.str.8,@object        // @.str.8
.L.str.8:
	.asciz	"\0013[BLK-IO]last_run_cnt: %lld"
	.size	.L.str.8, 29

	.type	.L.str.9,@object        // @.str.9
.L.str.9:
	.asciz	"\0013[BLK-IO]last_queue_tm(%d): %lld"
	.size	.L.str.9, 34

	.type	.L.str.10,@object       // @.str.10
.L.str.10:
	.asciz	"\0013[BLK-IO]async_fifo_list:\n"
	.size	.L.str.10, 28

	.type	.L__const.ufs_mq_flush_plug_list.bd,@object // @__const.ufs_mq_flush_plug_list.bd
	.section	.rodata.cst16,"aM",@progbits,16
	.p2align	3
.L__const.ufs_mq_flush_plug_list.bd:
	.xword	0
	.byte	1                       // 0x1
	.zero	7
	.size	.L__const.ufs_mq_flush_plug_list.bd, 16

	.type	g_recovery_pwron_info,@object // @g_recovery_pwron_info
	.bss
	.globl	g_recovery_pwron_info
	.p2align	3
g_recovery_pwron_info:
.Lg_recovery_pwron_info$local:
	.zero	160
	.size	g_recovery_pwron_info, 160

	.type	io_guard_queue_list_lock,@object // @io_guard_queue_list_lock
	.local	io_guard_queue_list_lock
	.comm	io_guard_queue_list_lock,4,4
	.type	mas_io_guard_queue_list,@object // @mas_io_guard_queue_list
	.data
	.p2align	3
mas_io_guard_queue_list:
	.xword	mas_io_guard_queue_list
	.xword	mas_io_guard_queue_list
	.size	mas_io_guard_queue_list, 16

	.type	mas_blk_io_guard_wq,@object // @mas_blk_io_guard_wq
	.local	mas_blk_io_guard_wq
	.comm	mas_blk_io_guard_wq,8,8
	.type	mas_io_guard_work,@object // @mas_io_guard_work
	.local	mas_io_guard_work
	.comm	mas_io_guard_work,136,8
	.type	mas_blk_mq_sync_disp_wq,@object // @mas_blk_mq_sync_disp_wq
	.local	mas_blk_mq_sync_disp_wq
	.comm	mas_blk_mq_sync_disp_wq,8,8
	.type	.L.str.12,@object       // @.str.12
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.12:
	.asciz	"\0013[BLK-IO]%s: vip_wait_cnt: %d\n"
	.size	.L.str.12, 32

	.type	.L.str.13,@object       // @.str.13
.L.str.13:
	.asciz	"\0013[BLK-IO]%s: h_tag_used_cnt: %d tag_used_cnt: %d r_tag_used_cnt: %d\n"
	.size	.L.str.13, 70

	.type	.L.str.14,@object       // @.str.14
.L.str.14:
	.asciz	"\0013[BLK-IO]%s: fg_inflt: %d, vip_inflt: %d, s_inflt: %d a_inflt: %d, cp_inflt: %d\n"
	.size	.L.str.14, 82

	.type	.L.str.15,@object       // @.str.15
.L.str.15:
	.asciz	"\0013[BLK-IO]%s %d Failed to alloc sched_ds!\n"
	.size	.L.str.15, 43

	.type	.L__func__.ufs_mq_iosched_init,@object // @__func__.ufs_mq_iosched_init
.L__func__.ufs_mq_iosched_init:
	.asciz	"ufs_mq_iosched_init"
	.size	.L__func__.ufs_mq_iosched_init, 20

	.type	mas_ufs_mq_async_io_fifo_sched,@object // @mas_ufs_mq_async_io_fifo_sched
	.data
	.p2align	3
mas_ufs_mq_async_io_fifo_sched:
	.word	0                       // 0x0
	.zero	4
	.xword	ufs_mq_async_sched_fifo_init
	.xword	ufs_mq_async_sched_fifo_insert
	.xword	ufs_mq_async_sched_fifo_seek
	.xword	ufs_mq_async_sched_fifo_requeue
	.xword	ufs_mq_async_sched_fifo_attempt_merge_bio
	.xword	ufs_mq_async_sched_fifo_empty
	.size	mas_ufs_mq_async_io_fifo_sched, 56

	.type	mas_ufs_mq,@object      // @mas_ufs_mq
	.globl	mas_ufs_mq
	.p2align	3
mas_ufs_mq:
.Lmas_ufs_mq$local:
	.xword	mas_ufs_mq_async_io_fifo_sched
	.size	mas_ufs_mq, 8

	.type	.L.str.16,@object       // @.str.16
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.16:
	.asciz	"\0013[BLK-IO]%s, reserved tag used cnt: %d\n"
	.size	.L.str.16, 41

	.type	.L__func__.ufs_mq_get_recovery_tag,@object // @__func__.ufs_mq_get_recovery_tag
.L__func__.ufs_mq_get_recovery_tag:
	.asciz	"ufs_mq_get_recovery_tag"
	.size	.L__func__.ufs_mq_get_recovery_tag, 24

	.type	.L.str.17,@object       // @.str.17
.L.str.17:
	.asciz	"\0013[BLK-IO]%s, there is no tag left\n"
	.size	.L.str.17, 36

	.type	.L__func__.ufs_mq_get_all_tag,@object // @__func__.ufs_mq_get_all_tag
.L__func__.ufs_mq_get_all_tag:
	.asciz	"ufs_mq_get_all_tag"
	.size	.L__func__.ufs_mq_get_all_tag, 19

	.type	.L.str.18,@object       // @.str.18
.L.str.18:
	.asciz	"\0013[BLK-IO]lst_in_t: %lld, lst_out_t: %lld, fail_exit: %lld, exit_cnt: %lld\n"
	.size	.L.str.18, 76

	.type	.L.str.19,@object       // @.str.19
.L.str.19:
	.asciz	"\0013[BLK-IO]%s ret %d\n"
	.size	.L.str.19, 21

	.type	.L__func__.mas_blk_recovery_pwron_info_sync,@object // @__func__.mas_blk_recovery_pwron_info_sync
.L__func__.mas_blk_recovery_pwron_info_sync:
	.asciz	"mas_blk_recovery_pwron_info_sync"
	.size	.L__func__.mas_blk_recovery_pwron_info_sync, 33

	.type	.L.str.20,@object       // @.str.20
.L.str.20:
	.asciz	"\0013[BLK-IO]%s ret %d"
	.size	.L.str.20, 20

	.type	.L__func__.mas_blk_recovery_done_proc,@object // @__func__.mas_blk_recovery_done_proc
.L__func__.mas_blk_recovery_done_proc:
	.asciz	"mas_blk_recovery_done_proc"
	.size	.L__func__.mas_blk_recovery_done_proc, 27

	.type	.L.str.21,@object       // @.str.21
.L.str.21:
	.asciz	"\0013[BLK-IO]recovery_stor, %d, pre_pu: 0x%llx (0x%llx - 0x%llx) - 0x%llx, pre_lba: 0x%llx - 0x%llx, new_lba: 0x%llx - 0x%llx\n"
	.size	.L.str.21, 124

	.type	.L.str.22,@object       // @.str.22
.L.str.22:
	.asciz	"\0013[BLK-IO]%s pwron_type is err %u\n"
	.size	.L.str.22, 35

	.type	.L__func__.mas_blk_recovery_pwron_info_done,@object // @__func__.mas_blk_recovery_pwron_info_done
.L__func__.mas_blk_recovery_pwron_info_done:
	.asciz	"mas_blk_recovery_pwron_info_done"
	.size	.L__func__.mas_blk_recovery_pwron_info_done, 33

	.type	.L.str.24,@object       // @.str.24
.L.str.24:
	.asciz	"\0013[BLK-IO]%s - section size: %u, pu size: %u\n"
	.size	.L.str.24, 46

	.type	.L__func__.ufs_mq_dispatch_match_expected_lba,@object // @__func__.ufs_mq_dispatch_match_expected_lba
.L__func__.ufs_mq_dispatch_match_expected_lba:
	.asciz	"ufs_mq_dispatch_match_expected_lba"
	.size	.L__func__.ufs_mq_dispatch_match_expected_lba, 35

	.type	mas_blk_mq_async_disp_wq,@object // @mas_blk_mq_async_disp_wq
	.local	mas_blk_mq_async_disp_wq
	.comm	mas_blk_mq_async_disp_wq,8,8
	.type	.L.str.26,@object       // @.str.26
.L.str.26:
	.asciz	"dump"
	.size	.L.str.26, 5

	.type	.L.str.27,@object       // @.str.27
.L.str.27:
	.asciz	"io_latency"
	.size	.L.str.27, 11

	.type	ufs_turbo_check_timer_expire.check_count,@object // @ufs_turbo_check_timer_expire.check_count
	.data
	.p2align	2
ufs_turbo_check_timer_expire.check_count:
	.word	18                      // 0x12
	.size	ufs_turbo_check_timer_expire.check_count, 4

	.type	.L.str.28,@object       // @.str.28
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.28:
	.asciz	"\0013[BLK-IO]%s: check_count = %d turbo_mode = %d prio io = %d, sync io = %d fg_inflt: %d, vip_inflt: %d, s_inflt: %d a_inflt: %d\n"
	.size	.L.str.28, 128

	.type	.L__func__.ufs_turbo_check_timer_expire,@object // @__func__.ufs_turbo_check_timer_expire
.L__func__.ufs_turbo_check_timer_expire:
	.asciz	"ufs_turbo_check_timer_expire"
	.size	.L__func__.ufs_turbo_check_timer_expire, 29

	.type	.L.str.29,@object       // @.str.29
.L.str.29:
	.asciz	"\0013[BLK-IO]UFS Sync start\n"
	.size	.L.str.29, 26

	.type	.L.str.30,@object       // @.str.30
.L.str.30:
	.asciz	"\0013[BLK-IO]UFS Sync complete\n"
	.size	.L.str.30, 29

	.type	.L.str.31,@object       // @.str.31
.L.str.31:
	.asciz	"sync_dispatch"
	.size	.L.str.31, 14

	.type	.L.str.32,@object       // @.str.32
.L.str.32:
	.asciz	"\0013[BLK-IO]%s %d Failed to alloc sync_dispatch_workqueue\n"
	.size	.L.str.32, 57

	.type	.L__func__.ufs_mq_workqueue_init,@object // @__func__.ufs_mq_workqueue_init
.L__func__.ufs_mq_workqueue_init:
	.asciz	"ufs_mq_workqueue_init"
	.size	.L__func__.ufs_mq_workqueue_init, 22

	.type	.L.str.33,@object       // @.str.33
.L.str.33:
	.asciz	"async_dispatch"
	.size	.L.str.33, 15

	.type	.L.str.34,@object       // @.str.34
.L.str.34:
	.asciz	"io_guard"
	.size	.L.str.34, 9

	.type	.L.str.35,@object       // @.str.35
.L.str.35:
	.asciz	"\0013[BLK-IO]%s init Failed!\n"
	.size	.L.str.35, 27

	.type	.Lswitch.table.ufs_mq_sync_dispatch,@object // @switch.table.ufs_mq_sync_dispatch
	.section	.rodata,"a",@progbits
	.p2align	2
.Lswitch.table.ufs_mq_sync_dispatch:
	.word	0                       // 0x0
	.word	0                       // 0x0
	.word	0                       // 0x0
	.word	0                       // 0x0
	.word	1                       // 0x1
	.word	0                       // 0x0
	.word	0                       // 0x0
	.size	.Lswitch.table.ufs_mq_sync_dispatch, 28

	.ident	"Android (6443078 based on r383902) clang version 11.0.1 (https://android.googlesource.com/toolchain/llvm-project b397f81060ce6d701042b782172ed13bee898b79)"
	.section	".note.GNU-stack","",@progbits
