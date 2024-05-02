	.text
	.file	"mas_blk_core.c"
	.globl	mas_blk_get_lld         // -- Begin function mas_blk_get_lld
	.p2align	2
	.type	mas_blk_get_lld,@function
mas_blk_get_lld:                        // @mas_blk_get_lld
.Lmas_blk_get_lld$local:
// %bb.0:
	ldr	x8, [x0, #48]
	cbz	x8, .LBB0_2
// %bb.1:
	ldr	x8, [x0, #3424]
	add	x9, x0, #1392           // =1392
	add	x10, x8, #64            // =64
	cmp	x8, #0                  // =0
	csel	x0, x9, x10, eq
	ret
.LBB0_2:
	add	x0, x0, #1392           // =1392
	ret
.Lfunc_end0:
	.size	mas_blk_get_lld, .Lfunc_end0-mas_blk_get_lld
                                        // -- End function
	.globl	mas_blk_get_queue_by_lld // -- Begin function mas_blk_get_queue_by_lld
	.p2align	2
	.type	mas_blk_get_queue_by_lld,@function
mas_blk_get_queue_by_lld:               // @mas_blk_get_queue_by_lld
.Lmas_blk_get_queue_by_lld$local:
// %bb.0:
	ldr	w8, [x0, #4]
	cmp	w8, #2                  // =2
	b.eq	.LBB1_4
// %bb.1:
	cmp	w8, #1                  // =1
	b.eq	.LBB1_5
// %bb.2:
	cbnz	w8, .LBB1_6
// %bb.3:
	ldr	x0, [x0, #8]
	ret
.LBB1_4:
	ldr	x8, [x0, #8]
	ldr	x8, [x8, #1616]
	sub	x0, x8, #3432           // =3432
	ret
.LBB1_5:
	ldr	x8, [x0, #8]
	ldr	x8, [x8, #80]
	sub	x0, x8, #3432           // =3432
	ret
.LBB1_6:
	mov	x0, xzr
	ret
.Lfunc_end1:
	.size	mas_blk_get_queue_by_lld, .Lfunc_end1-mas_blk_get_queue_by_lld
                                        // -- End function
	.globl	blk_dio_ck              // -- Begin function blk_dio_ck
	.p2align	2
	.type	blk_dio_ck,@function
blk_dio_ck:                             // @blk_dio_ck
.Lblk_dio_ck$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB2_12
// %bb.1:
	ldr	x8, [x0, #1264]
	cbz	x8, .LBB2_12
// %bb.2:
	mov	w19, w3
	mov	x20, x1
	cmp	w2, #1                  // =1
	b.eq	.LBB2_7
// %bb.3:
	cbnz	w2, .LBB2_12
// %bb.4:
	ldr	x9, [x8, #48]
	cbz	x9, .LBB2_13
// %bb.5:
	ldr	x9, [x8, #3424]
	add	x8, x8, #1392           // =1392
	add	x10, x9, #64            // =64
	cmp	x9, #0                  // =0
	csel	x8, x8, x10, eq
.LBB2_6:
	ldrsw	x21, [x8, #1448]
	cbnz	w21, .LBB2_9
	b	.LBB2_12
.LBB2_7:
	ldr	x9, [x8, #48]
	cbz	x9, .LBB2_14
// %bb.8:
	ldr	x9, [x8, #3424]
	add	x8, x8, #1392           // =1392
	add	x10, x9, #64            // =64
	cmp	x9, #0                  // =0
	csel	x8, x8, x10, eq
	ldrsw	x21, [x8, #1452]
	cbz	w21, .LBB2_12
.LBB2_9:
	bl	ktime_get
	sub	x8, x0, x20
	cmn	x8, #999                // =999
	b.lt	.LBB2_12
// %bb.10:
	mov	x9, #63439
	movk	x9, #58195, lsl #16
	movk	x9, #39845, lsl #32
	movk	x9, #8388, lsl #48
	mov	w10, #3906
	smulh	x8, x8, x9
	smull	x9, w19, w10
	asr	x10, x8, #7
	add	x8, x10, x8, lsr #63
	sdiv	x9, x9, x21
	sub	x0, x9, x8
	cmp	x0, #1                  // =1
	b.lt	.LBB2_12
// %bb.11:
	bl	__udelay
.LBB2_12:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB2_13:
	add	x8, x8, #1392           // =1392
	b	.LBB2_6
.LBB2_14:
	add	x8, x8, #1392           // =1392
	ldrsw	x21, [x8, #1452]
	cbnz	w21, .LBB2_9
	b	.LBB2_12
.Lfunc_end2:
	.size	blk_dio_ck, .Lfunc_end2-blk_dio_ck
                                        // -- End function
	.globl	blk_mq_tagset_vl_setup  // -- Begin function blk_mq_tagset_vl_setup
	.p2align	2
	.type	blk_mq_tagset_vl_setup,@function
blk_mq_tagset_vl_setup:                 // @blk_mq_tagset_vl_setup
.Lblk_mq_tagset_vl_setup$local:
// %bb.0:
	lsr	x8, x1, #28
	mov	w9, #500
	mov	w10, #300
	cmp	x8, #0                  // =0
	csel	w8, w10, w9, eq
	str	wzr, [x0, #1512]
	str	w8, [x0, #1516]
	ret
.Lfunc_end3:
	.size	blk_mq_tagset_vl_setup, .Lfunc_end3-blk_mq_tagset_vl_setup
                                        // -- End function
	.globl	blk_queue_set_inline_crypto_flag // -- Begin function blk_queue_set_inline_crypto_flag
	.p2align	2
	.type	blk_queue_set_inline_crypto_flag,@function
blk_queue_set_inline_crypto_flag:       // @blk_queue_set_inline_crypto_flag
.Lblk_queue_set_inline_crypto_flag$local:
// %bb.0:
	ldr	x8, [x0, #48]
	cbz	x8, .LBB4_3
// %bb.1:
	ldr	x8, [x0, #3424]
	add	x9, x0, #1392           // =1392
	add	x10, x8, #64            // =64
	cmp	x8, #0                  // =0
	csel	x8, x9, x10, eq
.LBB4_2:
	ldr	x9, [x8, #24]
	tst	w1, #0x1
	and	x10, x9, #0xffffffffffffffef
	orr	x9, x9, #0x10
	csel	x9, x9, x10, ne
	str	x9, [x8, #24]
	ret
.LBB4_3:
	add	x8, x0, #1392           // =1392
	b	.LBB4_2
.Lfunc_end4:
	.size	blk_queue_set_inline_crypto_flag, .Lfunc_end4-blk_queue_set_inline_crypto_flag
                                        // -- End function
	.globl	mas_blk_queue_usr_ctrl_set // -- Begin function mas_blk_queue_usr_ctrl_set
	.p2align	2
	.type	mas_blk_queue_usr_ctrl_set,@function
mas_blk_queue_usr_ctrl_set:             // @mas_blk_queue_usr_ctrl_set
.Lmas_blk_queue_usr_ctrl_set$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x8, [x0, #1080]
	mov	w9, #28570
	movk	w9, #4, lsl #16
	mov	x29, sp
	cmp	x8, x9
	b.ne	.LBB5_2
// %bb.1:
	mov	w8, #100
	str	x8, [x0, #1312]
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	mov	w9, #48032
	movk	w9, #13, lsl #16
	add	x0, x0, #1320           // =1320
	add	x1, x8, x9
	bl	mod_timer
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB5_2:
	str	xzr, [x0, #1312]
	add	x0, x0, #1320           // =1320
	bl	del_timer_sync
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end5:
	.size	mas_blk_queue_usr_ctrl_set, .Lfunc_end5-mas_blk_queue_usr_ctrl_set
                                        // -- End function
	.globl	mas_blk_queue_usr_ctrl_recovery_timer_expire // -- Begin function mas_blk_queue_usr_ctrl_recovery_timer_expire
	.p2align	2
	.type	mas_blk_queue_usr_ctrl_recovery_timer_expire,@function
mas_blk_queue_usr_ctrl_recovery_timer_expire: // @mas_blk_queue_usr_ctrl_recovery_timer_expire
.Lmas_blk_queue_usr_ctrl_recovery_timer_expire$local:
// %bb.0:
	str	xzr, [x0, #1080]
	str	xzr, [x0, #1312]
	ret
.Lfunc_end6:
	.size	mas_blk_queue_usr_ctrl_recovery_timer_expire, .Lfunc_end6-mas_blk_queue_usr_ctrl_recovery_timer_expire
                                        // -- End function
	.globl	mas_blk_bio_clone_fast  // -- Begin function mas_blk_bio_clone_fast
	.p2align	2
	.type	mas_blk_bio_clone_fast,@function
mas_blk_bio_clone_fast:                 // @mas_blk_bio_clone_fast
.Lmas_blk_bio_clone_fast$local:
// %bb.0:
	ldr	x8, [x1, #80]
	str	x8, [x0, #80]
	ldrb	w8, [x1, #264]
	strb	w8, [x0, #264]
	ldrb	w8, [x1, #265]
	strb	w8, [x0, #265]
	ldr	x8, [x1, #272]
	str	x8, [x0, #272]
	ldr	x8, [x1, #280]
	str	x8, [x0, #280]
	ldrb	w8, [x1, #288]
	strb	w8, [x0, #288]
	ldrb	w8, [x1, #289]
	strb	w8, [x0, #289]
	ret
.Lfunc_end7:
	.size	mas_blk_bio_clone_fast, .Lfunc_end7-mas_blk_bio_clone_fast
                                        // -- End function
	.globl	mas_blk_bio_queue_split // -- Begin function mas_blk_bio_queue_split
	.p2align	2
	.type	mas_blk_bio_queue_split,@function
mas_blk_bio_queue_split:                // @mas_blk_bio_queue_split
.Lmas_blk_bio_queue_split$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x1]
	mov	x20, x0
	mov	x29, sp
	mov	x19, x2
	mov	x0, x8
	bl	mas_blk_busyidle_check_bio_endio
	mov	x0, x20
	mov	x1, x19
	bl	mas_blk_busyidle_check_bio
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end8:
	.size	mas_blk_bio_queue_split, .Lfunc_end8-mas_blk_bio_queue_split
                                        // -- End function
	.globl	mas_blk_bio_split_pre   // -- Begin function mas_blk_bio_split_pre
	.p2align	2
	.type	mas_blk_bio_split_pre,@function
mas_blk_bio_split_pre:                  // @mas_blk_bio_split_pre
.Lmas_blk_bio_split_pre$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0, #296]
	mov	x29, sp
	cbz	x8, .LBB9_4
// %bb.1:
	ldr	w9, [x0, #304]
	mov	x19, x0
	mov	x20, x1
	cmp	w9, #48                 // =48
	b.eq	.LBB9_3
// %bb.2:
	cmp	w9, #64                 // =64
	b.ne	.LBB9_5
.LBB9_3:
	str	x8, [x20, #296]
	ldr	w8, [x19, #304]
	str	w8, [x20, #304]
	ldr	w8, [x19, #308]
	str	w8, [x20, #308]
	ldr	x8, [x19, #312]
	str	x8, [x20, #312]
.LBB9_4:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB9_5:
	adrp	x0, .L.str.5
	adrp	x1, .L__func__.mas_blk_lld_inline_crypto_bio_split_pre
	add	x0, x0, :lo12:.L.str.5
	add	x1, x1, :lo12:.L__func__.mas_blk_lld_inline_crypto_bio_split_pre
	bl	printk
	ldr	x8, [x19, #296]
	b	.LBB9_3
.Lfunc_end9:
	.size	mas_blk_bio_split_pre, .Lfunc_end9-mas_blk_bio_split_pre
                                        // -- End function
	.globl	mas_blk_bio_split_post  // -- Begin function mas_blk_bio_split_post
	.p2align	2
	.type	mas_blk_bio_split_post,@function
mas_blk_bio_split_post:                 // @mas_blk_bio_split_post
.Lmas_blk_bio_split_post$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	ldr	x8, [x0, #296]
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	cbz	x8, .LBB10_4
// %bb.1:
	ldr	w8, [x0, #304]
	mov	x19, x0
	cmp	w8, #48                 // =48
	b.eq	.LBB10_3
// %bb.2:
	cmp	w8, #64                 // =64
	b.ne	.LBB10_5
.LBB10_3:
	ldr	x8, [x19, #472]
	ldp	w9, w10, [x19, #44]
	add	x8, x8, x9, lsl #4
	ldr	w9, [x8, #12]
	ldr	x8, [x8]
	add	w9, w10, w9
	lsr	w9, w9, #12
	add	x8, x8, x9, lsl #6
	ldr	x8, [x8, #32]
	str	x8, [x19, #312]
.LBB10_4:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB10_5:
	adrp	x0, .L.str.6
	adrp	x1, .L__func__.mas_blk_lld_inline_crypto_bio_split_post
	add	x0, x0, :lo12:.L.str.6
	add	x1, x1, :lo12:.L__func__.mas_blk_lld_inline_crypto_bio_split_post
	bl	printk
	b	.LBB10_3
.Lfunc_end10:
	.size	mas_blk_bio_split_post, .Lfunc_end10-mas_blk_bio_split_post
                                        // -- End function
	.globl	mas_blk_bio_merge_allow // -- Begin function mas_blk_bio_merge_allow
	.p2align	2
	.type	mas_blk_bio_merge_allow,@function
mas_blk_bio_merge_allow:                // @mas_blk_bio_merge_allow
.Lmas_blk_bio_merge_allow$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0]
	mov	x19, x0
	mov	x20, x1
	str	x21, [sp, #16]          // 8-byte Folded Spill
	ldr	x9, [x8, #48]
	mov	x29, sp
	cbz	x9, .LBB11_17
// %bb.1:
	ldr	x9, [x8, #3424]
	add	x8, x8, #1392           // =1392
	add	x10, x9, #64            // =64
	cmp	x9, #0                  // =0
	csel	x8, x8, x10, eq
	ldrb	w8, [x8, #24]
	tbz	w8, #4, .LBB11_12
.LBB11_2:
	ldr	x8, [x20, #296]
	ldr	x9, [x19, #688]
	cbz	x8, .LBB11_11
// %bb.3:
	cbz	x9, .LBB11_11
// %bb.4:
	ldr	w10, [x20, #304]
	ldr	w11, [x19, #696]
	cmp	w10, w11
	b.ne	.LBB11_15
// %bb.5:
	cmp	w10, #48                // =48
	b.eq	.LBB11_7
// %bb.6:
	cmp	w10, #64                // =64
	b.ne	.LBB11_14
.LBB11_7:
	cmp	x8, x9
	b.ne	.LBB11_15
.LBB11_8:
	ldr	x21, [x19, #72]
	mov	x0, x19
	mov	x1, x20
	bl	blk_try_merge
	cmp	w0, #1                  // =1
	b.eq	.LBB11_13
// %bb.9:
	cmp	w0, #2                  // =2
	b.ne	.LBB11_15
// %bb.10:
	ldr	x8, [x21, #312]
	ldrh	w9, [x21, #458]
	ldr	x10, [x20, #312]
	add	x8, x8, x9
	cmp	x8, x10
	b.eq	.LBB11_12
	b	.LBB11_15
.LBB11_11:
	orr	x8, x8, x9
	cbnz	x8, .LBB11_15
.LBB11_12:
	ldr	w8, [x19, #240]
	ldr	w9, [x20, #320]
	eor	w8, w9, w8, lsr #3
	tst	w8, #0x1
	cset	w0, eq
	b	.LBB11_16
.LBB11_13:
	ldr	x8, [x19, #64]
	ldr	x9, [x20, #312]
	ldrh	w10, [x20, #458]
	ldr	x8, [x8, #312]
	add	x9, x9, x10
	cmp	x9, x8
	b.eq	.LBB11_12
	b	.LBB11_15
.LBB11_14:
	adrp	x0, .L.str.7
	adrp	x1, .L__func__.mas_blk_lld_inline_crypto_bio_merge_allow
	add	x0, x0, :lo12:.L.str.7
	add	x1, x1, :lo12:.L__func__.mas_blk_lld_inline_crypto_bio_merge_allow
	bl	printk
	ldr	x8, [x20, #296]
	ldr	x9, [x19, #688]
	cmp	x8, x9
	b.eq	.LBB11_8
.LBB11_15:
	mov	w0, wzr
.LBB11_16:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB11_17:
	add	x8, x8, #1392           // =1392
	ldrb	w8, [x8, #24]
	tbnz	w8, #4, .LBB11_2
	b	.LBB11_12
.Lfunc_end11:
	.size	mas_blk_bio_merge_allow, .Lfunc_end11-mas_blk_bio_merge_allow
                                        // -- End function
	.globl	mas_blk_bio_merge_done  // -- Begin function mas_blk_bio_merge_done
	.p2align	2
	.type	mas_blk_bio_merge_done,@function
mas_blk_bio_merge_done:                 // @mas_blk_bio_merge_done
.Lmas_blk_bio_merge_done$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x0, x1
	mov	x1, x2
	mov	x29, sp
	bl	mas_blk_latency_for_merge
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end12:
	.size	mas_blk_bio_merge_done, .Lfunc_end12-mas_blk_bio_merge_done
                                        // -- End function
	.globl	mas_blk_account_io_completion // -- Begin function mas_blk_account_io_completion
	.p2align	2
	.type	mas_blk_account_io_completion,@function
mas_blk_account_io_completion:          // @mas_blk_account_io_completion
.Lmas_blk_account_io_completion$local:
// %bb.0:
	cbz	x0, .LBB13_7
// %bb.1:
	ldr	x8, [x0]
	ldr	x9, [x8, #48]
	cbz	x9, .LBB13_8
// %bb.2:
	ldr	x9, [x8, #3424]
	add	x8, x8, #1392           // =1392
	add	x10, x9, #64            // =64
	cmp	x9, #0                  // =0
	csel	x8, x8, x10, eq
	cbz	x8, .LBB13_9
.LBB13_3:
	ldrb	w9, [x0, #24]
	cmp	w9, #3                  // =3
	b.ne	.LBB13_5
// %bb.4:
	ldr	x9, [x8, #1384]
	add	x9, x9, w1, uxtw
	str	x9, [x8, #1384]
	ldrb	w9, [x0, #24]
.LBB13_5:
	cmp	w9, #1                  // =1
	b.ne	.LBB13_9
// %bb.6:
	ldr	x9, [x8, #1376]
	mov	w0, wzr
	add	x9, x9, w1, uxtw
	str	x9, [x8, #1376]
	ret
.LBB13_7:
	mov	w0, #-1
	ret
.LBB13_8:
	add	x8, x8, #1392           // =1392
	cbnz	x8, .LBB13_3
.LBB13_9:
	mov	w0, wzr
	ret
.Lfunc_end13:
	.size	mas_blk_account_io_completion, .Lfunc_end13-mas_blk_account_io_completion
                                        // -- End function
	.globl	mas_blk_generic_make_request_check // -- Begin function mas_blk_generic_make_request_check
	.p2align	2
	.type	mas_blk_generic_make_request_check,@function
mas_blk_generic_make_request_check:     // @mas_blk_generic_make_request_check
.Lmas_blk_generic_make_request_check$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0, #8]
	mov	x29, sp
	mov	x19, x0
	ldr	x20, [x8, #1264]
	bl	mas_blk_bio_set_opf_unistore
	mov	x0, x19
	mov	w1, wzr
	bl	mas_blk_latency_bio_check
	mov	x0, x20
	mov	x1, x19
	bl	mas_blk_flush_async_dispatch
	tbnz	w0, #0, .LBB14_3
// %bb.1:
	mov	x0, x20
	mov	x1, x19
	bl	mas_blk_busyidle_check_bio
	mov	w0, wzr
.LBB14_2:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB14_3:
	mov	x0, x19
	strb	wzr, [x19, #26]
	bl	bio_endio
	mov	w0, #1
	b	.LBB14_2
.Lfunc_end14:
	.size	mas_blk_generic_make_request_check, .Lfunc_end14-mas_blk_generic_make_request_check
                                        // -- End function
	.globl	mas_blk_generic_make_request // -- Begin function mas_blk_generic_make_request
	.p2align	2
	.type	mas_blk_generic_make_request,@function
mas_blk_generic_make_request:           // @mas_blk_generic_make_request
.Lmas_blk_generic_make_request$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	w1, #2
	mov	x29, sp
	bl	mas_blk_latency_bio_check
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end15:
	.size	mas_blk_generic_make_request, .Lfunc_end15-mas_blk_generic_make_request
                                        // -- End function
	.globl	mas_blk_start_plug      // -- Begin function mas_blk_start_plug
	.p2align	2
	.type	mas_blk_start_plug,@function
mas_blk_start_plug:                     // @mas_blk_start_plug
.Lmas_blk_start_plug$local:
// %bb.0:
	add	x8, x0, #40             // =40
	str	x8, [x0, #40]
	stp	x8, xzr, [x0, #48]
	ret
.Lfunc_end16:
	.size	mas_blk_start_plug, .Lfunc_end16-mas_blk_start_plug
                                        // -- End function
	.globl	mas_blk_flush_plug_list // -- Begin function mas_blk_flush_plug_list
	.p2align	2
	.type	mas_blk_flush_plug_list,@function
mas_blk_flush_plug_list:                // @mas_blk_flush_plug_list
.Lmas_blk_flush_plug_list$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x8, x0
	ldr	x9, [x8, #40]!
	mov	x29, sp
	cmp	x8, x9
	b.eq	.LBB17_3
// %bb.1:
	ldr	x8, [x0, #56]
	cbz	x8, .LBB17_3
// %bb.2:
	and	w1, w1, #0x1
	blr	x8
.LBB17_3:
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end17:
	.size	mas_blk_flush_plug_list, .Lfunc_end17-mas_blk_flush_plug_list
                                        // -- End function
	.globl	mas_blk_bio_endio       // -- Begin function mas_blk_bio_endio
	.p2align	2
	.type	mas_blk_bio_endio,@function
mas_blk_bio_endio:                      // @mas_blk_bio_endio
.Lmas_blk_bio_endio$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x19, x0
	bl	mas_blk_busyidle_check_bio_endio
	mov	w1, #1
	mov	x0, x19
	bl	mas_blk_latency_bio_check
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end18:
	.size	mas_blk_bio_endio, .Lfunc_end18-mas_blk_bio_endio
                                        // -- End function
	.globl	mas_blk_bio_free        // -- Begin function mas_blk_bio_free
	.p2align	2
	.type	mas_blk_bio_free,@function
mas_blk_bio_free:                       // @mas_blk_bio_free
.Lmas_blk_bio_free$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldrb	w8, [x0, #136]
	mov	x29, sp
	tbnz	w8, #0, .LBB19_2
// %bb.1:
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB19_2:
	bl	mas_blk_busyidle_check_bio_endio
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end19:
	.size	mas_blk_bio_free, .Lfunc_end19-mas_blk_bio_free
                                        // -- End function
	.globl	mas_blk_request_init_from_bio // -- Begin function mas_blk_request_init_from_bio
	.p2align	2
	.type	mas_blk_request_init_from_bio,@function
mas_blk_request_init_from_bio:          // @mas_blk_request_init_from_bio
.Lmas_blk_request_init_from_bio$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x20, x1
	mov	w1, wzr
	mov	x29, sp
	mov	x19, x0
	bl	mas_blk_latency_req_check
	add	x8, x19, #260           // =260
	mov	x0, x19
	mov	x1, x20
	stur	xzr, [x19, #252]
	str	xzr, [x8]
	str	wzr, [x19, #268]
	bl	mas_blk_request_init_from_bio_unistore
	str	x19, [x20, #96]
	ldr	x8, [x19]
	ldr	x9, [x8, #48]
	cbz	x9, .LBB20_10
// %bb.1:
	ldr	x9, [x8, #3424]
	add	x8, x8, #1392           // =1392
	add	x10, x9, #64            // =64
	cmp	x9, #0                  // =0
	csel	x8, x8, x10, eq
	ldrb	w8, [x8, #24]
	tbz	w8, #4, .LBB20_6
.LBB20_2:
	ldr	x8, [x20, #296]
	cbz	x8, .LBB20_6
// %bb.3:
	ldr	w9, [x20, #304]
	cmp	w9, #48                 // =48
	b.eq	.LBB20_5
// %bb.4:
	cmp	w9, #64                 // =64
	b.ne	.LBB20_9
.LBB20_5:
	str	x8, [x19, #688]
	ldr	w8, [x20, #304]
	str	w8, [x19, #696]
	ldr	w8, [x20, #308]
	str	w8, [x19, #700]
.LBB20_6:
	ldrb	w8, [x20, #320]
	tbz	w8, #0, .LBB20_8
// %bb.7:
	ldr	x8, [x19, #240]
	orr	x8, x8, #0x8
	str	x8, [x19, #240]
.LBB20_8:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB20_9:
	adrp	x0, .L.str.8
	adrp	x1, .L__func__.mas_blk_lld_inline_crypto_init_request_from_bio
	add	x0, x0, :lo12:.L.str.8
	add	x1, x1, :lo12:.L__func__.mas_blk_lld_inline_crypto_init_request_from_bio
	mov	w2, #64
	mov	w3, #48
	bl	printk
	ldr	x8, [x20, #296]
	b	.LBB20_5
.LBB20_10:
	add	x8, x8, #1392           // =1392
	ldrb	w8, [x8, #24]
	tbnz	w8, #4, .LBB20_2
	b	.LBB20_6
.Lfunc_end20:
	.size	mas_blk_request_init_from_bio, .Lfunc_end20-mas_blk_request_init_from_bio
                                        // -- End function
	.globl	mas_blk_insert_cloned_request // -- Begin function mas_blk_insert_cloned_request
	.p2align	2
	.type	mas_blk_insert_cloned_request,@function
mas_blk_insert_cloned_request:          // @mas_blk_insert_cloned_request
.Lmas_blk_insert_cloned_request$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x29, sp
	bl	mas_blk_busyidle_check_request_bio
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end21:
	.size	mas_blk_insert_cloned_request, .Lfunc_end21-mas_blk_insert_cloned_request
                                        // -- End function
	.globl	mas_blk_request_execute_nowait // -- Begin function mas_blk_request_execute_nowait
	.p2align	2
	.type	mas_blk_request_execute_nowait,@function
mas_blk_request_execute_nowait:         // @mas_blk_request_execute_nowait
.Lmas_blk_request_execute_nowait$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x29, sp
	bl	mas_blk_busyidle_check_execute_request
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end22:
	.size	mas_blk_request_execute_nowait, .Lfunc_end22-mas_blk_request_execute_nowait
                                        // -- End function
	.globl	mas_blk_mq_rq_ctx_init  // -- Begin function mas_blk_mq_rq_ctx_init
	.p2align	2
	.type	mas_blk_mq_rq_ctx_init,@function
mas_blk_mq_rq_ctx_init:                 // @mas_blk_mq_rq_ctx_init
.Lmas_blk_mq_rq_ctx_init$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	ldr	x8, [x0, #1384]
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x2
	mov	x29, sp
	cbz	x8, .LBB23_3
// %bb.1:
	ldr	x8, [x8, #32]
	cbz	x8, .LBB23_3
// %bb.2:
	mov	x2, x19
	blr	x8
.LBB23_3:
	mov	x8, #-4294967296
	str	xzr, [x19, #688]
	str	x8, [x19, #696]
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end23:
	.size	mas_blk_mq_rq_ctx_init, .Lfunc_end23-mas_blk_mq_rq_ctx_init
                                        // -- End function
	.globl	mas_blk_mq_request_start // -- Begin function mas_blk_mq_request_start
	.p2align	2
	.type	mas_blk_mq_request_start,@function
mas_blk_mq_request_start:               // @mas_blk_mq_request_start
.Lmas_blk_mq_request_start$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	mov	w1, #5
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x19, x0
	bl	mas_blk_latency_req_check
	ldr	x8, [x19]
	mov	w10, #28570
	movk	w10, #4, lsl #16
	ldr	x9, [x8, #1080]
	cmp	x9, x10
	b.eq	.LBB24_2
.LBB24_1:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB24_2:
	ldrb	w9, [x19, #24]
	orr	w10, w9, #0x3
	cmp	w10, #35                // =35
	b.eq	.LBB24_1
// %bb.3:
	ldr	w10, [x19, #48]
	cbz	w10, .LBB24_1
// %bb.4:
	cmp	w9, #1                  // =1
	b.ne	.LBB24_11
// %bb.5:
	cmp	w10, #32, lsl #12       // =131072
	b.hi	.LBB24_8
// %bb.6:
	ldr	x0, [x8, #1312]
	cbz	x0, .LBB24_8
// %bb.7:
	bl	__udelay
.LBB24_8:
	ldr	x8, [x19]
	ldr	x0, [x8, #1296]
	cbz	x0, .LBB24_11
// %bb.9:
	ldr	w8, [x19, #48]
	cmp	w8, #32, lsl #12        // =131072
	b.ls	.LBB24_11
// %bb.10:
	bl	__udelay
.LBB24_11:
	ldrb	w8, [x19, #24]
	cbnz	w8, .LBB24_1
// %bb.12:
	ldr	x8, [x19]
	ldr	x0, [x8, #1304]
	cbz	x0, .LBB24_15
// %bb.13:
	ldr	w8, [x19, #48]
	cmp	w8, #32, lsl #12        // =131072
	b.hi	.LBB24_15
// %bb.14:
	bl	__udelay
.LBB24_15:
	ldr	x8, [x19]
	ldr	x0, [x8, #1288]
	cbz	x0, .LBB24_1
// %bb.16:
	ldr	w8, [x19, #48]
	cmp	w8, #32, lsl #12        // =131072
	b.ls	.LBB24_1
// %bb.17:
	bl	__udelay
	b	.LBB24_1
.Lfunc_end24:
	.size	mas_blk_mq_request_start, .Lfunc_end24-mas_blk_mq_request_start
                                        // -- End function
	.globl	mas_blk_request_start   // -- Begin function mas_blk_request_start
	.p2align	2
	.type	mas_blk_request_start,@function
mas_blk_request_start:                  // @mas_blk_request_start
.Lmas_blk_request_start$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	w1, #5
	mov	x29, sp
	bl	mas_blk_latency_req_check
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end25:
	.size	mas_blk_request_start, .Lfunc_end25-mas_blk_request_start
                                        // -- End function
	.globl	mas_blk_requeue_request // -- Begin function mas_blk_requeue_request
	.p2align	2
	.type	mas_blk_requeue_request,@function
mas_blk_requeue_request:                // @mas_blk_requeue_request
.Lmas_blk_requeue_request$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	w1, #22
	mov	x29, sp
	bl	mas_blk_latency_req_check
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end26:
	.size	mas_blk_requeue_request, .Lfunc_end26-mas_blk_requeue_request
                                        // -- End function
	.globl	mas_blk_request_update  // -- Begin function mas_blk_request_update
	.p2align	2
	.type	mas_blk_request_update,@function
mas_blk_request_update:                 // @mas_blk_request_update
.Lmas_blk_request_update$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	w19, w2
	mov	x20, x0
	ldr	x9, [x8, #48]
	mov	w21, w1
	mov	x29, sp
	cbz	x9, .LBB27_9
// %bb.1:
	ldr	x9, [x8, #3424]
	add	x8, x8, #1392           // =1392
	add	x10, x9, #64            // =64
	cmp	x9, #0                  // =0
	csel	x8, x8, x10, eq
.LBB27_2:
	ldrb	w9, [x20, #24]
	cmp	w9, #3                  // =3
	b.eq	.LBB27_7
// %bb.3:
	cmp	w9, #1                  // =1
	b.eq	.LBB27_8
.LBB27_4:
	cmp	w9, #2                  // =2
	b.ne	.LBB27_6
.LBB27_5:
	and	w1, w21, #0xff
	mov	x0, x20
	bl	mas_blk_flush_update
.LBB27_6:
	mov	w1, #43
	mov	x0, x20
	bl	mas_blk_latency_req_check
	mov	x0, x20
	mov	w1, w21
	mov	w2, w19
	bl	mas_blk_req_update_unistore
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB27_7:
	ldr	x9, [x8, #1384]
	add	x9, x9, w19, uxtw
	str	x9, [x8, #1384]
	ldrb	w9, [x20, #24]
	cmp	w9, #1                  // =1
	b.ne	.LBB27_4
.LBB27_8:
	ldr	x9, [x8, #1376]
	add	x9, x9, w19, uxtw
	str	x9, [x8, #1376]
	ldrb	w9, [x20, #24]
	cmp	w9, #2                  // =2
	b.eq	.LBB27_5
	b	.LBB27_6
.LBB27_9:
	add	x8, x8, #1392           // =1392
	b	.LBB27_2
.Lfunc_end27:
	.size	mas_blk_request_update, .Lfunc_end27-mas_blk_request_update
                                        // -- End function
	.globl	mas_blk_mq_request_free // -- Begin function mas_blk_mq_request_free
	.p2align	2
	.type	mas_blk_mq_request_free,@function
mas_blk_mq_request_free:                // @mas_blk_mq_request_free
.Lmas_blk_mq_request_free$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	w1, #44
	mov	x29, sp
	str	xzr, [x0, #680]
	bl	mas_blk_latency_req_check
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end28:
	.size	mas_blk_mq_request_free, .Lfunc_end28-mas_blk_mq_request_free
                                        // -- End function
	.globl	mas_blk_request_put     // -- Begin function mas_blk_request_put
	.p2align	2
	.type	mas_blk_request_put,@function
mas_blk_request_put:                    // @mas_blk_request_put
.Lmas_blk_request_put$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	w1, #44
	mov	x29, sp
	str	xzr, [x0, #680]
	bl	mas_blk_latency_req_check
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end29:
	.size	mas_blk_request_put, .Lfunc_end29-mas_blk_request_put
                                        // -- End function
	.globl	mas_blk_check_partition_done // -- Begin function mas_blk_check_partition_done
	.p2align	2
	.type	mas_blk_check_partition_done,@function
mas_blk_check_partition_done:           // @mas_blk_check_partition_done
.Lmas_blk_check_partition_done$local:
// %bb.0:
	ldr	x8, [x0, #1264]
	and	w9, w1, #0x1
	strb	w9, [x8, #1072]
	ret
.Lfunc_end30:
	.size	mas_blk_check_partition_done, .Lfunc_end30-mas_blk_check_partition_done
                                        // -- End function
	.globl	mas_blk_allocated_queue_init // -- Begin function mas_blk_allocated_queue_init
	.p2align	2
	.type	mas_blk_allocated_queue_init,@function
mas_blk_allocated_queue_init:           // @mas_blk_allocated_queue_init
.Lmas_blk_allocated_queue_init$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x19, x0
	add	x8, x0, #1272           // =1272
	add	x0, x0, #1392           // =1392
	mov	w9, #122
	add	x10, x19, #2788         // =2788
	add	x11, x19, #2796         // =2796
	mov	w12, #1
	mov	x29, sp
	str	x8, [x19, #1272]
	str	x8, [x19, #1280]
	strb	wzr, [x19, #1072]
	str	w9, [x19, #1392]
	str	xzr, [x10]
	str	x12, [x11]
	bl	mas_blk_dev_lld_init_unistore
	add	x20, x19, #2256         // =2256
	add	x8, x19, #2808          // =2808
	mov	x0, x20
	str	x8, [x19, #2808]
	str	x8, [x19, #2816]
	str	wzr, [x19, #1396]
	str	x19, [x19, #1400]
	bl	mas_blk_busyidle_state_init
	ldr	x8, [x19, #1416]
	cbz	w0, .LBB31_2
// %bb.1:
	and	x8, x8, #0xfffffffffffffff7
	str	x8, [x19, #1416]
.LBB31_2:
	tbz	w8, #6, .LBB31_4
// %bb.3:
	mov	w9, #1
	strb	w9, [x20]
.LBB31_4:
	and	x8, x8, #0xfffffffffffffe7f
	orr	x8, x8, #0x80
	strb	wzr, [x19, #2784]
	str	x8, [x19, #1416]
	str	xzr, [x19, #2840]
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end31:
	.size	mas_blk_allocated_queue_init, .Lfunc_end31-mas_blk_allocated_queue_init
                                        // -- End function
	.globl	mas_blk_sq_init_allocated_queue // -- Begin function mas_blk_sq_init_allocated_queue
	.p2align	2
	.type	mas_blk_sq_init_allocated_queue,@function
mas_blk_sq_init_allocated_queue:        // @mas_blk_sq_init_allocated_queue
.Lmas_blk_sq_init_allocated_queue$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x19, x0
	bl	mas_blk_dump_register_queue
	mov	x0, x19
	bl	mas_blk_queue_latency_init
	mov	x0, x19
	bl	mas_blk_queue_async_flush_init
	adrp	x1, _cfi_mas_blk_queue_usr_ctrl_recovery_timer_expire
	add	x0, x19, #1320          // =1320
	add	x1, x1, :lo12:_cfi_mas_blk_queue_usr_ctrl_recovery_timer_expire
	mov	w2, wzr
	mov	x3, xzr
	mov	x4, xzr
	str	xzr, [x19, #1288]
	str	xzr, [x19, #1296]
	str	xzr, [x19, #1304]
	str	xzr, [x19, #1312]
	bl	init_timer_key
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end32:
	.size	mas_blk_sq_init_allocated_queue, .Lfunc_end32-mas_blk_sq_init_allocated_queue
                                        // -- End function
	.globl	mas_blk_mq_init_allocated_queue // -- Begin function mas_blk_mq_init_allocated_queue
	.p2align	2
	.type	mas_blk_mq_init_allocated_queue,@function
mas_blk_mq_init_allocated_queue:        // @mas_blk_mq_init_allocated_queue
.Lmas_blk_mq_init_allocated_queue$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x20, [x0, #3424]
	mov	x19, x0
	mov	x29, sp
	ldr	w8, [x20, #1532]
	cbz	w8, .LBB33_5
// %bb.1:
	mov	x9, xzr
	b	.LBB33_3
.LBB33_2:                               //   in Loop: Header=BB33_3 Depth=1
	add	x9, x9, #1              // =1
	cmp	x9, w8, uxtw
	b.hs	.LBB33_5
.LBB33_3:                               // =>This Inner Loop Header: Depth=1
	ldr	x10, [x20, #1568]
	ldr	x10, [x10, x9, lsl #3]
	cbz	x10, .LBB33_2
// %bb.4:                               //   in Loop: Header=BB33_3 Depth=1
	str	x20, [x10, #24]
	ldr	w8, [x20, #1532]
	b	.LBB33_2
.LBB33_5:
	mov	x0, x19
	bl	mas_blk_dump_register_queue
	mov	x0, x19
	bl	mas_blk_queue_latency_init
	mov	x0, x19
	bl	mas_blk_queue_async_flush_init
	adrp	x1, _cfi_mas_blk_queue_usr_ctrl_recovery_timer_expire
	add	x0, x19, #1320          // =1320
	add	x1, x1, :lo12:_cfi_mas_blk_queue_usr_ctrl_recovery_timer_expire
	mov	w2, wzr
	mov	x3, xzr
	mov	x4, xzr
	str	xzr, [x19, #1288]
	str	xzr, [x19, #1296]
	str	xzr, [x19, #1304]
	str	xzr, [x19, #1312]
	bl	init_timer_key
	ldr	x8, [x20, #1520]
	cbz	x8, .LBB33_10
// %bb.6:
	ldr	x8, [x8, #40]
	str	x8, [x19, #1384]
	cbz	x8, .LBB33_11
// %bb.7:
	ldr	x8, [x8, #8]
	cbz	x8, .LBB33_11
// %bb.8:
	mov	x0, x19
	blr	x8
	cbz	w0, .LBB33_11
// %bb.9:
	mov	w2, w0
	adrp	x0, .L.str.1
	adrp	x1, .L__func__.mas_blk_mq_init_allocated_queue
	add	x0, x0, :lo12:.L.str.1
	add	x1, x1, :lo12:.L__func__.mas_blk_mq_init_allocated_queue
	bl	printk
	b	.LBB33_11
.LBB33_10:
	str	xzr, [x19, #1384]
.LBB33_11:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end33:
	.size	mas_blk_mq_init_allocated_queue, .Lfunc_end33-mas_blk_mq_init_allocated_queue
                                        // -- End function
	.globl	mas_blk_mq_free_queue   // -- Begin function mas_blk_mq_free_queue
	.p2align	2
	.type	mas_blk_mq_free_queue,@function
mas_blk_mq_free_queue:                  // @mas_blk_mq_free_queue
.Lmas_blk_mq_free_queue$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x8, [x0, #1384]
	mov	x29, sp
	cbz	x8, .LBB34_3
// %bb.1:
	ldr	x8, [x8, #16]
	cbz	x8, .LBB34_3
// %bb.2:
	blr	x8
.LBB34_3:
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end34:
	.size	mas_blk_mq_free_queue, .Lfunc_end34-mas_blk_mq_free_queue
                                        // -- End function
	.globl	mas_blk_cleanup_queue   // -- Begin function mas_blk_cleanup_queue
	.p2align	2
	.type	mas_blk_cleanup_queue,@function
mas_blk_cleanup_queue:                  // @mas_blk_cleanup_queue
.Lmas_blk_cleanup_queue$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x19, x0
	bl	mas_blk_dump_unregister_queue
	add	x0, x19, #1320          // =1320
	bl	del_timer_sync
	mov	x0, x19
	bl	mas_blk_flush_reduced_queue_unregister
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end35:
	.size	mas_blk_cleanup_queue, .Lfunc_end35-mas_blk_cleanup_queue
                                        // -- End function
	.globl	mas_blk_allocated_tags_init // -- Begin function mas_blk_allocated_tags_init
	.p2align	2
	.type	mas_blk_allocated_tags_init,@function
mas_blk_allocated_tags_init:            // @mas_blk_allocated_tags_init
.Lmas_blk_allocated_tags_init$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	adrp	x1, .L.str.2
	adrp	x2, mas_blk_allocated_tags_init.__key
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x19, x0
	add	x0, x0, #40             // =40
	add	x1, x1, :lo12:.L.str.2
	add	x2, x2, :lo12:mas_blk_allocated_tags_init.__key
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	bl	__mutex_init
	add	x20, x19, #88           // =88
	add	x8, x19, #72            // =72
	mov	w2, #1456
	mov	x0, x20
	mov	w1, wzr
	str	x8, [x19, #72]
	str	x8, [x19, #80]
	bl	memset
	mov	w8, #122
	add	x9, x19, #1484          // =1484
	add	x10, x19, #1492         // =1492
	mov	w21, #1
	mov	x0, x20
	str	w8, [x19, #88]
	str	xzr, [x9]
	str	x21, [x10]
	bl	mas_blk_dev_lld_init_unistore
	add	x20, x19, #952          // =952
	add	x8, x19, #1504          // =1504
	mov	x0, x20
	str	x8, [x19, #1504]
	str	x8, [x19, #1512]
	str	w21, [x19, #92]
	str	x19, [x19, #96]
	bl	mas_blk_busyidle_state_init
	ldr	x8, [x19, #112]
	cbz	w0, .LBB36_2
// %bb.1:
	and	x8, x8, #0xfffffffffffffff7
	str	x8, [x19, #112]
.LBB36_2:
	tbz	w8, #6, .LBB36_4
// %bb.3:
	mov	w9, #1
	strb	w9, [x20]
.LBB36_4:
	and	x8, x8, #0xfffffffffffffe7f
	orr	x8, x8, #0x80
	strb	wzr, [x19, #1480]
	str	x8, [x19, #112]
	str	xzr, [x19, #1536]
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end36:
	.size	mas_blk_allocated_tags_init, .Lfunc_end36-mas_blk_allocated_tags_init
                                        // -- End function
	.globl	blk_add_queue_tags      // -- Begin function blk_add_queue_tags
	.p2align	2
	.type	blk_add_queue_tags,@function
blk_add_queue_tags:                     // @blk_add_queue_tags
.Lblk_add_queue_tags$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	add	x19, x0, #40            // =40
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	mov	x21, x0
	mov	x0, x19
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	mov	x29, sp
	mov	x20, x1
	bl	mutex_lock
	ldr	x22, [x21, #80]
	add	x23, x20, #3432         // =3432
	add	x24, x21, #72           // =72
	mov	x0, x23
	mov	x1, x22
	mov	x2, x24
	bl	__list_add_valid
	tbz	w0, #0, .LBB37_2
// %bb.1:
	str	x23, [x21, #80]
	str	x24, [x20, #3432]
	str	x22, [x20, #3440]
	str	x23, [x22]
.LBB37_2:
	mov	x0, x19
	bl	mutex_unlock
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end37:
	.size	blk_add_queue_tags, .Lfunc_end37-blk_add_queue_tags
                                        // -- End function
	.globl	mas_blk_mq_allocated_tagset_init // -- Begin function mas_blk_mq_allocated_tagset_init
	.p2align	2
	.type	mas_blk_mq_allocated_tagset_init,@function
mas_blk_mq_allocated_tagset_init:       // @mas_blk_mq_allocated_tagset_init
.Lmas_blk_mq_allocated_tagset_init$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	mov	w8, #122
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x19, x0
	add	x9, x0, #1460           // =1460
	add	x10, x0, #1468          // =1468
	mov	w11, #1
	str	w8, [x0, #64]!
	mov	x29, sp
	str	xzr, [x9]
	str	x11, [x10]
	bl	mas_blk_dev_lld_init_unistore
	add	x20, x19, #928          // =928
	add	x8, x19, #1480          // =1480
	mov	w9, #2
	mov	x0, x20
	str	x8, [x19, #1480]
	str	x8, [x19, #1488]
	str	w9, [x19, #68]
	str	x19, [x19, #72]
	bl	mas_blk_busyidle_state_init
	ldr	x8, [x19, #88]
	cbz	w0, .LBB38_2
// %bb.1:
	and	x8, x8, #0xfffffffffffffff7
	str	x8, [x19, #88]
.LBB38_2:
	tbz	w8, #6, .LBB38_4
// %bb.3:
	mov	w9, #1
	strb	w9, [x20]
.LBB38_4:
	adrp	x10, mas_ufs_blk_tagset_ops
	and	x9, x8, #0xfffffffffffffe7f
	lsl	x8, x8, #58
	add	x10, x10, :lo12:mas_ufs_blk_tagset_ops
	orr	x9, x9, #0x80
	and	x8, x10, x8, asr #63
	strb	wzr, [x19, #1456]
	str	xzr, [x19, #1512]
	str	x9, [x19, #88]
	str	x8, [x19, #1520]
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end38:
	.size	mas_blk_mq_allocated_tagset_init, .Lfunc_end38-mas_blk_mq_allocated_tagset_init
                                        // -- End function
	.globl	mas_blk_queue_register  // -- Begin function mas_blk_queue_register
	.p2align	2
	.type	mas_blk_queue_register,@function
mas_blk_queue_register:                 // @mas_blk_queue_register
.Lmas_blk_queue_register$local:
// %bb.0:
	str	x1, [x0, #1064]
	ret
.Lfunc_end39:
	.size	mas_blk_queue_register, .Lfunc_end39-mas_blk_queue_register
                                        // -- End function
	.section	.init.text,"ax",@progbits
	.globl	mas_blk_dev_init        // -- Begin function mas_blk_dev_init
	.p2align	2
	.type	mas_blk_dev_init,@function
mas_blk_dev_init:                       // @mas_blk_dev_init
.Lmas_blk_dev_init$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x29, sp
	bl	mas_blk_dump_init
	bl	mas_blk_latency_init
	mov	w0, wzr
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end40:
	.size	mas_blk_dev_init, .Lfunc_end40-mas_blk_dev_init
                                        // -- End function
	.text
	.globl	blk_write_throttle      // -- Begin function blk_write_throttle
	.p2align	2
	.type	blk_write_throttle,@function
blk_write_throttle:                     // @blk_write_throttle
.Lblk_write_throttle$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB41_4
// %bb.1:
	ldr	x8, [x0, #1384]
	cbz	x8, .LBB41_4
// %bb.2:
	ldr	x8, [x8, #152]
	cbz	x8, .LBB41_4
// %bb.3:
	cmp	w1, #0                  // =0
	cset	w1, ne
	blr	x8
.LBB41_4:
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end41:
	.size	blk_write_throttle, .Lfunc_end41-blk_write_throttle
                                        // -- End function
	.globl	blk_generic_freeze      // -- Begin function blk_generic_freeze
	.p2align	2
	.type	blk_generic_freeze,@function
blk_generic_freeze:                     // @blk_generic_freeze
.Lblk_generic_freeze$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB42_11
// %bb.1:
	mov	w19, w2
	cmp	w1, #1                  // =1
	b.eq	.LBB42_8
// %bb.2:
	cbnz	w1, .LBB42_11
// %bb.3:
	ldr	w8, [x0, #4]
	cmp	w8, #2                  // =2
	b.eq	.LBB42_12
// %bb.4:
	cbnz	w8, .LBB42_11
// %bb.5:
	ldr	x0, [x0, #8]
	ldr	x8, [x0, #48]
	cbz	x8, .LBB42_11
// %bb.6:
	tbnz	w19, #0, .LBB42_10
.LBB42_7:
	bl	blk_mq_unfreeze_queue
	b	.LBB42_11
.LBB42_8:
	ldr	x8, [x0, #48]
	cbz	x8, .LBB42_11
// %bb.9:
	tbz	w19, #0, .LBB42_7
.LBB42_10:
	bl	blk_mq_freeze_queue
.LBB42_11:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB42_12:
	ldr	x8, [x0, #8]
	ldr	x20, [x8, #1608]
	add	x21, x8, #1608          // =1608
	cmp	x21, x20
	b.ne	.LBB42_15
	b	.LBB42_11
.LBB42_13:                              //   in Loop: Header=BB42_15 Depth=1
	bl	blk_mq_unfreeze_queue
.LBB42_14:                              //   in Loop: Header=BB42_15 Depth=1
	ldr	x20, [x20]
	cmp	x21, x20
	b.eq	.LBB42_11
.LBB42_15:                              // =>This Inner Loop Header: Depth=1
	subs	x0, x20, #3432          // =3432
	b.eq	.LBB42_14
// %bb.16:                              //   in Loop: Header=BB42_15 Depth=1
	sub	x8, x20, #3384          // =3384
	ldr	x8, [x8]
	cbz	x8, .LBB42_14
// %bb.17:                              //   in Loop: Header=BB42_15 Depth=1
	tbz	w19, #0, .LBB42_13
// %bb.18:                              //   in Loop: Header=BB42_15 Depth=1
	bl	blk_mq_freeze_queue
	ldr	x20, [x20]
	cmp	x21, x20
	b.ne	.LBB42_15
	b	.LBB42_11
.Lfunc_end42:
	.size	blk_generic_freeze, .Lfunc_end42-blk_generic_freeze
                                        // -- End function
	.globl	blk_dev_write_order_preserved // -- Begin function blk_dev_write_order_preserved
	.p2align	2
	.type	blk_dev_write_order_preserved,@function
blk_dev_write_order_preserved:          // @blk_dev_write_order_preserved
.Lblk_dev_write_order_preserved$local:
// %bb.0:
	cbz	x0, .LBB43_6
// %bb.1:
	ldr	x8, [x0, #128]
	cbz	x8, .LBB43_7
// %bb.2:
	ldr	x8, [x8, #1264]
	cbz	x8, .LBB43_7
// %bb.3:
	ldr	x9, [x8, #48]
	cbz	x9, .LBB43_8
// %bb.4:
	ldr	x9, [x8, #3424]
	add	x8, x8, #1392           // =1392
	add	x10, x9, #64            // =64
	cmp	x9, #0                  // =0
	csel	x8, x8, x10, eq
.LBB43_5:
	ldrb	w8, [x8, #25]
	and	w0, w8, #0x1
.LBB43_6:
	ret
.LBB43_7:
	mov	w0, wzr
	ret
.LBB43_8:
	add	x8, x8, #1392           // =1392
	b	.LBB43_5
.Lfunc_end43:
	.size	blk_dev_write_order_preserved, .Lfunc_end43-blk_dev_write_order_preserved
                                        // -- End function
	.globl	blk_req_set_make_req_nr // -- Begin function blk_req_set_make_req_nr
	.p2align	2
	.type	blk_req_set_make_req_nr,@function
blk_req_set_make_req_nr:                // @blk_req_set_make_req_nr
.Lblk_req_set_make_req_nr$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0]
	mov	x19, x0
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	ldr	x9, [x8, #48]
	cbz	x9, .LBB44_7
// %bb.1:
	ldr	x9, [x8, #3424]
	add	x8, x8, #1392           // =1392
	add	x10, x9, #64            // =64
	cmp	x9, #0                  // =0
	csel	x21, x8, x10, eq
	ldr	x8, [x21, #24]
	tbz	w8, #9, .LBB44_3
.LBB44_2:
	ldr	w8, [x19, #24]
	ldr	x9, [x19, #200]
	mov	w10, #59391
	movk	w10, #65471, lsl #16
	and	w8, w8, w10
	and	x9, x9, #0xfffffffffffffffd
	orr	w8, w8, #0x800
	str	x9, [x19, #200]
	str	w8, [x19, #24]
	ldr	x8, [x21, #24]
.LBB44_3:
	tst	x8, #0x300
	b.eq	.LBB44_6
// %bb.4:
	ldr	x8, [x19, #240]
	add	x20, x21, #1408         // =1408
	mov	x0, x20
	orr	x8, x8, #0x4
	str	x8, [x19, #240]
	bl	_raw_spin_lock_irqsave
	ldr	w8, [x21, #1404]
	mov	x1, x0
	cbz	w8, .LBB44_8
.LBB44_5:
	add	w9, w8, #1              // =1
	mov	x0, x20
	str	w9, [x21, #1404]
	str	w8, [x19, #680]
	bl	_raw_spin_unlock_irqrestore
.LBB44_6:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB44_7:
	add	x21, x8, #1392          // =1392
	ldr	x8, [x21, #24]
	tbnz	w8, #9, .LBB44_2
	b	.LBB44_3
.LBB44_8:
	mov	w8, #1
	str	w8, [x21, #1404]
	b	.LBB44_5
.Lfunc_end44:
	.size	blk_req_set_make_req_nr, .Lfunc_end44-blk_req_set_make_req_nr
                                        // -- End function
	.globl	blk_req_get_order_nr    // -- Begin function blk_req_get_order_nr
	.p2align	2
	.type	blk_req_get_order_nr,@function
blk_req_get_order_nr:                   // @blk_req_get_order_nr
.Lblk_req_get_order_nr$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldrb	w8, [x0, #240]
	mov	x19, x0
	mov	x29, sp
	tbnz	w8, #1, .LBB45_4
// %bb.1:
	ldr	x8, [x19]
	mov	w20, w1
	ldr	x9, [x8, #48]
	cbz	x9, .LBB45_12
// %bb.2:
	ldr	x9, [x8, #3424]
	add	x8, x8, #1392           // =1392
	add	x10, x9, #64            // =64
	cmp	x9, #0                  // =0
	csel	x22, x8, x10, eq
	ldr	w1, [x19, #684]
	cbnz	w1, .LBB45_13
.LBB45_3:
	ldrb	w8, [x22, #25]
	tbnz	w8, #0, .LBB45_7
.LBB45_4:
	mov	w21, wzr
.LBB45_5:
	str	w21, [x19, #684]
.LBB45_6:
	mov	w0, w21
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB45_7:
	tbz	w20, #0, .LBB45_9
// %bb.8:
	mov	x1, xzr
	b	.LBB45_10
.LBB45_9:
	add	x0, x22, #1400          // =1400
	bl	_raw_spin_lock_irqsave
	mov	x1, x0
.LBB45_10:
	ldr	w8, [x22, #1396]
	adds	w9, w8, #1              // =1
	add	w8, w8, #2              // =2
	csel	w21, w8, w9, hs
	str	w21, [x22, #1396]
	tbnz	w20, #0, .LBB45_5
// %bb.11:
	add	x0, x22, #1400          // =1400
	bl	_raw_spin_unlock_irqrestore
	b	.LBB45_5
.LBB45_12:
	add	x22, x8, #1392          // =1392
	ldr	w1, [x19, #684]
	cbz	w1, .LBB45_3
.LBB45_13:
	adrp	x0, .L.str.4
	add	x0, x0, :lo12:.L.str.4
	bl	printk
	ldr	w21, [x19, #684]
	b	.LBB45_6
.Lfunc_end45:
	.size	blk_req_get_order_nr, .Lfunc_end45-blk_req_get_order_nr
                                        // -- End function
	.globl	blk_queue_order_enable  // -- Begin function blk_queue_order_enable
	.p2align	2
	.type	blk_queue_order_enable,@function
blk_queue_order_enable:                 // @blk_queue_order_enable
.Lblk_queue_order_enable$local:
// %bb.0:
	ldr	x8, [x0, #48]
	cbz	x8, .LBB46_3
// %bb.1:
	ldr	x8, [x0, #3424]
	add	x9, x0, #1392           // =1392
	add	x10, x8, #64            // =64
	cmp	x8, #0                  // =0
	csel	x8, x9, x10, eq
.LBB46_2:
	ldr	x9, [x8, #24]
	tst	w1, #0x1
	and	w10, w1, #0x1
	strb	w10, [x8, #1392]
	and	x11, x9, #0xfffffffffffffeff
	orr	x9, x9, #0x100
	csel	x9, x9, x11, ne
	str	x9, [x8, #24]
	ret
.LBB46_3:
	add	x8, x0, #1392           // =1392
	b	.LBB46_2
.Lfunc_end46:
	.size	blk_queue_order_enable, .Lfunc_end46-blk_queue_order_enable
                                        // -- End function
	.globl	blk_queue_query_order_enable // -- Begin function blk_queue_query_order_enable
	.p2align	2
	.type	blk_queue_query_order_enable,@function
blk_queue_query_order_enable:           // @blk_queue_query_order_enable
.Lblk_queue_query_order_enable$local:
// %bb.0:
	ldr	x8, [x0, #48]
	cbz	x8, .LBB47_3
// %bb.1:
	ldr	x8, [x0, #3424]
	add	x9, x0, #1392           // =1392
	add	x10, x8, #64            // =64
	cmp	x8, #0                  // =0
	csel	x8, x9, x10, eq
.LBB47_2:
	ldrb	w8, [x8, #25]
	and	w0, w8, #0x1
	ret
.LBB47_3:
	add	x8, x0, #1392           // =1392
	b	.LBB47_2
.Lfunc_end47:
	.size	blk_queue_query_order_enable, .Lfunc_end47-blk_queue_query_order_enable
                                        // -- End function
	.globl	blk_order_nr_reset      // -- Begin function blk_order_nr_reset
	.p2align	2
	.type	blk_order_nr_reset,@function
blk_order_nr_reset:                     // @blk_order_nr_reset
.Lblk_order_nr_reset$local:
// %bb.0:
	cbz	x0, .LBB48_2
// %bb.1:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	add	x8, x0, #64             // =64
	str	wzr, [x0, #1460]
	mov	x0, x8
	mov	x29, sp
	bl	mas_blk_order_info_reset
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
.LBB48_2:
	ret
.Lfunc_end48:
	.size	blk_order_nr_reset, .Lfunc_end48-blk_order_nr_reset
                                        // -- End function
	.globl	blk_queue_cp_enable     // -- Begin function blk_queue_cp_enable
	.p2align	2
	.type	blk_queue_cp_enable,@function
blk_queue_cp_enable:                    // @blk_queue_cp_enable
.Lblk_queue_cp_enable$local:
// %bb.0:
	ldr	x8, [x0, #48]
	cbz	x8, .LBB49_3
// %bb.1:
	ldr	x8, [x0, #3424]
	add	x9, x0, #1392           // =1392
	add	x10, x8, #64            // =64
	cmp	x8, #0                  // =0
	csel	x8, x9, x10, eq
.LBB49_2:
	ldr	x9, [x8, #24]
	tst	w1, #0x1
	and	x10, x9, #0xffffffffffffff7f
	orr	x9, x9, #0x80
	csel	x9, x9, x10, ne
	str	x9, [x8, #24]
	ret
.LBB49_3:
	add	x8, x0, #1392           // =1392
	b	.LBB49_2
.Lfunc_end49:
	.size	blk_queue_cp_enable, .Lfunc_end49-blk_queue_cp_enable
                                        // -- End function
	.globl	mas_queue_status_show   // -- Begin function mas_queue_status_show
	.p2align	2
	.type	mas_queue_status_show,@function
mas_queue_status_show:                  // @mas_queue_status_show
.Lmas_queue_status_show$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #48]
	mov	x20, x2
	mov	x19, x0
	mov	x21, x1
	mov	x29, sp
	cbz	x8, .LBB50_3
// %bb.1:
	ldr	x8, [x19, #3424]
	add	x9, x19, #1392          // =1392
	add	x10, x8, #64            // =64
	cmp	x8, #0                  // =0
	csel	x0, x9, x10, eq
.LBB50_2:
	mov	x1, x21
	mov	w2, w20
	bl	mas_blk_dump_lld_status
	mov	w22, w0
	add	x1, x21, w0, sxtw
	sub	w2, w20, w0
	mov	x0, x19
	bl	mas_blk_dump_queue_status2
	add	w8, w0, w22
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	sxtw	x0, w8
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB50_3:
	add	x0, x19, #1392          // =1392
	b	.LBB50_2
.Lfunc_end50:
	.size	mas_queue_status_show, .Lfunc_end50-mas_queue_status_show
                                        // -- End function
	.type	.L.str.1,@object        // @.str.1
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.1:
	.asciz	"\0013[BLK-IO]%s: mq_iosched_init_fn failed. err = %d\n"
	.size	.L.str.1, 51

	.type	.L__func__.mas_blk_mq_init_allocated_queue,@object // @__func__.mas_blk_mq_init_allocated_queue
.L__func__.mas_blk_mq_init_allocated_queue:
	.asciz	"mas_blk_mq_init_allocated_queue"
	.size	.L__func__.mas_blk_mq_init_allocated_queue, 32

	.type	mas_blk_allocated_tags_init.__key,@object // @mas_blk_allocated_tags_init.__key
	.local	mas_blk_allocated_tags_init.__key
	.comm	mas_blk_allocated_tags_init.__key,1,1
	.type	.L.str.2,@object        // @.str.2
.L.str.2:
	.asciz	"&tags->tag_list_lock"
	.size	.L.str.2, 21

	.type	.L.str.4,@object        // @.str.4
.L.str.4:
	.asciz	"\0013[BLK-IO]old protocol_nr exist! %u\n"
	.size	.L.str.4, 37

	.type	.L.str.5,@object        // @.str.5
.L.str.5:
	.asciz	"\0013[BLK-IO]%s: Invalid ci_key_len!\n"
	.size	.L.str.5, 35

	.type	.L__func__.mas_blk_lld_inline_crypto_bio_split_pre,@object // @__func__.mas_blk_lld_inline_crypto_bio_split_pre
.L__func__.mas_blk_lld_inline_crypto_bio_split_pre:
	.asciz	"mas_blk_lld_inline_crypto_bio_split_pre"
	.size	.L__func__.mas_blk_lld_inline_crypto_bio_split_pre, 40

	.type	.L.str.6,@object        // @.str.6
.L.str.6:
	.asciz	"\0013[BLK-IO]%s: init key len invalid!\n"
	.size	.L.str.6, 37

	.type	.L__func__.mas_blk_lld_inline_crypto_bio_split_post,@object // @__func__.mas_blk_lld_inline_crypto_bio_split_post
.L__func__.mas_blk_lld_inline_crypto_bio_split_post:
	.asciz	"mas_blk_lld_inline_crypto_bio_split_post"
	.size	.L__func__.mas_blk_lld_inline_crypto_bio_split_post, 41

	.type	.L.str.7,@object        // @.str.7
.L.str.7:
	.asciz	"\0013[BLK-IO]%s: key len invalid!\n"
	.size	.L.str.7, 32

	.type	.L__func__.mas_blk_lld_inline_crypto_bio_merge_allow,@object // @__func__.mas_blk_lld_inline_crypto_bio_merge_allow
.L__func__.mas_blk_lld_inline_crypto_bio_merge_allow:
	.asciz	"mas_blk_lld_inline_crypto_bio_merge_allow"
	.size	.L__func__.mas_blk_lld_inline_crypto_bio_merge_allow, 42

	.type	.L.str.8,@object        // @.str.8
.L.str.8:
	.asciz	"\0013[BLK-IO]%s: init rq key len not %d or %d\n"
	.size	.L.str.8, 44

	.type	.L__func__.mas_blk_lld_inline_crypto_init_request_from_bio,@object // @__func__.mas_blk_lld_inline_crypto_init_request_from_bio
.L__func__.mas_blk_lld_inline_crypto_init_request_from_bio:
	.asciz	"mas_blk_lld_inline_crypto_init_request_from_bio"
	.size	.L__func__.mas_blk_lld_inline_crypto_init_request_from_bio, 48

	.ident	"Android (6443078 based on r383902) clang version 11.0.1 (https://android.googlesource.com/toolchain/llvm-project b397f81060ce6d701042b782172ed13bee898b79)"
	.section	".note.GNU-stack","",@progbits
