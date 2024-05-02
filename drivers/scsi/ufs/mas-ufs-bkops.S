	.text
	.file	"mas-ufs-bkops.c"
	.globl	ufshcd_bkops_status_query // -- Begin function ufshcd_bkops_status_query
	.p2align	2
	.type	ufshcd_bkops_status_query,@function
ufshcd_bkops_status_query:              // @ufshcd_bkops_status_query
.Lufshcd_bkops_status_query$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	w19, #-22
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB0_3
// %bb.1:
	mov	x21, x1
	cbz	x1, .LBB0_3
// %bb.2:
	mov	x20, x0
	ldr	x0, [x0, #64]
	mov	w1, #4
	bl	__pm_runtime_resume
	ldr	x8, [x20, #4728]
	mov	x0, x20
	mov	x1, x21
	ldr	x8, [x8]
	blr	x8
	ldr	x8, [x20, #64]
	mov	w19, w0
	mov	w1, #4
	mov	x0, x8
	bl	__pm_runtime_idle
	cbnz	w19, .LBB0_4
.LBB0_3:
	mov	w0, w19
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB0_4:
	adrp	x0, .L.str
	add	x0, x0, :lo12:.L.str
	bl	printk
	b	.LBB0_3
.Lfunc_end0:
	.size	ufshcd_bkops_status_query, .Lfunc_end0-ufshcd_bkops_status_query
                                        // -- End function
	.globl	ufshcd_bkops_start_stop // -- Begin function ufshcd_bkops_start_stop
	.p2align	2
	.type	ufshcd_bkops_start_stop,@function
ufshcd_bkops_start_stop:                // @ufshcd_bkops_start_stop
.Lufshcd_bkops_start_stop$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB1_4
// %bb.1:
	cbz	w1, .LBB1_5
// %bb.2:
	mov	x19, x0
	ldr	x0, [x0, #64]
	mov	w1, #4
	bl	__pm_runtime_resume
	ldr	x8, [x19, #4728]
	mov	x0, x19
	ldr	x8, [x8, #8]
	blr	x8
	ldr	x8, [x19, #64]
	mov	w19, w0
	mov	w1, #4
	mov	x0, x8
	bl	__pm_runtime_idle
	cbz	w19, .LBB1_6
// %bb.3:
	adrp	x0, .L.str.3
	add	x0, x0, :lo12:.L.str.3
	bl	printk
	b	.LBB1_6
.LBB1_4:
	mov	w19, #-22
	b	.LBB1_6
.LBB1_5:
	mov	w19, wzr
.LBB1_6:
	mov	w0, w19
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end1:
	.size	ufshcd_bkops_start_stop, .Lfunc_end1-ufshcd_bkops_start_stop
                                        // -- End function
	.globl	mas_ufs_manual_bkops_config // -- Begin function mas_ufs_manual_bkops_config
	.p2align	2
	.type	mas_ufs_manual_bkops_config,@function
mas_ufs_manual_bkops_config:            // @mas_ufs_manual_bkops_config
.Lmas_ufs_manual_bkops_config$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	str	x23, [sp, #16]          // 8-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB2_11
// %bb.1:
	ldr	x20, [x0]
	mov	x19, x0
	ldrb	w8, [x20, #1912]
	tbz	w8, #0, .LBB2_10
// %bb.2:
	ldr	x21, [x20, #8672]
	mov	w8, #8672
	add	x22, x20, x8
	cmp	x21, x22
	b.eq	.LBB2_5
.LBB2_3:                                // =>This Inner Loop Header: Depth=1
	ldp	x2, x3, [x19, #232]
	ldrh	w1, [x20, #4304]
	sub	x0, x21, #24            // =24
	bl	mas_ufs_is_bkops_id_match
	tbnz	w0, #0, .LBB2_9
// %bb.4:                               //   in Loop: Header=BB2_3 Depth=1
	ldr	x21, [x21]
	cmp	x21, x22
	b.ne	.LBB2_3
.LBB2_5:
	ldr	x22, [x20, #8656]
	mov	w8, #8656
	add	x23, x20, x8
	cmp	x22, x23
	b.eq	.LBB2_9
// %bb.6:
	add	x21, x20, #3912         // =3912
.LBB2_7:                                // =>This Inner Loop Header: Depth=1
	ldp	x2, x3, [x19, #232]
	ldrh	w1, [x20, #4304]
	sub	x0, x22, #24            // =24
	bl	mas_ufs_is_bkops_id_match
	tbnz	w0, #0, .LBB2_13
// %bb.8:                               //   in Loop: Header=BB2_7 Depth=1
	ldr	x22, [x22]
	cmp	x22, x23
	b.ne	.LBB2_7
.LBB2_9:
	adrp	x0, .L.str.1
	add	x0, x0, :lo12:.L.str.1
	bl	printk
.LBB2_10:
	mov	w0, wzr
	b	.LBB2_12
.LBB2_11:
	mov	w0, #-22
.LBB2_12:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB2_13:
	mov	w8, #8632
	ldrb	w8, [x20, x8]
	cbnz	w8, .LBB2_9
// %bb.14:
	bl	mas_bkops_alloc
	cbz	x0, .LBB2_18
// %bb.15:
	ldrh	w8, [x20, #4304]
	cmp	w8, #2230               // =2230
	b.eq	.LBB2_19
// %bb.16:
	cmp	w8, #429                // =429
	b.ne	.LBB2_21
// %bb.17:
	adrp	x8, hynix_bkops_ops
	add	x8, x8, :lo12:hynix_bkops_ops
	mov	w9, #3
	b	.LBB2_20
.LBB2_18:
	mov	w0, #-12
	b	.LBB2_12
.LBB2_19:
	adrp	x8, hi1861_bkops_ops
	add	x8, x8, :lo12:hi1861_bkops_ops
	mov	w9, #2
.LBB2_20:
	ldr	x10, [x0, #168]
	str	w9, [x0]
	orr	x9, x10, #0x1c
	str	x9, [x0, #168]
	str	x8, [x20, #8640]
.LBB2_21:
	adrp	x8, ufs_bkops_ops
	add	x8, x8, :lo12:ufs_bkops_ops
	mov	w9, #1
	mov	w10, #3600
	str	x0, [x20, #8648]
	stp	x8, x21, [x0, #8]
	str	w9, [x0, #244]
	str	x10, [x0, #248]
	bl	kfree
	mov	w0, #-11
	b	.LBB2_12
.Lfunc_end2:
	.size	mas_ufs_manual_bkops_config, .Lfunc_end2-mas_ufs_manual_bkops_config
                                        // -- End function
	.p2align	2               // -- Begin function mas_ufs_is_bkops_id_match
	.type	mas_ufs_is_bkops_id_match,@function
mas_ufs_is_bkops_id_match:              // @mas_ufs_is_bkops_id_match
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldrh	w8, [x0]
	mov	x29, sp
	cmp	w8, w1, uxth
	b.ne	.LBB3_5
// %bb.1:
	ldr	x22, [x0, #8]
	mov	x19, x0
	mov	x20, x3
	mov	x21, x2
	mov	x0, x22
	bl	strlen
	mov	x2, x0
	mov	x0, x21
	mov	x1, x22
	bl	strncmp
	cbz	w0, .LBB3_3
// %bb.2:
	ldr	x1, [x19, #8]
	adrp	x0, .L.str.4
	add	x0, x0, :lo12:.L.str.4
	bl	strcmp
	cbnz	w0, .LBB3_5
.LBB3_3:
	ldr	x21, [x19, #16]
	mov	x0, x21
	bl	strlen
	mov	x2, x0
	mov	x0, x20
	mov	x1, x21
	bl	strncmp
	cbz	w0, .LBB3_7
// %bb.4:
	ldr	x1, [x19, #16]
	adrp	x0, .L.str.5
	add	x0, x0, :lo12:.L.str.5
	bl	strcmp
	cbz	w0, .LBB3_7
.LBB3_5:
	mov	w0, wzr
.LBB3_6:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB3_7:
	mov	w0, #1
	b	.LBB3_6
.Lfunc_end3:
	.size	mas_ufs_is_bkops_id_match, .Lfunc_end3-mas_ufs_is_bkops_id_match
                                        // -- End function
	.p2align	2               // -- Begin function ufshcd_1861_bkops_query
	.type	ufshcd_1861_bkops_query,@function
ufshcd_1861_bkops_query:                // @ufshcd_1861_bkops_query
// %bb.0:
	cbz	x1, .LBB4_2
// %bb.1:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x2, x1
	mov	w1, #242
	mov	x29, sp
	bl	ufshcd_query_hi1861_bkops
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB4_2:
	mov	w0, #-22
	ret
.Lfunc_end4:
	.size	ufshcd_1861_bkops_query, .Lfunc_end4-ufshcd_1861_bkops_query
                                        // -- End function
	.p2align	2               // -- Begin function ufshcd_1861_bkops_start
	.type	ufshcd_1861_bkops_start,@function
ufshcd_1861_bkops_start:                // @ufshcd_1861_bkops_start
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	w1, #240
	mov	x2, xzr
	mov	x29, sp
	bl	ufshcd_query_hi1861_bkops
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end5:
	.size	ufshcd_1861_bkops_start, .Lfunc_end5-ufshcd_1861_bkops_start
                                        // -- End function
	.p2align	2               // -- Begin function ufshcd_1861_bkops_stop
	.type	ufshcd_1861_bkops_stop,@function
ufshcd_1861_bkops_stop:                 // @ufshcd_1861_bkops_stop
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	w1, #241
	mov	x2, xzr
	mov	x29, sp
	bl	ufshcd_query_hi1861_bkops
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end6:
	.size	ufshcd_1861_bkops_stop, .Lfunc_end6-ufshcd_1861_bkops_stop
                                        // -- End function
	.p2align	2               // -- Begin function ufshcd_query_hi1861_bkops
	.type	ufshcd_query_hi1861_bkops,@function
ufshcd_query_hi1861_bkops:              // @ufshcd_query_hi1861_bkops
// %bb.0:
	sub	sp, sp, #80             // =80
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	add	x29, sp, #32            // =32
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stp	xzr, xzr, [sp, #8]
	ldr	x8, [x0, #56]
	mov	x19, x0
	mov	x21, x2
	mov	w22, w1
	ldr	x0, [x8, #56]
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x19, #56]
	ldr	w20, [x19, #396]
	mov	x1, x0
	ldr	x8, [x8, #56]
	mov	x0, x8
	bl	_raw_spin_unlock_irqrestore
	cmp	w20, #2                 // =2
	b.ne	.LBB7_3
// %bb.1:
	mov	x0, x19
	mov	w1, wzr
	bl	ufshcd_hold
	add	x20, x19, #2072         // =2072
	mov	x0, x20
	bl	mutex_lock
	add	x1, sp, #16             // =16
	add	x2, sp, #8              // =8
	mov	x0, x19
	mov	w3, w22
	mov	w4, wzr
	mov	w5, wzr
	mov	w6, wzr
	bl	ufshcd_init_query
	ldr	x8, [sp, #16]
	sub	w9, w22, #240           // =240
	cmp	w9, #2                  // =2
	b.hs	.LBB7_4
// %bb.2:
	mov	w9, #129
	strb	w9, [x8]
	b	.LBB7_6
.LBB7_3:
	mov	w22, #-16
	b	.LBB7_10
.LBB7_4:
	cmp	w22, #242               // =242
	b.ne	.LBB7_12
// %bb.5:
	mov	w9, #1
	strb	w9, [x8]
	cbz	x21, .LBB7_14
.LBB7_6:
	mov	w1, #1
	mov	w2, #1300
	mov	x0, x19
	bl	ufshcd_exec_dev_cmd
	cbnz	w0, .LBB7_13
// %bb.7:
	mov	w22, wzr
	cbz	x21, .LBB7_9
// %bb.8:
	ldr	x8, [sp, #8]
	ldrb	w8, [x8, #19]
	str	w8, [x21]
.LBB7_9:
	mov	x0, x20
	bl	mutex_unlock
	mov	x0, x19
	bl	ufshcd_release
.LBB7_10:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB7_15
// %bb.11:
	mov	w0, w22
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #80             // =80
	ret
.LBB7_12:
	ldr	x0, [x19, #64]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.mas_bkops_upiu_query_opcode
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.mas_bkops_upiu_query_opcode
	mov	w3, w22
	bl	_dev_err
	mov	w22, #-22
	b	.LBB7_9
.LBB7_13:
	mov	w22, w0
	ldr	x0, [x19, #64]
	adrp	x1, .L.str.6
	adrp	x2, .L__func__.ufshcd_query_hi1861_bkops
	add	x1, x1, :lo12:.L.str.6
	add	x2, x2, :lo12:.L__func__.ufshcd_query_hi1861_bkops
	mov	w3, w22
	bl	_dev_err
	b	.LBB7_9
.LBB7_14:
	ldr	x0, [x19, #64]
	adrp	x1, .L.str.7
	adrp	x2, .L__func__.mas_bkops_upiu_query_opcode
	add	x1, x1, :lo12:.L.str.7
	add	x2, x2, :lo12:.L__func__.mas_bkops_upiu_query_opcode
	bl	_dev_err
	mov	w22, #-22
	b	.LBB7_9
.LBB7_15:
	bl	__stack_chk_fail
.Lfunc_end7:
	.size	ufshcd_query_hi1861_bkops, .Lfunc_end7-ufshcd_query_hi1861_bkops
                                        // -- End function
	.p2align	2               // -- Begin function ufshcd_hynix_bkops_query
	.type	ufshcd_hynix_bkops_query,@function
ufshcd_hynix_bkops_query:               // @ufshcd_hynix_bkops_query
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x1, .LBB8_3
// %bb.1:
	ldr	x8, [x0, #56]
	mov	x20, x0
	mov	x19, x1
	ldr	x0, [x8, #56]
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x20, #56]
	ldr	w21, [x20, #396]
	mov	x1, x0
	ldr	x8, [x8, #56]
	mov	x0, x8
	bl	_raw_spin_unlock_irqrestore
	cmp	w21, #2                 // =2
	b.ne	.LBB8_4
// %bb.2:
	mov	w1, #3
	mov	w2, #19
	mov	x0, x20
	mov	w3, wzr
	mov	w4, wzr
	mov	x5, x19
	bl	ufshcd_query_attr
	b	.LBB8_5
.LBB8_3:
	mov	w0, #-22
	b	.LBB8_5
.LBB8_4:
	mov	w0, #-16
.LBB8_5:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end8:
	.size	ufshcd_hynix_bkops_query, .Lfunc_end8-ufshcd_hynix_bkops_query
                                        // -- End function
	.p2align	2               // -- Begin function ufshcd_hynix_bkops_start
	.type	ufshcd_hynix_bkops_start,@function
ufshcd_hynix_bkops_start:               // @ufshcd_hynix_bkops_start
// %bb.0:
	sub	sp, sp, #48             // =48
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	mov	w9, #1
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	str	x8, [sp, #8]
	str	w9, [sp, #4]
	ldr	x8, [x0, #56]
	mov	x19, x0
	add	x29, sp, #16            // =16
	ldr	x0, [x8, #56]
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x19, #56]
	ldr	w20, [x19, #396]
	mov	x1, x0
	ldr	x8, [x8, #56]
	mov	x0, x8
	bl	_raw_spin_unlock_irqrestore
	cmp	w20, #2                 // =2
	b.ne	.LBB9_2
// %bb.1:
	add	x5, sp, #4              // =4
	mov	w1, #4
	mov	w2, #18
	mov	x0, x19
	mov	w3, wzr
	mov	w4, wzr
	bl	ufshcd_query_attr
	b	.LBB9_3
.LBB9_2:
	mov	w0, #-16
.LBB9_3:
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB9_5
// %bb.4:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #48             // =48
	ret
.LBB9_5:
	bl	__stack_chk_fail
.Lfunc_end9:
	.size	ufshcd_hynix_bkops_start, .Lfunc_end9-ufshcd_hynix_bkops_start
                                        // -- End function
	.p2align	2               // -- Begin function ufshcd_hynix_bkops_stop
	.type	ufshcd_hynix_bkops_stop,@function
ufshcd_hynix_bkops_stop:                // @ufshcd_hynix_bkops_stop
// %bb.0:
	sub	sp, sp, #48             // =48
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x19, x0
	str	x8, [sp, #8]
	str	wzr, [sp, #4]
	ldr	x8, [x0, #56]
	add	x29, sp, #16            // =16
	ldr	x0, [x8, #56]
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x19, #56]
	ldr	w20, [x19, #396]
	mov	x1, x0
	ldr	x8, [x8, #56]
	mov	x0, x8
	bl	_raw_spin_unlock_irqrestore
	cmp	w20, #2                 // =2
	b.ne	.LBB10_2
// %bb.1:
	add	x5, sp, #4              // =4
	mov	w1, #4
	mov	w2, #18
	mov	x0, x19
	mov	w3, wzr
	mov	w4, wzr
	bl	ufshcd_query_attr
	b	.LBB10_3
.LBB10_2:
	mov	w0, #-16
.LBB10_3:
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB10_5
// %bb.4:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #48             // =48
	ret
.LBB10_5:
	bl	__stack_chk_fail
.Lfunc_end10:
	.size	ufshcd_hynix_bkops_stop, .Lfunc_end10-ufshcd_hynix_bkops_stop
                                        // -- End function
	.type	.L.str,@object          // @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"\0013UFS bkops status query failed!\n"
	.size	.L.str, 34

	.type	.L.str.1,@object        // @.str.1
.L.str.1:
	.asciz	"\0013bkops not supported or has been enabled already!\n"
	.size	.L.str.1, 52

	.type	ufs_bkops_ops,@object   // @ufs_bkops_ops
	.data
	.p2align	3
ufs_bkops_ops:
	.xword	__ufshcd_bkops_start_stop
	.xword	__ufshcd_bkops_status_query
	.size	ufs_bkops_ops, 16

	.type	.L.str.3,@object        // @.str.3
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.3:
	.asciz	"\0013UFS bkops start failed!\n"
	.size	.L.str.3, 27

	.type	.L.str.4,@object        // @.str.4
.L.str.4:
	.asciz	"MAS_UFS_MODEL_ANY"
	.size	.L.str.4, 18

	.type	.L.str.5,@object        // @.str.5
.L.str.5:
	.asciz	"MAS_UFS_REV_ANY"
	.size	.L.str.5, 16

	.type	hi1861_bkops_ops,@object // @hi1861_bkops_ops
	.data
	.p2align	3
hi1861_bkops_ops:
	.xword	ufshcd_1861_bkops_query
	.xword	ufshcd_1861_bkops_start
	.xword	ufshcd_1861_bkops_stop
	.size	hi1861_bkops_ops, 24

	.type	hynix_bkops_ops,@object // @hynix_bkops_ops
	.p2align	3
hynix_bkops_ops:
	.xword	ufshcd_hynix_bkops_query
	.xword	ufshcd_hynix_bkops_start
	.xword	ufshcd_hynix_bkops_stop
	.size	hynix_bkops_ops, 24

	.type	.L.str.6,@object        // @.str.6
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.6:
	.asciz	"%s: Sending bkops query failed, err = %d\n"
	.size	.L.str.6, 42

	.type	.L__func__.ufshcd_query_hi1861_bkops,@object // @__func__.ufshcd_query_hi1861_bkops
.L__func__.ufshcd_query_hi1861_bkops:
	.asciz	"ufshcd_query_hi1861_bkops"
	.size	.L__func__.ufshcd_query_hi1861_bkops, 26

	.type	.L.str.7,@object        // @.str.7
.L.str.7:
	.asciz	"%s: Invalid argument for read request\n"
	.size	.L.str.7, 39

	.type	.L__func__.mas_bkops_upiu_query_opcode,@object // @__func__.mas_bkops_upiu_query_opcode
.L__func__.mas_bkops_upiu_query_opcode:
	.asciz	"mas_bkops_upiu_query_opcode"
	.size	.L__func__.mas_bkops_upiu_query_opcode, 28

	.type	.L.str.8,@object        // @.str.8
.L.str.8:
	.asciz	"%s: Expected query bkops opcode but got = %d\n"
	.size	.L.str.8, 46

	.ident	"Android (6443078 based on r383902) clang version 11.0.1 (https://android.googlesource.com/toolchain/llvm-project b397f81060ce6d701042b782172ed13bee898b79)"
	.section	".note.GNU-stack","",@progbits
