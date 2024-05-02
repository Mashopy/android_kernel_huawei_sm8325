	.text
	.file	"data.c"
	.section	.init.text,"ax",@progbits
	.globl	f2fs_init_bioset        // -- Begin function f2fs_init_bioset
	.p2align	2
	.type	f2fs_init_bioset,@function
f2fs_init_bioset:                       // @f2fs_init_bioset
.Lf2fs_init_bioset$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x0, f2fs_bioset
	add	x0, x0, :lo12:f2fs_bioset
	mov	w1, #8
	mov	w3, #1
	mov	w2, wzr
	mov	x29, sp
	bl	bioset_init
	cmp	w0, #0                  // =0
	mov	w8, #-12
	csel	w0, wzr, w8, eq
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end0:
	.size	f2fs_init_bioset, .Lfunc_end0-f2fs_init_bioset
                                        // -- End function
	.text
	.globl	f2fs_destroy_bioset     // -- Begin function f2fs_destroy_bioset
	.p2align	2
	.type	f2fs_destroy_bioset,@function
f2fs_destroy_bioset:                    // @f2fs_destroy_bioset
.Lf2fs_destroy_bioset$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x0, f2fs_bioset
	add	x0, x0, :lo12:f2fs_bioset
	mov	x29, sp
	bl	bioset_exit
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end1:
	.size	f2fs_destroy_bioset, .Lfunc_end1-f2fs_destroy_bioset
                                        // -- End function
	.globl	f2fs_bio_alloc          // -- Begin function f2fs_bio_alloc
	.p2align	2
	.type	f2fs_bio_alloc,@function
f2fs_bio_alloc:                         // @f2fs_bio_alloc
.Lf2fs_bio_alloc$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	tst	w2, #0x1
	mov	w8, #3264
	mov	w9, #3072
	adrp	x2, f2fs_bioset
	csel	w0, w9, w8, ne
	add	x2, x2, :lo12:f2fs_bioset
	mov	x29, sp
	bl	bio_alloc_bioset
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end2:
	.size	f2fs_bio_alloc, .Lfunc_end2-f2fs_bio_alloc
                                        // -- End function
	.globl	f2fs_target_device      // -- Begin function f2fs_target_device
	.p2align	2
	.type	f2fs_target_device,@function
f2fs_target_device:                     // @f2fs_target_device
.Lf2fs_target_device$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x9, [x0]
	ldr	w8, [x0, #3128]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x2
	ldr	x20, [x9, #248]
	cmp	w8, #2                  // =2
	mov	w21, w1
	mov	x29, sp
	b.lt	.LBB3_6
// %bb.1:
	ldr	x9, [x0, #3136]
	add	x9, x9, #80             // =80
	b	.LBB3_3
.LBB3_2:                                //   in Loop: Header=BB3_3 Depth=1
	subs	x8, x8, #1              // =1
	add	x9, x9, #88             // =88
	b.eq	.LBB3_6
.LBB3_3:                                // =>This Inner Loop Header: Depth=1
	ldur	w10, [x9, #-4]
	cmp	w10, w21
	b.hi	.LBB3_2
// %bb.4:                               //   in Loop: Header=BB3_3 Depth=1
	ldr	w11, [x9]
	cmp	w11, w21
	b.lo	.LBB3_2
// %bb.5:
	ldur	x20, [x9, #-80]
	sub	w21, w21, w10
.LBB3_6:
	cbz	x19, .LBB3_10
// %bb.7:
	ldr	x9, [x19, #8]
	ldr	x8, [x20, #128]
	cmp	x9, x8
	b.eq	.LBB3_9
// %bb.8:
	ldrh	w8, [x19, #20]
	and	w8, w8, #0xfffffdff
	strh	w8, [x19, #20]
	ldr	x8, [x20, #128]
.LBB3_9:
	str	x8, [x19, #8]
	ldrb	w8, [x20, #108]
	mov	x0, x19
	strb	w8, [x19, #27]
	bl	bio_associate_blkg
	mov	w8, w21
	lsl	x8, x8, #3
	str	x8, [x19, #32]
.LBB3_10:
	mov	x0, x20
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end3:
	.size	f2fs_target_device, .Lfunc_end3-f2fs_target_device
                                        // -- End function
	.globl	f2fs_target_device_index // -- Begin function f2fs_target_device_index
	.p2align	2
	.type	f2fs_target_device_index,@function
f2fs_target_device_index:               // @f2fs_target_device_index
.Lf2fs_target_device_index$local:
// %bb.0:
	ldr	w9, [x0, #3128]
	cmp	w9, #2                  // =2
	b.lt	.LBB4_6
// %bb.1:
	mov	x8, x0
	ldr	x8, [x8, #3136]
	mov	x0, xzr
	add	x8, x8, #80             // =80
	b	.LBB4_3
.LBB4_2:                                //   in Loop: Header=BB4_3 Depth=1
	add	x0, x0, #1              // =1
	cmp	x9, x0
	add	x8, x8, #88             // =88
	b.eq	.LBB4_6
.LBB4_3:                                // =>This Inner Loop Header: Depth=1
	ldur	w10, [x8, #-4]
	cmp	w10, w1
	b.hi	.LBB4_2
// %bb.4:                               //   in Loop: Header=BB4_3 Depth=1
	ldr	w10, [x8]
	cmp	w10, w1
	b.lo	.LBB4_2
// %bb.5:
                                        // kill: def $w0 killed $w0 killed $x0
	ret
.LBB4_6:
	mov	w0, wzr
                                        // kill: def $w0 killed $w0 killed $x0
	ret
.Lfunc_end4:
	.size	f2fs_target_device_index, .Lfunc_end4-f2fs_target_device_index
                                        // -- End function
	.globl	f2fs_submit_bio         // -- Begin function f2fs_submit_bio
	.p2align	2
	.type	f2fs_submit_bio,@function
f2fs_submit_bio:                        // @f2fs_submit_bio
.Lf2fs_submit_bio$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x29, sp
	bl	__submit_bio
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end5:
	.size	f2fs_submit_bio, .Lfunc_end5-f2fs_submit_bio
                                        // -- End function
	.p2align	2               // -- Begin function __submit_bio
	.type	__submit_bio,@function
__submit_bio:                           // @__submit_bio
// %bb.0:
	sub	sp, sp, #112            // =112
	stp	x22, x21, [sp, #80]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #96]     // 16-byte Folded Spill
	mov	w20, w2
	mov	x19, x1
	cmp	w2, #1                  // =1
	mov	x21, x0
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #32]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #48]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #64]     // 16-byte Folded Spill
	add	x29, sp, #16            // =16
	b.hi	.LBB6_11
// %bb.1:
	ldrb	w8, [x19, #16]
	cbz	w8, .LBB6_11
// %bb.2:
	ldr	w8, [x21, #1504]
	cmp	w8, #1                  // =1
	b.ne	.LBB6_5
// %bb.3:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldr	x0, [x8, #2280]
	cbz	x0, .LBB6_5
// %bb.4:
	bl	blk_finish_plug
.LBB6_5:
	ldr	w8, [x21, #1432]
	mov	w9, #1
	lsl	w9, w9, w8
	cmp	w9, #2                  // =2
	b.lt	.LBB6_11
// %bb.6:
	ldr	w10, [x19, #40]
	sub	w9, w9, #1              // =1
	ands	w9, w9, w10, lsr #12
	b.eq	.LBB6_11
// %bb.7:
	lsr	w8, w9, w8
	cbz	w8, .LBB6_27
.LBB6_8:
	cbz	x21, .LBB6_11
// %bb.9:
	cmp	w20, #1                 // =1
	b.ne	.LBB6_11
// %bb.10:
	add	x8, x21, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 32
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB6_11:
	ldrb	w8, [x19, #16]
	ldr	x21, [x21]
	cbz	w8, .LBB6_18
// %bb.12:
	adrp	x8, __tracepoint_f2fs_submit_write_bio+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_submit_write_bio+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB6_26
// %bb.13:
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
	cbz	x19, .LBB6_26
// %bb.14:
	mov	w10, #1
	lsl	x8, x10, x8
	and	x8, x8, x9
	cbz	x8, .LBB6_26
// %bb.15:
	//APP
	mrs x22, sp_el0
	//NO_APP
	ldr	w8, [x22, #24]
	add	w8, w8, #1              // =1
	str	w8, [x22, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_submit_write_bio+32
	ldr	x23, [x8, :lo12:__tracepoint_f2fs_submit_write_bio+32]
	cbz	x23, .LBB6_17
.LBB6_16:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	mov	x1, x21
	mov	w2, w20
	mov	x3, x19
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB6_16
.LBB6_17:
	//APP
	//NO_APP
	b	.LBB6_24
.LBB6_18:
	adrp	x8, __tracepoint_f2fs_submit_read_bio+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_submit_read_bio+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB6_26
// %bb.19:
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
	cbz	x19, .LBB6_26
// %bb.20:
	mov	w10, #1
	lsl	x8, x10, x8
	and	x8, x8, x9
	cbz	x8, .LBB6_26
// %bb.21:
	//APP
	mrs x22, sp_el0
	//NO_APP
	ldr	w8, [x22, #24]
	add	w8, w8, #1              // =1
	str	w8, [x22, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_submit_read_bio+32
	ldr	x23, [x8, :lo12:__tracepoint_f2fs_submit_read_bio+32]
	cbz	x23, .LBB6_23
.LBB6_22:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	mov	x1, x21
	mov	w2, w20
	mov	x3, x19
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB6_22
.LBB6_23:
	//APP
	//NO_APP
.LBB6_24:
	ldr	x8, [x22, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x22, #24]
	b.eq	.LBB6_63
// %bb.25:
	ldr	x8, [x22, #24]
	cbz	x8, .LBB6_63
.LBB6_26:
	mov	x0, x19
	bl	submit_bio
	ldp	x20, x19, [sp, #96]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #80]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #64]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #48]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #112            // =112
	ret
.LBB6_27:
	mov	x23, #134217728
	add	x22, x21, #3408         // =3408
	add	w26, w9, #1             // =1
	adrp	x27, f2fs_dclient
	movk	x23, #64, lsl #32
	mov	x24, #-549755813888
	//APP
	mrs x28, sp_el0
	//NO_APP
	b	.LBB6_29
.LBB6_28:                               //   in Loop: Header=BB6_29 Depth=1
	ldr	w8, [x21, #1432]
	lsr	w8, w26, w8
	add	w26, w26, #1            // =1
	cbnz	w8, .LBB6_8
.LBB6_29:                               // =>This Inner Loop Header: Depth=1
	ldr	x0, [x21, #216]
	mov	w1, #35840
	bl	mempool_alloc
	mov	x25, x0
	cbz	x0, .LBB6_55
.LBB6_30:                               //   in Loop: Header=BB6_29 Depth=1
	bl	print_bdev_access_info
	ldr	x0, [x27, :lo12:f2fs_dclient]
	cbz	x0, .LBB6_33
// %bb.31:                              //   in Loop: Header=BB6_29 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB6_33
// %bb.32:                              //   in Loop: Header=BB6_29 Depth=1
	ldr	x0, [x27, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.__submit_bio
	mov	w3, #916
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.__submit_bio
	bl	dsm_client_record
	ldr	x0, [x27, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB6_33:                               //   in Loop: Header=BB6_29 Depth=1
	mov	x0, x21
	bl	f2fs_print_raw_sb_info
	mov	x0, x21
	bl	f2fs_print_ckpt_info
	mov	x0, x21
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 916; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x21, .LBB6_38
// %bb.34:                              //   in Loop: Header=BB6_29 Depth=1
	add	x8, x21, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x21, #3376]
	cbnz	w8, .LBB6_38
// %bb.35:                              //   in Loop: Header=BB6_29 Depth=1
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB6_38
// %bb.36:                              //   in Loop: Header=BB6_29 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB6_38
// %bb.37:                              //   in Loop: Header=BB6_29 Depth=1
	mov	w0, #8
	mov	x2, x22
	bl	queue_work_on
.LBB6_38:                               //   in Loop: Header=BB6_29 Depth=1
	ldr	x8, [x25, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x25, x9, eq
	ldr	x9, [x8]
	str	x9, [sp, #8]
	ldr	x9, [sp, #8]
	tbnz	w9, #0, .LBB6_40
// %bb.39:                              //   in Loop: Header=BB6_29 Depth=1
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbz	w9, #0, .LBB6_41
.LBB6_40:                               //   in Loop: Header=BB6_29 Depth=1
	mov	x0, x25
	bl	__lock_page
.LBB6_41:                               //   in Loop: Header=BB6_29 Depth=1
	ldr	w8, [x28, #24]
	mov	w2, #4096
	mov	w1, wzr
	add	w8, w8, #1              // =1
	str	w8, [x28, #24]
	//APP
	//NO_APP
	ldr	w8, [x28, #2800]
	add	w8, w8, #1              // =1
	str	w8, [x28, #2800]
	//APP
	//NO_APP
	add	x8, x23, x25, lsl #6
	and	x8, x8, #0xfffffffffffff000
	add	x0, x8, x24
	bl	memset
	//APP
	//NO_APP
	ldr	w8, [x28, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x28, #2800]
	//APP
	//NO_APP
	ldr	x8, [x28, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x28, #24]
	b.eq	.LBB6_54
// %bb.42:                              //   in Loop: Header=BB6_29 Depth=1
	ldr	x8, [x28, #24]
	cbz	x8, .LBB6_54
.LBB6_43:                               //   in Loop: Header=BB6_29 Depth=1
	mov	x0, x25
	bl	flush_dcache_page
	ldr	x8, [x25]
	tbnz	w8, #13, .LBB6_45
// %bb.44:                              //   in Loop: Header=BB6_29 Depth=1
	ldr	x8, [x25, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x25, x9, eq
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
	prfm	pstl1strm, [x25]
1:	ldxr	x8, [x25]
	orr	x8, x8, 8192
	stxr	w9, x8, [x25]
	cbnz	w9, 1b
	//NO_APP
	str	xzr, [x25, #40]
.LBB6_45:                               //   in Loop: Header=BB6_29 Depth=1
	add	x8, x25, #40            // =40
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
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w2, #4096
	mov	x0, x19
	mov	x1, x25
	mov	w3, wzr
	bl	bio_add_page
	cmp	w0, #4095               // =4095
	b.hi	.LBB6_28
// %bb.46:                              //   in Loop: Header=BB6_29 Depth=1
	bl	print_bdev_access_info
	ldr	x0, [x27, :lo12:f2fs_dclient]
	cbz	x0, .LBB6_49
// %bb.47:                              //   in Loop: Header=BB6_29 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB6_49
// %bb.48:                              //   in Loop: Header=BB6_29 Depth=1
	ldr	x0, [x27, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.__submit_bio
	mov	w3, #931
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.__submit_bio
	bl	dsm_client_record
	ldr	x0, [x27, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB6_49:                               //   in Loop: Header=BB6_29 Depth=1
	mov	x0, x21
	bl	f2fs_print_raw_sb_info
	mov	x0, x21
	bl	f2fs_print_ckpt_info
	mov	x0, x21
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 931; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x21, .LBB6_28
// %bb.50:                              //   in Loop: Header=BB6_29 Depth=1
	add	x8, x21, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x21, #3376]
	cbnz	w8, .LBB6_28
// %bb.51:                              //   in Loop: Header=BB6_29 Depth=1
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB6_28
// %bb.52:                              //   in Loop: Header=BB6_29 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB6_28
// %bb.53:                              //   in Loop: Header=BB6_29 Depth=1
	mov	w0, #8
	mov	x2, x22
	bl	queue_work_on
	b	.LBB6_28
.LBB6_54:                               //   in Loop: Header=BB6_29 Depth=1
	bl	preempt_schedule
	b	.LBB6_43
.LBB6_55:                               //   in Loop: Header=BB6_29 Depth=1
	bl	print_bdev_access_info
	ldr	x0, [x27, :lo12:f2fs_dclient]
	cbz	x0, .LBB6_58
// %bb.56:                              //   in Loop: Header=BB6_29 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB6_58
// %bb.57:                              //   in Loop: Header=BB6_29 Depth=1
	ldr	x0, [x27, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.__submit_bio
	mov	w3, #913
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.__submit_bio
	bl	dsm_client_record
	ldr	x0, [x27, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB6_58:                               //   in Loop: Header=BB6_29 Depth=1
	mov	x0, x21
	bl	f2fs_print_raw_sb_info
	mov	x0, x21
	bl	f2fs_print_ckpt_info
	mov	x0, x21
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 913; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x21, .LBB6_30
// %bb.59:                              //   in Loop: Header=BB6_29 Depth=1
	add	x8, x21, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x21, #3376]
	cbnz	w8, .LBB6_30
// %bb.60:                              //   in Loop: Header=BB6_29 Depth=1
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB6_30
// %bb.61:                              //   in Loop: Header=BB6_29 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB6_30
// %bb.62:                              //   in Loop: Header=BB6_29 Depth=1
	mov	w0, #8
	mov	x2, x22
	bl	queue_work_on
	b	.LBB6_30
.LBB6_63:
	bl	preempt_schedule_notrace
	b	.LBB6_26
.Lfunc_end6:
	.size	__submit_bio, .Lfunc_end6-__submit_bio
                                        // -- End function
	.globl	f2fs_submit_merged_write // -- Begin function f2fs_submit_merged_write
	.p2align	2
	.type	f2fs_submit_merged_write,@function
f2fs_submit_merged_write:               // @f2fs_submit_merged_write
.Lf2fs_submit_merged_write$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	w4, w1
	mov	w5, #1
	mov	x1, xzr
	mov	x2, xzr
	mov	w3, wzr
	mov	x29, sp
	bl	__submit_merged_write_cond
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end7:
	.size	f2fs_submit_merged_write, .Lfunc_end7-f2fs_submit_merged_write
                                        // -- End function
	.p2align	2               // -- Begin function __submit_merged_write_cond
	.type	__submit_merged_write_cond,@function
__submit_merged_write_cond:             // @__submit_merged_write_cond
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	cmp	w4, #2                  // =2
	mov	w8, #2
	csel	w8, w4, w8, lo
	add	x8, x0, w8, uxtw #3
	str	x27, [sp, #16]          // 8-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	mov	w22, w5
	mov	w23, w4
	mov	w19, w3
	mov	x20, x2
	mov	x21, x1
	mov	x24, x0
	add	x27, x8, #144           // =144
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	tbnz	w5, #0, .LBB8_2
// %bb.1:
	ldr	x26, [x27]
	add	x25, x26, #160          // =160
	mov	x0, x25
	bl	down_read
	ldr	x0, [x26, #8]
	mov	x1, x21
	mov	x2, x20
	mov	w3, w19
	bl	__has_merged_page
	mov	w26, w0
	mov	x0, x25
	bl	up_read
	tbz	w26, #0, .LBB8_7
.LBB8_2:
	ldr	x26, [x27]
	add	x25, x26, #160          // =160
	mov	x0, x25
	bl	down_write
	cmp	w23, #4                 // =4
	b.lo	.LBB8_4
// %bb.3:
	mov	x9, #1
	mov	w8, #4
	movk	x9, #14336, lsl #32
	str	w8, [x26, #36]
	stur	x9, [x26, #44]
	ldr	w8, [x24, #1424]
	mov	w9, #14336
	tst	w8, #0x800
	mov	w8, #14336
	movk	w8, #12, lsl #16
	csel	w8, w8, w9, eq
	str	w8, [x26, #48]
.LBB8_4:
	mov	x0, x26
	bl	__submit_merged_bio
	mov	x0, x25
	bl	up_write
	cmp	w23, #1                 // =1
	b.hi	.LBB8_12
// %bb.5:
	tbz	w22, #0, .LBB8_8
// %bb.6:
	ldr	x8, [x27]
	add	x20, x8, #456           // =456
	mov	x0, x20
	add	x19, x8, #296           // =296
	bl	down_write
	mov	x0, x19
	bl	__submit_merged_bio
	mov	x0, x20
	bl	up_write
	b	.LBB8_11
.LBB8_7:
	cmp	w23, #2                 // =2
	b.hs	.LBB8_12
.LBB8_8:
	ldr	x24, [x27]
	add	x23, x24, #456          // =456
	mov	x0, x23
	bl	down_read
	ldr	x0, [x24, #304]
	mov	x1, x21
	mov	x2, x20
	mov	w3, w19
	bl	__has_merged_page
	mov	w24, w0
	mov	x0, x23
	bl	up_read
	tbz	w24, #0, .LBB8_10
// %bb.9:
	ldr	x8, [x27]
	add	x24, x8, #456           // =456
	mov	x0, x24
	add	x23, x8, #296           // =296
	bl	down_write
	mov	x0, x23
	bl	__submit_merged_bio
	mov	x0, x24
	bl	up_write
	tbnz	w22, #0, .LBB8_11
.LBB8_10:
	ldr	x23, [x27]
	add	x22, x23, #752          // =752
	mov	x0, x22
	bl	down_read
	ldr	x0, [x23, #600]
	mov	x1, x21
	mov	x2, x20
	mov	w3, w19
	bl	__has_merged_page
	mov	w19, w0
	mov	x0, x22
	bl	up_read
	tbz	w19, #0, .LBB8_12
.LBB8_11:
	ldr	x8, [x27]
	add	x20, x8, #752           // =752
	mov	x0, x20
	add	x19, x8, #592           // =592
	bl	down_write
	mov	x0, x19
	bl	__submit_merged_bio
	mov	x0, x20
	bl	up_write
.LBB8_12:
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldr	x27, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.Lfunc_end8:
	.size	__submit_merged_write_cond, .Lfunc_end8-__submit_merged_write_cond
                                        // -- End function
	.globl	f2fs_submit_merged_write_cond // -- Begin function f2fs_submit_merged_write_cond
	.p2align	2
	.type	f2fs_submit_merged_write_cond,@function
f2fs_submit_merged_write_cond:          // @f2fs_submit_merged_write_cond
.Lf2fs_submit_merged_write_cond$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	w5, wzr
	mov	x29, sp
	bl	__submit_merged_write_cond
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end9:
	.size	f2fs_submit_merged_write_cond, .Lfunc_end9-f2fs_submit_merged_write_cond
                                        // -- End function
	.globl	f2fs_flush_merged_writes // -- Begin function f2fs_flush_merged_writes
	.p2align	2
	.type	f2fs_flush_merged_writes,@function
f2fs_flush_merged_writes:               // @f2fs_flush_merged_writes
.Lf2fs_flush_merged_writes$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x20, [x0, #144]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	mov	x29, sp
	add	x21, x20, #160          // =160
	mov	x0, x21
	bl	down_write
	mov	x0, x20
	bl	__submit_merged_bio
	mov	x0, x21
	bl	up_write
	ldr	x8, [x19, #144]
	add	x21, x8, #456           // =456
	mov	x0, x21
	add	x20, x8, #296           // =296
	bl	down_write
	mov	x0, x20
	bl	__submit_merged_bio
	mov	x0, x21
	bl	up_write
	ldr	x8, [x19, #144]
	add	x21, x8, #752           // =752
	mov	x0, x21
	add	x20, x8, #592           // =592
	bl	down_write
	mov	x0, x20
	bl	__submit_merged_bio
	mov	x0, x21
	bl	up_write
	ldr	x20, [x19, #152]
	add	x21, x20, #160          // =160
	mov	x0, x21
	bl	down_write
	mov	x0, x20
	bl	__submit_merged_bio
	mov	x0, x21
	bl	up_write
	ldr	x8, [x19, #152]
	add	x21, x8, #456           // =456
	mov	x0, x21
	add	x20, x8, #296           // =296
	bl	down_write
	mov	x0, x20
	bl	__submit_merged_bio
	mov	x0, x21
	bl	up_write
	ldr	x8, [x19, #152]
	add	x21, x8, #752           // =752
	mov	x0, x21
	add	x20, x8, #592           // =592
	bl	down_write
	mov	x0, x20
	bl	__submit_merged_bio
	mov	x0, x21
	bl	up_write
	ldr	x19, [x19, #160]
	add	x20, x19, #160          // =160
	mov	x0, x20
	bl	down_write
	mov	x0, x19
	bl	__submit_merged_bio
	mov	x0, x20
	bl	up_write
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end10:
	.size	f2fs_flush_merged_writes, .Lfunc_end10-f2fs_flush_merged_writes
                                        // -- End function
	.globl	f2fs_submit_page_bio    // -- Begin function f2fs_submit_page_bio
	.p2align	2
	.type	f2fs_submit_page_bio,@function
f2fs_submit_page_bio:                   // @f2fs_submit_page_bio
.Lf2fs_submit_page_bio$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x21, [x0, #48]
	mov	x19, x0
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	cbnz	x21, .LBB11_2
// %bb.1:
	ldr	x21, [x19, #40]
.LBB11_2:
	ldr	x0, [x19]
	ldrb	w8, [x19, #89]
	ldr	w1, [x19, #28]
	cbz	w8, .LBB11_16
// %bb.3:
	mov	w2, #5
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB11_17
.LBB11_4:
	adrp	x8, __tracepoint_f2fs_submit_page_bio+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_submit_page_bio+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB11_11
// %bb.5:
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
	tbz	w8, #0, .LBB11_11
// %bb.6:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB11_11
// %bb.7:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_submit_page_bio+32
	ldr	x22, [x8, :lo12:__tracepoint_f2fs_submit_page_bio+32]
	cbz	x22, .LBB11_9
.LBB11_8:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x22]
	mov	x1, x21
	mov	x2, x19
	blr	x8
	ldr	x8, [x22, #24]!
	cbnz	x8, .LBB11_8
.LBB11_9:
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB11_50
// %bb.10:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB11_50
.LBB11_11:
	mov	w1, #1
	mov	x0, x19
	bl	__bio_alloc
	ldr	x8, [x19, #40]
	mov	x20, x0
	ldp	x9, x2, [x8, #24]
	ldr	x1, [x9]
	cbz	x19, .LBB11_18
// %bb.12:
	ldr	x8, [x19, #48]
	cbz	x8, .LBB11_18
// %bb.13:
	ldrb	w8, [x1, #13]
	tbz	w8, #6, .LBB11_19
// %bb.14:
	ldrh	w8, [x1]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB11_19
// %bb.15:
	mov	w8, #1
	strb	w8, [x20, #456]
	b	.LBB11_19
.LBB11_16:
	ldr	w8, [x19, #12]
	mov	w9, #7
	cmp	w8, #1                  // =1
	mov	w8, #9
	csel	w2, w8, w9, hi
	bl	f2fs_is_valid_blkaddr
	tbnz	w0, #0, .LBB11_4
.LBB11_17:
	mov	w0, #-117
	b	.LBB11_54
.LBB11_18:
	mov	w3, #3072
	mov	x0, x20
	bl	fscrypt_set_bio_crypt_ctx
.LBB11_19:
	mov	w2, #4096
	mov	x0, x20
	mov	x1, x21
	mov	w3, wzr
	bl	bio_add_page
	cmp	w0, #4095               // =4095
	b.hi	.LBB11_21
// %bb.20:
	mov	x0, x20
	bl	bio_put
	mov	w0, #-14
	b	.LBB11_54
.LBB11_21:
	ldr	x0, [x19, #104]
	cbz	x0, .LBB11_24
// %bb.22:
	ldr	w8, [x19, #20]
	cbz	w8, .LBB11_24
// %bb.23:
	mov	w2, #4096
	mov	x1, x21
	bl	wbc_account_cgroup_owner
.LBB11_24:
	ldr	w8, [x19, #12]
	cbnz	w8, .LBB11_29
// %bb.25:
	ldr	x8, [x19]
	ldr	w9, [x19, #16]
	mov	w10, #1
	ldr	w8, [x8, #2724]
	lsl	w9, w10, w9
	and	w9, w9, #0x7
	tst	w9, w8, lsr #3
	b.eq	.LBB11_27
// %bb.26:
	ldr	w10, [x19, #24]
	orr	w10, w10, #0x1000
	str	w10, [x19, #24]
.LBB11_27:
	tst	w9, w8
	b.eq	.LBB11_29
// %bb.28:
	ldr	w8, [x19, #24]
	orr	w8, w8, #0x40000
	str	w8, [x19, #24]
.LBB11_29:
	ldp	w8, w9, [x19, #20]
	orr	w8, w9, w8
	str	w8, [x20, #16]
	ldr	w8, [x19, #20]
	ldr	x22, [x19]
	cbz	w8, .LBB11_35
// %bb.30:
	ldr	x21, [x19, #40]
	ldr	x8, [x21, #24]
	cbz	x8, .LBB11_49
// %bb.31:
	ldr	x23, [x8]
	ldr	x8, [x23, #40]
	ldr	x9, [x8, #1008]
	ldr	x8, [x23, #64]
	ldr	w10, [x9, #1180]
	cmp	x8, x10
	b.eq	.LBB11_34
// %bb.32:
	ldr	w9, [x9, #1176]
	cmp	x8, x9
	b.eq	.LBB11_34
// %bb.33:
	ldrh	w8, [x23]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.ne	.LBB11_42
.LBB11_34:
	mov	w8, #7
	b	.LBB11_53
.LBB11_35:
	ldr	x8, [x21, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x21, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB11_37
// %bb.36:
	ldr	x8, [x21]
	tbnz	w8, #10, .LBB11_51
.LBB11_37:
	ldr	x0, [x21, #24]
	cbz	x0, .LBB11_52
.LBB11_38:
	ldr	x8, [x0]
	ldr	x9, [x8, #40]
	ldr	x8, [x8, #64]
	ldr	x9, [x9, #1008]
	ldr	w10, [x9, #1180]
	cmp	x8, x10
	b.ne	.LBB11_40
// %bb.39:
	mov	w8, #11
	b	.LBB11_53
.LBB11_40:
	ldr	w9, [x9, #1176]
	cmp	x8, x9
	b.ne	.LBB11_52
// %bb.41:
	mov	w8, #10
	b	.LBB11_53
.LBB11_42:
	mov	x0, x21
	bl	f2fs_is_compressed_page
	tbnz	w0, #0, .LBB11_49
// %bb.43:
	ldrh	w8, [x23]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB11_46
// %bb.44:
	ldr	x8, [x23, #736]
	tbnz	w8, #15, .LBB11_34
// %bb.45:
	ldrb	w8, [x23, #12]
	tbnz	w8, #5, .LBB11_34
.LBB11_46:
	ldr	x8, [x21]
	tbz	w8, #13, .LBB11_49
// %bb.47:
	ldr	x8, [x21, #40]
	tbz	w8, #0, .LBB11_49
// %bb.48:
	ldr	x8, [x21, #40]
	tbnz	w8, #3, .LBB11_34
.LBB11_49:
	mov	w8, #8
	b	.LBB11_53
.LBB11_50:
	bl	preempt_schedule_notrace
	b	.LBB11_11
.LBB11_51:
	mov	x0, x21
	bl	__page_file_mapping
	cbnz	x0, .LBB11_38
.LBB11_52:
	mov	w8, #9
.LBB11_53:
	add	x8, x22, x8, lsl #2
	add	x8, x8, #1272           // =1272
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x0, [x19]
	ldr	w2, [x19, #12]
	mov	x1, x20
	bl	__f2fs_submit_read_bio
	mov	w0, wzr
.LBB11_54:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end11:
	.size	f2fs_submit_page_bio, .Lfunc_end11-f2fs_submit_page_bio
                                        // -- End function
	.p2align	2               // -- Begin function __bio_alloc
	.type	__bio_alloc,@function
__bio_alloc:                            // @__bio_alloc
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x21, [x0]
	adrp	x2, f2fs_bioset
	mov	x20, x0
	add	x2, x2, :lo12:f2fs_bioset
	mov	w0, #3072
	mov	x29, sp
	bl	bio_alloc_bioset
	mov	x19, x0
	cbz	x0, .LBB12_2
// %bb.1:
	adrp	x8, f2fs_print_frag_info
	add	x8, x8, :lo12:f2fs_print_frag_info
	str	x8, [x19, #344]
.LBB12_2:
	ldr	x8, [x21]
	ldr	w9, [x21, #3128]
	ldr	w22, [x20, #28]
	ldr	x8, [x8, #248]
	cmp	w9, #2                  // =2
	b.lt	.LBB12_8
// %bb.3:
	ldr	x10, [x21, #3136]
	add	x10, x10, #80           // =80
	b	.LBB12_5
.LBB12_4:                               //   in Loop: Header=BB12_5 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB12_8
.LBB12_5:                               // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	subs	w11, w22, w11
	b.lo	.LBB12_4
// %bb.6:                               //   in Loop: Header=BB12_5 Depth=1
	ldr	w12, [x10]
	cmp	w12, w22
	b.lo	.LBB12_4
// %bb.7:
	ldur	x8, [x10, #-80]
	mov	w22, w11
.LBB12_8:
	cbz	x19, .LBB12_12
// %bb.9:
	ldr	x9, [x19, #8]
	ldr	x10, [x8, #128]
	cmp	x9, x10
	b.eq	.LBB12_11
// %bb.10:
	ldrh	w9, [x19, #20]
	and	w9, w9, #0xfffffdff
	strh	w9, [x19, #20]
	ldr	x9, [x8, #128]
.LBB12_11:
	str	x9, [x19, #8]
	ldrb	w8, [x8, #108]
	mov	x0, x19
	strb	w8, [x19, #27]
	bl	bio_associate_blkg
	mov	w8, w22
	lsl	x8, x8, #3
	str	x8, [x19, #32]
.LBB12_12:
	ldr	w8, [x20, #20]
	cbz	w8, .LBB12_14
// %bb.13:
	adrp	x8, f2fs_write_end_io
	add	x8, x8, :lo12:f2fs_write_end_io
	stp	x8, x21, [x19, #64]
	ldp	w1, w2, [x20, #12]
	mov	x0, x21
	bl	f2fs_io_type_to_rw_hint
	strh	w0, [x19, #24]
	ldr	x8, [x20, #104]
	cbnz	x8, .LBB12_15
	b	.LBB12_17
.LBB12_14:
	adrp	x8, f2fs_read_end_io
	add	x8, x8, :lo12:f2fs_read_end_io
	stp	x8, xzr, [x19, #64]
	ldr	x8, [x20, #104]
	cbz	x8, .LBB12_17
.LBB12_15:
	ldr	x8, [x8, #40]
	cbz	x8, .LBB12_17
// %bb.16:
	ldr	x1, [x8, #672]
	mov	x0, x19
	bl	bio_associate_blkg_from_css
.LBB12_17:
	mov	x0, x19
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end12:
	.size	__bio_alloc, .Lfunc_end12-__bio_alloc
                                        // -- End function
	.p2align	2               // -- Begin function __f2fs_submit_read_bio
	.type	__f2fs_submit_read_bio,@function
__f2fs_submit_read_bio:                 // @__f2fs_submit_read_bio
// %bb.0:
	sub	sp, sp, #384            // =384
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #288]    // 16-byte Folded Spill
	add	x29, sp, #288           // =288
	stp	x28, x27, [sp, #304]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #320]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #336]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #352]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #368]    // 16-byte Folded Spill
	stur	x8, [x29, #-16]
	adrp	x26, __tracepoint_android_fs_dataread_start+8
	ldr	w8, [x26, :lo12:__tracepoint_android_fs_dataread_start+8]
	mov	w19, w2
	mov	x20, x1
	mov	x21, x0
	cbnz	w2, .LBB13_18
// %bb.1:
	cmp	w8, #1                  // =1
	b.lt	.LBB13_18
// %bb.2:
	ldr	x8, [x20, #472]
	ldr	x22, [x8]
	cbz	x22, .LBB13_18
// %bb.3:
	ldr	x0, [x22, #24]
	cbz	x0, .LBB13_18
// %bb.4:
	ldr	x8, [x22, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x22, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB13_6
// %bb.5:
	ldr	x8, [x22]
	tbnz	w8, #10, .LBB13_20
.LBB13_6:
	ldr	x8, [x0]
	ldr	x9, [x8, #40]
	ldr	x8, [x8, #64]
	ldr	x9, [x9, #1008]
	ldr	w10, [x9, #1180]
	cmp	x8, x10
	b.eq	.LBB13_18
// %bb.7:
	ldr	w9, [x9, #1176]
	cmp	x8, x9
	b.eq	.LBB13_18
.LBB13_8:
	stp	xzr, xzr, [sp, #256]
	stp	xzr, xzr, [sp, #240]
	stp	xzr, xzr, [sp, #224]
	stp	xzr, xzr, [sp, #208]
	stp	xzr, xzr, [sp, #192]
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	stp	xzr, xzr, [sp, #128]
	stp	xzr, xzr, [sp, #112]
	stp	xzr, xzr, [sp, #96]
	stp	xzr, xzr, [sp, #80]
	stp	xzr, xzr, [sp, #64]
	stp	xzr, xzr, [sp, #48]
	stp	xzr, xzr, [sp, #32]
	stp	xzr, xzr, [sp, #16]
	ldr	x8, [x22, #24]
	ldr	x23, [x8]
	mov	x0, x23
	bl	ihold
	mov	x0, x23
	bl	d_obtain_alias
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB13_21
// %bb.9:
	add	x1, sp, #16             // =16
	mov	w2, #256
	mov	x24, x0
	bl	dentry_path_raw
	mov	x23, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB13_22
.LBB13_10:
	mov	x0, x24
	bl	dput
.LBB13_11:
	ldp	x9, x8, [x22, #24]
	ldr	w22, [x20, #40]
	//APP
	mrs x12, sp_el0
	//NO_APP
	ldr	w24, [x12, #1536]
	ldr	x25, [x9]
	ldr	w9, [x26, :lo12:__tracepoint_android_fs_dataread_start+8]
	cmp	w9, #1                  // =1
	b.lt	.LBB13_18
// %bb.12:
	adrp	x10, cpu_number
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
	adrp	x11, __cpu_online_mask
	add	x11, x11, :lo12:__cpu_online_mask
	add	w10, w9, #63            // =63
	cmp	w9, #0                  // =0
	csel	w10, w10, w9, lt
	asr	w10, w10, #6
	ldr	x10, [x11, w10, sxtw #3]
	lsr	x9, x10, x9
	tbz	w9, #0, .LBB13_18
// %bb.13:
	ldr	w9, [x12, #24]
	str	x12, [sp, #8]           // 8-byte Folded Spill
	add	w9, w9, #1              // =1
	str	w9, [x12, #24]
	//APP
	//NO_APP
	adrp	x9, __tracepoint_android_fs_dataread_start+32
	ldr	x28, [x9, :lo12:__tracepoint_android_fs_dataread_start+32]
	cbz	x28, .LBB13_16
// %bb.14:
	lsl	x26, x8, #12
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	add	x27, x8, #2000          // =2000
.LBB13_15:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x28]
	mov	x1, x25
	mov	x2, x26
	mov	w3, w22
	mov	w4, w24
	mov	x5, x23
	mov	x6, x27
	blr	x8
	ldr	x8, [x28, #24]!
	cbnz	x8, .LBB13_15
.LBB13_16:
	//APP
	//NO_APP
	ldr	x9, [sp, #8]            // 8-byte Folded Reload
	ldr	x8, [x9, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x9, #24]
	b.eq	.LBB13_23
// %bb.17:
	ldr	x8, [x9, #24]
	cbz	x8, .LBB13_23
.LBB13_18:
	mov	x0, x21
	mov	x1, x20
	mov	w2, w19
	bl	__submit_bio
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB13_24
// %bb.19:
	ldp	x20, x19, [sp, #368]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #352]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #336]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #320]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #304]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #288]    // 16-byte Folded Reload
	add	sp, sp, #384            // =384
	ret
.LBB13_20:
	mov	x0, x22
	bl	__page_file_mapping
	cbnz	x0, .LBB13_6
	b	.LBB13_8
.LBB13_21:
	adrp	x1, .L.str.10
	add	x1, x1, :lo12:.L.str.10
	add	x0, sp, #16             // =16
	add	x23, sp, #16            // =16
	bl	strcpy
	b	.LBB13_11
.LBB13_22:
	adrp	x1, .L.str.10
	add	x1, x1, :lo12:.L.str.10
	add	x0, sp, #16             // =16
	add	x23, sp, #16            // =16
	bl	strcpy
	b	.LBB13_10
.LBB13_23:
	bl	preempt_schedule_notrace
	b	.LBB13_18
.LBB13_24:
	bl	__stack_chk_fail
.Lfunc_end13:
	.size	__f2fs_submit_read_bio, .Lfunc_end13-__f2fs_submit_read_bio
                                        // -- End function
	.globl	f2fs_submit_merged_ipu_write // -- Begin function f2fs_submit_merged_ipu_write
	.p2align	2
	.type	f2fs_submit_merged_ipu_write,@function
f2fs_submit_merged_ipu_write:           // @f2fs_submit_merged_ipu_write
.Lf2fs_submit_merged_ipu_write$local:
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	mov	x21, x2
	mov	x19, x1
	mov	x20, x0
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x1, .LBB14_2
// %bb.1:
	ldr	x22, [x19]
	b	.LBB14_3
.LBB14_2:
	mov	x22, xzr
.LBB14_3:
	mov	x26, xzr
	mov	w24, wzr
	mov	w28, #296
.LBB14_4:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB14_9 Depth 2
                                        //     Child Loop BB14_15 Depth 2
	ldr	x8, [x20, #144]
	madd	x27, x26, x28, x8
	ldr	x9, [x27, #232]!
	cmp	x27, x9
	b.eq	.LBB14_26
// %bb.5:                               //   in Loop: Header=BB14_4 Depth=1
	madd	x8, x26, x28, x8
	add	x23, x8, #248           // =248
	mov	x0, x23
	bl	down_read
	ldr	x25, [x27]
	cmp	x27, x25
	b.ne	.LBB14_9
// %bb.6:                               //   in Loop: Header=BB14_4 Depth=1
	mov	x0, x23
	bl	up_read
	tbnz	w24, #0, .LBB14_12
	b	.LBB14_20
.LBB14_7:                               //   in Loop: Header=BB14_9 Depth=2
	mov	x1, xzr
	mov	x2, x21
	mov	w3, wzr
	bl	__has_merged_page
	tbnz	w0, #0, .LBB14_11
.LBB14_8:                               //   in Loop: Header=BB14_9 Depth=2
	ldr	x25, [x25]
	cmp	x27, x25
	b.eq	.LBB14_19
.LBB14_9:                               //   Parent Loop BB14_4 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	x0, [x25, #-8]
	cbz	x22, .LBB14_7
// %bb.10:                              //   in Loop: Header=BB14_9 Depth=2
	cmp	x22, x0
	b.ne	.LBB14_8
.LBB14_11:                              //   in Loop: Header=BB14_4 Depth=1
	mov	x0, x23
	bl	up_read
.LBB14_12:                              //   in Loop: Header=BB14_4 Depth=1
	mov	x0, x23
	bl	down_write
	ldr	x24, [x27]
	b	.LBB14_15
.LBB14_13:                              //   in Loop: Header=BB14_15 Depth=2
	mov	x1, xzr
	mov	x2, x21
	mov	w3, wzr
	bl	__has_merged_page
	tbnz	w0, #0, .LBB14_21
.LBB14_14:                              //   in Loop: Header=BB14_15 Depth=2
	ldr	x24, [x24]
.LBB14_15:                              //   Parent Loop BB14_4 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	cmp	x27, x24
	b.eq	.LBB14_18
// %bb.16:                              //   in Loop: Header=BB14_15 Depth=2
	mov	x25, x24
	ldr	x0, [x25, #-8]!
	cbz	x22, .LBB14_13
// %bb.17:                              //   in Loop: Header=BB14_15 Depth=2
	cmp	x22, x0
	b.ne	.LBB14_14
	b	.LBB14_22
.LBB14_18:                              //   in Loop: Header=BB14_4 Depth=1
	mov	w24, wzr
	b	.LBB14_25
.LBB14_19:                              //   in Loop: Header=BB14_4 Depth=1
	mov	x0, x23
	bl	up_read
.LBB14_20:                              //   in Loop: Header=BB14_4 Depth=1
	mov	w24, wzr
	b	.LBB14_26
.LBB14_21:                              //   in Loop: Header=BB14_4 Depth=1
	ldr	x22, [x25]
.LBB14_22:                              //   in Loop: Header=BB14_4 Depth=1
	mov	x0, x24
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB14_24
// %bb.23:                              //   in Loop: Header=BB14_4 Depth=1
	ldp	x9, x8, [x24]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB14_24:                              //   in Loop: Header=BB14_4 Depth=1
	mov	x9, #256
	movk	x9, #57005, lsl #48
	add	x8, x9, #34             // =34
	stp	x9, x8, [x24]
	adrp	x8, bio_entry_slab
	ldr	x0, [x8, :lo12:bio_entry_slab]
	mov	x1, x25
	bl	kmem_cache_free
	mov	w24, #1
.LBB14_25:                              //   in Loop: Header=BB14_4 Depth=1
	mov	x0, x23
	bl	up_write
.LBB14_26:                              //   in Loop: Header=BB14_4 Depth=1
	cmp	x26, #1                 // =1
	b.hi	.LBB14_28
// %bb.27:                              //   in Loop: Header=BB14_4 Depth=1
	add	x26, x26, #1            // =1
	tbz	w24, #0, .LBB14_4
.LBB14_28:
	tbz	w24, #0, .LBB14_30
// %bb.29:
	mov	x0, x20
	mov	x1, x22
	mov	w2, wzr
	bl	__submit_bio
.LBB14_30:
	cbz	x19, .LBB14_33
// %bb.31:
	ldr	x0, [x19]
	cbz	x0, .LBB14_33
// %bb.32:
	bl	bio_put
	str	xzr, [x19]
.LBB14_33:
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.Lfunc_end14:
	.size	f2fs_submit_merged_ipu_write, .Lfunc_end14-f2fs_submit_merged_ipu_write
                                        // -- End function
	.p2align	2               // -- Begin function __has_merged_page
	.type	__has_merged_page,@function
__has_merged_page:                      // @__has_merged_page
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB15_22
// %bb.1:
	mov	w19, w3
	mov	x20, x2
	mov	x21, x1
	mov	x22, x0
	cbnz	x1, .LBB15_5
// %bb.2:
	cbnz	x20, .LBB15_5
// %bb.3:
	cbnz	w19, .LBB15_5
.LBB15_4:
	mov	w0, #1
	b	.LBB15_22
.LBB15_5:
	ldrh	w8, [x22, #458]
	cbz	w8, .LBB15_21
// %bb.6:
	mov	x27, #-549755813888
	mov	w24, wzr
	mov	w26, wzr
	mov	x28, xzr
	mov	w25, #4096
	movk	x27, #4076
	b	.LBB15_8
.LBB15_7:                               //   in Loop: Header=BB15_8 Depth=1
	ldrh	w8, [x22, #458]
	cmp	w26, w8
	b.hs	.LBB15_21
.LBB15_8:                               // =>This Inner Loop Header: Depth=1
	ldr	x9, [x22, #472]
	mov	w8, w26
	cbz	w24, .LBB15_10
// %bb.9:                               //   in Loop: Header=BB15_8 Depth=1
	mov	w10, wzr
	add	x28, x28, #64           // =64
	b	.LBB15_11
.LBB15_10:                              //   in Loop: Header=BB15_8 Depth=1
	add	x10, x9, x8, lsl #4
	ldr	w11, [x10, #12]
	ldr	x10, [x10]
	lsr	x12, x11, #6
	and	x12, x12, #0x3ffffc0
	add	x28, x10, x12
	and	w10, w11, #0xfff
.LBB15_11:                              //   in Loop: Header=BB15_8 Depth=1
	add	x9, x9, x8, lsl #4
	ldr	w9, [x9, #8]
	sub	w10, w25, w10
	ldr	x11, [x28, #24]
	mov	x23, x28
	sub	w12, w9, w24
	cmp	w10, w12
	csel	w10, w10, w12, lo
	add	w10, w10, w24
	cmp	w10, w9
	cinc	w26, w8, eq
	csel	w24, wzr, w10, eq
	cbnz	x11, .LBB15_13
// %bb.12:                              //   in Loop: Header=BB15_8 Depth=1
	ldr	x23, [x28, #40]
	cmn	x23, #1, lsl #12        // =4096
	b.hi	.LBB15_7
.LBB15_13:                              //   in Loop: Header=BB15_8 Depth=1
	mov	x0, x23
	bl	f2fs_is_compressed_page
	tbz	w0, #0, .LBB15_15
// %bb.14:                              //   in Loop: Header=BB15_8 Depth=1
	mov	x0, x23
	bl	f2fs_compress_control_page
	mov	x23, x0
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB15_7
.LBB15_15:                              //   in Loop: Header=BB15_8 Depth=1
	cbz	x21, .LBB15_17
// %bb.16:                              //   in Loop: Header=BB15_8 Depth=1
	ldr	x8, [x23, #24]
	ldr	x8, [x8]
	cmp	x8, x21
	b.eq	.LBB15_4
.LBB15_17:                              //   in Loop: Header=BB15_8 Depth=1
	cbz	x20, .LBB15_19
// %bb.18:                              //   in Loop: Header=BB15_8 Depth=1
	cmp	x23, x20
	b.eq	.LBB15_4
.LBB15_19:                              //   in Loop: Header=BB15_8 Depth=1
	cbz	w19, .LBB15_7
// %bb.20:                              //   in Loop: Header=BB15_8 Depth=1
	mov	x8, #134217728
	movk	x8, #64, lsl #32
	add	x8, x8, x23, lsl #6
	and	x8, x8, #0xfffffffffffff000
	ldr	w8, [x8, x27]
	cmp	w8, w19
	b.ne	.LBB15_7
	b	.LBB15_4
.LBB15_21:
	mov	w0, wzr
.LBB15_22:
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.Lfunc_end15:
	.size	__has_merged_page, .Lfunc_end15-__has_merged_page
                                        // -- End function
	.globl	f2fs_merge_page_bio     // -- Begin function f2fs_merge_page_bio
	.p2align	2
	.type	f2fs_merge_page_bio,@function
f2fs_merge_page_bio:                    // @f2fs_merge_page_bio
.Lf2fs_merge_page_bio$local:
// %bb.0:
	sub	sp, sp, #128            // =128
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	add	x29, sp, #32            // =32
	stp	x28, x27, [sp, #48]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #64]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #80]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #96]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #112]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	str	xzr, [sp, #16]
	ldr	x8, [x0, #112]
	mov	x19, x0
	ldr	x8, [x8]
	str	x8, [sp, #16]
	ldr	x20, [x0, #48]
	cbnz	x20, .LBB16_2
// %bb.1:
	ldr	x20, [x19, #40]
.LBB16_2:
	ldr	w8, [x19, #12]
	ldr	x0, [x19]
	ldr	w1, [x19, #28]
	mov	w9, #6
	cmp	w8, #1                  // =1
	mov	w8, #9
	csel	w2, w8, w9, hi
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB16_64
// %bb.3:
	adrp	x8, __tracepoint_f2fs_submit_page_bio+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_submit_page_bio+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB16_10
// %bb.4:
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
	tbz	w8, #0, .LBB16_10
// %bb.5:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB16_10
// %bb.6:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w8, [x21, #24]
	add	w8, w8, #1              // =1
	str	w8, [x21, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_submit_page_bio+32
	ldr	x22, [x8, :lo12:__tracepoint_f2fs_submit_page_bio+32]
	cbz	x22, .LBB16_8
.LBB16_7:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x22]
	mov	x1, x20
	mov	x2, x19
	blr	x8
	ldr	x8, [x22, #24]!
	cbnz	x8, .LBB16_7
.LBB16_8:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	b.eq	.LBB16_110
// %bb.9:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB16_110
.LBB16_10:
	ldr	x8, [sp, #16]
	cbz	x8, .LBB16_67
.LBB16_11:
	ldr	x9, [x19, #120]
	ldr	x0, [x19]
	ldr	w10, [x9]
	ldr	x11, [x0, #1224]
	ldr	w9, [x19, #28]
	cbz	x11, .LBB16_13
// %bb.12:
	ldr	w12, [x8, #40]
	cmp	x11, x12
	b.ls	.LBB16_22
.LBB16_13:
	add	w10, w10, #1            // =1
	cmp	w10, w9
	b.ne	.LBB16_22
// %bb.14:
	ldr	x10, [x0]
	ldr	w11, [x0, #3128]
	ldr	x10, [x10, #248]
	cmp	w11, #2                 // =2
	b.lt	.LBB16_20
// %bb.15:
	ldr	x12, [x0, #3136]
	add	x12, x12, #80           // =80
	b	.LBB16_17
.LBB16_16:                              //   in Loop: Header=BB16_17 Depth=1
	subs	x11, x11, #1            // =1
	add	x12, x12, #88           // =88
	b.eq	.LBB16_20
.LBB16_17:                              // =>This Inner Loop Header: Depth=1
	ldur	w13, [x12, #-4]
	cmp	w13, w9
	b.hi	.LBB16_16
// %bb.18:                              //   in Loop: Header=BB16_17 Depth=1
	ldr	w13, [x12]
	cmp	w13, w9
	b.lo	.LBB16_16
// %bb.19:
	ldur	x10, [x12, #-80]
.LBB16_20:
	ldr	x9, [x8, #8]
	ldr	x11, [x10, #128]
	cmp	x9, x11
	b.ne	.LBB16_22
// %bb.21:
	ldrb	w9, [x8, #27]
	ldrb	w10, [x10, #108]
	cmp	w9, w10
	b.eq	.LBB16_23
.LBB16_22:
	add	x1, sp, #16             // =16
	mov	x2, xzr
	bl	f2fs_submit_merged_ipu_write
	ldr	x8, [sp, #16]
.LBB16_23:
	cbz	x8, .LBB16_67
// %bb.24:
	ldr	x21, [x19]
	mov	w8, #-11
	mov	x22, xzr
	mov	w28, wzr
	str	w8, [sp, #12]           // 4-byte Folded Spill
	mov	w23, #296
	add	x8, x21, #3408          // =3408
	str	x8, [sp]                // 8-byte Folded Spill
.LBB16_25:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB16_27 Depth 2
                                        //     Child Loop BB16_35 Depth 2
	ldr	x8, [x21, #144]
	madd	x24, x22, x23, x8
	add	x25, x24, #248          // =248
	mov	x0, x25
	bl	down_write
	ldr	x26, [x24, #232]!
	cmp	x24, x26
	b.eq	.LBB16_54
// %bb.26:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x8, [sp, #16]
.LBB16_27:                              //   Parent Loop BB16_25 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	mov	x27, x26
	ldr	x9, [x27, #-8]!
	cmp	x9, x8
	b.eq	.LBB16_29
// %bb.28:                              //   in Loop: Header=BB16_27 Depth=2
	ldr	x26, [x26]
	cmp	x24, x26
	b.ne	.LBB16_27
	b	.LBB16_54
.LBB16_29:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x9, [x19, #120]
	ldr	x11, [x21, #1224]
	ldr	w10, [x9]
	ldr	w9, [x19, #28]
	cbz	x11, .LBB16_31
// %bb.30:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	w12, [x8, #40]
	cmp	x11, x12
	b.ls	.LBB16_56
.LBB16_31:                              //   in Loop: Header=BB16_25 Depth=1
	add	w10, w10, #1            // =1
	cmp	w10, w9
	b.ne	.LBB16_56
// %bb.32:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x10, [x21]
	ldr	w11, [x21, #3128]
	ldr	x10, [x10, #248]
	cmp	w11, #2                 // =2
	b.lt	.LBB16_38
// %bb.33:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x12, [x21, #3136]
	add	x12, x12, #80           // =80
	b	.LBB16_35
.LBB16_34:                              //   in Loop: Header=BB16_35 Depth=2
	subs	x11, x11, #1            // =1
	add	x12, x12, #88           // =88
	b.eq	.LBB16_38
.LBB16_35:                              //   Parent Loop BB16_25 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w13, [x12, #-4]
	cmp	w13, w9
	b.hi	.LBB16_34
// %bb.36:                              //   in Loop: Header=BB16_35 Depth=2
	ldr	w13, [x12]
	cmp	w13, w9
	b.lo	.LBB16_34
// %bb.37:                              //   in Loop: Header=BB16_25 Depth=1
	ldur	x10, [x12, #-80]
.LBB16_38:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x9, [x8, #8]
	ldr	x11, [x10, #128]
	cmp	x9, x11
	b.ne	.LBB16_56
// %bb.39:                              //   in Loop: Header=BB16_25 Depth=1
	ldrb	w8, [x8, #27]
	ldrb	w9, [x10, #108]
	cmp	w8, w9
	b.ne	.LBB16_56
.LBB16_40:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x8, [x19, #40]
	ldr	x0, [sp, #16]
	ldp	x9, x2, [x8, #24]
	ldr	x1, [x9]
	cbz	x19, .LBB16_45
// %bb.41:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x8, [x19, #48]
	cbz	x8, .LBB16_45
// %bb.42:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x8, [x0, #448]
	cbnz	x8, .LBB16_50
// %bb.43:                              //   in Loop: Header=BB16_25 Depth=1
	ldrb	w9, [x1, #13]
	ldrb	w8, [x0, #456]
	tbnz	w9, #6, .LBB16_49
// %bb.44:                              //   in Loop: Header=BB16_25 Depth=1
	cbnz	w8, .LBB16_50
	b	.LBB16_47
.LBB16_45:                              //   in Loop: Header=BB16_25 Depth=1
	bl	fscrypt_mergeable_bio
	tbz	w0, #0, .LBB16_50
// %bb.46:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x0, [sp, #16]
.LBB16_47:                              //   in Loop: Header=BB16_25 Depth=1
	mov	w2, #4096
	mov	x1, x20
	mov	w3, wzr
	bl	bio_add_page
	cmp	w0, #1, lsl #12         // =4096
	b.ne	.LBB16_50
// %bb.48:                              //   in Loop: Header=BB16_25 Depth=1
	str	wzr, [sp, #12]          // 4-byte Folded Spill
	b	.LBB16_53
.LBB16_49:                              //   in Loop: Header=BB16_25 Depth=1
	ldrh	w9, [x1]
	cmp	w8, #0                  // =0
	cset	w8, eq
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	cset	w9, eq
	cmp	w8, w9
	b.ne	.LBB16_47
.LBB16_50:                              //   in Loop: Header=BB16_25 Depth=1
	mov	x0, x26
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB16_52
// %bb.51:                              //   in Loop: Header=BB16_25 Depth=1
	ldp	x9, x8, [x26]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB16_52:                              //   in Loop: Header=BB16_25 Depth=1
	mov	x9, #256
	movk	x9, #57005, lsl #48
	add	x8, x9, #34             // =34
	stp	x9, x8, [x26]
	adrp	x8, bio_entry_slab
	ldr	x0, [x8, :lo12:bio_entry_slab]
	mov	x1, x27
	bl	kmem_cache_free
	ldr	x1, [sp, #16]
	mov	x0, x21
	mov	w2, wzr
	bl	__submit_bio
.LBB16_53:                              //   in Loop: Header=BB16_25 Depth=1
	mov	w28, #1
.LBB16_54:                              //   in Loop: Header=BB16_25 Depth=1
	mov	x0, x25
	bl	up_write
	cmp	x22, #1                 // =1
	b.hi	.LBB16_65
// %bb.55:                              //   in Loop: Header=BB16_25 Depth=1
	add	x22, x22, #1            // =1
	tbz	w28, #0, .LBB16_25
	b	.LBB16_65
.LBB16_56:                              //   in Loop: Header=BB16_25 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_59
// %bb.57:                              //   in Loop: Header=BB16_25 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_59
// %bb.58:                              //   in Loop: Header=BB16_25 Depth=1
	adrp	x24, f2fs_dclient
	ldr	x0, [x24, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.add_ipu_page
	mov	w3, #1359
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.add_ipu_page
	bl	dsm_client_record
	ldr	x0, [x24, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_59:                              //   in Loop: Header=BB16_25 Depth=1
	mov	x0, x21
	bl	f2fs_print_raw_sb_info
	mov	x0, x21
	bl	f2fs_print_ckpt_info
	mov	x0, x21
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 1359; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x21, .LBB16_40
// %bb.60:                              //   in Loop: Header=BB16_25 Depth=1
	add	x8, x21, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x21, #3376]
	cbnz	w8, .LBB16_40
// %bb.61:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB16_40
// %bb.62:                              //   in Loop: Header=BB16_25 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB16_40
// %bb.63:                              //   in Loop: Header=BB16_25 Depth=1
	ldr	x2, [sp]                // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB16_40
.LBB16_64:
	mov	w0, #-117
	b	.LBB16_108
.LBB16_65:
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	cbz	w8, .LBB16_92
// %bb.66:
	ldr	x0, [sp, #16]
	bl	bio_put
	str	xzr, [sp, #16]
.LBB16_67:
	mov	w1, #256
	mov	x0, x19
	bl	__bio_alloc
	str	x0, [sp, #16]
	ldr	w8, [x19, #12]
	mov	x23, x0
	cbnz	w8, .LBB16_72
// %bb.68:
	ldr	x8, [x19]
	ldr	w9, [x19, #16]
	mov	w10, #1
	ldr	w8, [x8, #2724]
	lsl	w9, w10, w9
	and	w9, w9, #0x7
	tst	w9, w8, lsr #3
	b.eq	.LBB16_70
// %bb.69:
	ldr	w10, [x19, #24]
	orr	w10, w10, #0x1000
	str	w10, [x19, #24]
.LBB16_70:
	tst	w9, w8
	b.eq	.LBB16_72
// %bb.71:
	ldr	w8, [x19, #24]
	orr	w8, w8, #0x40000
	str	w8, [x19, #24]
.LBB16_72:
	ldr	x8, [x19, #40]
	ldp	x9, x2, [x8, #24]
	ldr	x1, [x9]
	cbz	x19, .LBB16_77
// %bb.73:
	ldr	x8, [x19, #48]
	cbz	x8, .LBB16_77
// %bb.74:
	ldrb	w8, [x1, #13]
	tbz	w8, #6, .LBB16_78
// %bb.75:
	ldrh	w8, [x1]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB16_78
// %bb.76:
	mov	w8, #1
	strb	w8, [x23, #456]
	b	.LBB16_78
.LBB16_77:
	mov	w3, #3072
	mov	x0, x23
	bl	fscrypt_set_bio_crypt_ctx
	ldr	x23, [sp, #16]
.LBB16_78:
	ldp	w8, w9, [x19, #20]
	mov	w1, #3136
	orr	w8, w9, w8
	str	w8, [x23, #16]
	ldr	x22, [x19]
	adrp	x8, bio_entry_slab
	ldr	x24, [x8, :lo12:bio_entry_slab]
	ldr	w25, [x19, #16]
	ldr	x26, [x22, #144]
	mov	x0, x24
	bl	kmem_cache_alloc
	mov	x21, x0
	cbnz	x0, .LBB16_80
// %bb.79:
	mov	w1, #35904
	mov	x0, x24
	bl	kmem_cache_alloc
	mov	x21, x0
.LBB16_80:
	str	x23, [x21]
	ldrh	w8, [x23, #20]
	mov	w2, #4096
	mov	x0, x23
	mov	x1, x20
	orr	w8, w8, #0x100
	strh	w8, [x23, #20]
	//APP
	dmb ish
	//NO_APP
	add	x8, x23, #464           // =464
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	mov	w3, wzr
	bl	bio_add_page
	cmp	w0, #1, lsl #12         // =4096
	b.eq	.LBB16_89
// %bb.81:
	bl	print_bdev_access_info
	adrp	x23, f2fs_dclient
	ldr	x0, [x23, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_84
// %bb.82:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_84
// %bb.83:
	ldr	x0, [x23, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.add_bio_entry
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.add_bio_entry
	mov	w3, #1324
	bl	dsm_client_record
	ldr	x0, [x23, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_84:
	mov	x0, x22
	bl	f2fs_print_raw_sb_info
	mov	x0, x22
	bl	f2fs_print_ckpt_info
	mov	x0, x22
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 1324; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x22, .LBB16_89
// %bb.85:
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
	cbnz	w8, .LBB16_89
// %bb.86:
	ldr	x1, [x22, #3392]
	cbz	x1, .LBB16_89
// %bb.87:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB16_89
// %bb.88:
	add	x2, x22, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB16_89:
	mov	w8, #296
	madd	x26, x25, x8, x26
	add	x22, x26, #248          // =248
	mov	x0, x22
	bl	down_write
	ldr	x23, [x26, #240]!
	add	x24, x21, #8            // =8
	mov	x0, x24
	sub	x25, x26, #8            // =8
	mov	x1, x23
	mov	x2, x25
	bl	__list_add_valid
	tbz	w0, #0, .LBB16_91
// %bb.90:
	str	x24, [x26]
	stp	x25, x23, [x21, #8]
	str	x24, [x23]
.LBB16_91:
	mov	x0, x22
	bl	up_write
.LBB16_92:
	ldr	x0, [x19, #104]
	cbz	x0, .LBB16_94
// %bb.93:
	mov	w2, #4096
	mov	x1, x20
	bl	wbc_account_cgroup_owner
.LBB16_94:
	ldr	x8, [x20, #24]
	ldr	x21, [x19]
	cbz	x8, .LBB16_106
// %bb.95:
	ldr	x22, [x8]
	ldr	x8, [x22, #40]
	ldr	x9, [x8, #1008]
	ldr	x8, [x22, #64]
	ldr	w10, [x9, #1180]
	cmp	x8, x10
	b.eq	.LBB16_98
// %bb.96:
	ldr	w9, [x9, #1176]
	cmp	x8, x9
	b.eq	.LBB16_98
// %bb.97:
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.ne	.LBB16_99
.LBB16_98:
	mov	w8, #7
	b	.LBB16_107
.LBB16_99:
	mov	x0, x20
	bl	f2fs_is_compressed_page
	tbnz	w0, #0, .LBB16_106
// %bb.100:
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB16_103
// %bb.101:
	ldr	x8, [x22, #736]
	tbnz	w8, #15, .LBB16_98
// %bb.102:
	ldrb	w8, [x22, #12]
	tbnz	w8, #5, .LBB16_98
.LBB16_103:
	ldr	x8, [x20]
	tbz	w8, #13, .LBB16_106
// %bb.104:
	ldr	x8, [x20, #40]
	tbz	w8, #0, .LBB16_106
// %bb.105:
	ldr	x8, [x20, #40]
	tbnz	w8, #3, .LBB16_98
.LBB16_106:
	mov	w8, #8
.LBB16_107:
	add	x8, x21, x8, lsl #2
	add	x8, x8, #1272           // =1272
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	w8, [x19, #28]
	ldr	x9, [x19, #120]
	mov	w0, wzr
	str	x8, [x9]
	ldr	x8, [sp, #16]
	ldr	x9, [x19, #112]
	str	x8, [x9]
.LBB16_108:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB16_111
// %bb.109:
	ldp	x20, x19, [sp, #112]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #96]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #80]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #64]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #48]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #128            // =128
	ret
.LBB16_110:
	bl	preempt_schedule_notrace
	ldr	x8, [sp, #16]
	cbnz	x8, .LBB16_11
	b	.LBB16_67
.LBB16_111:
	bl	__stack_chk_fail
.Lfunc_end16:
	.size	f2fs_merge_page_bio, .Lfunc_end16-f2fs_merge_page_bio
                                        // -- End function
	.globl	f2fs_submit_page_write  // -- Begin function f2fs_submit_page_write
	.p2align	2
	.type	f2fs_submit_page_write,@function
f2fs_submit_page_write:                 // @f2fs_submit_page_write
.Lf2fs_submit_page_write$local:
// %bb.0:
	sub	sp, sp, #144            // =144
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #64]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #80]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #96]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #112]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #128]    // 16-byte Folded Spill
	ldp	w8, w22, [x0, #12]
	ldr	x19, [x0]
	mov	w9, #2
	mov	x21, x0
	cmp	w8, #2                  // =2
	csel	w8, w8, w9, lo
	add	x8, x19, w8, uxtw #3
	ldr	x23, [x8, #144]
	ldr	w8, [x0, #20]
	mov	w9, #296
	add	x29, sp, #48            // =48
	madd	x20, x22, x9, x23
	cbz	w8, .LBB17_102
.LBB17_1:
	mov	w8, #296
	madd	x23, x22, x8, x23
	add	x0, x23, #160           // =160
	str	x0, [sp, #8]            // 8-byte Folded Spill
	bl	down_write
	add	x8, x23, #208           // =208
	stur	x8, [x29, #-8]          // 8-byte Folded Spill
	add	x8, x23, #216           // =216
	str	x8, [sp, #24]           // 8-byte Folded Spill
	add	x8, x23, #44            // =44
	str	x8, [sp, #16]           // 8-byte Folded Spill
	add	x8, x23, #48            // =48
	str	x8, [sp]                // 8-byte Folded Spill
	ldrb	w8, [x21, #88]
	add	x28, x23, #8            // =8
	add	x22, x23, #16           // =16
	add	x24, x23, #24           // =24
	cmp	w8, #0                  // =0
	mov	w26, #1
	mov	w27, #-1
	cset	w8, eq
	//APP
	mrs x9, sp_el0
	//NO_APP
	stur	x9, [x29, #-16]         // 8-byte Folded Spill
	b	.LBB17_5
.LBB17_2:                               //   in Loop: Header=BB17_5 Depth=1
	mov	w8, #7
.LBB17_3:                               //   in Loop: Header=BB17_5 Depth=1
	add	x8, x19, x8, lsl #2
	add	x8, x8, #1272           // =1272
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w26
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	strb	w26, [x21, #90]
.LBB17_4:                               //   in Loop: Header=BB17_5 Depth=1
	ldrb	w9, [x21, #88]
	mov	w8, wzr
	cbz	w9, .LBB17_97
.LBB17_5:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB17_23 Depth 2
                                        //     Child Loop BB17_47 Depth 2
                                        //     Child Loop BB17_80 Depth 2
	tbnz	w8, #0, .LBB17_10
// %bb.6:                               //   in Loop: Header=BB17_5 Depth=1
	ldur	x0, [x29, #-8]          // 8-byte Folded Reload
	bl	_raw_spin_lock
	ldr	x8, [sp, #24]           // 8-byte Folded Reload
	ldr	x25, [x8]
	cmp	x8, x25
	b.eq	.LBB17_96
// %bb.7:                               //   in Loop: Header=BB17_5 Depth=1
	mov	x0, x25
	sub	x21, x25, #64           // =64
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB17_9
// %bb.8:                               //   in Loop: Header=BB17_5 Depth=1
	ldp	x9, x8, [x25]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB17_9:                               //   in Loop: Header=BB17_5 Depth=1
	mov	x9, #256
	movk	x9, #57005, lsl #48
	add	x8, x9, #34             // =34
	stp	x9, x8, [x25]
	ldur	x0, [x29, #-8]          // 8-byte Folded Reload
	bl	_raw_spin_unlock
.LBB17_10:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w1, [x21, #32]
	ldr	x25, [x21]
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB17_19
.LBB17_11:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w8, [x21, #12]
	ldr	w1, [x21, #28]
	mov	w9, #9
	mov	x0, x25
	cmp	w8, #1                  // =1
	mov	w8, #7
	csel	w2, w9, w8, hi
	bl	verify_blkaddr
	ldr	x25, [x21, #48]
	cbnz	x25, .LBB17_14
// %bb.12:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x25, [x21, #56]
	cbnz	x25, .LBB17_14
// %bb.13:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x25, [x21, #40]
.LBB17_14:                              //   in Loop: Header=BB17_5 Depth=1
	strb	w26, [x21, #80]
	ldr	x8, [x25, #24]
	cbz	x8, .LBB17_35
// %bb.15:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x23, [x8]
	ldr	x8, [x23, #40]
	ldr	x9, [x8, #1008]
	ldr	x8, [x23, #64]
	ldr	w10, [x9, #1180]
	cmp	x8, x10
	b.eq	.LBB17_18
// %bb.16:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w9, [x9, #1176]
	cmp	x8, x9
	b.eq	.LBB17_18
// %bb.17:                              //   in Loop: Header=BB17_5 Depth=1
	ldrh	w8, [x23]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.ne	.LBB17_28
.LBB17_18:                              //   in Loop: Header=BB17_5 Depth=1
	mov	w8, #7
	b	.LBB17_36
.LBB17_19:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w9, [x25, #3128]
	cbz	w9, .LBB17_27
// %bb.20:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x25]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB17_26
// %bb.21:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x10, [x25, #3136]
	add	x10, x10, #80           // =80
	b	.LBB17_23
.LBB17_22:                              //   in Loop: Header=BB17_23 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB17_26
.LBB17_23:                              //   Parent Loop BB17_5 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB17_22
// %bb.24:                              //   in Loop: Header=BB17_23 Depth=2
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB17_22
// %bb.25:                              //   in Loop: Header=BB17_5 Depth=1
	ldur	x8, [x10, #-80]
.LBB17_26:                              //   in Loop: Header=BB17_5 Depth=1
	cbz	x8, .LBB17_11
.LBB17_27:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w8, [x21, #12]
	mov	w9, #6
	mov	x0, x25
	cmp	w8, #1                  // =1
	mov	w8, #9
	csel	w2, w8, w9, hi
	bl	verify_blkaddr
	b	.LBB17_11
.LBB17_28:                              //   in Loop: Header=BB17_5 Depth=1
	mov	x0, x25
	bl	f2fs_is_compressed_page
	tbnz	w0, #0, .LBB17_35
// %bb.29:                              //   in Loop: Header=BB17_5 Depth=1
	ldrh	w8, [x23]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB17_32
// %bb.30:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x23, #736]
	tbnz	w8, #15, .LBB17_18
// %bb.31:                              //   in Loop: Header=BB17_5 Depth=1
	ldrb	w8, [x23, #12]
	tbnz	w8, #5, .LBB17_18
.LBB17_32:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x25]
	tbz	w8, #13, .LBB17_35
// %bb.33:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x25, #40]
	tbz	w8, #0, .LBB17_35
// %bb.34:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x25, #40]
	tbnz	w8, #3, .LBB17_18
.LBB17_35:                              //   in Loop: Header=BB17_5 Depth=1
	mov	w8, #8
.LBB17_36:                              //   in Loop: Header=BB17_5 Depth=1
	add	x8, x19, x8, lsl #2
	add	x8, x8, #1272           // =1272
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x0, [x28]
	cbz	x0, .LBB17_62
// %bb.37:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w10, [x19, #1432]
	ldr	w9, [x22]
	ldr	w8, [x21, #28]
	lsl	w10, w26, w10
	cmp	w10, #2                 // =2
	b.lt	.LBB17_41
// %bb.38:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w11, [x21, #12]
	cmp	w11, #1                 // =1
	b.hi	.LBB17_41
// %bb.39:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w11, [x0, #40]
	sub	w12, w10, #1            // =1
	tst	w12, w11, lsr #12
	b.ne	.LBB17_41
// %bb.40:                              //   in Loop: Header=BB17_5 Depth=1
	ldrh	w11, [x0, #460]
	ldrh	w12, [x0, #458]
	sub	w11, w11, w12
	cmp	w11, w10
	b.lo	.LBB17_61
.LBB17_41:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x10, [x19, #1224]
	cbz	x10, .LBB17_43
// %bb.42:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w11, [x0, #40]
	cmp	x10, x11
	b.ls	.LBB17_61
.LBB17_43:                              //   in Loop: Header=BB17_5 Depth=1
	add	w9, w9, #1              // =1
	cmp	w9, w8
	b.ne	.LBB17_61
// %bb.44:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x9, [x19]
	ldr	w10, [x19, #3128]
	ldr	x9, [x9, #248]
	cmp	w10, #2                 // =2
	b.lt	.LBB17_50
// %bb.45:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x11, [x19, #3136]
	add	x11, x11, #80           // =80
	b	.LBB17_47
.LBB17_46:                              //   in Loop: Header=BB17_47 Depth=2
	subs	x10, x10, #1            // =1
	add	x11, x11, #88           // =88
	b.eq	.LBB17_50
.LBB17_47:                              //   Parent Loop BB17_5 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w12, [x11, #-4]
	cmp	w12, w8
	b.hi	.LBB17_46
// %bb.48:                              //   in Loop: Header=BB17_47 Depth=2
	ldr	w12, [x11]
	cmp	w12, w8
	b.lo	.LBB17_46
// %bb.49:                              //   in Loop: Header=BB17_5 Depth=1
	ldur	x9, [x11, #-80]
.LBB17_50:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x0, #8]
	ldr	x10, [x9, #128]
	cmp	x8, x10
	b.ne	.LBB17_61
// %bb.51:                              //   in Loop: Header=BB17_5 Depth=1
	ldrb	w8, [x0, #27]
	ldrb	w9, [x9, #108]
	cmp	w8, w9
	b.ne	.LBB17_61
// %bb.52:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	ldr	w9, [x21, #20]
	ldr	w8, [x8]
	cmp	w8, w9
	b.ne	.LBB17_61
// %bb.53:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [sp]                // 8-byte Folded Reload
	ldr	w9, [x21, #24]
	ldr	w8, [x8]
	cmp	w8, w9
	b.ne	.LBB17_61
// %bb.54:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x21, #40]
	ldp	x9, x2, [x8, #24]
	ldr	x1, [x9]
	cbz	x21, .LBB17_59
// %bb.55:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x21, #48]
	cbz	x8, .LBB17_59
// %bb.56:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x0, #448]
	cbnz	x8, .LBB17_61
// %bb.57:                              //   in Loop: Header=BB17_5 Depth=1
	ldrb	w9, [x1, #13]
	ldrb	w8, [x0, #456]
	tbnz	w9, #6, .LBB17_60
// %bb.58:                              //   in Loop: Header=BB17_5 Depth=1
	cbnz	w8, .LBB17_61
	b	.LBB17_62
.LBB17_59:                              //   in Loop: Header=BB17_5 Depth=1
	bl	fscrypt_mergeable_bio
	tbz	w0, #0, .LBB17_61
	b	.LBB17_62
.LBB17_60:                              //   in Loop: Header=BB17_5 Depth=1
	ldrh	w9, [x1]
	cmp	w8, #0                  // =0
	cset	w8, eq
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	cset	w9, eq
	eor	w8, w8, w9
	tbnz	w8, #0, .LBB17_62
.LBB17_61:                              //   in Loop: Header=BB17_5 Depth=1
	mov	x0, x20
	bl	__submit_merged_bio
.LBB17_62:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x0, [x28]
	cbnz	x0, .LBB17_73
// %bb.63:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w8, [x19, #1432]
	lsl	w9, w26, w8
	cmp	w9, #2                  // =2
	b.lt	.LBB17_66
// %bb.64:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w9, [x21, #12]
	cmp	w9, #1                  // =1
	b.hi	.LBB17_66
// %bb.65:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w9, [x21, #28]
	lsl	w8, w27, w8
	bics	wzr, w9, w8
	b.ne	.LBB17_84
.LBB17_66:                              //   in Loop: Header=BB17_5 Depth=1
	mov	w1, #256
	mov	x0, x21
	bl	__bio_alloc
	str	x0, [x28]
	ldr	x8, [x21, #40]
	ldp	x9, x2, [x8, #24]
	ldr	x1, [x9]
	cbz	x21, .LBB17_71
// %bb.67:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x21, #48]
	cbz	x8, .LBB17_71
// %bb.68:                              //   in Loop: Header=BB17_5 Depth=1
	ldrb	w8, [x1, #13]
	tbz	w8, #6, .LBB17_72
// %bb.69:                              //   in Loop: Header=BB17_5 Depth=1
	ldrh	w8, [x1]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB17_72
// %bb.70:                              //   in Loop: Header=BB17_5 Depth=1
	strb	w26, [x0, #456]
	b	.LBB17_72
.LBB17_71:                              //   in Loop: Header=BB17_5 Depth=1
	mov	w3, #3072
	bl	fscrypt_set_bio_crypt_ctx
.LBB17_72:                              //   in Loop: Header=BB17_5 Depth=1
	mov	w2, #136
	mov	x0, x24
	mov	x1, x21
	bl	memcpy
	ldr	x0, [x28]
.LBB17_73:                              //   in Loop: Header=BB17_5 Depth=1
	mov	w2, #4096
	mov	x1, x25
	mov	w3, wzr
	bl	bio_add_page
	cmp	w0, #4095               // =4095
	b.ls	.LBB17_61
// %bb.74:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x0, [x21, #104]
	cbz	x0, .LBB17_76
// %bb.75:                              //   in Loop: Header=BB17_5 Depth=1
	mov	w2, #4096
	mov	x1, x25
	bl	wbc_account_cgroup_owner
.LBB17_76:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w8, [x21, #28]
	str	x8, [x22]
	adrp	x8, __tracepoint_f2fs_submit_page_write+8
	ldr	x25, [x21, #40]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_submit_page_write+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB17_4
// %bb.77:                              //   in Loop: Header=BB17_5 Depth=1
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
	tbz	w8, #0, .LBB17_4
// %bb.78:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x25, #24]
	cbz	x8, .LBB17_4
// %bb.79:                              //   in Loop: Header=BB17_5 Depth=1
	ldur	x9, [x29, #-16]         // 8-byte Folded Reload
	ldr	w8, [x9, #24]
	add	w8, w8, #1              // =1
	str	w8, [x9, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_submit_page_write+32
	ldr	x23, [x8, :lo12:__tracepoint_f2fs_submit_page_write+32]
	cbz	x23, .LBB17_81
.LBB17_80:                              //   Parent Loop BB17_5 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldp	x8, x0, [x23]
	mov	x1, x25
	mov	x2, x21
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB17_80
.LBB17_81:                              //   in Loop: Header=BB17_5 Depth=1
	//APP
	//NO_APP
	ldur	x9, [x29, #-16]         // 8-byte Folded Reload
	ldr	x8, [x9, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x9, #24]
	b.eq	.LBB17_83
// %bb.82:                              //   in Loop: Header=BB17_5 Depth=1
	ldur	x8, [x29, #-16]         // 8-byte Folded Reload
	ldr	x8, [x8, #24]
	cbnz	x8, .LBB17_4
.LBB17_83:                              //   in Loop: Header=BB17_5 Depth=1
	bl	preempt_schedule_notrace
	b	.LBB17_4
.LBB17_84:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x25, #24]
	cbz	x8, .LBB17_95
// %bb.85:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x23, [x8]
	ldr	x8, [x23, #40]
	ldr	x9, [x8, #1008]
	ldr	x8, [x23, #64]
	ldr	w10, [x9, #1180]
	cmp	x8, x10
	b.eq	.LBB17_2
// %bb.86:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	w9, [x9, #1176]
	cmp	x8, x9
	b.eq	.LBB17_2
// %bb.87:                              //   in Loop: Header=BB17_5 Depth=1
	ldrh	w8, [x23]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB17_2
// %bb.88:                              //   in Loop: Header=BB17_5 Depth=1
	mov	x0, x25
	bl	f2fs_is_compressed_page
	tbnz	w0, #0, .LBB17_95
// %bb.89:                              //   in Loop: Header=BB17_5 Depth=1
	ldrh	w8, [x23]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB17_92
// %bb.90:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x23, #736]
	tbnz	w8, #15, .LBB17_2
// %bb.91:                              //   in Loop: Header=BB17_5 Depth=1
	ldrb	w8, [x23, #12]
	tbnz	w8, #5, .LBB17_2
.LBB17_92:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x25]
	tbz	w8, #13, .LBB17_95
// %bb.93:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x25, #40]
	tbz	w8, #0, .LBB17_95
// %bb.94:                              //   in Loop: Header=BB17_5 Depth=1
	ldr	x8, [x25, #40]
	tbnz	w8, #3, .LBB17_2
.LBB17_95:                              //   in Loop: Header=BB17_5 Depth=1
	mov	w8, #8
	b	.LBB17_3
.LBB17_96:
	ldur	x0, [x29, #-8]          // 8-byte Folded Reload
	bl	_raw_spin_unlock
.LBB17_97:
	ldr	x8, [x19, #80]
	tbnz	w8, #6, .LBB17_100
// %bb.98:
	ldr	x8, [x19, #80]
	tbz	w8, #8, .LBB17_101
// %bb.99:
	mov	x0, x19
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB17_101
.LBB17_100:
	mov	x0, x20
	bl	__submit_merged_bio
.LBB17_101:
	ldr	x0, [sp, #8]            // 8-byte Folded Reload
	bl	up_write
	ldp	x20, x19, [sp, #128]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #112]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #96]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #80]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #64]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #144            // =144
	ret
.LBB17_102:
	bl	print_bdev_access_info
	adrp	x24, f2fs_dclient
	ldr	x0, [x24, :lo12:f2fs_dclient]
	cbz	x0, .LBB17_105
// %bb.103:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB17_105
// %bb.104:
	ldr	x0, [x24, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.f2fs_submit_page_write
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.f2fs_submit_page_write
	mov	w3, #1509
	bl	dsm_client_record
	ldr	x0, [x24, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB17_105:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 1509; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB17_1
// %bb.106:
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
	cbnz	w8, .LBB17_1
// %bb.107:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB17_1
// %bb.108:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB17_1
// %bb.109:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB17_1
.Lfunc_end17:
	.size	f2fs_submit_page_write, .Lfunc_end17-f2fs_submit_page_write
                                        // -- End function
	.p2align	2               // -- Begin function __submit_merged_bio
	.type	__submit_merged_bio,@function
__submit_merged_bio:                    // @__submit_merged_bio
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #8]
	mov	x29, sp
	cbz	x8, .LBB18_22
// %bb.1:
	ldr	w9, [x0, #36]
	mov	x19, x0
	cbnz	w9, .LBB18_6
// %bb.2:
	ldr	x9, [x19, #24]
	ldr	w10, [x19, #40]
	mov	w11, #1
	ldr	w9, [x9, #2724]
	lsl	w10, w11, w10
	and	w10, w10, #0x7
	tst	w10, w9, lsr #3
	b.eq	.LBB18_4
// %bb.3:
	ldr	w11, [x19, #48]
	orr	w11, w11, #0x1000
	str	w11, [x19, #48]
.LBB18_4:
	tst	w10, w9
	b.eq	.LBB18_6
// %bb.5:
	ldr	w9, [x19, #48]
	orr	w9, w9, #0x40000
	str	w9, [x19, #48]
.LBB18_6:
	ldp	w9, w10, [x19, #44]
	orr	w9, w10, w9
	str	w9, [x8, #16]
	ldp	x8, x20, [x19]
	ldr	w9, [x19, #44]
	ldr	w22, [x19, #36]
	ldr	x21, [x8]
	cbz	w9, .LBB18_13
// %bb.7:
	adrp	x8, __tracepoint_f2fs_prepare_write_bio+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_prepare_write_bio+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB18_21
// %bb.8:
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
	cbz	x20, .LBB18_21
// %bb.9:
	mov	w10, #1
	lsl	x8, x10, x8
	and	x8, x8, x9
	cbz	x8, .LBB18_21
// %bb.10:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w8, [x23, #24]
	add	w8, w8, #1              // =1
	str	w8, [x23, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_prepare_write_bio+32
	ldr	x24, [x8, :lo12:__tracepoint_f2fs_prepare_write_bio+32]
	cbz	x24, .LBB18_12
.LBB18_11:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x24]
	mov	x1, x21
	mov	w2, w22
	mov	x3, x20
	blr	x8
	ldr	x8, [x24, #24]!
	cbnz	x8, .LBB18_11
.LBB18_12:
	//APP
	//NO_APP
	b	.LBB18_19
.LBB18_13:
	adrp	x8, __tracepoint_f2fs_prepare_read_bio+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_prepare_read_bio+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB18_21
// %bb.14:
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
	cbz	x20, .LBB18_21
// %bb.15:
	mov	w10, #1
	lsl	x8, x10, x8
	and	x8, x8, x9
	cbz	x8, .LBB18_21
// %bb.16:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w8, [x23, #24]
	add	w8, w8, #1              // =1
	str	w8, [x23, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_prepare_read_bio+32
	ldr	x24, [x8, :lo12:__tracepoint_f2fs_prepare_read_bio+32]
	cbz	x24, .LBB18_18
.LBB18_17:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x24]
	mov	x1, x21
	mov	w2, w22
	mov	x3, x20
	blr	x8
	ldr	x8, [x24, #24]!
	cbnz	x8, .LBB18_17
.LBB18_18:
	//APP
	//NO_APP
.LBB18_19:
	ldr	x8, [x23, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x23, #24]
	b.eq	.LBB18_23
// %bb.20:
	ldr	x8, [x23, #24]
	cbz	x8, .LBB18_23
.LBB18_21:
	ldp	x0, x1, [x19]
	ldr	w2, [x19, #36]
	bl	__submit_bio
	str	xzr, [x19, #8]
.LBB18_22:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB18_23:
	bl	preempt_schedule_notrace
	b	.LBB18_21
.Lfunc_end18:
	.size	__submit_merged_bio, .Lfunc_end18-__submit_merged_bio
                                        // -- End function
	.globl	f2fs_set_data_blkaddr   // -- Begin function f2fs_set_data_blkaddr
	.p2align	2
	.type	f2fs_set_data_blkaddr,@function
f2fs_set_data_blkaddr:                  // @f2fs_set_data_blkaddr
.Lf2fs_set_data_blkaddr$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	ldr	x0, [x0, #16]
	mov	w1, #1
	mov	w2, #1
	mov	w3, #1
	mov	x29, sp
	bl	f2fs_wait_on_page_writeback
	ldr	x8, [x19, #16]
	mov	x9, #134217728
	movk	x9, #64, lsl #32
	mov	x10, #-549755813888
	mov	x11, #-549755813888
	add	x8, x9, x8, lsl #6
	movk	x10, #4072
	movk	x11, #4076
	and	x12, x8, #0xfffffffffffff000
	ldr	w8, [x12, x10]
	ldr	w9, [x12, x11]
	mov	x10, #-549755813888
	add	x10, x12, x10
	cmp	w8, w9
	b.ne	.LBB19_2
// %bb.1:
	ldr	x11, [x19]
	ldr	x12, [x11, #736]
	tbnz	w12, #26, .LBB19_3
.LBB19_2:
	mov	w11, wzr
	b	.LBB19_4
.LBB19_3:
	ldr	w11, [x11, #1184]
	asr	w11, w11, #2
.LBB19_4:
	ldr	w13, [x19, #28]
	ldr	w14, [x19, #36]
	add	x12, x10, #360          // =360
	cmp	w8, w9
	csel	x8, x12, x10, eq
	add	w9, w13, w11
	str	w14, [x8, w9, uxtw #2]
	ldr	x0, [x19, #16]
	bl	set_page_dirty
	cbz	w0, .LBB19_6
// %bb.5:
	mov	w8, #1
	strb	w8, [x19, #33]
.LBB19_6:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end19:
	.size	f2fs_set_data_blkaddr, .Lfunc_end19-f2fs_set_data_blkaddr
                                        // -- End function
	.globl	f2fs_update_data_blkaddr // -- Begin function f2fs_update_data_blkaddr
	.p2align	2
	.type	f2fs_update_data_blkaddr,@function
f2fs_update_data_blkaddr:               // @f2fs_update_data_blkaddr
.Lf2fs_update_data_blkaddr$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	ldr	x0, [x0, #16]
	str	w1, [x19, #36]
	mov	w1, #1
	mov	w2, #1
	mov	w3, #1
	mov	x29, sp
	bl	f2fs_wait_on_page_writeback
	ldr	x8, [x19, #16]
	mov	x9, #134217728
	movk	x9, #64, lsl #32
	mov	x10, #-549755813888
	mov	x11, #-549755813888
	add	x8, x9, x8, lsl #6
	movk	x10, #4072
	movk	x11, #4076
	and	x12, x8, #0xfffffffffffff000
	ldr	w8, [x12, x10]
	ldr	w9, [x12, x11]
	mov	x10, #-549755813888
	add	x10, x12, x10
	cmp	w8, w9
	b.ne	.LBB20_2
// %bb.1:
	ldr	x11, [x19]
	ldr	x12, [x11, #736]
	tbnz	w12, #26, .LBB20_3
.LBB20_2:
	mov	w11, wzr
	b	.LBB20_4
.LBB20_3:
	ldr	w11, [x11, #1184]
	asr	w11, w11, #2
.LBB20_4:
	ldr	w13, [x19, #28]
	ldr	w14, [x19, #36]
	add	x12, x10, #360          // =360
	cmp	w8, w9
	csel	x8, x12, x10, eq
	add	w9, w13, w11
	str	w14, [x8, w9, uxtw #2]
	ldr	x0, [x19, #16]
	bl	set_page_dirty
	cbz	w0, .LBB20_6
// %bb.5:
	mov	w8, #1
	strb	w8, [x19, #33]
.LBB20_6:
	mov	x0, x19
	bl	f2fs_update_extent_cache
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end20:
	.size	f2fs_update_data_blkaddr, .Lfunc_end20-f2fs_update_data_blkaddr
                                        // -- End function
	.globl	f2fs_reserve_new_blocks // -- Begin function f2fs_reserve_new_blocks
	.p2align	2
	.type	f2fs_reserve_new_blocks,@function
f2fs_reserve_new_blocks:                // @f2fs_reserve_new_blocks
.Lf2fs_reserve_new_blocks$local:
// %bb.0:
	sub	sp, sp, #96             // =96
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	str	x25, [sp, #32]          // 8-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	add	x29, sp, #16            // =16
	str	x8, [sp, #8]
	str	x1, [sp]
	cbz	x1, .LBB21_27
// %bb.1:
	ldr	x1, [x0]
	mov	x19, x0
	ldr	x8, [x1, #40]
	ldr	x0, [x8, #1008]
	ldr	x8, [x1, #736]
	tbnz	w8, #6, .LBB21_30
// %bb.2:
	mov	x2, sp
	bl	inc_valid_block_count
	cbnz	w0, .LBB21_28
// %bb.3:
	adrp	x8, __tracepoint_f2fs_reserve_new_blocks+8
	ldr	x21, [x19]
	ldp	w22, w23, [x19, #24]
	ldr	x20, [sp]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_reserve_new_blocks+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB21_9
// %bb.4:
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
	tbz	w8, #0, .LBB21_9
// %bb.5:
	//APP
	mrs x24, sp_el0
	//NO_APP
	ldr	w8, [x24, #24]
	add	w8, w8, #1              // =1
	str	w8, [x24, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_reserve_new_blocks+32
	ldr	x25, [x8, :lo12:__tracepoint_f2fs_reserve_new_blocks+32]
	cbz	x25, .LBB21_7
.LBB21_6:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x25]
	mov	x1, x21
	mov	w2, w22
	mov	w3, w23
	mov	x4, x20
	blr	x8
	ldr	x8, [x25, #24]!
	cbnz	x8, .LBB21_6
.LBB21_7:
	//APP
	//NO_APP
	ldr	x8, [x24, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x24, #24]
	b.eq	.LBB21_31
// %bb.8:
	ldr	x8, [x24, #24]
	cbz	x8, .LBB21_31
.LBB21_9:
	ldr	x0, [x19, #16]
	mov	w1, #1
	mov	w2, #1
	mov	w3, #1
	bl	f2fs_wait_on_page_writeback
	cbz	x20, .LBB21_25
// %bb.10:
	ldr	w13, [x19, #28]
	mov	x8, #134217728
	mov	x10, #-549755813888
	mov	x11, #-549755813888
	movk	x8, #64, lsl #32
	mov	x9, #-549755813888
	movk	x10, #4072
	movk	x11, #4076
	mov	w12, #-1
	b	.LBB21_14
.LBB21_11:                              //   in Loop: Header=BB21_14 Depth=1
	mov	w15, wzr
.LBB21_12:                              //   in Loop: Header=BB21_14 Depth=1
	cmp	w16, w17
	csel	x14, x0, x14, eq
	add	w13, w15, w13
	str	w12, [x14, w13, uxtw #2]
	ldr	x13, [sp]
	sub	x20, x13, #1            // =1
	str	x20, [sp]
	ldr	w13, [x19, #28]
.LBB21_13:                              //   in Loop: Header=BB21_14 Depth=1
	add	w13, w13, #1            // =1
	str	w13, [x19, #28]
	cbz	x20, .LBB21_25
.LBB21_14:                              // =>This Inner Loop Header: Depth=1
	ldr	x14, [x19, #16]
	ldr	x15, [x19]
	add	x14, x8, x14, lsl #6
	and	x14, x14, #0xfffffffffffff000
	ldr	w16, [x14, x10]
	ldr	w17, [x14, x11]
	add	x14, x14, x9
	cmp	w16, w17
	b.ne	.LBB21_19
// %bb.15:                              //   in Loop: Header=BB21_14 Depth=1
	cbz	x15, .LBB21_18
// %bb.16:                              //   in Loop: Header=BB21_14 Depth=1
	ldr	x0, [x15, #736]
	tbz	w0, #26, .LBB21_19
// %bb.17:                              //   in Loop: Header=BB21_14 Depth=1
	ldr	w0, [x15, #1184]
	asr	w1, w0, #2
	b	.LBB21_20
.LBB21_18:                              //   in Loop: Header=BB21_14 Depth=1
	ldrb	w0, [x14, #3]
	tbnz	w0, #5, .LBB21_24
.LBB21_19:                              //   in Loop: Header=BB21_14 Depth=1
	mov	w1, wzr
.LBB21_20:                              //   in Loop: Header=BB21_14 Depth=1
	add	x0, x14, #360           // =360
	cmp	w16, w17
	csel	x16, x0, x14, eq
	add	w17, w1, w13
	ldr	w16, [x16, w17, uxtw #2]
	cbnz	w16, .LBB21_13
// %bb.21:                              //   in Loop: Header=BB21_14 Depth=1
	str	w12, [x19, #36]
	ldr	w16, [x14, #4072]
	ldr	w17, [x14, #4076]
	cmp	w16, w17
	b.ne	.LBB21_11
// %bb.22:                              //   in Loop: Header=BB21_14 Depth=1
	ldr	x1, [x15, #736]
	tbz	w1, #26, .LBB21_11
// %bb.23:                              //   in Loop: Header=BB21_14 Depth=1
	ldr	w15, [x15, #1184]
	asr	w15, w15, #2
	b	.LBB21_12
.LBB21_24:                              //   in Loop: Header=BB21_14 Depth=1
	ldrh	w0, [x14, #360]
	lsr	w1, w0, #2
	b	.LBB21_20
.LBB21_25:
	ldr	x0, [x19, #16]
	bl	set_page_dirty
	cbz	w0, .LBB21_28
// %bb.26:
	mov	w0, wzr
	mov	w8, #1
	strb	w8, [x19, #33]
	b	.LBB21_28
.LBB21_27:
	mov	w0, wzr
.LBB21_28:
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB21_32
// %bb.29:
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldr	x25, [sp, #32]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #96             // =96
	ret
.LBB21_30:
	mov	w0, #-1
	b	.LBB21_28
.LBB21_31:
	bl	preempt_schedule_notrace
	b	.LBB21_9
.LBB21_32:
	bl	__stack_chk_fail
.Lfunc_end21:
	.size	f2fs_reserve_new_blocks, .Lfunc_end21-f2fs_reserve_new_blocks
                                        // -- End function
	.p2align	2               // -- Begin function inc_valid_block_count
	.type	inc_valid_block_count,@function
inc_valid_block_count:                  // @inc_valid_block_count
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x2]
	ldrb	w9, [x1, #142]
	mov	x19, x1
	mov	x20, x2
	mov	x21, x0
	lsl	x1, x8, x9
	mov	w2, #3
	mov	x0, x19
	str	x25, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	bl	__dquot_alloc_space
	add	w8, w0, #28             // =28
	cmp	w8, #28                 // =28
	b.hi	.LBB22_2
// %bb.1:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB22_3
.LBB22_2:
	cmn	w0, #122                // =122
	b.ne	.LBB22_20
.LBB22_3:
	adrp	x24, percpu_counter_batch
	ldr	x1, [x20]
	ldr	w2, [x24, :lo12:percpu_counter_batch]
	add	x22, x21, #1336         // =1336
	mov	x0, x22
	bl	percpu_counter_add_batch
	add	x23, x21, #2340         // =2340
	mov	x0, x23
	bl	_raw_spin_lock
	ldr	w8, [x20]
	ldr	w9, [x21, #1240]
	ldr	w10, [x21, #1236]
	ldr	w11, [x21, #1256]
	add	w8, w9, w8
	str	w8, [x21, #1240]
	sub	w25, w10, w11
	cbz	x19, .LBB22_11
// %bb.4:
	ldrb	w8, [x21, #1427]
	tbnz	w8, #0, .LBB22_6
.LBB22_5:
	ldr	w8, [x21, #1436]
	sub	w25, w25, w8
	b	.LBB22_11
.LBB22_6:
	ldrb	w8, [x19, #12]
	tbnz	w8, #5, .LBB22_11
// %bb.7:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldr	x8, [x8, #1984]
	ldr	w9, [x21, #1440]
	ldr	w8, [x8, #28]
	cmp	w9, w8
	b.eq	.LBB22_11
// %bb.8:
	ldr	w0, [x21, #1444]
	cbz	w0, .LBB22_10
// %bb.9:
	bl	in_group_p
	cbnz	w0, .LBB22_11
.LBB22_10:
	mov	w0, #24
	bl	capable
	tbz	w0, #0, .LBB22_5
.LBB22_11:
	ldr	x8, [x21, #80]
	tbnz	w8, #8, .LBB22_21
.LBB22_12:
	ldr	w8, [x21, #1240]
	subs	w8, w8, w25
	b.hi	.LBB22_22
// %bb.13:
	mov	x0, x23
	bl	_raw_spin_unlock
.LBB22_14:
	ldr	w8, [x20]
	ldr	x21, [x19, #736]
	ldr	x20, [x19, #736]
	ldrb	w9, [x19, #142]
	mov	x0, x19
	lsl	x1, x8, x9
	bl	dquot_claim_space_nodirty
	cbnz	w0, .LBB22_16
// %bb.15:
	mov	w1, #1
	mov	x0, x19
	bl	__mark_inode_dirty
.LBB22_16:
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_mark_inode_dirty_sync
	tbz	w21, #1, .LBB22_18
// %bb.17:
	tbz	w20, #2, .LBB22_19
.LBB22_18:
	add	x8, x19, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB22_19:
	mov	w0, wzr
.LBB22_20:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB22_21:
	ldr	w8, [x21, #1264]
	subs	w8, w25, w8
	csel	w25, wzr, w8, lo
	b	.LBB22_12
.LBB22_22:
	ldr	x9, [x20]
	mov	x0, x23
	cmp	x9, x8
	csel	x25, x9, x8, lo
	sub	x8, x9, x25
	str	x8, [x20]
	ldr	w8, [x21, #1240]
	sub	w8, w8, w25
	str	w8, [x21, #1240]
	ldr	x21, [x20]
	bl	_raw_spin_unlock
	cbz	x21, .LBB22_25
// %bb.23:
	cbz	x25, .LBB22_14
// %bb.24:
	ldr	w2, [x24, :lo12:percpu_counter_batch]
	neg	x1, x25
	mov	x0, x22
	bl	percpu_counter_add_batch
	ldrb	w8, [x19, #142]
	mov	w2, #2
	mov	x0, x19
	lsl	x1, x25, x8
	bl	__dquot_free_space
	b	.LBB22_14
.LBB22_25:
	ldr	w2, [x24, :lo12:percpu_counter_batch]
	neg	x1, x25
	mov	x0, x22
	bl	percpu_counter_add_batch
	ldrb	w8, [x19, #142]
	mov	w2, #2
	mov	x0, x19
	lsl	x1, x25, x8
	bl	__dquot_free_space
	mov	w0, #-28
	b	.LBB22_20
.Lfunc_end22:
	.size	inc_valid_block_count, .Lfunc_end22-inc_valid_block_count
                                        // -- End function
	.globl	f2fs_reserve_new_block  // -- Begin function f2fs_reserve_new_block
	.p2align	2
	.type	f2fs_reserve_new_block,@function
f2fs_reserve_new_block:                 // @f2fs_reserve_new_block
.Lf2fs_reserve_new_block$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	w20, [x0, #28]
	mov	w1, #1
	mov	x29, sp
	mov	x19, x0
	bl	f2fs_reserve_new_blocks
	str	w20, [x19, #28]
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end23:
	.size	f2fs_reserve_new_block, .Lfunc_end23-f2fs_reserve_new_block
                                        // -- End function
	.globl	f2fs_reserve_block      // -- Begin function f2fs_reserve_block
	.p2align	2
	.type	f2fs_reserve_block,@function
f2fs_reserve_block:                     // @f2fs_reserve_block
.Lf2fs_reserve_block$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x21, [x0, #8]
	mov	w2, wzr
	mov	x29, sp
	mov	x19, x0
	bl	f2fs_get_dnode_of_data
	mov	w20, w0
	cbnz	w0, .LBB24_12
// %bb.1:
	ldr	w8, [x19, #36]
	cbz	w8, .LBB24_3
// %bb.2:
	mov	w20, wzr
	cbnz	x21, .LBB24_4
	b	.LBB24_5
.LBB24_3:
	ldr	w22, [x19, #28]
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_reserve_new_blocks
	mov	w20, w0
	str	w22, [x19, #28]
	cbz	x21, .LBB24_5
.LBB24_4:
	cbz	w20, .LBB24_12
.LBB24_5:
	ldr	x0, [x19, #16]
	cbz	x0, .LBB24_7
// %bb.6:
	mov	w1, #1
	bl	f2fs_put_page
.LBB24_7:
	ldr	x8, [x19, #8]
	cbz	x8, .LBB24_11
// %bb.8:
	ldr	x9, [x19, #16]
	cmp	x9, x8
	b.eq	.LBB24_11
// %bb.9:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB24_11
// %bb.10:
	bl	__put_page
.LBB24_11:
	stp	xzr, xzr, [x19, #8]
.LBB24_12:
	mov	w0, w20
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end24:
	.size	f2fs_reserve_block, .Lfunc_end24-f2fs_reserve_block
                                        // -- End function
	.globl	f2fs_get_block          // -- Begin function f2fs_get_block
	.p2align	2
	.type	f2fs_get_block,@function
f2fs_get_block:                         // @f2fs_get_block
.Lf2fs_get_block$local:
// %bb.0:
	sub	sp, sp, #64             // =64
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	add	x29, sp, #32            // =32
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	str	wzr, [sp, #16]
	str	xzr, [sp, #8]
	mov	x19, x0
	ldr	x0, [x0]
	add	x2, sp, #8              // =8
	mov	x20, x1
	bl	f2fs_lookup_extent_cache
	tbz	w0, #0, .LBB25_2
// %bb.1:
	ldr	w8, [sp, #16]
	ldr	w9, [sp, #8]
	mov	w0, wzr
	add	w8, w8, w20
	sub	w8, w8, w9
	str	w8, [x19, #36]
	b	.LBB25_3
.LBB25_2:
	mov	x0, x19
	mov	x1, x20
	bl	f2fs_reserve_block
.LBB25_3:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB25_5
// %bb.4:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #64             // =64
	ret
.LBB25_5:
	bl	__stack_chk_fail
.Lfunc_end25:
	.size	f2fs_get_block, .Lfunc_end25-f2fs_get_block
                                        // -- End function
	.globl	f2fs_get_read_data_page // -- Begin function f2fs_get_read_data_page
	.p2align	2
	.type	f2fs_get_read_data_page,@function
f2fs_get_read_data_page:                // @f2fs_get_read_data_page
.Lf2fs_get_read_data_page$local:
// %bb.0:
	sub	sp, sp, #144            // =144
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #80]     // 16-byte Folded Spill
	add	x29, sp, #80            // =80
	str	x23, [sp, #96]          // 8-byte Folded Spill
	stp	x22, x21, [sp, #112]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #128]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	mov	x22, x0
	ldr	x0, [x0, #48]
	mov	w21, w3
	mov	w20, w2
	mov	x23, x1
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	str	wzr, [sp, #16]
	str	xzr, [sp, #8]
	tbz	w3, #0, .LBB26_6
// %bb.1:
	mov	w2, #2
	mov	x1, x23
	bl	grab_cache_page_write_begin
	mov	x19, x0
	cbz	x0, .LBB26_7
.LBB26_2:
	add	x2, sp, #8              // =8
	mov	x0, x22
	mov	x1, x23
	bl	f2fs_lookup_extent_cache
	tbz	w0, #0, .LBB26_8
// %bb.3:
	ldr	w8, [sp, #16]
	ldr	w9, [sp, #8]
	mov	w2, #8
	add	w8, w8, w23
	sub	w1, w8, w9
	str	w1, [sp, #60]
	ldr	x8, [x22, #40]
.LBB26_4:
	ldr	x0, [x8, #1008]
	bl	f2fs_is_valid_blkaddr
	tbnz	w0, #0, .LBB26_19
// %bb.5:
	mov	w23, #-117
	b	.LBB26_9
.LBB26_6:
	ldr	w3, [x0, #24]
	mov	w2, #7
	mov	x1, x23
	bl	pagecache_get_page
	mov	x19, x0
	cbnz	x0, .LBB26_2
.LBB26_7:
	mov	x19, #-12
	b	.LBB26_24
.LBB26_8:
	add	x0, sp, #24             // =24
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	add	x0, sp, #24             // =24
	mov	w2, #1
	mov	x1, x23
	stp	x22, xzr, [sp, #24]
	str	xzr, [sp, #40]
	str	wzr, [sp, #48]
	bl	f2fs_get_dnode_of_data
	mov	w23, w0
	cbz	w0, .LBB26_10
.LBB26_9:
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_put_page
	sxtw	x19, w23
	b	.LBB26_24
.LBB26_10:
	ldr	x0, [sp, #40]
	cbz	x0, .LBB26_12
// %bb.11:
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #40]
	ldr	x8, [sp, #32]
	cbnz	x8, .LBB26_13
	b	.LBB26_16
.LBB26_12:
	mov	x9, xzr
	ldr	x8, [sp, #32]
	cbz	x8, .LBB26_16
.LBB26_13:
	cmp	x9, x8
	b.eq	.LBB26_16
// %bb.14:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB26_16
// %bb.15:
	bl	__put_page
.LBB26_16:
	ldr	w1, [sp, #60]
	stp	xzr, xzr, [sp, #32]
	cmn	w1, #1                  // =1
	b.eq	.LBB26_19
// %bb.17:
	cbz	w1, .LBB26_31
// %bb.18:
	ldr	x8, [x22, #40]
	mov	w2, #7
	b	.LBB26_4
.LBB26_19:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB26_22
// %bb.20:
	ldr	w2, [sp, #60]
	cmn	w2, #1                  // =1
	b.eq	.LBB26_26
// %bb.21:
	and	w4, w21, #0x1
	mov	x0, x22
	mov	x1, x19
	mov	w3, w20
	bl	f2fs_submit_page_read
	mov	w23, w0
	cbnz	w0, .LBB26_9
	b	.LBB26_24
.LBB26_22:
	//APP
	dmb ishld
	//NO_APP
.LBB26_23:
	mov	x0, x19
	bl	unlock_page
.LBB26_24:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB26_33
// %bb.25:
	mov	x0, x19
	ldp	x20, x19, [sp, #128]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #112]    // 16-byte Folded Reload
	ldr	x23, [sp, #96]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #80]     // 16-byte Folded Reload
	add	sp, sp, #144            // =144
	ret
.LBB26_26:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	mov	x9, #134217728
	movk	x9, #64, lsl #32
	mov	w2, #4096
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	ldr	w8, [x20, #2800]
	mov	w1, wzr
	add	w8, w8, #1              // =1
	str	w8, [x20, #2800]
	//APP
	//NO_APP
	add	x8, x9, x19, lsl #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	add	x0, x8, x9
	bl	memset
	//APP
	//NO_APP
	ldr	w8, [x20, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x20, #2800]
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB26_32
// %bb.27:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB26_32
.LBB26_28:
	mov	x0, x19
	bl	flush_dcache_page
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB26_30
// %bb.29:
	//APP
	dmb ishst
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x19]
1:	ldxr	x8, [x19]
	orr	x8, x8, 4
	stxr	w9, x8, [x19]
	cbnz	w9, 1b
	//NO_APP
	b	.LBB26_23
.LBB26_30:
	//APP
	dmb ishld
	//NO_APP
	b	.LBB26_23
.LBB26_31:
	mov	w23, #-2
	b	.LBB26_9
.LBB26_32:
	bl	preempt_schedule
	b	.LBB26_28
.LBB26_33:
	bl	__stack_chk_fail
.Lfunc_end26:
	.size	f2fs_get_read_data_page, .Lfunc_end26-f2fs_get_read_data_page
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_submit_page_read
	.type	f2fs_submit_page_read,@function
f2fs_submit_page_read:                  // @f2fs_submit_page_read
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	w23, w2
	mov	x21, x1
	ldr	x20, [x8, #1008]
	ldr	x8, [x1, #32]
	and	w5, w4, #0x1
	mov	w2, #1
	mov	w1, w23
	mov	x4, x8
	mov	x29, sp
	mov	x22, x0
	bl	f2fs_grab_read_bio
	mov	x19, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB27_6
// %bb.1:
	mov	x0, x22
	mov	w1, w23
	bl	f2fs_wait_on_block_writeback
	mov	w2, #4096
	mov	x0, x19
	mov	x1, x21
	mov	w3, wzr
	bl	bio_add_page
	cmp	w0, #4095               // =4095
	b.hi	.LBB27_3
// %bb.2:
	mov	x0, x19
	bl	bio_put
	mov	w19, #-14
	b	.LBB27_6
.LBB27_3:
	ldr	x8, [x21, #8]
	mov	w9, #256
	sub	x10, x8, #1             // =1
	tst	x8, #0x1
	csel	x8, x21, x10, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	add	x8, x20, #1308          // =1308
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldrb	w8, [x20, #2704]
	cbz	w8, .LBB27_5
// %bb.4:
	add	x21, x20, #2344         // =2344
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	x8, [x20, #2480]
	mov	x0, x21
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x20, #2480]
	bl	_raw_spin_unlock
	mov	x0, x20
	bl	f2fs_record_iostat
.LBB27_5:
	mov	x0, x20
	mov	x1, x19
	mov	w2, wzr
	bl	__submit_bio
	mov	w19, wzr
.LBB27_6:
	mov	w0, w19
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end27:
	.size	f2fs_submit_page_read, .Lfunc_end27-f2fs_submit_page_read
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_put_page
	.type	f2fs_put_page,@function
f2fs_put_page:                          // @f2fs_put_page
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB28_32
// %bb.1:
	mov	x19, x0
	cbz	w1, .LBB28_30
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
	b.ne	.LBB28_24
// %bb.3:
	tbnz	w8, #0, .LBB28_29
// %bb.4:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB28_7
// %bb.5:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB28_7
// %bb.6:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.f2fs_put_page
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.f2fs_put_page
	mov	w3, #3801
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB28_7:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB28_9
// %bb.8:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB28_33
.LBB28_9:
	ldr	x0, [x19, #24]
.LBB28_10:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_raw_sb_info
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB28_12
// %bb.11:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB28_34
.LBB28_12:
	ldr	x0, [x19, #24]
.LBB28_13:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_ckpt_info
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB28_15
// %bb.14:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB28_35
.LBB28_15:
	ldr	x0, [x19, #24]
.LBB28_16:
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
	tbz	w8, #19, .LBB28_18
// %bb.17:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB28_36
.LBB28_18:
	ldr	x0, [x19, #24]
.LBB28_19:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB28_29
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
	cbnz	w9, .LBB28_29
// %bb.21:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB28_29
// %bb.22:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB28_29
// %bb.23:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB28_29
.LBB28_24:
	tbnz	w8, #0, .LBB28_29
// %bb.25:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB28_28
// %bb.26:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB28_28
// %bb.27:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.f2fs_put_page
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.f2fs_put_page
	mov	w3, #3803
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB28_28:
	mov	x0, xzr
	bl	f2fs_print_raw_sb_info
	mov	x0, xzr
	bl	f2fs_print_ckpt_info
	mov	x0, xzr
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/f2fs.h"; .popsection; .long 14472b - 14470b; .short 3803; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
.LBB28_29:
	mov	x0, x19
	bl	unlock_page
.LBB28_30:
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
	cbnz	w10, .LBB28_32
// %bb.31:
	bl	__put_page
.LBB28_32:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB28_33:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB28_10
.LBB28_34:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB28_13
.LBB28_35:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB28_16
.LBB28_36:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB28_19
.Lfunc_end28:
	.size	f2fs_put_page, .Lfunc_end28-f2fs_put_page
                                        // -- End function
	.globl	f2fs_find_data_page     // -- Begin function f2fs_find_data_page
	.p2align	2
	.type	f2fs_find_data_page,@function
f2fs_find_data_page:                    // @f2fs_find_data_page
.Lf2fs_find_data_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x20, x0
	ldr	x0, [x0, #48]
	mov	w2, wzr
	mov	w3, wzr
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x21, x1
	bl	pagecache_get_page
	cbz	x0, .LBB29_4
// %bb.1:
	ldr	x8, [x0, #8]
	mov	x19, x0
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x0, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB29_13
// %bb.2:
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
	cbnz	w10, .LBB29_4
// %bb.3:
	bl	__put_page
.LBB29_4:
	mov	x0, x20
	mov	x1, x21
	mov	w2, wzr
	mov	w3, wzr
	bl	f2fs_get_read_data_page
	mov	x19, x0
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB29_16
// %bb.5:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB29_14
// %bb.6:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #0, .LBB29_8
// %bb.7:
	ldr	x8, [x19, #8]
	mov	w1, wzr
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x19, x9, eq
	bl	wait_on_page_bit
.LBB29_8:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB29_15
// %bb.9:
	cbz	x19, .LBB29_12
// %bb.10:
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
	cbnz	w10, .LBB29_12
// %bb.11:
	bl	__put_page
.LBB29_12:
	mov	x19, #-5
	b	.LBB29_16
.LBB29_13:
	//APP
	dmb ishld
	//NO_APP
	b	.LBB29_16
.LBB29_14:
	//APP
	dmb ishld
	//NO_APP
	b	.LBB29_16
.LBB29_15:
	//APP
	dmb ishld
	//NO_APP
.LBB29_16:
	mov	x0, x19
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end29:
	.size	f2fs_find_data_page, .Lfunc_end29-f2fs_find_data_page
                                        // -- End function
	.globl	f2fs_get_lock_data_page // -- Begin function f2fs_get_lock_data_page
	.p2align	2
	.type	f2fs_get_lock_data_page,@function
f2fs_get_lock_data_page:                // @f2fs_get_lock_data_page
.Lf2fs_get_lock_data_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	str	x23, [sp, #16]          // 8-byte Folded Spill
	ldr	x23, [x0, #48]
	mov	x20, x0
	mov	x21, x1
	and	w22, w2, #0x1
	mov	x29, sp
.LBB30_1:                               // =>This Inner Loop Header: Depth=1
	mov	w2, wzr
	mov	w3, w22
	bl	f2fs_get_read_data_page
	mov	x19, x0
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB30_10
// %bb.2:                               //   in Loop: Header=BB30_1 Depth=1
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x9, [x8]
	str	x9, [x29, #24]
	ldr	x9, [x29, #24]
	tbnz	w9, #0, .LBB30_4
// %bb.3:                               //   in Loop: Header=BB30_1 Depth=1
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbz	w9, #0, .LBB30_5
.LBB30_4:                               //   in Loop: Header=BB30_1 Depth=1
	mov	x0, x19
	bl	__lock_page
.LBB30_5:                               //   in Loop: Header=BB30_1 Depth=1
	ldr	x8, [x19, #24]
	cmp	x8, x23
	b.eq	.LBB30_7
// %bb.6:                               //   in Loop: Header=BB30_1 Depth=1
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_put_page
	mov	x0, x20
	mov	x1, x21
	b	.LBB30_1
.LBB30_7:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB30_9
// %bb.8:
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_put_page
	mov	x19, #-5
	b	.LBB30_10
.LBB30_9:
	//APP
	dmb ishld
	//NO_APP
.LBB30_10:
	mov	x0, x19
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end30:
	.size	f2fs_get_lock_data_page, .Lfunc_end30-f2fs_get_lock_data_page
                                        // -- End function
	.globl	f2fs_get_new_data_page  // -- Begin function f2fs_get_new_data_page
	.p2align	2
	.type	f2fs_get_new_data_page,@function
f2fs_get_new_data_page:                 // @f2fs_get_new_data_page
.Lf2fs_get_new_data_page$local:
// %bb.0:
	sub	sp, sp, #128            // =128
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #64]     // 16-byte Folded Spill
	add	x29, sp, #64            // =64
	stp	x24, x23, [sp, #80]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #96]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #112]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	mov	x19, x0
	ldr	x0, [x0, #48]
	mov	x20, x2
	mov	x23, x1
	mov	w2, #2
	mov	x1, x20
	mov	w21, w3
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	bl	grab_cache_page_write_begin
	cbz	x0, .LBB31_3
// %bb.1:
	mov	x22, x0
	add	x0, sp, #8              // =8
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	add	x0, sp, #8              // =8
	mov	x1, x20
	stp	x19, x23, [sp, #8]
	str	xzr, [sp, #24]
	str	wzr, [sp, #32]
	bl	f2fs_reserve_block
	cbz	w0, .LBB31_4
// %bb.2:
	mov	w24, w0
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
	sxtw	x22, w24
	b	.LBB31_22
.LBB31_3:
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_put_page
	mov	x22, #-12
	b	.LBB31_22
.LBB31_4:
	cbnz	x23, .LBB31_12
// %bb.5:
	ldr	x0, [sp, #24]
	cbz	x0, .LBB31_7
// %bb.6:
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #24]
	ldr	x8, [sp, #16]
	cbnz	x8, .LBB31_8
	b	.LBB31_11
.LBB31_7:
	mov	x9, xzr
	ldr	x8, [sp, #16]
	cbz	x8, .LBB31_11
.LBB31_8:
	cmp	x9, x8
	b.eq	.LBB31_11
// %bb.9:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB31_11
// %bb.10:
	bl	__put_page
.LBB31_11:
	stp	xzr, xzr, [sp, #16]
.LBB31_12:
	ldr	x8, [x22, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x22, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB31_16
// %bb.13:
	ldr	w8, [sp, #44]
	cmn	w8, #1                  // =1
	b.eq	.LBB31_24
// %bb.14:
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
	cbnz	x23, .LBB31_30
.LBB31_15:
	mov	w2, #1
	mov	x0, x19
	mov	x1, x20
	bl	f2fs_get_lock_data_page
	mov	x22, x0
	cmn	x0, #1, lsl #12         // =4096
	b.ls	.LBB31_17
	b	.LBB31_22
.LBB31_16:
	//APP
	dmb ishld
	//NO_APP
.LBB31_17:
	tbz	w21, #0, .LBB31_22
// %bb.18:
	ldr	x9, [x19, #80]
	lsl	x8, x20, #12
	add	x8, x8, #1, lsl #12     // =4096
	cmp	x9, x8
	b.ge	.LBB31_22
// %bb.19:
	ldr	x21, [x19, #736]
	ldr	x20, [x19, #736]
	mov	w1, #1
	mov	x0, x19
	str	x8, [x19, #80]
	bl	f2fs_mark_inode_dirty_sync
	tbz	w21, #1, .LBB31_21
// %bb.20:
	tbz	w20, #2, .LBB31_22
.LBB31_21:
	add	x8, x19, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB31_22:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB31_38
// %bb.23:
	mov	x0, x22
	ldp	x20, x19, [sp, #112]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #96]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #80]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #64]     // 16-byte Folded Reload
	add	sp, sp, #128            // =128
	ret
.LBB31_24:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w8, [x23, #24]
	mov	x9, #134217728
	movk	x9, #64, lsl #32
	mov	w2, #4096
	add	w8, w8, #1              // =1
	str	w8, [x23, #24]
	//APP
	//NO_APP
	ldr	w8, [x23, #2800]
	mov	w1, wzr
	add	w8, w8, #1              // =1
	str	w8, [x23, #2800]
	//APP
	//NO_APP
	add	x8, x9, x22, lsl #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	add	x0, x8, x9
	bl	memset
	//APP
	//NO_APP
	ldr	w8, [x23, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x23, #2800]
	//APP
	//NO_APP
	ldr	x8, [x23, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x23, #24]
	b.eq	.LBB31_29
// %bb.25:
	ldr	x8, [x23, #24]
	cbz	x8, .LBB31_29
.LBB31_26:
	mov	x0, x22
	bl	flush_dcache_page
	ldr	x8, [x22, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x22, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB31_28
// %bb.27:
	//APP
	dmb ishst
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x22]
1:	ldxr	x8, [x22]
	orr	x8, x8, 4
	stxr	w9, x8, [x22]
	cbnz	w9, 1b
	//NO_APP
	b	.LBB31_17
.LBB31_28:
	//APP
	dmb ishld
	//NO_APP
	b	.LBB31_17
.LBB31_29:
	bl	preempt_schedule
	b	.LBB31_26
.LBB31_30:
	bl	print_bdev_access_info
	adrp	x22, f2fs_dclient
	ldr	x0, [x22, :lo12:f2fs_dclient]
	cbz	x0, .LBB31_33
// %bb.31:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB31_33
// %bb.32:
	ldr	x0, [x22, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.f2fs_get_new_data_page
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.f2fs_get_new_data_page
	mov	w3, #2148
	bl	dsm_client_record
	ldr	x0, [x22, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB31_33:
	ldr	x8, [x19, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_raw_sb_info
	ldr	x8, [x19, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_ckpt_info
	ldr	x8, [x19, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 2148; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB31_15
// %bb.34:
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
	cbnz	w9, .LBB31_15
// %bb.35:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB31_15
// %bb.36:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB31_15
// %bb.37:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB31_15
.LBB31_38:
	bl	__stack_chk_fail
.Lfunc_end31:
	.size	f2fs_get_new_data_page, .Lfunc_end31-f2fs_get_new_data_page
                                        // -- End function
	.globl	f2fs_preallocate_blocks // -- Begin function f2fs_preallocate_blocks
	.p2align	2
	.type	f2fs_preallocate_blocks,@function
f2fs_preallocate_blocks:                // @f2fs_preallocate_blocks
.Lf2fs_preallocate_blocks$local:
// %bb.0:
	sub	sp, sp, #112            // =112
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	add	x29, sp, #48            // =48
	stp	x24, x23, [sp, #64]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #80]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #96]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x8, [x0]
	mov	w11, #8
	ldr	x19, [x8, #32]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	str	xzr, [sp]
	ldr	x8, [x0, #8]
	ldr	w22, [x0, #32]
	add	x9, x8, #4095           // =4095
	lsr	x9, x9, #12
	str	w9, [sp, #4]
	ldr	x10, [x1, #16]
	str	w11, [sp, #32]
	stp	xzr, xzr, [sp, #16]
	add	x8, x10, x8
	lsr	x10, x8, #12
	subs	w9, w10, w9
	mov	w10, #1
	csel	w9, wzr, w9, lo
	str	w9, [sp, #8]
	strb	w10, [sp, #36]
	tbz	w22, #17, .LBB32_7
// %bb.1:
	mov	x21, x0
	ldrh	w0, [x0, #36]
	mov	x20, x1
	bl	f2fs_rw_hint_to_seg_type
	str	w0, [sp, #32]
	ldr	x8, [x19, #40]
	ldr	w24, [x20]
	mov	x0, x21
	mov	x1, x20
	ldr	x23, [x8, #1008]
	bl	fscrypt_dio_supported
	tbz	w0, #0, .LBB32_10
// %bb.2:
	ldr	x8, [x19, #664]
	cbnz	x8, .LBB32_10
// %bb.3:
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	ldr	w9, [x8, #1532]
	cbz	w9, .LBB32_17
// %bb.4:
	ldr	w9, [x19, #1280]
	cbnz	w9, .LBB32_10
// %bb.5:
	ldr	x9, [x19, #736]
	tbnz	x9, #35, .LBB32_10
// %bb.6:
	ldr	x9, [x19, #736]
	tbnz	x9, #34, .LBB32_10
	b	.LBB32_19
.LBB32_7:
	ldr	w9, [x19, #1192]
	ldr	w10, [x19, #1184]
	mvn	w9, w9
	sub	w9, w9, w10, asr #2
	add	w9, w9, #923            // =923
	cmp	x8, w9, sxtw #2
	b.ls	.LBB32_9
// %bb.8:
	mov	x0, x19
	bl	f2fs_convert_inline_inode
	cbnz	w0, .LBB32_15
.LBB32_9:
	ldr	x8, [x19, #736]
	tbnz	w8, #10, .LBB32_14
.LBB32_10:
	mov	w3, #5
.LBB32_11:
	mov	x1, sp
	mov	w2, #1
	mov	x0, x19
	bl	f2fs_map_blocks
	ldr	w8, [sp, #8]
	cmp	w8, #0                  // =0
	cset	w8, eq
	cmn	w0, #28                 // =28
	cset	w9, ne
	orr	w8, w9, w8
	cmp	w8, #0                  // =0
	csel	w0, w0, wzr, ne
	tbnz	w22, #17, .LBB32_15
// %bb.12:
	tbnz	w8, #0, .LBB32_15
// %bb.13:
	add	x8, x19, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 16777216
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB32_14:
	mov	w0, wzr
.LBB32_15:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB32_27
// %bb.16:
	ldp	x20, x19, [sp, #96]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #80]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #64]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #112            // =112
	ret
.LBB32_17:
	ldrh	w9, [x19]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB32_19
// %bb.18:
	ldr	x9, [x19, #736]
	tbnz	w9, #31, .LBB32_10
.LBB32_19:
	ldr	x9, [x23, #16]
	ldrb	w9, [x9, #2180]
	tbnz	w9, #1, .LBB32_10
// %bb.20:
	tbz	w24, #0, .LBB32_25
// %bb.21:
	ldr	w9, [x23, #1504]
	cmp	w9, #1                  // =1
	b.ne	.LBB32_25
// %bb.22:
	ldrb	w8, [x19, #142]
	ldr	w21, [x21, #8]
	mov	w9, #-1
	mov	x0, x20
	lsl	w24, w9, w8
	bl	iov_iter_alignment
	orr	w8, w0, w21
	bics	wzr, w8, w24
	b.ne	.LBB32_10
// %bb.23:
	ldr	w8, [x23, #1432]
	mov	w9, #1
	lsl	w8, w9, w8
	cmp	w8, #1                  // =1
	b.gt	.LBB32_10
// %bb.24:
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
.LBB32_25:
	ldr	x8, [x8, #80]
	tbnz	w8, #8, .LBB32_10
// %bb.26:
	mov	w3, #4
	b	.LBB32_11
.LBB32_27:
	bl	__stack_chk_fail
.Lfunc_end32:
	.size	f2fs_preallocate_blocks, .Lfunc_end32-f2fs_preallocate_blocks
                                        // -- End function
	.globl	f2fs_map_blocks         // -- Begin function f2fs_map_blocks
	.p2align	2
	.type	f2fs_map_blocks,@function
f2fs_map_blocks:                        // @f2fs_map_blocks
.Lf2fs_map_blocks$local:
// %bb.0:
	sub	sp, sp, #256            // =256
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #160]    // 16-byte Folded Spill
	add	x29, sp, #160           // =160
	stp	x28, x27, [sp, #176]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #192]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #208]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #224]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #240]    // 16-byte Folded Spill
	stur	w2, [x29, #-76]         // 4-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	w25, [x1, #8]
	stp	xzr, xzr, [x29, #-24]
	stp	xzr, xzr, [x29, #-40]
	stp	xzr, xzr, [x29, #-56]
	ldr	x8, [x0, #40]
	ldrb	w27, [x1, #36]
	ldr	x21, [x8, #1008]
	stur	wzr, [x29, #-64]
	stur	xzr, [x29, #-72]
	cbz	w25, .LBB33_4
// %bb.1:
	mov	w22, w3
	mov	x19, x1
	mov	x20, x0
	cbz	w3, .LBB33_3
// %bb.2:
	ldr	x8, [x20, #736]
	tbnz	x8, #38, .LBB33_5
.LBB33_3:
	mov	x26, xzr
	mov	x24, xzr
	b	.LBB33_22
.LBB33_4:
	mov	w28, wzr
	b	.LBB33_186
.LBB33_5:
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB33_7
// %bb.6:
	ldr	x8, [x20, #736]
	tbnz	x8, #39, .LBB33_3
.LBB33_7:
	ldur	w8, [x29, #-76]         // 4-byte Folded Reload
	cbz	w8, .LBB33_16
// %bb.8:
	ldr	x2, [x20, #64]
	adrp	x1, .L.str.1
	add	x1, x1, :lo12:.L.str.1
	mov	x0, x21
	bl	f2fs_printk
	bl	print_bdev_access_info
	adrp	x23, f2fs_dclient
	ldr	x0, [x23, :lo12:f2fs_dclient]
	cbz	x0, .LBB33_11
// %bb.9:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB33_11
// %bb.10:
	ldr	x0, [x23, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.f2fs_map_blocks
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.f2fs_map_blocks
	mov	w3, #2483
	bl	dsm_client_record
	ldr	x0, [x23, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB33_11:
	mov	x0, x21
	bl	f2fs_print_raw_sb_info
	mov	x0, x21
	bl	f2fs_print_ckpt_info
	mov	x0, x21
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 2483; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x21, .LBB33_16
// %bb.12:
	add	x8, x21, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x21, #3376]
	cbnz	w8, .LBB33_16
// %bb.13:
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB33_16
// %bb.14:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB33_16
// %bb.15:
	add	x2, x21, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB33_16:
	add	x24, x20, #744          // =744
	mov	x0, x24
	bl	down_read
	ldr	x8, [x20, #736]
	tbnz	x8, #38, .LBB33_18
.LBB33_17:
	mov	x26, xzr
	b	.LBB33_21
.LBB33_18:
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB33_20
// %bb.19:
	ldr	x8, [x20, #736]
	tbnz	x8, #39, .LBB33_17
.LBB33_20:
	ldr	x0, [x20, #1304]
	bl	igrab
	mov	x26, x0
.LBB33_21:
	mov	x0, x24
	bl	up_read
	cmp	x26, #0                 // =0
	csel	x24, xzr, x20, eq
	csel	x20, x20, x26, eq
.LBB33_22:
	cmp	w25, #31                // =31
	cset	w8, gt
	cmp	w25, #255               // =255
	mov	w9, #2
	csel	w8, w9, w8, gt
	ldr	w23, [x19, #4]
	str	w8, [sp, #68]           // 4-byte Folded Spill
	ldur	w8, [x29, #-76]         // 4-byte Folded Reload
	str	xzr, [x19, #8]
	stp	x26, x23, [sp, #32]     // 16-byte Folded Spill
	str	x24, [sp, #24]          // 8-byte Folded Spill
	str	w27, [sp, #8]           // 4-byte Folded Spill
	cbnz	w8, .LBB33_28
// %bb.23:
	sub	x2, x29, #72            // =72
	mov	x0, x20
	mov	x1, x23
	bl	f2fs_lookup_extent_cache
	tbz	w0, #0, .LBB33_28
// %bb.24:
	cmp	w22, #3                 // =3
	b.ne	.LBB33_30
// %bb.25:
	ldr	w8, [x21, #1504]
	cmp	w8, #1                  // =1
	b.ne	.LBB33_30
// %bb.26:
	ldrb	w8, [x19, #36]
	cbz	w8, .LBB33_30
// %bb.27:
	mov	w8, #1
	str	x25, [sp, #16]          // 8-byte Folded Spill
	str	wzr, [sp, #64]          // 4-byte Folded Spill
	str	w8, [sp, #80]           // 4-byte Folded Spill
	b	.LBB33_36
.LBB33_28:
	mov	w8, wzr
	mov	w9, #1
	str	wzr, [sp, #64]          // 4-byte Folded Spill
	str	w9, [sp, #80]           // 4-byte Folded Spill
.LBB33_29:
	str	w8, [sp, #12]           // 4-byte Folded Spill
	ldrb	w8, [x19, #36]
	str	x25, [sp, #16]          // 8-byte Folded Spill
	b	.LBB33_35
.LBB33_30:
	ldp	w10, w8, [x29, #-68]
	ldur	w9, [x29, #-72]
	mov	w11, #32
	add	w8, w8, w23
	sub	w8, w8, w9
	str	w8, [x19]
	ldr	x8, [x19, #24]
	add	w9, w10, w9
	sub	x9, x9, x23
	cmp	x9, x25
	csel	x9, x25, x9, hi
	stp	w9, w11, [x19, #8]
	cbz	x8, .LBB33_32
// %bb.31:
	add	x9, x9, x23
	str	x9, [x8]
.LBB33_32:
	cmp	w22, #3                 // =3
	b.ne	.LBB33_34
// %bb.33:
	ldr	w1, [x19]
	ldr	w2, [x19, #8]
	mov	x0, x20
	bl	f2fs_wait_on_block_writeback_range
	ldr	x8, [x21, #248]
	ldr	w1, [x19]
	ldr	w9, [x19, #8]
	ldr	x0, [x8, #48]
	add	w8, w1, w9
	sub	w2, w8, #1              // =1
	bl	invalidate_mapping_pages
.LBB33_34:
	mov	w28, wzr
	b	.LBB33_172
.LBB33_35:
	tst	w8, #0xff
	mov	x25, x23
	b.eq	.LBB33_37
.LBB33_36:
	mov	w2, #1
	mov	x0, x21
	mov	w1, w22
	mov	w3, wzr
	bl	f2fs_do_map_lock
	mov	x25, x23
	str	w0, [sp, #12]           // 4-byte Folded Spill
.LBB33_37:
	sub	x0, x29, #56            // =56
	mov	w2, #48
	mov	w1, wzr
	eor	w27, w27, #0x1
	bl	memset
	sub	x0, x29, #56            // =56
	mov	x1, x25
	mov	w2, w27
	stp	x20, xzr, [x29, #-56]
	stur	xzr, [x29, #-40]
	stur	wzr, [x29, #-32]
	bl	f2fs_get_dnode_of_data
	cbnz	w0, .LBB33_188
// %bb.38:
	ldur	x9, [x29, #-40]
	mov	x11, #2097152
	movk	x11, #1, lsl #32
	mov	x8, #-549755813888
	add	x11, x9, x11
	mov	x10, #-549755813888
	lsl	x11, x11, #6
	movk	x8, #4072
	movk	x10, #4076
	and	x11, x11, #0xfffffffffffff000
	ldr	w8, [x11, x8]
	ldr	w10, [x11, x10]
	ldur	w13, [x29, #-28]
	str	x25, [sp, #56]          // 8-byte Folded Spill
	cmp	w8, w10
	b.ne	.LBB33_42
// %bb.39:
	ldr	w8, [x20, #1184]
	ldr	w10, [x20, #1192]
	ldrh	w11, [x20]
	mov	w12, #923
	add	w8, w10, w8, asr #2
	and	w10, w11, #0xf000
	cmp	w10, #8, lsl #12        // =32768
	ldr	x10, [sp, #40]          // 8-byte Folded Reload
	sub	w14, w12, w8
	b.ne	.LBB33_46
// %bb.40:
	ldr	x8, [x20, #736]
	tbz	w8, #31, .LBB33_46
// %bb.41:
	ldr	w8, [x20, #1292]
	neg	w8, w8
	and	w14, w14, w8
	b	.LBB33_46
.LBB33_42:
	ldrh	w8, [x20]
	ldr	x10, [sp, #40]          // 8-byte Folded Reload
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB33_45
// %bb.43:
	ldr	x8, [x20, #736]
	mov	w14, #1018
	tbz	w8, #31, .LBB33_46
// %bb.44:
	ldr	w8, [x20, #1292]
	neg	w8, w8
	and	w14, w8, w14
	b	.LBB33_46
.LBB33_45:
	mov	w14, #1018
.LBB33_46:
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	ldr	x23, [sp, #56]          // 8-byte Folded Reload
	mov	x26, xzr
	sub	w24, w22, #3            // =3
	add	x27, x10, x8
	mov	x25, #-549755813888
	mov	w8, w13
	str	w13, [sp, #52]          // 4-byte Folded Spill
	stp	w14, w13, [sp, #72]     // 8-byte Folded Spill
.LBB33_47:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB33_75 Depth 2
                                        //     Child Loop BB33_85 Depth 2
                                        //     Child Loop BB33_109 Depth 2
	mov	x10, #2097152
	movk	x10, #1, lsl #32
	add	x9, x9, x10
	lsl	x9, x9, #6
	and	x11, x9, #0xfffffffffffff000
	mov	x9, #-549755813888
	mov	x10, #-549755813888
	movk	x9, #4072
	movk	x10, #4076
	ldr	w9, [x11, x9]
	ldr	w10, [x11, x10]
	add	x11, x11, x25
	cmp	w9, w10
	b.ne	.LBB33_52
// %bb.48:                              //   in Loop: Header=BB33_47 Depth=1
	ldur	x12, [x29, #-56]
	cbz	x12, .LBB33_51
// %bb.49:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	x13, [x12, #736]
	tbz	w13, #26, .LBB33_52
// %bb.50:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w12, [x12, #1184]
	asr	w12, w12, #2
	b	.LBB33_53
.LBB33_51:                              //   in Loop: Header=BB33_47 Depth=1
	ldrb	w12, [x11, #3]
	tbnz	w12, #5, .LBB33_123
.LBB33_52:                              //   in Loop: Header=BB33_47 Depth=1
	mov	w12, wzr
.LBB33_53:                              //   in Loop: Header=BB33_47 Depth=1
	add	x13, x11, #360          // =360
	cmp	w9, w10
	csel	x9, x13, x11, eq
	add	w8, w12, w8
	ldr	w28, [x9, w8, uxtw #2]
	add	w8, w28, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB33_71
.LBB33_54:                              //   in Loop: Header=BB33_47 Depth=1
	ldur	w8, [x29, #-76]         // 4-byte Folded Reload
	cbz	w8, .LBB33_58
// %bb.55:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	x8, [x21, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB33_196
// %bb.56:                              //   in Loop: Header=BB33_47 Depth=1
	cmp	w22, #5                 // =5
	b.ne	.LBB33_63
// %bb.57:                              //   in Loop: Header=BB33_47 Depth=1
	ldur	w8, [x29, #-28]
	ldr	w9, [sp, #76]           // 4-byte Folded Reload
	cmp	w28, #0                 // =0
	cinc	x26, x26, eq
	csel	w9, w8, w9, eq
	str	w9, [sp, #76]           // 4-byte Folded Spill
	b	.LBB33_66
.LBB33_58:                              //   in Loop: Header=BB33_47 Depth=1
	cmp	w22, #6                 // =6
	b.eq	.LBB33_149
// %bb.59:                              //   in Loop: Header=BB33_47 Depth=1
	cmp	w22, #2                 // =2
	b.eq	.LBB33_154
// %bb.60:                              //   in Loop: Header=BB33_47 Depth=1
	cmp	w22, #1                 // =1
	b.ne	.LBB33_62
// %bb.61:                              //   in Loop: Header=BB33_47 Depth=1
	cbz	w28, .LBB33_130
.LBB33_62:                              //   in Loop: Header=BB33_47 Depth=1
	cmp	w22, #1                 // =1
	b.eq	.LBB33_94
	b	.LBB33_131
.LBB33_63:                              //   in Loop: Header=BB33_47 Depth=1
	cmp	w24, #2                 // =2
	b.hs	.LBB33_124
.LBB33_64:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w1, [x19, #32]
	ldr	w2, [sp, #68]           // 4-byte Folded Reload
	sub	x0, x29, #56            // =56
	bl	__allocate_data_block
	cbnz	w0, .LBB33_148
// %bb.65:                              //   in Loop: Header=BB33_47 Depth=1
	add	x8, x20, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4096
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	w14, [sp, #72]          // 4-byte Folded Reload
.LBB33_66:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w8, [x19, #12]
	orr	w8, w8, #0x40
	str	w8, [x19, #12]
	ldur	w28, [x29, #-20]
.LBB33_67:                              //   in Loop: Header=BB33_47 Depth=1
	cmp	w22, #5                 // =5
	b.ne	.LBB33_94
// %bb.68:                              //   in Loop: Header=BB33_47 Depth=1
	ldur	w8, [x29, #-28]
	add	x23, x23, #1            // =1
	cmp	x23, x27
	add	w8, w8, #1              // =1
	stur	w8, [x29, #-28]
	b.eq	.LBB33_111
// %bb.69:                              //   in Loop: Header=BB33_47 Depth=1
	cmp	w8, w14
	b.eq	.LBB33_111
// %bb.70:                              //   in Loop: Header=BB33_47 Depth=1
	mov	w28, #1
	b	.LBB33_118
.LBB33_71:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w9, [x21, #3128]
	cbz	w9, .LBB33_79
// %bb.72:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	x8, [x21]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB33_78
// %bb.73:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	x10, [x21, #3136]
	add	x10, x10, #80           // =80
	b	.LBB33_75
.LBB33_74:                              //   in Loop: Header=BB33_75 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB33_78
.LBB33_75:                              //   Parent Loop BB33_47 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	w11, w28
	b.hi	.LBB33_74
// %bb.76:                              //   in Loop: Header=BB33_75 Depth=2
	ldr	w11, [x10]
	cmp	w11, w28
	b.lo	.LBB33_74
// %bb.77:                              //   in Loop: Header=BB33_47 Depth=1
	ldur	x8, [x10, #-80]
.LBB33_78:                              //   in Loop: Header=BB33_47 Depth=1
	cbz	x8, .LBB33_80
.LBB33_79:                              //   in Loop: Header=BB33_47 Depth=1
	mov	w2, #6
	mov	x0, x21
	mov	w1, w28
	bl	f2fs_is_valid_blkaddr
	ldr	w14, [sp, #72]          // 4-byte Folded Reload
	tbz	w0, #0, .LBB33_155
.LBB33_80:                              //   in Loop: Header=BB33_47 Depth=1
	add	w8, w28, #3             // =3
	cmp	w8, #4                  // =4
	b.lo	.LBB33_54
// %bb.81:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w9, [x21, #3128]
	cbz	w9, .LBB33_89
// %bb.82:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	x8, [x21]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB33_88
// %bb.83:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	x10, [x21, #3136]
	add	x10, x10, #80           // =80
	b	.LBB33_85
.LBB33_84:                              //   in Loop: Header=BB33_85 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB33_88
.LBB33_85:                              //   Parent Loop BB33_47 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	w11, w28
	b.hi	.LBB33_84
// %bb.86:                              //   in Loop: Header=BB33_85 Depth=2
	ldr	w11, [x10]
	cmp	w11, w28
	b.lo	.LBB33_84
// %bb.87:                              //   in Loop: Header=BB33_47 Depth=1
	ldur	x8, [x10, #-80]
.LBB33_88:                              //   in Loop: Header=BB33_47 Depth=1
	cbz	x8, .LBB33_54
.LBB33_89:                              //   in Loop: Header=BB33_47 Depth=1
	cmp	w22, #3                 // =3
	b.ne	.LBB33_67
// %bb.90:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w8, [x21, #1504]
	cmp	w8, #1                  // =1
	b.ne	.LBB33_67
// %bb.91:                              //   in Loop: Header=BB33_47 Depth=1
	ldrb	w8, [x19, #36]
	cbz	w8, .LBB33_94
// %bb.92:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w1, [x19, #32]
	ldr	w2, [sp, #68]           // 4-byte Folded Reload
	sub	x0, x29, #56            // =56
	bl	__allocate_data_block
	cbnz	w0, .LBB33_197
// %bb.93:                              //   in Loop: Header=BB33_47 Depth=1
	ldur	w28, [x29, #-20]
	add	x8, x20, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4096
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	w14, [sp, #72]          // 4-byte Folded Reload
.LBB33_94:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w8, [x19, #8]
	cbz	w8, .LBB33_102
// %bb.95:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w9, [x19]
	cmn	w9, #1                  // =1
	b.eq	.LBB33_99
// %bb.96:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w10, [sp, #80]          // 4-byte Folded Reload
	add	w10, w9, w10
	cmp	w28, w10
	b.ne	.LBB33_99
// %bb.97:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w10, [sp, #64]          // 4-byte Folded Reload
	cmp	w28, w10
	b.ls	.LBB33_101
// %bb.98:                              //   in Loop: Header=BB33_47 Depth=1
	ldr	w10, [x21, #3128]
	cbz	w10, .LBB33_101
.LBB33_99:                              //   in Loop: Header=BB33_47 Depth=1
	cmp	w22, #4                 // =4
	b.eq	.LBB33_101
// %bb.100:                             //   in Loop: Header=BB33_47 Depth=1
	and	w9, w9, w28
	cmn	w9, #1                  // =1
	b.ne	.LBB33_133
.LBB33_101:                             //   in Loop: Header=BB33_47 Depth=1
	add	w8, w8, #1              // =1
	ldr	w9, [sp, #80]           // 4-byte Folded Reload
	str	w8, [x19, #8]
	ldur	w8, [x29, #-28]
	mov	w28, wzr
	add	w9, w9, #1              // =1
	str	w9, [sp, #80]           // 4-byte Folded Spill
	b	.LBB33_116
.LBB33_102:                             //   in Loop: Header=BB33_47 Depth=1
	ldr	w8, [x19, #12]
	cmn	w28, #1                 // =1
	b.ne	.LBB33_104
// %bb.103:                             //   in Loop: Header=BB33_47 Depth=1
	orr	w8, w8, #0x1000
	str	w8, [x19, #12]
.LBB33_104:                             //   in Loop: Header=BB33_47 Depth=1
	orr	w8, w8, #0x20
	add	w9, w28, #1             // =1
	cmp	w9, #2                  // =2
	str	w8, [x19, #12]
	mov	w8, #1
	str	w28, [x19]
	str	w8, [x19, #8]
	b.lo	.LBB33_115
// %bb.105:                             //   in Loop: Header=BB33_47 Depth=1
	ldr	w10, [x21, #3128]
	cbz	w10, .LBB33_115
// %bb.106:                             //   in Loop: Header=BB33_47 Depth=1
	ldr	x8, [x21, #3136]
	cmp	w10, #2                 // =2
	b.lt	.LBB33_121
// %bb.107:                             //   in Loop: Header=BB33_47 Depth=1
	mov	x9, xzr
	add	x11, x8, #80            // =80
	b	.LBB33_109
.LBB33_108:                             //   in Loop: Header=BB33_109 Depth=2
	add	x9, x9, #1              // =1
	cmp	x10, x9
	add	x11, x11, #88           // =88
	b.eq	.LBB33_121
.LBB33_109:                             //   Parent Loop BB33_47 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w12, [x11, #-4]
	cmp	w12, w28
	b.hi	.LBB33_108
// %bb.110:                             //   in Loop: Header=BB33_109 Depth=2
	ldr	w12, [x11]
	cmp	w12, w28
	b.lo	.LBB33_108
	b	.LBB33_122
.LBB33_111:                             //   in Loop: Header=BB33_47 Depth=1
	mov	x25, x20
	mov	x20, x21
	mov	w21, w24
	ldr	w24, [sp, #52]          // 4-byte Folded Reload
	sub	x0, x29, #56            // =56
	mov	x1, x26
	stur	w24, [x29, #-28]
	bl	f2fs_reserve_new_blocks
	cbnz	w0, .LBB33_156
// %bb.112:                             //   in Loop: Header=BB33_47 Depth=1
	ldur	w8, [x29, #-28]
	ldr	w9, [x19, #8]
	sub	w10, w8, w24
	add	w9, w10, w9
	mov	x10, x26
	str	w9, [x19, #8]
	cbz	x26, .LBB33_114
// %bb.113:                             //   in Loop: Header=BB33_47 Depth=1
	ldr	w9, [sp, #76]           // 4-byte Folded Reload
	add	w9, w9, #1              // =1
	cmp	w8, w9
	b.ne	.LBB33_157
.LBB33_114:                             //   in Loop: Header=BB33_47 Depth=1
	ldr	w14, [sp, #72]          // 4-byte Folded Reload
	mov	w28, #1
	mov	w24, w21
	mov	x21, x20
	mov	w8, w14
	mov	x20, x25
	mov	x26, x10
	mov	x25, #-549755813888
	stur	w14, [x29, #-28]
	b	.LBB33_118
.LBB33_115:                             //   in Loop: Header=BB33_47 Depth=1
	ldur	w8, [x29, #-28]
	mov	w28, wzr
.LBB33_116:                             //   in Loop: Header=BB33_47 Depth=1
	add	w8, w8, #1              // =1
.LBB33_117:                             //   in Loop: Header=BB33_47 Depth=1
	stur	w8, [x29, #-28]
	add	x23, x23, #1            // =1
.LBB33_118:                             //   in Loop: Header=BB33_47 Depth=1
	cmp	x23, x27
	b.hs	.LBB33_133
// %bb.119:                             //   in Loop: Header=BB33_47 Depth=1
	cmp	w8, w14
	b.hs	.LBB33_125
// %bb.120:                             //   in Loop: Header=BB33_47 Depth=1
	ldur	x9, [x29, #-40]
	b	.LBB33_47
.LBB33_121:                             //   in Loop: Header=BB33_47 Depth=1
	mov	x9, xzr
.LBB33_122:                             //   in Loop: Header=BB33_47 Depth=1
	mov	w11, #88
	smaddl	x8, w9, w11, x8
	ldur	w10, [x29, #-28]
	ldr	w8, [x8, #80]
	mov	w28, wzr
	str	w8, [sp, #64]           // 4-byte Folded Spill
	add	w8, w10, #1             // =1
	b	.LBB33_117
.LBB33_123:                             //   in Loop: Header=BB33_47 Depth=1
	ldrh	w12, [x11, #360]
	lsr	w12, w12, #2
	b	.LBB33_53
.LBB33_124:                             //   in Loop: Header=BB33_47 Depth=1
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 2593; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	b	.LBB33_64
.LBB33_125:
	cmp	w22, #6                 // =6
	b.ne	.LBB33_128
// %bb.126:
	ldrb	w8, [x19, #12]
	tbz	w8, #5, .LBB33_128
// %bb.127:
	ldp	w9, w8, [x19]
	ldr	x11, [sp, #56]          // 8-byte Folded Reload
	ldr	w10, [x19, #8]
	sub	x0, x29, #56            // =56
	sub	w8, w11, w8
	and	x1, x11, #0xffffffff
	add	w2, w9, w8
	sub	w3, w10, w8
	bl	f2fs_update_extent_cache_range
.LBB33_128:
	ldur	x0, [x29, #-40]
	ldp	x24, x26, [sp, #24]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldr	w27, [sp, #8]           // 4-byte Folded Reload
	cbz	x0, .LBB33_138
// %bb.129:
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	ldur	x8, [x29, #-48]
	cbnz	x8, .LBB33_139
	b	.LBB33_142
.LBB33_130:
	ldr	x8, [x19, #16]
	ldp	x24, x26, [sp, #24]     // 16-byte Folded Reload
	mov	w28, wzr
	cbnz	x8, .LBB33_153
	b	.LBB33_159
.LBB33_131:
	ldr	x8, [x19, #16]
	cbz	x8, .LBB33_133
// %bb.132:
	add	x9, x23, #1             // =1
	mov	w28, wzr
	str	x9, [x8]
	b	.LBB33_134
.LBB33_133:
	mov	w28, wzr
.LBB33_134:
	ldp	x24, x26, [sp, #24]     // 16-byte Folded Reload
	ldr	x11, [sp, #56]          // 8-byte Folded Reload
	cmp	w22, #6                 // =6
	b.eq	.LBB33_150
// %bb.135:
	cmp	w22, #3                 // =3
	b.ne	.LBB33_159
// %bb.136:
	ldrb	w8, [x19, #12]
	tbz	w8, #5, .LBB33_159
.LBB33_137:
	ldr	w1, [x19]
	ldr	w2, [x19, #8]
	mov	x0, x20
	bl	f2fs_wait_on_block_writeback_range
	ldr	x8, [x21, #248]
	ldr	w1, [x19]
	ldr	w9, [x19, #8]
	ldr	x0, [x8, #48]
	add	w8, w1, w9
	sub	w2, w8, #1              // =1
	bl	invalidate_mapping_pages
	b	.LBB33_159
.LBB33_138:
	mov	x9, xzr
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB33_142
.LBB33_139:
	cmp	x9, x8
	b.eq	.LBB33_142
// %bb.140:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB33_142
// %bb.141:
	bl	__put_page
.LBB33_142:
	stp	xzr, xzr, [x29, #-48]
	ldrb	w8, [x19, #36]
	cbz	w8, .LBB33_35
// %bb.143:
	cbz	w28, .LBB33_146
// %bb.144:
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	tbz	w8, #0, .LBB33_146
// %bb.145:
	add	x0, x21, #424           // =424
	b	.LBB33_147
.LBB33_146:
	add	x0, x21, #304           // =304
.LBB33_147:
	bl	up_read
	ldurb	w1, [x29, #-23]
	mov	x0, x21
	bl	f2fs_balance_fs
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	b	.LBB33_29
.LBB33_148:
	mov	w28, w0
	b	.LBB33_134
.LBB33_149:
	ldp	x24, x26, [sp, #24]     // 16-byte Folded Reload
	ldr	x11, [sp, #56]          // 8-byte Folded Reload
	mov	w28, wzr
.LBB33_150:
	ldrb	w8, [x19, #12]
	tbz	w8, #5, .LBB33_152
// %bb.151:
	ldp	w9, w8, [x19]
	ldr	w10, [x19, #8]
	and	x1, x11, #0xffffffff
	sub	x0, x29, #56            // =56
	sub	w8, w11, w8
	add	w2, w9, w8
	sub	w3, w10, w8
	bl	f2fs_update_extent_cache_range
.LBB33_152:
	ldr	x8, [x19, #24]
	cbz	x8, .LBB33_159
.LBB33_153:
	add	x9, x23, #1             // =1
	str	x9, [x8]
	b	.LBB33_159
.LBB33_154:
	str	wzr, [x19]
	ldp	x24, x26, [sp, #24]     // 16-byte Folded Reload
	mov	w28, wzr
	b	.LBB33_159
.LBB33_155:
	mov	w28, #-117
	b	.LBB33_134
.LBB33_156:
	mov	w28, w0
	b	.LBB33_158
.LBB33_157:
	mov	w28, #-28
.LBB33_158:
	ldp	x24, x26, [sp, #24]     // 16-byte Folded Reload
	mov	x21, x20
	mov	x20, x25
.LBB33_159:
	ldur	x0, [x29, #-40]
	cbz	x0, .LBB33_161
// %bb.160:
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	ldur	x8, [x29, #-48]
	cbnz	x8, .LBB33_162
	b	.LBB33_165
.LBB33_161:
	mov	x9, xzr
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB33_165
.LBB33_162:
	cmp	x9, x8
	b.eq	.LBB33_165
// %bb.163:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB33_165
// %bb.164:
	bl	__put_page
.LBB33_165:
	stp	xzr, xzr, [x29, #-48]
.LBB33_166:
	ldrb	w8, [x19, #36]
	cbz	w8, .LBB33_172
// %bb.167:
	cmp	w22, #5                 // =5
	b.ne	.LBB33_170
// %bb.168:
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	tbz	w8, #0, .LBB33_170
// %bb.169:
	add	x0, x21, #424           // =424
	b	.LBB33_171
.LBB33_170:
	add	x0, x21, #304           // =304
.LBB33_171:
	bl	up_read
	ldurb	w1, [x29, #-23]
	mov	x0, x21
	bl	f2fs_balance_fs
.LBB33_172:
	adrp	x22, cpu_number
	adrp	x21, __cpu_online_mask
	add	x22, x22, :lo12:cpu_number
	add	x21, x21, :lo12:__cpu_online_mask
	cbz	x26, .LBB33_180
// %bb.173:
	adrp	x8, __tracepoint_f2fs_dedup_map_blocks+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_dedup_map_blocks+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB33_179
// %bb.174:
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
	ldr	w8, [x8, x22]
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x21, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB33_179
// %bb.175:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_dedup_map_blocks+32
	ldr	x23, [x8, :lo12:__tracepoint_f2fs_dedup_map_blocks+32]
	cbz	x23, .LBB33_177
.LBB33_176:                             // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	mov	x1, x24
	mov	x2, x26
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB33_176
.LBB33_177:
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB33_199
// %bb.178:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB33_199
.LBB33_179:
	mov	x0, x26
	bl	put_inner_inode
	mov	x20, x24
.LBB33_180:
	adrp	x8, __tracepoint_f2fs_map_blocks+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_map_blocks+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB33_186
// %bb.181:
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
	ldr	w8, [x8, x22]
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x21, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB33_186
// %bb.182:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w8, [x21, #24]
	add	w8, w8, #1              // =1
	str	w8, [x21, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_map_blocks+32
	ldr	x22, [x8, :lo12:__tracepoint_f2fs_map_blocks+32]
	cbz	x22, .LBB33_184
.LBB33_183:                             // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x22]
	mov	x1, x20
	mov	x2, x19
	mov	w3, w28
	blr	x8
	ldr	x8, [x22, #24]!
	cbnz	x8, .LBB33_183
.LBB33_184:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	b.eq	.LBB33_198
// %bb.185:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB33_198
.LBB33_186:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB33_200
// %bb.187:
	mov	w0, w28
	ldp	x20, x19, [sp, #240]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #224]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #208]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #192]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #176]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #160]    // 16-byte Folded Reload
	add	sp, sp, #256            // =256
	ret
.LBB33_188:
	mov	w28, w0
	cmp	w22, #2                 // =2
	b.ne	.LBB33_190
// %bb.189:
	str	wzr, [x19]
	cmn	w28, #2                 // =2
	b.ne	.LBB33_166
	b	.LBB33_191
.LBB33_190:
	cmn	w28, #2                 // =2
	b.ne	.LBB33_166
.LBB33_191:
	ldr	x8, [x19, #16]
	cbz	x8, .LBB33_193
// %bb.192:
	sub	x0, x29, #56            // =56
	mov	x1, x25
	bl	f2fs_get_next_page_offset
	ldr	x8, [x19, #16]
	str	x0, [x8]
.LBB33_193:
	ldr	x8, [x19, #24]
	cbz	x8, .LBB33_195
// %bb.194:
	sub	x0, x29, #56            // =56
	mov	x1, x25
	bl	f2fs_get_next_page_offset
	ldr	x8, [x19, #24]
	mov	w28, wzr
	str	x0, [x8]
	b	.LBB33_166
.LBB33_195:
	mov	w28, wzr
	b	.LBB33_166
.LBB33_196:
	mov	w28, #-5
	b	.LBB33_134
.LBB33_197:
	ldp	x24, x26, [sp, #24]     // 16-byte Folded Reload
	mov	w28, w0
	ldrb	w8, [x19, #12]
	tbnz	w8, #5, .LBB33_137
	b	.LBB33_159
.LBB33_198:
	bl	preempt_schedule_notrace
	b	.LBB33_186
.LBB33_199:
	bl	preempt_schedule_notrace
	b	.LBB33_179
.LBB33_200:
	bl	__stack_chk_fail
.Lfunc_end33:
	.size	f2fs_map_blocks, .Lfunc_end33-f2fs_map_blocks
                                        // -- End function
	.globl	f2fs_do_map_lock        // -- Begin function f2fs_do_map_lock
	.p2align	2
	.type	f2fs_do_map_lock,@function
f2fs_do_map_lock:                       // @f2fs_do_map_lock
.Lf2fs_do_map_lock$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	cmp	w1, #5                  // =5
	mov	x19, x0
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	b.ne	.LBB34_9
// %bb.1:
	tbz	w2, #0, .LBB34_13
// %bb.2:
	ldr	x8, [x19, #352]
	cbnz	x8, .LBB34_6
// %bb.3:
	add	x20, x19, #424          // =424
.LBB34_4:                               // =>This Inner Loop Header: Depth=1
	mov	x0, x20
	bl	down_read
	ldr	x8, [x19, #352]
	cbz	x8, .LBB34_19
// %bb.5:                               //   in Loop: Header=BB34_4 Depth=1
	mov	x0, x20
	bl	up_read
	ldr	x8, [x19, #352]
	cbz	x8, .LBB34_4
.LBB34_6:
	add	x20, x19, #368          // =368
	mov	x0, x20
	bl	_raw_spin_lock
	ldr	x8, [x19, #352]
	cbz	x8, .LBB34_12
// %bb.7:
	adrp	x21, jiffies
	ldr	x8, [x19, #360]
	ldr	x9, [x21, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.pl	.LBB34_12
// %bb.8:
	//APP
	mrs x0, sp_el0
	//NO_APP
	b	.LBB34_21
.LBB34_9:
	tbz	w2, #0, .LBB34_15
// %bb.10:
	add	x20, x19, #368          // =368
	mov	x0, x20
	bl	_raw_spin_lock
	ldr	x8, [x19, #352]
	cbz	x8, .LBB34_12
// %bb.11:
	adrp	x21, jiffies
	ldr	x8, [x19, #360]
	ldr	x9, [x21, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB34_20
.LBB34_12:
	mov	x0, x20
	bl	_raw_spin_unlock
	add	x0, x19, #304           // =304
	bl	down_read
	b	.LBB34_17
.LBB34_13:
	tbz	w3, #0, .LBB34_15
// %bb.14:
	add	x0, x19, #424           // =424
	b	.LBB34_16
.LBB34_15:
	add	x0, x19, #304           // =304
.LBB34_16:
	bl	up_read
.LBB34_17:
	mov	w0, wzr
.LBB34_18:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB34_19:
	mov	w0, #1
	b	.LBB34_18
.LBB34_20:
	//APP
	mrs x0, sp_el0
	//NO_APP
.LBB34_21:
	mov	x1, xzr
	bl	show_stack
	ldr	x1, [x19, #360]
	ldr	x2, [x21, :lo12:jiffies]
	adrp	x0, .L.str.12
	add	x0, x0, :lo12:.L.str.12
	bl	printk
	ldr	x0, [x19, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB34_12
.Lfunc_end34:
	.size	f2fs_do_map_lock, .Lfunc_end34-f2fs_do_map_lock
                                        // -- End function
	.p2align	2               // -- Begin function __allocate_data_block
	.type	__allocate_data_block,@function
__allocate_data_block:                  // @__allocate_data_block
// %bb.0:
	sub	sp, sp, #128            // =128
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	add	x29, sp, #48            // =48
	stp	x26, x25, [sp, #64]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #80]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #96]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #112]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x8, [x0]
	mov	w10, #1
	ldr	x9, [x8, #40]
	ldr	x20, [x9, #1008]
	stur	wzr, [x29, #-13]
	stur	wzr, [x29, #-16]
	stp	xzr, xzr, [sp, #16]
	str	x10, [sp, #8]
	ldr	x8, [x8, #736]
	tbnz	w8, #6, .LBB35_37
// %bb.1:
	mov	w23, w1
	ldr	w1, [x0, #24]
	mov	w21, w2
	mov	x19, x0
	add	x2, sp, #16             // =16
	mov	x0, x20
	mov	w3, wzr
	bl	get_node_info_ex
	cbnz	w0, .LBB35_21
// %bb.2:
	ldr	x8, [x19, #16]
	mov	x26, #2097152
	movk	x26, #1, lsl #32
	mov	x9, #-549755813888
	add	x8, x8, x26
	mov	x10, #-549755813888
	lsl	x8, x8, #6
	movk	x9, #4072
	movk	x10, #4076
	and	x11, x8, #0xfffffffffffff000
	ldr	w9, [x11, x9]
	ldr	w10, [x11, x10]
	ldr	x1, [x19]
	ldr	w8, [x19, #28]
	mov	x12, #-549755813888
	cmp	w9, w10
	add	x11, x11, x12
	b.ne	.LBB35_7
// %bb.3:
	cbz	x1, .LBB35_6
// %bb.4:
	ldr	x12, [x1, #736]
	tbz	w12, #26, .LBB35_7
// %bb.5:
	ldr	w12, [x1, #1184]
	asr	w12, w12, #2
	b	.LBB35_8
.LBB35_6:
	ldrb	w12, [x11, #3]
	tbnz	w12, #5, .LBB35_33
.LBB35_7:
	mov	w12, wzr
.LBB35_8:
	add	x13, x11, #360          // =360
	cmp	w9, w10
	csel	x9, x13, x11, eq
	add	w10, w12, w8
	ldr	w22, [x9, w10, uxtw #2]
	mov	x24, x19
	str	w22, [x24, #36]!
	cbnz	w22, .LBB35_11
// %bb.9:
	add	x2, sp, #8              // =8
	mov	x0, x20
	bl	inc_valid_block_count
	cbnz	w0, .LBB35_21
// %bb.10:
	ldr	w8, [x19, #28]
	ldr	w22, [x19, #36]
.LBB35_11:
	ldr	w9, [x19, #24]
	ldrb	w10, [sp, #28]
	add	x25, x20, #2296         // =2296
	mov	x0, x25
	sturh	w8, [x29, #-11]
	stur	w9, [x29, #-16]
	sturb	w10, [x29, #-12]
	bl	mutex_lock
	ldr	x8, [x20, #2328]
	cbz	x8, .LBB35_13
// %bb.12:
	ldr	x9, [x8, #408]
	add	x9, x9, #1              // =1
	str	x9, [x8, #408]
.LBB35_13:
	mov	x0, x25
	bl	mutex_unlock
	sub	x4, x29, #16            // =16
	mov	x0, x20
	mov	x1, xzr
	mov	w2, w22
	mov	x3, x24
	mov	w5, w23
	mov	x6, xzr
	mov	w7, wzr
	str	w21, [sp]
	bl	f2fs_allocate_data_block
	add	w8, w22, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB35_23
.LBB35_14:
	ldr	x0, [x19, #16]
	mov	w1, #1
	mov	w2, #1
	mov	w3, #1
	bl	f2fs_wait_on_page_writeback
	ldr	x8, [x19, #16]
	mov	x9, #-549755813888
	mov	x10, #-549755813888
	movk	x9, #4072
	add	x8, x8, x26
	lsl	x8, x8, #6
	movk	x10, #4076
	and	x11, x8, #0xfffffffffffff000
	ldr	w8, [x11, x9]
	ldr	w9, [x11, x10]
	mov	x10, #-549755813888
	add	x10, x11, x10
	cmp	w8, w9
	b.ne	.LBB35_16
// %bb.15:
	ldr	x11, [x19]
	ldr	x12, [x11, #736]
	tbnz	w12, #26, .LBB35_17
.LBB35_16:
	mov	w11, wzr
	b	.LBB35_18
.LBB35_17:
	ldr	w11, [x11, #1184]
	asr	w11, w11, #2
.LBB35_18:
	ldr	w13, [x19, #28]
	ldr	w14, [x19, #36]
	add	x12, x10, #360          // =360
	cmp	w8, w9
	csel	x8, x12, x10, eq
	add	w9, w13, w11
	str	w14, [x8, w9, uxtw #2]
	ldr	x0, [x19, #16]
	bl	set_page_dirty
	cbz	w0, .LBB35_20
// %bb.19:
	mov	w8, #1
	strb	w8, [x19, #33]
.LBB35_20:
	mov	x0, x19
	bl	f2fs_update_extent_cache
	mov	w0, wzr
.LBB35_21:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB35_38
// %bb.22:
	ldp	x20, x19, [sp, #112]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #96]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #80]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #64]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #128            // =128
	ret
.LBB35_23:
	ldr	w9, [x20, #3128]
	cbz	w9, .LBB35_31
// %bb.24:
	ldr	x8, [x20]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB35_30
// %bb.25:
	ldr	x10, [x20, #3136]
	add	x10, x10, #80           // =80
	b	.LBB35_27
.LBB35_26:                              //   in Loop: Header=BB35_27 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB35_30
.LBB35_27:                              // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w22
	b.hi	.LBB35_26
// %bb.28:                              //   in Loop: Header=BB35_27 Depth=1
	ldr	w11, [x10]
	cmp	w11, w22
	b.lo	.LBB35_26
// %bb.29:
	ldur	x8, [x10, #-80]
.LBB35_30:
	cbz	x8, .LBB35_14
.LBB35_31:
	ldr	x8, [x20, #136]
	cbz	x8, .LBB35_34
// %bb.32:
	add	x9, x8, #80             // =80
	b	.LBB35_35
.LBB35_33:
	ldrh	w12, [x11, #360]
	lsr	w12, w12, #2
	b	.LBB35_8
.LBB35_34:
	ldr	x9, [x20, #16]
	add	x9, x9, #72             // =72
.LBB35_35:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x20, #1184]
	ldr	w8, [x8]
	sub	w9, w22, w9
	lsr	w9, w9, w10
	sub	w8, w9, w8
	cmn	w8, #1                  // =1
	b.eq	.LBB35_14
// %bb.36:
	ldr	x8, [x20, #248]
	mov	w1, w22
	mov	x2, x1
	ldr	x0, [x8, #48]
	bl	invalidate_mapping_pages
	mov	x0, x20
	mov	w1, w22
	bl	f2fs_invalidate_compress_page
	b	.LBB35_14
.LBB35_37:
	mov	w0, #-1
	b	.LBB35_21
.LBB35_38:
	bl	__stack_chk_fail
.Lfunc_end35:
	.size	__allocate_data_block, .Lfunc_end35-__allocate_data_block
                                        // -- End function
	.p2align	2               // -- Begin function put_inner_inode
	.type	put_inner_inode,@function
put_inner_inode:                        // @put_inner_inode
// %bb.0:
	cbz	x0, .LBB36_12
// %bb.1:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0, #736]
	mov	x19, x0
	mov	x29, sp
	tbnz	x8, #38, .LBB36_10
.LBB36_2:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB36_5
// %bb.3:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB36_5
// %bb.4:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.put_inner_inode
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.put_inner_inode
	mov	w3, #4583
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB36_5:
	ldr	x8, [x19, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_raw_sb_info
	ldr	x8, [x19, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_ckpt_info
	ldr	x8, [x19, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/f2fs.h"; .popsection; .long 14472b - 14470b; .short 4583; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB36_11
// %bb.6:
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
	cbnz	w9, .LBB36_11
// %bb.7:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB36_11
// %bb.8:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB36_11
// %bb.9:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB36_11
.LBB36_10:
	ldr	x8, [x19, #736]
	tbz	x8, #39, .LBB36_2
.LBB36_11:
	mov	x0, x19
	bl	iput
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
.LBB36_12:
	ret
.Lfunc_end36:
	.size	put_inner_inode, .Lfunc_end36-put_inner_inode
                                        // -- End function
	.globl	f2fs_overwrite_io       // -- Begin function f2fs_overwrite_io
	.p2align	2
	.type	f2fs_overwrite_io,@function
f2fs_overwrite_io:                      // @f2fs_overwrite_io
.Lf2fs_overwrite_io$local:
// %bb.0:
	sub	sp, sp, #80             // =80
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	add	x29, sp, #48            // =48
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	str	xzr, [sp]
	ldr	x8, [x0, #80]
	add	x9, x2, x1
	cmp	x9, x8
	b.ls	.LBB37_2
.LBB37_1:
	mov	w0, wzr
	b	.LBB37_8
.LBB37_2:
	add	x9, x9, #4095           // =4095
	lsr	x8, x1, #12
	lsr	x20, x9, #12
	mov	w10, #8
	cmp	w8, w20
	stp	xzr, xzr, [sp, #16]
	str	w8, [sp, #4]
	str	w10, [sp, #32]
	strb	wzr, [sp, #36]
	b.hs	.LBB37_7
// %bb.3:
	mov	x19, x0
.LBB37_4:                               // =>This Inner Loop Header: Depth=1
	sub	w8, w20, w8
	mov	x1, sp
	mov	x0, x19
	mov	w2, wzr
	mov	w3, wzr
	str	w8, [sp, #8]
	bl	f2fs_map_blocks
	cbnz	w0, .LBB37_1
// %bb.5:                               //   in Loop: Header=BB37_4 Depth=1
	ldr	w8, [sp, #8]
	cbz	w8, .LBB37_1
// %bb.6:                               //   in Loop: Header=BB37_4 Depth=1
	ldr	w9, [sp, #4]
	add	w8, w9, w8
	cmp	w8, w20
	str	w8, [sp, #4]
	b.lo	.LBB37_4
.LBB37_7:
	mov	w0, #1
.LBB37_8:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB37_10
// %bb.9:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #80             // =80
	ret
.LBB37_10:
	bl	__stack_chk_fail
.Lfunc_end37:
	.size	f2fs_overwrite_io, .Lfunc_end37-f2fs_overwrite_io
                                        // -- End function
	.globl	f2fs_fiemap             // -- Begin function f2fs_fiemap
	.p2align	2
	.type	f2fs_fiemap,@function
f2fs_fiemap:                            // @f2fs_fiemap
.Lf2fs_fiemap$local:
// %bb.0:
	sub	sp, sp, #208            // =208
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #112]    // 16-byte Folded Spill
	add	x29, sp, #112           // =112
	stp	x28, x27, [sp, #128]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #144]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #160]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #176]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #192]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stp	xzr, xzr, [sp, #72]
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	ldrb	w8, [x1]
	ldr	w19, [x0, #1292]
	mov	x22, x3
	mov	x20, x1
	mov	x21, x0
	mov	x26, x2
	tbz	w8, #2, .LBB38_2
// %bb.1:
	mov	x0, x21
	bl	f2fs_precache_extents
	mov	w23, w0
	cbnz	w0, .LBB38_58
.LBB38_2:
	mov	w1, #3
	mov	x0, x20
	bl	fiemap_check_flags
	mov	w23, w0
	cbnz	w0, .LBB38_58
// %bb.3:
	add	x24, x21, #160          // =160
	mov	x0, x24
	bl	down_write
	mov	x0, x21
	bl	max_file_blocks
	lsl	x8, x0, #12
	subs	x9, x8, x26
	b.hs	.LBB38_5
// %bb.4:
	mov	w23, #-27
	b	.LBB38_57
.LBB38_5:
	ldrb	w10, [x20]
	sub	x11, x8, x22
	cmp	x11, x26
	ccmp	x8, x22, #0, hs
	csel	x28, x9, x22, lo
	tbnz	w10, #1, .LBB38_35
// %bb.6:
	ldr	x8, [x21, #736]
	tbnz	w8, #10, .LBB38_8
// %bb.7:
	ldr	x8, [x21, #736]
	tbz	w8, #11, .LBB38_9
.LBB38_8:
	mov	x0, x21
	mov	x1, x20
	mov	x2, x26
	mov	x3, x28
	bl	f2fs_inline_data_fiemap
	mov	w27, w0
	cmn	w0, #11                 // =11
	b.ne	.LBB38_56
.LBB38_9:
	stp	xzr, xzr, [sp, #24]     // 16-byte Folded Spill
	ldrb	w8, [x21, #142]
	sub	w10, w19, #1            // =1
	mov	w9, #1
	stp	x24, x10, [sp]          // 16-byte Folded Spill
	lsr	x10, x28, x8
	cmp	x10, #0                 // =0
	lsl	x9, x9, x8
	csel	x9, x9, x28, eq
	str	x9, [sp, #16]           // 8-byte Folded Spill
	add	x9, x26, x9
	sub	x9, x9, #1              // =1
	mov	x25, xzr
	mov	w22, wzr
	mov	w23, wzr
	lsr	x28, x26, x8
	lsr	x26, x9, x8
	//APP
	mrs x24, sp_el0
	//NO_APP
	b	.LBB38_11
.LBB38_10:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	w8, [x24, #2128]
	ubfx	w8, w8, #8, #1
	cbnz	w8, .LBB38_47
.LBB38_11:                              // =>This Inner Loop Header: Depth=1
	add	x0, sp, #48             // =48
	mov	w2, #40
	mov	w1, wzr
	bl	memset
	add	x9, sp, #40             // =40
	ldrb	w8, [x21, #142]
	str	x9, [sp, #64]
	ldr	x9, [sp, #16]           // 8-byte Folded Reload
	lsr	x8, x9, x8
	stp	w28, w8, [sp, #52]
	mov	w8, #8
	str	w8, [sp, #80]
	tbz	w23, #0, .LBB38_13
// %bb.12:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	str	w8, [sp, #56]
.LBB38_13:                              //   in Loop: Header=BB38_11 Depth=1
	add	x1, sp, #48             // =48
	mov	w3, #1
	mov	x0, x21
	mov	w2, wzr
	bl	f2fs_map_blocks
	cbnz	w0, .LBB38_44
// %bb.14:                              //   in Loop: Header=BB38_11 Depth=1
	ldrh	w8, [sp, #60]
	mov	w9, #4192
	tst	w8, w9
	b.eq	.LBB38_23
// %bb.15:                              //   in Loop: Header=BB38_11 Depth=1
	cbz	x25, .LBB38_17
.LBB38_16:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	w8, [x21, #12]
	mov	w9, #4096
	ldp	x2, x1, [sp, #24]       // 16-byte Folded Reload
	mov	x0, x20
	tst	w8, #0x4000
	mov	w8, #4224
	csel	w8, w9, w8, eq
	orr	w22, w8, w22
	mov	x3, x25
	mov	w4, w22
	bl	fiemap_fill_next_extent
	cbnz	w0, .LBB38_44
.LBB38_17:                              //   in Loop: Header=BB38_11 Depth=1
	cmp	x28, x26
	b.hi	.LBB38_45
// %bb.18:                              //   in Loop: Header=BB38_11 Depth=1
	tbz	w23, #0, .LBB38_21
// %bb.19:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	add	x8, x28, x8
	cmp	x8, x26
	b.hi	.LBB38_45
// %bb.20:                              //   in Loop: Header=BB38_11 Depth=1
	ldrb	w9, [x21, #142]
	ldr	w10, [sp, #48]
	sub	x11, x28, #1            // =1
	mov	w23, wzr
	lsl	x11, x11, x9
	lsl	x10, x10, x9
	stp	x10, x11, [sp, #24]     // 16-byte Folded Spill
	lsl	x25, x19, x9
	orr	w22, w22, #0x8
	mov	x28, x8
	b	.LBB38_32
.LBB38_21:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	w8, [sp, #48]
	cmn	w8, #2                  // =2
	b.ne	.LBB38_28
// %bb.22:                              //   in Loop: Header=BB38_11 Depth=1
	mov	x25, xzr
	add	x28, x28, #1            // =1
	mov	w23, #1
	b	.LBB38_32
.LBB38_23:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	w10, [x21, #1184]
	ldr	w11, [x21, #1192]
	ldrh	w12, [x21]
	ldr	x28, [sp, #40]
	ldrb	w8, [x21, #142]
	add	w10, w11, w10, asr #2
	and	w11, w12, #0xf000
	cmp	w11, #8, lsl #12        // =32768
	mov	w11, #923
	lsl	x9, x28, x8
	sub	w10, w11, w10
	b.ne	.LBB38_29
// %bb.24:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	x11, [x21, #736]
	tbz	w11, #31, .LBB38_26
// %bb.25:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	w11, [x21, #1292]
	neg	w11, w11
	and	w10, w10, w11
.LBB38_26:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	x11, [x21, #736]
	mov	w10, w10
	tbnz	w11, #31, .LBB38_30
// %bb.27:                              //   in Loop: Header=BB38_11 Depth=1
	mov	w11, #22372
	movk	w11, #16129, lsl #16
	b	.LBB38_31
.LBB38_28:                              //   in Loop: Header=BB38_11 Depth=1
	ldrb	w9, [x21, #142]
	ldp	w10, w11, [sp, #56]
	mov	w23, wzr
	lsl	x12, x28, x9
	lsl	x8, x8, x9
	stp	x8, x12, [sp, #24]      // 16-byte Folded Spill
	lsl	x25, x10, x9
	lsr	w8, w11, #1
	and	w22, w8, #0x800
	lsr	x8, x25, x9
	add	x28, x8, x28
	b	.LBB38_32
.LBB38_29:                              //   in Loop: Header=BB38_11 Depth=1
	mov	w11, #22372
	mov	w10, w10
	movk	w11, #16129, lsl #16
	b	.LBB38_31
.LBB38_30:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	w11, [x21, #1292]
	mov	w12, #1018
	neg	w11, w11
	and	w11, w11, w12
	mov	w12, #55322
	movk	w12, #15, lsl #16
	mul	w11, w11, w12
.LBB38_31:                              //   in Loop: Header=BB38_11 Depth=1
	add	x10, x11, x10
	lsl	x8, x10, x8
	cmp	x9, x8
	b.hs	.LBB38_34
.LBB38_32:                              //   in Loop: Header=BB38_11 Depth=1
	ldr	x8, [x24]
	tbnz	w8, #0, .LBB38_10
// %bb.33:                              //   in Loop: Header=BB38_11 Depth=1
	mov	w8, wzr
	cbz	w8, .LBB38_11
	b	.LBB38_47
.LBB38_34:                              //   in Loop: Header=BB38_11 Depth=1
	orr	w22, w22, #0x1
	cbnz	x25, .LBB38_16
	b	.LBB38_17
.LBB38_35:
	ldr	x8, [x21, #40]
	ldr	x22, [x8, #1008]
	stp	xzr, xzr, [x29, #-24]
	ldr	w23, [x21, #824]
	ldr	x8, [x21, #736]
	tbnz	w8, #9, .LBB38_41
// %bb.36:
	mov	x19, x24
	mov	x24, xzr
	mov	x25, xzr
	mov	w26, wzr
.LBB38_37:
	cbz	w23, .LBB38_52
// %bb.38:
	ldr	x8, [x22, #128]
	mov	w2, #7
	mov	x1, x23
	ldr	x0, [x8, #48]
	ldr	w3, [x0, #24]
	bl	pagecache_get_page
	cbz	x0, .LBB38_49
// %bb.39:
	mov	x28, x0
	sub	x2, x29, #24            // =24
	mov	x0, x22
	mov	w1, w23
	mov	w3, wzr
	bl	get_node_info_ex
	cbz	w0, .LBB38_51
// %bb.40:
	mov	w27, w0
	mov	w1, #1
	mov	x0, x28
	bl	f2fs_put_page
	b	.LBB38_55
.LBB38_41:
	ldr	x8, [x22, #128]
	ldr	x1, [x21, #64]
	mov	w2, #7
	ldr	x0, [x8, #48]
	ldr	w3, [x0, #24]
	bl	pagecache_get_page
	cbz	x0, .LBB38_46
// %bb.42:
	ldr	w1, [x21, #64]
	mov	x26, x0
	sub	x2, x29, #24            // =24
	mov	x0, x22
	mov	w3, wzr
	mov	x19, x24
	bl	get_node_info_ex
	cbz	w0, .LBB38_50
// %bb.43:
	mov	w27, w0
	mov	w1, #1
	mov	x0, x26
	bl	f2fs_put_page
	b	.LBB38_55
.LBB38_44:
	ldr	x24, [sp]               // 8-byte Folded Reload
	mov	w27, w0
	b	.LBB38_56
.LBB38_45:
	mov	w23, wzr
	b	.LBB38_48
.LBB38_46:
	mov	x19, x24
	mov	w27, #-12
	b	.LBB38_55
.LBB38_47:
	mov	w23, #-4
.LBB38_48:
	ldr	x24, [sp]               // 8-byte Folded Reload
	b	.LBB38_57
.LBB38_49:
	mov	w27, #-12
	b	.LBB38_55
.LBB38_50:
	ldur	w8, [x29, #-16]
	ldrb	w9, [x21, #142]
	ldr	w10, [x21, #1192]
	ldr	x11, [x21, #736]
	mov	w12, #4052
	lsl	x8, x8, x9
	lsl	w9, w10, #2
	lsl	x10, x11, #54
	sub	w11, w12, w9
	sxtw	x9, w9
	mov	w1, #1
	mov	x0, x26
	add	x24, x8, w11, sxtw
	and	x25, x9, x10, asr #63
	bl	f2fs_put_page
	cmp	w23, #0                 // =0
	mov	w8, #768
	cinc	w26, w8, eq
	mov	x0, x20
	mov	x1, xzr
	mov	x2, x24
	mov	x3, x25
	mov	w4, w26
	bl	fiemap_fill_next_extent
	mov	w27, w0
	cbnz	w0, .LBB38_55
	b	.LBB38_37
.LBB38_51:
	ldr	x8, [x21, #40]
	ldur	w9, [x29, #-16]
	ldrb	w10, [x21, #142]
	mov	w1, #1
	ldr	x25, [x8, #24]
	mov	x0, x28
	lsl	x24, x9, x10
	mov	w26, #1
	bl	f2fs_put_page
.LBB38_52:
	cbz	x24, .LBB38_54
// %bb.53:
	mov	x0, x20
	mov	x1, xzr
	mov	x2, x24
	mov	x3, x25
	mov	w4, w26
	bl	fiemap_fill_next_extent
	and	w27, w0, w0, asr #31
	b	.LBB38_55
.LBB38_54:
	mov	w27, wzr
.LBB38_55:
	mov	x24, x19
.LBB38_56:
	cmp	w27, #1                 // =1
	csel	w23, wzr, w27, eq
.LBB38_57:
	mov	x0, x24
	bl	up_write
.LBB38_58:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB38_60
// %bb.59:
	mov	w0, w23
	ldp	x20, x19, [sp, #192]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #176]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #160]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #144]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #128]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #112]    // 16-byte Folded Reload
	add	sp, sp, #208            // =208
	ret
.LBB38_60:
	bl	__stack_chk_fail
.Lfunc_end38:
	.size	f2fs_fiemap, .Lfunc_end38-f2fs_fiemap
                                        // -- End function
	.globl	f2fs_read_multi_pages   // -- Begin function f2fs_read_multi_pages
	.p2align	2
	.type	f2fs_read_multi_pages,@function
f2fs_read_multi_pages:                  // @f2fs_read_multi_pages
.Lf2fs_read_multi_pages$local:
// %bb.0:
	sub	sp, sp, #240            // =240
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #144]    // 16-byte Folded Spill
	add	x29, sp, #144           // =144
	stp	x28, x27, [sp, #160]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #176]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #192]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #208]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #224]    // 16-byte Folded Spill
	str	w4, [sp, #16]           // 4-byte Folded Spill
	stur	x8, [x29, #-8]
	stp	xzr, xzr, [x29, #-24]
	stp	xzr, xzr, [x29, #-40]
	stp	xzr, xzr, [x29, #-56]
	ldp	x27, x8, [x0]
	ldr	w9, [x0, #20]
	ldr	x25, [x1]
	mov	w22, w5
	str	x8, [sp, #40]           // 8-byte Folded Spill
	ldr	x8, [x27, #40]
	ldrb	w19, [x27, #142]
	mov	x24, x3
	mov	w26, w2
	ldr	x8, [x8, #1008]
	mov	x21, x0
	str	x1, [sp, #56]           // 8-byte Folded Spill
	str	x9, [sp, #32]           // 8-byte Folded Spill
	stur	x8, [x29, #-64]         // 8-byte Folded Spill
	mov	w20, #1
	bl	f2fs_cluster_is_empty
	tbnz	w0, #0, .LBB39_116
.LBB39_1:
	ldrb	w8, [x27, #14]
	str	x24, [sp, #48]          // 8-byte Folded Spill
	str	w26, [sp, #28]          // 4-byte Folded Spill
	str	x25, [sp, #64]          // 8-byte Folded Spill
	tbnz	w8, #0, .LBB39_4
// %bb.2:
	ldr	x8, [x27, #736]
	tbnz	w8, #30, .LBB39_4
// %bb.3:
	mov	x25, x27
	add	x10, x27, #80           // =80
	b	.LBB39_5
.LBB39_4:
	ldr	x8, [x27, #40]
	mov	x25, x27
	add	x10, x8, #32            // =32
.LBB39_5:
	ldr	w8, [x21, #16]
	lsl	x9, x20, x19
	str	x9, [sp, #72]           // 8-byte Folded Spill
	cbz	w8, .LBB39_22
// %bb.6:
	ldr	x10, [x10]
	ldr	x12, [sp, #72]          // 8-byte Folded Reload
	ldrb	w11, [x25, #142]
	mov	x28, #2097152
	mov	w9, wzr
	add	x10, x10, w12, uxtw
	sub	x10, x10, #1            // =1
	movk	x28, #1, lsl #32
	mov	w27, #1
	lsr	x24, x10, x11
	mov	x20, #-549755813888
	//APP
	mrs x19, sp_el0
	//NO_APP
	b	.LBB39_9
.LBB39_7:                               //   in Loop: Header=BB39_9 Depth=1
	ldr	x8, [x21, #24]
	str	xzr, [x8, x26, lsl #3]
	ldr	w9, [x21, #32]
	ldr	w8, [x21, #16]
	sub	w9, w9, #1              // =1
	str	w9, [x21, #32]
.LBB39_8:                               //   in Loop: Header=BB39_9 Depth=1
	add	w9, w26, #1             // =1
	cmp	w9, w8
	b.hs	.LBB39_22
.LBB39_9:                               // =>This Inner Loop Header: Depth=1
	ldr	x10, [x21, #24]
	sxtw	x26, w9
	ldr	x23, [x10, w9, sxtw #3]
	cbz	x23, .LBB39_8
// %bb.10:                              //   in Loop: Header=BB39_9 Depth=1
	ldr	x9, [x23, #32]
	cmp	x9, x24
	b.hs	.LBB39_13
// %bb.11:                              //   in Loop: Header=BB39_9 Depth=1
	ldr	x9, [x23, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x9, x23, x10, eq
	ldr	x9, [x9]
	tbz	w9, #2, .LBB39_8
// %bb.12:                              //   in Loop: Header=BB39_9 Depth=1
	//APP
	dmb ishld
	//NO_APP
	b	.LBB39_18
.LBB39_13:                              //   in Loop: Header=BB39_9 Depth=1
	ldr	w8, [x19, #24]
	mov	w2, #4096
	mov	w1, wzr
	add	w8, w8, #1              // =1
	str	w8, [x19, #24]
	//APP
	//NO_APP
	ldr	w8, [x19, #2800]
	add	w8, w8, #1              // =1
	str	w8, [x19, #2800]
	add	x8, x23, x28
	//APP
	//NO_APP
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	add	x0, x8, x20
	bl	memset
	//APP
	//NO_APP
	ldr	w8, [x19, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x19, #2800]
	//APP
	//NO_APP
	ldr	x8, [x19, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x19, #24]
	b.eq	.LBB39_21
// %bb.14:                              //   in Loop: Header=BB39_9 Depth=1
	ldr	x8, [x19, #24]
	cbz	x8, .LBB39_21
.LBB39_15:                              //   in Loop: Header=BB39_9 Depth=1
	mov	x0, x23
	bl	flush_dcache_page
	ldr	x8, [x23, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x23, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB39_17
// %bb.16:                              //   in Loop: Header=BB39_9 Depth=1
	//APP
	dmb ishst
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x23]
1:	ldxr	x8, [x23]
	orr	x8, x8, 4
	stxr	w9, x8, [x23]
	cbnz	w9, 1b
	//NO_APP
	b	.LBB39_18
.LBB39_17:                              //   in Loop: Header=BB39_9 Depth=1
	//APP
	dmb ishld
	//NO_APP
.LBB39_18:                              //   in Loop: Header=BB39_9 Depth=1
	mov	x0, x23
	bl	unlock_page
	tbz	w22, #0, .LBB39_7
// %bb.19:                              //   in Loop: Header=BB39_9 Depth=1
	ldr	x8, [x23, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x23, x9, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w27
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB39_7
// %bb.20:                              //   in Loop: Header=BB39_9 Depth=1
	bl	__put_page
	b	.LBB39_7
.LBB39_21:                              //   in Loop: Header=BB39_9 Depth=1
	bl	preempt_schedule
	b	.LBB39_15
.LBB39_22:
	mov	x0, x21
	bl	f2fs_cluster_is_empty
	tbz	w0, #0, .LBB39_24
// %bb.23:
	ldr	x26, [sp, #64]          // 8-byte Folded Reload
	mov	w23, wzr
	ldr	w8, [x21, #16]
	cbnz	w8, .LBB39_26
	b	.LBB39_61
.LBB39_24:
	ldp	x9, x8, [sp, #32]       // 16-byte Folded Reload
	sub	x0, x29, #56            // =56
	mov	w2, #48
	mov	w1, wzr
	lsl	x19, x8, x9
	bl	memset
	and	x1, x19, #0xffffffff
	sub	x0, x29, #56            // =56
	mov	w2, #1
	stp	x25, xzr, [x29, #-56]
	stur	xzr, [x29, #-40]
	stur	wzr, [x29, #-32]
	bl	f2fs_get_dnode_of_data
	ldr	x26, [sp, #64]          // 8-byte Folded Reload
	mov	w23, w0
	cbz	w0, .LBB39_30
// %bb.25:
	ldr	w8, [x21, #16]
	cbz	w8, .LBB39_61
.LBB39_26:
	mov	w9, wzr
	mov	w19, #4
	mov	w20, #256
	b	.LBB39_28
.LBB39_27:                              //   in Loop: Header=BB39_28 Depth=1
	add	w9, w22, #1             // =1
	cmp	w9, w8
	b.hs	.LBB39_61
.LBB39_28:                              // =>This Inner Loop Header: Depth=1
	ldr	x10, [x21, #24]
	sxtw	x22, w9
	ldr	x10, [x10, w9, sxtw #3]
	cbz	x10, .LBB39_27
// %bb.29:                              //   in Loop: Header=BB39_28 Depth=1
	ldr	x8, [x10, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x10, x9, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x19
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x21, #24]
	lsl	x9, x22, #3
	ldr	x8, [x8, x9]
	ldr	x10, [x8, #8]
	sub	x11, x10, #1            // =1
	tst	x10, #0x1
	csel	x8, x8, x11, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x20
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	x8, [x21, #24]
	ldr	x0, [x8, x9]
	bl	unlock_page
	ldr	w8, [x21, #16]
	b	.LBB39_27
.LBB39_30:
	ldur	w8, [x29, #-20]
	cmn	w8, #2                  // =2
	b.ne	.LBB39_124
.LBB39_31:
	ldr	w8, [x21, #16]
	mov	x20, x25
	ldur	x25, [x29, #-64]        // 8-byte Folded Reload
	cmp	w8, #2                  // =2
	b.lo	.LBB39_51
// %bb.32:
	mov	x23, #2097152
	mov	x28, #-549755813888
	mov	x27, #-549755813888
	mov	w19, #1
	movk	x23, #1, lsl #32
	mov	x24, #-549755813888
	movk	x28, #4072
	movk	x27, #4076
.LBB39_33:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB39_44 Depth 2
	ldur	x8, [x29, #-40]
	ldur	w11, [x29, #-28]
	add	x8, x8, x23
	lsl	x8, x8, #6
	and	x12, x8, #0xfffffffffffff000
	ldr	w9, [x12, x28]
	ldr	w10, [x12, x27]
	add	w8, w11, w19
	add	x11, x12, x24
	cmp	w9, w10
	b.ne	.LBB39_38
// %bb.34:                              //   in Loop: Header=BB39_33 Depth=1
	ldur	x12, [x29, #-56]
	cbz	x12, .LBB39_37
// %bb.35:                              //   in Loop: Header=BB39_33 Depth=1
	ldr	x13, [x12, #736]
	tbz	w13, #26, .LBB39_38
// %bb.36:                              //   in Loop: Header=BB39_33 Depth=1
	ldr	w12, [x12, #1184]
	asr	w12, w12, #2
	b	.LBB39_39
.LBB39_37:                              //   in Loop: Header=BB39_33 Depth=1
	ldrb	w12, [x11, #3]
	tbnz	w12, #5, .LBB39_50
.LBB39_38:                              //   in Loop: Header=BB39_33 Depth=1
	mov	w12, wzr
.LBB39_39:                              //   in Loop: Header=BB39_33 Depth=1
	add	x13, x11, #360          // =360
	cmp	w9, w10
	csel	x9, x13, x11, eq
	add	w8, w8, w12
	ldr	w1, [x9, w8, uxtw #2]
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	b.lo	.LBB39_51
// %bb.40:                              //   in Loop: Header=BB39_33 Depth=1
	ldr	w9, [x25, #3128]
	cbz	w9, .LBB39_48
// %bb.41:                              //   in Loop: Header=BB39_33 Depth=1
	ldr	x8, [x25]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB39_47
// %bb.42:                              //   in Loop: Header=BB39_33 Depth=1
	ldr	x10, [x25, #3136]
	add	x10, x10, #80           // =80
	b	.LBB39_44
.LBB39_43:                              //   in Loop: Header=BB39_44 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB39_47
.LBB39_44:                              //   Parent Loop BB39_33 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB39_43
// %bb.45:                              //   in Loop: Header=BB39_44 Depth=2
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB39_43
// %bb.46:                              //   in Loop: Header=BB39_33 Depth=1
	ldur	x8, [x10, #-80]
.LBB39_47:                              //   in Loop: Header=BB39_33 Depth=1
	cbz	x8, .LBB39_51
.LBB39_48:                              //   in Loop: Header=BB39_33 Depth=1
	mov	w2, #6
	mov	x0, x25
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB39_100
// %bb.49:                              //   in Loop: Header=BB39_33 Depth=1
	ldr	w8, [x21, #48]
	ldr	w9, [x21, #16]
	add	w19, w19, #1            // =1
	add	w8, w8, #1              // =1
	cmp	w19, w9
	str	w8, [x21, #48]
	b.lo	.LBB39_33
	b	.LBB39_51
.LBB39_50:                              //   in Loop: Header=BB39_33 Depth=1
	ldrh	w12, [x11, #360]
	lsr	w12, w12, #2
	b	.LBB39_39
.LBB39_51:
	ldr	w8, [x21, #48]
	cbz	w8, .LBB39_55
// %bb.52:
	mov	x0, x21
	bl	f2fs_alloc_dic
	mov	x24, x0
	cmn	x0, #1, lsl #12         // =4096
	b.ls	.LBB39_64
// %bb.53:
	mov	w23, w24
	ldur	x0, [x29, #-40]
	cbnz	x0, .LBB39_56
.LBB39_54:
	mov	x9, xzr
	ldur	x8, [x29, #-48]
	cbnz	x8, .LBB39_57
	b	.LBB39_60
.LBB39_55:
	mov	w23, wzr
	ldur	x0, [x29, #-40]
	cbz	x0, .LBB39_54
.LBB39_56:
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB39_60
.LBB39_57:
	cmp	x9, x8
	b.eq	.LBB39_60
// %bb.58:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB39_60
// %bb.59:
	bl	__put_page
.LBB39_60:
	stp	xzr, xzr, [x29, #-48]
	ldr	w8, [x21, #16]
	cbnz	w8, .LBB39_26
.LBB39_61:
	ldr	x19, [sp, #56]          // 8-byte Folded Reload
.LBB39_62:
	str	x26, [x19]
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB39_132
// %bb.63:
	mov	w0, w23
	ldp	x20, x19, [sp, #224]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #208]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #192]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #176]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #160]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #144]    // 16-byte Folded Reload
	add	sp, sp, #240            // =240
	ret
.LBB39_64:
	ldr	w8, [x21, #48]
	cbz	w8, .LBB39_98
// %bb.65:
	ldr	w8, [sp, #16]           // 4-byte Folded Reload
	add	x9, x24, #108           // =108
	str	x9, [sp, #32]           // 8-byte Folded Spill
	add	x9, x25, #1308          // =1308
	tst	w8, #0x1
	mov	w8, #1048576
	mov	x27, x20
	mov	w19, wzr
	str	x9, [sp, #16]           // 8-byte Folded Spill
	add	x9, x25, #2344          // =2344
	mov	x20, #-549755813888
	csel	w8, w8, wzr, ne
	str	x9, [sp, #40]           // 8-byte Folded Spill
	str	w8, [sp, #12]           // 4-byte Folded Spill
	b	.LBB39_68
.LBB39_66:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	x8, [x23, #8]
	mov	w11, #256
	mov	x20, #-549755813888
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x23, x9, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x11
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	str	x28, [x8]
.LBB39_67:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	w8, [x21, #48]
	cmp	w19, w8
	b.hs	.LBB39_98
.LBB39_68:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB39_84 Depth 2
	ldur	x8, [x29, #-40]
	mov	x10, #2097152
	movk	x10, #1, lsl #32
	ldr	x9, [x24, #48]
	add	x8, x8, x10
	lsl	x8, x8, #6
	and	x12, x8, #0xfffffffffffff000
	mov	x8, #-549755813888
	movk	x8, #4072
	ldr	w10, [x12, x8]
	mov	x8, #-549755813888
	movk	x8, #4076
	ldr	w11, [x12, x8]
	ldr	x23, [x9, w19, sxtw #3]
	ldur	w8, [x29, #-28]
	add	x9, x12, x20
	cmp	w10, w11
	b.ne	.LBB39_73
// %bb.69:                              //   in Loop: Header=BB39_68 Depth=1
	ldur	x12, [x29, #-56]
	cbz	x12, .LBB39_72
// %bb.70:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	x13, [x12, #736]
	tbz	w13, #26, .LBB39_73
// %bb.71:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	w12, [x12, #1184]
	asr	w12, w12, #2
	b	.LBB39_74
.LBB39_72:                              //   in Loop: Header=BB39_68 Depth=1
	ldrb	w12, [x9, #3]
	tbnz	w12, #5, .LBB39_90
.LBB39_73:                              //   in Loop: Header=BB39_68 Depth=1
	mov	w12, wzr
.LBB39_74:                              //   in Loop: Header=BB39_68 Depth=1
	add	w19, w19, #1            // =1
	add	x13, x9, #360           // =360
	cmp	w10, w11
	add	w8, w19, w8
	csel	x9, x13, x9, eq
	add	w8, w8, w12
	ldr	w28, [x9, w8, uxtw #2]
	mov	x0, x27
	mov	w1, w28
	bl	f2fs_wait_on_block_writeback
	ldur	x0, [x29, #-64]         // 8-byte Folded Reload
	mov	x1, x23
	mov	w2, w28
	bl	f2fs_load_compressed_page
	tbz	w0, #0, .LBB39_77
// %bb.75:                              //   in Loop: Header=BB39_68 Depth=1
	add	x8, x24, #104           // =104
	mov	w11, #1
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w11
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB39_67
// %bb.76:                              //   in Loop: Header=BB39_68 Depth=1
	mov	x0, x24
	bl	f2fs_decompress_cluster
	b	.LBB39_67
.LBB39_77:                              //   in Loop: Header=BB39_68 Depth=1
	cbz	x26, .LBB39_92
// %bb.78:                              //   in Loop: Header=BB39_68 Depth=1
	ldur	x8, [x29, #-64]         // 8-byte Folded Reload
	ldr	x9, [x8, #1224]
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	ldr	w8, [x8]
	cbz	x9, .LBB39_80
// %bb.79:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	w10, [x26, #40]
	cmp	x9, x10
	b.ls	.LBB39_91
.LBB39_80:                              //   in Loop: Header=BB39_68 Depth=1
	add	w8, w8, #1              // =1
	cmp	w8, w28
	b.ne	.LBB39_91
// %bb.81:                              //   in Loop: Header=BB39_68 Depth=1
	ldur	x10, [x29, #-64]        // 8-byte Folded Reload
	ldr	x8, [x10]
	ldr	w9, [x10, #3128]
	ldr	x8, [x8, #248]
	cmp	w9, #2                  // =2
	b.lt	.LBB39_87
// %bb.82:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	x10, [x10, #3136]
	add	x10, x10, #80           // =80
	b	.LBB39_84
.LBB39_83:                              //   in Loop: Header=BB39_84 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB39_87
.LBB39_84:                              //   Parent Loop BB39_68 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	w11, w28
	b.hi	.LBB39_83
// %bb.85:                              //   in Loop: Header=BB39_84 Depth=2
	ldr	w11, [x10]
	cmp	w11, w28
	b.lo	.LBB39_83
// %bb.86:                              //   in Loop: Header=BB39_68 Depth=1
	ldur	x8, [x10, #-80]
.LBB39_87:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	x9, [x26, #8]
	ldr	x10, [x8, #128]
	cmp	x9, x10
	b.ne	.LBB39_91
// %bb.88:                              //   in Loop: Header=BB39_68 Depth=1
	ldrb	w9, [x26, #27]
	ldrb	w8, [x8, #108]
	cmp	w9, w8
	b.ne	.LBB39_91
// %bb.89:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	x2, [x23, #32]
	mov	x0, x26
	mov	x1, x27
	bl	fscrypt_mergeable_bio
	tbnz	w0, #0, .LBB39_92
	b	.LBB39_91
.LBB39_90:                              //   in Loop: Header=BB39_68 Depth=1
	ldrh	w12, [x9, #360]
	lsr	w12, w12, #2
	b	.LBB39_74
.LBB39_91:                              //   in Loop: Header=BB39_68 Depth=1
	ldur	x0, [x29, #-64]         // 8-byte Folded Reload
	mov	x1, x26
	mov	w2, wzr
	bl	__submit_bio
	mov	x26, xzr
.LBB39_92:                              //   in Loop: Header=BB39_68 Depth=1
	cbnz	x26, .LBB39_94
// %bb.93:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	x4, [x23, #32]
	ldr	w2, [sp, #28]           // 4-byte Folded Reload
	ldr	w3, [sp, #12]           // 4-byte Folded Reload
	and	w5, w22, #0x1
	mov	x0, x27
	mov	w1, w28
	bl	f2fs_grab_read_bio
	mov	x26, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB39_108
.LBB39_94:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	x25, [sp, #72]          // 8-byte Folded Reload
	mov	x0, x26
	mov	x1, x23
	mov	w3, wzr
	mov	w2, w25
	bl	bio_add_page
	cmp	w0, w25
	b.lo	.LBB39_91
// %bb.95:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	x8, [x26, #72]
	ldr	x0, [sp, #32]           // 8-byte Folded Reload
	ldr	w9, [x8, #64]
	orr	w9, w9, #0x2
	str	w9, [x8, #64]
	bl	refcount_inc_checked
	ldr	x10, [sp, #16]          // 8-byte Folded Reload
	//APP
	// atomic_add
	prfm	pstl1strm, [x10]
1:	ldxr	w8, [x10]
	add	w8, w8, 1
	stxr	w9, w8, [x10]
	cbnz	w9, 1b

	//NO_APP
	ldur	x25, [x29, #-64]        // 8-byte Folded Reload
	ldrb	w8, [x25, #2704]
	cbz	w8, .LBB39_66
// %bb.96:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	x20, [sp, #40]          // 8-byte Folded Reload
	mov	x0, x20
	bl	_raw_spin_lock
	ldr	x8, [x25, #2480]
	mov	x0, x20
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x25, #2480]
	bl	_raw_spin_unlock
	mov	x0, x25
	bl	f2fs_record_iostat
	ldrb	w8, [x25, #2704]
	cbz	w8, .LBB39_66
// %bb.97:                              //   in Loop: Header=BB39_68 Depth=1
	ldr	x20, [sp, #40]          // 8-byte Folded Reload
	mov	x0, x20
	bl	_raw_spin_lock
	ldr	x8, [x25, #2496]
	mov	x0, x20
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x25, #2496]
	bl	_raw_spin_unlock
	mov	x0, x25
	bl	f2fs_record_iostat
	b	.LBB39_66
.LBB39_98:
	ldur	x0, [x29, #-40]
	cbz	x0, .LBB39_101
// %bb.99:
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	b	.LBB39_102
.LBB39_100:
	mov	w23, #-14
	ldur	x0, [x29, #-40]
	cbnz	x0, .LBB39_56
	b	.LBB39_54
.LBB39_101:
	mov	x9, xzr
.LBB39_102:
	ldr	x19, [sp, #56]          // 8-byte Folded Reload
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB39_106
// %bb.103:
	cmp	x9, x8
	b.eq	.LBB39_106
// %bb.104:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB39_106
// %bb.105:
	bl	__put_page
.LBB39_106:
	mov	w23, wzr
.LBB39_107:
	stp	xzr, xzr, [x29, #-48]
	b	.LBB39_62
.LBB39_108:
	cmp	w26, #0                 // =0
	cset	w1, ne
	mov	x0, x24
	mov	w23, w26
	bl	f2fs_decompress_end_io
	ldur	x0, [x29, #-40]
	cbz	x0, .LBB39_110
// %bb.109:
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	b	.LBB39_111
.LBB39_110:
	mov	x9, xzr
.LBB39_111:
	ldr	x19, [sp, #56]          // 8-byte Folded Reload
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB39_115
// %bb.112:
	cmp	x9, x8
	b.eq	.LBB39_115
// %bb.113:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB39_115
// %bb.114:
	bl	__put_page
.LBB39_115:
	mov	x26, xzr
	b	.LBB39_107
.LBB39_116:
	bl	print_bdev_access_info
	adrp	x23, f2fs_dclient
	ldr	x0, [x23, :lo12:f2fs_dclient]
	cbz	x0, .LBB39_119
// %bb.117:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB39_119
// %bb.118:
	ldr	x0, [x23, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.f2fs_read_multi_pages
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.f2fs_read_multi_pages
	mov	w3, #3468
	bl	dsm_client_record
	ldr	x0, [x23, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB39_119:
	ldur	x23, [x29, #-64]        // 8-byte Folded Reload
	mov	x0, x23
	bl	f2fs_print_raw_sb_info
	mov	x0, x23
	bl	f2fs_print_ckpt_info
	mov	x0, x23
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 3468; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x23, .LBB39_1
// %bb.120:
	ldur	x11, [x29, #-64]        // 8-byte Folded Reload
	add	x8, x11, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x11, #3376]
	cbnz	w8, .LBB39_1
// %bb.121:
	ldur	x8, [x29, #-64]         // 8-byte Folded Reload
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB39_1
// %bb.122:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB39_1
// %bb.123:
	ldur	x8, [x29, #-64]         // 8-byte Folded Reload
	mov	w0, #8
	add	x2, x8, #3408           // =3408
	bl	queue_work_on
	b	.LBB39_1
.LBB39_124:
	bl	print_bdev_access_info
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	cbz	x0, .LBB39_127
// %bb.125:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB39_127
// %bb.126:
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.f2fs_read_multi_pages
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.f2fs_read_multi_pages
	mov	w3, #3502
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB39_127:
	ldur	x19, [x29, #-64]        // 8-byte Folded Reload
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 3502; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB39_31
// %bb.128:
	ldur	x11, [x29, #-64]        // 8-byte Folded Reload
	add	x8, x11, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x11, #3376]
	cbnz	w8, .LBB39_31
// %bb.129:
	ldur	x8, [x29, #-64]         // 8-byte Folded Reload
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB39_31
// %bb.130:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB39_31
// %bb.131:
	ldur	x8, [x29, #-64]         // 8-byte Folded Reload
	mov	w0, #8
	add	x2, x8, #3408           // =3408
	bl	queue_work_on
	b	.LBB39_31
.LBB39_132:
	bl	__stack_chk_fail
.Lfunc_end39:
	.size	f2fs_read_multi_pages, .Lfunc_end39-f2fs_read_multi_pages
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_grab_read_bio
	.type	f2fs_grab_read_bio,@function
f2fs_grab_read_bio:                     // @f2fs_grab_read_bio
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	tst	w5, #0x1
	mov	w9, #3264
	mov	w10, #3072
	mov	x20, x0
	ldr	x24, [x8, #1008]
	csel	w0, w10, w9, ne
	cmp	w2, #256                // =256
	mov	w8, #256
	mov	w23, w1
	csel	w1, w2, w8, lt
	adrp	x2, f2fs_bioset
	add	x2, x2, :lo12:f2fs_bioset
	mov	x29, sp
	mov	x21, x4
	mov	w22, w3
	bl	bio_alloc_bioset
	cbz	x0, .LBB40_11
// %bb.1:
	adrp	x8, f2fs_print_frag_info
	add	x8, x8, :lo12:f2fs_print_frag_info
	mov	w3, #3136
	mov	x1, x20
	mov	x2, x21
	mov	x19, x0
	str	x8, [x0, #344]
	bl	fscrypt_set_bio_crypt_ctx
	ldr	x8, [x24]
	ldr	w9, [x24, #3128]
	ldr	x8, [x8, #248]
	cmp	w9, #2                  // =2
	b.lt	.LBB40_7
// %bb.2:
	ldr	x10, [x24, #3136]
	add	x10, x10, #80           // =80
	b	.LBB40_4
.LBB40_3:                               //   in Loop: Header=BB40_4 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB40_7
.LBB40_4:                               // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w23
	b.hi	.LBB40_3
// %bb.5:                               //   in Loop: Header=BB40_4 Depth=1
	ldr	w12, [x10]
	cmp	w12, w23
	b.lo	.LBB40_3
// %bb.6:
	ldur	x8, [x10, #-80]
	sub	w23, w23, w11
.LBB40_7:
	ldr	x9, [x19, #8]
	ldr	x10, [x8, #128]
	cmp	x9, x10
	b.eq	.LBB40_9
// %bb.8:
	ldrh	w9, [x19, #20]
	and	w9, w9, #0xfffffdff
	strh	w9, [x19, #20]
	ldr	x9, [x8, #128]
.LBB40_9:
	str	x9, [x19, #8]
	ldrb	w8, [x8, #108]
	mov	x0, x19
	strb	w8, [x19, #27]
	bl	bio_associate_blkg
	mov	w8, w23
	adrp	x9, f2fs_read_end_io
	add	x9, x9, :lo12:f2fs_read_end_io
	lsl	x8, x8, #3
	mov	x0, x20
	str	x9, [x19, #64]
	str	x8, [x19, #32]
	str	w22, [x19, #16]
	bl	fscrypt_inode_uses_fs_layer_crypto
	ldr	x8, [x20, #664]
	cbz	x8, .LBB40_12
// %bb.10:
	ldr	x8, [x20, #80]
	mov	w9, #4
	bfxil	w9, w0, #0, #1
	add	x8, x8, #4095           // =4095
	lsr	x8, x8, #12
	cmp	x8, x21
	cset	w8, hi
	orr	w8, w0, w8
	csinc	w21, w9, wzr, hi
	tbnz	w8, #0, .LBB40_14
	b	.LBB40_16
.LBB40_11:
	mov	x19, #-12
	b	.LBB40_25
.LBB40_12:
	tbz	w0, #0, .LBB40_16
// %bb.13:
	mov	w21, #1
.LBB40_14:
	ldrsb	w8, [x20, #712]
	tbnz	w8, #31, .LBB40_23
.LBB40_15:
	adrp	x8, bio_post_read_ctx_pool
	ldr	x0, [x8, :lo12:bio_post_read_ctx_pool]
	mov	w1, #3136
	bl	mempool_alloc
	stp	x19, x24, [x0]
	str	w21, [x0, #64]
	str	x0, [x19, #72]
	str	xzr, [x0, #72]
	b	.LBB40_25
.LBB40_16:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB40_20
// %bb.17:
	ldr	w8, [x20, #1280]
	cbnz	w8, .LBB40_22
// %bb.18:
	ldr	x8, [x20, #736]
	tbnz	x8, #35, .LBB40_22
// %bb.19:
	ldr	x8, [x20, #736]
	tbnz	x8, #34, .LBB40_22
	b	.LBB40_25
.LBB40_20:
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB40_25
// %bb.21:
	ldr	x8, [x20, #736]
	tbz	w8, #31, .LBB40_25
.LBB40_22:
	mov	w21, wzr
	ldrsb	w8, [x20, #712]
	tbz	w8, #31, .LBB40_15
.LBB40_23:
	ldr	x8, [x20, #656]
	cbnz	x8, .LBB40_25
// %bb.24:
	mov	x0, x19
	bl	bio_put
	mov	x19, #-126
.LBB40_25:
	mov	x0, x19
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end40:
	.size	f2fs_grab_read_bio, .Lfunc_end40-f2fs_grab_read_bio
                                        // -- End function
	.globl	f2fs_mpage_readpages    // -- Begin function f2fs_mpage_readpages
	.p2align	2
	.type	f2fs_mpage_readpages,@function
f2fs_mpage_readpages:                   // @f2fs_mpage_readpages
.Lf2fs_mpage_readpages$local:
// %bb.0:
	sub	sp, sp, #368            // =368
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #272]    // 16-byte Folded Spill
	add	x29, sp, #272           // =272
	stp	x28, x27, [sp, #288]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #304]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #320]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #336]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #352]    // 16-byte Folded Spill
	stur	x8, [x29, #-16]
	stp	xzr, xzr, [x29, #-32]
	ldr	x19, [x0]
	mov	w9, #-1
	stp	xzr, xzr, [x29, #-48]
	stp	xzr, xzr, [x29, #-64]
	stur	xzr, [x29, #-72]
	stp	xzr, xzr, [sp, #136]
	stp	xzr, xzr, [sp, #120]
	stp	x9, xzr, [sp, #104]
	str	x19, [sp, #96]
	ldr	w8, [x19, #1292]
	mov	w21, w4
	mov	w20, w3
	mov	x24, x2
	str	w8, [sp, #112]
	ldrb	w8, [x19, #1285]
	mov	x25, x1
	mov	w9, #8
	str	x0, [sp, #80]           // 8-byte Folded Spill
	stur	w9, [x29, #-40]
	str	xzr, [sp, #120]
	str	wzr, [sp, #128]
	str	xzr, [sp, #136]
	str	wzr, [sp, #144]
	stp	xzr, xzr, [sp, #152]
	stp	xzr, xzr, [sp, #168]
	stp	xzr, xzr, [sp, #184]
	stp	xzr, xzr, [x29, #-72]
	stp	xzr, xzr, [x29, #-56]
	str	w8, [sp, #116]
	sturb	wzr, [x29, #-36]
	cbz	x1, .LBB41_2
// %bb.1:
	ldr	x8, [x19, #40]
	mov	w1, #4096
	mov	w2, #25
	mov	w3, #1
	ldr	x0, [x8, #248]
	bl	blk_throtl_get_quota
.LBB41_2:
	cbz	w20, .LBB41_116
// %bb.3:
	add	x8, x19, #1300          // =1300
	str	x8, [sp, #88]           // 8-byte Folded Spill
	add	x26, x19, #744          // =744
	tst	w21, #0x1
	mov	w8, #1048576
	add	x9, x19, #1312          // =1312
	mov	x23, xzr
	stp	x9, x26, [sp, #56]      // 16-byte Folded Spill
	add	x9, x19, #1320          // =1320
	mov	w27, w20
	csel	w8, w8, wzr, ne
	str	xzr, [sp, #24]          // 8-byte Folded Spill
	str	wzr, [sp, #52]          // 4-byte Folded Spill
	stp	w21, w20, [sp, #32]     // 8-byte Folded Spill
	str	x9, [sp, #40]           // 8-byte Folded Spill
	//APP
	mrs x20, sp_el0
	//NO_APP
	str	w8, [sp, #20]           // 4-byte Folded Spill
	str	x25, [sp, #72]          // 8-byte Folded Spill
	b	.LBB41_5
.LBB41_4:                               //   in Loop: Header=BB41_5 Depth=1
	cmp	w27, #1                 // =1
	sub	x27, x27, #1            // =1
	b.eq	.LBB41_119
.LBB41_5:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB41_86 Depth 2
	cbz	x25, .LBB41_12
// %bb.6:                               //   in Loop: Header=BB41_5 Depth=1
	ldr	x21, [x25, #8]
	mov	x0, x21
	sub	x24, x21, #8            // =8
	//APP
	prfm pstl1keep, [x24]

	//NO_APP
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB41_8
// %bb.7:                               //   in Loop: Header=BB41_5 Depth=1
	ldp	x9, x8, [x21]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB41_8:                               //   in Loop: Header=BB41_5 Depth=1
	mov	x9, #256
	movk	x9, #57005, lsl #48
	add	x8, x9, #34             // =34
	stp	x9, x8, [x21]
	ldr	x8, [x21]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x24, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB41_10
// %bb.9:                               //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x24]
	tbnz	w8, #10, .LBB41_99
.LBB41_10:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x2, [x21, #24]
.LBB41_11:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x1, [sp, #80]           // 8-byte Folded Reload
	mov	w9, #8192
	mov	w10, #8192
	movk	w9, #257, lsl #16
	ldr	w8, [x1, #24]
	movk	w10, #1, lsl #16
	mov	x0, x24
	tst	w8, #0x8
	csel	w9, w10, w9, eq
	orr	w3, w9, w8
	bl	add_to_page_cache_lru
	cbnz	w0, .LBB41_62
.LBB41_12:                              //   in Loop: Header=BB41_5 Depth=1
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB41_14
// %bb.13:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x19, #736]
	tbnz	w8, #31, .LBB41_34
.LBB41_14:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x10, [sp, #88]          // 8-byte Folded Reload
	//APP
	// atomic_add
	prfm	pstl1strm, [x10]
1:	ldxr	w8, [x10]
	add	w8, w8, 1
	stxr	w9, w8, [x10]
	cbnz	w9, 1b

	//NO_APP
	mov	x0, x26
	bl	down_read
	ldr	x8, [x19, #736]
	tbnz	x8, #38, .LBB41_16
.LBB41_15:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x22, xzr
	b	.LBB41_19
.LBB41_16:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x19, #736]
	tbz	x8, #38, .LBB41_18
// %bb.17:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x19, #736]
	tbnz	x8, #39, .LBB41_15
.LBB41_18:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x0, [x19, #1304]
	bl	igrab
	mov	x22, x0
.LBB41_19:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x26
	bl	up_read
	cmp	x22, #0                 // =0
	csel	x28, x22, x19, ne
	cmp	x23, #0                 // =0
	csel	x8, x28, x23, eq
	cmp	x28, x8
	b.ne	.LBB41_98
.LBB41_20:                              //   in Loop: Header=BB41_5 Depth=1
	ldur	x21, [x29, #-24]
	ldrb	w8, [x28, #142]
	ldr	x9, [x24, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x9, x24, x10, eq
	ldr	x9, [x9]
	mov	w10, #1
	lsl	x25, x10, x8
	tbz	w9, #19, .LBB41_22
// %bb.21:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x24]
	tbnz	w8, #10, .LBB41_100
.LBB41_22:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x26, [x24, #32]
.LBB41_23:                              //   in Loop: Header=BB41_5 Depth=1
	ldrb	w9, [x28, #14]
	add	x8, x26, x27
	tbnz	w9, #0, .LBB41_26
// %bb.24:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x9, [x28, #736]
	tbnz	w9, #30, .LBB41_26
// %bb.25:                              //   in Loop: Header=BB41_5 Depth=1
	add	x9, x28, #80            // =80
	b	.LBB41_27
.LBB41_26:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x9, [x28, #40]
	add	x9, x9, #32             // =32
.LBB41_27:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x9, [x9]
	ldrb	w10, [x28, #142]
	add	x9, x9, w25, uxtw
	sub	x9, x9, #1              // =1
	lsr	x9, x9, x10
	cmp	x8, x9
	csel	x8, x9, x8, hi
	cmp	x8, x26
	b.ls	.LBB41_41
// %bb.28:                              //   in Loop: Header=BB41_5 Depth=1
	ldurb	w9, [x29, #-60]
	tbz	w9, #5, .LBB41_31
// %bb.29:                              //   in Loop: Header=BB41_5 Depth=1
	ldur	w23, [x29, #-68]
	cmp	x26, x23
	b.ls	.LBB41_31
// %bb.30:                              //   in Loop: Header=BB41_5 Depth=1
	ldur	w9, [x29, #-64]
	add	w9, w9, w23
	cmp	x26, x9
	b.lo	.LBB41_69
.LBB41_31:                              //   in Loop: Header=BB41_5 Depth=1
	sub	w8, w8, w26
	sub	x1, x29, #72            // =72
	mov	x0, x28
	mov	w2, wzr
	mov	w3, wzr
	stp	w26, w8, [x29, #-68]
	bl	f2fs_map_blocks
	cbz	w0, .LBB41_40
// %bb.32:                              //   in Loop: Header=BB41_5 Depth=1
	mov	w23, w0
.LBB41_33:                              //   in Loop: Header=BB41_5 Depth=1
	ldp	x26, x25, [sp, #64]     // 16-byte Folded Reload
	stur	x21, [x29, #-24]
	cbnz	x22, .LBB41_51
	b	.LBB41_52
.LBB41_34:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x1, [x24, #32]
	add	x0, sp, #96             // =96
	bl	f2fs_cluster_can_merge_page
	tbnz	w0, #0, .LBB41_36
// %bb.35:                              //   in Loop: Header=BB41_5 Depth=1
	ldp	w8, w2, [sp, #32]       // 8-byte Folded Reload
	add	x0, sp, #96             // =96
	sub	x1, x29, #24            // =24
	sub	x3, x29, #32            // =32
	and	w4, w8, #0x1
	mov	w5, wzr
	bl	f2fs_read_multi_pages
	mov	w21, w0
	add	x0, sp, #96             // =96
	mov	w1, wzr
	bl	f2fs_destroy_compress_ctx
	cbnz	w21, .LBB41_39
.LBB41_36:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x1, [x24, #32]
	mov	x0, x19
	bl	f2fs_is_compressed_cluster
	tbnz	w0, #31, .LBB41_39
// %bb.37:                              //   in Loop: Header=BB41_5 Depth=1
	cbz	w0, .LBB41_14
// %bb.38:                              //   in Loop: Header=BB41_5 Depth=1
	add	x0, sp, #96             // =96
	bl	f2fs_init_compress_ctx
	cbz	w0, .LBB41_95
.LBB41_39:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x28, x23
	b	.LBB41_58
.LBB41_40:                              //   in Loop: Header=BB41_5 Depth=1
	ldurb	w8, [x29, #-60]
	tbnz	w8, #5, .LBB41_68
.LBB41_41:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	w8, [x20, #24]
	mov	x9, #-549755813888
	mov	w2, #4096
	mov	w1, wzr
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	ldr	w8, [x20, #2800]
	add	w8, w8, #1              // =1
	str	w8, [x20, #2800]
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x8, x24, x8
	//APP
	//NO_APP
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	add	x0, x8, x9
	bl	memset
	//APP
	//NO_APP
	ldr	w8, [x20, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x20, #2800]
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB41_97
// %bb.42:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x20, #24]
	cbz	x8, .LBB41_97
.LBB41_43:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x24
	bl	flush_dcache_page
	ldr	x8, [x24, #32]
	ldr	x9, [x28, #664]
	ldp	x26, x25, [sp, #64]     // 16-byte Folded Reload
	cbz	x9, .LBB41_46
// %bb.44:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x9, [x28, #80]
	add	x9, x9, #4095           // =4095
	cmp	x8, x9, lsr #12
	b.hs	.LBB41_46
// %bb.45:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x24
	bl	fsverity_verify_page
	tbz	w0, #0, .LBB41_67
.LBB41_46:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x24, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x24, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB41_48
// %bb.47:                              //   in Loop: Header=BB41_5 Depth=1
	//APP
	dmb ishst
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x24]
1:	ldxr	x8, [x24]
	orr	x8, x8, 4
	stxr	w9, x8, [x24]
	cbnz	w9, 1b
	//NO_APP
	b	.LBB41_49
.LBB41_48:                              //   in Loop: Header=BB41_5 Depth=1
	//APP
	dmb ishld
	//NO_APP
.LBB41_49:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x24
	bl	unlock_page
.LBB41_50:                              //   in Loop: Header=BB41_5 Depth=1
	mov	w23, wzr
	stur	x21, [x29, #-24]
	cbz	x22, .LBB41_52
.LBB41_51:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x22
	bl	put_inner_inode
.LBB41_52:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	w8, [x19, #1300]
	cbz	w8, .LBB41_55
// %bb.53:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x10, [sp, #88]          // 8-byte Folded Reload
	mov	w11, #1
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x10]
1:	ldxr	w8, [x10]
	sub	w8, w8, w11
	stlxr	w9, w8, [x10]
	cbnz	w9, 1b
	dmb ish
	//NO_APP
	cbnz	w8, .LBB41_57
// %bb.54:                              //   in Loop: Header=BB41_5 Depth=1
	//APP
	dmb ish
	//NO_APP
	ldr	x9, [sp, #40]           // 8-byte Folded Reload
	ldr	x8, [x9]
	cmp	x9, x8
	b.ne	.LBB41_56
	b	.LBB41_57
.LBB41_55:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x19, #40]
	ldr	x2, [x19, #64]
	adrp	x1, .L.str.13
	add	x1, x1, :lo12:.L.str.13
	ldr	x0, [x8, #1008]
	bl	f2fs_printk
.LBB41_56:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	mov	w1, #3
	mov	w2, wzr
	mov	x3, xzr
	bl	__wake_up
.LBB41_57:                              //   in Loop: Header=BB41_5 Depth=1
	cbz	w23, .LBB41_61
.LBB41_58:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x24, #8]
	mov	w2, #4096
	mov	w1, wzr
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x24, x9, eq
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 256
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	w8, [x20, #24]
	mov	x9, #-549755813888
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	ldr	w8, [x20, #2800]
	add	w8, w8, #1              // =1
	str	w8, [x20, #2800]
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x8, x24, x8
	//APP
	//NO_APP
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	add	x0, x8, x9
	bl	memset
	//APP
	//NO_APP
	ldr	w8, [x20, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x20, #2800]
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB41_96
// %bb.59:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x20, #24]
	cbz	x8, .LBB41_96
.LBB41_60:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x24
	bl	flush_dcache_page
	mov	x0, x24
	bl	unlock_page
.LBB41_61:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x23, x28
	cbz	x25, .LBB41_64
.LBB41_62:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x24, #8]
	mov	w11, #1
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x24, x9, eq
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
	cbnz	w9, .LBB41_64
// %bb.63:                              //   in Loop: Header=BB41_5 Depth=1
	bl	__put_page
.LBB41_64:                              //   in Loop: Header=BB41_5 Depth=1
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB41_4
// %bb.65:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x19, #736]
	cmp	x27, #1                 // =1
	b.ne	.LBB41_4
// %bb.66:                              //   in Loop: Header=BB41_5 Depth=1
	tbz	w8, #31, .LBB41_4
	b	.LBB41_117
.LBB41_67:                              //   in Loop: Header=BB41_5 Depth=1
	mov	w23, #-12
	stur	x21, [x29, #-24]
	cbnz	x22, .LBB41_51
	b	.LBB41_52
.LBB41_68:                              //   in Loop: Header=BB41_5 Depth=1
	ldur	w23, [x29, #-68]
.LBB41_69:                              //   in Loop: Header=BB41_5 Depth=1
	ldur	w11, [x29, #-72]
	ldr	x8, [x24, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x24, x9, eq
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 131072
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x24, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x24, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB41_77
// %bb.70:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x24, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x24, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB41_72
// %bb.71:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x24]
	tbnz	w8, #10, .LBB41_78
.LBB41_72:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x24, #24]
	ldr	x8, [x8]
	ldr	x8, [x8, #40]
	ldr	w8, [x8, #1160]
	tbnz	w8, #31, .LBB41_78
// %bb.73:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x24
	str	x11, [sp, #8]           // 8-byte Folded Spill
	bl	__cleancache_get_page
	ldr	x11, [sp, #8]           // 8-byte Folded Reload
	cbnz	w0, .LBB41_78
// %bb.74:                              //   in Loop: Header=BB41_5 Depth=1
	//APP
	dmb ishst
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x24]
1:	ldxr	x8, [x24]
	orr	x8, x8, 4
	stxr	w9, x8, [x24]
	cbnz	w9, 1b
	//NO_APP
	cbz	x21, .LBB41_76
// %bb.75:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x28, #40]
	mov	x1, x21
	mov	w2, wzr
	ldr	x0, [x8, #1008]
	ldr	w8, [sp, #52]           // 4-byte Folded Reload
	add	w8, w8, #1              // =1
	str	w8, [sp, #52]           // 4-byte Folded Spill
	bl	__f2fs_submit_read_bio
.LBB41_76:                              //   in Loop: Header=BB41_5 Depth=1
	ldp	x26, x25, [sp, #64]     // 16-byte Folded Reload
	mov	x0, x24
	bl	unlock_page
	mov	x21, xzr
	b	.LBB41_50
.LBB41_77:                              //   in Loop: Header=BB41_5 Depth=1
	//APP
	dmb ishld
	//NO_APP
.LBB41_78:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x28, #40]
	mov	w2, #8
	ldr	x0, [x8, #1008]
	add	x8, x26, x11
	sub	x23, x8, x23
	mov	w1, w23
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB41_115
// %bb.79:                              //   in Loop: Header=BB41_5 Depth=1
	cbz	x21, .LBB41_102
// %bb.80:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x28, #40]
	ldr	x10, [x8, #1008]
	ldur	w8, [x29, #-32]
	ldr	x9, [x10, #1224]
	cbz	x9, .LBB41_82
// %bb.81:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	w11, [x21, #40]
	cmp	x9, x11
	b.ls	.LBB41_101
.LBB41_82:                              //   in Loop: Header=BB41_5 Depth=1
	add	w8, w8, #1              // =1
	cmp	w8, w23
	b.ne	.LBB41_101
// %bb.83:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x10]
	ldr	w9, [x10, #3128]
	ldr	x8, [x8, #248]
	cmp	w9, #2                  // =2
	b.lt	.LBB41_89
// %bb.84:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x10, [x10, #3136]
	add	x10, x10, #80           // =80
	b	.LBB41_86
.LBB41_85:                              //   in Loop: Header=BB41_86 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB41_89
.LBB41_86:                              //   Parent Loop BB41_5 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	w11, w23
	b.hi	.LBB41_85
// %bb.87:                              //   in Loop: Header=BB41_86 Depth=2
	ldr	w11, [x10]
	cmp	w11, w23
	b.lo	.LBB41_85
// %bb.88:                              //   in Loop: Header=BB41_5 Depth=1
	ldur	x8, [x10, #-80]
.LBB41_89:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x9, [x21, #8]
	ldr	x10, [x8, #128]
	cmp	x9, x10
	b.ne	.LBB41_101
// %bb.90:                              //   in Loop: Header=BB41_5 Depth=1
	ldrb	w9, [x21, #27]
	ldrb	w8, [x8, #108]
	cmp	w9, w8
	b.ne	.LBB41_101
// %bb.91:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x2, [x24, #32]
	mov	x0, x21
	mov	x1, x28
	bl	fscrypt_mergeable_bio
	tbz	w0, #0, .LBB41_101
// %bb.92:                              //   in Loop: Header=BB41_5 Depth=1
	ldrb	w8, [x28, #13]
	tbz	w8, #6, .LBB41_102
// %bb.93:                              //   in Loop: Header=BB41_5 Depth=1
	ldr	x9, [sp, #24]           // 8-byte Folded Reload
	sub	x8, x26, #1             // =1
	cmp	x9, x8
	b.eq	.LBB41_102
// %bb.94:                              //   in Loop: Header=BB41_5 Depth=1
	ldrsb	w8, [x28, #712]
	tbnz	w8, #31, .LBB41_101
	b	.LBB41_102
.LBB41_95:                              //   in Loop: Header=BB41_5 Depth=1
	add	x0, sp, #96             // =96
	mov	x1, x24
	bl	f2fs_compress_ctx_add_page
	mov	x28, x23
	mov	x23, x28
	cbnz	x25, .LBB41_62
	b	.LBB41_64
.LBB41_96:                              //   in Loop: Header=BB41_5 Depth=1
	bl	preempt_schedule
	b	.LBB41_60
.LBB41_97:                              //   in Loop: Header=BB41_5 Depth=1
	bl	preempt_schedule
	b	.LBB41_43
.LBB41_98:                              //   in Loop: Header=BB41_5 Depth=1
	sub	x0, x29, #72            // =72
	mov	w2, #40
	mov	w1, wzr
	bl	memset
	mov	w8, #8
	stur	w8, [x29, #-40]
	b	.LBB41_20
.LBB41_99:                              //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x24
	bl	__page_file_index
	mov	x2, x0
	b	.LBB41_11
.LBB41_100:                             //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x24
	bl	__page_file_index
	mov	x26, x0
	b	.LBB41_23
.LBB41_101:                             //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x28, #40]
	mov	x1, x21
	mov	w2, wzr
	ldr	x0, [x8, #1008]
	ldr	w8, [sp, #52]           // 4-byte Folded Reload
	add	w8, w8, #1              // =1
	str	w8, [sp, #52]           // 4-byte Folded Spill
	bl	__f2fs_submit_read_bio
	mov	x21, xzr
.LBB41_102:                             //   in Loop: Header=BB41_5 Depth=1
	cbz	x21, .LBB41_107
.LBB41_103:                             //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x28
	mov	w1, w23
	bl	f2fs_wait_on_block_writeback
	mov	x0, x21
	mov	x1, x24
	mov	w2, w25
	mov	w3, wzr
	bl	bio_add_page
	cmp	w0, w25
	b.lo	.LBB41_101
// %bb.104:                             //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x28, #40]
	str	x23, [sp, #8]           // 8-byte Folded Spill
	ldr	x8, [x8, #1008]
	add	x8, x8, #1308           // =1308
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x28, #40]
	ldr	x23, [x8, #1008]
	ldrb	w8, [x23, #2704]
	cbz	w8, .LBB41_106
// %bb.105:                             //   in Loop: Header=BB41_5 Depth=1
	add	x25, x23, #2344         // =2344
	mov	x0, x25
	bl	_raw_spin_lock
	ldr	x8, [x23, #2480]
	mov	x0, x25
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x23, #2480]
	bl	_raw_spin_unlock
	mov	x0, x23
	bl	f2fs_record_iostat
.LBB41_106:                             //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x24, #8]
	mov	w11, #256
	mov	w23, wzr
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x24, x9, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x11
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	str	x26, [sp, #24]          // 8-byte Folded Spill
	stur	x8, [x29, #-32]
	b	.LBB41_33
.LBB41_107:                             //   in Loop: Header=BB41_5 Depth=1
	ldr	x4, [x24, #32]
	ldr	w3, [sp, #20]           // 4-byte Folded Reload
	mov	x0, x28
	mov	w1, w23
	mov	w2, w27
	mov	w5, wzr
	bl	f2fs_grab_read_bio
	mov	x21, x0
	cmn	x0, #4095               // =4095
	b.lo	.LBB41_109
// %bb.108:                             //   in Loop: Header=BB41_5 Depth=1
	mov	w23, w21
	mov	x21, xzr
	b	.LBB41_33
.LBB41_109:                             //   in Loop: Header=BB41_5 Depth=1
	ldr	x8, [x28, #40]
	mov	x1, x28
	ldr	x0, [x8, #1008]
	bl	f2fs_inode_support_dedup
	tbz	w0, #0, .LBB41_103
// %bb.110:                             //   in Loop: Header=BB41_5 Depth=1
	str	x23, [sp, #8]           // 8-byte Folded Spill
	ldr	x23, [x21, #72]
	cbnz	x23, .LBB41_113
// %bb.111:                             //   in Loop: Header=BB41_5 Depth=1
	adrp	x8, bio_post_read_ctx_pool
	ldr	x0, [x8, :lo12:bio_post_read_ctx_pool]
	mov	w1, #3136
	bl	mempool_alloc
	cbz	x0, .LBB41_114
// %bb.112:                             //   in Loop: Header=BB41_5 Depth=1
	mov	w2, #80
	mov	w1, wzr
	mov	x23, x0
	bl	memset
	str	x21, [x23]
	ldr	x8, [x28, #40]
	ldr	x8, [x8, #1008]
	str	x8, [x23, #8]
	str	x23, [x21, #72]
.LBB41_113:                             //   in Loop: Header=BB41_5 Depth=1
	ldr	w8, [x23, #64]
	str	x28, [x23, #72]
	orr	w8, w8, #0x8
	str	w8, [x23, #64]
	add	x8, x28, #1300          // =1300
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x23, [sp, #8]           // 8-byte Folded Reload
	b	.LBB41_103
.LBB41_114:                             //   in Loop: Header=BB41_5 Depth=1
	mov	x0, x21
	bl	bio_put
	mov	x21, xzr
.LBB41_115:                             //   in Loop: Header=BB41_5 Depth=1
	mov	w23, #-12
	b	.LBB41_33
.LBB41_116:
	str	wzr, [sp, #52]          // 4-byte Folded Spill
	b	.LBB41_119
.LBB41_117:
	add	x0, sp, #96             // =96
	bl	f2fs_cluster_is_empty
	tbnz	w0, #0, .LBB41_119
// %bb.118:
	ldp	w8, w2, [sp, #32]       // 8-byte Folded Reload
	add	x0, sp, #96             // =96
	sub	x1, x29, #24            // =24
	sub	x3, x29, #32            // =32
	and	w4, w8, #0x1
	mov	w5, wzr
	bl	f2fs_read_multi_pages
	add	x0, sp, #96             // =96
	mov	w1, wzr
	bl	f2fs_destroy_compress_ctx
.LBB41_119:
	cbz	x25, .LBB41_121
// %bb.120:
	ldr	x8, [x25]
	cmp	x8, x25
	b.ne	.LBB41_130
.LBB41_121:
	ldur	x1, [x29, #-24]
	ldr	w20, [sp, #52]          // 4-byte Folded Reload
	cbz	x1, .LBB41_123
// %bb.122:
	ldr	x8, [x19, #40]
	mov	w2, wzr
	add	w20, w20, #1            // =1
	ldr	x0, [x8, #1008]
	bl	__f2fs_submit_read_bio
.LBB41_123:
	cbz	w20, .LBB41_127
// %bb.124:
	subs	w20, w20, #1            // =1
	b.eq	.LBB41_127
// %bb.125:
	ldr	x8, [x19, #40]
	ldr	x19, [x8, #248]
.LBB41_126:                             // =>This Inner Loop Header: Depth=1
	mov	w1, #4096
	mov	w2, #25
	mov	w3, #1
	mov	x0, x19
	sub	w20, w20, #1            // =1
	bl	blk_throtl_get_quota
	cbnz	w20, .LBB41_126
.LBB41_127:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB41_129
// %bb.128:
	ldp	x20, x19, [sp, #352]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #336]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #320]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #304]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #288]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #272]    // 16-byte Folded Reload
	mov	w0, wzr
	add	sp, sp, #368            // =368
	ret
.LBB41_129:
	bl	__stack_chk_fail
.LBB41_130:
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 3910; .short 0; .popsection; 14471: brk 0x800
	//NO_APP
.Lfunc_end41:
	.size	f2fs_mpage_readpages, .Lfunc_end41-f2fs_mpage_readpages
                                        // -- End function
	.globl	f2fs_encrypt_one_page   // -- Begin function f2fs_encrypt_one_page
	.p2align	2
	.type	f2fs_encrypt_one_page,@function
f2fs_encrypt_one_page:                  // @f2fs_encrypt_one_page
.Lf2fs_encrypt_one_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	ldr	x9, [x8, #24]
	ldr	x21, [x9]
	ldrb	w9, [x21, #13]
	tbz	w9, #6, .LBB42_13
// %bb.1:
	ldrh	w9, [x21]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB42_13
// %bb.2:
	ldr	x9, [x0, #56]
	ldr	w1, [x0, #32]
	mov	x19, x0
	mov	x0, x21
	cmp	x9, #0                  // =0
	csel	x20, x8, x9, eq
	bl	f2fs_wait_on_block_writeback
	mov	x0, x21
	bl	fscrypt_inode_uses_inline_crypto
	tbnz	w0, #0, .LBB42_13
// %bb.3:
	ldrsb	w8, [x21, #712]
	tbnz	w8, #31, .LBB42_12
// %bb.4:
	mov	w1, #4096
	mov	w3, #3136
	mov	x0, x20
	mov	w2, wzr
	bl	fscrypt_encrypt_pagecache_blocks
	cmn	x0, #4095               // =4095
	str	x0, [x19, #48]
	b.lo	.LBB42_8
// %bb.5:
	mov	w21, #3136
.LBB42_6:                               // =>This Inner Loop Header: Depth=1
	cmn	x0, #12                 // =12
	b.ne	.LBB42_14
// %bb.7:                               //   in Loop: Header=BB42_6 Depth=1
	ldr	x0, [x19]
	bl	f2fs_flush_merged_writes
	mov	w1, #5
	mov	w0, wzr
	bl	congestion_wait
	orr	w21, w21, #0x8000
	mov	w1, #4096
	mov	x0, x20
	mov	w2, wzr
	mov	w3, w21
	bl	fscrypt_encrypt_pagecache_blocks
	cmn	x0, #1, lsl #12         // =4096
	str	x0, [x19, #48]
	b.hi	.LBB42_6
.LBB42_8:
	ldr	x8, [x19]
	ldr	w1, [x19, #32]
	mov	w2, #2
	mov	w3, wzr
	ldr	x8, [x8, #248]
	ldr	x0, [x8, #48]
	bl	pagecache_get_page
	cbz	x0, .LBB42_13
// %bb.9:
	ldr	x8, [x0, #8]
	mov	x20, x0
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x0, x9, eq
	ldr	x8, [x8]
	tbz	w8, #2, .LBB42_11
// %bb.10:
	//APP
	dmb ishld
	//NO_APP
	ldr	x10, [x19, #48]
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x9, x20, x8
	add	x8, x10, x8
	lsl	x9, x9, #6
	lsl	x8, x8, #6
	and	x9, x9, #0xfffffffffffff000
	mov	x11, #-549755813888
	and	x8, x8, #0xfffffffffffff000
	add	x0, x9, x11
	add	x1, x8, x11
	mov	w2, #4096
	bl	memcpy
.LBB42_11:
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
	b	.LBB42_13
.LBB42_12:
	ldr	x8, [x21, #656]
	cbz	x8, .LBB42_15
.LBB42_13:
	mov	w0, wzr
.LBB42_14:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
                                        // kill: def $w0 killed $w0 killed $x0
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB42_15:
	mov	w0, #-126
	b	.LBB42_14
.Lfunc_end42:
	.size	f2fs_encrypt_one_page, .Lfunc_end42-f2fs_encrypt_one_page
                                        // -- End function
	.globl	f2fs_should_update_inplace // -- Begin function f2fs_should_update_inplace
	.p2align	2
	.type	f2fs_should_update_inplace,@function
f2fs_should_update_inplace:             // @f2fs_should_update_inplace
.Lf2fs_should_update_inplace$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #736]
	mov	x29, sp
	tbnz	x8, #36, .LBB43_28
// %bb.1:
	ldr	x8, [x0, #736]
	mov	x21, x0
	tbnz	w8, #28, .LBB43_24
// %bb.2:
	ldrb	w8, [x21, #712]
	tbnz	w8, #0, .LBB43_18
// %bb.3:
	ldr	x8, [x21, #40]
	ldr	x20, [x8, #1008]
	ldr	x8, [x20, #136]
	ldr	w22, [x8, #136]
	tbnz	w22, #0, .LBB43_24
// %bb.4:
	mov	x19, x1
	tbz	w22, #1, .LBB43_6
// %bb.5:
	mov	x0, x20
	bl	f2fs_need_SSR
	tbnz	w0, #0, .LBB43_24
.LBB43_6:
	tbz	w22, #2, .LBB43_8
// %bb.7:
	ldr	x8, [x20, #136]
	ldr	w9, [x20, #1240]
	ldr	w10, [x20, #1236]
	mov	w11, #100
	ldr	w8, [x8, #140]
	mul	x9, x9, x11
	udiv	x9, x9, x10
	cmp	w8, w9
	b.lo	.LBB43_24
.LBB43_8:
	tbz	w22, #3, .LBB43_11
// %bb.9:
	mov	x0, x20
	bl	f2fs_need_SSR
	tbz	w0, #0, .LBB43_11
// %bb.10:
	ldr	x8, [x20, #136]
	ldr	w9, [x20, #1240]
	ldr	w10, [x20, #1236]
	mov	w11, #100
	ldr	w8, [x8, #140]
	mul	x9, x9, x11
	udiv	x9, x9, x10
	cmp	w8, w9
	b.lo	.LBB43_24
.LBB43_11:
	cbz	x19, .LBB43_16
// %bb.12:
	tbz	w22, #5, .LBB43_16
// %bb.13:
	ldr	w8, [x19, #20]
	cmp	w8, #1                  // =1
	b.ne	.LBB43_16
// %bb.14:
	ldrb	w8, [x19, #25]
	tbnz	w8, #3, .LBB43_16
// %bb.15:
	ldrb	w8, [x21, #13]
	tbz	w8, #6, .LBB43_24
.LBB43_16:
	tbnz	w22, #4, .LBB43_25
// %bb.17:
	cbnz	x19, .LBB43_27
	b	.LBB43_28
.LBB43_18:
	adrp	x8, __tracepoint_f2fs_cold_file_should_IPU+8
	ldr	w19, [x21, #64]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_cold_file_should_IPU+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB43_24
// %bb.19:
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
	tbz	w8, #0, .LBB43_24
// %bb.20:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_cold_file_should_IPU+32
	ldr	x21, [x8, :lo12:__tracepoint_f2fs_cold_file_should_IPU+32]
	cbz	x21, .LBB43_22
.LBB43_21:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	mov	w1, w19
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB43_21
.LBB43_22:
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB43_31
// %bb.23:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB43_31
.LBB43_24:
	mov	w8, #1
	b	.LBB43_29
.LBB43_25:
	ldr	x9, [x21, #736]
	ubfx	x8, x9, #14, #1
	tbnz	w9, #14, .LBB43_29
// %bb.26:
	cbz	x19, .LBB43_29
.LBB43_27:
	ldr	x8, [x20, #80]
	tbnz	w8, #8, .LBB43_30
.LBB43_28:
	mov	w8, wzr
.LBB43_29:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	and	w0, w8, #0x1
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB43_30:
	ldr	w1, [x19, #32]
	mov	x0, x20
	bl	f2fs_is_checkpointed_data
	eor	w8, w0, #0x1
	b	.LBB43_29
.LBB43_31:
	bl	preempt_schedule_notrace
	b	.LBB43_24
.Lfunc_end43:
	.size	f2fs_should_update_inplace, .Lfunc_end43-f2fs_should_update_inplace
                                        // -- End function
	.globl	f2fs_should_update_outplace // -- Begin function f2fs_should_update_outplace
	.p2align	2
	.type	f2fs_should_update_outplace,@function
f2fs_should_update_outplace:            // @f2fs_should_update_outplace
.Lf2fs_should_update_outplace$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x9, [x0, #40]
	mov	x8, x0
	mov	x29, sp
	ldr	x0, [x9, #1008]
	ldr	w9, [x0, #1504]
	cmp	w9, #1                  // =1
	b.eq	.LBB44_20
// %bb.1:
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #4, lsl #12         // =16384
	b.eq	.LBB44_20
// %bb.2:
	ldrb	w9, [x8, #12]
	tbnz	w9, #5, .LBB44_20
// %bb.3:
	ldr	x9, [x8, #736]
	tbnz	w9, #15, .LBB44_20
// %bb.4:
	ldr	x9, [x0, #80]
	tbnz	w9, #2, .LBB44_20
// %bb.5:
	ldr	x9, [x8, #736]
	tbz	x9, #38, .LBB44_7
// %bb.6:
	ldr	x9, [x8, #736]
	tbnz	x9, #41, .LBB44_20
.LBB44_7:
	cbz	x1, .LBB44_19
// %bb.8:
	ldr	w9, [x1, #92]
	cbnz	w9, .LBB44_20
// %bb.9:
	ldr	x8, [x8, #736]
	tbnz	x8, #36, .LBB44_20
// %bb.10:
	ldr	x8, [x1, #40]
	ldr	x9, [x8]
	tbz	w9, #13, .LBB44_13
// %bb.11:
	ldr	x9, [x8, #40]
	tbz	w9, #0, .LBB44_13
// %bb.12:
	ldr	x9, [x8, #40]
	tbnz	w9, #3, .LBB44_20
.LBB44_13:
	ldr	x9, [x8]
	tbz	w9, #13, .LBB44_16
// %bb.14:
	ldr	x9, [x8, #40]
	tbz	w9, #0, .LBB44_16
// %bb.15:
	ldr	x8, [x8, #40]
	tbnz	w8, #2, .LBB44_20
.LBB44_16:
	ldr	x8, [x0, #80]
	tbz	w8, #8, .LBB44_18
// %bb.17:
	ldr	w1, [x1, #32]
	bl	f2fs_is_checkpointed_data
	tbnz	w0, #0, .LBB44_20
.LBB44_18:
	mov	w0, wzr
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB44_19:
	ldr	x8, [x8, #736]
	tbz	x8, #36, .LBB44_18
.LBB44_20:
	mov	w0, #1
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end44:
	.size	f2fs_should_update_outplace, .Lfunc_end44-f2fs_should_update_outplace
                                        // -- End function
	.globl	f2fs_do_write_data_page // -- Begin function f2fs_do_write_data_page
	.p2align	2
	.type	f2fs_do_write_data_page,@function
f2fs_do_write_data_page:                // @f2fs_do_write_data_page
.Lf2fs_do_write_data_page$local:
// %bb.0:
	sub	sp, sp, #160            // =160
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #96]     // 16-byte Folded Spill
	add	x29, sp, #96            // =96
	stp	x24, x23, [sp, #112]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #128]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #144]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x20, [x0, #40]
	mov	x19, x0
	add	x0, sp, #40             // =40
	mov	w2, #48
	ldr	x8, [x20, #24]
	mov	w1, wzr
	ldr	x21, [x8]
	stp	xzr, xzr, [sp, #72]
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	str	wzr, [sp, #32]
	stp	xzr, xzr, [sp, #16]
	str	xzr, [sp, #8]
	ldr	x8, [x21, #40]
	ldr	x23, [x8, #1008]
	bl	memset
	stp	x21, xzr, [sp, #40]
	str	xzr, [sp, #56]
	str	wzr, [sp, #64]
	ldr	w8, [x19, #132]
	mov	x1, x19
	str	w8, [sp, #80]
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #24]
	ldr	x22, [x8]
	mov	x0, x22
	bl	f2fs_should_update_outplace
	tbnz	w0, #0, .LBB45_5
// %bb.1:
	mov	x0, x22
	mov	x1, x19
	bl	f2fs_should_update_inplace
	tbz	w0, #0, .LBB45_5
// %bb.2:
	ldr	x1, [x20, #32]
	add	x2, sp, #24             // =24
	mov	x0, x21
	bl	f2fs_lookup_extent_cache
	tbz	w0, #0, .LBB45_5
// %bb.3:
	ldr	w8, [x20, #32]
	ldr	w9, [sp, #32]
	ldr	w10, [sp, #24]
	ldr	x0, [x19]
	mov	w2, #7
	add	w8, w9, w8
	sub	w1, w8, w10
	str	w1, [x19, #32]
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB45_37
// %bb.4:
	ldr	w1, [x19, #32]
	mov	w22, #1
	str	w22, [x19, #84]
	b	.LBB45_11
.LBB45_5:
	ldr	w8, [x19, #84]
	cbnz	w8, .LBB45_7
// %bb.6:
	ldr	x8, [x19]
	add	x0, x8, #304            // =304
	bl	down_read_trylock
	cbz	w0, .LBB45_16
.LBB45_7:
	ldr	x1, [x20, #32]
	add	x0, sp, #40             // =40
	mov	w2, #1
	bl	f2fs_get_dnode_of_data
	mov	w22, w0
	cbz	w0, .LBB45_9
// %bb.8:
	ldr	w8, [x19, #84]
	cbnz	w8, .LBB45_102
	b	.LBB45_101
.LBB45_9:
	ldr	w1, [sp, #76]
	str	w1, [x19, #32]
	cbz	w1, .LBB45_33
// %bb.10:
	mov	w22, wzr
.LBB45_11:
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB45_43
.LBB45_12:
	tbz	w22, #0, .LBB45_17
.LBB45_13:
	ldrh	w8, [x21]
	add	x22, x23, #2296         // =2296
	mov	x0, x22
	and	w24, w8, #0xf000
	bl	mutex_lock
	ldr	x8, [x23, #2328]
	cmp	w24, #4, lsl #12        // =16384
	b.ne	.LBB45_38
// %bb.14:
	cbz	x8, .LBB45_40
// %bb.15:
	ldr	x9, [x8, #424]
	add	x9, x9, #1              // =1
	str	x9, [x8, #424]
	b	.LBB45_40
.LBB45_16:
	mov	w22, #-11
	b	.LBB45_102
.LBB45_17:
	ldr	w8, [x19, #32]
	add	w9, w8, #3              // =3
	cmp	w9, #4                  // =4
	b.hs	.LBB45_53
.LBB45_18:
	ldr	w8, [x19, #84]
	cmp	w8, #2                  // =2
	b.ne	.LBB45_21
// %bb.19:
	ldr	x8, [x19]
	add	x0, x8, #304            // =304
	bl	down_read_trylock
	cbz	w0, .LBB45_63
// %bb.20:
	str	wzr, [x19, #84]
.LBB45_21:
	ldr	x0, [x19]
	ldr	w1, [sp, #64]
	add	x2, sp, #8              // =8
	mov	w3, wzr
	bl	get_node_info_ex
	mov	w22, w0
	cbnz	w0, .LBB45_94
// %bb.22:
	ldrb	w8, [sp, #20]
	mov	x0, x19
	strb	w8, [x19, #128]
	bl	f2fs_encrypt_one_page
	mov	w22, w0
	cbnz	w0, .LBB45_94
// %bb.23:
	mov	x0, x20
	mov	w1, wzr
	bl	__test_set_page_writeback
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	mov	w9, #256
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	w8, [x19, #92]
	cbz	w8, .LBB45_29
// %bb.24:
	ldr	w9, [x19, #32]
	cmn	w9, #2                  // =2
	b.ne	.LBB45_29
// %bb.25:
	ldr	w9, [x21, #1292]
	ldr	w10, [x21, #1280]
	cbz	w10, .LBB45_29
// %bb.26:
	ldr	w10, [x21, #1280]
	sub	w8, w9, w8
	add	w22, w8, #1             // =1
	cmp	w22, w10
	b.le	.LBB45_28
// %bb.27:
	ldr	x8, [x21, #40]
	adrp	x1, .L.str.14
	adrp	x2, .L__func__.f2fs_i_compr_blocks_update
	add	x1, x1, :lo12:.L.str.14
	ldr	x0, [x8, #1008]
	ldr	w4, [x21, #1280]
	ldr	x5, [x21, #64]
	add	x2, x2, :lo12:.L__func__.f2fs_i_compr_blocks_update
	mov	w3, w22
	bl	f2fs_printk
.LBB45_28:
	add	x8, x21, #1280          // =1280
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w22
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x21, #40]
	sxtw	x9, w22
	mov	w1, #1
	mov	x0, x21
	ldr	x8, [x8, #1008]
	add	x8, x8, #2224           // =2224
	//APP
	// atomic64_sub
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	sub	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	bl	f2fs_mark_inode_dirty_sync
.LBB45_29:
	add	x0, sp, #40             // =40
	mov	x1, x19
	bl	f2fs_outplace_write_data
	cbz	w0, .LBB45_85
// %bb.30:
	mov	w22, w0
	mov	x0, x20
	bl	end_page_writeback
	ldr	w8, [x19, #92]
	cbz	w8, .LBB45_94
// %bb.31:
	ldr	w9, [x19, #32]
	cmn	w9, #2                  // =2
	b.ne	.LBB45_94
// %bb.32:
	ldr	w9, [x21, #1292]
	mov	w1, #1
	mov	x0, x21
	sub	w8, w9, w8
	add	w8, w8, #1              // =1
	add	x9, x21, #1280          // =1280
	//APP
	// atomic_add
	prfm	pstl1strm, [x9]
1:	ldxr	w10, [x9]
	add	w10, w10, w8
	stxr	w11, w10, [x9]
	cbnz	w11, 1b

	//NO_APP
	ldr	x9, [x21, #40]
	sxtw	x8, w8
	ldr	x9, [x9, #1008]
	add	x9, x9, #2224           // =2224
	//APP
	// atomic64_add
	prfm	pstl1strm, [x9]
1:	ldxr	x10, [x9]
	add	x10, x10, x8
	stxr	w11, x10, [x9]
	cbnz	w11, 1b
	//NO_APP
	bl	f2fs_mark_inode_dirty_sync
	b	.LBB45_94
.LBB45_33:
	ldr	x8, [x20, #8]
	mov	w9, #4
	add	x10, x20, #40           // =40
	sub	x11, x8, #1             // =1
	tst	x8, #0x1
	csel	x8, x20, x11, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x11, [x8]
	bic	x11, x11, x9
	stxr	w12, x11, [x8]
	cbnz	w12, 1b
	//NO_APP
	mov	w8, #8
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x10]
1:	ldxr	x9, [x10]
	bic	x9, x9, x8
	stxr	w11, x9, [x10]
	cbnz	w11, 1b
	//NO_APP
	ldr	x8, [x20, #40]
	cmp	x8, #1                  // =1
	b.ne	.LBB45_93
// %bb.34:
	str	xzr, [x20, #40]
	ldr	x8, [x20]
	tbz	w8, #13, .LBB45_93
// %bb.35:
	mov	w8, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x20]
1:	ldxr	x9, [x20]
	bic	x9, x9, x8
	stxr	w10, x9, [x20]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x20, x9, eq
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
	cbnz	w10, .LBB45_93
// %bb.36:
	bl	__put_page
	b	.LBB45_93
.LBB45_37:
	mov	w22, #-117
	b	.LBB45_102
.LBB45_38:
	cbz	x8, .LBB45_40
// %bb.39:
	ldr	x9, [x8, #432]
	add	x9, x9, #1              // =1
	str	x9, [x8, #432]
.LBB45_40:
	mov	x0, x22
	bl	mutex_unlock
	mov	x0, x19
	bl	f2fs_encrypt_one_page
	mov	w22, w0
	cbnz	w0, .LBB45_94
// %bb.41:
	mov	x0, x20
	mov	w1, wzr
	bl	__test_set_page_writeback
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	mov	w9, #256
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	x0, [sp, #56]
	cbz	x0, .LBB45_64
// %bb.42:
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #56]
	b	.LBB45_65
.LBB45_43:
	ldr	w9, [x23, #3128]
	cbz	w9, .LBB45_51
// %bb.44:
	ldr	x8, [x23]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB45_50
// %bb.45:
	ldr	x10, [x23, #3136]
	add	x10, x10, #80           // =80
	b	.LBB45_47
.LBB45_46:                              //   in Loop: Header=BB45_47 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB45_50
.LBB45_47:                              // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB45_46
// %bb.48:                              //   in Loop: Header=BB45_47 Depth=1
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB45_46
// %bb.49:
	ldur	x8, [x10, #-80]
.LBB45_50:
	cbz	x8, .LBB45_12
.LBB45_51:
	ldr	x0, [x19]
	mov	w2, #7
	bl	f2fs_is_valid_blkaddr
	tbnz	w0, #0, .LBB45_12
// %bb.52:
	mov	w22, #-117
	b	.LBB45_94
.LBB45_53:
	ldr	x11, [x19]
	ldr	w10, [x11, #3128]
	cbz	w10, .LBB45_61
// %bb.54:
	ldr	x9, [x11]
	cmp	w10, #1                 // =1
	ldr	x9, [x9, #248]
	b.lt	.LBB45_60
// %bb.55:
	ldr	x11, [x11, #3136]
	add	x11, x11, #80           // =80
	b	.LBB45_57
.LBB45_56:                              //   in Loop: Header=BB45_57 Depth=1
	subs	x10, x10, #1            // =1
	add	x11, x11, #88           // =88
	b.eq	.LBB45_60
.LBB45_57:                              // =>This Inner Loop Header: Depth=1
	ldur	w12, [x11, #-4]
	cmp	w12, w8
	b.hi	.LBB45_56
// %bb.58:                              //   in Loop: Header=BB45_57 Depth=1
	ldr	w12, [x11]
	cmp	w12, w8
	b.lo	.LBB45_56
// %bb.59:
	ldur	x9, [x11, #-80]
.LBB45_60:
	cbz	x9, .LBB45_18
.LBB45_61:
	ldr	x8, [x19, #40]
	mov	x1, x19
	ldr	x8, [x8, #24]
	ldr	x22, [x8]
	mov	x0, x22
	bl	f2fs_should_update_outplace
	tbnz	w0, #0, .LBB45_18
// %bb.62:
	mov	x0, x22
	mov	x1, x19
	bl	f2fs_should_update_inplace
	tbnz	w0, #0, .LBB45_13
	b	.LBB45_18
.LBB45_63:
	mov	w22, #-11
	b	.LBB45_94
.LBB45_64:
	mov	x9, xzr
.LBB45_65:
	ldr	x8, [sp, #48]
	cbz	x8, .LBB45_69
// %bb.66:
	cmp	x9, x8
	b.eq	.LBB45_69
// %bb.67:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB45_69
// %bb.68:
	bl	__put_page
.LBB45_69:
	stp	xzr, xzr, [sp, #48]
	ldr	w8, [x19, #84]
	cbnz	w8, .LBB45_71
// %bb.70:
	ldr	x8, [x19]
	add	x0, x8, #304            // =304
	bl	up_read
.LBB45_71:
	mov	x0, x19
	bl	f2fs_inplace_write_data
	mov	w22, w0
	cbz	w0, .LBB45_77
// %bb.72:
	mov	x0, x21
	bl	fscrypt_inode_uses_fs_layer_crypto
	tbz	w0, #0, .LBB45_75
// %bb.73:
	ldr	x0, [x19, #48]
	ldr	x8, [x0, #24]
	cbnz	x8, .LBB45_75
// %bb.74:
	ldr	x8, [x0, #40]
	str	x8, [x19, #48]
	bl	fscrypt_free_bounce_page
.LBB45_75:
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbz	w8, #15, .LBB45_78
// %bb.76:
	mov	x0, x20
	bl	end_page_writeback
	b	.LBB45_78
.LBB45_77:
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 8192
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB45_78:
	adrp	x8, __tracepoint_f2fs_do_write_data_page+8
	ldr	x19, [x19, #40]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_do_write_data_page+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB45_102
// %bb.79:
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
	tbz	w8, #0, .LBB45_102
// %bb.80:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_do_write_data_page+32
	ldr	x21, [x8, :lo12:__tracepoint_f2fs_do_write_data_page+32]
	cbz	x21, .LBB45_82
.LBB45_81:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	mov	w2, #9
	mov	x1, x19
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB45_81
.LBB45_82:
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB45_84
// %bb.83:
	ldr	x8, [x20, #24]
	cbnz	x8, .LBB45_102
.LBB45_84:
	bl	preempt_schedule_notrace
	b	.LBB45_102
.LBB45_85:
	adrp	x8, __tracepoint_f2fs_do_write_data_page+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_do_write_data_page+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB45_91
// %bb.86:
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
	tbz	w8, #0, .LBB45_91
// %bb.87:
	//APP
	mrs x22, sp_el0
	//NO_APP
	ldr	w8, [x22, #24]
	add	w8, w8, #1              // =1
	str	w8, [x22, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_do_write_data_page+32
	ldr	x23, [x8, :lo12:__tracepoint_f2fs_do_write_data_page+32]
	cbz	x23, .LBB45_89
.LBB45_88:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	mov	w2, #10
	mov	x1, x20
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB45_88
.LBB45_89:
	//APP
	//NO_APP
	ldr	x8, [x22, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x22, #24]
	b.eq	.LBB45_104
// %bb.90:
	ldr	x8, [x22, #24]
	cbz	x8, .LBB45_104
.LBB45_91:
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4096
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x9, [x20, #32]
	cbnz	x9, .LBB45_93
// %bb.92:
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 262144
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB45_93:
	mov	w22, wzr
.LBB45_94:
	ldr	x0, [sp, #56]
	cbz	x0, .LBB45_96
// %bb.95:
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #56]
	ldr	x8, [sp, #48]
	cbnz	x8, .LBB45_97
	b	.LBB45_100
.LBB45_96:
	mov	x9, xzr
	ldr	x8, [sp, #48]
	cbz	x8, .LBB45_100
.LBB45_97:
	cmp	x9, x8
	b.eq	.LBB45_100
// %bb.98:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB45_100
// %bb.99:
	bl	__put_page
.LBB45_100:
	stp	xzr, xzr, [sp, #48]
	ldr	w8, [x19, #84]
	cbnz	w8, .LBB45_102
.LBB45_101:
	ldr	x8, [x19]
	add	x0, x8, #304            // =304
	bl	up_read
.LBB45_102:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB45_105
// %bb.103:
	mov	w0, w22
	ldp	x20, x19, [sp, #144]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #128]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #112]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #96]     // 16-byte Folded Reload
	add	sp, sp, #160            // =160
	ret
.LBB45_104:
	bl	preempt_schedule_notrace
	b	.LBB45_91
.LBB45_105:
	bl	__stack_chk_fail
.Lfunc_end45:
	.size	f2fs_do_write_data_page, .Lfunc_end45-f2fs_do_write_data_page
                                        // -- End function
	.globl	f2fs_write_single_data_page // -- Begin function f2fs_write_single_data_page
	.p2align	2
	.type	f2fs_write_single_data_page,@function
f2fs_write_single_data_page:            // @f2fs_write_single_data_page
.Lf2fs_write_single_data_page$local:
// %bb.0:
	sub	sp, sp, #256            // =256
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #160]    // 16-byte Folded Spill
	add	x29, sp, #160           // =160
	stp	x28, x27, [sp, #176]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #192]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #208]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #224]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #240]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldp	x8, x9, [x0, #24]
	mov	w28, w7
	mov	w26, w6
	mov	x25, x4
	ldr	x24, [x8]
	mov	x19, x2
	mov	x23, x1
	mov	x22, x0
	ldr	x8, [x24, #40]
	ldr	x27, [x24, #80]
	ldr	x20, [x8, #1008]
	stp	xzr, xzr, [sp, #136]
	stp	xzr, xzr, [sp, #120]
	stp	xzr, xzr, [sp, #104]
	stp	xzr, xzr, [sp, #88]
	stp	xzr, xzr, [sp, #72]
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	stp	x9, x20, [sp, #8]       // 8-byte Folded Spill
	ldr	x8, [x24, #64]
	mov	w9, #1
	stur	xzr, [sp, #28]
	str	w9, [sp, #36]
	str	w8, [sp, #24]
	ldrh	w8, [x4, #36]
	ldr	w9, [x4, #32]
	and	w10, w8, #0x100
	cmp	w9, #1                  // =1
	lsl	w9, w10, #17
	b.ne	.LBB46_2
// %bb.1:
	orr	w9, w9, #0x800
	b	.LBB46_3
.LBB46_2:
	orr	w10, w9, #0x200000
	tst	w8, #0x3
	csel	w9, w9, w10, eq
.LBB46_3:
	str	w9, [sp, #40]
	mov	w9, #2
	ubfx	w8, w8, #6, #1
	stur	xzr, [sp, #44]
	stp	x22, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #72]
	str	xzr, [sp, #88]
	strb	wzr, [sp, #96]
	strh	wzr, [sp, #104]
	strb	wzr, [sp, #106]
	str	w26, [sp, #108]
	strb	wzr, [sp, #112]
	str	w5, [sp, #116]
	stp	x25, x19, [sp, #120]
	str	x3, [sp, #136]
	strb	wzr, [sp, #144]
	str	w9, [sp, #100]
	str	w8, [sp, #148]
	cbz	w26, .LBB46_5
// %bb.4:
	mov	w8, #1
	str	w8, [sp, #100]
.LBB46_5:
	adrp	x8, __tracepoint_f2fs_writepage+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_writepage+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB46_11
// %bb.6:
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
	tbz	w8, #0, .LBB46_11
// %bb.7:
	str	x27, [sp]               // 8-byte Folded Spill
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w8, [x21, #24]
	mov	x27, x25
	mov	x25, x20
	mov	x20, x23
	add	w8, w8, #1              // =1
	str	w8, [x21, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_writepage+32
	mov	x23, x19
	mov	w19, w28
	ldr	x28, [x8, :lo12:__tracepoint_f2fs_writepage+32]
	cbz	x28, .LBB46_9
.LBB46_8:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x28]
	mov	x1, x22
	mov	w2, wzr
	blr	x8
	ldr	x8, [x28, #24]!
	cbnz	x8, .LBB46_8
.LBB46_9:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	mov	w28, w19
	mov	x19, x23
	mov	x23, x20
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	mov	x20, x25
	mov	x25, x27
	ldr	x27, [sp]               // 8-byte Folded Reload
	b.eq	.LBB46_81
// %bb.10:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB46_81
.LBB46_11:
	ldr	x8, [x20, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB46_55
// %bb.12:
	ldr	x8, [x20, #80]
	tbnz	w8, #3, .LBB46_43
// %bb.13:
	ldr	x9, [x22, #32]
	lsr	x10, x27, #12
	cmp	x9, x10
	b.lo	.LBB46_21
// %bb.14:
	ldr	x8, [x24, #736]
	cbnz	w26, .LBB46_21
// %bb.15:
	tbnz	w8, #30, .LBB46_21
// %bb.16:
	ands	w8, w27, #0xfff
	mov	w26, wzr
	b.eq	.LBB46_40
// %bb.17:
	cmp	x9, x10
	mov	w27, w26
	b.hi	.LBB46_57
// %bb.18:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w9, [x21, #24]
	mov	x10, #134217728
	movk	x10, #64, lsl #32
	add	x10, x10, x22, lsl #6
	add	w9, w9, #1              // =1
	str	w9, [x21, #24]
	//APP
	//NO_APP
	ldr	w9, [x21, #2800]
	and	x10, x10, #0xfffffffffffff000
	orr	x10, x10, x8
	mov	w1, wzr
	add	w9, w9, #1              // =1
	str	w9, [x21, #2800]
	//APP
	//NO_APP
	mov	x9, #-549755813888
	add	x0, x10, x9
	mov	w9, #4096
	sub	w2, w9, w8
	bl	memset
	//APP
	//NO_APP
	ldr	w8, [x21, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x21, #2800]
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	b.eq	.LBB46_83
// %bb.19:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB46_83
.LBB46_20:
	mov	x0, x22
	bl	flush_dcache_page
.LBB46_21:
	ldr	x8, [x24, #736]
	tbnz	w8, #19, .LBB46_56
// %bb.22:
	ldr	x8, [x24, #736]
	tbz	w8, #17, .LBB46_26
// %bb.23:
	ldr	x8, [x22, #32]
	cbz	x8, .LBB46_43
// %bb.24:
	ldrb	w8, [x25, #36]
	tbnz	w8, #3, .LBB46_26
// %bb.25:
	mov	w1, #8
	mov	x0, x20
	bl	f2fs_available_free_memory
	tbnz	w0, #0, .LBB46_43
.LBB46_26:
	ldrh	w9, [x24]
	ldr	w8, [x24, #12]
	and	w9, w9, #0xf000
	cmp	w9, #4, lsl #12         // =16384
	b.ne	.LBB46_28
// %bb.27:
	add	x21, x24, #12           // =12
	tbnz	w8, #5, .LBB46_32
	b	.LBB46_33
.LBB46_28:
	tbnz	w8, #5, .LBB46_31
// %bb.29:
	ldrb	w8, [x25, #36]
	tbnz	w8, #3, .LBB46_42
// %bb.30:
	mov	w21, #1
	ldr	x8, [x24, #736]
	tbnz	w8, #10, .LBB46_45
	b	.LBB46_48
.LBB46_31:
	add	x21, x24, #12           // =12
.LBB46_32:
	add	x0, x20, #376           // =376
	bl	down_read
.LBB46_33:
	mov	w8, #1
	add	x0, sp, #16             // =16
	str	w8, [sp, #100]
	bl	f2fs_do_write_data_page
	ldrb	w8, [x21]
	mov	w27, w0
	tbz	w8, #5, .LBB46_35
// %bb.34:
	add	x0, x20, #376           // =376
	bl	up_read
.LBB46_35:
	mov	w21, wzr
.LBB46_36:
	cmp	w21, #0                 // =0
	cset	w26, ne
	cbz	w27, .LBB46_57
// %bb.37:
	cmn	w27, #2                 // =2
	b.eq	.LBB46_57
.LBB46_38:
	mov	x0, x25
	mov	x1, x22
	bl	redirty_page_for_writepage
	ldrb	w8, [x25, #36]
	tbnz	w8, #3, .LBB46_41
// %bb.39:
	mov	x0, x22
	bl	unlock_page
	b	.LBB46_79
.LBB46_40:
	mov	w27, w26
	b	.LBB46_57
.LBB46_41:
	mov	w27, #524288
	b	.LBB46_79
.LBB46_42:
	mov	x0, x20
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB46_44
.LBB46_43:
	mov	x0, x25
	mov	x1, x22
	bl	redirty_page_for_writepage
	mov	w27, #524288
	b	.LBB46_79
.LBB46_44:
	add	x8, x24, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 33554432
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w21, wzr
	ldr	x8, [x24, #736]
	tbz	w8, #10, .LBB46_48
.LBB46_45:
	ldr	w8, [sp, #148]
	mov	x0, x24
	mov	x1, x22
	cmp	w8, #1                  // =1
	cset	w2, eq
	bl	f2fs_write_inline_data_ex
	cmp	w21, #0                 // =0
	mov	w27, w0
	cset	w26, ne
	cbz	w0, .LBB46_57
// %bb.46:
	ldr	w8, [sp, #148]
	cmp	w8, #1                  // =1
	b.eq	.LBB46_38
// %bb.47:
	cmn	w27, #11                // =11
	b.ne	.LBB46_51
.LBB46_48:
	add	x0, sp, #16             // =16
	bl	f2fs_do_write_data_page
	mov	w27, w0
	cmn	w0, #11                 // =11
	b.ne	.LBB46_50
// %bb.49:
	add	x0, sp, #16             // =16
	str	wzr, [sp, #100]
	bl	f2fs_do_write_data_page
	mov	w27, w0
.LBB46_50:
	cbz	w27, .LBB46_52
.LBB46_51:
	ldrb	w8, [x24, #712]
	mov	w1, #1
	mov	x0, x24
	orr	w8, w8, #0x10
	strb	w8, [x24, #712]
	bl	f2fs_mark_inode_dirty_sync
	b	.LBB46_36
.LBB46_52:
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	add	x26, x24, #840          // =840
	mov	x0, x26
	lsl	x8, x8, #12
	add	x27, x8, #1, lsl #12    // =4096
	bl	_raw_spin_lock
	ldr	x8, [x24, #832]
	cmp	x8, x27
	b.ge	.LBB46_54
// %bb.53:
	str	x27, [x24, #832]
.LBB46_54:
	mov	x0, x26
	bl	_raw_spin_unlock
	cmp	w21, #0                 // =0
	mov	w27, wzr
	cset	w26, ne
	b	.LBB46_57
.LBB46_55:
	ldr	x26, [x22, #24]
	mov	w1, #-5
	mov	x0, x26
	bl	__filemap_set_wb_err
	add	x8, x26, #128           // =128
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 1
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrh	w8, [x24]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB46_43
.LBB46_56:
	mov	w26, wzr
	mov	w27, wzr
.LBB46_57:
	ldrh	w8, [x24]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB46_60
// %bb.58:
	cmp	w8, #10, lsl #12        // =40960
	b.eq	.LBB46_60
// %bb.59:
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB46_64
.LBB46_60:
	add	x8, x24, #792           // =792
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	ldr	x8, [x24, #736]
	tbz	x8, #37, .LBB46_62
// %bb.61:
	ldr	x8, [x24, #40]
	mov	w9, #1
	ldr	x8, [x8, #1008]
	add	x8, x8, #1328           // =1328
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
.LBB46_62:
	ldr	x8, [x24, #40]
	ldrh	w9, [x24]
	ldr	x8, [x8, #1008]
	and	w9, w9, #0xf000
	cmp	w9, #4, lsl #12         // =16384
	cset	w9, ne
	add	x8, x8, w9, uxtw #2
	add	x8, x8, #1272           // =1272
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	ldrb	w8, [x24, #12]
	tbz	w8, #5, .LBB46_64
// %bb.63:
	ldr	x8, [x24, #40]
	mov	w9, #1
	ldr	x8, [x8, #1008]
	add	x8, x8, #1280           // =1280
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
.LBB46_64:
	cbz	w27, .LBB46_69
// %bb.65:
	ldr	x8, [x22, #8]
	mov	w9, #4
	add	x10, x22, #40           // =40
	sub	x11, x8, #1             // =1
	tst	x8, #0x1
	csel	x8, x22, x11, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x11, [x8]
	bic	x11, x11, x9
	stxr	w12, x11, [x8]
	cbnz	w12, 1b
	//NO_APP
	mov	w8, #8
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x10]
1:	ldxr	x9, [x10]
	bic	x9, x9, x8
	stxr	w11, x9, [x10]
	cbnz	w11, 1b
	//NO_APP
	ldr	x8, [x22, #40]
	cmp	x8, #1                  // =1
	b.ne	.LBB46_69
// %bb.66:
	str	xzr, [x22, #40]
	ldr	x8, [x22]
	tbz	w8, #13, .LBB46_69
// %bb.67:
	mov	w8, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x22]
1:	ldxr	x9, [x22]
	bic	x9, x9, x8
	stxr	w10, x9, [x22]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x22, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x22, x9, eq
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
	cbnz	w10, .LBB46_69
// %bb.68:
	bl	__put_page
.LBB46_69:
	ldrb	w8, [x25, #36]
	tbz	w8, #3, .LBB46_71
// %bb.70:
	mov	x0, x20
	mov	x1, xzr
	mov	x2, x22
	mov	w3, wzr
	mov	w4, wzr
	mov	w5, wzr
	bl	__submit_merged_write_cond
	add	x8, x24, #736           // =736
	mov	w9, #33554432
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	x0, x24
	bl	f2fs_remove_dirty_inode
	mov	x23, xzr
.LBB46_71:
	mov	x0, x22
	bl	unlock_page
	ldrh	w8, [x24]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB46_76
// %bb.72:
	ldrb	w8, [x24, #12]
	tbnz	w8, #5, .LBB46_76
// %bb.73:
	ldr	x8, [x24, #816]
	cbnz	x8, .LBB46_76
// %bb.74:
	tbz	w28, #0, .LBB46_76
// %bb.75:
	mov	x0, x20
	mov	w1, w26
	bl	f2fs_balance_fs
.LBB46_76:
	ldr	x8, [x20, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB46_82
// %bb.77:
	mov	w27, wzr
	cbz	x23, .LBB46_79
// %bb.78:
	ldrb	w8, [sp, #96]
	str	w8, [x23]
.LBB46_79:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB46_84
// %bb.80:
	mov	w0, w27
	ldp	x20, x19, [sp, #240]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #224]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #208]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #192]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #176]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #160]    // 16-byte Folded Reload
	add	sp, sp, #256            // =256
	ret
.LBB46_81:
	bl	preempt_schedule_notrace
	b	.LBB46_11
.LBB46_82:
	ldr	x21, [x20, #144]
	add	x22, x21, #160          // =160
	mov	x0, x22
	bl	down_write
	mov	x0, x21
	bl	__submit_merged_bio
	mov	x0, x22
	bl	up_write
	ldr	x8, [x20, #144]
	add	x22, x8, #456           // =456
	mov	x0, x22
	add	x21, x8, #296           // =296
	bl	down_write
	mov	x0, x21
	bl	__submit_merged_bio
	mov	x0, x22
	bl	up_write
	ldr	x8, [x20, #144]
	add	x22, x8, #752           // =752
	mov	x0, x22
	add	x21, x8, #592           // =592
	bl	down_write
	mov	x0, x21
	bl	__submit_merged_bio
	mov	x0, x22
	bl	up_write
	mov	x0, x20
	mov	x1, x19
	mov	x2, xzr
	bl	f2fs_submit_merged_ipu_write
	mov	w27, wzr
	b	.LBB46_79
.LBB46_83:
	bl	preempt_schedule
	b	.LBB46_20
.LBB46_84:
	bl	__stack_chk_fail
.Lfunc_end46:
	.size	f2fs_write_single_data_page, .Lfunc_end46-f2fs_write_single_data_page
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
	tbnz	w8, #3, .LBB47_11
// %bb.1:
	ldr	x1, [x0, #136]
	ldr	x8, [x1, #8]
	ldr	w12, [x1, #100]
	ldr	w8, [x8, #8]
	cmn	w12, #1                 // =1
	udiv	w10, w12, w9
	b.eq	.LBB47_3
// %bb.2:
	mov	w17, w10
	cmp	w8, w17
	b.eq	.LBB47_4
	b	.LBB47_8
.LBB47_3:
	mov	w17, #-1
	cmp	w8, w17
	b.ne	.LBB47_8
.LBB47_4:
	ldr	w2, [x0, #1284]
	ldr	w3, [x0, #1272]
	ldr	w17, [x0, #1272]
	ldr	x0, [x1, #24]
	ldr	x1, [x1]
	mov	w5, #40
	add	w2, w3, w2
	ldr	w4, [x0, #460]
	ldr	x1, [x1, #120]
	madd	x4, x4, x5, x1
	ldrh	w4, [x4, #2]
	and	w3, w4, #0x3ff
	sub	w3, w16, w3
	cmp	w3, w2
	b.lo	.LBB47_8
// %bb.5:
	ldr	w3, [x0, #580]
	mov	w4, #40
	madd	x3, x3, x4, x1
	ldrh	w3, [x3, #2]
	and	w3, w3, #0x3ff
	sub	w3, w16, w3
	cmp	w3, w2
	b.lo	.LBB47_8
// %bb.6:
	ldr	w3, [x0, #700]
	mov	w4, #40
	madd	x3, x3, x4, x1
	ldrh	w3, [x3, #2]
	and	w3, w3, #0x3ff
	sub	w3, w16, w3
	cmp	w3, w2
	b.lo	.LBB47_8
// %bb.7:
	ldr	w0, [x0, #100]
	mov	w2, #40
	madd	x0, x0, x2, x1
	ldrh	w0, [x0, #2]
	and	w0, w0, #0x3ff
	sub	w0, w16, w0
	cmp	w0, w17
	b.hs	.LBB47_11
.LBB47_8:
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
	b.ne	.LBB47_10
// %bb.9:
	mov	w10, #-1
.LBB47_10:
	add	w9, w9, w10
	cmp	w8, w9
	cset	w0, ls
	ret
.LBB47_11:
	mov	w0, wzr
	ret
.Lfunc_end47:
	.size	has_not_enough_free_secs, .Lfunc_end47-has_not_enough_free_secs
                                        // -- End function
	.globl	f2fs_invalidate_page    // -- Begin function f2fs_invalidate_page
	.p2align	2
	.type	f2fs_invalidate_page,@function
f2fs_invalidate_page:                   // @f2fs_invalidate_page
.Lf2fs_invalidate_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #24]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	mov	x29, sp
	ldr	x20, [x8]
	ldr	x8, [x20, #40]
	ldr	x21, [x8, #1008]
	ldr	x8, [x20, #64]
	ldr	w9, [x21, #1172]
	cmp	x8, x9
	b.lo	.LBB48_3
// %bb.1:
	tst	w1, #0xfff
	b.ne	.LBB48_40
// %bb.2:
	cmp	w2, #1, lsl #12         // =4096
	b.ne	.LBB48_40
.LBB48_3:
	ldr	x9, [x19, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x9, x19, x10, eq
	ldr	x9, [x9]
	tbz	w9, #3, .LBB48_16
// %bb.4:
	ldr	w9, [x21, #1180]
	cmp	x8, x9
	b.ne	.LBB48_6
// %bb.5:
	add	x8, x21, #1288          // =1288
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	b	.LBB48_16
.LBB48_6:
	ldr	w9, [x21, #1176]
	cmp	x8, x9
	b.ne	.LBB48_8
// %bb.7:
	add	x8, x21, #1284          // =1284
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	b	.LBB48_16
.LBB48_8:
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB48_11
// %bb.9:
	cmp	w8, #10, lsl #12        // =40960
	b.eq	.LBB48_11
// %bb.10:
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB48_15
.LBB48_11:
	add	x8, x20, #792           // =792
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	ldr	x8, [x20, #736]
	tbz	x8, #37, .LBB48_13
// %bb.12:
	ldr	x8, [x20, #40]
	mov	w9, #1
	ldr	x8, [x8, #1008]
	add	x8, x8, #1328           // =1328
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
.LBB48_13:
	ldr	x8, [x20, #40]
	ldrh	w9, [x20]
	ldr	x8, [x8, #1008]
	and	w9, w9, #0xf000
	cmp	w9, #4, lsl #12         // =16384
	cset	w9, ne
	add	x8, x8, w9, uxtw #2
	add	x8, x8, #1272           // =1272
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	ldrb	w8, [x20, #12]
	tbz	w8, #5, .LBB48_15
// %bb.14:
	ldr	x8, [x20, #40]
	mov	w9, #1
	ldr	x8, [x8, #1008]
	add	x8, x8, #1280           // =1280
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
.LBB48_15:
	mov	x0, x20
	bl	f2fs_remove_dirty_inode
.LBB48_16:
	add	x8, x19, #40            // =40
	mov	w9, #8
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	x8, [x19, #40]
	cmp	x8, #1                  // =1
	b.ne	.LBB48_20
// %bb.17:
	str	xzr, [x19, #40]
	ldr	x8, [x19]
	tbz	w8, #13, .LBB48_20
// %bb.18:
	mov	w8, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x9, [x19]
	bic	x9, x9, x8
	stxr	w10, x9, [x19]
	cbnz	w10, 1b
	//NO_APP
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
	cbnz	w10, .LBB48_20
// %bb.19:
	bl	__put_page
.LBB48_20:
	ldrb	w8, [x21, #1427]
	tbz	w8, #6, .LBB48_33
// %bb.21:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB48_25
// %bb.22:
	ldr	w8, [x20, #1280]
	cbnz	w8, .LBB48_27
// %bb.23:
	ldr	x8, [x20, #736]
	tbnz	x8, #35, .LBB48_27
// %bb.24:
	ldr	x8, [x20, #736]
	tbnz	x8, #34, .LBB48_27
	b	.LBB48_28
.LBB48_25:
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB48_28
// %bb.26:
	ldr	x8, [x20, #736]
	tbz	w8, #31, .LBB48_28
.LBB48_27:
	ldr	w1, [x20, #64]
	mov	x0, x21
	bl	f2fs_invalidate_compress_pages
.LBB48_28:
	ldr	x8, [x21, #120]
	ldr	x9, [x20, #64]
	ldr	w8, [x8, #4]
	cmp	x9, x8
	b.ne	.LBB48_33
// %bb.29:
	ldr	x8, [x19, #40]
	and	x8, x8, #0x3f
	cmp	x8, #1                  // =1
	str	x8, [x19, #40]
	b.ne	.LBB48_33
// %bb.30:
	str	xzr, [x19, #40]
	ldr	x8, [x19]
	tbz	w8, #13, .LBB48_33
// %bb.31:
	mov	w8, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x9, [x19]
	bic	x9, x9, x8
	stxr	w10, x9, [x19]
	cbnz	w10, 1b
	//NO_APP
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
	cbnz	w10, .LBB48_33
// %bb.32:
	bl	__put_page
.LBB48_33:
	ldr	x8, [x19]
	tbz	w8, #13, .LBB48_36
// %bb.34:
	ldr	x8, [x19, #40]
	tbz	w8, #0, .LBB48_36
// %bb.35:
	ldr	x8, [x19, #40]
	tbnz	w8, #1, .LBB48_41
.LBB48_36:
	ldr	x8, [x19]
	tbz	w8, #13, .LBB48_39
// %bb.37:
	mov	w8, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x9, [x19]
	bic	x9, x9, x8
	stxr	w10, x9, [x19]
	cbnz	w10, 1b
	//NO_APP
	str	xzr, [x19, #40]
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
	cbnz	w10, .LBB48_39
// %bb.38:
	bl	__put_page
.LBB48_39:
	str	xzr, [x19, #40]
.LBB48_40:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB48_41:
	mov	x0, x20
	mov	x1, x19
	bl	f2fs_drop_inmem_page
	b	.LBB48_40
.Lfunc_end48:
	.size	f2fs_invalidate_page, .Lfunc_end48-f2fs_invalidate_page
                                        // -- End function
	.globl	f2fs_release_page       // -- Begin function f2fs_release_page
	.p2align	2
	.type	f2fs_release_page,@function
f2fs_release_page:                      // @f2fs_release_page
.Lf2fs_release_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #8]
	mov	x29, sp
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x0, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #3, .LBB49_15
// %bb.1:
	ldr	x8, [x0]
	mov	x19, x0
	tbz	w8, #13, .LBB49_4
// %bb.2:
	ldr	x8, [x19, #40]
	tbz	w8, #0, .LBB49_4
// %bb.3:
	ldr	x8, [x19, #40]
	tbnz	w8, #1, .LBB49_15
.LBB49_4:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB49_6
// %bb.5:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB49_33
.LBB49_6:
	ldr	x0, [x19, #24]
.LBB49_7:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldrb	w8, [x8, #1427]
	tbz	w8, #6, .LBB49_24
// %bb.8:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB49_10
// %bb.9:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB49_34
.LBB49_10:
	ldr	x8, [x19, #24]
	mov	x0, x8
.LBB49_11:
	ldr	x21, [x8]
	ldr	x9, [x0]
	ldr	x8, [x21, #40]
	ldr	x9, [x9, #40]
	ldr	x8, [x8, #1008]
	ldr	x20, [x9, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB49_16
// %bb.12:
	ldr	w8, [x21, #1280]
	cbnz	w8, .LBB49_18
// %bb.13:
	ldr	x8, [x21, #736]
	tbnz	x8, #35, .LBB49_18
// %bb.14:
	ldr	x8, [x21, #736]
	tbnz	x8, #34, .LBB49_18
	b	.LBB49_19
.LBB49_15:
	mov	w0, wzr
	b	.LBB49_32
.LBB49_16:
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB49_19
// %bb.17:
	ldr	x8, [x21, #736]
	tbz	w8, #31, .LBB49_19
.LBB49_18:
	ldr	w1, [x21, #64]
	mov	x0, x20
	bl	f2fs_invalidate_compress_pages
.LBB49_19:
	ldr	x8, [x20, #120]
	ldr	x9, [x21, #64]
	ldr	w8, [x8, #4]
	cmp	x9, x8
	b.ne	.LBB49_24
// %bb.20:
	ldr	x8, [x19, #40]
	and	x8, x8, #0x3f
	cmp	x8, #1                  // =1
	str	x8, [x19, #40]
	b.ne	.LBB49_24
// %bb.21:
	str	xzr, [x19, #40]
	ldr	x8, [x19]
	tbz	w8, #13, .LBB49_24
// %bb.22:
	mov	w8, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x9, [x19]
	bic	x9, x9, x8
	stxr	w10, x9, [x19]
	cbnz	w10, 1b
	//NO_APP
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
	cbnz	w10, .LBB49_24
// %bb.23:
	bl	__put_page
.LBB49_24:
	add	x8, x19, #40            // =40
	mov	w9, #8
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	x8, [x19, #40]
	cmp	x8, #1                  // =1
	b.ne	.LBB49_28
// %bb.25:
	str	xzr, [x19, #40]
	ldr	x8, [x19]
	tbz	w8, #13, .LBB49_28
// %bb.26:
	mov	w8, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x9, [x19]
	bic	x9, x9, x8
	stxr	w10, x9, [x19]
	cbnz	w10, 1b
	//NO_APP
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
	cbnz	w10, .LBB49_28
// %bb.27:
	bl	__put_page
.LBB49_28:
	ldr	x8, [x19]
	tbz	w8, #13, .LBB49_31
// %bb.29:
	mov	w8, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x9, [x19]
	bic	x9, x9, x8
	stxr	w10, x9, [x19]
	cbnz	w10, 1b
	//NO_APP
	str	xzr, [x19, #40]
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
	cbnz	w10, .LBB49_31
// %bb.30:
	bl	__put_page
.LBB49_31:
	mov	w0, #1
	str	xzr, [x19, #40]
.LBB49_32:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB49_33:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB49_7
.LBB49_34:
	mov	x0, x19
	bl	__page_file_mapping
	ldr	x8, [x19, #24]
	b	.LBB49_11
.Lfunc_end49:
	.size	f2fs_release_page, .Lfunc_end49-f2fs_release_page
                                        // -- End function
	.globl	f2fs_migrate_page       // -- Begin function f2fs_migrate_page
	.p2align	2
	.type	f2fs_migrate_page,@function
f2fs_migrate_page:                      // @f2fs_migrate_page
.Lf2fs_migrate_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x24, [x0]
	ldr	x8, [x2]
	mov	w21, w3
	mov	x19, x2
	mov	x22, x0
	mov	x20, x1
	mov	x29, sp
	tbz	w8, #13, .LBB50_2
// %bb.1:
	ldr	x8, [x19, #40]
	tbnz	w8, #0, .LBB50_3
.LBB50_2:
	mov	w8, wzr
	b	.LBB50_4
.LBB50_3:
	ldr	x8, [x19, #40]
	ubfx	x8, x8, #1, #1
.LBB50_4:
	ldr	x9, [x19, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x9, x19, x10, eq
	ldr	x9, [x9]
	tbnz	w9, #15, .LBB50_28
// %bb.5:
	cbz	w8, .LBB50_10
// %bb.6:
	cmp	w21, #2                 // =2
	b.ne	.LBB50_11
// %bb.7:
	add	x23, x24, #952          // =952
	mov	x0, x23
	bl	mutex_trylock
	cbz	w0, .LBB50_12
// %bb.8:
	mov	w3, #1
	mov	x0, x22
	mov	x1, x20
	mov	x2, x19
	bl	migrate_page_move_mapping
	cbz	w0, .LBB50_13
// %bb.9:
	mov	w22, w0
	mov	x0, x23
	bl	mutex_unlock
	b	.LBB50_27
.LBB50_10:
	mov	x0, x22
	mov	x1, x20
	mov	x2, x19
	mov	w3, wzr
	bl	migrate_page_move_mapping
	mov	w22, w0
	cbnz	w0, .LBB50_27
	b	.LBB50_20
.LBB50_11:
	mov	w22, #-16
	b	.LBB50_27
.LBB50_12:
	mov	w22, #-11
	b	.LBB50_27
.LBB50_13:
	ldr	x8, [x24, #928]
	add	x9, x24, #928           // =928
	cmp	x8, x9
	b.eq	.LBB50_17
.LBB50_14:                              // =>This Inner Loop Header: Depth=1
	ldr	x10, [x8, #16]
	cmp	x10, x19
	b.eq	.LBB50_16
// %bb.15:                              //   in Loop: Header=BB50_14 Depth=1
	ldr	x8, [x8]
	cmp	x8, x9
	b.ne	.LBB50_14
	b	.LBB50_17
.LBB50_16:
	str	x20, [x8, #16]
.LBB50_17:
	mov	x0, x23
	bl	mutex_unlock
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
	cbnz	w10, .LBB50_19
// %bb.18:
	bl	__put_page
.LBB50_19:
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	add	x8, x8, #52             // =52
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB50_20:
	str	xzr, [x20, #40]
	ldr	x8, [x19]
	tbz	w8, #13, .LBB50_23
// %bb.21:
	ldr	x8, [x19, #40]
	str	x8, [x20, #40]
	//APP
	// atomic64_or
	prfm	pstl1strm, [x20]
1:	ldxr	x8, [x20]
	orr	x8, x8, 8192
	stxr	w9, x8, [x20]
	cbnz	w9, 1b
	//NO_APP
	ldr	x8, [x20, #8]
	mov	w9, #8192
	sub	x10, x8, #1             // =1
	tst	x8, #0x1
	csel	x8, x20, x10, eq
	add	x8, x8, #52             // =52
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	add	w10, w10, 1
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	str	xzr, [x19, #40]
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x8, [x19]
	bic	x8, x8, x9
	stxr	w10, x8, [x19]
	cbnz	w10, 1b
	//NO_APP
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
	cbnz	w10, .LBB50_23
// %bb.22:
	bl	__put_page
.LBB50_23:
	mov	x0, x20
	mov	x1, x19
	cmp	w21, #3                 // =3
	b.ne	.LBB50_25
// %bb.24:
	bl	migrate_page_states
	b	.LBB50_26
.LBB50_25:
	bl	migrate_page_copy
.LBB50_26:
	mov	w22, wzr
.LBB50_27:
	mov	w0, w22
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB50_28:
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 6432; .short 0; .popsection; 14471: brk 0x800
	//NO_APP
.Lfunc_end50:
	.size	f2fs_migrate_page, .Lfunc_end50-f2fs_migrate_page
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_write_data_page
	.type	f2fs_write_data_page,@function
f2fs_write_data_page:                   // @f2fs_write_data_page
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0, #24]
	mov	x19, x0
	mov	x20, x1
	mov	x29, sp
	ldr	x0, [x8]
	ldr	x8, [x0, #40]
	ldr	x8, [x8, #1008]
	ldr	x9, [x8, #224]
	ldrb	w9, [x9, #132]
	tbnz	w9, #3, .LBB51_9
// %bb.1:
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB51_5
// %bb.2:
	ldr	w8, [x0, #1280]
	cbnz	w8, .LBB51_7
// %bb.3:
	ldr	x8, [x0, #736]
	tbnz	x8, #35, .LBB51_7
// %bb.4:
	ldr	x8, [x0, #736]
	tbnz	x8, #34, .LBB51_7
	b	.LBB51_9
.LBB51_5:
	ldrh	w8, [x0]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB51_9
// %bb.6:
	ldr	x8, [x0, #736]
	tbz	w8, #31, .LBB51_9
.LBB51_7:
	ldr	x1, [x19, #32]
	bl	f2fs_is_compressed_cluster
	cbz	w0, .LBB51_9
// %bb.8:
	mov	x0, x20
	mov	x1, x19
	bl	redirty_page_for_writepage
	mov	w0, #524288
	b	.LBB51_10
.LBB51_9:
	mov	w5, #4
	mov	w7, #1
	mov	x0, x19
	mov	x1, xzr
	mov	x2, xzr
	mov	x3, xzr
	mov	x4, x20
	mov	w6, wzr
	bl	f2fs_write_single_data_page
.LBB51_10:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end51:
	.size	f2fs_write_data_page, .Lfunc_end51-f2fs_write_data_page
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_read_data_page
	.type	f2fs_read_data_page,@function
f2fs_read_data_page:                    // @f2fs_read_data_page
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x1, #8]
	mov	x19, x1
	mov	x29, sp
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x1, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB52_2
// %bb.1:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB52_19
.LBB52_2:
	ldr	x0, [x19, #24]
.LBB52_3:
	adrp	x8, __tracepoint_f2fs_readpage+8
	ldr	x20, [x0]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_readpage+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB52_9
// %bb.4:
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
	tbz	w8, #0, .LBB52_9
// %bb.5:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w8, [x21, #24]
	add	w8, w8, #1              // =1
	str	w8, [x21, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_readpage+32
	ldr	x22, [x8, :lo12:__tracepoint_f2fs_readpage+32]
	cbz	x22, .LBB52_7
.LBB52_6:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x22]
	mov	x1, x19
	mov	w2, wzr
	blr	x8
	ldr	x8, [x22, #24]!
	cbnz	x8, .LBB52_6
.LBB52_7:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	b.eq	.LBB52_18
// %bb.8:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB52_18
.LBB52_9:
	mov	x0, x20
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB52_16
// %bb.10:
	ldr	x8, [x20, #736]
	tbz	w8, #10, .LBB52_12
// %bb.11:
	mov	x0, x20
	mov	x1, x19
	bl	f2fs_read_inline_data
	cmn	w0, #11                 // =11
	b.ne	.LBB52_17
.LBB52_12:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB52_14
// %bb.13:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB52_20
.LBB52_14:
	ldr	x0, [x19, #24]
.LBB52_15:
	mov	w3, #1
	mov	x1, xzr
	mov	x2, x19
	mov	w4, wzr
	bl	f2fs_mpage_readpages
	mov	w0, wzr
	b	.LBB52_17
.LBB52_16:
	mov	x0, x19
	bl	unlock_page
	mov	w0, #-95
.LBB52_17:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB52_18:
	bl	preempt_schedule_notrace
	b	.LBB52_9
.LBB52_19:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB52_3
.LBB52_20:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB52_15
.Lfunc_end52:
	.size	f2fs_read_data_page, .Lfunc_end52-f2fs_read_data_page
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_write_data_pages
	.type	f2fs_write_data_pages,@function
f2fs_write_data_pages:                  // @f2fs_write_data_pages
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	mov	x29, sp
	sub	sp, sp, #432            // =432
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	//APP
	mrs x24, sp_el0
	//NO_APP
	mov	w10, #9
	stur	x8, [x29, #-16]
	ldr	x23, [x0]
	str	x0, [sp, #56]           // 8-byte Folded Spill
	ldr	x8, [x23, #40]
	ldr	x9, [x23, #816]
	ldr	x25, [x8, #1008]
	stp	xzr, xzr, [sp, #120]
	stp	xzr, xzr, [sp, #104]
	stp	xzr, xzr, [sp, #88]
	stp	xzr, xzr, [sp, #72]
	ldr	x8, [x0, #120]
	cmp	x9, x24
	mov	w9, #4
	csel	w9, w10, w9, eq
	ldr	x8, [x8]
	str	w9, [sp, #68]           // 4-byte Folded Spill
	cbz	x8, .LBB53_148
// %bb.1:
	ldr	w8, [x23, #792]
	mov	x19, x1
	cbnz	w8, .LBB53_3
// %bb.2:
	ldr	w8, [x19, #32]
	cbz	w8, .LBB53_148
.LBB53_3:
	ldr	x8, [x25, #80]
	adrp	x21, __tracepoint_f2fs_writepages+8
	tbnz	w8, #3, .LBB53_142
// %bb.4:
	ldrh	w8, [x23]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB53_6
// %bb.5:
	ldrb	w8, [x23, #12]
	tbz	w8, #5, .LBB53_7
.LBB53_6:
	ldr	w8, [x19, #32]
	cbz	w8, .LBB53_28
.LBB53_7:
	ldr	x8, [x23, #736]
	tbz	x8, #37, .LBB53_19
// %bb.8:
	ldr	w8, [x19, #32]
	cbnz	w8, .LBB53_19
// %bb.9:
	ldr	w8, [x23, #792]
	cmp	w8, #4                  // =4
	b.gt	.LBB53_19
// %bb.10:
	ldr	w8, [x23, #1296]
	cmp	w8, #64                 // =64
	b.hi	.LBB53_19
// %bb.11:
	ldr	w9, [x25, #1328]
	cmp	w9, #128                // =128
	b.gt	.LBB53_19
// %bb.12:
	add	w8, w8, #1              // =1
	str	w8, [x23, #1296]
	adrp	x8, __tracepoint_f2fs_skip_log_writeback+8
	ldr	w20, [x23, #64]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_skip_log_writeback+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB53_142
// %bb.13:
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
	tbz	w8, #0, .LBB53_142
// %bb.14:
	ldr	w8, [x24, #24]
	add	w8, w8, #1              // =1
	str	w8, [x24, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_skip_log_writeback+32
	ldr	x22, [x8, :lo12:__tracepoint_f2fs_skip_log_writeback+32]
	cbz	x22, .LBB53_16
.LBB53_15:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x22]
	mov	w1, w20
	blr	x8
	ldr	x8, [x22, #24]!
	cbnz	x8, .LBB53_15
.LBB53_16:
	//APP
	//NO_APP
	ldr	x8, [x24, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x24, #24]
	b.eq	.LBB53_18
// %bb.17:
	ldr	x8, [x24, #24]
	cbnz	x8, .LBB53_142
.LBB53_18:
	bl	preempt_schedule_notrace
	b	.LBB53_142
.LBB53_19:
	str	wzr, [x23, #1296]
	ldr	x8, [x23, #736]
	tbnz	w8, #22, .LBB53_142
// %bb.20:
	ldr	x8, [sp, #56]           // 8-byte Folded Reload
	ldr	x20, [x8]
	ldr	w8, [x21, :lo12:__tracepoint_f2fs_writepages+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB53_26
// %bb.21:
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
	tbz	w8, #0, .LBB53_26
// %bb.22:
	ldr	w8, [x24, #24]
	add	w8, w8, #1              // =1
	str	w8, [x24, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_writepages+32
	ldr	x22, [x8, :lo12:__tracepoint_f2fs_writepages+32]
	cbz	x22, .LBB53_24
.LBB53_23:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x22]
	mov	x1, x20
	mov	x2, x19
	mov	w3, wzr
	blr	x8
	ldr	x8, [x22, #24]!
	cbnz	x8, .LBB53_23
.LBB53_24:
	//APP
	//NO_APP
	ldr	x8, [x24, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x24, #24]
	b.eq	.LBB53_156
// %bb.25:
	ldr	x8, [x24, #24]
	cbz	x8, .LBB53_156
.LBB53_26:
	ldr	w8, [x19, #32]
	cmp	w8, #1                  // =1
	b.ne	.LBB53_30
// %bb.27:
	add	x8, x25, #1376          // =1376
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB53_31
.LBB53_28:
	ldr	w8, [x23, #792]
	ldr	x9, [x25]
	ldr	x9, [x9, #256]
	ldr	w9, [x9, #496]
	cbz	w9, .LBB53_140
// %bb.29:
	mov	w9, wzr
	cmp	w8, w9
	b.lt	.LBB53_141
	b	.LBB53_7
.LBB53_30:
	ldr	w8, [x25, #1376]
	cbnz	w8, .LBB53_142
.LBB53_31:
	ldr	x8, [x23, #816]
	stp	x25, x23, [sp, #16]     // 16-byte Folded Spill
	str	x24, [sp]               // 8-byte Folded Spill
	cbnz	x8, .LBB53_37
// %bb.32:
	ldrh	w8, [x23]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB53_37
// %bb.33:
	ldrb	w8, [x23, #12]
	tbnz	w8, #5, .LBB53_37
// %bb.34:
	ldr	x8, [x23, #40]
	ldr	x8, [x8, #1008]
	ldr	w9, [x8, #1532]
	ldr	x10, [x23, #736]
	tbnz	w10, #31, .LBB53_151
.LBB53_35:
	ldr	w9, [x19, #32]
	cmp	w9, #1                  // =1
	b.ne	.LBB53_154
// %bb.36:
	ldr	w9, [x23, #792]
	ldr	x8, [x8, #136]
	ldr	w8, [x8, #148]
	cmp	w9, w8
	b.hs	.LBB53_154
.LBB53_37:
	str	wzr, [sp, #12]          // 4-byte Folded Spill
.LBB53_38:
	add	x0, sp, #72             // =72
	bl	blk_start_plug
	ldr	x8, [sp, #56]           // 8-byte Folded Reload
	stp	xzr, xzr, [x29, #-32]
	stp	xzr, xzr, [x29, #-48]
	stp	xzr, xzr, [x29, #-64]
	stp	xzr, xzr, [x29, #-80]
	stp	xzr, xzr, [x29, #-96]
	stp	xzr, xzr, [x29, #-112]
	stp	xzr, xzr, [x29, #-128]
	stp	xzr, xzr, [x29, #-144]
	ldr	x25, [x8]
	mov	w9, #-1
	ldr	x8, [x25, #40]
	ldr	x24, [x8, #1008]
	stp	xzr, xzr, [x29, #-168]
	stp	xzr, xzr, [sp, #200]
	stp	xzr, xzr, [sp, #184]
	stp	x9, xzr, [sp, #168]
	stp	xzr, x25, [sp, #152]
	ldr	w8, [x25, #1292]
	str	w8, [sp, #176]
	ldrb	w9, [x25, #1285]
	lsl	x8, x8, #12
	str	xzr, [sp, #184]
	str	wzr, [sp, #192]
	str	xzr, [sp, #200]
	str	wzr, [sp, #208]
	stp	xzr, xzr, [sp, #216]
	stp	x8, xzr, [sp, #232]
	stp	xzr, xzr, [sp, #248]
	str	wzr, [sp, #148]
	sturh	wzr, [x29, #-144]
	str	w9, [sp, #180]
	ldr	w8, [x25, #792]
	ldr	x9, [x25, #40]
	ldr	x9, [x9, #1008]
	ldr	x9, [x9, #136]
	ldr	w9, [x9, #152]
	cmp	w8, w9
	b.ls	.LBB53_41
// %bb.39:
	add	x8, x25, #736           // =736
	mov	w9, #33554432
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldrh	w9, [x19, #36]
	tbz	w9, #4, .LBB53_42
.LBB53_40:
	ldr	x8, [sp, #56]           // 8-byte Folded Reload
	str	wzr, [sp, #8]           // 4-byte Folded Spill
	mov	x23, #-1
	ldr	x26, [x8, #112]
	str	x26, [sp, #152]
	b	.LBB53_43
.LBB53_41:
	add	x8, x25, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 33554432
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrh	w9, [x19, #36]
	tbnz	w9, #4, .LBB53_40
.LBB53_42:
	ldp	x8, x10, [x19, #16]
	mov	x11, #9223372036854775807
	cmp	x8, #0                  // =0
	asr	x26, x8, #12
	cset	w8, eq
	cmp	x10, x11
	asr	x23, x10, #12
	cset	w10, eq
	and	w8, w8, w10
	str	x26, [sp, #152]
	str	w8, [sp, #8]            // 4-byte Folded Spill
.LBB53_43:
	ldr	w8, [x19, #32]
	lsr	w9, w9, #1
	mov	w10, #2
	and	w9, w9, #0x2
	cmp	w8, #1                  // =1
	mov	w27, wzr
	mov	w20, wzr
	mov	w22, wzr
	csel	w9, w10, w9, eq
	str	w9, [sp, #44]           // 4-byte Folded Spill
	cmp	w8, #1                  // =1
	b.eq	.LBB53_47
	b	.LBB53_45
.LBB53_44:
	str	xzr, [sp, #152]
	ldr	w8, [x19, #32]
	mov	x26, xzr
	mov	x23, #-1
	cmp	w8, #1                  // =1
	b.eq	.LBB53_47
.LBB53_45:
	ldrb	w8, [x19, #36]
	tbnz	w8, #2, .LBB53_47
// %bb.46:
	mov	w21, wzr
	cbz	w20, .LBB53_48
	b	.LBB53_117
.LBB53_47:
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	mov	x1, x26
	mov	x2, x23
	bl	tag_pages_for_writeback
	ldr	x26, [sp, #152]
	mov	w21, wzr
	cbnz	w20, .LBB53_117
.LBB53_48:
	cmp	x26, x23
	b.hi	.LBB53_117
// %bb.49:
	ldr	x1, [sp, #56]           // 8-byte Folded Reload
	ldr	w4, [sp, #44]           // 4-byte Folded Reload
	sub	x0, x29, #144           // =144
	add	x2, sp, #152            // =152
	mov	x3, x23
	bl	pagevec_lookup_range_tag
	cbz	w0, .LBB53_116
// %bb.50:
	str	x23, [sp, #32]          // 8-byte Folded Spill
.LBB53_51:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB53_53 Depth 2
                                        //       Child Loop BB53_56 Depth 3
	cmp	w0, #1                  // =1
	b.lt	.LBB53_104
// %bb.52:                              //   in Loop: Header=BB53_51 Depth=1
	mov	x28, xzr
	mov	w8, w0
	str	x8, [sp, #48]           // 8-byte Folded Spill
.LBB53_53:                              //   Parent Loop BB53_51 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB53_56 Depth 3
	sub	x8, x29, #144           // =144
	add	x8, x8, x28, lsl #3
	ldr	x20, [x8, #8]
	mov	w23, w22
	b	.LBB53_56
.LBB53_54:                              //   in Loop: Header=BB53_56 Depth=3
	cmp	w22, #128, lsl #12      // =524288
	b.ne	.LBB53_96
.LBB53_55:                              //   in Loop: Header=BB53_56 Depth=3
	mov	w23, wzr
	mov	w22, wzr
	tbz	w21, #0, .LBB53_102
.LBB53_56:                              //   Parent Loop BB53_51 Depth=1
                                        //     Parent Loop BB53_53 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	ldr	x8, [x25, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB53_60
// %bb.57:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	w8, [x25, #1280]
	cbnz	w8, .LBB53_63
// %bb.58:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x25, #736]
	tbnz	x8, #35, .LBB53_63
// %bb.59:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x25, #736]
	tbz	x8, #34, .LBB53_62
	b	.LBB53_63
.LBB53_60:                              //   in Loop: Header=BB53_56 Depth=3
	ldrh	w8, [x25]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB53_62
// %bb.61:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x25, #736]
	tbnz	w8, #31, .LBB53_63
.LBB53_62:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	w8, [x24, #1376]
	cbnz	w8, .LBB53_71
	b	.LBB53_74
.LBB53_63:                              //   in Loop: Header=BB53_56 Depth=3
	add	x0, sp, #160            // =160
	bl	f2fs_init_compress_ctx
	cbnz	w0, .LBB53_105
// %bb.64:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x1, [x20, #32]
	add	x0, sp, #160            // =160
	bl	f2fs_cluster_can_merge_page
	tbz	w0, #0, .LBB53_72
// %bb.65:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x24, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB53_73
// %bb.66:                              //   in Loop: Header=BB53_56 Depth=3
	add	x0, sp, #160            // =160
	bl	f2fs_cluster_is_empty
	tbz	w0, #0, .LBB53_73
// %bb.67:                              //   in Loop: Header=BB53_56 Depth=3
	stur	xzr, [x29, #-152]
	str	xzr, [sp, #136]
	ldr	x2, [x20, #32]
	add	x1, sp, #136            // =136
	sub	x3, x29, #152           // =152
	mov	x0, x25
	bl	f2fs_prepare_compress_overwrite
	mov	w22, w0
	tbnz	w0, #31, .LBB53_106
// %bb.68:                              //   in Loop: Header=BB53_56 Depth=3
	cbz	w22, .LBB53_70
// %bb.69:                              //   in Loop: Header=BB53_56 Depth=3
	ldur	x1, [x29, #-152]
	ldr	x2, [x20, #32]
	mov	w3, #1
	mov	x0, x25
	mov	w21, #1
	bl	f2fs_compress_write_end
	tbz	w0, #0, .LBB53_109
.LBB53_70:                              //   in Loop: Header=BB53_56 Depth=3
	mov	w23, wzr
	ldr	w8, [x24, #1376]
	cbz	w8, .LBB53_74
.LBB53_71:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	w8, [x19, #32]
	cbnz	w8, .LBB53_74
	b	.LBB53_107
.LBB53_72:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	w3, [sp, #68]           // 4-byte Folded Reload
	add	x0, sp, #160            // =160
	add	x1, sp, #148            // =148
	mov	x2, x19
	bl	f2fs_write_multi_pages
	cmp	w0, #0                  // =0
	mov	w22, w0
	cset	w21, eq
	b	.LBB53_93
.LBB53_73:                              //   in Loop: Header=BB53_56 Depth=3
	mov	w23, wzr
.LBB53_74:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x26, [x20, #32]
	mov	w21, wzr
.LBB53_75:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x9, [x8]
	stur	x9, [x29, #-152]
	ldur	x9, [x29, #-152]
	tbnz	w9, #0, .LBB53_77
// %bb.76:                              //   in Loop: Header=BB53_56 Depth=3
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbz	w9, #0, .LBB53_78
.LBB53_77:                              //   in Loop: Header=BB53_56 Depth=3
	mov	x0, x20
	bl	__lock_page
.LBB53_78:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x20, #24]
	ldr	x9, [sp, #56]           // 8-byte Folded Reload
	cmp	x8, x9
	b.ne	.LBB53_99
// %bb.79:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbz	w8, #3, .LBB53_99
// %bb.80:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbz	w8, #15, .LBB53_83
// %bb.81:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	w8, [x19, #32]
	cbz	w8, .LBB53_99
// %bb.82:                              //   in Loop: Header=BB53_56 Depth=3
	mov	w2, #1
	mov	w3, #1
	mov	x0, x20
	mov	w1, wzr
	bl	f2fs_wait_on_page_writeback
.LBB53_83:                              //   in Loop: Header=BB53_56 Depth=3
	mov	x0, x20
	bl	clear_page_dirty_for_io
	cbz	w0, .LBB53_99
// %bb.84:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x25, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB53_88
// %bb.85:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	w8, [x25, #1280]
	cbnz	w8, .LBB53_100
// %bb.86:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x25, #736]
	tbnz	x8, #35, .LBB53_100
// %bb.87:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x25, #736]
	tbz	x8, #34, .LBB53_90
	b	.LBB53_100
.LBB53_88:                              //   in Loop: Header=BB53_56 Depth=3
	ldrh	w8, [x25]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB53_90
// %bb.89:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	x8, [x25, #736]
	tbnz	w8, #31, .LBB53_100
.LBB53_90:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	w5, [sp, #68]           // 4-byte Folded Reload
	add	x1, sp, #148            // =148
	sub	x2, x29, #160           // =160
	sub	x3, x29, #168           // =168
	mov	w7, #1
	mov	x0, x20
	mov	x4, x19
	mov	w6, wzr
	bl	f2fs_write_single_data_page
	cmp	w0, #128, lsl #12       // =524288
	b.ne	.LBB53_92
// %bb.91:                              //   in Loop: Header=BB53_56 Depth=3
	mov	x0, x20
	bl	unlock_page
	ldrsw	x8, [sp, #148]
	ldr	x9, [x19]
	mov	w23, wzr
	mov	w22, wzr
	add	w27, w8, w27
	sub	x8, x9, x8
	str	x8, [x19]
	tbnz	w21, #0, .LBB53_56
	b	.LBB53_102
.LBB53_92:                              //   in Loop: Header=BB53_56 Depth=3
	mov	w22, w0
.LBB53_93:                              //   in Loop: Header=BB53_56 Depth=3
	ldrsw	x8, [sp, #148]
	ldr	x9, [x19]
	add	w27, w8, w27
	sub	x8, x9, x8
	str	x8, [x19]
	cbnz	w22, .LBB53_54
// %bb.94:                              //   in Loop: Header=BB53_56 Depth=3
	cmp	x8, #0                  // =0
	b.gt	.LBB53_55
// %bb.95:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	w8, [x19, #32]
	cbnz	w8, .LBB53_55
	b	.LBB53_108
.LBB53_96:                              //   in Loop: Header=BB53_56 Depth=3
	cmn	w22, #11                // =11
	b.ne	.LBB53_115
// %bb.97:                              //   in Loop: Header=BB53_56 Depth=3
	ldr	w8, [x19, #32]
	cmp	w8, #1                  // =1
	b.ne	.LBB53_55
// %bb.98:                              //   in Loop: Header=BB53_56 Depth=3
	mov	w1, #5
	mov	w0, wzr
	bl	congestion_wait
	mov	w23, wzr
	b	.LBB53_75
.LBB53_99:                              //   in Loop: Header=BB53_53 Depth=2
	mov	x0, x20
	bl	unlock_page
	b	.LBB53_101
.LBB53_100:                             //   in Loop: Header=BB53_53 Depth=2
	ldr	x8, [x20, #8]
	add	x0, sp, #160            // =160
	mov	x1, x20
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	add	x8, x8, #52             // =52
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	bl	f2fs_compress_ctx_add_page
.LBB53_101:                             //   in Loop: Header=BB53_53 Depth=2
	mov	w22, w23
.LBB53_102:                             //   in Loop: Header=BB53_53 Depth=2
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	add	x28, x28, #1            // =1
	cmp	x28, x8
	b.ne	.LBB53_53
// %bb.103:                             //   in Loop: Header=BB53_51 Depth=1
	mov	w21, wzr
	mov	w20, wzr
	b	.LBB53_110
.LBB53_104:                             //   in Loop: Header=BB53_51 Depth=1
	mov	w21, wzr
	mov	w20, wzr
	ldurb	w8, [x29, #-144]
	cbnz	w8, .LBB53_111
	b	.LBB53_112
.LBB53_105:                             //   in Loop: Header=BB53_51 Depth=1
	mov	w22, w0
.LBB53_106:                             //   in Loop: Header=BB53_51 Depth=1
	mov	w21, wzr
	mov	w20, #1
	b	.LBB53_110
.LBB53_107:                             //   in Loop: Header=BB53_51 Depth=1
	mov	w21, wzr
	mov	w20, #1
	mov	w22, w23
	b	.LBB53_110
.LBB53_108:                             //   in Loop: Header=BB53_51 Depth=1
	mov	w21, wzr
	mov	w22, wzr
	mov	w20, #1
	b	.LBB53_110
.LBB53_109:                             //   in Loop: Header=BB53_51 Depth=1
	mov	w20, wzr
	mov	w22, wzr
.LBB53_110:                             //   in Loop: Header=BB53_51 Depth=1
	ldr	x23, [sp, #32]          // 8-byte Folded Reload
	ldurb	w8, [x29, #-144]
	cbz	w8, .LBB53_112
.LBB53_111:                             //   in Loop: Header=BB53_51 Depth=1
	sub	x0, x29, #144           // =144
	bl	__pagevec_release
.LBB53_112:                             //   in Loop: Header=BB53_51 Depth=1
	orr	w8, w20, w21
	cbnz	w8, .LBB53_117
// %bb.113:                             //   in Loop: Header=BB53_51 Depth=1
	ldr	x8, [sp, #152]
	cmp	x8, x23
	b.hi	.LBB53_117
// %bb.114:                             //   in Loop: Header=BB53_51 Depth=1
	ldr	x1, [sp, #56]           // 8-byte Folded Reload
	ldr	w4, [sp, #44]           // 4-byte Folded Reload
	sub	x0, x29, #144           // =144
	add	x2, sp, #152            // =152
	mov	x3, x23
	bl	pagevec_lookup_range_tag
	cbnz	w0, .LBB53_51
	b	.LBB53_117
.LBB53_115:                             //   in Loop: Header=BB53_51 Depth=1
	ldr	x8, [x20, #32]
	mov	w21, wzr
	mov	w20, #1
	add	x26, x8, #1             // =1
	b	.LBB53_110
.LBB53_116:
	mov	w21, wzr
	mov	w20, wzr
.LBB53_117:
	ldrh	w8, [x25]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB53_121
// %bb.118:
	ldr	x8, [x25, #736]
	tbz	w8, #31, .LBB53_121
// %bb.119:
	add	x0, sp, #160            // =160
	bl	f2fs_cluster_is_empty
	tbnz	w0, #0, .LBB53_121
// %bb.120:
	ldr	w3, [sp, #68]           // 4-byte Folded Reload
	add	x0, sp, #160            // =160
	add	x1, sp, #148            // =148
	mov	x2, x19
	bl	f2fs_write_multi_pages
	ldrsw	x8, [sp, #148]
	ldr	x9, [x19]
	cmp	w0, #0                  // =0
	mov	w22, w0
	csel	w21, w21, wzr, eq
	add	w27, w8, w27
	sub	x8, x9, x8
	csinc	w20, w20, wzr, eq
	str	x8, [x19]
.LBB53_121:
	ldrh	w8, [x25]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB53_124
// %bb.122:
	ldr	x8, [x25, #736]
	tbz	w8, #31, .LBB53_124
// %bb.123:
	add	x0, sp, #160            // =160
	mov	w1, wzr
	bl	f2fs_destroy_compress_ctx
.LBB53_124:
	cbnz	w21, .LBB53_44
// %bb.125:
	ldrh	w9, [x19, #36]
	tst	w9, #0x10
	ccmp	w20, #0, #0, ne
	csel	x8, x26, xzr, ne
	tbnz	w9, #4, .LBB53_128
// %bb.126:
	ldr	w9, [sp, #8]            // 4-byte Folded Reload
	cbz	w9, .LBB53_129
// %bb.127:
	ldr	x9, [x19]
	cmp	x9, #1                  // =1
	b.lt	.LBB53_129
.LBB53_128:
	ldr	x9, [sp, #56]           // 8-byte Folded Reload
	str	x8, [x9, #112]
.LBB53_129:
	ldr	x20, [sp, #16]          // 8-byte Folded Reload
	cbz	w27, .LBB53_133
// %bb.130:
	adrp	x8, write_opt
	ldr	w8, [x8, :lo12:write_opt]
	cbz	w8, .LBB53_132
// %bb.131:
	ldr	x8, [sp]                // 8-byte Folded Reload
	ldr	x8, [x8, #1480]
	tbnz	w8, #28, .LBB53_133
.LBB53_132:
	ldr	x8, [sp, #56]           // 8-byte Folded Reload
	mov	x2, xzr
	mov	w3, wzr
	mov	w4, wzr
	ldr	x1, [x8]
	mov	w5, wzr
	ldr	x8, [x1, #40]
	ldr	x0, [x8, #1008]
	bl	__submit_merged_write_cond
.LBB53_133:
	ldur	x8, [x29, #-160]
	cbz	x8, .LBB53_135
// %bb.134:
	sub	x1, x29, #160           // =160
	mov	x0, x24
	mov	x2, xzr
	bl	f2fs_submit_merged_ipu_write
.LBB53_135:
	add	x0, sp, #72             // =72
	bl	blk_finish_plug
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	cbz	w8, .LBB53_137
// %bb.136:
	add	x0, x20, #88            // =88
	bl	mutex_unlock
.LBB53_137:
	ldr	w8, [x19, #32]
	ldr	x0, [sp, #24]           // 8-byte Folded Reload
	cmp	w8, #1                  // =1
	b.ne	.LBB53_139
// %bb.138:
	add	x8, x20, #1376          // =1376
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
.LBB53_139:
	bl	f2fs_remove_dirty_inode
	b	.LBB53_149
.LBB53_140:
	ldr	w9, [x25, #1188]
	cmp	w8, w9
	b.ge	.LBB53_7
.LBB53_141:
	mov	w1, #2
	mov	x0, x25
	bl	f2fs_available_free_memory
	tbz	w0, #0, .LBB53_7
.LBB53_142:
	ldrsw	x8, [x23, #792]
	ldr	x9, [x19, #8]
	add	x8, x9, x8
	str	x8, [x19, #8]
	ldr	x8, [sp, #56]           // 8-byte Folded Reload
	ldr	x20, [x8]
	ldr	w8, [x21, :lo12:__tracepoint_f2fs_writepages+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB53_148
// %bb.143:
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
	tbz	w8, #0, .LBB53_148
// %bb.144:
	ldr	w8, [x24, #24]
	add	w8, w8, #1              // =1
	str	w8, [x24, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_writepages+32
	ldr	x21, [x8, :lo12:__tracepoint_f2fs_writepages+32]
	cbz	x21, .LBB53_146
.LBB53_145:                             // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	mov	x1, x20
	mov	x2, x19
	mov	w3, wzr
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB53_145
.LBB53_146:
	//APP
	//NO_APP
	ldr	x8, [x24, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x24, #24]
	b.eq	.LBB53_155
// %bb.147:
	ldr	x8, [x24, #24]
	cbz	x8, .LBB53_155
.LBB53_148:
	mov	w22, wzr
.LBB53_149:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB53_157
// %bb.150:
	mov	w0, w22
	add	sp, sp, #432            // =432
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.LBB53_151:
	cbz	w9, .LBB53_154
// %bb.152:
	cmp	w9, #1                  // =1
	b.ne	.LBB53_35
// %bb.153:
	ldr	x9, [x23, #736]
	tbz	x9, #34, .LBB53_35
.LBB53_154:
	add	x0, x25, #88            // =88
	bl	mutex_lock
	mov	w8, #1
	str	w8, [sp, #12]           // 4-byte Folded Spill
	b	.LBB53_38
.LBB53_155:
	bl	preempt_schedule_notrace
	b	.LBB53_148
.LBB53_156:
	bl	preempt_schedule_notrace
	b	.LBB53_26
.LBB53_157:
	bl	__stack_chk_fail
.Lfunc_end53:
	.size	f2fs_write_data_pages, .Lfunc_end53-f2fs_write_data_pages
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_set_data_page_dirty
	.type	f2fs_set_data_page_dirty,@function
f2fs_set_data_page_dirty:               // @f2fs_set_data_page_dirty
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #8]
	mov	x19, x0
	mov	x29, sp
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x0, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB54_2
// %bb.1:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB54_27
.LBB54_2:
	ldr	x0, [x19, #24]
.LBB54_3:
	adrp	x8, __tracepoint_f2fs_set_page_dirty+8
	ldr	x20, [x0]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_set_page_dirty+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB54_9
// %bb.4:
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
	tbz	w8, #0, .LBB54_9
// %bb.5:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w8, [x21, #24]
	add	w8, w8, #1              // =1
	str	w8, [x21, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_set_page_dirty+32
	ldr	x22, [x8, :lo12:__tracepoint_f2fs_set_page_dirty+32]
	cbz	x22, .LBB54_7
.LBB54_6:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x22]
	mov	x1, x19
	mov	w2, wzr
	blr	x8
	ldr	x8, [x22, #24]!
	cbnz	x8, .LBB54_6
.LBB54_7:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	b.eq	.LBB54_26
// %bb.8:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB54_26
.LBB54_9:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB54_11
// %bb.10:
	//APP
	dmb ishst
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x19]
1:	ldxr	x8, [x19]
	orr	x8, x8, 4
	stxr	w9, x8, [x19]
	cbnz	w9, 1b
	//NO_APP
	b	.LBB54_12
.LBB54_11:
	//APP
	dmb ishld
	//NO_APP
.LBB54_12:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB54_14
// %bb.13:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB54_24
.LBB54_14:
	ldr	x8, [x20, #736]
	tbz	w8, #15, .LBB54_20
// %bb.15:
	ldr	x8, [x20, #736]
	tbnz	w8, #16, .LBB54_20
// %bb.16:
	ldr	x8, [x19]
	tbz	w8, #13, .LBB54_19
// %bb.17:
	ldr	x8, [x19, #40]
	tbz	w8, #0, .LBB54_19
// %bb.18:
	ldr	x8, [x19, #40]
	tbnz	w8, #1, .LBB54_23
.LBB54_19:
	mov	x0, x20
	mov	x1, x19
	bl	f2fs_register_inmem_page
	b	.LBB54_22
.LBB54_20:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #3, .LBB54_23
// %bb.21:
	mov	x0, x19
	bl	__set_page_dirty_nobuffers
	mov	x0, x20
	mov	x1, x19
	bl	f2fs_update_dirty_page
.LBB54_22:
	mov	w0, #1
	b	.LBB54_25
.LBB54_23:
	mov	w0, wzr
	b	.LBB54_25
.LBB54_24:
	mov	x0, x19
	bl	__set_page_dirty_nobuffers
.LBB54_25:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB54_26:
	bl	preempt_schedule_notrace
	b	.LBB54_9
.LBB54_27:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB54_3
.Lfunc_end54:
	.size	f2fs_set_data_page_dirty, .Lfunc_end54-f2fs_set_data_page_dirty
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_read_data_pages
	.type	f2fs_read_data_pages,@function
f2fs_read_data_pages:                   // @f2fs_read_data_pages
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	str	x25, [sp, #16]          // 8-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	adrp	x9, __tracepoint_f2fs_readpages+8
	ldr	x22, [x1]
	ldr	x8, [x2, #8]
	ldr	w9, [x9, :lo12:__tracepoint_f2fs_readpages+8]
	mov	x19, x2
	mov	x20, x1
	mov	w21, w3
	cmp	w9, #1                  // =1
	mov	x29, sp
	b.lt	.LBB55_7
// %bb.1:
	adrp	x10, cpu_number
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
	adrp	x11, __cpu_online_mask
	add	x11, x11, :lo12:__cpu_online_mask
	add	w10, w9, #63            // =63
	cmp	w9, #0                  // =0
	csel	w10, w10, w9, lt
	asr	w10, w10, #6
	ldr	x10, [x11, w10, sxtw #3]
	lsr	x9, x10, x9
	tbz	w9, #0, .LBB55_7
// %bb.2:
	//APP
	mrs x24, sp_el0
	//NO_APP
	ldr	w9, [x24, #24]
	add	w9, w9, #1              // =1
	str	w9, [x24, #24]
	//APP
	//NO_APP
	adrp	x9, __tracepoint_f2fs_readpages+32
	ldr	x25, [x9, :lo12:__tracepoint_f2fs_readpages+32]
	cbz	x25, .LBB55_5
// %bb.3:
	sub	x23, x8, #8             // =8
.LBB55_4:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x25]
	mov	x1, x22
	mov	x2, x23
	mov	w3, w21
	blr	x8
	ldr	x8, [x25, #24]!
	cbnz	x8, .LBB55_4
.LBB55_5:
	//APP
	//NO_APP
	ldr	x8, [x24, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x24, #24]
	b.eq	.LBB55_10
// %bb.6:
	ldr	x8, [x24, #24]
	cbz	x8, .LBB55_10
.LBB55_7:
	ldr	x8, [x22, #736]
	tbnz	w8, #10, .LBB55_9
.LBB55_8:
	mov	w4, #1
	mov	x0, x20
	mov	x1, x19
	mov	x2, xzr
	mov	w3, w21
	bl	f2fs_mpage_readpages
.LBB55_9:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	mov	w0, wzr
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB55_10:
	bl	preempt_schedule_notrace
	ldr	x8, [x22, #736]
	tbz	w8, #10, .LBB55_8
	b	.LBB55_9
.Lfunc_end55:
	.size	f2fs_read_data_pages, .Lfunc_end55-f2fs_read_data_pages
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_write_begin
	.type	f2fs_write_begin,@function
f2fs_write_begin:                       // @f2fs_write_begin
// %bb.0:
	sub	sp, sp, #496            // =496
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #400]    // 16-byte Folded Spill
	add	x29, sp, #400           // =400
	stp	x28, x27, [sp, #416]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #432]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #448]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #464]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #480]    // 16-byte Folded Spill
	stp	x6, x5, [sp, #72]       // 16-byte Folded Spill
	stur	x8, [x29, #-16]
	ldr	x24, [x1]
	adrp	x28, __tracepoint_android_fs_datawrite_start+8
	adrp	x10, cpu_number
	adrp	x20, __cpu_online_mask
	ldr	x8, [x24, #40]
	mov	w23, w4
	mov	w19, w3
	mov	x22, x2
	ldr	x21, [x8, #1008]
	ldr	w8, [x28, :lo12:__tracepoint_android_fs_datawrite_start+8]
	mov	x26, x1
	add	x10, x10, :lo12:cpu_number
	add	x20, x20, :lo12:__cpu_online_mask
	cmp	w8, #1                  // =1
	b.lt	.LBB56_12
// %bb.1:
	mov	x0, x24
	str	x26, [sp, #88]          // 8-byte Folded Spill
	stp	xzr, xzr, [sp, #368]
	stp	xzr, xzr, [sp, #352]
	stp	xzr, xzr, [sp, #336]
	stp	xzr, xzr, [sp, #320]
	stp	xzr, xzr, [sp, #304]
	stp	xzr, xzr, [sp, #288]
	stp	xzr, xzr, [sp, #272]
	stp	xzr, xzr, [sp, #256]
	stp	xzr, xzr, [sp, #240]
	stp	xzr, xzr, [sp, #224]
	stp	xzr, xzr, [sp, #208]
	stp	xzr, xzr, [sp, #192]
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	stp	xzr, xzr, [sp, #128]
	bl	ihold
	mov	x0, x24
	bl	d_obtain_alias
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB56_143
// %bb.2:
	add	x1, sp, #128            // =128
	mov	w2, #256
	mov	x27, x0
	bl	dentry_path_raw
	mov	x26, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB56_144
.LBB56_3:
	mov	x0, x27
	bl	dput
.LBB56_4:
	//APP
	mrs x25, sp_el0
	//NO_APP
	ldr	w27, [x25, #1536]
	ldr	w8, [x28, :lo12:__tracepoint_android_fs_datawrite_start+8]
	adrp	x10, cpu_number
	add	x10, x10, :lo12:cpu_number
	cmp	w8, #1                  // =1
	b.lt	.LBB56_11
// %bb.5:
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
	ldr	w8, [x8, x10]
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x20, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB56_11
// %bb.6:
	ldr	w8, [x25, #24]
	mov	x20, x21
	add	w8, w8, #1              // =1
	str	w8, [x25, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_android_fs_datawrite_start+32
	ldr	x21, [x8, :lo12:__tracepoint_android_fs_datawrite_start+32]
	cbz	x21, .LBB56_9
// %bb.7:
	add	x28, x25, #2000         // =2000
.LBB56_8:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	mov	x1, x24
	mov	x2, x22
	mov	w3, w19
	mov	w4, w27
	mov	x5, x26
	mov	x6, x28
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB56_8
.LBB56_9:
	//APP
	//NO_APP
	ldr	x8, [x25, #24]
	mov	x21, x20
	adrp	x20, __cpu_online_mask
	adrp	x10, cpu_number
	subs	x8, x8, #1              // =1
	add	x20, x20, :lo12:__cpu_online_mask
	add	x10, x10, :lo12:cpu_number
	str	w8, [x25, #24]
	b.eq	.LBB56_145
// %bb.10:
	ldr	x8, [x25, #24]
	cbz	x8, .LBB56_145
.LBB56_11:
	ldr	x26, [sp, #88]          // 8-byte Folded Reload
.LBB56_12:
	adrp	x8, __tracepoint_f2fs_write_begin+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_write_begin+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB56_18
// %bb.13:
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
	ldr	w8, [x8, x10]
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x20, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB56_18
// %bb.14:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	mov	x25, x21
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_write_begin+32
	ldr	x21, [x8, :lo12:__tracepoint_f2fs_write_begin+32]
	cbz	x21, .LBB56_16
.LBB56_15:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	mov	x1, x24
	mov	x2, x22
	mov	w3, w19
	mov	w4, w23
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB56_15
.LBB56_16:
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	mov	x21, x25
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB56_140
// %bb.17:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB56_140
.LBB56_18:
	ldr	x8, [x21, #80]
	tbnz	w8, #8, .LBB56_141
.LBB56_19:
	ldr	x8, [x24, #736]
	tbz	w8, #15, .LBB56_21
// %bb.20:
	mov	w1, #5
	mov	x0, x21
	bl	f2fs_available_free_memory
	tbz	w0, #0, .LBB56_25
.LBB56_21:
	ldr	x8, [x24, #736]
	tbnz	w8, #29, .LBB56_25
// %bb.22:
	lsr	x20, x22, #12
	cbz	x20, .LBB56_30
// %bb.23:
	mov	x0, x24
	bl	f2fs_convert_inline_inode
	cbz	w0, .LBB56_30
// %bb.24:
	mov	w27, w0
	mov	x0, xzr
	mov	w20, wzr
	b	.LBB56_26
.LBB56_25:
	mov	x0, xzr
	mov	w27, #-12
	mov	w20, #1
.LBB56_26:
	mov	w1, #1
	bl	f2fs_put_page
	add	x1, x22, w19, uxtw
	mov	x0, x26
	bl	f2fs_write_failed
	cbz	w20, .LBB56_28
// %bb.27:
	mov	x0, x21
	mov	w1, wzr
	bl	f2fs_drop_inmem_pages_all
.LBB56_28:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB56_147
// %bb.29:
	mov	w0, w27
	ldp	x20, x19, [sp, #480]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #464]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #448]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #432]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #416]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #400]    // 16-byte Folded Reload
	add	sp, sp, #496            // =496
	ret
.LBB56_30:
	ldr	x8, [x24, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB56_34
// %bb.31:
	ldr	w8, [x24, #1280]
	cbnz	w8, .LBB56_36
// %bb.32:
	ldr	x8, [x24, #736]
	tbnz	x8, #35, .LBB56_36
// %bb.33:
	ldr	x8, [x24, #736]
	tbnz	x8, #34, .LBB56_36
	b	.LBB56_38
.LBB56_34:
	ldrh	w8, [x24]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB56_38
// %bb.35:
	ldr	x8, [x24, #736]
	tbz	w8, #31, .LBB56_38
.LBB56_36:
	ldp	x3, x1, [sp, #72]       // 16-byte Folded Reload
	mov	x0, x24
	mov	x2, x20
	str	xzr, [x3]
	bl	f2fs_prepare_compress_overwrite
	cmp	w0, #0                  // =0
	cset	w9, ne
	mov	w10, #2
	csel	w9, w10, w9, lt
	cmp	w9, #2                  // =2
	b.eq	.LBB56_125
// %bb.37:
	mov	w27, wzr
	cbnz	w9, .LBB56_28
.LBB56_38:
	mov	w2, #14
	mov	w3, #3136
	mov	x0, x26
	mov	x1, x20
	str	x20, [sp, #32]          // 8-byte Folded Spill
	bl	pagecache_get_page
	cbz	x0, .LBB56_124
// %bb.39:
	and	x9, x22, #0xfffffffffffff000
	str	x9, [sp, #24]           // 8-byte Folded Spill
	add	x9, x21, #304           // =304
	mov	w8, w19
	str	x9, [sp, #16]           // 8-byte Folded Spill
	add	x9, x21, #424           // =424
	mov	x25, x0
	mov	w23, wzr
	stp	x8, x9, [sp]            // 16-byte Folded Spill
	and	x9, x22, #0xfff
	add	x8, x8, x22
	str	wzr, [sp, #44]          // 4-byte Folded Spill
	stp	x9, x8, [sp, #48]       // 16-byte Folded Spill
.LBB56_40:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB56_56 Depth 2
	ldr	x8, [sp, #80]           // 8-byte Folded Reload
	str	w23, [sp, #40]          // 4-byte Folded Spill
	cmp	w19, #1, lsl #12        // =4096
	str	x25, [x8]
	ldp	x8, x28, [x25, #24]
	ldr	x23, [x8]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	stp	xzr, xzr, [sp, #128]
	str	wzr, [sp, #104]
	str	xzr, [sp, #96]
	ldr	x8, [x23, #736]
	b.ne	.LBB56_45
// %bb.41:                              //   in Loop: Header=BB56_40 Depth=1
	tbnz	w8, #10, .LBB56_45
// %bb.42:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x23, #736]
	tbnz	w8, #24, .LBB56_45
// %bb.43:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x23, #736]
	tbnz	w8, #30, .LBB56_45
// %bb.44:                              //   in Loop: Header=BB56_40 Depth=1
	ldp	w23, w20, [sp, #40]     // 8-byte Folded Reload
	b	.LBB56_101
.LBB56_45:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x23, #736]
	tbz	w8, #10, .LBB56_48
// %bb.46:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	w8, [x23, #1192]
	ldr	w9, [x23, #1184]
	mvn	w8, w8
	sub	w8, w8, w9, asr #2
	ldr	x9, [sp, #56]           // 8-byte Folded Reload
	add	w8, w8, #923            // =923
	cmp	x9, w8, sxtw #2
	b.ls	.LBB56_48
// %bb.47:                              //   in Loop: Header=BB56_40 Depth=1
	str	wzr, [sp, #88]          // 4-byte Folded Spill
	b	.LBB56_49
.LBB56_48:                              //   in Loop: Header=BB56_40 Depth=1
	mov	w8, #5
	str	w8, [sp, #88]           // 4-byte Folded Spill
.LBB56_49:                              //   in Loop: Header=BB56_40 Depth=1
	str	x25, [sp, #64]          // 8-byte Folded Spill
	ldr	x8, [x23, #736]
	tbnz	w8, #10, .LBB56_52
// %bb.50:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x23, #80]
	ldr	x9, [sp, #24]           // 8-byte Folded Reload
	cmp	x9, x8
	b.hs	.LBB56_52
// %bb.51:                              //   in Loop: Header=BB56_40 Depth=1
	mov	x20, x26
	mov	w25, wzr
	str	wzr, [sp, #72]          // 4-byte Folded Spill
	b	.LBB56_53
.LBB56_52:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	w1, [sp, #88]           // 4-byte Folded Reload
	mov	w2, #1
	mov	x0, x21
	mov	w3, wzr
	mov	x20, x26
	mov	w25, #1
	bl	f2fs_do_map_lock
	str	w0, [sp, #72]           // 4-byte Folded Spill
.LBB56_53:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x1, [x23, #64]
	mov	x0, x21
	bl	f2fs_get_node_page
	mov	x26, x0
	cmn	x0, #1, lsl #12         // =4096
	b.ls	.LBB56_55
// %bb.54:                              //   in Loop: Header=BB56_40 Depth=1
	mov	w27, w26
	mov	x26, x20
	ldp	w23, w20, [sp, #40]     // 8-byte Folded Reload
	tbnz	w25, #0, .LBB56_95
	b	.LBB56_100
.LBB56_55:                              //   in Loop: Header=BB56_40 Depth=1
	mov	w27, wzr
.LBB56_56:                              //   Parent Loop BB56_40 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	add	x0, sp, #128            // =128
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	stp	x23, x26, [sp, #128]
	str	x26, [sp, #144]
	str	wzr, [sp, #152]
	ldr	x8, [x23, #736]
	tbnz	w8, #10, .LBB56_70
// %bb.57:                              //   in Loop: Header=BB56_56 Depth=2
	tbnz	w25, #0, .LBB56_74
// %bb.58:                              //   in Loop: Header=BB56_56 Depth=2
	add	x2, sp, #96             // =96
	mov	x0, x23
	mov	x1, x28
	bl	f2fs_lookup_extent_cache
	mov	x26, x20
	tbnz	w0, #0, .LBB56_76
// %bb.59:                              //   in Loop: Header=BB56_56 Depth=2
	add	x0, sp, #128            // =128
	mov	w2, #1
	mov	x1, x28
	bl	f2fs_get_dnode_of_data
	mov	w27, w0
	cbnz	w0, .LBB56_61
// %bb.60:                              //   in Loop: Header=BB56_56 Depth=2
	ldr	w8, [sp, #164]
	cbnz	w8, .LBB56_73
.LBB56_61:                              //   in Loop: Header=BB56_56 Depth=2
	ldr	x0, [sp, #144]
	cbz	x0, .LBB56_63
// %bb.62:                              //   in Loop: Header=BB56_56 Depth=2
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #144]
	ldr	x8, [sp, #136]
	cbnz	x8, .LBB56_64
	b	.LBB56_67
.LBB56_63:                              //   in Loop: Header=BB56_56 Depth=2
	mov	x9, xzr
	ldr	x8, [sp, #136]
	cbz	x8, .LBB56_67
.LBB56_64:                              //   in Loop: Header=BB56_56 Depth=2
	cmp	x9, x8
	b.eq	.LBB56_67
// %bb.65:                              //   in Loop: Header=BB56_56 Depth=2
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
	cbnz	w9, .LBB56_67
// %bb.66:                              //   in Loop: Header=BB56_56 Depth=2
	bl	__put_page
.LBB56_67:                              //   in Loop: Header=BB56_56 Depth=2
	mov	w1, #5
	mov	w2, #1
	mov	x0, x21
	mov	w3, wzr
	mov	x20, x26
	stp	xzr, xzr, [sp, #136]
	bl	f2fs_do_map_lock
	ldr	w8, [sp, #88]           // 4-byte Folded Reload
	str	w0, [sp, #72]           // 4-byte Folded Spill
	cmp	w8, #5                  // =5
	b.ne	.LBB56_69
.LBB56_68:                              //   in Loop: Header=BB56_56 Depth=2
	ldr	x1, [x23, #64]
	mov	x0, x21
	bl	f2fs_get_node_page
	mov	x26, x0
	cmn	x0, #4095               // =4095
	mov	w25, #1
	b.lo	.LBB56_56
	b	.LBB56_78
.LBB56_69:                              //   in Loop: Header=BB56_56 Depth=2
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 5547; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	b	.LBB56_68
.LBB56_70:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	w8, [x23, #1192]
	ldr	w9, [x23, #1184]
	mvn	w8, w8
	sub	w8, w8, w9, asr #2
	ldr	x9, [sp, #56]           // 8-byte Folded Reload
	add	w8, w8, #923            // =923
	cmp	x9, w8, sxtw #2
	b.ls	.LBB56_79
// %bb.71:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x1, [sp, #64]           // 8-byte Folded Reload
	add	x0, sp, #128            // =128
	bl	f2fs_convert_inline_page
	mov	x26, x20
	ldp	w23, w20, [sp, #40]     // 8-byte Folded Reload
	mov	w27, w0
	cbnz	w0, .LBB56_87
// %bb.72:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	w8, [sp, #164]
	cbz	w8, .LBB56_84
.LBB56_73:                              //   in Loop: Header=BB56_40 Depth=1
	mov	w27, wzr
	b	.LBB56_86
.LBB56_74:                              //   in Loop: Header=BB56_40 Depth=1
	add	x2, sp, #112            // =112
	mov	x0, x23
	mov	x1, x28
	str	wzr, [sp, #120]
	str	xzr, [sp, #112]
	bl	f2fs_lookup_extent_cache
	mov	x26, x20
	tbz	w0, #0, .LBB56_85
.LBB56_75:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	w8, [sp, #120]
	ldr	w9, [sp, #112]
	mov	w27, wzr
	b	.LBB56_77
.LBB56_76:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	w8, [sp, #104]
	ldr	w9, [sp, #96]
.LBB56_77:                              //   in Loop: Header=BB56_40 Depth=1
	add	w8, w8, w28
	sub	w8, w8, w9
	str	w8, [sp, #164]
	b	.LBB56_86
.LBB56_78:                              //   in Loop: Header=BB56_40 Depth=1
	mov	w27, w26
	mov	x26, x20
	ldp	w23, w20, [sp, #40]     // 8-byte Folded Reload
	b	.LBB56_95
.LBB56_79:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	mov	x1, x26
	bl	f2fs_do_read_inline_data
	add	x8, x23, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 1048576
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_mark_inode_dirty_sync
	ldr	w8, [x23, #72]
	cbz	w8, .LBB56_83
// %bb.80:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x26]
	tbnz	w8, #13, .LBB56_82
// %bb.81:                              //   in Loop: Header=BB56_40 Depth=1
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
.LBB56_82:                              //   in Loop: Header=BB56_40 Depth=1
	add	x8, x26, #40            // =40
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
	orr	x9, x9, 16
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB56_83:                              //   in Loop: Header=BB56_40 Depth=1
	mov	x26, x20
	b	.LBB56_86
.LBB56_84:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x0, [sp, #128]
	add	x2, sp, #112            // =112
	mov	x1, x28
	str	wzr, [sp, #120]
	str	xzr, [sp, #112]
	bl	f2fs_lookup_extent_cache
	tbnz	w0, #0, .LBB56_75
.LBB56_85:                              //   in Loop: Header=BB56_40 Depth=1
	add	x0, sp, #128            // =128
	mov	x1, x28
	bl	f2fs_reserve_block
	mov	w27, w0
.LBB56_86:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	w23, [sp, #164]
	ldrb	w20, [sp, #161]
.LBB56_87:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x0, [sp, #144]
	cbz	x0, .LBB56_89
// %bb.88:                              //   in Loop: Header=BB56_40 Depth=1
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #144]
	b	.LBB56_90
.LBB56_89:                              //   in Loop: Header=BB56_40 Depth=1
	mov	x9, xzr
.LBB56_90:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [sp, #136]
	cbz	x8, .LBB56_94
// %bb.91:                              //   in Loop: Header=BB56_40 Depth=1
	cmp	x9, x8
	b.eq	.LBB56_94
// %bb.92:                              //   in Loop: Header=BB56_40 Depth=1
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
	cbnz	w9, .LBB56_94
// %bb.93:                              //   in Loop: Header=BB56_40 Depth=1
	bl	__put_page
.LBB56_94:                              //   in Loop: Header=BB56_40 Depth=1
	stp	xzr, xzr, [sp, #136]
	tbz	w25, #0, .LBB56_100
.LBB56_95:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	w8, [sp, #88]           // 4-byte Folded Reload
	cmp	w8, #5                  // =5
	b.ne	.LBB56_98
// %bb.96:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	w8, [sp, #72]           // 4-byte Folded Reload
	tbz	w8, #0, .LBB56_98
// %bb.97:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x0, [sp, #8]            // 8-byte Folded Reload
	b	.LBB56_99
.LBB56_98:                              //   in Loop: Header=BB56_40 Depth=1
	ldr	x0, [sp, #16]           // 8-byte Folded Reload
.LBB56_99:                              //   in Loop: Header=BB56_40 Depth=1
	bl	up_read
.LBB56_100:                             //   in Loop: Header=BB56_40 Depth=1
	ldr	x25, [sp, #64]          // 8-byte Folded Reload
	cbnz	w27, .LBB56_128
.LBB56_101:                             //   in Loop: Header=BB56_40 Depth=1
	tst	w20, #0xff
	b.eq	.LBB56_109
// %bb.102:                             //   in Loop: Header=BB56_40 Depth=1
	ldrb	w8, [x24, #12]
	tbnz	w8, #5, .LBB56_109
// %bb.103:                             //   in Loop: Header=BB56_40 Depth=1
	mov	x0, x21
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB56_109
// %bb.104:                             //   in Loop: Header=BB56_40 Depth=1
	mov	x0, x25
	bl	unlock_page
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_balance_fs
	ldr	x8, [x25, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x25, x9, eq
	ldr	x9, [x8]
	str	x9, [sp, #128]
	ldr	x9, [sp, #128]
	tbnz	w9, #0, .LBB56_106
// %bb.105:                             //   in Loop: Header=BB56_40 Depth=1
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbz	w9, #0, .LBB56_107
.LBB56_106:                             //   in Loop: Header=BB56_40 Depth=1
	mov	x0, x25
	bl	__lock_page
.LBB56_107:                             //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x25, #24]
	cmp	x8, x26
	b.eq	.LBB56_109
.LBB56_108:                             //   in Loop: Header=BB56_40 Depth=1
	mov	w1, #1
	mov	x0, x25
	str	w20, [sp, #44]          // 4-byte Folded Spill
	bl	f2fs_put_page
	ldr	x1, [sp, #32]           // 8-byte Folded Reload
	mov	w2, #14
	mov	w3, #3136
	mov	x0, x26
	bl	pagecache_get_page
	mov	x25, x0
	mov	x0, xzr
	mov	w20, wzr
	mov	w27, #-12
	cbnz	x25, .LBB56_40
	b	.LBB56_26
.LBB56_109:                             //   in Loop: Header=BB56_40 Depth=1
	mov	w3, #1
	mov	x0, x25
	mov	w1, wzr
	mov	w2, wzr
	bl	f2fs_wait_on_page_writeback
	cmp	w19, #1, lsl #12        // =4096
	b.eq	.LBB56_127
// %bb.110:                             //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x25, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x25, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB56_129
// %bb.111:                             //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	cbnz	x8, .LBB56_114
// %bb.112:                             //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x24, #80]
	ldr	x9, [sp, #56]           // 8-byte Folded Reload
	cmp	x9, x8
	b.lt	.LBB56_114
// %bb.113:                             //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x24, #736]
	tbz	w8, #30, .LBB56_135
.LBB56_114:                             //   in Loop: Header=BB56_40 Depth=1
	cmn	w23, #1                 // =1
	b.eq	.LBB56_130
// %bb.115:                             //   in Loop: Header=BB56_40 Depth=1
	mov	w2, #8
	mov	x0, x21
	mov	w1, w23
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB56_133
// %bb.116:                             //   in Loop: Header=BB56_40 Depth=1
	mov	w4, #1
	mov	x0, x24
	mov	x1, x25
	mov	w2, w23
	mov	w3, wzr
	bl	f2fs_submit_page_read
	cbnz	w0, .LBB56_134
// %bb.117:                             //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x25, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x25, x9, eq
	ldr	x9, [x8]
	str	x9, [sp, #128]
	ldr	x9, [sp, #128]
	tbnz	w9, #0, .LBB56_119
// %bb.118:                             //   in Loop: Header=BB56_40 Depth=1
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbz	w9, #0, .LBB56_120
.LBB56_119:                             //   in Loop: Header=BB56_40 Depth=1
	mov	x0, x25
	bl	__lock_page
.LBB56_120:                             //   in Loop: Header=BB56_40 Depth=1
	ldr	x8, [x25, #24]
	cmp	x8, x26
	b.ne	.LBB56_108
// %bb.121:
	ldr	x8, [x25, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x25, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB56_126
// %bb.122:
	mov	w20, wzr
	mov	w27, #-5
.LBB56_123:
	mov	x0, x25
	b	.LBB56_26
.LBB56_124:
	mov	w20, wzr
	mov	w27, #-12
	b	.LBB56_26
.LBB56_125:
	mov	w8, w0
	mov	x0, xzr
	mov	w20, wzr
	and	w27, w8, w8, asr #31
	b	.LBB56_26
.LBB56_126:
	//APP
	dmb ishld
	//NO_APP
.LBB56_127:
	mov	w27, wzr
	b	.LBB56_28
.LBB56_128:
	mov	w20, wzr
	b	.LBB56_123
.LBB56_129:
	//APP
	dmb ishld
	//NO_APP
	mov	w27, wzr
	b	.LBB56_28
.LBB56_130:
	//APP
	mrs x19, sp_el0
	//NO_APP
	ldr	w8, [x19, #24]
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	add	x9, x25, x9
	add	w8, w8, #1              // =1
	str	w8, [x19, #24]
	//APP
	//NO_APP
	ldr	w8, [x19, #2800]
	mov	w2, #4096
	mov	w1, wzr
	add	w8, w8, #1              // =1
	str	w8, [x19, #2800]
	//APP
	//NO_APP
	lsl	x8, x9, #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	add	x0, x8, x9
	bl	memset
	//APP
	//NO_APP
	ldr	w8, [x19, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x19, #2800]
	//APP
	//NO_APP
	ldr	x8, [x19, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x19, #24]
	b.eq	.LBB56_146
// %bb.131:
	ldr	x8, [x19, #24]
	cbz	x8, .LBB56_146
.LBB56_132:
	mov	x0, x25
	bl	flush_dcache_page
	//APP
	dmb ishst
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x25]
1:	ldxr	x8, [x25]
	orr	x8, x8, 4
	stxr	w9, x8, [x25]
	cbnz	w9, 1b
	//NO_APP
	mov	w27, wzr
	b	.LBB56_28
.LBB56_133:
	mov	w20, wzr
	mov	w27, #-117
	b	.LBB56_123
.LBB56_134:
	mov	w27, w0
	mov	w20, wzr
	b	.LBB56_123
.LBB56_135:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	cmp	w19, #4095              // =4095
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	ldr	w8, [x20, #2800]
	add	w8, w8, #1              // =1
	str	w8, [x20, #2800]
	//APP
	//NO_APP
	b.hi	.LBB56_137
// %bb.136:
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	ldr	x11, [sp]               // 8-byte Folded Reload
	add	x8, x25, x8
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	mov	w10, #4096
	add	x8, x8, x11
	add	x0, x8, x9
	sub	w2, w10, w19
	mov	w1, wzr
	bl	memset
.LBB56_137:
	//APP
	//NO_APP
	ldr	w8, [x20, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x20, #2800]
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB56_148
// %bb.138:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB56_148
.LBB56_139:
	mov	x0, x25
	bl	flush_dcache_page
	mov	w27, wzr
	b	.LBB56_28
.LBB56_140:
	bl	preempt_schedule_notrace
	ldr	x8, [x21, #80]
	tbz	w8, #8, .LBB56_19
.LBB56_141:
	mov	x0, x21
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB56_19
// %bb.142:
	mov	x0, xzr
	mov	w20, wzr
	mov	w27, #-28
	b	.LBB56_26
.LBB56_143:
	adrp	x1, .L.str.10
	add	x1, x1, :lo12:.L.str.10
	add	x0, sp, #128            // =128
	add	x26, sp, #128           // =128
	bl	strcpy
	b	.LBB56_4
.LBB56_144:
	adrp	x1, .L.str.10
	add	x1, x1, :lo12:.L.str.10
	add	x0, sp, #128            // =128
	add	x26, sp, #128           // =128
	bl	strcpy
	b	.LBB56_3
.LBB56_145:
	bl	preempt_schedule_notrace
	adrp	x10, cpu_number
	add	x10, x10, :lo12:cpu_number
	b	.LBB56_11
.LBB56_146:
	bl	preempt_schedule
	b	.LBB56_132
.LBB56_147:
	bl	__stack_chk_fail
.LBB56_148:
	bl	preempt_schedule
	b	.LBB56_139
.Lfunc_end56:
	.size	f2fs_write_begin, .Lfunc_end56-f2fs_write_begin
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_write_end
	.type	f2fs_write_end,@function
f2fs_write_end:                         // @f2fs_write_end
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	ldr	x8, [x5, #24]
	adrp	x9, __tracepoint_android_fs_datawrite_end+8
	adrp	x26, cpu_number
	adrp	x25, __cpu_online_mask
	ldr	x20, [x8]
	ldr	w8, [x9, :lo12:__tracepoint_android_fs_datawrite_end+8]
	mov	x23, x6
	mov	x21, x5
	mov	w19, w4
	mov	w24, w3
	mov	x22, x2
	cmp	w8, #1                  // =1
	add	x26, x26, :lo12:cpu_number
	add	x25, x25, :lo12:__cpu_online_mask
	mov	x29, sp
	b.lt	.LBB57_6
// %bb.1:
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
	ldr	w8, [x8, x26]
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x25, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB57_6
// %bb.2:
	//APP
	mrs x27, sp_el0
	//NO_APP
	ldr	w8, [x27, #24]
	add	w8, w8, #1              // =1
	str	w8, [x27, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_android_fs_datawrite_end+32
	ldr	x28, [x8, :lo12:__tracepoint_android_fs_datawrite_end+32]
	cbz	x28, .LBB57_4
.LBB57_3:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x28]
	mov	x1, x20
	mov	x2, x22
	mov	w3, w24
	blr	x8
	ldr	x8, [x28, #24]!
	cbnz	x8, .LBB57_3
.LBB57_4:
	//APP
	//NO_APP
	ldr	x8, [x27, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x27, #24]
	b.eq	.LBB57_37
// %bb.5:
	ldr	x8, [x27, #24]
	cbz	x8, .LBB57_37
.LBB57_6:
	adrp	x8, __tracepoint_f2fs_write_end+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_write_end+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB57_12
// %bb.7:
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
	ldr	w8, [x8, x26]
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x25, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB57_12
// %bb.8:
	//APP
	mrs x25, sp_el0
	//NO_APP
	ldr	w8, [x25, #24]
	add	w8, w8, #1              // =1
	str	w8, [x25, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_write_end+32
	ldr	x26, [x8, :lo12:__tracepoint_f2fs_write_end+32]
	cbz	x26, .LBB57_10
.LBB57_9:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x26]
	mov	x1, x20
	mov	x2, x22
	mov	w3, w24
	mov	w4, w19
	blr	x8
	ldr	x8, [x26, #24]!
	cbnz	x8, .LBB57_9
.LBB57_10:
	//APP
	//NO_APP
	ldr	x8, [x25, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x25, #24]
	b.eq	.LBB57_38
// %bb.11:
	ldr	x8, [x25, #24]
	cbz	x8, .LBB57_38
.LBB57_12:
	ldr	x8, [x21, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x21, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB57_15
// %bb.13:
	cmp	w19, w24
	b.ne	.LBB57_39
// %bb.14:
	//APP
	dmb ishst
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x21]
1:	ldxr	x8, [x21]
	orr	x8, x8, 4
	stxr	w9, x8, [x21]
	cbnz	w9, 1b
	//NO_APP
	b	.LBB57_16
.LBB57_15:
	//APP
	dmb ishld
	//NO_APP
.LBB57_16:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB57_21
// %bb.17:
	ldr	w8, [x20, #1280]
	cbnz	w8, .LBB57_20
// %bb.18:
	ldr	x8, [x20, #736]
	tbnz	x8, #35, .LBB57_20
// %bb.19:
	ldr	x8, [x20, #736]
	ubfx	x8, x8, #34, #1
	cbnz	x23, .LBB57_23
	b	.LBB57_29
.LBB57_20:
	mov	w8, #1
	cbnz	x23, .LBB57_23
	b	.LBB57_29
.LBB57_21:
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB57_29
// %bb.22:
	ldr	x8, [x20, #736]
	ubfx	x8, x8, #31, #1
	cbz	x23, .LBB57_29
.LBB57_23:
	cbz	w8, .LBB57_29
// %bb.24:
	ldr	x2, [x21, #32]
	mov	x0, x20
	mov	x1, x23
	mov	w3, w19
	bl	f2fs_compress_write_end
	ldr	x8, [x20, #40]
	adrp	x9, jiffies
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	ldr	x9, [x20, #80]
	add	x8, x22, w19, uxtw
	cmp	x8, x9
	b.le	.LBB57_36
// %bb.25:
	ldr	x9, [x20, #736]
	tbnz	w9, #30, .LBB57_36
// %bb.26:
	ldr	x22, [x20, #736]
	ldr	x21, [x20, #736]
	mov	w1, #1
	mov	x0, x20
	str	x8, [x20, #80]
	bl	f2fs_mark_inode_dirty_sync
	tbz	w22, #1, .LBB57_28
// %bb.27:
	tbz	w21, #2, .LBB57_36
.LBB57_28:
	add	x8, x20, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB57_36
.LBB57_29:
	cbz	w19, .LBB57_35
// %bb.30:
	mov	x0, x21
	bl	set_page_dirty
	ldr	x9, [x20, #80]
	add	x8, x22, w19, uxtw
	cmp	x8, x9
	b.le	.LBB57_35
// %bb.31:
	ldr	x9, [x20, #736]
	tbnz	w9, #30, .LBB57_35
// %bb.32:
	ldr	x23, [x20, #736]
	ldr	x22, [x20, #736]
	mov	w1, #1
	mov	x0, x20
	str	x8, [x20, #80]
	bl	f2fs_mark_inode_dirty_sync
	tbz	w23, #1, .LBB57_34
// %bb.33:
	tbz	w22, #2, .LBB57_35
.LBB57_34:
	add	x8, x20, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB57_35:
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_put_page
	ldr	x8, [x20, #40]
	adrp	x9, jiffies
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
.LBB57_36:
	mov	w0, w19
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.LBB57_37:
	bl	preempt_schedule_notrace
	b	.LBB57_6
.LBB57_38:
	bl	preempt_schedule_notrace
	b	.LBB57_12
.LBB57_39:
	mov	w19, wzr
	b	.LBB57_16
.Lfunc_end57:
	.size	f2fs_write_end, .Lfunc_end57-f2fs_write_end
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_bmap
	.type	f2fs_bmap,@function
f2fs_bmap:                              // @f2fs_bmap
// %bb.0:
	sub	sp, sp, #112            // =112
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #64]     // 16-byte Folded Spill
	add	x29, sp, #64            // =64
	stp	x22, x21, [sp, #80]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #96]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x19, [x0]
	ldr	x8, [x19, #736]
	tbnz	w8, #10, .LBB58_8
// %bb.1:
	ldrb	w8, [x0, #15]
	mov	x21, x1
	tbz	w8, #1, .LBB58_3
// %bb.2:
	bl	filemap_write_and_wait
.LBB58_3:
	mov	x0, x19
	bl	max_file_blocks
	cmp	x0, x21
	b.ls	.LBB58_8
// %bb.4:
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB58_9
// %bb.5:
	ldr	w8, [x19, #1280]
	cbnz	w8, .LBB58_12
// %bb.6:
	ldr	x8, [x19, #736]
	tbnz	x8, #35, .LBB58_12
// %bb.7:
	ldr	x8, [x19, #736]
	tbz	x8, #34, .LBB58_11
	b	.LBB58_12
.LBB58_8:
	mov	x20, xzr
	b	.LBB58_30
.LBB58_9:
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB58_11
// %bb.10:
	ldr	x8, [x19, #736]
	tbnz	w8, #31, .LBB58_12
.LBB58_11:
	add	x0, sp, #8              // =8
	mov	w2, #40
	mov	w1, wzr
	stp	xzr, xzr, [sp, #32]
	stp	xzr, xzr, [sp, #16]
	str	xzr, [sp, #8]
	bl	memset
	mov	w8, #1
	mov	w9, #8
	add	x1, sp, #8              // =8
	mov	w3, #2
	mov	x0, x19
	mov	w2, wzr
	str	xzr, [sp, #24]
	stp	w21, w8, [sp, #12]
	str	w9, [sp, #40]
	bl	f2fs_map_blocks
	ldr	w8, [sp, #8]
	cmp	w0, #0                  // =0
	csel	w20, w8, wzr, eq
	b	.LBB58_30
.LBB58_12:
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	ldr	w8, [x19, #1292]
	add	x0, sp, #8              // =8
	mov	w2, #48
	mov	w1, wzr
	sub	w8, w8, #1              // =1
	bic	x22, x21, x8
	bl	memset
	add	x0, sp, #8              // =8
	mov	w2, #1
	mov	x1, x22
	stp	x19, xzr, [sp, #8]
	str	xzr, [sp, #24]
	str	wzr, [sp, #32]
	bl	f2fs_get_dnode_of_data
	mov	x20, xzr
	cbnz	w0, .LBB58_30
// %bb.13:
	ldr	w8, [sp, #44]
	cmn	w8, #2                  // =2
	b.ne	.LBB58_15
// %bb.14:
	ldr	x0, [sp, #24]
	b	.LBB58_22
.LBB58_15:
	ldr	w9, [sp, #36]
	ldr	x0, [sp, #24]
	mov	x10, #134217728
	sub	w8, w21, w22
	movk	x10, #64, lsl #32
	mov	x11, #-549755813888
	mov	x12, #-549755813888
	add	w8, w9, w8
	add	x9, x10, x0, lsl #6
	movk	x11, #4072
	movk	x12, #4076
	str	w8, [sp, #36]
	and	x13, x9, #0xfffffffffffff000
	ldr	w9, [x13, x11]
	ldr	w10, [x13, x12]
	mov	x11, #-549755813888
	add	x11, x13, x11
	cmp	w9, w10
	b.ne	.LBB58_20
// %bb.16:
	ldr	x12, [sp, #8]
	cbz	x12, .LBB58_19
// %bb.17:
	ldr	x13, [x12, #736]
	tbz	w13, #26, .LBB58_20
// %bb.18:
	ldr	w12, [x12, #1184]
	asr	w12, w12, #2
	b	.LBB58_21
.LBB58_19:
	ldrb	w12, [x11, #3]
	tbnz	w12, #5, .LBB58_40
.LBB58_20:
	mov	w12, wzr
.LBB58_21:
	add	x13, x11, #360          // =360
	cmp	w9, w10
	csel	x9, x13, x11, eq
	add	w8, w12, w8
	ldr	w20, [x9, w8, uxtw #2]
	add	w8, w20, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB58_32
.LBB58_22:
	mov	x20, xzr
.LBB58_23:
	cbz	x0, .LBB58_25
// %bb.24:
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #24]
	ldr	x8, [sp, #16]
	cbnz	x8, .LBB58_26
	b	.LBB58_29
.LBB58_25:
	mov	x9, xzr
	ldr	x8, [sp, #16]
	cbz	x8, .LBB58_29
.LBB58_26:
	cmp	x9, x8
	b.eq	.LBB58_29
// %bb.27:
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
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
	cbnz	w10, .LBB58_29
// %bb.28:
	bl	__put_page
.LBB58_29:
	stp	xzr, xzr, [sp, #16]
.LBB58_30:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB58_41
// %bb.31:
	mov	x0, x20
	ldp	x20, x19, [sp, #96]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #80]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #64]     // 16-byte Folded Reload
	add	sp, sp, #112            // =112
	ret
.LBB58_32:
	ldr	x8, [x19, #40]
	ldr	x10, [x8, #1008]
	ldr	w9, [x10, #3128]
	cbz	w9, .LBB58_23
// %bb.33:
	ldr	x8, [x10]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB58_39
// %bb.34:
	ldr	x10, [x10, #3136]
	add	x10, x10, #80           // =80
	b	.LBB58_36
.LBB58_35:                              //   in Loop: Header=BB58_36 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB58_39
.LBB58_36:                              // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w20
	b.hi	.LBB58_35
// %bb.37:                              //   in Loop: Header=BB58_36 Depth=1
	ldr	w11, [x10]
	cmp	w11, w20
	b.lo	.LBB58_35
// %bb.38:
	ldur	x8, [x10, #-80]
.LBB58_39:
	cbnz	x8, .LBB58_23
	b	.LBB58_22
.LBB58_40:
	ldrh	w12, [x11, #360]
	lsr	w12, w12, #2
	b	.LBB58_21
.LBB58_41:
	bl	__stack_chk_fail
.Lfunc_end58:
	.size	f2fs_bmap, .Lfunc_end58-f2fs_bmap
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_direct_IO
	.type	f2fs_direct_IO,@function
f2fs_direct_IO:                         // @f2fs_direct_IO
// %bb.0:
	sub	sp, sp, #416            // =416
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #320]    // 16-byte Folded Spill
	add	x29, sp, #320           // =320
	stp	x28, x27, [sp, #336]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #352]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #368]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #384]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #400]    // 16-byte Folded Spill
	stur	x8, [x29, #-16]
	ldp	x8, x19, [x0]
	ldr	x28, [x1, #16]
	ldr	w22, [x1]
	mov	x25, x0
	ldr	x26, [x8, #248]
	mov	x23, x1
	ldr	x20, [x26]
	ldr	x8, [x20, #40]
	ldr	x24, [x8, #1008]
	ldrh	w8, [x0, #36]
	mov	x0, x1
	str	w8, [sp, #40]           // 4-byte Folded Spill
	ldr	w8, [x24, #1484]
	str	w8, [sp, #44]           // 4-byte Folded Spill
	ldrb	w21, [x20, #142]
	bl	iov_iter_alignment
	ldr	x9, [x20, #40]
	ldr	w27, [x23]
	ldr	x10, [x9, #248]
	tbnz	w27, #0, .LBB59_2
// %bb.1:
	ldr	x8, [x20, #80]
	cmp	x8, x19
	b.le	.LBB59_11
.LBB59_2:
	mov	w8, #-1
	lsl	w8, w8, w21
	mvn	w11, w8
	orr	x8, x0, x19
	mov	w11, w11
	tst	x8, x11
	b.eq	.LBB59_9
// %bb.3:
	cbz	x10, .LBB59_12
// %bb.4:
	ldr	x9, [x10, #128]
	ldr	x10, [x9, #1264]
	mov	w9, #512
	cbz	x10, .LBB59_6
// %bb.5:
	ldr	w10, [x10, #2928]
	cmp	w10, #0                 // =0
	csel	w9, w9, w10, eq
.LBB59_6:
	mov	w10, #7
.LBB59_7:                               // =>This Inner Loop Header: Depth=1
	cmp	w9, #513                // =513
	lsr	w9, w9, #1
	add	w10, w10, #1            // =1
	b.hi	.LBB59_7
// %bb.8:
	mov	w9, #-2
	lsl	w9, w9, w10
	mvn	w9, w9
	tst	x8, x9
	mov	x8, #-22
	csel	x0, xzr, x8, eq
	b	.LBB59_13
.LBB59_9:
	ldr	x21, [x9, #1008]
	mov	x0, x25
	mov	x1, x23
	bl	fscrypt_dio_supported
	tbz	w0, #0, .LBB59_11
// %bb.10:
	ldr	x8, [x20, #664]
	cbz	x8, .LBB59_15
.LBB59_11:
	mov	x0, xzr
	b	.LBB59_13
.LBB59_12:
	mov	x0, #-22
.LBB59_13:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB59_134
// %bb.14:
	ldp	x20, x19, [sp, #400]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #384]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #368]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #352]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #336]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #320]    // 16-byte Folded Reload
	add	sp, sp, #416            // =416
	ret
.LBB59_15:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	ldr	w9, [x8, #1532]
	cbz	w9, .LBB59_19
// %bb.16:
	ldr	w9, [x20, #1280]
	cbnz	w9, .LBB59_11
// %bb.17:
	ldr	x9, [x20, #736]
	tbnz	x9, #35, .LBB59_11
// %bb.18:
	ldr	x9, [x20, #736]
	tbnz	x9, #34, .LBB59_11
	b	.LBB59_21
.LBB59_19:
	ldrh	w9, [x20]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB59_21
// %bb.20:
	ldr	x9, [x20, #736]
	tbnz	w9, #31, .LBB59_11
.LBB59_21:
	ldr	x9, [x21, #16]
	ldrb	w9, [x9, #2180]
	tbnz	w9, #1, .LBB59_11
// %bb.22:
	tbz	w27, #0, .LBB59_27
// %bb.23:
	ldr	w9, [x21, #1504]
	cmp	w9, #1                  // =1
	b.ne	.LBB59_27
// %bb.24:
	ldrb	w8, [x20, #142]
	ldr	w27, [x25, #8]
	mov	w9, #-1
	mov	x0, x23
	lsl	w8, w9, w8
	str	w8, [sp, #32]           // 4-byte Folded Spill
	bl	iov_iter_alignment
	ldr	w9, [sp, #32]           // 4-byte Folded Reload
	orr	w8, w0, w27
	bics	wzr, w8, w9
	b.ne	.LBB59_11
// %bb.25:
	ldr	w8, [x21, #1432]
	mov	w9, #1
	lsl	w8, w9, w8
	cmp	w8, #1                  // =1
	b.gt	.LBB59_11
// %bb.26:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
.LBB59_27:
	ldr	x9, [x8, #80]
	tbnz	w9, #8, .LBB59_11
// %bb.28:
	ldr	w8, [x8, #1504]
	and	w22, w22, #0x1
	str	wzr, [sp, #28]          // 4-byte Folded Spill
	cmp	w8, #1                  // =1
	b.ne	.LBB59_31
// %bb.29:
	ldr	w8, [x23]
	mov	x27, x28
	tbz	w8, #0, .LBB59_32
// %bb.30:
	ldrb	w8, [x20, #142]
	ldr	w21, [x25, #8]
	mov	w9, #-1
	mov	x0, x23
	lsl	w28, w9, w8
	bl	iov_iter_alignment
	orr	w8, w0, w21
	bics	wzr, w8, w28
	cset	w8, eq
	str	w8, [sp, #28]           // 4-byte Folded Spill
	b	.LBB59_32
.LBB59_31:
	mov	x27, x28
.LBB59_32:
	adrp	x8, __tracepoint_f2fs_direct_IO_enter+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_direct_IO_enter+8]
	mov	w28, w22
	cmp	w8, #1                  // =1
	b.lt	.LBB59_38
// %bb.33:
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
	tbz	w8, #0, .LBB59_38
// %bb.34:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w8, [x21, #24]
	add	w8, w8, #1              // =1
	str	w8, [x21, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_direct_IO_enter+32
	ldr	x22, [x8, :lo12:__tracepoint_f2fs_direct_IO_enter+32]
	cbz	x22, .LBB59_36
.LBB59_35:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x22]
	mov	x1, x20
	mov	x2, x19
	mov	x3, x27
	mov	w4, w28
	blr	x8
	ldr	x8, [x22, #24]!
	cbnz	x8, .LBB59_35
.LBB59_36:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	b.eq	.LBB59_132
// %bb.37:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB59_132
.LBB59_38:
	adrp	x21, __tracepoint_android_fs_dataread_start+8
	ldr	w8, [x21, :lo12:__tracepoint_android_fs_dataread_start+8]
	str	w28, [sp, #12]          // 4-byte Folded Spill
	str	x27, [sp, #16]          // 8-byte Folded Spill
	cbnz	w28, .LBB59_51
// %bb.39:
	cmp	w8, #1                  // =1
	b.lt	.LBB59_51
// %bb.40:
	mov	x0, x20
	stp	xzr, xzr, [sp, #288]
	stp	xzr, xzr, [sp, #272]
	stp	xzr, xzr, [sp, #256]
	stp	xzr, xzr, [sp, #240]
	stp	xzr, xzr, [sp, #224]
	stp	xzr, xzr, [sp, #208]
	stp	xzr, xzr, [sp, #192]
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	stp	xzr, xzr, [sp, #128]
	stp	xzr, xzr, [sp, #112]
	stp	xzr, xzr, [sp, #96]
	stp	xzr, xzr, [sp, #80]
	stp	xzr, xzr, [sp, #64]
	stp	xzr, xzr, [sp, #48]
	bl	ihold
	mov	x0, x20
	bl	d_obtain_alias
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB59_135
// %bb.41:
	add	x1, sp, #48             // =48
	mov	w2, #256
	mov	x22, x0
	bl	dentry_path_raw
	mov	x28, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB59_136
.LBB59_42:
	mov	x0, x22
	bl	dput
.LBB59_43:
	//APP
	mrs x11, sp_el0
	//NO_APP
	ldr	w27, [x11, #1536]
	ldr	w8, [x21, :lo12:__tracepoint_android_fs_dataread_start+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB59_50
// %bb.44:
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
	tbz	w8, #0, .LBB59_50
// %bb.45:
	ldr	w8, [x11, #24]
	str	x11, [sp]               // 8-byte Folded Spill
	add	w8, w8, #1              // =1
	str	w8, [x11, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_android_fs_dataread_start+32
	ldr	x21, [x8, :lo12:__tracepoint_android_fs_dataread_start+32]
	str	x28, [sp, #32]          // 8-byte Folded Spill
	ldr	x28, [sp, #16]          // 8-byte Folded Reload
	cbz	x21, .LBB59_48
// %bb.46:
	ldr	x8, [sp]                // 8-byte Folded Reload
	add	x22, x8, #2000          // =2000
.LBB59_47:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	ldr	x5, [sp, #32]           // 8-byte Folded Reload
	mov	x1, x20
	mov	x2, x19
	mov	w3, w28
	mov	w4, w27
	mov	x6, x22
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB59_47
.LBB59_48:
	//APP
	//NO_APP
	ldr	x9, [sp]                // 8-byte Folded Reload
	adrp	x21, __tracepoint_android_fs_dataread_start+8
	ldr	x8, [x9, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x9, #24]
	b.eq	.LBB59_139
// %bb.49:
	ldr	x8, [x9, #24]
	cbz	x8, .LBB59_139
.LBB59_50:
	ldr	w28, [sp, #12]          // 4-byte Folded Reload
	ldr	x27, [sp, #16]          // 8-byte Folded Reload
.LBB59_51:
	adrp	x8, __tracepoint_android_fs_datawrite_start+8
	ldr	w8, [x8, :lo12:__tracepoint_android_fs_datawrite_start+8]
	cbz	w28, .LBB59_64
// %bb.52:
	cmp	w8, #1                  // =1
	b.lt	.LBB59_64
// %bb.53:
	mov	x0, x20
	stp	xzr, xzr, [sp, #288]
	stp	xzr, xzr, [sp, #272]
	stp	xzr, xzr, [sp, #256]
	stp	xzr, xzr, [sp, #240]
	stp	xzr, xzr, [sp, #224]
	stp	xzr, xzr, [sp, #208]
	stp	xzr, xzr, [sp, #192]
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	stp	xzr, xzr, [sp, #128]
	stp	xzr, xzr, [sp, #112]
	stp	xzr, xzr, [sp, #96]
	stp	xzr, xzr, [sp, #80]
	stp	xzr, xzr, [sp, #64]
	stp	xzr, xzr, [sp, #48]
	bl	ihold
	mov	x0, x20
	bl	d_obtain_alias
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB59_137
// %bb.54:
	add	x1, sp, #48             // =48
	mov	w2, #256
	mov	x22, x0
	bl	dentry_path_raw
	mov	x28, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB59_138
.LBB59_55:
	mov	x0, x22
	bl	dput
.LBB59_56:
	adrp	x8, __tracepoint_android_fs_datawrite_start+8
	//APP
	mrs x11, sp_el0
	//NO_APP
	ldr	w27, [x11, #1536]
	ldr	w8, [x8, :lo12:__tracepoint_android_fs_datawrite_start+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB59_63
// %bb.57:
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
	tbz	w8, #0, .LBB59_63
// %bb.58:
	ldr	w8, [x11, #24]
	str	x11, [sp]               // 8-byte Folded Spill
	add	w8, w8, #1              // =1
	str	w8, [x11, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_android_fs_datawrite_start+32
	ldr	x21, [x8, :lo12:__tracepoint_android_fs_datawrite_start+32]
	str	x28, [sp, #32]          // 8-byte Folded Spill
	ldr	x28, [sp, #16]          // 8-byte Folded Reload
	cbz	x21, .LBB59_61
// %bb.59:
	ldr	x8, [sp]                // 8-byte Folded Reload
	add	x22, x8, #2000          // =2000
.LBB59_60:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	ldr	x5, [sp, #32]           // 8-byte Folded Reload
	mov	x1, x20
	mov	x2, x19
	mov	w3, w28
	mov	w4, w27
	mov	x6, x22
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB59_60
.LBB59_61:
	//APP
	//NO_APP
	ldr	x9, [sp]                // 8-byte Folded Reload
	adrp	x21, __tracepoint_android_fs_dataread_start+8
	ldr	x8, [x9, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x9, #24]
	b.eq	.LBB59_140
// %bb.62:
	ldr	x8, [x9, #24]
	cbz	x8, .LBB59_140
.LBB59_63:
	ldr	w28, [sp, #12]          // 4-byte Folded Reload
	ldr	x27, [sp, #16]          // 8-byte Folded Reload
.LBB59_64:
	cbz	w28, .LBB59_67
// %bb.65:
	ldr	w8, [sp, #44]           // 4-byte Folded Reload
	cbnz	w8, .LBB59_67
// %bb.66:
	strh	wzr, [x25, #36]
.LBB59_67:
	add	x9, x20, #992           // =992
	ldrb	w8, [x25, #32]
	str	x9, [sp, #32]           // 8-byte Folded Spill
	mov	w9, #48
	umaddl	x9, w28, w9, x20
	add	x22, x9, #992           // =992
	mov	x0, x22
	tbnz	w8, #3, .LBB59_70
// %bb.68:
	bl	down_read
	ldr	w8, [sp, #28]           // 4-byte Folded Reload
	cbz	w8, .LBB59_73
// %bb.69:
	ldr	x0, [sp, #32]           // 8-byte Folded Reload
	bl	down_read
	b	.LBB59_73
.LBB59_70:
	bl	down_read_trylock
	cbz	w0, .LBB59_104
// %bb.71:
	ldr	w8, [sp, #28]           // 4-byte Folded Reload
	cbz	w8, .LBB59_73
// %bb.72:
	ldr	x0, [sp, #32]           // 8-byte Folded Reload
	bl	down_read_trylock
	cbz	w0, .LBB59_103
.LBB59_73:
	ldr	x8, [x20, #40]
	cbz	w28, .LBB59_84
.LBB59_74:
	ldr	x2, [x8, #248]
	adrp	x8, get_data_block_dio
	adrp	x9, get_data_block_dio_write
	add	x8, x8, :lo12:get_data_block_dio
	add	x9, x9, :lo12:get_data_block_dio_write
	cmp	w28, #0                 // =0
	csel	x4, x9, x8, ne
	mov	w8, #2
	adrp	x6, f2fs_dio_submit_bio
	cinc	w7, w8, ne
	add	x6, x6, :lo12:f2fs_dio_submit_bio
	mov	x0, x25
	mov	x1, x20
	mov	x3, x23
	mov	x5, xzr
	mov	x21, x22
	bl	__blockdev_direct_IO
	ldr	w8, [sp, #28]           // 4-byte Folded Reload
	mov	x22, x0
	cbz	w8, .LBB59_76
// %bb.75:
	ldr	x0, [sp, #32]           // 8-byte Folded Reload
	bl	up_read
.LBB59_76:
	mov	x0, x21
	bl	up_read
	cbz	w28, .LBB59_88
// %bb.77:
	ldr	w8, [sp, #44]           // 4-byte Folded Reload
	adrp	x21, __tracepoint_android_fs_dataread_start+8
	cbnz	w8, .LBB59_79
// %bb.78:
	ldr	w8, [sp, #40]           // 4-byte Folded Reload
	strh	w8, [x25, #36]
.LBB59_79:
	cmp	w22, #1                 // =1
	b.lt	.LBB59_91
// %bb.80:
	ldr	x8, [x20, #40]
	ldr	x24, [x8, #1008]
	ldrb	w8, [x24, #2704]
	cbz	w8, .LBB59_82
// %bb.81:
	add	x23, x24, #2344         // =2344
	mov	x0, x23
	bl	_raw_spin_lock
	ldr	x8, [x24, #2352]
	ldr	x9, [x24, #2368]
	mov	x0, x23
	add	x8, x8, w22, sxtw
	str	x8, [x24, #2352]
	sub	x8, x9, x8
	str	x8, [x24, #2360]
	bl	_raw_spin_unlock
	mov	x0, x24
	bl	f2fs_record_iostat
.LBB59_82:
	ldr	w8, [sp, #28]           // 4-byte Folded Reload
	cbnz	w8, .LBB59_105
// %bb.83:
	add	x8, x20, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 8192
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB59_105
.LBB59_84:
	ldr	x9, [x8, #1008]
	ldr	w9, [x9, #1532]
	cbz	w9, .LBB59_94
// %bb.85:
	ldr	w9, [x20, #1280]
	cbnz	w9, .LBB59_96
// %bb.86:
	ldr	x9, [x20, #736]
	tbnz	x9, #35, .LBB59_96
// %bb.87:
	ldr	x9, [x20, #736]
	tbz	x9, #34, .LBB59_74
	b	.LBB59_96
.LBB59_88:
	cmp	w22, #1                 // =1
	adrp	x21, __tracepoint_android_fs_dataread_start+8
	b.lt	.LBB59_99
// %bb.89:
	ldrb	w8, [x24, #2704]
	cbz	w8, .LBB59_105
// %bb.90:
	add	x23, x24, #2344         // =2344
	mov	x0, x23
	bl	_raw_spin_lock
	ldr	x8, [x24, #2448]
	ldr	x9, [x24, #2464]
	mov	x0, x23
	add	x8, x8, w22, sxtw
	str	x8, [x24, #2448]
	sub	x8, x9, x8
	str	x8, [x24, #2456]
	bl	_raw_spin_unlock
	mov	x0, x24
	bl	f2fs_record_iostat
	b	.LBB59_105
.LBB59_91:
	cmn	w22, #529               // =529
	b.ne	.LBB59_128
// %bb.92:
	ldr	x8, [x20, #40]
	ldr	x22, [x8, #1008]
	ldrb	w8, [x22, #2704]
	cbz	w8, .LBB59_130
// %bb.93:
	ldr	x8, [x23, #16]
	add	x23, x22, #2344         // =2344
	mov	x0, x23
	mov	x24, x21
	sub	x21, x27, x8
	bl	_raw_spin_lock
	ldr	x8, [x22, #2352]
	ldr	x9, [x22, #2368]
	mov	x0, x23
	add	x8, x8, x21
	str	x8, [x22, #2352]
	sub	x8, x9, x8
	mov	x21, x24
	str	x8, [x22, #2360]
	bl	_raw_spin_unlock
	mov	x0, x22
	bl	f2fs_record_iostat
	mov	w22, #-529
	b	.LBB59_105
.LBB59_94:
	ldrh	w9, [x20]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB59_74
// %bb.95:
	ldr	x9, [x20, #736]
	tbz	w9, #31, .LBB59_74
.LBB59_96:
	ldr	w8, [sp, #40]           // 4-byte Folded Reload
	strh	w8, [x25, #36]
	ldr	w8, [sp, #28]           // 4-byte Folded Reload
	cbz	w8, .LBB59_98
// %bb.97:
	ldr	x0, [sp, #32]           // 8-byte Folded Reload
	bl	up_read
.LBB59_98:
	mov	x0, x22
	bl	up_read
	mov	w22, wzr
	b	.LBB59_105
.LBB59_99:
	cmn	w22, #529               // =529
	b.ne	.LBB59_105
// %bb.100:
	ldr	x8, [x20, #40]
	mov	x24, x21
	ldr	x22, [x8, #1008]
	ldrb	w8, [x22, #2704]
	cbz	w8, .LBB59_102
// %bb.101:
	ldr	x8, [x23, #16]
	add	x23, x22, #2344         // =2344
	mov	x0, x23
	sub	x21, x27, x8
	bl	_raw_spin_lock
	ldr	x8, [x22, #2448]
	ldr	x9, [x22, #2464]
	mov	x0, x23
	add	x8, x8, x21
	str	x8, [x22, #2448]
	sub	x8, x9, x8
	str	x8, [x22, #2456]
	bl	_raw_spin_unlock
	mov	x0, x22
	bl	f2fs_record_iostat
.LBB59_102:
	mov	w22, #-529
	mov	x21, x24
	b	.LBB59_105
.LBB59_103:
	mov	x0, x22
	bl	up_read
.LBB59_104:
	ldr	w8, [sp, #40]           // 4-byte Folded Reload
	mov	w22, #-11
	strh	w8, [x25, #36]
.LBB59_105:
	ldr	w8, [x21, :lo12:__tracepoint_android_fs_dataread_start+8]
	cbnz	w28, .LBB59_113
// %bb.106:
	cmp	w8, #1                  // =1
	b.lt	.LBB59_113
// %bb.107:
	adrp	x8, __tracepoint_android_fs_dataread_end+8
	ldr	w8, [x8, :lo12:__tracepoint_android_fs_dataread_end+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB59_113
// %bb.108:
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
	tbz	w8, #0, .LBB59_113
// %bb.109:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w8, [x21, #24]
	add	w8, w8, #1              // =1
	str	w8, [x21, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_android_fs_dataread_end+32
	ldr	x23, [x8, :lo12:__tracepoint_android_fs_dataread_end+32]
	cbz	x23, .LBB59_111
.LBB59_110:                             // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	mov	x1, x20
	mov	x2, x19
	mov	w3, w27
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB59_110
.LBB59_111:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	b.eq	.LBB59_141
// %bb.112:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB59_141
.LBB59_113:
	adrp	x8, __tracepoint_android_fs_datawrite_start+8
	ldr	w8, [x8, :lo12:__tracepoint_android_fs_datawrite_start+8]
	cbz	w28, .LBB59_121
// %bb.114:
	cmp	w8, #1                  // =1
	b.lt	.LBB59_121
// %bb.115:
	adrp	x8, __tracepoint_android_fs_datawrite_end+8
	ldr	w8, [x8, :lo12:__tracepoint_android_fs_datawrite_end+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB59_121
// %bb.116:
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
	tbz	w8, #0, .LBB59_121
// %bb.117:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w8, [x21, #24]
	add	w8, w8, #1              // =1
	str	w8, [x21, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_android_fs_datawrite_end+32
	ldr	x23, [x8, :lo12:__tracepoint_android_fs_datawrite_end+32]
	cbz	x23, .LBB59_119
.LBB59_118:                             // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	mov	x1, x20
	mov	x2, x19
	mov	w3, w27
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB59_118
.LBB59_119:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	b.eq	.LBB59_142
// %bb.120:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB59_142
.LBB59_121:
	adrp	x8, __tracepoint_f2fs_direct_IO_exit+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_direct_IO_exit+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB59_127
// %bb.122:
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
	tbz	w8, #0, .LBB59_127
// %bb.123:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w8, [x21, #24]
	add	w8, w8, #1              // =1
	str	w8, [x21, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_direct_IO_exit+32
	ldr	x23, [x8, :lo12:__tracepoint_f2fs_direct_IO_exit+32]
	cbz	x23, .LBB59_125
.LBB59_124:                             // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	mov	x1, x20
	mov	x2, x19
	mov	x3, x27
	mov	w4, w28
	mov	w5, w22
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB59_124
.LBB59_125:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	b.eq	.LBB59_133
// %bb.126:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB59_133
.LBB59_127:
	sxtw	x0, w22
	b	.LBB59_13
.LBB59_128:
	tbnz	w22, #31, .LBB59_131
// %bb.129:
	mov	w22, wzr
	b	.LBB59_105
.LBB59_130:
	mov	w22, #-529
	b	.LBB59_105
.LBB59_131:
	add	x1, x19, x27
	mov	x0, x26
	bl	f2fs_write_failed
	b	.LBB59_105
.LBB59_132:
	bl	preempt_schedule_notrace
	b	.LBB59_38
.LBB59_133:
	bl	preempt_schedule_notrace
	b	.LBB59_127
.LBB59_134:
	bl	__stack_chk_fail
.LBB59_135:
	adrp	x1, .L.str.10
	add	x1, x1, :lo12:.L.str.10
	add	x0, sp, #48             // =48
	add	x28, sp, #48            // =48
	bl	strcpy
	b	.LBB59_43
.LBB59_136:
	adrp	x1, .L.str.10
	add	x1, x1, :lo12:.L.str.10
	add	x0, sp, #48             // =48
	add	x28, sp, #48            // =48
	bl	strcpy
	b	.LBB59_42
.LBB59_137:
	adrp	x1, .L.str.10
	add	x1, x1, :lo12:.L.str.10
	add	x0, sp, #48             // =48
	add	x28, sp, #48            // =48
	bl	strcpy
	b	.LBB59_56
.LBB59_138:
	adrp	x1, .L.str.10
	add	x1, x1, :lo12:.L.str.10
	add	x0, sp, #48             // =48
	add	x28, sp, #48            // =48
	bl	strcpy
	b	.LBB59_55
.LBB59_139:
	bl	preempt_schedule_notrace
	b	.LBB59_50
.LBB59_140:
	bl	preempt_schedule_notrace
	b	.LBB59_63
.LBB59_141:
	bl	preempt_schedule_notrace
	b	.LBB59_113
.LBB59_142:
	bl	preempt_schedule_notrace
	b	.LBB59_121
.Lfunc_end59:
	.size	f2fs_direct_IO, .Lfunc_end59-f2fs_direct_IO
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_swap_activate
	.type	f2fs_swap_activate,@function
f2fs_swap_activate:                     // @f2fs_swap_activate
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
	ldr	x19, [x1, #32]
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB60_4
// %bb.1:
	ldr	x8, [x19, #40]
	mov	x21, x0
	ldr	x0, [x8, #1008]
	ldr	x8, [x0]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB60_5
// %bb.2:
	ldr	w8, [x0, #1504]
	cmp	w8, #1                  // =1
	b.ne	.LBB60_6
// %bb.3:
	adrp	x1, .L.str.15
	add	x1, x1, :lo12:.L.str.15
	bl	f2fs_printk
.LBB60_4:
	mov	w22, #-22
	b	.LBB60_47
.LBB60_5:
	mov	w22, #-30
	b	.LBB60_47
.LBB60_6:
	ldr	x8, [x19, #736]
	tbnz	x8, #38, .LBB60_46
// %bb.7:
	mov	x0, x19
	mov	x20, x2
	mov	x23, x1
	bl	f2fs_convert_inline_inode
	mov	w22, w0
	cbnz	w0, .LBB60_47
// %bb.8:
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB60_14
// %bb.9:
	ldr	x8, [x19, #736]
	tbz	w8, #31, .LBB60_14
// %bb.10:
	ldr	w8, [x19, #824]
	ldr	x9, [x19, #144]
	cmp	w8, #0                  // =0
	cset	w8, ne
	cmp	x8, x9, lsr #3
	b.lo	.LBB60_4
// %bb.11:
	ldr	x8, [x19, #704]
	and	x8, x8, #0xfffffffffffffffb
	str	x8, [x19, #704]
	ldr	x8, [x19, #736]
	tbz	w8, #31, .LBB60_13
// %bb.12:
	ldr	x8, [x19, #40]
	mov	w9, #1
	ldr	x8, [x8, #1008]
	add	x8, x8, #2220           // =2220
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
.LBB60_13:
	add	x8, x19, #736           // =736
	mov	w9, #-2147483648
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_mark_inode_dirty_sync
.LBB60_14:
	mov	x0, x19
	str	x20, [sp, #40]          // 8-byte Folded Spill
	bl	f2fs_precache_extents
	ldr	x8, [x23, #248]
	mov	w23, wzr
	ldr	x25, [x8]
	ldr	x8, [x25, #40]
	ldrb	w9, [x25, #142]
	ldr	x11, [x8, #1008]
	ldr	x8, [x25, #80]
	ldr	w10, [x11, #1192]
	str	x11, [sp, #56]          // 8-byte Folded Spill
	ldr	w11, [x11, #1188]
	lsr	x9, x8, x9
	mul	w10, w11, w10
	mov	w11, wzr
	cbz	x9, .LBB60_49
// %bb.15:
	mov	w8, w10
	str	x8, [sp, #80]           // 8-byte Folded Spill
	add	x8, x25, #1040          // =1040
	stur	x8, [x29, #-64]         // 8-byte Folded Spill
	add	x8, x25, #1088          // =1088
	stur	x8, [x29, #-72]         // 8-byte Folded Spill
	mov	x8, #-1
	sub	w12, w10, #1            // =1
	str	x8, [sp, #16]           // 8-byte Folded Spill
	//APP
	mrs x8, sp_el0
	//NO_APP
	mov	x26, xzr
	str	x8, [sp]                // 8-byte Folded Spill
	mov	w8, w12
	str	xzr, [sp, #24]          // 8-byte Folded Spill
	str	w12, [sp, #88]          // 4-byte Folded Spill
	stur	x8, [x29, #-104]        // 8-byte Folded Spill
	str	x21, [sp, #72]          // 8-byte Folded Spill
	str	w10, [sp, #52]          // 4-byte Folded Spill
	str	x9, [sp, #8]            // 8-byte Folded Spill
.LBB60_16:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB60_20 Depth 2
                                        //       Child Loop BB60_27 Depth 3
                                        //         Child Loop BB60_31 Depth 4
	ldr	w8, [x21, #60]
	cmp	x26, x8
	b.hs	.LBB60_45
// %bb.17:                              //   in Loop: Header=BB60_16 Depth=1
	sub	w8, w9, w26
	str	w11, [sp, #36]          // 4-byte Folded Spill
	stp	xzr, xzr, [x29, #-32]
	stp	xzr, xzr, [x29, #-48]
	stur	xzr, [x29, #-56]
	str	w8, [sp, #92]           // 4-byte Folded Spill
	str	x26, [sp, #112]         // 8-byte Folded Spill
	b	.LBB60_20
.LBB60_18:                              //   in Loop: Header=BB60_20 Depth=2
	mov	w22, wzr
.LBB60_19:                              //   in Loop: Header=BB60_20 Depth=2
	mov	w10, #4194304
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x11]
1:	ldxr	x8, [x11]
	bic	x8, x8, x10
	stxr	w9, x8, [x11]
	cbnz	w9, 1b
	//NO_APP
	mov	x10, #68719476736
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x11]
1:	ldxr	x8, [x11]
	bic	x8, x8, x10
	stxr	w9, x8, [x11]
	cbnz	w9, 1b
	//NO_APP
	ldur	x0, [x29, #-72]         // 8-byte Folded Reload
	bl	up_write
	ldur	x0, [x29, #-64]         // 8-byte Folded Reload
	bl	up_write
	ldr	x21, [sp, #72]          // 8-byte Folded Reload
	ldur	w23, [x29, #-76]        // 4-byte Folded Reload
	cbnz	w22, .LBB60_54
.LBB60_20:                              //   Parent Loop BB60_16 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB60_27 Depth 3
                                        //         Child Loop BB60_31 Depth 4
	sub	x0, x29, #56            // =56
	mov	w2, #40
	mov	w1, wzr
	bl	memset
	ldr	w8, [sp, #92]           // 4-byte Folded Reload
	sub	x1, x29, #56            // =56
	mov	w3, #1
	mov	x0, x25
	stp	w26, w8, [x29, #-52]
	mov	w8, #8
	mov	w2, wzr
	stp	xzr, xzr, [x29, #-40]
	stur	w8, [x29, #-24]
	sturb	wzr, [x29, #-20]
	bl	f2fs_map_blocks
	cbnz	w0, .LBB60_52
// %bb.21:                              //   in Loop: Header=BB60_20 Depth=2
	ldurh	w8, [x29, #-44]
	mov	w9, #4192
	tst	w8, w9
	b.eq	.LBB60_53
// %bb.22:                              //   in Loop: Header=BB60_20 Depth=2
	ldur	w20, [x29, #-48]
	ldr	w8, [sp, #88]           // 4-byte Folded Reload
	ldur	w22, [x29, #-56]
	tst	w20, w8
	b.ne	.LBB60_24
// %bb.23:                              //   in Loop: Header=BB60_20 Depth=2
	ldr	x8, [sp, #56]           // 8-byte Folded Reload
	ldur	x9, [x29, #-104]        // 8-byte Folded Reload
	ldr	x8, [x8, #136]
	ldr	w8, [x8, #84]
	sub	w8, w22, w8
	and	x8, x8, x9
	cbz	x8, .LBB60_41
.LBB60_24:                              //   in Loop: Header=BB60_20 Depth=2
	ldur	x8, [x29, #-104]        // 8-byte Folded Reload
	ldr	x11, [sp, #80]          // 8-byte Folded Reload
	ldr	w9, [x21, #60]
	add	w23, w23, #1            // =1
	add	x8, x20, x8
	udiv	x8, x8, x11
	mul	x8, x8, x11
	add	x10, x8, x26
	cmp	x10, x9
	csel	x9, x11, xzr, hi
	subs	x8, x8, x9
	b.eq	.LBB60_40
// %bb.25:                              //   in Loop: Header=BB60_20 Depth=2
	ldr	x9, [x25, #40]
	ldur	x0, [x29, #-64]         // 8-byte Folded Reload
	stur	w23, [x29, #-76]        // 4-byte Folded Spill
	ldr	x28, [x9, #1008]
	ldr	w9, [x28, #1192]
	ldr	w10, [x28, #1188]
	mul	w21, w10, w9
	udiv	w20, w26, w21
	udiv	w8, w8, w21
	add	w22, w8, w20
	bl	down_write
	ldur	x0, [x29, #-72]         // 8-byte Folded Reload
	bl	down_write
	add	x11, x25, #736          // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x11]
1:	ldxr	x8, [x11]
	orr	x8, x8, 68719476736
	stxr	w9, x8, [x11]
	cbnz	w9, 1b
	//NO_APP
	cmp	w20, w22
	stur	w22, [x29, #-80]        // 4-byte Folded Spill
	b.hs	.LBB60_18
// %bb.26:                              //   in Loop: Header=BB60_20 Depth=2
	add	x8, x28, #304           // =304
	add	x23, x28, #2040         // =2040
	add	x22, x28, #368          // =368
	stur	x8, [x29, #-96]         // 8-byte Folded Spill
	mov	w8, w21
	str	x11, [sp, #64]          // 8-byte Folded Spill
	str	x8, [sp, #96]           // 8-byte Folded Spill
	stur	x23, [x29, #-88]        // 8-byte Folded Spill
	str	x22, [sp, #104]         // 8-byte Folded Spill
.LBB60_27:                              //   Parent Loop BB60_16 Depth=1
                                        //     Parent Loop BB60_20 Depth=2
                                        // =>    This Loop Header: Depth=3
                                        //         Child Loop BB60_31 Depth 4
	mov	x0, x23
	bl	down_write
	mov	x0, x22
	bl	_raw_spin_lock
	ldr	x8, [x28, #352]
	cbz	x8, .LBB60_29
// %bb.28:                              //   in Loop: Header=BB60_27 Depth=3
	adrp	x9, jiffies
	ldr	x8, [x28, #360]
	ldr	x9, [x9, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB60_35
.LBB60_29:                              //   in Loop: Header=BB60_27 Depth=3
	mov	x0, x22
	bl	_raw_spin_unlock
	ldur	x22, [x29, #-96]        // 8-byte Folded Reload
	mov	x0, x22
	bl	down_read
	mov	w1, #6
	mov	x0, x28
	mov	w2, wzr
	bl	f2fs_allocate_new_section
	mov	x0, x22
	bl	up_read
	add	x26, x25, #736          // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x26]
1:	ldxr	x8, [x26]
	orr	x8, x8, 4194304
	stxr	w9, x8, [x26]
	cbnz	w9, 1b
	//NO_APP
	cbz	w21, .LBB60_33
// %bb.30:                              //   in Loop: Header=BB60_27 Depth=3
	ldr	x24, [sp, #96]          // 8-byte Folded Reload
	mov	x23, xzr
	mul	w27, w20, w21
.LBB60_31:                              //   Parent Loop BB60_16 Depth=1
                                        //     Parent Loop BB60_20 Depth=2
                                        //       Parent Loop BB60_27 Depth=3
                                        // =>      This Inner Loop Header: Depth=4
	add	w1, w27, w23
	mov	w2, #1
	mov	x0, x25
	bl	f2fs_get_lock_data_page
	mov	x22, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB60_36
// %bb.32:                              //   in Loop: Header=BB60_31 Depth=4
	mov	x0, x22
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
	subs	x24, x24, #1            // =1
	add	x23, x23, #1            // =1
	b.ne	.LBB60_31
.LBB60_33:                              //   in Loop: Header=BB60_27 Depth=3
	mov	w10, #4194304
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x26]
1:	ldxr	x8, [x26]
	bic	x8, x8, x10
	stxr	w9, x8, [x26]
	cbnz	w9, 1b
	//NO_APP
	ldr	x0, [x25, #48]
	bl	filemap_fdatawrite
	ldur	x23, [x29, #-88]        // 8-byte Folded Reload
	mov	w22, w0
	mov	x0, x23
	bl	up_write
	cbnz	w22, .LBB60_37
// %bb.34:                              //   in Loop: Header=BB60_27 Depth=3
	ldur	w8, [x29, #-80]         // 4-byte Folded Reload
	ldp	x22, x26, [sp, #104]    // 16-byte Folded Reload
	add	w20, w20, #1            // =1
	cmp	w20, w8
	b.lo	.LBB60_27
	b	.LBB60_39
.LBB60_35:                              //   in Loop: Header=BB60_27 Depth=3
	ldr	x0, [sp]                // 8-byte Folded Reload
	mov	x1, xzr
	bl	show_stack
	adrp	x8, jiffies
	ldr	x1, [x28, #360]
	ldr	x2, [x8, :lo12:jiffies]
	adrp	x0, .L.str.12
	add	x0, x0, :lo12:.L.str.12
	bl	printk
	ldr	x0, [x28, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB60_29
.LBB60_36:                              //   in Loop: Header=BB60_20 Depth=2
	ldur	x0, [x29, #-88]         // 8-byte Folded Reload
	bl	up_write
.LBB60_37:                              //   in Loop: Header=BB60_20 Depth=2
	ldr	x26, [sp, #112]         // 8-byte Folded Reload
.LBB60_38:                              //   in Loop: Header=BB60_20 Depth=2
	ldr	x11, [sp, #64]          // 8-byte Folded Reload
	b	.LBB60_19
.LBB60_39:                              //   in Loop: Header=BB60_20 Depth=2
	mov	w22, wzr
	b	.LBB60_38
.LBB60_40:                              //   in Loop: Header=BB60_16 Depth=1
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	adrp	x1, .L.str.18
	add	x1, x1, :lo12:.L.str.18
	bl	f2fs_printk
.LBB60_41:                              //   in Loop: Header=BB60_16 Depth=1
	ldr	w8, [x21, #60]
	add	x9, x26, x20
	sub	x10, x8, x26
	cmp	x9, x8
	csel	x24, x20, x10, lo
	cbz	x26, .LBB60_43
// %bb.42:                              //   in Loop: Header=BB60_16 Depth=1
	ldr	x9, [sp, #16]           // 8-byte Folded Reload
	add	x8, x22, x24
	sub	x8, x8, #1              // =1
	cmp	x9, x22
	csel	x9, x22, x9, hi
	str	x9, [sp, #16]           // 8-byte Folded Spill
	ldr	x9, [sp, #24]           // 8-byte Folded Reload
	cmp	x8, x9
	csel	x9, x8, x9, hi
	str	x9, [sp, #24]           // 8-byte Folded Spill
.LBB60_43:                              //   in Loop: Header=BB60_16 Depth=1
	mov	x0, x21
	mov	x1, x26
	mov	x2, x24
	mov	x3, x22
	bl	add_swap_extent
	mov	w22, w0
	tbnz	w0, #31, .LBB60_54
// %bb.44:                              //   in Loop: Header=BB60_16 Depth=1
	ldr	x9, [sp, #8]            // 8-byte Folded Reload
	ldr	w11, [sp, #36]          // 4-byte Folded Reload
	ldr	w10, [sp, #52]          // 4-byte Folded Reload
	add	x26, x24, x26
	cmp	x9, x26
	add	w11, w22, w11
	b.hi	.LBB60_16
.LBB60_45:
	ldp	x9, x8, [sp, #16]       // 16-byte Folded Reload
	sub	x8, x8, x9
	ldr	x9, [sp, #40]           // 8-byte Folded Reload
	add	x8, x8, #1              // =1
	str	x8, [x9]
	cbnz	x26, .LBB60_51
	b	.LBB60_50
.LBB60_46:
	adrp	x1, .L.str.16
	add	x1, x1, :lo12:.L.str.16
	bl	f2fs_printk
	mov	w22, #-95
.LBB60_47:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB60_59
// %bb.48:
	mov	w0, w22
	ldp	x20, x19, [sp, #304]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #288]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #272]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #256]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #240]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #224]    // 16-byte Folded Reload
	add	sp, sp, #320            // =320
	ret
.LBB60_49:
	ldr	x9, [sp, #40]           // 8-byte Folded Reload
	mov	w8, #2
	str	x8, [x9]
.LBB60_50:
	mov	w26, #1
.LBB60_51:
	sub	w8, w26, #1             // =1
	str	w26, [x21, #60]
	stp	w8, w8, [x21, #100]
	mov	w22, w11
	b	.LBB60_55
.LBB60_52:
	mov	w22, w0
	b	.LBB60_54
.LBB60_53:
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	adrp	x1, .L.str.17
	add	x1, x1, :lo12:.L.str.17
	bl	f2fs_printk
	mov	w22, #-22
.LBB60_54:
	ldr	w10, [sp, #52]          // 4-byte Folded Reload
.LBB60_55:
	cbz	w23, .LBB60_57
// %bb.56:
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	adrp	x1, .L.str.19
	lsl	w3, w10, #12
	add	x1, x1, :lo12:.L.str.19
	mov	w2, w23
	bl	f2fs_printk
.LBB60_57:
	tbnz	w22, #31, .LBB60_47
// %bb.58:
	add	x8, x19, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 268435456
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_mark_inode_dirty_sync
	ldr	x8, [x19, #40]
	adrp	x9, jiffies
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	b	.LBB60_47
.LBB60_59:
	bl	__stack_chk_fail
.Lfunc_end60:
	.size	f2fs_swap_activate, .Lfunc_end60-f2fs_swap_activate
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_swap_deactivate
	.type	f2fs_swap_deactivate,@function
f2fs_swap_deactivate:                   // @f2fs_swap_deactivate
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x0, [x0, #32]
	mov	w9, #268435456
	mov	w1, #1
	mov	x29, sp
	add	x8, x0, #736            // =736
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	bl	f2fs_mark_inode_dirty_sync
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end61:
	.size	f2fs_swap_deactivate, .Lfunc_end61-f2fs_swap_deactivate
                                        // -- End function
	.globl	f2fs_clear_page_cache_dirty_tag // -- Begin function f2fs_clear_page_cache_dirty_tag
	.p2align	2
	.type	f2fs_clear_page_cache_dirty_tag,@function
f2fs_clear_page_cache_dirty_tag:        // @f2fs_clear_page_cache_dirty_tag
.Lf2fs_clear_page_cache_dirty_tag$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	mov	x21, x0
	bl	page_mapping
	add	x19, x0, #8             // =8
	mov	x0, x19
	bl	_raw_spin_lock_irqsave
	ldr	x8, [x21, #8]
	mov	x20, x0
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x21, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB62_2
// %bb.1:
	ldr	x8, [x21]
	tbnz	w8, #10, .LBB62_4
.LBB62_2:
	ldr	x1, [x21, #32]
.LBB62_3:
	mov	x0, x19
	mov	w2, wzr
	bl	__xa_clear_mark
	mov	x0, x19
	mov	x1, x20
	bl	_raw_spin_unlock_irqrestore
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB62_4:
	mov	x0, x21
	bl	__page_file_index
	mov	x1, x0
	b	.LBB62_3
.Lfunc_end62:
	.size	f2fs_clear_page_cache_dirty_tag, .Lfunc_end62-f2fs_clear_page_cache_dirty_tag
                                        // -- End function
	.section	.init.text,"ax",@progbits
	.globl	f2fs_init_post_read_processing // -- Begin function f2fs_init_post_read_processing
	.p2align	2
	.type	f2fs_init_post_read_processing,@function
f2fs_init_post_read_processing:         // @f2fs_init_post_read_processing
.Lf2fs_init_post_read_processing$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	adrp	x0, .L.str.3
	add	x0, x0, :lo12:.L.str.3
	mov	w1, #80
	mov	w2, wzr
	mov	w3, wzr
	mov	x4, xzr
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	bl	kmem_cache_create
	adrp	x19, bio_post_read_ctx_cache
	str	x0, [x19, :lo12:bio_post_read_ctx_cache]
	cbz	x0, .LBB63_4
// %bb.1:
	adrp	x1, mempool_alloc_slab
	adrp	x2, mempool_free_slab
	mov	x3, x0
	add	x1, x1, :lo12:mempool_alloc_slab
	add	x2, x2, :lo12:mempool_free_slab
	mov	w0, #128
	bl	mempool_create
	adrp	x8, bio_post_read_ctx_pool
	str	x0, [x8, :lo12:bio_post_read_ctx_pool]
	cbz	x0, .LBB63_3
// %bb.2:
	mov	w0, wzr
	b	.LBB63_5
.LBB63_3:
	ldr	x0, [x19, :lo12:bio_post_read_ctx_cache]
	bl	kmem_cache_destroy
.LBB63_4:
	mov	w0, #-12
.LBB63_5:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end63:
	.size	f2fs_init_post_read_processing, .Lfunc_end63-f2fs_init_post_read_processing
                                        // -- End function
	.text
	.globl	f2fs_destroy_post_read_processing // -- Begin function f2fs_destroy_post_read_processing
	.p2align	2
	.type	f2fs_destroy_post_read_processing,@function
f2fs_destroy_post_read_processing:      // @f2fs_destroy_post_read_processing
.Lf2fs_destroy_post_read_processing$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x8, bio_post_read_ctx_pool
	ldr	x0, [x8, :lo12:bio_post_read_ctx_pool]
	mov	x29, sp
	bl	mempool_destroy
	adrp	x8, bio_post_read_ctx_cache
	ldr	x0, [x8, :lo12:bio_post_read_ctx_cache]
	bl	kmem_cache_destroy
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end64:
	.size	f2fs_destroy_post_read_processing, .Lfunc_end64-f2fs_destroy_post_read_processing
                                        // -- End function
	.globl	f2fs_init_post_read_wq  // -- Begin function f2fs_init_post_read_wq
	.p2align	2
	.type	f2fs_init_post_read_wq,@function
f2fs_init_post_read_wq:                 // @f2fs_init_post_read_wq
.Lf2fs_init_post_read_wq$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	ldr	x8, [x0, #16]
	mov	w9, #9217
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	ldrh	w8, [x8, #2180]
	tst	w8, w9
	b.eq	.LBB65_2
// %bb.1:
	adrp	x8, __num_online_cpus
	ldr	w2, [x8, :lo12:__num_online_cpus]
	mov	x19, x0
	adrp	x0, .L.str.4
	add	x0, x0, :lo12:.L.str.4
	mov	w1, #18
	bl	alloc_workqueue
	cmp	x0, #0                  // =0
	mov	w8, #-12
	str	x0, [x19, #3224]
	csel	w0, w8, wzr, eq
	b	.LBB65_3
.LBB65_2:
	mov	w0, wzr
.LBB65_3:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end65:
	.size	f2fs_init_post_read_wq, .Lfunc_end65-f2fs_init_post_read_wq
                                        // -- End function
	.globl	f2fs_destroy_post_read_wq // -- Begin function f2fs_destroy_post_read_wq
	.p2align	2
	.type	f2fs_destroy_post_read_wq,@function
f2fs_destroy_post_read_wq:              // @f2fs_destroy_post_read_wq
.Lf2fs_destroy_post_read_wq$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x0, [x0, #3224]
	mov	x29, sp
	cbz	x0, .LBB66_2
// %bb.1:
	bl	destroy_workqueue
.LBB66_2:
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end66:
	.size	f2fs_destroy_post_read_wq, .Lfunc_end66-f2fs_destroy_post_read_wq
                                        // -- End function
	.section	.init.text,"ax",@progbits
	.globl	f2fs_init_bio_entry_cache // -- Begin function f2fs_init_bio_entry_cache
	.p2align	2
	.type	f2fs_init_bio_entry_cache,@function
f2fs_init_bio_entry_cache:              // @f2fs_init_bio_entry_cache
.Lf2fs_init_bio_entry_cache$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x0, .L.str.5
	add	x0, x0, :lo12:.L.str.5
	mov	w1, #24
	mov	w3, #131072
	mov	w2, wzr
	mov	x4, xzr
	mov	x29, sp
	bl	kmem_cache_create
	cmp	x0, #0                  // =0
	mov	w8, #-12
	adrp	x9, bio_entry_slab
	csel	w8, w8, wzr, eq
	str	x0, [x9, :lo12:bio_entry_slab]
	mov	w0, w8
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end67:
	.size	f2fs_init_bio_entry_cache, .Lfunc_end67-f2fs_init_bio_entry_cache
                                        // -- End function
	.text
	.globl	f2fs_destroy_bio_entry_cache // -- Begin function f2fs_destroy_bio_entry_cache
	.p2align	2
	.type	f2fs_destroy_bio_entry_cache,@function
f2fs_destroy_bio_entry_cache:           // @f2fs_destroy_bio_entry_cache
.Lf2fs_destroy_bio_entry_cache$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x8, bio_entry_slab
	ldr	x0, [x8, :lo12:bio_entry_slab]
	mov	x29, sp
	bl	kmem_cache_destroy
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end68:
	.size	f2fs_destroy_bio_entry_cache, .Lfunc_end68-f2fs_destroy_bio_entry_cache
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_read_end_io
	.type	f2fs_read_end_io,@function
f2fs_read_end_io:                       // @f2fs_read_end_io
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldrb	w8, [x0, #20]
	mov	x19, x0
	str	x25, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	tbnz	w8, #1, .LBB69_6
.LBB69_1:
	ldr	x8, [x19, #472]
	ldr	x21, [x8]
	ldr	x8, [x21, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x21, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB69_3
// %bb.2:
	ldr	x8, [x21]
	tbnz	w8, #10, .LBB69_7
.LBB69_3:
	ldrb	w8, [x19, #26]
	cbz	w8, .LBB69_8
.LBB69_4:
	mov	x0, x19
	bl	f2fs_finish_read_bio
.LBB69_5:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB69_6:
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "include/linux/bio.h"; .popsection; .long 14472b - 14470b; .short 283; .short (1 << 0)|((1 << 1) | ((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	b	.LBB69_1
.LBB69_7:
	mov	x0, x21
	bl	__page_file_mapping
	ldrb	w8, [x19, #26]
	cbnz	w8, .LBB69_4
.LBB69_8:
	ldr	x20, [x19, #72]
	cbz	x21, .LBB69_21
// %bb.9:
	ldr	x8, [x21, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x21, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB69_11
// %bb.10:
	ldr	x8, [x21]
	tbnz	w8, #10, .LBB69_24
.LBB69_11:
	ldr	x0, [x21, #24]
	cbz	x0, .LBB69_14
.LBB69_12:
	ldr	x8, [x0]
	ldr	x9, [x8, #40]
	ldr	x8, [x8, #64]
	ldr	x9, [x9, #1008]
	ldr	w10, [x9, #1180]
	cmp	x8, x10
	b.eq	.LBB69_21
// %bb.13:
	ldr	w9, [x9, #1176]
	cmp	x8, x9
	b.eq	.LBB69_21
.LBB69_14:
	ldp	x9, x8, [x21, #24]
	adrp	x10, __tracepoint_android_fs_dataread_end+8
	ldr	w21, [x19, #40]
	ldr	x22, [x9]
	ldr	w9, [x10, :lo12:__tracepoint_android_fs_dataread_end+8]
	cmp	w9, #1                  // =1
	b.lt	.LBB69_21
// %bb.15:
	adrp	x10, cpu_number
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
	adrp	x11, __cpu_online_mask
	add	x11, x11, :lo12:__cpu_online_mask
	add	w10, w9, #63            // =63
	cmp	w9, #0                  // =0
	csel	w10, w10, w9, lt
	asr	w10, w10, #6
	ldr	x10, [x11, w10, sxtw #3]
	lsr	x9, x10, x9
	tbz	w9, #0, .LBB69_21
// %bb.16:
	//APP
	mrs x24, sp_el0
	//NO_APP
	ldr	w9, [x24, #24]
	add	w9, w9, #1              // =1
	str	w9, [x24, #24]
	//APP
	//NO_APP
	adrp	x9, __tracepoint_android_fs_dataread_end+32
	ldr	x25, [x9, :lo12:__tracepoint_android_fs_dataread_end+32]
	cbz	x25, .LBB69_19
// %bb.17:
	lsl	x23, x8, #12
.LBB69_18:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x25]
	mov	x1, x22
	mov	x2, x23
	mov	w3, w21
	blr	x8
	ldr	x8, [x25, #24]!
	cbnz	x8, .LBB69_18
.LBB69_19:
	//APP
	//NO_APP
	ldr	x8, [x24, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x24, #24]
	b.eq	.LBB69_25
// %bb.20:
	ldr	x8, [x24, #24]
	cbz	x8, .LBB69_25
.LBB69_21:
	cbz	x20, .LBB69_26
.LBB69_22:
	ldrb	w8, [x20, #64]
	tst	w8, #0x3
	b.eq	.LBB69_26
// %bb.23:
	mov	x8, #68719476704
	str	x8, [x20, #16]!
	add	x8, x20, #8             // =8
	str	x8, [x20, #8]
	ldur	x9, [x20, #-8]
	adrp	x10, f2fs_post_read_work
	add	x10, x10, :lo12:f2fs_post_read_work
	stp	x8, x10, [x20, #16]
	ldr	x1, [x9, #3224]
	mov	w0, #8
	mov	x2, x20
	bl	queue_work_on
	b	.LBB69_5
.LBB69_24:
	mov	x0, x21
	bl	__page_file_mapping
	cbnz	x0, .LBB69_12
	b	.LBB69_14
.LBB69_25:
	bl	preempt_schedule_notrace
	cbnz	x20, .LBB69_22
.LBB69_26:
	ldr	x0, [x19, #72]
	cbz	x0, .LBB69_4
// %bb.27:
	ldrb	w8, [x0, #64]
	tbz	w8, #2, .LBB69_4
// %bb.28:
	mov	x8, #68719476704
	adrp	x9, f2fs_verify_bio
	str	x8, [x0, #16]!
	add	x9, x9, :lo12:f2fs_verify_bio
	add	x8, x0, #8              // =8
	str	x8, [x0, #8]
	stp	x8, x9, [x0, #16]
	bl	fsverity_enqueue_verify_work
	b	.LBB69_5
.Lfunc_end69:
	.size	f2fs_read_end_io, .Lfunc_end69-f2fs_read_end_io
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_write_end_io
	.type	f2fs_write_end_io,@function
f2fs_write_end_io:                      // @f2fs_write_end_io
// %bb.0:
	sub	sp, sp, #112            // =112
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #32]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #48]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #64]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #80]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #96]     // 16-byte Folded Spill
	ldrh	w8, [x0, #458]
	ldr	x20, [x0, #72]
	mov	x19, x0
	add	x29, sp, #16            // =16
	cbz	w8, .LBB70_63
// %bb.1:
	mov	w28, wzr
	mov	w24, wzr
	mov	x22, xzr
	add	x8, x20, #3408          // =3408
	mov	w26, #4096
	mov	w23, #1
	str	x8, [sp, #8]            // 8-byte Folded Spill
	b	.LBB70_4
.LBB70_2:                               //   in Loop: Header=BB70_4 Depth=1
	mov	x0, x27
	bl	end_page_writeback
.LBB70_3:                               //   in Loop: Header=BB70_4 Depth=1
	ldrh	w8, [x19, #458]
	cmp	w24, w8
	b.hs	.LBB70_63
.LBB70_4:                               // =>This Inner Loop Header: Depth=1
	ldr	x9, [x19, #472]
	mov	w8, w24
	cbz	w28, .LBB70_6
// %bb.5:                               //   in Loop: Header=BB70_4 Depth=1
	mov	w10, wzr
	add	x22, x22, #64           // =64
	b	.LBB70_7
.LBB70_6:                               //   in Loop: Header=BB70_4 Depth=1
	add	x10, x9, x8, lsl #4
	ldr	w11, [x10, #12]
	ldr	x10, [x10]
	lsr	x12, x11, #6
	and	x12, x12, #0x3ffffc0
	add	x22, x10, x12
	and	w10, w11, #0xfff
.LBB70_7:                               //   in Loop: Header=BB70_4 Depth=1
	add	x9, x9, x8, lsl #4
	ldr	w11, [x9, #8]
	sub	w10, w26, w10
	ldr	x9, [x22, #24]
	sub	w12, w11, w28
	cmp	w10, w12
	csel	w10, w10, w12, lo
	add	w10, w10, w28
	cmp	w10, w11
	cinc	w24, w8, eq
	csel	w28, wzr, w10, eq
	cbz	x9, .LBB70_19
// %bb.8:                               //   in Loop: Header=BB70_4 Depth=1
	ldr	x21, [x9]
	ldr	x8, [x21, #40]
	ldr	x9, [x8, #1008]
	ldr	x8, [x21, #64]
	ldr	w10, [x9, #1180]
	cmp	x8, x10
	b.eq	.LBB70_11
// %bb.9:                               //   in Loop: Header=BB70_4 Depth=1
	ldr	w9, [x9, #1176]
	cmp	x8, x9
	b.eq	.LBB70_11
// %bb.10:                              //   in Loop: Header=BB70_4 Depth=1
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.ne	.LBB70_12
.LBB70_11:                              //   in Loop: Header=BB70_4 Depth=1
	mov	w25, #1
	mov	w21, #7
	ldr	x8, [x22]
	tbnz	w8, #13, .LBB70_20
	b	.LBB70_22
.LBB70_12:                              //   in Loop: Header=BB70_4 Depth=1
	mov	x0, x22
	bl	f2fs_is_compressed_page
	tbnz	w0, #0, .LBB70_19
// %bb.13:                              //   in Loop: Header=BB70_4 Depth=1
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB70_16
// %bb.14:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x8, [x21, #736]
	tbnz	w8, #15, .LBB70_11
// %bb.15:                              //   in Loop: Header=BB70_4 Depth=1
	ldrb	w8, [x21, #12]
	tbnz	w8, #5, .LBB70_11
.LBB70_16:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x8, [x22]
	tbz	w8, #13, .LBB70_19
// %bb.17:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x8, [x22, #40]
	tbz	w8, #0, .LBB70_19
// %bb.18:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x8, [x22, #40]
	tbnz	w8, #3, .LBB70_11
.LBB70_19:                              //   in Loop: Header=BB70_4 Depth=1
	mov	w25, wzr
	mov	w21, #8
	ldr	x8, [x22]
	tbz	w8, #13, .LBB70_22
.LBB70_20:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x8, [x22, #40]
	tbz	w8, #0, .LBB70_22
// %bb.21:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x8, [x22, #40]
	tbnz	w8, #2, .LBB70_43
.LBB70_22:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x8, [x22, #24]
	mov	x27, x22
	cbnz	x8, .LBB70_24
// %bb.23:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x27, [x22, #40]
	mov	x0, x22
	bl	fscrypt_free_bounce_page
.LBB70_24:                              //   in Loop: Header=BB70_4 Depth=1
	mov	x0, x27
	bl	f2fs_is_compressed_page
	tbz	w0, #0, .LBB70_26
// %bb.25:                              //   in Loop: Header=BB70_4 Depth=1
	mov	x0, x19
	mov	x1, x27
	bl	f2fs_compress_write_end_io
	b	.LBB70_3
.LBB70_26:                              //   in Loop: Header=BB70_4 Depth=1
	ldrb	w8, [x19, #26]
	cbnz	w8, .LBB70_57
.LBB70_27:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x8, [x20, #128]
	ldr	x9, [x27, #24]
	ldr	x8, [x8, #48]
	cmp	x9, x8
	b.ne	.LBB70_37
// %bb.28:                              //   in Loop: Header=BB70_4 Depth=1
	mov	x9, #134217728
	movk	x9, #64, lsl #32
	add	x9, x9, x27, lsl #6
	mov	x10, #-549755813888
	and	x9, x9, #0xfffffffffffff000
	movk	x10, #4072
	ldr	x8, [x27, #32]
	ldr	w9, [x9, x10]
	cmp	x8, x9
	b.eq	.LBB70_37
// %bb.29:                              //   in Loop: Header=BB70_4 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB70_32
// %bb.30:                              //   in Loop: Header=BB70_4 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB70_32
// %bb.31:                              //   in Loop: Header=BB70_4 Depth=1
	adrp	x25, f2fs_dclient
	ldr	x0, [x25, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.f2fs_write_end_io
	mov	w3, #675
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.f2fs_write_end_io
	bl	dsm_client_record
	ldr	x0, [x25, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB70_32:                              //   in Loop: Header=BB70_4 Depth=1
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 675; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB70_37
// %bb.33:                              //   in Loop: Header=BB70_4 Depth=1
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
	cbnz	w8, .LBB70_37
// %bb.34:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB70_37
// %bb.35:                              //   in Loop: Header=BB70_4 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB70_37
// %bb.36:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
.LBB70_37:                              //   in Loop: Header=BB70_4 Depth=1
	add	x8, x20, x21, lsl #2
	add	x8, x8, #1272           // =1272
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w23
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	mov	x0, x20
	mov	x1, x27
	bl	f2fs_in_warm_node_list
	tbz	w0, #0, .LBB70_39
// %bb.38:                              //   in Loop: Header=BB70_4 Depth=1
	mov	x0, x20
	mov	x1, x27
	bl	f2fs_del_fsync_node_entry
.LBB70_39:                              //   in Loop: Header=BB70_4 Depth=1
	add	x8, x27, #40            // =40
	mov	w11, #8
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x11
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x27, #40]
	cmp	x8, #1                  // =1
	b.ne	.LBB70_2
// %bb.40:                              //   in Loop: Header=BB70_4 Depth=1
	str	xzr, [x27, #40]
	ldr	x8, [x27]
	tbz	w8, #13, .LBB70_2
// %bb.41:                              //   in Loop: Header=BB70_4 Depth=1
	mov	w10, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x27]
1:	ldxr	x8, [x27]
	bic	x8, x8, x10
	stxr	w9, x8, [x27]
	cbnz	w9, 1b
	//NO_APP
	ldr	x8, [x27, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x27, x9, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w23
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB70_2
// %bb.42:                              //   in Loop: Header=BB70_4 Depth=1
	bl	__put_page
	b	.LBB70_2
.LBB70_43:                              //   in Loop: Header=BB70_4 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB70_46
// %bb.44:                              //   in Loop: Header=BB70_4 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB70_46
// %bb.45:                              //   in Loop: Header=BB70_4 Depth=1
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.f2fs_write_end_io
	mov	w3, #625
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.f2fs_write_end_io
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB70_46:                              //   in Loop: Header=BB70_4 Depth=1
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 625; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB70_51
// %bb.47:                              //   in Loop: Header=BB70_4 Depth=1
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
	cbnz	w8, .LBB70_51
// %bb.48:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB70_51
// %bb.49:                              //   in Loop: Header=BB70_4 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB70_51
// %bb.50:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
.LBB70_51:                              //   in Loop: Header=BB70_4 Depth=1
	add	x8, x22, #40            // =40
	mov	w11, #4
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x11
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x22, #40]
	cmp	x8, #1                  // =1
	b.ne	.LBB70_55
// %bb.52:                              //   in Loop: Header=BB70_4 Depth=1
	str	xzr, [x22, #40]
	ldr	x8, [x22]
	tbz	w8, #13, .LBB70_55
// %bb.53:                              //   in Loop: Header=BB70_4 Depth=1
	mov	w10, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x22]
1:	ldxr	x8, [x22]
	bic	x8, x8, x10
	stxr	w9, x8, [x22]
	cbnz	w9, 1b
	//NO_APP
	ldr	x8, [x22, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x22, x9, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w23
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB70_55
// %bb.54:                              //   in Loop: Header=BB70_4 Depth=1
	bl	__put_page
.LBB70_55:                              //   in Loop: Header=BB70_4 Depth=1
	mov	x0, x22
	bl	unlock_page
	ldr	x1, [x20, #216]
	mov	x0, x22
	bl	mempool_free
	ldrb	w8, [x19, #26]
	cbz	w8, .LBB70_3
// %bb.56:                              //   in Loop: Header=BB70_4 Depth=1
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_stop_checkpoint
	b	.LBB70_3
.LBB70_57:                              //   in Loop: Header=BB70_4 Depth=1
	ldr	x0, [x27, #24]
	mov	w1, #-5
	str	x0, [sp]                // 8-byte Folded Spill
	bl	__filemap_set_wb_err
	ldr	x8, [sp]                // 8-byte Folded Reload
	add	x8, x8, #128            // =128
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 1
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	cbz	w25, .LBB70_27
// %bb.58:                              //   in Loop: Header=BB70_4 Depth=1
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_stop_checkpoint
	ldrb	w2, [x19, #26]
	adrp	x1, .L.str.8
	mov	x0, x20
	add	x1, x1, :lo12:.L.str.8
	bl	f2fs_printk
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB70_61
// %bb.59:                              //   in Loop: Header=BB70_4 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB70_61
// %bb.60:                              //   in Loop: Header=BB70_4 Depth=1
	adrp	x25, f2fs_dclient
	ldr	x0, [x25, :lo12:f2fs_dclient]
	ldrb	w4, [x19, #26]
	adrp	x1, .L.str.9
	adrp	x2, .L__func__.f2fs_write_end_io
	mov	w3, #663
	add	x1, x1, :lo12:.L.str.9
	add	x2, x2, :lo12:.L__func__.f2fs_write_end_io
	bl	dsm_client_record
	ldr	x0, [x25, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB70_61:                              //   in Loop: Header=BB70_4 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #3                  // =3
	b.lo	.LBB70_67
// %bb.62:                              //   in Loop: Header=BB70_4 Depth=1
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 667; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	b	.LBB70_27
.LBB70_63:
	ldr	w8, [x20, #1300]
	cbnz	w8, .LBB70_66
// %bb.64:
	//APP
	dmb ish
	//NO_APP
	ldr	x8, [x20, #480]
	add	x9, x20, #480           // =480
	cmp	x9, x8
	b.eq	.LBB70_66
// %bb.65:
	add	x0, x20, #472           // =472
	mov	w1, #3
	mov	w2, #1
	mov	x3, xzr
	bl	__wake_up
.LBB70_66:
	mov	x0, x19
	bl	bio_put
	ldp	x20, x19, [sp, #96]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #80]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #64]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #48]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #112            // =112
	ret
.LBB70_67:
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 667; .short 0; .popsection; 14471: brk 0x800
	//NO_APP
.Lfunc_end70:
	.size	f2fs_write_end_io, .Lfunc_end70-f2fs_write_end_io
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_finish_read_bio
	.type	f2fs_finish_read_bio,@function
f2fs_finish_read_bio:                   // @f2fs_finish_read_bio
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	ldr	x8, [x0, #72]
	mov	x19, x0
	mov	x29, sp
	cbz	x8, .LBB71_7
// %bb.1:
	ldrb	w9, [x8, #64]
	tbz	w9, #3, .LBB71_7
// %bb.2:
	ldr	x20, [x8, #72]
	ldr	w8, [x20, #1300]
	cbz	w8, .LBB71_5
// %bb.3:
	add	x8, x20, #1300          // =1300
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
	cbnz	w10, .LBB71_7
// %bb.4:
	//APP
	dmb ish
	//NO_APP
	ldr	x8, [x20, #1320]
	add	x9, x20, #1320          // =1320
	cmp	x9, x8
	b.ne	.LBB71_6
	b	.LBB71_7
.LBB71_5:
	ldr	x8, [x20, #40]
	ldr	x2, [x20, #64]
	adrp	x1, .L.str.13
	add	x1, x1, :lo12:.L.str.13
	ldr	x0, [x8, #1008]
	bl	f2fs_printk
.LBB71_6:
	add	x0, x20, #1312          // =1312
	mov	w1, #3
	mov	w2, wzr
	mov	x3, xzr
	bl	__wake_up
.LBB71_7:
	ldrh	w8, [x19, #458]
	cbz	w8, .LBB71_38
// %bb.8:
	adrp	x22, .L__func__.f2fs_finish_read_bio
	mov	w24, wzr
	mov	w21, wzr
	mov	x20, xzr
	mov	w25, #4096
	mov	w26, #1
	add	x22, x22, :lo12:.L__func__.f2fs_finish_read_bio
	adrp	x27, f2fs_dclient
	mov	w23, #256
	b	.LBB71_12
.LBB71_9:                               //   in Loop: Header=BB71_12 Depth=1
	mov	w8, #9
.LBB71_10:                              //   in Loop: Header=BB71_12 Depth=1
	add	x8, x28, x8, lsl #2
	add	x8, x8, #1272           // =1272
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w26
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	mov	x0, x20
	bl	unlock_page
.LBB71_11:                              //   in Loop: Header=BB71_12 Depth=1
	ldrh	w8, [x19, #458]
	cmp	w21, w8
	b.hs	.LBB71_38
.LBB71_12:                              // =>This Inner Loop Header: Depth=1
	ldr	x9, [x19, #472]
	mov	w8, w21
	cbz	w24, .LBB71_14
// %bb.13:                              //   in Loop: Header=BB71_12 Depth=1
	mov	w10, wzr
	add	x20, x20, #64           // =64
	b	.LBB71_15
.LBB71_14:                              //   in Loop: Header=BB71_12 Depth=1
	add	x10, x9, x8, lsl #4
	ldr	w11, [x10, #12]
	ldr	x10, [x10]
	lsr	x12, x11, #6
	and	x12, x12, #0x3ffffc0
	add	x20, x10, x12
	and	w10, w11, #0xfff
.LBB71_15:                              //   in Loop: Header=BB71_12 Depth=1
	add	x9, x9, x8, lsl #4
	ldr	w9, [x9, #8]
	sub	w10, w25, w10
	mov	x0, x20
	sub	w11, w9, w24
	cmp	w10, w11
	csel	w10, w10, w11, lo
	add	w10, w10, w24
	cmp	w10, w9
	cinc	w21, w8, eq
	csel	w24, wzr, w10, eq
	bl	f2fs_is_compressed_page
	ldrb	w2, [x19, #26]
	tbz	w0, #0, .LBB71_19
// %bb.16:                              //   in Loop: Header=BB71_12 Depth=1
	cbz	w2, .LBB71_18
// %bb.17:                              //   in Loop: Header=BB71_12 Depth=1
	mov	w1, #1
	mov	x0, x20
	mov	w2, wzr
	bl	f2fs_end_read_compressed_page
.LBB71_18:                              //   in Loop: Header=BB71_12 Depth=1
	mov	x0, x20
	bl	f2fs_put_page_dic
	b	.LBB71_11
.LBB71_19:                              //   in Loop: Header=BB71_12 Depth=1
	cbnz	w2, .LBB71_32
// %bb.20:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #8, .LBB71_32
// %bb.21:                              //   in Loop: Header=BB71_12 Depth=1
	//APP
	dmb ishst
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x20]
1:	ldxr	x8, [x20]
	orr	x8, x8, 4
	stxr	w9, x8, [x20]
	cbnz	w9, 1b
	//NO_APP
.LBB71_22:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB71_24
// %bb.23:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	x8, [x20]
	tbnz	w8, #10, .LBB71_36
.LBB71_24:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	x0, [x20, #24]
.LBB71_25:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x28, [x8, #1008]
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB71_27
// %bb.26:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	x8, [x20]
	tbnz	w8, #10, .LBB71_37
.LBB71_27:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	x0, [x20, #24]
	cbz	x0, .LBB71_9
.LBB71_28:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	x8, [x0]
	ldr	x9, [x8, #40]
	ldr	x8, [x8, #64]
	ldr	x9, [x9, #1008]
	ldr	w10, [x9, #1180]
	cmp	x8, x10
	b.ne	.LBB71_30
// %bb.29:                              //   in Loop: Header=BB71_12 Depth=1
	mov	w8, #11
	b	.LBB71_10
.LBB71_30:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	w9, [x9, #1176]
	cmp	x8, x9
	b.ne	.LBB71_9
// %bb.31:                              //   in Loop: Header=BB71_12 Depth=1
	mov	w8, #10
	b	.LBB71_10
.LBB71_32:                              //   in Loop: Header=BB71_12 Depth=1
	adrp	x0, .L.str.6
	add	x0, x0, :lo12:.L.str.6
	mov	x1, x22
	bl	printk
	ldr	x0, [x27, :lo12:f2fs_dclient]
	cbz	x0, .LBB71_35
// %bb.33:                              //   in Loop: Header=BB71_12 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB71_35
// %bb.34:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	x0, [x27, :lo12:f2fs_dclient]
	ldrb	w3, [x19, #26]
	adrp	x1, .L.str.7
	add	x1, x1, :lo12:.L.str.7
	mov	x2, x22
	bl	dsm_client_record
	ldr	x0, [x27, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB71_35:                              //   in Loop: Header=BB71_12 Depth=1
	ldr	x8, [x20, #8]
	mov	w11, #4
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x11
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x23
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB71_22
.LBB71_36:                              //   in Loop: Header=BB71_12 Depth=1
	mov	x0, x20
	bl	__page_file_mapping
	b	.LBB71_25
.LBB71_37:                              //   in Loop: Header=BB71_12 Depth=1
	mov	x0, x20
	bl	__page_file_mapping
	cbnz	x0, .LBB71_28
	b	.LBB71_9
.LBB71_38:
	ldr	x0, [x19, #72]
	cbz	x0, .LBB71_40
// %bb.39:
	adrp	x8, bio_post_read_ctx_pool
	ldr	x1, [x8, :lo12:bio_post_read_ctx_pool]
	bl	mempool_free
.LBB71_40:
	mov	x0, x19
	bl	bio_put
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.Lfunc_end71:
	.size	f2fs_finish_read_bio, .Lfunc_end71-f2fs_finish_read_bio
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_post_read_work
	.type	f2fs_post_read_work,@function
f2fs_post_read_work:                    // @f2fs_post_read_work
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	ldr	w8, [x0, #48]
	mov	x19, x0
	mov	x29, sp
	tbz	w8, #0, .LBB72_6
// %bb.1:
	tbz	w8, #3, .LBB72_5
// %bb.2:
	ldr	x20, [x19, #56]
	cbz	x20, .LBB72_5
// %bb.3:
	ldur	x25, [x19, #-16]
	ldrh	w8, [x25, #458]
	cbz	w8, .LBB72_6
// %bb.4:
	mov	w26, wzr
	mov	w27, wzr
	mov	x21, xzr
	b	.LBB72_26
.LBB72_5:
	ldur	x0, [x19, #-16]
	bl	fscrypt_decrypt_bio
.LBB72_6:
	ldr	w8, [x19, #48]
	ldur	x0, [x19, #-16]
	tbz	w8, #1, .LBB72_18
// %bb.7:
	ldrh	w9, [x0, #458]
	cbz	w9, .LBB72_17
// %bb.8:
	ldr	x8, [x0, #32]
	mov	w22, wzr
	mov	w25, wzr
	mov	x21, xzr
	mov	w23, #1
	lsr	x20, x8, #3
	mov	w24, #4096
.LBB72_9:                               // =>This Inner Loop Header: Depth=1
	ldr	x9, [x0, #472]
	mov	w8, w25
	cbz	w22, .LBB72_11
// %bb.10:                              //   in Loop: Header=BB72_9 Depth=1
	mov	w10, wzr
	add	x21, x21, #64           // =64
	b	.LBB72_12
.LBB72_11:                              //   in Loop: Header=BB72_9 Depth=1
	add	x10, x9, x8, lsl #4
	ldr	w11, [x10, #12]
	ldr	x10, [x10]
	lsr	x12, x11, #6
	and	x12, x12, #0x3ffffc0
	add	x21, x10, x12
	and	w10, w11, #0xfff
.LBB72_12:                              //   in Loop: Header=BB72_9 Depth=1
	add	x9, x9, x8, lsl #4
	ldr	w9, [x9, #8]
	sub	w10, w24, w10
	mov	x0, x21
	sub	w11, w9, w22
	cmp	w10, w11
	csel	w10, w10, w11, lo
	add	w10, w10, w22
	cmp	w10, w9
	cinc	w25, w8, eq
	csel	w22, wzr, w10, eq
	bl	f2fs_is_compressed_page
	tbz	w0, #0, .LBB72_14
// %bb.13:                              //   in Loop: Header=BB72_9 Depth=1
	ldr	x8, [x21, #8]
	mov	x0, x21
	mov	w2, w20
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x21, x9, eq
	ldr	x8, [x8]
	ubfx	x1, x8, #8, #1
                                        // kill: def $w1 killed $w1 killed $x1
	bl	f2fs_end_read_compressed_page
	ldur	x0, [x19, #-16]
	add	w20, w20, #1            // =1
	ldrh	w8, [x0, #458]
	cmp	w25, w8
	b.lo	.LBB72_9
	b	.LBB72_15
.LBB72_14:                              //   in Loop: Header=BB72_9 Depth=1
	ldur	x0, [x19, #-16]
	mov	w23, wzr
	add	w20, w20, #1            // =1
	ldrh	w8, [x0, #458]
	cmp	w25, w8
	b.lo	.LBB72_9
	b	.LBB72_18
.LBB72_15:
	tbz	w23, #0, .LBB72_18
// %bb.16:
	ldr	w8, [x19, #48]
.LBB72_17:
	and	w8, w8, #0xfffffffb
	str	w8, [x19, #48]
.LBB72_18:
	ldr	x8, [x0, #72]
	cbz	x8, .LBB72_20
// %bb.19:
	ldrb	w9, [x8, #64]
	tbnz	w9, #2, .LBB72_21
.LBB72_20:
	bl	f2fs_finish_read_bio
	b	.LBB72_22
.LBB72_21:
	mov	x9, #68719476704
	adrp	x10, f2fs_verify_bio
	str	x9, [x8, #16]!
	add	x10, x10, :lo12:f2fs_verify_bio
	add	x9, x8, #8              // =8
	mov	x0, x8
	str	x9, [x8, #8]
	stp	x9, x10, [x8, #16]
	bl	fsverity_enqueue_verify_work
.LBB72_22:
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.LBB72_23:                              //   in Loop: Header=BB72_26 Depth=1
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 477; .short (1 << 0)|((1 << 1) | ((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
.LBB72_24:                              //   in Loop: Header=BB72_26 Depth=1
	ldr	x8, [x21, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x21, x9, eq
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 256
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB72_25:                              //   in Loop: Header=BB72_26 Depth=1
	ldrh	w8, [x25, #458]
	cmp	w27, w8
	b.hs	.LBB72_6
.LBB72_26:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB72_33 Depth 2
	ldr	x9, [x25, #472]
	mov	w8, w27
	cbz	w26, .LBB72_28
// %bb.27:                              //   in Loop: Header=BB72_26 Depth=1
	mov	w22, wzr
	add	x21, x21, #64           // =64
	b	.LBB72_29
.LBB72_28:                              //   in Loop: Header=BB72_26 Depth=1
	add	x10, x9, x8, lsl #4
	ldr	w11, [x10, #12]
	ldr	x10, [x10]
	lsr	x12, x11, #6
	and	x12, x12, #0x3ffffc0
	add	x21, x10, x12
	and	w22, w11, #0xfff
.LBB72_29:                              //   in Loop: Header=BB72_26 Depth=1
	add	x9, x9, x8, lsl #4
	ldr	w12, [x9, #8]
	mov	w10, #4096
	sub	w11, w10, w22
	ldrb	w9, [x20, #142]
	sub	w14, w12, w26
	ldr	x10, [x21, #32]
	ldr	x13, [x21, #8]
	cmp	w11, w14
	csel	w11, w11, w14, lo
	add	w14, w11, w26
	cmp	w14, w12
	sub	x12, x13, #1            // =1
	cinc	w27, w8, eq
	csel	w26, wzr, w14, eq
	tst	x13, #0x1
	csel	x8, x21, x12, eq
	ldr	x8, [x8]
	tbz	w8, #0, .LBB72_35
// %bb.30:                              //   in Loop: Header=BB72_26 Depth=1
	cbz	w11, .LBB72_23
// %bb.31:                              //   in Loop: Header=BB72_26 Depth=1
	mov	w8, #1
	lsl	w23, w8, w9
	mov	w12, #8191
	orr	w8, w11, w22
	add	w12, w23, w12
	and	w8, w12, w8
	cbnz	w8, .LBB72_23
// %bb.32:                              //   in Loop: Header=BB72_26 Depth=1
	mov	w8, #12
	sub	w8, w8, w9
	lsl	x8, x10, x8
	lsr	x9, x22, x9
	add	w28, w11, w22
	add	x24, x8, x9
.LBB72_33:                              //   Parent Loop BB72_26 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	mov	w5, #3136
	mov	x0, x20
	mov	x1, x21
	mov	w2, w23
	mov	w3, w22
	mov	x4, x24
	bl	fscrypt_encrypt_block_inplace
	cbnz	w0, .LBB72_24
// %bb.34:                              //   in Loop: Header=BB72_33 Depth=2
	add	w22, w22, w23
	cmp	w22, w28
	add	x24, x24, #1            // =1
	b.lo	.LBB72_33
	b	.LBB72_25
.LBB72_35:                              //   in Loop: Header=BB72_26 Depth=1
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/data.c"; .popsection; .long 14472b - 14470b; .short 472; .short (1 << 0)|((1 << 1) | ((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	b	.LBB72_24
.Lfunc_end72:
	.size	f2fs_post_read_work, .Lfunc_end72-f2fs_post_read_work
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_verify_bio
	.type	f2fs_verify_bio,@function
f2fs_verify_bio:                        // @f2fs_verify_bio
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	str	x23, [sp, #16]          // 8-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x19, [x0, #-16]!
	adrp	x8, bio_post_read_ctx_pool
	ldr	x1, [x8, :lo12:bio_post_read_ctx_pool]
	mov	x29, sp
	ldr	w20, [x0, #64]
	bl	mempool_free
	str	xzr, [x19, #72]
	tbnz	w20, #1, .LBB73_2
// %bb.1:
	mov	x0, x19
	bl	fsverity_verify_bio
	b	.LBB73_12
.LBB73_2:
	ldrh	w8, [x19, #458]
	cbz	w8, .LBB73_12
// %bb.3:
	mov	w21, wzr
	mov	w23, wzr
	mov	x20, xzr
	mov	w22, #4096
	b	.LBB73_5
.LBB73_4:                               //   in Loop: Header=BB73_5 Depth=1
	ldrh	w8, [x19, #458]
	cmp	w23, w8
	b.hs	.LBB73_12
.LBB73_5:                               // =>This Inner Loop Header: Depth=1
	ldr	x9, [x19, #472]
	mov	w8, w23
	cbz	w21, .LBB73_7
// %bb.6:                               //   in Loop: Header=BB73_5 Depth=1
	mov	w10, wzr
	add	x20, x20, #64           // =64
	b	.LBB73_8
.LBB73_7:                               //   in Loop: Header=BB73_5 Depth=1
	add	x10, x9, x8, lsl #4
	ldr	w11, [x10, #12]
	ldr	x10, [x10]
	lsr	x12, x11, #6
	and	x12, x12, #0x3ffffc0
	add	x20, x10, x12
	and	w10, w11, #0xfff
.LBB73_8:                               //   in Loop: Header=BB73_5 Depth=1
	add	x9, x9, x8, lsl #4
	ldr	w9, [x9, #8]
	sub	w10, w22, w10
	mov	x0, x20
	sub	w11, w9, w21
	cmp	w10, w11
	csel	w10, w10, w11, lo
	add	w10, w10, w21
	cmp	w10, w9
	cinc	w23, w8, eq
	csel	w21, wzr, w10, eq
	bl	f2fs_is_compressed_page
	tbnz	w0, #0, .LBB73_4
// %bb.9:                               //   in Loop: Header=BB73_5 Depth=1
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #8, .LBB73_4
// %bb.10:                              //   in Loop: Header=BB73_5 Depth=1
	mov	x0, x20
	bl	fsverity_verify_page
	tbnz	w0, #0, .LBB73_4
// %bb.11:                              //   in Loop: Header=BB73_5 Depth=1
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 256
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB73_4
.LBB73_12:
	mov	x0, x19
	bl	f2fs_finish_read_bio
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end73:
	.size	f2fs_verify_bio, .Lfunc_end73-f2fs_verify_bio
                                        // -- End function
	.p2align	2               // -- Begin function verify_blkaddr
	.type	verify_blkaddr,@function
verify_blkaddr:                         // @verify_blkaddr
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	mov	w20, w2
	mov	w21, w1
	mov	x19, x0
	bl	f2fs_is_valid_blkaddr
	tbnz	w0, #0, .LBB74_9
// %bb.1:
	adrp	x1, .L.str.11
	add	x1, x1, :lo12:.L.str.11
	mov	x0, x19
	mov	w2, w21
	mov	w3, w20
	bl	f2fs_printk
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB74_4
// %bb.2:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB74_4
// %bb.3:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str
	adrp	x2, .L__func__.verify_blkaddr
	add	x1, x1, :lo12:.L.str
	add	x2, x2, :lo12:.L__func__.verify_blkaddr
	mov	w3, #4868
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB74_4:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/f2fs.h"; .popsection; .long 14472b - 14470b; .short 4868; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB74_9
// %bb.5:
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
	cbnz	w8, .LBB74_9
// %bb.6:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB74_9
// %bb.7:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB74_9
// %bb.8:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB74_9:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end74:
	.size	verify_blkaddr, .Lfunc_end74-verify_blkaddr
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_write_failed
	.type	f2fs_write_failed,@function
f2fs_write_failed:                      // @f2fs_write_failed
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x19, [x0]
	mov	x29, sp
	ldr	w8, [x19, #12]
	ldr	x20, [x19, #80]
	and	w8, w8, #0x20
	cmp	x20, x1
	ccmp	w8, #0, #0, lt
	b.ne	.LBB75_3
// %bb.1:
	ldr	x8, [x19, #736]
	tbnz	w8, #30, .LBB75_3
// %bb.2:
	add	x21, x19, #1040         // =1040
	mov	x0, x21
	bl	down_write
	add	x22, x19, #1088         // =1088
	mov	x0, x22
	bl	down_write
	mov	x0, x19
	mov	x1, x20
	bl	truncate_pagecache
	mov	w2, #1
	mov	x0, x19
	mov	x1, x20
	bl	f2fs_truncate_blocks
	mov	x0, x22
	bl	up_write
	mov	x0, x21
	bl	up_write
.LBB75_3:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end75:
	.size	f2fs_write_failed, .Lfunc_end75-f2fs_write_failed
                                        // -- End function
	.p2align	2               // -- Begin function get_data_block_dio_write
	.type	get_data_block_dio_write,@function
get_data_block_dio_write:               // @get_data_block_dio_write
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x21, x0
	ldrb	w0, [x0, #143]
	mov	x29, sp
	mov	w19, w3
	mov	x20, x2
	mov	x22, x1
	bl	f2fs_rw_hint_to_seg_type
	mov	w4, w0
	mov	w5, #1
	mov	x0, x21
	mov	x1, x22
	mov	x2, x20
	mov	w3, w19
	bl	__get_data_block
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end76:
	.size	get_data_block_dio_write, .Lfunc_end76-get_data_block_dio_write
                                        // -- End function
	.p2align	2               // -- Begin function get_data_block_dio
	.type	get_data_block_dio,@function
get_data_block_dio:                     // @get_data_block_dio
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x21, x0
	ldrb	w0, [x0, #143]
	mov	x29, sp
	mov	w19, w3
	mov	x20, x2
	mov	x22, x1
	bl	f2fs_rw_hint_to_seg_type
	mov	w4, w0
	mov	x0, x21
	mov	x1, x22
	mov	x2, x20
	mov	w3, w19
	mov	w5, wzr
	bl	__get_data_block
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end77:
	.size	get_data_block_dio, .Lfunc_end77-get_data_block_dio
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_dio_submit_bio
	.type	f2fs_dio_submit_bio,@function
f2fs_dio_submit_bio:                    // @f2fs_dio_submit_bio
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	adrp	x8, kmalloc_caches+56
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	ldrb	w21, [x0, #16]
	ldr	x0, [x8, :lo12:kmalloc_caches+56]
	mov	x20, x1
	mov	w1, #3392
	mov	w2, #32
	mov	x29, sp
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB78_2
// %bb.1:
	ldr	x8, [x19, #64]
	cmp	w21, #1                 // =1
	cset	w9, eq
	mov	w10, #48
	str	x8, [x0, #16]
	ldr	x8, [x19, #72]
	strb	w9, [x0, #24]
	adrp	x9, f2fs_dio_end_io
	add	x9, x9, :lo12:f2fs_dio_end_io
	stp	x20, x8, [x0]
	stp	x9, x0, [x19, #64]
	ldr	x8, [x20, #40]
	mov	w9, #52
	csel	x9, x10, x9, eq
	mov	x0, x19
	ldr	x8, [x8, #1008]
	add	x8, x8, x9
	add	x8, x8, #1272           // =1272
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	bl	submit_bio
	b	.LBB78_3
.LBB78_2:
	mov	w8, #10
	mov	x0, x19
	strb	w8, [x19, #26]
	bl	bio_endio
.LBB78_3:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end78:
	.size	f2fs_dio_submit_bio, .Lfunc_end78-f2fs_dio_submit_bio
                                        // -- End function
	.p2align	2               // -- Begin function __get_data_block
	.type	__get_data_block,@function
__get_data_block:                       // @__get_data_block
// %bb.0:
	sub	sp, sp, #96             // =96
	adrp	x9, __stack_chk_guard
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	add	x29, sp, #48            // =48
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	stur	x9, [x29, #-8]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	str	xzr, [sp]
	ldr	x9, [x0, #40]
	ldrb	w11, [x0, #142]
	mov	w8, w3
	mov	x19, x2
	ldr	x22, [x9, #1008]
	ldr	x9, [x2, #32]
	and	w10, w5, #0x1
	mov	w3, #3
	mov	w2, w8
	lsr	x9, x9, x11
	stp	w1, w9, [sp, #4]
	mov	x1, sp
	mov	x20, x0
	stp	xzr, xzr, [sp, #16]
	str	w4, [sp, #32]
	strb	w10, [sp, #36]
	bl	f2fs_map_blocks
	mov	w21, w0
	cbnz	w0, .LBB79_15
// %bb.1:
	ldr	x9, [x20, #40]
	ldr	w8, [sp]
	ldr	x10, [x19]
	tbnz	w10, #5, .LBB79_3
// %bb.2:
	//APP
	// atomic64_or
	prfm	pstl1strm, [x19]
1:	ldxr	x10, [x19]
	orr	x10, x10, 32
	stxr	w11, x10, [x19]
	cbnz	w11, 1b
	//NO_APP
.LBB79_3:
	ldr	x10, [x9, #248]
	str	x8, [x19, #24]
	str	x10, [x19, #48]
	ldr	x9, [x9, #24]
	str	x9, [x19, #32]
	ldr	w9, [x22, #3128]
	cbz	w9, .LBB79_14
// %bb.4:
	ldr	w2, [sp]
	cmp	w9, #2                  // =2
	b.lt	.LBB79_11
// %bb.5:
	ldr	x10, [x22, #3136]
	mov	x11, xzr
	mov	x12, xzr
	mov	x14, #4294967296
	add	x13, x10, #80           // =80
	b	.LBB79_7
.LBB79_6:                               //   in Loop: Header=BB79_7 Depth=1
	add	x12, x12, #1            // =1
	add	x11, x11, x14
	cmp	x9, x12
	add	x13, x13, #88           // =88
	b.eq	.LBB79_11
.LBB79_7:                               // =>This Inner Loop Header: Depth=1
	ldur	w15, [x13, #-4]
	cmp	w15, w2
	b.hi	.LBB79_6
// %bb.8:                               //   in Loop: Header=BB79_7 Depth=1
	ldr	w15, [x13]
	cmp	w15, w2
	b.lo	.LBB79_6
// %bb.9:
	cbz	w12, .LBB79_11
// %bb.10:
	asr	x9, x11, #32
	mov	w11, #88
	mul	x11, x9, x11
	ldr	x10, [x10, x11]
	str	x10, [x19, #48]
	ldr	x10, [x22, #3136]
	add	x10, x10, x11
	ldr	w10, [x10, #76]
	sub	x8, x8, x10
	str	x8, [x19, #24]
	b	.LBB79_12
.LBB79_11:
	mov	x9, xzr
.LBB79_12:
	ldr	x8, [x22, #3136]
	mov	w10, #88
	mul	x9, x9, x10
	ldr	x8, [x8, x9]
	cbz	x8, .LBB79_14
// %bb.13:
	ldr	w1, [x20, #64]
	mov	x0, x22
	bl	f2fs_update_device_state
.LBB79_14:
	ldr	x8, [x19]
	ldr	w9, [sp, #12]
	mov	x10, #-4193
	and	x8, x8, x10
	orr	x8, x8, x9
	str	x8, [x19]
	ldr	w8, [sp, #8]
	ldrb	w9, [x20, #142]
	lsl	x8, x8, x9
	str	x8, [x19, #32]
.LBB79_15:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB79_17
// %bb.16:
	mov	w0, w21
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #96             // =96
	ret
.LBB79_17:
	bl	__stack_chk_fail
.Lfunc_end79:
	.size	__get_data_block, .Lfunc_end79-__get_data_block
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_dio_end_io
	.type	f2fs_dio_end_io,@function
f2fs_dio_end_io:                        // @f2fs_dio_end_io
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	ldr	x0, [x0, #72]
	mov	w10, #48
	mov	x29, sp
	ldr	x8, [x0]
	ldrb	w9, [x0, #24]
	ldr	x8, [x8, #40]
	cmp	w9, #0                  // =0
	mov	w9, #52
	csel	x9, x9, x10, eq
	ldr	x8, [x8, #1008]
	add	x8, x8, x9
	add	x8, x8, #1272           // =1272
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	ldr	x8, [x0, #8]
	str	x8, [x19, #72]
	ldr	x8, [x0, #16]
	str	x8, [x19, #64]
	bl	kfree
	mov	x0, x19
	bl	bio_endio
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end80:
	.size	f2fs_dio_end_io, .Lfunc_end80-f2fs_dio_end_io
                                        // -- End function
	.type	f2fs_bioset,@object     // @f2fs_bioset
	.local	f2fs_bioset
	.comm	f2fs_bioset,272,8
	.type	.L.str,@object          // @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"F2FS bug: %s:%d\n"
	.size	.L.str, 17

	.type	.L__func__.f2fs_submit_page_write,@object // @__func__.f2fs_submit_page_write
.L__func__.f2fs_submit_page_write:
	.asciz	"f2fs_submit_page_write"
	.size	.L__func__.f2fs_submit_page_write, 23

	.type	.L__func__.f2fs_get_new_data_page,@object // @__func__.f2fs_get_new_data_page
.L__func__.f2fs_get_new_data_page:
	.asciz	"f2fs_get_new_data_page"
	.size	.L__func__.f2fs_get_new_data_page, 23

	.type	.L.str.1,@object        // @.str.1
.L.str.1:
	.asciz	"\0013inode[%d] dedup addr error"
	.size	.L.str.1, 29

	.type	.L__func__.f2fs_map_blocks,@object // @__func__.f2fs_map_blocks
.L__func__.f2fs_map_blocks:
	.asciz	"f2fs_map_blocks"
	.size	.L__func__.f2fs_map_blocks, 16

	.type	.L__func__.f2fs_read_multi_pages,@object // @__func__.f2fs_read_multi_pages
.L__func__.f2fs_read_multi_pages:
	.asciz	"f2fs_read_multi_pages"
	.size	.L__func__.f2fs_read_multi_pages, 22

	.type	f2fs_dblock_aops,@object // @f2fs_dblock_aops
	.section	.rodata,"a",@progbits
	.globl	f2fs_dblock_aops
	.p2align	3
f2fs_dblock_aops:
.Lf2fs_dblock_aops$local:
	.xword	f2fs_write_data_page
	.xword	f2fs_read_data_page
	.xword	f2fs_write_data_pages
	.xword	f2fs_set_data_page_dirty
	.xword	f2fs_read_data_pages
	.xword	f2fs_write_begin
	.xword	f2fs_write_end
	.xword	f2fs_bmap
	.xword	f2fs_invalidate_page
	.xword	f2fs_release_page
	.xword	0
	.xword	f2fs_direct_IO
	.xword	f2fs_migrate_page
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	f2fs_swap_activate
	.xword	f2fs_swap_deactivate
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	f2fs_dblock_aops, 200

	.type	.L.str.3,@object        // @.str.3
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.3:
	.asciz	"f2fs_bio_post_read_ctx"
	.size	.L.str.3, 23

	.type	bio_post_read_ctx_cache,@object // @bio_post_read_ctx_cache
	.local	bio_post_read_ctx_cache
	.comm	bio_post_read_ctx_cache,8,8
	.type	bio_post_read_ctx_pool,@object // @bio_post_read_ctx_pool
	.local	bio_post_read_ctx_pool
	.comm	bio_post_read_ctx_pool,8,8
	.type	.L.str.4,@object        // @.str.4
.L.str.4:
	.asciz	"f2fs_post_read_wq"
	.size	.L.str.4, 18

	.type	.L.str.5,@object        // @.str.5
.L.str.5:
	.asciz	"f2fs_bio_entry_slab"
	.size	.L.str.5, 20

	.type	bio_entry_slab,@object  // @bio_entry_slab
	.local	bio_entry_slab
	.comm	bio_entry_slab,8,8
	.type	.L__func__.__submit_bio,@object // @__func__.__submit_bio
.L__func__.__submit_bio:
	.asciz	"__submit_bio"
	.size	.L__func__.__submit_bio, 13

	.type	.L.str.6,@object        // @.str.6
.L.str.6:
	.asciz	"\0013%s: read IO fault[%d]\n"
	.size	.L.str.6, 25

	.type	.L__func__.f2fs_finish_read_bio,@object // @__func__.f2fs_finish_read_bio
.L__func__.f2fs_finish_read_bio:
	.asciz	"f2fs_finish_read_bio"
	.size	.L__func__.f2fs_finish_read_bio, 21

	.type	.L.str.7,@object        // @.str.7
.L.str.7:
	.asciz	"%s: read IO fault[%d]\n"
	.size	.L.str.7, 23

	.type	.L__func__.f2fs_write_end_io,@object // @__func__.f2fs_write_end_io
.L__func__.f2fs_write_end_io:
	.asciz	"f2fs_write_end_io"
	.size	.L__func__.f2fs_write_end_io, 18

	.type	.L.str.8,@object        // @.str.8
.L.str.8:
	.asciz	"\0013f2fs reboot for bio submit error! erro_num = %d"
	.size	.L.str.8, 50

	.type	.L.str.9,@object        // @.str.9
.L.str.9:
	.asciz	"F2FS reboot: %s:%d [%d]\n"
	.size	.L.str.9, 25

	.type	.L.str.10,@object       // @.str.10
.L.str.10:
	.asciz	"ERROR"
	.size	.L.str.10, 6

	.type	.L__func__.add_bio_entry,@object // @__func__.add_bio_entry
.L__func__.add_bio_entry:
	.asciz	"add_bio_entry"
	.size	.L__func__.add_bio_entry, 14

	.type	.L__func__.add_ipu_page,@object // @__func__.add_ipu_page
.L__func__.add_ipu_page:
	.asciz	"add_ipu_page"
	.size	.L__func__.add_ipu_page, 13

	.type	.L.str.11,@object       // @.str.11
.L.str.11:
	.asciz	"\0013invalid blkaddr: %u, type: %d, run fsck to fix."
	.size	.L.str.11, 50

	.type	.L__func__.verify_blkaddr,@object // @__func__.verify_blkaddr
.L__func__.verify_blkaddr:
	.asciz	"verify_blkaddr"
	.size	.L__func__.verify_blkaddr, 15

	.type	.L__func__.f2fs_put_page,@object // @__func__.f2fs_put_page
.L__func__.f2fs_put_page:
	.asciz	"f2fs_put_page"
	.size	.L__func__.f2fs_put_page, 14

	.type	.L.str.12,@object       // @.str.12
.L.str.12:
	.asciz	"\0013cp_rwsem_timeout: start: %lu now: %lu\n"
	.size	.L.str.12, 41

	.type	.L__func__.put_inner_inode,@object // @__func__.put_inner_inode
.L__func__.put_inner_inode:
	.asciz	"put_inner_inode"
	.size	.L__func__.put_inner_inode, 16

	.type	.L.str.13,@object       // @.str.13
.L.str.13:
	.asciz	"\0013F2FS-fs: inode [%lu] inflight_read_io refcount may leak"
	.size	.L.str.13, 58

	.type	.L.str.14,@object       // @.str.14
.L.str.14:
	.asciz	"\0014%s, potential error with diff:%d, i_compr_blocks:%d, i_ino:%lu"
	.size	.L.str.14, 65

	.type	.L__func__.f2fs_i_compr_blocks_update,@object // @__func__.f2fs_i_compr_blocks_update
.L__func__.f2fs_i_compr_blocks_update:
	.asciz	"f2fs_i_compr_blocks_update"
	.size	.L__func__.f2fs_i_compr_blocks_update, 27

	.type	.L.str.15,@object       // @.str.15
.L.str.15:
	.asciz	"\0013Swapfile not supported in LFS mode"
	.size	.L.str.15, 37

	.type	.L.str.16,@object       // @.str.16
.L.str.16:
	.asciz	"\0013Swapfile not supported in dedup mode"
	.size	.L.str.16, 39

	.type	.L.str.17,@object       // @.str.17
.L.str.17:
	.asciz	"\0013Swapfile has holes"
	.size	.L.str.17, 21

	.type	.L.str.18,@object       // @.str.18
.L.str.18:
	.asciz	"\0014Swapfile: last extent is not aligned to section"
	.size	.L.str.18, 50

	.type	.L.str.19,@object       // @.str.19
.L.str.19:
	.asciz	"\0014Swapfile (%u) is not align to section: 1) creat(), 2) ioctl(F2FS_IOC_SET_PIN_FILE), 3) fallocate(%u * N)"
	.size	.L.str.19, 107

	.ident	"Android (6443078 based on r383902) clang version 11.0.1 (https://android.googlesource.com/toolchain/llvm-project b397f81060ce6d701042b782172ed13bee898b79)"
	.section	".note.GNU-stack","",@progbits
