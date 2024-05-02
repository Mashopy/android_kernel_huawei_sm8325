	.text
	.file	"segment.c"
	.globl	__find_rev_next_bit     // -- Begin function __find_rev_next_bit
	.p2align	2
	.type	__find_rev_next_bit,@function
__find_rev_next_bit:                    // @__find_rev_next_bit
.L__find_rev_next_bit$local:
// %bb.0:
	cmp	x2, x1
	b.hs	.LBB0_7
// %bb.1:
	lsr	x9, x2, #6
	and	x10, x2, #0xffffffffffffffc0
	sub	x10, x10, x1
	add	x11, x0, x9, lsl #3
	and	x12, x2, #0x3f
	neg	x8, x9, lsl #6
	add	x9, x10, #64            // =64
	add	x10, x11, #3            // =3
	mov	x11, #-1
.LBB0_2:                                // =>This Inner Loop Header: Depth=1
	ldur	x14, [x10, #-3]
	add	x13, x1, x8
	cbz	x14, .LBB0_4
// %bb.3:                               //   in Loop: Header=BB0_2 Depth=1
	ldurb	w15, [x10, #-2]
	ldurb	w16, [x10, #-1]
	lsl	x14, x14, #56
	ldrb	w17, [x10]
	bfi	x14, x15, #48, #8
	ldrb	w15, [x10, #1]
	bfi	x14, x16, #40, #8
	ldrb	w16, [x10, #2]
	bfi	x14, x17, #32, #8
	ldrb	w17, [x10, #3]
	bfi	x14, x15, #24, #8
	ldrb	w15, [x10, #4]
	bfi	x14, x16, #16, #8
	bfi	x14, x17, #8, #8
	cmp	x13, #64                // =64
	orr	x14, x14, x15
	lsl	x15, x11, x9
	lsr	x12, x11, x12
	csinv	x15, x15, xzr, lo
	and	x12, x15, x12
	ands	x12, x12, x14
	b.ne	.LBB0_6
.LBB0_4:                                //   in Loop: Header=BB0_2 Depth=1
	cmp	x13, #65                // =65
	b.lo	.LBB0_7
// %bb.5:                               //   in Loop: Header=BB0_2 Depth=1
	mov	x12, xzr
	sub	x8, x8, #64             // =64
	add	x9, x9, #64             // =64
	add	x10, x10, #8            // =8
	b	.LBB0_2
.LBB0_6:
	lsr	x9, x12, #32
	cmp	x9, #0                  // =0
	mov	w10, #16
	cset	w11, eq
	csel	x9, x12, x9, eq
	lsl	w12, w11, #5
	bfi	w10, w11, #5, #1
	lsr	x11, x9, #16
	tst	x9, #0xffff0000
	csel	x9, x9, x11, eq
	csel	w10, w10, w12, eq
	orr	w11, w10, #0x8
	lsr	x12, x9, #8
	tst	x9, #0xff00
	csel	x9, x9, x12, eq
	csel	w10, w11, w10, eq
	orr	w11, w10, #0x4
	lsr	x12, x9, #4
	tst	x9, #0xf0
	csel	x9, x9, x12, eq
	csel	w10, w11, w10, eq
	add	w11, w10, #2            // =2
	ubfx	x12, x9, #2, #30
	tst	x9, #0xc
	csel	w10, w11, w10, eq
	csel	w9, w9, w12, eq
	mov	w11, #1
	bic	w9, w11, w9, lsr #1
	add	w9, w9, w10
	sxtw	x9, w9
	sub	x1, x9, x8
.LBB0_7:
	mov	x0, x1
	ret
.Lfunc_end0:
	.size	__find_rev_next_bit, .Lfunc_end0-__find_rev_next_bit
                                        // -- End function
	.globl	__find_rev_next_zero_bit // -- Begin function __find_rev_next_zero_bit
	.p2align	2
	.type	__find_rev_next_zero_bit,@function
__find_rev_next_zero_bit:               // @__find_rev_next_zero_bit
.L__find_rev_next_zero_bit$local:
// %bb.0:
	cmp	x2, x1
	b.hs	.LBB1_7
// %bb.1:
	lsr	x9, x2, #6
	neg	x8, x9, lsl #6
	add	x9, x0, x9, lsl #3
	and	x11, x2, #0x3f
	add	x9, x9, #3              // =3
	mov	x10, #-1
.LBB1_2:                                // =>This Inner Loop Header: Depth=1
	ldur	x13, [x9, #-3]
	add	x12, x1, x8
	cmn	x13, #1                 // =1
	b.eq	.LBB1_4
// %bb.3:                               //   in Loop: Header=BB1_2 Depth=1
	neg	x4, x11
	cmp	x11, #0                 // =0
	lsl	x4, x10, x4
	ldurb	w14, [x9, #-2]
	lsr	x11, x10, x12
	csel	x4, xzr, x4, eq
	cmp	x12, #64                // =64
	ldurb	w15, [x9, #-1]
	csel	x11, x11, xzr, lo
	ldrb	w16, [x9]
	orr	x11, x4, x11
	ldrb	w17, [x9, #1]
	orr	x11, x11, x13, lsl #56
	ldrb	w0, [x9, #2]
	orr	x11, x11, x14, lsl #48
	ldrb	w2, [x9, #3]
	orr	x11, x11, x15, lsl #40
	ldrb	w3, [x9, #4]
	orr	x11, x11, x16, lsl #32
	orr	x11, x11, x17, lsl #24
	orr	x11, x11, x0, lsl #16
	orr	x11, x11, x2, lsl #8
	orr	x11, x11, x3
	cmn	x11, #1                 // =1
	b.ne	.LBB1_6
.LBB1_4:                                //   in Loop: Header=BB1_2 Depth=1
	cmp	x12, #65                // =65
	b.lo	.LBB1_7
// %bb.5:                               //   in Loop: Header=BB1_2 Depth=1
	mov	x11, xzr
	sub	x8, x8, #64             // =64
	add	x9, x9, #8              // =8
	b	.LBB1_2
.LBB1_6:
	mvn	x9, x11
	mov	x10, #-4294967297
	cmp	x11, x10
	lsr	x9, x9, #32
	mov	w12, #16
	cset	w10, hi
	csinv	x9, x9, x11, ls
	lsl	w11, w10, #5
	bfi	w12, w10, #5, #1
	lsr	x10, x9, #16
	tst	x9, #0xffff0000
	csel	x9, x9, x10, eq
	csel	w10, w12, w11, eq
	orr	w11, w10, #0x8
	lsr	x12, x9, #8
	tst	x9, #0xff00
	csel	x9, x9, x12, eq
	csel	w10, w11, w10, eq
	orr	w11, w10, #0x4
	lsr	x12, x9, #4
	tst	x9, #0xf0
	csel	x9, x9, x12, eq
	csel	w10, w11, w10, eq
	add	w11, w10, #2            // =2
	ubfx	x12, x9, #2, #30
	tst	x9, #0xc
	csel	w10, w11, w10, eq
	csel	w9, w9, w12, eq
	mov	w11, #1
	bic	w9, w11, w9, lsr #1
	add	w9, w9, w10
	sxtw	x9, w9
	sub	x1, x9, x8
.LBB1_7:
	mov	x0, x1
	ret
.Lfunc_end1:
	.size	__find_rev_next_zero_bit, .Lfunc_end1-__find_rev_next_zero_bit
                                        // -- End function
	.globl	find_next_free_extent   // -- Begin function find_next_free_extent
	.p2align	2
	.type	find_next_free_extent,@function
find_next_free_extent:                  // @find_next_free_extent
.Lfind_next_free_extent$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x19, x2
	ldr	x2, [x2]
	mov	x29, sp
	mov	x20, x1
	mov	x22, x0
	bl	__find_rev_next_zero_bit
	cmp	x0, x20
	b.hs	.LBB2_2
// %bb.1:
	mov	x21, x0
	mov	x0, x22
	mov	x1, x20
	mov	x2, x21
	bl	__find_rev_next_bit
	str	x0, [x19]
	sub	w0, w0, w21
	b	.LBB2_3
.LBB2_2:
	mov	w0, #-12
.LBB2_3:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end2:
	.size	find_next_free_extent, .Lfunc_end2-find_next_free_extent
                                        // -- End function
	.globl	f2fs_need_SSR           // -- Begin function f2fs_need_SSR
	.p2align	2
	.type	f2fs_need_SSR,@function
f2fs_need_SSR:                          // @f2fs_need_SSR
.Lf2fs_need_SSR$local:
// %bb.0:
	ldr	w8, [x0, #1192]
	ldr	w13, [x0, #1188]
	ldrsw	x10, [x0, #1284]
	ldr	w9, [x0, #1184]
	ldrsw	x11, [x0, #1272]
	ldrsw	x12, [x0, #1296]
	ldr	w14, [x0, #1504]
	cmp	w14, #1                 // =1
	b.ne	.LBB3_2
// %bb.1:
	mov	w0, wzr
	ret
.LBB3_2:
	ldr	w14, [x0, #1988]
	cmp	w14, #5                 // =5
	b.ne	.LBB3_5
// %bb.3:
	ldrb	w14, [x0, #1232]
	tbnz	w14, #3, .LBB3_5
.LBB3_4:
	mov	w0, #1
	ret
.LBB3_5:
	ldr	x14, [x0, #80]
	tbnz	w14, #8, .LBB3_4
// %bb.6:
	ldr	x14, [x0, #136]
	mul	w13, w13, w8
	sub	x13, x13, #1            // =1
	add	x10, x13, x10
	add	x11, x13, x11
	add	x12, x13, x12
	ldr	x13, [x14, #8]
	asr	x11, x11, x9
	asr	x10, x10, x9
	asr	x9, x12, x9
	udiv	w12, w11, w8
	ldr	w16, [x14, #156]
	ldr	w11, [x14, #100]
	udiv	w10, w10, w8
	udiv	w15, w9, w8
	ldr	w9, [x13, #8]
	add	w10, w10, w12, lsl #1
	add	w10, w10, w15
	cmn	w11, #1                 // =1
	add	w10, w10, w16
	b.eq	.LBB3_8
// %bb.7:
	udiv	w8, w11, w8
	b	.LBB3_9
.LBB3_8:
	mov	w8, #-1
.LBB3_9:
	add	w8, w10, w8
	cmp	w9, w8
	cset	w0, ls
	ret
.Lfunc_end3:
	.size	f2fs_need_SSR, .Lfunc_end3-f2fs_need_SSR
                                        // -- End function
	.globl	f2fs_register_inmem_page // -- Begin function f2fs_register_inmem_page
	.p2align	2
	.type	f2fs_register_inmem_page,@function
f2fs_register_inmem_page:               // @f2fs_register_inmem_page
.Lf2fs_register_inmem_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x1]
	mov	x19, x1
	mov	x20, x0
	mov	x29, sp
	tbnz	w8, #13, .LBB4_2
// %bb.1:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
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
	prfm	pstl1strm, [x19]
1:	ldxr	x8, [x19]
	orr	x8, x8, 8192
	stxr	w9, x8, [x19]
	cbnz	w9, 1b
	//NO_APP
	str	xzr, [x19, #40]
.LBB4_2:
	add	x8, x19, #40            // =40
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
	orr	x9, x9, 2
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	adrp	x8, inmem_entry_slab
	ldr	x22, [x8, :lo12:inmem_entry_slab]
	mov	w1, #3136
	mov	x0, x22
	bl	kmem_cache_alloc
	mov	x21, x0
	cbnz	x0, .LBB4_4
// %bb.3:
	mov	w1, #35904
	mov	x0, x22
	bl	kmem_cache_alloc
	mov	x21, x0
.LBB4_4:
	stp	x21, x19, [x21, #8]
	str	x21, [x21]
	ldr	x8, [x19, #8]
	add	x22, x20, #952          // =952
	mov	x0, x22
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	add	x8, x8, #52             // =52
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	bl	mutex_lock
	ldr	x23, [x20, #936]
	add	x24, x20, #928          // =928
	mov	x0, x21
	mov	x2, x24
	mov	x1, x23
	bl	__list_add_valid
	tbz	w0, #0, .LBB4_6
// %bb.5:
	str	x21, [x20, #936]
	stp	x24, x23, [x21]
	str	x21, [x23]
.LBB4_6:
	ldr	x8, [x20, #40]
	mov	x0, x22
	ldr	x8, [x8, #1008]
	add	x8, x8, #1292           // =1292
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	bl	mutex_unlock
	adrp	x8, __tracepoint_f2fs_register_inmem_page+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_register_inmem_page+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB4_12
// %bb.7:
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
	tbz	w8, #0, .LBB4_12
// %bb.8:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_register_inmem_page+32
	ldr	x21, [x8, :lo12:__tracepoint_f2fs_register_inmem_page+32]
	cbz	x21, .LBB4_10
.LBB4_9:                                // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	mov	w2, #5
	mov	x1, x19
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB4_9
.LBB4_10:
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB4_13
// %bb.11:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB4_13
.LBB4_12:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB4_13:
	bl	preempt_schedule_notrace
	b	.LBB4_12
.Lfunc_end4:
	.size	f2fs_register_inmem_page, .Lfunc_end4-f2fs_register_inmem_page
                                        // -- End function
	.globl	f2fs_drop_inmem_pages_all // -- Begin function f2fs_drop_inmem_pages_all
	.p2align	2
	.type	f2fs_drop_inmem_pages_all,@function
f2fs_drop_inmem_pages_all:              // @f2fs_drop_inmem_pages_all
.Lf2fs_drop_inmem_pages_all$local:
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	ldr	w26, [x0, #2000]
	add	x21, x0, #1028          // =1028
	mov	x20, x0
	add	x22, x0, #1000          // =1000
	mov	x0, x21
	str	x27, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	w19, w1
	bl	_raw_spin_lock
	ldr	x23, [x20, #1000]
	cmp	x22, x23
	b.eq	.LBB5_15
// %bb.1:
	mov	w27, wzr
	b	.LBB5_3
.LBB5_2:                                //   in Loop: Header=BB5_3 Depth=1
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	x23, [x22]
	cmp	x22, x23
	b.eq	.LBB5_15
.LBB5_3:                                // =>This Inner Loop Header: Depth=1
	sub	x0, x23, #912           // =912
	bl	igrab
	cbz	x0, .LBB5_12
// %bb.4:                               //   in Loop: Header=BB5_3 Depth=1
	mov	x24, x0
	mov	x0, x23
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB5_6
// %bb.5:                               //   in Loop: Header=BB5_3 Depth=1
	ldp	x9, x8, [x23]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB5_6:                                //   in Loop: Header=BB5_3 Depth=1
	ldr	x25, [x20, #1008]
	mov	x0, x23
	mov	x2, x22
	mov	x1, x25
	bl	__list_add_valid
	tbz	w0, #0, .LBB5_8
// %bb.7:                               //   in Loop: Header=BB5_3 Depth=1
	str	x23, [x20, #1008]
	stp	x22, x25, [x23]
	str	x23, [x25]
.LBB5_8:                                //   in Loop: Header=BB5_3 Depth=1
	mov	x0, x21
	bl	_raw_spin_unlock
	tbz	w19, #0, .LBB5_10
// %bb.9:                               //   in Loop: Header=BB5_3 Depth=1
	ldur	w8, [x23, #-188]
	cbz	w8, .LBB5_11
.LBB5_10:                               //   in Loop: Header=BB5_3 Depth=1
	add	x8, x24, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 536870912
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	x0, x24
	bl	f2fs_drop_inmem_pages
.LBB5_11:                               //   in Loop: Header=BB5_3 Depth=1
	mov	x0, x24
	bl	iput
	b	.LBB5_13
.LBB5_12:                               //   in Loop: Header=BB5_3 Depth=1
	mov	x0, x21
	bl	_raw_spin_unlock
.LBB5_13:                               //   in Loop: Header=BB5_3 Depth=1
	mov	w1, #5
	mov	w0, wzr
	bl	congestion_wait
	tbz	w19, #0, .LBB5_2
// %bb.14:                              //   in Loop: Header=BB5_3 Depth=1
	add	w27, w27, #1            // =1
	cmp	w27, w26
	b.lo	.LBB5_2
	b	.LBB5_16
.LBB5_15:
	mov	x0, x21
	bl	_raw_spin_unlock
.LBB5_16:
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldr	x27, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.Lfunc_end5:
	.size	f2fs_drop_inmem_pages_all, .Lfunc_end5-f2fs_drop_inmem_pages_all
                                        // -- End function
	.globl	f2fs_drop_inmem_pages   // -- Begin function f2fs_drop_inmem_pages
	.p2align	2
	.type	f2fs_drop_inmem_pages,@function
f2fs_drop_inmem_pages:                  // @f2fs_drop_inmem_pages
.Lf2fs_drop_inmem_pages$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	str	x23, [sp, #16]          // 8-byte Folded Spill
	add	x19, x0, #952           // =952
	mov	x20, x0
	ldr	x23, [x8, #1008]
	mov	x0, x19
	mov	x29, sp
	bl	mutex_lock
	ldr	x8, [x20, #928]
	add	x21, x20, #928          // =928
	cmp	x21, x8
	b.eq	.LBB6_2
.LBB6_1:                                // =>This Inner Loop Header: Depth=1
	mov	w2, #1
	mov	w4, #1
	mov	x0, x20
	mov	x1, x21
	mov	w3, wzr
	bl	__revoke_inmem_pages
	mov	x0, x19
	bl	mutex_unlock
	mov	x0, x19
	bl	mutex_lock
	ldr	x8, [x20, #928]
	cmp	x21, x8
	b.ne	.LBB6_1
.LBB6_2:
	add	x21, x23, #1028         // =1028
	mov	x0, x21
	str	wzr, [x20, #724]
	bl	_raw_spin_lock
	ldr	x8, [x20, #912]
	add	x22, x20, #912          // =912
	cmp	x8, x22
	b.eq	.LBB6_6
// %bb.3:
	mov	x0, x22
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB6_5
// %bb.4:
	ldr	x8, [x20, #920]
	ldr	x9, [x20, #912]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB6_5:
	str	x22, [x20, #912]
	str	x22, [x20, #920]
.LBB6_6:
	ldr	x8, [x20, #736]
	tbz	w8, #15, .LBB6_8
// %bb.7:
	add	x8, x20, #736           // =736
	mov	w9, #32768
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	w8, [x23, #2000]
	sub	w8, w8, #1              // =1
	str	w8, [x23, #2000]
.LBB6_8:
	mov	x0, x21
	bl	_raw_spin_unlock
	mov	x0, x19
	bl	mutex_unlock
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end6:
	.size	f2fs_drop_inmem_pages, .Lfunc_end6-f2fs_drop_inmem_pages
                                        // -- End function
	.p2align	2               // -- Begin function __revoke_inmem_pages
	.type	__revoke_inmem_pages,@function
__revoke_inmem_pages:                   // @__revoke_inmem_pages
// %bb.0:
	sub	sp, sp, #192            // =192
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #96]     // 16-byte Folded Spill
	add	x29, sp, #96            // =96
	stp	x28, x27, [sp, #112]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #128]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #144]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #160]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #176]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x24, [x1]
	cmp	x24, x1
	b.eq	.LBB7_51
// %bb.1:
	ldr	x8, [x0, #40]
	orr	w9, w2, w3
	stp	w3, w9, [sp, #8]        // 8-byte Folded Spill
	mov	w19, w4
	ldr	x8, [x8, #1008]
	mov	w20, w3
	mov	w21, w2
	mov	x22, x1
	mov	x23, x0
	mov	w27, wzr
	//APP
	mrs x28, sp_el0
	//NO_APP
	str	x8, [sp]                // 8-byte Folded Spill
	b	.LBB7_4
.LBB7_2:                                //   in Loop: Header=BB7_4 Depth=1
	mov	x9, #256
	movk	x9, #57005, lsl #48
	add	x8, x9, #34             // =34
	stp	x9, x8, [x24]
	adrp	x8, inmem_entry_slab
	ldr	x0, [x8, :lo12:inmem_entry_slab]
	mov	x1, x24
	bl	kmem_cache_free
	ldr	x8, [x23, #40]
	mov	w11, #1
	ldr	x8, [x8, #1008]
	add	x8, x8, #1292           // =1292
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w11
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB7_3:                                //   in Loop: Header=BB7_4 Depth=1
	cmp	x25, x22
	mov	x24, x25
	b.eq	.LBB7_52
.LBB7_4:                                // =>This Loop Header: Depth=1
                                        //     Child Loop BB7_8 Depth 2
                                        //     Child Loop BB7_21 Depth 2
                                        //     Child Loop BB7_26 Depth 2
	ldr	x25, [x24]
	ldr	x26, [x24, #16]
	tbz	w21, #0, .LBB7_11
// %bb.5:                               //   in Loop: Header=BB7_4 Depth=1
	adrp	x8, __tracepoint_f2fs_commit_inmem_page+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_commit_inmem_page+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB7_11
// %bb.6:                               //   in Loop: Header=BB7_4 Depth=1
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
	tbz	w8, #0, .LBB7_11
// %bb.7:                               //   in Loop: Header=BB7_4 Depth=1
	ldr	w8, [x28, #24]
	add	w8, w8, #1              // =1
	str	w8, [x28, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_commit_inmem_page+32
	ldr	x20, [x8, :lo12:__tracepoint_f2fs_commit_inmem_page+32]
	cbz	x20, .LBB7_9
.LBB7_8:                                //   Parent Loop BB7_4 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldp	x8, x0, [x20]
	mov	w2, #6
	mov	x1, x26
	blr	x8
	ldr	x8, [x20, #24]!
	cbnz	x8, .LBB7_8
.LBB7_9:                                //   in Loop: Header=BB7_4 Depth=1
	//APP
	//NO_APP
	ldr	x8, [x28, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x28, #24]
	ldr	w20, [sp, #8]           // 4-byte Folded Reload
	b.eq	.LBB7_50
// %bb.10:                              //   in Loop: Header=BB7_4 Depth=1
	ldr	x8, [x28, #24]
	cbz	x8, .LBB7_50
.LBB7_11:                               //   in Loop: Header=BB7_4 Depth=1
	ldr	x8, [x26, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x26, x9, eq
	tbz	w19, #0, .LBB7_14
// %bb.12:                              //   in Loop: Header=BB7_4 Depth=1
	ldr	x9, [x8]
	str	x9, [sp, #32]
	ldr	x9, [sp, #32]
	tbnz	w9, #0, .LBB7_3
// %bb.13:                              //   in Loop: Header=BB7_4 Depth=1
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbnz	w9, #0, .LBB7_3
	b	.LBB7_17
.LBB7_14:                               //   in Loop: Header=BB7_4 Depth=1
	ldr	x9, [x8]
	str	x9, [sp, #32]
	ldr	x9, [sp, #32]
	tbnz	w9, #0, .LBB7_16
// %bb.15:                              //   in Loop: Header=BB7_4 Depth=1
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbz	w9, #0, .LBB7_17
.LBB7_16:                               //   in Loop: Header=BB7_4 Depth=1
	mov	x0, x26
	bl	__lock_page
.LBB7_17:                               //   in Loop: Header=BB7_4 Depth=1
	mov	w2, #1
	mov	w3, #1
	mov	x0, x26
	mov	w1, wzr
	bl	f2fs_wait_on_page_writeback
	tbz	w20, #0, .LBB7_32
// %bb.18:                              //   in Loop: Header=BB7_4 Depth=1
	stp	xzr, xzr, [sp, #64]
	stp	xzr, xzr, [sp, #48]
	stp	xzr, xzr, [sp, #32]
	stp	xzr, xzr, [sp, #16]
	adrp	x8, __tracepoint_f2fs_commit_inmem_page+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_commit_inmem_page+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB7_26
// %bb.19:                              //   in Loop: Header=BB7_4 Depth=1
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
	tbz	w8, #0, .LBB7_26
// %bb.20:                              //   in Loop: Header=BB7_4 Depth=1
	ldr	w8, [x28, #24]
	add	w8, w8, #1              // =1
	str	w8, [x28, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_commit_inmem_page+32
	ldr	x20, [x8, :lo12:__tracepoint_f2fs_commit_inmem_page+32]
	cbz	x20, .LBB7_22
.LBB7_21:                               //   Parent Loop BB7_4 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldp	x8, x0, [x20]
	mov	w2, #8
	mov	x1, x26
	blr	x8
	ldr	x8, [x20, #24]!
	cbnz	x8, .LBB7_21
.LBB7_22:                               //   in Loop: Header=BB7_4 Depth=1
	//APP
	//NO_APP
	ldr	x8, [x28, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x28, #24]
	ldr	w20, [sp, #8]           // 4-byte Folded Reload
	b.eq	.LBB7_24
// %bb.23:                              //   in Loop: Header=BB7_4 Depth=1
	ldr	x8, [x28, #24]
	cbnz	x8, .LBB7_26
.LBB7_24:                               //   in Loop: Header=BB7_4 Depth=1
	bl	preempt_schedule_notrace
	b	.LBB7_26
.LBB7_25:                               //   in Loop: Header=BB7_26 Depth=2
	mov	w1, #5
	mov	w0, wzr
	bl	congestion_wait
.LBB7_26:                               //   Parent Loop BB7_4 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	add	x0, sp, #32             // =32
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	stp	x23, xzr, [sp, #32]
	str	xzr, [sp, #48]
	str	wzr, [sp, #56]
	ldr	x1, [x26, #32]
	add	x0, sp, #32             // =32
	mov	w2, #1
	bl	f2fs_get_dnode_of_data
	cmn	w0, #12                 // =12
	b.eq	.LBB7_25
// %bb.27:                              //   in Loop: Header=BB7_4 Depth=1
	cbnz	w0, .LBB7_31
// %bb.28:                              //   in Loop: Header=BB7_4 Depth=1
	ldr	w1, [sp, #56]
	ldr	x0, [sp]                // 8-byte Folded Reload
	add	x2, sp, #16             // =16
	mov	w3, wzr
	bl	get_node_info_ex
	cbnz	w0, .LBB7_54
// %bb.29:                              //   in Loop: Header=BB7_4 Depth=1
	ldr	w27, [x24, #24]
	ldr	w2, [sp, #68]
	cmn	w27, #1                 // =1
	b.eq	.LBB7_37
// %bb.30:                              //   in Loop: Header=BB7_4 Depth=1
	ldp	w8, w9, [sp, #56]
	ldrb	w10, [sp, #28]
	ldr	x0, [sp]                // 8-byte Folded Reload
	sub	x1, x29, #16            // =16
	mov	w4, #1
	mov	w5, #1
	mov	w3, w27
	mov	w6, wzr
	stur	w8, [x29, #-16]
	sturh	w9, [x29, #-11]
	sturb	w10, [x29, #-12]
	bl	f2fs_do_replace_block
	add	x0, sp, #32             // =32
	mov	w1, w27
	b	.LBB7_38
.LBB7_31:                               //   in Loop: Header=BB7_4 Depth=1
	mov	w27, #-11
.LBB7_32:                               //   in Loop: Header=BB7_4 Depth=1
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	tbz	w8, #0, .LBB7_45
.LBB7_33:                               //   in Loop: Header=BB7_4 Depth=1
	ldr	x8, [x26, #8]
	mov	w11, #4
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
	add	x8, x26, #40            // =40
	mov	w11, #8
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x11
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x26, #40]
	cmp	x8, #1                  // =1
	b.ne	.LBB7_45
// %bb.34:                              //   in Loop: Header=BB7_4 Depth=1
	str	xzr, [x26, #40]
	ldr	x8, [x26]
	tbz	w8, #13, .LBB7_45
// %bb.35:                              //   in Loop: Header=BB7_4 Depth=1
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
	cbnz	w9, .LBB7_45
// %bb.36:                              //   in Loop: Header=BB7_4 Depth=1
	bl	__put_page
	b	.LBB7_45
.LBB7_37:                               //   in Loop: Header=BB7_4 Depth=1
	ldr	x0, [sp]                // 8-byte Folded Reload
	mov	w1, w2
	bl	f2fs_invalidate_blocks
	add	x0, sp, #32             // =32
	mov	w1, #-1
.LBB7_38:                               //   in Loop: Header=BB7_4 Depth=1
	bl	f2fs_update_data_blkaddr
	ldr	x0, [sp, #48]
	cbz	x0, .LBB7_40
// %bb.39:                              //   in Loop: Header=BB7_4 Depth=1
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #48]
	ldr	x8, [sp, #40]
	cbnz	x8, .LBB7_41
	b	.LBB7_44
.LBB7_40:                               //   in Loop: Header=BB7_4 Depth=1
	mov	x9, xzr
	ldr	x8, [sp, #40]
	cbz	x8, .LBB7_44
.LBB7_41:                               //   in Loop: Header=BB7_4 Depth=1
	cmp	x9, x8
	b.eq	.LBB7_44
// %bb.42:                              //   in Loop: Header=BB7_4 Depth=1
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
	cbnz	w9, .LBB7_44
// %bb.43:                              //   in Loop: Header=BB7_4 Depth=1
	bl	__put_page
.LBB7_44:                               //   in Loop: Header=BB7_4 Depth=1
	mov	w27, wzr
	stp	xzr, xzr, [sp, #40]
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	tbnz	w8, #0, .LBB7_33
.LBB7_45:                               //   in Loop: Header=BB7_4 Depth=1
	ldr	x8, [x26]
	tbz	w8, #13, .LBB7_48
// %bb.46:                              //   in Loop: Header=BB7_4 Depth=1
	mov	w10, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x26]
1:	ldxr	x8, [x26]
	bic	x8, x8, x10
	stxr	w9, x8, [x26]
	cbnz	w9, 1b
	//NO_APP
	str	xzr, [x26, #40]
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
	cbnz	w9, .LBB7_48
// %bb.47:                              //   in Loop: Header=BB7_4 Depth=1
	bl	__put_page
.LBB7_48:                               //   in Loop: Header=BB7_4 Depth=1
	mov	w1, #1
	mov	x0, x26
	str	xzr, [x26, #40]
	bl	f2fs_put_page
	mov	x0, x24
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB7_2
// %bb.49:                              //   in Loop: Header=BB7_4 Depth=1
	ldp	x9, x8, [x24]
	str	x8, [x9, #8]
	str	x9, [x8]
	b	.LBB7_2
.LBB7_50:                               //   in Loop: Header=BB7_4 Depth=1
	bl	preempt_schedule_notrace
	b	.LBB7_11
.LBB7_51:
	mov	w27, wzr
.LBB7_52:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB7_60
// %bb.53:
	mov	w0, w27
	ldp	x20, x19, [sp, #176]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #160]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #144]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #128]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #112]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #96]     // 16-byte Folded Reload
	add	sp, sp, #192            // =192
	ret
.LBB7_54:
	mov	w27, w0
	ldr	x0, [sp, #48]
	cbz	x0, .LBB7_56
// %bb.55:
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #48]
	ldr	x8, [sp, #40]
	cbnz	x8, .LBB7_57
	b	.LBB7_52
.LBB7_56:
	mov	x9, xzr
	ldr	x8, [sp, #40]
	cbz	x8, .LBB7_52
.LBB7_57:
	cmp	x9, x8
	b.eq	.LBB7_52
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
	cbnz	w10, .LBB7_52
// %bb.59:
	bl	__put_page
	b	.LBB7_52
.LBB7_60:
	bl	__stack_chk_fail
.Lfunc_end7:
	.size	__revoke_inmem_pages, .Lfunc_end7-__revoke_inmem_pages
                                        // -- End function
	.globl	f2fs_drop_inmem_page    // -- Begin function f2fs_drop_inmem_page
	.p2align	2
	.type	f2fs_drop_inmem_page,@function
f2fs_drop_inmem_page:                   // @f2fs_drop_inmem_page
.Lf2fs_drop_inmem_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	mov	x19, x1
	mov	x21, x0
	add	x24, x0, #928           // =928
	ldr	x20, [x8, #1008]
	ldr	x8, [x1]
	mov	x29, sp
	tbz	w8, #13, .LBB8_30
// %bb.1:
	ldr	x8, [x19, #40]
	tbz	w8, #0, .LBB8_30
// %bb.2:
	ldr	x8, [x19, #40]
	tbz	w8, #1, .LBB8_30
.LBB8_3:
	add	x22, x21, #952          // =952
	mov	x0, x22
	bl	mutex_lock
	ldr	x21, [x21, #928]
	cmp	x21, x24
	b.eq	.LBB8_8
// %bb.4:
	ldr	x8, [x21, #16]
	cmp	x8, x19
	b.eq	.LBB8_7
.LBB8_5:                                // =>This Inner Loop Header: Depth=1
	ldr	x21, [x21]
	cmp	x21, x24
	b.eq	.LBB8_8
// %bb.6:                               //   in Loop: Header=BB8_5 Depth=1
	ldr	x8, [x21, #16]
	cmp	x8, x19
	b.ne	.LBB8_5
.LBB8_7:
	mov	x23, x21
	b	.LBB8_9
.LBB8_8:
	mov	x23, x24
.LBB8_9:
	ldr	x8, [x24]
	cmp	x24, x8
	b.eq	.LBB8_38
// %bb.10:
	ldr	x8, [x21, #16]
	cmp	x8, x19
	b.ne	.LBB8_38
.LBB8_11:
	mov	x0, x23
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB8_13
// %bb.12:
	ldp	x9, x8, [x21]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB8_13:
	mov	x8, #256
	movk	x8, #57005, lsl #48
	mov	w9, #34
	orr	x9, x8, x9
	mov	x0, x22
	stp	x8, x9, [x21]
	bl	mutex_unlock
	add	x8, x20, #1292          // =1292
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	adrp	x8, inmem_entry_slab
	ldr	x0, [x8, :lo12:inmem_entry_slab]
	mov	x1, x21
	bl	kmem_cache_free
	ldr	x8, [x19, #8]
	mov	w9, #4
	add	x10, x19, #40           // =40
	sub	x11, x8, #1             // =1
	tst	x8, #0x1
	csel	x8, x19, x11, eq
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x11, [x8]
	bic	x11, x11, x9
	stxr	w12, x11, [x8]
	cbnz	w12, 1b
	//NO_APP
	mov	w8, #2
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x10]
1:	ldxr	x9, [x10]
	bic	x9, x9, x8
	stxr	w11, x9, [x10]
	cbnz	w11, 1b
	//NO_APP
	ldr	x8, [x19, #40]
	cmp	x8, #1                  // =1
	b.ne	.LBB8_16
// %bb.14:
	str	xzr, [x19, #40]
	ldr	x8, [x19]
	tbz	w8, #13, .LBB8_16
// %bb.15:
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
	cbz	w10, .LBB8_29
.LBB8_16:
	cbz	x19, .LBB8_18
.LBB8_17:
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
	cbz	w10, .LBB8_21
.LBB8_18:
	ldr	x8, [x19]
	tbz	w8, #13, .LBB8_22
.LBB8_19:
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
	cbnz	w10, .LBB8_22
// %bb.20:
	bl	__put_page
	b	.LBB8_22
.LBB8_21:
	bl	__put_page
	ldr	x8, [x19]
	tbnz	w8, #13, .LBB8_19
.LBB8_22:
	str	xzr, [x19, #40]
	adrp	x8, __tracepoint_f2fs_commit_inmem_page+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_commit_inmem_page+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB8_28
// %bb.23:
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
	tbz	w8, #0, .LBB8_28
// %bb.24:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_commit_inmem_page+32
	ldr	x21, [x8, :lo12:__tracepoint_f2fs_commit_inmem_page+32]
	cbz	x21, .LBB8_26
.LBB8_25:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	mov	w2, #7
	mov	x1, x19
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB8_25
.LBB8_26:
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB8_46
// %bb.27:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB8_46
.LBB8_28:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB8_29:
	bl	__put_page
	cbnz	x19, .LBB8_17
	b	.LBB8_18
.LBB8_30:
	bl	print_bdev_access_info
	adrp	x22, f2fs_dclient
	ldr	x0, [x22, :lo12:f2fs_dclient]
	cbz	x0, .LBB8_33
// %bb.31:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB8_33
// %bb.32:
	ldr	x0, [x22, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_drop_inmem_page
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_drop_inmem_page
	mov	w3, #609
	bl	dsm_client_record
	ldr	x0, [x22, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB8_33:
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 609; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB8_3
// %bb.34:
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
	cbnz	w8, .LBB8_3
// %bb.35:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB8_3
// %bb.36:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB8_3
// %bb.37:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB8_3
.LBB8_38:
	bl	print_bdev_access_info
	adrp	x24, f2fs_dclient
	ldr	x0, [x24, :lo12:f2fs_dclient]
	cbz	x0, .LBB8_41
// %bb.39:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB8_41
// %bb.40:
	ldr	x0, [x24, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_drop_inmem_page
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_drop_inmem_page
	mov	w3, #619
	bl	dsm_client_record
	ldr	x0, [x24, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB8_41:
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 619; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB8_11
// %bb.42:
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
	cbnz	w8, .LBB8_11
// %bb.43:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB8_11
// %bb.44:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB8_11
// %bb.45:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB8_11
.LBB8_46:
	bl	preempt_schedule_notrace
	b	.LBB8_28
.Lfunc_end8:
	.size	f2fs_drop_inmem_page, .Lfunc_end8-f2fs_drop_inmem_page
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_put_page
	.type	f2fs_put_page,@function
f2fs_put_page:                          // @f2fs_put_page
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB9_32
// %bb.1:
	mov	x19, x0
	cbz	w1, .LBB9_30
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
	b.ne	.LBB9_24
// %bb.3:
	tbnz	w8, #0, .LBB9_29
// %bb.4:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB9_7
// %bb.5:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB9_7
// %bb.6:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_put_page
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_put_page
	mov	w3, #3801
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB9_7:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB9_9
// %bb.8:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB9_33
.LBB9_9:
	ldr	x0, [x19, #24]
.LBB9_10:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_raw_sb_info
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB9_12
// %bb.11:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB9_34
.LBB9_12:
	ldr	x0, [x19, #24]
.LBB9_13:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_ckpt_info
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB9_15
// %bb.14:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB9_35
.LBB9_15:
	ldr	x0, [x19, #24]
.LBB9_16:
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
	tbz	w8, #19, .LBB9_18
// %bb.17:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB9_36
.LBB9_18:
	ldr	x0, [x19, #24]
.LBB9_19:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB9_29
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
	cbnz	w9, .LBB9_29
// %bb.21:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB9_29
// %bb.22:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB9_29
// %bb.23:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB9_29
.LBB9_24:
	tbnz	w8, #0, .LBB9_29
// %bb.25:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB9_28
// %bb.26:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB9_28
// %bb.27:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_put_page
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_put_page
	mov	w3, #3803
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB9_28:
	mov	x0, xzr
	bl	f2fs_print_raw_sb_info
	mov	x0, xzr
	bl	f2fs_print_ckpt_info
	mov	x0, xzr
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/f2fs.h"; .popsection; .long 14472b - 14470b; .short 3803; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
.LBB9_29:
	mov	x0, x19
	bl	unlock_page
.LBB9_30:
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
	cbnz	w10, .LBB9_32
// %bb.31:
	bl	__put_page
.LBB9_32:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB9_33:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB9_10
.LBB9_34:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB9_13
.LBB9_35:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB9_16
.LBB9_36:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB9_19
.Lfunc_end9:
	.size	f2fs_put_page, .Lfunc_end9-f2fs_put_page
                                        // -- End function
	.globl	f2fs_commit_inmem_pages // -- Begin function f2fs_commit_inmem_pages
	.p2align	2
	.type	f2fs_commit_inmem_pages,@function
f2fs_commit_inmem_pages:                // @f2fs_commit_inmem_pages
.Lf2fs_commit_inmem_pages$local:
// %bb.0:
	sub	sp, sp, #304            // =304
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #208]    // 16-byte Folded Spill
	add	x29, sp, #208           // =208
	stp	x28, x27, [sp, #224]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #240]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #256]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #272]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #288]    // 16-byte Folded Spill
	stur	x8, [x29, #-16]
	ldr	x8, [x0, #40]
	mov	x19, x0
	mov	w1, #1
	ldr	x21, [x8, #1008]
	mov	x0, x21
	bl	f2fs_balance_fs
	add	x26, x19, #1040         // =1040
	mov	x0, x26
	bl	down_write
	add	x22, x21, #368          // =368
	mov	x0, x22
	bl	_raw_spin_lock
	ldr	x8, [x21, #352]
	cbz	x8, .LBB10_2
// %bb.1:
	adrp	x20, jiffies
	ldr	x8, [x21, #360]
	ldr	x9, [x20, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB10_42
.LBB10_2:
	mov	x0, x22
	bl	_raw_spin_unlock
	add	x20, x21, #304          // =304
	mov	x0, x20
	bl	down_read
	add	x8, x19, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 65536
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	add	x22, x19, #952          // =952
	mov	x0, x22
	bl	mutex_lock
	ldr	x8, [x19, #40]
	mov	x9, #1
	movk	x9, #10240, lsl #32
	mov	w10, #4
	ldr	x24, [x8, #1008]
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	stp	xzr, xzr, [sp, #128]
	stp	xzr, xzr, [sp, #112]
	stp	xzr, xzr, [sp, #96]
	stp	xzr, xzr, [sp, #80]
	str	xzr, [sp, #72]
	ldr	x8, [x19, #64]
	stur	x9, [sp, #76]
	add	x9, sp, #40             // =40
	str	w10, [sp, #156]
	str	xzr, [sp, #40]
	stp	w8, wzr, [sp, #64]
	str	x9, [sp, #40]
	stp	x9, x24, [sp, #48]
	ldr	x25, [x19, #928]
	add	x23, x19, #928          // =928
	cmp	x25, x23
	b.eq	.LBB10_39
// %bb.3:
	mov	w28, #1
	stp	x20, x26, [sp, #8]      // 16-byte Folded Spill
	str	wzr, [sp, #28]          // 4-byte Folded Spill
	//APP
	mrs x20, sp_el0
	//NO_APP
	b	.LBB10_5
.LBB10_4:                               //   in Loop: Header=BB10_5 Depth=1
	cmp	x21, x23
	mov	x25, x21
	b.eq	.LBB10_37
.LBB10_5:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB10_12 Depth 2
                                        //     Child Loop BB10_25 Depth 2
	ldr	x26, [x25, #16]
	ldr	x21, [x25]
	ldr	x8, [x26, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x26, x9, eq
	ldr	x9, [x8]
	str	x9, [sp, #32]
	ldr	x9, [sp, #32]
	tbnz	w9, #0, .LBB10_7
// %bb.6:                               //   in Loop: Header=BB10_5 Depth=1
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbz	w9, #0, .LBB10_8
.LBB10_7:                               //   in Loop: Header=BB10_5 Depth=1
	mov	x0, x26
	bl	__lock_page
.LBB10_8:                               //   in Loop: Header=BB10_5 Depth=1
	ldr	x8, [x26, #24]
	ldr	x9, [x19, #48]
	cmp	x8, x9
	b.ne	.LBB10_28
// %bb.9:                               //   in Loop: Header=BB10_5 Depth=1
	adrp	x8, __tracepoint_f2fs_commit_inmem_page+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_commit_inmem_page+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB10_15
// %bb.10:                              //   in Loop: Header=BB10_5 Depth=1
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
	tbz	w8, #0, .LBB10_15
// %bb.11:                              //   in Loop: Header=BB10_5 Depth=1
	ldr	w8, [x20, #24]
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_commit_inmem_page+32
	ldr	x27, [x8, :lo12:__tracepoint_f2fs_commit_inmem_page+32]
	cbz	x27, .LBB10_13
.LBB10_12:                              //   Parent Loop BB10_5 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldp	x8, x0, [x27]
	mov	w2, #5
	mov	x1, x26
	blr	x8
	ldr	x8, [x27, #24]!
	cbnz	x8, .LBB10_12
.LBB10_13:                              //   in Loop: Header=BB10_5 Depth=1
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB10_32
// %bb.14:                              //   in Loop: Header=BB10_5 Depth=1
	ldr	x8, [x20, #24]
	cbz	x8, .LBB10_32
.LBB10_15:                              //   in Loop: Header=BB10_5 Depth=1
	mov	w2, #1
	mov	w3, #1
	mov	x0, x26
	mov	w1, wzr
	bl	f2fs_wait_on_page_writeback
	mov	x0, x26
	bl	set_page_dirty
	mov	x0, x26
	bl	clear_page_dirty_for_io
	cbz	w0, .LBB10_25
// %bb.16:                              //   in Loop: Header=BB10_5 Depth=1
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB10_19
// %bb.17:                              //   in Loop: Header=BB10_5 Depth=1
	cmp	w8, #10, lsl #12        // =40960
	b.eq	.LBB10_19
// %bb.18:                              //   in Loop: Header=BB10_5 Depth=1
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB10_23
.LBB10_19:                              //   in Loop: Header=BB10_5 Depth=1
	add	x8, x19, #792           // =792
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w28
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x19, #736]
	tbz	x8, #37, .LBB10_21
// %bb.20:                              //   in Loop: Header=BB10_5 Depth=1
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	add	x8, x8, #1328           // =1328
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w28
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB10_21:                              //   in Loop: Header=BB10_5 Depth=1
	ldr	x8, [x19, #40]
	ldrh	w9, [x19]
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
	sub	w9, w9, w28
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldrb	w8, [x19, #12]
	tbz	w8, #5, .LBB10_23
// %bb.22:                              //   in Loop: Header=BB10_5 Depth=1
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	add	x8, x8, #1280           // =1280
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w28
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB10_23:                              //   in Loop: Header=BB10_5 Depth=1
	mov	x0, x19
	bl	f2fs_remove_dirty_inode
	b	.LBB10_25
.LBB10_24:                              //   in Loop: Header=BB10_25 Depth=2
	mov	w1, #5
	mov	w0, wzr
	bl	congestion_wait
.LBB10_25:                              //   Parent Loop BB10_5 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	add	x0, sp, #56             // =56
	str	wzr, [sp, #88]
	stp	x26, xzr, [sp, #96]
	str	w28, [sp, #140]
	bl	f2fs_do_write_data_page
	cmn	w0, #12                 // =12
	b.eq	.LBB10_24
// %bb.26:                              //   in Loop: Header=BB10_5 Depth=1
	mov	w27, w0
	cbnz	w0, .LBB10_33
// %bb.27:                              //   in Loop: Header=BB10_5 Depth=1
	ldr	w8, [sp, #88]
	str	w8, [x25, #24]
	mov	w8, #1
	str	w8, [sp, #28]           // 4-byte Folded Spill
.LBB10_28:                              //   in Loop: Header=BB10_5 Depth=1
	mov	x0, x26
	bl	unlock_page
	mov	x0, x25
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB10_30
// %bb.29:                              //   in Loop: Header=BB10_5 Depth=1
	ldp	x9, x8, [x25]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB10_30:                              //   in Loop: Header=BB10_5 Depth=1
	ldr	x26, [sp, #48]
	add	x2, sp, #40             // =40
	mov	x0, x25
	mov	x1, x26
	bl	__list_add_valid
	tbz	w0, #0, .LBB10_4
// %bb.31:                              //   in Loop: Header=BB10_5 Depth=1
	add	x8, sp, #40             // =40
	str	x25, [sp, #48]
	stp	x8, x26, [x25]
	str	x25, [x26]
	b	.LBB10_4
.LBB10_32:                              //   in Loop: Header=BB10_5 Depth=1
	bl	preempt_schedule_notrace
	b	.LBB10_15
.LBB10_33:
	mov	x0, x26
	bl	unlock_page
	ldp	x20, x26, [sp, #8]      // 16-byte Folded Reload
	ldr	w8, [sp, #28]           // 4-byte Folded Reload
	tbz	w8, #0, .LBB10_35
.LBB10_34:
	mov	x0, x24
	mov	x1, x19
	mov	x2, xzr
	mov	w3, wzr
	mov	w4, wzr
	bl	f2fs_submit_merged_write_cond
.LBB10_35:
	cbz	w27, .LBB10_39
// %bb.36:
	add	x1, sp, #40             // =40
	mov	w3, #1
	mov	x0, x19
	mov	w2, wzr
	mov	w4, wzr
	bl	__revoke_inmem_pages
	mov	w24, w0
	mov	w2, #1
	mov	x0, x19
	mov	x1, x23
	mov	w3, wzr
	mov	w4, wzr
	bl	__revoke_inmem_pages
	b	.LBB10_40
.LBB10_37:
	ldp	x20, x26, [sp, #8]      // 16-byte Folded Reload
	ldr	w8, [sp, #28]           // 4-byte Folded Reload
	tbz	w8, #0, .LBB10_39
// %bb.38:
	mov	w27, wzr
	b	.LBB10_34
.LBB10_39:
	add	x1, sp, #40             // =40
	mov	x0, x19
	mov	w2, wzr
	mov	w3, wzr
	mov	w4, wzr
	bl	__revoke_inmem_pages
	mov	w24, wzr
.LBB10_40:
	mov	x0, x22
	bl	mutex_unlock
	add	x8, x19, #736           // =736
	mov	w9, #65536
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	x0, x20
	bl	up_read
	mov	x0, x26
	bl	up_write
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB10_43
// %bb.41:
	mov	w0, w24
	ldp	x20, x19, [sp, #288]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #272]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #256]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #240]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #224]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #208]    // 16-byte Folded Reload
	add	sp, sp, #304            // =304
	ret
.LBB10_42:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x21, #360]
	ldr	x2, [x20, :lo12:jiffies]
	adrp	x0, .L.str.14
	add	x0, x0, :lo12:.L.str.14
	bl	printk
	ldr	x0, [x21, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB10_2
.LBB10_43:
	bl	__stack_chk_fail
.Lfunc_end10:
	.size	f2fs_commit_inmem_pages, .Lfunc_end10-f2fs_commit_inmem_pages
                                        // -- End function
	.globl	f2fs_balance_fs         // -- Begin function f2fs_balance_fs
	.p2align	2
	.type	f2fs_balance_fs,@function
f2fs_balance_fs:                        // @f2fs_balance_fs
.Lf2fs_balance_fs$local:
// %bb.0:
	sub	sp, sp, #176            // =176
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	adrp	x9, autoremove_wake_function
	add	x10, sp, #24            // =24
	stp	x29, x30, [sp, #96]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #160]    // 16-byte Folded Spill
	add	x29, sp, #96            // =96
	mov	x19, x0
	add	x9, x9, :lo12:autoremove_wake_function
	add	x10, x10, #24           // =24
	stp	x26, x25, [sp, #112]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #128]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #144]    // 16-byte Folded Spill
	//APP
	mrs x22, sp_el0
	//NO_APP
	stur	x8, [x29, #-8]
	stp	xzr, x22, [sp, #24]
	stp	x9, x10, [sp, #40]
	str	x10, [sp, #56]
	tbz	w1, #0, .LBB11_3
// %bb.1:
	ldr	x8, [x19, #120]
	ldr	w8, [x8, #132]
	lsr	w8, w8, #5
	cmp	w8, #3125               // =3125
	b.lo	.LBB11_3
// %bb.2:
	mov	x0, x19
	mov	w1, wzr
	bl	f2fs_balance_fs_bg
.LBB11_3:
	ldr	x8, [x19, #80]
	tbnz	w8, #8, .LBB11_39
.LBB11_4:
	mov	x0, x19
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB11_9
// %bb.5:
	ldr	x8, [x19, #1712]
	cbz	x8, .LBB11_16
// %bb.6:
	adrp	x0, f2fs_balance_fs.fg_gc_rs
	adrp	x1, .L__func__.f2fs_balance_fs
	add	x0, x0, :lo12:f2fs_balance_fs.fg_gc_rs
	add	x1, x1, :lo12:.L__func__.f2fs_balance_fs
	bl	___ratelimit
	cbnz	w0, .LBB11_40
.LBB11_7:
	add	x20, x19, #1776         // =1776
	add	x1, sp, #24             // =24
	mov	w2, #2
	mov	x0, x20
	bl	prepare_to_wait
	ldr	x8, [x19, #1712]
	cbz	x8, .LBB11_15
// %bb.8:
	add	x0, x19, #1720          // =1720
	mov	w1, #3
	mov	w2, #1
	mov	x3, xzr
	bl	__wake_up
	bl	schedule
	add	x1, sp, #24             // =24
	mov	x0, x20
	bl	finish_wait
	b	.LBB11_36
.LBB11_9:
	ldr	w8, [x19, #1192]
	ldr	w13, [x19, #1188]
	ldrsw	x12, [x19, #1284]
	ldr	w9, [x19, #1184]
	ldrsw	x11, [x19, #1272]
	ldrsw	x10, [x19, #1296]
	ldr	w14, [x19, #1504]
	cmp	w14, #1                 // =1
	b.eq	.LBB11_36
// %bb.10:
	ldr	w14, [x19, #1988]
	cmp	w14, #5                 // =5
	b.ne	.LBB11_12
// %bb.11:
	ldrb	w14, [x19, #1232]
	tbz	w14, #3, .LBB11_20
.LBB11_12:
	ldr	x14, [x19, #80]
	tbnz	w14, #8, .LBB11_20
// %bb.13:
	mul	w13, w13, w8
	sub	x13, x13, #1            // =1
	add	x12, x13, x12
	add	x11, x13, x11
	add	x10, x13, x10
	ldr	x13, [x19, #136]
	asr	x12, x12, x9
	asr	x11, x11, x9
	asr	x9, x10, x9
	udiv	w10, w12, w8
	udiv	w12, w9, w8
	ldr	x9, [x13, #8]
	udiv	w11, w11, w8
	add	w10, w10, w11, lsl #1
	ldr	w11, [x13, #100]
	ldr	w13, [x13, #156]
	ldr	w9, [x9, #8]
	add	w10, w10, w12
	cmn	w11, #1                 // =1
	add	w10, w10, w13
	b.eq	.LBB11_18
// %bb.14:
	udiv	w8, w11, w8
	b	.LBB11_19
.LBB11_15:
	add	x1, sp, #24             // =24
	mov	x0, x20
	bl	finish_wait
.LBB11_16:
	add	x0, x19, #1664          // =1664
	bl	down_write
	ldr	w8, [x22, #60]
	mov	w4, #-1
	mov	x0, x19
	mov	w1, wzr
.LBB11_17:
	orr	w8, w8, #0x20000000
	str	w8, [x22, #60]
	mov	w2, wzr
	mov	w3, wzr
	bl	f2fs_gc
	ldr	w8, [x22, #60]
	and	w8, w8, #0xdfffffff
	str	w8, [x22, #60]
	b	.LBB11_36
.LBB11_18:
	mov	w8, #-1
.LBB11_19:
	add	w8, w10, w8
	cmp	w9, w8
	b.hi	.LBB11_36
.LBB11_20:
	ldr	x8, [x19, #136]
	adrp	x9, .L__const.f2fs_bd_balance_decision.ts
	add	x9, x9, :lo12:.L__const.f2fs_bd_balance_decision.ts
	ldp	x10, x9, [x9]
	ldr	x23, [x8, #16]
	sub	x0, x29, #24            // =24
	stur	wzr, [x29, #-28]
	stp	x10, x9, [x29, #-24]
	bl	timespec64_to_jiffies
	ldr	x24, [x19, #2328]
	add	x20, x19, #2296         // =2296
	mov	x21, x0
	mov	x0, x20
	bl	mutex_lock
	ldr	x26, [x24, #208]
	mov	x0, x20
	bl	mutex_unlock
	adrp	x25, jiffies
	ldr	x8, [x25, :lo12:jiffies]
	add	x9, x26, x21
	cmp	x9, x8
	b.pl	.LBB11_36
// %bb.21:
	mov	x0, x20
	bl	mutex_lock
	ldp	x8, x9, [x24, #192]
	ldp	x10, x11, [x24, #176]
	mov	x0, x20
	stp	x8, x9, [x24, #176]
	ldr	x12, [x25, :lo12:jiffies]
	sub	x21, x8, x10
	sub	x25, x9, x11
	str	x12, [x24, #208]
	bl	mutex_unlock
	ldr	w24, [x19, #1188]
	cmp	x21, x24
	b.gt	.LBB11_23
// %bb.22:
	cmp	x25, x24
	b.le	.LBB11_36
.LBB11_23:
	ldp	w8, w9, [x23, #112]
	ldp	w10, w11, [x23, #124]
	ldr	w12, [x23, #120]
	ldr	w13, [x23, #132]
	add	w8, w9, w8
	add	w9, w11, w10
	add	w26, w8, w12
	add	w23, w9, w13
	adds	w20, w23, w26
	b.eq	.LBB11_36
// %bb.24:
	ldr	x8, [x19, #136]
	cmp	x25, x24
	ldr	w8, [x8, #100]
	b.le	.LBB11_28
// %bb.25:
	cmp	w26, w8
	b.hs	.LBB11_28
// %bb.26:
	sub	x0, x29, #28            // =28
	mov	w1, #4
	bl	get_random_bytes
	ldur	w8, [x29, #-28]
	mov	w9, #34079
	movk	w9, #20971, lsl #16
	mov	w10, #100
	mul	x9, x8, x9
	mul	w11, w26, w10
	lsr	x9, x9, #37
	msub	w8, w9, w10, w8
	udiv	w9, w11, w20
	cmp	w8, w9
	b.hi	.LBB11_31
// %bb.27:
	ldr	x8, [x19, #136]
	ldr	w8, [x8, #100]
.LBB11_28:
	cmp	x21, x24
	b.le	.LBB11_36
// %bb.29:
	cmp	w23, w8
	b.hs	.LBB11_36
// %bb.30:
	sub	x0, x29, #28            // =28
	mov	w1, #4
	bl	get_random_bytes
	ldur	w8, [x29, #-28]
	mov	w9, #34079
	movk	w9, #20971, lsl #16
	mov	w10, #100
	mul	x9, x8, x9
	mul	w11, w23, w10
	lsr	x9, x9, #37
	msub	w8, w9, w10, w8
	udiv	w9, w11, w20
	cmp	w8, w9
	b.ls	.LBB11_36
.LBB11_31:
	adrp	x8, f2fs_balance_fs.FG_GC_count
	ldr	w9, [x8, :lo12:f2fs_balance_fs.FG_GC_count]
	ldr	w10, [x19, #1236]
	ldr	w11, [x19, #1168]
	ldr	w12, [x19, #1240]
	add	w4, w9, #1              // =1
	str	w4, [x8, :lo12:f2fs_balance_fs.FG_GC_count]
	ldr	x8, [x19, #136]
	sub	w9, w10, w12
	mul	x10, x11, x10
	mul	x11, x9, x11
	ldr	x12, [x8, #8]
	ldr	w9, [x8, #100]
	ldr	w8, [x19, #1192]
	lsr	x2, x10, #20
	ldr	w5, [x12, #8]
	cmn	w9, #1                  // =1
	lsr	x3, x11, #20
	b.eq	.LBB11_33
// %bb.32:
	udiv	w6, w9, w8
	b	.LBB11_34
.LBB11_33:
	mov	w6, #-1
.LBB11_34:
	ldr	w9, [x19, #1188]
	ldrsw	x10, [x19, #1284]
	ldr	w11, [x19, #1184]
	ldrsw	x12, [x19, #1272]
	ldrsw	x13, [x19, #1296]
	mul	w9, w9, w8
	sub	x9, x9, #1              // =1
	add	x10, x9, x10
	add	x12, x9, x12
	add	x9, x9, x13
	asr	x10, x10, x11
	asr	x12, x12, x11
	asr	x9, x9, x11
	adrp	x1, .L.str.3
	udiv	w7, w10, w8
	udiv	w10, w12, w8
	udiv	w8, w9, w8
	add	x1, x1, :lo12:.L.str.3
	mov	x0, x19
	str	w8, [sp, #8]
	str	w10, [sp]
	bl	f2fs_printk
	add	x8, x19, #2336          // =2336
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x19, #1712]
	cbz	x8, .LBB11_38
// %bb.35:
	add	x0, x19, #1720          // =1720
	mov	w1, #3
	mov	w2, #1
	mov	x3, xzr
	bl	__wake_up
.LBB11_36:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB11_44
// %bb.37:
	ldp	x20, x19, [sp, #160]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #144]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #128]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #112]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #96]     // 16-byte Folded Reload
	add	sp, sp, #176            // =176
	ret
.LBB11_38:
	add	x0, x19, #1664          // =1664
	bl	down_write
	ldr	w8, [x22, #60]
	mov	w1, #1
	mov	w4, #-1
	mov	x0, x19
	b	.LBB11_17
.LBB11_39:
	mov	x0, x19
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB11_4
	b	.LBB11_36
.LBB11_40:
	adrp	x8, f2fs_balance_fs.FG_GC_count
	ldr	w9, [x8, :lo12:f2fs_balance_fs.FG_GC_count]
	ldr	w10, [x19, #1236]
	ldr	w11, [x19, #1168]
	ldr	w12, [x19, #1240]
	add	w4, w9, #1              // =1
	str	w4, [x8, :lo12:f2fs_balance_fs.FG_GC_count]
	ldr	x8, [x19, #136]
	sub	w9, w10, w12
	mul	x10, x11, x10
	mul	x11, x9, x11
	ldr	x12, [x8, #8]
	ldr	w9, [x8, #100]
	ldr	w8, [x19, #1192]
	lsr	x2, x10, #20
	ldr	w5, [x12, #8]
	cmn	w9, #1                  // =1
	lsr	x3, x11, #20
	b.eq	.LBB11_42
// %bb.41:
	udiv	w6, w9, w8
	b	.LBB11_43
.LBB11_42:
	mov	w6, #-1
.LBB11_43:
	ldr	w9, [x19, #1188]
	ldrsw	x10, [x19, #1284]
	ldr	w11, [x19, #1184]
	ldrsw	x12, [x19, #1272]
	ldrsw	x13, [x19, #1296]
	mul	w9, w9, w8
	sub	x9, x9, #1              // =1
	add	x10, x9, x10
	add	x12, x9, x12
	add	x9, x9, x13
	asr	x10, x10, x11
	asr	x12, x12, x11
	asr	x9, x9, x11
	adrp	x1, .L.str.2
	udiv	w7, w10, w8
	udiv	w10, w12, w8
	udiv	w8, w9, w8
	add	x1, x1, :lo12:.L.str.2
	mov	x0, x19
	str	w8, [sp, #8]
	str	w10, [sp]
	bl	f2fs_printk
	b	.LBB11_7
.LBB11_44:
	bl	__stack_chk_fail
.Lfunc_end11:
	.size	f2fs_balance_fs, .Lfunc_end11-f2fs_balance_fs
                                        // -- End function
	.globl	f2fs_balance_fs_bg      // -- Begin function f2fs_balance_fs_bg
	.p2align	2
	.type	f2fs_balance_fs_bg,@function
f2fs_balance_fs_bg:                     // @f2fs_balance_fs_bg
.Lf2fs_balance_fs_bg$local:
// %bb.0:
	sub	sp, sp, #112            // =112
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #80]     // 16-byte Folded Spill
	add	x29, sp, #80            // =80
	stp	x20, x19, [sp, #96]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x8, [x0, #80]
	tbnz	w8, #3, .LBB12_15
// %bb.1:
	mov	w20, w1
	mov	w1, #4
	mov	x19, x0
	bl	f2fs_available_free_memory
	tbnz	w0, #0, .LBB12_3
// %bb.2:
	mov	w1, #128
	mov	x0, x19
	bl	f2fs_shrink_extent_tree
.LBB12_3:
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_available_free_memory
	tbnz	w0, #0, .LBB12_5
// %bb.4:
	mov	w1, #455
	mov	x0, x19
	bl	f2fs_try_to_free_nats
.LBB12_5:
	mov	x0, x19
	mov	w1, wzr
	bl	f2fs_available_free_memory
	tbz	w0, #0, .LBB12_7
// %bb.6:
	mov	x0, x19
	mov	w1, wzr
	mov	w2, wzr
	bl	f2fs_build_free_nids
	b	.LBB12_8
.LBB12_7:
	mov	w1, #3640
	mov	x0, x19
	bl	f2fs_try_to_free_nids
.LBB12_8:
	ldr	x8, [x19, #120]
	ldr	w9, [x8, #4]
	ldr	w10, [x8, #24]
	ldr	w8, [x8, #136]
	mul	w9, w10, w9
	mov	w10, #34079
	movk	w10, #20971, lsl #16
	umull	x9, w9, w10
	lsr	x9, x9, #37
	cmp	w8, w9
	b.hs	.LBB12_11
// %bb.9:
	ldrsw	x8, [x19, #1284]
	ldr	w9, [x19, #1188]
	lsl	w9, w9, #3
	cmp	x8, x9
	b.ge	.LBB12_11
// %bb.10:
	ldr	x8, [x19, #136]
	ldr	x9, [x8, #16]
	ldr	w10, [x8, #108]
	ldr	w9, [x9, #140]
	cmp	w9, w10
	b.ls	.LBB12_17
.LBB12_11:
	ldrb	w8, [x19, #1425]
	tbz	w8, #7, .LBB12_14
// %bb.12:
	tbz	w20, #0, .LBB12_14
// %bb.13:
	add	x20, x19, #1032         // =1032
	mov	x0, x20
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	bl	mutex_lock
	add	x0, sp, #8              // =8
	bl	blk_start_plug
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_sync_dirty_inodes
	add	x0, sp, #8              // =8
	bl	blk_finish_plug
	mov	x0, x20
	bl	mutex_unlock
.LBB12_14:
	ldr	x0, [x19]
	mov	w1, #1
	bl	f2fs_sync_fs
	ldr	x8, [x19, #2120]
	ldr	w9, [x8, #412]
	add	w9, w9, #1              // =1
	str	w9, [x8, #412]
.LBB12_15:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB12_34
// %bb.16:
	ldp	x20, x19, [sp, #96]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #80]     // 16-byte Folded Reload
	add	sp, sp, #112            // =112
	ret
.LBB12_17:
	ldr	w9, [x19, #1308]
	cbnz	w9, .LBB12_15
// %bb.18:
	ldr	w9, [x19, #1312]
	cbnz	w9, .LBB12_15
// %bb.19:
	ldr	w9, [x19, #1316]
	cbnz	w9, .LBB12_15
// %bb.20:
	ldr	w9, [x19, #1304]
	cbnz	w9, .LBB12_15
// %bb.21:
	ldr	w9, [x19, #1300]
	cbnz	w9, .LBB12_15
// %bb.22:
	ldr	w9, [x19, #1324]
	cbnz	w9, .LBB12_15
// %bb.23:
	ldr	w9, [x19, #1320]
	cbnz	w9, .LBB12_15
// %bb.24:
	cbz	x8, .LBB12_29
// %bb.25:
	ldr	x9, [x8, #168]
	cbz	x9, .LBB12_27
// %bb.26:
	ldr	w9, [x9, #8336]
	cbnz	w9, .LBB12_15
.LBB12_27:
	ldr	x8, [x8, #160]
	cbz	x8, .LBB12_29
// %bb.28:
	ldr	w8, [x8, #36]
	cbnz	w8, .LBB12_15
.LBB12_29:
	ldr	x9, [x19, #552]
	ldr	x10, [x19, #504]
	adrp	x8, jiffies
	ldr	x11, [x8, :lo12:jiffies]
	mov	w12, #250
	madd	x9, x9, x12, x10
	cmp	x9, x11
	b.mi	.LBB12_31
// %bb.30:
	ldr	x9, [x19, #304]
	cbnz	x9, .LBB12_15
.LBB12_31:
	ldr	x9, [x19, #544]
	ldr	x10, [x19, #496]
	ldr	x8, [x8, :lo12:jiffies]
	mov	w11, #250
	madd	x9, x9, x11, x10
	cmp	x9, x8
	b.mi	.LBB12_11
// %bb.32:
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_available_free_memory
	tbnz	w0, #0, .LBB12_15
// %bb.33:
	mov	w1, #3
	mov	x0, x19
	bl	f2fs_available_free_memory
	tbz	w0, #0, .LBB12_11
	b	.LBB12_15
.LBB12_34:
	bl	__stack_chk_fail
.Lfunc_end12:
	.size	f2fs_balance_fs_bg, .Lfunc_end12-f2fs_balance_fs_bg
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
	tbnz	w8, #3, .LBB13_11
// %bb.1:
	ldr	x1, [x0, #136]
	ldr	x8, [x1, #8]
	ldr	w12, [x1, #100]
	ldr	w8, [x8, #8]
	cmn	w12, #1                 // =1
	udiv	w10, w12, w9
	b.eq	.LBB13_3
// %bb.2:
	mov	w17, w10
	cmp	w8, w17
	b.eq	.LBB13_4
	b	.LBB13_8
.LBB13_3:
	mov	w17, #-1
	cmp	w8, w17
	b.ne	.LBB13_8
.LBB13_4:
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
	b.lo	.LBB13_8
// %bb.5:
	ldr	w3, [x0, #580]
	mov	w4, #40
	madd	x3, x3, x4, x1
	ldrh	w3, [x3, #2]
	and	w3, w3, #0x3ff
	sub	w3, w16, w3
	cmp	w3, w2
	b.lo	.LBB13_8
// %bb.6:
	ldr	w3, [x0, #700]
	mov	w4, #40
	madd	x3, x3, x4, x1
	ldrh	w3, [x3, #2]
	and	w3, w3, #0x3ff
	sub	w3, w16, w3
	cmp	w3, w2
	b.lo	.LBB13_8
// %bb.7:
	ldr	w0, [x0, #100]
	mov	w2, #40
	madd	x0, x0, x2, x1
	ldrh	w0, [x0, #2]
	and	w0, w0, #0x3ff
	sub	w0, w16, w0
	cmp	w0, w17
	b.hs	.LBB13_11
.LBB13_8:
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
	b.ne	.LBB13_10
// %bb.9:
	mov	w10, #-1
.LBB13_10:
	add	w9, w9, w10
	cmp	w8, w9
	cset	w0, ls
	ret
.LBB13_11:
	mov	w0, wzr
	ret
.Lfunc_end13:
	.size	has_not_enough_free_secs, .Lfunc_end13-has_not_enough_free_secs
                                        // -- End function
	.globl	f2fs_issue_flush        // -- Begin function f2fs_issue_flush
	.p2align	2
	.type	f2fs_issue_flush,@function
f2fs_issue_flush:                       // @f2fs_issue_flush
.Lf2fs_issue_flush$local:
// %bb.0:
	sub	sp, sp, #144            // =144
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #64]     // 16-byte Folded Spill
	add	x29, sp, #64            // =64
	str	x25, [sp, #80]          // 8-byte Folded Spill
	stp	x24, x23, [sp, #96]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #112]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #128]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x8, [x0, #136]
	ldr	x23, [x8, #160]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	ldr	w8, [x0, #1424]
	tbnz	w8, #11, .LBB14_7
// %bb.1:
	mov	x19, x0
	mov	w20, w1
	tbnz	w8, #10, .LBB14_8
// %bb.2:
	add	x8, x23, #36            // =36
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	w8, [x19, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB14_16
// %bb.3:
	mov	x22, xzr
	mov	x21, xzr
	b	.LBB14_5
.LBB14_4:                               //   in Loop: Header=BB14_5 Depth=1
	ldrsw	x8, [x19, #3128]
	add	x21, x21, #1            // =1
	add	x22, x22, #88           // =88
	cmp	x21, x8
	b.ge	.LBB14_18
.LBB14_5:                               // =>This Inner Loop Header: Depth=1
	mov	w3, #4
	mov	x0, x19
	mov	w1, w20
	mov	w2, w21
	bl	f2fs_is_dirty_device
	tbz	w0, #0, .LBB14_4
// %bb.6:                               //   in Loop: Header=BB14_5 Depth=1
	ldr	x8, [x19, #3136]
	mov	x0, x19
	ldr	x1, [x8, x22]
	bl	__submit_flush_wait
	cbz	w0, .LBB14_4
	b	.LBB14_19
.LBB14_7:
	mov	w0, wzr
	b	.LBB14_22
.LBB14_8:
	ldr	x8, [x19]
	ldr	x0, [x8, #248]
	bl	blk_flush_async_support
	cbnz	w0, .LBB14_11
// %bb.9:
	add	x8, x23, #36            // =36
	//APP
	// atomic_add_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cmp	w9, #1                  // =1
	b.eq	.LBB14_11
// %bb.10:
	ldr	w8, [x19, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB14_24
.LBB14_11:
	ldr	w8, [x19, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB14_17
// %bb.12:
	mov	x22, xzr
	mov	x21, xzr
	b	.LBB14_14
.LBB14_13:                              //   in Loop: Header=BB14_14 Depth=1
	ldrsw	x8, [x19, #3128]
	add	x21, x21, #1            // =1
	add	x22, x22, #88           // =88
	cmp	x21, x8
	b.ge	.LBB14_20
.LBB14_14:                              // =>This Inner Loop Header: Depth=1
	mov	w3, #4
	mov	x0, x19
	mov	w1, w20
	mov	w2, w21
	bl	f2fs_is_dirty_device
	tbz	w0, #0, .LBB14_13
// %bb.15:                              //   in Loop: Header=BB14_14 Depth=1
	ldr	x8, [x19, #3136]
	mov	x0, x19
	ldr	x1, [x8, x22]
	bl	__submit_flush_wait
	cbz	w0, .LBB14_13
	b	.LBB14_21
.LBB14_16:
	ldr	x8, [x19]
	mov	x0, x19
	ldr	x1, [x8, #248]
	bl	__submit_flush_wait
	b	.LBB14_19
.LBB14_17:
	ldr	x8, [x19]
	mov	x0, x19
	ldr	x1, [x8, #248]
	bl	__submit_flush_wait
	b	.LBB14_21
.LBB14_18:
	mov	w0, wzr
.LBB14_19:
	add	x8, x23, #36            // =36
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	add	x8, x23, #32            // =32
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB14_22
.LBB14_20:
	mov	w0, wzr
.LBB14_21:
	add	x8, x23, #36            // =36
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	add	x8, x23, #32            // =32
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB14_22:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB14_45
// %bb.23:
	ldp	x20, x19, [sp, #128]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #112]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #96]     // 16-byte Folded Reload
	ldr	x25, [sp, #80]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #64]     // 16-byte Folded Reload
	add	sp, sp, #144            // =144
	ret
.LBB14_24:
	add	x21, sp, #8             // =8
	adrp	x1, .L.str.15
	adrp	x2, __init_completion.__key
	add	x0, x21, #8             // =8
	add	x1, x1, :lo12:.L.str.15
	add	x2, x2, :lo12:__init_completion.__key
	str	w20, [sp, #48]
	str	wzr, [sp, #8]
	bl	__init_waitqueue_head
	add	x0, x21, #32            // =32
	add	x21, x23, #40           // =40
	mov	x1, x0
	mov	x2, x21
	bl	llist_add_batch
	//APP
	dmb ish
	//NO_APP
	mov	x8, x23
	ldr	x9, [x8, #16]!
	cmp	x8, x9
	b.eq	.LBB14_26
// %bb.25:
	add	x0, x23, #8             // =8
	mov	w1, #3
	mov	w2, #1
	mov	x3, xzr
	bl	__wake_up
.LBB14_26:
	ldr	x8, [x23]
	cbz	x8, .LBB14_29
// %bb.27:
	add	x0, sp, #8              // =8
	bl	wait_for_completion
	add	x8, x23, #36            // =36
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
.LBB14_28:
	ldr	w0, [sp, #52]
	b	.LBB14_22
.LBB14_29:
	//APP
		prfm	pstl1strm, [x21]
1:	ldxr	x24, [x21]
	stlxr	w9, x8, [x21]
	cbnz	w9, 1b
	dmb ish
	//NO_APP
	cbz	x24, .LBB14_36
// %bb.30:
	ldr	w8, [x19, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB14_37
// %bb.31:
	mov	x25, xzr
	mov	x22, xzr
	b	.LBB14_33
.LBB14_32:                              //   in Loop: Header=BB14_33 Depth=1
	ldrsw	x8, [x19, #3128]
	add	x22, x22, #1            // =1
	add	x25, x25, #88           // =88
	cmp	x22, x8
	b.ge	.LBB14_38
.LBB14_33:                              // =>This Inner Loop Header: Depth=1
	mov	w3, #4
	mov	x0, x19
	mov	w1, w20
	mov	w2, w22
	bl	f2fs_is_dirty_device
	tbz	w0, #0, .LBB14_32
// %bb.34:                              //   in Loop: Header=BB14_33 Depth=1
	ldr	x8, [x19, #3136]
	mov	x0, x19
	ldr	x1, [x8, x25]
	bl	__submit_flush_wait
	cbz	w0, .LBB14_32
// %bb.35:
	mov	w21, w0
	b	.LBB14_39
.LBB14_36:
	add	x0, sp, #8              // =8
	bl	wait_for_completion
	add	x8, x23, #36            // =36
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	b	.LBB14_28
.LBB14_37:
	ldr	x8, [x19]
	mov	x0, x19
	ldr	x1, [x8, #248]
	bl	__submit_flush_wait
	mov	w21, w0
	b	.LBB14_39
.LBB14_38:
	mov	w21, wzr
.LBB14_39:
	sub	x0, x24, #32            // =32
	cmn	x0, #32                 // =32
	b.eq	.LBB14_28
// %bb.40:
	add	x19, sp, #8             // =8
	mov	w20, #1
	b	.LBB14_43
.LBB14_41:                              //   in Loop: Header=BB14_43 Depth=1
	str	w21, [x0, #44]
	bl	complete
.LBB14_42:                              //   in Loop: Header=BB14_43 Depth=1
	cmn	x22, #32                // =32
	mov	x0, x22
	b.eq	.LBB14_28
.LBB14_43:                              // =>This Inner Loop Header: Depth=1
	ldr	x8, [x0, #32]
	cmp	x0, x19
	sub	x22, x8, #32            // =32
	b.ne	.LBB14_41
// %bb.44:                              //   in Loop: Header=BB14_43 Depth=1
	str	w21, [sp, #52]
	add	x8, x23, #36            // =36
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w20
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB14_42
.LBB14_45:
	bl	__stack_chk_fail
.Lfunc_end14:
	.size	f2fs_issue_flush, .Lfunc_end14-f2fs_issue_flush
                                        // -- End function
	.globl	f2fs_create_flush_cmd_control // -- Begin function f2fs_create_flush_cmd_control
	.p2align	2
	.type	f2fs_create_flush_cmd_control,@function
f2fs_create_flush_cmd_control:          // @f2fs_create_flush_cmd_control
.Lf2fs_create_flush_cmd_control$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0]
	ldr	x9, [x0, #136]
	mov	x19, x0
	mov	x29, sp
	ldr	x8, [x8, #248]
	ldr	x20, [x9, #160]
	ldr	w22, [x8]
	cbz	x20, .LBB15_3
// %bb.1:
	ldr	x8, [x20]
	cbz	x8, .LBB15_5
// %bb.2:
	mov	w21, wzr
	b	.LBB15_9
.LBB15_3:
	adrp	x8, kmalloc_caches+56
	ldr	x0, [x8, :lo12:kmalloc_caches+56]
	mov	w1, #3520
	mov	w2, #56
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB15_8
// %bb.4:
	adrp	x1, .L.str.4
	adrp	x2, f2fs_create_flush_cmd_control.__key
	mov	x20, x0
	str	wzr, [x0, #32]
	str	wzr, [x0, #36]
	add	x0, x0, #8              // =8
	add	x1, x1, :lo12:.L.str.4
	add	x2, x2, :lo12:f2fs_create_flush_cmd_control.__key
	bl	__init_waitqueue_head
	str	xzr, [x20, #40]
	ldr	x8, [x19, #136]
	mov	w21, wzr
	str	x20, [x8, #160]
	ldrb	w8, [x19, #1425]
	tbz	w8, #2, .LBB15_9
.LBB15_5:
	adrp	x0, issue_flush_thread
	adrp	x3, .L.str.5
	lsr	w4, w22, #20
	and	w5, w22, #0xfffff
	add	x0, x0, :lo12:issue_flush_thread
	add	x3, x3, :lo12:.L.str.5
	mov	w2, #-1
	mov	x1, x19
	bl	kthread_create_on_node
	mov	x21, x0
	cmn	x0, #1, lsl #12         // =4096
	b.ls	.LBB15_7
// %bb.6:
	mov	x0, x20
	str	x21, [x20]
	bl	kfree
	ldr	x8, [x19, #136]
	str	xzr, [x8, #160]
	b	.LBB15_9
.LBB15_7:
	mov	x0, x21
	bl	wake_up_process
	str	x21, [x20]
	mov	w21, wzr
	b	.LBB15_9
.LBB15_8:
	mov	w21, #-12
.LBB15_9:
	mov	w0, w21
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end15:
	.size	f2fs_create_flush_cmd_control, .Lfunc_end15-f2fs_create_flush_cmd_control
                                        // -- End function
	.p2align	2               // -- Begin function issue_flush_thread
	.type	issue_flush_thread,@function
issue_flush_thread:                     // @issue_flush_thread
// %bb.0:
	sub	sp, sp, #128            // =128
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	add	x29, sp, #48            // =48
	str	x25, [sp, #64]          // 8-byte Folded Spill
	stp	x24, x23, [sp, #80]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #96]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #112]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x8, [x0, #136]
	mov	x19, x0
	ldr	x24, [x8, #160]
	bl	kthread_should_stop
	tbz	w0, #0, .LBB16_3
.LBB16_1:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB16_25
// %bb.2:
	ldp	x20, x19, [sp, #112]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #96]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #80]     // 16-byte Folded Reload
	ldr	x25, [sp, #64]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	mov	w0, wzr
	add	sp, sp, #128            // =128
	ret
.LBB16_3:
	add	x20, x24, #8            // =8
	b	.LBB16_6
.LBB16_4:                               //   in Loop: Header=BB16_6 Depth=1
	mov	x1, sp
	mov	x0, x20
	bl	finish_wait
.LBB16_5:                               //   in Loop: Header=BB16_6 Depth=1
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB16_1
.LBB16_6:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB16_10 Depth 2
                                        //     Child Loop BB16_16 Depth 2
                                        //     Child Loop BB16_21 Depth 2
	ldr	x8, [x24, #40]
	cbz	x8, .LBB16_18
// %bb.7:                               //   in Loop: Header=BB16_6 Depth=1
	mov	x8, xzr
	add	x9, x24, #40            // =40
	//APP
		prfm	pstl1strm, [x9]
1:	ldxr	x0, [x9]
	stlxr	w10, x8, [x9]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	str	x0, [x24, #48]
	bl	llist_reverse_order
	str	x0, [x24, #48]
	ldr	w8, [x19, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB16_13
// %bb.8:                               //   in Loop: Header=BB16_6 Depth=1
	ldr	w22, [x0, #8]
	mov	x25, xzr
	mov	x23, xzr
	b	.LBB16_10
.LBB16_9:                               //   in Loop: Header=BB16_10 Depth=2
	ldrsw	x8, [x19, #3128]
	add	x23, x23, #1            // =1
	add	x25, x25, #88           // =88
	cmp	x23, x8
	b.ge	.LBB16_14
.LBB16_10:                              //   Parent Loop BB16_6 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	mov	w3, #4
	mov	x0, x19
	mov	w1, w22
	mov	w2, w23
	bl	f2fs_is_dirty_device
	tbz	w0, #0, .LBB16_9
// %bb.11:                              //   in Loop: Header=BB16_10 Depth=2
	ldr	x8, [x19, #3136]
	mov	x0, x19
	ldr	x1, [x8, x25]
	bl	__submit_flush_wait
	cbz	w0, .LBB16_9
// %bb.12:                              //   in Loop: Header=BB16_6 Depth=1
	mov	w21, w0
	b	.LBB16_15
.LBB16_13:                              //   in Loop: Header=BB16_6 Depth=1
	ldr	x8, [x19]
	mov	x0, x19
	ldr	x1, [x8, #248]
	bl	__submit_flush_wait
	mov	w21, w0
	b	.LBB16_15
.LBB16_14:                              //   in Loop: Header=BB16_6 Depth=1
	mov	w21, wzr
.LBB16_15:                              //   in Loop: Header=BB16_6 Depth=1
	add	x8, x24, #32            // =32
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x24, #48]
	sub	x0, x8, #32             // =32
	cmn	x0, #32                 // =32
	b.eq	.LBB16_17
.LBB16_16:                              //   Parent Loop BB16_6 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	x22, [x8]
	str	w21, [x8, #12]
	bl	complete
	sub	x0, x22, #32            // =32
	cmn	x0, #32                 // =32
	mov	x8, x22
	b.ne	.LBB16_16
.LBB16_17:                              //   in Loop: Header=BB16_6 Depth=1
	str	xzr, [x24, #48]
.LBB16_18:                              //   in Loop: Header=BB16_6 Depth=1
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB16_5
// %bb.19:                              //   in Loop: Header=BB16_6 Depth=1
	ldr	x8, [x24, #40]
	cbnz	x8, .LBB16_5
// %bb.20:                              //   in Loop: Header=BB16_6 Depth=1
	mov	x0, sp
	mov	w1, wzr
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	str	xzr, [sp]
	bl	init_wait_entry
.LBB16_21:                              //   Parent Loop BB16_6 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	mov	x1, sp
	mov	w2, #1
	mov	x0, x20
	bl	prepare_to_wait_event
	mov	x21, x0
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB16_4
// %bb.22:                              //   in Loop: Header=BB16_21 Depth=2
	ldr	x8, [x24, #40]
	cbnz	x8, .LBB16_4
// %bb.23:                              //   in Loop: Header=BB16_21 Depth=2
	cbnz	x21, .LBB16_5
// %bb.24:                              //   in Loop: Header=BB16_21 Depth=2
	bl	schedule
	b	.LBB16_21
.LBB16_25:
	bl	__stack_chk_fail
.Lfunc_end16:
	.size	issue_flush_thread, .Lfunc_end16-issue_flush_thread
                                        // -- End function
	.globl	f2fs_destroy_flush_cmd_control // -- Begin function f2fs_destroy_flush_cmd_control
	.p2align	2
	.type	f2fs_destroy_flush_cmd_control,@function
f2fs_destroy_flush_cmd_control:         // @f2fs_destroy_flush_cmd_control
.Lf2fs_destroy_flush_cmd_control$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x0
	mov	w21, w1
	ldr	x20, [x8, #160]
	mov	x29, sp
	cbz	x20, .LBB17_3
// %bb.1:
	ldr	x0, [x20]
	cbz	x0, .LBB17_3
// %bb.2:
	str	xzr, [x20]
	bl	kthread_stop
.LBB17_3:
	tbz	w21, #0, .LBB17_5
// %bb.4:
	mov	x0, x20
	bl	kfree
	ldr	x8, [x19, #136]
	str	xzr, [x8, #160]
.LBB17_5:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end17:
	.size	f2fs_destroy_flush_cmd_control, .Lfunc_end17-f2fs_destroy_flush_cmd_control
                                        // -- End function
	.globl	f2fs_flush_device_cache // -- Begin function f2fs_flush_device_cache
	.p2align	2
	.type	f2fs_flush_device_cache,@function
f2fs_flush_device_cache:                // @f2fs_flush_device_cache
.Lf2fs_flush_device_cache$local:
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x26, x25, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	w8, [x0, #3128]
	mov	x29, sp
	cbz	w8, .LBB18_8
// %bb.1:
	ldrb	w9, [x0, #1425]
	mov	x19, x0
	tbnz	w9, #3, .LBB18_8
// %bb.2:
	cmp	w8, #2                  // =2
	b.lt	.LBB18_8
// %bb.3:
	add	x21, x19, #3144         // =3144
	add	x20, x19, #3148         // =3148
	mov	w22, #1
	mov	w23, #88
	mov	w24, #1
	b	.LBB18_6
.LBB18_4:                               //   in Loop: Header=BB18_6 Depth=1
	mov	x0, x20
	bl	_raw_spin_lock
	ldrb	w8, [x21, x25]
	mov	x0, x20
	bic	w8, w8, w26
	strb	w8, [x21, x25]
	bl	_raw_spin_unlock
	ldr	w8, [x19, #3128]
.LBB18_5:                               //   in Loop: Header=BB18_6 Depth=1
	add	x24, x24, #1            // =1
	cmp	x24, w8, sxtw
	add	x23, x23, #88           // =88
	b.ge	.LBB18_8
.LBB18_6:                               // =>This Inner Loop Header: Depth=1
	ubfx	x25, x24, #3, #29
	ldrb	w9, [x21, x25]
	mvn	w10, w24
	and	w10, w10, #0x7
	lsl	w26, w22, w10
	tst	w26, w9
	b.eq	.LBB18_5
// %bb.7:                               //   in Loop: Header=BB18_6 Depth=1
	ldr	x8, [x19, #3136]
	mov	x0, x19
	ldr	x1, [x8, x23]
	bl	__submit_flush_wait
	cbz	w0, .LBB18_4
	b	.LBB18_9
.LBB18_8:
	mov	w0, wzr
.LBB18_9:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.Lfunc_end18:
	.size	f2fs_flush_device_cache, .Lfunc_end18-f2fs_flush_device_cache
                                        // -- End function
	.p2align	2               // -- Begin function __submit_flush_wait
	.type	__submit_flush_wait,@function
__submit_flush_wait:                    // @__submit_flush_wait
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	mov	x19, x1
	mov	w2, #1
	mov	w1, wzr
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	mov	x21, x0
	bl	f2fs_bio_alloc
	cbz	x0, .LBB19_4
// %bb.1:
	adrp	x8, f2fs_print_frag_info
	mov	w9, #2049
	add	x8, x8, :lo12:f2fs_print_frag_info
	movk	w9, #8, lsl #16
	str	x8, [x0, #344]
	str	w9, [x0, #16]
	ldr	x8, [x0, #8]
	ldr	x9, [x19, #128]
	mov	x22, x0
	cmp	x8, x9
	b.eq	.LBB19_3
// %bb.2:
	ldrh	w8, [x22, #20]
	and	w8, w8, #0xfffffdff
	strh	w8, [x22, #20]
	ldr	x8, [x19, #128]
.LBB19_3:
	str	x8, [x22, #8]
	ldrb	w8, [x19, #108]
	mov	x0, x22
	strb	w8, [x22, #27]
	bl	bio_associate_blkg
	mov	x0, x22
	bl	blk_flush_set_async
	mov	x0, x22
	bl	submit_bio_wait
	mov	w20, w0
	mov	x0, x22
	bl	bio_put
	b	.LBB19_5
.LBB19_4:
	mov	w20, #-12
.LBB19_5:
	adrp	x9, __tracepoint_f2fs_issue_flush+8
	ldr	w8, [x21, #1424]
	ldr	w9, [x9, :lo12:__tracepoint_f2fs_issue_flush+8]
	cmp	w9, #1                  // =1
	b.lt	.LBB19_12
// %bb.6:
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
	tbz	w9, #0, .LBB19_12
// %bb.7:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w9, [x23, #24]
	add	w9, w9, #1              // =1
	str	w9, [x23, #24]
	//APP
	//NO_APP
	adrp	x9, __tracepoint_f2fs_issue_flush+32
	ldr	x24, [x9, :lo12:__tracepoint_f2fs_issue_flush+32]
	cbz	x24, .LBB19_10
// %bb.8:
	and	w21, w8, #0x800
	and	w22, w8, #0x400
.LBB19_9:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x24]
	mov	x1, x19
	mov	w2, w21
	mov	w3, w22
	mov	w4, w20
	blr	x8
	ldr	x8, [x24, #24]!
	cbnz	x8, .LBB19_9
.LBB19_10:
	//APP
	//NO_APP
	ldr	x8, [x23, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x23, #24]
	b.eq	.LBB19_13
// %bb.11:
	ldr	x8, [x23, #24]
	cbz	x8, .LBB19_13
.LBB19_12:
	mov	w0, w20
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB19_13:
	bl	preempt_schedule_notrace
	b	.LBB19_12
.Lfunc_end19:
	.size	__submit_flush_wait, .Lfunc_end19-__submit_flush_wait
                                        // -- End function
	.globl	f2fs_dirty_to_prefree   // -- Begin function f2fs_dirty_to_prefree
	.p2align	2
	.type	f2fs_dirty_to_prefree,@function
f2fs_dirty_to_prefree:                  // @f2fs_dirty_to_prefree
.Lf2fs_dirty_to_prefree$local:
// %bb.0:
	sub	sp, sp, #80             // =80
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	x19, x0
	add	x29, sp, #16            // =16
	ldr	x22, [x8, #16]
	add	x20, x22, #80           // =80
	mov	x0, x20
	bl	mutex_lock
	ldr	x8, [x19, #136]
	ldr	x0, [x22, #56]
	mov	x2, xzr
	ldr	w1, [x8, #96]
	bl	find_next_bit
	ldr	x8, [x19, #136]
	mov	x21, x0
	ldr	w9, [x8, #96]
	cmp	w9, w21
	b.ls	.LBB20_16
// %bb.1:
	mov	w23, #40
	mov	w24, #1
	b	.LBB20_4
.LBB20_2:                               //   in Loop: Header=BB20_4 Depth=1
	mov	w2, #6
	mov	x0, x19
	mov	w1, w21
	bl	__remove_dirty_segment
	ldr	x8, [x19, #136]
.LBB20_3:                               //   in Loop: Header=BB20_4 Depth=1
	ldr	x0, [x22, #56]
	ldr	w1, [x8, #96]
	add	w2, w21, #1             // =1
	bl	find_next_bit
	ldr	x8, [x19, #136]
	mov	x21, x0
	ldr	w9, [x8, #96]
	cmp	w9, w21
	b.ls	.LBB20_16
.LBB20_4:                               // =>This Inner Loop Header: Depth=1
	ldr	x9, [x8]
	and	x10, x21, #0xffffffff
	mul	x10, x10, x23
	ldr	x9, [x9, #120]
	ldrh	w9, [x9, x10]
	tst	w9, #0xffc0
	b.ne	.LBB20_3
// %bb.5:                               //   in Loop: Header=BB20_4 Depth=1
	ldr	x9, [x8, #24]
	ldr	w10, [x9, #100]
	cmp	w10, w21
	b.eq	.LBB20_3
// %bb.6:                               //   in Loop: Header=BB20_4 Depth=1
	ldr	w10, [x9, #220]
	cmp	w10, w21
	b.eq	.LBB20_3
// %bb.7:                               //   in Loop: Header=BB20_4 Depth=1
	ldr	w10, [x9, #340]
	cmp	w10, w21
	b.eq	.LBB20_3
// %bb.8:                               //   in Loop: Header=BB20_4 Depth=1
	ldr	w10, [x9, #460]
	cmp	w10, w21
	b.eq	.LBB20_3
// %bb.9:                               //   in Loop: Header=BB20_4 Depth=1
	ldr	w10, [x9, #580]
	cmp	w10, w21
	b.eq	.LBB20_3
// %bb.10:                              //   in Loop: Header=BB20_4 Depth=1
	ldr	w10, [x9, #700]
	cmp	w10, w21
	b.eq	.LBB20_3
// %bb.11:                              //   in Loop: Header=BB20_4 Depth=1
	ldr	w10, [x9, #820]
	cmp	w10, w21
	b.eq	.LBB20_3
// %bb.12:                              //   in Loop: Header=BB20_4 Depth=1
	ldr	w9, [x9, #940]
	cmp	w9, w21
	b.eq	.LBB20_3
// %bb.13:                              //   in Loop: Header=BB20_4 Depth=1
	ldr	x8, [x8, #16]
	ubfx	x11, x21, #6, #26
	ldr	x9, [x8, #64]
	ldr	x10, [x9, x11, lsl #3]
	str	x10, [sp, #8]
	ldr	x12, [sp, #8]
	lsl	x10, x24, x21
	tst	x12, x10
	b.ne	.LBB20_2
// %bb.14:                              //   in Loop: Header=BB20_4 Depth=1
	add	x9, x9, x11, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x9]
1:	ldxr	x11, [x9]
	orr	x12, x11, x10
	stlxr	w13, x12, [x9]
	cbnz	w13, 1b
	dmb ish
	//NO_APP
	tst	x11, x10
	b.ne	.LBB20_2
// %bb.15:                              //   in Loop: Header=BB20_4 Depth=1
	ldr	w9, [x8, #140]
	add	w9, w9, #1              // =1
	str	w9, [x8, #140]
	b	.LBB20_2
.LBB20_16:
	mov	x0, x20
	bl	mutex_unlock
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #80             // =80
	ret
.Lfunc_end20:
	.size	f2fs_dirty_to_prefree, .Lfunc_end20-f2fs_dirty_to_prefree
                                        // -- End function
	.p2align	2               // -- Begin function __locate_dirty_segment
	.type	__locate_dirty_segment,@function
__locate_dirty_segment:                 // @__locate_dirty_segment
// %bb.0:
	sub	sp, sp, #64             // =64
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
                                        // kill: def $w1 killed $w1 def $x1
	add	x29, sp, #32            // =32
	ldr	x9, [x8, #24]
	ldr	w10, [x9, #100]
	cmp	w10, w1
	b.eq	.LBB21_31
// %bb.1:
	ldr	w10, [x9, #220]
	cmp	w10, w1
	b.eq	.LBB21_31
// %bb.2:
	ldr	w10, [x9, #340]
	cmp	w10, w1
	b.eq	.LBB21_31
// %bb.3:
	ldr	w10, [x9, #460]
	cmp	w10, w1
	b.eq	.LBB21_31
// %bb.4:
	ldr	w10, [x9, #580]
	cmp	w10, w1
	b.eq	.LBB21_31
// %bb.5:
	ldr	w10, [x9, #700]
	cmp	w10, w1
	b.eq	.LBB21_31
// %bb.6:
	ldr	w10, [x9, #820]
	cmp	w10, w1
	b.eq	.LBB21_31
// %bb.7:
	ldr	w9, [x9, #940]
	cmp	w9, w1
	b.eq	.LBB21_31
// %bb.8:
	ldr	x19, [x8, #16]
	add	x8, x19, w2, uxtw #3
	ldr	x10, [x8, #8]
	lsr	w8, w1, #6
	ldr	x9, [x10, w8, uxtw #3]
	stur	x9, [x29, #-8]
	ldur	x11, [x29, #-8]
	mov	w9, #1
	lsl	x9, x9, x1
	tst	x11, x9
	b.ne	.LBB21_11
// %bb.9:
	add	x10, x10, x8, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x10]
1:	ldxr	x11, [x10]
	orr	x12, x11, x9
	stlxr	w13, x12, [x10]
	cbnz	w13, 1b
	dmb ish
	//NO_APP
	tst	x11, x9
	b.ne	.LBB21_11
// %bb.10:
	mov	w10, w2
	add	x10, x19, x10, lsl #2
	ldr	w11, [x10, #112]
	add	w11, w11, #1            // =1
	str	w11, [x10, #112]
.LBB21_11:
	cmp	w2, #6                  // =6
	b.ne	.LBB21_31
// %bb.12:
	ldr	x10, [x0, #136]
	mov	w11, #40
	umull	x11, w1, w11
	ldr	x10, [x10]
	ldr	x10, [x10, #120]
	ldr	w10, [x10, x11]
	and	x10, x10, #0x3f
	cmp	w10, #6                 // =6
	b.hs	.LBB21_32
// %bb.13:
	add	x11, x19, x10, lsl #3
	ldr	x11, [x11, #8]
	ldr	x12, [x11, x8, lsl #3]
	stur	x12, [x29, #-8]
	ldur	x12, [x29, #-8]
	tst	x12, x9
	b.ne	.LBB21_16
// %bb.14:
	add	x8, x11, x8, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x8]
1:	ldxr	x11, [x8]
	orr	x12, x11, x9
	stlxr	w13, x12, [x8]
	cbnz	w13, 1b
	dmb ish
	//NO_APP
	tst	x11, x9
	b.ne	.LBB21_16
// %bb.15:
	add	x8, x19, x10, lsl #2
	ldr	w9, [x8, #112]
	add	w9, w9, #1              // =1
	str	w9, [x8, #112]
.LBB21_16:
	ldr	w9, [x0, #1192]
	cmp	w9, #2                  // =2
	b.lo	.LBB21_31
// %bb.17:
	cmn	w1, #1                  // =1
	b.eq	.LBB21_19
// %bb.18:
	udiv	w8, w1, w9
	mov	x10, x8
	b	.LBB21_20
.LBB21_19:
	mov	w8, #-1
	mov	w10, #-1
.LBB21_20:
	ldr	x11, [x0, #136]
	ldr	x11, [x11]
	ldr	x11, [x11, #128]
	ldr	w10, [x11, x10, lsl #2]
	cbz	w10, .LBB21_40
// %bb.21:
	ldr	w11, [x0, #1188]
	mul	w9, w11, w9
	cmp	w10, w9
	b.eq	.LBB21_40
.LBB21_22:
	ldr	x9, [x0, #136]
	ldr	w10, [x0, #1192]
	ldr	x9, [x9, #24]
	ldr	w11, [x9, #100]
	udiv	w11, w11, w10
	cmp	w8, w11
	b.eq	.LBB21_31
// %bb.23:
	ldr	w11, [x9, #220]
	udiv	w11, w11, w10
	cmp	w8, w11
	b.eq	.LBB21_31
// %bb.24:
	ldr	w11, [x9, #340]
	udiv	w11, w11, w10
	cmp	w8, w11
	b.eq	.LBB21_31
// %bb.25:
	ldr	w11, [x9, #460]
	udiv	w11, w11, w10
	cmp	w8, w11
	b.eq	.LBB21_31
// %bb.26:
	ldr	w11, [x9, #580]
	udiv	w11, w11, w10
	cmp	w8, w11
	b.eq	.LBB21_31
// %bb.27:
	ldr	w11, [x9, #700]
	udiv	w11, w11, w10
	cmp	w8, w11
	b.eq	.LBB21_31
// %bb.28:
	ldr	w11, [x9, #820]
	udiv	w11, w11, w10
	cmp	w8, w11
	b.eq	.LBB21_31
// %bb.29:
	ldr	w9, [x9, #940]
	udiv	w9, w9, w10
	cmp	w8, w9
	b.eq	.LBB21_31
// %bb.30:
	ldr	x9, [x19, #72]
	lsr	w10, w8, #6
	mov	w11, #1
	lsl	x8, x11, x8
	add	x9, x9, w10, uxtw #3
	//APP
	// atomic64_or
	prfm	pstl1strm, [x9]
1:	ldxr	x10, [x9]
	orr	x10, x10, x8
	stxr	w11, x10, [x9]
	cbnz	w11, 1b
	//NO_APP
.LBB21_31:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #64             // =64
	ret
.LBB21_32:
	mov	x19, x0
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB21_35
// %bb.33:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB21_35
// %bb.34:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__locate_dirty_segment
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__locate_dirty_segment
	mov	w3, #1401
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB21_35:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 1401; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB21_31
// %bb.36:
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
	cbnz	w8, .LBB21_31
// %bb.37:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB21_31
// %bb.38:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB21_31
// %bb.39:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB21_31
.LBB21_40:
	stp	x19, x8, [sp]           // 16-byte Folded Spill
	str	x0, [sp, #16]           // 8-byte Folded Spill
	bl	print_bdev_access_info
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	cbz	x0, .LBB21_43
// %bb.41:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB21_43
// %bb.42:
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__locate_dirty_segment
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__locate_dirty_segment
	mov	w3, #1413
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB21_43:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	mov	x0, x19
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 1413; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldp	x19, x8, [sp]           // 16-byte Folded Reload
	cbz	x0, .LBB21_22
// %bb.44:
	add	x9, x0, #80             // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x9]
1:	ldxr	x10, [x9]
	orr	x10, x10, 4
	stxr	w11, x10, [x9]
	cbnz	w11, 1b
	//NO_APP
	ldrb	w9, [x0, #3376]
	cbnz	w9, .LBB21_22
// %bb.45:
	ldr	x1, [x0, #3392]
	cbz	x1, .LBB21_22
// %bb.46:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB21_22
// %bb.47:
	add	x2, x0, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	ldp	x8, x0, [sp, #8]        // 16-byte Folded Reload
	b	.LBB21_22
.Lfunc_end21:
	.size	__locate_dirty_segment, .Lfunc_end21-__locate_dirty_segment
                                        // -- End function
	.p2align	2               // -- Begin function __remove_dirty_segment
	.type	__remove_dirty_segment,@function
__remove_dirty_segment:                 // @__remove_dirty_segment
// %bb.0:
	sub	sp, sp, #16             // =16
	ldr	x8, [x0, #136]
                                        // kill: def $w1 killed $w1 def $x1
	ldr	x8, [x8, #16]
	add	x9, x8, w2, uxtw #3
	ldr	x11, [x9, #8]
	lsr	w9, w1, #6
	ldr	x10, [x11, w9, uxtw #3]
	str	x10, [sp, #8]
	ldr	x12, [sp, #8]
	mov	w10, #1
	lsl	x10, x10, x1
	tst	x12, x10
	b.eq	.LBB22_3
// %bb.1:
	add	x11, x11, x9, lsl #3
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x11]
1:	ldxr	x12, [x11]
	bic	x13, x12, x10
	stlxr	w14, x13, [x11]
	cbnz	w14, 1b
	dmb ish
	//NO_APP
	tst	x12, x10
	b.eq	.LBB22_3
// %bb.2:
	mov	w11, w2
	add	x11, x8, x11, lsl #2
	ldr	w12, [x11, #112]
	sub	w12, w12, #1            // =1
	str	w12, [x11, #112]
.LBB22_3:
	cmp	w2, #6                  // =6
	b.ne	.LBB22_12
// %bb.4:
	ldr	x11, [x0, #136]
	mov	w12, #40
	umull	x12, w1, w12
	ldr	x11, [x11]
	ldr	x11, [x11, #120]
	ldr	w11, [x11, x12]
	and	x11, x11, #0x3f
	add	x12, x8, x11, lsl #3
	ldr	x12, [x12, #8]
	ldr	x13, [x12, x9, lsl #3]
	str	x13, [sp, #8]
	ldr	x13, [sp, #8]
	tst	x13, x10
	b.eq	.LBB22_7
// %bb.5:
	add	x9, x12, x9, lsl #3
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x9]
1:	ldxr	x12, [x9]
	bic	x13, x12, x10
	stlxr	w14, x13, [x9]
	cbnz	w14, 1b
	dmb ish
	//NO_APP
	tst	x12, x10
	b.eq	.LBB22_7
// %bb.6:
	add	x9, x8, x11, lsl #2
	ldr	w10, [x9, #112]
	sub	w10, w10, #1            // =1
	str	w10, [x9, #112]
.LBB22_7:
	ldr	x10, [x0, #136]
	ldr	w9, [x0, #1192]
	ldr	x10, [x10]
	cmp	w9, #2                  // =2
	b.lo	.LBB22_10
// %bb.8:
	ldr	x10, [x10, #128]
	cmn	w1, #1                  // =1
	b.eq	.LBB22_13
// %bb.9:
	udiv	w11, w1, w9
	ldr	w11, [x10, x11, lsl #2]
	cbnz	w11, .LBB22_11
	b	.LBB22_14
.LBB22_10:
	ldr	x10, [x10, #120]
	mov	w11, w1
	mov	w12, #40
	mul	x11, x11, x12
	ldr	w10, [x10, x11]
	ubfx	w11, w10, #6, #10
	cbz	w11, .LBB22_14
.LBB22_11:
	cmp	w9, #2                  // =2
	b.hs	.LBB22_18
.LBB22_12:
	add	sp, sp, #16             // =16
	ret
.LBB22_13:
	mov	w11, #-1
	ldr	w11, [x10, x11, lsl #2]
	cbnz	w11, .LBB22_11
.LBB22_14:
	cmn	w1, #1                  // =1
	b.eq	.LBB22_16
// %bb.15:
	udiv	w9, w1, w9
	b	.LBB22_17
.LBB22_16:
	mov	w9, #-1
.LBB22_17:
	ldr	x10, [x8, #144]
	lsr	w12, w9, #6
	mov	w13, #1
	lsl	x9, x13, x9
	add	x10, x10, w12, uxtw #3
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x10]
1:	ldxr	x12, [x10]
	bic	x12, x12, x9
	stxr	w13, x12, [x10]
	cbnz	w13, 1b
	//NO_APP
	ldr	w9, [x0, #1192]
	cmp	w9, #2                  // =2
	b.lo	.LBB22_12
.LBB22_18:
	cmn	w1, #1                  // =1
	b.eq	.LBB22_20
// %bb.19:
	udiv	w10, w1, w9
	cbnz	w11, .LBB22_21
	b	.LBB22_22
.LBB22_20:
	mov	w10, #-1
	cbz	w11, .LBB22_22
.LBB22_21:
	ldr	w12, [x0, #1188]
	mul	w12, w12, w9
	cmp	w11, w12
	b.ne	.LBB22_23
.LBB22_22:
	ldr	x8, [x8, #72]
	lsr	w9, w10, #6
	mov	w11, #1
	add	x8, x8, w9, uxtw #3
	lsl	x9, x11, x10
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	add	sp, sp, #16             // =16
	ret
.LBB22_23:
	ldr	x11, [x0, #136]
	ldr	x11, [x11, #24]
	ldr	w12, [x11, #100]
	udiv	w12, w12, w9
	cmp	w10, w12
	b.eq	.LBB22_12
// %bb.24:
	ldr	w12, [x11, #220]
	udiv	w12, w12, w9
	cmp	w10, w12
	b.eq	.LBB22_12
// %bb.25:
	ldr	w12, [x11, #340]
	udiv	w12, w12, w9
	cmp	w10, w12
	b.eq	.LBB22_12
// %bb.26:
	ldr	w12, [x11, #460]
	udiv	w12, w12, w9
	cmp	w10, w12
	b.eq	.LBB22_12
// %bb.27:
	ldr	w12, [x11, #580]
	udiv	w12, w12, w9
	cmp	w10, w12
	b.eq	.LBB22_12
// %bb.28:
	ldr	w12, [x11, #700]
	udiv	w12, w12, w9
	cmp	w10, w12
	b.eq	.LBB22_12
// %bb.29:
	ldr	w12, [x11, #820]
	udiv	w12, w12, w9
	cmp	w10, w12
	b.eq	.LBB22_12
// %bb.30:
	ldr	w11, [x11, #940]
	udiv	w9, w11, w9
	cmp	w10, w9
	b.eq	.LBB22_12
// %bb.31:
	ldr	x8, [x8, #72]
	lsr	w9, w10, #6
	mov	w11, #1
	add	x8, x8, w9, uxtw #3
	lsl	x9, x11, x10
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	orr	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	add	sp, sp, #16             // =16
	ret
.Lfunc_end22:
	.size	__remove_dirty_segment, .Lfunc_end22-__remove_dirty_segment
                                        // -- End function
	.globl	f2fs_get_unusable_blocks // -- Begin function f2fs_get_unusable_blocks
	.p2align	2
	.type	f2fs_get_unusable_blocks,@function
f2fs_get_unusable_blocks:               // @f2fs_get_unusable_blocks
.Lf2fs_get_unusable_blocks$local:
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	str	x25, [sp, #16]          // 8-byte Folded Spill
	ldr	w11, [x0, #1184]
	mov	x20, x0
	ldr	x22, [x8, #16]
	ldp	w10, w9, [x8, #100]
	mov	x29, sp
	add	x19, x22, #80           // =80
	sub	w8, w9, w10
	mov	x0, x19
	lsl	w21, w8, w11
	bl	mutex_lock
	ldr	x8, [x20, #136]
	ldr	x0, [x22, #56]
	mov	x2, xzr
	ldr	w1, [x8, #96]
	bl	find_next_bit
	ldr	x8, [x20, #136]
	ldr	w9, [x8, #96]
	cmp	w9, w0
	b.ls	.LBB23_3
// %bb.1:
	mov	w24, wzr
	mov	w23, wzr
	mov	w25, #40
.LBB23_2:                               // =>This Inner Loop Header: Depth=1
	ldr	x8, [x8]
	and	x10, x0, #0xffffffff
	mul	x10, x10, x25
	mov	w1, w9
	ldr	x8, [x8, #120]
	add	w2, w0, #1              // =1
	ldr	w10, [x8, x10]
	ldr	w8, [x20, #1188]
	ubfx	w11, w10, #6, #10
	sub	w11, w8, w11
	ldr	x8, [x22, #56]
	and	w9, w10, #0x3f
	sub	w9, w9, #3              // =3
	cmp	w9, #3                  // =3
	csel	w9, wzr, w11, lo
	csel	w10, w11, wzr, lo
	mov	x0, x8
	add	w23, w9, w23
	add	w24, w10, w24
	bl	find_next_bit
	ldr	x8, [x20, #136]
	ldr	w9, [x8, #96]
	cmp	w9, w0
	b.hi	.LBB23_2
	b	.LBB23_4
.LBB23_3:
	mov	w23, wzr
	mov	w24, wzr
.LBB23_4:
	mov	x0, x19
	bl	mutex_unlock
	cmp	w23, w24
	csel	w8, w23, w24, hi
	subs	w8, w8, w21
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	csel	w0, wzr, w8, lo
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.Lfunc_end23:
	.size	f2fs_get_unusable_blocks, .Lfunc_end23-f2fs_get_unusable_blocks
                                        // -- End function
	.globl	f2fs_disable_cp_again   // -- Begin function f2fs_disable_cp_again
	.p2align	2
	.type	f2fs_disable_cp_again,@function
f2fs_disable_cp_again:                  // @f2fs_disable_cp_again
.Lf2fs_disable_cp_again$local:
// %bb.0:
	ldr	w8, [x0, #1520]
	cmp	w8, w1
	b.hs	.LBB24_2
// %bb.1:
	mov	w0, #-11
	ret
.LBB24_2:
	ldr	x10, [x0, #136]
	ldp	w9, w8, [x10, #100]
	ldr	x11, [x0, #80]
	tbz	w11, #9, .LBB24_5
// %bb.3:
	ldr	x10, [x10, #16]
	sub	w8, w8, w9
	ldp	w11, w12, [x10, #112]
	ldp	w9, w13, [x10, #120]
	add	w11, w12, w11
	ldp	w12, w10, [x10, #128]
	add	w9, w11, w9
	add	w9, w9, w13
	add	w9, w9, w12
	add	w9, w9, w10
	cmp	w9, w8
	b.ls	.LBB24_5
// %bb.4:
	mov	w0, #-11
	ret
.LBB24_5:
	mov	w0, wzr
	ret
.Lfunc_end24:
	.size	f2fs_disable_cp_again, .Lfunc_end24-f2fs_disable_cp_again
                                        // -- End function
	.globl	f2fs_drop_discard_cmd   // -- Begin function f2fs_drop_discard_cmd
	.p2align	2
	.type	f2fs_drop_discard_cmd,@function
f2fs_drop_discard_cmd:                  // @f2fs_drop_discard_cmd
.Lf2fs_drop_discard_cmd$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	x29, sp
	bl	__drop_discard_cmd
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end25:
	.size	f2fs_drop_discard_cmd, .Lfunc_end25-f2fs_drop_discard_cmd
                                        // -- End function
	.p2align	2               // -- Begin function __drop_discard_cmd
	.type	__drop_discard_cmd,@function
__drop_discard_cmd:                     // @__drop_discard_cmd
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	x19, x0
	mov	x29, sp
	ldr	x25, [x8, #168]
	mov	w8, #8280
	add	x20, x25, x8
	mov	x0, x20
	bl	mutex_lock
	adrp	x23, .L__func__.__drop_discard_cmd
	mov	w24, wzr
	add	x21, x19, #3408         // =3408
	mov	w26, #511
	adrp	x27, f2fs_dclient
	add	x23, x23, :lo12:.L__func__.__drop_discard_cmd
	b	.LBB26_3
.LBB26_1:                               //   in Loop: Header=BB26_3 Depth=1
	mov	w24, #1
.LBB26_2:                               //   in Loop: Header=BB26_3 Depth=1
	cmp	x26, #0                 // =0
	sub	x26, x26, #1            // =1
	b.le	.LBB26_14
.LBB26_3:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB26_4 Depth 2
	add	x28, x25, x26, lsl #4
	ldr	x22, [x28, #24]!
	cmp	x28, x22
	b.eq	.LBB26_2
.LBB26_4:                               //   Parent Loop BB26_3 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	mov	x8, x22
	ldrb	w9, [x22, #58]
	ldr	x22, [x22]
	sub	x24, x8, #40            // =40
	cbnz	w9, .LBB26_6
.LBB26_5:                               //   in Loop: Header=BB26_4 Depth=2
	mov	x0, x19
	mov	x1, x24
	bl	__remove_discard_cmd
	cmp	x28, x22
	b.ne	.LBB26_4
	b	.LBB26_1
.LBB26_6:                               //   in Loop: Header=BB26_4 Depth=2
	bl	print_bdev_access_info
	ldr	x0, [x27, :lo12:f2fs_dclient]
	cbz	x0, .LBB26_9
// %bb.7:                               //   in Loop: Header=BB26_4 Depth=2
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB26_9
// %bb.8:                               //   in Loop: Header=BB26_4 Depth=2
	ldr	x0, [x27, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	mov	w3, #2262
	add	x1, x1, :lo12:.L.str.1
	mov	x2, x23
	bl	dsm_client_record
	ldr	x0, [x27, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB26_9:                               //   in Loop: Header=BB26_4 Depth=2
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 2262; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB26_5
// %bb.10:                              //   in Loop: Header=BB26_4 Depth=2
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
	cbnz	w8, .LBB26_5
// %bb.11:                              //   in Loop: Header=BB26_4 Depth=2
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB26_5
// %bb.12:                              //   in Loop: Header=BB26_4 Depth=2
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB26_5
// %bb.13:                              //   in Loop: Header=BB26_4 Depth=2
	mov	w0, #8
	mov	x2, x21
	bl	queue_work_on
	b	.LBB26_5
.LBB26_14:
	mov	x0, x20
	bl	mutex_unlock
	and	w0, w24, #0x1
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.Lfunc_end26:
	.size	__drop_discard_cmd, .Lfunc_end26-__drop_discard_cmd
                                        // -- End function
	.globl	f2fs_stop_discard_thread // -- Begin function f2fs_stop_discard_thread
	.p2align	2
	.type	f2fs_stop_discard_thread,@function
f2fs_stop_discard_thread:               // @f2fs_stop_discard_thread
.Lf2fs_stop_discard_thread$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	x29, sp
	ldr	x8, [x8, #168]
	cbz	x8, .LBB27_3
// %bb.1:
	ldr	x0, [x8]
	cbz	x0, .LBB27_3
// %bb.2:
	str	xzr, [x8]
	bl	kthread_stop
.LBB27_3:
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end27:
	.size	f2fs_stop_discard_thread, .Lfunc_end27-f2fs_stop_discard_thread
                                        // -- End function
	.globl	f2fs_issue_discard_timeout // -- Begin function f2fs_issue_discard_timeout
	.p2align	2
	.type	f2fs_issue_discard_timeout,@function
f2fs_issue_discard_timeout:             // @f2fs_issue_discard_timeout
.Lf2fs_issue_discard_timeout$local:
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
	ldr	x8, [x0, #136]
	adrp	x21, dpolicys+180
	add	x21, x21, :lo12:dpolicys+180
	ldr	w22, [x21, #116]
	ldur	x23, [x21, #92]
	ldur	x24, [x21, #100]
	ldur	x25, [x21, #108]
	ldur	x26, [x21, #60]
	ldur	x27, [x21, #68]
	ldur	x28, [x21, #76]
	ldur	x20, [x21, #84]
	ldr	x8, [x8, #168]
	add	x1, sp, #24             // =24
	mov	x19, x0
	str	w22, [sp, #80]
	stp	x8, x26, [sp, #16]      // 8-byte Folded Spill
	stp	x23, x24, [sp, #56]
	str	x25, [sp, #72]
	stp	x27, x28, [sp, #32]
	str	x20, [sp, #48]
	strb	wzr, [sp, #80]
	bl	__issue_discard_cmd
	mov	x0, x19
	bl	__drop_discard_cmd
	ldr	w8, [x21, #56]
	ldp	x9, x10, [x21, #32]
	str	w0, [sp, #12]           // 4-byte Folded Spill
	sub	x1, x29, #72            // =72
	stur	w8, [x29, #-16]
	ldr	x8, [x21, #48]
	stp	x9, x10, [x29, #-40]
	ldp	x9, x10, [x21]
	mov	w3, #-1
	stur	x8, [x29, #-24]
	ldp	x8, x11, [x21, #16]
	mov	x0, x19
	mov	w2, wzr
	stp	x9, x10, [x29, #-72]
	stp	x8, x11, [x29, #-56]
	sturb	wzr, [x29, #-16]
	bl	__wait_discard_cmd_range
	sub	x1, x29, #72            // =72
	mov	w3, #-1
	mov	x0, x19
	mov	w2, wzr
	stur	w22, [x29, #-16]
	stp	x23, x24, [x29, #-40]
	stur	x25, [x29, #-24]
	stp	x26, x27, [x29, #-72]
	stp	x28, x20, [x29, #-56]
	sturb	wzr, [x29, #-16]
	bl	__wait_discard_cmd_range
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	ldr	w8, [x8, #8340]
	cbnz	w8, .LBB28_3
.LBB28_1:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB28_11
// %bb.2:
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	ldp	x20, x19, [sp, #240]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #224]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #208]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #192]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #176]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #160]    // 16-byte Folded Reload
	and	w0, w8, #0x1
	add	sp, sp, #256            // =256
	ret
.LBB28_3:
	bl	print_bdev_access_info
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	cbz	x0, .LBB28_6
// %bb.4:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB28_6
// %bb.5:
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_issue_discard_timeout
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_issue_discard_timeout
	mov	w3, #2412
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB28_6:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 2412; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB28_1
// %bb.7:
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
	cbnz	w8, .LBB28_1
// %bb.8:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB28_1
// %bb.9:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB28_1
// %bb.10:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB28_1
.LBB28_11:
	bl	__stack_chk_fail
.Lfunc_end28:
	.size	f2fs_issue_discard_timeout, .Lfunc_end28-f2fs_issue_discard_timeout
                                        // -- End function
	.p2align	2               // -- Begin function __issue_discard_cmd
	.type	__issue_discard_cmd,@function
__issue_discard_cmd:                    // @__issue_discard_cmd
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
	ldr	x8, [x0, #136]
	mov	x19, x1
	mov	x20, x0
	adrp	x9, jiffies
	ldr	x8, [x8, #168]
	stp	xzr, xzr, [sp, #136]
	stp	xzr, xzr, [sp, #120]
	stp	xzr, xzr, [sp, #104]
	stp	xzr, xzr, [sp, #88]
	str	wzr, [sp, #84]
	str	x8, [sp, #64]           // 8-byte Folded Spill
	ldrb	w8, [x1, #56]
	cbz	w8, .LBB29_2
// %bb.1:
	ldr	x8, [x9, :lo12:jiffies]
	str	x8, [x20, #536]
.LBB29_2:
	add	x11, x20, #3408         // =3408
	str	x11, [sp, #24]          // 8-byte Folded Spill
	add	x11, x19, #36           // =36
	str	x11, [sp, #40]          // 8-byte Folded Spill
	add	x11, x19, #20           // =20
	str	x11, [sp, #72]          // 8-byte Folded Spill
	add	x11, x19, #28           // =28
	str	x11, [sp, #32]          // 8-byte Folded Spill
	ldr	x11, [sp, #64]          // 8-byte Folded Reload
	mov	w8, #8360
	mov	w9, #8280
	mov	w10, #8344
	add	x8, x11, x8
	mov	x24, xzr
	mov	w22, wzr
	mov	w23, #250
	str	x8, [sp, #48]           // 8-byte Folded Spill
	add	x28, x11, x9
	add	x8, x11, x10
	str	x8, [sp, #16]           // 8-byte Folded Spill
	str	x28, [sp, #56]          // 8-byte Folded Spill
	b	.LBB29_4
.LBB29_3:                               //   in Loop: Header=BB29_4 Depth=1
	mov	w3, #-1
	mov	x0, x20
	mov	x1, x21
	mov	w2, wzr
	bl	__wait_discard_cmd_range
	mov	w22, w25
.LBB29_4:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB29_6 Depth 2
                                        //       Child Loop BB29_19 Depth 3
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	str	wzr, [sp, #84]
	ldrb	w8, [x8]
	cbnz	w8, .LBB29_66
.LBB29_5:                               //   in Loop: Header=BB29_4 Depth=1
	add	x0, sp, #88             // =88
	bl	blk_start_plug
	mov	w21, wzr
	mov	w27, #511
.LBB29_6:                               //   Parent Loop BB29_4 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB29_19 Depth 3
	ldrb	w8, [x19, #56]
	cbz	w8, .LBB29_8
// %bb.7:                               //   in Loop: Header=BB29_6 Depth=2
	ldr	x8, [x20, #584]
	ldr	x9, [x20, #536]
	adrp	x10, jiffies
	ldr	x10, [x10, :lo12:jiffies]
	madd	x8, x8, x23, x9
	cmp	x8, x10
	b.mi	.LBB29_59
.LBB29_8:                               //   in Loop: Header=BB29_6 Depth=2
	ldr	w8, [x19, #4]
	add	x25, x27, #1            // =1
	cmp	x25, x8
	b.lo	.LBB29_59
// %bb.9:                               //   in Loop: Header=BB29_6 Depth=2
	ldr	w26, [x19]
	ldr	x24, [sp, #72]          // 8-byte Folded Reload
	cmp	w26, #3                 // =3
	b.eq	.LBB29_12
// %bb.10:                              //   in Loop: Header=BB29_6 Depth=2
	ldr	x24, [sp, #72]          // 8-byte Folded Reload
	cmp	x27, #510               // =510
	b.hi	.LBB29_14
// %bb.11:                              //   in Loop: Header=BB29_6 Depth=2
	ldp	x9, x8, [sp, #32]       // 16-byte Folded Reload
	cmp	x27, #14                // =14
	csel	x24, x9, x8, hi
.LBB29_12:                              //   in Loop: Header=BB29_6 Depth=2
	cmp	x27, #15                // =15
	b.hi	.LBB29_14
// %bb.13:                              //   in Loop: Header=BB29_6 Depth=2
	ldrb	w8, [x19, #17]
	cbnz	w8, .LBB29_76
.LBB29_14:                              //   in Loop: Header=BB29_6 Depth=2
	ldr	x8, [sp, #64]           // 8-byte Folded Reload
	mov	x0, x28
	add	x26, x8, x27, lsl #4
	bl	mutex_lock
	ldr	x8, [x26, #24]!
	cmp	x26, x8
	b.eq	.LBB29_47
// %bb.15:                              //   in Loop: Header=BB29_6 Depth=2
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	ldrb	w8, [x8]
	cbnz	w8, .LBB29_50
.LBB29_16:                              //   in Loop: Header=BB29_6 Depth=2
	ldr	x8, [x26]
	b	.LBB29_19
.LBB29_17:                              //   in Loop: Header=BB29_19 Depth=3
	add	x4, sp, #84             // =84
	mov	x0, x20
	mov	x1, x19
	mov	w2, w27
	mov	x3, x28
	bl	__submit_discard_cmd
	ldr	w21, [sp, #84]
.LBB29_18:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w8, [x24]
	cmp	w21, w8
	mov	x8, x25
	b.hs	.LBB29_47
.LBB29_19:                              //   Parent Loop BB29_4 Depth=1
                                        //     Parent Loop BB29_6 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	mov	x28, x8
	ldr	x25, [x28], #-40
	cmp	x26, x8
	b.eq	.LBB29_47
// %bb.20:                              //   in Loop: Header=BB29_19 Depth=3
	ldrb	w8, [x8, #58]
	cbnz	w8, .LBB29_34
.LBB29_21:                              //   in Loop: Header=BB29_19 Depth=3
	ldrb	w8, [x19, #56]
	cbz	w8, .LBB29_23
// %bb.22:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	x8, [x20, #584]
	ldr	x9, [x20, #536]
	adrp	x10, jiffies
	ldr	x10, [x10, :lo12:jiffies]
	madd	x8, x8, x23, x9
	cmp	x8, x10
	b.mi	.LBB29_47
.LBB29_23:                              //   in Loop: Header=BB29_19 Depth=3
	ldrb	w8, [x19, #8]
	cbz	w8, .LBB29_17
// %bb.24:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w8, [x19, #12]
	cmp	x27, x8
	b.hs	.LBB29_17
// %bb.25:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w8, [x20, #1988]
	cmp	w8, #5                  // =5
	b.eq	.LBB29_17
// %bb.26:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w9, [x20, #1308]
	cbnz	w9, .LBB29_33
// %bb.27:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w9, [x20, #1312]
	cbnz	w9, .LBB29_33
// %bb.28:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w9, [x20, #1316]
	cbnz	w9, .LBB29_33
// %bb.29:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w9, [x20, #1304]
	cbnz	w9, .LBB29_33
// %bb.30:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w9, [x20, #1300]
	cbnz	w9, .LBB29_33
// %bb.31:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w9, [x20, #1324]
	cbnz	w9, .LBB29_33
// %bb.32:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w9, [x20, #1320]
	cbz	w9, .LBB29_42
.LBB29_33:                              //   in Loop: Header=BB29_19 Depth=3
	mov	w22, #1
	b	.LBB29_18
.LBB29_34:                              //   in Loop: Header=BB29_19 Depth=3
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB29_37
// %bb.35:                              //   in Loop: Header=BB29_19 Depth=3
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB29_37
// %bb.36:                              //   in Loop: Header=BB29_19 Depth=3
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__issue_discard_cmd
	mov	w3, #2210
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__issue_discard_cmd
	bl	dsm_client_record
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB29_37:                              //   in Loop: Header=BB29_19 Depth=3
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 2210; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB29_21
// %bb.38:                              //   in Loop: Header=BB29_19 Depth=3
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
	cbnz	w8, .LBB29_21
// %bb.39:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB29_21
// %bb.40:                              //   in Loop: Header=BB29_19 Depth=3
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB29_21
// %bb.41:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	x2, [sp, #24]           // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB29_21
.LBB29_42:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	x9, [x20, #136]
	cbz	x9, .LBB29_45
// %bb.43:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	x9, [x9, #160]
	cbz	x9, .LBB29_45
// %bb.44:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	w9, [x9, #36]
	cbnz	w9, .LBB29_33
.LBB29_45:                              //   in Loop: Header=BB29_19 Depth=3
	cmp	w8, #6                  // =6
	b.eq	.LBB29_17
// %bb.46:                              //   in Loop: Header=BB29_19 Depth=3
	ldr	x8, [x20, #560]
	ldr	x9, [x20, #512]
	adrp	x10, jiffies
	ldr	x10, [x10, :lo12:jiffies]
	madd	x8, x8, x23, x9
	cmp	x8, x10
	b.pl	.LBB29_33
	b	.LBB29_17
.LBB29_47:                              //   in Loop: Header=BB29_6 Depth=2
	ldr	x28, [sp, #56]          // 8-byte Folded Reload
	mov	x0, x28
	bl	mutex_unlock
	tbnz	w22, #0, .LBB29_59
// %bb.48:                              //   in Loop: Header=BB29_6 Depth=2
	ldr	w8, [x24]
	cmp	w21, w8
	b.hs	.LBB29_59
// %bb.49:                              //   in Loop: Header=BB29_6 Depth=2
	mov	w22, wzr
	mov	w25, wzr
	cmp	x27, #0                 // =0
	sub	x27, x27, #1            // =1
	b.gt	.LBB29_6
	b	.LBB29_60
.LBB29_50:                              //   in Loop: Header=BB29_6 Depth=2
	ldr	x1, [sp, #16]           // 8-byte Folded Reload
	mov	x0, x20
	mov	w2, wzr
	bl	f2fs_check_rb_tree_consistence
	tbnz	w0, #0, .LBB29_16
// %bb.51:                              //   in Loop: Header=BB29_6 Depth=2
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB29_54
// %bb.52:                              //   in Loop: Header=BB29_6 Depth=2
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB29_54
// %bb.53:                              //   in Loop: Header=BB29_6 Depth=2
	adrp	x25, f2fs_dclient
	ldr	x0, [x25, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__issue_discard_cmd
	mov	w3, #2204
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__issue_discard_cmd
	bl	dsm_client_record
	ldr	x0, [x25, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB29_54:                              //   in Loop: Header=BB29_6 Depth=2
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 2204; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB29_16
// %bb.55:                              //   in Loop: Header=BB29_6 Depth=2
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
	cbnz	w8, .LBB29_16
// %bb.56:                              //   in Loop: Header=BB29_6 Depth=2
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB29_16
// %bb.57:                              //   in Loop: Header=BB29_6 Depth=2
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB29_16
// %bb.58:                              //   in Loop: Header=BB29_6 Depth=2
	ldr	x2, [sp, #24]           // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB29_16
.LBB29_59:                              //   in Loop: Header=BB29_4 Depth=1
	mov	w25, w22
.LBB29_60:                              //   in Loop: Header=BB29_4 Depth=1
	add	x0, sp, #88             // =88
	bl	blk_finish_plug
	cbz	x24, .LBB29_62
// %bb.61:                              //   in Loop: Header=BB29_4 Depth=1
	ldr	w8, [x24, #4]
	str	w8, [x19, #44]
.LBB29_62:                              //   in Loop: Header=BB29_4 Depth=1
	ldr	w8, [x19]
	cmp	w8, #4                  // =4
	b.ne	.LBB29_99
// %bb.63:                              //   in Loop: Header=BB29_4 Depth=1
	cbz	w21, .LBB29_99
// %bb.64:                              //   in Loop: Header=BB29_4 Depth=1
	sub	x8, x29, #80            // =80
	mov	x21, x19
	stur	wzr, [x29, #-24]
	stp	xzr, xzr, [x8, #40]
	stp	xzr, xzr, [x8, #24]
	stp	xzr, xzr, [x29, #-72]
	stur	xzr, [x29, #-80]
	cbnz	x19, .LBB29_3
// %bb.65:                              //   in Loop: Header=BB29_4 Depth=1
	adrp	x26, dpolicys+180
	add	x26, x26, :lo12:dpolicys+180
	ldr	w8, [x26, #56]
	ldp	x9, x10, [x26, #32]
	ldr	x11, [x26, #48]
	sub	x22, x29, #80           // =80
	stur	w8, [x29, #-24]
	ldp	x8, x12, [x26]
	stp	x10, x11, [x22, #40]
	ldp	x10, x11, [x26, #16]
	sub	x1, x29, #80            // =80
	mov	w3, #-1
	mov	x0, x20
	mov	w2, wzr
	stp	x8, x12, [x29, #-80]
	stur	x10, [x29, #-64]
	stp	x11, x9, [x22, #24]
	sturb	wzr, [x29, #-24]
	sub	x21, x29, #80           // =80
	bl	__wait_discard_cmd_range
	ldr	w8, [x26, #116]
	ldur	x10, [x26, #100]
	ldur	x11, [x26, #108]
	ldur	x9, [x26, #92]
	ldur	x12, [x26, #60]
	ldur	x13, [x26, #68]
	stur	w8, [x29, #-24]
	ldur	x8, [x26, #76]
	stp	x10, x11, [x22, #40]
	ldur	x10, [x26, #84]
	stp	x12, x13, [x29, #-80]
	stur	x8, [x29, #-64]
	stp	x10, x9, [x22, #24]
	sturb	wzr, [x29, #-24]
	b	.LBB29_3
.LBB29_66:                              //   in Loop: Header=BB29_4 Depth=1
	mov	x0, x28
	bl	mutex_lock
	ldr	x1, [sp, #16]           // 8-byte Folded Reload
	mov	x0, x20
	mov	w2, wzr
	bl	f2fs_check_rb_tree_consistence
	tbz	w0, #0, .LBB29_68
.LBB29_67:                              //   in Loop: Header=BB29_4 Depth=1
	mov	x0, x28
	bl	mutex_unlock
	b	.LBB29_5
.LBB29_68:                              //   in Loop: Header=BB29_4 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB29_71
// %bb.69:                              //   in Loop: Header=BB29_4 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB29_71
// %bb.70:                              //   in Loop: Header=BB29_4 Depth=1
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__issue_discard_cmd
	mov	w3, #2172
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__issue_discard_cmd
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB29_71:                              //   in Loop: Header=BB29_4 Depth=1
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 2172; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x28, [sp, #56]          // 8-byte Folded Reload
	cbz	x20, .LBB29_67
// %bb.72:                              //   in Loop: Header=BB29_4 Depth=1
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
	cbnz	w8, .LBB29_67
// %bb.73:                              //   in Loop: Header=BB29_4 Depth=1
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB29_67
// %bb.74:                              //   in Loop: Header=BB29_4 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB29_67
// %bb.75:                              //   in Loop: Header=BB29_4 Depth=1
	ldr	x2, [sp, #24]           // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB29_67
.LBB29_76:
	ldr	x8, [x20, #136]
	ldr	x22, [x8, #168]
	sub	x8, x29, #80            // =80
	stp	xzr, xzr, [x29, #-96]
	stp	xzr, xzr, [x29, #-112]
	stp	xzr, xzr, [x8, #48]
	stp	xzr, xzr, [x8, #32]
	str	xzr, [x8, #24]
	stp	xzr, xzr, [x29, #-72]
	stur	xzr, [x29, #-80]
	ldr	w21, [x22, #8328]
	mov	w8, #8280
	add	x28, x22, x8
	mov	x0, x28
	stur	wzr, [x29, #-116]
	bl	mutex_lock
	mov	w8, #8344
	add	x0, x22, x8
	sub	x3, x29, #88            // =88
	sub	x4, x29, #96            // =96
	sub	x5, x29, #104           // =104
	sub	x6, x29, #112           // =112
	mov	w7, #1
	mov	x1, xzr
	mov	w2, w21
	str	xzr, [sp]
	bl	f2fs_lookup_rb_tree_ret
	ldur	x8, [x29, #-96]
	cmp	x0, #0                  // =0
	csel	x23, x8, x0, eq
	sub	x0, x29, #80            // =80
	bl	blk_start_plug
	mov	w21, wzr
	cbz	x23, .LBB29_98
// %bb.77:
	ldp	x9, x8, [sp, #32]       // 16-byte Folded Reload
	cmp	w25, #15                // =15
	csel	x8, x9, x8, gt
	ldr	x9, [sp, #72]           // 8-byte Folded Reload
	cmp	w25, #511               // =511
	ccmp	w26, #3, #4, le
	csel	x26, x9, x8, eq
	b	.LBB29_80
.LBB29_78:                              //   in Loop: Header=BB29_80 Depth=1
	mov	x0, x23
	bl	rb_next
	mov	x24, x0
.LBB29_79:                              //   in Loop: Header=BB29_80 Depth=1
	mov	x23, x24
	cbz	x24, .LBB29_98
.LBB29_80:                              // =>This Inner Loop Header: Depth=1
	ldrb	w8, [x23, #98]
	cbnz	w8, .LBB29_78
// %bb.81:                              //   in Loop: Header=BB29_80 Depth=1
	ldrb	w8, [x19, #8]
	cbz	w8, .LBB29_95
// %bb.82:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	w8, [x20, #1988]
	cmp	w8, #5                  // =5
	b.eq	.LBB29_95
// %bb.83:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	w9, [x20, #1308]
	cbnz	w9, .LBB29_102
// %bb.84:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	w9, [x20, #1312]
	cbnz	w9, .LBB29_102
// %bb.85:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	w9, [x20, #1316]
	cbnz	w9, .LBB29_102
// %bb.86:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	w9, [x20, #1304]
	cbnz	w9, .LBB29_102
// %bb.87:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	w9, [x20, #1300]
	cbnz	w9, .LBB29_102
// %bb.88:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	w9, [x20, #1324]
	cbnz	w9, .LBB29_102
// %bb.89:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	w9, [x20, #1320]
	cbnz	w9, .LBB29_102
// %bb.90:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	x9, [x20, #136]
	cbz	x9, .LBB29_93
// %bb.91:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	x9, [x9, #160]
	cbz	x9, .LBB29_93
// %bb.92:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	w9, [x9, #36]
	cbnz	w9, .LBB29_102
.LBB29_93:                              //   in Loop: Header=BB29_80 Depth=1
	cmp	w8, #6                  // =6
	b.eq	.LBB29_95
// %bb.94:                              //   in Loop: Header=BB29_80 Depth=1
	ldr	x8, [x20, #560]
	ldr	x9, [x20, #512]
	adrp	x10, jiffies
	ldr	x10, [x10, :lo12:jiffies]
	mov	w11, #250
	madd	x8, x8, x11, x9
	cmp	x8, x10
	b.pl	.LBB29_102
.LBB29_95:                              //   in Loop: Header=BB29_80 Depth=1
	ldp	w8, w9, [x23, #24]
	sub	x4, x29, #116           // =116
	mov	x0, x20
	mov	x1, x19
	add	w8, w9, w8
	mov	w2, w27
	mov	x3, x23
	str	w8, [x22, #8328]
	bl	__submit_discard_cmd
	ldur	w21, [x29, #-116]
	ldr	w8, [x26]
	cmp	w21, w8
	b.hs	.LBB29_103
// %bb.96:                              //   in Loop: Header=BB29_80 Depth=1
	mov	w25, w0
	mov	x0, x23
	bl	rb_next
	mov	x24, x0
	cbz	w25, .LBB29_79
// %bb.97:                              //   in Loop: Header=BB29_80 Depth=1
	mov	x0, x20
	mov	x1, x23
	bl	__remove_discard_cmd
	b	.LBB29_79
.LBB29_98:
	sub	x0, x29, #80            // =80
	bl	blk_finish_plug
	mov	w19, wzr
	str	wzr, [x22, #8328]
	b	.LBB29_105
.LBB29_99:
	cbnz	w21, .LBB29_109
// %bb.100:
	eor	w8, w25, #0x1
	tbnz	w8, #0, .LBB29_109
// %bb.101:
	mov	w21, #-1
	str	w21, [sp, #84]
	b	.LBB29_109
.LBB29_102:
	mov	w19, #1
	b	.LBB29_104
.LBB29_103:
	mov	w19, wzr
.LBB29_104:
	sub	x0, x29, #80            // =80
	bl	blk_finish_plug
.LBB29_105:
	mov	x0, x28
	bl	mutex_unlock
	cbnz	w21, .LBB29_108
// %bb.106:
	cbz	w19, .LBB29_108
// %bb.107:
	mov	w21, #-1
	stur	w21, [x29, #-116]
.LBB29_108:
	add	x0, sp, #88             // =88
	str	w21, [sp, #84]
	bl	blk_finish_plug
.LBB29_109:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB29_111
// %bb.110:
	mov	w0, w21
	ldp	x20, x19, [sp, #352]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #336]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #320]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #304]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #288]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #272]    // 16-byte Folded Reload
	add	sp, sp, #368            // =368
	ret
.LBB29_111:
	bl	__stack_chk_fail
.Lfunc_end29:
	.size	__issue_discard_cmd, .Lfunc_end29-__issue_discard_cmd
                                        // -- End function
	.globl	f2fs_release_discard_addrs // -- Begin function f2fs_release_discard_addrs
	.p2align	2
	.type	f2fs_release_discard_addrs,@function
f2fs_release_discard_addrs:             // @f2fs_release_discard_addrs
.Lf2fs_release_discard_addrs$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	ldr	x20, [x8, #168]
	ldr	x19, [x20, #8]!
	cmp	x19, x20
	b.eq	.LBB30_5
// %bb.1:
	mov	x21, #256
	movk	x21, #57005, lsl #48
	adrp	x22, discard_entry_slab
	b	.LBB30_3
.LBB30_2:                               //   in Loop: Header=BB30_3 Depth=1
	add	x8, x21, #34            // =34
	stp	x21, x8, [x19]
	ldr	x0, [x22, :lo12:discard_entry_slab]
	mov	x1, x19
	bl	kmem_cache_free
	cmp	x23, x20
	mov	x19, x23
	b.eq	.LBB30_5
.LBB30_3:                               // =>This Inner Loop Header: Depth=1
	ldr	x23, [x19]
	mov	x0, x19
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB30_2
// %bb.4:                               //   in Loop: Header=BB30_3 Depth=1
	ldp	x9, x8, [x19]
	str	x8, [x9, #8]
	str	x9, [x8]
	b	.LBB30_2
.LBB30_5:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end30:
	.size	f2fs_release_discard_addrs, .Lfunc_end30-f2fs_release_discard_addrs
                                        // -- End function
	.globl	f2fs_clear_prefree_segments // -- Begin function f2fs_clear_prefree_segments
	.p2align	2
	.type	f2fs_clear_prefree_segments,@function
f2fs_clear_prefree_segments:            // @f2fs_clear_prefree_segments
.Lf2fs_clear_prefree_segments$local:
// %bb.0:
	sub	sp, sp, #144            // =144
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #64]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #80]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #96]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #112]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #128]    // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	ldr	w9, [x0, #1504]
	ldr	w10, [x1]
	add	x29, sp, #48            // =48
	ldr	x25, [x8, #16]
	ldr	x8, [x8, #168]
	mov	x19, x0
	cmp	w9, #1                  // =1
	ldr	x21, [x25, #64]
	str	x8, [sp]                // 8-byte Folded Spill
	add	x8, x8, #8              // =8
	str	x1, [sp, #24]           // 8-byte Folded Spill
	stur	w10, [x29, #-12]        // 4-byte Folded Spill
	str	x8, [sp, #8]            // 8-byte Folded Spill
	b.ne	.LBB31_2
// %bb.1:
	ldr	w8, [x19, #1192]
	cmp	w8, #1                  // =1
	cset	w26, hi
	b	.LBB31_3
.LBB31_2:
	mov	w26, wzr
.LBB31_3:
	add	x27, x25, #80           // =80
	mov	x0, x27
	bl	mutex_lock
	ldr	x8, [x19, #136]
	mov	x0, x21
	mov	x2, xzr
	ldr	w1, [x8, #96]
	bl	find_next_bit
	ldr	x8, [x19, #136]
	mov	x24, x0
	ldr	w8, [x8, #96]
	cmp	w8, w24
	b.ls	.LBB31_56
// %bb.4:
	mov	w22, #1
	mov	w28, #40
	b	.LBB31_8
.LBB31_5:                               //   in Loop: Header=BB31_8 Depth=1
	ldr	x10, [x19, #16]
	add	x10, x10, #72           // =72
.LBB31_6:                               //   in Loop: Header=BB31_8 Depth=1
	ldr	x8, [x8, #8]
	ldr	w11, [x19, #1184]
	ldr	w10, [x10]
	mov	x0, x19
	ldr	w8, [x8]
	lsl	w2, w9, w11
	add	w8, w8, w24
	lsl	w8, w8, w11
	add	w1, w8, w10
	bl	f2fs_issue_discard
.LBB31_7:                               //   in Loop: Header=BB31_8 Depth=1
	ldr	x8, [x19, #136]
	cmn	w23, #1                 // =1
	cset	w9, ne
	mov	x0, x21
	ldr	w1, [x8, #96]
	and	w8, w26, w9
	sub	w8, w23, w8
	add	w2, w8, #1              // =1
	bl	find_next_bit
	ldr	x8, [x19, #136]
	mov	x24, x0
	ldr	w8, [x8, #96]
	cmp	w8, w24
	b.ls	.LBB31_56
.LBB31_8:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB31_13 Depth 2
                                        //     Child Loop BB31_21 Depth 2
                                        //     Child Loop BB31_37 Depth 2
	mov	w1, w8
	add	w2, w24, #1             // =1
	mov	x0, x21
	bl	find_next_zero_bit
	mov	x23, x0
	cbz	w26, .LBB31_10
// %bb.9:                               //   in Loop: Header=BB31_8 Depth=1
	ldr	w8, [x19, #1192]
	udiv	w9, w24, w8
	add	w10, w23, w8
	mul	w24, w9, w8
	sub	w9, w10, #1             // =1
	udiv	w9, w9, w8
	mul	w23, w9, w8
.LBB31_10:                              //   in Loop: Header=BB31_8 Depth=1
	subs	w9, w23, w24
	b.ls	.LBB31_16
// %bb.11:                              //   in Loop: Header=BB31_8 Depth=1
	mov	w8, w24
	b	.LBB31_13
.LBB31_12:                              //   in Loop: Header=BB31_13 Depth=2
	add	w8, w8, #1              // =1
	cmp	w23, w8
	b.eq	.LBB31_16
.LBB31_13:                              //   Parent Loop BB31_8 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	lsr	w11, w8, #6
	ldr	x10, [x21, w11, uxtw #3]
	stur	x10, [x29, #-8]
	ldur	x12, [x29, #-8]
	lsl	x10, x22, x8
	tst	x12, x10
	b.eq	.LBB31_12
// %bb.14:                              //   in Loop: Header=BB31_13 Depth=2
	add	x11, x21, x11, lsl #3
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x11]
1:	ldxr	x12, [x11]
	bic	x13, x12, x10
	stlxr	w14, x13, [x11]
	cbnz	w14, 1b
	dmb ish
	//NO_APP
	tst	x12, x10
	b.eq	.LBB31_12
// %bb.15:                              //   in Loop: Header=BB31_13 Depth=2
	ldr	w10, [x25, #140]
	sub	w10, w10, #1            // =1
	str	w10, [x25, #140]
	b	.LBB31_12
.LBB31_16:                              //   in Loop: Header=BB31_8 Depth=1
	ldrb	w8, [x19, #1424]
	tbnz	w8, #2, .LBB31_18
.LBB31_17:                              //   in Loop: Header=BB31_8 Depth=1
	ldr	x8, [x19, #16]
	ldrb	w8, [x8, #2180]
	tbz	w8, #1, .LBB31_7
	b	.LBB31_27
.LBB31_18:                              //   in Loop: Header=BB31_8 Depth=1
	ldr	w8, [x19, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB31_24
// %bb.19:                              //   in Loop: Header=BB31_8 Depth=1
	ldr	x10, [x19, #3136]
	b	.LBB31_21
.LBB31_20:                              //   in Loop: Header=BB31_21 Depth=2
	subs	x8, x8, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB31_17
.LBB31_21:                              //   Parent Loop BB31_8 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	x11, [x10]
	ldr	x11, [x11, #128]
	ldr	x11, [x11, #1264]
	ldr	x12, [x11, #104]
	tbnz	w12, #8, .LBB31_27
// %bb.22:                              //   in Loop: Header=BB31_21 Depth=2
	cbz	x11, .LBB31_20
// %bb.23:                              //   in Loop: Header=BB31_21 Depth=2
	ldr	w11, [x11, #2980]
	sub	w11, w11, #1            // =1
	cmp	w11, #2                 // =2
	b.hs	.LBB31_20
	b	.LBB31_27
.LBB31_24:                              //   in Loop: Header=BB31_8 Depth=1
	ldr	x8, [x19]
	ldr	x8, [x8, #248]
	ldr	x8, [x8, #128]
	ldr	x8, [x8, #1264]
	ldr	x10, [x8, #104]
	tbnz	w10, #8, .LBB31_27
// %bb.25:                              //   in Loop: Header=BB31_8 Depth=1
	cbz	x8, .LBB31_17
// %bb.26:                              //   in Loop: Header=BB31_8 Depth=1
	ldr	w8, [x8, #2980]
	sub	w8, w8, #1              // =1
	cmp	w8, #2                  // =2
	b.hs	.LBB31_17
.LBB31_27:                              //   in Loop: Header=BB31_8 Depth=1
	ldur	w8, [x29, #-12]         // 4-byte Folded Reload
	tbz	w8, #4, .LBB31_30
// %bb.28:                              //   in Loop: Header=BB31_8 Depth=1
	ldr	x8, [sp, #24]           // 8-byte Folded Reload
	ldr	x8, [x8, #8]
	cmp	x8, w24, uxtw
	b.hi	.LBB31_30
// %bb.29:                              //   in Loop: Header=BB31_8 Depth=1
	ldr	x8, [sp, #24]           // 8-byte Folded Reload
	sub	w10, w23, #1            // =1
	ldr	x8, [x8, #16]
	cmp	x8, x10
	b.hs	.LBB31_7
.LBB31_30:                              //   in Loop: Header=BB31_8 Depth=1
	ldr	w8, [x19, #1504]
	cmp	w8, #1                  // =1
	b.ne	.LBB31_32
// %bb.31:                              //   in Loop: Header=BB31_8 Depth=1
	ldr	w8, [x19, #1192]
	cmp	w8, #2                  // =2
	b.hs	.LBB31_37
.LBB31_32:                              //   in Loop: Header=BB31_8 Depth=1
	ldr	x8, [x19, #136]
	cbz	x8, .LBB31_5
// %bb.33:                              //   in Loop: Header=BB31_8 Depth=1
	add	x10, x8, #80            // =80
	b	.LBB31_6
.LBB31_34:                              //   in Loop: Header=BB31_37 Depth=2
	add	x10, x9, #80            // =80
.LBB31_35:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	x9, [x9, #8]
	ldr	w11, [x19, #1184]
	ldr	w10, [x10]
	mov	x0, x19
	ldr	w9, [x9]
	lsl	w2, w8, w11
	add	w9, w9, w20
	lsl	w9, w9, w11
	add	w1, w9, w10
	bl	f2fs_issue_discard
	ldr	w8, [x19, #1192]
.LBB31_36:                              //   in Loop: Header=BB31_37 Depth=2
	add	w24, w8, w20
	cmp	w24, w23
	b.hs	.LBB31_55
.LBB31_37:                              //   Parent Loop BB31_8 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	cmn	w24, #1                 // =1
	b.eq	.LBB31_39
// %bb.38:                              //   in Loop: Header=BB31_37 Depth=2
	udiv	w10, w24, w8
	b	.LBB31_40
.LBB31_39:                              //   in Loop: Header=BB31_37 Depth=2
	mov	w10, #-1
.LBB31_40:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	x9, [x19, #136]
	mul	w20, w10, w8
	ldr	x11, [x9, #24]
	ldr	w12, [x11, #100]
	udiv	w12, w12, w8
	cmp	w10, w12
	b.eq	.LBB31_36
// %bb.41:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	w12, [x11, #220]
	udiv	w12, w12, w8
	cmp	w10, w12
	b.eq	.LBB31_36
// %bb.42:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	w12, [x11, #340]
	udiv	w12, w12, w8
	cmp	w10, w12
	b.eq	.LBB31_36
// %bb.43:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	w12, [x11, #460]
	udiv	w12, w12, w8
	cmp	w10, w12
	b.eq	.LBB31_36
// %bb.44:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	w12, [x11, #580]
	udiv	w12, w12, w8
	cmp	w10, w12
	b.eq	.LBB31_36
// %bb.45:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	w12, [x11, #700]
	udiv	w12, w12, w8
	cmp	w10, w12
	b.eq	.LBB31_36
// %bb.46:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	w12, [x11, #820]
	udiv	w12, w12, w8
	cmp	w10, w12
	b.eq	.LBB31_36
// %bb.47:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	w11, [x11, #940]
	udiv	w11, w11, w8
	cmp	w10, w11
	b.eq	.LBB31_36
// %bb.48:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	x10, [x9]
	cmp	w8, #2                  // =2
	b.lo	.LBB31_51
// %bb.49:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	x10, [x10, #128]
	cmn	w24, #1                 // =1
	b.eq	.LBB31_52
// %bb.50:                              //   in Loop: Header=BB31_37 Depth=2
	udiv	w11, w24, w8
	ldr	w10, [x10, x11, lsl #2]
	cbnz	w10, .LBB31_36
	b	.LBB31_53
.LBB31_51:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	x10, [x10, #120]
	umull	x11, w24, w28
	ldr	w10, [x10, x11]
	ubfx	w10, w10, #6, #10
	cbnz	w10, .LBB31_36
	b	.LBB31_53
.LBB31_52:                              //   in Loop: Header=BB31_37 Depth=2
	mov	w11, #-1
	ldr	w10, [x10, x11, lsl #2]
	cbnz	w10, .LBB31_36
.LBB31_53:                              //   in Loop: Header=BB31_37 Depth=2
	cbnz	x9, .LBB31_34
// %bb.54:                              //   in Loop: Header=BB31_37 Depth=2
	ldr	x10, [x19, #16]
	add	x10, x10, #72           // =72
	b	.LBB31_35
.LBB31_55:                              //   in Loop: Header=BB31_8 Depth=1
	sub	w23, w24, #1            // =1
	b	.LBB31_7
.LBB31_56:
	mov	x0, x27
	bl	mutex_unlock
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	ldr	x21, [x8]
	cmp	x21, x8
	b.eq	.LBB31_71
// %bb.57:
	mov	x20, x21
	//APP
	mrs x8, sp_el0
	//NO_APP
	str	x8, [sp, #16]           // 8-byte Folded Spill
	b	.LBB31_59
.LBB31_58:                              //   in Loop: Header=BB31_59 Depth=1
	mov	x9, #256
	movk	x9, #57005, lsl #48
	add	x8, x9, #34             // =34
	stp	x9, x8, [x22]
	adrp	x8, discard_entry_slab
	ldr	x0, [x8, :lo12:discard_entry_slab]
	mov	x1, x22
	bl	kmem_cache_free
	ldp	x9, x10, [sp]           // 16-byte Folded Reload
	mov	x21, x20
	ldr	w8, [x9, #8312]
	cmp	x20, x10
	sub	w8, w8, w28
	str	w8, [x9, #8312]
	b.eq	.LBB31_71
.LBB31_59:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB31_62 Depth 2
	mov	x22, x20
	mov	x23, x22
	ldr	x20, [x20]
	ldr	x9, [x23, #20]!
	ldr	w8, [x19, #1188]
	mov	w24, wzr
	mov	w28, wzr
	and	w27, w9, #0x1
	b	.LBB31_62
.LBB31_60:                              //   in Loop: Header=BB31_62 Depth=2
	mov	x2, x25
	bl	find_next_bit
	mov	x24, x0
.LBB31_61:                              //   in Loop: Header=BB31_62 Depth=2
	ldr	w8, [x19, #1188]
	eor	w27, w27, #0x1
	cmp	w24, w8
	b.hs	.LBB31_69
.LBB31_62:                              //   Parent Loop BB31_59 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	mov	w1, w8
	mov	w25, w24
	mov	x0, x23
	tbz	w27, #0, .LBB31_60
// %bb.63:                              //   in Loop: Header=BB31_62 Depth=2
	mov	x2, x25
	bl	find_next_zero_bit
	ldr	x8, [x19, #16]
	mov	x24, x0
	ldrb	w8, [x8, #2180]
	tbnz	w8, #1, .LBB31_61
// %bb.64:                              //   in Loop: Header=BB31_62 Depth=2
	ldur	w8, [x29, #-12]         // 4-byte Folded Reload
	sub	w26, w24, w25
	tbz	w8, #4, .LBB31_68
// %bb.65:                              //   in Loop: Header=BB31_62 Depth=2
	ldr	x8, [sp, #24]           // 8-byte Folded Reload
	ldr	x8, [x8, #24]
	cmp	x8, w26, uxtw
	b.hi	.LBB31_61
// %bb.66:                              //   in Loop: Header=BB31_62 Depth=2
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	ldr	x8, [x8]
	tbz	w8, #0, .LBB31_68
// %bb.67:                              //   in Loop: Header=BB31_62 Depth=2
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	ldrb	w8, [x8, #2128]
	tbnz	w8, #1, .LBB31_61
.LBB31_68:                              //   in Loop: Header=BB31_62 Depth=2
	ldr	w8, [x22, #16]
	mov	x0, x19
	mov	w2, w26
	add	w1, w8, w25
	bl	f2fs_issue_discard
	add	w28, w26, w28
	b	.LBB31_61
.LBB31_69:                              //   in Loop: Header=BB31_59 Depth=1
	mov	x0, x21
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB31_58
// %bb.70:                              //   in Loop: Header=BB31_59 Depth=1
	ldp	x9, x8, [x22]
	str	x8, [x9, #8]
	str	x9, [x8]
	b	.LBB31_58
.LBB31_71:
	ldr	x8, [x19, #136]
	ldr	x21, [x8, #168]
	mov	w8, #8280
	add	x20, x21, x8
	mov	x0, x20
	bl	mutex_lock
	ldr	w8, [x21, #8320]
	mov	w9, #512
.LBB31_72:                              // =>This Inner Loop Header: Depth=1
	cmp	w9, w8
	b.lo	.LBB31_75
// %bb.73:                              //   in Loop: Header=BB31_72 Depth=1
	sub	w10, w9, #1             // =1
	add	x10, x21, w10, uxtw #4
	ldr	x11, [x10, #24]!
	cmp	x10, x11
	b.ne	.LBB31_76
// %bb.74:                              //   in Loop: Header=BB31_72 Depth=1
	subs	w9, w9, #1              // =1
	b.ne	.LBB31_72
.LBB31_75:
	mov	x0, x20
	bl	mutex_unlock
	b	.LBB31_85
.LBB31_76:
	mov	x0, x20
	bl	mutex_unlock
	ldr	w8, [x19, #1988]
	cmp	w8, #5                  // =5
	b.ne	.LBB31_78
.LBB31_77:
	mov	w9, #8248
	mov	w8, #1
	add	x0, x21, x9
	mov	w1, #1
	mov	w2, wzr
	mov	x3, xzr
	str	w8, [x21, #8272]
	bl	__wake_up
	b	.LBB31_85
.LBB31_78:
	ldr	w9, [x19, #1308]
	cbnz	w9, .LBB31_85
// %bb.79:
	ldr	w9, [x19, #1312]
	cbnz	w9, .LBB31_85
// %bb.80:
	ldr	w9, [x19, #1316]
	cbnz	w9, .LBB31_85
// %bb.81:
	ldr	w9, [x19, #1304]
	cbnz	w9, .LBB31_85
// %bb.82:
	ldr	w9, [x19, #1300]
	cbnz	w9, .LBB31_85
// %bb.83:
	ldr	w9, [x19, #1324]
	cbnz	w9, .LBB31_85
// %bb.84:
	ldr	w9, [x19, #1320]
	cbz	w9, .LBB31_86
.LBB31_85:
	ldp	x20, x19, [sp, #128]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #112]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #96]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #80]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #64]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #144            // =144
	ret
.LBB31_86:
	ldr	x9, [x19, #136]
	cbz	x9, .LBB31_89
// %bb.87:
	ldr	x9, [x9, #160]
	cbz	x9, .LBB31_89
// %bb.88:
	ldr	w9, [x9, #36]
	cbnz	w9, .LBB31_85
.LBB31_89:
	cmp	w8, #6                  // =6
	b.eq	.LBB31_77
// %bb.90:
	ldr	x8, [x19, #560]
	ldr	x9, [x19, #512]
	adrp	x10, jiffies
	ldr	x10, [x10, :lo12:jiffies]
	mov	w11, #250
	madd	x8, x8, x11, x9
	cmp	x8, x10
	b.pl	.LBB31_85
	b	.LBB31_77
.Lfunc_end31:
	.size	f2fs_clear_prefree_segments, .Lfunc_end31-f2fs_clear_prefree_segments
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_issue_discard
	.type	f2fs_issue_discard,@function
f2fs_issue_discard:                     // @f2fs_issue_discard
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	mov	w20, w2
	mov	x2, xzr
	str	x27, [sp, #16]          // 8-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	mov	x29, sp
	mov	w22, w1
	mov	x19, x0
	bl	f2fs_target_device
	add	w8, w20, w22
	cmp	w8, w22
	b.ls	.LBB32_29
// %bb.1:
	mov	w22, w22
	mov	x21, x0
	mov	x20, xzr
	mov	w25, w8
	mov	w26, #40
	mov	w27, #1
	mov	x23, x22
	b	.LBB32_3
.LBB32_2:                               //   in Loop: Header=BB32_3 Depth=1
	add	x22, x22, #1            // =1
	cmp	x22, x25
	add	x20, x20, #1            // =1
	b.eq	.LBB32_27
.LBB32_3:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB32_19 Depth 2
	cmp	x23, x22
	b.ne	.LBB32_5
// %bb.4:                               //   in Loop: Header=BB32_3 Depth=1
	mov	x23, x22
	b	.LBB32_7
.LBB32_5:                               //   in Loop: Header=BB32_3 Depth=1
	mov	x0, x19
	mov	w1, w22
	mov	x2, xzr
	bl	f2fs_target_device
	cmp	x0, x21
	b.eq	.LBB32_7
// %bb.6:                               //   in Loop: Header=BB32_3 Depth=1
	mov	x24, x0
	mov	x0, x19
	mov	x1, x21
	mov	w2, w23
	mov	w3, w20
	bl	__issue_discard_async
	mov	x20, xzr
	mov	x23, x22
	mov	x21, x24
.LBB32_7:                               //   in Loop: Header=BB32_3 Depth=1
	add	w8, w22, #3             // =3
	cmp	w8, #3                  // =3
	b.hi	.LBB32_15
// %bb.8:                               //   in Loop: Header=BB32_3 Depth=1
	cmp	w8, #2                  // =2
	b.eq	.LBB32_15
.LBB32_9:                               //   in Loop: Header=BB32_3 Depth=1
	mov	w9, #-1
.LBB32_10:                              //   in Loop: Header=BB32_3 Depth=1
	ldr	x10, [x19, #136]
	mov	w9, w9
	ldr	x8, [x10]
	ldr	x8, [x8, #120]
	cbz	x10, .LBB32_12
// %bb.11:                              //   in Loop: Header=BB32_3 Depth=1
	add	x10, x10, #80           // =80
	b	.LBB32_13
.LBB32_12:                              //   in Loop: Header=BB32_3 Depth=1
	ldr	x10, [x19, #16]
	add	x10, x10, #72           // =72
.LBB32_13:                              //   in Loop: Header=BB32_3 Depth=1
	ldr	w10, [x10]
	ldr	w11, [x19, #1188]
	madd	x8, x9, x26, x8
	ldr	x8, [x8, #24]
	sub	w9, w22, w10
	sub	w10, w11, #1            // =1
	and	w9, w10, w9
	lsr	x10, x9, #3
	ldrb	w11, [x8, x10]
	mvn	w9, w9
	and	w9, w9, #0x7
	lsl	w9, w27, w9
	tst	w9, w11
	orr	w9, w11, w9
	strb	w9, [x8, x10]
	b.ne	.LBB32_2
// %bb.14:                              //   in Loop: Header=BB32_3 Depth=1
	ldr	w8, [x19, #1244]
	sub	w8, w8, #1              // =1
	str	w8, [x19, #1244]
	b	.LBB32_2
.LBB32_15:                              //   in Loop: Header=BB32_3 Depth=1
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB32_23
// %bb.16:                              //   in Loop: Header=BB32_3 Depth=1
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB32_22
// %bb.17:                              //   in Loop: Header=BB32_3 Depth=1
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB32_19
.LBB32_18:                              //   in Loop: Header=BB32_19 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB32_22
.LBB32_19:                              //   Parent Loop BB32_3 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	x22, x11
	b.lo	.LBB32_18
// %bb.20:                              //   in Loop: Header=BB32_19 Depth=2
	ldr	w11, [x10]
	cmp	x22, x11
	b.hi	.LBB32_18
// %bb.21:                              //   in Loop: Header=BB32_3 Depth=1
	ldur	x8, [x10, #-80]
.LBB32_22:                              //   in Loop: Header=BB32_3 Depth=1
	cbz	x8, .LBB32_9
.LBB32_23:                              //   in Loop: Header=BB32_3 Depth=1
	ldr	x8, [x19, #136]
	cbz	x8, .LBB32_25
// %bb.24:                              //   in Loop: Header=BB32_3 Depth=1
	add	x9, x8, #80             // =80
	b	.LBB32_26
.LBB32_25:                              //   in Loop: Header=BB32_3 Depth=1
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB32_26:                              //   in Loop: Header=BB32_3 Depth=1
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w22, w9
	lsr	w9, w9, w10
	sub	w9, w9, w8
	b	.LBB32_10
.LBB32_27:
	cbz	x20, .LBB32_29
// %bb.28:
	mov	x0, x19
	mov	x1, x21
	mov	w2, w23
	mov	w3, w20
	bl	__issue_discard_async
.LBB32_29:
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldr	x27, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.Lfunc_end32:
	.size	f2fs_issue_discard, .Lfunc_end32-f2fs_issue_discard
                                        // -- End function
	.globl	__set_sit_entry_type    // -- Begin function __set_sit_entry_type
	.p2align	2
	.type	__set_sit_entry_type,@function
__set_sit_entry_type:                   // @__set_sit_entry_type
.L__set_sit_entry_type$local:
// %bb.0:
	ldr	x8, [x0, #136]
	mov	w9, #40
	umull	x9, w2, w9
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	ldr	w10, [x8, x9]
	bfxil	w10, w1, #0, #6
	str	w10, [x8, x9]
	cbz	w3, .LBB33_2
// %bb.1:
	ldr	x8, [x0, #136]
	add	w9, w2, #63             // =63
	cmp	w2, #0                  // =0
	csel	w9, w9, w2, lt
	ldr	x8, [x8]
	asr	w11, w9, #6
	sbfiz	x11, x11, #3, #32
	and	w9, w9, #0xffffffc0
	ldr	x10, [x8, #56]
	sub	w9, w2, w9
	mov	w13, #1
	lsl	x9, x13, x9
	ldr	x12, [x10, x11]
	orr	x13, x12, x9
	tst	x12, x9
	str	x13, [x10, x11]
	b.eq	.LBB33_3
.LBB33_2:
	ret
.LBB33_3:
	ldr	w9, [x8, #64]
	add	w9, w9, #1              // =1
	str	w9, [x8, #64]
	ret
.Lfunc_end33:
	.size	__set_sit_entry_type, .Lfunc_end33-__set_sit_entry_type
                                        // -- End function
	.globl	f2fs_invalidate_blocks  // -- Begin function f2fs_invalidate_blocks
	.p2align	2
	.type	f2fs_invalidate_blocks,@function
f2fs_invalidate_blocks:                 // @f2fs_invalidate_blocks
.Lf2fs_invalidate_blocks$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	add	w8, w1, #3              // =3
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	mov	w20, w1
	cmp	w8, #4                  // =4
	mov	x19, x0
	str	x23, [sp, #16]          // 8-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	b.hs	.LBB34_2
.LBB34_1:
	mov	w21, #-1
	b	.LBB34_14
.LBB34_2:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB34_10
// %bb.3:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB34_9
// %bb.4:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB34_6
.LBB34_5:                               //   in Loop: Header=BB34_6 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB34_9
.LBB34_6:                               // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w20
	b.hi	.LBB34_5
// %bb.7:                               //   in Loop: Header=BB34_6 Depth=1
	ldr	w11, [x10]
	cmp	w11, w20
	b.lo	.LBB34_5
// %bb.8:
	ldur	x8, [x10, #-80]
.LBB34_9:
	cbz	x8, .LBB34_1
.LBB34_10:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB34_12
// %bb.11:
	add	x9, x8, #80             // =80
	b	.LBB34_13
.LBB34_12:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB34_13:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w20, w9
	lsr	w9, w9, w10
	sub	w21, w9, w8
.LBB34_14:
	ldr	x8, [x19, #136]
	ldr	x22, [x8]
	cbz	w20, .LBB34_18
// %bb.15:
	cmn	w20, #3                 // =3
	b.hi	.LBB34_17
.LBB34_16:
	ldr	x8, [x19, #248]
	mov	w1, w20
	mov	x2, x1
	ldr	x0, [x8, #48]
	bl	invalidate_mapping_pages
	mov	x0, x19
	mov	w1, w20
	bl	f2fs_invalidate_compress_page
	add	x22, x22, #72           // =72
	mov	x0, x22
	bl	down_write
	mov	x0, x19
	mov	w1, w20
	mov	x2, xzr
	bl	update_segment_mtime
	mov	w2, #-1
	mov	x0, x19
	mov	w1, w20
	bl	update_sit_entry
	mov	x0, x19
	mov	w1, w21
	bl	locate_dirty_segment
	mov	x0, x22
	bl	up_write
.LBB34_17:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB34_18:
	bl	print_bdev_access_info
	adrp	x23, f2fs_dclient
	ldr	x0, [x23, :lo12:f2fs_dclient]
	cbz	x0, .LBB34_21
// %bb.19:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB34_21
// %bb.20:
	ldr	x0, [x23, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_invalidate_blocks
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_invalidate_blocks
	mov	w3, #3093
	bl	dsm_client_record
	ldr	x0, [x23, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB34_21:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 3093; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB34_16
// %bb.22:
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
	cbnz	w8, .LBB34_16
// %bb.23:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB34_16
// %bb.24:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB34_16
// %bb.25:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB34_16
.Lfunc_end34:
	.size	f2fs_invalidate_blocks, .Lfunc_end34-f2fs_invalidate_blocks
                                        // -- End function
	.p2align	2               // -- Begin function update_segment_mtime
	.type	update_segment_mtime,@function
update_segment_mtime:                   // @update_segment_mtime
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	add	w8, w1, #3              // =3
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x20, x2
	cmp	w8, #4                  // =4
	mov	x19, x0
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	mov	x29, sp
	b.hs	.LBB35_2
.LBB35_1:
	mov	w21, #-1
	b	.LBB35_14
.LBB35_2:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB35_10
// %bb.3:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB35_9
// %bb.4:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB35_6
.LBB35_5:                               //   in Loop: Header=BB35_6 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB35_9
.LBB35_6:                               // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB35_5
// %bb.7:                               //   in Loop: Header=BB35_6 Depth=1
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB35_5
// %bb.8:
	ldur	x8, [x10, #-80]
.LBB35_9:
	cbz	x8, .LBB35_1
.LBB35_10:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB35_12
// %bb.11:
	add	x9, x8, #80             // =80
	b	.LBB35_13
.LBB35_12:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB35_13:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w1, w9
	lsr	w9, w9, w10
	sub	w21, w9, w8
.LBB35_14:
	ldr	x8, [x19, #136]
	mov	w0, #1
	ldr	x22, [x8]
	bl	ktime_get_coarse_with_offset
	mov	x9, #38067
	movk	x9, #9942, lsl #16
	movk	x9, #3048, lsl #32
	movk	x9, #4398, lsl #48
	ldr	x8, [x22, #144]
	smulh	x9, x0, x9
	asr	x10, x9, #26
	add	x9, x10, x9, lsr #63
	subs	x10, x8, x9
	b.ls	.LBB35_16
// %bb.15:
	ldr	x8, [x22, #136]
	subs	x8, x8, x10
	csel	x8, xzr, x8, lo
	b	.LBB35_17
.LBB35_16:
	ldr	x10, [x22, #136]
	sub	x8, x9, x8
	add	x8, x8, x10
.LBB35_17:
	cmp	x20, #0                 // =0
	csel	x9, x8, x20, eq
	cmn	w21, #1                 // =1
	b.eq	.LBB35_22
// %bb.18:
	ldr	x10, [x19, #136]
	ldr	x10, [x10]
	ldr	x11, [x10, #120]
	mov	w10, #40
	umaddl	x10, w21, w10, x11
	ldr	x12, [x10, #32]!
	cbz	x12, .LBB35_20
// %bb.19:
	mov	w13, w21
	mov	w14, #40
	mul	x13, x13, x14
	ldr	w11, [x11, x13]
	ubfx	x11, x11, #6, #10
	madd	x9, x12, x11, x9
	add	w11, w11, #1            // =1
	udiv	x9, x9, x11
.LBB35_20:
	str	x9, [x10]
	ldr	x9, [x19, #136]
	ldr	x9, [x9]
	ldr	x10, [x9, #160]
	cmp	x8, x10
	b.ls	.LBB35_22
// %bb.21:
	str	x8, [x9, #160]
.LBB35_22:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end35:
	.size	update_segment_mtime, .Lfunc_end35-update_segment_mtime
                                        // -- End function
	.p2align	2               // -- Begin function update_sit_entry
	.type	update_sit_entry,@function
update_sit_entry:                       // @update_sit_entry
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	sub	x9, sp, #416            // =416
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	mov	x29, sp
	and	sp, x9, #0xffffffffffffff80
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	mov	w20, w2
	mov	x19, x0
	mov	w21, w1
	str	x8, [sp, #408]
	ldr	x8, [x0, #136]
	cbz	x8, .LBB36_2
// %bb.1:
	add	x10, x8, #80            // =80
	add	x9, x8, #92             // =92
	b	.LBB36_3
.LBB36_2:
	ldr	x9, [x19, #16]
	add	x10, x9, #72            // =72
	add	x9, x9, #48             // =48
.LBB36_3:
	ldr	w11, [x9]
	ldr	w9, [x19, #1184]
	ldr	w10, [x10]
	lsl	w11, w11, w9
	add	w10, w11, w10
	cmp	w10, w21
	b.hi	.LBB36_6
// %bb.4:
	ldrb	w10, [x19, #3456]
	cbz	w10, .LBB36_6
// %bb.5:
	adrp	x1, .L.str.20
	adrp	x2, .L__func__.update_sit_entry
	add	x1, x1, :lo12:.L.str.20
	add	x2, x2, :lo12:.L__func__.update_sit_entry
	mov	x0, x19
	mov	w3, w21
	bl	f2fs_printk
	b	.LBB36_84
.LBB36_6:
	cbz	x8, .LBB36_9
// %bb.7:
	ldr	w10, [x8, #84]
	cmp	w10, w21
	b.hi	.LBB36_12
// %bb.8:
	add	x10, x8, #80            // =80
	add	x8, x8, #92             // =92
	b	.LBB36_11
.LBB36_9:
	ldr	x8, [x19, #16]
	ldr	w10, [x8, #92]
	cmp	w10, w21
	b.hi	.LBB36_12
// %bb.10:
	add	x10, x8, #72            // =72
	add	x8, x8, #48             // =48
.LBB36_11:
	ldr	w8, [x8]
	ldr	w10, [x10]
	lsl	w8, w8, w9
	add	w8, w8, w10
	cmp	w8, w21
	b.hi	.LBB36_36
.LBB36_12:
	adrp	x1, .L.str.21
	add	x1, x1, :lo12:.L.str.21
	mov	x0, x19
	mov	w2, w21
	bl	f2fs_printk
	ldrb	w8, [x19, #3376]
	cbnz	w8, .LBB36_28
// %bb.13:
	ldr	x8, [x19, #16]
	ldr	x9, [x19]
	ldr	w10, [x19, #1188]
	mov	w3, #8
	ldr	w8, [x8, #76]
	ldr	x0, [x9, #248]
	ldr	w2, [x9, #24]
	add	w8, w8, w10
	sub	w1, w8, #1              // =1
	bl	__bread_gfp
	cbz	x0, .LBB36_86
// %bb.14:
	ldr	x8, [x0]
	mov	x22, x0
	str	x8, [sp, #128]
	ldr	x8, [sp, #128]
	tbnz	w8, #2, .LBB36_16
// %bb.15:
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x22]
1:	ldaxr	x8, [x22]
	orr	x9, x8, 4
	stxr	w10, x9, [x22]
	cbnz	w10, 1b
	
	//NO_APP
	tbz	w8, #2, .LBB36_17
.LBB36_16:
	mov	x0, x22
	bl	__lock_buffer
.LBB36_17:
	ldr	x23, [x22, #40]
	mov	w8, #17227
	movk	w8, #18003, lsl #16
	ldr	w9, [x23]
	cmp	w9, w8
	b.ne	.LBB36_19
// %bb.18:
	mov	x0, x22
	bl	unlock_buffer
	b	.LBB36_27
.LBB36_19:
	str	w8, [x23]
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
	ldr	x8, [x19, #3208]
	ldr	w9, [x8]
	cmp	w9, #4                  // =4
	b.ne	.LBB36_108
// %bb.20:
	str	x8, [sp, #128]
	mov	w8, #8208
	movk	w8, #62197, lsl #16
	add	x0, sp, #128            // =128
	mov	w2, #4092
	mov	x1, x23
	str	w8, [sp, #256]
	bl	crypto_shash_update
	cbnz	w0, .LBB36_109
// %bb.21:
	ldr	w8, [sp, #256]
	str	w8, [x23, #4092]
	ldr	x8, [x22]
	tbnz	w8, #0, .LBB36_23
// %bb.22:
	//APP
	// atomic64_or
	prfm	pstl1strm, [x22]
1:	ldxr	x8, [x22]
	orr	x8, x8, 1
	stxr	w9, x8, [x22]
	cbnz	w9, 1b
	//NO_APP
.LBB36_23:
	ldr	x8, [x22]
	tbnz	w8, #1, .LBB36_25
// %bb.24:
	//APP
	// atomic64_or
	prfm	pstl1strm, [x22]
1:	ldxr	x8, [x22]
	orr	x8, x8, 2
	stxr	w9, x8, [x22]
	cbnz	w9, 1b
	//NO_APP
.LBB36_25:
	mov	x0, x22
	bl	unlock_buffer
	mov	w1, #2048
	movk	w1, #12, lsl #16
	mov	x0, x22
	bl	__sync_dirty_buffer
	cbnz	w0, .LBB36_27
// %bb.26:
	mov	w8, #1
	strb	w8, [x19, #3376]
.LBB36_27:
	mov	x0, x22
	bl	__brelse
.LBB36_28:
	bl	print_bdev_access_info
	adrp	x22, f2fs_dclient
	ldr	x0, [x22, :lo12:f2fs_dclient]
	cbz	x0, .LBB36_31
// %bb.29:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB36_31
// %bb.30:
	ldr	x0, [x22, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.update_sit_entry
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.update_sit_entry
	mov	w3, #2968
	bl	dsm_client_record
	ldr	x0, [x22, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB36_31:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 2968; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB36_36
// %bb.32:
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
	cbnz	w8, .LBB36_36
// %bb.33:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB36_36
// %bb.34:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB36_36
// %bb.35:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB36_36:
	add	w8, w21, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB36_38
.LBB36_37:
	mov	w8, #-1
	b	.LBB36_50
.LBB36_38:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB36_46
// %bb.39:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB36_45
// %bb.40:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB36_42
.LBB36_41:                              //   in Loop: Header=BB36_42 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB36_45
.LBB36_42:                              // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w21
	b.hi	.LBB36_41
// %bb.43:                              //   in Loop: Header=BB36_42 Depth=1
	ldr	w11, [x10]
	cmp	w11, w21
	b.lo	.LBB36_41
// %bb.44:
	ldur	x8, [x10, #-80]
.LBB36_45:
	cbz	x8, .LBB36_37
.LBB36_46:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB36_48
// %bb.47:
	add	x9, x8, #80             // =80
	b	.LBB36_49
.LBB36_48:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB36_49:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w21, w9
	lsr	w9, w9, w10
	sub	w8, w9, w8
.LBB36_50:
	ldr	x9, [x19, #136]
	mov	w23, w8
	ldr	x10, [x9]
	ldr	x25, [x10, #120]
	mov	w10, #40
	umaddl	x24, w8, w10, x25
	ldr	w10, [x24]
	ubfx	w8, w10, #6, #10
	add	w8, w8, w20
	sxtw	x22, w8
	cbz	x9, .LBB36_52
// %bb.51:
	add	x8, x9, #80             // =80
	b	.LBB36_53
.LBB36_52:
	ldr	x8, [x19, #16]
	add	x8, x8, #72             // =72
.LBB36_53:
	ldr	w8, [x8]
	ldr	w9, [x19, #1188]
	sub	w8, w21, w8
	sub	w10, w9, #1             // =1
	and	w8, w10, w8
	tbnz	w22, #31, .LBB36_56
// %bb.54:
	cmp	x22, x9
	b.gt	.LBB36_56
// %bb.55:
	mvn	w9, w8
	lsr	w26, w8, #3
	and	w8, w9, #0x7
	mov	w9, #1
	lsl	w27, w9, w8
	b	.LBB36_64
.LBB36_56:
	mov	w9, #40
	madd	x9, x23, x9, x25
	ldr	x9, [x9, #8]
	mvn	w10, w8
	mov	w8, w8
	lsr	x26, x8, #3
	ldrb	w9, [x9, x26]
	and	w8, w10, #0x7
	mov	w10, #1
	lsl	w27, w10, w8
	adrp	x8, .L.str.23
	adrp	x10, .L.str.24
	add	x8, x8, :lo12:.L.str.23
	add	x10, x10, :lo12:.L.str.24
	tst	w27, w9
	adrp	x1, .L.str.22
	csel	x4, x10, x8, eq
	add	x1, x1, :lo12:.L.str.22
	mov	x0, x19
	mov	x2, x22
	mov	w3, w21
	bl	f2fs_printk
	bl	print_bdev_access_info
	adrp	x28, f2fs_dclient
	ldr	x0, [x28, :lo12:f2fs_dclient]
	cbz	x0, .LBB36_59
// %bb.57:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB36_59
// %bb.58:
	ldr	x0, [x28, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.update_sit_entry
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.update_sit_entry
	mov	w3, #2993
	bl	dsm_client_record
	ldr	x0, [x28, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB36_59:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 2993; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB36_64
// %bb.60:
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
	cbnz	w8, .LBB36_64
// %bb.61:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB36_64
// %bb.62:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB36_64
// %bb.63:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB36_64:
	ldr	w8, [x24]
	mov	w9, #40
	cmp	w20, #1                 // =1
	bfi	w8, w22, #6, #10
	str	w8, [x24]
	madd	x8, x23, x9, x25
	ldr	x8, [x8, #8]
	ldrb	w10, [x8, x26]
	and	w9, w27, w10
	b.lt	.LBB36_68
// %bb.65:
	orr	w10, w10, w27
	strb	w10, [x8, x26]
	cbnz	w9, .LBB36_87
.LBB36_66:
	mov	w8, #40
	madd	x8, x23, x8, x25
	ldr	x8, [x8, #24]
	ldrb	w9, [x8, x26]
	tst	w27, w9
	orr	w9, w9, w27
	strb	w9, [x8, x26]
	b.eq	.LBB36_72
// %bb.67:
	ldr	x8, [x19, #80]
	tbz	w8, #8, .LBB36_73
	b	.LBB36_75
.LBB36_68:
	mvn	w22, w27
	bic	w10, w10, w27
	strb	w10, [x8, x26]
	cbz	w9, .LBB36_96
// %bb.69:
	ldr	x8, [x19, #80]
	tbnz	w8, #8, .LBB36_105
.LBB36_70:
	mov	w8, #40
	madd	x8, x23, x8, x25
	ldr	x8, [x8, #24]
	ldrb	w9, [x8, x26]
	tst	w27, w9
	and	w9, w9, w22
	strb	w9, [x8, x26]
	b.eq	.LBB36_75
// %bb.71:
	ldr	w8, [x19, #1244]
	add	w8, w8, #1              // =1
	str	w8, [x19, #1244]
	b	.LBB36_75
.LBB36_72:
	ldr	w8, [x19, #1244]
	sub	w8, w8, #1              // =1
	str	w8, [x19, #1244]
	ldr	x8, [x19, #80]
	tbnz	w8, #8, .LBB36_75
.LBB36_73:
	mov	w8, #40
	madd	x8, x23, x8, x25
	ldr	x8, [x8, #16]
	ldrb	w9, [x8, x26]
	tst	w27, w9
	orr	w9, w9, w27
	strb	w9, [x8, x26]
	b.ne	.LBB36_75
// %bb.74:
	ldr	w8, [x24]
	add	w9, w8, #16, lsl #12    // =65536
	lsr	w9, w9, #16
	bfi	w8, w9, #16, #10
	str	w8, [x24]
.LBB36_75:
	mov	w8, #40
	madd	x8, x23, x8, x25
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, x26]
	tst	w27, w8
	b.ne	.LBB36_77
// %bb.76:
	ldr	w8, [x24]
	add	w9, w8, w20, lsl #16
	lsr	w9, w9, #16
	bfi	w8, w9, #16, #10
	str	w8, [x24]
.LBB36_77:
	ldr	x8, [x19, #136]
	add	w9, w23, #63            // =63
	cmp	w23, #0                 // =0
	csel	w9, w9, w23, lt
	ldr	x8, [x8]
	asr	w12, w9, #6
	sbfiz	x12, x12, #3, #32
	and	w9, w9, #0xffffffc0
	ldr	x11, [x8, #56]
	mov	w10, #1
	sub	w9, w23, w9
	lsl	x9, x10, x9
	ldr	x13, [x11, x12]
	orr	x10, x13, x9
	tst	x13, x9
	str	x10, [x11, x12]
	b.ne	.LBB36_79
// %bb.78:
	ldr	w9, [x8, #64]
	add	w9, w9, #1              // =1
	str	w9, [x8, #64]
.LBB36_79:
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
	ldr	w9, [x8, #16]
	add	w9, w9, w20
	str	w9, [x8, #16]
	ldr	w9, [x19, #1192]
	cmp	w9, #2                  // =2
	b.lo	.LBB36_84
// %bb.80:
	ldr	x8, [x19, #136]
	cmn	w23, #1                 // =1
	ldr	x8, [x8]
	ldr	x8, [x8, #128]
	b.eq	.LBB36_82
// %bb.81:
	udiv	w9, w23, w9
	b	.LBB36_83
.LBB36_82:
	mov	w9, #-1
.LBB36_83:
	lsl	x9, x9, #2
	ldr	w10, [x8, x9]
	add	w10, w10, w20
	str	w10, [x8, x9]
.LBB36_84:
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #408]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB36_107
// %bb.85:
	mov	sp, x29
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.LBB36_86:
	adrp	x0, .L.str.27
	add	x0, x0, :lo12:.L.str.27
	bl	printk
	b	.LBB36_28
.LBB36_87:
	adrp	x1, .L.str.25
	add	x1, x1, :lo12:.L.str.25
	mov	x0, x19
	mov	w2, w21
	bl	f2fs_printk
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB36_90
// %bb.88:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB36_90
// %bb.89:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.update_sit_entry
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.update_sit_entry
	mov	w3, #3019
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB36_90:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 3019; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB36_95
// %bb.91:
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
	cbnz	w8, .LBB36_95
// %bb.92:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB36_95
// %bb.93:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB36_95
// %bb.94:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB36_95:
	ldr	w8, [x24]
	mov	w9, #65472
	mov	w20, wzr
	add	w9, w8, w9
	lsr	w9, w9, #6
	bfi	w8, w9, #6, #10
	str	w8, [x24]
	b	.LBB36_66
.LBB36_96:
	adrp	x1, .L.str.26
	add	x1, x1, :lo12:.L.str.26
	mov	x0, x19
	mov	w2, w21
	bl	f2fs_printk
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB36_99
// %bb.97:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB36_99
// %bb.98:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.update_sit_entry
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.update_sit_entry
	mov	w3, #3049
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB36_99:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 3049; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB36_104
// %bb.100:
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
	cbnz	w8, .LBB36_104
// %bb.101:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB36_104
// %bb.102:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB36_104
// %bb.103:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB36_104:
	ldr	w8, [x24]
	mov	w20, wzr
	add	w9, w8, #64             // =64
	lsr	w9, w9, #6
	bfi	w8, w9, #6, #10
	str	w8, [x24]
	b	.LBB36_70
.LBB36_105:
	mov	w8, #40
	madd	x8, x23, x8, x25
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, x26]
	tst	w27, w8
	b.eq	.LBB36_70
// %bb.106:
	add	x21, x19, #2340         // =2340
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	w8, [x19, #1264]
	mov	x0, x21
	add	w8, w8, #1              // =1
	str	w8, [x19, #1264]
	bl	_raw_spin_unlock
	b	.LBB36_70
.LBB36_107:
	bl	__stack_chk_fail
.LBB36_108:
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/f2fs.h"; .popsection; .long 14472b - 14470b; .short 2937; .short 0; .popsection; 14471: brk 0x800
	//NO_APP
.LBB36_109:
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/f2fs.h"; .popsection; .long 14472b - 14470b; .short 2943; .short 0; .popsection; 14471: brk 0x800
	//NO_APP
.Lfunc_end36:
	.size	update_sit_entry, .Lfunc_end36-update_sit_entry
                                        // -- End function
	.p2align	2               // -- Begin function locate_dirty_segment
	.type	locate_dirty_segment,@function
locate_dirty_segment:                   // @locate_dirty_segment
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	cmn	w1, #1                  // =1
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	b.eq	.LBB37_28
// %bb.1:
	ldr	x8, [x0, #136]
	mov	w19, w1
	mov	x20, x0
	ldr	x9, [x8, #24]
	ldr	w10, [x9, #100]
	cmp	w10, w1
	b.eq	.LBB37_28
// %bb.2:
	ldr	w10, [x9, #220]
	cmp	w10, w19
	b.eq	.LBB37_28
// %bb.3:
	ldr	w10, [x9, #340]
	cmp	w10, w19
	b.eq	.LBB37_28
// %bb.4:
	ldr	w10, [x9, #460]
	cmp	w10, w19
	b.eq	.LBB37_28
// %bb.5:
	ldr	w10, [x9, #580]
	cmp	w10, w19
	b.eq	.LBB37_28
// %bb.6:
	ldr	w10, [x9, #700]
	cmp	w10, w19
	b.eq	.LBB37_28
// %bb.7:
	ldr	w10, [x9, #820]
	cmp	w10, w19
	b.eq	.LBB37_28
// %bb.8:
	ldr	w9, [x9, #940]
	cmp	w9, w19
	b.eq	.LBB37_28
// %bb.9:
	ldr	x8, [x8, #16]
	add	x21, x8, #80            // =80
	mov	x0, x21
	bl	mutex_lock
	ldr	x8, [x20, #136]
	mov	w10, #40
	umull	x10, w19, w10
	ldr	x9, [x8]
	ldr	x9, [x9, #120]
	ldr	w11, [x9, x10]
	ubfx	w9, w11, #6, #10
	cbz	w9, .LBB37_13
// %bb.10:
	ldr	w10, [x20, #1188]
.LBB37_11:
	cmp	w9, w10
	b.hs	.LBB37_26
// %bb.12:
	mov	w2, #6
	mov	x0, x20
	mov	w1, w19
	bl	__locate_dirty_segment
	b	.LBB37_27
.LBB37_13:
	ldr	x10, [x20, #80]
	tbz	w10, #8, .LBB37_15
// %bb.14:
	ldr	w10, [x20, #1188]
	ubfx	w11, w11, #16, #10
	cmp	w11, w10
	b.ne	.LBB37_11
.LBB37_15:
	ldr	x9, [x8, #24]
	ldr	w10, [x9, #100]
	cmp	w10, w19
	b.eq	.LBB37_26
// %bb.16:
	ldr	w10, [x9, #220]
	cmp	w10, w19
	b.eq	.LBB37_26
// %bb.17:
	ldr	w10, [x9, #340]
	cmp	w10, w19
	b.eq	.LBB37_26
// %bb.18:
	ldr	w10, [x9, #460]
	cmp	w10, w19
	b.eq	.LBB37_26
// %bb.19:
	ldr	w10, [x9, #580]
	cmp	w10, w19
	b.eq	.LBB37_26
// %bb.20:
	ldr	w10, [x9, #700]
	cmp	w10, w19
	b.eq	.LBB37_26
// %bb.21:
	ldr	w10, [x9, #820]
	cmp	w10, w19
	b.eq	.LBB37_26
// %bb.22:
	ldr	w9, [x9, #940]
	cmp	w9, w19
	b.eq	.LBB37_26
// %bb.23:
	ldr	x8, [x8, #16]
	lsr	w11, w19, #6
	ldr	x9, [x8, #64]
	ldr	x10, [x9, w11, uxtw #3]
	str	x10, [x29, #24]
	ldr	x12, [x29, #24]
	mov	w10, #1
	lsl	x10, x10, x19
	tst	x12, x10
	b.ne	.LBB37_26
// %bb.24:
	add	x9, x9, x11, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x9]
1:	ldxr	x11, [x9]
	orr	x12, x11, x10
	stlxr	w13, x12, [x9]
	cbnz	w13, 1b
	dmb ish
	//NO_APP
	tst	x11, x10
	b.ne	.LBB37_26
// %bb.25:
	ldr	w9, [x8, #140]
	add	w9, w9, #1              // =1
	str	w9, [x8, #140]
.LBB37_26:
	mov	w2, #6
	mov	x0, x20
	mov	w1, w19
	bl	__remove_dirty_segment
.LBB37_27:
	mov	x0, x21
	bl	mutex_unlock
.LBB37_28:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end37:
	.size	locate_dirty_segment, .Lfunc_end37-locate_dirty_segment
                                        // -- End function
	.globl	f2fs_is_checkpointed_data // -- Begin function f2fs_is_checkpointed_data
	.p2align	2
	.type	f2fs_is_checkpointed_data,@function
f2fs_is_checkpointed_data:              // @f2fs_is_checkpointed_data
.Lf2fs_is_checkpointed_data$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	b.hs	.LBB38_2
.LBB38_1:
	mov	w19, #1
	b	.LBB38_28
.LBB38_2:
	ldr	x8, [x0, #136]
	ldr	w10, [x0, #3128]
	mov	w19, w1
	mov	x20, x0
	ldr	x8, [x8]
	cbz	w10, .LBB38_10
// %bb.3:
	ldr	x9, [x20]
	cmp	w10, #1                 // =1
	ldr	x9, [x9, #248]
	b.lt	.LBB38_9
// %bb.4:
	ldr	x11, [x20, #3136]
	add	x11, x11, #80           // =80
	b	.LBB38_6
.LBB38_5:                               //   in Loop: Header=BB38_6 Depth=1
	subs	x10, x10, #1            // =1
	add	x11, x11, #88           // =88
	b.eq	.LBB38_9
.LBB38_6:                               // =>This Inner Loop Header: Depth=1
	ldur	w12, [x11, #-4]
	cmp	w12, w19
	b.hi	.LBB38_5
// %bb.7:                               //   in Loop: Header=BB38_6 Depth=1
	ldr	w12, [x11]
	cmp	w12, w19
	b.lo	.LBB38_5
// %bb.8:
	ldur	x9, [x11, #-80]
.LBB38_9:
	cbz	x9, .LBB38_1
.LBB38_10:
	add	x21, x8, #72            // =72
	mov	x0, x21
	bl	down_read
	add	w8, w19, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB38_12
.LBB38_11:
	mov	w8, #-1
	b	.LBB38_24
.LBB38_12:
	ldr	w9, [x20, #3128]
	cbz	w9, .LBB38_20
// %bb.13:
	ldr	x8, [x20]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB38_19
// %bb.14:
	ldr	x10, [x20, #3136]
	add	x10, x10, #80           // =80
	b	.LBB38_16
.LBB38_15:                              //   in Loop: Header=BB38_16 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB38_19
.LBB38_16:                              // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w19
	b.hi	.LBB38_15
// %bb.17:                              //   in Loop: Header=BB38_16 Depth=1
	ldr	w11, [x10]
	cmp	w11, w19
	b.lo	.LBB38_15
// %bb.18:
	ldur	x8, [x10, #-80]
.LBB38_19:
	cbz	x8, .LBB38_11
.LBB38_20:
	ldr	x8, [x20, #136]
	cbz	x8, .LBB38_22
// %bb.21:
	add	x9, x8, #80             // =80
	b	.LBB38_23
.LBB38_22:
	ldr	x9, [x20, #16]
	add	x9, x9, #72             // =72
.LBB38_23:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x20, #1184]
	ldr	w8, [x8]
	sub	w9, w19, w9
	lsr	w9, w9, w10
	sub	w8, w9, w8
.LBB38_24:
	ldr	x10, [x20, #136]
	ldr	x9, [x10]
	ldr	x9, [x9, #120]
	cbz	x10, .LBB38_26
// %bb.25:
	add	x10, x10, #80           // =80
	b	.LBB38_27
.LBB38_26:
	ldr	x10, [x20, #16]
	add	x10, x10, #72           // =72
.LBB38_27:
	ldr	w10, [x10]
	ldr	w11, [x20, #1188]
	mov	w12, #40
	madd	x8, x8, x12, x9
	ldr	x8, [x8, #16]
	sub	w9, w19, w10
	sub	w10, w11, #1            // =1
	and	w9, w10, w9
	lsr	x10, x9, #3
	ldrb	w8, [x8, x10]
	mvn	w9, w9
	and	w9, w9, #0x7
	mov	x0, x21
	lsr	w8, w8, w9
	and	w19, w8, #0x1
	bl	up_read
.LBB38_28:
	mov	w0, w19
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end38:
	.size	f2fs_is_checkpointed_data, .Lfunc_end38-f2fs_is_checkpointed_data
                                        // -- End function
	.globl	f2fs_npages_for_summary_flush // -- Begin function f2fs_npages_for_summary_flush
	.p2align	2
	.type	f2fs_npages_for_summary_flush,@function
f2fs_npages_for_summary_flush:          // @f2fs_npages_for_summary_flush
.Lf2fs_npages_for_summary_flush$local:
// %bb.0:
	ldr	x8, [x0, #224]
	ldrb	w9, [x8, #176]
	cmp	w9, #1                  // =1
	b.ne	.LBB39_4
// %bb.1:
	ldrb	w10, [x8, #177]
	ldr	w9, [x0, #1188]
	cmp	w10, #1                 // =1
	b.ne	.LBB39_7
// %bb.2:
	ldrb	w10, [x8, #178]
	ldr	w11, [x0, #1188]
	cmp	w10, #1                 // =1
	add	w9, w11, w9
	b.eq	.LBB39_14
// %bb.3:
	tbnz	w1, #0, .LBB39_9
	b	.LBB39_12
.LBB39_4:
	tbz	w1, #0, .LBB39_10
// %bb.5:
	ldrb	w10, [x8, #177]
	ldrh	w9, [x8, #116]
	cmp	w10, #1                 // =1
	b.ne	.LBB39_8
// %bb.6:
	ldrb	w10, [x8, #178]
	ldr	w11, [x0, #1188]
	cmp	w10, #1                 // =1
	add	w9, w11, w9
	b.ne	.LBB39_9
	b	.LBB39_14
.LBB39_7:
	tbz	w1, #0, .LBB39_13
.LBB39_8:
	ldrb	w10, [x8, #178]
	ldrh	w11, [x8, #118]
	cmp	w10, #1                 // =1
	add	w9, w9, w11
	b.eq	.LBB39_14
.LBB39_9:
	ldrh	w8, [x8, #120]
	b	.LBB39_15
.LBB39_10:
	ldr	x9, [x0, #136]
	ldrb	w10, [x8, #177]
	ldr	x9, [x9, #24]
	cmp	w10, #1                 // =1
	ldrh	w9, [x9, #104]
	b.ne	.LBB39_13
// %bb.11:
	ldrb	w8, [x8, #178]
	ldr	w10, [x0, #1188]
	cmp	w8, #1                  // =1
	add	w9, w10, w9
	b.eq	.LBB39_14
.LBB39_12:
	ldr	x8, [x0, #136]
	ldr	x8, [x8, #24]
	ldrh	w8, [x8, #344]
	b	.LBB39_15
.LBB39_13:
	ldr	x10, [x0, #136]
	ldrb	w8, [x8, #178]
	ldr	x10, [x10, #24]
	cmp	w8, #1                  // =1
	ldrh	w10, [x10, #224]
	add	w9, w9, w10
	b.ne	.LBB39_12
.LBB39_14:
	ldr	w8, [x0, #1188]
.LBB39_15:
	add	w8, w8, w9
	sub	w10, w8, #439           // =439
	mov	w9, #2
	cmp	w10, #585               // =585
	cinc	w9, w9, hs
	cmp	w8, #440                // =440
	csinc	w0, w9, wzr, ge
	ret
.Lfunc_end39:
	.size	f2fs_npages_for_summary_flush, .Lfunc_end39-f2fs_npages_for_summary_flush
                                        // -- End function
	.globl	f2fs_get_sum_page       // -- Begin function f2fs_get_sum_page
	.p2align	2
	.type	f2fs_get_sum_page,@function
f2fs_get_sum_page:                      // @f2fs_get_sum_page
.Lf2fs_get_sum_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x8, [x0, #224]
	mov	x29, sp
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB40_2
// %bb.1:
	ldr	x8, [x0, #136]
	ldr	w8, [x8, #88]
	add	w1, w8, w1
	bl	f2fs_get_meta_page_retry
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB40_2:
	mov	x0, #-5
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end40:
	.size	f2fs_get_sum_page, .Lfunc_end40-f2fs_get_sum_page
                                        // -- End function
	.globl	f2fs_update_meta_page   // -- Begin function f2fs_update_meta_page
	.p2align	2
	.type	f2fs_update_meta_page,@function
f2fs_update_meta_page:                  // @f2fs_update_meta_page
.Lf2fs_update_meta_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x19, x1
	mov	w1, w2
	mov	x29, sp
	bl	f2fs_grab_meta_page
	mov	x8, #134217728
	movk	x8, #64, lsl #32
	add	x8, x8, x0, lsl #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	mov	x20, x0
	add	x0, x8, x9
	mov	w2, #4096
	mov	x1, x19
	bl	memcpy
	mov	x0, x20
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end41:
	.size	f2fs_update_meta_page, .Lfunc_end41-f2fs_update_meta_page
                                        // -- End function
	.globl	write_sum_page          // -- Begin function write_sum_page
	.p2align	2
	.type	write_sum_page,@function
write_sum_page:                         // @write_sum_page
.Lwrite_sum_page$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x19, x1
	mov	w1, w2
	mov	x29, sp
	bl	f2fs_grab_meta_page
	mov	x8, #134217728
	movk	x8, #64, lsl #32
	add	x8, x8, x0, lsl #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	mov	x20, x0
	add	x0, x8, x9
	mov	w2, #4096
	mov	x1, x19
	bl	memcpy
	mov	x0, x20
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end42:
	.size	write_sum_page, .Lfunc_end42-write_sum_page
                                        // -- End function
	.globl	f2fs_segment_has_free_slot // -- Begin function f2fs_segment_has_free_slot
	.p2align	2
	.type	f2fs_segment_has_free_slot,@function
f2fs_segment_has_free_slot:             // @f2fs_segment_has_free_slot
.Lf2fs_segment_has_free_slot$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	w10, #40
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x2, xzr
	ldr	x8, [x8]
	mov	x29, sp
	ldr	x9, [x8, #120]
	ldr	x8, [x8, #48]
	umaddl	x9, w1, w10, x9
	ldp	x9, x10, [x9, #8]
	ldr	x11, [x10]
	ldr	x12, [x9]
	orr	x11, x12, x11
	str	x11, [x8]
	ldr	x11, [x10, #8]
	ldr	x12, [x9, #8]
	orr	x11, x12, x11
	str	x11, [x8, #8]
	ldr	x11, [x10, #16]
	ldr	x12, [x9, #16]
	orr	x11, x12, x11
	str	x11, [x8, #16]
	ldr	x11, [x10, #24]
	ldr	x12, [x9, #24]
	orr	x11, x12, x11
	str	x11, [x8, #24]
	ldr	x11, [x10, #32]
	ldr	x12, [x9, #32]
	orr	x11, x12, x11
	str	x11, [x8, #32]
	ldr	x11, [x10, #40]
	ldr	x12, [x9, #40]
	orr	x11, x12, x11
	str	x11, [x8, #40]
	ldr	x11, [x10, #48]
	ldr	x12, [x9, #48]
	orr	x11, x12, x11
	str	x11, [x8, #48]
	ldr	x10, [x10, #56]
	ldr	x9, [x9, #56]
	orr	x9, x9, x10
	str	x9, [x8, #56]
	ldr	w19, [x0, #1188]
	mov	x0, x8
	mov	x1, x19
	bl	__find_rev_next_zero_bit
	cmp	w19, w0
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	cset	w0, hi
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end43:
	.size	f2fs_segment_has_free_slot, .Lfunc_end43-f2fs_segment_has_free_slot
                                        // -- End function
	.globl	f2fs_init_inmem_curseg  // -- Begin function f2fs_init_inmem_curseg
	.p2align	2
	.type	f2fs_init_inmem_curseg,@function
f2fs_init_inmem_curseg:                 // @f2fs_init_inmem_curseg
.Lf2fs_init_inmem_curseg$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldrb	w8, [x0, #1920]
	mov	x29, sp
	cbz	w8, .LBB44_5
// %bb.1:
	ldr	x8, [x0, #136]
	mov	x19, x0
	ldr	x20, [x8, #24]
	add	x0, x8, #32             // =32
	bl	down_read
	add	x20, x20, #840          // =840
	mov	x0, x20
	bl	mutex_lock
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
	add	x0, x8, #72             // =72
	bl	down_write
	ldr	x8, [x19, #136]
	mov	w22, #2
	mov	w1, #7
	mov	w2, #1
	ldr	x21, [x8, #24]
	mov	x0, x19
	mov	x3, xzr
	strh	w22, [x21, #938]
	bl	get_ssr_segment
	cbz	w0, .LBB44_3
// %bb.2:
	ldr	x8, [x19, #136]
	ldr	w9, [x21, #952]
	mov	w10, #40
	mov	w1, #7
	ldr	x8, [x8]
	mul	x9, x9, x10
	mov	x0, x19
	ldr	x8, [x8, #120]
	ldrh	w8, [x8, x9]
	and	w8, w8, #0x3f
	strh	w8, [x21, #938]
	bl	change_curseg
	b	.LBB44_4
.LBB44_3:
	mov	w1, #7
	mov	w2, #1
	mov	x0, x19
	strh	w22, [x21, #938]
	bl	new_curseg
.LBB44_4:
	ldrb	w8, [x21, #936]
	add	x8, x19, x8, lsl #2
	ldr	w9, [x8, #2144]
	add	w9, w9, #1              // =1
	str	w9, [x8, #2144]
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
	add	x0, x8, #72             // =72
	bl	up_write
	mov	x0, x20
	bl	mutex_unlock
	ldr	x8, [x19, #136]
	add	x0, x8, #32             // =32
	bl	up_read
.LBB44_5:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end44:
	.size	f2fs_init_inmem_curseg, .Lfunc_end44-f2fs_init_inmem_curseg
                                        // -- End function
	.globl	f2fs_save_inmem_curseg  // -- Begin function f2fs_save_inmem_curseg
	.p2align	2
	.type	f2fs_save_inmem_curseg,@function
f2fs_save_inmem_curseg:                 // @f2fs_save_inmem_curseg
.Lf2fs_save_inmem_curseg$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	mov	w1, #6
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x19, x0
	bl	__f2fs_save_inmem_curseg
	ldrb	w8, [x19, #1920]
	cbz	w8, .LBB45_2
// %bb.1:
	mov	w1, #7
	mov	x0, x19
	bl	__f2fs_save_inmem_curseg
.LBB45_2:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end45:
	.size	f2fs_save_inmem_curseg, .Lfunc_end45-f2fs_save_inmem_curseg
                                        // -- End function
	.p2align	2               // -- Begin function __f2fs_save_inmem_curseg
	.type	__f2fs_save_inmem_curseg,@function
__f2fs_save_inmem_curseg:               // @__f2fs_save_inmem_curseg
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	x20, x0
	mov	x29, sp
	mov	w21, w1
	ldr	x22, [x8, #24]
	mov	w8, #120
	smaddl	x19, w1, w8, x22
	mov	x0, x19
	bl	mutex_lock
	ldrb	w8, [x19, #116]
	cbz	w8, .LBB46_4
// %bb.1:
	sxtw	x8, w21
	mov	w9, #120
	madd	x21, x8, x9, x22
	ldr	w9, [x21, #100]!
	ldr	x10, [x20, #136]
	mov	w12, #40
	mul	x12, x9, x12
	ldr	x11, [x10]
	ldr	x11, [x11, #120]
	ldrh	w11, [x11, x12]
	tst	w11, #0xffc0
	b.eq	.LBB46_3
// %bb.2:
	mov	w11, #120
	ldr	w10, [x10, #88]
	madd	x8, x8, x11, x22
	ldr	x21, [x8, #32]
	mov	x0, x20
	add	w1, w10, w9
	bl	f2fs_grab_meta_page
	mov	x8, #134217728
	movk	x8, #64, lsl #32
	add	x8, x8, x0, lsl #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	mov	x20, x0
	add	x0, x8, x9
	mov	w2, #4096
	mov	x1, x21
	bl	memcpy
	mov	x0, x20
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
	b	.LBB46_4
.LBB46_3:
	ldr	x8, [x10, #16]
	add	x0, x8, #80             // =80
	bl	mutex_lock
	ldr	w1, [x21]
	mov	w2, #1
	mov	x0, x20
	bl	__set_test_and_free
	ldr	x8, [x20, #136]
	ldr	x8, [x8, #16]
	add	x0, x8, #80             // =80
	bl	mutex_unlock
.LBB46_4:
	mov	x0, x19
	bl	mutex_unlock
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end46:
	.size	__f2fs_save_inmem_curseg, .Lfunc_end46-__f2fs_save_inmem_curseg
                                        // -- End function
	.globl	f2fs_restore_inmem_curseg // -- Begin function f2fs_restore_inmem_curseg
	.p2align	2
	.type	f2fs_restore_inmem_curseg,@function
f2fs_restore_inmem_curseg:              // @f2fs_restore_inmem_curseg
.Lf2fs_restore_inmem_curseg$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	mov	w1, #6
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x19, x0
	bl	__f2fs_restore_inmem_curseg
	ldrb	w8, [x19, #1920]
	cbz	w8, .LBB47_2
// %bb.1:
	mov	w1, #7
	mov	x0, x19
	bl	__f2fs_restore_inmem_curseg
.LBB47_2:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end47:
	.size	f2fs_restore_inmem_curseg, .Lfunc_end47-f2fs_restore_inmem_curseg
                                        // -- End function
	.p2align	2               // -- Begin function __f2fs_restore_inmem_curseg
	.type	__f2fs_restore_inmem_curseg,@function
__f2fs_restore_inmem_curseg:            // @__f2fs_restore_inmem_curseg
// %bb.0:
	sub	sp, sp, #80             // =80
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	x20, x0
	add	x29, sp, #16            // =16
	mov	w21, w1
	ldr	x22, [x8, #24]
	mov	w8, #120
	smaddl	x19, w1, w8, x22
	mov	x0, x19
	bl	mutex_lock
	ldrb	w8, [x19, #116]
	cbz	w8, .LBB48_11
// %bb.1:
	sxtw	x8, w21
	mov	w9, #120
	madd	x21, x8, x9, x22
	ldr	w9, [x21, #100]!
	ldr	x8, [x20, #136]
	mov	w11, #40
	mul	x9, x9, x11
	ldr	x10, [x8]
	ldr	x10, [x10, #120]
	ldrh	w9, [x10, x9]
	tst	w9, #0xffc0
	b.ne	.LBB48_11
// %bb.2:
	ldr	x8, [x8, #16]
	add	x0, x8, #80             // =80
	bl	mutex_lock
	ldr	x8, [x20, #136]
	ldr	w24, [x21]
	ldr	x22, [x8, #8]
	cmn	w24, #1                 // =1
	b.eq	.LBB48_4
// %bb.3:
	ldr	w8, [x20, #1192]
	udiv	w23, w24, w8
	b	.LBB48_5
.LBB48_4:
	mov	w23, #-1
.LBB48_5:
	add	x21, x22, #12           // =12
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	x8, [x22, #16]
	ubfx	x9, x24, #6, #26
	ldr	x10, [x8, w9, uxtw #3]
	str	x10, [sp, #8]
	ldr	x11, [sp, #8]
	mov	w10, #1
	lsl	x10, x10, x24
	tst	x11, x10
	b.ne	.LBB48_10
// %bb.6:
	add	x8, x8, x9, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x11, x9, x10
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tst	x9, x10
	b.ne	.LBB48_10
// %bb.7:
	ldr	w9, [x22, #4]
	ldr	x8, [x22, #24]
	lsr	w10, w23, #6
	sub	w9, w9, #1              // =1
	str	w9, [x22, #4]
	ldr	x9, [x8, w10, uxtw #3]
	str	x9, [sp, #8]
	ldr	x11, [sp, #8]
	mov	w9, #1
	lsl	x9, x9, x23
	tst	x11, x9
	b.ne	.LBB48_10
// %bb.8:
	add	x8, x8, x10, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	orr	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tst	x10, x9
	b.ne	.LBB48_10
// %bb.9:
	ldr	w8, [x22, #8]
	sub	w8, w8, #1              // =1
	str	w8, [x22, #8]
.LBB48_10:
	mov	x0, x21
	bl	_raw_spin_unlock
	ldr	x8, [x20, #136]
	ldr	x8, [x8, #16]
	add	x0, x8, #80             // =80
	bl	mutex_unlock
.LBB48_11:
	mov	x0, x19
	bl	mutex_unlock
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #80             // =80
	ret
.Lfunc_end48:
	.size	__f2fs_restore_inmem_curseg, .Lfunc_end48-__f2fs_restore_inmem_curseg
                                        // -- End function
	.globl	f2fs_allocate_segment_for_resize // -- Begin function f2fs_allocate_segment_for_resize
	.p2align	2
	.type	f2fs_allocate_segment_for_resize,@function
f2fs_allocate_segment_for_resize:       // @f2fs_allocate_segment_for_resize
.Lf2fs_allocate_segment_for_resize$local:
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x26, x25, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	w20, w1
	mov	x19, x0
	mov	x29, sp
	ldr	x25, [x8, #24]
	add	x0, x8, #32             // =32
	mov	w23, w3
	mov	w24, w2
	sxtw	x26, w20
	bl	down_read
	mov	w22, #120
	smaddl	x21, w20, w22, x25
	mov	x0, x21
	bl	mutex_lock
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
	add	x0, x8, #72             // =72
	bl	down_write
	ldr	x8, [x19, #136]
	ldr	x9, [x8, #24]
	smaddl	x9, w20, w22, x9
	ldr	w22, [x9, #100]
	cmp	w22, w24
	b.lo	.LBB49_14
// %bb.1:
	cmp	w22, w23
	b.hi	.LBB49_14
// %bb.2:
	ldr	w9, [x19, #1192]
	ldr	w14, [x19, #1188]
	ldrsw	x13, [x19, #1284]
	ldr	w10, [x19, #1184]
	ldrsw	x12, [x19, #1272]
	ldrsw	x11, [x19, #1296]
	ldr	w15, [x19, #1504]
	cmp	w15, #1                 // =1
	b.eq	.LBB49_12
// %bb.3:
	ldr	w15, [x19, #1988]
	cmp	w15, #5                 // =5
	b.ne	.LBB49_5
// %bb.4:
	ldrb	w15, [x19, #1232]
	tbz	w15, #3, .LBB49_10
.LBB49_5:
	ldr	x15, [x19, #80]
	tbnz	w15, #8, .LBB49_10
// %bb.6:
	mul	w14, w14, w9
	sub	x14, x14, #1            // =1
	add	x13, x14, x13
	add	x12, x14, x12
	add	x11, x14, x11
	ldr	x14, [x8, #8]
	asr	x13, x13, x10
	asr	x12, x12, x10
	asr	x10, x11, x10
	ldr	w15, [x8, #156]
	ldr	w11, [x8, #100]
	udiv	w13, w13, w9
	udiv	w12, w12, w9
	ldr	w8, [x14, #8]
	udiv	w10, w10, w9
	add	w12, w13, w12, lsl #1
	add	w10, w12, w10
	cmn	w11, #1                 // =1
	add	w10, w10, w15
	b.eq	.LBB49_8
// %bb.7:
	udiv	w9, w11, w9
	b	.LBB49_9
.LBB49_8:
	mov	w9, #-1
.LBB49_9:
	add	w9, w10, w9
	cmp	w8, w9
	b.hi	.LBB49_12
.LBB49_10:
	mov	w2, #1
	mov	x0, x19
	mov	w1, w20
	mov	x3, xzr
	bl	get_ssr_segment
	cbz	w0, .LBB49_12
// %bb.11:
	mov	x0, x19
	mov	w1, w20
	bl	change_curseg
	b	.LBB49_13
.LBB49_12:
	mov	w2, #1
	mov	x0, x19
	mov	w1, w20
	bl	new_curseg
.LBB49_13:
	mov	w8, #120
	madd	x8, x26, x8, x25
	ldrb	w8, [x8, #96]
	mov	x0, x19
	mov	w1, w22
	add	x8, x19, x8, lsl #2
	ldr	w9, [x8, #2144]
	add	w9, w9, #1              // =1
	str	w9, [x8, #2144]
	bl	locate_dirty_segment
	ldr	x8, [x19, #136]
.LBB49_14:
	ldr	x8, [x8]
	add	x0, x8, #72             // =72
	bl	up_write
	mov	w8, #120
	madd	x8, x26, x8, x25
	ldr	w4, [x8, #100]
	cmp	w22, w4
	b.eq	.LBB49_16
// %bb.15:
	adrp	x1, .L.str.6
	add	x1, x1, :lo12:.L.str.6
	mov	x0, x19
	mov	w2, w20
	mov	w3, w22
	bl	f2fs_printk
.LBB49_16:
	mov	x0, x21
	bl	mutex_unlock
	ldr	x8, [x19, #136]
	add	x0, x8, #32             // =32
	bl	up_read
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.Lfunc_end49:
	.size	f2fs_allocate_segment_for_resize, .Lfunc_end49-f2fs_allocate_segment_for_resize
                                        // -- End function
	.p2align	2               // -- Begin function get_ssr_segment
	.type	get_ssr_segment,@function
get_ssr_segment:                        // @get_ssr_segment
// %bb.0:
	sub	sp, sp, #112            // =112
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	str	x27, [sp, #32]          // 8-byte Folded Spill
	stp	x26, x25, [sp, #48]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #64]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #80]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #96]     // 16-byte Folded Spill
	str	x8, [sp, #8]
	ldr	x8, [x0, #136]
	mov	x20, x3
	mov	w23, w1
	mov	x19, x0
	ldp	x9, x24, [x8, #16]
	mov	w8, #-1
	mov	w21, w2
	add	x29, sp, #16            // =16
	ldr	x26, [x9]
	str	w8, [sp, #4]
	mov	w8, #120
	smaddl	x8, w1, w8, x24
	ldrh	w22, [x8, #98]
	cmp	w22, #6                 // =6
	b.hs	.LBB50_17
.LBB50_1:
	ldr	x8, [x26]
	add	x1, sp, #4              // =4
	mov	x0, x19
	mov	w2, wzr
	mov	w3, w22
	mov	w4, w21
	mov	x5, x20
	sxtw	x25, w23
	blr	x8
	cbz	w0, .LBB50_13
// %bb.2:
	sub	w8, w22, #3             // =3
	cmp	w8, #2                  // =2
	b.hi	.LBB50_4
// %bb.3:
	cmp	w22, #3                 // =3
	mov	w9, #3
	mov	w10, #5
	cset	w8, hi
	csel	w23, w10, w9, hi
	b	.LBB50_5
.LBB50_4:
	cmp	w22, #0                 // =0
	cset	w8, ne
	lsl	w23, w8, #1
.LBB50_5:
	cmp	w8, #0                  // =0
	mov	w8, #-1
	cneg	w27, w8, eq
	cmp	w23, w22
	b.eq	.LBB50_7
// %bb.6:
	ldr	x8, [x26]
	add	x1, sp, #4              // =4
	mov	x0, x19
	mov	w2, wzr
	mov	w3, w23
	mov	w4, w21
	mov	x5, x20
	blr	x8
	cbz	w0, .LBB50_13
.LBB50_7:
	add	w23, w23, w27
	cmp	w23, w22
	b.eq	.LBB50_9
// %bb.8:
	ldr	x8, [x26]
	add	x1, sp, #4              // =4
	mov	x0, x19
	mov	w2, wzr
	mov	w3, w23
	mov	w4, w21
	mov	x5, x20
	blr	x8
	cbz	w0, .LBB50_13
.LBB50_9:
	add	w3, w23, w27
	cmp	w3, w22
	b.eq	.LBB50_11
// %bb.10:
	ldr	x8, [x26]
	add	x1, sp, #4              // =4
	mov	x0, x19
	mov	w2, wzr
	mov	w4, w21
	mov	x5, x20
	blr	x8
	cbz	w0, .LBB50_13
.LBB50_11:
	ldr	x8, [x19, #80]
	tbnz	w8, #8, .LBB50_25
// %bb.12:
	mov	w0, wzr
	b	.LBB50_15
.LBB50_13:
	ldr	w8, [sp, #4]
	mov	w9, #120
	madd	x9, x25, x9, x24
	str	w8, [x9, #112]
.LBB50_14:
	mov	w0, #1
.LBB50_15:
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB50_32
// %bb.16:
	ldp	x20, x19, [sp, #96]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #80]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #64]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #48]     // 16-byte Folded Reload
	ldr	x27, [sp, #32]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #112            // =112
	ret
.LBB50_17:
	bl	print_bdev_access_info
	adrp	x25, f2fs_dclient
	ldr	x0, [x25, :lo12:f2fs_dclient]
	cbz	x0, .LBB50_20
// %bb.18:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB50_20
// %bb.19:
	ldr	x0, [x25, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.sanity_check_seg_type
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.sanity_check_seg_type
	mov	w3, #46
	bl	dsm_client_record
	ldr	x0, [x25, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB50_20:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.h"; .popsection; .long 14472b - 14470b; .short 46; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB50_1
// %bb.21:
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
	cbnz	w8, .LBB50_1
// %bb.22:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB50_1
// %bb.23:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB50_1
// %bb.24:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB50_1
.LBB50_25:
	ldr	x8, [x19, #136]
	ldr	x22, [x8, #16]
	add	x20, x22, #80           // =80
	mov	x0, x20
	bl	mutex_lock
	ldr	x8, [x19, #136]
	ldr	x0, [x22, #56]
	mov	x2, xzr
	ldr	w1, [x8, #96]
	bl	find_next_bit
	ldr	x9, [x19, #136]
	mov	x21, x0
	ldr	w8, [x9, #96]
	cmp	w8, w21
	b.ls	.LBB50_31
// %bb.26:
	ldr	x9, [x9]
	and	x10, x21, #0xffffffff
	mov	w11, #40
	mul	x10, x10, x11
	ldr	x9, [x9, #120]
	ldr	w9, [x9, x10]
	tst	w9, #0x3ffffc0
	b.eq	.LBB50_30
// %bb.27:
	mov	w23, #40
.LBB50_28:                              // =>This Inner Loop Header: Depth=1
	ldr	x0, [x22, #56]
	mov	w1, w8
	add	w2, w21, #1             // =1
	bl	find_next_bit
	ldr	x9, [x19, #136]
	mov	x21, x0
	ldr	w8, [x9, #96]
	cmp	w8, w21
	b.ls	.LBB50_31
// %bb.29:                              //   in Loop: Header=BB50_28 Depth=1
	ldr	x9, [x9]
	and	x10, x21, #0xffffffff
	mul	x10, x10, x23
	ldr	x9, [x9, #120]
	ldr	w9, [x9, x10]
	tst	w9, #0x3ffffc0
	b.ne	.LBB50_28
.LBB50_30:
	mov	x0, x20
	bl	mutex_unlock
	mov	w8, #120
	madd	x8, x25, x8, x24
	str	w21, [sp, #4]
	str	w21, [x8, #112]
	b	.LBB50_14
.LBB50_31:
	mov	x0, x20
	bl	mutex_unlock
	mov	w8, #-1
	mov	w0, wzr
	str	w8, [sp, #4]
	b	.LBB50_15
.LBB50_32:
	bl	__stack_chk_fail
.Lfunc_end50:
	.size	get_ssr_segment, .Lfunc_end50-get_ssr_segment
                                        // -- End function
	.p2align	2               // -- Begin function change_curseg
	.type	change_curseg,@function
change_curseg:                          // @change_curseg
// %bb.0:
	sub	sp, sp, #128            // =128
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #48]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #64]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #80]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #96]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #112]    // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	w9, #120
	mov	w21, w1
	mov	x25, #2097152
	ldp	x28, x26, [x8, #16]
	add	x29, sp, #32            // =32
	mov	x19, x0
	movk	x25, #1, lsl #32
	smaddl	x24, w1, w9, x26
	ldr	x22, [x24, #32]!
	ldr	w8, [x8, #88]
	mov	x27, x24
	ldr	w20, [x24, #80]
	ldr	w9, [x27, #68]!
	add	w1, w9, w8
	bl	f2fs_grab_meta_page
	add	x8, x0, x25
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	mov	x23, x0
	add	x0, x8, x9
	mov	w2, #4096
	mov	x1, x22
	bl	memcpy
	mov	x0, x23
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_put_page
	ldr	x8, [x19, #136]
	cmn	w20, #1                 // =1
	ldr	x23, [x8, #8]
	stp	x28, x26, [sp, #8]      // 16-byte Folded Spill
	b.eq	.LBB51_2
// %bb.1:
	ldr	w8, [x19, #1192]
	udiv	w26, w20, w8
	b	.LBB51_3
.LBB51_2:
	mov	w26, #-1
.LBB51_3:
	add	x22, x23, #12           // =12
	mov	x0, x22
	bl	_raw_spin_lock
	ldr	x8, [x23, #16]
	lsr	x25, x20, #6
	mov	w10, #1
	lsl	x28, x10, x20
	ldr	x9, [x8, x25, lsl #3]
	stur	x9, [x29, #-8]
	ldur	x9, [x29, #-8]
	tst	x9, x28
	b.ne	.LBB51_8
// %bb.4:
	add	x8, x8, x25, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x10, x9, x28
	stlxr	w11, x10, [x8]
	cbnz	w11, 1b
	dmb ish
	//NO_APP
	tst	x9, x28
	b.ne	.LBB51_8
// %bb.5:
	ldr	w9, [x23, #4]
	ldr	x8, [x23, #24]
	lsr	w10, w26, #6
	sub	w9, w9, #1              // =1
	str	w9, [x23, #4]
	ldr	x9, [x8, w10, uxtw #3]
	stur	x9, [x29, #-8]
	ldur	x11, [x29, #-8]
	mov	w9, #1
	lsl	x9, x9, x26
	tst	x11, x9
	b.ne	.LBB51_8
// %bb.6:
	add	x8, x8, x10, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	orr	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tst	x10, x9
	b.ne	.LBB51_8
// %bb.7:
	ldr	w8, [x23, #8]
	sub	w8, w8, #1              // =1
	str	w8, [x23, #8]
.LBB51_8:
	mov	x0, x22
	sxtw	x23, w21
	bl	_raw_spin_unlock
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	add	x22, x8, #80            // =80
	mov	x0, x22
	bl	mutex_lock
	ldr	x8, [x19, #136]
	ldr	x8, [x8, #16]
	ldr	x9, [x8, #64]
	ldr	x10, [x9, x25, lsl #3]
	stur	x10, [x29, #-8]
	ldur	x10, [x29, #-8]
	tst	x10, x28
	b.eq	.LBB51_11
// %bb.9:
	add	x9, x9, x25, lsl #3
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x9]
1:	ldxr	x10, [x9]
	bic	x11, x10, x28
	stlxr	w12, x11, [x9]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tst	x10, x28
	b.eq	.LBB51_11
// %bb.10:
	ldr	w9, [x8, #140]
	sub	w9, w9, #1              // =1
	str	w9, [x8, #140]
.LBB51_11:
	mov	w2, #6
	mov	x0, x19
	mov	w1, w20
	bl	__remove_dirty_segment
	mov	x0, x22
	bl	mutex_unlock
	mov	w2, #1
	mov	x0, x19
	mov	w1, w21
	mov	w22, #1
	bl	reset_curseg
	ldr	x9, [sp, #16]           // 8-byte Folded Reload
	mov	w8, #120
	mov	w11, #40
	mov	x2, xzr
	madd	x21, x23, x8, x9
	strb	w22, [x21, #96]
	ldr	x8, [x19, #136]
	ldr	w9, [x27]
	ldr	x8, [x8]
	ldr	x10, [x8, #120]
	ldr	x0, [x8, #48]
	madd	x9, x9, x11, x10
	ldp	x9, x10, [x9, #8]
	ldr	x11, [x10]
	ldr	x12, [x9]
	orr	x8, x12, x11
	str	x8, [x0]
	ldr	x8, [x10, #8]
	ldr	x11, [x9, #8]
	orr	x8, x11, x8
	str	x8, [x0, #8]
	ldr	x8, [x10, #16]
	ldr	x11, [x9, #16]
	orr	x8, x11, x8
	str	x8, [x0, #16]
	ldr	x8, [x10, #24]
	ldr	x11, [x9, #24]
	orr	x8, x11, x8
	str	x8, [x0, #24]
	ldr	x8, [x10, #32]
	ldr	x11, [x9, #32]
	orr	x8, x11, x8
	str	x8, [x0, #32]
	ldr	x8, [x10, #40]
	ldr	x11, [x9, #40]
	orr	x8, x11, x8
	str	x8, [x0, #40]
	ldr	x8, [x10, #48]
	ldr	x11, [x9, #48]
	orr	x8, x11, x8
	str	x8, [x0, #48]
	ldr	x8, [x10, #56]
	ldr	x9, [x9, #56]
	orr	x8, x9, x8
	str	x8, [x0, #56]
	ldr	w1, [x19, #1188]
	bl	__find_rev_next_zero_bit
	strh	w0, [x21, #104]
	ldr	x8, [x19, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB51_13
// %bb.12:
	ldr	x8, [x19, #136]
	mov	x0, x19
	ldr	w8, [x8, #88]
	add	w1, w8, w20
	bl	f2fs_get_meta_page_retry
	cmn	x0, #4095               // =4095
	b.lo	.LBB51_14
.LBB51_13:
	ldr	x0, [x24]
	mov	w2, #3584
	mov	w1, wzr
	bl	memset
	b	.LBB51_15
.LBB51_14:
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	mov	x19, x0
	add	x8, x0, x8
	ldr	x0, [x24]
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	add	x1, x8, x9
	mov	w2, #3584
	bl	memcpy
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_put_page
.LBB51_15:
	ldp	x20, x19, [sp, #112]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #96]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #80]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #64]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #48]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #128            // =128
	ret
.Lfunc_end51:
	.size	change_curseg, .Lfunc_end51-change_curseg
                                        // -- End function
	.p2align	2               // -- Begin function new_curseg
	.type	new_curseg,@function
new_curseg:                             // @new_curseg
// %bb.0:
	sub	sp, sp, #160            // =160
	stp	x29, x30, [sp, #64]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #80]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #96]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #112]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #128]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #144]    // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	w9, #120
                                        // kill: def $w1 killed $w1 def $x1
	add	x29, sp, #64            // =64
	mov	w23, w2
	ldr	x22, [x8, #24]
	mov	x20, x0
	sxtw	x25, w1
	stur	x1, [x29, #-24]         // 8-byte Folded Spill
	smaddl	x9, w1, w9, x22
	ldrb	w10, [x9, #116]
	ldrh	w19, [x9, #98]
	str	x22, [sp, #32]          // 8-byte Folded Spill
	cbz	w10, .LBB52_2
// %bb.1:
	mov	w9, #120
	mul	x24, x25, x9
	add	x9, x22, x24
	ldr	w10, [x9, #100]
	ldr	w8, [x8, #88]
	ldr	x21, [x9, #32]
	mov	x0, x20
	add	w1, w8, w10
	bl	f2fs_grab_meta_page
	mov	x8, #134217728
	movk	x8, #64, lsl #32
	add	x8, x8, x0, lsl #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	mov	x22, x0
	add	x0, x8, x9
	mov	w2, #4096
	mov	x1, x21
	bl	memcpy
	mov	x0, x22
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
	ldr	x8, [x20, #136]
	ldr	x22, [x8, #24]
	add	x8, x22, x24
	ldrh	w24, [x8, #98]
	b	.LBB52_3
.LBB52_2:
	mov	w24, w19
.LBB52_3:
	ldrb	w8, [x20, #1424]
	sub	w9, w19, #1             // =1
	cmp	w9, #1                  // =1
	cset	w19, hi
	tst	w8, #0x8
	cset	w26, eq
	cmp	w24, #6                 // =6
	b.hs	.LBB52_80
.LBB52_4:
	ldr	w8, [x20, #1192]
	cmp	w8, #2                  // =2
	b.lo	.LBB52_6
.LBB52_5:
	mov	w9, #120
	madd	x9, x25, x9, x22
	ldr	w24, [x9, #100]
	b	.LBB52_14
.LBB52_6:
	mov	w9, #120
	madd	x9, x25, x9, x22
	ldrb	w9, [x9, #116]
	cbz	w9, .LBB52_13
// %bb.7:
	ldr	x9, [x20, #80]
	tbnz	w9, #8, .LBB52_13
// %bb.8:
	ldrb	w9, [x20, #1424]
	tbz	w9, #3, .LBB52_11
// %bb.9:
	cmp	w24, #6                 // =6
	b.hs	.LBB52_11
// %bb.10:
	mov	w9, #57
	lsr	w9, w9, w24
	tbnz	w9, #0, .LBB52_13
.LBB52_11:
	ldr	x9, [x20, #136]
	ldr	x9, [x9]
	ldr	w24, [x9, #196]
	cbnz	w24, .LBB52_14
// %bb.12:
	ldr	w9, [x20, #1488]
	cmp	w9, #1                  // =1
	b.ne	.LBB52_5
.LBB52_13:
	mov	w24, wzr
.LBB52_14:
	ldr	x9, [x20, #136]
	str	x25, [sp, #24]          // 8-byte Folded Spill
	ldr	w22, [x20, #1200]
	ldr	w28, [x20, #1196]
	ldr	x25, [x9, #8]
	cmn	w24, #1                 // =1
	b.eq	.LBB52_17
// %bb.15:
	udiv	w21, w24, w8
	cmn	w21, #1                 // =1
	b.eq	.LBB52_18
// %bb.16:
	udiv	w8, w21, w28
	b	.LBB52_19
.LBB52_17:
	mov	w21, #-1
.LBB52_18:
	mov	w8, #-1
.LBB52_19:
	add	x0, x25, #12            // =12
	stur	w8, [x29, #-12]         // 4-byte Folded Spill
	str	x0, [sp, #16]           // 8-byte Folded Spill
	bl	_raw_spin_lock
	tbnz	w23, #0, .LBB52_25
// %bb.20:
	ldr	w9, [x20, #1192]
	add	w8, w24, #1             // =1
	udiv	w10, w8, w9
	msub	w10, w10, w9, w8
	cbz	w10, .LBB52_25
// %bb.21:
	ldr	x0, [x25, #16]
	add	w23, w21, #1            // =1
	mul	w1, w9, w23
	mov	w2, w8
	bl	find_next_zero_bit
	ldr	w8, [x20, #1192]
	mov	x27, x0
	mul	w8, w8, w23
	cmp	w8, w27
	b.ls	.LBB52_25
.LBB52_22:
	ldr	x8, [x25, #16]
	add	w9, w27, #63            // =63
	cmp	w27, #0                 // =0
	csel	w9, w9, w27, lt
	asr	w9, w9, #6
	ldr	x8, [x8, w9, sxtw #3]
	ldp	x23, x22, [sp, #24]     // 16-byte Folded Reload
	mov	w9, #1
	lsl	x19, x9, x27
	tst	x19, x8
	b.ne	.LBB52_88
.LBB52_23:
	ldr	x8, [x20, #136]
	cmn	w27, #1                 // =1
	ldr	x8, [x8, #8]
	b.eq	.LBB52_75
// %bb.24:
	ldr	w9, [x20, #1192]
	udiv	w9, w27, w9
	b	.LBB52_76
.LBB52_25:
	udiv	w8, w22, w28
	mov	w24, wzr
	and	w23, w19, w26
	stur	w8, [x29, #-16]         // 4-byte Folded Spill
	add	x8, x20, #3408          // =3408
	mov	w19, #1
	mov	w26, w21
	str	x8, [sp, #8]            // 8-byte Folded Spill
	b	.LBB52_27
.LBB52_26:                              //   in Loop: Header=BB52_27 Depth=1
	mul	w8, w10, w9
	sub	w21, w8, #1             // =1
.LBB52_27:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB52_33 Depth 2
	ldr	x0, [x25, #24]
	mov	w2, w21
	mov	x1, x22
	bl	find_next_zero_bit
	mov	x28, x0
	cmp	w22, w28
	b.hi	.LBB52_31
// %bb.28:                              //   in Loop: Header=BB52_27 Depth=1
	tbz	w23, #0, .LBB52_30
// %bb.29:                              //   in Loop: Header=BB52_27 Depth=1
	sub	w26, w21, #1            // =1
	mov	w24, #1
	b	.LBB52_32
.LBB52_30:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	x0, [x25, #24]
	mov	x1, x22
	mov	x2, xzr
	bl	find_next_zero_bit
	mov	x28, x0
	cmp	w22, w28
	b.ls	.LBB52_39
.LBB52_31:                              //   in Loop: Header=BB52_27 Depth=1
	cbz	w24, .LBB52_35
.LBB52_32:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	x0, [x25, #24]
	add	w8, w26, #63            // =63
	cmp	w26, #0                 // =0
	csel	w8, w8, w26, lt
	asr	w8, w8, #6
	ldr	x8, [x0, w8, sxtw #3]
	lsr	x8, x8, x26
	tbz	w8, #0, .LBB52_55
.LBB52_33:                              //   Parent Loop BB52_27 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	cbz	w26, .LBB52_36
// %bb.34:                              //   in Loop: Header=BB52_33 Depth=2
	sub	w28, w26, #1            // =1
	add	w8, w26, #62            // =62
	cmp	w28, #0                 // =0
	csel	w8, w8, w28, lt
	asr	w8, w8, #6
	ldr	x8, [x0, w8, sxtw #3]
	mov	w26, w28
	lsr	x8, x8, x28
	tbnz	w8, #0, .LBB52_33
	b	.LBB52_38
.LBB52_35:                              //   in Loop: Header=BB52_27 Depth=1
	mov	w8, #1
	b	.LBB52_57
.LBB52_36:                              //   in Loop: Header=BB52_27 Depth=1
	mov	x1, x22
	mov	x2, xzr
	bl	find_next_zero_bit
	mov	x26, x0
	cmp	w22, w26
	b.ls	.LBB52_47
// %bb.37:                              //   in Loop: Header=BB52_27 Depth=1
	mov	w28, w26
.LBB52_38:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w9, [x20, #1192]
	mov	w8, wzr
	mov	w26, w28
	mul	w27, w9, w28
	b	.LBB52_58
.LBB52_39:                              //   in Loop: Header=BB52_27 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB52_42
// %bb.40:                              //   in Loop: Header=BB52_27 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB52_42
// %bb.41:                              //   in Loop: Header=BB52_27 Depth=1
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.get_new_segment
	mov	w3, #3339
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.get_new_segment
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB52_42:                              //   in Loop: Header=BB52_27 Depth=1
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 3339; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB52_31
// %bb.43:                              //   in Loop: Header=BB52_27 Depth=1
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
	cbnz	w8, .LBB52_31
// %bb.44:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB52_31
// %bb.45:                              //   in Loop: Header=BB52_27 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB52_31
// %bb.46:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB52_31
.LBB52_47:                              //   in Loop: Header=BB52_27 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB52_50
// %bb.48:                              //   in Loop: Header=BB52_27 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB52_50
// %bb.49:                              //   in Loop: Header=BB52_27 Depth=1
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.get_new_segment
	mov	w3, #3356
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.get_new_segment
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB52_50:                              //   in Loop: Header=BB52_27 Depth=1
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 3356; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB52_55
// %bb.51:                              //   in Loop: Header=BB52_27 Depth=1
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
	cbnz	w8, .LBB52_55
// %bb.52:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB52_56
// %bb.53:                              //   in Loop: Header=BB52_27 Depth=1
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB52_56
// %bb.54:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
.LBB52_55:                              //   in Loop: Header=BB52_27 Depth=1
	mov	w8, wzr
.LBB52_56:                              //   in Loop: Header=BB52_27 Depth=1
	mov	w28, w26
.LBB52_57:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w9, [x20, #1192]
	cmn	w28, #1                 // =1
	mul	w27, w9, w28
	b.eq	.LBB52_59
.LBB52_58:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w9, [x20, #1196]
	udiv	w9, w28, w9
	tbnz	w19, #0, .LBB52_60
	b	.LBB52_22
.LBB52_59:                              //   in Loop: Header=BB52_27 Depth=1
	mov	w9, #-1
	tbz	w19, #0, .LBB52_22
.LBB52_60:                              //   in Loop: Header=BB52_27 Depth=1
	ldur	w10, [x29, #-12]        // 4-byte Folded Reload
	cmp	w9, w10
	b.eq	.LBB52_22
// %bb.61:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w10, [x20, #1196]
	cmp	w10, #1                 // =1
	b.eq	.LBB52_22
// %bb.62:                              //   in Loop: Header=BB52_27 Depth=1
	cbz	w23, .LBB52_65
// %bb.63:                              //   in Loop: Header=BB52_27 Depth=1
	ldur	w13, [x29, #-16]        // 4-byte Folded Reload
	cmp	w24, #0                 // =0
	add	w12, w9, #1             // =1
	cset	w11, ne
	cmp	w12, w13
	cset	w12, lo
	cmp	w9, #0                  // =0
	cset	w13, eq
	and	w13, w11, w13
	tbnz	w13, #0, .LBB52_22
// %bb.64:                              //   in Loop: Header=BB52_27 Depth=1
	orr	w11, w11, w12
	cbz	w11, .LBB52_22
.LBB52_65:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	x11, [x20, #136]
	ldr	x11, [x11, #24]
	ldr	w12, [x11, #108]
	cmp	w12, w9
	b.eq	.LBB52_73
// %bb.66:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w12, [x11, #228]
	cmp	w12, w9
	b.eq	.LBB52_73
// %bb.67:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w12, [x11, #348]
	cmp	w12, w9
	b.eq	.LBB52_73
// %bb.68:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w12, [x11, #468]
	cmp	w12, w9
	b.eq	.LBB52_73
// %bb.69:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w12, [x11, #588]
	cmp	w12, w9
	b.eq	.LBB52_73
// %bb.70:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w12, [x11, #708]
	cmp	w12, w9
	b.eq	.LBB52_73
// %bb.71:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w12, [x11, #828]
	cmp	w12, w9
	b.eq	.LBB52_73
// %bb.72:                              //   in Loop: Header=BB52_27 Depth=1
	ldr	w11, [x11, #948]
	cmp	w11, w9
	b.ne	.LBB52_22
.LBB52_73:                              //   in Loop: Header=BB52_27 Depth=1
	mov	w19, wzr
	tbz	w8, #0, .LBB52_26
// %bb.74:                              //   in Loop: Header=BB52_27 Depth=1
	add	w8, w9, #1              // =1
	mul	w9, w10, w8
	ldur	w10, [x29, #-16]        // 4-byte Folded Reload
	cmp	w8, w10
	csel	w21, w9, wzr, lo
	b	.LBB52_27
.LBB52_75:
	mov	w9, #-1
.LBB52_76:
	ldr	x10, [x8, #16]
	lsr	w11, w27, #6
	mov	w13, #1
	add	x10, x10, w11, uxtw #3
	//APP
	// atomic64_or
	prfm	pstl1strm, [x10]
1:	ldxr	x11, [x10]
	orr	x11, x11, x19
	stxr	w12, x11, [x10]
	cbnz	w12, 1b
	//NO_APP
	ldr	w11, [x8, #4]
	ldr	x10, [x8, #24]
	sub	w11, w11, #1            // =1
	str	w11, [x8, #4]
	lsr	w11, w9, #6
	ldr	x12, [x10, w11, uxtw #3]
	lsl	x9, x13, x9
	stur	x12, [x29, #-8]
	ldur	x12, [x29, #-8]
	tst	x12, x9
	b.ne	.LBB52_79
// %bb.77:
	add	x10, x10, x11, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x10]
1:	ldxr	x11, [x10]
	orr	x12, x11, x9
	stlxr	w13, x12, [x10]
	cbnz	w13, 1b
	dmb ish
	//NO_APP
	tst	x11, x9
	b.ne	.LBB52_79
// %bb.78:
	ldr	w9, [x8, #8]
	sub	w9, w9, #1              // =1
	str	w9, [x8, #8]
.LBB52_79:
	ldr	x0, [sp, #16]           // 8-byte Folded Reload
	bl	_raw_spin_unlock
	mov	w8, #120
	madd	x19, x23, x8, x22
	mov	w2, #1
	mov	x0, x20
	ldur	x1, [x29, #-24]         // 8-byte Folded Reload
	str	w27, [x19, #112]
                                        // kill: def $w1 killed $w1 killed $x1
	bl	reset_curseg
	strb	wzr, [x19, #96]
	ldp	x20, x19, [sp, #144]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #128]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #112]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #96]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #80]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #64]     // 16-byte Folded Reload
	add	sp, sp, #160            // =160
	ret
.LBB52_80:
	bl	print_bdev_access_info
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	cbz	x0, .LBB52_83
// %bb.81:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB52_83
// %bb.82:
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.sanity_check_seg_type
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.sanity_check_seg_type
	mov	w3, #46
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB52_83:
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.h"; .popsection; .long 14472b - 14470b; .short 46; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB52_4
// %bb.84:
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
	cbnz	w8, .LBB52_4
// %bb.85:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB52_4
// %bb.86:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB52_4
// %bb.87:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB52_4
.LBB52_88:
	bl	print_bdev_access_info
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	cbz	x0, .LBB52_91
// %bb.89:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB52_91
// %bb.90:
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.get_new_segment
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.get_new_segment
	mov	w3, #3407
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB52_91:
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 3407; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB52_23
// %bb.92:
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
	cbnz	w8, .LBB52_23
// %bb.93:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB52_23
// %bb.94:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB52_23
// %bb.95:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB52_23
.Lfunc_end52:
	.size	new_curseg, .Lfunc_end52-new_curseg
                                        // -- End function
	.globl	f2fs_allocate_new_section // -- Begin function f2fs_allocate_new_section
	.p2align	2
	.type	f2fs_allocate_new_section,@function
f2fs_allocate_new_section:              // @f2fs_allocate_new_section
.Lf2fs_allocate_new_section$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x21, x0
	mov	x29, sp
	add	x0, x8, #32             // =32
	mov	w19, w2
	mov	w20, w1
	bl	down_read
	ldr	x8, [x21, #136]
	ldr	x8, [x8]
	add	x0, x8, #72             // =72
	bl	down_write
	and	w3, w19, #0x1
	mov	w2, #1
	mov	x0, x21
	mov	w1, w20
	bl	__allocate_new_segment
	ldr	x8, [x21, #136]
	ldr	x8, [x8]
	add	x0, x8, #72             // =72
	bl	up_write
	ldr	x8, [x21, #136]
	add	x0, x8, #32             // =32
	bl	up_read
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end53:
	.size	f2fs_allocate_new_section, .Lfunc_end53-f2fs_allocate_new_section
                                        // -- End function
	.globl	f2fs_allocate_new_segments // -- Begin function f2fs_allocate_new_segments
	.p2align	2
	.type	f2fs_allocate_new_segments,@function
f2fs_allocate_new_segments:             // @f2fs_allocate_new_segments
.Lf2fs_allocate_new_segments$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	x19, x0
	mov	x29, sp
	add	x0, x8, #32             // =32
	bl	down_read
	ldr	x8, [x19, #136]
	ldr	x8, [x8]
	add	x0, x8, #72             // =72
	bl	down_write
	ldr	x8, [x19, #136]
	ldr	x9, [x8, #24]
	ldrb	w10, [x9, #116]
	cbz	w10, .LBB54_3
// %bb.1:
	ldrh	w10, [x9, #104]
	cbnz	w10, .LBB54_3
// %bb.2:
	ldr	x10, [x8]
	ldr	w11, [x9, #100]
	mov	w12, #40
	ldr	x10, [x10, #120]
	mul	x11, x11, x12
	ldr	w10, [x10, x11]
	tst	w10, #0x3ffffc0
	b.eq	.LBB54_4
.LBB54_3:
	ldr	x8, [x8]
	ldr	w20, [x9, #100]
	mov	w2, #1
	mov	x0, x19
	ldr	x8, [x8]
	mov	w1, wzr
	mov	w3, wzr
	ldr	x8, [x8]
	blr	x8
	mov	x0, x19
	mov	w1, w20
	bl	locate_dirty_segment
	ldr	x8, [x19, #136]
	ldr	x9, [x8, #24]
.LBB54_4:
	ldrb	w10, [x9, #236]
	cbz	w10, .LBB54_7
// %bb.5:
	ldrh	w10, [x9, #224]
	cbnz	w10, .LBB54_7
// %bb.6:
	ldr	x10, [x8]
	ldr	w11, [x9, #220]
	mov	w12, #40
	ldr	x10, [x10, #120]
	mul	x11, x11, x12
	ldr	w10, [x10, x11]
	tst	w10, #0x3ffffc0
	b.eq	.LBB54_8
.LBB54_7:
	ldr	x8, [x8]
	ldr	w20, [x9, #220]
	mov	w1, #1
	mov	w2, #1
	ldr	x8, [x8]
	mov	x0, x19
	mov	w3, wzr
	ldr	x8, [x8]
	blr	x8
	mov	x0, x19
	mov	w1, w20
	bl	locate_dirty_segment
	ldr	x8, [x19, #136]
	ldr	x9, [x8, #24]
.LBB54_8:
	ldrb	w10, [x9, #356]
	cbz	w10, .LBB54_11
// %bb.9:
	ldrh	w10, [x9, #344]
	cbnz	w10, .LBB54_11
// %bb.10:
	ldr	x10, [x8]
	ldr	w11, [x9, #340]
	mov	w13, #40
	ldr	x12, [x10, #120]
	mul	x11, x11, x13
	ldr	w11, [x12, x11]
	tst	w11, #0x3ffffc0
	b.eq	.LBB54_12
.LBB54_11:
	ldr	x8, [x8]
	ldr	w20, [x9, #340]
	mov	w1, #2
	mov	w2, #1
	ldr	x8, [x8]
	mov	x0, x19
	mov	w3, wzr
	ldr	x8, [x8]
	blr	x8
	mov	x0, x19
	mov	w1, w20
	bl	locate_dirty_segment
	ldr	x8, [x19, #136]
	ldr	x10, [x8]
.LBB54_12:
	add	x0, x10, #72            // =72
	bl	up_write
	ldr	x8, [x19, #136]
	add	x0, x8, #32             // =32
	bl	up_read
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end54:
	.size	f2fs_allocate_new_segments, .Lfunc_end54-f2fs_allocate_new_segments
                                        // -- End function
	.p2align	2               // -- Begin function __allocate_new_segment
	.type	__allocate_new_segment,@function
__allocate_new_segment:                 // @__allocate_new_segment
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	w10, #120
                                        // kill: def $w1 killed $w1 def $x1
	mov	x19, x0
	mov	x29, sp
	ldr	x9, [x8, #24]
	smaddl	x10, w1, w10, x9
	ldrb	w11, [x10, #116]
	sxtw	x10, w1
	cbz	w11, .LBB55_16
// %bb.1:
	tbnz	w3, #0, .LBB55_16
// %bb.2:
	mov	w11, #120
	madd	x11, x10, x11, x9
	ldrh	w11, [x11, #104]
	cbnz	w11, .LBB55_16
// %bb.3:
	mov	w11, #120
	madd	x11, x10, x11, x9
	ldr	w11, [x11, #100]
	tbz	w2, #0, .LBB55_7
// %bb.4:
	ldr	w12, [x19, #1192]
	cmp	w12, #2                 // =2
	b.lo	.LBB55_7
// %bb.5:
	ldr	x13, [x8]
	cmn	w11, #1                 // =1
	ldr	x14, [x13, #128]
	b.eq	.LBB55_12
// %bb.6:
	udiv	w15, w11, w12
	ldr	w14, [x14, x15, lsl #2]
	cbnz	w14, .LBB55_16
	b	.LBB55_13
.LBB55_7:
	ldr	x13, [x8]
	mov	w14, #40
	mul	x14, x11, x14
	ldr	x12, [x13, #120]
	ldr	w12, [x12, x14]
	tst	w12, #0xffc0
	b.ne	.LBB55_16
// %bb.8:
	tbz	w2, #0, .LBB55_11
// %bb.9:
	ldr	w12, [x19, #1192]
	cmp	w12, #1                 // =1
	b.hi	.LBB55_13
// %bb.10:
	ldr	x12, [x13, #120]
	mov	w13, #40
	mul	x11, x11, x13
	ldr	w12, [x12, x11]
.LBB55_11:
	ubfx	w14, w12, #16, #10
	b	.LBB55_15
.LBB55_12:
	mov	w15, #-1
	ldr	w14, [x14, x15, lsl #2]
	cbnz	w14, .LBB55_16
.LBB55_13:
	mov	w15, #4767
	movk	w15, #10724, lsl #16
	mul	x15, x11, x15
	lsr	x15, x15, #32
	ldr	x13, [x13, #120]
	sub	w11, w11, w15
	add	w11, w15, w11, lsr #1
	mov	w16, #55
	lsr	w11, w11, #5
	mov	w14, wzr
	mul	w11, w11, w16
	mov	w15, #40
.LBB55_14:                              // =>This Inner Loop Header: Depth=1
	umaddl	x16, w11, w15, x13
	ldrh	w16, [x16, #2]
	subs	w12, w12, #1            // =1
	add	w11, w11, #1            // =1
	and	w16, w16, #0x3ff
	add	w14, w16, w14
	b.ne	.LBB55_14
.LBB55_15:
	cbz	w14, .LBB55_17
.LBB55_16:
	ldr	x8, [x8]
	mov	w11, #120
	madd	x9, x10, x11, x9
	ldr	w20, [x9, #100]
	ldr	x8, [x8]
	mov	w2, #1
	mov	x0, x19
	mov	w3, wzr
	ldr	x8, [x8]
                                        // kill: def $w1 killed $w1 killed $x1
	blr	x8
	mov	x0, x19
	mov	w1, w20
	bl	locate_dirty_segment
.LBB55_17:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end55:
	.size	__allocate_new_segment, .Lfunc_end55-__allocate_new_segment
                                        // -- End function
	.globl	f2fs_exist_trim_candidates // -- Begin function f2fs_exist_trim_candidates
	.p2align	2
	.type	f2fs_exist_trim_candidates,@function
f2fs_exist_trim_candidates:             // @f2fs_exist_trim_candidates
.Lf2fs_exist_trim_candidates$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	ldr	x22, [x1, #8]
	mov	x20, x0
	mov	x29, sp
	ldr	x8, [x8]
	mov	x19, x1
	add	x0, x8, #72             // =72
	bl	down_write
	ldp	x8, x9, [x19, #8]
	cmp	x8, x9
	b.hi	.LBB56_4
// %bb.1:
	mov	w21, #1
.LBB56_2:                               // =>This Inner Loop Header: Depth=1
	mov	w2, #1
	mov	x0, x20
	mov	x1, x19
	bl	add_discard_addrs
	tbnz	w0, #0, .LBB56_5
// %bb.3:                               //   in Loop: Header=BB56_2 Depth=1
	ldp	x8, x9, [x19, #8]
	add	x8, x8, #1              // =1
	cmp	x8, x9
	str	x8, [x19, #8]
	b.ls	.LBB56_2
.LBB56_4:
	mov	w21, wzr
.LBB56_5:
	ldr	x8, [x20, #136]
	ldr	x8, [x8]
	add	x0, x8, #72             // =72
	bl	up_write
	str	x22, [x19, #8]
	mov	w0, w21
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end56:
	.size	f2fs_exist_trim_candidates, .Lfunc_end56-f2fs_exist_trim_candidates
                                        // -- End function
	.p2align	2               // -- Begin function add_discard_addrs
	.type	add_discard_addrs,@function
add_discard_addrs:                      // @add_discard_addrs
// %bb.0:
	sub	sp, sp, #128            // =128
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #48]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #64]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #80]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #96]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #112]    // 16-byte Folded Spill
	ldr	x13, [x0, #136]
	ldr	w8, [x1, #8]
	mov	w10, #40
	ldrsw	x21, [x0, #1188]
	ldr	x12, [x13]
	add	x29, sp, #32            // =32
	ldr	x9, [x12, #120]
	madd	x9, x8, x10, x9
	ldr	w11, [x9]
	ubfx	w8, w11, #6, #10
	cmp	w8, w21
	b.ne	.LBB57_3
.LBB57_1:
	mov	w0, wzr
.LBB57_2:
	ldp	x20, x19, [sp, #112]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #96]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #80]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #64]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #48]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #128            // =128
	ret
.LBB57_3:
	ldr	x23, [x12, #48]
	ldr	w12, [x0, #3128]
	ldp	x10, x8, [x9, #8]
	ldr	x9, [x9, #24]
	ldr	w24, [x1]
	ldr	x17, [x13, #168]
	mov	x19, x1
	mov	x20, x0
	mov	w28, w2
	cmp	w12, #2                 // =2
	b.lt	.LBB57_9
// %bb.4:
	ldr	x13, [x20, #3136]
	mov	x14, x12
	b	.LBB57_6
.LBB57_5:                               //   in Loop: Header=BB57_6 Depth=1
	subs	x14, x14, #1            // =1
	add	x13, x13, #88           // =88
	b.eq	.LBB57_1
.LBB57_6:                               // =>This Inner Loop Header: Depth=1
	ldr	x15, [x13]
	ldr	x15, [x15, #128]
	ldr	x15, [x15, #1264]
	ldr	x16, [x15, #104]
	tbnz	w16, #8, .LBB57_12
// %bb.7:                               //   in Loop: Header=BB57_6 Depth=1
	cbz	x15, .LBB57_5
// %bb.8:                               //   in Loop: Header=BB57_6 Depth=1
	ldr	w15, [x15, #2980]
	sub	w15, w15, #1            // =1
	cmp	w15, #2                 // =2
	b.hs	.LBB57_5
	b	.LBB57_12
.LBB57_9:
	ldr	x13, [x20]
	ldr	x13, [x13, #248]
	ldr	x13, [x13, #128]
	ldr	x13, [x13, #1264]
	ldr	x14, [x13, #104]
	tbnz	w14, #8, .LBB57_12
// %bb.10:
	cbz	x13, .LBB57_1
// %bb.11:
	ldr	w13, [x13, #2980]
	sub	w13, w13, #1            // =1
	cmp	w13, #1                 // =1
	b.hi	.LBB57_1
.LBB57_12:
	tbz	w24, #4, .LBB57_35
.LBB57_13:
	ldr	x10, [x8]
	ldr	x11, [x9]
	orr	x10, x11, x10
	mvn	x10, x10
	str	x10, [x23]
	ldr	x10, [x8, #8]
	ldr	x11, [x9, #8]
	orr	x10, x11, x10
	mvn	x10, x10
	str	x10, [x23, #8]
	ldr	x10, [x8, #16]
	ldr	x11, [x9, #16]
	orr	x10, x11, x10
	mvn	x10, x10
	str	x10, [x23, #16]
	ldr	x10, [x8, #24]
	ldr	x11, [x9, #24]
	orr	x10, x11, x10
	mvn	x10, x10
	str	x10, [x23, #24]
	ldr	x10, [x8, #32]
	ldr	x11, [x9, #32]
	orr	x10, x11, x10
	mvn	x10, x10
	str	x10, [x23, #32]
	ldr	x10, [x8, #40]
	ldr	x11, [x9, #40]
	orr	x10, x11, x10
	mvn	x10, x10
	str	x10, [x23, #40]
	ldr	x10, [x8, #48]
	ldr	x11, [x9, #48]
	orr	x10, x11, x10
	mvn	x10, x10
	str	x10, [x23, #48]
	ldr	x8, [x8, #56]
	ldr	x9, [x9, #56]
	orr	x8, x9, x8
	mvn	x8, x8
.LBB57_14:
	add	x9, x17, #8             // =8
	mov	x25, xzr
	stp	x9, x17, [sp, #8]       // 16-byte Folded Spill
	str	x8, [x23, #56]
	mov	w26, #-1
	mov	w22, #1
	stur	w28, [x29, #-4]         // 4-byte Folded Spill
	b	.LBB57_16
.LBB57_15:                              //   in Loop: Header=BB57_16 Depth=1
	ldr	x9, [x20, #136]
	ldur	w28, [x29, #-4]         // 4-byte Folded Reload
	ldr	x9, [x9, #168]
	ldr	w10, [x9, #8312]
	add	w8, w8, w10
	str	w8, [x9, #8312]
.LBB57_16:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB57_34 Depth 2
	tbnz	w24, #4, .LBB57_18
// %bb.17:                              //   in Loop: Header=BB57_16 Depth=1
	ldr	x8, [x20, #136]
	ldr	x8, [x8, #168]
	ldr	w9, [x8, #8312]
	ldr	w8, [x8, #8316]
	cmp	w9, w8
	b.hi	.LBB57_1
.LBB57_18:                              //   in Loop: Header=BB57_16 Depth=1
	add	w2, w26, #1             // =1
	mov	x0, x23
	mov	x1, x21
	bl	__find_rev_next_bit
	mov	x27, x0
	cmp	w21, w27
	b.ls	.LBB57_1
// %bb.19:                              //   in Loop: Header=BB57_16 Depth=1
	add	w2, w27, #1             // =1
	mov	x0, x23
	mov	x1, x21
	bl	__find_rev_next_zero_bit
	mov	x26, x0
	cmp	w21, w26
	b.eq	.LBB57_23
// %bb.20:                              //   in Loop: Header=BB57_16 Depth=1
	tbz	w24, #4, .LBB57_23
// %bb.21:                              //   in Loop: Header=BB57_16 Depth=1
	cbz	w27, .LBB57_23
// %bb.22:                              //   in Loop: Header=BB57_16 Depth=1
	ldr	x8, [x19, #24]
	sub	w9, w26, w27
	cmp	x9, x8
	b.lo	.LBB57_16
.LBB57_23:                              //   in Loop: Header=BB57_16 Depth=1
	tbnz	w28, #0, .LBB57_50
// %bb.24:                              //   in Loop: Header=BB57_16 Depth=1
	cbnz	x25, .LBB57_32
// %bb.25:                              //   in Loop: Header=BB57_16 Depth=1
	adrp	x8, discard_entry_slab
	ldr	x28, [x8, :lo12:discard_entry_slab]
	mov	w1, #3392
	mov	x0, x28
	bl	kmem_cache_alloc
	mov	x25, x0
	cbnz	x0, .LBB57_27
// %bb.26:                              //   in Loop: Header=BB57_16 Depth=1
	mov	w1, #36160
	mov	x0, x28
	bl	kmem_cache_alloc
	mov	x25, x0
.LBB57_27:                              //   in Loop: Header=BB57_16 Depth=1
	ldr	x8, [x20, #136]
	cbz	x8, .LBB57_29
// %bb.28:                              //   in Loop: Header=BB57_16 Depth=1
	add	x9, x8, #80             // =80
	b	.LBB57_30
.LBB57_29:                              //   in Loop: Header=BB57_16 Depth=1
	ldr	x9, [x20, #16]
	add	x9, x9, #72             // =72
.LBB57_30:                              //   in Loop: Header=BB57_16 Depth=1
	ldr	x8, [x8, #8]
	ldr	x10, [x19, #8]
	ldr	w11, [x20, #1184]
	ldr	w9, [x9]
	ldr	w8, [x8]
	mov	x0, x25
	add	x8, x10, x8
	lsl	x8, x8, x11
	add	w8, w9, w8
	str	w8, [x25, #16]
	ldp	x2, x8, [sp, #8]        // 16-byte Folded Reload
	ldr	x28, [x8, #16]
	mov	x1, x28
	bl	__list_add_valid
	tbz	w0, #0, .LBB57_32
// %bb.31:                              //   in Loop: Header=BB57_16 Depth=1
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	str	x25, [x8, #16]
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	stp	x8, x28, [x25]
	str	x25, [x28]
.LBB57_32:                              //   in Loop: Header=BB57_16 Depth=1
	subs	w8, w26, w27
	b.ls	.LBB57_15
// %bb.33:                              //   in Loop: Header=BB57_16 Depth=1
	add	x9, x25, #20            // =20
.LBB57_34:                              //   Parent Loop BB57_16 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	add	w10, w27, #63           // =63
	cmp	w27, #0                 // =0
	csel	w10, w10, w27, lt
	asr	w11, w10, #6
	sbfiz	x11, x11, #3, #32
	ldr	x12, [x9, x11]
	and	w10, w10, #0xffffffc0
	sub	w10, w27, w10
	add	w27, w27, #1            // =1
	lsl	x10, x22, x10
	orr	x10, x12, x10
	cmp	w26, w27
	str	x10, [x9, x11]
	b.ne	.LBB57_34
	b	.LBB57_15
.LBB57_35:
	ldrb	w13, [x20, #1424]
	tbnz	w13, #2, .LBB57_37
.LBB57_36:
	ldr	x12, [x20, #16]
	ldrb	w12, [x12, #2180]
	tbz	w12, #1, .LBB57_1
	b	.LBB57_46
.LBB57_37:
	cmp	w12, #2                 // =2
	b.lt	.LBB57_43
// %bb.38:
	ldr	x13, [x20, #3136]
	b	.LBB57_40
.LBB57_39:                              //   in Loop: Header=BB57_40 Depth=1
	subs	x12, x12, #1            // =1
	add	x13, x13, #88           // =88
	b.eq	.LBB57_36
.LBB57_40:                              // =>This Inner Loop Header: Depth=1
	ldr	x14, [x13]
	ldr	x14, [x14, #128]
	ldr	x14, [x14, #1264]
	ldr	x15, [x14, #104]
	tbnz	w15, #8, .LBB57_46
// %bb.41:                              //   in Loop: Header=BB57_40 Depth=1
	cbz	x14, .LBB57_39
// %bb.42:                              //   in Loop: Header=BB57_40 Depth=1
	ldr	w14, [x14, #2980]
	sub	w14, w14, #1            // =1
	cmp	w14, #2                 // =2
	b.hs	.LBB57_39
	b	.LBB57_46
.LBB57_43:
	ldr	x12, [x20]
	ldr	x12, [x12, #248]
	ldr	x12, [x12, #128]
	ldr	x12, [x12, #1264]
	ldr	x13, [x12, #104]
	tbnz	w13, #8, .LBB57_46
// %bb.44:
	cbz	x12, .LBB57_36
// %bb.45:
	ldr	w12, [x12, #2980]
	sub	w12, w12, #1            // =1
	cmp	w12, #2                 // =2
	b.hs	.LBB57_36
.LBB57_46:
	tst	w11, #0xffc0
	b.eq	.LBB57_1
// %bb.47:
	ldr	w11, [x17, #8312]
	ldr	w12, [x17, #8316]
	cmp	w11, w12
	b.hs	.LBB57_1
// %bb.48:
	tbnz	w24, #4, .LBB57_13
// %bb.49:
	ldr	x9, [x10]
	ldr	x11, [x8]
	bic	x9, x11, x9
	str	x9, [x23]
	ldr	x9, [x10, #8]
	ldr	x11, [x8, #8]
	bic	x9, x11, x9
	str	x9, [x23, #8]
	ldr	x9, [x10, #16]
	ldr	x11, [x8, #16]
	bic	x9, x11, x9
	str	x9, [x23, #16]
	ldr	x9, [x10, #24]
	ldr	x11, [x8, #24]
	bic	x9, x11, x9
	str	x9, [x23, #24]
	ldr	x9, [x10, #32]
	ldr	x11, [x8, #32]
	bic	x9, x11, x9
	str	x9, [x23, #32]
	ldr	x9, [x10, #40]
	ldr	x11, [x8, #40]
	bic	x9, x11, x9
	str	x9, [x23, #40]
	ldr	x9, [x10, #48]
	ldr	x11, [x8, #48]
	bic	x9, x11, x9
	str	x9, [x23, #48]
	ldr	x9, [x10, #56]
	ldr	x8, [x8, #56]
	bic	x8, x8, x9
	b	.LBB57_14
.LBB57_50:
	mov	w0, #1
	b	.LBB57_2
.Lfunc_end57:
	.size	add_discard_addrs, .Lfunc_end57-add_discard_addrs
                                        // -- End function
	.globl	f2fs_trim_fs            // -- Begin function f2fs_trim_fs
	.p2align	2
	.type	f2fs_trim_fs,@function
f2fs_trim_fs:                           // @f2fs_trim_fs
.Lf2fs_trim_fs$local:
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
	stur	x8, [x29, #-16]
	ldp	x10, x11, [x1]
	stp	xzr, xzr, [sp, #136]
	stp	xzr, xzr, [sp, #120]
	stur	wzr, [x29, #-88]
	stp	xzr, xzr, [x29, #-104]
	stp	xzr, xzr, [x29, #-120]
	stp	xzr, xzr, [x29, #-136]
	stur	xzr, [x29, #-144]
	ldr	w8, [x0, #1504]
	mov	x19, x1
	mov	x20, x0
	cmp	w8, #1                  // =1
	mov	w8, wzr
	b.ne	.LBB58_2
// %bb.1:
	ldr	w8, [x20, #1192]
	cmp	w8, #1                  // =1
	cset	w8, hi
.LBB58_2:
	ldr	x9, [x20, #136]
	lsr	x13, x10, #12
	cbz	x9, .LBB58_4
// %bb.3:
	add	x12, x9, #80            // =80
	add	x10, x9, #92            // =92
	b	.LBB58_5
.LBB58_4:
	ldr	x10, [x20, #16]
	add	x12, x10, #72           // =72
	add	x10, x10, #48           // =48
.LBB58_5:
	ldr	w14, [x10]
	ldr	w10, [x20, #1184]
	ldr	w12, [x12]
	lsl	w14, w14, w10
	add	w12, w14, w12
	cmp	x13, x12
	b.hs	.LBB58_7
// %bb.6:
	ldr	w12, [x20, #1168]
	cmp	x11, x12
	b.hs	.LBB58_10
.LBB58_7:
	mov	w21, #-22
.LBB58_8:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB58_128
// %bb.9:
	mov	w0, w21
	ldp	x20, x19, [sp, #480]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #464]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #448]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #432]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #416]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #400]    // 16-byte Folded Reload
	add	sp, sp, #496            // =496
	ret
.LBB58_10:
	add	x11, x13, x11, lsr #12
	sub	x11, x11, #1            // =1
	cbz	x9, .LBB58_12
// %bb.11:
	add	x12, x9, #84            // =84
	b	.LBB58_13
.LBB58_12:
	ldr	x12, [x20, #16]
	add	x12, x12, #92           // =92
.LBB58_13:
	ldr	w12, [x12]
	cmp	x11, x12
	b.hs	.LBB58_16
.LBB58_14:
	mov	x8, xzr
.LBB58_15:
	mov	w21, wzr
	str	x8, [x19, #8]
	b	.LBB58_8
.LBB58_16:
	ldr	x12, [x20, #80]
	tbnz	w12, #2, .LBB58_24
// %bb.17:
	ldr	w12, [x20, #3128]
	cbz	w12, .LBB58_33
// %bb.18:
	ldr	x14, [x20]
	cmp	w12, #1                 // =1
	ldr	x14, [x14, #248]
	b.lt	.LBB58_32
// %bb.19:
	ldr	x15, [x20, #3136]
	mov	x17, x12
	add	x16, x15, #80           // =80
	b	.LBB58_21
.LBB58_20:                              //   in Loop: Header=BB58_21 Depth=1
	subs	x17, x17, #1            // =1
	add	x16, x16, #88           // =88
	b.eq	.LBB58_25
.LBB58_21:                              // =>This Inner Loop Header: Depth=1
	ldur	w0, [x16, #-4]
	cmp	w0, w13
	b.hi	.LBB58_20
// %bb.22:                              //   in Loop: Header=BB58_21 Depth=1
	ldr	w0, [x16]
	cmp	w0, w13
	b.lo	.LBB58_20
// %bb.23:
	ldur	x16, [x16, #-80]
	b	.LBB58_26
.LBB58_24:
	adrp	x1, .L.str.7
	add	x1, x1, :lo12:.L.str.7
	mov	x0, x20
	bl	f2fs_printk
	mov	w21, #-117
	b	.LBB58_8
.LBB58_25:
	mov	x16, x14
.LBB58_26:
	cbz	x16, .LBB58_14
// %bb.27:
	mov	x16, xzr
	add	x17, x15, #80           // =80
	b	.LBB58_29
.LBB58_28:                              //   in Loop: Header=BB58_29 Depth=1
	add	x16, x16, #1            // =1
	cmp	x12, x16
	add	x17, x17, #88           // =88
	b.eq	.LBB58_32
.LBB58_29:                              // =>This Inner Loop Header: Depth=1
	ldur	w0, [x17, #-4]
	cmp	w0, w11
	b.hi	.LBB58_28
// %bb.30:                              //   in Loop: Header=BB58_29 Depth=1
	ldr	w0, [x17]
	cmp	w0, w11
	b.lo	.LBB58_28
// %bb.31:
	and	x14, x16, #0xffffffff
	mov	w16, #88
	mul	x14, x14, x16
	ldr	x14, [x15, x14]
.LBB58_32:
	cbz	x14, .LBB58_14
.LBB58_33:
	cbz	x9, .LBB58_35
// %bb.34:
	add	x14, x9, #84            // =84
	b	.LBB58_36
.LBB58_35:
	ldr	x14, [x20, #16]
	add	x14, x14, #92           // =92
.LBB58_36:
	ldr	w14, [x14]
	cmp	x13, x14
	b.ls	.LBB58_39
// %bb.37:
	add	w14, w13, #3            // =3
	cmp	w14, #4                 // =4
	b.hs	.LBB58_40
.LBB58_38:
	mov	w23, #-1
	b	.LBB58_52
.LBB58_39:
	mov	w23, wzr
	b	.LBB58_52
.LBB58_40:
	cbz	w12, .LBB58_48
// %bb.41:
	ldr	x14, [x20]
	cmp	w12, #1                 // =1
	ldr	x14, [x14, #248]
	b.lt	.LBB58_47
// %bb.42:
	ldr	x15, [x20, #3136]
	mov	x16, x12
	add	x15, x15, #80           // =80
	b	.LBB58_44
.LBB58_43:                              //   in Loop: Header=BB58_44 Depth=1
	subs	x16, x16, #1            // =1
	add	x15, x15, #88           // =88
	b.eq	.LBB58_47
.LBB58_44:                              // =>This Inner Loop Header: Depth=1
	ldur	w17, [x15, #-4]
	cmp	w17, w13
	b.hi	.LBB58_43
// %bb.45:                              //   in Loop: Header=BB58_44 Depth=1
	ldr	w17, [x15]
	cmp	w17, w13
	b.lo	.LBB58_43
// %bb.46:
	ldur	x14, [x15, #-80]
.LBB58_47:
	cbz	x14, .LBB58_38
.LBB58_48:
	cbz	x9, .LBB58_50
// %bb.49:
	add	x14, x9, #80            // =80
	b	.LBB58_51
.LBB58_50:
	ldr	x14, [x20, #16]
	add	x14, x14, #72           // =72
.LBB58_51:
	ldr	x15, [x9, #8]
	ldr	w14, [x14]
	ldr	w15, [x15]
	sub	x13, x13, x14
	lsr	x13, x13, x10
	sub	w23, w13, w15
.LBB58_52:
	cbz	x9, .LBB58_54
// %bb.53:
	add	x13, x9, #80            // =80
	add	x14, x9, #92            // =92
	b	.LBB58_55
.LBB58_54:
	ldr	x14, [x20, #16]
	add	x13, x14, #72           // =72
	add	x14, x14, #48           // =48
.LBB58_55:
	ldr	w14, [x14]
	ldr	w13, [x13]
	lsl	w14, w14, w10
	add	w13, w14, w13
	cmp	x11, x13
	b.hs	.LBB58_58
// %bb.56:
	add	w13, w11, #3            // =3
	cmp	w13, #4                 // =4
	b.hs	.LBB58_59
.LBB58_57:
	mov	w24, #-1
	b	.LBB58_71
.LBB58_58:
	ldr	w9, [x9, #96]
	sub	w24, w9, #1             // =1
	b	.LBB58_71
.LBB58_59:
	cbz	w12, .LBB58_67
// %bb.60:
	ldr	x13, [x20]
	cmp	w12, #1                 // =1
	ldr	x13, [x13, #248]
	b.lt	.LBB58_66
// %bb.61:
	ldr	x14, [x20, #3136]
	add	x14, x14, #80           // =80
	b	.LBB58_63
.LBB58_62:                              //   in Loop: Header=BB58_63 Depth=1
	subs	x12, x12, #1            // =1
	add	x14, x14, #88           // =88
	b.eq	.LBB58_66
.LBB58_63:                              // =>This Inner Loop Header: Depth=1
	ldur	w15, [x14, #-4]
	cmp	w15, w11
	b.hi	.LBB58_62
// %bb.64:                              //   in Loop: Header=BB58_63 Depth=1
	ldr	w15, [x14]
	cmp	w15, w11
	b.lo	.LBB58_62
// %bb.65:
	ldur	x13, [x14, #-80]
.LBB58_66:
	cbz	x13, .LBB58_57
.LBB58_67:
	cbz	x9, .LBB58_69
// %bb.68:
	add	x12, x9, #80            // =80
	b	.LBB58_70
.LBB58_69:
	ldr	x12, [x20, #16]
	add	x12, x12, #72           // =72
.LBB58_70:
	ldr	x9, [x9, #8]
	ldr	w12, [x12]
	ldr	w9, [x9]
	sub	x11, x11, x12
	lsr	x10, x11, x10
	sub	x24, x10, x9
.LBB58_71:
	cbz	w8, .LBB58_73
// %bb.72:
	ldr	w8, [x20, #1192]
	udiv	w9, w23, w8
	add	w10, w8, w24
	mul	w23, w9, w8
	udiv	w9, w10, w8
	msub	w8, w9, w8, w10
	mvn	w8, w8
	add	w24, w10, w8
.LBB58_73:
	mov	w8, #16
	str	w8, [sp, #120]
	ldr	x8, [x19, #16]
	mov	w9, w23
	mov	w10, w24
	str	x9, [sp, #128]
	lsr	x8, x8, #12
	cmp	x8, #0                  // =0
	csinc	x8, x8, xzr, ne
	stp	x10, x8, [sp, #136]
	ldr	w8, [x20, #1244]
	cbz	w8, .LBB58_14
// %bb.74:
	add	x22, x20, #1664         // =1664
	mov	x0, x22
	bl	down_write
	//APP
	mrs x26, sp_el0
	//NO_APP
	ldr	w8, [x26, #60]
	add	x1, sp, #120            // =120
	mov	x0, x20
	orr	w8, w8, #0x20000000
	str	w8, [x26, #60]
	bl	f2fs_write_checkpoint
	ldr	w8, [x26, #60]
	mov	w21, w0
	mov	x0, x22
	and	w8, w8, #0xdfffffff
	str	w8, [x26, #60]
	bl	up_write
	ldr	x8, [x26]
	tbz	w8, #0, .LBB58_76
// %bb.75:
	ldrb	w8, [x26, #2128]
	tbnz	w8, #1, .LBB58_117
.LBB58_76:
	cbnz	w21, .LBB58_8
// %bb.77:
	ldrb	w8, [x20, #1424]
	tbnz	w8, #2, .LBB58_118
.LBB58_78:
	ldr	x9, [x20, #16]
	ldrb	w8, [x9, #2180]
	tbnz	w8, #1, .LBB58_14
// %bb.79:
	ldr	x8, [x20, #136]
	ldr	w10, [x20, #1184]
	add	x9, x9, #72             // =72
	adrp	x14, dpolicys+180
	ldr	x11, [x8, #8]
	add	x13, x8, #80            // =80
	cmp	x8, #0                  // =0
	csel	x13, x9, x13, eq
	ldr	w11, [x11]
	ldr	w12, [sp, #144]
	add	x14, x14, :lo12:dpolicys+180
	ldp	x15, x16, [x14, #32]
	add	w9, w11, w23
	add	w11, w24, w11
	add	w11, w11, #1            // =1
	ldr	x17, [x14, #48]
	ldr	w0, [x14, #56]
	ldp	x1, x2, [x14]
	lsl	w9, w9, w10
	lsl	w10, w11, w10
	ldp	x3, x14, [x14, #16]
	ldr	w11, [x13]
	cmp	w12, #1                 // =1
	stp	x16, x15, [sp, #72]     // 16-byte Folded Spill
	stp	x15, x16, [x29, #-112]
	str	x17, [sp, #64]          // 8-byte Folded Spill
	stur	x17, [x29, #-96]
	str	w0, [sp, #60]           // 4-byte Folded Spill
	stur	w0, [x29, #-88]
	stp	x2, x1, [sp, #40]       // 16-byte Folded Spill
	stp	x1, x2, [x29, #-144]
	stp	x14, x3, [sp, #24]      // 16-byte Folded Spill
	stp	x3, x14, [x29, #-128]
	b.eq	.LBB58_81
// %bb.80:
	stur	w12, [x29, #-140]
.LBB58_81:
	sturb	wzr, [x29, #-88]
	ldr	x24, [x8, #168]
	mov	w8, #8360
	stp	xzr, xzr, [x29, #-160]
	stp	xzr, xzr, [x29, #-176]
	stp	xzr, xzr, [sp, #208]
	stp	xzr, xzr, [sp, #192]
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	str	wzr, [sp, #156]
	add	x25, x24, x8
	ldrb	w8, [x25]
	add	w23, w9, w11
	add	w21, w11, w10
	cbz	w8, .LBB58_84
// %bb.82:
	mov	w8, #8280
	add	x22, x24, x8
	mov	x0, x22
	bl	mutex_lock
	mov	w8, #8344
	add	x1, x24, x8
	mov	x0, x20
	mov	w2, wzr
	bl	f2fs_check_rb_tree_consistence
	tbz	w0, #0, .LBB58_129
.LBB58_83:
	mov	x0, x22
	bl	mutex_unlock
.LBB58_84:
	mov	w8, #8280
	mov	w9, #8344
	add	x10, x20, #3408         // =3408
	stp	x10, x25, [sp, #8]      // 16-byte Folded Spill
	mov	w10, #8232
	add	x28, x24, x8
	add	x8, x24, x9
	str	wzr, [sp, #116]         // 4-byte Folded Spill
	stp	x28, x8, [sp, #96]      // 16-byte Folded Spill
	add	x27, x24, x10
	mov	w22, w23
	str	w23, [sp, #92]          // 4-byte Folded Spill
.LBB58_85:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB58_90 Depth 2
	mov	x0, x28
	str	wzr, [sp, #156]
	bl	mutex_lock
	ldrb	w8, [x25]
	cbnz	w8, .LBB58_108
.LBB58_86:                              //   in Loop: Header=BB58_85 Depth=1
	ldr	x0, [sp, #104]          // 8-byte Folded Reload
	sub	x3, x29, #152           // =152
	sub	x4, x29, #160           // =160
	sub	x5, x29, #168           // =168
	sub	x6, x29, #176           // =176
	mov	w7, #1
	mov	x1, xzr
	mov	w2, w22
	str	xzr, [sp]
	bl	f2fs_lookup_rb_tree_ret
	ldur	x8, [x29, #-160]
	cmp	x0, #0                  // =0
	csel	x28, x8, x0, eq
	add	x0, sp, #160            // =160
	bl	blk_start_plug
	cbz	x28, .LBB58_124
// %bb.87:                              //   in Loop: Header=BB58_85 Depth=1
	mov	x22, x28
	ldr	w8, [x22, #24]!
	cmp	w8, w21
	b.ls	.LBB58_90
	b	.LBB58_124
.LBB58_88:                              //   in Loop: Header=BB58_90 Depth=2
	cbz	x25, .LBB58_124
.LBB58_89:                              //   in Loop: Header=BB58_90 Depth=2
	mov	x22, x25
	ldr	w8, [x22, #24]!
	mov	x28, x25
	cmp	w8, w21
	b.hi	.LBB58_124
.LBB58_90:                              //   Parent Loop BB58_85 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	w8, [x28, #28]
	ldur	w9, [x29, #-140]
	cmp	w8, w9
	b.lo	.LBB58_96
// %bb.91:                              //   in Loop: Header=BB58_90 Depth=2
	ldrb	w8, [x28, #98]
	cbz	w8, .LBB58_102
// %bb.92:                              //   in Loop: Header=BB58_90 Depth=2
	add	x22, x28, #40           // =40
	mov	x0, x22
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB58_94
// %bb.93:                              //   in Loop: Header=BB58_90 Depth=2
	ldp	x9, x8, [x28, #40]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB58_94:                              //   in Loop: Header=BB58_90 Depth=2
	ldr	x23, [x24, #8240]
	mov	x0, x22
	mov	x2, x27
	mov	x1, x23
	bl	__list_add_valid
	tbz	w0, #0, .LBB58_96
// %bb.95:                              //   in Loop: Header=BB58_90 Depth=2
	str	x22, [x24, #8240]
	stp	x27, x23, [x28, #40]
	str	x22, [x23]
.LBB58_96:                              //   in Loop: Header=BB58_90 Depth=2
	mov	x0, x28
	bl	rb_next
	mov	x25, x0
.LBB58_97:                              //   in Loop: Header=BB58_90 Depth=2
	ldr	x8, [x26]
	tbz	w8, #0, .LBB58_99
// %bb.98:                              //   in Loop: Header=BB58_90 Depth=2
	ldrb	w8, [x26, #2129]
	tbnz	w8, #0, .LBB58_124
.LBB58_99:                              //   in Loop: Header=BB58_90 Depth=2
	ldr	x8, [x26]
	tbz	w8, #0, .LBB58_88
// %bb.100:                             //   in Loop: Header=BB58_90 Depth=2
	cbz	x25, .LBB58_124
// %bb.101:                             //   in Loop: Header=BB58_90 Depth=2
	ldr	x8, [x26, #2128]
	tbz	w8, #1, .LBB58_89
	b	.LBB58_124
.LBB58_102:                             //   in Loop: Header=BB58_90 Depth=2
	sub	x1, x29, #144           // =144
	add	x4, sp, #156            // =156
	mov	x0, x20
	mov	w2, wzr
	mov	x3, x28
	bl	__submit_discard_cmd
	ldr	w8, [sp, #156]
	ldur	w9, [x29, #-124]
	mov	w23, w0
	cmp	w8, w9
	b.hs	.LBB58_105
// %bb.103:                             //   in Loop: Header=BB58_90 Depth=2
	mov	x0, x28
	bl	rb_next
	mov	x25, x0
	cbz	w23, .LBB58_97
// %bb.104:                             //   in Loop: Header=BB58_90 Depth=2
	mov	x0, x20
	mov	x1, x28
	bl	__remove_discard_cmd
	b	.LBB58_97
.LBB58_105:                             //   in Loop: Header=BB58_85 Depth=1
	ldr	w8, [x22]
	ldr	w9, [x28, #28]
	add	w22, w9, w8
	cbz	w23, .LBB58_107
// %bb.106:                             //   in Loop: Header=BB58_85 Depth=1
	mov	x0, x20
	mov	x1, x28
	bl	__remove_discard_cmd
.LBB58_107:                             //   in Loop: Header=BB58_85 Depth=1
	add	x0, sp, #160            // =160
	bl	blk_finish_plug
	ldr	x28, [sp, #96]          // 8-byte Folded Reload
	mov	x0, x28
	bl	mutex_unlock
	ldr	w8, [sp, #60]           // 4-byte Folded Reload
	sub	x1, x29, #80            // =80
	mov	w3, #-1
	mov	x0, x20
	stur	w8, [x29, #-24]
	ldr	x8, [sp, #80]           // 8-byte Folded Reload
	mov	w2, wzr
	sturb	wzr, [x29, #-24]
	stur	x8, [x29, #-48]
	ldr	x8, [sp, #72]           // 8-byte Folded Reload
	stur	x8, [x29, #-40]
	ldr	x8, [sp, #64]           // 8-byte Folded Reload
	stur	x8, [x29, #-32]
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	stur	x8, [x29, #-80]
	ldr	x8, [sp, #40]           // 8-byte Folded Reload
	stur	x8, [x29, #-72]
	ldr	x8, [sp, #32]           // 8-byte Folded Reload
	stur	x8, [x29, #-64]
	ldr	x8, [sp, #24]           // 8-byte Folded Reload
	stur	x8, [x29, #-56]
	bl	__wait_discard_cmd_range
	adrp	x13, dpolicys+180
	add	x13, x13, :lo12:dpolicys+180
	ldr	w8, [x13, #116]
	ldur	x9, [x13, #92]
	ldur	x10, [x13, #100]
	ldur	x11, [x13, #108]
	ldur	x12, [x13, #60]
	stur	w8, [x29, #-24]
	ldur	x8, [x13, #68]
	stp	x9, x10, [x29, #-48]
	ldur	x9, [x13, #76]
	ldur	x10, [x13, #84]
	mov	w23, w0
	sub	x1, x29, #80            // =80
	mov	w3, #-1
	mov	x0, x20
	mov	w2, wzr
	stur	x11, [x29, #-32]
	stp	x12, x8, [x29, #-80]
	stp	x9, x10, [x29, #-64]
	sturb	wzr, [x29, #-24]
	bl	__wait_discard_cmd_range
	ldr	w8, [sp, #116]          // 4-byte Folded Reload
	mov	w1, #5
	add	w8, w23, w8
	add	w8, w8, w0
	mov	w0, wzr
	str	w8, [sp, #116]          // 4-byte Folded Spill
	bl	congestion_wait
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	b	.LBB58_85
.LBB58_108:                             //   in Loop: Header=BB58_85 Depth=1
	ldr	x1, [sp, #104]          // 8-byte Folded Reload
	mov	x0, x20
	mov	w2, wzr
	bl	f2fs_check_rb_tree_consistence
	tbnz	w0, #0, .LBB58_86
// %bb.109:                             //   in Loop: Header=BB58_85 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB58_112
// %bb.110:                             //   in Loop: Header=BB58_85 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB58_112
// %bb.111:                             //   in Loop: Header=BB58_85 Depth=1
	adrp	x23, f2fs_dclient
	ldr	x0, [x23, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__issue_discard_cmd_range
	mov	w3, #4636
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__issue_discard_cmd_range
	bl	dsm_client_record
	ldr	x0, [x23, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB58_112:                             //   in Loop: Header=BB58_85 Depth=1
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 4636; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB58_86
// %bb.113:                             //   in Loop: Header=BB58_85 Depth=1
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
	cbnz	w8, .LBB58_86
// %bb.114:                             //   in Loop: Header=BB58_85 Depth=1
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB58_86
// %bb.115:                             //   in Loop: Header=BB58_85 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB58_86
// %bb.116:                             //   in Loop: Header=BB58_85 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB58_86
.LBB58_117:
	mov	w21, #-4
	b	.LBB58_8
.LBB58_118:
	ldr	w8, [x20, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB58_125
// %bb.119:
	ldr	x9, [x20, #3136]
	b	.LBB58_121
.LBB58_120:                             //   in Loop: Header=BB58_121 Depth=1
	subs	x8, x8, #1              // =1
	add	x9, x9, #88             // =88
	b.eq	.LBB58_78
.LBB58_121:                             // =>This Inner Loop Header: Depth=1
	ldr	x10, [x9]
	ldr	x10, [x10, #128]
	ldr	x10, [x10, #1264]
	ldr	x11, [x10, #104]
	tbnz	w11, #8, .LBB58_14
// %bb.122:                             //   in Loop: Header=BB58_121 Depth=1
	cbz	x10, .LBB58_120
// %bb.123:                             //   in Loop: Header=BB58_121 Depth=1
	ldr	w10, [x10, #2980]
	sub	w10, w10, #1            // =1
	cmp	w10, #2                 // =2
	b.hs	.LBB58_120
	b	.LBB58_14
.LBB58_124:
	add	x0, sp, #160            // =160
	bl	blk_finish_plug
	ldr	x0, [sp, #96]           // 8-byte Folded Reload
	bl	mutex_unlock
	ldr	w2, [sp, #92]           // 4-byte Folded Reload
	sub	x1, x29, #144           // =144
	mov	x0, x20
	mov	w3, w21
	bl	__wait_discard_cmd_range
	ldr	w9, [sp, #116]          // 4-byte Folded Reload
	mov	w8, w0
	add	x8, x8, w9, uxtw
	lsl	x8, x8, #12
	b	.LBB58_15
.LBB58_125:
	ldr	x8, [x20]
	ldr	x8, [x8, #248]
	ldr	x8, [x8, #128]
	ldr	x8, [x8, #1264]
	ldr	x9, [x8, #104]
	tbnz	w9, #8, .LBB58_14
// %bb.126:
	cbz	x8, .LBB58_78
// %bb.127:
	ldr	w8, [x8, #2980]
	sub	w8, w8, #1              // =1
	cmp	w8, #2                  // =2
	b.lo	.LBB58_14
	b	.LBB58_78
.LBB58_128:
	bl	__stack_chk_fail
.LBB58_129:
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB58_132
// %bb.130:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB58_132
// %bb.131:
	adrp	x27, f2fs_dclient
	ldr	x0, [x27, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__issue_discard_cmd_range
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__issue_discard_cmd_range
	mov	w3, #4622
	bl	dsm_client_record
	ldr	x0, [x27, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB58_132:
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 4622; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB58_83
// %bb.133:
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
	cbnz	w8, .LBB58_83
// %bb.134:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB58_83
// %bb.135:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB58_83
// %bb.136:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB58_83
.Lfunc_end58:
	.size	f2fs_trim_fs, .Lfunc_end58-f2fs_trim_fs
                                        // -- End function
	.p2align	2               // -- Begin function __wait_discard_cmd_range
	.type	__wait_discard_cmd_range,@function
__wait_discard_cmd_range:               // @__wait_discard_cmd_range
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	ldr	w9, [x1]
	mov	w10, #8232
	mov	w11, #8216
	ldr	x8, [x8, #168]
	cmp	w9, #3                  // =3
	mov	w9, #8280
	mov	x22, x0
	add	x23, x8, x9
	add	x10, x8, x10
	add	x11, x8, x11
	mov	x0, x23
	mov	x29, sp
	mov	w19, w3
	mov	w20, w2
	mov	x21, x1
	csel	x26, x10, x11, eq
	bl	mutex_lock
	ldr	x28, [x26]
	cmp	x26, x28
	b.eq	.LBB59_16
// %bb.1:
	mov	w24, wzr
	b	.LBB59_4
.LBB59_2:                               //   in Loop: Header=BB59_4 Depth=1
	mov	x0, x22
	mov	x1, x25
	bl	__remove_discard_cmd
.LBB59_3:                               //   in Loop: Header=BB59_4 Depth=1
	cmp	x26, x27
	mov	x28, x27
	b.eq	.LBB59_17
.LBB59_4:                               // =>This Inner Loop Header: Depth=1
	ldur	w9, [x28, #-16]
	ldr	x27, [x28]
	cmp	w9, w19
	b.hs	.LBB59_3
// %bb.5:                               //   in Loop: Header=BB59_4 Depth=1
	ldur	w8, [x28, #-12]
	add	w9, w8, w9
	cmp	w9, w20
	b.ls	.LBB59_3
// %bb.6:                               //   in Loop: Header=BB59_4 Depth=1
	ldr	w9, [x21, #4]
	cmp	w8, w9
	b.lo	.LBB59_3
// %bb.7:                               //   in Loop: Header=BB59_4 Depth=1
	mov	x8, x28
	ldrh	w9, [x8, #56]!
	sub	x25, x28, #40           // =40
	ldrb	w10, [x8, #2]
	cmp	w10, #3                 // =3
	b.ne	.LBB59_9
// %bb.8:                               //   in Loop: Header=BB59_4 Depth=1
	cbz	w9, .LBB59_10
.LBB59_9:                               //   in Loop: Header=BB59_4 Depth=1
	add	w9, w9, #1              // =1
	mov	x0, x23
	strh	w9, [x8]
	bl	mutex_unlock
	mov	x0, x22
	mov	x1, x25
	bl	__wait_one_discard_bio
	add	w24, w0, w24
	mov	x0, x23
	bl	mutex_lock
	ldr	x28, [x26]
	cmp	x26, x28
	b.ne	.LBB59_4
	b	.LBB59_17
.LBB59_10:                              //   in Loop: Header=BB59_4 Depth=1
	add	x0, x28, #16            // =16
	bl	wait_for_completion_io
	ldr	w8, [x28, #60]
	cbnz	w8, .LBB59_12
// %bb.11:                              //   in Loop: Header=BB59_4 Depth=1
	ldur	w8, [x28, #-12]
	add	w24, w8, w24
.LBB59_12:                              //   in Loop: Header=BB59_4 Depth=1
	ldr	w8, [x21]
	cmp	w8, #3                  // =3
	b.eq	.LBB59_2
// %bb.13:                              //   in Loop: Header=BB59_4 Depth=1
	ldur	w8, [x28, #-12]
	cmp	w8, #512                // =512
	b.lo	.LBB59_2
// %bb.14:                              //   in Loop: Header=BB59_4 Depth=1
	ldr	x8, [x28, #64]
	mov	w9, #61568
	movk	w9, #762, lsl #16
	cmp	x8, x9
	b.lo	.LBB59_2
// %bb.15:                              //   in Loop: Header=BB59_4 Depth=1
	mov	w8, #10
	str	w8, [x21, #44]
	b	.LBB59_2
.LBB59_16:
	mov	w24, wzr
.LBB59_17:
	mov	x0, x23
	bl	mutex_unlock
	mov	w0, w24
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.Lfunc_end59:
	.size	__wait_discard_cmd_range, .Lfunc_end59-__wait_discard_cmd_range
                                        // -- End function
	.globl	f2fs_rw_hint_to_seg_type // -- Begin function f2fs_rw_hint_to_seg_type
	.p2align	2
	.type	f2fs_rw_hint_to_seg_type,@function
f2fs_rw_hint_to_seg_type:               // @f2fs_rw_hint_to_seg_type
.Lf2fs_rw_hint_to_seg_type$local:
// %bb.0:
	cmp	w0, #5                  // =5
	mov	w8, #1
	cinc	w8, w8, eq
	cmp	w0, #2                  // =2
	csel	w0, wzr, w8, eq
	ret
.Lfunc_end60:
	.size	f2fs_rw_hint_to_seg_type, .Lfunc_end60-f2fs_rw_hint_to_seg_type
                                        // -- End function
	.globl	f2fs_io_type_to_rw_hint // -- Begin function f2fs_io_type_to_rw_hint
	.p2align	2
	.type	f2fs_io_type_to_rw_hint,@function
f2fs_io_type_to_rw_hint:                // @f2fs_io_type_to_rw_hint
.Lf2fs_io_type_to_rw_hint$local:
// %bb.0:
	ldr	w8, [x0, #1484]
	cmp	w8, #2                  // =2
	b.eq	.LBB61_5
// %bb.1:
	cmp	w8, #1                  // =1
	b.ne	.LBB61_14
// %bb.2:
	cbnz	w1, .LBB61_14
// %bb.3:
	cmp	w2, #3                  // =3
	b.hs	.LBB61_14
// %bb.4:
	adrp	x8, .Lswitch.table.f2fs_io_type_to_rw_hint
	add	x8, x8, :lo12:.Lswitch.table.f2fs_io_type_to_rw_hint
	ldr	w0, [x8, w2, sxtw #2]
	ret
.LBB61_5:
	cmp	w1, #2                  // =2
	b.eq	.LBB61_10
// %bb.6:
	cmp	w1, #1                  // =1
	b.eq	.LBB61_11
// %bb.7:
	cbnz	w1, .LBB61_14
// %bb.8:
	cmp	w2, #3                  // =3
	b.hs	.LBB61_14
// %bb.9:
	adrp	x8, .Lswitch.table.f2fs_io_type_to_rw_hint.45
	add	x8, x8, :lo12:.Lswitch.table.f2fs_io_type_to_rw_hint.45
	ldr	w0, [x8, w2, sxtw #2]
	ret
.LBB61_10:
	mov	w0, #3
	ret
.LBB61_11:
	cmp	w2, #2                  // =2
	b.lo	.LBB61_14
// %bb.12:
	b.ne	.LBB61_14
// %bb.13:
	mov	w0, #1
	ret
.LBB61_14:
	mov	w0, wzr
	ret
.Lfunc_end61:
	.size	f2fs_io_type_to_rw_hint, .Lfunc_end61-f2fs_io_type_to_rw_hint
                                        // -- End function
	.globl	f2fs_allocate_data_block // -- Begin function f2fs_allocate_data_block
	.p2align	2
	.type	f2fs_allocate_data_block,@function
f2fs_allocate_data_block:               // @f2fs_allocate_data_block
.Lf2fs_allocate_data_block$local:
// %bb.0:
	sub	sp, sp, #224            // =224
	stp	x29, x30, [sp, #128]    // 16-byte Folded Spill
	stp	x28, x27, [sp, #144]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #160]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #176]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #192]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #208]    // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	add	x29, sp, #128           // =128
	mov	x19, x0
	mov	w22, w7
	ldr	x20, [x8]
	ldr	x26, [x8, #24]
	add	x0, x8, #32             // =32
	mov	x21, x6
	mov	w24, w5
	mov	x23, x4
	stur	x3, [x29, #-8]          // 8-byte Folded Spill
	mov	w27, w2
	mov	x25, x1
	bl	down_read
	mov	w8, #120
	stur	x26, [x29, #-16]        // 8-byte Folded Spill
	smaddl	x26, w24, w8, x26
	mov	x0, x26
	bl	mutex_lock
	add	x28, x20, #72           // =72
	mov	x0, x28
	str	x20, [sp, #56]          // 8-byte Folded Spill
	bl	down_write
	cmp	w24, #7                 // =7
	stur	x23, [x29, #-32]        // 8-byte Folded Spill
	b.ne	.LBB62_10
// %bb.1:
	add	w8, w27, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB62_11
.LBB62_2:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB62_5
// %bb.3:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB62_5
// %bb.4:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_allocate_data_block
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_allocate_data_block
	mov	w3, #5070
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB62_5:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 5070; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB62_23
// %bb.6:
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
	cbnz	w8, .LBB62_23
// %bb.7:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB62_23
// %bb.8:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB62_23
// %bb.9:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB62_23
.LBB62_10:
	str	xzr, [sp, #48]          // 8-byte Folded Spill
	b	.LBB62_39
.LBB62_11:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB62_19
// %bb.12:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB62_18
// %bb.13:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB62_15
.LBB62_14:                              //   in Loop: Header=BB62_15 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB62_18
.LBB62_15:                              // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w27
	b.hi	.LBB62_14
// %bb.16:                              //   in Loop: Header=BB62_15 Depth=1
	ldr	w11, [x10]
	cmp	w11, w27
	b.lo	.LBB62_14
// %bb.17:
	ldur	x8, [x10, #-80]
.LBB62_18:
	cbz	x8, .LBB62_2
.LBB62_19:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB62_21
// %bb.20:
	add	x9, x8, #80             // =80
	b	.LBB62_22
.LBB62_21:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB62_22:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w27, w9
	lsr	w9, w9, w10
	sub	w8, w9, w8
	cmn	w8, #1                  // =1
	b.eq	.LBB62_2
.LBB62_23:
	add	w8, w27, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB62_25
.LBB62_24:
	mov	w8, #-1
	b	.LBB62_37
.LBB62_25:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB62_33
// %bb.26:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB62_32
// %bb.27:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB62_29
.LBB62_28:                              //   in Loop: Header=BB62_29 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB62_32
.LBB62_29:                              // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w27
	b.hi	.LBB62_28
// %bb.30:                              //   in Loop: Header=BB62_29 Depth=1
	ldr	w11, [x10]
	cmp	w11, w27
	b.lo	.LBB62_28
// %bb.31:
	ldur	x8, [x10, #-80]
.LBB62_32:
	cbz	x8, .LBB62_24
.LBB62_33:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB62_35
// %bb.34:
	add	x9, x8, #80             // =80
	b	.LBB62_36
.LBB62_35:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB62_36:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w27, w9
	lsr	w9, w9, w10
	sub	w8, w9, w8
.LBB62_37:
	ldr	x9, [x19, #136]
	mov	w10, #40
	ldr	x9, [x9]
	ldr	x9, [x9, #120]
	umaddl	x23, w8, w10, x9
	ldr	w8, [x23]
	and	w8, w8, #0x3e
	cmp	w8, #6                  // =6
	b.hs	.LBB62_156
.LBB62_38:
	ldr	w8, [x23]
	str	x23, [sp, #48]          // 8-byte Folded Spill
	and	w8, w8, #0x3f
	sub	w8, w8, #3              // =3
	cmp	w8, #2                  // =2
	b.ls	.LBB62_164
.LBB62_39:
	ldur	x15, [x29, #-16]        // 8-byte Folded Reload
	sxtw	x23, w24
	cbz	x21, .LBB62_45
// %bb.40:
	ldr	w8, [x21, #132]
	cbz	w8, .LBB62_45
// %bb.41:
	mov	w8, #120
	madd	x8, x23, x8, x15
	ldrh	w9, [x8, #104]!
	ldurb	w10, [x8, #-8]
	cmp	w10, #1                 // =1
	b.ne	.LBB62_43
// %bb.42:
	ldr	x10, [x19, #136]
	mov	w11, #120
	madd	x11, x23, x11, x15
	ldr	w11, [x11, #100]
	ldr	x10, [x10]
	mov	w13, #40
	ldr	x12, [x10, #120]
	ldr	x10, [x10, #48]
	madd	x11, x11, x13, x12
	ldp	x11, x12, [x11, #8]
	ldr	x13, [x12]
	ldr	x14, [x11]
	orr	x13, x14, x13
	str	x13, [x10]
	ldr	x13, [x12, #8]
	ldr	x14, [x11, #8]
	orr	x13, x14, x13
	str	x13, [x10, #8]
	ldr	x13, [x12, #16]
	ldr	x14, [x11, #16]
	orr	x13, x14, x13
	str	x13, [x10, #16]
	ldr	x13, [x12, #24]
	ldr	x14, [x11, #24]
	orr	x13, x14, x13
	str	x13, [x10, #24]
	ldr	x13, [x12, #32]
	ldr	x14, [x11, #32]
	orr	x13, x14, x13
	str	x13, [x10, #32]
	ldr	x13, [x12, #40]
	ldr	x14, [x11, #40]
	orr	x13, x14, x13
	str	x13, [x10, #40]
	ldr	x13, [x12, #48]
	ldr	x14, [x11, #48]
	orr	x13, x14, x13
	str	x13, [x10, #48]
	ldr	x12, [x12, #56]
	ldr	x11, [x11, #56]
	orr	x11, x11, x12
	str	x11, [x10, #56]
	ldrh	w10, [x8]
	b	.LBB62_44
.LBB62_43:
	add	w10, w9, #1             // =1
	strh	w10, [x8]
.LBB62_44:
	ldr	w11, [x19, #1188]
	strh	w9, [x8]
	cmp	w11, w10, uxth
	b.ls	.LBB62_52
.LBB62_45:
	ldrb	w8, [x19, #1232]
	tbz	w8, #4, .LBB62_60
// %bb.46:
	cbz	x25, .LBB62_50
// %bb.47:
	ldr	x8, [x21, #40]
	ldr	x9, [x8]
	tbz	w9, #13, .LBB62_54
// %bb.48:
	ldr	x9, [x8, #40]
	tbz	w9, #0, .LBB62_54
// %bb.49:
	ldr	x8, [x8, #40]
	tbz	w8, #3, .LBB62_54
.LBB62_50:
	mov	w8, #120
	madd	x8, x23, x8, x15
	ldrb	w8, [x8, #96]
	cmp	w8, #1                  // =1
	b.ne	.LBB62_53
// %bb.51:
	ldr	w8, [x19, #2264]
	add	w8, w8, #1              // =1
	str	w8, [x19, #2264]
	cbnz	x25, .LBB62_54
	b	.LBB62_60
.LBB62_52:
	mov	x0, x28
	bl	up_write
	mov	w20, #-12
	b	.LBB62_223
.LBB62_53:
	ldr	w8, [x19, #2260]
	add	w8, w8, #1              // =1
	str	w8, [x19, #2260]
	cbz	x25, .LBB62_60
.LBB62_54:
	sub	w8, w24, #3             // =3
	cmp	w8, #2                  // =2
	b.hi	.LBB62_60
// %bb.55:
	ldr	x8, [x25, #40]
	cmn	x8, #3                  // =3
	b.ne	.LBB62_60
// %bb.56:
	mov	w8, #120
	madd	x8, x23, x8, x15
	ldrb	w8, [x8, #96]
	cmp	w8, #1                  // =1
	b.ne	.LBB62_58
// %bb.57:
	ldr	w8, [x19, #2272]
	add	w8, w8, #1              // =1
	str	w8, [x19, #2272]
	b	.LBB62_59
.LBB62_58:
	ldr	w8, [x19, #2268]
	add	w8, w8, #1              // =1
	str	w8, [x19, #2268]
.LBB62_59:
	str	xzr, [x25, #40]
.LBB62_60:
	ldr	x8, [x19, #136]
	stp	x28, x21, [x29, #-48]   // 16-byte Folded Spill
	cbz	x8, .LBB62_62
// %bb.61:
	add	x9, x8, #80             // =80
	b	.LBB62_63
.LBB62_62:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB62_63:
	mov	w10, #120
	madd	x28, x23, x10, x15
	ldr	w9, [x9]
	ldr	w10, [x28, #100]!
	ldr	x8, [x8, #8]
	ldr	w11, [x19, #1184]
	stur	x28, [x29, #-56]        // 8-byte Folded Spill
	ldr	w8, [x8]
	ldrh	w12, [x28, #4]!
	ldur	x21, [x29, #-8]         // 8-byte Folded Reload
	add	w8, w8, w10
	lsl	w8, w8, w11
	add	w8, w8, w9
	add	w8, w8, w12
	str	w8, [x21]
	ldrh	w8, [x28]
	ldr	w9, [x19, #1188]
	cmp	w9, w8
	b.ls	.LBB62_148
.LBB62_64:
	ldr	x8, [x19, #136]
	stur	x25, [x29, #-24]        // 8-byte Folded Spill
	ldr	w25, [x21]
	stur	w22, [x29, #-60]        // 4-byte Folded Spill
	ldr	x20, [x8, #168]
	mov	w8, #8280
	add	x22, x20, x8
	mov	x0, x22
	bl	mutex_lock
	mov	w8, #8344
	add	x0, x20, x8
	mov	x1, xzr
	mov	w2, w25
	bl	f2fs_lookup_rb_tree
	cbz	x0, .LBB62_67
// %bb.65:
	ldrb	w8, [x0, #98]
	mov	x20, x0
	cbz	w8, .LBB62_68
// %bb.66:
	ldrh	w8, [x20, #96]
	mov	x0, x22
	add	w8, w8, #1              // =1
	strh	w8, [x20, #96]
	bl	mutex_unlock
	mov	x0, x19
	mov	x1, x20
	bl	__wait_one_discard_bio
	b	.LBB62_71
.LBB62_67:
	mov	x0, x22
	bl	mutex_unlock
	b	.LBB62_71
.LBB62_68:
	ldr	w21, [x20, #28]
	str	x26, [sp, #40]          // 8-byte Folded Spill
	cmp	w21, #1                 // =1
	b.ne	.LBB62_130
// %bb.69:
	mov	x0, x19
	mov	x1, x20
	bl	__remove_discard_cmd
.LBB62_70:
	mov	x0, x22
	bl	mutex_unlock
	ldr	x26, [sp, #40]          // 8-byte Folded Reload
.LBB62_71:
	ldr	x8, [x19, #136]
	mov	w9, #120
	mul	x20, x23, x9
	ldur	x1, [x29, #-32]         // 8-byte Folded Reload
	ldr	x8, [x8, #24]
	mov	w2, #7
	add	x8, x8, x20
	ldrh	w9, [x8, #104]
	ldr	x8, [x8, #32]
	lsl	x10, x9, #3
	sub	x9, x10, x9
	add	x0, x8, x9
	bl	memcpy
	ldp	x25, x8, [x29, #-24]    // 16-byte Folded Reload
	add	x21, x8, x20
	ldrb	w8, [x21, #96]!
	cmp	w8, #1                  // =1
	b.ne	.LBB62_73
// %bb.72:
	ldr	x8, [x19, #136]
	ldur	x9, [x29, #-56]         // 8-byte Folded Reload
	mov	w11, #40
	ldr	x8, [x8]
	ldr	w9, [x9]
	ldr	x10, [x8, #120]
	ldr	x0, [x8, #48]
	ldrh	w8, [x28]
	madd	x9, x9, x11, x10
	ldp	x9, x10, [x9, #8]
	add	x2, x8, #1              // =1
	ldr	x11, [x10]
	ldr	x12, [x9]
	orr	x11, x12, x11
	str	x11, [x0]
	ldr	x11, [x10, #8]
	ldr	x12, [x9, #8]
	orr	x11, x12, x11
	str	x11, [x0, #8]
	ldr	x11, [x10, #16]
	ldr	x12, [x9, #16]
	orr	x11, x12, x11
	str	x11, [x0, #16]
	ldr	x11, [x10, #24]
	ldr	x12, [x9, #24]
	orr	x11, x12, x11
	str	x11, [x0, #24]
	ldr	x11, [x10, #32]
	ldr	x12, [x9, #32]
	orr	x11, x12, x11
	str	x11, [x0, #32]
	ldr	x11, [x10, #40]
	ldr	x12, [x9, #40]
	orr	x11, x12, x11
	str	x11, [x0, #40]
	ldr	x11, [x10, #48]
	ldr	x12, [x9, #48]
	orr	x11, x12, x11
	str	x11, [x0, #48]
	ldr	x10, [x10, #56]
	ldr	x9, [x9, #56]
	orr	x9, x9, x10
	str	x9, [x0, #56]
	ldr	w1, [x19, #1188]
	bl	__find_rev_next_zero_bit
	ldrb	w8, [x21]
	b	.LBB62_74
.LBB62_73:
	ldrh	w9, [x28]
	add	w0, w9, #1              // =1
.LBB62_74:
	strh	w0, [x28]
	add	x8, x19, w8, uxtw #2
	ldr	w9, [x8, #2152]
	ldur	x23, [x29, #-8]         // 8-byte Folded Reload
	cmp	w24, #7                 // =7
	add	w9, w9, #1              // =1
	str	w9, [x8, #2152]
	b.ne	.LBB62_86
// %bb.75:
	add	w8, w27, #3             // =3
	cmp	w8, #4                  // =4
	b.lo	.LBB62_87
// %bb.76:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB62_84
// %bb.77:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB62_83
// %bb.78:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB62_80
.LBB62_79:                              //   in Loop: Header=BB62_80 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB62_83
.LBB62_80:                              // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w27
	b.hi	.LBB62_79
// %bb.81:                              //   in Loop: Header=BB62_80 Depth=1
	ldr	w11, [x10]
	cmp	w11, w27
	b.lo	.LBB62_79
// %bb.82:
	ldur	x8, [x10, #-80]
.LBB62_83:
	cbz	x8, .LBB62_87
.LBB62_84:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB62_137
// %bb.85:
	add	x9, x8, #80             // =80
	b	.LBB62_138
.LBB62_86:
	mov	x0, x19
	mov	w1, w27
	mov	x2, xzr
	bl	update_segment_mtime
.LBB62_87:
	mov	x2, xzr
.LBB62_88:
	ldr	w1, [x23]
	mov	x0, x19
	bl	update_segment_mtime
	add	x20, x19, #2296         // =2296
	mov	x0, x20
	bl	mutex_lock
	cmp	w24, #2                 // =2
	b.hi	.LBB62_92
// %bb.89:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB62_96
// %bb.90:
	ldrb	w9, [x21]
	add	x8, x8, x9, lsl #2
	ldr	w9, [x8, #160]
	add	w9, w9, #1              // =1
	str	w9, [x8, #160]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB62_96
// %bb.91:
	ldr	x9, [x8, #200]
	add	x9, x9, #1              // =1
	str	x9, [x8, #200]
	b	.LBB62_96
.LBB62_92:
	sub	w8, w24, #3             // =3
	cmp	w8, #2                  // =2
	b.hi	.LBB62_96
// %bb.93:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB62_96
// %bb.94:
	ldrb	w9, [x21]
	add	x8, x8, x9, lsl #2
	ldr	w9, [x8, #152]
	add	w9, w9, #1              // =1
	str	w9, [x8, #152]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB62_96
// %bb.95:
	ldr	x9, [x8, #192]
	add	x9, x9, #1              // =1
	str	x9, [x8, #192]
.LBB62_96:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB62_98
// %bb.97:
	add	w9, w24, #1             // =1
	add	x8, x8, w9, sxtw #3
	ldr	x9, [x8, #304]
	add	x9, x9, #1              // =1
	str	x9, [x8, #304]
.LBB62_98:
	mov	x0, x20
	bl	mutex_unlock
	ldr	w1, [x23]
	mov	w2, #1
	mov	x0, x19
	bl	update_sit_entry
	add	w8, w27, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB62_118
.LBB62_99:
	ldr	x8, [x19, #16]
	ldrb	w9, [x8, #2180]
	tbnz	w9, #1, .LBB62_101
// %bb.100:
	ldrh	w9, [x28]
	ldr	w10, [x19, #1188]
	cmp	w10, w9
	b.hi	.LBB62_174
.LBB62_101:
	ldr	w20, [x29, #96]
	cbnz	w20, .LBB62_108
// %bb.102:
	cbz	x25, .LBB62_107
// %bb.103:
	ldr	x9, [x25, #24]
	cbz	x9, .LBB62_107
// %bb.104:
	ldr	x10, [x19, #128]
	ldr	x10, [x10, #48]
	cmp	x9, x10
	b.eq	.LBB62_107
// %bb.105:
	ldr	x10, [x19, #248]
	ldr	x10, [x10, #48]
	cmp	x9, x10
	b.eq	.LBB62_107
// %bb.106:
	ldr	x9, [x9]
	ldr	w9, [x9, #792]
	cmp	w9, #31                 // =31
	cset	w10, gt
	cmp	w9, #255                // =255
	mov	w9, #2
	csel	w20, w9, w10, gt
	b	.LBB62_108
.LBB62_107:
	mov	w20, wzr
.LBB62_108:
	ldr	x9, [x19, #136]
	adrp	x11, __tracepoint_f2fs_grading_ssr_allocate+8
	ldr	w8, [x8, #36]
	ldr	x10, [x9, #8]
	ldr	w9, [x19, #1240]
	ldr	w22, [x10, #4]
	ldr	w10, [x11, :lo12:__tracepoint_f2fs_grading_ssr_allocate+8]
	cmp	w10, #1                 // =1
	b.lt	.LBB62_115
// %bb.109:
	adrp	x11, cpu_number
	//APP
	.if 1 == 1
661:
	mrs x10, tpidr_el1
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
	mrs x10, tpidr_el2
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	add	x11, x11, :lo12:cpu_number
	ldr	w10, [x10, x11]
	adrp	x12, __cpu_online_mask
	add	x12, x12, :lo12:__cpu_online_mask
	add	w11, w10, #63           // =63
	cmp	w10, #0                 // =0
	csel	w11, w11, w10, lt
	asr	w11, w11, #6
	ldr	x11, [x12, w11, sxtw #3]
	lsr	x10, x11, x10
	tbz	w10, #0, .LBB62_115
// %bb.110:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w10, [x21, #24]
	add	w10, w10, #1            // =1
	str	w10, [x21, #24]
	//APP
	//NO_APP
	adrp	x10, __tracepoint_f2fs_grading_ssr_allocate+32
	ldr	x23, [x10, :lo12:__tracepoint_f2fs_grading_ssr_allocate+32]
	cbz	x23, .LBB62_113
// %bb.111:
	sub	w25, w8, w9
.LBB62_112:                             // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	mov	w1, w25
	mov	w2, w22
	mov	w3, w20
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB62_112
.LBB62_113:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	ldur	x25, [x29, #-24]        // 8-byte Folded Reload
	ldur	x23, [x29, #-8]         // 8-byte Folded Reload
	b.eq	.LBB62_172
// %bb.114:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB62_172
.LBB62_115:
	cmp	w24, #7                 // =7
	b.ne	.LBB62_173
.LBB62_116:
	ldr	x10, [sp, #48]          // 8-byte Folded Reload
	ldr	x8, [x19, #136]
	mov	w1, #7
	mov	w2, #2
	ldrh	w9, [x10]
	ldr	x20, [x8, #24]
	ldr	x3, [x10, #32]
	mov	x0, x19
	and	w8, w9, #0x3f
	strh	w8, [x20, #938]
	mov	w21, #2
	bl	get_ssr_segment
	cbz	w0, .LBB62_128
// %bb.117:
	ldr	x8, [x19, #136]
	ldr	w9, [x20, #952]
	mov	w10, #40
	mov	w1, #7
	ldr	x8, [x8]
	mul	x9, x9, x10
	mov	x0, x19
	ldr	x8, [x8, #120]
	ldrh	w8, [x8, x9]
	and	w8, w8, #0x3f
	strh	w8, [x20, #938]
	bl	change_curseg
	b	.LBB62_129
.LBB62_118:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB62_126
// %bb.119:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB62_125
// %bb.120:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB62_122
.LBB62_121:                             //   in Loop: Header=BB62_122 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB62_125
.LBB62_122:                             // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w27
	b.hi	.LBB62_121
// %bb.123:                             //   in Loop: Header=BB62_122 Depth=1
	ldr	w11, [x10]
	cmp	w11, w27
	b.lo	.LBB62_121
// %bb.124:
	ldur	x8, [x10, #-80]
.LBB62_125:
	cbz	x8, .LBB62_99
.LBB62_126:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB62_145
// %bb.127:
	add	x9, x8, #80             // =80
	b	.LBB62_146
.LBB62_128:
	mov	w1, #7
	mov	w2, #1
	mov	x0, x19
	strh	w21, [x20, #938]
	bl	new_curseg
.LBB62_129:
	ldrb	w8, [x20, #936]
	add	x8, x19, x8, lsl #2
	ldr	w9, [x8, #2144]
	add	w9, w9, #1              // =1
	str	w9, [x8, #2144]
	b	.LBB62_174
.LBB62_130:
	ldr	x8, [x19, #136]
	ldr	w9, [x20, #24]
	ldr	w10, [x20, #32]
	ldr	x26, [x8, #168]
	cmp	w9, w25
	ldr	w8, [x26, #8324]
	sub	w8, w8, w21
	str	w8, [x26, #8324]
	b.hs	.LBB62_140
// %bb.131:
	ldr	w8, [x20, #24]
	str	w9, [sp, #20]           // 4-byte Folded Spill
	add	x0, x20, #40            // =40
	str	w10, [sp, #4]           // 4-byte Folded Spill
	sub	w8, w25, w8
	str	w8, [x20, #28]
	ldr	w9, [x26, #8324]
	add	w8, w9, w8
	str	w8, [x26, #8324]
	ldr	w8, [x20, #28]
	sub	w9, w8, #1              // =1
	cmp	w8, #511                // =511
	mov	w8, #511
	csel	w8, w8, w9, hi
	str	x8, [sp, #8]            // 8-byte Folded Spill
	add	x8, x26, w8, uxtw #4
	add	x8, x8, #24             // =24
	stp	x8, x0, [sp, #24]       // 16-byte Folded Spill
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB62_133
// %bb.132:
	ldp	x9, x8, [x20, #40]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB62_133:
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	add	x26, x26, x8, lsl #4
	ldr	x1, [x26, #32]!
	ldp	x2, x0, [sp, #24]       // 16-byte Folded Reload
	str	x1, [sp, #8]            // 8-byte Folded Spill
	bl	__list_add_valid
	tbz	w0, #0, .LBB62_135
// %bb.134:
	ldp	x8, x9, [sp, #24]       // 16-byte Folded Reload
	str	x9, [x26]
	str	x8, [x20, #40]
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	str	x8, [x20, #48]
	str	x9, [x8]
.LBB62_135:
	ldr	w9, [sp, #20]           // 4-byte Folded Reload
	add	w8, w9, w21
	sub	w8, w8, #1              // =1
	subs	w4, w8, w25
	b.ls	.LBB62_70
// %bb.136:
	add	w2, w25, #1             // =1
	sub	w8, w2, w9
	ldr	w9, [sp, #4]            // 4-byte Folded Reload
	ldr	x1, [x20, #88]
	mov	x0, x19
	add	w3, w8, w9
	bl	__insert_discard_tree
	b	.LBB62_70
.LBB62_137:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB62_138:
	ldr	x10, [x8, #8]
	ldr	w9, [x9]
	ldr	w11, [x19, #1184]
	ldr	w10, [x10]
	sub	w9, w27, w9
	lsr	w9, w9, w11
	sub	w9, w9, w10
	cmn	w9, #1                  // =1
	b.eq	.LBB62_87
// %bb.139:
	ldr	x8, [x8]
	mov	w10, #40
	ldr	x8, [x8, #120]
	umaddl	x8, w9, w10, x8
	ldr	x2, [x8, #32]
	b	.LBB62_88
.LBB62_140:
	add	w8, w9, w21
	sub	w8, w8, #1              // =1
	cmp	w8, w25
	b.ls	.LBB62_70
// %bb.141:
	ldp	w8, w9, [x20, #24]
	ldr	w10, [x20, #32]
	add	x0, x20, #40            // =40
	add	w8, w8, #1              // =1
	sub	w9, w9, #1              // =1
	add	w10, w10, #1            // =1
	stp	w8, w9, [x20, #24]
	str	w10, [x20, #32]
	ldr	w8, [x26, #8324]
	add	w8, w8, w9
	str	w8, [x26, #8324]
	ldr	w8, [x20, #28]
	sub	w9, w8, #1              // =1
	cmp	w8, #511                // =511
	mov	w8, #511
	csel	w21, w8, w9, hi
	add	x8, x26, w21, uxtw #4
	add	x8, x8, #24             // =24
	stp	x0, x8, [sp, #24]       // 16-byte Folded Spill
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB62_143
// %bb.142:
	ldp	x9, x8, [x20, #40]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB62_143:
	add	x21, x26, x21, lsl #4
	ldr	x25, [x21, #32]!
	ldp	x26, x2, [sp, #24]      // 16-byte Folded Reload
	mov	x1, x25
	mov	x0, x26
	bl	__list_add_valid
	tbz	w0, #0, .LBB62_70
// %bb.144:
	ldr	x8, [sp, #32]           // 8-byte Folded Reload
	str	x26, [x21]
	stp	x8, x25, [x20, #40]
	str	x26, [x25]
	b	.LBB62_70
.LBB62_145:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB62_146:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w27, w9
	lsr	w9, w9, w10
	sub	w8, w9, w8
	cmn	w8, #1                  // =1
	b.eq	.LBB62_99
// %bb.147:
	mov	w2, #-1
	mov	x0, x19
	mov	w1, w27
	bl	update_sit_entry
	b	.LBB62_99
.LBB62_148:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB62_151
// %bb.149:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB62_151
// %bb.150:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_allocate_data_block
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_allocate_data_block
	mov	w3, #5136
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB62_151:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 5136; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldur	x21, [x29, #-8]         // 8-byte Folded Reload
	cbz	x19, .LBB62_64
// %bb.152:
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
	cbnz	w8, .LBB62_64
// %bb.153:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB62_64
// %bb.154:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB62_64
// %bb.155:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB62_64
.LBB62_156:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB62_159
// %bb.157:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB62_159
// %bb.158:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.sanity_check_seg_type
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.sanity_check_seg_type
	mov	w3, #46
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB62_159:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.h"; .popsection; .long 14472b - 14470b; .short 46; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB62_38
// %bb.160:
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
	cbnz	w8, .LBB62_38
// %bb.161:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB62_38
// %bb.162:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB62_38
// %bb.163:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB62_38
.LBB62_164:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB62_167
// %bb.165:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB62_167
// %bb.166:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_allocate_data_block
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_allocate_data_block
	mov	w3, #5073
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB62_167:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 5073; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB62_39
// %bb.168:
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
	cbnz	w8, .LBB62_39
// %bb.169:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB62_39
// %bb.170:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB62_39
// %bb.171:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB62_39
.LBB62_172:
	bl	preempt_schedule_notrace
	cmp	w24, #7                 // =7
	b.eq	.LBB62_116
.LBB62_173:
	ldr	x8, [sp, #56]           // 8-byte Folded Reload
	mov	x0, x19
	mov	w1, w24
	mov	w2, wzr
	ldr	x8, [x8]
	mov	w3, w20
	ldr	x8, [x8]
	blr	x8
.LBB62_174:
	add	w8, w27, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB62_176
.LBB62_175:
	mov	w1, #-1
	b	.LBB62_188
.LBB62_176:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB62_184
// %bb.177:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB62_183
// %bb.178:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB62_180
.LBB62_179:                             //   in Loop: Header=BB62_180 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB62_183
.LBB62_180:                             // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w27
	b.hi	.LBB62_179
// %bb.181:                             //   in Loop: Header=BB62_180 Depth=1
	ldr	w11, [x10]
	cmp	w11, w27
	b.lo	.LBB62_179
// %bb.182:
	ldur	x8, [x10, #-80]
.LBB62_183:
	cbz	x8, .LBB62_175
.LBB62_184:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB62_186
// %bb.185:
	add	x9, x8, #80             // =80
	b	.LBB62_187
.LBB62_186:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB62_187:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w27, w9
	lsr	w9, w9, w10
	sub	w1, w9, w8
.LBB62_188:
	mov	x0, x19
	bl	locate_dirty_segment
	ldr	w8, [x23]
	add	w9, w8, #3              // =3
	cmp	w9, #4                  // =4
	b.hs	.LBB62_190
.LBB62_189:
	mov	w1, #-1
	b	.LBB62_202
.LBB62_190:
	ldr	w10, [x19, #3128]
	cbz	w10, .LBB62_198
// %bb.191:
	ldr	x9, [x19]
	cmp	w10, #1                 // =1
	ldr	x9, [x9, #248]
	b.lt	.LBB62_197
// %bb.192:
	ldr	x11, [x19, #3136]
	add	x11, x11, #80           // =80
	b	.LBB62_194
.LBB62_193:                             //   in Loop: Header=BB62_194 Depth=1
	subs	x10, x10, #1            // =1
	add	x11, x11, #88           // =88
	b.eq	.LBB62_197
.LBB62_194:                             // =>This Inner Loop Header: Depth=1
	ldur	w12, [x11, #-4]
	cmp	w12, w8
	b.hi	.LBB62_193
// %bb.195:                             //   in Loop: Header=BB62_194 Depth=1
	ldr	w12, [x11]
	cmp	w12, w8
	b.lo	.LBB62_193
// %bb.196:
	ldur	x9, [x11, #-80]
.LBB62_197:
	cbz	x9, .LBB62_189
.LBB62_198:
	ldr	x9, [x19, #136]
	cbz	x9, .LBB62_200
// %bb.199:
	add	x10, x9, #80            // =80
	b	.LBB62_201
.LBB62_200:
	ldr	x10, [x19, #16]
	add	x10, x10, #72           // =72
.LBB62_201:
	ldr	x9, [x9, #8]
	ldr	w10, [x10]
	ldr	w11, [x19, #1184]
	ldr	w9, [x9]
	sub	w8, w8, w10
	lsr	w8, w8, w11
	sub	w1, w8, w9
.LBB62_202:
	mov	x0, x19
	bl	locate_dirty_segment
	ldur	x0, [x29, #-48]         // 8-byte Folded Reload
	bl	up_write
	cbz	x25, .LBB62_216
// %bb.203:
	sub	w8, w24, #3             // =3
	cmp	w8, #2                  // =2
	b.hi	.LBB62_216
// %bb.204:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB62_206
// %bb.205:
	add	x9, x8, #80             // =80
	b	.LBB62_207
.LBB62_206:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB62_207:
	ldr	w22, [x9]
	ldr	x8, [x8, #8]
	ldur	x9, [x29, #-56]         // 8-byte Folded Reload
	ldr	w10, [x19, #1184]
	ldrh	w20, [x28]
	ldr	w8, [x8]
	ldr	w9, [x9]
	ldr	x11, [x25, #8]
	add	w8, w8, w9
	sub	x12, x11, #1            // =1
	tst	x11, #0x1
	csel	x11, x25, x12, eq
	ldr	x11, [x11]
	lsl	w23, w8, w10
	tbz	w11, #19, .LBB62_209
// %bb.208:
	ldr	x8, [x25]
	tbnz	w8, #10, .LBB62_224
.LBB62_209:
	ldr	x0, [x25, #24]
.LBB62_210:
	ldr	x8, [x0]
	add	w22, w23, w22
	ldr	x8, [x8, #40]
	ldr	x21, [x8, #1008]
	ldr	x8, [x25, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x25, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB62_212
// %bb.211:
	ldr	x8, [x25]
	tbnz	w8, #10, .LBB62_225
.LBB62_212:
	ldr	x0, [x25, #24]
.LBB62_213:
	ldr	x8, [x0]
	mov	x10, #134217728
	movk	x10, #64, lsl #32
	add	x10, x10, x25, lsl #6
	ldr	x8, [x8, #40]
	and	x10, x10, #0xfffffffffffff000
	mov	x12, #-549755813888
	add	x10, x10, x12
	ldr	x8, [x8, #1008]
	ldr	x9, [x8, #224]
	add	w8, w22, w20
	ldrb	w11, [x9, #132]
	ldr	x9, [x9]
	tbz	w11, #6, .LBB62_215
// %bb.214:
	ldr	x11, [x21, #240]
	orr	x9, x9, x11, lsl #32
.LBB62_215:
	add	x11, x10, #4084         // =4084
	mov	x0, x19
	mov	x1, x25
	str	x9, [x11]
	str	w8, [x10, #4092]
	bl	f2fs_inode_chksum_set
.LBB62_216:
	ldur	x25, [x29, #-40]        // 8-byte Folded Reload
	mov	w20, wzr
	cbz	x25, .LBB62_223
// %bb.217:
	ldur	w8, [x29, #-60]         // 4-byte Folded Reload
	tbz	w8, #0, .LBB62_223
// %bb.218:
	ldr	w9, [x19, #1432]
	mov	w8, #1
	lsl	w9, w8, w9
	cmp	w9, #2                  // =2
	b.lt	.LBB62_220
// %bb.219:
	strb	wzr, [x25, #90]
.LBB62_220:
	add	x20, x25, #64           // =64
	str	x20, [x25, #64]
	strb	w8, [x25, #88]
	ldp	w9, w8, [x25, #12]
	str	x20, [x25, #72]
	mov	w10, #296
	add	x9, x19, x9, lsl #3
	ldr	x9, [x9, #144]
	madd	x21, x8, x10, x9
	add	x22, x21, #208          // =208
	mov	x0, x22
	bl	_raw_spin_lock
	ldr	x23, [x21, #224]!
	mov	x0, x20
	sub	x24, x21, #8            // =8
	mov	x1, x23
	mov	x2, x24
	bl	__list_add_valid
	tbz	w0, #0, .LBB62_222
// %bb.221:
	str	x20, [x21]
	stp	x24, x23, [x25, #64]
	str	x20, [x23]
.LBB62_222:
	mov	x0, x22
	bl	_raw_spin_unlock
	mov	w20, wzr
.LBB62_223:
	mov	x0, x26
	bl	mutex_unlock
	ldr	x8, [x19, #136]
	add	x0, x8, #32             // =32
	bl	up_read
	mov	w0, w20
	ldp	x20, x19, [sp, #208]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #192]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #176]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #160]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #144]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #128]    // 16-byte Folded Reload
	add	sp, sp, #224            // =224
	ret
.LBB62_224:
	mov	x0, x25
	bl	__page_file_mapping
	b	.LBB62_210
.LBB62_225:
	mov	x0, x25
	bl	__page_file_mapping
	b	.LBB62_213
.Lfunc_end62:
	.size	f2fs_allocate_data_block, .Lfunc_end62-f2fs_allocate_data_block
                                        // -- End function
	.globl	f2fs_update_device_state // -- Begin function f2fs_update_device_state
	.p2align	2
	.type	f2fs_update_device_state,@function
f2fs_update_device_state:               // @f2fs_update_device_state
.Lf2fs_update_device_state$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	w8, [x0, #3128]
	mov	x29, sp
	cbz	w8, .LBB63_3
// %bb.1:
	mov	w20, w1
	mov	w1, w2
	mov	x19, x0
	bl	f2fs_target_device_index
	mov	w21, w0
	mvn	w22, w0
	mov	w3, #4
	mov	x0, x19
	mov	w1, w20
	mov	w2, w21
	bl	f2fs_set_dirty_device
	mov	w8, w21
	add	x20, x19, #3144         // =3144
	lsr	x21, x8, #3
	ldrb	w8, [x20, x21]
	and	w9, w22, #0x7
	mov	w10, #1
	lsl	w22, w10, w9
	tst	w22, w8
	b.ne	.LBB63_3
// %bb.2:
	add	x19, x19, #3148         // =3148
	mov	x0, x19
	bl	_raw_spin_lock
	ldrb	w8, [x20, x21]
	mov	x0, x19
	orr	w8, w8, w22
	strb	w8, [x20, x21]
	bl	_raw_spin_unlock
.LBB63_3:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end63:
	.size	f2fs_update_device_state, .Lfunc_end63-f2fs_update_device_state
                                        // -- End function
	.globl	f2fs_do_write_meta_page // -- Begin function f2fs_do_write_meta_page
	.p2align	2
	.type	f2fs_do_write_meta_page,@function
f2fs_do_write_meta_page:                // @f2fs_do_write_meta_page
.Lf2fs_do_write_meta_page$local:
// %bb.0:
	sub	sp, sp, #192            // =192
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #144]    // 16-byte Folded Spill
	add	x29, sp, #144           // =144
	str	x21, [sp, #160]         // 8-byte Folded Spill
	stp	x20, x19, [sp, #176]    // 16-byte Folded Spill
	mov	x9, #8589934592
	stur	x8, [x29, #-8]
	mov	x8, #4294967296
	stp	xzr, xzr, [sp]
	stp	x9, xzr, [sp, #8]
	stp	xzr, xzr, [sp, #120]
	stp	xzr, xzr, [sp, #104]
	stp	xzr, xzr, [sp, #88]
	stp	xzr, xzr, [sp, #72]
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	str	x0, [sp]
	str	x8, [sp, #16]
	ldr	x8, [x1, #32]
	mov	w9, #14336
	stp	x1, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #64]
	str	xzr, [sp, #56]
	strb	wzr, [sp, #80]
	str	wzr, [sp, #84]
	strh	wzr, [sp, #88]
	strb	wzr, [sp, #90]
	str	wzr, [sp, #92]
	strb	wzr, [sp, #96]
	str	wzr, [sp, #100]
	stp	xzr, xzr, [sp, #104]
	str	xzr, [sp, #120]
	strb	wzr, [sp, #128]
	stp	w9, w8, [sp, #24]
	str	w8, [sp, #32]
	str	wzr, [sp, #132]
	ldr	x9, [x0, #136]
	mov	x21, x1
	mov	x19, x0
	mov	w20, w2
	cbz	x9, .LBB64_2
// %bb.1:
	add	x9, x9, #84             // =84
	b	.LBB64_3
.LBB64_2:
	ldr	x9, [x19, #16]
	add	x9, x9, #92             // =92
.LBB64_3:
	ldr	w9, [x9]
	cmp	x8, x9
	b.hs	.LBB64_33
.LBB64_4:
	mov	x0, x21
	mov	w1, wzr
	bl	__test_set_page_writeback
	ldr	x8, [x21, #8]
	mov	x0, sp
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x21, x9, eq
	mov	w9, #256
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	bl	f2fs_submit_page_write
	ldr	x9, [x19, #136]
	ldr	x8, [x21, #32]
	ldr	x10, [x9]
	ldr	w10, [x10, #8]
	cmp	x8, x10
	b.hs	.LBB64_6
// %bb.5:
	add	x8, x19, #2128          // =2128
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB64_12
.LBB64_6:
	ldr	x10, [x19, #120]
	ldr	w10, [x10]
	cmp	x8, x10
	b.hs	.LBB64_8
// %bb.7:
	add	x8, x19, #2136          // =2136
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	b	.LBB64_12
.LBB64_8:
	ldr	w10, [x9, #88]
	cmp	x8, x10
	add	x10, x19, #2132         // =2132
	b.hs	.LBB64_10
// %bb.9:
	//APP
	// atomic_add
	prfm	pstl1strm, [x10]
1:	ldxr	w8, [x10]
	add	w8, w8, 1
	stxr	w9, w8, [x10]
	cbnz	w9, 1b

	//NO_APP
	b	.LBB64_12
.LBB64_10:
	ldr	w9, [x9, #84]
	cmp	x8, x9
	b.hs	.LBB64_12
// %bb.11:
	add	x8, x10, #8             // =8
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB64_12:
	ldrb	w8, [x19, #2704]
	cbz	w8, .LBB64_18
// %bb.13:
	add	x21, x19, #2344         // =2344
	mov	x0, x21
	bl	_raw_spin_lock
	add	x8, x19, w20, uxtw #3
	ldr	x9, [x8, #2352]
	tst	w20, #0xfffffffd
	add	x9, x9, #1, lsl #12     // =4096
	str	x9, [x8, #2352]
	b.ne	.LBB64_15
// %bb.14:
	ldr	x8, [x19, #2368]
	ldr	x9, [x19, #2352]
	sub	x8, x8, x9
	str	x8, [x19, #2360]
.LBB64_15:
	orr	w8, w20, #0x2
	cmp	w8, #14                 // =14
	b.ne	.LBB64_17
// %bb.16:
	ldr	x8, [x19, #2464]
	ldr	x9, [x19, #2448]
	sub	x8, x8, x9
	str	x8, [x19, #2456]
.LBB64_17:
	mov	x0, x21
	bl	_raw_spin_unlock
	mov	x0, x19
	bl	f2fs_record_iostat
.LBB64_18:
	add	x20, x19, #2296         // =2296
	mov	x0, x20
	bl	mutex_lock
	ldr	x9, [x19, #16]
	ldr	w8, [sp, #28]
	ldr	w10, [x9, #76]
	cmp	w8, w10
	b.lo	.LBB64_22
// %bb.19:
	ldr	w10, [x9, #80]
	cmp	w8, w10
	b.hs	.LBB64_22
// %bb.20:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB64_31
// %bb.21:
	ldr	x9, [x8, #376]
	add	x9, x9, #1              // =1
	str	x9, [x8, #376]
	b	.LBB64_31
.LBB64_22:
	ldr	w10, [x9, #84]
	cmp	w8, w10
	b.hs	.LBB64_25
// %bb.23:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB64_31
// %bb.24:
	ldr	x9, [x8, #384]
	add	x9, x9, #1              // =1
	str	x9, [x8, #384]
	b	.LBB64_31
.LBB64_25:
	ldr	w10, [x9, #88]
	cmp	w8, w10
	b.hs	.LBB64_28
// %bb.26:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB64_31
// %bb.27:
	ldr	x9, [x8, #392]
	add	x9, x9, #1              // =1
	str	x9, [x8, #392]
	b	.LBB64_31
.LBB64_28:
	ldr	w9, [x9, #92]
	cmp	w8, w9
	b.hs	.LBB64_31
// %bb.29:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB64_31
// %bb.30:
	ldr	x9, [x8, #400]
	add	x9, x9, #1              // =1
	str	x9, [x8, #400]
.LBB64_31:
	mov	x0, x20
	bl	mutex_unlock
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB64_34
// %bb.32:
	ldp	x20, x19, [sp, #176]    // 16-byte Folded Reload
	ldr	x21, [sp, #160]         // 8-byte Folded Reload
	ldp	x29, x30, [sp, #144]    // 16-byte Folded Reload
	add	sp, sp, #192            // =192
	ret
.LBB64_33:
	mov	w8, #10240
	str	w8, [sp, #24]
	b	.LBB64_4
.LBB64_34:
	bl	__stack_chk_fail
.Lfunc_end64:
	.size	f2fs_do_write_meta_page, .Lfunc_end64-f2fs_do_write_meta_page
                                        // -- End function
	.globl	f2fs_do_write_node_page // -- Begin function f2fs_do_write_node_page
	.p2align	2
	.type	f2fs_do_write_node_page,@function
f2fs_do_write_node_page:                // @f2fs_do_write_node_page
.Lf2fs_do_write_node_page$local:
// %bb.0:
	sub	sp, sp, #64             // =64
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	str	x21, [sp, #32]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	str	x8, [sp, #8]
	str	w0, [sp]
	mov	x0, sp
	add	x29, sp, #16            // =16
	mov	x20, x1
	sturh	wzr, [sp, #5]
	strb	wzr, [sp, #4]
	bl	do_write_page
	ldr	x19, [x20]
	ldrb	w8, [x19, #2704]
	cbz	w8, .LBB65_6
// %bb.1:
	ldr	w21, [x20, #100]
	add	x20, x19, #2344         // =2344
	mov	x0, x20
	bl	_raw_spin_lock
	add	x8, x19, x21, lsl #3
	ldr	x9, [x8, #2352]
	tst	w21, #0xfffffffd
	add	x9, x9, #1, lsl #12     // =4096
	str	x9, [x8, #2352]
	b.ne	.LBB65_3
// %bb.2:
	ldr	x8, [x19, #2368]
	ldr	x9, [x19, #2352]
	sub	x8, x8, x9
	str	x8, [x19, #2360]
.LBB65_3:
	orr	w8, w21, #0x2
	cmp	w8, #14                 // =14
	b.ne	.LBB65_5
// %bb.4:
	ldr	x8, [x19, #2464]
	ldr	x9, [x19, #2448]
	sub	x8, x8, x9
	str	x8, [x19, #2456]
.LBB65_5:
	mov	x0, x20
	bl	_raw_spin_unlock
	mov	x0, x19
	bl	f2fs_record_iostat
.LBB65_6:
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB65_8
// %bb.7:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldr	x21, [sp, #32]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #64             // =64
	ret
.LBB65_8:
	bl	__stack_chk_fail
.Lfunc_end65:
	.size	f2fs_do_write_node_page, .Lfunc_end65-f2fs_do_write_node_page
                                        // -- End function
	.p2align	2               // -- Begin function do_write_page
	.type	do_write_page,@function
do_write_page:                          // @do_write_page
// %bb.0:
	sub	sp, sp, #80             // =80
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	mov	x20, x0
	mov	x0, x1
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	add	x29, sp, #16            // =16
	mov	x19, x1
	bl	__get_segment_type
	mov	w21, w0
	ldr	x0, [x19]
	ldr	w8, [x0, #1504]
	cmp	w8, #1                  // =1
	cset	w8, eq
	cmp	w21, #2                 // =2
	cset	w9, eq
	and	w23, w9, w8
	cmp	w23, #1                 // =1
	b.ne	.LBB66_2
// %bb.1:
	add	x0, x0, #168            // =168
	bl	down_read
	ldr	x0, [x19]
.LBB66_2:
	ldr	x1, [x19, #40]
	ldr	w2, [x19, #32]
	add	x22, x19, #28           // =28
	mov	w7, #1
	mov	x3, x22
	mov	x4, x20
	mov	w5, w21
	mov	x6, x19
	str	wzr, [sp]
	bl	f2fs_allocate_data_block
	cbz	w0, .LBB66_20
.LBB66_3:
	mov	w20, #-12
	cbz	w23, .LBB66_5
.LBB66_4:
	ldr	x8, [x19]
	add	x0, x8, #168            // =168
	bl	up_read
.LBB66_5:
	mov	w0, w20
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #80             // =80
	ret
.LBB66_6:                               //   in Loop: Header=BB66_20 Depth=1
	ldr	w2, [x19, #28]
	ldr	x0, [x19]
	ldr	x1, [x19, #40]
	mov	w7, #1
	mov	x3, x22
	mov	x4, x20
	mov	w5, w21
	mov	x6, x19
	str	w2, [x19, #32]
	str	wzr, [sp]
	bl	f2fs_allocate_data_block
	cbz	w0, .LBB66_20
	b	.LBB66_3
.LBB66_7:                               //   in Loop: Header=BB66_20 Depth=1
	ldr	x8, [x19]
	ldr	w10, [x8, #3128]
	cbz	w10, .LBB66_15
// %bb.8:                               //   in Loop: Header=BB66_20 Depth=1
	ldr	x9, [x8]
	cmp	w10, #1                 // =1
	ldr	x9, [x9, #248]
	b.lt	.LBB66_14
// %bb.9:                               //   in Loop: Header=BB66_20 Depth=1
	ldr	x11, [x8, #3136]
	add	x11, x11, #80           // =80
	b	.LBB66_11
.LBB66_10:                              //   in Loop: Header=BB66_11 Depth=2
	subs	x10, x10, #1            // =1
	add	x11, x11, #88           // =88
	b.eq	.LBB66_14
.LBB66_11:                              //   Parent Loop BB66_20 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w12, [x11, #-4]
	cmp	w12, w1
	b.hi	.LBB66_10
// %bb.12:                              //   in Loop: Header=BB66_11 Depth=2
	ldr	w12, [x11]
	cmp	w12, w1
	b.lo	.LBB66_10
// %bb.13:                              //   in Loop: Header=BB66_20 Depth=1
	ldur	x9, [x11, #-80]
.LBB66_14:                              //   in Loop: Header=BB66_20 Depth=1
	cbz	x9, .LBB66_21
.LBB66_15:                              //   in Loop: Header=BB66_20 Depth=1
	ldr	x9, [x8, #136]
	cbz	x9, .LBB66_17
// %bb.16:                              //   in Loop: Header=BB66_20 Depth=1
	add	x10, x9, #80            // =80
	b	.LBB66_18
.LBB66_17:                              //   in Loop: Header=BB66_20 Depth=1
	ldr	x10, [x8, #16]
	add	x10, x10, #72           // =72
.LBB66_18:                              //   in Loop: Header=BB66_20 Depth=1
	ldr	x9, [x9, #8]
	ldr	w10, [x10]
	ldr	w11, [x8, #1184]
	ldr	w9, [x9]
	sub	w10, w1, w10
	lsr	w10, w10, w11
	sub	w9, w10, w9
	cmn	w9, #1                  // =1
	b.eq	.LBB66_21
// %bb.19:                              //   in Loop: Header=BB66_20 Depth=1
	ldr	x8, [x8, #248]
	mov	x2, x1
	ldr	x0, [x8, #48]
	bl	invalidate_mapping_pages
	ldr	x0, [x19]
	ldr	w1, [x19, #32]
	bl	f2fs_invalidate_compress_page
	b	.LBB66_21
.LBB66_20:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB66_11 Depth 2
	ldr	w1, [x19, #32]
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB66_7
.LBB66_21:                              //   in Loop: Header=BB66_20 Depth=1
	mov	x0, x19
	bl	f2fs_submit_page_write
	ldrb	w8, [x19, #90]
	cbnz	w8, .LBB66_6
// %bb.22:
	ldr	x20, [x19]
	ldr	w8, [x20, #3128]
	cbz	w8, .LBB66_24
// %bb.23:
	ldr	w1, [x19, #28]
	ldr	w21, [x19, #8]
	mov	x0, x20
	bl	f2fs_target_device_index
	mov	w22, w0
	mvn	w24, w0
	mov	w3, #4
	mov	x0, x20
	mov	w1, w21
	mov	w2, w22
	bl	f2fs_set_dirty_device
	mov	w8, w22
	add	x21, x20, #3144         // =3144
	lsr	x22, x8, #3
	ldrb	w8, [x21, x22]
	and	w9, w24, #0x7
	mov	w10, #1
	lsl	w24, w10, w9
	tst	w24, w8
	b.eq	.LBB66_25
.LBB66_24:
	mov	w20, wzr
	cbz	w23, .LBB66_5
	b	.LBB66_4
.LBB66_25:
	add	x20, x20, #3148         // =3148
	mov	x0, x20
	bl	_raw_spin_lock
	ldrb	w8, [x21, x22]
	mov	x0, x20
	orr	w8, w8, w24
	strb	w8, [x21, x22]
	bl	_raw_spin_unlock
	mov	w20, wzr
	cbz	w23, .LBB66_5
	b	.LBB66_4
.Lfunc_end66:
	.size	do_write_page, .Lfunc_end66-do_write_page
                                        // -- End function
	.globl	f2fs_outplace_write_data // -- Begin function f2fs_outplace_write_data
	.p2align	2
	.type	f2fs_outplace_write_data,@function
f2fs_outplace_write_data:               // @f2fs_outplace_write_data
.Lf2fs_outplace_write_data$local:
// %bb.0:
	sub	sp, sp, #64             // =64
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	str	x8, [sp, #8]
	ldr	x19, [x1]
	stur	wzr, [sp, #3]
	str	wzr, [sp]
	ldr	w8, [x0, #36]
	mov	x20, x1
	mov	x21, x0
	add	x29, sp, #16            // =16
	cbz	w8, .LBB67_12
.LBB67_1:
	ldp	w8, w9, [x21, #24]
	ldrb	w10, [x20, #128]
	mov	x0, sp
	mov	x1, x20
	str	w8, [sp]
	sturh	w9, [sp, #5]
	strb	w10, [sp, #4]
	bl	do_write_page
	cbz	w0, .LBB67_3
// %bb.2:
	mov	w0, #-12
	b	.LBB67_10
.LBB67_3:
	ldr	w1, [x20, #28]
	mov	x0, x21
	bl	f2fs_update_data_blkaddr
	ldrb	w8, [x19, #2704]
	cbz	w8, .LBB67_9
// %bb.4:
	ldr	w21, [x20, #100]
	add	x20, x19, #2344         // =2344
	mov	x0, x20
	bl	_raw_spin_lock
	add	x8, x19, x21, lsl #3
	ldr	x9, [x8, #2352]
	tst	w21, #0xfffffffd
	add	x9, x9, #1, lsl #12     // =4096
	str	x9, [x8, #2352]
	b.ne	.LBB67_6
// %bb.5:
	ldr	x8, [x19, #2368]
	ldr	x9, [x19, #2352]
	sub	x8, x8, x9
	str	x8, [x19, #2360]
.LBB67_6:
	orr	w8, w21, #0x2
	cmp	w8, #14                 // =14
	b.ne	.LBB67_8
// %bb.7:
	ldr	x8, [x19, #2464]
	ldr	x9, [x19, #2448]
	sub	x8, x8, x9
	str	x8, [x19, #2456]
.LBB67_8:
	mov	x0, x20
	bl	_raw_spin_unlock
	mov	x0, x19
	bl	f2fs_record_iostat
.LBB67_9:
	mov	w0, wzr
.LBB67_10:
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB67_20
// %bb.11:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #64             // =64
	ret
.LBB67_12:
	bl	print_bdev_access_info
	adrp	x22, f2fs_dclient
	ldr	x0, [x22, :lo12:f2fs_dclient]
	cbz	x0, .LBB67_15
// %bb.13:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB67_15
// %bb.14:
	ldr	x0, [x22, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_outplace_write_data
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_outplace_write_data
	mov	w3, #5430
	bl	dsm_client_record
	ldr	x0, [x22, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB67_15:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 5430; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB67_1
// %bb.16:
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
	cbnz	w8, .LBB67_1
// %bb.17:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB67_1
// %bb.18:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB67_1
// %bb.19:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB67_1
.LBB67_20:
	bl	__stack_chk_fail
.Lfunc_end67:
	.size	f2fs_outplace_write_data, .Lfunc_end67-f2fs_outplace_write_data
                                        // -- End function
	.globl	f2fs_inplace_write_data // -- Begin function f2fs_inplace_write_data
	.p2align	2
	.type	f2fs_inplace_write_data,@function
f2fs_inplace_write_data:                // @f2fs_inplace_write_data
.Lf2fs_inplace_write_data$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	w8, [x0, #32]
	ldr	x20, [x0]
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x19, x0
	str	w8, [x0, #28]
	bl	__get_segment_type
	ldr	w1, [x19, #28]
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB68_2
.LBB68_1:
	mov	w21, #-1
	b	.LBB68_14
.LBB68_2:
	ldr	w9, [x20, #3128]
	cbz	w9, .LBB68_10
// %bb.3:
	ldr	x8, [x20]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB68_9
// %bb.4:
	ldr	x10, [x20, #3136]
	add	x10, x10, #80           // =80
	b	.LBB68_6
.LBB68_5:                               //   in Loop: Header=BB68_6 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB68_9
.LBB68_6:                               // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB68_5
// %bb.7:                               //   in Loop: Header=BB68_6 Depth=1
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB68_5
// %bb.8:
	ldur	x8, [x10, #-80]
.LBB68_9:
	cbz	x8, .LBB68_1
.LBB68_10:
	ldr	x8, [x20, #136]
	cbz	x8, .LBB68_12
// %bb.11:
	add	x9, x8, #80             // =80
	b	.LBB68_13
.LBB68_12:
	ldr	x9, [x20, #16]
	add	x9, x9, #72             // =72
.LBB68_13:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x20, #1184]
	ldr	w8, [x8]
	sub	w9, w1, w9
	lsr	w9, w9, w10
	sub	w21, w9, w8
.LBB68_14:
	ldr	x8, [x20, #136]
	mov	w9, #40
	umull	x9, w21, w9
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	ldr	w8, [x8, x9]
	and	w8, w8, #0x3f
	cmp	w8, #3                  // =3
	b.hs	.LBB68_21
// %bb.15:
	ldr	x8, [x20, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB68_27
// %bb.16:
	ldr	x8, [x20, #248]
	mov	x2, x1
	ldr	x0, [x8, #48]
	bl	invalidate_mapping_pages
	ldr	x8, [x19]
	add	x21, x20, #2296         // =2296
	mov	x0, x21
	add	x8, x8, #2168           // =2168
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	bl	mutex_lock
	ldr	x8, [x20, #2328]
	cbz	x8, .LBB68_18
// %bb.17:
	ldr	w9, [x8, #168]
	add	w9, w9, #1              // =1
	str	w9, [x8, #168]
.LBB68_18:
	mov	x0, x21
	bl	mutex_unlock
	ldr	x8, [x19, #112]
	cbz	x8, .LBB68_30
// %bb.19:
	ldr	x8, [x20, #136]
	ldrb	w8, [x8, #136]
	tbnz	w8, #6, .LBB68_30
// %bb.20:
	mov	x0, x19
	bl	f2fs_merge_page_bio
	mov	w20, w0
	cbnz	w0, .LBB68_41
	b	.LBB68_31
.LBB68_21:
	cbz	x20, .LBB68_26
// %bb.22:
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
	cbnz	w8, .LBB68_26
// %bb.23:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB68_26
// %bb.24:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB68_26
// %bb.25:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB68_26:
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.f2fs_inplace_write_data
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.f2fs_inplace_write_data
	mov	x0, x20
	mov	w3, w21
	bl	f2fs_printk
	mov	w20, #-117
	ldr	x8, [x19, #112]
	cbnz	x8, .LBB68_28
	b	.LBB68_41
.LBB68_27:
	mov	w20, #-5
	ldr	x8, [x19, #112]
	cbz	x8, .LBB68_41
.LBB68_28:
	ldr	x0, [x8]
	cbz	x0, .LBB68_41
// %bb.29:
	mov	w8, #10
	strb	w8, [x0, #26]
	bl	bio_endio
	ldr	x8, [x19, #112]
	str	xzr, [x8]
	b	.LBB68_41
.LBB68_30:
	mov	x0, x19
	bl	f2fs_submit_page_bio
	mov	w20, w0
	cbnz	w0, .LBB68_41
.LBB68_31:
	ldr	x20, [x19]
	ldr	w8, [x20, #3128]
	cbz	w8, .LBB68_34
// %bb.32:
	ldr	w1, [x19, #28]
	ldr	w21, [x19, #8]
	mov	x0, x20
	bl	f2fs_target_device_index
	mov	w22, w0
	mvn	w23, w0
	mov	w3, #4
	mov	x0, x20
	mov	w1, w21
	mov	w2, w22
	bl	f2fs_set_dirty_device
	mov	w8, w22
	add	x21, x20, #3144         // =3144
	lsr	x22, x8, #3
	ldrb	w8, [x21, x22]
	and	w9, w23, #0x7
	mov	w10, #1
	lsl	w23, w10, w9
	tst	w23, w8
	b.ne	.LBB68_34
// %bb.33:
	add	x20, x20, #3148         // =3148
	mov	x0, x20
	bl	_raw_spin_lock
	ldrb	w8, [x21, x22]
	mov	x0, x20
	orr	w8, w8, w23
	strb	w8, [x21, x22]
	bl	_raw_spin_unlock
.LBB68_34:
	ldr	x20, [x19]
	ldrb	w8, [x20, #2704]
	cbz	w8, .LBB68_40
// %bb.35:
	ldr	w21, [x19, #100]
	add	x19, x20, #2344         // =2344
	mov	x0, x19
	bl	_raw_spin_lock
	add	x8, x20, x21, lsl #3
	ldr	x9, [x8, #2352]
	tst	w21, #0xfffffffd
	add	x9, x9, #1, lsl #12     // =4096
	str	x9, [x8, #2352]
	b.ne	.LBB68_37
// %bb.36:
	ldr	x8, [x20, #2368]
	ldr	x9, [x20, #2352]
	sub	x8, x8, x9
	str	x8, [x20, #2360]
.LBB68_37:
	orr	w8, w21, #0x2
	cmp	w8, #14                 // =14
	b.ne	.LBB68_39
// %bb.38:
	ldr	x8, [x20, #2464]
	ldr	x9, [x20, #2448]
	sub	x8, x8, x9
	str	x8, [x20, #2456]
.LBB68_39:
	mov	x0, x19
	bl	_raw_spin_unlock
	mov	x0, x20
	bl	f2fs_record_iostat
.LBB68_40:
	mov	w20, wzr
.LBB68_41:
	mov	w0, w20
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end68:
	.size	f2fs_inplace_write_data, .Lfunc_end68-f2fs_inplace_write_data
                                        // -- End function
	.p2align	2               // -- Begin function __get_segment_type
	.type	__get_segment_type,@function
__get_segment_type:                     // @__get_segment_type
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x9, [x0]
	mov	x19, x0
	mov	x29, sp
	ldr	w8, [x9, #1448]
	cmp	w8, #6                  // =6
	b.eq	.LBB69_4
// %bb.1:
	cmp	w8, #4                  // =4
	b.eq	.LBB69_8
// %bb.2:
	cmp	w8, #2                  // =2
	b.ne	.LBB69_12
// %bb.3:
	ldr	w8, [x19, #12]
	str	wzr, [x19, #16]
	cmp	w8, #0                  // =0
	mov	w8, #3
	csel	w0, wzr, w8, eq
	b	.LBB69_62
.LBB69_4:
	ldr	w8, [x19, #12]
	ldr	x10, [x19, #40]
	cbz	w8, .LBB69_20
// %bb.5:
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x8, x10, x8
	lsl	x8, x8, #6
	mov	x9, #-549755813888
	and	x8, x8, #0xfffffffffffff000
	movk	x9, #4080
	ldr	w8, [x8, x9]
	mov	w0, #5
	lsr	w9, w8, #3
	cmp	w9, #2040               // =2040
	b.gt	.LBB69_37
// %bb.6:
	cmp	w9, #3                  // =3
	b.eq	.LBB69_59
// %bb.7:
	cmp	w9, #1022               // =1022
	b.eq	.LBB69_59
	b	.LBB69_39
.LBB69_8:
	ldr	w9, [x19, #12]
	ldr	x8, [x19, #40]
	cbz	w9, .LBB69_34
// %bb.9:
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	add	x8, x8, x9
	lsl	x8, x8, #6
	mov	x9, #-549755813888
	and	x8, x8, #0xfffffffffffff000
	movk	x9, #4080
	ldr	w8, [x8, x9]
	mov	w0, #5
	lsr	w9, w8, #3
	cmp	w9, #2040               // =2040
	b.gt	.LBB69_43
// %bb.10:
	cmp	w9, #3                  // =3
	b.eq	.LBB69_59
// %bb.11:
	cmp	w9, #1022               // =1022
	b.eq	.LBB69_59
	b	.LBB69_45
.LBB69_12:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB69_15
// %bb.13:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB69_15
// %bb.14:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__get_segment_type
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__get_segment_type
	mov	w3, #4991
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB69_15:
	ldr	x0, [x19]
	bl	f2fs_print_raw_sb_info
	ldr	x0, [x19]
	bl	f2fs_print_ckpt_info
	ldr	x0, [x19]
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 4991; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x8, [x19]
	cbz	x8, .LBB69_35
// %bb.16:
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
	cbnz	w9, .LBB69_35
// %bb.17:
	ldr	x1, [x8, #3392]
	mov	w0, wzr
	cbz	x1, .LBB69_36
// %bb.18:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB69_36
// %bb.19:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB69_35
.LBB69_20:
	ldr	x8, [x10, #24]
	ldr	x8, [x8]
	ldr	x11, [x8, #736]
	tbnz	x11, #36, .LBB69_50
// %bb.21:
	ldr	x11, [x10]
	tbz	w11, #13, .LBB69_24
// %bb.22:
	ldr	x11, [x10, #40]
	tbz	w11, #0, .LBB69_24
// %bb.23:
	ldr	x10, [x10, #40]
	tbnz	w10, #3, .LBB69_51
.LBB69_24:
	ldrb	w9, [x8, #712]
	tbnz	w9, #0, .LBB69_58
// %bb.25:
	ldrh	w10, [x8]
	and	w10, w10, #0xf000
	cmp	w10, #8, lsl #12        // =32768
	b.ne	.LBB69_27
// %bb.26:
	ldr	x10, [x8, #40]
	ldr	x10, [x10, #1008]
	ldr	w10, [x10, #1532]
	ldr	x11, [x8, #736]
	tbnz	w11, #31, .LBB69_55
.LBB69_27:
	tbnz	w9, #5, .LBB69_35
// %bb.28:
	ldr	x9, [x8, #736]
	tbnz	w9, #25, .LBB69_35
// %bb.29:
	ldr	x9, [x8, #736]
	tbnz	w9, #15, .LBB69_35
// %bb.30:
	ldr	x9, [x8, #736]
	tbnz	w9, #17, .LBB69_35
// %bb.31:
	ldrb	w8, [x8, #143]
	cmp	w8, #2                  // =2
	b.eq	.LBB69_35
// %bb.32:
	cmp	w8, #5                  // =5
	add	x8, x19, #16            // =16
	b.ne	.LBB69_63
// %bb.33:
	mov	w0, #2
	b	.LBB69_60
.LBB69_34:
	ldr	x8, [x8, #24]
	ldr	x8, [x8]
	ldrh	w8, [x8]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.ne	.LBB69_58
.LBB69_35:
	mov	w0, wzr
.LBB69_36:
	str	wzr, [x19, #16]
	b	.LBB69_62
.LBB69_37:
	cmp	w9, #2041               // =2041
	b.eq	.LBB69_59
// %bb.38:
	mov	w10, #536870911
	cmp	w9, w10
	b.eq	.LBB69_41
.LBB69_39:
	lsr	w10, w8, #4
	cmp	w10, #1021              // =1021
	b.lo	.LBB69_41
// %bb.40:
	mov	w10, #36659
	mov	w11, #49970
	movk	w10, #10354, lsl #16
	movk	w11, #24117, lsl #16
	madd	w9, w9, w10, w11
	mov	w10, #20581
	movk	w10, #64, lsl #16
	cmp	w9, w10
	b.lo	.LBB69_48
.LBB69_41:
	tbnz	w8, #0, .LBB69_49
// %bb.42:
	mov	w0, #3
	str	wzr, [x19, #16]
	b	.LBB69_62
.LBB69_43:
	cmp	w9, #2041               // =2041
	b.eq	.LBB69_59
// %bb.44:
	mov	w10, #536870911
	cmp	w9, w10
	b.eq	.LBB69_47
.LBB69_45:
	lsr	w10, w8, #4
	cmp	w10, #1021              // =1021
	b.lo	.LBB69_47
// %bb.46:
	mov	w10, #36659
	mov	w11, #49970
	movk	w10, #10354, lsl #16
	movk	w11, #24117, lsl #16
	madd	w9, w9, w10, w11
	mov	w10, #20581
	movk	w10, #64, lsl #16
	cmp	w9, w10
	b.lo	.LBB69_48
.LBB69_47:
	tbnz	w8, #0, .LBB69_49
.LBB69_48:
	mov	w0, #5
	b	.LBB69_59
.LBB69_49:
	add	x8, x19, #16            // =16
	mov	w0, #4
	mov	w9, #1
	b	.LBB69_61
.LBB69_50:
	mov	w0, #6
	b	.LBB69_59
.LBB69_51:
	ldrb	w8, [x9, #1920]
	cbz	w8, .LBB69_58
// %bb.52:
	ldr	w8, [x19, #100]
	cmp	w8, #4                  // =4
	b.ne	.LBB69_58
// %bb.53:
	ldr	w8, [x9, #1988]
	cmp	w8, #5                  // =5
	b.eq	.LBB69_58
// %bb.54:
	mov	w0, #7
	b	.LBB69_59
.LBB69_55:
	cbz	w10, .LBB69_58
// %bb.56:
	cmp	w10, #1                 // =1
	b.ne	.LBB69_27
// %bb.57:
	ldr	x10, [x8, #736]
	tbz	x10, #34, .LBB69_27
.LBB69_58:
	mov	w0, #2
.LBB69_59:
	add	x8, x19, #16            // =16
.LBB69_60:
	mov	w9, #2
.LBB69_61:
	str	w9, [x8]
.LBB69_62:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB69_63:
	mov	w0, #1
	mov	w9, #1
	b	.LBB69_61
.Lfunc_end69:
	.size	__get_segment_type, .Lfunc_end69-__get_segment_type
                                        // -- End function
	.globl	f2fs_do_replace_block   // -- Begin function f2fs_do_replace_block
	.p2align	2
	.type	f2fs_do_replace_block,@function
f2fs_do_replace_block:                  // @f2fs_do_replace_block
.Lf2fs_do_replace_block$local:
// %bb.0:
	sub	sp, sp, #160            // =160
	stp	x29, x30, [sp, #64]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #80]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #96]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #112]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #128]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #144]    // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	add	w9, w3, #3              // =3
	add	x29, sp, #64            // =64
	mov	w21, w3
	ldr	x23, [x8]
	mov	w22, w2
	mov	x19, x0
	cmp	w9, #4                  // =4
	str	w6, [sp, #20]           // 4-byte Folded Spill
	stur	w4, [x29, #-4]          // 4-byte Folded Spill
	stur	w5, [x29, #-28]         // 4-byte Folded Spill
	stur	x1, [x29, #-16]         // 8-byte Folded Spill
	b.hs	.LBB70_2
.LBB70_1:
	mov	w28, #-1
	b	.LBB70_14
.LBB70_2:
	ldr	w10, [x19, #3128]
	cbz	w10, .LBB70_10
// %bb.3:
	ldr	x9, [x19]
	cmp	w10, #1                 // =1
	ldr	x9, [x9, #248]
	b.lt	.LBB70_9
// %bb.4:
	ldr	x11, [x19, #3136]
	add	x11, x11, #80           // =80
	b	.LBB70_6
.LBB70_5:                               //   in Loop: Header=BB70_6 Depth=1
	subs	x10, x10, #1            // =1
	add	x11, x11, #88           // =88
	b.eq	.LBB70_9
.LBB70_6:                               // =>This Inner Loop Header: Depth=1
	ldur	w12, [x11, #-4]
	cmp	w12, w21
	b.hi	.LBB70_5
// %bb.7:                               //   in Loop: Header=BB70_6 Depth=1
	ldr	w12, [x11]
	cmp	w12, w21
	b.lo	.LBB70_5
// %bb.8:
	ldur	x9, [x11, #-80]
.LBB70_9:
	cbz	x9, .LBB70_1
.LBB70_10:
	cbz	x8, .LBB70_12
// %bb.11:
	add	x9, x8, #80             // =80
	b	.LBB70_13
.LBB70_12:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB70_13:
	ldr	x10, [x8, #8]
	ldr	w9, [x9]
	ldr	w11, [x19, #1184]
	ldr	w10, [x10]
	sub	w9, w21, w9
	lsr	w9, w9, w11
	sub	w28, w9, w10
.LBB70_14:
	ldr	x9, [x23, #120]
	mov	w10, #40
	add	x0, x8, #32             // =32
	umaddl	x24, w28, w10, x9
	ldr	w20, [x24]
	bl	down_write
	ldur	w8, [x29, #-4]          // 4-byte Folded Reload
	tbz	w8, #0, .LBB70_17
// %bb.15:
	ldr	x8, [x19, #136]
	ldr	x8, [x8, #24]
	ldr	w9, [x8, #100]
	cmp	w28, w9
	b.ne	.LBB70_27
// %bb.16:
	mov	w20, wzr
	b	.LBB70_61
.LBB70_17:
	ldrh	w8, [x24]
	tst	w8, #0xffc0
	b.ne	.LBB70_26
// %bb.18:
	ldr	x8, [x19, #136]
	ldr	x8, [x8, #24]
	ldr	w9, [x8, #100]
	cmp	w28, w9
	b.eq	.LBB70_26
// %bb.19:
	ldr	w9, [x8, #220]
	cmp	w28, w9
	b.eq	.LBB70_26
// %bb.20:
	ldr	w9, [x8, #340]
	cmp	w28, w9
	b.eq	.LBB70_26
// %bb.21:
	ldr	w9, [x8, #460]
	cmp	w28, w9
	b.eq	.LBB70_26
// %bb.22:
	ldr	w9, [x8, #580]
	cmp	w28, w9
	b.eq	.LBB70_26
// %bb.23:
	ldr	w9, [x8, #700]
	cmp	w28, w9
	b.eq	.LBB70_26
// %bb.24:
	ldr	w9, [x8, #820]
	cmp	w28, w9
	b.eq	.LBB70_26
// %bb.25:
	ldr	w8, [x8, #940]
	cmp	w28, w8
	b.ne	.LBB70_120
.LBB70_26:
	and	w20, w20, #0x3f
	cmp	w20, #3                 // =3
	b.lo	.LBB70_61
	b	.LBB70_53
.LBB70_27:
	ldr	w9, [x8, #220]
	cmp	w28, w9
	b.ne	.LBB70_29
.LBB70_28:
	mov	w20, #1
	b	.LBB70_61
.LBB70_29:
	ldr	w9, [x8, #340]
	cmp	w28, w9
	b.eq	.LBB70_35
// %bb.30:
	ldr	w9, [x8, #460]
	cmp	w28, w9
	b.eq	.LBB70_35
// %bb.31:
	ldr	w9, [x8, #580]
	cmp	w28, w9
	b.eq	.LBB70_35
// %bb.32:
	ldr	w9, [x8, #700]
	cmp	w28, w9
	b.eq	.LBB70_35
// %bb.33:
	ldr	w9, [x8, #820]
	cmp	w28, w9
	b.eq	.LBB70_35
// %bb.34:
	ldr	w9, [x8, #940]
	cmp	w28, w9
	b.ne	.LBB70_28
.LBB70_35:
	ldr	w9, [x8, #340]
	cmp	w9, w28
	b.ne	.LBB70_37
// %bb.36:
	mov	w20, #2
	b	.LBB70_61
.LBB70_37:
	ldr	w9, [x8, #460]
	cmp	w9, w28
	b.ne	.LBB70_39
// %bb.38:
	mov	w20, #3
	b	.LBB70_53
.LBB70_39:
	ldr	w9, [x8, #580]
	cmp	w9, w28
	b.ne	.LBB70_41
// %bb.40:
	mov	w20, #4
	b	.LBB70_53
.LBB70_41:
	ldr	w9, [x8, #700]
	cmp	w9, w28
	b.ne	.LBB70_43
// %bb.42:
	mov	w20, #5
	b	.LBB70_53
.LBB70_43:
	ldr	w9, [x8, #820]
	cmp	w9, w28
	b.ne	.LBB70_45
// %bb.44:
	mov	w20, #6
	b	.LBB70_53
.LBB70_45:
	ldr	w8, [x8, #940]
	cmp	w8, w28
	b.ne	.LBB70_47
// %bb.46:
	mov	w20, #7
	b	.LBB70_53
.LBB70_47:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB70_50
// %bb.48:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB70_50
// %bb.49:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_do_replace_block
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_do_replace_block
	mov	w3, #5558
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB70_50:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 5558; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB70_52
// %bb.51:
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
	cbz	w8, .LBB70_121
.LBB70_52:
	mov	w20, #8
.LBB70_53:
	bl	print_bdev_access_info
	adrp	x24, f2fs_dclient
	ldr	x0, [x24, :lo12:f2fs_dclient]
	cbz	x0, .LBB70_56
// %bb.54:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB70_56
// %bb.55:
	ldr	x0, [x24, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_do_replace_block
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_do_replace_block
	mov	w3, #5564
	bl	dsm_client_record
	ldr	x0, [x24, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB70_56:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 5564; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB70_61
// %bb.57:
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
	cbnz	w8, .LBB70_61
// %bb.58:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB70_61
// %bb.59:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB70_61
// %bb.60:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB70_61:
	ldr	x8, [x19, #136]
	mov	w27, w20
	ldr	x9, [x8, #24]
	mov	w8, #120
	umaddl	x26, w20, w8, x9
	mov	x0, x26
	str	x9, [sp, #24]           // 8-byte Folded Spill
	bl	mutex_lock
	add	x0, x23, #72            // =72
	stur	x0, [x29, #-24]         // 8-byte Folded Spill
	bl	down_write
	mov	x24, x26
	mov	x25, x26
	ldr	w23, [x24, #100]!
	ldrh	w8, [x25, #104]!
	mov	x20, x26
	cmp	w28, w23
	str	w8, [sp, #16]           // 4-byte Folded Spill
	ldrb	w8, [x20, #96]!
	str	w8, [sp, #12]           // 4-byte Folded Spill
	b.eq	.LBB70_63
// %bb.62:
	ldr	x9, [sp, #24]           // 8-byte Folded Reload
	mov	w8, #120
	mov	x0, x19
	mov	w1, w27
	madd	x8, x27, x8, x9
	str	w28, [x8, #112]
	bl	change_curseg
.LBB70_63:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB70_65
// %bb.64:
	add	x8, x8, #80             // =80
	b	.LBB70_66
.LBB70_65:
	ldr	x8, [x19, #16]
	add	x8, x8, #72             // =72
.LBB70_66:
	ldr	w8, [x8]
	ldr	w9, [x19, #1188]
	ldur	x1, [x29, #-16]         // 8-byte Folded Reload
	ldur	w28, [x29, #-4]         // 4-byte Folded Reload
	sub	w8, w21, w8
	sub	w9, w9, #1              // =1
	and	w8, w9, w8
	strh	w8, [x25]
	ldr	x8, [x19, #136]
	mov	w9, #120
	mov	w2, #7
	ldr	x8, [x8, #24]
	madd	x8, x27, x9, x8
	ldrh	w9, [x8, #104]
	ldr	x8, [x8, #32]
	lsl	x10, x9, #3
	sub	x9, x10, x9
	add	x0, x8, x9
	bl	memcpy
	tbz	w28, #0, .LBB70_68
// %bb.67:
	ldur	w8, [x29, #-28]         // 4-byte Folded Reload
	tbz	w8, #0, .LBB70_71
.LBB70_68:
	ldr	w8, [sp, #20]           // 4-byte Folded Reload
	tbnz	w8, #0, .LBB70_70
// %bb.69:
	mov	x0, x19
	mov	w1, w21
	mov	x2, xzr
	bl	update_segment_mtime
.LBB70_70:
	mov	w2, #1
	mov	x0, x19
	mov	w1, w21
	bl	update_sit_entry
.LBB70_71:
	add	w8, w22, #3             // =3
	cmp	w8, #4                  // =4
	b.lo	.LBB70_88
// %bb.72:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB70_80
// %bb.73:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB70_79
// %bb.74:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB70_76
.LBB70_75:                              //   in Loop: Header=BB70_76 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB70_79
.LBB70_76:                              // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w22
	b.hi	.LBB70_75
// %bb.77:                              //   in Loop: Header=BB70_76 Depth=1
	ldr	w11, [x10]
	cmp	w11, w22
	b.lo	.LBB70_75
// %bb.78:
	ldur	x8, [x10, #-80]
.LBB70_79:
	cbz	x8, .LBB70_87
.LBB70_80:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB70_82
// %bb.81:
	add	x9, x8, #80             // =80
	b	.LBB70_83
.LBB70_82:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB70_83:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w22, w9
	lsr	w9, w9, w10
	sub	w8, w9, w8
	cmn	w8, #1                  // =1
	b.eq	.LBB70_87
// %bb.84:
	ldr	x8, [x19, #248]
	mov	w1, w22
	mov	x2, x1
	ldr	x0, [x8, #48]
	bl	invalidate_mapping_pages
	mov	x0, x19
	mov	w1, w22
	bl	f2fs_invalidate_compress_page
	ldr	w8, [sp, #20]           // 4-byte Folded Reload
	tbnz	w8, #0, .LBB70_86
// %bb.85:
	mov	x0, x19
	mov	w1, w22
	mov	x2, xzr
	bl	update_segment_mtime
.LBB70_86:
	mov	w2, #-1
	mov	x0, x19
	mov	w1, w22
	bl	update_sit_entry
.LBB70_87:
	add	w8, w22, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB70_101
.LBB70_88:
	mov	w1, #-1
.LBB70_89:
	mov	x0, x19
	bl	locate_dirty_segment
	add	w8, w21, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB70_91
.LBB70_90:
	mov	w1, #-1
	b	.LBB70_113
.LBB70_91:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB70_99
// %bb.92:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB70_98
// %bb.93:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB70_95
.LBB70_94:                              //   in Loop: Header=BB70_95 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB70_98
.LBB70_95:                              // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w21
	b.hi	.LBB70_94
// %bb.96:                              //   in Loop: Header=BB70_95 Depth=1
	ldr	w11, [x10]
	cmp	w11, w21
	b.lo	.LBB70_94
// %bb.97:
	ldur	x8, [x10, #-80]
.LBB70_98:
	cbz	x8, .LBB70_90
.LBB70_99:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB70_111
// %bb.100:
	add	x9, x8, #80             // =80
	b	.LBB70_112
.LBB70_101:
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB70_109
// %bb.102:
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB70_108
// %bb.103:
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB70_105
.LBB70_104:                             //   in Loop: Header=BB70_105 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB70_108
.LBB70_105:                             // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w22
	b.hi	.LBB70_104
// %bb.106:                             //   in Loop: Header=BB70_105 Depth=1
	ldr	w11, [x10]
	cmp	w11, w22
	b.lo	.LBB70_104
// %bb.107:
	ldur	x8, [x10, #-80]
.LBB70_108:
	cbz	x8, .LBB70_88
.LBB70_109:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB70_118
// %bb.110:
	add	x9, x8, #80             // =80
	b	.LBB70_119
.LBB70_111:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB70_112:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w21, w9
	lsr	w9, w9, w10
	sub	w1, w9, w8
.LBB70_113:
	mov	x0, x19
	bl	locate_dirty_segment
	mov	x0, x19
	mov	w1, w23
	bl	locate_dirty_segment
	tbz	w28, #0, .LBB70_117
// %bb.114:
	ldr	w8, [x24]
	cmp	w23, w8
	b.eq	.LBB70_116
// %bb.115:
	ldr	x9, [sp, #24]           // 8-byte Folded Reload
	mov	w8, #120
	mov	x0, x19
	mov	w1, w27
	madd	x8, x27, x8, x9
	str	w23, [x8, #112]
	bl	change_curseg
.LBB70_116:
	ldr	w8, [sp, #16]           // 4-byte Folded Reload
	strh	w8, [x25]
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	strb	w8, [x20]
.LBB70_117:
	ldur	x0, [x29, #-24]         // 8-byte Folded Reload
	bl	up_write
	mov	x0, x26
	bl	mutex_unlock
	ldr	x8, [x19, #136]
	add	x0, x8, #32             // =32
	bl	up_write
	ldp	x20, x19, [sp, #144]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #128]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #112]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #96]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #80]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #64]     // 16-byte Folded Reload
	add	sp, sp, #160            // =160
	ret
.LBB70_118:
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB70_119:
	ldr	x8, [x8, #8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	ldr	w8, [x8]
	sub	w9, w22, w9
	lsr	w9, w9, w10
	sub	w1, w9, w8
	b	.LBB70_89
.LBB70_120:
	cmp	w22, #0                 // =0
	mov	w8, #1
	cinc	w20, w8, eq
	b	.LBB70_61
.LBB70_121:
	ldr	x1, [x19, #3392]
	mov	w20, #8
	cbz	x1, .LBB70_53
// %bb.122:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB70_53
// %bb.123:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	mov	w20, #8
	bl	queue_work_on
	b	.LBB70_53
.Lfunc_end70:
	.size	f2fs_do_replace_block, .Lfunc_end70-f2fs_do_replace_block
                                        // -- End function
	.globl	f2fs_replace_block      // -- Begin function f2fs_replace_block
	.p2align	2
	.type	f2fs_replace_block,@function
f2fs_replace_block:                     // @f2fs_replace_block
.Lf2fs_replace_block$local:
// %bb.0:
	sub	sp, sp, #48             // =48
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x20, x1
	str	x8, [sp, #8]
	ldp	w8, w9, [x1, #24]
	strb	w4, [sp, #4]
	and	w4, w5, #0x1
	and	w5, w6, #0x1
	mov	x1, sp
	mov	w6, wzr
	add	x29, sp, #16            // =16
	mov	w19, w3
	str	w8, [sp]
	sturh	w9, [sp, #5]
	bl	f2fs_do_replace_block
	mov	x0, x20
	mov	w1, w19
	bl	f2fs_update_data_blkaddr
	adrp	x9, __stack_chk_guard
	ldr	x8, [sp, #8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB71_2
// %bb.1:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #48             // =48
	ret
.LBB71_2:
	bl	__stack_chk_fail
.Lfunc_end71:
	.size	f2fs_replace_block, .Lfunc_end71-f2fs_replace_block
                                        // -- End function
	.globl	f2fs_wait_on_page_writeback // -- Begin function f2fs_wait_on_page_writeback
	.p2align	2
	.type	f2fs_wait_on_page_writeback,@function
f2fs_wait_on_page_writeback:            // @f2fs_wait_on_page_writeback
.Lf2fs_wait_on_page_writeback$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #8]
	mov	x29, sp
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x0, x9, eq
	ldr	x8, [x8]
	tbz	w8, #15, .LBB72_16
// %bb.1:
	ldr	x8, [x0, #8]
	mov	w21, w3
	mov	w22, w2
	mov	x20, x0
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x0, x9, eq
	ldr	x8, [x8]
	mov	w4, w1
	tbz	w8, #19, .LBB72_3
// %bb.2:
	ldr	x8, [x20]
	tbnz	w8, #10, .LBB72_17
.LBB72_3:
	ldr	x0, [x20, #24]
.LBB72_4:
	ldr	x8, [x0]
	mov	x1, xzr
	mov	x2, x20
	mov	w3, wzr
	ldr	x8, [x8, #40]
	ldr	x19, [x8, #1008]
	mov	x0, x19
	bl	f2fs_submit_merged_write_cond
	mov	x0, x19
	mov	x1, xzr
	mov	x2, x20
	bl	f2fs_submit_merged_ipu_write
	tbz	w22, #0, .LBB72_15
// %bb.5:
	mov	x0, x20
	bl	wait_on_page_writeback
	tbz	w21, #0, .LBB72_16
// %bb.6:
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbz	w8, #15, .LBB72_16
// %bb.7:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB72_10
// %bb.8:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB72_10
// %bb.9:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_wait_on_page_writeback
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_wait_on_page_writeback
	mov	w3, #5671
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB72_10:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 5671; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB72_16
// %bb.11:
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
	cbnz	w8, .LBB72_16
// %bb.12:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB72_16
// %bb.13:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB72_16
// %bb.14:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB72_16
.LBB72_15:
	mov	x0, x20
	bl	wait_for_stable_page
.LBB72_16:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB72_17:
	mov	x0, x20
	mov	w19, w4
	bl	__page_file_mapping
	mov	w4, w19
	b	.LBB72_4
.Lfunc_end72:
	.size	f2fs_wait_on_page_writeback, .Lfunc_end72-f2fs_wait_on_page_writeback
                                        // -- End function
	.globl	f2fs_wait_on_block_writeback // -- Begin function f2fs_wait_on_block_writeback
	.p2align	2
	.type	f2fs_wait_on_block_writeback,@function
f2fs_wait_on_block_writeback:           // @f2fs_wait_on_block_writeback
.Lf2fs_wait_on_block_writeback$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	ldrb	w9, [x0, #13]
	str	x19, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	ldr	x8, [x8, #1008]
	tbz	w9, #6, .LBB73_2
// %bb.1:
	ldrh	w9, [x0]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.eq	.LBB73_3
.LBB73_2:
	ldr	x9, [x0, #664]
	cbz	x9, .LBB73_5
.LBB73_3:
	add	w9, w1, #3              // =3
	cmp	w9, #4                  // =4
	b.hs	.LBB73_9
.LBB73_4:
	ldr	x19, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB73_5:
	ldr	w9, [x8, #1532]
	cbz	w9, .LBB73_19
// %bb.6:
	ldr	w9, [x0, #1280]
	cbnz	w9, .LBB73_3
// %bb.7:
	ldr	x9, [x0, #736]
	tbnz	x9, #35, .LBB73_3
// %bb.8:
	ldr	x9, [x0, #736]
	tbnz	x9, #34, .LBB73_3
	b	.LBB73_4
.LBB73_9:
	ldr	w10, [x8, #3128]
	cbz	w10, .LBB73_17
// %bb.10:
	ldr	x9, [x8]
	cmp	w10, #1                 // =1
	ldr	x9, [x9, #248]
	b.lt	.LBB73_16
// %bb.11:
	ldr	x11, [x8, #3136]
	add	x11, x11, #80           // =80
	b	.LBB73_13
.LBB73_12:                              //   in Loop: Header=BB73_13 Depth=1
	subs	x10, x10, #1            // =1
	add	x11, x11, #88           // =88
	b.eq	.LBB73_16
.LBB73_13:                              // =>This Inner Loop Header: Depth=1
	ldur	w12, [x11, #-4]
	cmp	w12, w1
	b.hi	.LBB73_12
// %bb.14:                              //   in Loop: Header=BB73_13 Depth=1
	ldr	w12, [x11]
	cmp	w12, w1
	b.lo	.LBB73_12
// %bb.15:
	ldur	x9, [x11, #-80]
.LBB73_16:
	cbz	x9, .LBB73_4
.LBB73_17:
	ldr	x8, [x8, #248]
	mov	w1, w1
	mov	w2, #2
	mov	w3, wzr
	ldr	x0, [x8, #48]
	bl	pagecache_get_page
	cbz	x0, .LBB73_4
// %bb.18:
	mov	w2, #1
	mov	w3, #1
	mov	w1, wzr
	mov	x19, x0
	bl	f2fs_wait_on_page_writeback
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_put_page
	b	.LBB73_4
.LBB73_19:
	ldrh	w9, [x0]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB73_4
// %bb.20:
	ldr	x9, [x0, #736]
	tbz	w9, #31, .LBB73_4
	b	.LBB73_3
.Lfunc_end73:
	.size	f2fs_wait_on_block_writeback, .Lfunc_end73-f2fs_wait_on_block_writeback
                                        // -- End function
	.globl	f2fs_wait_on_block_writeback_range // -- Begin function f2fs_wait_on_block_writeback_range
	.p2align	2
	.type	f2fs_wait_on_block_writeback_range,@function
f2fs_wait_on_block_writeback_range:     // @f2fs_wait_on_block_writeback_range
.Lf2fs_wait_on_block_writeback_range$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	w2, .LBB74_3
// %bb.1:
	mov	w19, w2
	mov	w20, w1
	mov	x21, x0
.LBB74_2:                               // =>This Inner Loop Header: Depth=1
	mov	x0, x21
	mov	w1, w20
	bl	f2fs_wait_on_block_writeback
	subs	w19, w19, #1            // =1
	add	w20, w20, #1            // =1
	b.ne	.LBB74_2
.LBB74_3:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end74:
	.size	f2fs_wait_on_block_writeback_range, .Lfunc_end74-f2fs_wait_on_block_writeback_range
                                        // -- End function
	.globl	init_virtual_curseg     // -- Begin function init_virtual_curseg
	.p2align	2
	.type	init_virtual_curseg,@function
init_virtual_curseg:                    // @init_virtual_curseg
.Linit_virtual_curseg$local:
// %bb.0:
	ret
.Lfunc_end75:
	.size	init_virtual_curseg, .Lfunc_end75-init_virtual_curseg
                                        // -- End function
	.globl	write_append_journal    // -- Begin function write_append_journal
	.p2align	2
	.type	write_append_journal,@function
write_append_journal:                   // @write_append_journal
.Lwrite_append_journal$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #224]
	mov	x21, #2097152
	mov	x19, x0
	movk	x21, #1, lsl #32
	ldr	w8, [x8, #132]
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	tbz	w8, #11, .LBB76_2
// %bb.1:
	ldr	x8, [x19, #136]
	add	w23, w1, #1             // =1
	mov	w1, w1
	mov	x0, x19
	ldr	x22, [x8, #24]
	bl	f2fs_grab_meta_page
	ldr	x9, [x22, #88]
	add	x8, x0, x21
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	mov	x10, #-549755813888
	mov	x20, x0
	add	x0, x8, x10
	add	x1, x9, #496            // =496
	mov	w2, #4095
	bl	memcpy
	mov	x0, x20
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
	ldr	x8, [x19, #224]
	mov	w1, w23
	ldr	w8, [x8, #132]
.LBB76_2:
	tbz	w8, #10, .LBB76_4
// %bb.3:
	ldr	x8, [x19, #136]
	mov	w1, w1
	mov	x0, x19
	ldr	x20, [x8, #24]
	bl	f2fs_grab_meta_page
	ldr	x9, [x20, #328]
	add	x8, x0, x21
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	mov	x10, #-549755813888
	mov	x19, x0
	add	x0, x8, x10
	add	x1, x9, #470            // =470
	mov	w2, #4056
	bl	memcpy
	mov	x0, x19
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_put_page
.LBB76_4:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end76:
	.size	write_append_journal, .Lfunc_end76-write_append_journal
                                        // -- End function
	.globl	get_max_reclaimable_segments // -- Begin function get_max_reclaimable_segments
	.p2align	2
	.type	get_max_reclaimable_segments,@function
get_max_reclaimable_segments:           // @get_max_reclaimable_segments
.Lget_max_reclaimable_segments$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	x19, x0
	mov	x2, xzr
	str	x23, [sp, #16]          // 8-byte Folded Spill
	ldr	x21, [x8, #16]
	ldr	w20, [x8, #96]
	mov	x29, sp
	ldr	x0, [x21, #56]
	mov	x1, x20
	bl	find_next_bit
	cmp	w20, w0
	mov	x22, xzr
	b.ls	.LBB77_3
// %bb.1:
	mov	x22, xzr
	mov	w23, #40
.LBB77_2:                               // =>This Inner Loop Header: Depth=1
	ldr	x8, [x19, #136]
	and	x9, x0, #0xffffffff
	mul	x9, x9, x23
	add	w2, w0, #1              // =1
	ldr	x8, [x8]
	mov	x1, x20
	ldr	x8, [x8, #120]
	ldr	w9, [x8, x9]
	ldr	x8, [x21, #56]
	ubfx	x9, x9, #6, #10
	mov	x0, x8
	add	x22, x22, x9
	bl	find_next_bit
	cmp	w20, w0
	b.hi	.LBB77_2
.LBB77_3:
	ldr	w8, [x19, #1188]
	ldr	w9, [x21, #136]
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	udiv	x8, x22, x8
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	sub	w0, w9, w8
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end77:
	.size	get_max_reclaimable_segments, .Lfunc_end77-get_max_reclaimable_segments
                                        // -- End function
	.globl	f2fs_write_data_summaries // -- Begin function f2fs_write_data_summaries
	.p2align	2
	.type	f2fs_write_data_summaries,@function
f2fs_write_data_summaries:              // @f2fs_write_data_summaries
.Lf2fs_write_data_summaries$local:
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	ldr	x8, [x0, #224]
	mov	x19, x0
	mov	x29, sp
	ldrb	w8, [x8, #132]
	tbnz	w8, #2, .LBB78_2
// %bb.1:
	mov	x0, x19
	mov	w2, wzr
	bl	write_normal_summaries
	b	.LBB78_16
.LBB78_2:
	mov	x21, #2097152
	add	w24, w1, #1             // =1
	mov	w1, w1
	mov	x0, x19
	movk	x21, #1, lsl #32
	bl	f2fs_grab_meta_page
	add	x8, x0, x21
	lsl	x8, x8, #6
	and	x22, x8, #0xfffffffffffff000
	mov	x23, #-549755813888
	add	x21, x22, x23
	mov	x20, x0
	mov	w2, #4096
	mov	x0, x21
	mov	w1, wzr
	bl	memset
	ldr	x8, [x19, #136]
	mov	w2, #507
	mov	x0, x21
	ldr	x8, [x8, #24]
	ldr	x1, [x8, #88]
	bl	memcpy
	ldr	x8, [x19, #136]
	mov	w2, #507
	ldr	x8, [x8, #24]
	ldr	x1, [x8, #328]
	mov	x8, #-549755813888
	movk	x8, #507
	add	x0, x22, x8
	bl	memcpy
	mov	x25, xzr
	mov	w8, #1014
	b	.LBB78_5
.LBB78_3:                               //   in Loop: Header=BB78_5 Depth=1
	mov	w22, w8
.LBB78_4:                               //   in Loop: Header=BB78_5 Depth=1
	add	x25, x25, #1            // =1
	cmp	x25, #3                 // =3
	mov	w8, w22
	b.eq	.LBB78_14
.LBB78_5:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB78_10 Depth 2
	ldr	x9, [x19, #224]
	ldr	x10, [x19, #136]
	add	x9, x9, x25
	ldrb	w11, [x9, #176]
	ldr	x9, [x10, #24]
	cmp	w11, #1                 // =1
	b.ne	.LBB78_7
// %bb.6:                               //   in Loop: Header=BB78_5 Depth=1
	ldr	w10, [x19, #1188]
	tst	w10, #0xffff
	b.ne	.LBB78_8
	b	.LBB78_3
.LBB78_7:                               //   in Loop: Header=BB78_5 Depth=1
	mov	w10, #120
	madd	x10, x25, x10, x9
	ldrh	w10, [x10, #104]
	tst	w10, #0xffff
	b.eq	.LBB78_3
.LBB78_8:                               //   in Loop: Header=BB78_5 Depth=1
	mov	w11, #120
	madd	x9, x25, x11, x9
	and	x10, x10, #0xffff
	add	x28, x9, #32            // =32
	lsl	x9, x10, #3
	mov	x27, xzr
	sub	x26, x9, x10
	b	.LBB78_10
.LBB78_9:                               //   in Loop: Header=BB78_10 Depth=2
	add	x27, x27, #7            // =7
	cmp	x26, x27
	mov	w8, w22
	b.eq	.LBB78_4
.LBB78_10:                              //   Parent Loop BB78_5 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	cbnz	x20, .LBB78_12
// %bb.11:                              //   in Loop: Header=BB78_10 Depth=2
	mov	w1, w24
	mov	x0, x19
	add	w22, w24, #1            // =1
	bl	f2fs_grab_meta_page
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x8, x0, x8
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	add	x21, x8, x23
	mov	x20, x0
	mov	w2, #4096
	mov	x0, x21
	mov	w1, wzr
	bl	memset
	mov	w8, wzr
	mov	w24, w22
.LBB78_12:                              //   in Loop: Header=BB78_10 Depth=2
	ldr	x9, [x28]
	add	w12, w8, #14            // =14
	add	x10, x21, w8, sxtw
	cmp	w12, #4092              // =4092
	add	x9, x9, x27
	ldur	w11, [x9, #3]
	ldr	w9, [x9]
	add	w22, w8, #7             // =7
	stur	w11, [x10, #3]
	str	w9, [x10]
	b.lo	.LBB78_9
// %bb.13:                              //   in Loop: Header=BB78_10 Depth=2
	mov	x0, x20
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
	mov	x20, xzr
	b	.LBB78_9
.LBB78_14:
	cbz	x20, .LBB78_16
// %bb.15:
	mov	x0, x20
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
.LBB78_16:
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.Lfunc_end78:
	.size	f2fs_write_data_summaries, .Lfunc_end78-f2fs_write_data_summaries
                                        // -- End function
	.p2align	2               // -- Begin function write_normal_summaries
	.type	write_normal_summaries,@function
write_normal_summaries:                 // @write_normal_summaries
// %bb.0:
	mov	w8, #2147483644
	cmp	w2, w8
	b.gt	.LBB79_2
// %bb.1:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	w20, w1
	mov	w1, w1
	mov	x29, sp
	ldr	x23, [x8, #24]
	mov	w19, w2
	mov	x21, x0
	bl	f2fs_grab_meta_page
	mov	x9, #2097152
	mov	w8, #120
	movk	x9, #1, lsl #32
	smaddl	x23, w19, w8, x23
	add	x8, x0, x9
	lsl	x8, x8, #6
	ldr	x24, [x23, #32]
	and	x28, x8, #0xfffffffffffff000
	mov	x8, #-549755813888
	add	x25, x28, x8
	mov	x22, x0
	mov	w2, #4096
	mov	x0, x25
	mov	w1, wzr
	mov	w27, #120
	bl	memset
	mov	x0, x23
	bl	mutex_lock
	add	x26, x23, #40           // =40
	mov	x0, x26
	bl	down_read
	ldr	x1, [x23, #88]
	mov	x8, #-549755813888
	movk	x8, #3584
	add	x0, x28, x8
	mov	w2, #507
	bl	memcpy
	mov	x0, x26
	bl	up_read
	mov	w2, #3584
	mov	x0, x25
	mov	x1, x24
	bl	memcpy
	mov	x8, #-549755813888
	movk	x8, #4091
	add	x0, x28, x8
	add	x1, x24, #4091          // =4091
	mov	w2, #5
	bl	memcpy
	mov	x0, x23
	bl	mutex_unlock
	mov	x0, x22
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
	ldr	x8, [x21, #136]
	add	w1, w20, #1             // =1
	mov	x0, x21
	ldr	x23, [x8, #24]
	bl	f2fs_grab_meta_page
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x8, x0, x8
	smaddl	x28, w19, w27, x23
	lsl	x8, x8, #6
	ldr	x24, [x28, #152]
	and	x27, x8, #0xfffffffffffff000
	mov	x8, #-549755813888
	add	x25, x27, x8
	mov	x22, x0
	mov	w2, #4096
	mov	x0, x25
	mov	w1, wzr
	add	x23, x28, #120          // =120
	bl	memset
	mov	x0, x23
	bl	mutex_lock
	add	x26, x28, #160          // =160
	mov	x0, x26
	bl	down_read
	ldr	x1, [x28, #208]
	mov	x28, #-549755813888
	movk	x28, #3584
	add	x0, x27, x28
	mov	w2, #507
	bl	memcpy
	mov	x0, x26
	bl	up_read
	mov	w2, #3584
	mov	x0, x25
	mov	x1, x24
	bl	memcpy
	mov	x26, #-549755813888
	movk	x26, #4091
	add	x0, x27, x26
	add	x1, x24, #4091          // =4091
	mov	w2, #5
	bl	memcpy
	mov	x0, x23
	bl	mutex_unlock
	mov	x0, x22
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
	ldr	x8, [x21, #136]
	add	w1, w20, #2             // =2
	mov	x0, x21
	ldr	x22, [x8, #24]
	bl	f2fs_grab_meta_page
	mov	w8, #120
	smaddl	x24, w19, w8, x22
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x8, x0, x8
	lsl	x8, x8, #6
	ldr	x21, [x24, #272]
	and	x25, x8, #0xfffffffffffff000
	mov	x8, #-549755813888
	add	x22, x25, x8
	mov	x20, x0
	mov	w2, #4096
	mov	x0, x22
	mov	w1, wzr
	add	x19, x24, #240          // =240
	bl	memset
	mov	x0, x19
	bl	mutex_lock
	add	x23, x24, #280          // =280
	mov	x0, x23
	bl	down_read
	ldr	x1, [x24, #328]
	add	x0, x25, x28
	mov	w2, #507
	bl	memcpy
	mov	x0, x23
	bl	up_read
	mov	w2, #3584
	mov	x0, x22
	mov	x1, x21
	bl	memcpy
	add	x0, x25, x26
	add	x1, x21, #4091          // =4091
	mov	w2, #5
	bl	memcpy
	mov	x0, x19
	bl	mutex_unlock
	mov	x0, x20
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
.LBB79_2:
	ret
.Lfunc_end79:
	.size	write_normal_summaries, .Lfunc_end79-write_normal_summaries
                                        // -- End function
	.globl	f2fs_write_node_summaries // -- Begin function f2fs_write_node_summaries
	.p2align	2
	.type	f2fs_write_node_summaries,@function
f2fs_write_node_summaries:              // @f2fs_write_node_summaries
.Lf2fs_write_node_summaries$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	mov	w2, #3
	mov	x29, sp
	bl	write_normal_summaries
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end80:
	.size	f2fs_write_node_summaries, .Lfunc_end80-f2fs_write_node_summaries
                                        // -- End function
	.globl	f2fs_lookup_journal_in_cursum // -- Begin function f2fs_lookup_journal_in_cursum
	.p2align	2
	.type	f2fs_lookup_journal_in_cursum,@function
f2fs_lookup_journal_in_cursum:          // @f2fs_lookup_journal_in_cursum
.Lf2fs_lookup_journal_in_cursum$local:
// %bb.0:
	mov	x8, x0
	cbz	w1, .LBB81_7
// %bb.1:
	cmp	w1, #1                  // =1
	b.ne	.LBB81_19
// %bb.2:
	ldrh	w0, [x8]
	cbz	w0, .LBB81_13
// %bb.3:
	mov	w9, wzr
	add	x10, x8, #2             // =2
	mov	w11, #78
.LBB81_4:                               // =>This Inner Loop Header: Depth=1
	smull	x12, w9, w11
	ldr	w12, [x10, x12]
	cmp	w12, w2
	b.eq	.LBB81_12
// %bb.5:                               //   in Loop: Header=BB81_4 Depth=1
	add	w9, w9, #1              // =1
	cmp	w9, w0
	b.lo	.LBB81_4
// %bb.6:
	add	w9, w0, #1              // =1
	cbnz	w3, .LBB81_14
	b	.LBB81_19
.LBB81_7:
	ldrh	w0, [x8]
	cbz	w0, .LBB81_15
// %bb.8:
	mov	w9, wzr
	mov	w10, #13
.LBB81_9:                               // =>This Inner Loop Header: Depth=1
	smaddl	x11, w9, w10, x8
	ldur	w11, [x11, #2]
	cmp	w11, w2
	b.eq	.LBB81_12
// %bb.10:                              //   in Loop: Header=BB81_9 Depth=1
	add	w9, w9, #1              // =1
	cmp	w9, w0
	b.lo	.LBB81_9
// %bb.11:
	add	w9, w0, #1              // =1
	cbnz	w3, .LBB81_16
	b	.LBB81_19
.LBB81_12:
	mov	w0, w9
	ret
.LBB81_13:
	mov	w9, #1
	cbz	w3, .LBB81_19
.LBB81_14:
	adrp	x10, write_opt
	ldr	w10, [x10, :lo12:write_opt]
	mov	w11, #58
	cmp	w10, #0                 // =0
	mov	w10, #6
	b	.LBB81_17
.LBB81_15:
	mov	w9, #1
	cbz	w3, .LBB81_19
.LBB81_16:
	adrp	x10, write_opt
	ldr	w10, [x10, :lo12:write_opt]
	mov	w11, #353
	cmp	w10, #0                 // =0
	mov	w10, #38
.LBB81_17:
	csel	w10, w10, w11, eq
	cmp	w10, w0
	b.ls	.LBB81_19
// %bb.18:
	strh	w9, [x8]
	ret
.LBB81_19:
	mov	w0, #-1
	ret
.Lfunc_end81:
	.size	f2fs_lookup_journal_in_cursum, .Lfunc_end81-f2fs_lookup_journal_in_cursum
                                        // -- End function
	.globl	f2fs_flush_sit_entries  // -- Begin function f2fs_flush_sit_entries
	.p2align	2
	.type	f2fs_flush_sit_entries,@function
f2fs_flush_sit_entries:                 // @f2fs_flush_sit_entries
.Lf2fs_flush_sit_entries$local:
// %bb.0:
	sub	sp, sp, #224            // =224
	stp	x29, x30, [sp, #128]    // 16-byte Folded Spill
	stp	x28, x27, [sp, #144]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #160]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #176]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #192]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #208]    // 16-byte Folded Spill
	ldr	x21, [x0, #136]
	add	x29, sp, #128           // =128
	mov	x23, x0
	mov	x24, x1
	ldr	x22, [x21]
	ldr	x20, [x21, #24]
	ldr	x8, [x22, #56]
	stur	x8, [x29, #-40]         // 8-byte Folded Spill
	ldr	x8, [x20, #328]
	stur	x8, [x29, #-56]         // 8-byte Folded Spill
	ldr	x19, [x0, #80]
	add	x0, x22, #72            // =72
	str	x0, [sp]                // 8-byte Folded Spill
	tst	x19, #0x2000
	cset	w8, eq
	stur	w8, [x29, #-4]          // 4-byte Folded Spill
	bl	down_write
	ldr	w8, [x22, #64]
	cbz	w8, .LBB82_70
// %bb.1:
	str	x22, [sp, #24]          // 8-byte Folded Spill
	ldr	x22, [x23, #136]
	mov	x2, xzr
	stur	x24, [x29, #-48]        // 8-byte Folded Spill
	ldr	x8, [x22]
	ldr	w1, [x22, #96]
	ldr	x25, [x8, #56]
	add	x8, x21, #120           // =120
	str	x8, [sp, #32]           // 8-byte Folded Spill
	mov	x0, x25
	bl	find_next_bit
	ldr	x8, [x23, #136]
	mov	x24, x0
	ldr	w9, [x8, #96]
	cmp	w9, w24
	b.ls	.LBB82_5
// %bb.2:
	mov	x21, x23
	add	x23, x22, #120          // =120
.LBB82_3:                               // =>This Inner Loop Header: Depth=1
	mov	w0, w24
	mov	x1, x23
	bl	add_sit_entry
	ldr	x8, [x21, #136]
	add	w2, w24, #1             // =1
	mov	x0, x25
	ldr	w1, [x8, #96]
	bl	find_next_bit
	ldr	x8, [x21, #136]
	mov	x24, x0
	ldr	w9, [x8, #96]
	cmp	w9, w24
	b.hi	.LBB82_3
// %bb.4:
	mov	x23, x21
.LBB82_5:
	adrp	x9, write_opt
	ldr	w9, [x9, :lo12:write_opt]
	ldr	x25, [sp, #24]          // 8-byte Folded Reload
	mov	w10, #58
	stur	x23, [x29, #-24]        // 8-byte Folded Spill
	cmp	w9, #0                  // =0
	mov	w9, #6
	csel	w9, w9, w10, eq
	tbnz	w19, #13, .LBB82_7
// %bb.6:
	ldur	x10, [x29, #-56]        // 8-byte Folded Reload
	ldr	w11, [x25, #64]
	ldrh	w10, [x10]
	sub	w9, w9, w10
	cmp	w9, w11
	b.ge	.LBB82_18
.LBB82_7:
	ldr	x8, [x8, #24]
	ldr	x19, [x8, #328]
	add	x24, x8, #280           // =280
	mov	x0, x24
	bl	down_write
	ldrh	w8, [x19]
	cbz	w8, .LBB82_16
// %bb.8:
	adrp	x23, .L.str.28
	mov	x21, xzr
	add	x22, x19, #2            // =2
	mov	w25, #1
	add	x23, x23, :lo12:.L.str.28
	b	.LBB82_12
.LBB82_9:                               //   in Loop: Header=BB82_12 Depth=1
	ldrb	w9, [x0, #3456]
	cbz	w9, .LBB82_13
// %bb.10:                              //   in Loop: Header=BB82_12 Depth=1
	mov	x1, x23
	bl	f2fs_printk
.LBB82_11:                              //   in Loop: Header=BB82_12 Depth=1
	ldrh	w8, [x19]
	add	x21, x21, #1            // =1
	add	x22, x22, #78           // =78
	cmp	x21, x8
	b.hs	.LBB82_15
.LBB82_12:                              // =>This Inner Loop Header: Depth=1
	ldur	x0, [x29, #-24]         // 8-byte Folded Reload
	ldr	w2, [x22]
	ldr	x8, [x0, #136]
	ldr	w3, [x8, #96]
	cmp	w2, w3
	b.hs	.LBB82_9
.LBB82_13:                              //   in Loop: Header=BB82_12 Depth=1
	ldr	x8, [x8]
	add	w9, w2, #63             // =63
	cmp	w2, #0                  // =0
	csel	w9, w9, w2, lt
	ldr	x10, [x8, #56]
	asr	w11, w9, #6
	sbfiz	x11, x11, #3, #32
	and	w9, w9, #0xffffffc0
	ldr	x12, [x10, x11]
	sub	w9, w2, w9
	lsl	x9, x25, x9
	orr	x13, x12, x9
	tst	x12, x9
	str	x13, [x10, x11]
	b.ne	.LBB82_11
// %bb.14:                              //   in Loop: Header=BB82_12 Depth=1
	ldr	w9, [x8, #64]
	mov	w0, w2
	add	w9, w9, #1              // =1
	str	w9, [x8, #64]
	ldur	x8, [x29, #-24]         // 8-byte Folded Reload
	ldr	x8, [x8, #136]
	add	x1, x8, #120            // =120
	bl	add_sit_entry
	b	.LBB82_11
.LBB82_15:
	ldur	x23, [x29, #-24]        // 8-byte Folded Reload
	ldr	x25, [sp, #24]          // 8-byte Folded Reload
	b	.LBB82_17
.LBB82_16:
	mov	w21, wzr
.LBB82_17:
	sub	w8, w8, w21
	mov	x0, x24
	strh	w8, [x19]
	bl	up_write
.LBB82_18:
	ldr	x8, [sp, #32]           // 8-byte Folded Reload
	ldur	x24, [x29, #-48]        // 8-byte Folded Reload
	ldr	x19, [x8]
	cmp	x19, x8
	b.eq	.LBB82_68
// %bb.19:
	add	x8, x20, #280           // =280
	str	x8, [sp, #16]           // 8-byte Folded Spill
	ldur	x8, [x29, #-56]         // 8-byte Folded Reload
	mov	w21, #40
	add	x20, x8, #2             // =2
	add	x8, x23, #3408          // =3408
	str	x8, [sp, #8]            // 8-byte Folded Spill
	b	.LBB82_21
.LBB82_20:                              //   in Loop: Header=BB82_21 Depth=1
	mov	x9, #256
	movk	x9, #57005, lsl #48
	add	x8, x9, #34             // =34
	stp	x9, x8, [x19]
	adrp	x8, sit_entry_set_slab
	ldr	x0, [x8, :lo12:sit_entry_set_slab]
	mov	x1, x19
	bl	kmem_cache_free
	ldr	x8, [sp, #32]           // 8-byte Folded Reload
	ldr	x19, [sp, #48]          // 8-byte Folded Reload
	cmp	x19, x8
	b.eq	.LBB82_68
.LBB82_21:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB82_25 Depth 2
                                        //     Child Loop BB82_33 Depth 2
                                        //       Child Loop BB82_38 Depth 3
	ldr	x8, [x23, #136]
	ldr	w27, [x19, #16]
	ldr	x10, [x19]
	stur	x19, [x29, #-32]        // 8-byte Folded Spill
	ldr	w9, [x8, #96]
	add	x22, x27, #55           // =55
	str	x10, [sp, #48]          // 8-byte Folded Spill
	cmp	x22, x9
	csel	x9, x22, x9, lo
	stur	x9, [x29, #-16]         // 8-byte Folded Spill
	ldur	w9, [x29, #-4]          // 4-byte Folded Reload
	tbz	w9, #0, .LBB82_23
// %bb.22:                              //   in Loop: Header=BB82_21 Depth=1
	ldur	x10, [x29, #-56]        // 8-byte Folded Reload
	adrp	x9, write_opt
	ldr	w9, [x9, :lo12:write_opt]
	ldr	w11, [x19, #20]
	ldrh	w10, [x10]
	mov	w12, #6
	cmp	w9, #0                  // =0
	mov	w9, #58
	csel	w9, w12, w9, eq
	sub	w9, w9, w10
	cmp	w9, w11
	b.ge	.LBB82_27
.LBB82_23:                              //   in Loop: Header=BB82_21 Depth=1
	ldr	x8, [x8]
	mov	x0, x23
	mov	w1, w27
	str	x8, [sp, #40]           // 8-byte Folded Spill
	bl	current_sit_addr
	ldr	x8, [x23, #136]
	ldr	x8, [x8]
	ldp	w9, w8, [x8, #8]
	sub	x9, x0, x9
	cmp	x9, x8
	cneg	x8, x8, hs
	add	x1, x8, x0
	mov	x0, x23
	bl	f2fs_grab_meta_page
	ldr	x8, [x23, #136]
	mov	x9, #134217728
	movk	x9, #64, lsl #32
	add	x9, x9, x0, lsl #6
	ldr	w8, [x8, #96]
	and	x9, x9, #0xfffffffffffff000
	str	x0, [sp, #56]           // 8-byte Folded Spill
	mov	w2, #4096
	cmp	x22, x8
	csel	w22, w22, w8, lo
	mov	x8, #-549755813888
	add	x28, x9, x8
	mov	x0, x28
	mov	w1, wzr
	bl	memset
	subs	w8, w22, w27
	mov	w11, #74
	b.eq	.LBB82_26
// %bb.24:                              //   in Loop: Header=BB82_21 Depth=1
	mov	x22, xzr
	mov	x24, xzr
	mov	w25, w8
.LBB82_25:                              //   Parent Loop BB82_21 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	x8, [x23, #136]
	add	w9, w27, w24
	mov	w2, #64
	ldr	x8, [x8]
	ldr	x8, [x8, #120]
	umaddl	x26, w9, w21, x8
	ldr	w8, [x26]
	asr	x9, x22, #32
	madd	x19, x9, x11, x28
	add	x0, x19, #2             // =2
	lsl	w10, w8, #10
	bfxil	w10, w8, #6, #10
	strh	w10, [x19]
	ldr	x1, [x26, #8]
	bl	memcpy
	ldr	x8, [x26, #32]
	mov	x9, #4294967296
	add	x24, x24, #1            // =1
	mov	w11, #74
	cmp	x24, x25
	add	x22, x22, x9
	stur	x8, [x19, #66]
	b.lo	.LBB82_25
.LBB82_26:                              //   in Loop: Header=BB82_21 Depth=1
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	str	x28, [sp, #64]          // 8-byte Folded Spill
	bl	set_page_dirty
	mov	w8, #4767
	ldr	x10, [sp, #40]          // 8-byte Folded Reload
	movk	w8, #10724, lsl #16
	mul	x8, x27, x8
	lsr	x8, x8, #32
	sub	w9, w27, w8
	ldr	x10, [x10, #32]
	add	w8, w8, w9, lsr #1
	lsr	w8, w8, #5
	lsr	x9, x8, #3
	ldrb	w11, [x10, x9]
	mvn	w8, w8
	ldur	x24, [x29, #-48]        // 8-byte Folded Reload
	ldr	x25, [sp, #24]          // 8-byte Folded Reload
	ldur	x19, [x29, #-32]        // 8-byte Folded Reload
	and	w8, w8, #0x7
	mov	w12, #1
	lsl	w8, w12, w8
	eor	w8, w11, w8
	stur	wzr, [x29, #-4]         // 4-byte Folded Spill
	strb	w8, [x10, x9]
	b	.LBB82_28
.LBB82_27:                              //   in Loop: Header=BB82_21 Depth=1
	ldr	x0, [sp, #16]           // 8-byte Folded Reload
	bl	down_write
	mov	w8, #1
	stp	xzr, xzr, [sp, #56]     // 16-byte Folded Spill
	stur	w8, [x29, #-4]          // 4-byte Folded Spill
.LBB82_28:                              //   in Loop: Header=BB82_21 Depth=1
	ldur	x22, [x29, #-16]        // 8-byte Folded Reload
	ldur	x0, [x29, #-40]         // 8-byte Folded Reload
	mov	x2, x27
	mov	x1, x22
	bl	find_next_bit
	mov	x27, x0
	cmp	w27, w22
	b.lo	.LBB82_33
.LBB82_29:                              //   in Loop: Header=BB82_21 Depth=1
	ldur	w8, [x29, #-4]          // 4-byte Folded Reload
	cbz	w8, .LBB82_56
// %bb.30:                              //   in Loop: Header=BB82_21 Depth=1
	ldr	x0, [sp, #16]           // 8-byte Folded Reload
	bl	up_write
	b	.LBB82_57
.LBB82_31:                              //   in Loop: Header=BB82_33 Depth=2
	ldr	w8, [x25, #68]
	ldr	w9, [x22]
	mov	w28, #40
	mov	x21, x25
	udiv	w10, w27, w8
	msub	w8, w10, w8, w27
	ldr	x10, [sp, #64]          // 8-byte Folded Reload
	lsl	w11, w9, #10
	bfxil	w11, w9, #6, #10
	mov	w9, #74
	smaddl	x25, w8, w9, x10
	strh	w11, [x25]
.LBB82_32:                              //   in Loop: Header=BB82_33 Depth=2
	madd	x23, x19, x28, x26
	ldr	x1, [x23, #8]
	add	x24, x25, #2            // =2
	mov	w2, #64
	mov	x0, x24
	bl	memcpy
	ldr	x8, [x23, #32]
	mov	w2, #64
	mov	x1, x24
	stur	x8, [x25, #66]
	madd	x8, x19, x28, x26
	ldr	x0, [x8, #16]
	bl	memcpy
	ldr	w8, [x22]
	ldur	x23, [x29, #-24]        // 8-byte Folded Reload
	mov	w1, w27
	mov	x2, x25
	lsr	w9, w8, #6
	bfi	w8, w9, #16, #10
	mov	x0, x23
	str	w8, [x22]
	bl	check_block_count
	add	w8, w27, #63            // =63
	cmp	w27, #0                 // =0
	ldp	x0, x19, [x29, #-40]    // 16-byte Folded Reload
	csel	w8, w8, w27, lt
	asr	w9, w8, #6
	sbfiz	x9, x9, #3, #32
	ldr	x10, [x0, x9]
	and	w8, w8, #0xffffffc0
	sub	w8, w27, w8
	mov	w11, #1
	lsl	x8, x11, x8
	bic	x8, x10, x8
	str	x8, [x0, x9]
	ldr	w8, [x21, #64]
	ldur	x22, [x29, #-16]        // 8-byte Folded Reload
	add	w2, w27, #1             // =1
	sub	w8, w8, #1              // =1
	str	w8, [x21, #64]
	ldr	w8, [x19, #20]
	mov	x1, x22
	sub	w8, w8, #1              // =1
	str	w8, [x19, #20]
	bl	find_next_bit
	ldur	x24, [x29, #-48]        // 8-byte Folded Reload
	mov	x27, x0
	cmp	w27, w22
	mov	x25, x21
	mov	w21, #40
	b.hs	.LBB82_29
.LBB82_33:                              //   Parent Loop BB82_21 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB82_38 Depth 3
	ldr	x8, [x23, #136]
	and	x19, x27, #0xffffffff
	ldr	x8, [x8]
	ldr	x26, [x8, #120]
	ldrb	w8, [x24]
	madd	x22, x19, x21, x26
	tbnz	w8, #4, .LBB82_35
// %bb.34:                              //   in Loop: Header=BB82_33 Depth=2
	mov	x0, x23
	mov	x1, x24
	mov	w2, wzr
	str	x19, [x24, #8]
	bl	add_discard_addrs
.LBB82_35:                              //   in Loop: Header=BB82_33 Depth=2
	ldur	w8, [x29, #-4]          // 4-byte Folded Reload
	cbz	w8, .LBB82_31
// %bb.36:                              //   in Loop: Header=BB82_33 Depth=2
	ldur	x8, [x29, #-56]         // 8-byte Folded Reload
	ldrh	w8, [x8]
	cbz	w8, .LBB82_42
// %bb.37:                              //   in Loop: Header=BB82_33 Depth=2
	mov	w24, wzr
	mov	w23, #78
.LBB82_38:                              //   Parent Loop BB82_21 Depth=1
                                        //     Parent Loop BB82_33 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	smull	x9, w24, w23
	ldr	w9, [x20, x9]
	cmp	w9, w27
	b.eq	.LBB82_44
// %bb.39:                              //   in Loop: Header=BB82_38 Depth=3
	add	w24, w24, #1            // =1
	cmp	w8, w24
	b.ne	.LBB82_38
// %bb.40:                              //   in Loop: Header=BB82_33 Depth=2
	adrp	x9, write_opt
	ldr	w9, [x9, :lo12:write_opt]
	mov	w10, #6
	cmp	w9, #0                  // =0
	mov	w9, #58
	csel	w9, w10, w9, eq
	cmp	w9, w8
	b.ls	.LBB82_45
// %bb.41:                              //   in Loop: Header=BB82_33 Depth=2
	add	w9, w8, #1              // =1
	b	.LBB82_43
.LBB82_42:                              //   in Loop: Header=BB82_33 Depth=2
	mov	w9, #1
	mov	w23, #78
.LBB82_43:                              //   in Loop: Header=BB82_33 Depth=2
	ldur	x10, [x29, #-56]        // 8-byte Folded Reload
	mov	w24, w8
	strh	w9, [x10]
	b	.LBB82_52
.LBB82_44:                              //   in Loop: Header=BB82_33 Depth=2
	tbz	w24, #31, .LBB82_52
	b	.LBB82_46
.LBB82_45:                              //   in Loop: Header=BB82_33 Depth=2
	mov	w24, #-1
.LBB82_46:                              //   in Loop: Header=BB82_33 Depth=2
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB82_49
// %bb.47:                              //   in Loop: Header=BB82_33 Depth=2
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB82_49
// %bb.48:                              //   in Loop: Header=BB82_33 Depth=2
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_flush_sit_entries
	mov	w3, #6381
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_flush_sit_entries
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB82_49:                              //   in Loop: Header=BB82_33 Depth=2
	ldur	x23, [x29, #-24]        // 8-byte Folded Reload
	mov	x0, x23
	bl	f2fs_print_raw_sb_info
	mov	x0, x23
	bl	f2fs_print_ckpt_info
	mov	x0, x23
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 6381; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x23, .LBB82_51
// %bb.50:                              //   in Loop: Header=BB82_33 Depth=2
	add	x8, x23, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x23, #3376]
	cbz	w8, .LBB82_53
.LBB82_51:                              //   in Loop: Header=BB82_33 Depth=2
	mov	w23, #78
.LBB82_52:                              //   in Loop: Header=BB82_33 Depth=2
	mov	x21, x25
	smaddl	x25, w24, w23, x20
	str	w27, [x25]
	ldr	w8, [x22]
	mov	w28, #40
	lsl	w9, w8, #10
	bfxil	w9, w8, #6, #10
	strh	w9, [x25, #4]!
	b	.LBB82_32
.LBB82_53:                              //   in Loop: Header=BB82_33 Depth=2
	ldr	x1, [x23, #3392]
	cbz	x1, .LBB82_51
// %bb.54:                              //   in Loop: Header=BB82_33 Depth=2
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	mov	w23, #78
	cmp	w8, #2                  // =2
	b.hi	.LBB82_52
// %bb.55:                              //   in Loop: Header=BB82_33 Depth=2
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB82_52
.LBB82_56:                              //   in Loop: Header=BB82_21 Depth=1
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	mov	w1, #1
	bl	f2fs_put_page
.LBB82_57:                              //   in Loop: Header=BB82_21 Depth=1
	ldr	w8, [x19, #20]
	cbnz	w8, .LBB82_60
.LBB82_58:                              //   in Loop: Header=BB82_21 Depth=1
	mov	x0, x19
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB82_20
// %bb.59:                              //   in Loop: Header=BB82_21 Depth=1
	ldp	x9, x8, [x19]
	str	x8, [x9, #8]
	str	x9, [x8]
	b	.LBB82_20
.LBB82_60:                              //   in Loop: Header=BB82_21 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB82_63
// %bb.61:                              //   in Loop: Header=BB82_21 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB82_63
// %bb.62:                              //   in Loop: Header=BB82_21 Depth=1
	adrp	x22, f2fs_dclient
	ldr	x0, [x22, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_flush_sit_entries
	mov	w3, #6406
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_flush_sit_entries
	bl	dsm_client_record
	ldr	x0, [x22, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB82_63:                              //   in Loop: Header=BB82_21 Depth=1
	mov	x0, x23
	bl	f2fs_print_raw_sb_info
	mov	x0, x23
	bl	f2fs_print_ckpt_info
	mov	x0, x23
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 6406; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x23, .LBB82_58
// %bb.64:                              //   in Loop: Header=BB82_21 Depth=1
	add	x8, x23, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x23, #3376]
	cbnz	w8, .LBB82_58
// %bb.65:                              //   in Loop: Header=BB82_21 Depth=1
	ldr	x1, [x23, #3392]
	cbz	x1, .LBB82_58
// %bb.66:                              //   in Loop: Header=BB82_21 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB82_58
// %bb.67:                              //   in Loop: Header=BB82_21 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB82_58
.LBB82_68:
	ldr	x9, [sp, #32]           // 8-byte Folded Reload
	ldr	x8, [x9]
	cmp	x9, x8
	b.ne	.LBB82_78
.LBB82_69:
	ldr	w8, [x25, #64]
	cbnz	w8, .LBB82_86
.LBB82_70:
	ldrb	w8, [x24]
	tbz	w8, #4, .LBB82_74
// %bb.71:
	ldp	x19, x8, [x24, #8]
	cmp	x19, x8
	b.hi	.LBB82_73
.LBB82_72:                              // =>This Inner Loop Header: Depth=1
	mov	x0, x23
	mov	x1, x24
	mov	w2, wzr
	bl	add_discard_addrs
	ldp	x8, x9, [x24, #8]
	add	x8, x8, #1              // =1
	cmp	x8, x9
	str	x8, [x24, #8]
	b.ls	.LBB82_72
.LBB82_73:
	str	x19, [x24, #8]
.LBB82_74:
	ldr	x0, [sp]                // 8-byte Folded Reload
	bl	up_write
	ldr	x8, [x23, #136]
	ldr	x19, [x8, #16]
	add	x20, x19, #80           // =80
	mov	x0, x20
	bl	mutex_lock
	ldr	x8, [x23, #136]
	ldr	x0, [x19, #64]
	mov	x2, xzr
	ldr	w1, [x8, #96]
.LBB82_75:                              // =>This Inner Loop Header: Depth=1
	bl	find_next_bit
	ldr	x8, [x23, #136]
	mov	x21, x0
	ldr	w8, [x8, #96]
	cmp	w8, w21
	b.ls	.LBB82_77
// %bb.76:                              //   in Loop: Header=BB82_75 Depth=1
	mov	x0, x23
	mov	w1, w21
	mov	w2, wzr
	bl	__set_test_and_free
	ldr	x8, [x23, #136]
	ldr	x0, [x19, #64]
	add	w2, w21, #1             // =1
	ldr	w1, [x8, #96]
	b	.LBB82_75
.LBB82_77:
	mov	x0, x20
	bl	mutex_unlock
	ldp	x20, x19, [sp, #208]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #192]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #176]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #160]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #144]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #128]    // 16-byte Folded Reload
	add	sp, sp, #224            // =224
	ret
.LBB82_78:
	bl	print_bdev_access_info
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	cbz	x0, .LBB82_81
// %bb.79:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB82_81
// %bb.80:
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_flush_sit_entries
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_flush_sit_entries
	mov	w3, #6410
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB82_81:
	mov	x0, x23
	bl	f2fs_print_raw_sb_info
	mov	x0, x23
	bl	f2fs_print_ckpt_info
	mov	x0, x23
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 6410; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x23, .LBB82_69
// %bb.82:
	add	x8, x23, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x23, #3376]
	cbnz	w8, .LBB82_69
// %bb.83:
	ldr	x1, [x23, #3392]
	cbz	x1, .LBB82_69
// %bb.84:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB82_69
// %bb.85:
	add	x2, x23, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB82_69
.LBB82_86:
	bl	print_bdev_access_info
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	cbz	x0, .LBB82_89
// %bb.87:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB82_89
// %bb.88:
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.f2fs_flush_sit_entries
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.f2fs_flush_sit_entries
	mov	w3, #6411
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB82_89:
	mov	x0, x23
	bl	f2fs_print_raw_sb_info
	mov	x0, x23
	bl	f2fs_print_ckpt_info
	mov	x0, x23
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 6411; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x23, .LBB82_70
// %bb.90:
	add	x8, x23, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x23, #3376]
	cbnz	w8, .LBB82_70
// %bb.91:
	ldr	x1, [x23, #3392]
	cbz	x1, .LBB82_70
// %bb.92:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB82_70
// %bb.93:
	add	x2, x23, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB82_70
.Lfunc_end82:
	.size	f2fs_flush_sit_entries, .Lfunc_end82-f2fs_flush_sit_entries
                                        // -- End function
	.p2align	2               // -- Begin function check_block_count
	.type	check_block_count,@function
check_block_count:                      // @check_block_count
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	str	x25, [sp, #16]          // 8-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	mov	x23, x2
	mov	x19, x0
	ldr	x9, [x23, #2]!
	ldr	w8, [x19, #1188]
	mov	x22, x2
	mov	w21, w1
	mov	w20, wzr
	mov	w0, wzr
	and	w25, w9, #0x1
	mov	x29, sp
	b	.LBB83_3
.LBB83_1:                               //   in Loop: Header=BB83_3 Depth=1
	mov	x2, x24
	bl	find_next_bit
.LBB83_2:                               //   in Loop: Header=BB83_3 Depth=1
	ldr	w8, [x19, #1188]
	eor	w25, w25, #0x1
	cmp	w0, w8
	b.hs	.LBB83_5
.LBB83_3:                               // =>This Inner Loop Header: Depth=1
	mov	w1, w8
	sxtw	x24, w0
	mov	x0, x23
	tbz	w25, #0, .LBB83_1
// %bb.4:                               //   in Loop: Header=BB83_3 Depth=1
	mov	x2, x24
	bl	find_next_zero_bit
	sub	w8, w20, w24
	add	w20, w8, w0
	b	.LBB83_2
.LBB83_5:
	ldrh	w9, [x22]
	and	w2, w9, #0x3ff
	cmp	w20, w2
	b.ne	.LBB83_21
// %bb.6:
	cmp	w8, w20
	b.lo	.LBB83_13
// %bb.7:
	ldr	x8, [x19, #136]
	cbz	x8, .LBB83_9
// %bb.8:
	add	x8, x8, #92             // =92
	b	.LBB83_10
.LBB83_9:
	ldr	x8, [x19, #16]
	add	x8, x8, #48             // =48
.LBB83_10:
	ldr	w8, [x8]
	sub	w8, w8, #1              // =1
	cmp	w8, w21
	b.lo	.LBB83_13
// %bb.11:
	mov	w0, wzr
.LBB83_12:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB83_13:
	adrp	x1, .L.str.31
	add	x1, x1, :lo12:.L.str.31
	mov	x0, x19
	mov	w2, w20
	mov	w3, w21
	bl	f2fs_printk
	cbz	x19, .LBB83_18
// %bb.14:
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
	cbnz	w8, .LBB83_18
// %bb.15:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB83_18
// %bb.16:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB83_18
// %bb.17:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB83_18:
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	cbz	x0, .LBB83_30
// %bb.19:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB83_30
// %bb.20:
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.30
	adrp	x2, .L__func__.check_block_count
	add	x1, x1, :lo12:.L.str.30
	add	x2, x2, :lo12:.L__func__.check_block_count
	mov	w3, #1292
	b	.LBB83_29
.LBB83_21:
	adrp	x1, .L.str.29
	add	x1, x1, :lo12:.L.str.29
	mov	x0, x19
	mov	w3, w20
	bl	f2fs_printk
	cbz	x19, .LBB83_26
// %bb.22:
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
	cbnz	w8, .LBB83_26
// %bb.23:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB83_26
// %bb.24:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB83_26
// %bb.25:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB83_26:
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	cbz	x0, .LBB83_30
// %bb.27:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB83_30
// %bb.28:
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.30
	adrp	x2, .L__func__.check_block_count
	add	x1, x1, :lo12:.L.str.30
	add	x2, x2, :lo12:.L__func__.check_block_count
	mov	w3, #1282
.LBB83_29:
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w1, #15244
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB83_30:
	mov	w0, #-117
	b	.LBB83_12
.Lfunc_end83:
	.size	check_block_count, .Lfunc_end83-check_block_count
                                        // -- End function
	.globl	f2fs_fix_curseg_write_pointer // -- Begin function f2fs_fix_curseg_write_pointer
	.p2align	2
	.type	f2fs_fix_curseg_write_pointer,@function
f2fs_fix_curseg_write_pointer:          // @f2fs_fix_curseg_write_pointer
.Lf2fs_fix_curseg_write_pointer$local:
// %bb.0:
	mov	w0, wzr
	ret
.Lfunc_end84:
	.size	f2fs_fix_curseg_write_pointer, .Lfunc_end84-f2fs_fix_curseg_write_pointer
                                        // -- End function
	.globl	f2fs_check_write_pointer // -- Begin function f2fs_check_write_pointer
	.p2align	2
	.type	f2fs_check_write_pointer,@function
f2fs_check_write_pointer:               // @f2fs_check_write_pointer
.Lf2fs_check_write_pointer$local:
// %bb.0:
	mov	w0, wzr
	ret
.Lfunc_end85:
	.size	f2fs_check_write_pointer, .Lfunc_end85-f2fs_check_write_pointer
                                        // -- End function
	.globl	f2fs_usable_blks_in_seg // -- Begin function f2fs_usable_blks_in_seg
	.p2align	2
	.type	f2fs_usable_blks_in_seg,@function
f2fs_usable_blks_in_seg:                // @f2fs_usable_blks_in_seg
.Lf2fs_usable_blks_in_seg$local:
// %bb.0:
	ldr	x8, [x0, #16]
	ldrb	w8, [x8, #2180]
	tbnz	w8, #1, .LBB86_2
// %bb.1:
	ldr	w0, [x0, #1188]
	ret
.LBB86_2:
	mov	w0, wzr
	ret
.Lfunc_end86:
	.size	f2fs_usable_blks_in_seg, .Lfunc_end86-f2fs_usable_blks_in_seg
                                        // -- End function
	.globl	f2fs_usable_segs_in_sec // -- Begin function f2fs_usable_segs_in_sec
	.p2align	2
	.type	f2fs_usable_segs_in_sec,@function
f2fs_usable_segs_in_sec:                // @f2fs_usable_segs_in_sec
.Lf2fs_usable_segs_in_sec$local:
// %bb.0:
	ldr	x8, [x0, #16]
	ldrb	w8, [x8, #2180]
	tbnz	w8, #1, .LBB87_2
// %bb.1:
	ldr	w0, [x0, #1192]
	ret
.LBB87_2:
	mov	w0, wzr
	ret
.Lfunc_end87:
	.size	f2fs_usable_segs_in_sec, .Lfunc_end87-f2fs_usable_segs_in_sec
                                        // -- End function
	.globl	f2fs_build_segment_manager // -- Begin function f2fs_build_segment_manager
	.p2align	2
	.type	f2fs_build_segment_manager,@function
f2fs_build_segment_manager:             // @f2fs_build_segment_manager
.Lf2fs_build_segment_manager$local:
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
	adrp	x28, kmalloc_caches+64
	stur	x8, [x29, #-8]
	mov	x19, x0
	ldr	x20, [x0, #16]
	ldr	x21, [x0, #224]
	ldr	x0, [x28, :lo12:kmalloc_caches+64]
	mov	w1, #3520
	mov	w2, #176
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB88_232
// %bb.1:
	mov	x26, x19
	str	x0, [x26, #136]!
	ldr	w8, [x20, #72]
	mov	w10, #16483
	movk	w10, #6, lsl #16
	str	w8, [x0, #80]
	ldr	w8, [x20, #92]
	str	w8, [x0, #84]
	ldr	w8, [x20, #48]
	str	w8, [x0, #92]
	ldr	w8, [x21, #24]
	str	w8, [x0, #100]
	ldr	w9, [x21, #28]
	str	w9, [x0, #104]
	ldr	w9, [x20, #68]
	str	w9, [x0, #96]
	add	w9, w9, w9, lsl #2
	cmp	w9, w10
	mov	w10, #34079
	movk	w10, #20971, lsl #16
	ldr	w11, [x20, #88]
	umull	x9, w9, w10
	mov	w10, #4096
	lsr	x9, x9, #37
	csel	w9, w10, w9, hi
	str	w11, [x0, #88]
	str	w9, [x0, #108]
	ldr	w9, [x26, #1368]
	cmp	w9, #1                  // =1
	b.eq	.LBB88_3
// %bb.2:
	mov	w9, #16
	str	w9, [x0, #136]
.LBB88_3:
	mov	x9, #70
	movk	x9, #8, lsl #32
	stur	x9, [x0, #140]
	ldr	w10, [x19, #1188]
	ldr	w9, [x19, #1192]
	mov	w11, #16
	cmn	w8, #1                  // =1
	mul	w10, w9, w10
	stp	w10, w11, [x0, #148]
	b.eq	.LBB88_5
// %bb.4:
	udiv	w8, w8, w9
.LBB88_5:
	str	w8, [x0, #156]
	add	x8, x0, #120            // =120
	adrp	x1, .L.str.9
	adrp	x2, f2fs_build_segment_manager.__key
	str	x8, [x0, #120]
	str	x8, [x0, #128]
	add	x0, x0, #32             // =32
	add	x1, x1, :lo12:.L.str.9
	add	x2, x2, :lo12:f2fs_build_segment_manager.__key
	bl	__init_rwsem
	ldr	x8, [x19]
	ldrb	w9, [x8, #80]
	tbnz	w9, #0, .LBB88_8
// %bb.6:
	mov	x0, x19
	bl	f2fs_create_flush_cmd_control
	mov	w22, w0
	cbnz	w0, .LBB88_233
// %bb.7:
	ldr	x8, [x19]
.LBB88_8:
	ldr	x9, [x26]
	ldr	x8, [x8, #248]
	ldr	x20, [x9, #168]
	ldr	w21, [x8]
	cbnz	x20, .LBB88_13
// %bb.9:
	mov	w0, #8368
	mov	w1, #3520
	mov	w2, #2
	bl	kmalloc_order_trace
	cbz	x0, .LBB88_232
// %bb.10:
	mov	x20, x0
	mov	x8, xzr
	mov	w9, #512
	add	x10, x0, #8             // =8
	str	w9, [x0, #8320]
	str	x10, [x0, #8]
	str	x10, [x0, #16]
.LBB88_11:                              // =>This Inner Loop Header: Depth=1
	add	x9, x20, x8
	add	x8, x8, #16             // =16
	add	x10, x9, #24            // =24
	cmp	x8, #2, lsl #12         // =8192
	str	x10, [x9, #24]
	str	x10, [x9, #32]
	b.ne	.LBB88_11
// %bb.12:
	mov	w8, #8216
	mov	w9, #8232
	mov	w10, #8280
	adrp	x1, .L.str.32
	adrp	x2, create_discard_cmd_control.__key
	add	x8, x20, x8
	add	x9, x20, x9
	add	x0, x20, x10
	add	x1, x1, :lo12:.L.str.32
	add	x2, x2, :lo12:create_discard_cmd_control.__key
	str	x8, [x20, #8216]
	str	x8, [x20, #8224]
	str	x9, [x20, #8232]
	str	x9, [x20, #8240]
	bl	__mutex_init
	str	wzr, [x20, #8332]
	str	wzr, [x20, #8336]
	str	wzr, [x20, #8340]
	str	wzr, [x20, #8312]
	ldr	x8, [x19, #136]
	ldr	w9, [x19, #1184]
	adrp	x1, .L.str.34
	adrp	x2, create_discard_cmd_control.__key.33
	ldr	w8, [x8, #96]
	add	x1, x1, :lo12:.L.str.34
	add	x2, x2, :lo12:create_discard_cmd_control.__key.33
	lsl	w8, w8, w9
	mov	w9, #8324
	str	w8, [x20, #8316]
	str	xzr, [x20, x9]
	mov	w9, #8248
	mov	w8, #8360
	add	x0, x20, x9
	str	xzr, [x20, #8344]
	str	xzr, [x20, #8352]
	strb	wzr, [x20, x8]
	bl	__init_waitqueue_head
	ldr	x8, [x19, #136]
	str	x20, [x8, #168]
.LBB88_13:
	adrp	x0, issue_discard_thread
	adrp	x3, .L.str.35
	lsr	w4, w21, #20
	and	w5, w21, #0xfffff
	add	x0, x0, :lo12:issue_discard_thread
	add	x3, x3, :lo12:.L.str.35
	mov	w2, #-1
	mov	x1, x19
	bl	kthread_create_on_node
	mov	x22, x0
	cmn	x0, #1, lsl #12         // =4096
	b.ls	.LBB88_15
// %bb.14:
	mov	x0, x20
	str	x22, [x20]
	bl	kfree
	ldr	x8, [x26]
	str	xzr, [x8, #168]
	cbnz	w22, .LBB88_233
	b	.LBB88_16
.LBB88_15:
	mov	x0, x22
	bl	wake_up_process
	str	x22, [x20]
.LBB88_16:
	ldr	x0, [x28, :lo12:kmalloc_caches+64]
	ldr	x22, [x19, #16]
	mov	w1, #3520
	mov	w2, #208
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB88_232
// %bb.17:
	ldr	x8, [x26]
	mov	x20, x0
	mov	w1, #3520
	mov	w2, #-1
	str	x0, [x8]
	ldr	x8, [x26]
	ldr	w8, [x8, #96]
	add	x8, x8, x8, lsl #2
	lsl	x0, x8, #3
	bl	kvmalloc_node
	str	x0, [x20, #120]
	cbz	x0, .LBB88_232
// %bb.18:
	ldr	x8, [x26]
	mov	w1, #3520
	mov	w2, #-1
	ldr	w8, [x8, #96]
	add	x8, x8, #63             // =63
	lsr	x8, x8, #3
	and	x0, x8, #0x3ffffff8
	bl	kvmalloc_node
	str	x0, [x20, #56]
	cbz	x0, .LBB88_232
// %bb.19:
	ldr	x8, [x26]
	mov	w1, #3520
	mov	w2, #-1
	ldr	w8, [x8, #96]
	add	w8, w8, w8, lsl #1
	lsl	w0, w8, #6
	bl	kvmalloc_node
	str	x0, [x20, #24]
	cbz	x0, .LBB88_232
// %bb.20:
	ldr	x8, [x26]
	ldr	w8, [x8, #96]
	cbz	w8, .LBB88_23
// %bb.21:
	mov	x8, xzr
	mov	x9, xzr
.LBB88_22:                              // =>This Inner Loop Header: Depth=1
	ldr	x10, [x20, #120]
	add	x11, x0, #64            // =64
	add	x9, x9, #1              // =1
	add	x10, x10, x8
	str	x0, [x10, #8]
	ldr	x10, [x20, #120]
	add	x10, x10, x8
	str	x11, [x10, #16]
	ldr	x10, [x20, #120]
	add	x11, x0, #128           // =128
	add	x0, x0, #192            // =192
	add	x10, x10, x8
	str	x11, [x10, #24]
	ldr	x10, [x26]
	add	x8, x8, #40             // =40
	ldr	w10, [x10, #96]
	cmp	x9, x10
	b.lo	.LBB88_22
.LBB88_23:
	adrp	x23, kmalloc_caches+56
	ldr	x0, [x23, :lo12:kmalloc_caches+56]
	mov	w1, #3520
	mov	w2, #64
	bl	kmem_cache_alloc_trace
	str	x0, [x20, #48]
	cbz	x0, .LBB88_232
// %bb.24:
	ldr	w8, [x19, #1192]
	cmp	w8, #2                  // =2
	b.lo	.LBB88_26
// %bb.25:
	ldr	w8, [x19, #1200]
	mov	w1, #3520
	mov	w2, #-1
	lsl	x0, x8, #2
	bl	kvmalloc_node
	str	x0, [x20, #128]
	cbz	x0, .LBB88_232
.LBB88_26:
	ldr	x8, [x19, #224]
	ldr	w24, [x22, #56]
	ldrb	w9, [x8, #133]
	ldr	w21, [x8, #156]
	tbnz	w9, #5, .LBB88_29
// %bb.27:
	ldr	x9, [x19, #16]
	ldr	w9, [x9, #1664]
	cbz	w9, .LBB88_30
// %bb.28:
	add	x0, x8, #1, lsl #12     // =4096
	b	.LBB88_31
.LBB88_29:
	ldrsw	x9, [x8, #160]
	add	x8, x8, x9
	add	x0, x8, #196            // =196
	b	.LBB88_31
.LBB88_30:
	add	x0, x8, #192            // =192
.LBB88_31:
	mov	w2, #3264
	mov	x1, x21
	bl	kmemdup
	str	x0, [x20, #32]
	cbz	x0, .LBB88_232
// %bb.32:
	adrp	x8, default_salloc_ops
	add	x8, x8, :lo12:default_salloc_ops
	str	x8, [x20]
	ldr	w8, [x22, #80]
	lsr	w9, w24, #1
	mov	x10, #236223201280
	mov	w0, #1
	str	w8, [x20, #8]
	ldr	w8, [x19, #1184]
	str	w21, [x20, #40]
	str	x10, [x20, #64]
	lsl	w8, w9, w8
	stp	w8, wzr, [x20, #12]
	ldr	x8, [x19, #224]
	ldr	x8, [x8, #168]
	str	x8, [x20, #136]
	bl	ktime_get_coarse_with_offset
	mov	x8, #38067
	movk	x8, #9942, lsl #16
	movk	x8, #3048, lsl #32
	movk	x8, #4398, lsl #48
	smulh	x8, x0, x8
	asr	x9, x8, #26
	adrp	x1, .L.str.36
	adrp	x2, build_sit_info.__key
	add	x8, x9, x8, lsr #63
	add	x0, x20, #72            // =72
	add	x1, x1, :lo12:.L.str.36
	add	x2, x2, :lo12:build_sit_info.__key
	str	x8, [x20, #144]
	bl	__init_rwsem
	ldr	x0, [x23, :lo12:kmalloc_caches+56]
	mov	w1, #3520
	mov	w2, #32
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB88_232
// %bb.33:
	ldr	x8, [x26]
	mov	x20, x0
	mov	w1, #3264
	mov	w2, #-1
	str	x0, [x8, #8]
	ldr	x8, [x26]
	ldr	w8, [x8, #96]
	add	x8, x8, #63             // =63
	lsr	x8, x8, #3
	and	x21, x8, #0x3ffffff8
	mov	x0, x21
	bl	kvmalloc_node
	str	x0, [x20, #16]
	cbz	x0, .LBB88_232
// %bb.34:
	ldr	w8, [x19, #1200]
	mov	w1, #3264
	mov	w2, #-1
	add	x8, x8, #63             // =63
	lsr	x8, x8, #3
	and	x22, x8, #0x3ffffff8
	mov	x0, x22
	bl	kvmalloc_node
	str	x0, [x20, #24]
	cbz	x0, .LBB88_232
// %bb.35:
	ldr	x0, [x20, #16]
	mov	w1, #255
	mov	x2, x21
	bl	memset
	ldr	x0, [x20, #24]
	mov	w1, #255
	mov	x2, x22
	bl	memset
	ldr	x9, [x26]
	cbz	x9, .LBB88_37
// %bb.36:
	add	x8, x9, #84             // =84
	add	x9, x9, #80             // =80
	b	.LBB88_38
.LBB88_37:
	ldr	x9, [x19, #16]
	add	x8, x9, #92             // =92
	add	x9, x9, #72             // =72
.LBB88_38:
	ldr	w8, [x8]
	ldr	w9, [x9]
	ldr	w10, [x19, #1184]
	adrp	x11, kmalloc_caches+80
	ldr	x0, [x11, :lo12:kmalloc_caches+80]
	sub	w8, w8, w9
	lsr	w8, w8, w10
	mov	w1, #3520
	mov	w2, #960
	stur	xzr, [x20, #4]
	str	w8, [x20]
	str	wzr, [x20, #12]
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB88_232
// %bb.39:
	ldr	x8, [x26]
	mov	x20, x0
	mov	x27, xzr
	mov	x21, xzr
	mov	w23, #4526
	mov	w22, #4591
	mov	w24, #-1
	str	x0, [x8, #24]
	b	.LBB88_43
.LBB88_40:                              //   in Loop: Header=BB88_43 Depth=1
	mov	w8, w21
.LBB88_41:                              //   in Loop: Header=BB88_43 Depth=1
	add	x9, x20, x27
	strh	w8, [x9, #98]
.LBB88_42:                              //   in Loop: Header=BB88_43 Depth=1
	add	x8, x20, x27
	add	x27, x27, #120          // =120
	cmp	x27, #960               // =960
	add	x21, x21, #1            // =1
	str	w24, [x8, #100]
	strh	wzr, [x8, #104]
	strb	wzr, [x8, #116]
	b.eq	.LBB88_48
.LBB88_43:                              // =>This Inner Loop Header: Depth=1
	add	x25, x20, x27
	adrp	x1, .L.str.37
	adrp	x2, build_curseg.__key
	mov	x0, x25
	add	x1, x1, :lo12:.L.str.37
	add	x2, x2, :lo12:build_curseg.__key
	bl	__mutex_init
	adrp	x8, kmalloc_caches+96
	ldr	x0, [x8, :lo12:kmalloc_caches+96]
	mov	w1, #3520
	mov	w2, #4096
	bl	kmem_cache_alloc_trace
	str	x0, [x25, #32]
	cbz	x0, .LBB88_232
// %bb.44:                              //   in Loop: Header=BB88_43 Depth=1
	adrp	x1, .L.str.39
	adrp	x2, build_curseg.__key.38
	add	x0, x25, #40            // =40
	add	x1, x1, :lo12:.L.str.39
	add	x2, x2, :lo12:build_curseg.__key.38
	bl	__init_rwsem
	cmp	x27, #240               // =240
	mov	w8, #507
	csel	x8, x23, x8, eq
	cmp	x27, #0                 // =0
	csel	x0, x22, x8, eq
	mov	w1, #3520
	bl	__kmalloc
	str	x0, [x25, #88]
	cbz	x0, .LBB88_232
// %bb.45:                              //   in Loop: Header=BB88_43 Depth=1
	cmp	x21, #5                 // =5
	b.ls	.LBB88_40
// %bb.46:                              //   in Loop: Header=BB88_43 Depth=1
	and	x8, x21, #0xfffffffe
	cmp	x8, #6                  // =6
	b.ne	.LBB88_42
// %bb.47:                              //   in Loop: Header=BB88_43 Depth=1
	mov	w8, #2
	b	.LBB88_41
.LBB88_48:
	ldr	x8, [x19, #136]
	ldr	x21, [x19, #224]
	ldr	x8, [x8, #24]
	ldrb	w9, [x21, #132]
	ldr	x10, [x8, #328]
	ldr	x8, [x8, #88]
	stp	x8, x10, [sp, #24]      // 16-byte Folded Spill
	tbnz	w9, #2, .LBB88_50
// %bb.49:
	mov	w20, wzr
	b	.LBB88_67
.LBB88_50:
	ldrb	w8, [x21, #176]
	cmp	w8, #1                  // =1
	b.ne	.LBB88_53
// %bb.51:
	ldrb	w9, [x21, #177]
	ldr	w8, [x19, #1188]
	cmp	w9, #1                  // =1
	b.ne	.LBB88_55
// %bb.52:
	ldrb	w9, [x21, #178]
	cmp	w9, #1                  // =1
	lsl	w9, w8, #1
	b.ne	.LBB88_57
	b	.LBB88_58
.LBB88_53:
	ldrb	w9, [x21, #177]
	ldrh	w8, [x21, #116]
	cmp	w9, #1                  // =1
	b.ne	.LBB88_55
// %bb.54:
	ldrb	w9, [x21, #178]
	ldr	w10, [x19, #1188]
	cmp	w9, #1                  // =1
	add	w9, w10, w8
	mov	w8, w10
	b.eq	.LBB88_58
	b	.LBB88_57
.LBB88_55:
	ldrb	w9, [x21, #178]
	ldrh	w10, [x21, #118]
	cmp	w9, #1                  // =1
	add	w9, w8, w10
	b.ne	.LBB88_57
// %bb.56:
	ldr	w8, [x19, #1188]
	b	.LBB88_58
.LBB88_57:
	ldrh	w8, [x21, #120]
.LBB88_58:
	add	w8, w8, w9
	sub	w10, w8, #439           // =439
	mov	w9, #2
	cmp	w10, #585               // =585
	cinc	w9, w9, hs
	cmp	w8, #440                // =440
	csinc	w2, w9, wzr, ge
	cmp	w2, #2                  // =2
	b.lo	.LBB88_62
// %bb.59:
	ldr	x8, [x19, #16]
	ldr	w9, [x19, #232]
	ldr	w8, [x8, #76]
	cmp	w9, #2                  // =2
	b.ne	.LBB88_61
// %bb.60:
	ldr	w9, [x19, #1188]
	add	w8, w9, w8
.LBB88_61:
	ldr	w9, [x21, #140]
	mov	w4, #1
	mov	x0, x19
	mov	w3, wzr
	add	w1, w9, w8
	bl	f2fs_ra_meta_pages
	ldr	x21, [x19, #224]
.LBB88_62:
	ldr	x8, [x19, #16]
	ldr	w9, [x19, #232]
	ldr	w8, [x8, #76]
	cmp	w9, #2                  // =2
	b.ne	.LBB88_64
// %bb.63:
	ldr	w9, [x19, #1188]
	add	w8, w9, w8
.LBB88_64:
	ldr	w9, [x21, #140]
	mov	x0, x19
	add	w20, w9, w8
	mov	x1, x20
	bl	f2fs_get_meta_page
	mov	x22, x0
	cmn	x0, #1, lsl #12         // =4096
	b.ls	.LBB88_120
.LBB88_65:
	cbnz	w22, .LBB88_233
.LBB88_66:
	mov	w20, #3
.LBB88_67:
	ldr	x8, [x19, #224]
	mov	w25, #33
	ldrb	w9, [x8, #132]
	tst	w9, w25
	b.eq	.LBB88_73
// %bb.68:
	ldr	x9, [x19, #16]
	ldr	w10, [x19, #232]
	ldr	w9, [x9, #76]
	cmp	w10, #2                 // =2
	b.ne	.LBB88_70
// %bb.69:
	ldr	w10, [x19, #1188]
	add	w9, w10, w9
.LBB88_70:
	ldr	w8, [x8, #136]
	add	w9, w20, w9
	mov	w10, #6
	sub	w2, w10, w20
	add	w8, w9, w8
	sub	w1, w8, #7              // =7
	mov	w4, #1
	mov	x0, x19
	mov	w3, wzr
	bl	f2fs_ra_meta_pages
	b	.LBB88_73
.LBB88_71:                              //   in Loop: Header=BB88_73 Depth=1
	mov	w22, w23
	cbnz	w23, .LBB88_233
.LBB88_72:                              //   in Loop: Header=BB88_73 Depth=1
	add	x20, x20, #1            // =1
	cmp	x20, #6                 // =6
	b.eq	.LBB88_93
.LBB88_73:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB88_89 Depth 2
	ldr	x27, [x19, #224]
	cmp	x20, #2                 // =2
	b.hi	.LBB88_78
// %bb.74:                              //   in Loop: Header=BB88_73 Depth=1
	ldr	x10, [x19, #16]
	add	x8, x27, x20, lsl #2
	add	x9, x27, x20, lsl #1
	ldrb	w11, [x27, #132]
	ldr	w21, [x8, #84]
	ldrh	w22, [x9, #116]
	ldr	w8, [x10, #76]
	ldr	w9, [x19, #232]
	tst	w11, w25
	b.ne	.LBB88_80
// %bb.75:                              //   in Loop: Header=BB88_73 Depth=1
	cmp	w9, #2                  // =2
	b.ne	.LBB88_77
// %bb.76:                              //   in Loop: Header=BB88_73 Depth=1
	ldr	w9, [x19, #1188]
	add	w8, w9, w8
.LBB88_77:                              //   in Loop: Header=BB88_73 Depth=1
	ldr	w9, [x27, #136]
	add	w8, w20, w8
	add	w8, w8, w9
	sub	w8, w8, #4              // =4
	b	.LBB88_84
.LBB88_78:                              //   in Loop: Header=BB88_73 Depth=1
	sub	x8, x20, #3             // =3
	ldrb	w9, [x27, #132]
	add	x10, x27, x8, lsl #2
	add	x8, x27, x8, lsl #1
	ldr	w21, [x10, #36]
	ldrh	w22, [x8, #68]
	tst	w9, w25
	b.eq	.LBB88_83
// %bb.79:                              //   in Loop: Header=BB88_73 Depth=1
	ldr	x8, [x19, #16]
	ldr	w9, [x19, #232]
	ldr	w8, [x8, #76]
.LBB88_80:                              //   in Loop: Header=BB88_73 Depth=1
	cmp	w9, #2                  // =2
	b.ne	.LBB88_82
// %bb.81:                              //   in Loop: Header=BB88_73 Depth=1
	ldr	w9, [x19, #1188]
	add	w8, w9, w8
.LBB88_82:                              //   in Loop: Header=BB88_73 Depth=1
	ldr	w9, [x27, #136]
	add	w8, w20, w8
	add	w8, w8, w9
	sub	w8, w8, #7              // =7
	b	.LBB88_84
.LBB88_83:                              //   in Loop: Header=BB88_73 Depth=1
	ldr	x8, [x26]
	ldr	w8, [x8, #88]
	add	w8, w8, w21
.LBB88_84:                              //   in Loop: Header=BB88_73 Depth=1
	mov	w1, w8
	mov	x0, x19
	bl	f2fs_get_meta_page
	mov	x23, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB88_71
// %bb.85:                              //   in Loop: Header=BB88_73 Depth=1
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x8, x23, x8
	sub	w9, w20, #3             // =3
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	cmp	w9, #2                  // =2
	mov	x9, #-549755813888
	add	x24, x8, x9
	str	w22, [sp, #48]          // 4-byte Folded Spill
	b.hi	.LBB88_91
// %bb.86:                              //   in Loop: Header=BB88_73 Depth=1
	ldr	x8, [x19, #224]
	ldrb	w8, [x8, #132]
	tst	w8, w25
	b.eq	.LBB88_90
// %bb.87:                              //   in Loop: Header=BB88_73 Depth=1
	ldr	w8, [x19, #1188]
	cbz	w8, .LBB88_91
// %bb.88:                              //   in Loop: Header=BB88_73 Depth=1
	mov	w8, wzr
	add	x9, x24, #5             // =5
.LBB88_89:                              //   Parent Loop BB88_73 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	sturb	wzr, [x9, #-1]
	strh	wzr, [x9], #7
	ldr	w10, [x19, #1188]
	add	w8, w8, #1              // =1
	cmp	w8, w10
	b.lo	.LBB88_89
	b	.LBB88_91
.LBB88_90:                              //   in Loop: Header=BB88_73 Depth=1
	mov	x0, x19
	mov	w1, w21
	mov	x2, x24
	bl	f2fs_restore_node_summary
	mov	w22, w0
	cbnz	w0, .LBB88_92
.LBB88_91:                              //   in Loop: Header=BB88_73 Depth=1
	ldr	x8, [x19, #136]
	mov	w9, #120
	ldr	x8, [x8, #24]
	madd	x22, x20, x9, x8
	mov	x0, x22
	bl	mutex_lock
	add	x25, x22, #40           // =40
	mov	x0, x25
	bl	down_write
	ldr	x0, [x22, #88]
	add	x1, x24, #3584          // =3584
	mov	w2, #507
	bl	memcpy
	mov	x0, x25
	mov	w25, #33
	bl	up_write
	ldr	x0, [x22, #32]
	mov	w2, #3584
	mov	x1, x24
	bl	memcpy
	ldr	x8, [x22, #32]
	add	x1, x24, #4091          // =4091
	mov	w2, #5
	add	x0, x8, #4091           // =4091
	bl	memcpy
	mov	x0, x19
	mov	w1, w20
	mov	w2, wzr
	str	w21, [x22, #112]
	bl	reset_curseg
	add	x8, x27, x20
	ldrb	w8, [x8, #176]
	ldr	w9, [sp, #48]           // 4-byte Folded Reload
	mov	x0, x22
	strb	w8, [x22, #96]
	strh	w9, [x22, #104]
	bl	mutex_unlock
	mov	w22, wzr
.LBB88_92:                              //   in Loop: Header=BB88_73 Depth=1
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_put_page
	cbz	w22, .LBB88_72
	b	.LBB88_233
.LBB88_93:
	ldr	x8, [sp, #24]           // 8-byte Folded Reload
	ldrh	w2, [x8]
	ldr	x8, [sp, #32]           // 8-byte Folded Reload
	cmp	w2, #353                // =353
	ldrh	w3, [x8]
	b.hi	.LBB88_131
// %bb.94:
	cmp	w3, #59                 // =59
	b.hs	.LBB88_131
// %bb.95:
	ldr	x8, [x19, #16]
	ldr	w9, [x19, #232]
	ldr	w8, [x8, #76]
	cmp	w9, #2                  // =2
	b.ne	.LBB88_97
// %bb.96:
	ldr	w9, [x19, #1188]
	add	w8, w9, w8
.LBB88_97:
	ldr	x9, [x19, #224]
	ldp	w9, w10, [x9, #132]
	add	w8, w10, w8
	tbz	w9, #11, .LBB88_99
// %bb.98:
	ldr	x9, [x19, #136]
	mov	w1, w8
	mov	x0, x19
	add	w22, w8, #1             // =1
	ldr	x21, [x9, #24]
	bl	f2fs_get_meta_page
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	ldr	x9, [x21, #88]
	add	x8, x0, x8
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	mov	x10, #-549755813888
	mov	x20, x0
	add	x1, x8, x10
	add	x0, x9, #496            // =496
	mov	w2, #4095
	bl	memcpy
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
	ldr	x8, [x19, #224]
	ldr	w9, [x8, #132]
	mov	w8, w22
.LBB88_99:
	tbz	w9, #10, .LBB88_101
// %bb.100:
	ldr	x9, [x19, #136]
	mov	w1, w8
	mov	x0, x19
	ldr	x21, [x9, #24]
	bl	f2fs_get_meta_page
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	ldr	x9, [x21, #328]
	add	x8, x0, x8
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	mov	x10, #-549755813888
	mov	x20, x0
	add	x1, x8, x10
	add	x0, x9, #470            // =470
	mov	w2, #4056
	bl	memcpy
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
.LBB88_101:
	ldr	x8, [x19, #136]
	mov	x9, #58387
	movk	x9, #16681, lsl #16
	movk	x9, #4766, lsl #32
	ldr	x11, [x8, #24]
	ldr	x25, [x8]
	movk	x9, #10724, lsl #48
	add	x10, sp, #56            // =56
	str	x11, [sp, #8]           // 8-byte Folded Spill
	ldr	x11, [x11, #328]
	strh	wzr, [sp, #128]
	stp	xzr, xzr, [sp, #112]
	stp	xzr, xzr, [sp, #96]
	stp	xzr, xzr, [sp, #80]
	stp	xzr, xzr, [sp, #64]
	str	xzr, [sp, #56]
	ldr	w8, [x8, #96]
	mov	w23, wzr
	orr	x10, x10, #0x2
	str	wzr, [sp, #48]          // 4-byte Folded Spill
	add	x8, x8, #54             // =54
	umulh	x9, x8, x9
	sub	x8, x8, x9
	add	x8, x9, x8, lsr #1
	lsr	x8, x8, #5
	str	x10, [sp, #32]          // 8-byte Folded Spill
	stp	x11, x8, [sp, #16]      // 16-byte Folded Spill
	b	.LBB88_103
.LBB88_102:                             //   in Loop: Header=BB88_103 Depth=1
	ldr	x8, [sp, #24]           // 8-byte Folded Reload
	cmp	w23, w8
	b.hs	.LBB88_174
.LBB88_103:                             // =>This Loop Header: Depth=1
                                        //     Child Loop BB88_107 Depth 2
	mov	w2, #256
	mov	w3, #2
	mov	w4, #1
	mov	x0, x19
	mov	w1, w23
	bl	f2fs_ra_meta_pages
	ldr	w9, [x25, #68]
	mul	w8, w9, w23
	add	w23, w0, w23
	mul	w9, w23, w9
	cmp	w8, w9
	b.hs	.LBB88_102
// %bb.104:                             //   in Loop: Header=BB88_103 Depth=1
	mov	w9, w9
	str	x9, [sp, #40]           // 8-byte Folded Spill
	mov	w9, #40
	mov	w24, w8
	umull	x8, w8, w9
	add	x20, x8, #32            // =32
	b	.LBB88_107
.LBB88_105:                             //   in Loop: Header=BB88_107 Depth=2
	add	x21, x21, x20
	ldr	w8, [x19, #1188]
	ldur	w9, [x21, #-32]
	ldur	x0, [x21, #-24]
	mov	w1, #255
	mov	w2, #64
	bfi	w9, w8, #6, #10
	stur	w9, [x21, #-32]
	ldr	w8, [x19, #1188]
	bfi	w9, w8, #16, #10
	stur	w9, [x21, #-32]
	bl	memset
	ldur	x0, [x21, #-16]
	mov	w1, #255
	mov	w2, #64
	bl	memset
	ldur	w8, [x21, #-32]
	and	w8, w8, #0xffffffc0
	orr	w8, w8, #0x4
	stur	w8, [x21, #-32]
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	str	x8, [x21]
.LBB88_106:                             //   in Loop: Header=BB88_107 Depth=2
	ldr	x8, [sp, #40]           // 8-byte Folded Reload
	add	x24, x24, #1            // =1
	add	x20, x20, #40           // =40
	cmp	x8, x24
	b.eq	.LBB88_102
.LBB88_107:                             //   Parent Loop BB88_103 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	x8, [x26]
	ldr	w8, [x8, #96]
	cmp	x24, x8
	b.hs	.LBB88_102
// %bb.108:                             //   in Loop: Header=BB88_107 Depth=2
	ldr	x21, [x25, #120]
	mov	x0, x19
	mov	w1, w24
	bl	current_sit_addr
	mov	x1, x0
	mov	x0, x19
	bl	f2fs_get_meta_page
	mov	x22, x0
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB88_132
// %bb.109:                             //   in Loop: Header=BB88_107 Depth=2
	ldr	w8, [x25, #68]
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	add	x9, x22, x9
	lsl	x9, x9, #6
	udiv	w10, w24, w8
	and	x9, x9, #0xfffffffffffff000
	msub	w8, w10, w8, w24
	mov	w10, #74
	umaddl	x8, w8, w10, x9
	mov	x9, #-549755813888
	add	x8, x8, x9
	ldp	x10, x9, [x8]
	mov	w1, #1
	mov	x0, x22
	stp	x10, x9, [sp, #56]
	ldp	x10, x9, [x8, #32]
	ldp	x12, x11, [x8, #16]
	stp	x10, x9, [sp, #88]
	stp	x12, x11, [sp, #72]
	ldrh	w9, [x8, #72]
	ldp	x11, x10, [x8, #56]
	ldr	x8, [x8, #48]
	strh	w9, [sp, #128]
	stp	x11, x10, [sp, #112]
	str	x8, [sp, #104]
	bl	f2fs_put_page
	add	x2, sp, #56             // =56
	mov	x0, x19
	mov	w1, w24
	bl	check_block_count
	cbnz	w0, .LBB88_147
// %bb.110:                             //   in Loop: Header=BB88_107 Depth=2
	ldr	w8, [x19, #3128]
	cmp	w8, #1                  // =1
	b.lt	.LBB88_115
// %bb.111:                             //   in Loop: Header=BB88_107 Depth=2
	ldr	x8, [x26]
	cbz	x8, .LBB88_113
// %bb.112:                             //   in Loop: Header=BB88_107 Depth=2
	add	x9, x8, #80             // =80
	b	.LBB88_114
.LBB88_113:                             //   in Loop: Header=BB88_107 Depth=2
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB88_114:                             //   in Loop: Header=BB88_107 Depth=2
	ldr	x8, [x8, #8]
	ldr	w10, [x19, #1184]
	ldr	w9, [x9]
	mov	x0, x19
	ldr	w8, [x8]
	add	w8, w24, w8
	lsl	w8, w8, w10
	add	w1, w8, w9
	bl	f2fs_target_device_index
	ldr	x8, [x19, #3136]
	mov	w9, #88
	smull	x9, w0, w9
	ldr	x8, [x8, x9]
	cbz	x8, .LBB88_105
.LBB88_115:                             //   in Loop: Header=BB88_107 Depth=2
	add	x22, x21, x20
	ldrh	w8, [sp, #56]
	ldur	w9, [x22, #-32]
	ldur	x0, [x22, #-24]
	mov	w2, #64
	bfi	w9, w8, #6, #10
	stur	w9, [x22, #-32]
	ldrh	w8, [sp, #56]
	and	w8, w8, #0x3ff
	bfi	w9, w8, #16, #10
	stur	w9, [x22, #-32]
	ldr	x27, [sp, #32]          // 8-byte Folded Reload
	mov	x1, x27
	bl	memcpy
	ldur	x0, [x22, #-16]
	mov	w2, #64
	mov	x1, x27
	bl	memcpy
	ldur	w8, [x22, #-32]
	ldrh	w9, [sp, #56]
	add	x21, x21, x20
	and	w8, w8, #0xffffffc0
	orr	w8, w8, w9, lsr #10
	stur	w8, [x22, #-32]
	ldur	x9, [sp, #122]
	and	w10, w8, #0x3f
	sub	w10, w10, #3            // =3
	cmp	w10, #3                 // =3
	str	x9, [x21]
	ldr	x9, [x19, #224]
	ldr	w10, [sp, #48]          // 4-byte Folded Reload
	ldur	x0, [x21, #-8]
	ubfx	w8, w8, #6, #10
	ldrb	w9, [x9, #133]
	csel	w8, w8, wzr, lo
	add	w10, w8, w10
	str	w10, [sp, #48]          // 4-byte Folded Spill
	tbnz	w9, #0, .LBB88_117
// %bb.116:                             //   in Loop: Header=BB88_107 Depth=2
	ldur	x1, [x21, #-24]
	mov	w2, #64
	bl	memcpy
	ldr	w8, [x19, #1188]
	ldur	w9, [x21, #-32]
	ldr	w10, [x19, #1244]
	ubfx	w9, w9, #6, #10
	add	w8, w10, w8
	sub	w8, w8, w9
	str	w8, [x19, #1244]
	b	.LBB88_118
.LBB88_117:                             //   in Loop: Header=BB88_107 Depth=2
	mov	w1, #255
	mov	w2, #64
	bl	memset
.LBB88_118:                             //   in Loop: Header=BB88_107 Depth=2
	ldr	w8, [x19, #1192]
	cmp	w8, #2                  // =2
	b.lo	.LBB88_106
// %bb.119:                             //   in Loop: Header=BB88_107 Depth=2
	ldr	x9, [x26]
	udiv	w8, w24, w8
	ldur	w10, [x21, #-32]
	lsl	x8, x8, #2
	ldr	x9, [x9]
	ubfx	w10, w10, #6, #10
	ldr	x9, [x9, #128]
	ldr	w11, [x9, x8]
	add	w10, w11, w10
	str	w10, [x9, x8]
	b	.LBB88_106
.LBB88_120:
	ldr	x8, [x26]
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	add	x9, x22, x9
	ldr	x8, [x8, #24]
	lsl	x9, x9, #6
	add	w25, w20, #1            // =1
	and	x20, x9, #0xfffffffffffff000
	ldr	x0, [x8, #88]
	mov	x8, #-549755813888
	str	x21, [sp, #16]          // 8-byte Folded Spill
	add	x21, x20, x8
	mov	w2, #507
	mov	x1, x21
	bl	memcpy
	ldr	x8, [x26]
	mov	w2, #507
	ldr	x8, [x8, #24]
	ldr	x0, [x8, #328]
	mov	x8, #-549755813888
	movk	x8, #507
	add	x1, x20, x8
	bl	memcpy
	mov	x8, xzr
	mov	w24, #1014
	b	.LBB88_122
.LBB88_121:                             //   in Loop: Header=BB88_122 Depth=1
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	add	x8, x8, #1              // =1
	cmp	x8, #3                  // =3
	b.eq	.LBB88_173
.LBB88_122:                             // =>This Loop Header: Depth=1
                                        //     Child Loop BB88_128 Depth 2
	str	x8, [sp, #48]           // 8-byte Folded Spill
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldr	x9, [sp, #48]           // 8-byte Folded Reload
	ldr	x8, [x19, #136]
	mov	w10, #120
	mov	x0, x19
	add	x9, x23, x9, lsl #2
	ldr	x11, [x8, #24]
	ldr	w8, [x9, #84]
	ldr	x9, [sp, #48]           // 8-byte Folded Reload
	ldr	x1, [sp, #48]           // 8-byte Folded Reload
	mov	w2, wzr
	str	x11, [sp, #40]          // 8-byte Folded Spill
	add	x9, x23, x9, lsl #1
	ldrh	w20, [x9, #116]
	ldr	x9, [sp, #48]           // 8-byte Folded Reload
                                        // kill: def $w1 killed $w1 killed $x1
	madd	x27, x9, x10, x11
	str	w8, [x27, #112]
	bl	reset_curseg
	ldr	x9, [sp, #48]           // 8-byte Folded Reload
	add	x8, x23, x9
	ldrb	w8, [x8, #176]
	strh	w20, [x27, #104]
	cmp	w8, #1                  // =1
	strb	w8, [x27, #96]
	b.ne	.LBB88_124
// %bb.123:                             //   in Loop: Header=BB88_122 Depth=1
	ldr	w20, [x19, #1188]
.LBB88_124:                             //   in Loop: Header=BB88_122 Depth=1
	tst	w20, #0xffff
	b.eq	.LBB88_121
// %bb.125:                             //   in Loop: Header=BB88_122 Depth=1
	ldr	x10, [sp, #40]          // 8-byte Folded Reload
	mov	w8, #120
	mov	x27, xzr
	madd	x8, x9, x8, x10
	and	x9, x20, #0xffff
	add	x23, x8, #32            // =32
	lsl	x8, x9, #3
	sub	x20, x8, x9
	b	.LBB88_128
.LBB88_126:                             //   in Loop: Header=BB88_128 Depth=2
	add	w24, w24, #7            // =7
.LBB88_127:                             //   in Loop: Header=BB88_128 Depth=2
	add	x27, x27, #7            // =7
	cmp	x20, x27
	b.eq	.LBB88_121
.LBB88_128:                             //   Parent Loop BB88_122 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	add	x8, x21, w24, sxtw
	ldr	x9, [x23]
	ldur	w11, [x8, #3]
	ldr	w8, [x8]
	add	w10, w24, #14           // =14
	cmp	w10, #4092              // =4092
	add	x9, x9, x27
	stur	w11, [x9, #3]
	str	w8, [x9]
	b.lo	.LBB88_126
// %bb.129:                             //   in Loop: Header=BB88_128 Depth=2
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
	mov	w1, w25
	mov	x0, x19
	bl	f2fs_get_meta_page
	mov	x22, x0
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB88_65
// %bb.130:                             //   in Loop: Header=BB88_128 Depth=2
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	add	x8, x22, x8
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	mov	x9, #-549755813888
	mov	w24, wzr
	add	w25, w25, #1            // =1
	add	x21, x8, x9
	b	.LBB88_127
.LBB88_131:
	adrp	x1, .L.str.40
	add	x1, x1, :lo12:.L.str.40
	mov	x0, x19
	bl	f2fs_printk
	mov	w22, #-22
	b	.LBB88_233
.LBB88_132:
	cbnz	w22, .LBB88_233
.LBB88_133:
	ldr	x8, [x26]
	ldr	w9, [x8, #96]
	cbz	w9, .LBB88_148
// %bb.134:
	mov	x22, xzr
	mov	x23, xzr
	mov	w25, #88
	b	.LBB88_137
.LBB88_135:                             //   in Loop: Header=BB88_137 Depth=1
	ldr	w8, [x9, #16]
	add	w8, w8, w10
	str	w8, [x9, #16]
.LBB88_136:                             //   in Loop: Header=BB88_137 Depth=1
	ldr	x8, [x26]
	add	x23, x23, #1            // =1
	add	x22, x22, #40           // =40
	ldr	w9, [x8, #96]
	cmp	x23, x9
	b.hs	.LBB88_148
.LBB88_137:                             // =>This Inner Loop Header: Depth=1
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB88_143
// %bb.138:                             //   in Loop: Header=BB88_137 Depth=1
	cbz	x8, .LBB88_140
// %bb.139:                             //   in Loop: Header=BB88_137 Depth=1
	add	x9, x8, #80             // =80
	b	.LBB88_141
.LBB88_140:                             //   in Loop: Header=BB88_137 Depth=1
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB88_141:                             //   in Loop: Header=BB88_137 Depth=1
	ldr	x8, [x8, #8]
	ldr	w10, [x19, #1184]
	ldr	w9, [x9]
	mov	x0, x19
	ldr	w8, [x8]
	add	w8, w23, w8
	lsl	w8, w8, w10
	add	w1, w8, w9
	bl	f2fs_target_device_index
	ldr	x8, [x19, #3136]
	smull	x9, w0, w25
	ldr	x8, [x8, x9]
	cbz	x8, .LBB88_136
// %bb.142:                             //   in Loop: Header=BB88_137 Depth=1
	ldr	x8, [x26]
.LBB88_143:                             //   in Loop: Header=BB88_137 Depth=1
	ldr	x9, [x8]
	ldr	x10, [x9, #120]
	ldr	w10, [x10, x22]
	ubfx	w10, w10, #6, #10
	cbnz	w10, .LBB88_135
// %bb.144:                             //   in Loop: Header=BB88_137 Depth=1
	ldr	x27, [x8, #8]
	ldr	w9, [x19, #1192]
	add	x20, x27, #12           // =12
	udiv	w24, w23, w9
	mov	x0, x20
	mul	w21, w24, w9
	bl	_raw_spin_lock
	ldr	x8, [x27, #16]
	ubfx	x9, x23, #6, #26
	mov	x2, x21
	add	x8, x8, x9, lsl #3
	mov	w9, #1
	lsl	x9, x9, x23
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	w8, [x27, #4]
	ldr	x0, [x27, #16]
	add	w8, w8, #1              // =1
	str	w8, [x27, #4]
	ldr	w8, [x19, #1192]
	add	w1, w8, w21
	bl	find_next_bit
	ldr	w8, [x19, #1192]
	add	w8, w8, w21
	cmp	w8, w0
	b.hi	.LBB88_146
// %bb.145:                             //   in Loop: Header=BB88_137 Depth=1
	ldr	x8, [x27, #24]
	lsr	w9, w24, #6
	add	x8, x8, w9, uxtw #3
	mov	w9, #1
	lsl	x9, x9, x24
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	w8, [x27, #8]
	add	w8, w8, #1              // =1
	str	w8, [x27, #8]
.LBB88_146:                             //   in Loop: Header=BB88_137 Depth=1
	mov	x0, x20
	bl	_raw_spin_unlock
	b	.LBB88_136
.LBB88_147:
	mov	w22, w0
	b	.LBB88_233
.LBB88_148:
	mov	w21, #100
	mov	w22, #1
.LBB88_149:                             // =>This Inner Loop Header: Depth=1
	ldr	x9, [x8, #24]
	ldr	x23, [x8, #8]
	ldr	w25, [x9, x21]
	cmn	w25, #1                 // =1
	b.eq	.LBB88_151
// %bb.150:                             //   in Loop: Header=BB88_149 Depth=1
	ldr	w8, [x19, #1192]
	udiv	w24, w25, w8
	b	.LBB88_152
.LBB88_151:                             //   in Loop: Header=BB88_149 Depth=1
	mov	w24, #-1
.LBB88_152:                             //   in Loop: Header=BB88_149 Depth=1
	add	x20, x23, #12           // =12
	mov	x0, x20
	bl	_raw_spin_lock
	ldr	x8, [x23, #16]
	ubfx	x9, x25, #6, #26
	ldr	x10, [x8, w9, uxtw #3]
	str	x10, [sp, #56]
	ldr	x11, [sp, #56]
	lsl	x10, x22, x25
	tst	x11, x10
	b.ne	.LBB88_157
// %bb.153:                             //   in Loop: Header=BB88_149 Depth=1
	add	x8, x8, x9, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x11, x9, x10
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tst	x9, x10
	b.ne	.LBB88_157
// %bb.154:                             //   in Loop: Header=BB88_149 Depth=1
	ldr	w9, [x23, #4]
	ldr	x8, [x23, #24]
	lsr	w10, w24, #6
	sub	w9, w9, #1              // =1
	str	w9, [x23, #4]
	ldr	x9, [x8, w10, uxtw #3]
	str	x9, [sp, #56]
	ldr	x11, [sp, #56]
	lsl	x9, x22, x24
	tst	x11, x9
	b.ne	.LBB88_157
// %bb.155:                             //   in Loop: Header=BB88_149 Depth=1
	add	x8, x8, x10, lsl #3
	//APP
	// atomic64_fetch_or
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	orr	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tst	x10, x9
	b.ne	.LBB88_157
// %bb.156:                             //   in Loop: Header=BB88_149 Depth=1
	ldr	w8, [x23, #8]
	sub	w8, w8, #1              // =1
	str	w8, [x23, #8]
.LBB88_157:                             //   in Loop: Header=BB88_149 Depth=1
	mov	x0, x20
	bl	_raw_spin_unlock
	cmp	x21, #700               // =700
	b.eq	.LBB88_159
// %bb.158:                             //   in Loop: Header=BB88_149 Depth=1
	ldr	x8, [x26]
	add	x21, x21, #120          // =120
	b	.LBB88_149
.LBB88_159:
	ldr	x0, [x28, :lo12:kmalloc_caches+64]
	mov	w1, #3520
	mov	w2, #152
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB88_232
// %bb.160:
	ldr	x8, [x26]
	adrp	x1, .L.str.43
	adrp	x2, build_dirty_segmap.__key
	mov	x20, x0
	str	x0, [x8, #16]
	add	x0, x0, #80             // =80
	add	x1, x1, :lo12:.L.str.43
	add	x2, x2, :lo12:build_dirty_segmap.__key
	bl	__mutex_init
	ldr	x8, [x26]
	mov	w1, #3520
	mov	w2, #-1
	ldr	w8, [x8, #96]
	add	x8, x8, #63             // =63
	lsr	x8, x8, #3
	and	x21, x8, #0x3ffffff8
	mov	x0, x21
	bl	kvmalloc_node
	str	x0, [x20, #8]
	cbz	x0, .LBB88_232
// %bb.161:
	mov	w1, #3520
	mov	w2, #-1
	mov	x0, x21
	bl	kvmalloc_node
	str	x0, [x20, #16]
	cbz	x0, .LBB88_232
// %bb.162:
	mov	w1, #3520
	mov	w2, #-1
	mov	x0, x21
	bl	kvmalloc_node
	str	x0, [x20, #24]
	cbz	x0, .LBB88_232
// %bb.163:
	mov	w1, #3520
	mov	w2, #-1
	mov	x0, x21
	bl	kvmalloc_node
	str	x0, [x20, #32]
	cbz	x0, .LBB88_232
// %bb.164:
	mov	w1, #3520
	mov	w2, #-1
	mov	x0, x21
	bl	kvmalloc_node
	str	x0, [x20, #40]
	cbz	x0, .LBB88_232
// %bb.165:
	mov	w1, #3520
	mov	w2, #-1
	mov	x0, x21
	bl	kvmalloc_node
	str	x0, [x20, #48]
	cbz	x0, .LBB88_232
// %bb.166:
	mov	w1, #3520
	mov	w2, #-1
	mov	x0, x21
	bl	kvmalloc_node
	str	x0, [x20, #56]
	cbz	x0, .LBB88_232
// %bb.167:
	mov	w1, #3520
	mov	w2, #-1
	mov	x0, x21
	bl	kvmalloc_node
	str	x0, [x20, #64]
	cbz	x0, .LBB88_232
// %bb.168:
	ldr	w8, [x19, #1192]
	cmp	w8, #1                  // =1
	str	w8, [sp, #40]           // 4-byte Folded Spill
	b.ls	.LBB88_171
// %bb.169:
	ldr	w8, [x19, #1200]
	mov	w1, #3520
	mov	w2, #-1
	add	x8, x8, #63             // =63
	lsr	x8, x8, #3
	and	x0, x8, #0x3ffffff8
	bl	kvmalloc_node
	str	x0, [x20, #72]
	cbz	x0, .LBB88_232
// %bb.170:
	ldr	w8, [x19, #1192]
	str	w8, [sp, #40]           // 4-byte Folded Spill
.LBB88_171:
	ldr	x8, [x19, #136]
	ldp	x24, x9, [x8, #8]
	ldr	w21, [x8, #96]
	str	x9, [sp, #48]           // 8-byte Folded Spill
	ldr	w9, [x19, #1188]
	add	x20, x24, #12           // =12
	mov	x0, x20
	str	w9, [sp, #32]           // 4-byte Folded Spill
	bl	_raw_spin_lock
	ldr	x0, [x24, #16]
	mov	x1, x21
	mov	x2, xzr
	bl	find_next_bit
	mov	x25, x0
	mov	x0, x20
	bl	_raw_spin_unlock
	ldr	x8, [x19, #136]
	ldr	w9, [x8, #96]
	cmp	w9, w25
	b.ls	.LBB88_211
// %bb.172:
	ldr	x9, [sp, #48]           // 8-byte Folded Reload
	mov	w22, #40
	adrp	x23, f2fs_dclient
	mov	w28, #88
	add	x21, x9, #80            // =80
	add	x9, x19, #3408          // =3408
	str	x9, [sp, #24]           // 8-byte Folded Spill
	b	.LBB88_194
.LBB88_173:
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
	b	.LBB88_66
.LBB88_174:
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	add	x0, x8, #280            // =280
	str	x0, [sp, #8]            // 8-byte Folded Spill
	bl	down_read
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	ldrh	w8, [x8]
	cbz	w8, .LBB88_184
// %bb.175:
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	str	xzr, [sp, #40]          // 8-byte Folded Spill
	add	x27, x8, #6             // =6
	b	.LBB88_177
.LBB88_176:                             //   in Loop: Header=BB88_177 Depth=1
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	ldr	x9, [sp, #40]           // 8-byte Folded Reload
	add	x27, x27, #78           // =78
	ldrh	w8, [x8]
	add	x9, x9, #1              // =1
	str	x9, [sp, #40]           // 8-byte Folded Spill
	cmp	x9, x8
	b.hs	.LBB88_184
.LBB88_177:                             // =>This Inner Loop Header: Depth=1
	ldr	x8, [x26]
	ldur	w24, [x27, #-4]
	ldr	w8, [x8, #96]
	cmp	w24, w8
	b.hs	.LBB88_189
// %bb.178:                             //   in Loop: Header=BB88_177 Depth=1
	ldp	x9, x8, [x27]
	ldr	x21, [x25, #120]
	add	x2, sp, #56             // =56
	mov	x0, x19
	stp	x9, x8, [sp, #56]
	ldp	x9, x8, [x27, #32]
	ldp	x11, x10, [x27, #16]
	mov	w1, w24
	stp	x9, x8, [sp, #88]
	stp	x11, x10, [sp, #72]
	ldrh	w8, [x27, #72]
	ldp	x10, x9, [x27, #56]
	ldr	x11, [x27, #48]
	strh	w8, [sp, #128]
	mov	w8, #40
	stp	x10, x9, [sp, #112]
	str	x11, [sp, #104]
	madd	x23, x24, x8, x21
	ldr	w20, [x23]
	bl	check_block_count
	cbnz	w0, .LBB88_190
// %bb.179:                             //   in Loop: Header=BB88_177 Depth=1
	ldrh	w8, [sp, #56]
	ldr	w9, [x23]
	ubfx	w10, w20, #6, #10
	mov	w2, #64
	bfi	w9, w8, #6, #10
	str	w9, [x23]
	ldrh	w8, [sp, #56]
	and	w8, w8, #0x3ff
	bfi	w9, w8, #16, #10
	str	w9, [x23]
	mov	w9, #40
	madd	x21, x24, x9, x21
	and	w8, w20, #0x3f
	ldr	x0, [x21, #8]!
	ldr	w9, [sp, #48]           // 4-byte Folded Reload
	ldr	x20, [sp, #32]          // 8-byte Folded Reload
	sub	w8, w8, #3              // =3
	cmp	w8, #3                  // =3
	csneg	w8, wzr, w10, hs
	add	w8, w8, w9
	mov	x1, x20
	str	w10, [sp, #24]          // 4-byte Folded Spill
	str	w8, [sp, #48]           // 4-byte Folded Spill
	bl	memcpy
	ldr	x0, [x21, #8]
	mov	w2, #64
	mov	x1, x20
	bl	memcpy
	ldr	w8, [x23]
	ldrh	w9, [sp, #56]
	and	w10, w8, #0xffffffc0
	orr	w10, w10, w9, lsr #10
	str	w10, [x23]
	ldur	x10, [sp, #122]
	ldr	x0, [x21, #16]
	sub	w9, w9, #3072           // =3072
	ubfx	w8, w8, #6, #10
	str	x10, [x21, #24]
	ldr	x10, [x19, #224]
	cmp	w9, #3072               // =3072
	csel	w22, w8, wzr, lo
	ldrb	w10, [x10, #133]
	tbnz	w10, #0, .LBB88_181
// %bb.180:                             //   in Loop: Header=BB88_177 Depth=1
	ldr	x1, [x21]
	mov	w2, #64
	bl	memcpy
	ldr	w8, [x19, #1244]
	ldr	w9, [sp, #24]           // 4-byte Folded Reload
	add	w8, w8, w9
	str	w8, [x19, #1244]
	ldr	w9, [x23]
	ubfx	w9, w9, #6, #10
	sub	w8, w8, w9
	str	w8, [x19, #1244]
	b	.LBB88_182
.LBB88_181:                             //   in Loop: Header=BB88_177 Depth=1
	mov	w1, #255
	mov	w2, #64
	bl	memset
.LBB88_182:                             //   in Loop: Header=BB88_177 Depth=1
	ldr	w8, [x19, #1192]
	ldr	w9, [sp, #48]           // 4-byte Folded Reload
	cmp	w8, #2                  // =2
	add	w9, w9, w22
	str	w9, [sp, #48]           // 4-byte Folded Spill
	b.lo	.LBB88_176
// %bb.183:                             //   in Loop: Header=BB88_177 Depth=1
	ldr	x9, [x19, #136]
	udiv	w8, w24, w8
	ldr	w10, [x23]
	lsl	x8, x8, #2
	ldr	x9, [x9]
	ubfx	w10, w10, #6, #10
	ldr	x9, [x9, #128]
	ldr	w11, [x9, x8]
	add	w10, w11, w10
	str	w10, [x9, x8]
	ldr	x8, [x19, #136]
	ldr	w9, [x19, #1192]
	ldr	w11, [sp, #24]          // 4-byte Folded Reload
	ldr	x8, [x8]
	udiv	w9, w24, w9
	lsl	x9, x9, #2
	ldr	x8, [x8, #128]
	ldr	w10, [x8, x9]
	sub	w10, w10, w11
	str	w10, [x8, x9]
	b	.LBB88_176
.LBB88_184:
	ldr	x0, [sp, #8]            // 8-byte Folded Reload
	bl	up_read
	ldr	w3, [x19, #1208]
	ldr	w8, [sp, #48]           // 4-byte Folded Reload
	cmp	w8, w3
	b.eq	.LBB88_133
// %bb.185:
	ldr	w2, [sp, #48]           // 4-byte Folded Reload
	adrp	x1, .L.str.42
	add	x1, x1, :lo12:.L.str.42
	mov	x0, x19
	bl	f2fs_printk
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	cbz	x0, .LBB88_188
// %bb.186:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB88_188
// %bb.187:
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.30
	adrp	x2, .L__func__.build_sit_entries
	add	x1, x1, :lo12:.L.str.30
	add	x2, x2, :lo12:.L__func__.build_sit_entries
	mov	w3, #6758
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w8, #15241
	movk	w8, #14160, lsl #16
	add	w1, w8, #3              // =3
	bl	dsm_client_notify
.LBB88_188:
	mov	w22, #-117
	b	.LBB88_233
.LBB88_189:
	adrp	x1, .L.str.41
	add	x1, x1, :lo12:.L.str.41
	mov	x0, x19
	mov	w2, w24
	bl	f2fs_printk
	mov	w22, #-117
	b	.LBB88_191
.LBB88_190:
	mov	w22, w0
.LBB88_191:
	ldr	x0, [sp, #8]            // 8-byte Folded Reload
	bl	up_read
	b	.LBB88_233
.LBB88_192:                             //   in Loop: Header=BB88_194 Depth=1
	mov	x0, x21
	bl	mutex_lock
	mov	w2, #6
	mov	x0, x19
	mov	w1, w25
	bl	__locate_dirty_segment
	mov	x0, x21
	bl	mutex_unlock
.LBB88_193:                             //   in Loop: Header=BB88_194 Depth=1
	ldr	x8, [x26]
	mov	x0, x20
	ldr	w25, [x8, #96]
	bl	_raw_spin_lock
	ldr	x0, [x24, #16]
	mov	w2, w27
	mov	x1, x25
	bl	find_next_bit
	mov	x25, x0
	mov	x0, x20
	bl	_raw_spin_unlock
	ldr	x8, [x26]
	ldr	w9, [x8, #96]
	cmp	w9, w25
	b.ls	.LBB88_211
.LBB88_194:                             // =>This Inner Loop Header: Depth=1
	ldr	w9, [x19, #3128]
	add	w27, w25, #1            // =1
	cbz	w9, .LBB88_200
// %bb.195:                             //   in Loop: Header=BB88_194 Depth=1
	cbz	x8, .LBB88_197
// %bb.196:                             //   in Loop: Header=BB88_194 Depth=1
	add	x9, x8, #80             // =80
	b	.LBB88_198
.LBB88_197:                             //   in Loop: Header=BB88_194 Depth=1
	ldr	x9, [x19, #16]
	add	x9, x9, #72             // =72
.LBB88_198:                             //   in Loop: Header=BB88_194 Depth=1
	ldr	x8, [x8, #8]
	ldr	w10, [x19, #1184]
	ldr	w9, [x9]
	mov	x0, x19
	ldr	w8, [x8]
	add	w8, w8, w25
	lsl	w8, w8, w10
	add	w1, w8, w9
	bl	f2fs_target_device_index
	ldr	x8, [x19, #3136]
	smull	x9, w0, w28
	ldr	x8, [x8, x9]
	cbz	x8, .LBB88_193
// %bb.199:                             //   in Loop: Header=BB88_194 Depth=1
	ldr	x8, [x26]
.LBB88_200:                             //   in Loop: Header=BB88_194 Depth=1
	ldr	x8, [x8]
	and	x9, x25, #0xffffffff
	mul	x9, x9, x22
	ldr	x8, [x8, #120]
	ldr	w9, [x8, x9]
	ldr	w8, [x19, #1188]
	ubfx	w9, w9, #6, #10
	cmp	w9, w8
	b.eq	.LBB88_193
// %bb.201:                             //   in Loop: Header=BB88_194 Depth=1
	cbz	w9, .LBB88_193
// %bb.202:                             //   in Loop: Header=BB88_194 Depth=1
	cmp	w9, w8
	b.ls	.LBB88_192
// %bb.203:                             //   in Loop: Header=BB88_194 Depth=1
	bl	print_bdev_access_info
	ldr	x0, [x23, :lo12:f2fs_dclient]
	cbz	x0, .LBB88_206
// %bb.204:                             //   in Loop: Header=BB88_194 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB88_206
// %bb.205:                             //   in Loop: Header=BB88_194 Depth=1
	ldr	x0, [x23, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.init_dirty_segmap
	mov	w3, #6910
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.init_dirty_segmap
	bl	dsm_client_record
	ldr	x0, [x23, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB88_206:                             //   in Loop: Header=BB88_194 Depth=1
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 6910; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB88_193
// %bb.207:                             //   in Loop: Header=BB88_194 Depth=1
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
	cbnz	w8, .LBB88_193
// %bb.208:                             //   in Loop: Header=BB88_194 Depth=1
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB88_193
// %bb.209:                             //   in Loop: Header=BB88_194 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB88_193
// %bb.210:                             //   in Loop: Header=BB88_194 Depth=1
	ldr	x2, [sp, #24]           // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB88_193
.LBB88_211:
	ldr	w9, [x19, #1192]
	cmp	w9, #2                  // =2
	b.lo	.LBB88_230
// %bb.212:
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	add	x20, x8, #80            // =80
	mov	x0, x20
	bl	mutex_lock
	ldr	x10, [x26]
	ldr	w8, [x10, #96]
	cbz	w8, .LBB88_229
// %bb.213:
	ldr	w9, [sp, #40]           // 4-byte Folded Reload
	ldr	w11, [sp, #32]          // 4-byte Folded Reload
	ldr	w13, [x19, #1192]
	mov	w8, wzr
	mov	w12, #40
	mul	w9, w11, w9
	mov	w11, #1
	b	.LBB88_215
.LBB88_214:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	w14, [x10, #96]
	add	w8, w13, w8
	cmp	w8, w14
	b.hs	.LBB88_229
.LBB88_215:                             // =>This Inner Loop Header: Depth=1
	ldr	x14, [x10]
	cmp	w13, #1                 // =1
	b.ls	.LBB88_217
// %bb.216:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	x15, [x14, #128]
	udiv	w14, w8, w13
	ldr	w15, [x15, w14, uxtw #2]
	b	.LBB88_218
.LBB88_217:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	x14, [x14, #120]
	umull	x15, w8, w12
	ldr	w14, [x14, x15]
	ubfx	w15, w14, #6, #10
	udiv	w14, w8, w13
.LBB88_218:                             //   in Loop: Header=BB88_215 Depth=1
	cbz	w15, .LBB88_214
// %bb.219:                             //   in Loop: Header=BB88_215 Depth=1
	cmp	w15, w9
	b.eq	.LBB88_214
// %bb.220:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	x15, [x10, #24]
	ldr	w16, [x15, #100]
	udiv	w16, w16, w13
	cmp	w14, w16
	b.eq	.LBB88_214
// %bb.221:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	w16, [x15, #220]
	udiv	w16, w16, w13
	cmp	w14, w16
	b.eq	.LBB88_214
// %bb.222:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	w16, [x15, #340]
	udiv	w16, w16, w13
	cmp	w14, w16
	b.eq	.LBB88_214
// %bb.223:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	w16, [x15, #460]
	udiv	w16, w16, w13
	cmp	w14, w16
	b.eq	.LBB88_214
// %bb.224:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	w16, [x15, #580]
	udiv	w16, w16, w13
	cmp	w14, w16
	b.eq	.LBB88_214
// %bb.225:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	w16, [x15, #700]
	udiv	w16, w16, w13
	cmp	w14, w16
	b.eq	.LBB88_214
// %bb.226:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	w16, [x15, #820]
	udiv	w16, w16, w13
	cmp	w14, w16
	b.eq	.LBB88_214
// %bb.227:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	w15, [x15, #940]
	udiv	w15, w15, w13
	cmp	w14, w15
	b.eq	.LBB88_214
// %bb.228:                             //   in Loop: Header=BB88_215 Depth=1
	ldr	x10, [sp, #48]          // 8-byte Folded Reload
	lsr	w13, w14, #6
	lsl	x14, x11, x14
	ldr	x10, [x10, #72]
	add	x10, x10, w13, uxtw #3
	//APP
	// atomic64_or
	prfm	pstl1strm, [x10]
1:	ldxr	x13, [x10]
	orr	x13, x13, x14
	stxr	w15, x13, [x10]
	cbnz	w15, 1b
	//NO_APP
	ldr	w13, [x19, #1192]
	ldr	x10, [x19, #136]
	b	.LBB88_214
.LBB88_229:
	mov	x0, x20
	bl	mutex_unlock
	ldr	x8, [x26]
.LBB88_230:
	ldr	w9, [x19, #1200]
	ldr	x20, [x8, #16]
	mov	w1, #3520
	mov	w2, #-1
	add	x8, x9, #63             // =63
	lsr	x8, x8, #3
	and	x0, x8, #0x3ffffff8
	bl	kvmalloc_node
	str	x0, [x20, #144]
	cbz	x0, .LBB88_232
// %bb.231:
	mov	x20, xzr
	add	x8, x19, #3408          // =3408
	mov	w25, #120
	str	x8, [sp, #48]           // 8-byte Folded Spill
	b	.LBB88_237
.LBB88_232:
	mov	w22, #-12
.LBB88_233:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB88_235
// %bb.234:
	mov	w0, w22
	ldp	x20, x19, [sp, #224]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #208]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #192]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #176]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #160]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #144]    // 16-byte Folded Reload
	add	sp, sp, #240            // =240
	ret
.LBB88_235:
	bl	__stack_chk_fail
.LBB88_236:                             //   in Loop: Header=BB88_237 Depth=1
	add	x20, x20, #1            // =1
	cmp	x20, #6                 // =6
	b.eq	.LBB88_255
.LBB88_237:                             // =>This Loop Header: Depth=1
                                        //     Child Loop BB88_244 Depth 2
	ldr	x8, [x19, #136]
	ldr	x23, [x8, #24]
	madd	x27, x20, x25, x23
	ldr	w21, [x27, #100]!
	ldr	x8, [x8]
	mov	x22, x27
	ldr	x28, [x8, #120]
	ldrh	w24, [x22, #4]!
	ldr	x8, [x19, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #6, .LBB88_240
// %bb.238:                             //   in Loop: Header=BB88_237 Depth=1
	cmp	w20, #3                 // =3
	b.eq	.LBB88_240
// %bb.239:                             //   in Loop: Header=BB88_237 Depth=1
	cbnz	w20, .LBB88_236
.LBB88_240:                             //   in Loop: Header=BB88_237 Depth=1
	madd	x8, x20, x25, x23
	ldrh	w8, [x8, #98]
	cmp	w8, #6                  // =6
	b.hs	.LBB88_246
.LBB88_241:                             //   in Loop: Header=BB88_237 Depth=1
	mov	w8, #40
	madd	x8, x21, x8, x28
	ldr	x8, [x8, #8]
	lsr	x9, x24, #3
	madd	x11, x20, x25, x23
	ldrb	w4, [x11, #96]
	ldrb	w9, [x8, x9]
	mvn	w10, w24
	and	w10, w10, #0x7
	lsr	w9, w9, w10
	tbnz	w9, #0, .LBB88_254
// %bb.242:                             //   in Loop: Header=BB88_237 Depth=1
	cmp	w4, #1                  // =1
	b.eq	.LBB88_236
// %bb.243:                             //   in Loop: Header=BB88_237 Depth=1
	ldr	w9, [x19, #1188]
.LBB88_244:                             //   Parent Loop BB88_237 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	add	w24, w24, #1            // =1
	cmp	w24, w9
	b.hs	.LBB88_236
// %bb.245:                             //   in Loop: Header=BB88_244 Depth=2
	mov	w10, w24
	lsr	x10, x10, #3
	ldrb	w10, [x8, x10]
	mvn	w11, w24
	and	w11, w11, #0x7
	lsr	w10, w10, w11
	tbz	w10, #0, .LBB88_244
	b	.LBB88_254
.LBB88_246:                             //   in Loop: Header=BB88_237 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB88_249
// %bb.247:                             //   in Loop: Header=BB88_237 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB88_249
// %bb.248:                             //   in Loop: Header=BB88_237 Depth=1
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.sanity_check_seg_type
	mov	w3, #46
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.sanity_check_seg_type
	bl	dsm_client_record
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB88_249:                             //   in Loop: Header=BB88_237 Depth=1
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.h"; .popsection; .long 14472b - 14470b; .short 46; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB88_241
// %bb.250:                             //   in Loop: Header=BB88_237 Depth=1
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
	cbnz	w8, .LBB88_241
// %bb.251:                             //   in Loop: Header=BB88_237 Depth=1
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB88_241
// %bb.252:                             //   in Loop: Header=BB88_237 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB88_241
// %bb.253:                             //   in Loop: Header=BB88_237 Depth=1
	ldr	x2, [sp, #48]           // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB88_241
.LBB88_254:
	ldr	w3, [x27]
	ldrh	w5, [x22]
	adrp	x1, .L.str.44
	add	x1, x1, :lo12:.L.str.44
	mov	x0, x19
	mov	w2, w20
	mov	w6, w24
	bl	f2fs_printk
	mov	w22, #-117
	b	.LBB88_233
.LBB88_255:
	ldr	x8, [x26]
	ldr	x21, [x8]
	add	x20, x21, #72           // =72
	mov	x0, x20
	bl	down_write
	mov	x8, #-1
	str	x8, [x21, #152]
	ldr	x8, [x26]
	ldr	w9, [x8, #96]
	cbz	w9, .LBB88_264
// %bb.256:
	ldr	w10, [x19, #1192]
	mov	w9, wzr
	mov	x12, #-1
	mov	w11, #40
	b	.LBB88_258
.LBB88_257:                             //   in Loop: Header=BB88_258 Depth=1
	ldr	w13, [x8, #96]
	add	w9, w10, w9
	cmp	w9, w13
	b.hs	.LBB88_264
.LBB88_258:                             // =>This Loop Header: Depth=1
                                        //     Child Loop BB88_260 Depth 2
	cbz	w10, .LBB88_261
// %bb.259:                             //   in Loop: Header=BB88_258 Depth=1
	ldr	x14, [x8]
	mov	x13, xzr
	mov	w16, w9
	ldr	x15, [x14, #120]
	mov	w14, w10
	mov	x17, x14
.LBB88_260:                             //   Parent Loop BB88_258 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	umaddl	x0, w16, w11, x15
	ldr	x0, [x0, #32]
	subs	x17, x17, #1            // =1
	add	w16, w16, #1            // =1
	add	x13, x0, x13
	b.ne	.LBB88_260
	b	.LBB88_262
.LBB88_261:                             //   in Loop: Header=BB88_258 Depth=1
	mov	x14, xzr
	mov	x13, xzr
.LBB88_262:                             //   in Loop: Header=BB88_258 Depth=1
	udiv	x13, x13, x14
	cmp	x12, x13
	b.ls	.LBB88_257
// %bb.263:                             //   in Loop: Header=BB88_258 Depth=1
	str	x13, [x21, #152]
	ldr	w10, [x19, #1192]
	ldr	x8, [x19, #136]
	mov	x12, x13
	b	.LBB88_257
.LBB88_264:
	ldr	x19, [x8]
	mov	w0, #1
	bl	ktime_get_coarse_with_offset
	ldr	x8, [x19, #144]
	mov	w9, #51712
	movk	w9, #15258, lsl #16
	sdiv	x9, x0, x9
	subs	x10, x8, x9
	b.ls	.LBB88_266
// %bb.265:
	ldr	x8, [x19, #136]
	subs	x8, x8, x10
	csel	x8, xzr, x8, lo
	b	.LBB88_267
.LBB88_266:
	ldr	x10, [x19, #136]
	sub	x8, x9, x8
	add	x8, x8, x10
.LBB88_267:
	mov	x0, x20
	str	x8, [x21, #160]
	str	xzr, [x21, #176]
	bl	up_write
	mov	w22, wzr
	b	.LBB88_233
.Lfunc_end88:
	.size	f2fs_build_segment_manager, .Lfunc_end88-f2fs_build_segment_manager
                                        // -- End function
	.globl	f2fs_destroy_segment_manager // -- Begin function f2fs_destroy_segment_manager
	.p2align	2
	.type	f2fs_destroy_segment_manager,@function
f2fs_destroy_segment_manager:           // @f2fs_destroy_segment_manager
.Lf2fs_destroy_segment_manager$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x19, [x0, #136]
	mov	x29, sp
	cbz	x19, .LBB89_22
// %bb.1:
	ldr	x21, [x19, #160]
	mov	x20, x0
	cbz	x21, .LBB89_4
// %bb.2:
	ldr	x0, [x21]
	cbz	x0, .LBB89_4
// %bb.3:
	str	xzr, [x21]
	bl	kthread_stop
.LBB89_4:
	mov	x0, x21
	bl	kfree
	ldr	x8, [x20, #136]
	str	xzr, [x8, #160]
	ldr	x8, [x20, #136]
	ldr	x21, [x8, #168]
	cbz	x21, .LBB89_9
// %bb.5:
	ldr	x0, [x21]
	cbz	x0, .LBB89_7
// %bb.6:
	str	xzr, [x21]
	bl	kthread_stop
.LBB89_7:
	ldr	w8, [x21, #8340]
	cbnz	w8, .LBB89_23
.LBB89_8:
	mov	x0, x21
	bl	kfree
	ldr	x8, [x20, #136]
	str	xzr, [x8, #168]
	ldr	x8, [x20, #136]
.LBB89_9:
	ldr	x21, [x8, #16]
	cbz	x21, .LBB89_13
// %bb.10:
	add	x22, x21, #80           // =80
	mov	x0, x22
	bl	mutex_lock
	ldr	x0, [x21, #8]
	bl	kvfree
	mov	x0, x22
	str	wzr, [x21, #112]
	bl	mutex_unlock
	ldr	x8, [x20, #136]
	ldr	x24, [x8, #16]
	add	x23, x24, #80           // =80
	mov	x0, x23
	bl	mutex_lock
	ldr	x0, [x24, #16]
	bl	kvfree
	mov	x0, x23
	str	wzr, [x24, #116]
	bl	mutex_unlock
	ldr	x8, [x20, #136]
	ldr	x24, [x8, #16]
	add	x23, x24, #80           // =80
	mov	x0, x23
	bl	mutex_lock
	ldr	x0, [x24, #24]
	bl	kvfree
	mov	x0, x23
	str	wzr, [x24, #120]
	bl	mutex_unlock
	ldr	x8, [x20, #136]
	ldr	x24, [x8, #16]
	add	x23, x24, #80           // =80
	mov	x0, x23
	bl	mutex_lock
	ldr	x0, [x24, #32]
	bl	kvfree
	mov	x0, x23
	str	wzr, [x24, #124]
	bl	mutex_unlock
	ldr	x8, [x20, #136]
	ldr	x24, [x8, #16]
	add	x23, x24, #80           // =80
	mov	x0, x23
	bl	mutex_lock
	ldr	x0, [x24, #40]
	bl	kvfree
	mov	x0, x23
	str	wzr, [x24, #128]
	bl	mutex_unlock
	ldr	x8, [x20, #136]
	ldr	x24, [x8, #16]
	add	x23, x24, #80           // =80
	mov	x0, x23
	bl	mutex_lock
	ldr	x0, [x24, #48]
	bl	kvfree
	mov	x0, x23
	str	wzr, [x24, #132]
	bl	mutex_unlock
	ldr	x8, [x20, #136]
	ldr	x24, [x8, #16]
	add	x23, x24, #80           // =80
	mov	x0, x23
	bl	mutex_lock
	ldr	x0, [x24, #56]
	bl	kvfree
	mov	x0, x23
	str	wzr, [x24, #136]
	bl	mutex_unlock
	ldr	x8, [x20, #136]
	ldr	x24, [x8, #16]
	add	x23, x24, #80           // =80
	mov	x0, x23
	bl	mutex_lock
	ldr	x0, [x24, #64]
	bl	kvfree
	mov	x0, x23
	str	wzr, [x24, #140]
	bl	mutex_unlock
	ldr	w8, [x20, #1192]
	cmp	w8, #2                  // =2
	b.lo	.LBB89_12
// %bb.11:
	mov	x0, x22
	bl	mutex_lock
	ldr	x0, [x21, #72]
	bl	kvfree
	mov	x0, x22
	bl	mutex_unlock
.LBB89_12:
	ldr	x8, [x20, #136]
	ldr	x8, [x8, #16]
	ldr	x0, [x8, #144]
	bl	kvfree
	ldr	x8, [x20, #136]
	mov	x0, x21
	str	xzr, [x8, #16]
	bl	kfree
	ldr	x8, [x20, #136]
.LBB89_13:
	ldr	x21, [x8, #24]
	cbz	x21, .LBB89_15
// %bb.14:
	str	xzr, [x8, #24]
	ldr	x0, [x21, #32]
	bl	kfree
	ldr	x0, [x21, #88]
	bl	kfree
	ldr	x0, [x21, #152]
	bl	kfree
	ldr	x0, [x21, #208]
	bl	kfree
	ldr	x0, [x21, #272]
	bl	kfree
	ldr	x0, [x21, #328]
	bl	kfree
	ldr	x0, [x21, #392]
	bl	kfree
	ldr	x0, [x21, #448]
	bl	kfree
	ldr	x0, [x21, #512]
	bl	kfree
	ldr	x0, [x21, #568]
	bl	kfree
	ldr	x0, [x21, #632]
	bl	kfree
	ldr	x0, [x21, #688]
	bl	kfree
	ldr	x0, [x21, #752]
	bl	kfree
	ldr	x0, [x21, #808]
	bl	kfree
	ldr	x0, [x21, #872]
	bl	kfree
	ldr	x0, [x21, #928]
	bl	kfree
	mov	x0, x21
	bl	kfree
	ldr	x8, [x20, #136]
.LBB89_15:
	ldr	x21, [x8, #8]
	cbz	x21, .LBB89_17
// %bb.16:
	str	xzr, [x8, #8]
	ldr	x0, [x21, #16]
	bl	kvfree
	ldr	x0, [x21, #24]
	bl	kvfree
	mov	x0, x21
	bl	kfree
	ldr	x8, [x20, #136]
.LBB89_17:
	ldr	x21, [x8]
	cbz	x21, .LBB89_21
// %bb.18:
	ldr	x8, [x21, #120]
	cbz	x8, .LBB89_20
// %bb.19:
	ldr	x0, [x21, #24]
	bl	kvfree
.LBB89_20:
	ldr	x0, [x21, #48]
	bl	kfree
	ldr	x0, [x21, #120]
	bl	kvfree
	ldr	x0, [x21, #128]
	bl	kvfree
	ldr	x0, [x21, #56]
	bl	kvfree
	ldr	x8, [x20, #136]
	str	xzr, [x8]
	ldr	x0, [x21, #32]
	bl	kvfree
	mov	x0, x21
	bl	kfree
.LBB89_21:
	mov	x0, x19
	str	xzr, [x20, #136]
	bl	kfree
.LBB89_22:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB89_23:
	mov	x0, x20
	bl	f2fs_issue_discard_timeout
	b	.LBB89_8
.Lfunc_end89:
	.size	f2fs_destroy_segment_manager, .Lfunc_end89-f2fs_destroy_segment_manager
                                        // -- End function
	.section	.init.text,"ax",@progbits
	.globl	f2fs_create_segment_manager_caches // -- Begin function f2fs_create_segment_manager_caches
	.p2align	2
	.type	f2fs_create_segment_manager_caches,@function
f2fs_create_segment_manager_caches:     // @f2fs_create_segment_manager_caches
.Lf2fs_create_segment_manager_caches$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	adrp	x0, .L.str.10
	add	x0, x0, :lo12:.L.str.10
	mov	w1, #88
	mov	w3, #131072
	mov	w2, wzr
	mov	x4, xzr
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	bl	kmem_cache_create
	adrp	x19, discard_entry_slab
	str	x0, [x19, :lo12:discard_entry_slab]
	cbz	x0, .LBB90_9
// %bb.1:
	adrp	x0, .L.str.11
	add	x0, x0, :lo12:.L.str.11
	mov	w1, #120
	mov	w3, #131072
	mov	w2, wzr
	mov	x4, xzr
	bl	kmem_cache_create
	adrp	x20, discard_cmd_slab
	str	x0, [x20, :lo12:discard_cmd_slab]
	cbz	x0, .LBB90_8
// %bb.2:
	adrp	x0, .L.str.12
	add	x0, x0, :lo12:.L.str.12
	mov	w1, #24
	mov	w3, #131072
	mov	w2, wzr
	mov	x4, xzr
	bl	kmem_cache_create
	adrp	x21, sit_entry_set_slab
	str	x0, [x21, :lo12:sit_entry_set_slab]
	cbz	x0, .LBB90_7
// %bb.3:
	adrp	x0, .L.str.13
	add	x0, x0, :lo12:.L.str.13
	mov	w1, #32
	mov	w3, #131072
	mov	w2, wzr
	mov	x4, xzr
	bl	kmem_cache_create
	adrp	x22, inmem_entry_slab
	str	x0, [x22, :lo12:inmem_entry_slab]
	cbz	x0, .LBB90_6
// %bb.4:
	bl	create_page_info_slab
	cbz	w0, .LBB90_10
// %bb.5:
	ldr	x0, [x22, :lo12:inmem_entry_slab]
	bl	kmem_cache_destroy
.LBB90_6:
	ldr	x0, [x21, :lo12:sit_entry_set_slab]
	bl	kmem_cache_destroy
.LBB90_7:
	ldr	x0, [x20, :lo12:discard_cmd_slab]
	bl	kmem_cache_destroy
.LBB90_8:
	ldr	x0, [x19, :lo12:discard_entry_slab]
	bl	kmem_cache_destroy
.LBB90_9:
	mov	w0, #-12
.LBB90_10:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end90:
	.size	f2fs_create_segment_manager_caches, .Lfunc_end90-f2fs_create_segment_manager_caches
                                        // -- End function
	.text
	.globl	f2fs_destroy_segment_manager_caches // -- Begin function f2fs_destroy_segment_manager_caches
	.p2align	2
	.type	f2fs_destroy_segment_manager_caches,@function
f2fs_destroy_segment_manager_caches:    // @f2fs_destroy_segment_manager_caches
.Lf2fs_destroy_segment_manager_caches$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x8, sit_entry_set_slab
	ldr	x0, [x8, :lo12:sit_entry_set_slab]
	mov	x29, sp
	bl	kmem_cache_destroy
	adrp	x8, discard_cmd_slab
	ldr	x0, [x8, :lo12:discard_cmd_slab]
	bl	kmem_cache_destroy
	adrp	x8, discard_entry_slab
	ldr	x0, [x8, :lo12:discard_entry_slab]
	bl	kmem_cache_destroy
	adrp	x8, inmem_entry_slab
	ldr	x0, [x8, :lo12:inmem_entry_slab]
	bl	kmem_cache_destroy
	bl	destroy_page_info_slab
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end91:
	.size	f2fs_destroy_segment_manager_caches, .Lfunc_end91-f2fs_destroy_segment_manager_caches
                                        // -- End function
	.p2align	2               // -- Begin function __remove_discard_cmd
	.type	__remove_discard_cmd,@function
__remove_discard_cmd:                   // @__remove_discard_cmd
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x26, x25, [sp, #16]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	adrp	x9, __tracepoint_f2fs_remove_discard+8
	ldr	x21, [x1, #88]
	ldp	w23, w22, [x1, #28]
	ldr	x24, [x8, #168]
	ldr	w8, [x9, :lo12:__tracepoint_f2fs_remove_discard+8]
	mov	x19, x1
	mov	x20, x0
	mov	x29, sp
	cmp	w8, #1                  // =1
	b.lt	.LBB92_6
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
	tbz	w8, #0, .LBB92_6
// %bb.2:
	//APP
	mrs x25, sp_el0
	//NO_APP
	ldr	w8, [x25, #24]
	add	w8, w8, #1              // =1
	str	w8, [x25, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_remove_discard+32
	ldr	x26, [x8, :lo12:__tracepoint_f2fs_remove_discard+32]
	cbz	x26, .LBB92_4
.LBB92_3:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x26]
	mov	x1, x21
	mov	w2, w22
	mov	w3, w23
	blr	x8
	ldr	x8, [x26, #24]!
	cbnz	x8, .LBB92_3
.LBB92_4:
	//APP
	//NO_APP
	ldr	x8, [x25, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x25, #24]
	b.eq	.LBB92_31
// %bb.5:
	ldr	x8, [x25, #24]
	cbz	x8, .LBB92_31
.LBB92_6:
	add	x21, x19, #112          // =112
	mov	x0, x21
	bl	_raw_spin_lock_irqsave
	ldrh	w22, [x19, #116]
	mov	x1, x0
	mov	x0, x21
	bl	_raw_spin_unlock_irqrestore
	cbnz	w22, .LBB92_29
// %bb.7:
	ldrh	w8, [x19, #96]
	cbnz	w8, .LBB92_32
.LBB92_8:
	ldrb	w8, [x19, #98]
	cmp	w8, #3                  // =3
	b.ne	.LBB92_18
// %bb.9:
	ldr	w8, [x19, #100]
	cbnz	w8, .LBB92_18
// %bb.10:
	ldr	x8, [x19, #104]
	cbz	x8, .LBB92_18
// %bb.11:
	add	x21, x20, #2296         // =2296
	mov	x0, x21
	bl	mutex_lock
	ldr	x8, [x20, #2328]
	cbz	x8, .LBB92_17
// %bb.12:
	ldr	w9, [x19, #28]
	ldr	w10, [x8, #52]
	add	w9, w10, w9
	str	w9, [x8, #52]
	ldr	x8, [x20, #2328]
	cbz	x8, .LBB92_17
// %bb.13:
	ldr	w9, [x8, #48]
	add	w9, w9, #1              // =1
	str	w9, [x8, #48]
	ldr	x8, [x20, #2328]
	cbz	x8, .LBB92_17
// %bb.14:
	ldr	x9, [x19, #104]
	ldr	x10, [x8, #64]
	add	x9, x10, x9
	str	x9, [x8, #64]
	ldr	x8, [x20, #2328]
	cbz	x8, .LBB92_17
// %bb.15:
	ldr	x10, [x8, #72]
	ldr	x9, [x19, #104]
	cmp	x10, x9
	b.hs	.LBB92_17
// %bb.16:
	str	x9, [x8, #72]
.LBB92_17:
	mov	x0, x21
	bl	mutex_unlock
.LBB92_18:
	ldr	w8, [x19, #100]
	cbz	w8, .LBB92_22
// %bb.19:
	cmn	w8, #95                 // =95
	b.ne	.LBB92_21
// %bb.20:
	str	wzr, [x19, #100]
	b	.LBB92_22
.LBB92_21:
	adrp	x0, __remove_discard_cmd._rs
	adrp	x1, .L__func__.__remove_discard_cmd
	add	x0, x0, :lo12:__remove_discard_cmd._rs
	add	x1, x1, :lo12:.L__func__.__remove_discard_cmd
	bl	___ratelimit
	cbnz	w0, .LBB92_30
.LBB92_22:
	ldrb	w8, [x19, #98]
	cmp	w8, #3                  // =3
	b.ne	.LBB92_24
// %bb.23:
	mov	w9, #8336
	ldrb	w8, [x19, #99]
	add	x9, x24, x9
	//APP
	// atomic_sub
	prfm	pstl1strm, [x9]
1:	ldxr	w10, [x9]
	sub	w10, w10, w8
	stxr	w11, w10, [x9]
	cbnz	w11, 1b

	//NO_APP
.LBB92_24:
	add	x0, x19, #40            // =40
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB92_26
// %bb.25:
	ldp	x9, x8, [x19, #40]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB92_26:
	mov	x8, #256
	movk	x8, #57005, lsl #48
	mov	w9, #34
	orr	x9, x8, x9
	stp	x8, x9, [x19, #40]
	ldr	x8, [x24, #8352]
	cmp	x8, x19
	b.ne	.LBB92_28
// %bb.27:
	mov	x0, x19
	bl	rb_next
	str	x0, [x24, #8352]
.LBB92_28:
	mov	w8, #8344
	add	x1, x24, x8
	mov	x0, x19
	bl	rb_erase
	ldr	w8, [x19, #28]
	ldr	w9, [x24, #8324]
	mov	x1, x19
	sub	w8, w9, w8
	str	w8, [x24, #8324]
	adrp	x8, discard_cmd_slab
	ldr	x0, [x8, :lo12:discard_cmd_slab]
	bl	kmem_cache_free
	mov	w8, #8340
	add	x8, x24, x8
	mov	w9, #1
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
.LBB92_29:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB92_30:
	ldr	x8, [x20]
	ldp	w5, w4, [x19, #28]
	ldr	w3, [x19, #24]
	ldr	w6, [x19, #100]
	adrp	x0, .L.str.16
	adrp	x1, .L.str.17
	add	x2, x8, #1056           // =1056
	add	x0, x0, :lo12:.L.str.16
	add	x1, x1, :lo12:.L.str.17
	bl	printk
	b	.LBB92_22
.LBB92_31:
	bl	preempt_schedule_notrace
	b	.LBB92_6
.LBB92_32:
	bl	print_bdev_access_info
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	cbz	x0, .LBB92_35
// %bb.33:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB92_35
// %bb.34:
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__remove_discard_cmd
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__remove_discard_cmd
	mov	w3, #1665
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB92_35:
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 1665; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB92_8
// %bb.36:
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
	cbnz	w8, .LBB92_8
// %bb.37:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB92_8
// %bb.38:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB92_8
// %bb.39:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB92_8
.Lfunc_end92:
	.size	__remove_discard_cmd, .Lfunc_end92-__remove_discard_cmd
                                        // -- End function
	.p2align	2               // -- Begin function __submit_discard_cmd
	.type	__submit_discard_cmd,@function
__submit_discard_cmd:                   // @__submit_discard_cmd
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
	ldr	x23, [x3, #88]
	ldr	x8, [x0, #136]
	ldr	w10, [x1]
	ldrb	w21, [x1, #16]
	ldr	x9, [x23, #128]
	ldr	x12, [x8, #168]
	mov	w11, #8232
	cmp	w10, #3                 // =3
	ldr	x9, [x9, #1264]
	add	x10, x12, x11
	mov	x22, x4
	mov	x19, x3
	ldr	w26, [x9, #2944]
	mov	w9, #8216
	add	x9, x12, x9
	mov	x20, x0
	csel	x9, x10, x9, eq
	stur	x9, [x29, #-24]         // 8-byte Folded Spill
	b.eq	.LBB93_2
// %bb.1:
	add	w9, w2, #1              // =1
	cmp	w9, #512                // =512
	b.lt	.LBB93_32
.LBB93_2:
	add	x11, x1, #20            // =20
	ldrb	w9, [x19, #98]
	cbnz	w9, .LBB93_36
.LBB93_3:
	ldr	x9, [x20, #80]
	tbnz	w9, #2, .LBB93_36
// %bb.4:
	ldrb	w9, [x20, #3456]
	str	x12, [sp, #16]          // 8-byte Folded Spill
	str	x20, [sp, #56]          // 8-byte Folded Spill
	cbnz	w9, .LBB93_42
.LBB93_5:
	adrp	x8, __tracepoint_f2fs_issue_discard+8
	ldp	w24, w25, [x19, #28]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_issue_discard+8]
	stur	x11, [x29, #-40]        // 8-byte Folded Spill
	cmp	w8, #1                  // =1
	b.lt	.LBB93_12
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
	tbz	w8, #0, .LBB93_11
// %bb.7:
	//APP
	mrs x27, sp_el0
	//NO_APP
	ldr	w8, [x27, #24]
	mov	x28, x22
	add	w8, w8, #1              // =1
	str	w8, [x27, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_issue_discard+32
	ldr	x22, [x8, :lo12:__tracepoint_f2fs_issue_discard+32]
	cbz	x22, .LBB93_9
.LBB93_8:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x22]
	mov	x1, x23
	mov	w2, w25
	mov	w3, w24
	blr	x8
	ldr	x8, [x22, #24]!
	cbnz	x8, .LBB93_8
.LBB93_9:
	//APP
	//NO_APP
	ldr	x8, [x27, #24]
	mov	x22, x28
	subs	x8, x8, #1              // =1
	str	w8, [x27, #24]
	ldur	x11, [x29, #-40]        // 8-byte Folded Reload
	b.eq	.LBB93_44
// %bb.10:
	ldr	x8, [x27, #24]
	cbz	x8, .LBB93_44
.LBB93_11:
	ldr	w24, [x19, #28]
.LBB93_12:
	ldr	w8, [x19, #24]
	ldr	w25, [x19, #32]
	str	wzr, [x19, #28]
	cbz	w24, .LBB93_36
// %bb.13:
	stur	w8, [x29, #-28]         // 4-byte Folded Spill
	lsr	w8, w26, #3
	str	w8, [sp, #52]           // 4-byte Folded Spill
	lsl	w8, w21, #11
	ldr	x10, [sp, #16]          // 8-byte Folded Reload
	str	w8, [sp, #48]           // 4-byte Folded Spill
	add	x8, x20, #3408          // =3408
	str	x8, [sp, #8]            // 8-byte Folded Spill
	add	x8, x19, #112           // =112
	stur	x8, [x29, #-48]         // 8-byte Folded Spill
	mov	w8, #8332
	add	x9, x20, #2344          // =2344
	str	x9, [sp, #24]           // 8-byte Folded Spill
	mov	w9, #8336
	add	x8, x10, x8
	add	x28, x19, #40           // =40
	str	x8, [sp, #40]           // 8-byte Folded Spill
	add	x8, x10, x9
	str	x8, [sp, #32]           // 8-byte Folded Spill
	b	.LBB93_15
.LBB93_14:                              //   in Loop: Header=BB93_15 Depth=1
	ldur	x11, [x29, #-40]        // 8-byte Folded Reload
	subs	w24, w24, w27
	add	w25, w27, w25
	b.eq	.LBB93_36
.LBB93_15:                              // =>This Inner Loop Header: Depth=1
	ldr	w8, [x22]
	ldr	w9, [x11]
	cmp	w8, w9
	b.hs	.LBB93_35
// %bb.16:                              //   in Loop: Header=BB93_15 Depth=1
	ldr	w10, [sp, #52]          // 4-byte Folded Reload
	add	w8, w8, #1              // =1
	stur	xzr, [x29, #-16]
	str	w8, [x22]
	cmp	w24, w10
	ldr	w9, [x11]
	csel	w27, w10, w24, hi
	ldr	w10, [x19, #28]
	mov	x20, x22
	cset	w22, ls
	cmp	w8, w9
	mov	w8, w25
	mov	x0, x23
	lsl	x1, x8, #3
	add	w8, w10, w27
	lsl	w2, w27, #3
	sub	x5, x29, #16            // =16
	mov	w3, #3136
	mov	w4, wzr
	cset	w26, eq
	str	w8, [x19, #28]
	bl	__blkdev_issue_discard
	cbnz	w0, .LBB93_39
// %bb.17:                              //   in Loop: Header=BB93_15 Depth=1
	ldur	x8, [x29, #-16]
	orr	w21, w26, w22
	mov	x22, x20
	ldr	x20, [sp, #56]          // 8-byte Folded Reload
	cbz	x8, .LBB93_24
.LBB93_18:                              //   in Loop: Header=BB93_15 Depth=1
	ldur	x26, [x29, #-48]        // 8-byte Folded Reload
	mov	x0, x26
	bl	_raw_spin_lock_irqsave
	ldrh	w8, [x19, #116]
	cmp	w21, #0                 // =0
	mov	w9, #1
	mov	x1, x0
	cinc	w9, w9, ne
	add	w8, w8, #1              // =1
	mov	x0, x26
	strb	w9, [x19, #98]
	strh	w8, [x19, #116]
	bl	_raw_spin_unlock_irqrestore
	bl	ktime_get
	str	x0, [x19, #104]
	ldr	x10, [sp, #32]          // 8-byte Folded Reload
	//APP
	// atomic_add
	prfm	pstl1strm, [x10]
1:	ldxr	w8, [x10]
	add	w8, w8, 1
	stxr	w9, w8, [x10]
	cbnz	w9, 1b

	//NO_APP
	ldrb	w8, [x19, #99]
	mov	x0, x28
	add	w8, w8, #1              // =1
	strb	w8, [x19, #99]
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB93_20
// %bb.19:                              //   in Loop: Header=BB93_15 Depth=1
	ldp	x9, x8, [x19, #40]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB93_20:                              //   in Loop: Header=BB93_15 Depth=1
	ldur	x2, [x29, #-24]         // 8-byte Folded Reload
	mov	x0, x28
	ldr	x21, [x2, #8]
	mov	x1, x21
	bl	__list_add_valid
	tbz	w0, #0, .LBB93_22
// %bb.21:                              //   in Loop: Header=BB93_15 Depth=1
	ldur	x8, [x29, #-24]         // 8-byte Folded Reload
	str	x28, [x8, #8]
	stp	x8, x21, [x19, #40]
	str	x28, [x21]
.LBB93_22:                              //   in Loop: Header=BB93_15 Depth=1
	ldur	w9, [x29, #-28]         // 4-byte Folded Reload
	ldur	x0, [x29, #-16]
	add	w9, w27, w9
	stur	w9, [x29, #-28]         // 4-byte Folded Spill
	adrp	x9, f2fs_submit_discard_endio
	add	x9, x9, :lo12:f2fs_submit_discard_endio
	stp	x9, x19, [x0, #64]
	ldr	w8, [x0, #16]
	ldr	w9, [sp, #48]           // 4-byte Folded Reload
	orr	w8, w8, w9
	str	w8, [x0, #16]
	bl	submit_bio
	ldr	x10, [sp, #40]          // 8-byte Folded Reload
	//APP
	// atomic_add
	prfm	pstl1strm, [x10]
1:	ldxr	w8, [x10]
	add	w8, w8, 1
	stxr	w9, w8, [x10]
	cbnz	w9, 1b

	//NO_APP
	ldrb	w8, [x20, #2704]
	cbz	w8, .LBB93_14
// %bb.23:                              //   in Loop: Header=BB93_15 Depth=1
	ldr	x21, [sp, #24]          // 8-byte Folded Reload
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	x8, [x20, #2520]
	mov	x0, x21
	add	x8, x8, #1              // =1
	str	x8, [x20, #2520]
	bl	_raw_spin_unlock
	mov	x0, x20
	bl	f2fs_record_iostat
	b	.LBB93_14
.LBB93_24:                              //   in Loop: Header=BB93_15 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB93_27
// %bb.25:                              //   in Loop: Header=BB93_15 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB93_27
// %bb.26:                              //   in Loop: Header=BB93_15 Depth=1
	adrp	x26, f2fs_dclient
	ldr	x0, [x26, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__submit_discard_cmd
	mov	w3, #1866
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__submit_discard_cmd
	bl	dsm_client_record
	ldr	x0, [x26, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB93_27:                              //   in Loop: Header=BB93_15 Depth=1
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 1866; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB93_18
// %bb.28:                              //   in Loop: Header=BB93_15 Depth=1
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
	cbnz	w8, .LBB93_18
// %bb.29:                              //   in Loop: Header=BB93_15 Depth=1
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB93_18
// %bb.30:                              //   in Loop: Header=BB93_15 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB93_18
// %bb.31:                              //   in Loop: Header=BB93_15 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB93_18
.LBB93_32:
	cmp	w9, #16                 // =16
	b.lt	.LBB93_34
// %bb.33:
	add	x11, x1, #28            // =28
	ldrb	w9, [x19, #98]
	cbnz	w9, .LBB93_36
	b	.LBB93_3
.LBB93_34:
	add	x11, x1, #36            // =36
	ldrb	w9, [x19, #98]
	cbnz	w9, .LBB93_36
	b	.LBB93_3
.LBB93_35:
	ldr	x9, [sp, #16]           // 8-byte Folded Reload
	ldur	w2, [x29, #-28]         // 4-byte Folded Reload
	mov	x0, x20
	mov	x1, x23
	ldr	w8, [x9, #8324]
	mov	w3, w25
	mov	w4, w24
	sub	w8, w8, w24
	str	w8, [x9, #8324]
	bl	__update_discard_tree_range
.LBB93_36:
	mov	w21, wzr
.LBB93_37:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB93_53
// %bb.38:
	mov	w0, w21
	ldp	x20, x19, [sp, #192]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #176]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #160]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #144]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #128]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #112]    // 16-byte Folded Reload
	add	sp, sp, #208            // =208
	ret
.LBB93_39:
	mov	w21, w0
	ldur	x0, [x29, #-48]         // 8-byte Folded Reload
	bl	_raw_spin_lock_irqsave
	ldrb	w8, [x19, #98]
	mov	x1, x0
	cmp	w8, #1                  // =1
	b.ne	.LBB93_41
// %bb.40:
	mov	w8, #2
	strb	w8, [x19, #98]
.LBB93_41:
	ldur	x0, [x29, #-48]         // 8-byte Folded Reload
	bl	_raw_spin_unlock_irqrestore
	b	.LBB93_37
.LBB93_42:
	ldr	w2, [x19, #32]
	mov	x27, x11
	mov	x25, x23
	mov	x24, x22
	cbz	x8, .LBB93_45
// %bb.43:
	add	x9, x8, #80             // =80
	add	x10, x8, #92            // =92
	b	.LBB93_46
.LBB93_44:
	bl	preempt_schedule_notrace
	ldur	x11, [x29, #-40]        // 8-byte Folded Reload
	b	.LBB93_11
.LBB93_45:
	ldr	x10, [x20, #16]
	add	x9, x10, #72            // =72
	add	x10, x10, #48           // =48
.LBB93_46:
	ldr	w11, [x10]
	ldr	w10, [x20, #1184]
	ldr	w9, [x9]
	ldr	w3, [x19, #28]
	lsl	w11, w11, w10
	add	w9, w11, w9
	cmp	w2, w9
	b.hs	.LBB93_49
// %bb.47:
	add	w11, w3, w2
	cbz	x8, .LBB93_50
// %bb.48:
	ldr	w12, [x8, #92]
	ldr	w9, [x8, #80]
	ldr	x20, [sp, #56]          // 8-byte Folded Reload
	mov	x22, x24
	lsl	w8, w12, w10
	add	w10, w8, w9
	cmp	w11, w10
	mov	x23, x25
	b	.LBB93_51
.LBB93_49:
	adrp	x1, .L.str.18
	add	x1, x1, :lo12:.L.str.18
	mov	x0, x20
	bl	f2fs_printk
	mov	x0, x20
	mov	x1, x19
	bl	__remove_discard_cmd
	b	.LBB93_36
.LBB93_50:
	ldr	x20, [sp, #56]          // 8-byte Folded Reload
	mov	x22, x24
	mov	x23, x25
	ldr	x8, [x20, #16]
	ldr	w12, [x8, #48]
	ldr	w9, [x8, #72]
	lsl	w8, w12, w10
	add	w10, w8, w9
	cmp	w11, w10
.LBB93_51:
	mov	x11, x27
	b.ls	.LBB93_5
// %bb.52:
	ldr	x20, [sp, #56]          // 8-byte Folded Reload
	sub	w9, w9, w2
	adrp	x1, .L.str.19
	add	w2, w9, w8
	add	x1, x1, :lo12:.L.str.19
	mov	x0, x20
	str	w2, [x19, #28]
	bl	f2fs_printk
	mov	x22, x24
	mov	x23, x25
	mov	x11, x27
	b	.LBB93_5
.LBB93_53:
	bl	__stack_chk_fail
.Lfunc_end93:
	.size	__submit_discard_cmd, .Lfunc_end93-__submit_discard_cmd
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_submit_discard_endio
	.type	f2fs_submit_discard_endio,@function
f2fs_submit_discard_endio:              // @f2fs_submit_discard_endio
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x22, [x0, #72]
	mov	x19, x0
	mov	x29, sp
	add	x20, x22, #112          // =112
	mov	x0, x20
	bl	_raw_spin_lock_irqsave
	ldr	w8, [x22, #100]
	mov	x21, x0
	cbnz	w8, .LBB94_2
// %bb.1:
	ldrb	w0, [x19, #26]
	bl	blk_status_to_errno
	str	w0, [x22, #100]
.LBB94_2:
	ldrh	w8, [x22, #116]
	sub	w8, w8, #1              // =1
	tst	w8, #0xffff
	strh	w8, [x22, #116]
	b.ne	.LBB94_7
// %bb.3:
	ldrb	w8, [x22, #98]
	cmp	w8, #2                  // =2
	b.ne	.LBB94_7
// %bb.4:
	ldr	x8, [x22, #104]
	mov	w9, #3
	strb	w9, [x22, #98]
	cbz	x8, .LBB94_6
// %bb.5:
	bl	ktime_get
	ldr	x8, [x22, #104]
	subs	x8, x0, x8
	csel	x8, xzr, x8, lo
	str	x8, [x22, #104]
.LBB94_6:
	add	x0, x22, #56            // =56
	bl	complete_all
.LBB94_7:
	mov	x0, x20
	mov	x1, x21
	bl	_raw_spin_unlock_irqrestore
	mov	x0, x19
	bl	bio_put
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end94:
	.size	f2fs_submit_discard_endio, .Lfunc_end94-f2fs_submit_discard_endio
                                        // -- End function
	.p2align	2               // -- Begin function __update_discard_tree_range
	.type	__update_discard_tree_range,@function
__update_discard_tree_range:            // @__update_discard_tree_range
// %bb.0:
	sub	sp, sp, #192            // =192
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #96]     // 16-byte Folded Spill
	add	x29, sp, #96            // =96
	stp	x28, x27, [sp, #112]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #128]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #144]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #160]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #176]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x8, [x0, #136]
	mov	w22, w4
	mov	w26, w3
	mov	x20, x1
	ldr	x9, [x8, #168]
	stp	xzr, xzr, [x29, #-24]
	stp	xzr, xzr, [x29, #-40]
	ldr	x8, [x1, #128]
	mov	x28, x0
	sub	x3, x29, #16            // =16
	sub	x4, x29, #24            // =24
	ldr	x8, [x8, #1264]
	sub	x5, x29, #32            // =32
	sub	x6, x29, #40            // =40
	mov	w7, #1
	ldr	w21, [x8, #2944]
	mov	w8, #8344
	add	x0, x9, x8
	mov	x1, xzr
	mov	w19, w2
	str	x9, [sp, #48]           // 8-byte Folded Spill
	str	xzr, [sp]
	bl	f2fs_lookup_rb_tree_ret
	cbz	x0, .LBB95_3
// %bb.1:
	mov	x25, x0
	stur	x0, [x29, #-16]
	add	w27, w22, w19
	cbz	x25, .LBB95_4
.LBB95_2:
	mov	w22, wzr
	mov	w23, wzr
	mov	w24, wzr
	b	.LBB95_7
.LBB95_3:
	ldur	x25, [x29, #-16]
	add	w27, w22, w19
	cbnz	x25, .LBB95_2
.LBB95_4:
	ldur	x9, [x29, #-24]
	mov	w8, w22
	cbz	x9, .LBB95_6
// %bb.5:
	ldr	w8, [x9, #24]
	sub	w8, w8, w19
.LBB95_6:
	cmp	w8, w22
	csel	w23, w8, w22, lo
	mov	w22, w19
	mov	w24, w26
.LBB95_7:
	lsr	w8, w21, #3
	sub	w21, w26, w19
	str	w8, [sp, #44]           // 4-byte Folded Spill
	cbz	x25, .LBB95_22
.LBB95_8:
	ldp	w8, w9, [x25, #24]
	add	w8, w9, w8
	cmp	w8, w19
	csel	w22, w19, w8, lo
	cmp	w27, w22
	b.ls	.LBB95_39
// %bb.9:
	ldur	x11, [x29, #-24]
	mov	w10, w27
	cbz	x11, .LBB95_11
// %bb.10:
	ldr	w10, [x11, #24]
	cmp	w10, w27
	csel	w10, w27, w10, hi
.LBB95_11:
	subs	w23, w10, w22
	add	w24, w21, w22
	b.eq	.LBB95_30
// %bb.12:
	ldrb	w10, [x25, #98]
	cbnz	w10, .LBB95_23
// %bb.13:
	ldr	x10, [x25, #88]
	cmp	x10, x20
	b.ne	.LBB95_23
// %bb.14:
	mov	x26, xzr
	cmp	w8, w19
	mov	w8, wzr
	b.lo	.LBB95_21
// %bb.15:
	ldr	w10, [sp, #44]          // 4-byte Folded Reload
	add	w9, w9, w23
	cmp	w9, w10
	b.hi	.LBB95_21
// %bb.16:
	ldr	x10, [sp, #48]          // 8-byte Folded Reload
	str	w9, [x25, #28]
	add	x22, x25, #40           // =40
	mov	x0, x22
	ldr	w8, [x10, #8324]
	add	w8, w8, w23
	str	w8, [x10, #8324]
	ldr	w8, [x25, #28]
	sub	w9, w8, #1              // =1
	cmp	w8, #511                // =511
	mov	w8, #511
	csel	w24, w8, w9, hi
	add	x8, x10, w24, uxtw #4
	add	x23, x8, #24            // =24
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB95_18
// %bb.17:
	ldp	x9, x8, [x25, #40]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB95_18:
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	mov	x0, x22
	mov	x2, x23
	add	x26, x8, x24, lsl #4
	ldr	x24, [x26, #32]!
	mov	x1, x24
	bl	__list_add_valid
	tbz	w0, #0, .LBB95_20
// %bb.19:
	str	x22, [x26]
	stp	x23, x24, [x25, #40]
	str	x22, [x24]
.LBB95_20:
	ldur	x26, [x29, #-16]
	mov	w8, #1
	ldp	w22, w23, [x26, #24]
	ldr	w24, [x26, #32]
.LBB95_21:
	ldur	x25, [x29, #-24]
	cbnz	x25, .LBB95_24
	b	.LBB95_28
.LBB95_22:
	cbz	w23, .LBB95_30
.LBB95_23:
	mov	w8, wzr
	mov	x26, xzr
	ldur	x25, [x29, #-24]
	cbz	x25, .LBB95_28
.LBB95_24:
	ldrb	w9, [x25, #98]
	cbnz	w9, .LBB95_28
// %bb.25:
	ldr	x9, [x25, #88]
	cmp	x9, x20
	b.ne	.LBB95_28
// %bb.26:
	ldr	w9, [x25, #24]
	add	w10, w23, w22
	cmp	w10, w9
	b.ne	.LBB95_28
// %bb.27:
	ldr	w9, [x25, #28]
	ldr	w10, [sp, #44]          // 4-byte Folded Reload
	add	w9, w9, w23
	cmp	w9, w10
	b.ls	.LBB95_32
.LBB95_28:
	tbnz	w8, #0, .LBB95_30
// %bb.29:
	mov	x0, x28
	mov	x1, x20
	mov	w2, w22
	mov	w3, w24
	mov	w4, w23
	bl	__insert_discard_tree
.LBB95_30:
	ldur	x0, [x29, #-24]
	stur	x0, [x29, #-16]
	cbz	x0, .LBB95_39
// %bb.31:
	bl	rb_next
	ldur	x25, [x29, #-16]
	stur	x0, [x29, #-24]
	cbnz	x25, .LBB95_8
	b	.LBB95_22
.LBB95_32:
	str	x28, [sp, #16]          // 8-byte Folded Spill
	stp	w22, w9, [x25, #24]
	ldr	x10, [sp, #48]          // 8-byte Folded Reload
	str	w24, [x25, #32]
	add	x0, x25, #40            // =40
	ldr	w8, [x10, #8324]
	add	w8, w8, w23
	str	w8, [x10, #8324]
	ldr	w8, [x25, #28]
	sub	w9, w8, #1              // =1
	cmp	w8, #511                // =511
	mov	w8, #511
	csel	w28, w8, w9, hi
	add	x8, x10, w28, uxtw #4
	add	x8, x8, #24             // =24
	stp	x8, x0, [sp, #24]       // 16-byte Folded Spill
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB95_34
// %bb.33:
	ldp	x9, x8, [x25, #40]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB95_34:
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	add	x28, x8, x28, lsl #4
	ldr	x1, [x28, #32]!
	ldp	x2, x0, [sp, #24]       // 16-byte Folded Reload
	str	x1, [sp, #8]            // 8-byte Folded Spill
	bl	__list_add_valid
	tbz	w0, #0, .LBB95_36
// %bb.35:
	ldp	x8, x9, [sp, #24]       // 16-byte Folded Reload
	str	x9, [x28]
	str	x8, [x25, #40]
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	str	x8, [x25, #48]
	str	x9, [x8]
.LBB95_36:
	cbz	x26, .LBB95_38
// %bb.37:
	ldr	x28, [sp, #16]          // 8-byte Folded Reload
	mov	x1, x26
	mov	x0, x28
	bl	__remove_discard_cmd
	b	.LBB95_30
.LBB95_38:
	ldr	x28, [sp, #16]          // 8-byte Folded Reload
	b	.LBB95_30
.LBB95_39:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB95_41
// %bb.40:
	ldp	x20, x19, [sp, #176]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #160]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #144]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #128]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #112]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #96]     // 16-byte Folded Reload
	add	sp, sp, #192            // =192
	ret
.LBB95_41:
	bl	__stack_chk_fail
.Lfunc_end95:
	.size	__update_discard_tree_range, .Lfunc_end95-__update_discard_tree_range
                                        // -- End function
	.p2align	2               // -- Begin function __insert_discard_tree
	.type	__insert_discard_tree,@function
__insert_discard_tree:                  // @__insert_discard_tree
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
	ldr	x8, [x0, #136]
	mov	w9, #1
	mov	w23, w2
	mov	w19, w4
	ldr	x8, [x8, #168]
	strb	w9, [sp, #12]
	mov	w9, #8344
	mov	w21, w3
	mov	x22, x1
	add	x1, x8, x9
	add	x2, sp, #16             // =16
	add	x4, sp, #12             // =12
	mov	w3, w23
	mov	x24, x0
	str	xzr, [sp, #16]
	bl	f2fs_lookup_rb_tree_for_insert
	ldr	x8, [x24, #136]
	ldr	x28, [sp, #16]
	ldrb	w9, [sp, #12]
	str	x0, [sp]                // 8-byte Folded Spill
	ldr	x27, [x8, #168]
	str	w9, [sp, #8]            // 4-byte Folded Spill
	cbz	w19, .LBB96_9
.LBB96_1:
	adrp	x10, discard_cmd_slab
	ldr	x26, [x10, :lo12:discard_cmd_slab]
	sub	w8, w19, #1             // =1
	cmp	w19, #511               // =511
	mov	w9, #511
	csel	w20, w9, w8, hi
	add	x8, x27, w20, uxtw #4
	mov	w1, #3136
	mov	x0, x26
	add	x25, x8, #24            // =24
	bl	kmem_cache_alloc
	mov	x24, x0
	cbnz	x0, .LBB96_3
// %bb.2:
	mov	w1, #35904
	mov	x0, x26
	bl	kmem_cache_alloc
	mov	x24, x0
.LBB96_3:
	adrp	x1, .L.str.15
	adrp	x2, __init_completion.__key
	add	x26, x24, #40           // =40
	add	x0, x24, #64            // =64
	add	x1, x1, :lo12:.L.str.15
	add	x2, x2, :lo12:__init_completion.__key
	str	x26, [x24, #40]
	str	x26, [x24, #48]
	stp	w19, w21, [x24, #28]
	str	w23, [x24, #24]
	stp	x22, xzr, [x24, #88]
	str	xzr, [x24, #104]
	str	wzr, [x24, #56]
	bl	__init_waitqueue_head
	add	x20, x27, x20, lsl #4
	ldr	x21, [x20, #32]!
	mov	x0, x26
	mov	x2, x25
	mov	x1, x21
	bl	__list_add_valid
	tbz	w0, #0, .LBB96_5
// %bb.4:
	str	x26, [x20]
	stp	x25, x21, [x24, #40]
	str	x26, [x21]
.LBB96_5:
	str	wzr, [x24, #112]
	strh	wzr, [x24, #116]
	mov	w8, #8340
	add	x8, x27, x8
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	w8, [x27, #8324]
	add	w8, w8, w19
	str	w8, [x27, #8324]
	ldr	x8, [sp]                // 8-byte Folded Reload
	stp	x28, xzr, [x24]
	str	xzr, [x24, #16]
	str	x24, [x8]
	ldr	w8, [sp, #8]            // 4-byte Folded Reload
	cbz	w8, .LBB96_7
// %bb.6:
	str	x24, [x27, #8352]
.LBB96_7:
	mov	w8, #8344
	add	x1, x27, x8
	mov	x0, x24
	bl	rb_insert_color
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB96_17
// %bb.8:
	ldp	x20, x19, [sp, #112]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #96]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #80]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #64]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #48]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #128            // =128
	ret
.LBB96_9:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB96_12
// %bb.10:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB96_12
// %bb.11:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__create_discard_cmd
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__create_discard_cmd
	mov	w3, #1593
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB96_12:
	mov	x0, x24
	bl	f2fs_print_raw_sb_info
	mov	x0, x24
	bl	f2fs_print_ckpt_info
	mov	x0, x24
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 1593; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x24, .LBB96_1
// %bb.13:
	add	x8, x24, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrb	w8, [x24, #3376]
	cbnz	w8, .LBB96_1
// %bb.14:
	ldr	x1, [x24, #3392]
	cbz	x1, .LBB96_1
// %bb.15:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB96_1
// %bb.16:
	add	x2, x24, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB96_1
.LBB96_17:
	bl	__stack_chk_fail
.Lfunc_end96:
	.size	__insert_discard_tree, .Lfunc_end96-__insert_discard_tree
                                        // -- End function
	.p2align	2               // -- Begin function __issue_discard_async
	.type	__issue_discard_async,@function
__issue_discard_async:                  // @__issue_discard_async
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x1, #128]
	mov	w19, w3
	mov	w20, w2
	mov	x22, x1
	ldr	x8, [x8, #1264]
	mov	x21, x0
	mov	x29, sp
	ldr	x9, [x8, #104]
	tbnz	w9, #8, .LBB97_3
// %bb.1:
	cbz	x8, .LBB97_12
// %bb.2:
	ldr	w8, [x8, #2980]
	sub	w8, w8, #1              // =1
	cmp	w8, #1                  // =1
	b.hi	.LBB97_12
.LBB97_3:
	adrp	x8, __tracepoint_f2fs_queue_discard+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_queue_discard+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB97_9
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
	tbz	w8, #0, .LBB97_9
// %bb.5:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w8, [x23, #24]
	add	w8, w8, #1              // =1
	str	w8, [x23, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_queue_discard+32
	ldr	x24, [x8, :lo12:__tracepoint_f2fs_queue_discard+32]
	cbz	x24, .LBB97_7
.LBB97_6:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x24]
	mov	x1, x22
	mov	w2, w20
	mov	w3, w19
	blr	x8
	ldr	x8, [x24, #24]!
	cbnz	x8, .LBB97_6
.LBB97_7:
	//APP
	//NO_APP
	ldr	x8, [x23, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x23, #24]
	b.eq	.LBB97_13
// %bb.8:
	ldr	x8, [x23, #24]
	cbz	x8, .LBB97_13
.LBB97_9:
	ldr	w8, [x21, #3128]
	mov	w23, w20
	cmp	w8, #1                  // =1
	b.le	.LBB97_11
// %bb.10:
	mov	x0, x21
	mov	w1, w20
	bl	f2fs_target_device_index
	ldr	x8, [x21, #3136]
	mov	w9, #88
	smaddl	x8, w0, w9, x8
	ldr	w8, [x8, #76]
	sub	w23, w20, w8
.LBB97_11:
	ldr	x8, [x21, #136]
	mov	w24, #8280
	ldr	x8, [x8, #168]
	add	x0, x8, x24
	bl	mutex_lock
	mov	x0, x21
	mov	x1, x22
	mov	w2, w20
	mov	w3, w23
	mov	w4, w19
	bl	__update_discard_tree_range
	ldr	x8, [x21, #136]
	ldr	x8, [x8, #168]
	add	x0, x8, x24
	bl	mutex_unlock
.LBB97_12:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB97_13:
	bl	preempt_schedule_notrace
	b	.LBB97_9
.Lfunc_end97:
	.size	__issue_discard_async, .Lfunc_end97-__issue_discard_async
                                        // -- End function
	.p2align	2               // -- Begin function __set_test_and_free
	.type	__set_test_and_free,@function
__set_test_and_free:                    // @__set_test_and_free
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	str	x25, [sp, #16]          // 8-byte Folded Spill
	ldr	w25, [x0, #1192]
	mov	x20, x0
	ldr	x23, [x8, #8]
	mov	w22, w1
	cmn	w1, #1                  // =1
	mov	w21, w2
	mov	x29, sp
	b.eq	.LBB98_2
// %bb.1:
	udiv	w24, w22, w25
	b	.LBB98_3
.LBB98_2:
	mov	w24, #-1
.LBB98_3:
	add	x19, x23, #12           // =12
	mov	x0, x19
	bl	_raw_spin_lock
	ldr	x8, [x23, #16]
	lsr	w10, w22, #6
	ldr	x9, [x8, w10, uxtw #3]
	str	x9, [x29, #24]
	ldr	x11, [x29, #24]
	mov	w9, #1
	lsl	x9, x9, x22
	tst	x11, x9
	b.eq	.LBB98_19
// %bb.4:
	add	x8, x8, x10, lsl #3
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tst	x10, x9
	b.eq	.LBB98_19
// %bb.5:
	ldr	w8, [x23, #4]
	add	w8, w8, #1              // =1
	str	w8, [x23, #4]
	tbz	w21, #0, .LBB98_11
// %bb.6:
	ldr	w8, [x20, #1192]
.LBB98_7:
	ldr	x0, [x23, #16]
	mul	w21, w25, w24
	add	w1, w8, w21
	mov	w2, w21
	bl	find_next_bit
	ldr	w8, [x20, #1192]
	add	w8, w8, w21
	cmp	w8, w0
	b.hi	.LBB98_19
// %bb.8:
	ldr	x8, [x23, #24]
	lsr	w10, w24, #6
	ldr	x9, [x8, w10, uxtw #3]
	str	x9, [x29, #24]
	ldr	x11, [x29, #24]
	mov	w9, #1
	lsl	x9, x9, x24
	tst	x11, x9
	b.eq	.LBB98_19
// %bb.9:
	add	x8, x8, x10, lsl #3
	//APP
	// atomic64_fetch_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x11, x10, x9
	stlxr	w12, x11, [x8]
	cbnz	w12, 1b
	dmb ish
	//NO_APP
	tst	x10, x9
	b.eq	.LBB98_19
// %bb.10:
	ldr	w8, [x23, #8]
	add	w8, w8, #1              // =1
	str	w8, [x23, #8]
	b	.LBB98_19
.LBB98_11:
	ldr	x8, [x20, #136]
	ldr	x9, [x8, #24]
	ldr	w8, [x20, #1192]
	ldr	w10, [x9, #100]
	udiv	w10, w10, w8
	cmp	w24, w10
	b.eq	.LBB98_19
// %bb.12:
	ldr	w10, [x9, #220]
	udiv	w10, w10, w8
	cmp	w24, w10
	b.eq	.LBB98_19
// %bb.13:
	ldr	w10, [x9, #340]
	udiv	w10, w10, w8
	cmp	w24, w10
	b.eq	.LBB98_19
// %bb.14:
	ldr	w10, [x9, #460]
	udiv	w10, w10, w8
	cmp	w24, w10
	b.eq	.LBB98_19
// %bb.15:
	ldr	w10, [x9, #580]
	udiv	w10, w10, w8
	cmp	w24, w10
	b.eq	.LBB98_19
// %bb.16:
	ldr	w10, [x9, #700]
	udiv	w10, w10, w8
	cmp	w24, w10
	b.eq	.LBB98_19
// %bb.17:
	ldr	w10, [x9, #820]
	udiv	w10, w10, w8
	cmp	w24, w10
	b.eq	.LBB98_19
// %bb.18:
	ldr	w9, [x9, #940]
	udiv	w9, w9, w8
	cmp	w24, w9
	b.ne	.LBB98_7
.LBB98_19:
	mov	x0, x19
	bl	_raw_spin_unlock
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.Lfunc_end98:
	.size	__set_test_and_free, .Lfunc_end98-__set_test_and_free
                                        // -- End function
	.p2align	2               // -- Begin function reset_curseg
	.type	reset_curseg,@function
reset_curseg:                           // @reset_curseg
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	w9, #120
	str	x23, [sp, #16]          // 8-byte Folded Spill
                                        // kill: def $w1 killed $w1 def $x1
	mov	w11, #1
	ldr	x8, [x8, #24]
	mov	w20, w2
	mov	x19, x0
	sxtw	x10, w1
	smaddl	x23, w1, w9, x8
	mov	x9, x23
	ldr	w12, [x9, #112]!
	mov	x29, sp
	str	w12, [x23, #100]!
	ldurh	w22, [x23, #-2]
	strb	w11, [x23, #16]
	cmn	w12, #1                 // =1
	mov	w11, #-1
	b.eq	.LBB99_3
// %bb.1:
	ldr	w13, [x19, #1192]
	udiv	w12, w12, w13
	cmn	w12, #1                 // =1
	b.eq	.LBB99_3
// %bb.2:
	ldr	w13, [x19, #1196]
	udiv	w12, w12, w13
.LBB99_3:
	mov	w13, #120
	madd	x8, x10, x13, x8
	str	w12, [x8, #108]
	strh	wzr, [x8, #104]
	str	w11, [x9]
	ldr	x8, [x8, #32]
	mov	w2, #5
	mov	w1, wzr
	add	x21, x8, #4091          // =4091
	mov	x0, x21
	bl	memset
	cmp	w22, #6                 // =6
	b.hs	.LBB99_11
// %bb.4:
	cmp	w22, #2                 // =2
	b.hi	.LBB99_6
// %bb.5:
	strb	wzr, [x21]
	b	.LBB99_7
.LBB99_6:
	mov	w8, #1
	strb	w8, [x21]
.LBB99_7:
	ldr	x8, [x19, #136]
	mov	w10, #40
	ldr	x9, [x8]
	ldr	w8, [x23]
	ldr	x9, [x9, #120]
	mul	x10, x8, x10
	ldr	w11, [x9, x10]
	bfxil	w11, w22, #0, #6
	str	w11, [x9, x10]
	cbz	w20, .LBB99_10
// %bb.8:
	ldr	x9, [x19, #136]
	add	w10, w8, #63            // =63
	cmp	w8, #0                  // =0
	csel	w10, w10, w8, lt
	ldr	x9, [x9]
	asr	w12, w10, #6
	sbfiz	x12, x12, #3, #32
	and	w10, w10, #0xffffffc0
	ldr	x11, [x9, #56]
	sub	w8, w8, w10
	mov	w10, #1
	lsl	x8, x10, x8
	ldr	x13, [x11, x12]
	orr	x10, x13, x8
	tst	x13, x8
	str	x10, [x11, x12]
	b.ne	.LBB99_10
// %bb.9:
	ldr	w8, [x9, #64]
	add	w8, w8, #1              // =1
	str	w8, [x9, #64]
.LBB99_10:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB99_11:
	bl	print_bdev_access_info
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	cbz	x0, .LBB99_14
// %bb.12:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB99_14
// %bb.13:
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.sanity_check_seg_type
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.sanity_check_seg_type
	mov	w3, #46
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB99_14:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.h"; .popsection; .long 14472b - 14470b; .short 46; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB99_7
// %bb.15:
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
	cbnz	w8, .LBB99_7
// %bb.16:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB99_7
// %bb.17:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB99_7
// %bb.18:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB99_7
.Lfunc_end99:
	.size	reset_curseg, .Lfunc_end99-reset_curseg
                                        // -- End function
	.p2align	2               // -- Begin function __wait_one_discard_bio
	.type	__wait_one_discard_bio,@function
__wait_one_discard_bio:                 // @__wait_one_discard_bio
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	x20, x0
	add	x0, x1, #56             // =56
	mov	x29, sp
	ldr	x19, [x8, #168]
	mov	x21, x1
	bl	wait_for_completion_io
	mov	w8, #8280
	add	x19, x19, x8
	mov	x0, x19
	bl	mutex_lock
	ldrb	w8, [x21, #98]
	cmp	w8, #3                  // =3
	b.ne	.LBB100_8
.LBB100_1:
	ldrh	w8, [x21, #96]
	sub	w8, w8, #1              // =1
	tst	w8, #0xffff
	strh	w8, [x21, #96]
	b.eq	.LBB100_3
// %bb.2:
	mov	w22, wzr
	b	.LBB100_7
.LBB100_3:
	ldr	w8, [x21, #100]
	cbz	w8, .LBB100_5
// %bb.4:
	mov	w22, wzr
	b	.LBB100_6
.LBB100_5:
	ldr	w22, [x21, #28]
.LBB100_6:
	mov	x0, x20
	mov	x1, x21
	bl	__remove_discard_cmd
.LBB100_7:
	mov	x0, x19
	bl	mutex_unlock
	mov	w0, w22
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB100_8:
	bl	print_bdev_access_info
	adrp	x22, f2fs_dclient
	ldr	x0, [x22, :lo12:f2fs_dclient]
	cbz	x0, .LBB100_11
// %bb.9:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB100_11
// %bb.10:
	ldr	x0, [x22, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.__wait_one_discard_bio
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.__wait_one_discard_bio
	mov	w3, #2285
	bl	dsm_client_record
	ldr	x0, [x22, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB100_11:
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.c"; .popsection; .long 14472b - 14470b; .short 2285; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB100_1
// %bb.12:
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
	cbnz	w8, .LBB100_1
// %bb.13:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB100_1
// %bb.14:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB100_1
// %bb.15:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB100_1
.Lfunc_end100:
	.size	__wait_one_discard_bio, .Lfunc_end100-__wait_one_discard_bio
                                        // -- End function
	.p2align	2               // -- Begin function add_sit_entry
	.type	add_sit_entry,@function
add_sit_entry:                          // @add_sit_entry
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	mov	w8, #4767
	movk	w8, #10724, lsl #16
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	umull	x8, w0, w8
	ldr	x20, [x1]
	lsr	x8, x8, #32
	sub	w10, w0, w8
	add	w8, w8, w10, lsr #1
	mov	w9, #55
	lsr	w8, w8, #5
	mov	x19, x1
	cmp	x20, x1
	mul	w22, w8, w9
	mov	x29, sp
	b.eq	.LBB101_3
.LBB101_1:                              // =>This Inner Loop Header: Depth=1
	ldr	w8, [x20, #16]
	cmp	w8, w22
	b.eq	.LBB101_7
// %bb.2:                               //   in Loop: Header=BB101_1 Depth=1
	ldr	x20, [x20]
	cmp	x20, x19
	b.ne	.LBB101_1
.LBB101_3:
	adrp	x8, sit_entry_set_slab
	ldr	x21, [x8, :lo12:sit_entry_set_slab]
	mov	w1, #3136
	mov	x0, x21
	bl	kmem_cache_alloc
	mov	x20, x0
	cbnz	x0, .LBB101_5
// %bb.4:
	mov	w1, #35904
	mov	x0, x21
	bl	kmem_cache_alloc
	mov	x20, x0
.LBB101_5:
	mov	w8, #1
	str	x20, [x20]
	str	x20, [x20, #8]
	stp	w22, w8, [x20, #16]
	ldr	x21, [x19]
	mov	x0, x20
	mov	x1, x19
	mov	x2, x21
	bl	__list_add_valid
	tbz	w0, #0, .LBB101_15
// %bb.6:
	str	x20, [x21, #8]
	stp	x21, x19, [x20]
	str	x20, [x19]
	b	.LBB101_15
.LBB101_7:
	ldr	w8, [x20, #20]
	ldr	x22, [x20]
	add	w8, w8, #1              // =1
	cmp	x22, x19
	str	w8, [x20, #20]
	b.eq	.LBB101_15
.LBB101_8:                              // =>This Inner Loop Header: Depth=1
	ldr	w9, [x22, #20]
	cmp	w8, w9
	b.ls	.LBB101_10
// %bb.9:                               //   in Loop: Header=BB101_8 Depth=1
	ldr	x22, [x22]
	cmp	x22, x19
	b.ne	.LBB101_8
	b	.LBB101_11
.LBB101_10:
	mov	x19, x22
.LBB101_11:
	mov	x0, x20
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB101_13
// %bb.12:
	ldp	x9, x8, [x20]
	str	x8, [x9, #8]
	str	x9, [x8]
.LBB101_13:
	ldr	x21, [x22, #8]
	mov	x0, x20
	mov	x2, x19
	mov	x1, x21
	bl	__list_add_valid
	tbz	w0, #0, .LBB101_15
// %bb.14:
	str	x20, [x22, #8]
	stp	x19, x21, [x20]
	str	x20, [x21]
.LBB101_15:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end101:
	.size	add_sit_entry, .Lfunc_end101-add_sit_entry
                                        // -- End function
	.p2align	2               // -- Begin function current_sit_addr
	.type	current_sit_addr,@function
current_sit_addr:                       // @current_sit_addr
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	w9, #4767
	movk	w9, #10724, lsl #16
	umull	x9, w1, w9
	ldr	x20, [x8]
	lsr	x9, x9, #32
	sub	w10, w1, w9
	add	w9, w9, w10, lsr #1
	ldr	w21, [x20, #8]
	mov	x19, x0
	lsr	w22, w9, #5
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	cbz	x8, .LBB102_2
// %bb.1:
	add	x8, x8, #92             // =92
	b	.LBB102_3
.LBB102_2:
	ldr	x8, [x19, #16]
	add	x8, x8, #48             // =48
.LBB102_3:
	ldr	w8, [x8]
	sub	w8, w8, #1              // =1
	cmp	w8, w1
	b.lo	.LBB102_7
.LBB102_4:
	ldr	x8, [x20, #32]
	lsr	x9, x22, #3
	ldrb	w8, [x8, x9]
	mvn	w9, w22
	and	w9, w9, #0x7
	lsr	w9, w8, w9
	add	w8, w21, w22
	tbz	w9, #0, .LBB102_6
// %bb.5:
	ldr	w9, [x20, #12]
	add	w8, w9, w8
.LBB102_6:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	mov	w0, w8
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB102_7:
	bl	print_bdev_access_info
	adrp	x23, f2fs_dclient
	ldr	x0, [x23, :lo12:f2fs_dclient]
	cbz	x0, .LBB102_10
// %bb.8:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB102_10
// %bb.9:
	ldr	x0, [x23, :lo12:f2fs_dclient]
	adrp	x1, .L.str.1
	adrp	x2, .L__func__.check_seg_range
	add	x1, x1, :lo12:.L.str.1
	add	x2, x2, :lo12:.L__func__.check_seg_range
	mov	w3, #1239
	bl	dsm_client_record
	ldr	x0, [x23, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB102_10:
	mov	x0, x19
	bl	f2fs_print_raw_sb_info
	mov	x0, x19
	bl	f2fs_print_ckpt_info
	mov	x0, x19
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/segment.h"; .popsection; .long 14472b - 14470b; .short 1239; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x19, .LBB102_4
// %bb.11:
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
	cbnz	w8, .LBB102_4
// %bb.12:
	ldr	x1, [x19, #3392]
	cbz	x1, .LBB102_4
// %bb.13:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB102_4
// %bb.14:
	add	x2, x19, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB102_4
.Lfunc_end102:
	.size	current_sit_addr, .Lfunc_end102-current_sit_addr
                                        // -- End function
	.p2align	2               // -- Begin function issue_discard_thread
	.type	issue_discard_thread,@function
issue_discard_thread:                   // @issue_discard_thread
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
	stur	x8, [x29, #-8]
	ldr	x8, [x0, #136]
	mov	w9, #8248
	mov	x19, x0
	ldr	x25, [x8, #168]
	str	wzr, [sp, #64]
	stp	xzr, xzr, [sp, #48]
	stp	xzr, xzr, [sp, #32]
	stp	xzr, xzr, [sp, #16]
	add	x20, x25, x9
	str	xzr, [sp, #8]
	bl	set_freezable
	mov	w24, #34079
	adrp	x27, dpolicys+120
	adrp	x21, dpolicys+180
	adrp	x26, dpolicys+60
	mov	w22, #50
	movk	w24, #20971, lsl #16
	add	x27, x27, :lo12:dpolicys+120
	adrp	x28, system_freezing_cnt
	add	x21, x21, :lo12:dpolicys+180
	add	x26, x26, :lo12:dpolicys+60
	//APP
	mrs x8, sp_el0
	//NO_APP
	str	x8, [sp]                // 8-byte Folded Spill
	b	.LBB103_3
.LBB103_1:                              //   in Loop: Header=BB103_3 Depth=1
	ldr	w22, [sp, #60]
.LBB103_2:                              //   in Loop: Header=BB103_3 Depth=1
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB103_48
.LBB103_3:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB103_14 Depth 2
	ldr	x8, [x19, #136]
	ldr	w9, [x19, #1236]
	ldr	w10, [x19, #1184]
	ldr	w11, [x19, #1240]
	ldr	w12, [x19, #1188]
	ldr	w13, [x8, #104]
	ldr	w14, [x8, #96]
	ldr	x8, [x8, #168]
	sub	w9, w9, w11
	lsl	w10, w13, w10
	mul	w11, w12, w14
	ldr	w8, [x8, #8324]
	add	w9, w9, w10
	add	w10, w11, w11, lsl #2
	lsl	w11, w10, #2
	lsl	w10, w10, #1
	umull	x11, w11, w24
	lsr	x11, x11, #37
	umull	x10, w10, w24
	sub	w8, w9, w8
	cmp	w9, w11
	lsr	x9, x10, #37
	b.hs	.LBB103_6
// %bb.4:                               //   in Loop: Header=BB103_3 Depth=1
	cmp	w8, w9
	b.hs	.LBB103_7
// %bb.5:                               //   in Loop: Header=BB103_3 Depth=1
	ldp	x8, x9, [x27, #32]
	ldr	x10, [x27, #48]
	ldr	w11, [x27, #56]
	stp	x8, x9, [sp, #40]
	ldp	x8, x9, [x27]
	str	x10, [sp, #56]
	str	w11, [sp, #64]
	ldp	x10, x11, [x27, #16]
	b	.LBB103_9
.LBB103_6:                              //   in Loop: Header=BB103_3 Depth=1
	cmp	w8, w9
	b.hs	.LBB103_8
.LBB103_7:                              //   in Loop: Header=BB103_3 Depth=1
	ldp	x8, x9, [x26, #32]
	ldr	x10, [x26, #48]
	ldr	w11, [x26, #56]
	stp	x8, x9, [sp, #40]
	ldp	x8, x9, [x26]
	str	x10, [sp, #56]
	str	w11, [sp, #64]
	ldp	x10, x11, [x26, #16]
	b	.LBB103_9
.LBB103_8:                              //   in Loop: Header=BB103_3 Depth=1
	adrp	x12, dpolicys
	add	x12, x12, :lo12:dpolicys
	ldp	x8, x9, [x12, #32]
	ldr	x10, [x12, #48]
	ldr	w11, [x12, #56]
	stp	x8, x9, [sp, #40]
	ldp	x8, x9, [x12]
	str	x10, [sp, #56]
	str	w11, [sp, #64]
	ldp	x10, x11, [x12, #16]
.LBB103_9:                              //   in Loop: Header=BB103_3 Depth=1
	mov	w0, w22
	stp	x8, x9, [sp, #8]
	stp	x10, x11, [sp, #24]
	strb	wzr, [sp, #64]
	bl	__msecs_to_jiffies
	mov	x23, x0
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB103_23
// %bb.10:                              //   in Loop: Header=BB103_3 Depth=1
	ldr	w8, [x28, :lo12:system_freezing_cnt]
	cbnz	w8, .LBB103_47
.LBB103_11:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	w8, [x25, #8272]
	cbz	x23, .LBB103_24
// %bb.12:                              //   in Loop: Header=BB103_3 Depth=1
	cbnz	w8, .LBB103_24
// %bb.13:                              //   in Loop: Header=BB103_3 Depth=1
	mov	w0, w22
	stp	xzr, xzr, [sp, #96]
	stp	xzr, xzr, [sp, #80]
	str	xzr, [sp, #72]
	bl	__msecs_to_jiffies
	mov	x23, x0
	add	x0, sp, #72             // =72
	mov	w1, wzr
	bl	init_wait_entry
.LBB103_14:                             //   Parent Loop BB103_3 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	add	x1, sp, #72             // =72
	mov	w2, #1
	mov	x0, x20
	bl	prepare_to_wait_event
	mov	x24, x0
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB103_21
// %bb.15:                              //   in Loop: Header=BB103_14 Depth=2
	ldr	w8, [x28, :lo12:system_freezing_cnt]
	cbnz	w8, .LBB103_20
.LBB103_16:                             //   in Loop: Header=BB103_14 Depth=2
	ldr	w8, [x25, #8272]
	cmp	w8, #0                  // =0
	ccmp	x23, #0, #0, ne
	csinc	x0, x23, xzr, ne
	cbnz	w8, .LBB103_21
// %bb.17:                              //   in Loop: Header=BB103_14 Depth=2
	cbz	x0, .LBB103_21
// %bb.18:                              //   in Loop: Header=BB103_14 Depth=2
	cbnz	x24, .LBB103_22
// %bb.19:                              //   in Loop: Header=BB103_14 Depth=2
	bl	schedule_timeout
	mov	x23, x0
	b	.LBB103_14
.LBB103_20:                             //   in Loop: Header=BB103_14 Depth=2
	ldr	x0, [sp]                // 8-byte Folded Reload
	bl	freezing_slow_path
	tbz	w0, #0, .LBB103_16
.LBB103_21:                             //   in Loop: Header=BB103_3 Depth=1
	add	x1, sp, #72             // =72
	mov	x0, x20
	bl	finish_wait
.LBB103_22:                             //   in Loop: Header=BB103_3 Depth=1
	mov	w24, #34079
	movk	w24, #20971, lsl #16
.LBB103_23:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	w8, [x25, #8272]
.LBB103_24:                             //   in Loop: Header=BB103_3 Depth=1
	cbz	w8, .LBB103_26
// %bb.25:                              //   in Loop: Header=BB103_3 Depth=1
	str	wzr, [x25, #8272]
.LBB103_26:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	w8, [x25, #8336]
	cbz	w8, .LBB103_28
// %bb.27:                              //   in Loop: Header=BB103_3 Depth=1
	ldr	w8, [x21, #56]
	ldp	x9, x10, [x21, #32]
	ldr	x11, [x21, #48]
	add	x1, sp, #72             // =72
	str	w8, [sp, #128]
	ldp	x8, x12, [x21]
	stp	x9, x10, [sp, #104]
	ldp	x9, x10, [x21, #16]
	mov	w3, #-1
	mov	x0, x19
	mov	w2, wzr
	str	x11, [sp, #120]
	stp	x8, x12, [sp, #72]
	stp	x9, x10, [sp, #88]
	strb	wzr, [sp, #128]
	bl	__wait_discard_cmd_range
	ldr	w8, [x21, #116]
	ldur	x9, [x21, #92]
	ldur	x10, [x21, #100]
	ldur	x11, [x21, #108]
	ldur	x12, [x21, #60]
	str	w8, [sp, #128]
	ldur	x8, [x21, #68]
	stp	x9, x10, [sp, #104]
	ldur	x9, [x21, #76]
	ldur	x10, [x21, #84]
	add	x1, sp, #72             // =72
	mov	w3, #-1
	mov	x0, x19
	mov	w2, wzr
	str	x11, [sp, #120]
	stp	x12, x8, [sp, #72]
	stp	x9, x10, [sp, #88]
	strb	wzr, [sp, #128]
	bl	__wait_discard_cmd_range
.LBB103_28:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	w8, [x28, :lo12:system_freezing_cnt]
	cbnz	w8, .LBB103_45
.LBB103_29:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	x8, [x19]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB103_2
// %bb.30:                              //   in Loop: Header=BB103_3 Depth=1
	bl	kthread_should_stop
	tbnz	w0, #0, .LBB103_48
// %bb.31:                              //   in Loop: Header=BB103_3 Depth=1
	ldr	x8, [x19, #80]
	tbnz	w8, #2, .LBB103_1
// %bb.32:                              //   in Loop: Header=BB103_3 Depth=1
	ldr	w8, [x25, #8340]
	cbz	w8, .LBB103_2
// %bb.33:                              //   in Loop: Header=BB103_3 Depth=1
	ldr	w8, [x19, #1988]
	cmp	w8, #5                  // =5
	b.eq	.LBB103_35
// %bb.34:                              //   in Loop: Header=BB103_3 Depth=1
	mov	w1, #6
	mov	x0, x19
	bl	f2fs_available_free_memory
	tbnz	w0, #0, .LBB103_37
.LBB103_35:                             //   in Loop: Header=BB103_3 Depth=1
	ldrb	w8, [x19, #1232]
	tbnz	w8, #3, .LBB103_37
// %bb.36:                              //   in Loop: Header=BB103_3 Depth=1
	ldr	w8, [x27, #56]
	ldp	x9, x10, [x27, #32]
	ldr	x11, [x27, #48]
	str	w8, [sp, #64]
	ldp	x8, x12, [x27]
	stp	x9, x10, [sp, #40]
	ldp	x9, x10, [x27, #16]
	str	x11, [sp, #56]
	stp	x8, x12, [sp, #8]
	strb	wzr, [sp, #64]
	stp	x9, x10, [sp, #24]
.LBB103_37:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	x0, [x19]
	mov	w1, #3
	mov	w2, #1
	bl	__sb_start_write
	add	x1, sp, #8              // =8
	mov	x0, x19
	bl	__issue_discard_cmd
	cmp	w0, #1                  // =1
	b.lt	.LBB103_39
// %bb.38:                              //   in Loop: Header=BB103_3 Depth=1
	add	x1, sp, #8              // =8
	mov	w3, #-1
	mov	x0, x19
	mov	w2, wzr
	bl	__wait_discard_cmd_range
	ldr	w22, [sp, #52]
	b	.LBB103_44
.LBB103_39:                             //   in Loop: Header=BB103_3 Depth=1
	cmn	w0, #1                  // =1
	b.eq	.LBB103_41
// %bb.40:                              //   in Loop: Header=BB103_3 Depth=1
	ldr	w22, [sp, #60]
	b	.LBB103_44
.LBB103_41:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	x8, [x19, #560]
	ldr	x9, [x19, #512]
	adrp	x10, jiffies
	ldr	x10, [x10, :lo12:jiffies]
	mov	w11, #250
	madd	x8, x8, x11, x9
	sub	x0, x8, x10
	cmp	x0, #1                  // =1
	b.lt	.LBB103_43
// %bb.42:                              //   in Loop: Header=BB103_3 Depth=1
	bl	jiffies_to_msecs
	mov	w22, w0
	cbnz	w0, .LBB103_44
.LBB103_43:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	w22, [sp, #56]
.LBB103_44:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	x0, [x19]
	mov	w1, #3
	bl	__sb_end_write
	b	.LBB103_2
.LBB103_45:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	x0, [sp]                // 8-byte Folded Reload
	bl	freezing_slow_path
	tbz	w0, #0, .LBB103_29
// %bb.46:                              //   in Loop: Header=BB103_3 Depth=1
	mov	w0, wzr
	bl	__refrigerator
	tbnz	w0, #0, .LBB103_2
	b	.LBB103_29
.LBB103_47:                             //   in Loop: Header=BB103_3 Depth=1
	ldr	x0, [sp]                // 8-byte Folded Reload
	bl	freezing_slow_path
	tbz	w0, #0, .LBB103_11
	b	.LBB103_23
.LBB103_48:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB103_50
// %bb.49:
	ldp	x20, x19, [sp, #224]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #208]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #192]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #176]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #160]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #144]    // 16-byte Folded Reload
	mov	w0, wzr
	add	sp, sp, #240            // =240
	ret
.LBB103_50:
	bl	__stack_chk_fail
.Lfunc_end103:
	.size	issue_discard_thread, .Lfunc_end103-issue_discard_thread
                                        // -- End function
	.p2align	2               // -- Begin function allocate_segment_by_default
	.type	allocate_segment_by_default,@function
allocate_segment_by_default:            // @allocate_segment_by_default
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #136]
	mov	w20, w1
	mov	w22, w2
	mov	w2, w20
	ldr	x23, [x8, #24]
	mov	w8, #120
	mov	x29, sp
	mov	w21, w3
	smaddl	x1, w1, w8, x23
	mov	x19, x0
	bl	segment_prealloc
	tbnz	w0, #0, .LBB104_40
// %bb.1:
	sxtw	x24, w20
	tbz	w22, #0, .LBB104_3
// %bb.2:
	mov	w2, #1
	mov	x0, x19
	mov	w1, w20
	b	.LBB104_38
.LBB104_3:
	ldr	x8, [x19, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #6, .LBB104_5
// %bb.4:
	mov	w8, #120
	madd	x8, x24, x8, x23
	ldrh	w8, [x8, #98]
	cmp	w8, #4                  // =4
	b.eq	.LBB104_37
.LBB104_5:
	mov	w8, #120
	madd	x8, x24, x8, x23
	ldrb	w8, [x8, #96]
	cbz	w8, .LBB104_7
// %bb.6:
	ldr	x11, [x19, #136]
	ldr	w8, [x11, #96]
	ldr	x10, [x11, #8]
	b	.LBB104_10
.LBB104_7:
	ldr	x11, [x19, #136]
	mov	w8, #120
	madd	x8, x24, x8, x23
	ldr	w9, [x8, #100]
	ldr	w8, [x11, #96]
	ldr	x10, [x11, #8]
	add	w9, w9, #1              // =1
	cmp	w9, w8
	b.hs	.LBB104_10
// %bb.8:
	ldr	w12, [x19, #1192]
	udiv	w13, w9, w12
	msub	w12, w13, w12, w9
	cbz	w12, .LBB104_10
// %bb.9:
	ldr	x12, [x10, #16]
	add	w13, w9, #63            // =63
	cmp	w9, #0                  // =0
	csel	w13, w13, w9, lt
	asr	w13, w13, #6
	ldr	x12, [x12, w13, sxtw #3]
	lsr	x9, x12, x9
	tbz	w9, #0, .LBB104_29
.LBB104_10:
	ldr	w13, [x19, #1192]
	ldr	w9, [x19, #1188]
	ldrsw	x16, [x19, #1284]
	ldr	w12, [x19, #1184]
	ldrsw	x17, [x19, #1272]
	ldrsw	x0, [x19, #1296]
	ldr	w14, [x19, #1504]
	cmp	w14, #1                 // =1
	b.eq	.LBB104_37
// %bb.11:
	ldr	w14, [x19, #1988]
	cmp	w14, #3                 // =3
	b.eq	.LBB104_35
// %bb.12:
	mul	w1, w9, w13
	ldp	w9, w15, [x10, #4]
	sub	x10, x1, #1             // =1
	add	x1, x10, x16
	add	x17, x10, x17
	ldr	w14, [x19, #1240]
	add	x0, x10, x0
	ldp	w16, w10, [x11, #100]
	asr	x11, x1, x12
	asr	x17, x17, x12
	asr	x0, x0, x12
	udiv	w11, w11, w13
	udiv	w17, w17, w13
	udiv	w0, w0, w13
	add	w11, w11, w17, lsl #1
	cmp	w20, #1                 // =1
	add	w11, w11, w0
	b.ne	.LBB104_18
// %bb.13:
	cmp	w21, #2                 // =2
	b.ne	.LBB104_18
// %bb.14:
	cmn	w16, #1                 // =1
	b.eq	.LBB104_16
// %bb.15:
	udiv	w17, w16, w13
	add	w17, w17, w17, lsl #1
	cmp	w17, #0                 // =0
	cinc	w17, w17, lt
	asr	w17, w17, #1
	b	.LBB104_17
.LBB104_16:
	mov	w17, #-1
.LBB104_17:
	add	w17, w17, w11
	cmp	w15, w17
	b.hi	.LBB104_37
.LBB104_18:
	cmn	w16, #1                 // =1
	b.eq	.LBB104_20
// %bb.19:
	udiv	w13, w16, w13
	lsl	w13, w13, #1
	b	.LBB104_21
.LBB104_20:
	mov	w13, #-2
.LBB104_21:
	add	w11, w13, w11
	cmp	w15, w11
	b.ls	.LBB104_35
// %bb.22:
	ldr	w11, [x19, #3340]
	cbz	w11, .LBB104_37
// %bb.23:
	cmp	w21, #0                 // =0
	b.gt	.LBB104_37
// %bb.24:
	lsl	w8, w8, w12
	cmp	w8, #1024, lsl #12      // =4194304
	b.ls	.LBB104_37
// %bb.25:
	sub	x8, x8, x14
	lsl	x8, x8, #2
	lsl	w11, w10, #11
	subs	x8, x8, x11
	b.eq	.LBB104_37
// %bb.26:
	cmp	w20, #4                 // =4
	b.hi	.LBB104_37
// %bb.27:
	adrp	x12, .LJTI104_0
	mov	w11, w20
	add	x12, x12, :lo12:.LJTI104_0
	adr	x13, .LBB104_28
	ldrb	w14, [x12, x11]
	add	x13, x13, x14, lsl #2
	br	x13
.LBB104_28:
	add	x11, x19, #3344         // =3344
	add	x12, x19, #3348         // =3348
	b	.LBB104_33
.LBB104_29:
	ldr	x9, [x19, #80]
	tbz	w9, #8, .LBB104_37
	b	.LBB104_10
.LBB104_30:
	add	x11, x19, #3352         // =3352
	add	x12, x19, #3356         // =3356
	b	.LBB104_33
.LBB104_31:
	add	x11, x19, #3360         // =3360
	add	x12, x19, #3364         // =3364
	b	.LBB104_33
.LBB104_32:
	add	x11, x19, #3368         // =3368
	add	x12, x19, #3372         // =3372
.LBB104_33:
	ldr	w11, [x11]
	cmp	x8, x11
	b.hi	.LBB104_37
// %bb.34:
	ldr	w11, [x12]
	sub	w9, w9, w10
	mov	w10, #100
	mul	w9, w9, w10
	lsr	x8, x8, #11
	udiv	x8, x9, x8
	cmp	x8, x11
	b.hi	.LBB104_37
.LBB104_35:
	mov	w2, #1
	mov	x0, x19
	mov	w1, w20
	mov	x3, xzr
	bl	get_ssr_segment
	cbz	w0, .LBB104_37
// %bb.36:
	mov	x0, x19
	mov	w1, w20
	bl	change_curseg
	b	.LBB104_39
.LBB104_37:
	mov	x0, x19
	mov	w1, w20
	mov	w2, wzr
.LBB104_38:
	bl	new_curseg
.LBB104_39:
	mov	w8, #120
	madd	x8, x24, x8, x23
	ldrb	w8, [x8, #96]
	add	x8, x19, x8, lsl #2
	ldr	w9, [x8, #2144]
	add	w9, w9, #1              // =1
	str	w9, [x8, #2144]
.LBB104_40:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end104:
	.size	allocate_segment_by_default, .Lfunc_end104-allocate_segment_by_default
	.section	.rodata,"a",@progbits
.LJTI104_0:
	.byte	(.LBB104_28-.LBB104_28)>>2
	.byte	(.LBB104_30-.LBB104_28)>>2
	.byte	(.LBB104_37-.LBB104_28)>>2
	.byte	(.LBB104_31-.LBB104_28)>>2
	.byte	(.LBB104_32-.LBB104_28)>>2
                                        // -- End function
	.type	inmem_entry_slab,@object // @inmem_entry_slab
	.local	inmem_entry_slab
	.comm	inmem_entry_slab,8,8
	.type	.L.str.1,@object        // @.str.1
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.1:
	.asciz	"F2FS bug: %s:%d\n"
	.size	.L.str.1, 17

	.type	.L__func__.f2fs_drop_inmem_page,@object // @__func__.f2fs_drop_inmem_page
.L__func__.f2fs_drop_inmem_page:
	.asciz	"f2fs_drop_inmem_page"
	.size	.L__func__.f2fs_drop_inmem_page, 21

	.type	f2fs_balance_fs.FG_GC_count,@object // @f2fs_balance_fs.FG_GC_count
	.local	f2fs_balance_fs.FG_GC_count
	.comm	f2fs_balance_fs.FG_GC_count,4,4
	.type	f2fs_balance_fs.fg_gc_rs,@object // @f2fs_balance_fs.fg_gc_rs
	.data
	.p2align	3
f2fs_balance_fs.fg_gc_rs:
	.zero	4
	.word	50000                   // 0xc350
	.word	2                       // 0x2
	.word	0                       // 0x0
	.word	0                       // 0x0
	.zero	4
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	f2fs_balance_fs.fg_gc_rs, 40

	.type	.L__func__.f2fs_balance_fs,@object // @__func__.f2fs_balance_fs
	.section	.rodata.str1.1,"aMS",@progbits,1
.L__func__.f2fs_balance_fs:
	.asciz	"f2fs_balance_fs"
	.size	.L__func__.f2fs_balance_fs, 16

	.type	.L.str.2,@object        // @.str.2
.L.str.2:
	.asciz	"\0015FG_GC: Size=%lluMB,Free=%lluMB,count=%d,free_sec=%u, \t\t\t\treserved_sec=%u,node_secs=%d,dent_secs=%d,meta_secs=%d"
	.size	.L.str.2, 114

	.type	.L.str.3,@object        // @.str.3
.L.str.3:
	.asciz	"\0015FG_GC: Size=%lluMB,Free=%lluMB,count=%d,free_sec=%u, \t\t\treserved_sec=%u,node_secs=%d,dent_secs=%d,meta_secs=%d"
	.size	.L.str.3, 113

	.type	f2fs_create_flush_cmd_control.__key,@object // @f2fs_create_flush_cmd_control.__key
	.local	f2fs_create_flush_cmd_control.__key
	.comm	f2fs_create_flush_cmd_control.__key,1,1
	.type	.L.str.4,@object        // @.str.4
.L.str.4:
	.asciz	"&fcc->flush_wait_queue"
	.size	.L.str.4, 23

	.type	.L.str.5,@object        // @.str.5
.L.str.5:
	.asciz	"f2fs_flush-%u:%u"
	.size	.L.str.5, 17

	.type	.L__func__.f2fs_issue_discard_timeout,@object // @__func__.f2fs_issue_discard_timeout
.L__func__.f2fs_issue_discard_timeout:
	.asciz	"f2fs_issue_discard_timeout"
	.size	.L__func__.f2fs_issue_discard_timeout, 27

	.type	.L__func__.f2fs_invalidate_blocks,@object // @__func__.f2fs_invalidate_blocks
.L__func__.f2fs_invalidate_blocks:
	.asciz	"f2fs_invalidate_blocks"
	.size	.L__func__.f2fs_invalidate_blocks, 23

	.type	.L.str.6,@object        // @.str.6
.L.str.6:
	.asciz	"\0015For resize: curseg of type %d: %u ==> %u"
	.size	.L.str.6, 43

	.type	.L.str.7,@object        // @.str.7
.L.str.7:
	.asciz	"\0015Found FS corruption, run fsck to fix."
	.size	.L.str.7, 40

	.type	.L__func__.f2fs_allocate_data_block,@object // @__func__.f2fs_allocate_data_block
.L__func__.f2fs_allocate_data_block:
	.asciz	"f2fs_allocate_data_block"
	.size	.L__func__.f2fs_allocate_data_block, 25

	.type	.L__func__.f2fs_outplace_write_data,@object // @__func__.f2fs_outplace_write_data
.L__func__.f2fs_outplace_write_data:
	.asciz	"f2fs_outplace_write_data"
	.size	.L__func__.f2fs_outplace_write_data, 25

	.type	.L.str.8,@object        // @.str.8
.L.str.8:
	.asciz	"\0014%s: incorrect segment(%u) type, run fsck to fix."
	.size	.L.str.8, 51

	.type	.L__func__.f2fs_inplace_write_data,@object // @__func__.f2fs_inplace_write_data
.L__func__.f2fs_inplace_write_data:
	.asciz	"f2fs_inplace_write_data"
	.size	.L__func__.f2fs_inplace_write_data, 24

	.type	.L__func__.f2fs_do_replace_block,@object // @__func__.f2fs_do_replace_block
.L__func__.f2fs_do_replace_block:
	.asciz	"f2fs_do_replace_block"
	.size	.L__func__.f2fs_do_replace_block, 22

	.type	.L__func__.f2fs_wait_on_page_writeback,@object // @__func__.f2fs_wait_on_page_writeback
.L__func__.f2fs_wait_on_page_writeback:
	.asciz	"f2fs_wait_on_page_writeback"
	.size	.L__func__.f2fs_wait_on_page_writeback, 28

	.type	.L__func__.f2fs_flush_sit_entries,@object // @__func__.f2fs_flush_sit_entries
.L__func__.f2fs_flush_sit_entries:
	.asciz	"f2fs_flush_sit_entries"
	.size	.L__func__.f2fs_flush_sit_entries, 23

	.type	f2fs_build_segment_manager.__key,@object // @f2fs_build_segment_manager.__key
	.local	f2fs_build_segment_manager.__key
	.comm	f2fs_build_segment_manager.__key,1,1
	.type	.L.str.9,@object        // @.str.9
.L.str.9:
	.asciz	"&sm_info->curseg_lock"
	.size	.L.str.9, 22

	.type	.L.str.10,@object       // @.str.10
.L.str.10:
	.asciz	"f2fs_discard_entry"
	.size	.L.str.10, 19

	.type	discard_entry_slab,@object // @discard_entry_slab
	.local	discard_entry_slab
	.comm	discard_entry_slab,8,8
	.type	.L.str.11,@object       // @.str.11
.L.str.11:
	.asciz	"f2fs_discard_cmd"
	.size	.L.str.11, 17

	.type	discard_cmd_slab,@object // @discard_cmd_slab
	.local	discard_cmd_slab
	.comm	discard_cmd_slab,8,8
	.type	.L.str.12,@object       // @.str.12
.L.str.12:
	.asciz	"f2fs_sit_entry_set"
	.size	.L.str.12, 19

	.type	sit_entry_set_slab,@object // @sit_entry_set_slab
	.local	sit_entry_set_slab
	.comm	sit_entry_set_slab,8,8
	.type	.L.str.13,@object       // @.str.13
.L.str.13:
	.asciz	"f2fs_inmem_page_entry"
	.size	.L.str.13, 22

	.type	.L__func__.f2fs_put_page,@object // @__func__.f2fs_put_page
.L__func__.f2fs_put_page:
	.asciz	"f2fs_put_page"
	.size	.L__func__.f2fs_put_page, 14

	.type	.L.str.14,@object       // @.str.14
.L.str.14:
	.asciz	"\0013cp_rwsem_timeout: start: %lu now: %lu\n"
	.size	.L.str.14, 41

	.type	.L__const.f2fs_bd_balance_decision.ts,@object // @__const.f2fs_bd_balance_decision.ts
	.section	.rodata.cst16,"aM",@progbits,16
	.p2align	3
.L__const.f2fs_bd_balance_decision.ts:
	.xword	15                      // 0xf
	.xword	0                       // 0x0
	.size	.L__const.f2fs_bd_balance_decision.ts, 16

	.type	__init_completion.__key,@object // @__init_completion.__key
	.local	__init_completion.__key
	.comm	__init_completion.__key,1,1
	.type	.L.str.15,@object       // @.str.15
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.15:
	.asciz	"&x->wait"
	.size	.L.str.15, 9

	.type	.L__func__.__locate_dirty_segment,@object // @__func__.__locate_dirty_segment
.L__func__.__locate_dirty_segment:
	.asciz	"__locate_dirty_segment"
	.size	.L__func__.__locate_dirty_segment, 23

	.type	.L__func__.__drop_discard_cmd,@object // @__func__.__drop_discard_cmd
.L__func__.__drop_discard_cmd:
	.asciz	"__drop_discard_cmd"
	.size	.L__func__.__drop_discard_cmd, 19

	.type	.L__func__.__remove_discard_cmd,@object // @__func__.__remove_discard_cmd
.L__func__.__remove_discard_cmd:
	.asciz	"__remove_discard_cmd"
	.size	.L__func__.__remove_discard_cmd, 21

	.type	__remove_discard_cmd._rs,@object // @__remove_discard_cmd._rs
	.data
	.p2align	3
__remove_discard_cmd._rs:
	.zero	4
	.word	1250                    // 0x4e2
	.word	10                      // 0xa
	.word	0                       // 0x0
	.word	0                       // 0x0
	.zero	4
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	__remove_discard_cmd._rs, 40

	.type	.L.str.16,@object       // @.str.16
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.16:
	.asciz	"%sF2FS-fs (%s): Issue discard(%u, %u, %u) failed, ret: %d"
	.size	.L.str.16, 58

	.type	.L.str.17,@object       // @.str.17
.L.str.17:
	.asciz	"\0016"
	.size	.L.str.17, 3

	.type	dpolicys,@object        // @dpolicys
	.section	.rodata,"a",@progbits
	.p2align	2
dpolicys:
	.word	0                       // 0x0
	.word	512                     // 0x200
	.byte	0                       // 0x0
	.zero	3
	.word	512                     // 0x200
	.byte	1                       // 0x1
	.byte	0                       // 0x0
	.zero	2
	.word	1                       // 0x1
	.word	0                       // 0x0
	.zero	8
	.zero	8
	.word	0                       // 0x0
	.word	500                     // 0x1f4
	.word	60000                   // 0xea60
	.byte	0                       // 0x0
	.zero	3
	.word	1                       // 0x1
	.word	16                      // 0x10
	.byte	1                       // 0x1
	.zero	3
	.word	511                     // 0x1ff
	.byte	1                       // 0x1
	.byte	0                       // 0x0
	.zero	2
	.word	1                       // 0x1
	.word	0                       // 0x0
	.word	2                       // 0x2
	.word	50                      // 0x32
	.zero	8
	.word	0                       // 0x0
	.word	500                     // 0x1f4
	.word	60000                   // 0xea60
	.byte	0                       // 0x0
	.zero	3
	.word	2                       // 0x2
	.word	1                       // 0x1
	.byte	1                       // 0x1
	.zero	3
	.word	511                     // 0x1ff
	.byte	1                       // 0x1
	.byte	0                       // 0x0
	.zero	2
	.word	1                       // 0x1
	.word	0                       // 0x0
	.word	2                       // 0x2
	.word	50                      // 0x32
	.word	4                       // 0x4
	.word	2000                    // 0x7d0
	.word	0                       // 0x0
	.word	500                     // 0x1f4
	.word	60000                   // 0xea60
	.byte	0                       // 0x0
	.zero	3
	.word	3                       // 0x3
	.word	1                       // 0x1
	.byte	0                       // 0x0
	.zero	3
	.word	512                     // 0x200
	.byte	1                       // 0x1
	.byte	0                       // 0x0
	.zero	2
	.word	8                       // 0x8
	.word	0                       // 0x0
	.word	8                       // 0x8
	.word	0                       // 0x0
	.word	8                       // 0x8
	.word	0                       // 0x0
	.word	0                       // 0x0
	.word	500                     // 0x1f4
	.word	60000                   // 0xea60
	.byte	0                       // 0x0
	.zero	3
	.word	4                       // 0x4
	.word	1                       // 0x1
	.byte	0                       // 0x0
	.zero	3
	.word	512                     // 0x200
	.byte	1                       // 0x1
	.byte	0                       // 0x0
	.zero	2
	.zero	24
	.word	0                       // 0x0
	.word	500                     // 0x1f4
	.word	60000                   // 0xea60
	.byte	0                       // 0x0
	.zero	3
	.size	dpolicys, 300

	.type	.L__func__.__issue_discard_cmd,@object // @__func__.__issue_discard_cmd
	.section	.rodata.str1.1,"aMS",@progbits,1
.L__func__.__issue_discard_cmd:
	.asciz	"__issue_discard_cmd"
	.size	.L__func__.__issue_discard_cmd, 20

	.type	.L.str.18,@object       // @.str.18
.L.str.18:
	.asciz	"\0015RESIZE: Remove dc: start %u, len %u"
	.size	.L.str.18, 38

	.type	.L.str.19,@object       // @.str.19
.L.str.19:
	.asciz	"\0015RESIZE: dc len truncated to %u"
	.size	.L.str.19, 33

	.type	.L__func__.__submit_discard_cmd,@object // @__func__.__submit_discard_cmd
.L__func__.__submit_discard_cmd:
	.asciz	"__submit_discard_cmd"
	.size	.L__func__.__submit_discard_cmd, 21

	.type	.L__func__.__create_discard_cmd,@object // @__func__.__create_discard_cmd
.L__func__.__create_discard_cmd:
	.asciz	"__create_discard_cmd"
	.size	.L__func__.__create_discard_cmd, 21

	.type	.L.str.20,@object       // @.str.20
.L.str.20:
	.asciz	"\0015RESIZE: skip %s at blkaddr %u"
	.size	.L.str.20, 32

	.type	.L__func__.update_sit_entry,@object // @__func__.update_sit_entry
.L__func__.update_sit_entry:
	.asciz	"update_sit_entry"
	.size	.L__func__.update_sit_entry, 17

	.type	.L.str.21,@object       // @.str.21
.L.str.21:
	.asciz	"\0013Invalid blkaddr 0x%x"
	.size	.L.str.21, 23

	.type	.L.str.22,@object       // @.str.22
.L.str.22:
	.asciz	"\0013Invalid blk count %ld, blkaddr = 0x%x, bitmap is %s"
	.size	.L.str.22, 54

	.type	.L.str.23,@object       // @.str.23
.L.str.23:
	.asciz	"set"
	.size	.L.str.23, 4

	.type	.L.str.24,@object       // @.str.24
.L.str.24:
	.asciz	"not set"
	.size	.L.str.24, 8

	.type	.L.str.25,@object       // @.str.25
.L.str.25:
	.asciz	"\0013Bitmap was wrongly set, blk:%u"
	.size	.L.str.25, 33

	.type	.L.str.26,@object       // @.str.26
.L.str.26:
	.asciz	"\0013Bitmap was wrongly cleared, blk:%u"
	.size	.L.str.26, 37

	.type	.L.str.27,@object       // @.str.27
.L.str.27:
	.asciz	"\0013Cannot get buffer_head of extra_flags_block\n"
	.size	.L.str.27, 47

	.type	.L__func__.get_new_segment,@object // @__func__.get_new_segment
.L__func__.get_new_segment:
	.asciz	"get_new_segment"
	.size	.L__func__.get_new_segment, 16

	.type	.L__func__.__issue_discard_cmd_range,@object // @__func__.__issue_discard_cmd_range
.L__func__.__issue_discard_cmd_range:
	.asciz	"__issue_discard_cmd_range"
	.size	.L__func__.__issue_discard_cmd_range, 26

	.type	.L__func__.__wait_one_discard_bio,@object // @__func__.__wait_one_discard_bio
.L__func__.__wait_one_discard_bio:
	.asciz	"__wait_one_discard_bio"
	.size	.L__func__.__wait_one_discard_bio, 23

	.type	.L__func__.sanity_check_seg_type,@object // @__func__.sanity_check_seg_type
.L__func__.sanity_check_seg_type:
	.asciz	"sanity_check_seg_type"
	.size	.L__func__.sanity_check_seg_type, 22

	.type	.L__func__.__get_segment_type,@object // @__func__.__get_segment_type
.L__func__.__get_segment_type:
	.asciz	"__get_segment_type"
	.size	.L__func__.__get_segment_type, 19

	.type	.L.str.28,@object       // @.str.28
.L.str.28:
	.asciz	"\0015RESIZE: Skip segno %u / %u in jnl!"
	.size	.L.str.28, 37

	.type	.L__func__.check_seg_range,@object // @__func__.check_seg_range
.L__func__.check_seg_range:
	.asciz	"check_seg_range"
	.size	.L__func__.check_seg_range, 16

	.type	.L.str.29,@object       // @.str.29
.L.str.29:
	.asciz	"\0013Mismatch valid blocks %d vs. %d"
	.size	.L.str.29, 34

	.type	.L.str.30,@object       // @.str.30
.L.str.30:
	.asciz	"SBI_NEED_FSCK:%s:%d\n"
	.size	.L.str.30, 21

	.type	.L__func__.check_block_count,@object // @__func__.check_block_count
.L__func__.check_block_count:
	.asciz	"check_block_count"
	.size	.L__func__.check_block_count, 18

	.type	.L.str.31,@object       // @.str.31
.L.str.31:
	.asciz	"\0013Wrong valid blocks %d or segno %u"
	.size	.L.str.31, 36

	.type	create_discard_cmd_control.__key,@object // @create_discard_cmd_control.__key
	.local	create_discard_cmd_control.__key
	.comm	create_discard_cmd_control.__key,1,1
	.type	.L.str.32,@object       // @.str.32
.L.str.32:
	.asciz	"&dcc->cmd_lock"
	.size	.L.str.32, 15

	.type	create_discard_cmd_control.__key.33,@object // @create_discard_cmd_control.__key.33
	.local	create_discard_cmd_control.__key.33
	.comm	create_discard_cmd_control.__key.33,1,1
	.type	.L.str.34,@object       // @.str.34
.L.str.34:
	.asciz	"&dcc->discard_wait_queue"
	.size	.L.str.34, 25

	.type	.L.str.35,@object       // @.str.35
.L.str.35:
	.asciz	"f2fs_discard-%u:%u"
	.size	.L.str.35, 19

	.type	default_salloc_ops,@object // @default_salloc_ops
	.section	.rodata,"a",@progbits
	.p2align	3
default_salloc_ops:
	.xword	allocate_segment_by_default
	.size	default_salloc_ops, 8

	.type	build_sit_info.__key,@object // @build_sit_info.__key
	.local	build_sit_info.__key
	.comm	build_sit_info.__key,1,1
	.type	.L.str.36,@object       // @.str.36
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.36:
	.asciz	"&sit_i->sentry_lock"
	.size	.L.str.36, 20

	.type	build_curseg.__key,@object // @build_curseg.__key
	.local	build_curseg.__key
	.comm	build_curseg.__key,1,1
	.type	.L.str.37,@object       // @.str.37
.L.str.37:
	.asciz	"&array[i].curseg_mutex"
	.size	.L.str.37, 23

	.type	build_curseg.__key.38,@object // @build_curseg.__key.38
	.local	build_curseg.__key.38
	.comm	build_curseg.__key.38,1,1
	.type	.L.str.39,@object       // @.str.39
.L.str.39:
	.asciz	"&array[i].journal_rwsem"
	.size	.L.str.39, 24

	.type	.L.str.40,@object       // @.str.40
.L.str.40:
	.asciz	"\0013invalid journal entries nats %u sits %u\n"
	.size	.L.str.40, 43

	.type	.L.str.41,@object       // @.str.41
.L.str.41:
	.asciz	"\0013Wrong journal entry on segno %u"
	.size	.L.str.41, 34

	.type	.L.str.42,@object       // @.str.42
.L.str.42:
	.asciz	"\0013SIT is corrupted node# %u vs %u"
	.size	.L.str.42, 34

	.type	.L__func__.build_sit_entries,@object // @__func__.build_sit_entries
.L__func__.build_sit_entries:
	.asciz	"build_sit_entries"
	.size	.L__func__.build_sit_entries, 18

	.type	build_dirty_segmap.__key,@object // @build_dirty_segmap.__key
	.local	build_dirty_segmap.__key
	.comm	build_dirty_segmap.__key,1,1
	.type	.L.str.43,@object       // @.str.43
.L.str.43:
	.asciz	"&dirty_i->seglist_lock"
	.size	.L.str.43, 23

	.type	.L__func__.init_dirty_segmap,@object // @__func__.init_dirty_segmap
.L__func__.init_dirty_segmap:
	.asciz	"init_dirty_segmap"
	.size	.L__func__.init_dirty_segmap, 18

	.type	.L.str.44,@object       // @.str.44
.L.str.44:
	.asciz	"\0013Current segment's next free block offset is inconsistent with bitmap, logtype:%u, segno:%u, type:%u, next_blkoff:%u, blkofs:%u"
	.size	.L.str.44, 129

	.type	.Lswitch.table.f2fs_io_type_to_rw_hint,@object // @switch.table.f2fs_io_type_to_rw_hint
	.section	.rodata,"a",@progbits
	.p2align	2
.Lswitch.table.f2fs_io_type_to_rw_hint:
	.word	2                       // 0x2
	.word	0                       // 0x0
	.word	5                       // 0x5
	.size	.Lswitch.table.f2fs_io_type_to_rw_hint, 12

	.type	.Lswitch.table.f2fs_io_type_to_rw_hint.45,@object // @switch.table.f2fs_io_type_to_rw_hint.45
	.p2align	2
.Lswitch.table.f2fs_io_type_to_rw_hint.45:
	.word	2                       // 0x2
	.word	4                       // 0x4
	.word	5                       // 0x5
	.size	.Lswitch.table.f2fs_io_type_to_rw_hint.45, 12

	.ident	"Android (6443078 based on r383902) clang version 11.0.1 (https://android.googlesource.com/toolchain/llvm-project b397f81060ce6d701042b782172ed13bee898b79)"
	.section	".note.GNU-stack","",@progbits
