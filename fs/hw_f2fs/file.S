	.text
	.file	"file.c"
	.globl	create_page_info_slab   // -- Begin function create_page_info_slab
	.p2align	2
	.type	create_page_info_slab,@function
create_page_info_slab:                  // @create_page_info_slab
.Lcreate_page_info_slab$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x0, .L.str
	add	x0, x0, :lo12:.L.str
	mov	w1, #24
	mov	w3, #131072
	mov	w2, wzr
	mov	x4, xzr
	mov	x29, sp
	bl	kmem_cache_create
	cmp	x0, #0                  // =0
	mov	w8, #-12
	adrp	x9, page_info_slab
	csel	w8, w8, wzr, eq
	str	x0, [x9, :lo12:page_info_slab]
	mov	w0, w8
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end0:
	.size	create_page_info_slab, .Lfunc_end0-create_page_info_slab
                                        // -- End function
	.globl	destroy_page_info_slab  // -- Begin function destroy_page_info_slab
	.p2align	2
	.type	destroy_page_info_slab,@function
destroy_page_info_slab:                 // @destroy_page_info_slab
.Ldestroy_page_info_slab$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	adrp	x8, page_info_slab
	ldr	x0, [x8, :lo12:page_info_slab]
	mov	x29, sp
	cbz	x0, .LBB1_2
// %bb.1:
	bl	kmem_cache_destroy
.LBB1_2:
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end1:
	.size	destroy_page_info_slab, .Lfunc_end1-destroy_page_info_slab
                                        // -- End function
	.globl	f2fs_drop_deduped_link  // -- Begin function f2fs_drop_deduped_link
	.p2align	2
	.type	f2fs_drop_deduped_link,@function
f2fs_drop_deduped_link:                 // @f2fs_drop_deduped_link
.Lf2fs_drop_deduped_link$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	add	x21, x0, #744           // =744
	mov	x19, x0
	ldr	x20, [x8, #1008]
	mov	x0, x21
	mov	x29, sp
	bl	down_write
	mov	x0, x19
	bl	drop_nlink
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x21
	bl	up_write
	ldr	w8, [x19, #72]
	cbz	w8, .LBB2_2
// %bb.1:
	mov	x0, x20
	bl	f2fs_release_orphan_inode
	b	.LBB2_3
.LBB2_2:
	mov	x0, x19
	bl	f2fs_add_orphan_inode
.LBB2_3:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end2:
	.size	f2fs_drop_deduped_link, .Lfunc_end2-f2fs_drop_deduped_link
                                        // -- End function
	.globl	f2fs_set_inode_addr     // -- Begin function f2fs_set_inode_addr
	.p2align	2
	.type	f2fs_set_inode_addr,@function
f2fs_set_inode_addr:                    // @f2fs_set_inode_addr
.Lf2fs_set_inode_addr$local:
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x28, x27, [sp, #16]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	mov	x21, x0
	mov	w22, w1
	mov	x29, sp
	ldr	x19, [x8, #1008]
	ldr	x8, [x0, #64]
	mov	x0, x19
	mov	x1, x8
	bl	f2fs_get_node_page
	mov	x20, x0
	cmn	x0, #12                 // =12
	b.eq	.LBB3_3
// %bb.1:
	cmn	x20, #4095              // =4095
	b.lo	.LBB3_8
.LBB3_2:
	adrp	x1, .L.str.3
	adrp	x2, .L__func__.f2fs_set_inode_addr
	add	x1, x1, :lo12:.L.str.3
	add	x2, x2, :lo12:.L__func__.f2fs_set_inode_addr
	mov	x0, x19
	bl	f2fs_printk
	b	.LBB3_40
.LBB3_3:
	mov	w26, #32401
	mov	w27, #61231
	mov	w28, #36280
	adrp	x24, .L.str.1
	adrp	x25, .L__func__.f2fs_set_inode_addr
	mov	w23, #2
	movk	w26, #15099, lsl #16
	movk	w27, #50488, lsl #16
	movk	w28, #6, lsl #16
	add	x24, x24, :lo12:.L.str.1
	add	x25, x25, :lo12:.L__func__.f2fs_set_inode_addr
	b	.LBB3_5
.LBB3_4:                                //   in Loop: Header=BB3_5 Depth=1
	ldr	x1, [x21, #64]
	mov	x0, x19
	bl	f2fs_get_node_page
	cmn	x0, #12                 // =12
	add	w23, w23, #1            // =1
	b.ne	.LBB3_7
.LBB3_5:                                // =>This Inner Loop Header: Depth=1
	madd	w8, w23, w26, w27
	ror	w8, w8, #4
	cmp	w8, w28
	b.hi	.LBB3_4
// %bb.6:                               //   in Loop: Header=BB3_5 Depth=1
	mov	x0, x19
	mov	x1, x24
	mov	x2, x25
	mov	w3, w23
	bl	f2fs_printk
	b	.LBB3_4
.LBB3_7:
	mov	x20, x0
	cmn	x20, #4095              // =4095
	b.hs	.LBB3_2
.LBB3_8:
	mov	w1, #1
	mov	w2, #1
	mov	w3, #1
	mov	x0, x20
	bl	f2fs_wait_on_page_writeback
	mov	x8, #134217728
	movk	x8, #64, lsl #32
	ldr	x9, [x21, #736]
	add	x8, x8, x20, lsl #6
	and	x8, x8, #0xfffffffffffff000
	mov	x10, #-549755813888
	add	x26, x8, x10
	tbnz	w9, #26, .LBB3_10
// %bb.9:
	mov	w27, wzr
	b	.LBB3_11
.LBB3_10:
	ldr	w8, [x21, #1184]
	asr	w27, w8, #2
.LBB3_11:
	mov	w28, wzr
	mov	w23, #923
	b	.LBB3_13
.LBB3_12:                               //   in Loop: Header=BB3_13 Depth=1
	str	w22, [x24]
	add	w28, w28, #1            // =1
.LBB3_13:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB3_22 Depth 2
	ldr	w8, [x21, #1184]
	ldr	w9, [x21, #1192]
	ldrh	w10, [x21]
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	sub	w8, w23, w8
	b.ne	.LBB3_16
// %bb.14:                              //   in Loop: Header=BB3_13 Depth=1
	ldr	x9, [x21, #736]
	tbz	w9, #31, .LBB3_16
// %bb.15:                              //   in Loop: Header=BB3_13 Depth=1
	ldr	w9, [x21, #1292]
	neg	w9, w9
	and	w8, w8, w9
.LBB3_16:                               //   in Loop: Header=BB3_13 Depth=1
	cmp	w28, w8
	b.hs	.LBB3_28
// %bb.17:                              //   in Loop: Header=BB3_13 Depth=1
	add	w25, w28, w27
	add	x8, x26, w25, sxtw #2
	ldr	w1, [x8, #360]
	add	x24, x8, #360           // =360
	add	w9, w1, #3              // =3
	cmp	w9, #4                  // =4
	b.lo	.LBB3_12
// %bb.18:                              //   in Loop: Header=BB3_13 Depth=1
	ldr	w9, [x19, #3128]
	cbz	w9, .LBB3_26
// %bb.19:                              //   in Loop: Header=BB3_13 Depth=1
	ldr	x8, [x19]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB3_25
// %bb.20:                              //   in Loop: Header=BB3_13 Depth=1
	ldr	x10, [x19, #3136]
	add	x10, x10, #80           // =80
	b	.LBB3_22
.LBB3_21:                               //   in Loop: Header=BB3_22 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB3_25
.LBB3_22:                               //   Parent Loop BB3_13 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB3_21
// %bb.23:                              //   in Loop: Header=BB3_22 Depth=2
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB3_21
// %bb.24:                              //   in Loop: Header=BB3_13 Depth=1
	ldur	x8, [x10, #-80]
.LBB3_25:                               //   in Loop: Header=BB3_13 Depth=1
	cbz	x8, .LBB3_12
.LBB3_26:                               //   in Loop: Header=BB3_13 Depth=1
	mov	w2, #7
	mov	x0, x19
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB3_12
// %bb.27:                              //   in Loop: Header=BB3_13 Depth=1
	ldr	x3, [x21, #64]
	ldr	w5, [x24]
	adrp	x1, .L.str.4
	adrp	x2, .L__func__.f2fs_set_inode_addr
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.4
	add	x2, x2, :lo12:.L__func__.f2fs_set_inode_addr
	mov	w4, w25
	bl	f2fs_printk
	add	w28, w28, #1            // =1
	b	.LBB3_13
.LBB3_28:
	ldr	w5, [x26, #4052]
	cbz	w5, .LBB3_34
// %bb.29:
	ldr	x3, [x21, #64]
	adrp	x1, .L.str.5
	adrp	x2, .L__func__.f2fs_set_inode_addr
	add	x1, x1, :lo12:.L.str.5
	add	x2, x2, :lo12:.L__func__.f2fs_set_inode_addr
	mov	x0, x19
	mov	w4, wzr
	bl	f2fs_printk
	ldr	w5, [x26, #4056]
	cbz	w5, .LBB3_35
.LBB3_30:
	ldr	x3, [x21, #64]
	adrp	x1, .L.str.5
	adrp	x2, .L__func__.f2fs_set_inode_addr
	add	x1, x1, :lo12:.L.str.5
	add	x2, x2, :lo12:.L__func__.f2fs_set_inode_addr
	mov	w4, #1
	mov	x0, x19
	bl	f2fs_printk
	ldr	w5, [x26, #4060]
	cbz	w5, .LBB3_36
.LBB3_31:
	ldr	x3, [x21, #64]
	adrp	x1, .L.str.5
	adrp	x2, .L__func__.f2fs_set_inode_addr
	add	x1, x1, :lo12:.L.str.5
	add	x2, x2, :lo12:.L__func__.f2fs_set_inode_addr
	mov	w4, #2
	mov	x0, x19
	bl	f2fs_printk
	ldr	w5, [x26, #4064]
	cbz	w5, .LBB3_37
.LBB3_32:
	ldr	x3, [x21, #64]
	adrp	x1, .L.str.5
	adrp	x2, .L__func__.f2fs_set_inode_addr
	add	x1, x1, :lo12:.L.str.5
	add	x2, x2, :lo12:.L__func__.f2fs_set_inode_addr
	mov	w4, #3
	mov	x0, x19
	bl	f2fs_printk
	ldr	w5, [x26, #4068]
	cbz	w5, .LBB3_38
.LBB3_33:
	ldr	x3, [x21, #64]
	adrp	x1, .L.str.5
	adrp	x2, .L__func__.f2fs_set_inode_addr
	add	x1, x1, :lo12:.L.str.5
	add	x2, x2, :lo12:.L__func__.f2fs_set_inode_addr
	mov	w4, #4
	mov	x0, x19
	bl	f2fs_printk
	b	.LBB3_39
.LBB3_34:
	str	wzr, [x26, #4052]
	ldr	w5, [x26, #4056]
	cbnz	w5, .LBB3_30
.LBB3_35:
	str	wzr, [x26, #4056]
	ldr	w5, [x26, #4060]
	cbnz	w5, .LBB3_31
.LBB3_36:
	str	wzr, [x26, #4060]
	ldr	w5, [x26, #4064]
	cbnz	w5, .LBB3_32
.LBB3_37:
	str	wzr, [x26, #4064]
	ldr	w5, [x26, #4068]
	cbnz	w5, .LBB3_33
.LBB3_38:
	str	wzr, [x26, #4068]
.LBB3_39:
	mov	x0, x20
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_put_page
	mov	w20, wzr
.LBB3_40:
	mov	w0, w20
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.Lfunc_end3:
	.size	f2fs_set_inode_addr, .Lfunc_end3-f2fs_set_inode_addr
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_put_page
	.type	f2fs_put_page,@function
f2fs_put_page:                          // @f2fs_put_page
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x0, .LBB4_32
// %bb.1:
	mov	x19, x0
	cbz	w1, .LBB4_30
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
	b.ne	.LBB4_24
// %bb.3:
	tbnz	w8, #0, .LBB4_29
// %bb.4:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB4_7
// %bb.5:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB4_7
// %bb.6:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.f2fs_put_page
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.f2fs_put_page
	mov	w3, #3801
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB4_7:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB4_9
// %bb.8:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB4_33
.LBB4_9:
	ldr	x0, [x19, #24]
.LBB4_10:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_raw_sb_info
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB4_12
// %bb.11:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB4_34
.LBB4_12:
	ldr	x0, [x19, #24]
.LBB4_13:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_ckpt_info
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #19, .LBB4_15
// %bb.14:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB4_35
.LBB4_15:
	ldr	x0, [x19, #24]
.LBB4_16:
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
	tbz	w8, #19, .LBB4_18
// %bb.17:
	ldr	x8, [x19]
	tbnz	w8, #10, .LBB4_36
.LBB4_18:
	ldr	x0, [x19, #24]
.LBB4_19:
	ldr	x8, [x0]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB4_29
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
	cbnz	w9, .LBB4_29
// %bb.21:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB4_29
// %bb.22:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB4_29
// %bb.23:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB4_29
.LBB4_24:
	tbnz	w8, #0, .LBB4_29
// %bb.25:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB4_28
// %bb.26:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB4_28
// %bb.27:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.f2fs_put_page
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.f2fs_put_page
	mov	w3, #3803
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB4_28:
	mov	x0, xzr
	bl	f2fs_print_raw_sb_info
	mov	x0, xzr
	bl	f2fs_print_ckpt_info
	mov	x0, xzr
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/f2fs.h"; .popsection; .long 14472b - 14470b; .short 3803; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
.LBB4_29:
	mov	x0, x19
	bl	unlock_page
.LBB4_30:
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
	cbnz	w10, .LBB4_32
// %bb.31:
	bl	__put_page
.LBB4_32:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.LBB4_33:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB4_10
.LBB4_34:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB4_13
.LBB4_35:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB4_16
.LBB4_36:
	mov	x0, x19
	bl	__page_file_mapping
	b	.LBB4_19
.Lfunc_end4:
	.size	f2fs_put_page, .Lfunc_end4-f2fs_put_page
                                        // -- End function
	.globl	f2fs_is_hole_blkaddr    // -- Begin function f2fs_is_hole_blkaddr
	.p2align	2
	.type	f2fs_is_hole_blkaddr,@function
f2fs_is_hole_blkaddr:                   // @f2fs_is_hole_blkaddr
.Lf2fs_is_hole_blkaddr$local:
// %bb.0:
	sub	sp, sp, #96             // =96
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #64]     // 16-byte Folded Spill
	add	x29, sp, #64            // =64
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	ldr	x8, [x0, #736]
	tbnz	w8, #10, .LBB5_30
// %bb.1:
	ldr	x8, [x0, #736]
	mov	x19, x0
	tbnz	w8, #11, .LBB5_30
// %bb.2:
	mov	x20, x1
	add	x0, sp, #8              // =8
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	add	x0, sp, #8              // =8
	mov	w2, #1
	mov	x1, x20
	stp	x19, xzr, [sp, #8]
	str	xzr, [sp, #24]
	str	wzr, [sp, #32]
	bl	f2fs_get_dnode_of_data
	cbz	w0, .LBB5_5
// %bb.3:
	cmn	w0, #2                  // =2
	mov	w0, wzr
	b.ne	.LBB5_31
// %bb.4:
	mov	w0, #1
	b	.LBB5_31
.LBB5_5:
	ldr	x0, [sp, #24]
	mov	x8, #134217728
	movk	x8, #64, lsl #32
	mov	x9, #-549755813888
	mov	x10, #-549755813888
	add	x8, x8, x0, lsl #6
	movk	x9, #4072
	movk	x10, #4076
	and	x11, x8, #0xfffffffffffff000
	ldr	w9, [x11, x9]
	ldr	w10, [x11, x10]
	ldr	w8, [sp, #36]
	mov	x12, #-549755813888
	add	x11, x11, x12
	cmp	w9, w10
	b.ne	.LBB5_10
// %bb.6:
	ldr	x12, [sp, #8]
	cbz	x12, .LBB5_9
// %bb.7:
	ldr	x13, [x12, #736]
	tbz	w13, #26, .LBB5_10
// %bb.8:
	ldr	w12, [x12, #1184]
	asr	w12, w12, #2
	b	.LBB5_11
.LBB5_9:
	ldrb	w12, [x11, #3]
	tbnz	w12, #5, .LBB5_33
.LBB5_10:
	mov	w12, wzr
.LBB5_11:
	add	x13, x11, #360          // =360
	cmp	w9, w10
	csel	x9, x13, x11, eq
	add	w8, w12, w8
	ldr	w20, [x9, w8, uxtw #2]
	cbz	x0, .LBB5_13
// %bb.12:
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #24]
	ldr	x8, [sp, #16]
	cbnz	x8, .LBB5_14
	b	.LBB5_17
.LBB5_13:
	mov	x9, xzr
	ldr	x8, [sp, #16]
	cbz	x8, .LBB5_17
.LBB5_14:
	cmp	x9, x8
	b.eq	.LBB5_17
// %bb.15:
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
	cbnz	w10, .LBB5_17
// %bb.16:
	bl	__put_page
.LBB5_17:
	add	w8, w20, #3             // =3
	cmp	w8, #4                  // =4
	stp	xzr, xzr, [sp, #16]
	b.hs	.LBB5_19
// %bb.18:
	and	w8, w8, #0xf
	cmp	w8, #3                  // =3
	b	.LBB5_29
.LBB5_19:
	ldr	x8, [x19, #40]
	ldr	x0, [x8, #1008]
	ldr	w9, [x0, #3128]
	cbz	w9, .LBB5_27
// %bb.20:
	ldr	x8, [x0]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB5_26
// %bb.21:
	ldr	x10, [x0, #3136]
	add	x10, x10, #80           // =80
	b	.LBB5_23
.LBB5_22:                               //   in Loop: Header=BB5_23 Depth=1
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB5_26
.LBB5_23:                               // =>This Inner Loop Header: Depth=1
	ldur	w11, [x10, #-4]
	cmp	w11, w20
	b.hi	.LBB5_22
// %bb.24:                              //   in Loop: Header=BB5_23 Depth=1
	ldr	w11, [x10]
	cmp	w11, w20
	b.lo	.LBB5_22
// %bb.25:
	ldur	x8, [x10, #-80]
.LBB5_26:
	cbz	x8, .LBB5_28
.LBB5_27:
	mov	w2, #6
	mov	w1, w20
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB5_30
.LBB5_28:
	cmp	w20, #0                 // =0
.LBB5_29:
	cset	w0, eq
	b	.LBB5_31
.LBB5_30:
	mov	w0, wzr
.LBB5_31:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB5_34
// %bb.32:
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #64]     // 16-byte Folded Reload
	add	sp, sp, #96             // =96
	ret
.LBB5_33:
	ldrh	w12, [x11, #360]
	lsr	w12, w12, #2
	b	.LBB5_11
.LBB5_34:
	bl	__stack_chk_fail
.Lfunc_end5:
	.size	f2fs_is_hole_blkaddr, .Lfunc_end5-f2fs_is_hole_blkaddr
                                        // -- End function
	.globl	f2fs_revoke_deduped_inode // -- Begin function f2fs_revoke_deduped_inode
	.p2align	2
	.type	f2fs_revoke_deduped_inode,@function
f2fs_revoke_deduped_inode:              // @f2fs_revoke_deduped_inode
.Lf2fs_revoke_deduped_inode$local:
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
	ldr	x8, [x0, #40]
	ldr	x21, [x8, #1008]
	ldr	x8, [x21, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB6_100
// %bb.1:
	ldr	x8, [x0, #736]
	mov	x26, x0
	tbnz	x8, #38, .LBB6_5
.LBB6_2:
	mov	w22, #-22
.LBB6_3:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB6_102
// %bb.4:
	mov	w0, w22
	ldp	x20, x19, [sp, #304]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #288]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #272]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #256]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #240]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #224]    // 16-byte Folded Reload
	add	sp, sp, #320            // =320
	ret
.LBB6_5:
	ldr	x8, [x26, #736]
	mov	x24, x1
	tbz	x8, #38, .LBB6_7
// %bb.6:
	ldr	x8, [x26, #736]
	tbnz	x8, #39, .LBB6_2
.LBB6_7:
	mov	x0, x26
	bl	dquot_initialize
	add	w8, w0, #28             // =28
	mov	w22, w0
	cmp	w8, #28                 // =28
	b.hi	.LBB6_9
// %bb.8:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB6_10
.LBB6_9:
	cmn	w22, #122               // =122
	b.ne	.LBB6_3
.LBB6_10:
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_balance_fs
	ldr	x0, [x26, #1304]
	cbz	x0, .LBB6_19
// %bb.11:
	ldrb	w8, [x0, #13]
	ldr	w27, [x0, #64]
	tbz	w8, #6, .LBB6_13
// %bb.12:
	bl	__fscrypt_prepare_encrypt_info
	mov	w22, w0
	cbnz	w0, .LBB6_3
.LBB6_13:
	ldr	x8, [x26, #40]
	ldr	x23, [x8, #1008]
	add	x21, x23, #368          // =368
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	x8, [x23, #352]
	cbz	x8, .LBB6_15
// %bb.14:
	adrp	x19, jiffies
	ldr	x8, [x23, #360]
	ldr	x9, [x19, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB6_82
.LBB6_15:
	mov	x0, x21
	bl	_raw_spin_unlock
	add	x21, x23, #304          // =304
	mov	x0, x21
	bl	down_read
	mov	x0, x23
	bl	f2fs_acquire_orphan_inode
	cbz	w0, .LBB6_20
// %bb.16:
	mov	w22, w0
	mov	x0, x21
	bl	up_read
	ldr	x2, [x26, #64]
	adrp	x1, .L.str.15
	add	x1, x1, :lo12:.L.str.15
	mov	x0, x23
	mov	w3, w22
	bl	f2fs_printk
.LBB6_17:
	adrp	x0, f2fs_revoke_deduped_inode.revoke_bigdata_rs
	adrp	x1, .L__func__.f2fs_revoke_deduped_inode
	add	x0, x0, :lo12:f2fs_revoke_deduped_inode.revoke_bigdata_rs
	add	x1, x1, :lo12:.L__func__.f2fs_revoke_deduped_inode
	bl	___ratelimit
	cbz	w0, .LBB6_3
// %bb.18:
	stp	xzr, xzr, [sp, #192]
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	stp	xzr, xzr, [sp, #128]
	stp	xzr, xzr, [sp, #112]
	stp	xzr, xzr, [sp, #96]
	stp	xzr, xzr, [sp, #80]
	ldr	x8, [x26, #40]
	ldr	x4, [x26, #64]
	adrp	x1, .L.str.19
	add	x1, x1, :lo12:.L.str.19
	ldr	x0, [x8, #1008]
	mov	x2, x24
	mov	w3, w22
	mov	w5, w27
	bl	f2fs_printk
	adrp	x2, .L.str.20
	add	x2, x2, :lo12:.L.str.20
	add	x0, sp, #80             // =80
	mov	w1, #128
	mov	w3, w22
	bl	snprintf
	add	x3, sp, #80             // =80
	mov	x0, x26
	mov	w1, w27
	mov	x2, x24
	bl	f2fs_dedup_imonitor_send
	b	.LBB6_3
.LBB6_19:
	ldr	x2, [x26, #64]
	adrp	x1, .L.str.6
	add	x1, x1, :lo12:.L.str.6
	mov	x0, x21
	bl	f2fs_printk
	b	.LBB6_2
.LBB6_20:
	mov	x0, x26
	str	w27, [sp, #16]          // 4-byte Folded Spill
	str	x24, [sp, #8]           // 8-byte Folded Spill
	bl	f2fs_add_orphan_inode
	add	x8, x26, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 2199023255552
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x26
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x21
	bl	up_read
	ldr	x8, [x26, #40]
	mov	x0, x26
	mov	w1, wzr
	ldr	x8, [x8, #1008]
	str	x8, [sp]                // 8-byte Folded Spill
	ldr	w8, [x8, #1188]
	str	w8, [sp, #20]           // 4-byte Folded Spill
	bl	f2fs_set_inode_addr
	ldr	x8, [x26, #80]
	str	x26, [sp, #56]          // 8-byte Folded Spill
	add	x8, x8, #4095           // =4095
	lsr	x20, x8, #12
	cbz	w20, .LBB6_53
// %bb.21:
	ldr	x22, [sp, #56]          // 8-byte Folded Reload
	mov	x24, #256
	mov	x10, xzr
	movk	x24, #57005, lsl #48
	add	x19, sp, #64            // =64
	adrp	x26, page_info_slab
.LBB6_22:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB6_25 Depth 2
                                        //     Child Loop BB6_33 Depth 2
                                        //     Child Loop BB6_48 Depth 2
	ldp	x8, x25, [x22, #40]
	ldr	w9, [sp, #20]           // 4-byte Folded Reload
	mov	x1, xzr
	mov	x2, x10
	ldr	x8, [x8, #1008]
	cmp	w9, w20
	csel	w21, w9, w20, lt
	sxtw	x3, w21
	mov	x0, x25
	mov	x4, xzr
	str	x8, [sp, #48]           // 8-byte Folded Spill
	stp	xzr, xzr, [sp, #112]
	stp	xzr, xzr, [sp, #96]
	stp	xzr, xzr, [sp, #80]
	stp	x19, x19, [sp, #64]
	mov	x22, x10
	str	x3, [sp, #24]           // 8-byte Folded Spill
	bl	__do_page_cache_readahead
	cmp	w21, #1                 // =1
	stp	x21, x22, [sp, #32]     // 16-byte Folded Spill
	b.lt	.LBB6_42
// %bb.23:                              //   in Loop: Header=BB6_22 Depth=1
	mov	w23, w21
	b	.LBB6_25
.LBB6_24:                               //   in Loop: Header=BB6_25 Depth=2
	subs	w23, w23, #1            // =1
	add	x22, x22, #1            // =1
	b.eq	.LBB6_30
.LBB6_25:                               //   Parent Loop BB6_22 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	mov	x0, x25
	mov	x1, x22
	mov	x2, xzr
	mov	x3, xzr
	bl	read_cache_page
	mov	x27, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB6_44
// %bb.26:                              //   in Loop: Header=BB6_25 Depth=2
	ldr	x28, [x26, :lo12:page_info_slab]
	mov	w1, #3136
	mov	x0, x28
	bl	kmem_cache_alloc
	mov	x21, x0
	cbnz	x0, .LBB6_28
// %bb.27:                              //   in Loop: Header=BB6_25 Depth=2
	mov	w1, #35904
	mov	x0, x28
	bl	kmem_cache_alloc
	mov	x21, x0
.LBB6_28:                               //   in Loop: Header=BB6_25 Depth=2
	stp	x21, x27, [x21, #8]
	str	x21, [x21]
	ldr	x27, [sp, #72]
	add	x2, sp, #64             // =64
	mov	x0, x21
	mov	x1, x27
	bl	__list_add_valid
	tbz	w0, #0, .LBB6_24
// %bb.29:                              //   in Loop: Header=BB6_25 Depth=2
	str	x21, [sp, #72]
	stp	x19, x27, [x21]
	str	x21, [x27]
	b	.LBB6_24
.LBB6_30:                               //   in Loop: Header=BB6_22 Depth=1
	ldp	x8, x28, [sp, #32]      // 16-byte Folded Reload
	mov	w23, w8
	b	.LBB6_33
.LBB6_31:                               //   in Loop: Header=BB6_33 Depth=2
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
.LBB6_32:                               //   in Loop: Header=BB6_33 Depth=2
	subs	w23, w23, #1            // =1
	add	x28, x28, #1            // =1
	b.eq	.LBB6_42
.LBB6_33:                               //   Parent Loop BB6_22 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	mov	w2, #2
	mov	x0, x25
	mov	x1, x28
	mov	w3, wzr
	bl	pagecache_get_page
	cbz	x0, .LBB6_43
// %bb.34:                              //   in Loop: Header=BB6_33 Depth=2
	ldr	x27, [sp, #56]          // 8-byte Folded Reload
	mov	x22, x0
	mov	x1, x28
	ldr	x0, [x27, #1304]
	bl	f2fs_is_hole_blkaddr
	tbnz	w0, #0, .LBB6_31
// %bb.35:                              //   in Loop: Header=BB6_33 Depth=2
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	mov	w1, #5
	mov	w2, #1
	mov	w3, wzr
	bl	f2fs_do_map_lock
	mov	w21, w0
	add	x0, sp, #80             // =80
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	add	x0, sp, #80             // =80
	mov	x1, x28
	stp	x27, xzr, [sp, #80]
	str	xzr, [sp, #96]
	str	wzr, [sp, #104]
	bl	f2fs_get_block
	ldr	x8, [sp, #96]
	mov	w27, w0
	cbz	x8, .LBB6_37
// %bb.36:                              //   in Loop: Header=BB6_33 Depth=2
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_put_page
	ldr	x9, [sp, #96]
	ldr	x8, [sp, #88]
	cbnz	x8, .LBB6_38
	b	.LBB6_41
.LBB6_37:                               //   in Loop: Header=BB6_33 Depth=2
	mov	x9, xzr
	ldr	x8, [sp, #88]
	cbz	x8, .LBB6_41
.LBB6_38:                               //   in Loop: Header=BB6_33 Depth=2
	cmp	x9, x8
	b.eq	.LBB6_41
// %bb.39:                              //   in Loop: Header=BB6_33 Depth=2
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
	cbnz	w9, .LBB6_41
// %bb.40:                              //   in Loop: Header=BB6_33 Depth=2
	bl	__put_page
.LBB6_41:                               //   in Loop: Header=BB6_33 Depth=2
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	and	w3, w21, #0x1
	mov	w1, #5
	mov	w2, wzr
	stp	xzr, xzr, [sp, #88]
	bl	f2fs_do_map_lock
	mov	x0, x22
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
	cbz	w27, .LBB6_32
	b	.LBB6_44
.LBB6_42:                               //   in Loop: Header=BB6_22 Depth=1
	mov	w27, wzr
	b	.LBB6_44
.LBB6_43:                               //   in Loop: Header=BB6_22 Depth=1
	mov	w27, #-12
.LBB6_44:                               //   in Loop: Header=BB6_22 Depth=1
	ldr	x21, [sp, #64]
	mov	w22, #1
	cmp	x19, x21
	b.ne	.LBB6_48
.LBB6_45:                               //   in Loop: Header=BB6_22 Depth=1
	tbnz	w27, #31, .LBB6_54
// %bb.46:                              //   in Loop: Header=BB6_22 Depth=1
	ldr	x22, [sp, #56]          // 8-byte Folded Reload
	ldr	x0, [x22, #48]
	bl	filemap_fdatawrite
	ldp	x8, x10, [sp, #32]      // 16-byte Folded Reload
	subs	w20, w20, w8
	ldr	x8, [sp, #24]           // 8-byte Folded Reload
	add	x10, x10, x8
	b.ne	.LBB6_22
	b	.LBB6_55
.LBB6_47:                               //   in Loop: Header=BB6_48 Depth=2
	add	x8, x24, #34            // =34
	stp	x24, x8, [x21]
	ldr	x0, [x26, :lo12:page_info_slab]
	mov	x1, x21
	bl	kmem_cache_free
	ldr	x21, [sp, #64]
	cmp	x19, x21
	b.eq	.LBB6_45
.LBB6_48:                               //   Parent Loop BB6_22 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	x8, [x21, #16]
	cbz	x8, .LBB6_51
// %bb.49:                              //   in Loop: Header=BB6_48 Depth=2
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w22
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB6_51
// %bb.50:                              //   in Loop: Header=BB6_48 Depth=2
	bl	__put_page
.LBB6_51:                               //   in Loop: Header=BB6_48 Depth=2
	mov	x0, x21
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB6_47
// %bb.52:                              //   in Loop: Header=BB6_48 Depth=2
	ldp	x9, x8, [x21]
	str	x8, [x9, #8]
	str	x9, [x8]
	b	.LBB6_47
.LBB6_53:
	ldr	x22, [sp, #56]          // 8-byte Folded Reload
	mov	w27, wzr
	b	.LBB6_55
.LBB6_54:
	ldr	x22, [sp, #56]          // 8-byte Folded Reload
.LBB6_55:
	ldr	x0, [x22, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w21, w0
	orr	w8, w0, w27
	cbz	w8, .LBB6_57
// %bb.56:
	ldr	x3, [x22, #64]
	ldr	x0, [sp]                // 8-byte Folded Reload
	adrp	x1, .L.str.16
	adrp	x2, .L__func__.__revoke_deduped_data
	add	x1, x1, :lo12:.L.str.16
	add	x2, x2, :lo12:.L__func__.__revoke_deduped_data
	mov	w4, w27
	mov	w5, w21
	bl	f2fs_printk
.LBB6_57:
	ldr	x8, [x22, #40]
	cmp	w27, #0                 // =0
	csel	w22, w21, w27, eq
	ldr	x24, [x8, #1008]
	add	x23, x24, #368          // =368
	mov	x0, x23
	bl	_raw_spin_lock
	ldr	x8, [x24, #352]
	cbz	w22, .LBB6_67
// %bb.58:
	ldr	w27, [sp, #16]          // 4-byte Folded Reload
	cbz	x8, .LBB6_60
// %bb.59:
	adrp	x19, jiffies
	ldr	x8, [x24, #360]
	ldr	x9, [x19, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB6_98
.LBB6_60:
	mov	x0, x23
	bl	_raw_spin_unlock
	add	x21, x24, #304          // =304
	mov	x0, x21
	bl	down_read
	ldr	x26, [sp, #56]          // 8-byte Folded Reload
	mov	w1, #41
	mov	x0, x26
	bl	f2fs_truncate_dedup_inode
	ldr	w1, [x26, #64]
	mov	x0, x24
	bl	f2fs_remove_orphan_inode
	ldr	x8, [x24, #224]
	mov	x0, x21
	ldr	x8, [x8]
	str	x8, [x26, #1336]
	bl	up_read
	adrp	x8, __tracepoint_f2fs_dedup_revoke_fail+8
	ldr	x21, [x26, #1304]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_dedup_revoke_fail+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB6_79
// %bb.61:
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
	ldr	x24, [sp, #8]           // 8-byte Folded Reload
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB6_17
// %bb.62:
	//APP
	mrs x19, sp_el0
	//NO_APP
	ldr	w8, [x19, #24]
	add	w8, w8, #1              // =1
	str	w8, [x19, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_dedup_revoke_fail+32
	ldr	x20, [x8, :lo12:__tracepoint_f2fs_dedup_revoke_fail+32]
	cbz	x20, .LBB6_64
.LBB6_63:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x20]
	mov	x1, x26
	mov	x2, x21
	blr	x8
	ldr	x8, [x20, #24]!
	cbnz	x8, .LBB6_63
.LBB6_64:
	//APP
	//NO_APP
	ldr	x8, [x19, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x19, #24]
	b.eq	.LBB6_66
// %bb.65:
	ldr	x8, [x19, #24]
	cbnz	x8, .LBB6_17
.LBB6_66:
	bl	preempt_schedule_notrace
	b	.LBB6_17
.LBB6_67:
	ldr	w27, [sp, #16]          // 4-byte Folded Reload
	cbz	x8, .LBB6_69
// %bb.68:
	adrp	x19, jiffies
	ldr	x8, [x24, #360]
	ldr	x9, [x19, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB6_99
.LBB6_69:
	mov	x0, x23
	bl	_raw_spin_unlock
	add	x25, x24, #304          // =304
	mov	x0, x25
	bl	down_read
	ldr	x26, [sp, #56]          // 8-byte Folded Reload
	mov	x0, x24
	ldr	w1, [x26, #64]
	bl	f2fs_remove_orphan_inode
	add	x21, x26, #744          // =744
	mov	x0, x21
	bl	down_write
	add	x19, x26, #736          // =736
	mov	x8, #2199023255552
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x9, [x19]
	bic	x9, x9, x8
	stxr	w10, x9, [x19]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x26
	bl	f2fs_mark_inode_dirty_sync
	mov	x8, #274877906944
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x9, [x19]
	bic	x9, x9, x8
	stxr	w10, x9, [x19]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x26
	bl	f2fs_mark_inode_dirty_sync
	mov	x8, #8796093022208
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x9, [x19]
	bic	x9, x9, x8
	stxr	w10, x9, [x19]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x26
	bl	f2fs_mark_inode_dirty_sync
	mov	x8, #17592186044416
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x19]
1:	ldxr	x9, [x19]
	bic	x9, x9, x8
	stxr	w10, x9, [x19]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x26
	bl	f2fs_mark_inode_dirty_sync
	ldr	x23, [x26, #1304]
	str	xzr, [x26, #1304]
	ldr	x8, [x24, #224]
	mov	x0, x21
	ldr	x8, [x8]
	str	x8, [x26, #1336]
	bl	up_write
	mov	x0, x24
	bl	f2fs_acquire_orphan_inode
	mov	w22, w0
	cbz	w0, .LBB6_80
// %bb.70:
	cbz	x24, .LBB6_75
// %bb.71:
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
	cbnz	w8, .LBB6_75
// %bb.72:
	ldr	x1, [x24, #3392]
	cbz	x1, .LBB6_75
// %bb.73:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB6_75
// %bb.74:
	add	x2, x24, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB6_75:
	adrp	x19, f2fs_dclient
	ldr	x0, [x19, :lo12:f2fs_dclient]
	cbz	x0, .LBB6_78
// %bb.76:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB6_78
// %bb.77:
	ldr	x0, [x19, :lo12:f2fs_dclient]
	adrp	x1, .L.str.17
	adrp	x2, .L__func__.__revoke_deduped_inode_end
	add	x1, x1, :lo12:.L.str.17
	add	x2, x2, :lo12:.L__func__.__revoke_deduped_inode_end
	mov	w3, #343
	bl	dsm_client_record
	ldr	x0, [x19, :lo12:f2fs_dclient]
	mov	w1, #15244
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB6_78:
	ldr	x3, [x23, #64]
	adrp	x1, .L.str.18
	adrp	x2, .L__func__.__revoke_deduped_inode_end
	add	x1, x1, :lo12:.L.str.18
	add	x2, x2, :lo12:.L__func__.__revoke_deduped_inode_end
	mov	x0, x24
	bl	f2fs_printk
	b	.LBB6_84
.LBB6_79:
	ldr	x24, [sp, #8]           // 8-byte Folded Reload
	b	.LBB6_17
.LBB6_80:
	ldr	x8, [x23, #40]
	add	x24, x23, #744          // =744
	mov	x0, x24
	ldr	x21, [x8, #1008]
	bl	down_write
	mov	x0, x23
	bl	drop_nlink
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x24
	bl	up_write
	ldr	w8, [x23, #72]
	cbz	w8, .LBB6_83
// %bb.81:
	mov	x0, x21
	bl	f2fs_release_orphan_inode
	b	.LBB6_84
.LBB6_82:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x23, #360]
	ldr	x2, [x19, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x23, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB6_15
.LBB6_83:
	mov	x0, x23
	bl	f2fs_add_orphan_inode
.LBB6_84:
	ldr	x24, [sp, #8]           // 8-byte Folded Reload
	mov	x0, x25
	bl	up_read
	adrp	x8, __tracepoint_f2fs_dedup_revoke_inode+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_dedup_revoke_inode+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB6_90
// %bb.85:
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
	tbz	w8, #0, .LBB6_90
// %bb.86:
	//APP
	mrs x19, sp_el0
	//NO_APP
	ldr	w8, [x19, #24]
	add	w8, w8, #1              // =1
	str	w8, [x19, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_dedup_revoke_inode+32
	ldr	x20, [x8, :lo12:__tracepoint_f2fs_dedup_revoke_inode+32]
	cbz	x20, .LBB6_88
.LBB6_87:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x20]
	mov	x1, x26
	mov	x2, x23
	blr	x8
	ldr	x8, [x20, #24]!
	cbnz	x8, .LBB6_87
.LBB6_88:
	//APP
	//NO_APP
	ldr	x8, [x19, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x19, #24]
	b.eq	.LBB6_101
// %bb.89:
	ldr	x8, [x19, #24]
	cbz	x8, .LBB6_101
.LBB6_90:
	ldr	w8, [x23, #72]
	cbnz	w8, .LBB6_97
.LBB6_91:
	ldr	x8, [x23, #704]
	mov	x0, x23
	and	x8, x8, #0xffffffffffffffef
	str	x8, [x23, #704]
	bl	f2fs_set_inode_flags
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_mark_inode_dirty_sync
	ldr	w8, [x23, #1300]
	cbz	w8, .LBB6_96
// %bb.92:
	add	x0, sp, #80             // =80
	mov	w1, wzr
	stp	xzr, xzr, [sp, #104]
	stp	xzr, xzr, [sp, #88]
	str	xzr, [sp, #80]
	bl	init_wait_entry
	add	x21, x23, #1312         // =1312
.LBB6_93:                               // =>This Inner Loop Header: Depth=1
	add	x1, sp, #80             // =80
	mov	w2, #2
	mov	x0, x21
	bl	prepare_to_wait_event
	ldr	w8, [x23, #1300]
	cbz	w8, .LBB6_95
// %bb.94:                              //   in Loop: Header=BB6_93 Depth=1
	bl	schedule
	b	.LBB6_93
.LBB6_95:
	add	x1, sp, #80             // =80
	mov	x0, x21
	bl	finish_wait
.LBB6_96:
	add	x21, x26, #992          // =992
	mov	x0, x21
	bl	down_write
	mov	x0, x26
	bl	inode_dio_wait
	mov	x0, x21
	bl	up_write
.LBB6_97:
	mov	x0, x23
	bl	iput
	b	.LBB6_17
.LBB6_98:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x24, #360]
	ldr	x2, [x19, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x24, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB6_60
.LBB6_99:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x24, #360]
	ldr	x2, [x19, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x24, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB6_69
.LBB6_100:
	mov	w22, #-5
	b	.LBB6_3
.LBB6_101:
	bl	preempt_schedule_notrace
	ldr	w8, [x23, #72]
	cbnz	w8, .LBB6_97
	b	.LBB6_91
.LBB6_102:
	bl	__stack_chk_fail
.Lfunc_end6:
	.size	f2fs_revoke_deduped_inode, .Lfunc_end6-f2fs_revoke_deduped_inode
                                        // -- End function
	.globl	f2fs_inode_support_dedup // -- Begin function f2fs_inode_support_dedup
	.p2align	2
	.type	f2fs_inode_support_dedup,@function
f2fs_inode_support_dedup:               // @f2fs_inode_support_dedup
.Lf2fs_inode_support_dedup$local:
// %bb.0:
	ldr	x8, [x0, #16]
	ldrb	w8, [x8, #2181]
	tbnz	w8, #7, .LBB7_2
// %bb.1:
	mov	w0, wzr
	ret
.LBB7_2:
	ldr	x8, [x1, #736]
	tbnz	w8, #26, .LBB7_4
// %bb.3:
	mov	w0, wzr
	ret
.LBB7_4:
	ldrsw	x8, [x1, #1184]
	add	x8, x8, #360            // =360
	cmp	x8, #399                // =399
	cset	w0, hi
	ret
.Lfunc_end7:
	.size	f2fs_inode_support_dedup, .Lfunc_end7-f2fs_inode_support_dedup
                                        // -- End function
	.globl	f2fs_truncate_dedup_inode // -- Begin function f2fs_truncate_dedup_inode
	.p2align	2
	.type	f2fs_truncate_dedup_inode,@function
f2fs_truncate_dedup_inode:              // @f2fs_truncate_dedup_inode
.Lf2fs_truncate_dedup_inode$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #736]
	mov	x19, x0
	mov	x29, sp
                                        // kill: def $w1 killed $w1 def $x1
	tbnz	x8, #38, .LBB8_7
.LBB8_1:
	ldr	x8, [x19, #40]
	ldr	x2, [x19, #64]
	adrp	x1, .L.str.7
	add	x1, x1, :lo12:.L.str.7
	ldr	x0, [x8, #1008]
	bl	f2fs_printk
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB8_4
// %bb.2:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB8_4
// %bb.3:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.f2fs_truncate_dedup_inode
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.f2fs_truncate_dedup_inode
	mov	w3, #1107
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB8_4:
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
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/file.c"; .popsection; .long 14472b - 14470b; .short 1107; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB8_6
// %bb.5:
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
	cbz	w9, .LBB8_15
.LBB8_6:
	mov	w20, wzr
	b	.LBB8_31
.LBB8_7:
	ldr	x8, [x19, #736]
	tbz	x8, #38, .LBB8_9
// %bb.8:
	ldr	x8, [x19, #736]
	tbnz	x8, #39, .LBB8_1
.LBB8_9:
	add	x8, x19, #736           // =736
	lsr	w9, w1, #6
	mov	w10, #1
	add	x8, x8, w9, uxtw #3
	lsl	x9, x10, x1
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	cmp	w1, #44                 // =44
	b.hi	.LBB8_12
// %bb.10:
	mov	w8, w1
	mov	w9, #1
	lsl	x8, x9, x8
	mov	x9, #3585
	movk	x9, #4144, lsl #16
	movk	x9, #7880, lsl #32
	tst	x8, x9
	b.eq	.LBB8_12
// %bb.11:
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_mark_inode_dirty_sync
.LBB8_12:
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB8_18
// %bb.13:
	ldr	w8, [x19, #1280]
	cbnz	w8, .LBB8_20
// %bb.14:
	ldr	x8, [x19, #736]
	tbz	x8, #35, .LBB8_19
	b	.LBB8_20
.LBB8_15:
	ldr	x1, [x8, #3392]
	mov	w20, wzr
	cbz	x1, .LBB8_31
// %bb.16:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB8_31
// %bb.17:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	mov	w20, wzr
	b	.LBB8_31
.LBB8_18:
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB8_20
.LBB8_19:
	ldr	xzr, [x19, #736]
.LBB8_20:
	mov	x0, x19
	mov	x1, xzr
	mov	w2, wzr
	bl	f2fs_do_truncate_blocks
	mov	w20, w0
	cbnz	w0, .LBB8_22
// %bb.21:
	mov	w1, #-3
	mov	x0, x19
	bl	f2fs_set_inode_addr
	mov	w20, w0
	cbz	w0, .LBB8_31
.LBB8_22:
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB8_27
// %bb.23:
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
	cbnz	w9, .LBB8_27
// %bb.24:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB8_27
// %bb.25:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB8_27
// %bb.26:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB8_27:
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	cbz	x0, .LBB8_30
// %bb.28:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB8_30
// %bb.29:
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.17
	adrp	x2, .L__func__._truncate_error_handle
	add	x1, x1, :lo12:.L.str.17
	add	x2, x2, :lo12:.L__func__._truncate_error_handle
	mov	w3, #1094
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w8, #15241
	movk	w8, #14160, lsl #16
	add	w1, w8, #3              // =3
	bl	dsm_client_notify
.LBB8_30:
	ldr	x8, [x19, #40]
	ldr	x2, [x19, #64]
	adrp	x1, .L.str.28
	add	x1, x1, :lo12:.L.str.28
	ldr	x0, [x8, #1008]
	mov	w3, w20
	bl	f2fs_printk
.LBB8_31:
	mov	w0, w20
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end8:
	.size	f2fs_truncate_dedup_inode, .Lfunc_end8-f2fs_truncate_dedup_inode
                                        // -- End function
	.globl	f2fs_truncate_blocks    // -- Begin function f2fs_truncate_blocks
	.p2align	2
	.type	f2fs_truncate_blocks,@function
f2fs_truncate_blocks:                   // @f2fs_truncate_blocks
.Lf2fs_truncate_blocks$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #16]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	mov	w19, w2
	mov	x20, x0
	mov	x21, x1
	ldr	x8, [x8, #1008]
	mov	x29, sp
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB9_4
// %bb.1:
	ldr	w8, [x20, #1280]
	cbnz	w8, .LBB9_6
// %bb.2:
	ldr	x8, [x20, #736]
	tbnz	x8, #35, .LBB9_6
// %bb.3:
	ldr	x8, [x20, #736]
	mov	x22, x21
	tbnz	x8, #34, .LBB9_6
	b	.LBB9_7
.LBB9_4:
	ldrh	w8, [x20]
	mov	x22, x21
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB9_7
// %bb.5:
	ldr	x8, [x20, #736]
	mov	x22, x21
	tbz	w8, #31, .LBB9_7
.LBB9_6:
	ldr	w8, [x20, #1292]
	sub	x9, x21, #1             // =1
	lsl	w8, w8, #12
	sub	w8, w8, #1              // =1
	orr	x8, x9, x8
	add	x22, x8, #1             // =1
.LBB9_7:
	and	w2, w19, #0x1
	mov	x0, x20
	mov	x1, x22
	bl	f2fs_do_truncate_blocks
	cbnz	w0, .LBB9_11
// %bb.8:
	cmp	x22, x21
	b.eq	.LBB9_10
// %bb.9:
	and	w2, w19, #0x1
	mov	x0, x20
	mov	x1, x21
	bl	f2fs_truncate_partial_cluster
	cbnz	w0, .LBB9_11
.LBB9_10:
	mov	w0, wzr
.LBB9_11:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end9:
	.size	f2fs_truncate_blocks, .Lfunc_end9-f2fs_truncate_blocks
                                        // -- End function
	.globl	f2fs_sync_file          // -- Begin function f2fs_sync_file
	.p2align	2
	.type	f2fs_sync_file,@function
f2fs_sync_file:                         // @f2fs_sync_file
.Lf2fs_sync_file$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #32]
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	ldr	x9, [x8, #40]
	ldr	x9, [x9, #1008]
	ldr	x9, [x9, #224]
	ldrb	w9, [x9, #132]
	tbnz	w9, #3, .LBB10_3
// %bb.1:
	add	x23, x8, #160           // =160
	mov	x21, x0
	mov	x0, x23
	mov	w19, w3
	mov	x20, x2
	mov	x22, x1
	bl	down_write
	mov	x0, x21
	mov	x1, x22
	mov	x2, x20
	mov	w3, w19
	mov	w4, wzr
	bl	f2fs_do_sync_file
	mov	w19, w0
	mov	x0, x23
	bl	up_write
.LBB10_2:
	mov	w0, w19
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB10_3:
	mov	w19, #-5
	b	.LBB10_2
.Lfunc_end10:
	.size	f2fs_sync_file, .Lfunc_end10-f2fs_sync_file
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_do_sync_file
	.type	f2fs_do_sync_file,@function
f2fs_do_sync_file:                      // @f2fs_do_sync_file
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
	ldr	x8, [x0, #248]
	mov	w9, #1
	ldr	x20, [x8]
	ldr	x8, [x20, #40]
	ldr	x28, [x20, #64]
	ldr	x19, [x8, #1008]
	mov	x8, #9223372036854775807
	stp	x9, xzr, [sp, #80]
	stp	xzr, xzr, [sp, #136]
	stp	xzr, xzr, [sp, #120]
	stp	xzr, xzr, [sp, #104]
	str	xzr, [sp, #96]
	stp	xzr, xzr, [sp, #64]
	stp	x8, xzr, [sp, #48]
	str	wzr, [sp, #44]
	ldr	x8, [x20, #40]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB11_97
// %bb.1:
	ldr	x8, [x19, #80]
	tbnz	w8, #8, .LBB11_97
// %bb.2:
	adrp	x8, __tracepoint_f2fs_sync_file_enter+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_sync_file_enter+8]
	mov	w25, w4
	mov	w21, w3
	mov	x22, x2
	mov	x23, x0
	mov	x24, x1
	cmp	w8, #1                  // =1
	b.lt	.LBB11_8
// %bb.3:
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
	tbz	w8, #0, .LBB11_8
// %bb.4:
	//APP
	mrs x26, sp_el0
	//NO_APP
	ldr	w8, [x26, #24]
	add	w8, w8, #1              // =1
	str	w8, [x26, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_sync_file_enter+32
	ldr	x27, [x8, :lo12:__tracepoint_f2fs_sync_file_enter+32]
	cbz	x27, .LBB11_6
.LBB11_5:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x27]
	mov	x1, x20
	blr	x8
	ldr	x8, [x27, #24]!
	cbnz	x8, .LBB11_5
.LBB11_6:
	//APP
	//NO_APP
	ldr	x8, [x26, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x26, #24]
	b.eq	.LBB11_132
// %bb.7:
	ldr	x8, [x26, #24]
	cbz	x8, .LBB11_132
.LBB11_8:
	ldrh	w8, [x20]
	adrp	x26, __tracepoint_f2fs_sync_file_exit+8
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.ne	.LBB11_10
// %bb.9:
	str	xzr, [sp, #32]          // 8-byte Folded Spill
	mov	x24, xzr
	b	.LBB11_48
.LBB11_10:
	bl	sched_clock
	str	x0, [sp, #32]           // 8-byte Folded Spill
	cbz	w21, .LBB11_20
.LBB11_11:
	add	x8, x20, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 16384
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB11_12:
	mov	x0, x23
	mov	x1, x24
	mov	x2, x22
	mov	x27, x19
	mov	w19, w25
	mov	x25, x28
	bl	file_write_and_wait_range
	mov	w28, w0
	bl	sched_clock
	add	x8, x20, #736           // =736
	mov	w9, #16384
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	cbz	w28, .LBB11_26
// %bb.13:
	ldr	w8, [x26, :lo12:__tracepoint_f2fs_sync_file_exit+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB11_98
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
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB11_98
// %bb.15:
	//APP
	mrs x19, sp_el0
	//NO_APP
	ldr	w8, [x19, #24]
	add	w8, w8, #1              // =1
	str	w8, [x19, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_sync_file_exit+32
	ldr	x22, [x8, :lo12:__tracepoint_f2fs_sync_file_exit+32]
	cbz	x22, .LBB11_17
.LBB11_16:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x22]
	mov	x1, x20
	mov	w2, wzr
	mov	w3, w21
	mov	w4, w28
	blr	x8
	ldr	x8, [x22, #24]!
	cbnz	x8, .LBB11_16
.LBB11_17:
	//APP
	//NO_APP
	ldr	x8, [x19, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x19, #24]
	b.eq	.LBB11_19
// %bb.18:
	ldr	x8, [x19, #24]
	cbnz	x8, .LBB11_98
.LBB11_19:
	bl	preempt_schedule_notrace
	b	.LBB11_98
.LBB11_20:
	ldr	w8, [x20, #792]
	ldr	x9, [x19, #136]
	ldr	w9, [x9, #144]
	cmp	w8, w9
	b.hi	.LBB11_12
// %bb.21:
	adrp	x8, write_opt
	ldr	w8, [x8, :lo12:write_opt]
	cbz	w8, .LBB11_11
// %bb.22:
	ldr	w9, [x20, #1184]
	ldr	w10, [x20, #1192]
	ldrh	w11, [x20]
	ldr	x8, [x20, #80]
	mov	w12, #923
	add	w9, w10, w9, asr #2
	and	w10, w11, #0xf000
	cmp	w10, #8, lsl #12        // =32768
	sub	w9, w12, w9
	b.ne	.LBB11_25
// %bb.23:
	ldr	x10, [x20, #736]
	tbz	w10, #31, .LBB11_25
// %bb.24:
	ldr	w10, [x20, #1292]
	neg	w10, w10
	and	w9, w9, w10
.LBB11_25:
	mov	w9, w9
	cmp	x8, x9, lsl #12
	b.gt	.LBB11_11
	b	.LBB11_12
.LBB11_26:
	mov	x24, x0
	cbz	w21, .LBB11_34
// %bb.27:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	add	x22, x8, #1024          // =1024
	mov	x0, x22
	bl	_raw_spin_lock
	ldr	x23, [x20, #896]
	mov	x0, x22
	mov	x26, x24
	add	x24, x20, #896          // =896
	bl	_raw_spin_unlock
	cmp	x23, x24
	mov	x24, x26
	mov	x28, x25
	b.ne	.LBB11_46
.LBB11_28:
	ldr	x8, [x20, #736]
	mov	w25, w19
	tbnz	w8, #12, .LBB11_47
// %bb.29:
	mov	w2, #1
	mov	x0, x27
	mov	w1, w28
	mov	x19, x27
	bl	f2fs_exist_written_data
	tbnz	w0, #0, .LBB11_48
// %bb.30:
	ldr	x8, [x19, #128]
	and	x1, x28, #0xffffffff
	mov	w2, wzr
	mov	w3, wzr
	ldr	x0, [x8, #48]
	bl	pagecache_get_page
	mov	x22, x0
	cbz	x0, .LBB11_32
// %bb.31:
	ldr	x8, [x22, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x22, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #3, .LBB11_33
.LBB11_32:
	mov	x0, x19
	mov	w1, w28
	bl	f2fs_need_inode_block_update
	tbz	w0, #0, .LBB11_111
.LBB11_33:
	mov	w23, #1
	b	.LBB11_112
.LBB11_34:
	ldr	x8, [x20, #736]
	mov	x28, x25
	tbz	w8, #2, .LBB11_46
// %bb.35:
	ldrb	w8, [x20, #712]
	tbnz	w8, #4, .LBB11_46
// %bb.36:
	ldrh	w8, [x20, #80]
	tst	w8, #0xfff
	b.ne	.LBB11_46
// %bb.37:
	ldr	x8, [x20, #1216]
	ldr	x9, [x20, #88]
	cmp	x8, x9
	b.ne	.LBB11_46
// %bb.38:
	ldr	x8, [x20, #1224]
	ldr	x9, [x20, #96]
	cmp	x8, x9
	b.ne	.LBB11_46
// %bb.39:
	ldr	x8, [x20, #1232]
	ldr	x9, [x20, #120]
	cmp	x8, x9
	b.ne	.LBB11_46
// %bb.40:
	ldr	x8, [x20, #1240]
	ldr	x9, [x20, #128]
	cmp	x8, x9
	b.ne	.LBB11_46
// %bb.41:
	ldr	x8, [x20, #1248]
	ldr	x9, [x20, #104]
	cmp	x8, x9
	b.ne	.LBB11_46
// %bb.42:
	ldr	x8, [x20, #1256]
	ldr	x9, [x20, #112]
	cmp	x8, x9
	b.ne	.LBB11_46
// %bb.43:
	ldr	x8, [x20, #1264]
	ldr	x9, [x20, #1200]
	cmp	x8, x9
	b.ne	.LBB11_46
// %bb.44:
	ldr	x8, [x20, #1272]
	ldr	x9, [x20, #1208]
	cmp	x8, x9
	b.ne	.LBB11_46
// %bb.45:
	add	x22, x20, #840          // =840
	mov	x0, x22
	bl	_raw_spin_lock
	ldr	x23, [x20, #832]
	mov	x25, x24
	ldr	x24, [x20, #80]
	mov	x0, x22
	bl	_raw_spin_unlock
	cmp	x23, x24
	mov	x24, x25
	b.eq	.LBB11_28
.LBB11_46:
	mov	x0, x20
	mov	x1, xzr
	bl	f2fs_write_inode
	mov	w25, w19
.LBB11_47:
	mov	x19, x27
.LBB11_48:
	add	x26, x20, #744          // =744
	mov	x0, x26
	bl	down_read
	ldrh	w8, [x20]
	str	x24, [sp, #24]          // 8-byte Folded Spill
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB11_54
// %bb.49:
	ldr	x8, [x20, #40]
	ldr	x22, [x8, #1008]
	ldr	w8, [x22, #1532]
	cbz	w8, .LBB11_72
// %bb.50:
	ldr	w8, [x20, #1280]
	cbnz	w8, .LBB11_53
// %bb.51:
	ldr	x8, [x20, #736]
	tbnz	x8, #35, .LBB11_53
// %bb.52:
	ldr	x8, [x20, #736]
	tbz	x8, #34, .LBB11_73
.LBB11_53:
	mov	w22, #2
	b	.LBB11_55
.LBB11_54:
	mov	w22, #1
.LBB11_55:
	mov	x0, x26
	bl	up_read
	bl	sched_clock
	ldr	x8, [x20, #40]
	str	x0, [sp, #16]           // 8-byte Folded Spill
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_sync_fs
	mov	w28, w0
	bl	sched_clock
	str	x0, [sp, #8]            // 8-byte Folded Spill
	mov	x0, x26
	bl	down_write
	ldrb	w8, [x20, #712]
	tbz	w8, #1, .LBB11_59
// %bb.56:
	ldr	w8, [x20, #72]
	cmp	w8, #1                  // =1
	b.ne	.LBB11_59
// %bb.57:
	mov	x0, x20
	bl	d_find_alias
	cbz	x0, .LBB11_59
// %bb.58:
	add	x27, x0, #88            // =88
	mov	x25, x0
	mov	x0, x27
	bl	_raw_spin_lock
	ldr	x8, [x25, #24]
	mov	x0, x27
	ldr	x8, [x8, #48]
	ldr	x24, [x8, #64]
	bl	_raw_spin_unlock
	mov	x0, x25
	bl	dput
	mov	w1, #1
	mov	x0, x20
	str	w24, [x20, #728]
	bl	f2fs_mark_inode_dirty_sync
	ldrb	w8, [x20, #712]
	mov	w1, #1
	mov	x0, x20
	and	w8, w8, #0xfffffffd
	strb	w8, [x20, #712]
	bl	f2fs_mark_inode_dirty_sync
.LBB11_59:
	mov	x0, x26
	bl	up_write
	add	x8, x20, #736           // =736
	mov	w9, #4096
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w9, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	x25, xzr
	mov	x27, xzr
	mov	x23, xzr
	mov	x26, xzr
.LBB11_60:
	adrp	x8, __tracepoint_f2fs_sync_file_exit+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_sync_file_exit+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB11_66
// %bb.61:
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
	tbz	w8, #0, .LBB11_66
// %bb.62:
	str	x19, [sp]               // 8-byte Folded Spill
	//APP
	mrs x24, sp_el0
	//NO_APP
	ldr	w8, [x24, #24]
	mov	x19, x27
	mov	x27, x26
	mov	x26, x23
	add	w8, w8, #1              // =1
	str	w8, [x24, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_sync_file_exit+32
	mov	x23, x25
	ldr	x25, [x8, :lo12:__tracepoint_f2fs_sync_file_exit+32]
	cbz	x25, .LBB11_64
.LBB11_63:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x25]
	mov	x1, x20
	mov	w2, w22
	mov	w3, w21
	mov	w4, w28
	blr	x8
	ldr	x8, [x25, #24]!
	cbnz	x8, .LBB11_63
.LBB11_64:
	//APP
	//NO_APP
	ldr	x8, [x24, #24]
	mov	x25, x23
	mov	x23, x26
	mov	x26, x27
	subs	x8, x8, #1              // =1
	str	w8, [x24, #24]
	mov	x27, x19
	ldr	x19, [sp]               // 8-byte Folded Reload
	b.eq	.LBB11_142
// %bb.65:
	ldr	x8, [x24, #24]
	cbz	x8, .LBB11_142
.LBB11_66:
	cbnz	w28, .LBB11_98
.LBB11_67:
	ldr	x8, [sp, #32]           // 8-byte Folded Reload
	cbz	x8, .LBB11_98
// %bb.68:
	bl	sched_clock
	add	x21, x19, #2296         // =2296
	mov	x22, x0
	mov	x0, x21
	bl	mutex_lock
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB11_75
// %bb.69:
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB11_77
// %bb.70:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB11_77
// %bb.71:
	ldr	w9, [x8, #216]
	add	w9, w9, #1              // =1
	str	w9, [x8, #216]
	b	.LBB11_77
.LBB11_72:
	ldr	x8, [x20, #736]
	tbnz	w8, #31, .LBB11_53
.LBB11_73:
	ldr	x8, [x22, #224]
	ldr	x9, [x20, #1336]
	ldr	x8, [x8]
	cmp	x9, x8
	b.ne	.LBB11_100
// %bb.74:
	mov	w22, #11
	b	.LBB11_55
.LBB11_75:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB11_77
// %bb.76:
	ldr	w9, [x8, #220]
	add	w9, w9, #1              // =1
	str	w9, [x8, #220]
.LBB11_77:
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB11_87
// %bb.78:
	ldr	x9, [sp, #32]           // 8-byte Folded Reload
	ldr	x10, [x8, #224]
	sub	x9, x22, x9
	add	x10, x9, x10
	str	x10, [x8, #224]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB11_87
// %bb.79:
	ldr	x10, [x8, #232]
	cmp	x10, x9
	b.hs	.LBB11_81
// %bb.80:
	str	x9, [x8, #232]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB11_87
.LBB11_81:
	ldp	x11, x9, [sp, #24]      // 16-byte Folded Reload
	ldr	x10, [x8, #256]
	sub	x9, x11, x9
	add	x10, x9, x10
	str	x10, [x8, #256]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB11_87
// %bb.82:
	ldr	x10, [x8, #264]
	cmp	x10, x9
	b.hs	.LBB11_84
// %bb.83:
	str	x9, [x8, #264]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB11_87
.LBB11_84:
	ldp	x11, x9, [sp, #8]       // 16-byte Folded Reload
	ldr	x10, [x8, #240]
	sub	x9, x11, x9
	add	x10, x9, x10
	str	x10, [x8, #240]
	ldr	x8, [x19, #2328]
	cbz	x8, .LBB11_87
// %bb.85:
	ldr	x10, [x8, #248]
	cmp	x10, x9
	b.hs	.LBB11_87
// %bb.86:
	str	x9, [x8, #248]
.LBB11_87:
	cbz	x27, .LBB11_92
// %bb.88:
	ldr	x9, [x19, #2328]
	cbz	x9, .LBB11_96
// %bb.89:
	ldr	x10, [x9, #272]
	sub	x8, x27, x25
	add	x10, x8, x10
	str	x10, [x9, #272]
	ldr	x9, [x19, #2328]
	cbz	x9, .LBB11_96
// %bb.90:
	ldr	x10, [x9, #280]
	cmp	x10, x8
	b.hs	.LBB11_92
// %bb.91:
	str	x8, [x9, #280]
.LBB11_92:
	ldr	x9, [x19, #2328]
	cbz	x9, .LBB11_96
// %bb.93:
	ldr	x10, [x9, #288]
	sub	x8, x26, x23
	add	x10, x8, x10
	str	x10, [x9, #288]
	ldr	x9, [x19, #2328]
	cbz	x9, .LBB11_96
// %bb.94:
	ldr	x10, [x9, #296]
	cmp	x10, x8
	b.hs	.LBB11_96
// %bb.95:
	str	x8, [x9, #296]
.LBB11_96:
	mov	x0, x21
	bl	mutex_unlock
.LBB11_97:
	mov	w28, wzr
.LBB11_98:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB11_148
// %bb.99:
	mov	w0, w28
	ldp	x20, x19, [sp, #240]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #224]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #208]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #192]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #176]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #160]    // 16-byte Folded Reload
	add	sp, sp, #256            // =256
	ret
.LBB11_100:
	ldr	w8, [x20, #72]
	cmp	w8, #1                  // =1
	b.ne	.LBB11_108
// %bb.101:
	ldr	x8, [x22, #80]
	tbnz	w8, #5, .LBB11_109
// %bb.102:
	ldrb	w8, [x20, #712]
	tbnz	w8, #1, .LBB11_110
// %bb.103:
	mov	x0, x22
	bl	f2fs_space_for_roll_forward
	tbz	w0, #0, .LBB11_119
// %bb.104:
	ldr	w1, [x20, #728]
	mov	x0, x22
	bl	f2fs_is_checkpointed_node
	tbz	w0, #0, .LBB11_125
// %bb.105:
	ldrb	w8, [x22, #1425]
	tbnz	w8, #4, .LBB11_131
// %bb.106:
	ldr	w8, [x22, #1448]
	cmp	w8, #2                  // =2
	b.ne	.LBB11_133
// %bb.107:
	mov	w22, #9
	b	.LBB11_55
.LBB11_108:
	mov	w22, #3
	b	.LBB11_55
.LBB11_109:
	mov	w22, #4
	b	.LBB11_55
.LBB11_110:
	mov	w22, #5
	b	.LBB11_55
.LBB11_111:
	mov	w23, wzr
.LBB11_112:
	cbz	x22, .LBB11_115
// %bb.113:
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
	cbnz	w10, .LBB11_115
// %bb.114:
	bl	__put_page
.LBB11_115:
	tbnz	w23, #0, .LBB11_48
// %bb.116:
	ldr	x8, [x20, #736]
	str	x24, [sp, #24]          // 8-byte Folded Spill
	tbnz	w8, #13, .LBB11_120
// %bb.117:
	mov	w2, #2
	mov	x0, x19
	mov	w1, w28
	bl	f2fs_exist_written_data
	mov	x26, xzr
	tbz	w0, #0, .LBB11_126
// %bb.118:
	mov	x24, x26
	mov	x27, x26
	str	x28, [sp, #16]          // 8-byte Folded Spill
	tbz	w25, #0, .LBB11_121
	b	.LBB11_122
.LBB11_119:
	mov	w22, #6
	b	.LBB11_55
.LBB11_120:
	mov	x24, xzr
	mov	x27, xzr
	str	x28, [sp, #16]          // 8-byte Folded Spill
	tbnz	w25, #0, .LBB11_122
.LBB11_121:
	ldr	w8, [x19, #1492]
	cmp	w8, #2                  // =2
	b.ne	.LBB11_127
.LBB11_122:
	adrp	x22, delayflush
	adrp	x8, current_flush_merge
	ldr	w9, [x22, :lo12:delayflush]
	ldr	x8, [x8, :lo12:current_flush_merge]
	add	w9, w9, #1              // =1
	sub	x8, x8, #1              // =1
	cmp	x8, w9, sxtw
	str	w9, [x22, :lo12:delayflush]
	b.hs	.LBB11_124
// %bb.123:
	bl	sched_clock
	ldr	w1, [x20, #64]
	mov	x23, x0
	mov	x0, x19
	bl	f2fs_issue_flush
	mov	w28, w0
	bl	sched_clock
	mov	x26, x0
	mov	w8, #1
	str	w8, [x22, :lo12:delayflush]
	b	.LBB11_128
.LBB11_124:
	mov	x26, xzr
	mov	x23, xzr
	mov	x25, x24
	b	.LBB11_129
.LBB11_125:
	mov	w22, #7
	b	.LBB11_55
.LBB11_126:
	mov	w28, wzr
	mov	w22, wzr
	stp	x26, x26, [sp, #8]      // 16-byte Folded Spill
	mov	x25, x26
	mov	x27, x26
	mov	x23, x26
	b	.LBB11_60
.LBB11_127:
	bl	sched_clock
	ldr	w1, [x20, #64]
	mov	x23, x0
	mov	x0, x19
	bl	f2fs_issue_flush
	mov	w28, w0
	bl	sched_clock
	mov	x26, x0
.LBB11_128:
	mov	x25, x24
	cbnz	w28, .LBB11_130
.LBB11_129:
	ldr	x22, [sp, #16]          // 8-byte Folded Reload
	mov	w2, #2
	mov	x0, x19
	mov	w1, w22
	bl	f2fs_remove_ino_entry
	add	x8, x20, #736           // =736
	mov	w9, #8192
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w2, #4
	mov	x0, x19
	mov	w1, w22
	bl	f2fs_remove_ino_entry
	mov	w28, wzr
.LBB11_130:
	stp	xzr, xzr, [sp, #8]      // 16-byte Folded Spill
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	mov	w22, wzr
	stp	x8, x8, [x19, #504]
	str	x8, [x19, #520]
	b	.LBB11_60
.LBB11_131:
	mov	w22, #8
	b	.LBB11_55
.LBB11_132:
	bl	preempt_schedule_notrace
	b	.LBB11_8
.LBB11_133:
	ldr	w8, [x22, #1492]
	mov	x27, x28
	cmp	w8, #1                  // =1
	b.ne	.LBB11_137
// %bb.134:
	ldr	w1, [x20, #64]
	mov	x0, x22
	bl	f2fs_need_dentry_mark
	cbz	w0, .LBB11_137
// %bb.135:
	ldr	w1, [x20, #728]
	mov	w2, #3
	mov	x0, x22
	bl	f2fs_exist_written_data
	tbz	w0, #0, .LBB11_137
// %bb.136:
	mov	w22, #10
	b	.LBB11_55
.LBB11_137:
	mov	x0, x26
	bl	up_read
	add	x22, x19, #1380         // =1380
	mov	w23, #1
.LBB11_138:                             // =>This Inner Loop Header: Depth=1
	bl	sched_clock
	//APP
	// atomic_add
	prfm	pstl1strm, [x22]
1:	ldxr	w8, [x22]
	add	w8, w8, 1
	stxr	w9, w8, [x22]
	cbnz	w9, 1b

	//NO_APP
	mov	x24, x0
	and	w3, w25, #0x1
	add	x2, sp, #48             // =48
	add	x4, sp, #44             // =44
	mov	x0, x19
	mov	x1, x20
	bl	f2fs_fsync_node_pages
	//APP
	// atomic_sub
	prfm	pstl1strm, [x22]
1:	ldxr	w8, [x22]
	sub	w8, w8, w23
	stxr	w9, w8, [x22]
	cbnz	w9, 1b

	//NO_APP
	cbnz	w0, .LBB11_145
// %bb.139:                             //   in Loop: Header=BB11_138 Depth=1
	ldr	x8, [x19, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB11_147
// %bb.140:                             //   in Loop: Header=BB11_138 Depth=1
	mov	x0, x19
	mov	w1, w27
	bl	f2fs_need_inode_block_update
	tbz	w0, #0, .LBB11_143
// %bb.141:                             //   in Loop: Header=BB11_138 Depth=1
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x20
	mov	x1, xzr
	bl	f2fs_write_inode
	b	.LBB11_138
.LBB11_142:
	bl	preempt_schedule_notrace
	cbz	w28, .LBB11_67
	b	.LBB11_98
.LBB11_143:
	tbnz	w25, #0, .LBB11_146
// %bb.144:
	ldr	w1, [sp, #44]
	mov	x0, x19
	bl	f2fs_wait_on_node_pages_writeback
	cbz	w0, .LBB11_146
.LBB11_145:
	mov	w28, w0
	mov	w22, wzr
	stp	xzr, xzr, [sp, #8]      // 16-byte Folded Spill
	mov	x27, xzr
	mov	x23, xzr
	mov	x26, xzr
	mov	x25, x24
	b	.LBB11_60
.LBB11_146:
	mov	x28, x27
	mov	w2, #1
	mov	x0, x19
	mov	w1, w28
	bl	f2fs_remove_ino_entry
	add	x8, x20, #736           // =736
	mov	w9, #4096
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	bl	sched_clock
	mov	x27, x0
	str	x28, [sp, #16]          // 8-byte Folded Spill
	tbz	w25, #0, .LBB11_121
	b	.LBB11_122
.LBB11_147:
	mov	w22, wzr
	stp	xzr, xzr, [sp, #8]      // 16-byte Folded Spill
	mov	x27, xzr
	mov	x23, xzr
	mov	x26, xzr
	mov	w28, #-5
	mov	x25, x24
	b	.LBB11_60
.LBB11_148:
	bl	__stack_chk_fail
.Lfunc_end11:
	.size	f2fs_do_sync_file, .Lfunc_end11-f2fs_do_sync_file
                                        // -- End function
	.globl	f2fs_truncate_data_blocks_range // -- Begin function f2fs_truncate_data_blocks_range
	.p2align	2
	.type	f2fs_truncate_data_blocks_range,@function
f2fs_truncate_data_blocks_range:        // @f2fs_truncate_data_blocks_range
.Lf2fs_truncate_data_blocks_range$local:
// %bb.0:
	sub	sp, sp, #112            // =112
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #32]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #48]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #64]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #80]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #96]     // 16-byte Folded Spill
	ldr	x26, [x0]
	ldr	w23, [x0, #28]
	mov	x12, #2097152
	movk	x12, #1, lsl #32
	ldr	x9, [x26, #40]
	ldr	w8, [x26, #1292]
	mov	x10, #-549755813888
	mov	x11, #-549755813888
	ldr	x21, [x9, #1008]
	ldr	w9, [x26, #1280]
	movk	x10, #4072
	movk	x11, #4076
	mov	x20, x0
	str	w9, [sp, #8]            // 4-byte Folded Spill
	ldr	x9, [x0, #16]
	mov	w22, w1
	add	x29, sp, #16            // =16
	add	x9, x9, x12
	lsl	x9, x9, #6
	and	x9, x9, #0xfffffffffffff000
	ldr	w10, [x9, x10]
	ldr	w11, [x9, x11]
	cmp	w10, w11
	b.ne	.LBB12_2
// %bb.1:
	ldr	x12, [x26, #736]
	tbnz	w12, #26, .LBB12_43
.LBB12_2:
	mov	x12, xzr
	cmp	w22, #1                 // =1
	b.lt	.LBB12_44
.LBB12_3:
	mov	x13, #-549755813888
	sub	w28, w8, #1             // =1
	add	x8, x9, x13
	cmp	w10, w11
	add	x9, x8, #360            // =360
	csel	x8, x9, x8, eq
	sxtw	x14, w23
	add	x8, x8, x12, lsl #2
	mov	w24, wzr
	mov	w27, wzr
	mov	w19, wzr
	str	x23, [sp]               // 8-byte Folded Spill
	add	x23, x8, x14, lsl #2
	mov	x0, x21
	stur	wzr, [x29, #-4]         // 4-byte Folded Spill
.LBB12_4:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB12_22 Depth 2
	ldr	w8, [x0, #1532]
	ldr	w25, [x23]
	cbz	w8, .LBB12_8
// %bb.5:                               //   in Loop: Header=BB12_4 Depth=1
	ldr	w8, [x26, #1280]
	cbnz	w8, .LBB12_10
// %bb.6:                               //   in Loop: Header=BB12_4 Depth=1
	ldr	x8, [x26, #736]
	tbnz	x8, #35, .LBB12_10
// %bb.7:                               //   in Loop: Header=BB12_4 Depth=1
	ldr	x8, [x26, #736]
	tbnz	x8, #34, .LBB12_10
	b	.LBB12_11
.LBB12_8:                               //   in Loop: Header=BB12_4 Depth=1
	ldrh	w8, [x26]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB12_11
// %bb.9:                               //   in Loop: Header=BB12_4 Depth=1
	ldr	x8, [x26, #736]
	tbz	w8, #31, .LBB12_11
.LBB12_10:                              //   in Loop: Header=BB12_4 Depth=1
	tst	w24, w28
	b.eq	.LBB12_28
.LBB12_11:                              //   in Loop: Header=BB12_4 Depth=1
	cbz	w25, .LBB12_34
.LBB12_12:                              //   in Loop: Header=BB12_4 Depth=1
	mov	x0, x20
	str	wzr, [x20, #36]
	bl	f2fs_set_data_blkaddr
	add	w8, w25, #2             // =2
	cmp	w8, #2                  // =2
	b.hs	.LBB12_17
.LBB12_13:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	w8, [x20, #28]
	cbnz	w8, .LBB12_16
// %bb.14:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	x8, [x20, #16]
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	mov	x10, #-549755813888
	add	x8, x8, x9
	lsl	x8, x8, #6
	mov	x9, #-549755813888
	and	x8, x8, #0xfffffffffffff000
	movk	x9, #4072
	movk	x10, #4076
	ldr	w9, [x8, x9]
	ldr	w8, [x8, x10]
	cmp	w9, w8
	b.ne	.LBB12_16
// %bb.15:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	x8, [x20]
	mov	w11, #262144
	add	x8, x8, #736            // =736
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x11
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB12_16:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	w8, [sp, #8]            // 4-byte Folded Reload
	mov	x0, x21
	mov	w1, w25
	cmp	w8, #0                  // =0
	cset	w26, ne
	bl	f2fs_invalidate_blocks
	cmn	w25, #2                 // =2
	cset	w8, ne
	orr	w8, w26, w8
	add	w19, w19, w8
	b	.LBB12_34
.LBB12_17:                              //   in Loop: Header=BB12_4 Depth=1
	cmn	w25, #3                 // =3
	b.eq	.LBB12_34
// %bb.18:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	w9, [x21, #3128]
	cbz	w9, .LBB12_26
// %bb.19:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	x8, [x21]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB12_25
// %bb.20:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	x10, [x21, #3136]
	add	x10, x10, #80           // =80
	b	.LBB12_22
.LBB12_21:                              //   in Loop: Header=BB12_22 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB12_25
.LBB12_22:                              //   Parent Loop BB12_4 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	w11, w25
	b.hi	.LBB12_21
// %bb.23:                              //   in Loop: Header=BB12_22 Depth=2
	ldr	w11, [x10]
	cmp	w11, w25
	b.lo	.LBB12_21
// %bb.24:                              //   in Loop: Header=BB12_4 Depth=1
	ldur	x8, [x10, #-80]
.LBB12_25:                              //   in Loop: Header=BB12_4 Depth=1
	cbz	x8, .LBB12_13
.LBB12_26:                              //   in Loop: Header=BB12_4 Depth=1
	mov	w2, #7
	mov	x0, x21
	mov	w1, w25
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB12_34
// %bb.27:                              //   in Loop: Header=BB12_4 Depth=1
	ldur	w9, [x29, #-4]          // 4-byte Folded Reload
	and	w8, w27, #0x1
	add	w9, w9, w8
	stur	w9, [x29, #-4]          // 4-byte Folded Spill
	b	.LBB12_13
.LBB12_28:                              //   in Loop: Header=BB12_4 Depth=1
	tbz	w27, #0, .LBB12_33
// %bb.29:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	w8, [x26, #1292]
	ldr	w9, [x26, #1280]
	cbz	w9, .LBB12_33
// %bb.30:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	w9, [x26, #1280]
	ldur	w10, [x29, #-4]         // 4-byte Folded Reload
	sub	w27, w8, w10
	cmp	w27, w9
	b.le	.LBB12_32
// %bb.31:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	w4, [x26, #1280]
	ldr	x5, [x26, #64]
	adrp	x1, .L.str.29
	adrp	x2, .L__func__.f2fs_i_compr_blocks_update
	add	x1, x1, :lo12:.L.str.29
	add	x2, x2, :lo12:.L__func__.f2fs_i_compr_blocks_update
	mov	w3, w27
	bl	f2fs_printk
.LBB12_32:                              //   in Loop: Header=BB12_4 Depth=1
	add	x8, x26, #1280          // =1280
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w27
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x26, #40]
	sxtw	x9, w27
	mov	w1, #1
	mov	x0, x26
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
.LBB12_33:                              //   in Loop: Header=BB12_4 Depth=1
	cmn	w25, #2                 // =2
	cset	w27, eq
	stur	wzr, [x29, #-4]         // 4-byte Folded Spill
	cbnz	w25, .LBB12_12
.LBB12_34:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	w8, [x20, #28]
	add	w24, w24, #1            // =1
	cmp	w24, w22
	add	w8, w8, #1              // =1
	str	w8, [x20, #28]
	b.eq	.LBB12_36
// %bb.35:                              //   in Loop: Header=BB12_4 Depth=1
	ldr	x26, [x20]
	add	x23, x23, #4            // =4
	ldr	x8, [x26, #40]
	ldr	x0, [x8, #1008]
	b	.LBB12_4
.LBB12_36:
	tbz	w27, #0, .LBB12_41
// %bb.37:
	ldr	x23, [x20]
	ldr	w8, [x23, #1292]
	ldr	w9, [x23, #1280]
	cbz	w9, .LBB12_41
// %bb.38:
	ldr	w9, [x23, #1280]
	ldur	w10, [x29, #-4]         // 4-byte Folded Reload
	sub	w24, w8, w10
	cmp	w24, w9
	b.le	.LBB12_40
// %bb.39:
	ldr	x8, [x23, #40]
	adrp	x1, .L.str.29
	adrp	x2, .L__func__.f2fs_i_compr_blocks_update
	add	x1, x1, :lo12:.L.str.29
	ldr	x0, [x8, #1008]
	ldr	w4, [x23, #1280]
	ldr	x5, [x23, #64]
	add	x2, x2, :lo12:.L__func__.f2fs_i_compr_blocks_update
	mov	w3, w24
	bl	f2fs_printk
.LBB12_40:
	add	x8, x23, #1280          // =1280
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w24
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x23, #40]
	sxtw	x9, w24
	mov	w1, #1
	mov	x0, x23
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
.LBB12_41:
	cbz	w19, .LBB12_45
// %bb.42:
	ldr	x8, [x20, #16]
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	ldr	x1, [x20]
	add	x8, x8, x9
	lsl	x8, x8, #6
	mov	x9, #-549755813888
	and	x8, x8, #0xfffffffffffff000
	movk	x9, #4080
	ldr	w8, [x8, x9]
	lsr	w0, w8, #3
	bl	f2fs_start_bidx_of_node
	ldr	x23, [sp]               // 8-byte Folded Reload
	mov	w2, wzr
	mov	w3, w22
	add	w1, w0, w23
	mov	x0, x20
	bl	f2fs_update_extent_cache_range
	ldr	x1, [x20]
	mov	x0, x21
	mov	w2, w19
	bl	dec_valid_block_count
	b	.LBB12_46
.LBB12_43:
	ldr	w12, [x26, #1184]
	sbfx	x12, x12, #2, #30
	cmp	w22, #1                 // =1
	b.ge	.LBB12_3
.LBB12_44:
	mov	w19, wzr
	b	.LBB12_46
.LBB12_45:
	ldr	x23, [sp]               // 8-byte Folded Reload
.LBB12_46:
	str	w23, [x20, #28]
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	adrp	x9, __tracepoint_f2fs_truncate_data_blocks_range+8
	stp	x8, x8, [x21, #504]
	str	x8, [x21, #520]
	ldr	x21, [x20]
	ldp	w22, w20, [x20, #24]
	ldr	w8, [x9, :lo12:__tracepoint_f2fs_truncate_data_blocks_range+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB12_52
// %bb.47:
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
	tbz	w8, #0, .LBB12_52
// %bb.48:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w8, [x23, #24]
	add	w8, w8, #1              // =1
	str	w8, [x23, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_truncate_data_blocks_range+32
	ldr	x24, [x8, :lo12:__tracepoint_f2fs_truncate_data_blocks_range+32]
	cbz	x24, .LBB12_50
.LBB12_49:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x24]
	mov	x1, x21
	mov	w2, w22
	mov	w3, w20
	mov	w4, w19
	blr	x8
	ldr	x8, [x24, #24]!
	cbnz	x8, .LBB12_49
.LBB12_50:
	//APP
	//NO_APP
	ldr	x8, [x23, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x23, #24]
	b.eq	.LBB12_53
// %bb.51:
	ldr	x8, [x23, #24]
	cbz	x8, .LBB12_53
.LBB12_52:
	ldp	x20, x19, [sp, #96]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #80]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #64]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #48]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #112            // =112
	ret
.LBB12_53:
	bl	preempt_schedule_notrace
	b	.LBB12_52
.Lfunc_end12:
	.size	f2fs_truncate_data_blocks_range, .Lfunc_end12-f2fs_truncate_data_blocks_range
                                        // -- End function
	.p2align	2               // -- Begin function dec_valid_block_count
	.type	dec_valid_block_count,@function
dec_valid_block_count:                  // @dec_valid_block_count
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	add	x23, x0, #2340          // =2340
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	mov	x20, x0
	mov	x0, x23
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	mov	w22, w2
	mov	x19, x1
	lsl	w21, w2, #3
	bl	_raw_spin_lock
	ldr	w8, [x20, #1240]
	cmp	w8, w22
	b.lo	.LBB13_9
.LBB13_1:
	ldr	w9, [x20, #1240]
	ldr	w8, [x20, #1252]
	sub	w9, w9, w22
	str	w9, [x20, #1240]
	cbz	w8, .LBB13_4
// %bb.2:
	ldr	w9, [x20, #1256]
	cmp	w9, w8
	b.hs	.LBB13_4
// %bb.3:
	add	w9, w9, w22
	cmp	w8, w9
	csel	w8, w8, w9, lo
	str	w8, [x20, #1256]
.LBB13_4:
	mov	x0, x23
	bl	_raw_spin_unlock
	ldr	x3, [x19, #144]
	cmp	x3, x21
	b.lo	.LBB13_17
// %bb.5:
	ldr	x21, [x19, #736]
	ldr	x20, [x19, #736]
	ldrb	w8, [x19, #142]
	mov	w9, w22
	mov	x0, x19
	mov	w2, wzr
	lsl	x1, x9, x8
	bl	__dquot_free_space
	mov	w1, #1
	mov	x0, x19
	bl	__mark_inode_dirty
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_mark_inode_dirty_sync
	tbz	w21, #1, .LBB13_7
// %bb.6:
	tbz	w20, #2, .LBB13_8
.LBB13_7:
	add	x8, x19, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB13_8:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB13_9:
	bl	print_bdev_access_info
	adrp	x24, f2fs_dclient
	ldr	x0, [x24, :lo12:f2fs_dclient]
	cbz	x0, .LBB13_12
// %bb.10:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB13_12
// %bb.11:
	ldr	x0, [x24, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.dec_valid_block_count
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.dec_valid_block_count
	mov	w3, #3473
	bl	dsm_client_record
	ldr	x0, [x24, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB13_12:
	mov	x0, x20
	bl	f2fs_print_raw_sb_info
	mov	x0, x20
	bl	f2fs_print_ckpt_info
	mov	x0, x20
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/f2fs.h"; .popsection; .long 14472b - 14470b; .short 3473; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x20, .LBB13_1
// %bb.13:
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
	cbnz	w8, .LBB13_1
// %bb.14:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB13_1
// %bb.15:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB13_1
// %bb.16:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB13_1
.LBB13_17:
	ldr	x2, [x19, #64]
	adrp	x1, .L.str.30
	add	x1, x1, :lo12:.L.str.30
	mov	x0, x20
	mov	x4, x21
	bl	f2fs_printk
	cbz	x20, .LBB13_8
// %bb.18:
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
	cbnz	w8, .LBB13_8
// %bb.19:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB13_8
// %bb.20:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB13_8
// %bb.21:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB13_8
.Lfunc_end13:
	.size	dec_valid_block_count, .Lfunc_end13-dec_valid_block_count
                                        // -- End function
	.globl	f2fs_truncate_data_blocks // -- Begin function f2fs_truncate_data_blocks
	.p2align	2
	.type	f2fs_truncate_data_blocks,@function
f2fs_truncate_data_blocks:              // @f2fs_truncate_data_blocks
.Lf2fs_truncate_data_blocks$local:
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x8, [x0]
	mov	x29, sp
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB14_3
// %bb.1:
	ldr	x9, [x8, #736]
	mov	w1, #1018
	tbz	w9, #31, .LBB14_4
// %bb.2:
	ldr	w8, [x8, #1292]
	neg	w8, w8
	and	w1, w8, w1
	b	.LBB14_4
.LBB14_3:
	mov	w1, #1018
.LBB14_4:
	bl	f2fs_truncate_data_blocks_range
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end14:
	.size	f2fs_truncate_data_blocks, .Lfunc_end14-f2fs_truncate_data_blocks
                                        // -- End function
	.globl	f2fs_do_truncate_blocks // -- Begin function f2fs_do_truncate_blocks
	.p2align	2
	.type	f2fs_do_truncate_blocks,@function
f2fs_do_truncate_blocks:                // @f2fs_do_truncate_blocks
.Lf2fs_do_truncate_blocks$local:
// %bb.0:
	sub	sp, sp, #160            // =160
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #64]     // 16-byte Folded Spill
	add	x29, sp, #64            // =64
	stp	x28, x27, [sp, #80]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #96]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #112]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #128]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #144]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x8, [x0, #40]
	adrp	x9, __tracepoint_f2fs_truncate_blocks_enter+8
	adrp	x27, cpu_number
	adrp	x26, __cpu_online_mask
	ldr	x22, [x8, #1008]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	ldr	w8, [x9, :lo12:__tracepoint_f2fs_truncate_blocks_enter+8]
	mov	w21, w2
	mov	x20, x1
	mov	x19, x0
	cmp	w8, #1                  // =1
	add	x27, x27, :lo12:cpu_number
	add	x26, x26, :lo12:__cpu_online_mask
	b.lt	.LBB15_6
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
	ldr	w8, [x8, x27]
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x26, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB15_6
// %bb.2:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w8, [x23, #24]
	add	w8, w8, #1              // =1
	str	w8, [x23, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_truncate_blocks_enter+32
	ldr	x24, [x8, :lo12:__tracepoint_f2fs_truncate_blocks_enter+32]
	cbz	x24, .LBB15_4
.LBB15_3:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x24]
	mov	x1, x19
	mov	x2, x20
	blr	x8
	ldr	x8, [x24, #24]!
	cbnz	x8, .LBB15_3
.LBB15_4:
	//APP
	//NO_APP
	ldr	x8, [x23, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x23, #24]
	b.eq	.LBB15_51
// %bb.5:
	ldr	x8, [x23, #24]
	cbz	x8, .LBB15_51
.LBB15_6:
	add	x8, x20, #4095          // =4095
	mov	x0, x19
	lsr	x23, x8, #12
	bl	max_file_blocks
	cmp	x23, x0
	b.hs	.LBB15_19
// %bb.7:
	tbz	w21, #0, .LBB15_11
// %bb.8:
	add	x24, x22, #368          // =368
	mov	x0, x24
	bl	_raw_spin_lock
	ldr	x8, [x22, #352]
	cbz	x8, .LBB15_10
// %bb.9:
	adrp	x25, jiffies
	ldr	x8, [x22, #360]
	ldr	x9, [x25, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB15_50
.LBB15_10:
	mov	x0, x24
	bl	_raw_spin_unlock
	add	x0, x22, #304           // =304
	bl	down_read
.LBB15_11:
	ldr	x1, [x19, #64]
	mov	x0, x22
	bl	f2fs_get_node_page
	mov	x24, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB15_49
// %bb.12:
	ldr	x8, [x19, #736]
	tbnz	w8, #10, .LBB15_20
// %bb.13:
	add	x0, sp, #8              // =8
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	add	x0, sp, #8              // =8
	mov	w2, #2
	mov	x1, x23
	stp	x19, x24, [sp, #8]
	str	xzr, [sp, #24]
	str	wzr, [sp, #32]
	bl	f2fs_get_dnode_of_data
	cmn	w0, #2                  // =2
	b.eq	.LBB15_48
// %bb.14:
	mov	w24, w0
	mov	w25, wzr
	cbnz	w0, .LBB15_21
// %bb.15:
	ldr	x8, [sp, #24]
	mov	x25, #2097152
	movk	x25, #1, lsl #32
	mov	x9, #-549755813888
	add	x8, x8, x25
	mov	x10, #-549755813888
	lsl	x8, x8, #6
	movk	x9, #4072
	movk	x10, #4076
	and	x8, x8, #0xfffffffffffff000
	ldr	w9, [x8, x9]
	ldr	w8, [x8, x10]
	cmp	w9, w8
	b.ne	.LBB15_33
// %bb.16:
	ldr	w8, [x19, #1184]
	ldr	w9, [x19, #1192]
	ldrh	w10, [x19]
	mov	w11, #923
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	sub	w8, w11, w8
	b.ne	.LBB15_37
// %bb.17:
	ldr	x9, [x19, #736]
	tbz	w9, #31, .LBB15_37
// %bb.18:
	ldr	w9, [x19, #1292]
	neg	w9, w9
	and	w8, w8, w9
	b	.LBB15_37
.LBB15_19:
	mov	w25, wzr
	b	.LBB15_24
.LBB15_20:
	mov	x0, x19
	mov	x1, x24
	mov	x2, x20
	bl	f2fs_truncate_inline_inode
	mov	w1, #1
	mov	x0, x24
	mov	w25, #1
	bl	f2fs_put_page
	mov	w24, wzr
.LBB15_21:
	tbz	w21, #0, .LBB15_23
.LBB15_22:
	add	x0, x22, #304           // =304
	bl	up_read
.LBB15_23:
	cbnz	w24, .LBB15_25
.LBB15_24:
	mov	x0, x19
	mov	x1, x20
	mov	w2, w25
	bl	truncate_partial_data_page
	mov	w24, w0
.LBB15_25:
	adrp	x8, __tracepoint_f2fs_truncate_blocks_exit+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_truncate_blocks_exit+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB15_31
// %bb.26:
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
	ldr	w8, [x8, x27]
	add	w9, w8, #63             // =63
	cmp	w8, #0                  // =0
	csel	w9, w9, w8, lt
	asr	w9, w9, #6
	ldr	x9, [x26, w9, sxtw #3]
	lsr	x8, x9, x8
	tbz	w8, #0, .LBB15_31
// %bb.27:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_truncate_blocks_exit+32
	ldr	x21, [x8, :lo12:__tracepoint_f2fs_truncate_blocks_exit+32]
	cbz	x21, .LBB15_29
.LBB15_28:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	mov	x1, x19
	mov	w2, w24
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB15_28
.LBB15_29:
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB15_52
// %bb.30:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB15_52
.LBB15_31:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB15_62
// %bb.32:
	mov	w0, w24
	ldp	x20, x19, [sp, #144]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #128]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #112]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #96]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #80]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #64]     // 16-byte Folded Reload
	add	sp, sp, #160            // =160
	ret
.LBB15_33:
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB15_36
// %bb.34:
	ldr	x9, [x19, #736]
	mov	w8, #1018
	tbz	w9, #31, .LBB15_37
// %bb.35:
	ldr	w9, [x19, #1292]
	neg	w9, w9
	and	w8, w9, w8
	b	.LBB15_37
.LBB15_36:
	mov	w8, #1018
.LBB15_37:
	ldr	w9, [sp, #36]
	subs	w24, w8, w9
	b.mi	.LBB15_53
// %bb.38:
	cbnz	w9, .LBB15_40
.LBB15_39:
	ldr	x0, [sp, #24]
	mov	x8, #-549755813888
	mov	x9, #-549755813888
	movk	x8, #4072
	add	x10, x0, x25
	lsl	x10, x10, #6
	movk	x9, #4076
	and	x10, x10, #0xfffffffffffff000
	ldr	w8, [x10, x8]
	ldr	w9, [x10, x9]
	cmp	w8, w9
	b.ne	.LBB15_41
.LBB15_40:
	add	x0, sp, #8              // =8
	mov	w1, w24
	bl	f2fs_truncate_data_blocks_range
	ldr	x0, [sp, #24]
	add	x23, x23, w24, sxtw
.LBB15_41:
	cbz	x0, .LBB15_43
// %bb.42:
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x0, [sp, #24]
.LBB15_43:
	ldr	x8, [sp, #16]
	cbz	x8, .LBB15_47
// %bb.44:
	cmp	x0, x8
	b.eq	.LBB15_47
// %bb.45:
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
	cbnz	w10, .LBB15_47
// %bb.46:
	bl	__put_page
.LBB15_47:
	stp	xzr, xzr, [sp, #16]
.LBB15_48:
	mov	x0, x19
	mov	x1, x23
	bl	f2fs_truncate_inode_blocks
	mov	w24, w0
.LBB15_49:
	mov	w25, wzr
	tbnz	w21, #0, .LBB15_22
	b	.LBB15_23
.LBB15_50:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x22, #360]
	ldr	x2, [x25, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x22, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB15_10
.LBB15_51:
	bl	preempt_schedule_notrace
	b	.LBB15_6
.LBB15_52:
	bl	preempt_schedule_notrace
	b	.LBB15_31
.LBB15_53:
	bl	print_bdev_access_info
	adrp	x28, f2fs_dclient
	ldr	x0, [x28, :lo12:f2fs_dclient]
	cbz	x0, .LBB15_56
// %bb.54:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB15_56
// %bb.55:
	ldr	x0, [x28, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.f2fs_do_truncate_blocks
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.f2fs_do_truncate_blocks
	mov	w3, #2929
	bl	dsm_client_record
	ldr	x0, [x28, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB15_56:
	mov	x0, x22
	bl	f2fs_print_raw_sb_info
	mov	x0, x22
	bl	f2fs_print_ckpt_info
	mov	x0, x22
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/file.c"; .popsection; .long 14472b - 14470b; .short 2929; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x22, .LBB15_61
// %bb.57:
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
	cbnz	w8, .LBB15_61
// %bb.58:
	ldr	x1, [x22, #3392]
	cbz	x1, .LBB15_61
// %bb.59:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB15_61
// %bb.60:
	add	x2, x22, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB15_61:
	ldr	w9, [sp, #36]
	cbnz	w9, .LBB15_40
	b	.LBB15_39
.LBB15_62:
	bl	__stack_chk_fail
.Lfunc_end15:
	.size	f2fs_do_truncate_blocks, .Lfunc_end15-f2fs_do_truncate_blocks
                                        // -- End function
	.p2align	2               // -- Begin function truncate_partial_data_page
	.type	truncate_partial_data_page,@function
truncate_partial_data_page:             // @truncate_partial_data_page
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	mov	x20, x0
	ldr	x0, [x0, #48]
	ands	x22, x1, #0xfff
	mov	w21, w2
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	b.ne	.LBB16_2
// %bb.1:
	tbz	w21, #0, .LBB16_22
.LBB16_2:
	lsr	x1, x1, #12
	tbz	w21, #0, .LBB16_6
// %bb.3:
	mov	w2, #2
	mov	w3, wzr
	bl	pagecache_get_page
	mov	x19, x0
	cbz	x0, .LBB16_21
// %bb.4:
	ldr	x8, [x19, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x19, x9, eq
	ldr	x8, [x8]
	tbz	w8, #2, .LBB16_21
// %bb.5:
	//APP
	dmb ishld
	//NO_APP
	b	.LBB16_8
.LBB16_6:
	mov	w2, #1
	mov	x0, x20
	bl	f2fs_get_lock_data_page
	mov	x19, x0
	cmn	x0, #4095               // =4095
	b.lo	.LBB16_8
// %bb.7:
	cmn	x19, #2                 // =2
	csel	w0, wzr, w19, eq
	b	.LBB16_23
.LBB16_8:
	mov	w2, #1
	mov	w3, #1
	mov	x0, x19
	mov	w1, wzr
	bl	f2fs_wait_on_page_writeback
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w8, [x23, #24]
	mov	x9, #134217728
	movk	x9, #64, lsl #32
	add	x9, x9, x19, lsl #6
	add	w8, w8, #1              // =1
	str	w8, [x23, #24]
	//APP
	//NO_APP
	ldr	w8, [x23, #2800]
	and	x9, x9, #0xfffffffffffff000
	orr	x9, x9, x22
	mov	w1, wzr
	add	w8, w8, #1              // =1
	str	w8, [x23, #2800]
	//APP
	//NO_APP
	mov	w8, #4096
	sub	x2, x8, x22
	mov	x8, #-549755813888
	add	x0, x9, x8
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
	b.eq	.LBB16_24
// %bb.9:
	ldr	x8, [x23, #24]
	cbz	x8, .LBB16_24
.LBB16_10:
	mov	x0, x19
	bl	flush_dcache_page
	tbz	w21, #0, .LBB16_20
// %bb.11:
	ldrb	w8, [x20, #13]
	tbz	w8, #6, .LBB16_21
// %bb.12:
	bl	print_bdev_access_info
	adrp	x21, f2fs_dclient
	ldr	x0, [x21, :lo12:f2fs_dclient]
	cbz	x0, .LBB16_15
// %bb.13:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB16_15
// %bb.14:
	ldr	x0, [x21, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.truncate_partial_data_page
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.truncate_partial_data_page
	mov	w3, #2872
	bl	dsm_client_record
	ldr	x0, [x21, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB16_15:
	ldr	x8, [x20, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_raw_sb_info
	ldr	x8, [x20, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_ckpt_info
	ldr	x8, [x20, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/file.c"; .popsection; .long 14472b - 14470b; .short 2872; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB16_21
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
	cbnz	w9, .LBB16_21
// %bb.17:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB16_21
// %bb.18:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB16_21
// %bb.19:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB16_21
.LBB16_20:
	mov	x0, x19
	bl	set_page_dirty
.LBB16_21:
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_put_page
.LBB16_22:
	mov	w0, wzr
.LBB16_23:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB16_24:
	bl	preempt_schedule
	b	.LBB16_10
.Lfunc_end16:
	.size	truncate_partial_data_page, .Lfunc_end16-truncate_partial_data_page
                                        // -- End function
	.globl	f2fs_truncate           // -- Begin function f2fs_truncate
	.p2align	2
	.type	f2fs_truncate,@function
f2fs_truncate:                          // @f2fs_truncate
.Lf2fs_truncate$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB17_28
// %bb.1:
	ldrh	w8, [x0]
	mov	x19, x0
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB17_4
// %bb.2:
	cmp	w8, #10, lsl #12        // =40960
	b.eq	.LBB17_4
// %bb.3:
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB17_26
.LBB17_4:
	adrp	x8, __tracepoint_f2fs_truncate+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_truncate+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB17_10
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
	tbz	w8, #0, .LBB17_10
// %bb.6:
	//APP
	mrs x20, sp_el0
	//NO_APP
	ldr	w8, [x20, #24]
	add	w8, w8, #1              // =1
	str	w8, [x20, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_truncate+32
	ldr	x21, [x8, :lo12:__tracepoint_f2fs_truncate+32]
	cbz	x21, .LBB17_8
.LBB17_7:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x21]
	mov	x1, x19
	blr	x8
	ldr	x8, [x21, #24]!
	cbnz	x8, .LBB17_7
.LBB17_8:
	//APP
	//NO_APP
	ldr	x8, [x20, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x20, #24]
	b.eq	.LBB17_29
// %bb.9:
	ldr	x8, [x20, #24]
	cbz	x8, .LBB17_29
.LBB17_10:
	mov	x0, x19
	bl	dquot_initialize
	add	w8, w0, #28             // =28
	cmp	w8, #28                 // =28
	b.hi	.LBB17_12
// %bb.11:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB17_13
.LBB17_12:
	cmn	w0, #122                // =122
	b.ne	.LBB17_27
.LBB17_13:
	mov	x0, x19
	bl	f2fs_may_inline_data
	tbnz	w0, #0, .LBB17_15
// %bb.14:
	mov	x0, x19
	bl	f2fs_convert_inline_inode
	cbnz	w0, .LBB17_27
.LBB17_15:
	ldr	x8, [x19, #40]
	ldr	x20, [x19, #80]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB17_19
// %bb.16:
	ldr	w8, [x19, #1280]
	cbnz	w8, .LBB17_21
// %bb.17:
	ldr	x8, [x19, #736]
	tbnz	x8, #35, .LBB17_21
// %bb.18:
	ldr	x8, [x19, #736]
	mov	x21, x20
	tbnz	x8, #34, .LBB17_21
	b	.LBB17_22
.LBB17_19:
	ldrh	w8, [x19]
	mov	x21, x20
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB17_22
// %bb.20:
	ldr	x8, [x19, #736]
	mov	x21, x20
	tbz	w8, #31, .LBB17_22
.LBB17_21:
	ldr	w8, [x19, #1292]
	sub	x9, x20, #1             // =1
	lsl	w8, w8, #12
	sub	w8, w8, #1              // =1
	orr	x8, x9, x8
	add	x21, x8, #1             // =1
.LBB17_22:
	mov	w2, #1
	mov	x0, x19
	mov	x1, x21
	bl	f2fs_do_truncate_blocks
	cbnz	w0, .LBB17_27
// %bb.23:
	cmp	x21, x20
	b.eq	.LBB17_25
// %bb.24:
	mov	w2, #1
	mov	x0, x19
	mov	x1, x20
	bl	f2fs_truncate_partial_cluster
	cbnz	w0, .LBB17_27
.LBB17_25:
	mov	x0, x19
	bl	current_time
	stp	x0, x1, [x19, #120]
	stp	x0, x1, [x19, #104]
	mov	x0, x19
	mov	w1, wzr
	bl	f2fs_mark_inode_dirty_sync
.LBB17_26:
	mov	w0, wzr
.LBB17_27:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB17_28:
	mov	w0, #-5
	b	.LBB17_27
.LBB17_29:
	bl	preempt_schedule_notrace
	b	.LBB17_10
.Lfunc_end17:
	.size	f2fs_truncate, .Lfunc_end17-f2fs_truncate
                                        // -- End function
	.globl	f2fs_getattr            // -- Begin function f2fs_getattr
	.p2align	2
	.type	f2fs_getattr,@function
f2fs_getattr:                           // @f2fs_getattr
.Lf2fs_getattr$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0, #8]
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x19, x1
	mov	x29, sp
	ldr	x20, [x8, #48]
	ldr	x8, [x20, #736]
	tbz	w8, #26, .LBB18_4
// %bb.1:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #0, .LBB18_4
// %bb.2:
	ldrsw	x8, [x20, #1184]
	add	x8, x8, #360            // =360
	cmp	x8, #380                // =380
	b.lo	.LBB18_4
// %bb.3:
	ldr	w8, [x19]
	orr	w8, w8, #0x800
	str	w8, [x19]
	ldr	x8, [x20, #1200]
	str	x8, [x19, #112]
	ldr	x8, [x20, #1208]
	str	x8, [x19, #120]
.LBB18_4:
	ldr	w8, [x20, #704]
	tbnz	w8, #2, .LBB18_24
// %bb.5:
	tbnz	w8, #5, .LBB18_25
.LBB18_6:
	ldrb	w9, [x20, #13]
	tbnz	w9, #6, .LBB18_26
.LBB18_7:
	tbnz	w8, #4, .LBB18_27
.LBB18_8:
	tbnz	w8, #6, .LBB18_28
.LBB18_9:
	ldrb	w8, [x20, #14]
	tbz	w8, #0, .LBB18_11
.LBB18_10:
	ldr	x8, [x19, #16]
	orr	x8, x8, #0x100000
	str	x8, [x19, #16]
.LBB18_11:
	ldr	x8, [x19, #24]
	mov	w9, #2164
	movk	w9, #16, lsl #16
	mov	x0, x20
	orr	x8, x8, x9
	mov	x1, x19
	str	x8, [x19, #24]
	bl	generic_fillattr
	add	x21, x20, #744          // =744
	mov	x0, x21
	bl	down_read
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB18_17
// %bb.12:
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB18_14
// %bb.13:
	ldr	x8, [x20, #736]
	tbnz	x8, #39, .LBB18_17
.LBB18_14:
	ldr	x0, [x20, #1304]
	bl	igrab
	mov	x22, x0
	mov	x0, x21
	bl	up_read
	cbz	x22, .LBB18_19
// %bb.15:
	add	x21, x22, #744          // =744
	mov	x0, x21
	bl	down_read
	ldr	w8, [x22, #72]
	cbz	w8, .LBB18_29
// %bb.16:
	ldr	x9, [x22, #144]
	udiv	x8, x9, x8
	str	x8, [x19, #128]
	b	.LBB18_18
.LBB18_17:
	mov	x22, xzr
.LBB18_18:
	mov	x0, x21
	bl	up_read
.LBB18_19:
	mov	x0, x22
	bl	put_inner_inode
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB18_21
// %bb.20:
	ldr	x8, [x20, #736]
	tbnz	w8, #10, .LBB18_22
.LBB18_21:
	ldr	x8, [x20, #736]
	tbz	w8, #11, .LBB18_23
.LBB18_22:
	ldr	x8, [x19, #56]
	ldr	x9, [x19, #128]
	add	x8, x8, #511            // =511
	add	x8, x9, x8, asr #9
	str	x8, [x19, #128]
.LBB18_23:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	mov	w0, wzr
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB18_24:
	ldr	x9, [x19, #16]
	orr	x9, x9, #0x4
	str	x9, [x19, #16]
	tbz	w8, #5, .LBB18_6
.LBB18_25:
	ldr	x9, [x19, #16]
	orr	x9, x9, #0x20
	str	x9, [x19, #16]
	ldrb	w9, [x20, #13]
	tbz	w9, #6, .LBB18_7
.LBB18_26:
	ldr	x9, [x19, #16]
	orr	x9, x9, #0x800
	str	x9, [x19, #16]
	tbz	w8, #4, .LBB18_8
.LBB18_27:
	ldr	x9, [x19, #16]
	orr	x9, x9, #0x10
	str	x9, [x19, #16]
	tbz	w8, #6, .LBB18_9
.LBB18_28:
	ldr	x8, [x19, #16]
	orr	x8, x8, #0x40
	str	x8, [x19, #16]
	ldrb	w8, [x20, #14]
	tbnz	w8, #0, .LBB18_10
	b	.LBB18_11
.LBB18_29:
	bl	print_bdev_access_info
	adrp	x23, f2fs_dclient
	ldr	x0, [x23, :lo12:f2fs_dclient]
	cbz	x0, .LBB18_32
// %bb.30:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB18_32
// %bb.31:
	ldr	x0, [x23, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.f2fs_getattr
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.f2fs_getattr
	mov	w3, #3084
	bl	dsm_client_record
	ldr	x0, [x23, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB18_32:
	ldr	x8, [x20, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_raw_sb_info
	ldr	x8, [x20, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_ckpt_info
	ldr	x8, [x20, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/file.c"; .popsection; .long 14472b - 14470b; .short 3084; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB18_18
// %bb.33:
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
	cbnz	w9, .LBB18_18
// %bb.34:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB18_18
// %bb.35:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB18_18
// %bb.36:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB18_18
.Lfunc_end18:
	.size	f2fs_getattr, .Lfunc_end18-f2fs_getattr
                                        // -- End function
	.p2align	2               // -- Begin function put_inner_inode
	.type	put_inner_inode,@function
put_inner_inode:                        // @put_inner_inode
// %bb.0:
	cbz	x0, .LBB19_12
// %bb.1:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0, #736]
	mov	x19, x0
	mov	x29, sp
	tbnz	x8, #38, .LBB19_10
.LBB19_2:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB19_5
// %bb.3:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB19_5
// %bb.4:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.put_inner_inode
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.put_inner_inode
	mov	w3, #4583
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB19_5:
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
	cbz	x8, .LBB19_11
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
	cbnz	w9, .LBB19_11
// %bb.7:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB19_11
// %bb.8:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB19_11
// %bb.9:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB19_11
.LBB19_10:
	ldr	x8, [x19, #736]
	tbz	x8, #39, .LBB19_2
.LBB19_11:
	mov	x0, x19
	bl	iput
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
.LBB19_12:
	ret
.Lfunc_end19:
	.size	put_inner_inode, .Lfunc_end19-put_inner_inode
                                        // -- End function
	.globl	f2fs_setattr            // -- Begin function f2fs_setattr
	.p2align	2
	.type	f2fs_setattr,@function
f2fs_setattr:                           // @f2fs_setattr
.Lf2fs_setattr$local:
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x19, [x0, #48]
	mov	x29, sp
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB20_83
// %bb.1:
	ldr	w9, [x19, #12]
	tbnz	w9, #3, .LBB20_82
// %bb.2:
	ldr	w8, [x1]
	mov	x22, x0
	mov	x20, x1
	tbz	w9, #2, .LBB20_4
// %bb.3:
	mov	w9, #7
	movk	w9, #1, lsl #16
	and	w9, w8, w9
	cbnz	w9, .LBB20_82
.LBB20_4:
	tbz	w8, #3, .LBB20_6
// %bb.5:
	mov	x0, x19
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB20_41
.LBB20_6:
	mov	x0, x22
	mov	x1, x20
	bl	setattr_prepare
	mov	w21, w0
	cbnz	w0, .LBB20_42
// %bb.7:
	ldrb	w8, [x20]
	tbz	w8, #3, .LBB20_9
// %bb.8:
	ldr	x23, [x22, #48]
	ldrb	w8, [x23, #13]
	tbnz	w8, #6, .LBB20_43
.LBB20_9:
	mov	x0, x22
	mov	x1, x20
	bl	fsverity_prepare_setattr
	mov	w21, w0
	cbnz	w0, .LBB20_42
// %bb.10:
	ldr	w8, [x20]
	tbnz	w8, #3, .LBB20_15
// %bb.11:
	tbz	w8, #1, .LBB20_13
// %bb.12:
	ldr	w9, [x20, #8]
	ldr	w10, [x19, #4]
	cmp	w9, w10
	b.ne	.LBB20_15
.LBB20_13:
	tbz	w8, #2, .LBB20_18
// %bb.14:
	ldr	w8, [x20, #12]
	ldr	w9, [x19, #8]
	cmp	w8, w9
	b.eq	.LBB20_18
.LBB20_15:
	mov	x0, x19
	bl	dquot_initialize
	add	w8, w0, #28             // =28
	mov	w21, w0
	cmp	w8, #28                 // =28
	b.hi	.LBB20_17
// %bb.16:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB20_18
.LBB20_17:
	cmn	w21, #122               // =122
	b.ne	.LBB20_42
.LBB20_18:
	ldr	w8, [x20]
	tbz	w8, #1, .LBB20_20
// %bb.19:
	ldr	w9, [x20, #8]
	ldr	w10, [x19, #4]
	cmp	w9, w10
	b.ne	.LBB20_22
.LBB20_20:
	tbz	w8, #2, .LBB20_32
// %bb.21:
	ldr	w9, [x20, #12]
	ldr	w10, [x19, #8]
	cmp	w9, w10
	b.eq	.LBB20_32
.LBB20_22:
	ldr	x8, [x19, #40]
	ldr	x22, [x8, #1008]
	add	x21, x22, #368          // =368
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	x8, [x22, #352]
	cbz	x8, .LBB20_24
// %bb.23:
	adrp	x23, jiffies
	ldr	x8, [x22, #360]
	ldr	x9, [x23, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB20_81
.LBB20_24:
	mov	x0, x21
	bl	_raw_spin_unlock
	add	x0, x22, #304           // =304
	bl	down_read
	mov	x0, x19
	mov	x1, x20
	bl	dquot_transfer
	add	w8, w0, #28             // =28
	mov	w21, w0
	cmp	w8, #28                 // =28
	b.hi	.LBB20_26
// %bb.25:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB20_27
.LBB20_26:
	cmn	w21, #122               // =122
	b.ne	.LBB20_78
.LBB20_27:
	ldr	w8, [x20]
	tbz	w8, #1, .LBB20_29
// %bb.28:
	ldr	w8, [x20, #8]
	str	w8, [x19, #4]
	ldr	w8, [x20]
.LBB20_29:
	tbz	w8, #2, .LBB20_31
// %bb.30:
	ldr	w8, [x20, #12]
	str	w8, [x19, #8]
.LBB20_31:
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_mark_inode_dirty_sync
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	add	x0, x8, #304            // =304
	bl	up_read
	ldr	w8, [x20]
.LBB20_32:
	tbnz	w8, #3, .LBB20_46
// %bb.33:
	tbnz	w8, #1, .LBB20_65
.LBB20_34:
	tbnz	w8, #2, .LBB20_66
.LBB20_35:
	tbnz	w8, #4, .LBB20_67
.LBB20_36:
	tbnz	w8, #5, .LBB20_68
.LBB20_37:
	tbnz	w8, #6, .LBB20_69
.LBB20_38:
	tbnz	w8, #0, .LBB20_70
.LBB20_39:
	ldrb	w8, [x20]
	tbnz	w8, #0, .LBB20_73
.LBB20_40:
	mov	w21, wzr
	b	.LBB20_77
.LBB20_41:
	mov	w21, #-95
.LBB20_42:
	mov	w0, w21
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB20_43:
	mov	x0, x23
	bl	fscrypt_get_encryption_info
	mov	w21, w0
	cbnz	w0, .LBB20_42
// %bb.44:
	ldr	x8, [x23, #656]
	cbnz	x8, .LBB20_9
// %bb.45:
	mov	w21, #-126
	b	.LBB20_42
.LBB20_46:
	ldr	w9, [x19, #1192]
	ldr	w10, [x19, #1184]
	ldr	x8, [x20, #16]
	ldr	x24, [x19, #80]
	mvn	w9, w9
	sub	w9, w9, w10, asr #2
	add	w9, w9, #923            // =923
	cmp	x8, w9, sxtw #2
	b.ls	.LBB20_49
// %bb.47:
	mov	x0, x19
	bl	f2fs_convert_inline_inode
	mov	w21, w0
	cbnz	w0, .LBB20_42
// %bb.48:
	ldr	x8, [x20, #16]
.LBB20_49:
	cmp	x8, x24
	b.gt	.LBB20_53
// %bb.50:
	ldr	x9, [x19, #736]
	tbz	x9, #35, .LBB20_53
// %bb.51:
	mov	w1, #10
	mov	x0, x19
	bl	f2fs_reserve_compress_blocks_internal
	mov	w21, w0
	cbnz	w0, .LBB20_42
// %bb.52:
	ldr	x8, [x20, #16]
.LBB20_53:
	cmp	x8, x24
	b.gt	.LBB20_60
// %bb.54:
	ldr	x8, [x19, #736]
	tbz	x8, #38, .LBB20_60
// %bb.55:
	ldr	x8, [x19, #736]
	tbz	x8, #38, .LBB20_57
// %bb.56:
	ldr	x8, [x19, #736]
	tbnz	x8, #39, .LBB20_60
.LBB20_57:
	ldr	x8, [x19, #736]
	tbz	x8, #44, .LBB20_59
// %bb.58:
	add	x8, x19, #736           // =736
	mov	x9, #17592186044416
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
.LBB20_59:
	adrp	x1, .L__func__.f2fs_setattr
	add	x1, x1, :lo12:.L__func__.f2fs_setattr
	mov	x0, x19
	bl	f2fs_revoke_deduped_inode
	mov	w21, w0
	cbnz	w0, .LBB20_42
.LBB20_60:
	add	x22, x19, #1040         // =1040
	mov	x0, x22
	bl	down_write
	add	x23, x19, #1088         // =1088
	mov	x0, x23
	bl	down_write
	ldr	x1, [x20, #16]
	mov	x0, x19
	bl	truncate_setsize
	ldr	x8, [x20, #16]
	cmp	x8, x24
	b.le	.LBB20_62
// %bb.61:
	mov	w21, wzr
	b	.LBB20_63
.LBB20_62:
	mov	x0, x19
	bl	f2fs_truncate
	mov	w21, w0
.LBB20_63:
	mov	x0, x23
	bl	up_write
	mov	x0, x22
	bl	up_write
	cbnz	w21, .LBB20_42
// %bb.64:
	add	x21, x19, #840          // =840
	mov	x0, x21
	bl	_raw_spin_lock
	mov	x0, x19
	bl	current_time
	ldr	x8, [x19, #80]
	stp	x0, x1, [x19, #120]
	stp	x0, x1, [x19, #104]
	mov	x0, x21
	str	x8, [x19, #832]
	bl	_raw_spin_unlock
	ldr	w8, [x20]
	tbz	w8, #1, .LBB20_34
.LBB20_65:
	ldr	w9, [x20, #8]
	str	w9, [x19, #4]
	tbz	w8, #2, .LBB20_35
.LBB20_66:
	ldr	w9, [x20, #12]
	str	w9, [x19, #8]
	tbz	w8, #4, .LBB20_36
.LBB20_67:
	ldp	x10, x9, [x20, #24]
	stp	x10, x9, [x19, #88]
	tbz	w8, #5, .LBB20_37
.LBB20_68:
	ldp	x10, x9, [x20, #40]
	stp	x10, x9, [x19, #104]
	tbz	w8, #6, .LBB20_38
.LBB20_69:
	ldp	x10, x9, [x20, #56]
	stp	x10, x9, [x19, #120]
	tbz	w8, #0, .LBB20_39
.LBB20_70:
	ldr	w0, [x19, #8]
	ldrh	w21, [x20, #4]
	bl	in_group_p
	cbnz	w0, .LBB20_72
// %bb.71:
	mov	w1, #4
	mov	x0, x19
	bl	capable_wrt_inode_uidgid
	and	w8, w21, #0xfffffbff
	tst	w0, #0x1
	csel	w21, w21, w8, ne
.LBB20_72:
	strh	w21, [x19, #732]
	add	x8, x19, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 32
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	x0, x19
	mov	w1, wzr
	bl	f2fs_mark_inode_dirty_sync
	ldrb	w8, [x20]
	tbz	w8, #0, .LBB20_40
.LBB20_73:
	ldr	x8, [x19, #736]
	mov	w9, #732
	mov	x0, x19
	lsl	x8, x8, #58
	and	x8, x9, x8, asr #63
	ldrh	w1, [x19, x8]
	bl	posix_acl_chmod
	ldr	x8, [x19, #736]
	mov	w21, w0
	tbz	w8, #5, .LBB20_77
// %bb.74:
	cbnz	w21, .LBB20_76
// %bb.75:
	ldrh	w8, [x19, #732]
	strh	w8, [x19]
.LBB20_76:
	add	x8, x19, #736           // =736
	mov	w9, #32
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
.LBB20_77:
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_mark_inode_dirty_sync
	ldr	x8, [x19, #40]
	mov	w1, #1
	ldr	x0, [x8, #1008]
	bl	f2fs_balance_fs
	b	.LBB20_42
.LBB20_78:
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB20_80
// %bb.79:
	add	x8, x8, #80             // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4096
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
.LBB20_80:
	add	x0, x8, #304            // =304
	bl	up_read
	b	.LBB20_42
.LBB20_81:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x22, #360]
	ldr	x2, [x23, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x22, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB20_24
.LBB20_82:
	mov	w21, #-1
	b	.LBB20_42
.LBB20_83:
	mov	w21, #-5
	b	.LBB20_42
.Lfunc_end20:
	.size	f2fs_setattr, .Lfunc_end20-f2fs_setattr
                                        // -- End function
	.globl	f2fs_reserve_compress_blocks_internal // -- Begin function f2fs_reserve_compress_blocks_internal
	.p2align	2
	.type	f2fs_reserve_compress_blocks_internal,@function
f2fs_reserve_compress_blocks_internal:  // @f2fs_reserve_compress_blocks_internal
.Lf2fs_reserve_compress_blocks_internal$local:
// %bb.0:
	sub	sp, sp, #176            // =176
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #80]     // 16-byte Folded Spill
	add	x29, sp, #80            // =80
	stp	x28, x27, [sp, #96]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #112]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #128]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #144]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #160]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x8, [x0, #40]
	mov	x21, x0
	mov	w26, w1
	ldr	x20, [x8, #1008]
	bl	dquot_initialize
	add	w8, w0, #28             // =28
	mov	w25, w0
	cmp	w8, #28                 // =28
	b.hi	.LBB21_2
// %bb.1:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB21_3
.LBB21_2:
	cmn	w25, #122               // =122
	b.ne	.LBB21_39
.LBB21_3:
	ldr	x8, [x21, #40]
	mov	w1, #1
	str	x20, [sp, #16]          // 8-byte Folded Spill
	ldr	x0, [x8, #1008]
	bl	f2fs_balance_fs
	add	x23, x21, #1040         // =1040
	mov	x0, x23
	bl	down_write
	add	x24, x21, #1088         // =1088
	mov	x0, x24
	bl	down_write
	ldr	x8, [x21, #80]
	str	w26, [sp, #12]          // 4-byte Folded Spill
	add	x8, x8, #4095           // =4095
	lsr	x28, x8, #12
	cbz	x28, .LBB21_29
// %bb.4:
	mov	x20, #-549755813888
	mov	x19, #-549755813888
	mov	x26, xzr
	mov	w22, wzr
	movk	x20, #4072
	movk	x19, #4076
	b	.LBB21_7
.LBB21_5:                               //   in Loop: Header=BB21_7 Depth=1
	stp	xzr, xzr, [sp, #32]
	tbnz	w25, #31, .LBB21_25
// %bb.6:                               //   in Loop: Header=BB21_7 Depth=1
	add	x26, x27, x26
	add	w22, w25, w22
	cmp	x26, x28
	b.hs	.LBB21_30
.LBB21_7:                               // =>This Inner Loop Header: Depth=1
	add	x0, sp, #24             // =24
	mov	w2, #48
	mov	w1, wzr
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	bl	memset
	add	x0, sp, #24             // =24
	mov	w2, #1
	mov	x1, x26
	stp	x21, xzr, [sp, #24]
	str	xzr, [sp, #40]
	str	wzr, [sp, #48]
	bl	f2fs_get_dnode_of_data
	cbz	w0, .LBB21_10
// %bb.8:                               //   in Loop: Header=BB21_7 Depth=1
	mov	w25, w0
	cmn	w0, #2                  // =2
	b.ne	.LBB21_25
// %bb.9:                               //   in Loop: Header=BB21_7 Depth=1
	add	x0, sp, #24             // =24
	mov	x1, x26
	bl	f2fs_get_next_page_offset
	mov	x26, x0
	mov	w25, wzr
	cmp	x26, x28
	b.lo	.LBB21_7
	b	.LBB21_30
.LBB21_10:                              //   in Loop: Header=BB21_7 Depth=1
	ldr	x8, [sp, #40]
	mov	x9, #134217728
	movk	x9, #64, lsl #32
	add	x8, x9, x8, lsl #6
	and	x8, x8, #0xfffffffffffff000
	ldr	w9, [x8, x20]
	ldr	w8, [x8, x19]
	cmp	w9, w8
	b.ne	.LBB21_14
// %bb.11:                              //   in Loop: Header=BB21_7 Depth=1
	ldr	w8, [x21, #1184]
	ldr	w9, [x21, #1192]
	ldrh	w10, [x21]
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	mov	w9, #923
	sub	w8, w9, w8
	b.ne	.LBB21_19
// %bb.12:                              //   in Loop: Header=BB21_7 Depth=1
	ldr	x9, [x21, #736]
	tbz	w9, #31, .LBB21_19
// %bb.13:                              //   in Loop: Header=BB21_7 Depth=1
	ldr	w9, [x21, #1292]
	neg	w9, w9
	b	.LBB21_18
.LBB21_14:                              //   in Loop: Header=BB21_7 Depth=1
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB21_16
// %bb.15:                              //   in Loop: Header=BB21_7 Depth=1
	ldr	x8, [x21, #736]
	tbnz	w8, #31, .LBB21_17
.LBB21_16:                              //   in Loop: Header=BB21_7 Depth=1
	mov	w8, #1018
	b	.LBB21_19
.LBB21_17:                              //   in Loop: Header=BB21_7 Depth=1
	ldr	w8, [x21, #1292]
	mov	w9, #1018
	neg	w8, w8
.LBB21_18:                              //   in Loop: Header=BB21_7 Depth=1
	and	w8, w8, w9
.LBB21_19:                              //   in Loop: Header=BB21_7 Depth=1
	ldr	w9, [sp, #52]
	mov	w8, w8
	ldr	w11, [x21, #1292]
	sub	x10, x28, x26
	sub	x8, x8, x9
	cmp	x8, x10
	csel	x8, x8, x10, lo
	sub	x8, x8, #1              // =1
	sub	w9, w11, #1             // =1
	orr	x8, x8, x9
	add	x27, x8, #1             // =1
	add	x0, sp, #24             // =24
	mov	x1, x27
	bl	reserve_compress_blocks
	ldr	x8, [sp, #40]
	mov	w25, w0
	cbz	x8, .LBB21_21
// %bb.20:                              //   in Loop: Header=BB21_7 Depth=1
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_put_page
	ldr	x9, [sp, #40]
	ldr	x8, [sp, #32]
	cbnz	x8, .LBB21_22
	b	.LBB21_5
.LBB21_21:                              //   in Loop: Header=BB21_7 Depth=1
	mov	x9, xzr
	ldr	x8, [sp, #32]
	cbz	x8, .LBB21_5
.LBB21_22:                              //   in Loop: Header=BB21_7 Depth=1
	cmp	x9, x8
	b.eq	.LBB21_5
// %bb.23:                              //   in Loop: Header=BB21_7 Depth=1
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
	cbnz	w9, .LBB21_5
// %bb.24:                              //   in Loop: Header=BB21_7 Depth=1
	bl	__put_page
	b	.LBB21_5
.LBB21_25:
	cbz	w22, .LBB21_30
// %bb.26:
	tbz	w25, #31, .LBB21_30
// %bb.27:
	ldr	w8, [x21, #1280]
	cbz	w8, .LBB21_30
// %bb.28:
	mov	x0, x21
	mov	x1, x26
	bl	f2fs_release_reserved_blocks
	sub	w22, w22, w0
	b	.LBB21_30
.LBB21_29:
	mov	w25, wzr
	mov	w22, wzr
.LBB21_30:
	mov	x0, x23
	bl	up_write
	mov	x0, x24
	bl	up_write
	tbnz	w25, #31, .LBB21_32
// %bb.31:
	add	x8, x21, #736           // =736
	mov	x9, #34359738368
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x21
	bl	current_time
	stp	x0, x1, [x21, #120]
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	ldr	x20, [sp, #16]          // 8-byte Folded Reload
	ldr	w26, [sp, #12]          // 4-byte Folded Reload
	b	.LBB21_40
.LBB21_32:
	ldr	x4, [x21, #64]
	ldr	x5, [x21, #144]
	ldr	w7, [x21, #1280]
	ldr	x20, [sp, #16]          // 8-byte Folded Reload
	adrp	x1, .L.str.10
	adrp	x2, .L__func__.f2fs_reserve_compress_blocks_internal
	add	x1, x1, :lo12:.L.str.10
	add	x2, x2, :lo12:.L__func__.f2fs_reserve_compress_blocks_internal
	mov	x0, x20
	mov	w3, w25
	mov	w6, w22
	bl	f2fs_printk
	ldr	w26, [sp, #12]          // 4-byte Folded Reload
	cbz	w22, .LBB21_40
// %bb.33:
	ldr	w8, [x21, #1280]
	cbz	x20, .LBB21_40
// %bb.34:
	cbz	w8, .LBB21_40
// %bb.35:
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
	cbnz	w8, .LBB21_40
// %bb.36:
	ldr	x1, [x20, #3392]
	cbz	x1, .LBB21_40
// %bb.37:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB21_40
// %bb.38:
	add	x2, x20, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB21_40
.LBB21_39:
	mov	w22, wzr
.LBB21_40:
	cmp	w26, #9                 // =9
	and	w23, w25, w25, asr #31
	b.eq	.LBB21_46
// %bb.41:
	mov	w0, #42678
	movk	w0, #15148, lsl #16
	bl	hiview_hievent_create
	cbz	x0, .LBB21_45
// %bb.42:
	adrp	x1, .L.str.32
	mov	w2, w26
	add	x1, x1, :lo12:.L.str.32
	mov	x24, x0
	bl	hiview_hievent_put_integral
	ldr	x2, [x21, #64]
	adrp	x1, .L.str.33
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.33
	mov	x0, x24
	bl	hiview_hievent_put_integral
	adrp	x1, .L.str.34
	orr	w19, w0, w21
	mov	w2, w22
	add	x1, x1, :lo12:.L.str.34
	mov	x0, x24
	bl	hiview_hievent_put_integral
	adrp	x1, .L.str.35
	orr	w19, w19, w0
	sxtw	x2, w23
	add	x1, x1, :lo12:.L.str.35
	mov	x0, x24
	bl	hiview_hievent_put_integral
	orr	w8, w19, w0
	cbnz	w8, .LBB21_44
// %bb.43:
	mov	x0, x24
	bl	hiview_hievent_report
.LBB21_44:
	adrp	x1, .L.str.37
	adrp	x2, .L__func__.f2fs_compress_imonitor_send
	add	x1, x1, :lo12:.L.str.37
	add	x2, x2, :lo12:.L__func__.f2fs_compress_imonitor_send
	mov	x0, x20
	mov	w3, w26
	bl	f2fs_printk
	mov	x0, x24
	bl	hiview_hievent_destroy
	b	.LBB21_46
.LBB21_45:
	adrp	x1, .L.str.21
	adrp	x2, .L__func__.f2fs_compress_imonitor_send
	add	x1, x1, :lo12:.L.str.21
	add	x2, x2, :lo12:.L__func__.f2fs_compress_imonitor_send
	mov	x0, x20
	bl	f2fs_printk
.LBB21_46:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB21_48
// %bb.47:
	mov	w0, w23
	ldp	x20, x19, [sp, #160]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #144]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #128]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #112]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #96]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #80]     // 16-byte Folded Reload
	add	sp, sp, #176            // =176
	ret
.LBB21_48:
	bl	__stack_chk_fail
.Lfunc_end21:
	.size	f2fs_reserve_compress_blocks_internal, .Lfunc_end21-f2fs_reserve_compress_blocks_internal
                                        // -- End function
	.globl	f2fs_truncate_hole      // -- Begin function f2fs_truncate_hole
	.p2align	2
	.type	f2fs_truncate_hole,@function
f2fs_truncate_hole:                     // @f2fs_truncate_hole
.Lf2fs_truncate_hole$local:
// %bb.0:
	sub	sp, sp, #160            // =160
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #64]     // 16-byte Folded Spill
	add	x29, sp, #64            // =64
	cmp	x1, x2
	stp	x28, x27, [sp, #80]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #96]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #112]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #128]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #144]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	b.hs	.LBB22_29
// %bb.1:
	mov	x26, #134217728
	mov	x27, #-549755813888
	mov	x28, #-549755813888
	mov	x19, x2
	mov	x21, x1
	mov	x20, x0
	mov	w25, #1
	movk	x26, #64, lsl #32
	movk	x27, #4072
	movk	x28, #4076
	mov	w22, #923
	mov	w23, #1018
	b	.LBB22_3
.LBB22_2:                               //   in Loop: Header=BB22_3 Depth=1
	add	x0, sp, #8              // =8
	mov	x1, x21
	bl	f2fs_get_next_page_offset
	mov	x21, x0
	cmp	x21, x19
	b.hs	.LBB22_29
.LBB22_3:                               // =>This Inner Loop Header: Depth=1
	add	x0, sp, #8              // =8
	mov	w2, #48
	mov	w1, wzr
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	bl	memset
	add	x0, sp, #8              // =8
	mov	w2, #1
	mov	x1, x21
	stp	x20, xzr, [sp, #8]
	str	xzr, [sp, #24]
	str	wzr, [sp, #32]
	bl	f2fs_get_dnode_of_data
	cmn	w0, #2                  // =2
	b.eq	.LBB22_2
// %bb.4:                               //   in Loop: Header=BB22_3 Depth=1
	cbnz	w0, .LBB22_30
// %bb.5:                               //   in Loop: Header=BB22_3 Depth=1
	ldr	x8, [sp, #24]
	add	x8, x26, x8, lsl #6
	and	x8, x8, #0xfffffffffffff000
	ldr	w9, [x8, x27]
	ldr	w8, [x8, x28]
	cmp	w9, w8
	b.ne	.LBB22_9
// %bb.6:                               //   in Loop: Header=BB22_3 Depth=1
	ldr	w8, [x20, #1184]
	ldr	w9, [x20, #1192]
	ldrh	w10, [x20]
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	sub	w8, w22, w8
	b.ne	.LBB22_12
// %bb.7:                               //   in Loop: Header=BB22_3 Depth=1
	ldr	x9, [x20, #736]
	tbz	w9, #31, .LBB22_12
// %bb.8:                               //   in Loop: Header=BB22_3 Depth=1
	ldr	w9, [x20, #1292]
	neg	w9, w9
	and	w8, w8, w9
	b	.LBB22_12
.LBB22_9:                               //   in Loop: Header=BB22_3 Depth=1
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB22_11
// %bb.10:                              //   in Loop: Header=BB22_3 Depth=1
	ldr	x8, [x20, #736]
	tbnz	w8, #31, .LBB22_20
.LBB22_11:                              //   in Loop: Header=BB22_3 Depth=1
	mov	w8, #1018
.LBB22_12:                              //   in Loop: Header=BB22_3 Depth=1
	ldr	w9, [sp, #36]
	mov	w8, w8
	sub	x10, x19, x21
	sub	x9, x8, x9
	cmp	x9, x10
	csel	x24, x9, x10, lo
	sub	x9, x24, #1             // =1
	cmp	x9, x8
	b.hs	.LBB22_21
.LBB22_13:                              //   in Loop: Header=BB22_3 Depth=1
	add	x0, sp, #8              // =8
	mov	w1, w24
	bl	f2fs_truncate_data_blocks_range
	ldr	x0, [sp, #24]
	cbz	x0, .LBB22_15
// %bb.14:                              //   in Loop: Header=BB22_3 Depth=1
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #24]
	ldr	x8, [sp, #16]
	cbnz	x8, .LBB22_16
	b	.LBB22_19
.LBB22_15:                              //   in Loop: Header=BB22_3 Depth=1
	mov	x9, xzr
	ldr	x8, [sp, #16]
	cbz	x8, .LBB22_19
.LBB22_16:                              //   in Loop: Header=BB22_3 Depth=1
	cmp	x9, x8
	b.eq	.LBB22_19
// %bb.17:                              //   in Loop: Header=BB22_3 Depth=1
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
	cbnz	w9, .LBB22_19
// %bb.18:                              //   in Loop: Header=BB22_3 Depth=1
	bl	__put_page
.LBB22_19:                              //   in Loop: Header=BB22_3 Depth=1
	add	x21, x24, x21
	stp	xzr, xzr, [sp, #16]
	cmp	x21, x19
	b.lo	.LBB22_3
	b	.LBB22_29
.LBB22_20:                              //   in Loop: Header=BB22_3 Depth=1
	ldr	w8, [x20, #1292]
	neg	w8, w8
	and	w8, w8, w23
	b	.LBB22_12
.LBB22_21:                              //   in Loop: Header=BB22_3 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB22_24
// %bb.22:                              //   in Loop: Header=BB22_3 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB22_24
// %bb.23:                              //   in Loop: Header=BB22_3 Depth=1
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.f2fs_truncate_hole
	mov	w3, #3351
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.f2fs_truncate_hole
	bl	dsm_client_record
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB22_24:                              //   in Loop: Header=BB22_3 Depth=1
	ldr	x8, [x20, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_raw_sb_info
	ldr	x8, [x20, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_ckpt_info
	ldr	x8, [x20, #40]
	ldr	x0, [x8, #1008]
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/file.c"; .popsection; .long 14472b - 14470b; .short 3351; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB22_13
// %bb.25:                              //   in Loop: Header=BB22_3 Depth=1
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
	cbnz	w9, .LBB22_13
// %bb.26:                              //   in Loop: Header=BB22_3 Depth=1
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB22_13
// %bb.27:                              //   in Loop: Header=BB22_3 Depth=1
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB22_13
// %bb.28:                              //   in Loop: Header=BB22_3 Depth=1
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB22_13
.LBB22_29:
	mov	w0, wzr
.LBB22_30:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB22_32
// %bb.31:
	ldp	x20, x19, [sp, #144]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #128]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #112]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #96]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #80]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #64]     // 16-byte Folded Reload
	add	sp, sp, #160            // =160
	ret
.LBB22_32:
	bl	__stack_chk_fail
.Lfunc_end22:
	.size	f2fs_truncate_hole, .Lfunc_end22-f2fs_truncate_hole
                                        // -- End function
	.globl	f2fs_transfer_project_quota // -- Begin function f2fs_transfer_project_quota
	.p2align	2
	.type	f2fs_transfer_project_quota,@function
f2fs_transfer_project_quota:            // @f2fs_transfer_project_quota
.Lf2fs_transfer_project_quota$local:
// %bb.0:
	sub	sp, sp, #64             // =64
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	add	x29, sp, #32            // =32
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stp	xzr, xzr, [sp, #8]
	str	xzr, [sp]
	ldr	x8, [x0, #40]
	mov	x19, x0
	ldr	x20, [x8, #1008]
	mov	x8, #8589934592
	bfxil	x8, x1, #0, #32
	mov	x1, x8
	ldr	x0, [x20]
	bl	dqget
	cmn	x0, #1, lsl #12         // =4096
	str	x0, [sp, #16]
	b.ls	.LBB23_2
// %bb.1:
	mov	w19, wzr
	b	.LBB23_6
.LBB23_2:
	mov	x1, sp
	mov	x0, x19
	bl	__dquot_transfer
	mov	w19, w0
	cbz	x20, .LBB23_5
// %bb.3:
	cbz	w19, .LBB23_5
// %bb.4:
	add	x8, x20, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4096
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB23_5:
	ldr	x0, [sp, #16]
	bl	dqput
.LBB23_6:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB23_8
// %bb.7:
	mov	w0, w19
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #64             // =64
	ret
.LBB23_8:
	bl	__stack_chk_fail
.Lfunc_end23:
	.size	f2fs_transfer_project_quota, .Lfunc_end23-f2fs_transfer_project_quota
                                        // -- End function
	.globl	f2fs_pin_file_control   // -- Begin function f2fs_pin_file_control
	.p2align	2
	.type	f2fs_pin_file_control,@function
f2fs_pin_file_control:                  // @f2fs_pin_file_control
.Lf2fs_pin_file_control$local:
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	mov	x19, x0
	mov	x29, sp
	ldr	x20, [x8, #1008]
	tbz	w1, #0, .LBB24_2
// %bb.1:
	ldr	w8, [x19, #720]
	mov	w1, #1
	mov	x0, x19
	add	w8, w8, #1              // =1
	str	w8, [x19, #720]
	bl	f2fs_mark_inode_dirty_sync
.LBB24_2:
	ldr	w4, [x19, #720]
	ldr	x8, [x20, #2032]
	cmp	x8, x4
	b.hs	.LBB24_4
// %bb.3:
	ldr	x3, [x19, #64]
	adrp	x1, .L.str.9
	adrp	x2, .L__func__.f2fs_pin_file_control
	add	x1, x1, :lo12:.L.str.9
	add	x2, x2, :lo12:.L__func__.f2fs_pin_file_control
	mov	x0, x20
                                        // kill: def $w4 killed $w4 killed $x4
	bl	f2fs_printk
	add	x8, x19, #736           // =736
	mov	w9, #268435456
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
	mov	w0, #-11
	b	.LBB24_5
.LBB24_4:
	mov	w0, wzr
.LBB24_5:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end24:
	.size	f2fs_pin_file_control, .Lfunc_end24-f2fs_pin_file_control
                                        // -- End function
	.globl	f2fs_precache_extents   // -- Begin function f2fs_precache_extents
	.p2align	2
	.type	f2fs_precache_extents,@function
f2fs_precache_extents:                  // @f2fs_precache_extents
.Lf2fs_precache_extents$local:
// %bb.0:
	sub	sp, sp, #112            // =112
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #64]     // 16-byte Folded Spill
	add	x29, sp, #64            // =64
	stp	x22, x21, [sp, #80]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #96]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	ldr	x8, [x0, #736]
	tbnz	w8, #8, .LBB25_6
// %bb.1:
	add	x8, sp, #8              // =8
	mov	w9, #8
	mov	x19, x0
	str	wzr, [sp, #20]
	stp	xzr, x8, [sp, #32]
	str	w9, [sp, #48]
	strb	wzr, [sp, #52]
	bl	max_file_blocks
	cmp	x0, #1                  // =1
	b.lt	.LBB25_5
// %bb.2:
	mov	x20, x0
	mov	w8, wzr
	add	x21, x19, #1040         // =1040
.LBB25_3:                               // =>This Inner Loop Header: Depth=1
	sub	w8, w20, w8
	mov	x0, x21
	str	w8, [sp, #24]
	bl	down_write
	add	x1, sp, #16             // =16
	mov	w3, #6
	mov	x0, x19
	mov	w2, wzr
	bl	f2fs_map_blocks
	mov	w22, w0
	mov	x0, x21
	bl	up_write
	cbnz	w22, .LBB25_7
// %bb.4:                               //   in Loop: Header=BB25_3 Depth=1
	ldr	x8, [sp, #8]
	cmp	x20, w8, uxtw
	str	w8, [sp, #20]
	b.gt	.LBB25_3
.LBB25_5:
	mov	w22, wzr
	b	.LBB25_7
.LBB25_6:
	mov	w22, #-95
.LBB25_7:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB25_9
// %bb.8:
	mov	w0, w22
	ldp	x20, x19, [sp, #96]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #80]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #64]     // 16-byte Folded Reload
	add	sp, sp, #112            // =112
	ret
.LBB25_9:
	bl	__stack_chk_fail
.Lfunc_end25:
	.size	f2fs_precache_extents, .Lfunc_end25-f2fs_precache_extents
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_release_reserved_blocks
	.type	f2fs_release_reserved_blocks,@function
f2fs_release_reserved_blocks:           // @f2fs_release_reserved_blocks
// %bb.0:
	sub	sp, sp, #176            // =176
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #80]     // 16-byte Folded Spill
	add	x29, sp, #80            // =80
	stp	x28, x27, [sp, #96]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #112]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #128]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #144]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #160]    // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	cbz	x1, .LBB26_30
// %bb.1:
	ldr	x8, [x0, #40]
	mov	x25, #134217728
	mov	x26, #-549755813888
	mov	x23, #-549755813888
	ldr	x8, [x8, #1008]
	mov	x20, x1
	mov	x21, x0
	mov	w19, wzr
	mov	x22, xzr
	movk	x25, #64, lsl #32
	movk	x26, #4072
	movk	x23, #4076
	mov	w24, #923
	str	x8, [sp, #16]           // 8-byte Folded Spill
	add	x8, x8, #3408           // =3408
	str	x8, [sp, #8]            // 8-byte Folded Spill
	b	.LBB26_3
.LBB26_2:                               //   in Loop: Header=BB26_3 Depth=1
	add	x0, sp, #24             // =24
	mov	x1, x22
	bl	f2fs_get_next_page_offset
	mov	x22, x0
	cmp	x22, x20
	b.hs	.LBB26_31
.LBB26_3:                               // =>This Inner Loop Header: Depth=1
	add	x0, sp, #24             // =24
	mov	w2, #48
	mov	w1, wzr
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	bl	memset
	add	x0, sp, #24             // =24
	mov	w2, #1
	mov	x1, x22
	stp	x21, xzr, [sp, #24]
	str	xzr, [sp, #40]
	str	wzr, [sp, #48]
	bl	f2fs_get_dnode_of_data
	cmn	w0, #2                  // =2
	b.eq	.LBB26_2
// %bb.4:                               //   in Loop: Header=BB26_3 Depth=1
	cbnz	w0, .LBB26_31
// %bb.5:                               //   in Loop: Header=BB26_3 Depth=1
	ldr	x8, [sp, #40]
	add	x8, x25, x8, lsl #6
	and	x8, x8, #0xfffffffffffff000
	ldr	w9, [x8, x26]
	ldr	w8, [x8, x23]
	cmp	w9, w8
	b.ne	.LBB26_9
// %bb.6:                               //   in Loop: Header=BB26_3 Depth=1
	ldr	w8, [x21, #1184]
	ldr	w9, [x21, #1192]
	ldrh	w10, [x21]
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	sub	w8, w24, w8
	b.ne	.LBB26_14
// %bb.7:                               //   in Loop: Header=BB26_3 Depth=1
	ldr	x9, [x21, #736]
	tbz	w9, #31, .LBB26_14
// %bb.8:                               //   in Loop: Header=BB26_3 Depth=1
	ldr	w9, [x21, #1292]
	neg	w9, w9
	b	.LBB26_13
.LBB26_9:                               //   in Loop: Header=BB26_3 Depth=1
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB26_11
// %bb.10:                              //   in Loop: Header=BB26_3 Depth=1
	ldr	x8, [x21, #736]
	tbnz	w8, #31, .LBB26_12
.LBB26_11:                              //   in Loop: Header=BB26_3 Depth=1
	mov	w8, #1018
	b	.LBB26_14
.LBB26_12:                              //   in Loop: Header=BB26_3 Depth=1
	ldr	w8, [x21, #1292]
	mov	w9, #1018
	neg	w8, w8
.LBB26_13:                              //   in Loop: Header=BB26_3 Depth=1
	and	w8, w8, w9
.LBB26_14:                              //   in Loop: Header=BB26_3 Depth=1
	ldr	w9, [sp, #52]
	mov	w8, w8
	ldr	w11, [x21, #1292]
	sub	x10, x20, x22
	sub	x8, x8, x9
	cmp	x8, x10
	csel	x8, x8, x10, lo
	sub	x8, x8, #1              // =1
	sub	w9, w11, #1             // =1
	orr	x8, x8, x9
	add	x27, x8, #1             // =1
	add	x0, sp, #24             // =24
	mov	x1, x27
	bl	release_compress_blocks
	ldr	x8, [sp, #40]
	mov	w28, w0
	cbz	x8, .LBB26_16
// %bb.15:                              //   in Loop: Header=BB26_3 Depth=1
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_put_page
	ldr	x9, [sp, #40]
	ldr	x8, [sp, #32]
	cbnz	x8, .LBB26_17
	b	.LBB26_20
.LBB26_16:                              //   in Loop: Header=BB26_3 Depth=1
	mov	x9, xzr
	ldr	x8, [sp, #32]
	cbz	x8, .LBB26_20
.LBB26_17:                              //   in Loop: Header=BB26_3 Depth=1
	cmp	x9, x8
	b.eq	.LBB26_20
// %bb.18:                              //   in Loop: Header=BB26_3 Depth=1
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
	cbnz	w9, .LBB26_20
// %bb.19:                              //   in Loop: Header=BB26_3 Depth=1
	bl	__put_page
.LBB26_20:                              //   in Loop: Header=BB26_3 Depth=1
	stp	xzr, xzr, [sp, #32]
	tbnz	w28, #31, .LBB26_22
.LBB26_21:                              //   in Loop: Header=BB26_3 Depth=1
	add	x22, x27, x22
	add	w19, w28, w19
	cmp	x22, x20
	b.lo	.LBB26_3
	b	.LBB26_31
.LBB26_22:                              //   in Loop: Header=BB26_3 Depth=1
	bl	print_bdev_access_info
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	cbz	x0, .LBB26_25
// %bb.23:                              //   in Loop: Header=BB26_3 Depth=1
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB26_25
// %bb.24:                              //   in Loop: Header=BB26_3 Depth=1
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.f2fs_release_reserved_blocks
	mov	w3, #6657
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.f2fs_release_reserved_blocks
	bl	dsm_client_record
	adrp	x8, f2fs_dclient
	ldr	x0, [x8, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB26_25:                              //   in Loop: Header=BB26_3 Depth=1
	ldr	x0, [sp, #16]           // 8-byte Folded Reload
	bl	f2fs_print_raw_sb_info
	ldr	x0, [sp, #16]           // 8-byte Folded Reload
	bl	f2fs_print_ckpt_info
	ldr	x0, [sp, #16]           // 8-byte Folded Reload
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/file.c"; .popsection; .long 14472b - 14470b; .short 6657; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	cbz	x8, .LBB26_21
// %bb.26:                              //   in Loop: Header=BB26_3 Depth=1
	ldr	x11, [sp, #16]          // 8-byte Folded Reload
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
	cbnz	w8, .LBB26_21
// %bb.27:                              //   in Loop: Header=BB26_3 Depth=1
	ldr	x8, [sp, #16]           // 8-byte Folded Reload
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB26_21
// %bb.28:                              //   in Loop: Header=BB26_3 Depth=1
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB26_21
// %bb.29:                              //   in Loop: Header=BB26_3 Depth=1
	ldr	x2, [sp, #8]            // 8-byte Folded Reload
	mov	w0, #8
	bl	queue_work_on
	b	.LBB26_21
.LBB26_30:
	mov	w19, wzr
.LBB26_31:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB26_33
// %bb.32:
	mov	w0, w19
	ldp	x20, x19, [sp, #160]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #144]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #128]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #112]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #96]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #80]     // 16-byte Folded Reload
	add	sp, sp, #176            // =176
	ret
.LBB26_33:
	bl	__stack_chk_fail
.Lfunc_end26:
	.size	f2fs_release_reserved_blocks, .Lfunc_end26-f2fs_release_reserved_blocks
                                        // -- End function
	.globl	f2fs_decompress_file_internal // -- Begin function f2fs_decompress_file_internal
	.p2align	2
	.type	f2fs_decompress_file_internal,@function
f2fs_decompress_file_internal:          // @f2fs_decompress_file_internal
.Lf2fs_decompress_file_internal$local:
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	ldrb	w8, [x0, #68]
	str	x27, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	tbnz	w8, #1, .LBB27_2
// %bb.1:
	mov	x21, #-9
	b	.LBB27_16
.LBB27_2:
	ldr	x21, [x0, #32]
	mov	w19, w2
	mov	w22, w1
	ldr	x8, [x21, #40]
	mov	x0, x21
	ldr	x20, [x8, #1008]
	bl	dquot_initialize
                                        // kill: def $w0 killed $w0 def $x0
	add	w8, w0, #28             // =28
	cmp	w8, #28                 // =28
	b.hi	.LBB27_4
// %bb.3:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB27_5
.LBB27_4:
	cmn	w0, #122                // =122
	b.ne	.LBB27_14
.LBB27_5:
	ldr	x8, [x21, #40]
	mov	w1, #1
	ldr	x0, [x8, #1008]
	bl	f2fs_balance_fs
	ldr	x8, [x21, #736]
	tbz	x8, #35, .LBB27_8
// %bb.6:
	mov	x0, x21
	mov	w1, w22
	bl	f2fs_reserve_compress_blocks_internal
                                        // kill: def $w0 killed $w0 def $x0
	cbz	w0, .LBB27_8
// %bb.7:
	adrp	x1, .L.str.11
	sxtw	x21, w0
	add	x1, x1, :lo12:.L.str.11
	b	.LBB27_12
.LBB27_8:
	ldr	x8, [x21, #736]
	tbnz	x8, #38, .LBB27_13
// %bb.9:
	ldr	x8, [x21, #736]
	tbnz	x8, #33, .LBB27_15
// %bb.10:
	ldp	x8, x0, [x21, #40]
	ldr	w26, [x21, #1292]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	ldr	x8, [x8, #1008]
	ldr	w25, [x8, #1188]
	bl	filemap_write_and_wait_range
	mov	w22, w0
	cbz	w0, .LBB27_17
.LBB27_11:
	adrp	x1, .L.str.13
	sxtw	x21, w22
	add	x1, x1, :lo12:.L.str.13
.LBB27_12:
	adrp	x2, .L__func__.f2fs_decompress_file_internal
	add	x2, x2, :lo12:.L__func__.f2fs_decompress_file_internal
	mov	x0, x20
	mov	w3, w19
	mov	x4, x21
	bl	f2fs_printk
	b	.LBB27_16
.LBB27_13:
	adrp	x1, .L.str.12
	adrp	x2, .L__func__.f2fs_decompress_file_internal
	add	x1, x1, :lo12:.L.str.12
	add	x2, x2, :lo12:.L__func__.f2fs_decompress_file_internal
	mov	x0, x20
	bl	f2fs_printk
	mov	x21, #-13
	b	.LBB27_16
.LBB27_14:
	sxtw	x21, w0
	b	.LBB27_16
.LBB27_15:
	mov	x21, #-16
.LBB27_16:
	mov	x0, x21
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldr	x27, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.LBB27_17:
	ldr	w8, [x21, #1280]
	cbz	w8, .LBB27_26
// %bb.18:
	ldr	x8, [x21, #80]
	add	x8, x8, #4095           // =4095
	lsr	x27, x8, #12
	cbz	w27, .LBB27_25
// %bb.19:
	mov	x23, xzr
	b	.LBB27_21
.LBB27_20:                              //   in Loop: Header=BB27_21 Depth=1
	subs	w27, w27, w24
	add	x23, x23, w24, sxtw
	b.eq	.LBB27_24
.LBB27_21:                              // =>This Inner Loop Header: Depth=1
	cmp	w26, w27
	csel	w24, w26, w27, lt
	mov	x0, x21
	mov	x1, x23
	mov	w2, w24
	bl	redirty_blocks
	mov	w22, w0
	tbnz	w0, #31, .LBB27_11
// %bb.22:                              //   in Loop: Header=BB27_21 Depth=1
	ldr	w8, [x21, #792]
	cmp	w8, w25
	b.lo	.LBB27_20
// %bb.23:                              //   in Loop: Header=BB27_21 Depth=1
	ldr	x0, [x21, #48]
	bl	filemap_fdatawrite
	b	.LBB27_20
.LBB27_24:
	cbnz	w22, .LBB27_11
.LBB27_25:
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w22, w0
	cbnz	w0, .LBB27_11
.LBB27_26:
	adrp	x1, .L.str.14
	adrp	x2, .L__func__.f2fs_decompress_file_internal
	add	x1, x1, :lo12:.L.str.14
	add	x2, x2, :lo12:.L__func__.f2fs_decompress_file_internal
	mov	x0, x20
	mov	w3, w19
	bl	f2fs_printk
	mov	x21, xzr
	b	.LBB27_16
.Lfunc_end27:
	.size	f2fs_decompress_file_internal, .Lfunc_end27-f2fs_decompress_file_internal
                                        // -- End function
	.globl	f2fs_ioctl              // -- Begin function f2fs_ioctl
	.p2align	2
	.type	f2fs_ioctl,@function
f2fs_ioctl:                             // @f2fs_ioctl
.Lf2fs_ioctl$local:
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0, #32]
	mov	x20, x0
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	ldr	x8, [x0, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB28_4
// %bb.1:
	ldr	x8, [x0, #80]
	mov	x19, x2
	mov	w21, w1
	tbnz	w8, #8, .LBB28_5
.LBB28_2:
	mov	x0, x20
	mov	w1, w21
	mov	x2, x19
	bl	__f2fs_ioctl
.LBB28_3:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB28_4:
	mov	x0, #-5
	b	.LBB28_3
.LBB28_5:
	mov	w1, wzr
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB28_2
// %bb.6:
	mov	x0, #-28
	b	.LBB28_3
.Lfunc_end28:
	.size	f2fs_ioctl, .Lfunc_end28-f2fs_ioctl
                                        // -- End function
	.p2align	2               // -- Begin function __f2fs_ioctl
	.type	__f2fs_ioctl,@function
__f2fs_ioctl:                           // @__f2fs_ioctl
// %bb.0:
	sub	sp, sp, #384            // =384
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	mov	w9, #26136
	movk	w9, #49216, lsl #16
	stp	x29, x30, [sp, #288]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #368]    // 16-byte Folded Spill
	add	x29, sp, #288           // =288
	mov	x20, x2
	mov	x19, x0
	cmp	w1, w9
	mov	x0, #-25
	stp	x28, x27, [sp, #304]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #320]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #336]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #352]    // 16-byte Folded Spill
	stur	x8, [x29, #-16]
	b.gt	.LBB29_11
// %bb.1:
	mov	w8, #26130
	movk	w8, #32780, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_24
// %bb.2:
	mov	w8, #26112
	movk	w8, #32776, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_47
// %bb.3:
	mov	w8, #62731
	movk	w8, #32772, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_70
// %bb.4:
	mov	w8, #62740
	movk	w8, #32770, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_134
// %bb.5:
	mov	w8, #62781
	movk	w8, #32769, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_347
// %bb.6:
	mov	w8, #62783
	movk	w8, #32769, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.7:
	ldr	x21, [x19, #32]
	add	x19, x21, #160          // =160
	mov	x0, x19
	bl	down_write
	ldr	x8, [x21, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB29_543
// %bb.8:
	ldr	w8, [x21, #1280]
	cbnz	w8, .LBB29_10
// %bb.9:
	ldr	x8, [x21, #736]
	tbz	x8, #35, .LBB29_547
.LBB29_10:
	mov	w21, #1
	b	.LBB29_548
.LBB29_11:
	mov	w8, #62725
	movk	w8, #16388, lsl #16
	cmp	w1, w8
	b.le	.LBB29_30
// %bb.12:
	mov	w8, #62802
	movk	w8, #16393, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_64
// %bb.13:
	mov	w8, #26113
	movk	w8, #16392, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_93
// %bb.14:
	mov	w8, #62769
	movk	w8, #16388, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_170
// %bb.15:
	mov	w8, #62726
	movk	w8, #16388, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_417
// %bb.16:
	mov	w8, #62733
	movk	w8, #16388, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.17:
	ldr	x21, [x19, #32]
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_19
// %bb.18:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_20
.LBB29_19:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_20:
	//APP
		adds	x10, x10, 4
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_1035
// %bb.21:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x11, x20, x9, asr #8
	//APP
		bics	xzr, x11, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w11, [x10, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	b.lt	.LBB29_980
// %bb.22:
	adrp	x11, cpu_hwcap_keys+16
	ldr	w11, [x11, :lo12:cpu_hwcap_keys+16]
	cmp	w11, #0                 // =0
	b.le	.LBB29_981
.LBB29_23:
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
	b	.LBB29_984
.LBB29_24:
	mov	w8, #62750
	movk	w8, #49156, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_51
// %bb.25:
	mov	w8, #62773
	movk	w8, #32784, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_75
// %bb.26:
	mov	w8, #62800
	movk	w8, #32781, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_144
// %bb.27:
	mov	w8, #26131
	movk	w8, #32780, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_354
// %bb.28:
	mov	w8, #62802
	movk	w8, #32780, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.29:
	mov	x0, x19
	mov	x1, x20
	bl	f2fs_fscrypt_ioctl_get_policy_type
	b	.LBB29_968
.LBB29_30:
	mov	w8, #62720
	cmp	w1, w8
	b.le	.LBB29_116
// %bb.31:
	mov	w8, #-62721
	add	w8, w1, w8
	cmp	w8, #63                 // =63
	b.hi	.LBB29_202
// %bb.32:
	adrp	x9, .LJTI29_1
	add	x9, x9, :lo12:.LJTI29_1
	adr	x10, .LBB29_33
	ldrh	w11, [x9, x8, lsl #1]
	add	x10, x10, x11, lsl #2
	br	x10
.LBB29_33:
	ldr	x21, [x19, #32]
	ldr	x8, [x21, #40]
	mov	x0, x21
	ldr	x27, [x8, #1008]
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_494
// %bb.34:
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_542
// %bb.35:
	ldrb	w8, [x19, #66]
	tbnz	w8, #0, .LBB29_542
// %bb.36:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w23, w0
	cbnz	w0, .LBB29_1454
// %bb.37:
	add	x20, x21, #160          // =160
	mov	x0, x20
	bl	down_write
	ldr	x8, [x21, #736]
	tbz	x8, #44, .LBB29_39
// %bb.38:
	add	x8, x21, #736           // =736
	mov	x9, #17592186044416
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
.LBB29_39:
	ldr	x8, [x21, #736]
	tbz	x8, #38, .LBB29_43
// %bb.40:
	ldr	x8, [x21, #736]
	tbz	x8, #38, .LBB29_42
// %bb.41:
	ldr	x8, [x21, #736]
	tbnz	x8, #39, .LBB29_43
.LBB29_42:
	adrp	x1, .L__func__.f2fs_ioc_start_atomic_write
	add	x1, x1, :lo12:.L__func__.f2fs_ioc_start_atomic_write
	mov	x0, x21
	bl	f2fs_revoke_deduped_inode
	mov	w23, w0
	cbnz	w0, .LBB29_703
.LBB29_43:
	ldr	x8, [x21, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB29_684
// %bb.44:
	ldr	w8, [x21, #1280]
	cbnz	w8, .LBB29_686
// %bb.45:
	ldr	x8, [x21, #736]
	tbnz	x8, #35, .LBB29_686
// %bb.46:
	ldr	x8, [x21, #736]
	tbnz	x8, #34, .LBB29_686
	b	.LBB29_687
.LBB29_47:
	mov	w8, #2799
	movk	w8, #32759, lsl #16
	add	w8, w1, w8
	cmp	w8, #45                 // =45
	b.hi	.LBB29_125
// %bb.48:
	adrp	x9, .LJTI29_0
	add	x9, x9, :lo12:.LJTI29_0
	adr	x10, .LBB29_49
	ldrh	w11, [x9, x8, lsl #1]
	add	x10, x10, x11, lsl #2
	br	x10
.LBB29_49:
	ldr	x8, [x19, #32]
	ldr	x9, [x8, #40]
	ldr	x9, [x9, #1008]
	ldr	x9, [x9, #16]
	ldrb	w9, [x9, #2181]
	tbnz	w9, #5, .LBB29_488
// %bb.50:
	mov	w9, #-95
	sxtw	x0, w9
	b	.LBB29_1455
.LBB29_51:
	mov	w8, #62727
	movk	w8, #49168, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_87
// %bb.52:
	mov	w8, #62775
	movk	w8, #49160, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_148
// %bb.53:
	mov	w8, #62751
	movk	w8, #49156, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_356
// %bb.54:
	mov	w8, #62736
	movk	w8, #49160, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.55:
	ldr	x8, [x19, #32]
	mov	w0, #21
	ldr	x8, [x8, #40]
	ldr	x19, [x8, #1008]
	str	xzr, [sp, #144]
	bl	capable
	tbz	w0, #0, .LBB29_495
// %bb.56:
	ldr	x8, [x19]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_484
// %bb.57:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_59
// %bb.58:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_60
.LBB29_59:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_60:
	//APP
		adds	x10, x10, 8
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_1289
// %bb.61:
	adrp	x21, arm64_const_caps_ready
	ldr	w9, [x21, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1290
// %bb.62:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1291
.LBB29_63:
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
	b	.LBB29_1294
.LBB29_64:
	mov	w8, #62730
	movk	w8, #16408, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_106
// %bb.65:
	mov	w8, #62799
	movk	w8, #16397, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_181
// %bb.66:
	mov	w8, #62803
	movk	w8, #16393, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_425
// %bb.67:
	mov	w8, #26133
	movk	w8, #16396, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.68:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2180]
	tbz	w8, #0, .LBB29_377
// %bb.69:
	mov	x0, x19
	mov	x1, x20
	bl	fscrypt_ioctl_get_policy
	b	.LBB29_968
.LBB29_70:
	mov	w8, #62789
	movk	w8, #32772, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_152
// %bb.71:
	mov	w8, #62732
	movk	w8, #32772, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_357
// %bb.72:
	mov	w8, #62734
	movk	w8, #32772, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.73:
	ldr	x8, [x19, #32]
	ldr	x9, [x8, #736]
	tbnz	w9, #28, .LBB29_496
// %bb.74:
	mov	w8, wzr
	b	.LBB29_497
.LBB29_75:
	mov	w8, #37936
	movk	w8, #33024, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_162
// %bb.76:
	mov	w8, #62774
	movk	w8, #32784, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_364
// %bb.77:
	mov	w8, #22559
	movk	w8, #32796, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.78:
	ldr	x19, [x19, #32]
	str	wzr, [sp, #168]
	stp	xzr, xzr, [sp, #152]
	str	xzr, [sp, #144]
	ldr	w8, [x19, #704]
	add	x0, sp, #144            // =144
	mov	w2, #28
	mov	w1, wzr
	lsr	w11, w8, #1
	lsr	w10, w8, #3
	and	w12, w11, #0x8
	bfi	w12, w10, #5, #1
	and	w10, w11, #0x10
	lsr	w9, w8, #6
	orr	w10, w12, w10
	lsr	w8, w8, #20
	and	w11, w11, #0x40
	bfi	w10, w9, #7, #1
	orr	w9, w10, w11
	and	w8, w8, #0x200
	orr	w21, w9, w8
	bl	memset
	str	w21, [sp, #144]
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2180]
	tbz	w8, #4, .LBB29_80
// %bb.79:
	ldr	w1, [x19, #1188]
	adrp	x0, init_user_ns
	add	x0, x0, :lo12:init_user_ns
	bl	from_kprojid
	str	w0, [sp, #156]
.LBB29_80:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_82
// %bb.81:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_83
.LBB29_82:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_83:
	//APP
		adds	x10, x10, 28
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_654
// %bb.84:
	adrp	x19, arm64_const_caps_ready
	ldr	w9, [x19, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_987
// %bb.85:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_988
.LBB29_86:
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
	b	.LBB29_991
.LBB29_87:
	mov	w8, #62728
	movk	w8, #49184, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_166
// %bb.88:
	mov	w8, #62728
	movk	w8, #49168, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_365
// %bb.89:
	mov	w8, #22649
	movk	w8, #49176, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.90:
	ldr	x22, [x19, #32]
	mov	w0, #21
	ldr	x24, [x22, #40]
	ldr	x8, [x24, #248]
	ldr	x8, [x8, #128]
	ldr	x26, [x8, #1264]
	stp	xzr, xzr, [sp, #152]
	str	xzr, [sp, #144]
	bl	capable
	tbz	w0, #0, .LBB29_342
// %bb.91:
	ldr	x9, [x24, #1008]
	ldr	w8, [x9, #3128]
	cmp	w8, #2                  // =2
	b.lt	.LBB29_558
// %bb.92:
	ldr	x9, [x9, #3136]
	b	.LBB29_122
.LBB29_93:
	mov	w8, #62770
	movk	w8, #16392, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_185
// %bb.94:
	mov	w8, #26114
	movk	w8, #16392, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_427
// %bb.95:
	mov	w8, #62730
	movk	w8, #16392, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.96:
	ldr	x8, [x19, #32]
	mov	w0, #21
	ldr	x8, [x8, #40]
	ldr	x21, [x8, #1008]
	ldr	x8, [x21, #136]
	ldr	x24, [x8]
	str	xzr, [sp, #144]
	bl	capable
	tbz	w0, #0, .LBB29_505
// %bb.97:
	ldr	x8, [x21]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_568
// %bb.98:
	ldr	x8, [x21, #80]
	tbz	w8, #8, .LBB29_99
	b	.LBB29_1334
.LBB29_99:
	//APP
	mrs x25, sp_el0
	//NO_APP
	ldrb	w9, [x25, #62]
	ldr	x8, [x25, #8]
	tbnz	w9, #5, .LBB29_101
// %bb.100:
	ldr	x10, [x25]
	mov	x9, x20
	tbz	w10, #26, .LBB29_102
.LBB29_101:
	lsl	x9, x20, #8
	and	x9, x20, x9, asr #8
.LBB29_102:
	//APP
		adds	x9, x9, 8
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1297
// %bb.103:
	adrp	x22, arm64_const_caps_ready
	ldr	w8, [x22, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1306
// %bb.104:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1307
.LBB29_105:
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
	b	.LBB29_1310
.LBB29_106:
	mov	w8, #26244
	movk	w8, #16512, lsl #16
	cmp	w1, w8
	b.gt	.LBB29_197
// %bb.107:
	mov	w8, #62731
	movk	w8, #16408, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_436
// %bb.108:
	mov	w8, #22560
	movk	w8, #16412, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.109:
	ldr	x22, [x19, #32]
	str	wzr, [sp, #168]
	stp	xzr, xzr, [sp, #152]
	str	xzr, [sp, #144]
	str	wzr, [sp, #112]
	stp	xzr, xzr, [sp, #96]
	str	xzr, [sp, #88]
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldrb	w10, [x21, #62]
	ldr	x9, [x21, #8]
	lsl	x8, x20, #8
	tbnz	w10, #5, .LBB29_111
// %bb.110:
	ldr	x11, [x21]
	mov	x10, x20
	tbz	w11, #26, .LBB29_112
.LBB29_111:
	and	x10, x20, x8, asr #8
.LBB29_112:
	//APP
		adds	x10, x10, 28
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_887
// %bb.113:
	adrp	x24, arm64_const_caps_ready
	ldr	w9, [x24, :lo12:arm64_const_caps_ready]
	adrp	x23, cpu_hwcap_keys+16
	cmp	w9, #1                  // =1
	b.lt	.LBB29_895
// %bb.114:
	ldr	w9, [x23, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_896
.LBB29_115:
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
	b	.LBB29_899
.LBB29_116:
	mov	w8, #26137
	movk	w8, #49216, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_345
// %bb.117:
	mov	w8, #26135
	movk	w8, #49232, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_343
// %bb.118:
	mov	w8, #26138
	movk	w8, #49280, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.119:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2180]
	tbz	w8, #0, .LBB29_377
// %bb.120:
	mov	x0, x19
	mov	x1, x20
	bl	fscrypt_ioctl_get_key_status
	b	.LBB29_968
.LBB29_121:                             //   in Loop: Header=BB29_122 Depth=1
	subs	x8, x8, #1              // =1
	add	x9, x9, #88             // =88
	b.eq	.LBB29_578
.LBB29_122:                             // =>This Inner Loop Header: Depth=1
	ldr	x10, [x9]
	ldr	x10, [x10, #128]
	ldr	x10, [x10, #1264]
	ldr	x11, [x10, #104]
	tbnz	w11, #8, .LBB29_561
// %bb.123:                             //   in Loop: Header=BB29_122 Depth=1
	cbz	x10, .LBB29_121
// %bb.124:                             //   in Loop: Header=BB29_122 Depth=1
	ldr	w10, [x10, #2980]
	sub	w10, w10, #1            // =1
	cmp	w10, #2                 // =2
	b.hs	.LBB29_121
	b	.LBB29_561
.LBB29_125:
	mov	w8, #26113
	movk	w8, #32776, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_474
// %bb.126:
	mov	w8, #30209
	movk	w8, #32776, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.127:
	ldr	x11, [x19, #32]
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_129
// %bb.128:
	ldr	x12, [x8]
	mov	x10, x20
	tbz	w12, #26, .LBB29_130
.LBB29_129:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_130:
	//APP
		adds	x10, x10, 4
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_486
// %bb.131:
	lsl	x10, x20, #8
	ldr	x12, [x8, #8]
	adrp	x9, arm64_const_caps_ready
	and	x13, x20, x10, asr #8
	//APP
		bics	xzr, x13, x12
	csel	x10, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	ldr	w11, [x11, #640]
	ldr	w12, [x9, :lo12:arm64_const_caps_ready]
	cmp	w12, #1                 // =1
	b.lt	.LBB29_1088
// %bb.132:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_1089
.LBB29_133:
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
	b	.LBB29_1092
.LBB29_134:
	mov	w8, #62741
	movk	w8, #32770, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_376
// %bb.135:
	mov	w8, #22653
	movk	w8, #32772, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.136:
	ldr	x8, [x19, #32]
	mov	w0, #21
	ldr	x8, [x8, #40]
	ldr	x21, [x8, #1008]
	ldr	x22, [x21]
	bl	capable
	tbz	w0, #0, .LBB29_504
// %bb.137:
	//APP
	mrs x24, sp_el0
	//NO_APP
	ldrb	w9, [x24, #62]
	ldr	x8, [x24, #8]
	tbnz	w9, #5, .LBB29_139
// %bb.138:
	ldr	x10, [x24]
	mov	x9, x20
	tbz	w10, #26, .LBB29_140
.LBB29_139:
	lsl	x9, x20, #8
	and	x9, x20, x9, asr #8
.LBB29_140:
	//APP
		adds	x9, x9, 4
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1211
// %bb.141:
	ldr	x9, [x24, #8]
	lsl	x8, x20, #8
	and	x10, x20, x8, asr #8
	//APP
		bics	xzr, x10, x9
	csel	x8, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x9, arm64_const_caps_ready
	ldr	w10, [x9, :lo12:arm64_const_caps_ready]
	cmp	w10, #1                 // =1
	b.lt	.LBB29_1178
// %bb.142:
	adrp	x10, cpu_hwcap_keys+16
	ldr	w10, [x10, :lo12:cpu_hwcap_keys+16]
	cmp	w10, #0                 // =0
	b.le	.LBB29_1179
.LBB29_143:
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
	b	.LBB29_1182
.LBB29_144:
	mov	w8, #62801
	movk	w8, #32781, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_378
// %bb.145:
	mov	w8, #26139
	movk	w8, #32784, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.146:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2180]
	tbz	w8, #0, .LBB29_377
// %bb.147:
	mov	x0, x19
	mov	x1, x20
	bl	fscrypt_ioctl_get_nonce
	b	.LBB29_968
.LBB29_148:
	mov	w8, #62776
	movk	w8, #49160, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_379
// %bb.149:
	mov	w8, #26134
	movk	w8, #49161, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.150:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2180]
	tbz	w8, #0, .LBB29_377
// %bb.151:
	mov	x0, x19
	mov	x1, x20
	bl	fscrypt_ioctl_get_policy_ex
	b	.LBB29_968
.LBB29_152:
	mov	w8, #62790
	movk	w8, #32772, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_394
// %bb.153:
	mov	w8, #62791
	movk	w8, #32772, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.154:
	ldr	x21, [x19, #32]
	mov	x0, x21
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_485
// %bb.155:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_157
// %bb.156:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_158
.LBB29_157:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_158:
	//APP
		adds	x10, x10, 4
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_1250
// %bb.159:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x11, x20, x9, asr #8
	//APP
		bics	xzr, x11, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w11, [x10, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	b.lt	.LBB29_1185
// %bb.160:
	adrp	x11, cpu_hwcap_keys+16
	ldr	w11, [x11, :lo12:cpu_hwcap_keys+16]
	cmp	w11, #0                 // =0
	b.le	.LBB29_1186
.LBB29_161:
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
	b	.LBB29_1189
.LBB29_162:
	mov	w8, #37937
	movk	w8, #33024, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_401
// %bb.163:
	mov	w8, #26246
	movk	w8, #49156, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.164:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #2, .LBB29_377
// %bb.165:
	mov	x0, x19
	mov	x1, x20
	bl	fsverity_ioctl_measure
	b	.LBB29_968
.LBB29_166:
	mov	w8, #62729
	movk	w8, #49184, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_410
// %bb.167:
	mov	w8, #26136
	movk	w8, #49216, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.168:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2180]
	tbz	w8, #0, .LBB29_377
// %bb.169:
	mov	x0, x19
	mov	x1, x20
	bl	fscrypt_ioctl_remove_key
	b	.LBB29_968
.LBB29_170:
	mov	w8, #62770
	movk	w8, #16388, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_443
// %bb.171:
	mov	w8, #62772
	movk	w8, #16388, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.172:
	ldr	x20, [x19, #32]
	ldr	x8, [x20, #40]
	ldr	x21, [x8, #1008]
	ldr	x8, [x21, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB29_888
// %bb.173:
	ldr	x8, [x21]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_537
// %bb.174:
	mov	x0, x20
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_572
// %bb.175:
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_591
// %bb.176:
	ldr	x8, [x21, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #7, .LBB29_592
// %bb.177:
	ldr	x8, [x20, #736]
	tbz	w8, #26, .LBB29_592
// %bb.178:
	ldrsw	x8, [x20, #1184]
	add	x8, x8, #360            // =360
	cmp	x8, #400                // =400
	b.lo	.LBB29_592
// %bb.179:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w21, w0
	cbnz	w0, .LBB29_1166
// %bb.180:
	add	x22, x20, #160          // =160
	mov	x0, x22
	bl	down_write
	adrp	x1, .L__func__.f2fs_ioc_dedup_revoke
	add	x1, x1, :lo12:.L__func__.f2fs_ioc_dedup_revoke
	mov	x0, x20
	bl	f2fs_revoke_deduped_inode
	mov	w21, w0
	mov	x0, x22
	bl	up_write
	b	.LBB29_341
.LBB29_181:
	mov	w8, #62800
	movk	w8, #16397, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_456
// %bb.182:
	mov	w8, #26132
	movk	w8, #16400, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.183:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x22, [x8, #1008]
	ldr	x8, [x22, #16]
	ldrb	w8, [x8, #2180]
	tbnz	w8, #0, .LBB29_506
// %bb.184:
	mov	w23, #-95
	b	.LBB29_1454
.LBB29_185:
	mov	w8, #62771
	movk	w8, #16392, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_459
// %bb.186:
	mov	w8, #62775
	movk	w8, #16392, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.187:
	ldr	x21, [x19, #32]
	str	xzr, [sp, #120]
	ldr	x8, [x21, #40]
	ldr	x26, [x8, #1008]
	ldr	x8, [x26, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB29_889
// %bb.188:
	ldr	x8, [x26]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_629
// %bb.189:
	mov	x0, x21
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_573
// %bb.190:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_192
// %bb.191:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_193
.LBB29_192:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_193:
	//APP
		adds	x10, x10, 8
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_1298
// %bb.194:
	adrp	x22, arm64_const_caps_ready
	ldr	w9, [x22, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1313
// %bb.195:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1314
.LBB29_196:
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
	b	.LBB29_1317
.LBB29_197:
	mov	w8, #26245
	movk	w8, #16512, lsl #16
	cmp	w1, w8
	b.eq	.LBB29_472
// %bb.198:
	mov	w8, #37938
	movk	w8, #16640, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.199:
	ldr	x8, [x19, #32]
	mov	w0, #21
	ldr	x8, [x8, #40]
	ldr	x21, [x8, #1008]
	bl	capable
	tbz	w0, #0, .LBB29_530
// %bb.200:
	mov	w1, #256
	mov	x0, x20
	bl	strndup_user
	mov	x20, x0
	cmn	x0, #4095               // =4095
	b.lo	.LBB29_569
// %bb.201:
	mov	w22, w20
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_202:
	mov	w8, #62742
	movk	w8, #16386, lsl #16
	cmp	w1, w8
	b.ne	.LBB29_1455
// %bb.203:
	ldr	x22, [x19, #32]
	ldr	x8, [x22, #40]
	ldr	x21, [x8, #1008]
	strh	wzr, [sp, #144]
	ldr	x8, [x21, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #5, .LBB29_696
// %bb.204:
	ldrb	w8, [x19, #68]
	tbz	w8, #1, .LBB29_326
// %bb.205:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_207
// %bb.206:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_208
.LBB29_207:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_208:
	//APP
		adds	x10, x10, 2
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_1402
// %bb.209:
	adrp	x23, arm64_const_caps_ready
	ldr	w9, [x23, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1403
// %bb.210:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1404
.LBB29_211:
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
	b	.LBB29_1407
.LBB29_212:
	ldr	x26, [x19, #32]
	ldr	x8, [x26, #40]
	ldr	x21, [x8, #1008]
	ldr	x8, [x21, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #5, .LBB29_923
// %bb.213:
	ldrh	w8, [x26]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_215
// %bb.214:
	ldr	x8, [x26, #736]
	tbnz	w8, #31, .LBB29_593
.LBB29_215:
	mov	w24, #-22
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_216:
	ldr	x22, [x19, #32]
	ldr	x8, [x22, #40]
	ldr	x21, [x8, #1008]
	ldr	x8, [x21, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #5, .LBB29_248
// %bb.217:
	ldr	w8, [x21, #1532]
	cmp	w8, #1                  // =1
	b.ne	.LBB29_248
// %bb.218:
	ldrb	w8, [x19, #68]
	tbnz	w8, #1, .LBB29_219
	b	.LBB29_1383
.LBB29_219:
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_493
// %bb.220:
	ldr	w27, [x21, #1188]
	ldr	w28, [x22, #1292]
	ldr	x8, [x22, #736]
	tbz	w8, #31, .LBB29_493
// %bb.221:
	ldr	x8, [x22, #736]
	tbnz	w8, #10, .LBB29_694
// %bb.222:
	mov	x0, x22
	bl	dquot_initialize
	add	w8, w0, #28             // =28
	mov	w25, w0
	cmp	w8, #28                 // =28
	b.hi	.LBB29_224
// %bb.223:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB29_225
.LBB29_224:
	cmn	w25, #122               // =122
	b.ne	.LBB29_1391
.LBB29_225:
	ldr	x8, [x22, #40]
	mov	w1, #1
	ldr	x0, [x8, #1008]
	bl	f2fs_balance_fs
	ldr	x8, [x19, #32]
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB29_227
// %bb.226:
	ldr	x0, [x8, #40]
	mov	w1, #1
	mov	w2, #1
	bl	__sb_start_write
.LBB29_227:
	add	x26, x22, #160          // =160
	mov	x0, x26
	bl	down_write
	add	x24, x21, #368          // =368
	mov	x0, x24
	bl	_raw_spin_lock
	ldr	x8, [x21, #352]
	cbz	x8, .LBB29_229
// %bb.228:
	adrp	x23, jiffies
	ldr	x8, [x21, #360]
	ldr	x9, [x23, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB29_975
.LBB29_229:
	mov	x0, x24
	bl	_raw_spin_unlock
	add	x24, x21, #304          // =304
	mov	x0, x24
	bl	down_read
	mov	x0, x22
	bl	f2fs_drop_extent_tree
	mov	x0, x24
	bl	up_read
	ldr	x8, [x22, #736]
	tbnz	x8, #35, .LBB29_792
// %bb.230:
	str	x26, [sp, #72]          // 8-byte Folded Spill
	ldr	x8, [x22, #736]
	tbnz	x8, #38, .LBB29_821
// %bb.231:
	mov	x0, x22
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB29_1112
// %bb.232:
	ldr	x8, [x22, #736]
	tbnz	w8, #15, .LBB29_886
// %bb.233:
	ldr	x8, [x22, #736]
	tbnz	w8, #28, .LBB29_1110
// %bb.234:
	ldr	x8, [x22, #736]
	tbz	x8, #33, .LBB29_235
	b	.LBB29_1287
.LBB29_235:
	ldr	x0, [x22, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	cbz	w0, .LBB29_1461
// %bb.236:
	mov	w25, w0
	b	.LBB29_1611
.LBB29_237:
	ldr	x22, [x19, #32]
	ldr	x8, [x22, #40]
	ldr	x21, [x8, #1008]
	ldr	x8, [x21, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #5, .LBB29_248
// %bb.238:
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_493
// %bb.239:
	ldr	x8, [x22, #736]
	tbz	w8, #31, .LBB29_493
// %bb.240:
	ldr	x8, [x21]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_629
// %bb.241:
	mov	x0, x22
	bl	dquot_initialize
	add	w8, w0, #28             // =28
	mov	w25, w0
	cmp	w8, #28                 // =28
	b.hi	.LBB29_243
// %bb.242:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB29_244
.LBB29_243:
	cmn	w25, #122               // =122
	b.ne	.LBB29_1391
.LBB29_244:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w25, w0
	cbnz	w0, .LBB29_1391
// %bb.245:
	ldr	w8, [x22, #1280]
	cbz	w8, .LBB29_782
// %bb.246:
	mov	x0, x19
	bl	mnt_drop_write_file
	mov	w23, wzr
	b	.LBB29_842
.LBB29_247:
	ldr	x22, [x19, #32]
	ldr	x8, [x22, #40]
	ldr	x21, [x8, #1008]
	ldr	x8, [x21, #16]
	ldrb	w8, [x8, #2181]
	tbnz	w8, #5, .LBB29_491
.LBB29_248:
	mov	w25, #-95
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_249:
	ldr	x21, [x19, #32]
	ldr	x8, [x21, #40]
	ldr	x8, [x8, #1008]
	str	xzr, [sp, #144]
	ldr	x9, [x8, #224]
	ldrb	w9, [x9, #132]
	tbnz	w9, #3, .LBB29_837
// %bb.250:
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #7, .LBB29_377
// %bb.251:
	ldr	x8, [x21, #736]
	tbz	w8, #26, .LBB29_377
// %bb.252:
	ldrsw	x8, [x21, #1184]
	add	x8, x8, #360            // =360
	cmp	x8, #400                // =400
	b.lo	.LBB29_377
// %bb.253:
	add	x19, x21, #160          // =160
	mov	x0, x19
	bl	down_write
	ldr	x8, [x21, #736]
	tbnz	x8, #38, .LBB29_643
// %bb.254:
	strh	wzr, [sp, #144]
	b	.LBB29_647
.LBB29_255:
	ldr	x21, [x19, #32]
	ldr	x8, [x21, #40]
	ldr	x20, [x8, #1008]
	ldr	x8, [x20, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #5, .LBB29_696
// %bb.256:
	ldr	w8, [x20, #1532]
	cmp	w8, #1                  // =1
	b.ne	.LBB29_696
// %bb.257:
	ldrb	w8, [x19, #68]
	tbz	w8, #1, .LBB29_326
// %bb.258:
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_1417
// %bb.259:
	ldr	w26, [x20, #1188]
	ldr	w27, [x21, #1292]
	ldr	x8, [x21, #736]
	tbnz	w8, #31, .LBB29_260
	b	.LBB29_1417
.LBB29_260:
	ldr	x8, [x21, #736]
	tbnz	w8, #10, .LBB29_695
// %bb.261:
	mov	x0, x21
	bl	dquot_initialize
	add	w8, w0, #28             // =28
	mov	w23, w0
	cmp	w8, #28                 // =28
	b.hi	.LBB29_263
// %bb.262:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB29_264
.LBB29_263:
	cmn	w23, #122               // =122
	b.ne	.LBB29_1454
.LBB29_264:
	ldr	x8, [x21, #40]
	mov	w1, #1
	ldr	x0, [x8, #1008]
	bl	f2fs_balance_fs
	ldr	x8, [x19, #32]
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB29_266
// %bb.265:
	ldr	x0, [x8, #40]
	mov	w1, #1
	mov	w2, #1
	bl	__sb_start_write
.LBB29_266:
	add	x22, x21, #160          // =160
	mov	x0, x22
	bl	down_write
	ldr	x8, [x21, #736]
	tbnz	x8, #38, .LBB29_793
// %bb.267:
	ldr	x8, [x21, #736]
	tbz	x8, #35, .LBB29_269
// %bb.268:
	mov	w1, #9
	mov	x0, x21
	bl	f2fs_reserve_compress_blocks_internal
	mov	w23, w0
	cbnz	w0, .LBB29_860
.LBB29_269:
	mov	x0, x21
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB29_827
// %bb.270:
	ldr	x8, [x21, #736]
	tbnz	x8, #33, .LBB29_859
// %bb.271:
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w23, w0
	cbnz	w0, .LBB29_860
// %bb.272:
	ldr	w8, [x21, #1280]
	cbz	w8, .LBB29_1434
// %bb.273:
	ldr	x8, [x21, #80]
	add	x8, x8, #4095           // =4095
	lsr	x28, x8, #12
	cbz	w28, .LBB29_1459
// %bb.274:
	mov	x24, xzr
	b	.LBB29_823
.LBB29_275:
	ldr	x0, [x19, #32]
	bl	f2fs_precache_extents
	b	.LBB29_968
.LBB29_276:
	ldr	x20, [x19, #32]
	mov	x0, x20
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_435
// %bb.277:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w22, w0
	cbnz	w0, .LBB29_668
// %bb.278:
	add	x21, x20, #160          // =160
	mov	x0, x21
	bl	down_write
	ldr	x8, [x20, #736]
	tbz	w8, #15, .LBB29_280
// %bb.279:
	mov	x0, x20
	bl	f2fs_drop_inmem_pages
.LBB29_280:
	ldr	x8, [x20, #736]
	tbnz	w8, #17, .LBB29_630
// %bb.281:
	mov	w22, wzr
	b	.LBB29_631
.LBB29_282:
	ldr	x20, [x19, #32]
	mov	x0, x20
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_435
// %bb.283:
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_541
// %bb.284:
	ldrb	w8, [x19, #66]
	tbnz	w8, #0, .LBB29_541
// %bb.285:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w22, w0
	cbnz	w0, .LBB29_668
// %bb.286:
	add	x21, x20, #160          // =160
	mov	x0, x21
	bl	down_write
	ldr	x8, [x20, #736]
	tbz	x8, #35, .LBB29_288
// %bb.287:
	mov	w1, #3
	mov	x0, x20
	bl	f2fs_reserve_compress_blocks_internal
	mov	w22, w0
	cbnz	w0, .LBB29_665
.LBB29_288:
	ldr	x8, [x20, #736]
	tbz	x8, #44, .LBB29_290
// %bb.289:
	add	x8, x20, #736           // =736
	mov	x9, #17592186044416
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_mark_inode_dirty_sync
.LBB29_290:
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB29_294
// %bb.291:
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB29_293
// %bb.292:
	ldr	x8, [x20, #736]
	tbnz	x8, #39, .LBB29_294
.LBB29_293:
	adrp	x1, .L__func__.f2fs_ioc_start_volatile_write
	add	x1, x1, :lo12:.L__func__.f2fs_ioc_start_volatile_write
	mov	x0, x20
	bl	f2fs_revoke_deduped_inode
	mov	w22, w0
	cbnz	w0, .LBB29_665
.LBB29_294:
	ldr	x8, [x20, #736]
	tbnz	w8, #17, .LBB29_693
// %bb.295:
	mov	x0, x20
	bl	f2fs_convert_inline_inode
	mov	w22, w0
	cbnz	w0, .LBB29_665
// %bb.296:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	add	x8, x8, #2232           // =2232
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	ldr	w9, [x8, #2232]
	ldr	w10, [x8, #2240]
	cmp	w9, w10
	b.le	.LBB29_298
// %bb.297:
	str	w9, [x8, #2240]
.LBB29_298:
	add	x8, x20, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 131072
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x20, #40]
	adrp	x9, jiffies
	mov	w22, wzr
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	b	.LBB29_665
.LBB29_299:
	ldr	x21, [x19, #32]
	mov	x0, x21
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_435
// %bb.300:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w22, w0
	cbnz	w0, .LBB29_668
// %bb.301:
	add	x20, x21, #160          // =160
	mov	x0, x20
	bl	down_write
	ldr	x8, [x21, #736]
	tbnz	w8, #17, .LBB29_632
// %bb.302:
	mov	w22, wzr
	b	.LBB29_659
.LBB29_303:
	ldr	x20, [x19, #32]
	mov	x0, x20
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_435
// %bb.304:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w22, w0
	cbnz	w0, .LBB29_668
// %bb.305:
	ldr	x8, [x20, #40]
	mov	w1, #1
	ldr	x0, [x8, #1008]
	bl	f2fs_balance_fs
	add	x21, x20, #160          // =160
	mov	x0, x21
	bl	down_write
	ldr	x8, [x20, #736]
	tbnz	w8, #17, .LBB29_634
// %bb.306:
	ldr	x8, [x20, #736]
	tbnz	w8, #15, .LBB29_660
// %bb.307:
	mov	x2, #9223372036854775807
	mov	w3, #1
	mov	x0, x19
	mov	x1, xzr
	mov	w4, wzr
	bl	f2fs_do_sync_file
	mov	w22, w0
	b	.LBB29_663
.LBB29_308:
	ldr	x22, [x19, #32]
	ldr	x8, [x22, #40]
	ldr	x20, [x8, #1008]
	ldr	x8, [x20, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #5, .LBB29_696
// %bb.309:
	ldr	w8, [x20, #1532]
	cmp	w8, #1                  // =1
	b.ne	.LBB29_696
// %bb.310:
	ldrb	w8, [x19, #68]
	tbz	w8, #1, .LBB29_326
// %bb.311:
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_1417
// %bb.312:
	ldr	w26, [x20, #1188]
	ldr	w27, [x22, #1292]
	ldr	x8, [x22, #736]
	tbnz	w8, #31, .LBB29_313
	b	.LBB29_1417
.LBB29_313:
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_balance_fs
	ldr	x8, [x19, #32]
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB29_315
// %bb.314:
	ldr	x0, [x8, #40]
	mov	w1, #1
	mov	w2, #1
	bl	__sb_start_write
.LBB29_315:
	add	x21, x22, #160          // =160
	mov	x0, x21
	bl	down_write
	add	x23, x20, #368          // =368
	mov	x0, x23
	bl	_raw_spin_lock
	ldr	x8, [x20, #352]
	cbz	x8, .LBB29_317
// %bb.316:
	adrp	x24, jiffies
	ldr	x8, [x20, #360]
	ldr	x9, [x24, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB29_858
.LBB29_317:
	mov	x0, x23
	bl	_raw_spin_unlock
	add	x23, x20, #304          // =304
	mov	x0, x23
	bl	down_read
	mov	x0, x22
	bl	f2fs_drop_extent_tree
	mov	x0, x23
	bl	up_read
	ldr	x8, [x22, #736]
	tbnz	x8, #38, .LBB29_676
// %bb.318:
	mov	x0, x22
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB29_697
// %bb.319:
	ldr	x8, [x22, #736]
	tbnz	x8, #33, .LBB29_780
// %bb.320:
	ldr	x0, [x22, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w23, w0
	cbnz	w0, .LBB29_781
// %bb.321:
	add	x10, x22, #736          // =736
	str	x10, [sp, #72]          // 8-byte Folded Spill
	//APP
	// atomic64_or
	prfm	pstl1strm, [x10]
1:	ldxr	x8, [x10]
	orr	x8, x8, 17179869184
	stxr	w9, x8, [x10]
	cbnz	w9, 1b
	//NO_APP
	ldr	x8, [x22, #80]
	add	x8, x8, #4095           // =4095
	lsr	x28, x8, #12
	cbz	w28, .LBB29_978
// %bb.322:
	mov	x24, xzr
	b	.LBB29_699
.LBB29_323:
	ldr	x22, [x19, #32]
	ldr	x8, [x22, #40]
	ldr	x20, [x8, #1008]
	ldr	x8, [x20, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #5, .LBB29_696
// %bb.324:
	ldr	w8, [x20, #1532]
	cmp	w8, #1                  // =1
	b.ne	.LBB29_696
// %bb.325:
	ldrb	w8, [x19, #68]
	tbnz	w8, #1, .LBB29_580
.LBB29_326:
	mov	w23, #-9
	sxtw	x0, w23
	b	.LBB29_1455
.LBB29_327:
	ldr	x19, [x19, #32]
	ldr	x8, [x19, #40]
	ldr	x20, [x8, #1008]
	ldr	x8, [x20, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB29_837
// %bb.328:
	ldr	x8, [x20]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_484
// %bb.329:
	mov	x0, x19
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_557
// %bb.330:
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_487
// %bb.331:
	ldr	x8, [x20, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #7, .LBB29_377
// %bb.332:
	ldr	x8, [x19, #736]
	tbz	w8, #26, .LBB29_377
// %bb.333:
	ldrsw	x8, [x19, #1184]
	add	x8, x8, #360            // =360
	cmp	x8, #400                // =400
	b.lo	.LBB29_377
// %bb.334:
	ldr	x8, [x19, #736]
	tbnz	w8, #10, .LBB29_487
// %bb.335:
	mov	w2, #1
	mov	x0, x20
	mov	x1, x19
	bl	f2fs_inode_param_check
	b	.LBB29_968
.LBB29_336:
	ldr	x8, [x19, #32]
	mov	w0, #21
	ldr	x8, [x8, #40]
	ldr	x20, [x8, #1008]
	bl	capable
	tbz	w0, #0, .LBB29_342
// %bb.337:
	ldr	x8, [x20]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_556
// %bb.338:
	ldr	x8, [x20, #80]
	tbz	w8, #8, .LBB29_339
	b	.LBB29_1288
.LBB29_339:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w21, w0
	cbnz	w0, .LBB29_1166
// %bb.340:
	ldr	x0, [x20]
	mov	w1, #1
	bl	f2fs_sync_fs
	mov	w21, w0
.LBB29_341:
	mov	x0, x19
	bl	mnt_drop_write_file
	b	.LBB29_1166
.LBB29_342:
	mov	w21, #-1
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_343:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2180]
	tbz	w8, #0, .LBB29_377
// %bb.344:
	mov	x0, x19
	mov	x1, x20
	bl	fscrypt_ioctl_add_key
	b	.LBB29_968
.LBB29_345:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2180]
	tbz	w8, #0, .LBB29_377
// %bb.346:
	mov	x0, x19
	mov	x1, x20
	bl	fscrypt_ioctl_remove_key_all_users
	b	.LBB29_968
.LBB29_347:
	ldr	x21, [x19, #32]
	add	x19, x21, #160          // =160
	mov	x0, x19
	bl	down_write
	ldr	x21, [x21, #736]
	mov	x0, x19
	bl	up_write
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_349
// %bb.348:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_350
.LBB29_349:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_350:
	//APP
		adds	x10, x10, 1
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_555
// %bb.351:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x11, x20, x9, asr #8
	//APP
		bics	xzr, x11, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w11, [x10, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	ubfx	x11, x21, #35, #1
	b.lt	.LBB29_994
// %bb.352:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_995
.LBB29_353:
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
	b	.LBB29_998
.LBB29_354:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x9, [x8, #16]
	ldrb	w9, [x9, #2180]
	tbz	w9, #0, .LBB29_377
// %bb.355:
	adrp	x9, jiffies
	ldr	x9, [x9, :lo12:jiffies]
	mov	x0, x19
	mov	x1, x20
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	bl	fscrypt_ioctl_set_policy
	b	.LBB29_968
.LBB29_356:
	mov	x0, x19
	mov	x1, x20
	bl	f2fs_ioc_set_hp_file
	b	.LBB29_968
.LBB29_357:
	ldr	x8, [x19, #32]
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x9, [x8, #16]
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	w11, [x9, #2180]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_359
// %bb.358:
	ldr	x12, [x8]
	mov	x10, x20
	tbz	w12, #26, .LBB29_360
.LBB29_359:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_360:
	//APP
		adds	x10, x10, 4
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_555
// %bb.361:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x12, x20, x9, asr #8
	//APP
		bics	xzr, x12, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w12, [x10, :lo12:arm64_const_caps_ready]
	orr	w11, w11, #0x4
	cmp	w12, #1                 // =1
	b.lt	.LBB29_1001
// %bb.362:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_1002
.LBB29_363:
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
	b	.LBB29_1005
.LBB29_364:
	mov	x0, xzr
	b	.LBB29_1455
.LBB29_365:
	ldr	x22, [x19, #32]
	mov	w0, #21
	ldr	x8, [x22, #40]
	ldr	x21, [x8, #1008]
	stp	xzr, xzr, [sp, #88]
	bl	capable
	tbz	w0, #0, .LBB29_531
// %bb.366:
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_545
// %bb.367:
	ldr	x8, [x22, #736]
	tbnz	w8, #15, .LBB29_545
// %bb.368:
	ldr	x8, [x21]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_655
// %bb.369:
	//APP
	mrs x26, sp_el0
	//NO_APP
	ldrb	w9, [x26, #62]
	ldr	x8, [x26, #8]
	tbnz	w9, #5, .LBB29_371
// %bb.370:
	ldr	x10, [x26]
	mov	x9, x20
	tbz	w10, #26, .LBB29_372
.LBB29_371:
	lsl	x9, x20, #8
	and	x9, x20, x9, asr #8
.LBB29_372:
	//APP
		adds	x9, x9, 16
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1518
// %bb.373:
	adrp	x27, arm64_const_caps_ready
	ldr	w8, [x27, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1519
// %bb.374:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1520
.LBB29_375:
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
	b	.LBB29_1523
.LBB29_376:
	ldr	x21, [x19, #32]
	strh	wzr, [sp, #144]
	ldr	x8, [x21, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2181]
	tbnz	w8, #5, .LBB29_532
.LBB29_377:
	mov	x0, #-95
	b	.LBB29_1455
.LBB29_378:
	mov	x0, x19
	mov	x1, x20
	bl	f2fs_fscrypt_ioctl_get_sdp_policy
	b	.LBB29_968
.LBB29_379:
	ldr	x21, [x19, #32]
	str	xzr, [sp, #88]
	ldr	x8, [x21, #40]
	ldr	x23, [x8, #1008]
	ldr	x8, [x23, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB29_890
// %bb.380:
	ldr	x8, [x23]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_538
// %bb.381:
	mov	x0, x21
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_485
// %bb.382:
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_579
// %bb.383:
	ldr	x8, [x23, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #7, .LBB29_1252
// %bb.384:
	ldr	x8, [x21, #736]
	tbz	w8, #26, .LBB29_1252
// %bb.385:
	ldrsw	x8, [x21, #1184]
	add	x8, x8, #360            // =360
	cmp	x8, #400                // =400
	b.lo	.LBB29_1252
// %bb.386:
	ldr	x8, [x21, #736]
	tbnz	w8, #10, .LBB29_579
// %bb.387:
	//APP
	mrs x26, sp_el0
	//NO_APP
	ldrb	w9, [x26, #62]
	ldr	x8, [x26, #8]
	tbnz	w9, #5, .LBB29_389
// %bb.388:
	ldr	x10, [x26]
	mov	x9, x20
	tbz	w10, #26, .LBB29_390
.LBB29_389:
	lsl	x9, x20, #8
	and	x9, x20, x9, asr #8
.LBB29_390:
	//APP
		adds	x9, x9, 8
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1765
// %bb.391:
	adrp	x27, arm64_const_caps_ready
	ldr	w8, [x27, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1766
// %bb.392:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1767
.LBB29_393:
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
	b	.LBB29_1770
.LBB29_394:
	ldr	x9, [x19, #32]
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x11, [x9, #704]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_396
// %bb.395:
	ldr	x12, [x8]
	mov	x10, x20
	tbz	w12, #26, .LBB29_397
.LBB29_396:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_397:
	//APP
		adds	x10, x10, 4
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_555
// %bb.398:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x12, x20, x9, asr #8
	//APP
		bics	xzr, x12, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w12, [x10, :lo12:arm64_const_caps_ready]
	mov	w13, #2
	movk	w13, #1552, lsl #16
	and	w11, w11, w13
	cmp	w12, #1                 // =1
	b.lt	.LBB29_1008
// %bb.399:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_1009
.LBB29_400:
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
	b	.LBB29_1012
.LBB29_401:
	ldr	x8, [x19, #32]
	adrp	x9, kmalloc_caches+72
	ldr	x0, [x9, :lo12:kmalloc_caches+72]
	mov	w1, #3520
	ldr	x8, [x8, #40]
	mov	w2, #512
	ldr	x22, [x8, #1008]
	bl	kmem_cache_alloc_trace
	cbz	x0, .LBB29_546
// %bb.402:
	add	x21, x22, #24           // =24
	mov	x19, x0
	mov	x0, x21
	bl	down_read
	ldr	x8, [x22, #16]
	mov	w1, #512
	mov	w2, #1
	mov	w4, #512
	add	x0, x8, #124            // =124
	mov	x3, x19
	bl	utf16s_to_utf8s
	mov	w22, w0
	mov	x0, x21
	bl	up_read
	cmp	w22, #256               // =256
	mov	w8, #256
	csel	w8, w22, w8, lt
	sxtw	x21, w8
	tbnz	w8, #31, .LBB29_1015
// %bb.403:
	mov	w2, #1
	mov	x0, x19
	mov	x1, x21
	bl	__check_object_size
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_405
// %bb.404:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_406
.LBB29_405:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_406:
	//APP
		adds	x10, x10, x21
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_1150
// %bb.407:
	adrp	x22, arm64_const_caps_ready
	ldr	w9, [x22, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1139
// %bb.408:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1140
.LBB29_409:
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
	b	.LBB29_1143
.LBB29_410:
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_412
// %bb.411:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_413
.LBB29_412:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_413:
	//APP
		adds	x10, x10, 32
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_891
// %bb.414:
	adrp	x21, arm64_const_caps_ready
	ldr	w9, [x21, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_902
// %bb.415:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_903
.LBB29_416:
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
	b	.LBB29_906
.LBB29_417:
	ldr	x8, [x19, #32]
	mov	w0, #21
	ldr	x8, [x8, #40]
	ldr	x21, [x8, #1008]
	bl	capable
	tbz	w0, #0, .LBB29_535
// %bb.418:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldrb	w9, [x23, #62]
	ldr	x8, [x23, #8]
	tbnz	w9, #5, .LBB29_420
// %bb.419:
	ldr	x10, [x23]
	mov	x9, x20
	tbz	w10, #26, .LBB29_421
.LBB29_420:
	lsl	x9, x20, #8
	and	x9, x20, x9, asr #8
.LBB29_421:
	//APP
		adds	x9, x9, 4
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1273
// %bb.422:
	ldr	x9, [x23, #8]
	lsl	x8, x20, #8
	and	x10, x20, x8, asr #8
	//APP
		bics	xzr, x10, x9
	csel	x8, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x9, arm64_const_caps_ready
	ldr	w10, [x9, :lo12:arm64_const_caps_ready]
	cmp	w10, #1                 // =1
	b.lt	.LBB29_1192
// %bb.423:
	adrp	x10, cpu_hwcap_keys+16
	ldr	w10, [x10, :lo12:cpu_hwcap_keys+16]
	cmp	w10, #0                 // =0
	b.le	.LBB29_1193
.LBB29_424:
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
	b	.LBB29_1196
.LBB29_425:
	cbz	x19, .LBB29_487
// %bb.426:
	ldr	x8, [x19, #32]
	adrp	x9, jiffies
	mov	x0, x19
	mov	x1, x20
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	bl	f2fs_fscrypt_ioctl_set_dps_policy
	b	.LBB29_968
.LBB29_427:
	ldr	x21, [x19, #32]
	mov	x0, x21
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_435
// %bb.428:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_430
// %bb.429:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_431
.LBB29_430:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_431:
	//APP
		adds	x10, x10, 4
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_1273
// %bb.432:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x11, x20, x9, asr #8
	//APP
		bics	xzr, x11, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w11, [x10, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	b.lt	.LBB29_1199
// %bb.433:
	adrp	x11, cpu_hwcap_keys+16
	ldr	w11, [x11, :lo12:cpu_hwcap_keys+16]
	cmp	w11, #0                 // =0
	b.le	.LBB29_1200
.LBB29_434:
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
	b	.LBB29_1203
.LBB29_435:
	mov	w22, #-13
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_436:
	stp	xzr, xzr, [sp, #152]
	str	xzr, [sp, #144]
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_438
// %bb.437:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_439
.LBB29_438:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_439:
	//APP
		adds	x10, x10, 24
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_892
// %bb.440:
	adrp	x21, arm64_const_caps_ready
	ldr	w9, [x21, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_909
// %bb.441:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_910
.LBB29_442:
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
	b	.LBB29_913
.LBB29_443:
	ldr	x21, [x19, #32]
	ldr	x8, [x21, #40]
	ldr	x26, [x8, #1008]
	str	wzr, [sp, #88]
	ldr	x8, [x26, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB29_893
// %bb.444:
	ldr	x8, [x26]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_539
// %bb.445:
	mov	x0, x21
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_574
// %bb.446:
	ldr	x8, [x26, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #7, .LBB29_576
// %bb.447:
	ldr	x8, [x21, #736]
	tbz	w8, #26, .LBB29_576
// %bb.448:
	ldrsw	x8, [x21, #1184]
	add	x8, x8, #360            // =360
	cmp	x8, #400                // =400
	b.lo	.LBB29_576
// %bb.449:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldrb	w9, [x23, #62]
	ldr	x8, [x23, #8]
	tbnz	w9, #5, .LBB29_451
// %bb.450:
	ldr	x10, [x23]
	mov	x9, x20
	tbz	w10, #26, .LBB29_452
.LBB29_451:
	lsl	x9, x20, #8
	and	x9, x20, x9, asr #8
.LBB29_452:
	//APP
		adds	x9, x9, 4
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1612
// %bb.453:
	adrp	x22, arm64_const_caps_ready
	ldr	w8, [x22, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1614
// %bb.454:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1615
.LBB29_455:
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
	b	.LBB29_1618
.LBB29_456:
	mov	x0, #-22
	cbz	x19, .LBB29_1455
// %bb.457:
	cbz	x20, .LBB29_1455
// %bb.458:
	ldr	x8, [x19, #32]
	adrp	x9, jiffies
	mov	x0, x19
	mov	x1, x20
	ldr	x8, [x8, #40]
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	bl	f2fs_fscrypt_ioctl_set_sdp_policy
	b	.LBB29_968
.LBB29_459:
	ldr	x23, [x19, #32]
	ldr	x8, [x23, #40]
	ldr	x27, [x8, #1008]
	str	xzr, [sp, #88]
	ldr	x8, [x27, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB29_894
// %bb.460:
	ldr	x8, [x27]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_540
// %bb.461:
	mov	x0, x23
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_575
// %bb.462:
	ldr	x8, [x27, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #7, .LBB29_577
// %bb.463:
	ldr	x8, [x23, #736]
	tbz	w8, #26, .LBB29_577
// %bb.464:
	ldrsw	x8, [x23, #1184]
	add	x8, x8, #360            // =360
	cmp	x8, #400                // =400
	b.lo	.LBB29_577
// %bb.465:
	//APP
	mrs x24, sp_el0
	//NO_APP
	ldrb	w9, [x24, #62]
	ldr	x8, [x24, #8]
	tbnz	w9, #5, .LBB29_467
// %bb.466:
	ldr	x10, [x24]
	mov	x9, x20
	tbz	w10, #26, .LBB29_468
.LBB29_467:
	lsl	x9, x20, #8
	and	x9, x20, x9, asr #8
.LBB29_468:
	//APP
		adds	x9, x9, 8
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1613
// %bb.469:
	adrp	x21, arm64_const_caps_ready
	ldr	w8, [x21, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1621
// %bb.470:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1622
.LBB29_471:
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
	b	.LBB29_1625
.LBB29_472:
	ldr	x8, [x19, #32]
	adrp	x10, jiffies
	ldr	x9, [x8, #40]
	ldr	x9, [x9, #1008]
	ldr	x10, [x10, :lo12:jiffies]
	stp	x10, x10, [x9, #504]
	str	x10, [x9, #520]
	ldr	x9, [x8, #40]
	ldr	x0, [x9, #1008]
	ldr	x9, [x0, #16]
	ldrb	w9, [x9, #2181]
	tbnz	w9, #2, .LBB29_536
// %bb.473:
	ldr	x2, [x8, #64]
	adrp	x1, .L.str.44
	add	x1, x1, :lo12:.L.str.44
	bl	f2fs_printk
	mov	x0, #-95
	b	.LBB29_1455
.LBB29_474:
	ldr	x8, [x19, #32]
	mov	w9, #1276
	movk	w9, #24577, lsl #16
	orr	w9, w9, #0x1000
	ldr	w10, [x8, #704]
	ldr	w11, [x8, #12]
	ldr	x12, [x8, #736]
	and	w9, w10, w9
	lsr	w10, w11, #16
	lsr	w11, w11, #3
	and	w11, w11, #0x800
	orr	w9, w11, w9
	bfi	w9, w10, #20, #1
	tbnz	w12, #10, .LBB29_476
// %bb.475:
	ldr	x10, [x8, #736]
	tbz	w10, #11, .LBB29_477
.LBB29_476:
	orr	w9, w9, #0x10000000
.LBB29_477:
	ldr	x10, [x8, #736]
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w12, [x8, #62]
	ldr	x11, [x8, #8]
	tbnz	w12, #5, .LBB29_479
// %bb.478:
	ldr	x13, [x8]
	mov	x12, x20
	tbz	w13, #26, .LBB29_480
.LBB29_479:
	lsl	x12, x20, #8
	and	x12, x20, x12, asr #8
.LBB29_480:
	//APP
		adds	x12, x12, 4
	csel	x11, xzr, x11, hi
	csinv	x12, x12, xzr, cc
	sbcs	xzr, x12, x11
	cset	x12, ls

	//NO_APP
	cbz	x12, .LBB29_486
// %bb.481:
	ldr	x11, [x8, #8]
	ubfx	x12, x10, #5, #27
	lsl	x10, x20, #8
	and	x13, x20, x10, asr #8
	//APP
		bics	xzr, x13, x11
	csel	x10, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x11, arm64_const_caps_ready
	ldr	w13, [x11, :lo12:arm64_const_caps_ready]
	mov	w14, #7422
	movk	w14, #30353, lsl #16
	and	w12, w12, #0x800000
	and	w9, w9, w14
	cmp	w13, #1                 // =1
	orr	w9, w12, w9
	b.lt	.LBB29_1099
// %bb.482:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_1100
.LBB29_483:
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
	b	.LBB29_1103
.LBB29_484:
	mov	x0, #-30
	b	.LBB29_1455
.LBB29_485:
	mov	w0, #-13
	sxtw	x0, w0
	b	.LBB29_1455
.LBB29_486:
	mov	w8, #-14
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_487:
	mov	x0, #-22
	b	.LBB29_1455
.LBB29_488:
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB29_490
// %bb.489:
	ldr	x9, [x8, #736]
	tbnz	w9, #31, .LBB29_603
.LBB29_490:
	mov	w9, #-22
	sxtw	x0, w9
	b	.LBB29_1455
.LBB29_491:
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_493
// %bb.492:
	ldr	x8, [x22, #736]
	tbnz	w8, #31, .LBB29_611
.LBB29_493:
	mov	w25, #-22
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_494:
	mov	w23, #-13
	b	.LBB29_1454
.LBB29_495:
	mov	x0, #-1
	b	.LBB29_1455
.LBB29_496:
	ldr	w8, [x8, #720]
.LBB29_497:
	//APP
	mrs x9, sp_el0
	//NO_APP
	ldrb	w11, [x9, #62]
	ldr	x10, [x9, #8]
	tbnz	w11, #5, .LBB29_499
// %bb.498:
	ldr	x12, [x9]
	mov	x11, x20
	tbz	w12, #26, .LBB29_500
.LBB29_499:
	lsl	x11, x20, #8
	and	x11, x20, x11, asr #8
.LBB29_500:
	//APP
		adds	x11, x11, 4
	csel	x10, xzr, x10, hi
	csinv	x11, x11, xzr, cc
	sbcs	xzr, x11, x10
	cset	x11, ls

	//NO_APP
	cbz	x11, .LBB29_610
// %bb.501:
	ldr	x11, [x9, #8]
	lsl	x10, x20, #8
	and	x12, x20, x10, asr #8
	//APP
		bics	xzr, x12, x11
	csel	x10, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x11, arm64_const_caps_ready
	ldr	w12, [x11, :lo12:arm64_const_caps_ready]
	cmp	w12, #1                 // =1
	b.lt	.LBB29_1016
// %bb.502:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_1017
.LBB29_503:
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
	b	.LBB29_1020
.LBB29_504:
	mov	w23, #-1
	b	.LBB29_1454
.LBB29_505:
	mov	w20, #-1
	sxtw	x0, w20
	b	.LBB29_1455
.LBB29_506:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w23, w0
	cbnz	w0, .LBB29_1454
// %bb.507:
	add	x21, x22, #24           // =24
	mov	x0, x21
	bl	down_write
	ldr	x8, [x22, #16]
	ldrb	w9, [x8, #2185]
	cbnz	w9, .LBB29_523
// %bb.508:
	ldrb	w9, [x8, #2186]
	cbnz	w9, .LBB29_523
// %bb.509:
	ldrb	w9, [x8, #2187]
	cbnz	w9, .LBB29_523
// %bb.510:
	ldrb	w9, [x8, #2188]
	cbnz	w9, .LBB29_523
// %bb.511:
	ldrb	w9, [x8, #2189]
	cbnz	w9, .LBB29_523
// %bb.512:
	ldrb	w9, [x8, #2190]
	cbnz	w9, .LBB29_523
// %bb.513:
	ldrb	w9, [x8, #2191]
	cbnz	w9, .LBB29_523
// %bb.514:
	ldrb	w9, [x8, #2192]
	cbnz	w9, .LBB29_523
// %bb.515:
	ldrb	w9, [x8, #2193]
	cbnz	w9, .LBB29_523
// %bb.516:
	ldrb	w9, [x8, #2194]
	cbnz	w9, .LBB29_523
// %bb.517:
	ldrb	w9, [x8, #2195]
	cbnz	w9, .LBB29_523
// %bb.518:
	ldrb	w9, [x8, #2196]
	cbnz	w9, .LBB29_523
// %bb.519:
	ldrb	w9, [x8, #2197]
	cbnz	w9, .LBB29_523
// %bb.520:
	ldrb	w9, [x8, #2198]
	cbnz	w9, .LBB29_523
// %bb.521:
	ldrb	w9, [x8, #2199]
	cbnz	w9, .LBB29_523
// %bb.522:
	ldrb	w9, [x8, #2200]
	cbz	w9, .LBB29_1949
.LBB29_523:
	//APP
	mrs x9, sp_el0
	//NO_APP
	ldrb	w11, [x9, #62]
	ldr	x10, [x9, #8]
	tbnz	w11, #5, .LBB29_525
// %bb.524:
	ldr	x12, [x9]
	mov	x11, x20
	tbz	w12, #26, .LBB29_526
.LBB29_525:
	lsl	x11, x20, #8
	and	x11, x20, x11, asr #8
.LBB29_526:
	//APP
		adds	x11, x11, 16
	csel	x10, xzr, x10, hi
	csinv	x11, x11, xzr, cc
	sbcs	xzr, x11, x10
	cset	x11, ls

	//NO_APP
	cbz	x11, .LBB29_642
// %bb.527:
	adrp	x22, arm64_const_caps_ready
	ldr	w10, [x22, :lo12:arm64_const_caps_ready]
	add	x1, x8, #2185           // =2185
	cmp	w10, #1                 // =1
	b.lt	.LBB29_1440
// %bb.528:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1441
.LBB29_529:
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
	b	.LBB29_1444
.LBB29_530:
	mov	w22, #-1
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_531:
	mov	w25, #-1
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_532:
	add	x19, x21, #160          // =160
	mov	x0, x19
	bl	down_read
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_534
// %bb.533:
	ldr	x8, [x21, #736]
	tbnz	w8, #31, .LBB29_635
.LBB29_534:
	mov	x0, x19
	bl	up_read
	mov	x0, #-61
	b	.LBB29_1455
.LBB29_535:
	mov	w22, #-1
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_536:
	mov	x0, x19
	mov	x1, x20
	bl	fsverity_ioctl_enable
	b	.LBB29_968
.LBB29_537:
	mov	w21, #-30
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_538:
	mov	w0, #-30
	sxtw	x0, w0
	b	.LBB29_1455
.LBB29_539:
	mov	w27, #-30
	sxtw	x0, w27
	b	.LBB29_1455
.LBB29_540:
	mov	w28, #-30
	sxtw	x0, w28
	b	.LBB29_1455
.LBB29_541:
	mov	w22, #-22
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_542:
	mov	w23, #-22
	b	.LBB29_1454
.LBB29_543:
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_547
// %bb.544:
	ldr	x8, [x21, #736]
	ubfx	x21, x8, #31, #1
	b	.LBB29_548
.LBB29_545:
	mov	w25, #-22
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_546:
	mov	x0, #-12
	b	.LBB29_1455
.LBB29_547:
	mov	w21, wzr
.LBB29_548:
	mov	x0, x19
	bl	up_write
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_550
// %bb.549:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_551
.LBB29_550:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_551:
	//APP
		adds	x10, x10, 1
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_555
// %bb.552:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x11, x20, x9, asr #8
	//APP
		bics	xzr, x11, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w11, [x10, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	b.lt	.LBB29_1023
// %bb.553:
	adrp	x11, cpu_hwcap_keys+16
	ldr	w11, [x11, :lo12:cpu_hwcap_keys+16]
	cmp	w11, #0                 // =0
	b.le	.LBB29_1024
.LBB29_554:
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
	b	.LBB29_1027
.LBB29_555:
	mov	w8, #-14
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_556:
	mov	w21, #-30
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_557:
	mov	x0, #-13
	b	.LBB29_1455
.LBB29_558:
	ldr	x8, [x9]
	ldr	x8, [x8, #248]
	ldr	x8, [x8, #128]
	ldr	x8, [x8, #1264]
	ldr	x9, [x8, #104]
	tbnz	w9, #8, .LBB29_561
// %bb.559:
	cbz	x8, .LBB29_578
// %bb.560:
	ldr	w8, [x8, #2980]
	sub	w8, w8, #1              // =1
	cmp	w8, #1                  // =1
	b.hi	.LBB29_578
.LBB29_561:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldrb	w9, [x23, #62]
	ldr	x8, [x23, #8]
	tbnz	w9, #5, .LBB29_563
// %bb.562:
	ldr	x10, [x23]
	mov	x9, x20
	tbz	w10, #26, .LBB29_564
.LBB29_563:
	lsl	x9, x20, #8
	and	x9, x20, x9, asr #8
.LBB29_564:
	//APP
		adds	x9, x9, 24
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1152
// %bb.565:
	adrp	x25, arm64_const_caps_ready
	ldr	w8, [x25, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1153
// %bb.566:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1154
.LBB29_567:
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
	b	.LBB29_1157
.LBB29_568:
	mov	w20, #-30
	sxtw	x0, w20
	b	.LBB29_1455
.LBB29_569:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w22, w0
	cbnz	w0, .LBB29_571
// %bb.570:
	add	x23, x21, #24           // =24
	mov	x0, x23
	bl	down_write
	ldr	x8, [x21, #16]
	mov	w2, #1024
	mov	w1, wzr
	add	x0, x8, #124            // =124
	bl	memset
	mov	x0, x20
	bl	strlen
	ldr	x8, [x21, #16]
	mov	x1, x0
	mov	w2, #1
	mov	w4, #512
	add	x3, x8, #124            // =124
	mov	x0, x20
                                        // kill: def $w1 killed $w1 killed $x1
	bl	utf8s_to_utf16s
	mov	x0, x21
	mov	w1, wzr
	bl	f2fs_commit_super
	mov	w22, w0
	mov	x0, x23
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
.LBB29_571:
	mov	x0, x20
	bl	kfree
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_572:
	mov	w21, #-13
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_573:
	mov	w25, #-13
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_574:
	mov	w27, #-13
	sxtw	x0, w27
	b	.LBB29_1455
.LBB29_575:
	mov	w28, #-13
	sxtw	x0, w28
	b	.LBB29_1455
.LBB29_576:
	mov	w27, #-95
	sxtw	x0, w27
	b	.LBB29_1455
.LBB29_577:
	mov	w28, #-95
	sxtw	x0, w28
	b	.LBB29_1455
.LBB29_578:
	mov	w21, #-95
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_579:
	mov	w0, #-22
	sxtw	x0, w0
	b	.LBB29_1455
.LBB29_580:
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_1417
// %bb.581:
	ldr	w26, [x20, #1188]
	ldr	w27, [x22, #1292]
	ldr	x8, [x22, #736]
	tbnz	w8, #31, .LBB29_582
	b	.LBB29_1417
.LBB29_582:
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_balance_fs
	ldr	x8, [x19, #32]
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB29_584
// %bb.583:
	ldr	x0, [x8, #40]
	mov	w1, #1
	mov	w2, #1
	bl	__sb_start_write
.LBB29_584:
	add	x21, x22, #160          // =160
	mov	x0, x21
	bl	down_write
	ldr	x8, [x22, #736]
	tbnz	x8, #38, .LBB29_676
// %bb.585:
	mov	x0, x22
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB29_697
// %bb.586:
	ldr	x8, [x22, #736]
	tbnz	x8, #33, .LBB29_780
// %bb.587:
	ldr	x0, [x22, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w23, w0
	cbnz	w0, .LBB29_781
// %bb.588:
	ldr	w8, [x22, #1280]
	cbz	w8, .LBB29_971
// %bb.589:
	ldr	x8, [x22, #80]
	add	x8, x8, #4095           // =4095
	lsr	x28, x8, #12
	cbz	w28, .LBB29_1136
// %bb.590:
	mov	x24, xzr
	b	.LBB29_777
.LBB29_591:
	mov	w21, #-22
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_592:
	mov	w21, #-95
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_593:
	ldr	x8, [x21]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_656
// %bb.594:
	add	x24, x26, #160          // =160
	mov	x0, x24
	bl	down_write
	ldr	w8, [x26, #1280]
	cbnz	w8, .LBB29_669
// %bb.595:
	ldr	x8, [x26, #736]
	tbnz	x8, #35, .LBB29_669
// %bb.596:
	mov	x0, x24
	bl	up_write
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_598
// %bb.597:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_599
.LBB29_598:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_599:
	//APP
		adds	x10, x10, 8
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_941
// %bb.600:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x11, x20, x9, asr #8
	//APP
		bics	xzr, x11, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w11, [x10, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	b.lt	.LBB29_1834
// %bb.601:
	adrp	x11, cpu_hwcap_keys+16
	ldr	w11, [x11, :lo12:cpu_hwcap_keys+16]
	cmp	w11, #0                 // =0
	b.le	.LBB29_1835
.LBB29_602:
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
	b	.LBB29_1838
.LBB29_603:
	ldrsw	x8, [x8, #1280]
	//APP
	mrs x9, sp_el0
	//NO_APP
	ldrb	w11, [x9, #62]
	ldr	x10, [x9, #8]
	tbnz	w11, #5, .LBB29_605
// %bb.604:
	ldr	x12, [x9]
	mov	x11, x20
	tbz	w12, #26, .LBB29_606
.LBB29_605:
	lsl	x11, x20, #8
	and	x11, x20, x11, asr #8
.LBB29_606:
	//APP
		adds	x11, x11, 8
	csel	x10, xzr, x10, hi
	csinv	x11, x11, xzr, cc
	sbcs	xzr, x11, x10
	cset	x11, ls

	//NO_APP
	cbz	x11, .LBB29_610
// %bb.607:
	ldr	x11, [x9, #8]
	lsl	x10, x20, #8
	and	x12, x20, x10, asr #8
	//APP
		bics	xzr, x12, x11
	csel	x10, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x11, arm64_const_caps_ready
	ldr	w12, [x11, :lo12:arm64_const_caps_ready]
	cmp	w12, #1                 // =1
	b.lt	.LBB29_1571
// %bb.608:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_1572
.LBB29_609:
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
	b	.LBB29_1575
.LBB29_610:
	mov	w9, #-14
	sxtw	x0, w9
	b	.LBB29_1455
.LBB29_611:
	ldr	x8, [x21]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_629
// %bb.612:
	mov	x0, x22
	bl	dquot_initialize
	add	w8, w0, #28             // =28
	mov	w25, w0
	cmp	w8, #28                 // =28
	b.hi	.LBB29_614
// %bb.613:
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB29_615
.LBB29_614:
	cmn	w25, #122               // =122
	b.ne	.LBB29_1391
.LBB29_615:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w25, w0
	cbnz	w0, .LBB29_1391
// %bb.616:
	ldr	x8, [x22, #40]
	mov	w1, #1
	ldr	x0, [x8, #1008]
	bl	f2fs_balance_fs
	add	x24, x22, #160          // =160
	mov	x0, x24
	bl	down_write
	ldr	w9, [x22, #392]
	ldr	w8, [x19, #68]
	cmp	w9, #1                  // =1
	and	w8, w8, #0x2
	b.eq	.LBB29_618
// %bb.617:
	cbnz	w8, .LBB29_620
.LBB29_618:
	cbz	w9, .LBB29_795
// %bb.619:
	cbnz	w8, .LBB29_795
.LBB29_620:
	mov	w23, wzr
	mov	w25, #-16
.LBB29_621:
	mov	x0, x24
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
	tbnz	w25, #31, .LBB29_784
// %bb.622:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_624
// %bb.623:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_625
.LBB29_624:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_625:
	//APP
		adds	x10, x10, 8
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_1555
// %bb.626:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x11, x20, x9, asr #8
	//APP
		bics	xzr, x11, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w11, [x10, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	mov	w11, w23
	b.lt	.LBB29_1841
// %bb.627:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_1842
.LBB29_628:
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
	b	.LBB29_1845
.LBB29_629:
	mov	w25, #-30
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_630:
	add	x8, x20, #736           // =736
	mov	w9, #131072
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	x8, [x20, #40]
	mov	w9, #1
	mov	x2, #9223372036854775807
	mov	w4, #1
	ldr	x8, [x8, #1008]
	mov	x0, x19
	mov	x1, xzr
	mov	w3, wzr
	add	x8, x8, #2232           // =2232
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
	bl	f2fs_do_sync_file
	mov	w22, w0
.LBB29_631:
	add	x8, x20, #736           // =736
	mov	w9, #536870912
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	x0, x21
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
	ldr	x8, [x20, #40]
	adrp	x9, jiffies
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	b	.LBB29_668
.LBB29_632:
	ldr	x8, [x21, #736]
	tbnz	w8, #18, .LBB29_657
// %bb.633:
	mov	w2, #1
	mov	x0, x21
	mov	x1, xzr
	bl	truncate_partial_data_page
	b	.LBB29_658
.LBB29_634:
	mov	w22, #-22
	b	.LBB29_663
.LBB29_635:
	ldrb	w8, [x21, #1284]
	mov	x0, x19
	strb	w8, [sp, #144]
	ldrb	w8, [x21, #1285]
	strb	w8, [sp, #145]
	bl	up_read
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_637
// %bb.636:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_638
.LBB29_637:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_638:
	//APP
		adds	x10, x10, 2
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_654
// %bb.639:
	adrp	x19, arm64_const_caps_ready
	ldr	w9, [x19, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1582
// %bb.640:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1583
.LBB29_641:
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
	b	.LBB29_1586
.LBB29_642:
	mov	w23, #-14
	b	.LBB29_1452
.LBB29_643:
	mov	w8, #1
	strb	w8, [sp, #145]
	ldr	x22, [x21, #1304]
	cbz	x22, .LBB29_647
// %bb.644:
	add	x21, x22, #744          // =744
	mov	x0, x21
	bl	down_write
	ldr	w8, [x22, #72]
	cmp	w8, #2                  // =2
	b.lo	.LBB29_646
// %bb.645:
	mov	w8, #1
	strb	w8, [sp, #144]
.LBB29_646:
	ldr	x8, [x22, #64]
	mov	x0, x21
	str	w8, [sp, #148]
	bl	up_write
.LBB29_647:
	mov	x0, x19
	bl	up_write
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_649
// %bb.648:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_650
.LBB29_649:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_650:
	//APP
		adds	x10, x10, 8
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_654
// %bb.651:
	adrp	x19, arm64_const_caps_ready
	ldr	w9, [x19, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1467
// %bb.652:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1468
.LBB29_653:
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
	b	.LBB29_1471
.LBB29_654:
	mov	x0, #-14
	b	.LBB29_1455
.LBB29_655:
	mov	w25, #-30
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_656:
	mov	w24, #-30
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_657:
	mov	w2, #4096
	mov	x0, x21
	mov	x1, xzr
	bl	punch_hole
.LBB29_658:
	mov	w22, w0
.LBB29_659:
	mov	x0, x20
	b	.LBB29_666
.LBB29_660:
	mov	x0, x20
	bl	f2fs_commit_inmem_pages
	mov	w22, w0
	cbnz	w0, .LBB29_663
// %bb.661:
	mov	x2, #9223372036854775807
	mov	w4, #1
	mov	x0, x19
	mov	x1, xzr
	mov	w3, wzr
	bl	f2fs_do_sync_file
	mov	w22, w0
	cbnz	w0, .LBB29_663
// %bb.662:
	mov	x0, x20
	bl	f2fs_drop_inmem_pages
.LBB29_663:
	ldr	x8, [x20, #736]
	tbz	w8, #29, .LBB29_665
// %bb.664:
	add	x8, x20, #736           // =736
	mov	w9, #536870912
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w22, #-22
.LBB29_665:
	mov	x0, x21
.LBB29_666:
	bl	up_write
.LBB29_667:
	mov	x0, x19
	bl	mnt_drop_write_file
.LBB29_668:
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_669:
	ldr	w9, [x26, #392]
	ldr	w8, [x19, #68]
	str	x24, [sp, #64]          // 8-byte Folded Spill
	cmp	w9, #1                  // =1
	and	w8, w8, #0x2
	b.eq	.LBB29_671
// %bb.670:
	cbnz	w8, .LBB29_673
.LBB29_671:
	cbz	w9, .LBB29_704
// %bb.672:
	cbnz	w8, .LBB29_704
.LBB29_673:
	mov	x19, xzr
	mov	w24, #-16
.LBB29_674:
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_write
	ldr	x8, [x26, #80]
	add	x8, x8, #4095           // =4095
	lsr	x3, x8, #12
	subs	x8, x3, x19
	b.hs	.LBB29_677
// %bb.675:
	adrp	x1, .L.str.47
	adrp	x2, .L__func__.f2fs_ioc_get_saved_blocks
	add	x1, x1, :lo12:.L.str.47
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_get_saved_blocks
	mov	x0, x21
	mov	x4, x19
	bl	f2fs_printk
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_676:
	mov	w23, #-13
	b	.LBB29_781
.LBB29_677:
	//APP
	mrs x9, sp_el0
	//NO_APP
	ldrb	w11, [x9, #62]
	ldr	x10, [x9, #8]
	tbnz	w11, #5, .LBB29_679
// %bb.678:
	ldr	x12, [x9]
	mov	x11, x20
	tbz	w12, #26, .LBB29_680
.LBB29_679:
	lsl	x11, x20, #8
	and	x11, x20, x11, asr #8
.LBB29_680:
	//APP
		adds	x11, x11, 8
	csel	x10, xzr, x10, hi
	csinv	x11, x11, xzr, cc
	sbcs	xzr, x11, x10
	cset	x11, ls

	//NO_APP
	cbz	x11, .LBB29_941
// %bb.681:
	ldr	x11, [x9, #8]
	lsl	x10, x20, #8
	and	x12, x20, x10, asr #8
	//APP
		bics	xzr, x12, x11
	csel	x10, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x11, arm64_const_caps_ready
	ldr	w12, [x11, :lo12:arm64_const_caps_ready]
	cmp	w12, #1                 // =1
	b.lt	.LBB29_1745
// %bb.682:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_1746
.LBB29_683:
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
	b	.LBB29_1749
.LBB29_684:
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_687
// %bb.685:
	ldr	x8, [x21, #736]
	tbz	w8, #31, .LBB29_687
.LBB29_686:
	mov	w1, #11
	mov	x0, x19
	mov	w2, wzr
	bl	f2fs_decompress_file_internal
	mov	x23, x0
	cbnz	w23, .LBB29_703
.LBB29_687:
	ldr	x8, [x21, #736]
	tbnz	w8, #15, .LBB29_702
// %bb.688:
	mov	x0, x21
	bl	f2fs_convert_inline_inode
	mov	w23, w0
	cbnz	w0, .LBB29_703
// %bb.689:
	add	x22, x21, #1040         // =1040
	mov	x0, x22
	bl	down_write
	ldr	w8, [x21, #792]
	cbz	w8, .LBB29_691
// %bb.690:
	ldr	x8, [x21, #40]
	ldr	x2, [x21, #64]
	adrp	x1, .L.str.38
	add	x1, x1, :lo12:.L.str.38
	ldr	x0, [x8, #1008]
	ldr	w3, [x21, #792]
	bl	f2fs_printk
.LBB29_691:
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	cbz	w0, .LBB29_828
// %bb.692:
	mov	w23, w0
	mov	x0, x22
	bl	up_write
	b	.LBB29_703
.LBB29_693:
	mov	w22, wzr
	b	.LBB29_665
.LBB29_694:
	ldr	x2, [x22, #64]
	ldr	x3, [x22, #144]
	adrp	x1, .L.str.48
	add	x1, x1, :lo12:.L.str.48
	mov	x0, x21
	bl	f2fs_printk
	b	.LBB29_248
.LBB29_695:
	ldr	x2, [x21, #64]
	ldr	x3, [x21, #144]
	adrp	x1, .L.str.48
	add	x1, x1, :lo12:.L.str.48
	mov	x0, x20
	bl	f2fs_printk
.LBB29_696:
	mov	w23, #-95
	sxtw	x0, w23
	b	.LBB29_1455
.LBB29_697:
	mov	w23, #-95
	b	.LBB29_781
.LBB29_698:                             //   in Loop: Header=BB29_699 Depth=1
	subs	w28, w28, w25
	add	x24, x24, w25, sxtw
	b.eq	.LBB29_976
.LBB29_699:                             // =>This Inner Loop Header: Depth=1
	cmp	w27, w28
	csel	w25, w27, w28, lt
	mov	x0, x22
	mov	x1, x24
	mov	w2, w25
	bl	redirty_blocks
	mov	w23, w0
	tbnz	w0, #31, .LBB29_977
// %bb.700:                             //   in Loop: Header=BB29_699 Depth=1
	ldr	w8, [x22, #792]
	cmp	w8, w26
	b.lo	.LBB29_698
// %bb.701:                             //   in Loop: Header=BB29_699 Depth=1
	ldr	x0, [x22, #48]
	bl	filemap_fdatawrite
	b	.LBB29_698
.LBB29_702:
	ldr	x8, [x21, #736]
	tst	x8, #0x20000000
	mov	w8, #-22
	csel	w23, wzr, w8, eq
.LBB29_703:
	mov	x0, x20
	b	.LBB29_1453
.LBB29_704:
	ldr	x8, [x26, #80]
	add	x8, x8, #4095           // =4095
	lsr	x23, x8, #12
	cbz	x23, .LBB29_794
// %bb.705:
	mov	x28, #2097152
	mov	x22, #-549755813888
	mov	x27, #-549755813888
	mov	x19, xzr
	mov	x25, xzr
	movk	x28, #1, lsl #32
	movk	x22, #4072
	movk	x27, #4076
	str	x23, [sp, #48]          // 8-byte Folded Spill
.LBB29_706:                             // =>This Loop Header: Depth=1
                                        //     Child Loop BB29_718 Depth 2
                                        //       Child Loop BB29_731 Depth 3
                                        //     Child Loop BB29_740 Depth 2
                                        //       Child Loop BB29_744 Depth 3
                                        //         Child Loop BB29_756 Depth 4
	add	x0, sp, #144            // =144
	mov	w2, #48
	mov	w1, wzr
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	bl	memset
	add	x0, sp, #144            // =144
	mov	w2, #1
	mov	x1, x25
	stp	x26, xzr, [sp, #144]
	str	xzr, [sp, #160]
	str	wzr, [sp, #168]
	str	x25, [sp, #72]          // 8-byte Folded Spill
	bl	f2fs_get_dnode_of_data
	cbz	w0, .LBB29_709
// %bb.707:                             //   in Loop: Header=BB29_706 Depth=1
	mov	w24, w0
	cmn	w0, #2                  // =2
	b.ne	.LBB29_674
// %bb.708:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	x1, [sp, #72]           // 8-byte Folded Reload
	add	x0, sp, #144            // =144
	bl	f2fs_get_next_page_offset
	mov	x25, x0
	mov	w24, wzr
	cmp	x23, x25
	b.hi	.LBB29_706
	b	.LBB29_674
.LBB29_709:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	x0, [sp, #160]
	mov	x14, #-549755813888
	add	x8, x0, x28
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	ldr	w9, [x8, x22]
	ldr	w8, [x8, x27]
	cmp	w9, w8
	b.ne	.LBB29_713
// %bb.710:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	w8, [x26, #1184]
	ldr	w9, [x26, #1192]
	ldrh	w10, [x26]
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	mov	w9, #923
	sub	w9, w9, w8
	b.ne	.LBB29_716
// %bb.711:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	x8, [x26, #736]
	tbz	w8, #31, .LBB29_716
// %bb.712:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	w8, [x26, #1292]
	neg	w8, w8
	and	w9, w9, w8
	b	.LBB29_716
.LBB29_713:                             //   in Loop: Header=BB29_706 Depth=1
	ldrh	w8, [x26]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_715
// %bb.714:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	x8, [x26, #736]
	tbnz	w8, #31, .LBB29_774
.LBB29_715:                             //   in Loop: Header=BB29_706 Depth=1
	mov	w9, #1018
.LBB29_716:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	w8, [sp, #172]
	ldr	x10, [sp, #72]          // 8-byte Folded Reload
	ldr	w11, [x26, #1292]
	mov	w9, w9
	sub	x9, x9, x8
	sub	x10, x23, x10
	cmp	x9, x10
	csel	x9, x9, x10, lo
	sub	w10, w11, #1            // =1
	add	x9, x9, x10
	udiv	x9, x9, x11
	mul	x23, x9, x11
	cbz	x23, .LBB29_764
// %bb.717:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	x9, [sp, #144]
	str	x19, [sp, #56]          // 8-byte Folded Spill
	mov	w24, wzr
	ldr	x10, [x9, #40]
	ldrsw	x19, [x9, #1292]
	ldr	x25, [x10, #1008]
.LBB29_718:                             //   Parent Loop BB29_706 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB29_731 Depth 3
	add	x10, x0, x28
	lsl	x10, x10, #6
	and	x12, x10, #0xfffffffffffff000
	ldr	w10, [x12, x22]
	ldr	w11, [x12, x27]
	add	w8, w24, w8
	add	x12, x12, x14
	cmp	w10, w11
	b.ne	.LBB29_723
// %bb.719:                             //   in Loop: Header=BB29_718 Depth=2
	cbz	x9, .LBB29_722
// %bb.720:                             //   in Loop: Header=BB29_718 Depth=2
	ldr	x13, [x9, #736]
	tbz	w13, #26, .LBB29_723
// %bb.721:                             //   in Loop: Header=BB29_718 Depth=2
	ldr	w9, [x9, #1184]
	asr	w9, w9, #2
	b	.LBB29_724
.LBB29_722:                             //   in Loop: Header=BB29_718 Depth=2
	ldrb	w9, [x12, #3]
	tbnz	w9, #5, .LBB29_736
.LBB29_723:                             //   in Loop: Header=BB29_718 Depth=2
	mov	w9, wzr
.LBB29_724:                             //   in Loop: Header=BB29_718 Depth=2
	add	x13, x12, #360          // =360
	cmp	w10, w11
	csel	x10, x13, x12, eq
	add	w8, w8, w9
	ldr	w1, [x10, w8, uxtw #2]
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB29_727
.LBB29_725:                             //   in Loop: Header=BB29_718 Depth=2
	add	w24, w24, #1            // =1
	cmp	x23, w24, sxtw
	b.ls	.LBB29_737
// %bb.726:                             //   in Loop: Header=BB29_718 Depth=2
	ldr	x9, [sp, #144]
	ldr	x0, [sp, #160]
	ldr	w8, [sp, #172]
	b	.LBB29_718
.LBB29_727:                             //   in Loop: Header=BB29_718 Depth=2
	ldr	w9, [x25, #3128]
	cbz	w9, .LBB29_735
// %bb.728:                             //   in Loop: Header=BB29_718 Depth=2
	ldr	x8, [x25]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB29_734
// %bb.729:                             //   in Loop: Header=BB29_718 Depth=2
	ldr	x10, [x25, #3136]
	add	x10, x10, #80           // =80
	b	.LBB29_731
.LBB29_730:                             //   in Loop: Header=BB29_731 Depth=3
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB29_734
.LBB29_731:                             //   Parent Loop BB29_706 Depth=1
                                        //     Parent Loop BB29_718 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB29_730
// %bb.732:                             //   in Loop: Header=BB29_731 Depth=3
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB29_730
// %bb.733:                             //   in Loop: Header=BB29_718 Depth=2
	ldur	x8, [x10, #-80]
.LBB29_734:                             //   in Loop: Header=BB29_718 Depth=2
	cbz	x8, .LBB29_725
.LBB29_735:                             //   in Loop: Header=BB29_718 Depth=2
	mov	w2, #6
	mov	x0, x25
	bl	f2fs_is_valid_blkaddr
	mov	x14, #-549755813888
	tbnz	w0, #0, .LBB29_725
	b	.LBB29_775
.LBB29_736:                             //   in Loop: Header=BB29_718 Depth=2
	ldrh	w9, [x12, #360]
	lsr	w9, w9, #2
	b	.LBB29_724
.LBB29_737:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	x0, [sp, #160]
	ldr	x8, [sp, #144]
	mov	x11, #-549755813888
	movk	x11, #360
	add	x9, x0, x28
	lsl	x9, x9, #6
	and	x10, x9, #0xfffffffffffff000
	mov	w24, wzr
	add	x9, x10, x14
	add	x10, x10, x11
	mov	x11, x23
	b	.LBB29_740
.LBB29_738:                             //   in Loop: Header=BB29_740 Depth=2
	mov	w12, wzr
.LBB29_739:                             //   in Loop: Header=BB29_740 Depth=2
	add	w24, w12, w24
	subs	x11, x11, x19
	b.eq	.LBB29_763
.LBB29_740:                             //   Parent Loop BB29_706 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB29_744 Depth 3
                                        //         Child Loop BB29_756 Depth 4
	cmp	w19, #1                 // =1
	b.lt	.LBB29_738
// %bb.741:                             //   in Loop: Header=BB29_740 Depth=2
	ldr	w15, [x9, #4072]
	ldr	w16, [x9, #4076]
	ldr	w14, [sp, #172]
	mov	w12, wzr
	mov	w13, wzr
	cmp	w15, w16
	csel	x17, x10, x9, eq
	b	.LBB29_744
.LBB29_742:                             //   in Loop: Header=BB29_744 Depth=3
	cmn	w1, #2                  // =2
	b.ne	.LBB29_762
.LBB29_743:                             //   in Loop: Header=BB29_744 Depth=3
	add	w13, w13, #1            // =1
	add	w14, w14, #1            // =1
	cmp	w13, w19
	str	w14, [sp, #172]
	b.eq	.LBB29_739
.LBB29_744:                             //   Parent Loop BB29_706 Depth=1
                                        //     Parent Loop BB29_740 Depth=2
                                        // =>    This Loop Header: Depth=3
                                        //         Child Loop BB29_756 Depth 4
	cmp	w15, w16
	b.ne	.LBB29_749
// %bb.745:                             //   in Loop: Header=BB29_744 Depth=3
	cbz	x8, .LBB29_748
// %bb.746:                             //   in Loop: Header=BB29_744 Depth=3
	ldr	x1, [x8, #736]
	tbz	w1, #26, .LBB29_749
// %bb.747:                             //   in Loop: Header=BB29_744 Depth=3
	ldr	w1, [x8, #1184]
	asr	w1, w1, #2
	b	.LBB29_750
.LBB29_748:                             //   in Loop: Header=BB29_744 Depth=3
	ldrb	w1, [x9, #3]
	tbnz	w1, #5, .LBB29_761
.LBB29_749:                             //   in Loop: Header=BB29_744 Depth=3
	mov	w1, wzr
.LBB29_750:                             //   in Loop: Header=BB29_744 Depth=3
	add	w1, w1, w14
	ldr	w1, [x17, w1, uxtw #2]
	cbz	w13, .LBB29_742
// %bb.751:                             //   in Loop: Header=BB29_744 Depth=3
	add	w2, w1, #3              // =3
	cmp	w2, #4                  // =4
	b.lo	.LBB29_743
// %bb.752:                             //   in Loop: Header=BB29_744 Depth=3
	ldr	w3, [x25, #3128]
	cbz	w3, .LBB29_760
// %bb.753:                             //   in Loop: Header=BB29_744 Depth=3
	ldr	x2, [x25]
	cmp	w3, #1                  // =1
	ldr	x2, [x2, #248]
	b.lt	.LBB29_759
// %bb.754:                             //   in Loop: Header=BB29_744 Depth=3
	ldr	x4, [x25, #3136]
	add	x4, x4, #80             // =80
	b	.LBB29_756
.LBB29_755:                             //   in Loop: Header=BB29_756 Depth=4
	subs	x3, x3, #1              // =1
	add	x4, x4, #88             // =88
	b.eq	.LBB29_759
.LBB29_756:                             //   Parent Loop BB29_706 Depth=1
                                        //     Parent Loop BB29_740 Depth=2
                                        //       Parent Loop BB29_744 Depth=3
                                        // =>      This Inner Loop Header: Depth=4
	ldur	w5, [x4, #-4]
	cmp	w5, w1
	b.hi	.LBB29_755
// %bb.757:                             //   in Loop: Header=BB29_756 Depth=4
	ldr	w5, [x4]
	cmp	w5, w1
	b.lo	.LBB29_755
// %bb.758:                             //   in Loop: Header=BB29_744 Depth=3
	ldur	x2, [x4, #-80]
.LBB29_759:                             //   in Loop: Header=BB29_744 Depth=3
	cbz	x2, .LBB29_743
.LBB29_760:                             //   in Loop: Header=BB29_744 Depth=3
	add	w12, w12, #1            // =1
	b	.LBB29_743
.LBB29_761:                             //   in Loop: Header=BB29_744 Depth=3
	ldrh	w1, [x9, #360]
	lsr	w1, w1, #2
	b	.LBB29_750
.LBB29_762:                             //   in Loop: Header=BB29_740 Depth=2
	add	w12, w14, w19
	str	w12, [sp, #172]
	subs	x11, x11, x19
	b.ne	.LBB29_740
.LBB29_763:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	x19, [sp, #56]          // 8-byte Folded Reload
	b	.LBB29_765
.LBB29_764:                             //   in Loop: Header=BB29_706 Depth=1
	mov	w24, wzr
.LBB29_765:                             //   in Loop: Header=BB29_706 Depth=1
	cbz	x0, .LBB29_767
// %bb.766:                             //   in Loop: Header=BB29_706 Depth=1
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #160]
	b	.LBB29_768
.LBB29_767:                             //   in Loop: Header=BB29_706 Depth=1
	mov	x9, xzr
.LBB29_768:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	x8, [sp, #152]
	cbz	x8, .LBB29_772
// %bb.769:                             //   in Loop: Header=BB29_706 Depth=1
	cmp	x9, x8
	b.eq	.LBB29_772
// %bb.770:                             //   in Loop: Header=BB29_706 Depth=1
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
	cbnz	w9, .LBB29_772
// %bb.771:                             //   in Loop: Header=BB29_706 Depth=1
	bl	__put_page
.LBB29_772:                             //   in Loop: Header=BB29_706 Depth=1
	stp	xzr, xzr, [sp, #152]
	tbnz	w24, #31, .LBB29_674
// %bb.773:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	x25, [sp, #72]          // 8-byte Folded Reload
	add	x19, x19, w24, uxtw
	add	x25, x23, x25
	ldr	x23, [sp, #48]          // 8-byte Folded Reload
	cmp	x23, x25
	b.hi	.LBB29_706
	b	.LBB29_674
.LBB29_774:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	w8, [x26, #1292]
	mov	w9, #1018
	neg	w8, w8
	and	w9, w8, w9
	b	.LBB29_716
.LBB29_775:                             //   in Loop: Header=BB29_706 Depth=1
	ldr	x0, [sp, #160]
	mov	w24, #-117
	b	.LBB29_763
.LBB29_776:                             //   in Loop: Header=BB29_777 Depth=1
	subs	w28, w28, w25
	add	x24, x24, w25, sxtw
	b.eq	.LBB29_1135
.LBB29_777:                             // =>This Inner Loop Header: Depth=1
	cmp	w27, w28
	csel	w25, w27, w28, lt
	mov	x0, x22
	mov	x1, x24
	mov	w2, w25
	bl	redirty_blocks
	mov	w23, w0
	tbnz	w0, #31, .LBB29_1137
// %bb.778:                             //   in Loop: Header=BB29_777 Depth=1
	ldr	w8, [x22, #792]
	cmp	w8, w26
	b.lo	.LBB29_776
// %bb.779:                             //   in Loop: Header=BB29_777 Depth=1
	ldr	x0, [x22, #48]
	bl	filemap_fdatawrite
	b	.LBB29_776
.LBB29_780:
	mov	w23, #-16
.LBB29_781:
	mov	x0, x21
	b	.LBB29_1426
.LBB29_782:
	ldr	x8, [x22, #40]
	mov	w1, #1
	ldr	x0, [x8, #1008]
	bl	f2fs_balance_fs
	add	x24, x22, #160          // =160
	mov	x0, x24
	bl	down_write
	ldr	x8, [x22, #736]
	tbnz	x8, #35, .LBB29_798
// %bb.783:
	mov	w23, wzr
	mov	w25, #-22
	b	.LBB29_841
.LBB29_784:
	cbz	w23, .LBB29_1391
// %bb.785:
	ldr	w8, [x22, #1280]
	cbz	w8, .LBB29_1391
// %bb.786:
	cbz	x21, .LBB29_791
// %bb.787:
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
	cbnz	w8, .LBB29_791
// %bb.788:
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB29_791
// %bb.789:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB29_791
// %bb.790:
	add	x2, x21, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB29_791:
	ldr	x3, [x22, #64]
	ldr	x4, [x22, #144]
	ldr	w6, [x22, #1280]
	adrp	x1, .L.str.45
	adrp	x2, .L__func__.f2fs_release_compress_blocks
	add	x1, x1, :lo12:.L.str.45
	add	x2, x2, :lo12:.L__func__.f2fs_release_compress_blocks
	b	.LBB29_857
.LBB29_792:
	mov	w27, wzr
	mov	w24, wzr
	mov	w25, wzr
	b	.LBB29_1114
.LBB29_793:
	mov	w23, #-13
	b	.LBB29_860
.LBB29_794:
	mov	w24, wzr
	mov	x19, xzr
	b	.LBB29_674
.LBB29_795:
	ldr	x8, [x22, #736]
	tbnz	x8, #35, .LBB29_826
// %bb.796:
	ldr	x0, [x22, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	cbz	w0, .LBB29_862
// %bb.797:
	mov	w25, w0
	mov	w23, wzr
	b	.LBB29_621
.LBB29_798:
	add	x0, x22, #1040          // =1040
	str	x0, [sp, #72]           // 8-byte Folded Spill
	bl	down_write
	add	x0, x22, #1088          // =1088
	str	x0, [sp, #64]           // 8-byte Folded Spill
	bl	down_write
	ldr	x8, [x22, #80]
	add	x8, x8, #4095           // =4095
	lsr	x27, x8, #12
	cbz	x27, .LBB29_838
// %bb.799:
	mov	x28, xzr
	mov	w23, wzr
	b	.LBB29_802
.LBB29_800:                             //   in Loop: Header=BB29_802 Depth=1
	stp	xzr, xzr, [sp, #152]
	tbnz	w25, #31, .LBB29_833
// %bb.801:                             //   in Loop: Header=BB29_802 Depth=1
	add	x28, x26, x28
	add	w23, w25, w23
	cmp	x27, x28
	b.ls	.LBB29_839
.LBB29_802:                             // =>This Inner Loop Header: Depth=1
	add	x0, sp, #144            // =144
	mov	w2, #48
	mov	w1, wzr
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	bl	memset
	add	x0, sp, #144            // =144
	mov	w2, #1
	mov	x1, x28
	stp	x22, xzr, [sp, #144]
	str	xzr, [sp, #160]
	str	wzr, [sp, #168]
	bl	f2fs_get_dnode_of_data
	cbz	w0, .LBB29_805
// %bb.803:                             //   in Loop: Header=BB29_802 Depth=1
	mov	w25, w0
	cmn	w0, #2                  // =2
	b.ne	.LBB29_833
// %bb.804:                             //   in Loop: Header=BB29_802 Depth=1
	add	x0, sp, #144            // =144
	mov	x1, x28
	bl	f2fs_get_next_page_offset
	mov	x28, x0
	mov	w25, wzr
	cmp	x27, x28
	b.hi	.LBB29_802
	b	.LBB29_839
.LBB29_805:                             //   in Loop: Header=BB29_802 Depth=1
	ldr	x8, [sp, #160]
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	mov	x10, #-549755813888
	add	x8, x8, x9
	lsl	x8, x8, #6
	mov	x9, #-549755813888
	and	x8, x8, #0xfffffffffffff000
	movk	x9, #4072
	movk	x10, #4076
	ldr	w9, [x8, x9]
	ldr	w8, [x8, x10]
	cmp	w9, w8
	b.ne	.LBB29_809
// %bb.806:                             //   in Loop: Header=BB29_802 Depth=1
	ldr	w8, [x22, #1184]
	ldr	w9, [x22, #1192]
	ldrh	w10, [x22]
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	mov	w9, #923
	sub	w8, w9, w8
	b.ne	.LBB29_814
// %bb.807:                             //   in Loop: Header=BB29_802 Depth=1
	ldr	x9, [x22, #736]
	tbz	w9, #31, .LBB29_814
// %bb.808:                             //   in Loop: Header=BB29_802 Depth=1
	ldr	w9, [x22, #1292]
	neg	w9, w9
	b	.LBB29_813
.LBB29_809:                             //   in Loop: Header=BB29_802 Depth=1
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_811
// %bb.810:                             //   in Loop: Header=BB29_802 Depth=1
	ldr	x8, [x22, #736]
	tbnz	w8, #31, .LBB29_812
.LBB29_811:                             //   in Loop: Header=BB29_802 Depth=1
	mov	w8, #1018
	b	.LBB29_814
.LBB29_812:                             //   in Loop: Header=BB29_802 Depth=1
	ldr	w8, [x22, #1292]
	mov	w9, #1018
	neg	w8, w8
.LBB29_813:                             //   in Loop: Header=BB29_802 Depth=1
	and	w8, w8, w9
.LBB29_814:                             //   in Loop: Header=BB29_802 Depth=1
	ldr	w9, [sp, #172]
	mov	w8, w8
	ldr	w11, [x22, #1292]
	sub	x10, x27, x28
	sub	x8, x8, x9
	cmp	x8, x10
	csel	x8, x8, x10, lo
	sub	x8, x8, #1              // =1
	sub	w9, w11, #1             // =1
	orr	x8, x8, x9
	add	x26, x8, #1             // =1
	add	x0, sp, #144            // =144
	mov	x1, x26
	bl	reserve_compress_blocks
	ldr	x8, [sp, #160]
	mov	w25, w0
	cbz	x8, .LBB29_816
// %bb.815:                             //   in Loop: Header=BB29_802 Depth=1
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_put_page
	ldr	x9, [sp, #160]
	b	.LBB29_817
.LBB29_816:                             //   in Loop: Header=BB29_802 Depth=1
	mov	x9, xzr
.LBB29_817:                             //   in Loop: Header=BB29_802 Depth=1
	ldr	x8, [sp, #152]
	cbz	x8, .LBB29_800
// %bb.818:                             //   in Loop: Header=BB29_802 Depth=1
	cmp	x9, x8
	b.eq	.LBB29_800
// %bb.819:                             //   in Loop: Header=BB29_802 Depth=1
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
	cbnz	w9, .LBB29_800
// %bb.820:                             //   in Loop: Header=BB29_802 Depth=1
	bl	__put_page
	b	.LBB29_800
.LBB29_821:
	mov	w27, wzr
	mov	w24, wzr
	mov	w25, #-13
	b	.LBB29_1113
.LBB29_822:                             //   in Loop: Header=BB29_823 Depth=1
	subs	w28, w28, w25
	add	x24, x24, w25, sxtw
	b.eq	.LBB29_1458
.LBB29_823:                             // =>This Inner Loop Header: Depth=1
	cmp	w27, w28
	csel	w25, w27, w28, lt
	mov	x0, x21
	mov	x1, x24
	mov	w2, w25
	bl	redirty_blocks
	mov	w23, w0
	tbnz	w0, #31, .LBB29_1460
// %bb.824:                             //   in Loop: Header=BB29_823 Depth=1
	ldr	w8, [x21, #792]
	cmp	w8, w26
	b.lo	.LBB29_822
// %bb.825:                             //   in Loop: Header=BB29_823 Depth=1
	ldr	x0, [x21, #48]
	bl	filemap_fdatawrite
	b	.LBB29_822
.LBB29_826:
	mov	w23, wzr
	mov	w25, #-22
	b	.LBB29_621
.LBB29_827:
	mov	w23, #-95
	b	.LBB29_860
.LBB29_828:
	add	x23, x27, #1028         // =1028
	mov	x0, x23
	bl	_raw_spin_lock
	ldr	x8, [x21, #912]
	add	x24, x21, #912          // =912
	cmp	x8, x24
	b.ne	.LBB29_831
// %bb.829:
	ldr	x25, [x27, #1008]
	add	x26, x27, #1000         // =1000
	mov	x0, x24
	mov	x2, x26
	mov	x1, x25
	bl	__list_add_valid
	tbz	w0, #0, .LBB29_831
// %bb.830:
	str	x24, [x27, #1008]
	str	x26, [x21, #912]
	str	x25, [x21, #920]
	str	x24, [x25]
.LBB29_831:
	ldr	w8, [x27, #2000]
	mov	x0, x23
	add	w8, w8, #1              // =1
	str	w8, [x27, #2000]
	bl	_raw_spin_unlock
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 32768
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w9, #536870912
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	x0, x22
	bl	up_write
	ldr	x8, [x21, #40]
	adrp	x9, jiffies
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	ldr	x8, [x21, #40]
	//APP
	mrs x9, sp_el0
	//NO_APP
	str	x9, [x21, #944]
	ldr	x8, [x8, #1008]
	ldr	w9, [x8, #2000]
	ldr	w10, [x8, #2236]
	cmp	w9, w10
	b.le	.LBB29_861
// %bb.832:
	mov	w23, wzr
	str	w9, [x8, #2236]
	b	.LBB29_703
.LBB29_833:
	cbz	w23, .LBB29_839
// %bb.834:
	tbz	w25, #31, .LBB29_839
// %bb.835:
	ldr	w8, [x22, #1280]
	cbz	w8, .LBB29_839
// %bb.836:
	mov	x0, x22
	mov	x1, x28
	bl	f2fs_release_reserved_blocks
	sub	w23, w23, w0
	b	.LBB29_839
.LBB29_837:
	mov	x0, #-5
	b	.LBB29_1455
.LBB29_838:
	mov	w25, wzr
	mov	w23, wzr
.LBB29_839:
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	up_write
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_write
	tbnz	w25, #31, .LBB29_841
// %bb.840:
	add	x8, x22, #736           // =736
	mov	x9, #34359738368
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x22
	bl	current_time
	stp	x0, x1, [x22, #120]
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_mark_inode_dirty_sync
.LBB29_841:
	mov	x0, x24
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
	tbnz	w25, #31, .LBB29_849
.LBB29_842:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_844
// %bb.843:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_845
.LBB29_844:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_845:
	//APP
		adds	x10, x10, 8
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_1555
// %bb.846:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x11, x20, x9, asr #8
	//APP
		bics	xzr, x11, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w11, [x10, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	mov	w11, w23
	b.lt	.LBB29_1822
// %bb.847:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_1823
.LBB29_848:
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
	b	.LBB29_1826
.LBB29_849:
	cbz	w23, .LBB29_1391
// %bb.850:
	ldr	w8, [x22, #1280]
	cbz	w8, .LBB29_1391
// %bb.851:
	cbz	x21, .LBB29_856
// %bb.852:
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
	cbnz	w8, .LBB29_856
// %bb.853:
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB29_856
// %bb.854:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB29_856
// %bb.855:
	add	x2, x21, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB29_856:
	ldr	x3, [x22, #64]
	ldr	x4, [x22, #144]
	ldr	w6, [x22, #1280]
	adrp	x1, .L.str.46
	adrp	x2, .L__func__.f2fs_reserve_compress_blocks
	add	x1, x1, :lo12:.L.str.46
	add	x2, x2, :lo12:.L__func__.f2fs_reserve_compress_blocks
.LBB29_857:
	mov	x0, x21
	mov	w5, w23
	bl	f2fs_printk
	b	.LBB29_1391
.LBB29_858:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x20, #360]
	ldr	x2, [x24, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x20, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB29_317
.LBB29_859:
	mov	w23, #-16
.LBB29_860:
	mov	x0, x22
	b	.LBB29_1426
.LBB29_861:
	mov	w23, wzr
	b	.LBB29_703
.LBB29_862:
	add	x8, x22, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 34359738368
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x22
	bl	current_time
	stp	x0, x1, [x22, #120]
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_mark_inode_dirty_sync
	ldr	w8, [x22, #1280]
	cbz	w8, .LBB29_1134
// %bb.863:
	add	x0, x22, #1040          // =1040
	str	x0, [sp, #72]           // 8-byte Folded Spill
	bl	down_write
	add	x0, x22, #1088          // =1088
	str	x0, [sp, #64]           // 8-byte Folded Spill
	bl	down_write
	ldr	x8, [x22, #80]
	add	x8, x8, #4095           // =4095
	lsr	x27, x8, #12
	cbz	x27, .LBB29_1285
// %bb.864:
	mov	x28, xzr
	mov	w23, wzr
.LBB29_865:                             // =>This Inner Loop Header: Depth=1
	add	x0, sp, #144            // =144
	mov	w2, #48
	mov	w1, wzr
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	bl	memset
	add	x0, sp, #144            // =144
	mov	w2, #1
	mov	x1, x28
	stp	x22, xzr, [sp, #144]
	str	xzr, [sp, #160]
	str	wzr, [sp, #168]
	bl	f2fs_get_dnode_of_data
	cbz	w0, .LBB29_868
// %bb.866:                             //   in Loop: Header=BB29_865 Depth=1
	mov	w25, w0
	cmn	w0, #2                  // =2
	b.ne	.LBB29_1286
// %bb.867:                             //   in Loop: Header=BB29_865 Depth=1
	add	x0, sp, #144            // =144
	mov	x1, x28
	bl	f2fs_get_next_page_offset
	mov	x28, x0
	mov	w25, wzr
	cmp	x27, x28
	b.hi	.LBB29_865
	b	.LBB29_1286
.LBB29_868:                             //   in Loop: Header=BB29_865 Depth=1
	ldr	x8, [sp, #160]
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	mov	x10, #-549755813888
	add	x8, x8, x9
	lsl	x8, x8, #6
	mov	x9, #-549755813888
	and	x8, x8, #0xfffffffffffff000
	movk	x9, #4072
	movk	x10, #4076
	ldr	w9, [x8, x9]
	ldr	w8, [x8, x10]
	cmp	w9, w8
	b.ne	.LBB29_872
// %bb.869:                             //   in Loop: Header=BB29_865 Depth=1
	ldr	w8, [x22, #1184]
	ldr	w9, [x22, #1192]
	ldrh	w10, [x22]
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	mov	w9, #923
	sub	w8, w9, w8
	b.ne	.LBB29_877
// %bb.870:                             //   in Loop: Header=BB29_865 Depth=1
	ldr	x9, [x22, #736]
	tbz	w9, #31, .LBB29_877
// %bb.871:                             //   in Loop: Header=BB29_865 Depth=1
	ldr	w9, [x22, #1292]
	neg	w9, w9
	b	.LBB29_876
.LBB29_872:                             //   in Loop: Header=BB29_865 Depth=1
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_874
// %bb.873:                             //   in Loop: Header=BB29_865 Depth=1
	ldr	x8, [x22, #736]
	tbnz	w8, #31, .LBB29_875
.LBB29_874:                             //   in Loop: Header=BB29_865 Depth=1
	mov	w8, #1018
	b	.LBB29_877
.LBB29_875:                             //   in Loop: Header=BB29_865 Depth=1
	ldr	w8, [x22, #1292]
	mov	w9, #1018
	neg	w8, w8
.LBB29_876:                             //   in Loop: Header=BB29_865 Depth=1
	and	w8, w8, w9
.LBB29_877:                             //   in Loop: Header=BB29_865 Depth=1
	ldr	w9, [sp, #172]
	mov	w8, w8
	ldr	w11, [x22, #1292]
	sub	x10, x27, x28
	sub	x8, x8, x9
	cmp	x8, x10
	csel	x8, x8, x10, lo
	sub	x8, x8, #1              // =1
	sub	w9, w11, #1             // =1
	orr	x8, x8, x9
	add	x26, x8, #1             // =1
	add	x0, sp, #144            // =144
	mov	x1, x26
	bl	release_compress_blocks
	ldr	x8, [sp, #160]
	mov	w25, w0
	cbz	x8, .LBB29_879
// %bb.878:                             //   in Loop: Header=BB29_865 Depth=1
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_put_page
	ldr	x9, [sp, #160]
	b	.LBB29_880
.LBB29_879:                             //   in Loop: Header=BB29_865 Depth=1
	mov	x9, xzr
.LBB29_880:                             //   in Loop: Header=BB29_865 Depth=1
	ldr	x8, [sp, #152]
	cbz	x8, .LBB29_884
// %bb.881:                             //   in Loop: Header=BB29_865 Depth=1
	cmp	x9, x8
	b.eq	.LBB29_884
// %bb.882:                             //   in Loop: Header=BB29_865 Depth=1
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
	cbnz	w9, .LBB29_884
// %bb.883:                             //   in Loop: Header=BB29_865 Depth=1
	bl	__put_page
.LBB29_884:                             //   in Loop: Header=BB29_865 Depth=1
	stp	xzr, xzr, [sp, #152]
	tbnz	w25, #31, .LBB29_1286
// %bb.885:                             //   in Loop: Header=BB29_865 Depth=1
	add	x28, x26, x28
	add	w23, w25, w23
	cmp	x27, x28
	b.hi	.LBB29_865
	b	.LBB29_1286
.LBB29_886:
	ldr	x2, [x22, #64]
	adrp	x1, .L.str.49
	add	x1, x1, :lo12:.L.str.49
	b	.LBB29_1111
.LBB29_887:
	mov	w2, #28
	b	.LBB29_940
.LBB29_888:
	mov	w21, #-5
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_889:
	mov	w25, #-5
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_890:
	mov	w0, #-5
	sxtw	x0, w0
	b	.LBB29_1455
.LBB29_891:
	mov	w2, #32
	b	.LBB29_961
.LBB29_892:
	mov	w2, #24
	b	.LBB29_969
.LBB29_893:
	mov	w27, #-5
	sxtw	x0, w27
	b	.LBB29_1455
.LBB29_894:
	mov	w28, #-5
	sxtw	x0, w28
	b	.LBB29_1455
.LBB29_895:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_115
.LBB29_896:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_898
// %bb.897:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_898:
	ldr	x10, [x21, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_899:
	ldr	x9, [x21, #8]
	and	x8, x20, x8, asr #8
	//APP
		bics	xzr, x8, x9
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #144            // =144
	mov	w2, #28
	bl	__arch_copy_from_user
	ldr	w8, [x24, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_916
// %bb.900:
	ldr	w8, [x23, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_917
.LBB29_901:
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
	b	.LBB29_920
.LBB29_902:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_416
.LBB29_903:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_905
// %bb.904:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_905:
	ldr	x10, [x8, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_906:
	lsl	x9, x20, #8
	ldr	x8, [x8, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #144            // =144
	mov	w2, #32
	bl	__arch_copy_from_user
	ldr	w8, [x21, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_955
// %bb.907:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_956
.LBB29_908:
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
	b	.LBB29_959
.LBB29_909:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_442
.LBB29_910:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_912
// %bb.911:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_912:
	ldr	x10, [x8, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_913:
	lsl	x9, x20, #8
	ldr	x8, [x8, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #144            // =144
	mov	w2, #24
	bl	__arch_copy_from_user
	ldr	w8, [x21, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_962
// %bb.914:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_963
.LBB29_915:
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
	b	.LBB29_966
.LBB29_916:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_901
.LBB29_917:
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
	cbnz	w9, .LBB29_919
// %bb.918:
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
.LBB29_919:
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
.LBB29_920:
	cbnz	x2, .LBB29_940
// %bb.921:
	mov	x0, x22
	bl	inode_owner_or_capable
	tbz	w0, #0, .LBB29_924
// %bb.922:
	ldr	w8, [sp, #144]
	mov	w9, #-761
	tst	w8, w9
	b.eq	.LBB29_925
.LBB29_923:
	mov	w24, #-95
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_924:
	mov	w24, #-13
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_925:
	lsr	w13, w8, #2
	lsr	w12, w8, #3
	and	w13, w13, #0x8
	lsr	w10, w8, #9
	lsr	w9, w8, #6
	lsr	w11, w8, #4
	lsr	w8, w8, #1
	bfi	w13, w12, #4, #1
	ldrh	w12, [x22]
	and	w8, w8, #0x40
	bfi	w13, w11, #5, #1
	orr	w8, w13, w8
	bfi	w8, w9, #7, #1
	and	w9, w12, #0xf000
	mov	w23, w8
	cmp	w9, #4, lsl #12         // =16384
	bfi	w23, w10, #29, #1
	b.eq	.LBB29_929
// %bb.926:
	cmp	w9, #8, lsl #12         // =32768
	b.eq	.LBB29_928
// %bb.927:
	and	w8, w8, #0xc0
.LBB29_928:
	cmp	w8, w23
	b.ne	.LBB29_923
.LBB29_929:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w24, w0
	cbnz	w0, .LBB29_974
// %bb.930:
	add	x20, x22, #160          // =160
	mov	x0, x20
	bl	down_write
	ldr	w8, [x22, #704]
	add	x0, sp, #88             // =88
	mov	w2, #28
	mov	w1, wzr
	lsr	w11, w8, #1
	lsr	w10, w8, #3
	and	w12, w11, #0x8
	and	w13, w11, #0x10
	bfi	w12, w10, #5, #1
	lsr	w9, w8, #6
	orr	w10, w12, w13
	lsr	w8, w8, #20
	and	w11, w11, #0x40
	bfi	w10, w9, #7, #1
	orr	w9, w10, w11
	and	w8, w8, #0x200
	orr	w24, w9, w8
	bl	memset
	str	w24, [sp, #88]
	ldr	x8, [x22, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2180]
	tbz	w8, #4, .LBB29_932
// %bb.931:
	ldr	w1, [x22, #1188]
	adrp	x0, init_user_ns
	add	x0, x0, :lo12:init_user_ns
	bl	from_kprojid
	str	w0, [sp, #100]
.LBB29_932:
	add	x1, sp, #88             // =88
	add	x2, sp, #144            // =144
	mov	x0, x22
	bl	vfs_ioc_fssetxattr_check
	mov	w24, w0
	cbnz	w0, .LBB29_973
// %bb.933:
	mov	w2, #248
	movk	w2, #8192, lsl #16
	mov	x0, x22
	mov	w1, w23
	bl	f2fs_setflags_common
	mov	w24, w0
	cbnz	w0, .LBB29_973
// %bb.934:
	ldr	x22, [x19, #32]
	ldr	w1, [sp, #156]
	ldr	x8, [x22, #40]
	ldr	x25, [x8, #1008]
	ldr	x8, [x25, #16]
	ldrb	w8, [x8, #2180]
	tbnz	w8, #4, .LBB29_936
// %bb.935:
	cmp	w1, #0                  // =0
	mov	w8, #-95
	csel	w24, wzr, w8, eq
	b	.LBB29_973
.LBB29_936:
	ldr	x8, [x22, #736]
	tbnz	w8, #26, .LBB29_938
// %bb.937:
	mov	w24, #-95
	b	.LBB29_973
.LBB29_938:
	adrp	x0, init_user_ns
	add	x0, x0, :lo12:init_user_ns
	bl	make_kprojid
	ldr	w8, [x22, #1188]
	mov	x23, x0
	cmp	w8, w23
	b.ne	.LBB29_942
// %bb.939:
	mov	w24, wzr
	b	.LBB29_973
.LBB29_940:
	add	x8, sp, #144            // =144
	sub	x8, x8, x2
	add	x0, x8, #28             // =28
	mov	w1, wzr
	bl	memset
.LBB29_941:
	mov	w24, #-14
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_942:
	ldrb	w8, [x22, #12]
	tbnz	w8, #5, .LBB29_972
// %bb.943:
	ldr	x1, [x22, #64]
	mov	x0, x25
	bl	f2fs_get_node_page
	mov	x24, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB29_973
// %bb.944:
	ldrsw	x8, [x22, #1184]
	mov	w1, #1
	mov	x0, x24
	add	x26, x8, #360           // =360
	bl	f2fs_put_page
	cmp	x26, #368               // =368
	b.lo	.LBB29_1435
// %bb.945:
	mov	x0, x22
	bl	dquot_initialize
	mov	w24, w0
	cbnz	w0, .LBB29_973
// %bb.946:
	add	x24, x25, #368          // =368
	mov	x0, x24
	and	x26, x23, #0xffffffff
	bl	_raw_spin_lock
	ldr	x8, [x25, #352]
	cbz	x8, .LBB29_948
// %bb.947:
	adrp	x27, jiffies
	ldr	x8, [x25, #360]
	ldr	x9, [x27, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB29_1874
.LBB29_948:
	mov	x0, x24
	bl	_raw_spin_unlock
	add	x21, x25, #304          // =304
	mov	x0, x21
	bl	down_read
	stp	xzr, xzr, [sp, #128]
	str	xzr, [sp, #120]
	ldr	x8, [x22, #40]
	orr	x1, x26, #0x200000000
	ldr	x25, [x8, #1008]
	ldr	x0, [x25]
	bl	dqget
	cmn	x0, #4095               // =4095
	str	x0, [sp, #136]
	b.hs	.LBB29_953
// %bb.949:
	add	x1, sp, #120            // =120
	mov	x0, x22
	bl	__dquot_transfer
	mov	w24, w0
	cbz	x25, .LBB29_952
// %bb.950:
	cbz	w24, .LBB29_952
// %bb.951:
	add	x8, x25, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4096
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB29_952:
	ldr	x0, [sp, #136]
	bl	dqput
	cbnz	w24, .LBB29_954
.LBB29_953:
	mov	x0, x22
	str	w23, [x22, #1188]
	bl	current_time
	stp	x0, x1, [x22, #120]
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_mark_inode_dirty_sync
	mov	w24, wzr
.LBB29_954:
	mov	x0, x21
	bl	up_read
	b	.LBB29_973
.LBB29_955:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_908
.LBB29_956:
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
	cbnz	w9, .LBB29_958
// %bb.957:
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
.LBB29_958:
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
.LBB29_959:
	cbnz	x2, .LBB29_961
// %bb.960:
	add	x1, sp, #144            // =144
	mov	x0, x19
	bl	__f2fs_ioc_move_range
	b	.LBB29_968
.LBB29_961:
	add	x8, sp, #144            // =144
	sub	x8, x8, x2
	add	x0, x8, #32             // =32
	b	.LBB29_970
.LBB29_962:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_915
.LBB29_963:
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
	cbnz	w9, .LBB29_965
// %bb.964:
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
.LBB29_965:
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
.LBB29_966:
	cbnz	x2, .LBB29_969
// %bb.967:
	add	x1, sp, #144            // =144
	mov	x0, x19
	bl	__f2fs_ioc_gc_range
.LBB29_968:
                                        // kill: def $w0 killed $w0 def $x0
	sxtw	x0, w0
	b	.LBB29_1455
.LBB29_969:
	add	x8, sp, #144            // =144
	sub	x8, x8, x2
	add	x0, x8, #24             // =24
.LBB29_970:
	mov	w1, wzr
	bl	memset
	mov	x0, #-14
	b	.LBB29_1455
.LBB29_971:
	mov	w23, wzr
	b	.LBB29_781
.LBB29_972:
	mov	w24, #-1
.LBB29_973:
	mov	x0, x20
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
.LBB29_974:
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_975:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x21, #360]
	ldr	x2, [x23, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x21, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB29_229
.LBB29_976:
	cbz	w23, .LBB29_978
.LBB29_977:
	add	x8, x22, #736           // =736
	mov	x9, #17179869184
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	b	.LBB29_979
.LBB29_978:
	ldr	x0, [x22, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	x8, #17179869184
	ldr	x11, [sp, #72]          // 8-byte Folded Reload
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x11]
1:	ldxr	x9, [x11]
	bic	x9, x9, x8
	stxr	w10, x9, [x11]
	cbnz	w10, 1b
	//NO_APP
	mov	w23, w0
	cbz	w0, .LBB29_781
.LBB29_979:
	adrp	x1, .L.str.51
	adrp	x2, .L__func__.f2fs_ioc_compress_file
	add	x1, x1, :lo12:.L.str.51
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_compress_file
	b	.LBB29_1138
.LBB29_980:
	adrp	x11, cpu_hwcaps
	ldr	x11, [x11, :lo12:cpu_hwcaps]
	tbnz	w11, #4, .LBB29_23
.LBB29_981:
	//APP
	.if 1 == 1
661:
	mrs	x11, daif
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
	mrs_s x11, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w12, w11, #0x00000080
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
	eor	w12, w11, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w12, .LBB29_983
// %bb.982:
	mov	w12, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_983:
	ldr	x8, [x8, #16]
	//APP
	mrs x12, ttbr1_el1
	//NO_APP
	mov	x13, x8
	bfxil	x13, x12, #0, #48
	//APP
	msr ttbr1_el1, x13
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x11
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x11
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_984:
	mov	w8, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     w23, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr w23, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w8, -14
	mov	x23, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w9, [x10, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1030
// %bb.985:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1031
.LBB29_986:
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
	b	.LBB29_1034
.LBB29_987:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_86
.LBB29_988:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_990
// %bb.989:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_990:
	ldr	x10, [x8, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_991:
	lsl	x9, x20, #8
	ldr	x8, [x8, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x0, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x1, sp, #144            // =144
	mov	w2, #28
	bl	__arch_copy_to_user
	ldr	w8, [x19, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1063
// %bb.992:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1064
.LBB29_993:
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
	b	.LBB29_1067
.LBB29_994:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbnz	w12, #4, .LBB29_353
.LBB29_995:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_997
// %bb.996:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_997:
	ldr	x8, [x8, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x8
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_998:
	mov	w8, wzr
	//APP
	1:.if 1 == 1
661:
	strb     w11, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttrb w11, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w8, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w9, [x10, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1068
// %bb.999:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1069
.LBB29_1000:
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
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1001:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbnz	w12, #4, .LBB29_363
.LBB29_1002:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_1004
// %bb.1003:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1004:
	ldr	x8, [x8, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x8
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1005:
	mov	w8, wzr
	//APP
	1:.if 1 == 1
661:
	str     w11, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr w11, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w8, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w9, [x10, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1072
// %bb.1006:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1073
.LBB29_1007:
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
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1008:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbnz	w12, #4, .LBB29_400
.LBB29_1009:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_1011
// %bb.1010:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1011:
	ldr	x8, [x8, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x8
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1012:
	mov	w8, wzr
	//APP
	1:.if 1 == 1
661:
	str     w11, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr w11, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w8, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w9, [x10, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1076
// %bb.1013:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1077
.LBB29_1014:
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
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1015:
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "include/linux/thread_info.h"; .popsection; .long 14472b - 14470b; .short 150; .short (1 << 0)|((1 << 1) | ((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	b	.LBB29_1150
.LBB29_1016:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbnz	w12, #4, .LBB29_503
.LBB29_1017:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_1019
// %bb.1018:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1019:
	ldr	x9, [x9, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x9
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1020:
	mov	w9, wzr
	//APP
	1:.if 1 == 1
661:
	str     w8, [x10]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr w8, [x10]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w9, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w8, [x11, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1080
// %bb.1021:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1081
.LBB29_1022:
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
	sxtw	x0, w9
	b	.LBB29_1455
.LBB29_1023:
	adrp	x11, cpu_hwcaps
	ldr	x11, [x11, :lo12:cpu_hwcaps]
	tbnz	w11, #4, .LBB29_554
.LBB29_1024:
	//APP
	.if 1 == 1
661:
	mrs	x11, daif
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
	mrs_s x11, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w12, w11, #0x00000080
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
	eor	w12, w11, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w12, .LBB29_1026
// %bb.1025:
	mov	w12, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1026:
	ldr	x8, [x8, #16]
	//APP
	mrs x12, ttbr1_el1
	//NO_APP
	mov	x13, x8
	bfxil	x13, x12, #0, #48
	//APP
	msr ttbr1_el1, x13
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x11
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x11
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1027:
	mov	w8, wzr
	//APP
	1:.if 1 == 1
661:
	strb     w21, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttrb w21, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w8, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w9, [x10, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1084
// %bb.1028:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1085
.LBB29_1029:
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
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1030:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_986
.LBB29_1031:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1033
// %bb.1032:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1033:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1034:
	cbz	w8, .LBB29_1036
.LBB29_1035:
	mov	w22, #-14
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_1036:
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_1041
// %bb.1037:
	ldr	x8, [x21, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_1042
// %bb.1038:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w22, w0
	cbnz	w0, .LBB29_668
// %bb.1039:
	add	x20, x21, #160          // =160
	mov	x0, x20
	bl	down_write
	mov	x0, x21
	mov	x1, xzr
	bl	f2fs_should_update_outplace
	tbz	w0, #0, .LBB29_1043
// %bb.1040:
	mov	w22, #-22
	b	.LBB29_659
.LBB29_1041:
	mov	w22, #-22
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_1042:
	mov	w22, #-30
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_1043:
	ldr	x8, [x21, #736]
	tbz	x8, #44, .LBB29_1045
// %bb.1044:
	add	x8, x21, #736           // =736
	mov	x9, #17592186044416
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
.LBB29_1045:
	ldr	x8, [x21, #736]
	tbz	x8, #38, .LBB29_1049
// %bb.1046:
	ldr	x8, [x21, #736]
	tbz	x8, #38, .LBB29_1048
// %bb.1047:
	ldr	x8, [x21, #736]
	tbnz	x8, #39, .LBB29_1049
.LBB29_1048:
	adrp	x1, .L__func__.f2fs_ioc_set_pin_file
	add	x1, x1, :lo12:.L__func__.f2fs_ioc_set_pin_file
	mov	x0, x21
	bl	f2fs_revoke_deduped_inode
	mov	w22, w0
	cbnz	w0, .LBB29_659
.LBB29_1049:
	cbz	w23, .LBB29_1052
// %bb.1050:
	ldr	x8, [x21, #40]
	ldr	w4, [x21, #720]
	ldr	x0, [x8, #1008]
	ldr	x8, [x0, #2032]
	cmp	x8, x4
	b.hs	.LBB29_1054
// %bb.1051:
	ldr	x3, [x21, #64]
	adrp	x1, .L.str.9
	adrp	x2, .L__func__.f2fs_pin_file_control
	add	x1, x1, :lo12:.L.str.9
	add	x2, x2, :lo12:.L__func__.f2fs_pin_file_control
                                        // kill: def $w4 killed $w4 killed $x4
	bl	f2fs_printk
	add	x8, x21, #736           // =736
	mov	w9, #268435456
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	mov	w22, #-11
	b	.LBB29_659
.LBB29_1052:
	add	x8, x21, #736           // =736
	mov	w9, #268435456
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	mov	w1, #1
	mov	x0, x21
	str	wzr, [x21, #720]
	bl	f2fs_mark_inode_dirty_sync
	mov	w22, wzr
.LBB29_1053:
	ldr	x8, [x21, #40]
	adrp	x9, jiffies
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	b	.LBB29_659
.LBB29_1054:
	mov	x0, x21
	bl	f2fs_convert_inline_inode
	mov	w22, w0
	cbnz	w0, .LBB29_659
// %bb.1055:
	ldr	x8, [x21, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB29_1059
// %bb.1056:
	ldr	w8, [x21, #1280]
	cbnz	w8, .LBB29_1061
// %bb.1057:
	ldr	x8, [x21, #736]
	tbnz	x8, #35, .LBB29_1061
// %bb.1058:
	ldr	x8, [x21, #736]
	tbnz	x8, #34, .LBB29_1061
	b	.LBB29_1062
.LBB29_1059:
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_1062
// %bb.1060:
	ldr	x8, [x21, #736]
	tbz	w8, #31, .LBB29_1062
.LBB29_1061:
	mov	w1, #12
	mov	x0, x19
	mov	w2, wzr
	bl	f2fs_decompress_file_internal
	mov	x22, x0
	cbnz	w22, .LBB29_659
.LBB29_1062:
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 268435456
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	ldr	w22, [x21, #720]
	b	.LBB29_1053
.LBB29_1063:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_993
.LBB29_1064:
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
	cbnz	w9, .LBB29_1066
// %bb.1065:
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
.LBB29_1066:
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
.LBB29_1067:
	cmp	x0, #0                  // =0
	b	.LBB29_1151
.LBB29_1068:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_1000
.LBB29_1069:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1071
// %bb.1070:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1071:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1072:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_1007
.LBB29_1073:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1075
// %bb.1074:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1075:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1076:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_1014
.LBB29_1077:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1079
// %bb.1078:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1079:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1080:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1022
.LBB29_1081:
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
	and	w10, w8, #0x00000080
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
	eor	w10, w8, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1083
// %bb.1082:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1083:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
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
	sxtw	x0, w9
	b	.LBB29_1455
.LBB29_1084:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_1029
.LBB29_1085:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1087
// %bb.1086:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1087:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1088:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbnz	w12, #4, .LBB29_133
.LBB29_1089:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_1091
// %bb.1090:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1091:
	ldr	x8, [x8, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x8
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1092:
	mov	w8, wzr
	//APP
	1:.if 1 == 1
661:
	str     w11, [x10]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr w11, [x10]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w8, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w9, [x9, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1095
// %bb.1093:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1096
.LBB29_1094:
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
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1095:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_1094
.LBB29_1096:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1098
// %bb.1097:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1098:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1099:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbnz	w12, #4, .LBB29_483
.LBB29_1100:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_1102
// %bb.1101:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1102:
	ldr	x8, [x8, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x8
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1103:
	mov	w8, wzr
	//APP
	1:.if 1 == 1
661:
	str     w9, [x10]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr w9, [x10]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w8, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w9, [x11, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1106
// %bb.1104:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1107
.LBB29_1105:
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
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1106:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_1105
.LBB29_1107:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1109
// %bb.1108:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1109:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	sxtw	x0, w8
	b	.LBB29_1455
.LBB29_1110:
	ldr	x2, [x22, #64]
	adrp	x1, .L.str.50
	add	x1, x1, :lo12:.L.str.50
.LBB29_1111:
	mov	x0, x21
	bl	f2fs_printk
.LBB29_1112:
	mov	w27, wzr
	mov	w24, wzr
	mov	w25, #-95
.LBB29_1113:
	ldr	x26, [sp, #72]          // 8-byte Folded Reload
.LBB29_1114:
	mov	x0, x26
	bl	up_write
	ldr	x8, [x19, #32]
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB29_1116
// %bb.1115:
	ldr	x0, [x8, #40]
	mov	w1, #1
	bl	__sb_end_write
.LBB29_1116:
	cbnz	w25, .LBB29_1391
// %bb.1117:
	tbnz	w27, #31, .LBB29_1125
// %bb.1118:
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w10, [x8, #62]
	ldr	x9, [x8, #8]
	tbnz	w10, #5, .LBB29_1120
// %bb.1119:
	ldr	x11, [x8]
	mov	x10, x20
	tbz	w11, #26, .LBB29_1121
.LBB29_1120:
	lsl	x10, x20, #8
	and	x10, x20, x10, asr #8
.LBB29_1121:
	//APP
		adds	x10, x10, 8
	csel	x9, xzr, x9, hi
	csinv	x10, x10, xzr, cc
	sbcs	xzr, x10, x9
	cset	x10, ls

	//NO_APP
	cbz	x10, .LBB29_1555
// %bb.1122:
	ldr	x10, [x8, #8]
	lsl	x9, x20, #8
	and	x11, x20, x9, asr #8
	//APP
		bics	xzr, x11, x10
	csel	x9, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x10, arm64_const_caps_ready
	ldr	w11, [x10, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	mov	w11, w24
	b.lt	.LBB29_1900
// %bb.1123:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB29_1901
.LBB29_1124:
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
	b	.LBB29_1904
.LBB29_1125:
	cbz	w24, .LBB29_1133
// %bb.1126:
	ldr	w8, [x22, #1280]
	cbz	w8, .LBB29_1133
// %bb.1127:
	cbz	x21, .LBB29_1132
// %bb.1128:
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
	cbnz	w8, .LBB29_1132
// %bb.1129:
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB29_1132
// %bb.1130:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB29_1132
// %bb.1131:
	add	x2, x21, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB29_1132:
	ldr	x3, [x22, #64]
	ldr	x4, [x22, #144]
	ldr	w6, [x22, #1280]
	adrp	x1, .L.str.45
	adrp	x2, .L__func__.f2fs_ioc_compress_release_file
	add	x1, x1, :lo12:.L.str.45
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_compress_release_file
	mov	x0, x21
	mov	w5, w24
	bl	f2fs_printk
.LBB29_1133:
	mov	w25, w27
	b	.LBB29_1391
.LBB29_1134:
	mov	w25, wzr
	mov	w23, wzr
	b	.LBB29_621
.LBB29_1135:
	cbnz	w23, .LBB29_1137
.LBB29_1136:
	ldr	x0, [x22, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w23, w0
	cbz	w0, .LBB29_781
.LBB29_1137:
	adrp	x1, .L.str.55
	adrp	x2, .L__func__.f2fs_ioc_decompress_file
	add	x1, x1, :lo12:.L.str.55
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_decompress_file
.LBB29_1138:
	mov	x0, x20
	mov	w3, w23
	bl	f2fs_printk
	b	.LBB29_781
.LBB29_1139:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_409
.LBB29_1140:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1142
// %bb.1141:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1142:
	ldr	x10, [x8, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1143:
	lsl	x9, x20, #8
	ldr	x8, [x8, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x0, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	mov	x1, x19
	mov	x2, x21
	bl	__arch_copy_to_user
	ldr	w8, [x22, :lo12:arm64_const_caps_ready]
	mov	x21, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1146
// %bb.1144:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1147
.LBB29_1145:
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
	b	.LBB29_1150
.LBB29_1146:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1145
.LBB29_1147:
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
	cbnz	w9, .LBB29_1149
// %bb.1148:
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
.LBB29_1149:
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
.LBB29_1150:
	mov	x0, x19
	bl	kfree
	cmp	x21, #0                 // =0
.LBB29_1151:
	mov	x8, #-14
	csel	x0, xzr, x8, eq
	b	.LBB29_1455
.LBB29_1152:
	mov	w2, #24
	b	.LBB29_1177
.LBB29_1153:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_567
.LBB29_1154:
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
	cbnz	w9, .LBB29_1156
// %bb.1155:
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
.LBB29_1156:
	ldr	x9, [x23, #16]
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	mov	x11, x9
	bfxil	x11, x10, #0, #48
	//APP
	msr ttbr1_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
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
.LBB29_1157:
	lsl	x9, x20, #8
	ldr	x8, [x23, #8]
	and	x27, x20, x9, asr #8
	//APP
		bics	xzr, x27, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #144            // =144
	mov	w2, #24
	bl	__arch_copy_from_user
	ldr	w8, [x25, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1160
// %bb.1158:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1161
.LBB29_1159:
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
	b	.LBB29_1164
.LBB29_1160:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1159
.LBB29_1161:
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
	cbnz	w9, .LBB29_1163
// %bb.1162:
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
.LBB29_1163:
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
.LBB29_1164:
	cbnz	x2, .LBB29_1177
// %bb.1165:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w21, w0
	cbz	w0, .LBB29_1167
.LBB29_1166:
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_1167:
	ldr	w8, [sp, #160]
	ldr	w9, [x26, #2960]
	adrp	x0, f2fs_ioc_fitrim._rs
	adrp	x1, .L__func__.f2fs_ioc_fitrim
	add	x0, x0, :lo12:f2fs_ioc_fitrim._rs
	cmp	w9, w8
	csel	w8, w8, w9, lo
	add	x1, x1, :lo12:.L__func__.f2fs_ioc_fitrim
	str	x8, [sp, #160]
	bl	___ratelimit
	cbnz	w0, .LBB29_1176
.LBB29_1168:
	ldr	x0, [x24, #1008]
	add	x1, sp, #144            // =144
	bl	f2fs_trim_fs
	mov	w21, w0
	mov	x0, x19
	bl	mnt_drop_write_file
	tbnz	w21, #31, .LBB29_1166
// %bb.1169:
	ldrb	w9, [x23, #62]
	ldr	x8, [x23, #8]
	tbnz	w9, #5, .LBB29_1171
// %bb.1170:
	ldr	x10, [x23]
	mov	x9, x20
	tbz	w10, #26, .LBB29_1172
.LBB29_1171:
	mov	x9, x27
.LBB29_1172:
	//APP
		adds	x9, x9, 24
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1605
// %bb.1173:
	ldr	w8, [x25, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1593
// %bb.1174:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1594
.LBB29_1175:
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
	b	.LBB29_1597
.LBB29_1176:
	adrp	x0, .L.str.39
	adrp	x1, .L__func__.f2fs_ioc_fitrim
	add	x0, x0, :lo12:.L.str.39
	add	x1, x1, :lo12:.L__func__.f2fs_ioc_fitrim
	bl	printk
	b	.LBB29_1168
.LBB29_1177:
	add	x8, sp, #144            // =144
	sub	x8, x8, x2
	add	x0, x8, #24             // =24
	mov	w1, wzr
	bl	memset
	b	.LBB29_1605
.LBB29_1178:
	adrp	x10, cpu_hwcaps
	ldr	x10, [x10, :lo12:cpu_hwcaps]
	tbz	w10, #4, .LBB29_1179
	b	.LBB29_143
.LBB29_1179:
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
	cbnz	w11, .LBB29_1181
// %bb.1180:
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
.LBB29_1181:
	ldr	x11, [x24, #16]
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
.LBB29_1182:
	mov	w10, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     w20, [x8]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr w20, [x8]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w10, -14
	mov	x20, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w8, [x9, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1206
// %bb.1183:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1207
.LBB29_1184:
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
	b	.LBB29_1210
.LBB29_1185:
	adrp	x11, cpu_hwcaps
	ldr	x11, [x11, :lo12:cpu_hwcaps]
	tbz	w11, #4, .LBB29_1186
	b	.LBB29_161
.LBB29_1186:
	//APP
	.if 1 == 1
661:
	mrs	x11, daif
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
	mrs_s x11, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w12, w11, #0x00000080
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
	eor	w12, w11, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w12, .LBB29_1188
// %bb.1187:
	mov	w12, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1188:
	ldr	x8, [x8, #16]
	//APP
	mrs x12, ttbr1_el1
	//NO_APP
	mov	x13, x8
	bfxil	x13, x12, #0, #48
	//APP
	msr ttbr1_el1, x13
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x11
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x11
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1189:
	mov	w8, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     w22, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr w22, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w8, -14
	mov	x22, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w9, [x10, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1245
// %bb.1190:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1246
.LBB29_1191:
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
	b	.LBB29_1249
.LBB29_1192:
	adrp	x10, cpu_hwcaps
	ldr	x10, [x10, :lo12:cpu_hwcaps]
	tbnz	w10, #4, .LBB29_424
.LBB29_1193:
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
	cbnz	w11, .LBB29_1195
// %bb.1194:
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
.LBB29_1195:
	ldr	x11, [x23, #16]
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
.LBB29_1196:
	mov	w10, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     w20, [x8]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr w20, [x8]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w10, -14
	mov	x20, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w8, [x9, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1255
// %bb.1197:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1256
.LBB29_1198:
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
	b	.LBB29_1259
.LBB29_1199:
	adrp	x11, cpu_hwcaps
	ldr	x11, [x11, :lo12:cpu_hwcaps]
	tbnz	w11, #4, .LBB29_434
.LBB29_1200:
	//APP
	.if 1 == 1
661:
	mrs	x11, daif
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
	mrs_s x11, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w12, w11, #0x00000080
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
	eor	w12, w11, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w12, .LBB29_1202
// %bb.1201:
	mov	w12, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1202:
	ldr	x8, [x8, #16]
	//APP
	mrs x12, ttbr1_el1
	//NO_APP
	mov	x13, x8
	bfxil	x13, x12, #0, #48
	//APP
	msr ttbr1_el1, x13
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x11
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x11
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1203:
	mov	w8, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     w23, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr w23, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w8, -14
	mov	x23, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w9, [x10, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB29_1268
// %bb.1204:
	adrp	x9, cpu_hwcap_keys+16
	ldr	w9, [x9, :lo12:cpu_hwcap_keys+16]
	cmp	w9, #0                  // =0
	b.le	.LBB29_1269
.LBB29_1205:
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
	b	.LBB29_1272
.LBB29_1206:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1184
.LBB29_1207:
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
	cbnz	w9, .LBB29_1209
// %bb.1208:
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
.LBB29_1209:
	//APP
	mrs x9, ttbr1_el1
	//NO_APP
	and	x9, x9, #0xffffffffffff
	sub	x11, x9, #1, lsl #12    // =4096
	//APP
	msr ttbr0_el1, x11
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
.LBB29_1210:
	cbz	w10, .LBB29_1212
.LBB29_1211:
	mov	w23, #-14
	b	.LBB29_1454
.LBB29_1212:
	cbz	w20, .LBB29_1219
// %bb.1213:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w23, w0
	cbnz	w0, .LBB29_1454
// %bb.1214:
	sub	w8, w20, #1             // =1
	cmp	w8, #3                  // =3
	b.hi	.LBB29_1236
// %bb.1215:
	adrp	x9, .LJTI29_2
	add	x9, x9, :lo12:.LJTI29_2
	adr	x10, .LBB29_1216
	ldrb	w11, [x9, x8]
	add	x10, x10, x11, lsl #2
	br	x10
.LBB29_1216:
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_sync_fs
	mov	w22, w0
	cbnz	w0, .LBB29_1237
// %bb.1217:
	mov	x0, x21
	mov	w1, wzr
	bl	f2fs_stop_checkpoint
	cbz	x21, .LBB29_1235
// %bb.1218:
	add	x8, x21, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 64
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB29_1235
.LBB29_1219:
	ldr	x0, [x22, #248]
	bl	freeze_bdev
	mov	x23, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB29_1238
// %bb.1220:
	cbz	x23, .LBB29_1235
// %bb.1221:
	mov	x0, x21
	mov	w1, wzr
	bl	f2fs_stop_checkpoint
	cbz	x21, .LBB29_1223
// %bb.1222:
	add	x8, x21, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 64
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB29_1223:
	ldr	x0, [x23, #248]
	mov	x1, x23
	bl	thaw_bdev
	b	.LBB29_1235
.LBB29_1224:
	mov	w1, #2
	mov	x2, #9223372036854775807
	mov	w3, #6
	mov	x0, x21
	bl	f2fs_sync_meta_pages
	mov	x0, x21
	mov	w1, wzr
	bl	f2fs_stop_checkpoint
	cbz	x21, .LBB29_1235
// %bb.1225:
	add	x8, x21, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 64
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB29_1235
.LBB29_1226:
	cbz	x21, .LBB29_1232
// %bb.1227:
	add	x23, x21, #80           // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x23]
1:	ldxr	x8, [x23]
	orr	x8, x8, 4
	stxr	w9, x8, [x23]
	cbnz	w9, 1b
	//NO_APP
	ldrb	w8, [x21, #3376]
	cbnz	w8, .LBB29_1231
// %bb.1228:
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB29_1231
// %bb.1229:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB29_1231
// %bb.1230:
	add	x2, x21, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB29_1231:
	//APP
	// atomic64_or
	prfm	pstl1strm, [x23]
1:	ldxr	x8, [x23]
	orr	x8, x8, 512
	stxr	w9, x8, [x23]
	cbnz	w9, 1b
	//NO_APP
	//APP
	// atomic64_or
	prfm	pstl1strm, [x23]
1:	ldxr	x8, [x23]
	orr	x8, x8, 1
	stxr	w9, x8, [x23]
	cbnz	w9, 1b
	//NO_APP
.LBB29_1232:
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_sync_fs
	mov	w22, w0
	b	.LBB29_1237
.LBB29_1233:
	mov	x0, x21
	mov	w1, wzr
	bl	f2fs_stop_checkpoint
	cbz	x21, .LBB29_1235
// %bb.1234:
	add	x8, x21, #80            // =80
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 64
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB29_1235:
	mov	x0, x21
	bl	f2fs_stop_gc_thread
	mov	x0, x21
	bl	f2fs_stop_discard_thread
	mov	x0, x21
	bl	f2fs_drop_discard_cmd
	ldr	w8, [x21, #1424]
	adrp	x9, jiffies
	mov	w22, wzr
	mov	w23, wzr
	and	w8, w8, #0xfffffffb
	str	w8, [x21, #1424]
	ldr	x8, [x9, :lo12:jiffies]
	stp	x8, x8, [x21, #504]
	str	x8, [x21, #520]
	cbnz	w20, .LBB29_1237
	b	.LBB29_1238
.LBB29_1236:
	mov	w22, #-22
.LBB29_1237:
	mov	x0, x19
	bl	mnt_drop_write_file
	mov	w23, w22
.LBB29_1238:
	adrp	x8, __tracepoint_f2fs_shutdown+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_shutdown+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1454
// %bb.1239:
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
	tbz	w8, #0, .LBB29_1454
// %bb.1240:
	ldr	w8, [x24, #24]
	add	w8, w8, #1              // =1
	str	w8, [x24, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_shutdown+32
	ldr	x19, [x8, :lo12:__tracepoint_f2fs_shutdown+32]
	cbz	x19, .LBB29_1242
.LBB29_1241:                            // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x19]
	mov	x1, x21
	mov	w2, w20
	mov	w3, w23
	blr	x8
	ldr	x8, [x19, #24]!
	cbnz	x8, .LBB29_1241
.LBB29_1242:
	//APP
	//NO_APP
	ldr	x8, [x24, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x24, #24]
	b.eq	.LBB29_1244
// %bb.1243:
	ldr	x8, [x24, #24]
	cbnz	x8, .LBB29_1454
.LBB29_1244:
	bl	preempt_schedule_notrace
	b	.LBB29_1454
.LBB29_1245:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_1191
.LBB29_1246:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1248
// %bb.1247:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1248:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1249:
	cbz	w8, .LBB29_1251
.LBB29_1250:
	mov	w0, #-14
	sxtw	x0, w0
	b	.LBB29_1455
.LBB29_1251:
	mov	w23, #65533
	movk	w23, #63983, lsl #16
	tst	x22, x23
	b.eq	.LBB29_1253
.LBB29_1252:
	mov	w0, #-95
	sxtw	x0, w0
	b	.LBB29_1455
.LBB29_1253:
	mov	x0, x19
	bl	mnt_want_write_file
                                        // kill: def $w0 killed $w0 def $x0
	cbnz	w0, .LBB29_1899
// %bb.1254:
	add	x20, x21, #160          // =160
	mov	x0, x20
	bl	down_write
	ldr	x8, [x21, #704]
	mov	w9, #2
	movk	w9, #1552, lsl #16
	and	x9, x22, x9
	and	x8, x8, x23
	orr	x8, x8, x9
	mov	x0, x21
	str	x8, [x21, #704]
	bl	current_time
	stp	x0, x1, [x21, #120]
	mov	x0, x21
	bl	f2fs_set_inode_flags
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x20
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
	mov	w0, wzr
	b	.LBB29_1899
.LBB29_1255:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1198
.LBB29_1256:
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
	cbnz	w9, .LBB29_1258
// %bb.1257:
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
.LBB29_1258:
	//APP
	mrs x9, ttbr1_el1
	//NO_APP
	and	x9, x9, #0xffffffffffff
	sub	x11, x9, #1, lsl #12    // =4096
	//APP
	msr ttbr0_el1, x11
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
.LBB29_1259:
	cbnz	w10, .LBB29_1273
// %bb.1260:
	ldr	x8, [x21]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB29_1264
// %bb.1261:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w22, w0
	cbnz	w0, .LBB29_668
// %bb.1262:
	add	x0, x21, #1664          // =1664
	cbz	w20, .LBB29_1265
// %bb.1263:
	bl	down_write
	b	.LBB29_1266
.LBB29_1264:
	mov	w22, #-30
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_1265:
	bl	down_write_trylock
	cbz	w0, .LBB29_1267
.LBB29_1266:
	ldr	w8, [x21, #1236]
	ldr	w9, [x21, #1168]
	ldr	w10, [x21, #1240]
	adrp	x11, IOC_GC_count
	ldr	w4, [x11, :lo12:IOC_GC_count]
	mul	x12, x9, x8
	sub	w8, w8, w10
	mul	x8, x8, x9
	adrp	x1, .L.str.41
	cmp	w20, #0                 // =0
	lsr	x2, x12, #20
	add	w9, w4, #1              // =1
	lsr	x3, x8, #20
	add	x1, x1, :lo12:.L.str.41
	mov	x0, x21
	mov	w5, w20
	cset	w22, ne
	str	w9, [x11, :lo12:IOC_GC_count]
	bl	f2fs_printk
	ldr	w8, [x23, #60]
	mov	w2, #1
	mov	w4, #-1
	mov	x0, x21
	orr	w8, w8, #0x20000000
	mov	w1, w22
	mov	w3, wzr
	str	w8, [x23, #60]
	bl	f2fs_gc
	ldr	w8, [x23, #60]
	mov	w22, w0
	and	w8, w8, #0xdfffffff
	str	w8, [x23, #60]
	b	.LBB29_667
.LBB29_1267:
	mov	w22, #-16
	b	.LBB29_667
.LBB29_1268:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #4, .LBB29_1205
.LBB29_1269:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1271
// %bb.1270:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1271:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1272:
	cbz	w8, .LBB29_1274
.LBB29_1273:
	mov	w22, #-14
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_1274:
	mov	w8, #58113
	movk	w8, #35182, lsl #16
	tst	w23, w8
	b.eq	.LBB29_1276
.LBB29_1275:
	mov	w22, #-95
	sxtw	x0, w22
	b	.LBB29_1455
.LBB29_1276:
	ldrh	w8, [x21]
	mov	w24, #1276
	movk	w24, #24577, lsl #16
	and	w20, w23, w24
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB29_1281
// %bb.1277:
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_1279
// %bb.1278:
	mov	w8, #1276
	b	.LBB29_1280
.LBB29_1279:
	mov	w8, #192
.LBB29_1280:
	and	w8, w8, w23
	cmp	w8, w20
	b.ne	.LBB29_1275
.LBB29_1281:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w22, w0
	cbnz	w0, .LBB29_668
// %bb.1282:
	mov	w8, #1278
	movk	w8, #26129, lsl #16
	and	w22, w23, w8
	add	x23, x21, #160          // =160
	mov	x0, x23
	bl	down_write
	ldr	w8, [x21, #704]
	add	w9, w24, #1, lsl #12    // =4096
	mov	x0, x21
	mov	w2, w22
	and	w1, w8, w9
	bl	vfs_ioc_setflags_prepare
	mov	w22, w0
	cbnz	w0, .LBB29_1284
// %bb.1283:
	mov	w2, #1276
	movk	w2, #24577, lsl #16
	mov	x0, x21
	mov	w1, w20
	bl	f2fs_setflags_common
	mov	w22, w0
.LBB29_1284:
	mov	x0, x23
	b	.LBB29_666
.LBB29_1285:
	mov	w23, wzr
	mov	w25, wzr
.LBB29_1286:
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	up_write
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_write
	b	.LBB29_621
.LBB29_1287:
	mov	w27, wzr
	mov	w24, wzr
	mov	w25, #-16
	b	.LBB29_1113
.LBB29_1288:
	adrp	x1, .L.str.42
	add	x1, x1, :lo12:.L.str.42
	mov	x0, x20
	bl	f2fs_printk
	mov	w21, #-22
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_1289:
	mov	w2, #8
	b	.LBB29_1305
.LBB29_1290:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbz	w9, #4, .LBB29_1291
	b	.LBB29_63
.LBB29_1291:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1293
// %bb.1292:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1293:
	ldr	x10, [x8, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1294:
	lsl	x9, x20, #8
	ldr	x8, [x8, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #144            // =144
	mov	w2, #8
	bl	__arch_copy_from_user
	ldr	w8, [x21, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1299
// %bb.1295:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1300
.LBB29_1296:
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
	b	.LBB29_1303
.LBB29_1297:
	mov	w2, #8
	b	.LBB29_1373
.LBB29_1298:
	mov	w2, #8
	b	.LBB29_1401
.LBB29_1299:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1296
.LBB29_1300:
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
	cbnz	w9, .LBB29_1302
// %bb.1301:
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
.LBB29_1302:
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
.LBB29_1303:
	cbnz	x2, .LBB29_1305
// %bb.1304:
	ldr	x1, [sp, #144]
	mov	x0, x19
	bl	f2fs_resize_fs
	b	.LBB29_968
.LBB29_1305:
	add	x8, sp, #144            // =144
	sub	x8, x8, x2
	add	x0, x8, #8              // =8
	b	.LBB29_970
.LBB29_1306:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbz	w8, #4, .LBB29_1307
	b	.LBB29_105
.LBB29_1307:
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
	cbnz	w9, .LBB29_1309
// %bb.1308:
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
.LBB29_1309:
	ldr	x9, [x25, #16]
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	mov	x11, x9
	bfxil	x11, x10, #0, #48
	//APP
	msr ttbr1_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
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
.LBB29_1310:
	lsl	x9, x20, #8
	ldr	x8, [x25, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #144            // =144
	mov	w2, #8
	bl	__arch_copy_from_user
	ldr	w8, [x22, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1320
// %bb.1311:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1321
.LBB29_1312:
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
	b	.LBB29_1324
.LBB29_1313:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbz	w9, #4, .LBB29_1314
	b	.LBB29_196
.LBB29_1314:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1316
// %bb.1315:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1316:
	ldr	x10, [x8, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1317:
	lsl	x9, x20, #8
	ldr	x8, [x8, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #120            // =120
	mov	w2, #8
	bl	__arch_copy_from_user
	ldr	w8, [x22, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1374
// %bb.1318:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1375
.LBB29_1319:
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
	b	.LBB29_1378
.LBB29_1320:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1312
.LBB29_1321:
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
	cbnz	w9, .LBB29_1323
// %bb.1322:
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
.LBB29_1323:
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
.LBB29_1324:
	cbnz	x2, .LBB29_1373
// %bb.1325:
	ldr	w3, [x21, #3128]
	subs	w8, w3, #1              // =1
	b.le	.LBB29_1332
// %bb.1326:
	ldr	w2, [sp, #144]
	cmp	w8, w2
	b.ls	.LBB29_1333
// %bb.1327:
	ldr	w8, [x21, #1192]
	cmp	w8, #2                  // =2
	b.hs	.LBB29_1333
// %bb.1328:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w20, w0
	cbnz	w0, .LBB29_1372
// %bb.1329:
	ldr	w10, [sp, #144]
	ldr	x8, [x21, #3136]
	cbz	w10, .LBB29_1335
// %bb.1330:
	mov	w9, #88
	madd	x9, x10, x9, x8
	ldr	w9, [x9, #76]
	add	w11, w9, #3             // =3
	cmp	w11, #4                 // =4
	b.hs	.LBB29_1336
.LBB29_1331:
	mov	w9, #-1
	b	.LBB29_1348
.LBB29_1332:
	ldr	w2, [sp, #144]
.LBB29_1333:
	ldr	w4, [x21, #1192]
	adrp	x1, .L.str.43
	add	x1, x1, :lo12:.L.str.43
	mov	x0, x21
	bl	f2fs_printk
.LBB29_1334:
	mov	w20, #-22
	sxtw	x0, w20
	b	.LBB29_1455
.LBB29_1335:
	mov	x10, xzr
	mov	w9, wzr
	b	.LBB29_1348
.LBB29_1336:
	ldr	w12, [x21, #3128]
	cbz	w12, .LBB29_1344
// %bb.1337:
	ldr	x11, [x21]
	cmp	w12, #1                 // =1
	ldr	x11, [x11, #248]
	b.lt	.LBB29_1343
// %bb.1338:
	add	x13, x8, #80            // =80
	b	.LBB29_1340
.LBB29_1339:                            //   in Loop: Header=BB29_1340 Depth=1
	subs	x12, x12, #1            // =1
	add	x13, x13, #88           // =88
	b.eq	.LBB29_1343
.LBB29_1340:                            // =>This Inner Loop Header: Depth=1
	ldur	w14, [x13, #-4]
	cmp	w14, w9
	b.hi	.LBB29_1339
// %bb.1341:                            //   in Loop: Header=BB29_1340 Depth=1
	ldr	w14, [x13]
	cmp	w14, w9
	b.lo	.LBB29_1339
// %bb.1342:
	ldur	x11, [x13, #-80]
.LBB29_1343:
	cbz	x11, .LBB29_1331
.LBB29_1344:
	ldr	x11, [x21, #136]
	cbz	x11, .LBB29_1346
// %bb.1345:
	add	x12, x11, #80           // =80
	b	.LBB29_1347
.LBB29_1346:
	ldr	x12, [x21, #16]
	add	x12, x12, #72           // =72
.LBB29_1347:
	ldr	x11, [x11, #8]
	ldr	w12, [x12]
	ldr	w13, [x21, #1184]
	ldr	w11, [x11]
	sub	w9, w9, w12
	lsr	w9, w9, w13
	sub	w9, w9, w11
.LBB29_1348:
	mov	w11, #88
	madd	x10, x10, x11, x8
	ldr	w10, [x10, #80]
	add	w11, w10, #3            // =3
	cmp	w11, #4                 // =4
	b.hs	.LBB29_1350
.LBB29_1349:
	mov	w8, #-1
	b	.LBB29_1362
.LBB29_1350:
	ldr	w12, [x21, #3128]
	cbz	w12, .LBB29_1358
// %bb.1351:
	ldr	x11, [x21]
	cmp	w12, #1                 // =1
	ldr	x11, [x11, #248]
	b.lt	.LBB29_1357
// %bb.1352:
	add	x8, x8, #80             // =80
	b	.LBB29_1354
.LBB29_1353:                            //   in Loop: Header=BB29_1354 Depth=1
	subs	x12, x12, #1            // =1
	add	x8, x8, #88             // =88
	b.eq	.LBB29_1357
.LBB29_1354:                            // =>This Inner Loop Header: Depth=1
	ldur	w13, [x8, #-4]
	cmp	w13, w10
	b.hi	.LBB29_1353
// %bb.1355:                            //   in Loop: Header=BB29_1354 Depth=1
	ldr	w13, [x8]
	cmp	w13, w10
	b.lo	.LBB29_1353
// %bb.1356:
	ldur	x11, [x8, #-80]
.LBB29_1357:
	cbz	x11, .LBB29_1349
.LBB29_1358:
	ldr	x8, [x21, #136]
	cbz	x8, .LBB29_1360
// %bb.1359:
	add	x11, x8, #80            // =80
	b	.LBB29_1361
.LBB29_1360:
	ldr	x11, [x21, #16]
	add	x11, x11, #72           // =72
.LBB29_1361:
	ldr	x8, [x8, #8]
	ldr	w11, [x11]
	ldr	w12, [x21, #1184]
	ldr	w8, [x8]
	sub	w10, w10, w11
	lsr	w10, w10, w12
	sub	w8, w10, w8
.LBB29_1362:
	ldr	w10, [x24, #200]
	ldr	w11, [sp, #148]
	cmp	w10, w8
	ccmp	w10, w9, #0, lo
	csel	w22, w10, w9, hs
	add	w9, w22, w11
	cmp	w9, w8
	csel	w26, w9, w8, lo
	cmp	w22, w26
	b.hs	.LBB29_1369
// %bb.1363:
	add	x23, x21, #1664         // =1664
	add	w27, w26, #1            // =1
	b	.LBB29_1366
.LBB29_1364:                            //   in Loop: Header=BB29_1366 Depth=1
	mov	w20, wzr
.LBB29_1365:                            //   in Loop: Header=BB29_1366 Depth=1
	add	w22, w22, #1            // =1
	cmp	w22, w26
	b.hs	.LBB29_1371
.LBB29_1366:                            // =>This Inner Loop Header: Depth=1
	mov	x0, x23
	bl	down_write_trylock
	cbz	w0, .LBB29_1370
// %bb.1367:                            //   in Loop: Header=BB29_1366 Depth=1
	stp	w27, w27, [x24, #184]
	str	w27, [x24, #196]
	ldr	w8, [x25, #60]
	mov	w1, #1
	mov	w2, #1
	mov	w3, #1
	orr	w8, w8, #0x20000000
	mov	x0, x21
	mov	w4, w22
	str	w8, [x25, #60]
	bl	f2fs_gc
	ldr	w8, [x25, #60]
	cmn	w0, #11                 // =11
	and	w8, w8, #0xdfffffff
	str	w8, [x25, #60]
	b.eq	.LBB29_1364
// %bb.1368:                            //   in Loop: Header=BB29_1366 Depth=1
	mov	w20, w0
	tbz	w0, #31, .LBB29_1365
	b	.LBB29_1371
.LBB29_1369:
	mov	w20, wzr
	b	.LBB29_1371
.LBB29_1370:
	mov	w20, #-16
.LBB29_1371:
	mov	x0, x19
	bl	mnt_drop_write_file
.LBB29_1372:
	sxtw	x0, w20
	b	.LBB29_1455
.LBB29_1373:
	add	x8, sp, #144            // =144
	sub	x8, x8, x2
	add	x0, x8, #8              // =8
	mov	w1, wzr
	bl	memset
	mov	w20, #-14
	sxtw	x0, w20
	b	.LBB29_1455
.LBB29_1374:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1319
.LBB29_1375:
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
	cbnz	w9, .LBB29_1377
// %bb.1376:
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
.LBB29_1377:
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
.LBB29_1378:
	cbnz	x2, .LBB29_1401
// %bb.1379:
	ldr	w0, [sp, #120]
	bl	__fdget_pos
	ands	x20, x0, #0xfffffffffffffffc
	b.eq	.LBB29_1383
// %bb.1380:
	ldr	x24, [x20, #32]
	ldr	x8, [x21, #40]
	mov	x22, x0
	ldr	x9, [x24, #40]
	cmp	x8, x9
	b.eq	.LBB29_1384
// %bb.1381:
	ldr	x3, [x21, #64]
	ldr	x4, [x24, #64]
	adrp	x1, .L.str.82
	adrp	x2, .L__func__.f2fs_ioc_clone_file
	add	x1, x1, :lo12:.L.str.82
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_clone_file
	mov	x0, x26
	bl	f2fs_printk
	mov	w25, #-22
	tbz	w22, #1, .LBB29_1389
.LBB29_1382:
	mov	x0, x20
	bl	__f_unlock_pos
	tbnz	w22, #0, .LBB29_1390
	b	.LBB29_1391
.LBB29_1383:
	mov	w25, #-9
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_1384:
	ldr	x8, [x26, #16]
	ldrb	w8, [x8, #2181]
	tbz	w8, #7, .LBB29_1392
// %bb.1385:
	ldr	x8, [x24, #736]
	tbz	w8, #26, .LBB29_1392
// %bb.1386:
	ldrsw	x8, [x24, #1184]
	add	x8, x8, #360            // =360
	cmp	x8, #400                // =400
	b.lo	.LBB29_1392
// %bb.1387:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w25, w0
	cbz	w0, .LBB29_1393
// %bb.1388:
	tbnz	w22, #1, .LBB29_1382
.LBB29_1389:
	tbz	w22, #0, .LBB29_1391
.LBB29_1390:
	mov	x0, x20
	bl	fput
.LBB29_1391:
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_1392:
	mov	w25, #-95
	tbz	w22, #1, .LBB29_1389
	b	.LBB29_1382
.LBB29_1393:
	add	x0, x21, #160           // =160
	str	x0, [sp, #72]           // 8-byte Folded Spill
	bl	down_write
	cmp	x24, x21
	b.eq	.LBB29_1399
// %bb.1394:
	ldr	x8, [x24, #40]
	ldr	x9, [x21, #40]
	cmp	x8, x9
	b.ne	.LBB29_1399
// %bb.1395:
	mov	w2, #1
	mov	x0, x26
	mov	x1, x24
	bl	f2fs_inode_param_check
                                        // kill: def $w0 killed $w0 def $x0
	mov	x25, x0
	cbnz	w0, .LBB29_1400
// %bb.1396:
	mov	w2, #2
	mov	x0, x26
	mov	x1, x21
	bl	f2fs_inode_param_check
                                        // kill: def $w0 killed $w0 def $x0
	mov	x25, x0
	cbnz	w0, .LBB29_1400
// %bb.1397:
	ldr	x8, [x24, #40]
	mov	x0, x21
	ldr	x8, [x8, #1008]
	str	x8, [sp]                // 8-byte Folded Spill
	bl	f2fs_convert_inline_inode
                                        // kill: def $w0 killed $w0 def $x0
	cbz	w0, .LBB29_1436
// %bb.1398:
	mov	x25, x0
	ldr	x2, [x21, #64]
	ldr	x0, [sp]                // 8-byte Folded Reload
	adrp	x1, .L.str.83
	add	x1, x1, :lo12:.L.str.83
	mov	w3, w25
	bl	f2fs_printk
	b	.LBB29_1400
.LBB29_1399:
	ldr	x3, [x24, #64]
	ldr	x4, [x21, #64]
	adrp	x1, .L.str.60
	adrp	x2, .L__func__.f2fs_dedup_param_check
	add	x1, x1, :lo12:.L.str.60
	add	x2, x2, :lo12:.L__func__.f2fs_dedup_param_check
	mov	x0, x26
	bl	f2fs_printk
	mov	w25, #-22
.LBB29_1400:
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
	tbz	w22, #1, .LBB29_1389
	b	.LBB29_1382
.LBB29_1401:
	add	x8, sp, #120            // =120
	sub	x8, x8, x2
	add	x0, x8, #8              // =8
	mov	w1, wzr
	bl	memset
	b	.LBB29_1555
.LBB29_1402:
	mov	w2, #2
	b	.LBB29_1433
.LBB29_1403:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbz	w9, #4, .LBB29_1404
	b	.LBB29_211
.LBB29_1404:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1406
// %bb.1405:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1406:
	ldr	x10, [x8, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1407:
	lsl	x9, x20, #8
	ldr	x8, [x8, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #144            // =144
	mov	w2, #2
	bl	__arch_copy_from_user
	ldr	w8, [x23, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1410
// %bb.1408:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1411
.LBB29_1409:
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
	b	.LBB29_1414
.LBB29_1410:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1409
.LBB29_1411:
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
	cbnz	w9, .LBB29_1413
// %bb.1412:
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
.LBB29_1413:
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
.LBB29_1414:
	cbnz	x2, .LBB29_1433
// %bb.1415:
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_1417
// %bb.1416:
	ldr	x8, [x22, #736]
	tbnz	w8, #31, .LBB29_1418
.LBB29_1417:
	mov	w23, #-22
	sxtw	x0, w23
	b	.LBB29_1455
.LBB29_1418:
	ldrb	w8, [sp, #145]
	mov	w23, #-22
	sub	w8, w8, #3              // =3
	cmp	w8, #5                  // =5
	b.hi	.LBB29_1454
// %bb.1419:
	ldrb	w8, [sp, #144]
	cmp	w8, #3                  // =3
	b.hi	.LBB29_1454
// %bb.1420:
	ldr	x8, [x19, #32]
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB29_1422
// %bb.1421:
	ldr	x0, [x8, #40]
	mov	w1, #1
	mov	w2, #1
	bl	__sb_start_write
.LBB29_1422:
	add	x20, x22, #160          // =160
	mov	x0, x20
	bl	down_write
	ldr	x8, [x22, #736]
	tbnz	x8, #33, .LBB29_1424
// %bb.1423:
	ldr	w8, [x22, #792]
	cbz	w8, .LBB29_1428
.LBB29_1424:
	mov	w23, #-16
.LBB29_1425:
	mov	x0, x20
.LBB29_1426:
	bl	up_write
	ldr	x8, [x19, #32]
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB29_1454
// %bb.1427:
	ldr	x0, [x8, #40]
	mov	w1, #1
	bl	__sb_end_write
	b	.LBB29_1454
.LBB29_1428:
	ldr	x8, [x22, #80]
	cbz	x8, .LBB29_1430
// %bb.1429:
	mov	w23, #-27
	b	.LBB29_1425
.LBB29_1430:
	ldrb	w8, [sp, #144]
	mov	w9, #1
	mov	w1, #1
	mov	x0, x22
	strb	w8, [x22, #1284]
	ldrb	w8, [sp, #145]
	strb	w8, [x22, #1285]
	lsl	w8, w9, w8
	str	w8, [x22, #1292]
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x22
	bl	f2fs_is_compress_backend_ready
	tbnz	w0, #0, .LBB29_1432
// %bb.1431:
	adrp	x1, .L.str.54
	add	x1, x1, :lo12:.L.str.54
	mov	x0, x21
	bl	f2fs_printk
.LBB29_1432:
	mov	w23, wzr
	b	.LBB29_1425
.LBB29_1433:
	add	x8, sp, #144            // =144
	sub	x8, x8, x2
	add	x0, x8, #2              // =2
	mov	w1, wzr
	bl	memset
	mov	w23, #-14
	sxtw	x0, w23
	b	.LBB29_1455
.LBB29_1434:
	mov	w23, wzr
	b	.LBB29_860
.LBB29_1435:
	mov	w24, #-75
	b	.LBB29_973
.LBB29_1436:
	ldr	w8, [sp, #124]
	tbz	w8, #5, .LBB29_1438
// %bb.1437:
	ldr	w8, [x24, #4]
	str	w8, [x21, #4]
	ldr	w8, [x24, #8]
	str	w8, [x21, #8]
	ldr	x8, [x24, #80]
	str	x8, [x21, #80]
	ldr	w8, [sp, #124]
.LBB29_1438:
	tbnz	w8, #6, .LBB29_1478
.LBB29_1439:
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 17592186044416
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	ldr	x8, [x21, #704]
	mov	x0, x21
	orr	x8, x8, #0x10
	str	x8, [x21, #704]
	bl	f2fs_set_inode_flags
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	mov	w25, wzr
	b	.LBB29_1400
.LBB29_1440:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbz	w8, #4, .LBB29_1441
	b	.LBB29_529
.LBB29_1441:
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
	and	w10, w8, #0x00000080
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
	eor	w10, w8, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1443
// %bb.1442:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1443:
	ldr	x10, [x9, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
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
.LBB29_1444:
	ldr	x8, [x9, #8]
	lsl	x9, x20, #8
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x0, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	mov	w2, #16
	bl	__arch_copy_to_user
	ldr	w8, [x22, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1447
// %bb.1445:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1448
.LBB29_1446:
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
	b	.LBB29_1451
.LBB29_1447:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1446
.LBB29_1448:
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
	cbnz	w9, .LBB29_1450
// %bb.1449:
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
.LBB29_1450:
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
.LBB29_1451:
	cmp	x0, #0                  // =0
	mov	w8, #-14
	csel	w23, wzr, w8, eq
.LBB29_1452:
	mov	x0, x21
.LBB29_1453:
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
.LBB29_1454:
	sxtw	x0, w23
.LBB29_1455:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB29_1457
// %bb.1456:
	ldp	x20, x19, [sp, #368]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #352]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #336]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #320]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #304]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #288]    // 16-byte Folded Reload
	add	sp, sp, #384            // =384
	ret
.LBB29_1457:
	bl	__stack_chk_fail
.LBB29_1458:
	cbnz	w23, .LBB29_1460
.LBB29_1459:
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w23, w0
	cbz	w0, .LBB29_860
.LBB29_1460:
	ldr	x3, [x21, #64]
	adrp	x1, .L.str.53
	adrp	x2, .L__func__.f2fs_ioc_reserve_decompress_file
	add	x1, x1, :lo12:.L.str.53
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_reserve_decompress_file
	mov	x0, x20
	mov	w4, w23
	bl	f2fs_printk
	b	.LBB29_860
.LBB29_1461:
	add	x10, x22, #736          // =736
	str	x10, [sp, #56]          // 8-byte Folded Spill
	//APP
	// atomic64_or
	prfm	pstl1strm, [x10]
1:	ldxr	x8, [x10]
	orr	x8, x8, 17179869184
	stxr	w9, x8, [x10]
	cbnz	w9, 1b
	//NO_APP
	add	x24, x22, #992          // =992
	mov	x0, x24
	bl	down_write
	mov	x0, x22
	bl	inode_dio_wait
	mov	x0, x24
	bl	up_write
	ldr	x8, [x22, #80]
	add	x8, x8, #4095           // =4095
	lsr	x8, x8, #12
	str	x8, [sp, #64]           // 8-byte Folded Spill
	cbz	w8, .LBB29_1609
// %bb.1462:
	ldr	x8, [sp, #64]           // 8-byte Folded Reload
	mov	x24, xzr
	mov	w23, w8
	b	.LBB29_1464
.LBB29_1463:                            //   in Loop: Header=BB29_1464 Depth=1
	subs	w23, w23, w26
	add	x24, x24, w26, sxtw
	b.eq	.LBB29_1607
.LBB29_1464:                            // =>This Inner Loop Header: Depth=1
	cmp	w28, w23
	csel	w26, w28, w23, lt
	mov	x0, x22
	mov	x1, x24
	mov	w2, w26
	bl	redirty_blocks
	mov	w25, w0
	tbnz	w0, #31, .LBB29_1608
// %bb.1465:                            //   in Loop: Header=BB29_1464 Depth=1
	ldr	w8, [x22, #792]
	cmp	w8, w27
	b.lo	.LBB29_1463
// %bb.1466:                            //   in Loop: Header=BB29_1464 Depth=1
	ldr	x0, [x22, #48]
	bl	filemap_fdatawrite
	b	.LBB29_1463
.LBB29_1467:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbz	w9, #4, .LBB29_1468
	b	.LBB29_653
.LBB29_1468:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1470
// %bb.1469:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1470:
	ldr	x10, [x8, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1471:
	lsl	x9, x20, #8
	ldr	x8, [x8, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x0, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x1, sp, #144            // =144
	mov	w2, #8
	bl	__arch_copy_to_user
	ldr	w8, [x19, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1474
// %bb.1472:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1475
.LBB29_1473:
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
	b	.LBB29_1067
.LBB29_1474:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1473
.LBB29_1475:
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
	cbnz	w9, .LBB29_1477
// %bb.1476:
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
.LBB29_1477:
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
	b	.LBB29_1067
.LBB29_1478:
	ldr	x8, [x24, #80]
	mov	w1, #1
	str	x8, [x21, #80]
	ldr	x8, [x24, #40]
	ldr	x0, [x8, #1008]
	ldr	w8, [x0, #1188]
	str	w8, [sp, #12]           // 4-byte Folded Spill
	bl	f2fs_balance_fs
	ldr	x8, [x24, #80]
	add	x8, x8, #4095           // =4095
	lsr	x8, x8, #12
	str	x8, [sp, #40]           // 8-byte Folded Spill
	cbz	w8, .LBB29_1516
// %bb.1479:
	str	xzr, [sp, #48]          // 8-byte Folded Spill
.LBB29_1480:                            // =>This Loop Header: Depth=1
                                        //     Child Loop BB29_1483 Depth 2
                                        //     Child Loop BB29_1491 Depth 2
                                        //     Child Loop BB29_1507 Depth 2
	ldr	x8, [x21, #40]
	ldp	x9, x2, [sp, #40]       // 16-byte Folded Reload
	mov	x1, xzr
	mov	x4, xzr
	ldr	x8, [x8, #1008]
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	str	x8, [sp, #56]           // 8-byte Folded Spill
	add	x8, sp, #88             // =88
	stp	x8, x8, [sp, #88]
	ldr	x8, [x21, #48]
	ldr	x0, [x24, #48]
	str	x8, [sp, #24]           // 8-byte Folded Spill
	ldr	w8, [sp, #12]           // 4-byte Folded Reload
	str	x0, [sp, #64]           // 8-byte Folded Spill
	cmp	w8, w9
	csel	w23, w8, w9, lt
	sxtw	x3, w23
	str	x3, [sp, #16]           // 8-byte Folded Spill
	bl	__do_page_cache_readahead
	cmp	w23, #1                 // =1
	str	x23, [sp, #32]          // 8-byte Folded Spill
	b.lt	.LBB29_1503
// %bb.1481:                            //   in Loop: Header=BB29_1480 Depth=1
	ldr	x8, [sp, #32]           // 8-byte Folded Reload
	ldr	x26, [sp, #48]          // 8-byte Folded Reload
	mov	w27, w8
	b	.LBB29_1483
.LBB29_1482:                            //   in Loop: Header=BB29_1483 Depth=2
	subs	w27, w27, #1            // =1
	add	x26, x26, #1            // =1
	b.eq	.LBB29_1488
.LBB29_1483:                            //   Parent Loop BB29_1480 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	mov	x1, x26
	mov	x2, xzr
	mov	x3, xzr
	bl	read_cache_page
	mov	x25, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB29_1507
// %bb.1484:                            //   in Loop: Header=BB29_1483 Depth=2
	adrp	x8, page_info_slab
	ldr	x23, [x8, :lo12:page_info_slab]
	mov	w1, #3136
	mov	x0, x23
	bl	kmem_cache_alloc
	mov	x28, x0
	cbnz	x0, .LBB29_1486
// %bb.1485:                            //   in Loop: Header=BB29_1483 Depth=2
	mov	w1, #35904
	mov	x0, x23
	bl	kmem_cache_alloc
	mov	x28, x0
.LBB29_1486:                            //   in Loop: Header=BB29_1483 Depth=2
	stp	x28, x25, [x28, #8]
	str	x28, [x28]
	ldr	x23, [sp, #96]
	add	x2, sp, #88             // =88
	mov	x0, x28
	mov	x1, x23
	bl	__list_add_valid
	tbz	w0, #0, .LBB29_1482
// %bb.1487:                            //   in Loop: Header=BB29_1483 Depth=2
	add	x8, sp, #88             // =88
	str	x28, [sp, #96]
	stp	x8, x23, [x28]
	str	x28, [x23]
	b	.LBB29_1482
.LBB29_1488:                            //   in Loop: Header=BB29_1480 Depth=1
	ldr	x8, [sp, #32]           // 8-byte Folded Reload
	ldr	x26, [sp, #48]          // 8-byte Folded Reload
	mov	w23, w8
	b	.LBB29_1491
.LBB29_1489:                            //   in Loop: Header=BB29_1491 Depth=2
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	add	x8, x28, x9
	add	x9, x0, x9
	lsl	x8, x8, #6
	lsl	x9, x9, #6
	and	x8, x8, #0xfffffffffffff000
	and	x9, x9, #0xfffffffffffff000
	mov	x10, #-549755813888
	mov	x25, x0
	add	x1, x8, x10
	add	x0, x9, x10
	mov	w2, #4096
	bl	memcpy
	mov	x0, x25
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x25
	bl	f2fs_put_page
.LBB29_1490:                            //   in Loop: Header=BB29_1491 Depth=2
	mov	w1, #1
	mov	x0, x28
	bl	f2fs_put_page
	subs	w23, w23, #1            // =1
	add	x26, x26, #1            // =1
	b.eq	.LBB29_1503
.LBB29_1491:                            //   Parent Loop BB29_1480 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	mov	w2, #2
	mov	x1, x26
	mov	w3, wzr
	bl	pagecache_get_page
	cbz	x0, .LBB29_1504
// %bb.1492:                            //   in Loop: Header=BB29_1491 Depth=2
	mov	x28, x0
	mov	x0, x24
	mov	x1, x26
	bl	f2fs_is_hole_blkaddr
	tbnz	w0, #0, .LBB29_1490
// %bb.1493:                            //   in Loop: Header=BB29_1491 Depth=2
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	mov	w1, #5
	mov	w2, #1
	mov	w3, wzr
	bl	f2fs_do_map_lock
	mov	w27, w0
	add	x0, sp, #144            // =144
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	add	x0, sp, #144            // =144
	mov	x1, x26
	stp	x21, xzr, [sp, #144]
	str	xzr, [sp, #160]
	str	wzr, [sp, #168]
	bl	f2fs_get_block
	ldr	x8, [sp, #160]
	mov	w25, w0
	cbz	x8, .LBB29_1495
// %bb.1494:                            //   in Loop: Header=BB29_1491 Depth=2
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_put_page
	ldr	x9, [sp, #160]
	b	.LBB29_1496
.LBB29_1495:                            //   in Loop: Header=BB29_1491 Depth=2
	mov	x9, xzr
.LBB29_1496:                            //   in Loop: Header=BB29_1491 Depth=2
	ldr	x8, [sp, #152]
	cbz	x8, .LBB29_1500
// %bb.1497:                            //   in Loop: Header=BB29_1491 Depth=2
	cmp	x9, x8
	b.eq	.LBB29_1500
// %bb.1498:                            //   in Loop: Header=BB29_1491 Depth=2
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
	cbnz	w9, .LBB29_1500
// %bb.1499:                            //   in Loop: Header=BB29_1491 Depth=2
	bl	__put_page
.LBB29_1500:                            //   in Loop: Header=BB29_1491 Depth=2
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	and	w3, w27, #0x1
	mov	w1, #5
	mov	w2, wzr
	stp	xzr, xzr, [sp, #152]
	bl	f2fs_do_map_lock
	cbnz	w25, .LBB29_1505
// %bb.1501:                            //   in Loop: Header=BB29_1491 Depth=2
	ldr	x0, [sp, #24]           // 8-byte Folded Reload
	mov	w2, #2
	mov	x1, x26
	bl	grab_cache_page_write_begin
	cbnz	x0, .LBB29_1489
// %bb.1502:                            //   in Loop: Header=BB29_1480 Depth=1
	mov	w1, #1
	mov	x0, x28
	bl	f2fs_put_page
	mov	w25, #-12
	b	.LBB29_1507
.LBB29_1503:                            //   in Loop: Header=BB29_1480 Depth=1
	mov	w25, wzr
	b	.LBB29_1507
.LBB29_1504:                            //   in Loop: Header=BB29_1480 Depth=1
	mov	w25, #-12
	b	.LBB29_1507
.LBB29_1505:                            //   in Loop: Header=BB29_1480 Depth=1
	mov	w1, #1
	mov	x0, x28
	bl	f2fs_put_page
	b	.LBB29_1507
.LBB29_1506:                            //   in Loop: Header=BB29_1507 Depth=2
	mov	x9, #256
	movk	x9, #57005, lsl #48
	add	x8, x9, #34             // =34
	stp	x9, x8, [x26]
	adrp	x8, page_info_slab
	ldr	x0, [x8, :lo12:page_info_slab]
	mov	x1, x26
	bl	kmem_cache_free
.LBB29_1507:                            //   Parent Loop BB29_1480 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldr	x26, [sp, #88]
	add	x8, sp, #88             // =88
	cmp	x8, x26
	b.eq	.LBB29_1513
// %bb.1508:                            //   in Loop: Header=BB29_1507 Depth=2
	ldr	x8, [x26, #16]
	cbz	x8, .LBB29_1511
// %bb.1509:                            //   in Loop: Header=BB29_1507 Depth=2
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
	cbnz	w9, .LBB29_1511
// %bb.1510:                            //   in Loop: Header=BB29_1507 Depth=2
	bl	__put_page
.LBB29_1511:                            //   in Loop: Header=BB29_1507 Depth=2
	mov	x0, x26
	bl	__list_del_entry_valid
	tbz	w0, #0, .LBB29_1506
// %bb.1512:                            //   in Loop: Header=BB29_1507 Depth=2
	ldp	x9, x8, [x26]
	str	x8, [x9, #8]
	str	x9, [x8]
	b	.LBB29_1506
.LBB29_1513:                            //   in Loop: Header=BB29_1480 Depth=1
	tbnz	w25, #31, .LBB29_1517
// %bb.1514:                            //   in Loop: Header=BB29_1480 Depth=1
	ldr	x0, [x21, #48]
	bl	filemap_fdatawrite
	ldp	x9, x8, [sp, #32]       // 16-byte Folded Reload
	subs	w8, w8, w9
	str	x8, [sp, #40]           // 8-byte Folded Spill
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	ldr	x9, [sp, #16]           // 8-byte Folded Reload
	add	x8, x8, x9
	str	x8, [sp, #48]           // 8-byte Folded Spill
	b.ne	.LBB29_1480
// %bb.1515:
	cbnz	w25, .LBB29_1517
.LBB29_1516:
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
                                        // kill: def $w0 killed $w0 def $x0
	mov	x25, x0
	cbz	w0, .LBB29_1439
.LBB29_1517:
	ldr	x2, [x24, #64]
	ldr	x3, [x21, #64]
	ldr	x0, [sp]                // 8-byte Folded Reload
	adrp	x1, .L.str.84
	add	x1, x1, :lo12:.L.str.84
	mov	w4, w25
	bl	f2fs_printk
	b	.LBB29_1400
.LBB29_1518:
	mov	w2, #16
	b	.LBB29_1569
.LBB29_1519:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbz	w8, #4, .LBB29_1520
	b	.LBB29_375
.LBB29_1520:
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
	cbnz	w9, .LBB29_1522
// %bb.1521:
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
.LBB29_1522:
	ldr	x9, [x26, #16]
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	mov	x11, x9
	bfxil	x11, x10, #0, #48
	//APP
	msr ttbr1_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
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
.LBB29_1523:
	lsl	x9, x20, #8
	ldr	x8, [x26, #8]
	and	x28, x20, x9, asr #8
	//APP
		bics	xzr, x28, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #88             // =88
	mov	w2, #16
	bl	__arch_copy_from_user
	ldr	w8, [x27, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1526
// %bb.1524:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1527
.LBB29_1525:
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
	b	.LBB29_1530
.LBB29_1526:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1525
.LBB29_1527:
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
	cbnz	w9, .LBB29_1529
// %bb.1528:
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
.LBB29_1529:
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
.LBB29_1530:
	cbnz	x2, .LBB29_1569
// %bb.1531:
	ldr	x8, [sp, #88]
	tst	x8, #0xfff
	b.ne	.LBB29_493
// %bb.1532:
	ldr	x9, [sp, #96]
	tst	x9, #0xfff
	b.ne	.LBB29_493
// %bb.1533:
	mov	x0, x22
	add	x23, x9, x8
	bl	max_file_blocks
	cmp	x0, x23, lsr #12
	b.lo	.LBB29_493
// %bb.1534:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w25, w0
	cbnz	w0, .LBB29_1391
// %bb.1535:
	mov	w8, #8
	ldr	x22, [x19, #32]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	str	wzr, [sp, #128]
	str	xzr, [sp, #120]
	str	x8, [sp, #176]
	str	xzr, [sp, #80]
	ldr	w8, [x21, #1188]
	mov	x0, x22
	mov	x1, xzr
	str	w8, [sp, #72]           // 4-byte Folded Spill
	bl	f2fs_should_update_inplace
	tbz	w0, #0, .LBB29_1537
// %bb.1536:
	mov	w25, #-22
	b	.LBB29_1547
.LBB29_1537:
	ldp	x23, x24, [sp, #88]
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_balance_fs
	add	x0, x22, #160           // =160
	str	x0, [sp, #64]           // 8-byte Folded Spill
	bl	down_write
	ldr	x8, [x22, #736]
	tbz	x8, #35, .LBB29_1539
// %bb.1538:
	mov	w1, #4
	mov	x0, x22
	bl	f2fs_reserve_compress_blocks_internal
	mov	w25, w0
	cbnz	w0, .LBB29_1546
.LBB29_1539:
	ldr	x8, [x22, #736]
	tbz	x8, #44, .LBB29_1541
// %bb.1540:
	add	x8, x22, #736           // =736
	mov	x9, #17592186044416
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_mark_inode_dirty_sync
.LBB29_1541:
	ldr	x8, [x22, #736]
	tbz	x8, #38, .LBB29_1545
// %bb.1542:
	ldr	x8, [x22, #736]
	tbz	x8, #38, .LBB29_1544
// %bb.1543:
	ldr	x8, [x22, #736]
	tbnz	x8, #39, .LBB29_1545
.LBB29_1544:
	adrp	x1, .L__func__.f2fs_defragment_range
	add	x1, x1, :lo12:.L__func__.f2fs_defragment_range
	mov	x0, x22
	bl	f2fs_revoke_deduped_inode
	mov	w25, w0
	cbnz	w0, .LBB29_1546
.LBB29_1545:
	ldp	x1, x8, [sp, #88]
	ldr	x0, [x22, #48]
	add	x8, x1, x8
	sub	x2, x8, #1              // =1
	bl	filemap_write_and_wait_range
	mov	w25, w0
	cbz	w0, .LBB29_1556
.LBB29_1546:
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_write
.LBB29_1547:
	mov	x0, x19
	bl	mnt_drop_write_file
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	stp	x8, x8, [x21, #504]
	str	x8, [x21, #520]
	tbnz	w25, #31, .LBB29_1391
// %bb.1548:
	ldrb	w9, [x26, #62]
	ldr	x8, [x26, #8]
	tbnz	w9, #5, .LBB29_1550
// %bb.1549:
	ldr	x10, [x26]
	mov	x9, x20
	tbz	w10, #26, .LBB29_1551
.LBB29_1550:
	mov	x9, x28
.LBB29_1551:
	//APP
		adds	x9, x9, 16
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1555
// %bb.1552:
	ldr	w8, [x27, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1911
// %bb.1553:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1912
.LBB29_1554:
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
	b	.LBB29_1915
.LBB29_1555:
	mov	w25, #-14
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_1556:
	add	x8, x24, x23
	lsr	x24, x23, #12
	add	x2, sp, #120            // =120
	mov	x0, x22
	mov	x1, x24
	lsr	x23, x8, #12
	bl	f2fs_lookup_extent_cache
	tbz	w0, #0, .LBB29_1558
// %bb.1557:
	ldp	w8, w9, [sp, #120]
	add	w8, w9, w8
	cmp	x23, x8
	b.ls	.LBB29_1567
.LBB29_1558:
	add	x8, sp, #80             // =80
	cmp	x23, w24, uxtw
	str	w24, [sp, #148]
	str	x8, [sp, #160]
	b.ls	.LBB29_1567
// %bb.1559:
	mov	w8, w24
	str	wzr, [sp, #56]          // 4-byte Folded Spill
	str	wzr, [sp, #40]          // 4-byte Folded Spill
	str	wzr, [sp, #48]          // 4-byte Folded Spill
.LBB29_1560:                            // =>This Inner Loop Header: Depth=1
	sub	w8, w23, w8
	add	x1, sp, #144            // =144
	mov	x0, x22
	mov	w2, wzr
	mov	w3, wzr
	str	w8, [sp, #152]
	bl	f2fs_map_blocks
	cbnz	w0, .LBB29_1570
// %bb.1561:                            //   in Loop: Header=BB29_1560 Depth=1
	ldrh	w8, [sp, #156]
	mov	w9, #4192
	tst	w8, w9
	b.ne	.LBB29_1563
// %bb.1562:                            //   in Loop: Header=BB29_1560 Depth=1
	ldr	x8, [sp, #80]
	cmp	x23, w8, uxtw
	str	w8, [sp, #148]
	b.hi	.LBB29_1560
	b	.LBB29_1564
.LBB29_1563:                            //   in Loop: Header=BB29_1560 Depth=1
	ldr	w12, [sp, #40]          // 4-byte Folded Reload
	ldp	w8, w10, [sp, #144]
	ldr	w9, [sp, #152]
	cmp	w12, #0                 // =0
	cset	w11, ne
	cmp	w12, w8
	ldr	w12, [sp, #48]          // 4-byte Folded Reload
	add	w8, w9, w8
	str	w8, [sp, #40]           // 4-byte Folded Spill
	add	w8, w10, w9
	ldr	w10, [sp, #56]          // 4-byte Folded Reload
	add	w12, w9, w12
	cset	w9, ne
	and	w9, w11, w9
	orr	w10, w10, w9
	str	w12, [sp, #48]          // 4-byte Folded Spill
	cmp	x23, x8
	str	w10, [sp, #56]          // 4-byte Folded Spill
	str	w8, [sp, #148]
	b.hi	.LBB29_1560
.LBB29_1564:
	ldr	w8, [sp, #56]           // 4-byte Folded Reload
	tbz	w8, #0, .LBB29_1567
// %bb.1565:
	ldr	w8, [x21, #1192]
	ldr	w9, [x21, #1188]
	mov	x0, x21
	mul	w8, w9, w8
	ldr	w9, [sp, #48]           // 4-byte Folded Reload
	add	w9, w9, w8
	sub	w9, w9, #1              // =1
	udiv	w1, w9, w8
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB29_1706
// %bb.1566:
	mov	w25, #-11
	b	.LBB29_1546
.LBB29_1567:
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_write
	str	wzr, [sp, #56]          // 4-byte Folded Spill
.LBB29_1568:
	ldr	w8, [sp, #56]           // 4-byte Folded Reload
	mov	w25, wzr
	mov	w8, w8
	lsl	x8, x8, #12
	str	x8, [sp, #96]
	b	.LBB29_1547
.LBB29_1569:
	add	x8, sp, #88             // =88
	sub	x8, x8, x2
	add	x0, x8, #16             // =16
	mov	w1, wzr
	bl	memset
	mov	w25, #-14
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_1570:
	mov	w25, w0
	b	.LBB29_1546
.LBB29_1571:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbz	w12, #4, .LBB29_1572
	b	.LBB29_609
.LBB29_1572:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_1574
// %bb.1573:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1574:
	ldr	x9, [x9, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x9
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1575:
	mov	w9, wzr
	//APP
	1:.if 1 == 1
661:
	str     x8, [x10]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr x8, [x10]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w9, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w8, [x11, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1578
// %bb.1576:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1579
.LBB29_1577:
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
	sxtw	x0, w9
	b	.LBB29_1455
.LBB29_1578:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1577
.LBB29_1579:
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
	and	w10, w8, #0x00000080
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
	eor	w10, w8, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1581
// %bb.1580:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1581:
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	and	x10, x10, #0xffffffffffff
	sub	x11, x10, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x10
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
	sxtw	x0, w9
	b	.LBB29_1455
.LBB29_1582:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbz	w9, #4, .LBB29_1583
	b	.LBB29_641
.LBB29_1583:
	//APP
	.if 1 == 1
661:
	mrs	x9, daif
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
	mrs_s x9, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w10, w9, #0x00000080
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
	eor	w10, w9, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w10, .LBB29_1585
// %bb.1584:
	mov	w10, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x10
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1585:
	ldr	x10, [x8, #16]
	//APP
	mrs x11, ttbr1_el1
	//NO_APP
	mov	x12, x10
	bfxil	x12, x11, #0, #48
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x9
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x9
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1586:
	lsl	x9, x20, #8
	ldr	x8, [x8, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x0, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x1, sp, #144            // =144
	mov	w2, #2
	bl	__arch_copy_to_user
	ldr	w8, [x19, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1589
// %bb.1587:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1590
.LBB29_1588:
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
	b	.LBB29_1067
.LBB29_1589:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1588
.LBB29_1590:
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
	cbnz	w9, .LBB29_1592
// %bb.1591:
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
.LBB29_1592:
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
	b	.LBB29_1067
.LBB29_1593:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1175
.LBB29_1594:
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
	cbnz	w9, .LBB29_1596
// %bb.1595:
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
.LBB29_1596:
	ldr	x9, [x23, #16]
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	mov	x11, x9
	bfxil	x11, x10, #0, #48
	//APP
	msr ttbr1_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
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
.LBB29_1597:
	ldr	x8, [x23, #8]
	//APP
		bics	xzr, x27, x8
	csel	x0, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x1, sp, #144            // =144
	mov	w2, #24
	bl	__arch_copy_to_user
	ldr	w8, [x25, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1600
// %bb.1598:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1601
.LBB29_1599:
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
	b	.LBB29_1604
.LBB29_1600:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1599
.LBB29_1601:
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
	cbnz	w9, .LBB29_1603
// %bb.1602:
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
.LBB29_1603:
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
.LBB29_1604:
	cbz	x0, .LBB29_1606
.LBB29_1605:
	mov	w21, #-14
	sxtw	x0, w21
	b	.LBB29_1455
.LBB29_1606:
	ldr	x8, [x22, #40]
	adrp	x9, jiffies
	mov	w21, wzr
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	b	.LBB29_1166
.LBB29_1607:
	cbz	w25, .LBB29_1609
.LBB29_1608:
	add	x8, x22, #736           // =736
	mov	x9, #17179869184
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	b	.LBB29_1610
.LBB29_1609:
	ldr	x0, [x22, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	x8, #17179869184
	ldr	x11, [sp, #56]          // 8-byte Folded Reload
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x11]
1:	ldxr	x9, [x11]
	bic	x9, x9, x8
	stxr	w10, x9, [x11]
	cbnz	w10, 1b
	//NO_APP
	mov	w25, w0
	cbz	w0, .LBB29_1719
.LBB29_1610:
	adrp	x1, .L.str.51
	adrp	x2, .L__func__.f2fs_ioc_compress_release_file
	add	x1, x1, :lo12:.L.str.51
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_compress_release_file
	mov	x0, x21
	mov	w3, w25
	bl	f2fs_printk
.LBB29_1611:
	mov	w27, wzr
	mov	w24, wzr
	b	.LBB29_1113
.LBB29_1612:
	mov	w2, #4
	b	.LBB29_1665
.LBB29_1613:
	mov	w2, #8
	b	.LBB29_1700
.LBB29_1614:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbz	w8, #4, .LBB29_1615
	b	.LBB29_455
.LBB29_1615:
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
	cbnz	w9, .LBB29_1617
// %bb.1616:
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
.LBB29_1617:
	ldr	x9, [x23, #16]
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	mov	x11, x9
	bfxil	x11, x10, #0, #48
	//APP
	msr ttbr1_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
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
.LBB29_1618:
	lsl	x9, x20, #8
	ldr	x8, [x23, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #88             // =88
	mov	w2, #4
	bl	__arch_copy_from_user
	ldr	w8, [x22, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1628
// %bb.1619:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1629
.LBB29_1620:
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
	b	.LBB29_1632
.LBB29_1621:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbz	w8, #4, .LBB29_1622
	b	.LBB29_471
.LBB29_1622:
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
	cbnz	w9, .LBB29_1624
// %bb.1623:
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
.LBB29_1624:
	ldr	x9, [x24, #16]
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	mov	x11, x9
	bfxil	x11, x10, #0, #48
	//APP
	msr ttbr1_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
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
.LBB29_1625:
	lsl	x9, x20, #8
	ldr	x8, [x24, #8]
	and	x9, x20, x9, asr #8
	//APP
		bics	xzr, x9, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #88             // =88
	mov	w2, #8
	bl	__arch_copy_from_user
	ldr	w8, [x21, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1667
// %bb.1626:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1668
.LBB29_1627:
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
	b	.LBB29_1671
.LBB29_1628:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1620
.LBB29_1629:
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
	cbnz	w9, .LBB29_1631
// %bb.1630:
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
.LBB29_1631:
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
.LBB29_1632:
	cbnz	x2, .LBB29_1665
// %bb.1633:
	ldr	w0, [sp, #88]
	bl	__fdget_pos
	ands	x20, x0, #0xfffffffffffffffc
	b.eq	.LBB29_1639
// %bb.1634:
	mov	x22, x0
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w27, w0
	cbz	w0, .LBB29_1640
// %bb.1635:
	tbnz	w22, #1, .LBB29_1648
.LBB29_1636:
	tbz	w22, #0, .LBB29_1638
.LBB29_1637:
	mov	x0, x20
	bl	fput
.LBB29_1638:
	sxtw	x0, w27
	b	.LBB29_1455
.LBB29_1639:
	mov	w27, #-9
	sxtw	x0, w27
	b	.LBB29_1455
.LBB29_1640:
	add	x24, x21, #160          // =160
	mov	x0, x24
	bl	down_write
	ldr	x8, [x21, #736]
	tbnz	x8, #38, .LBB29_1644
// %bb.1641:
	ldr	x25, [x20, #32]
	ldr	x8, [x21, #40]
	ldr	x9, [x25, #40]
	cmp	x8, x9
	b.eq	.LBB29_1649
// %bb.1642:
	ldr	x3, [x21, #64]
	ldr	x4, [x25, #64]
	adrp	x1, .L.str.57
	adrp	x2, .L__func__.f2fs_ioc_create_layered_inode
	add	x1, x1, :lo12:.L.str.57
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_create_layered_inode
	mov	x0, x26
.LBB29_1643:
	bl	f2fs_printk
	b	.LBB29_1646
.LBB29_1644:
	ldr	x2, [x21, #64]
	adrp	x1, .L.str.56
	add	x1, x1, :lo12:.L.str.56
.LBB29_1645:
	mov	x0, x26
	bl	f2fs_printk
.LBB29_1646:
	mov	w27, #-22
.LBB29_1647:
	mov	x0, x24
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
	tbz	w22, #1, .LBB29_1636
.LBB29_1648:
	mov	x0, x20
	bl	__f_unlock_pos
	tbnz	w22, #0, .LBB29_1637
	b	.LBB29_1638
.LBB29_1649:
	ldrb	w8, [x25, #12]
	tbnz	w8, #3, .LBB29_1651
// %bb.1650:
	ldr	x2, [x25, #64]
	adrp	x1, .L.str.58
	add	x1, x1, :lo12:.L.str.58
	b	.LBB29_1645
.LBB29_1651:
	cmp	x21, x25
	b.eq	.LBB29_1664
// %bb.1652:
	mov	w2, #1
	mov	x0, x26
	mov	x1, x21
	bl	f2fs_inode_param_check
	mov	w27, w0
	cbnz	w0, .LBB29_1647
// %bb.1653:
	mov	w2, #2
	mov	x0, x26
	mov	x1, x25
	bl	f2fs_inode_param_check
	mov	w27, w0
	cbnz	w0, .LBB29_1647
// %bb.1654:
	ldr	w8, [x21, #72]
	cbz	w8, .LBB29_1666
// %bb.1655:
	ldr	x0, [x19, #24]
	ldrb	w8, [x0, #3]
	tbnz	w8, #2, .LBB29_1951
.LBB29_1656:
	bl	dget_parent
	mov	x26, x0
	ldr	x0, [x0, #48]
	ldrb	w8, [x0, #13]
	tbz	w8, #6, .LBB29_1658
// %bb.1657:
	ldr	x8, [x25, #40]
	mov	x1, x25
	ldr	x27, [x8, #1008]
	bl	fscrypt_has_permitted_context
	cbz	w0, .LBB29_1744
.LBB29_1658:
	mov	x0, x26
	bl	dput
	ldr	x0, [x21, #48]
	bl	filemap_fdatawrite
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w27, w0
	cbnz	w0, .LBB29_1647
// %bb.1659:
	ldr	x8, [x21, #40]
	mov	x0, x21
	mov	x1, x25
	ldr	x26, [x8, #1008]
	bl	dedup_file_is_same
	tbz	w0, #0, .LBB29_1764
// %bb.1660:
	add	x0, x26, #368           // =368
	str	x0, [sp, #72]           // 8-byte Folded Spill
	bl	_raw_spin_lock
	ldr	x8, [x26, #352]
	cbz	x8, .LBB29_1662
// %bb.1661:
	adrp	x27, jiffies
	ldr	x8, [x26, #360]
	ldr	x9, [x27, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB29_1935
.LBB29_1662:
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	_raw_spin_unlock
	add	x0, x26, #304           // =304
	str	x0, [sp, #64]           // 8-byte Folded Spill
	bl	down_read
	mov	x0, x26
	bl	f2fs_acquire_orphan_inode
	cbz	w0, .LBB29_1870
// %bb.1663:
	ldr	x2, [x21, #64]
	ldr	x3, [x25, #64]
	adrp	x1, .L.str.71
	mov	w27, w0
	add	x1, x1, :lo12:.L.str.71
	mov	x0, x26
	bl	f2fs_printk
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_read
	b	.LBB29_1647
.LBB29_1664:
	ldr	x3, [x21, #64]
	adrp	x1, .L.str.60
	adrp	x2, .L__func__.f2fs_dedup_param_check
	add	x1, x1, :lo12:.L.str.60
	add	x2, x2, :lo12:.L__func__.f2fs_dedup_param_check
	mov	x0, x26
	mov	x4, x3
	b	.LBB29_1643
.LBB29_1665:
	add	x8, sp, #88             // =88
	sub	x8, x8, x2
	add	x0, x8, #4              // =4
	mov	w1, wzr
	bl	memset
	mov	w27, #-14
	sxtw	x0, w27
	b	.LBB29_1455
.LBB29_1666:
	ldr	x2, [x21, #64]
	adrp	x1, .L.str.59
	add	x1, x1, :lo12:.L.str.59
	mov	x0, x26
	bl	f2fs_printk
	mov	w27, #-2
	b	.LBB29_1647
.LBB29_1667:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1627
.LBB29_1668:
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
	cbnz	w9, .LBB29_1670
// %bb.1669:
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
.LBB29_1670:
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
.LBB29_1671:
	cbnz	x2, .LBB29_1700
// %bb.1672:
	ldr	w0, [sp, #88]
	bl	__fdget_pos
	ands	x20, x0, #0xfffffffffffffffc
	b.eq	.LBB29_1677
// %bb.1673:
	ldr	x8, [x20, #32]
	ldr	x9, [x23, #40]
	mov	x21, x0
	ldr	x10, [x8, #40]
	cmp	x9, x10
	b.eq	.LBB29_1678
// %bb.1674:
	ldr	x3, [x23, #64]
	ldr	x4, [x8, #64]
	adrp	x1, .L.str.73
	adrp	x2, .L__func__.f2fs_ioc_dedup_file
	add	x1, x1, :lo12:.L.str.73
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_dedup_file
	mov	x0, x27
.LBB29_1675:
	bl	f2fs_printk
	mov	w28, #-22
	tbz	w21, #1, .LBB29_1684
.LBB29_1676:
	mov	x0, x20
	bl	__f_unlock_pos
	tbnz	w21, #0, .LBB29_1685
	b	.LBB29_1686
.LBB29_1677:
	mov	w28, #-9
	sxtw	x0, w28
	b	.LBB29_1455
.LBB29_1678:
	ldr	x9, [x27, #16]
	ldrb	w9, [x9, #2181]
	tbz	w9, #7, .LBB29_1687
// %bb.1679:
	mov	x25, x8
	ldr	x8, [x8, #736]
	tbz	w8, #26, .LBB29_1687
// %bb.1680:
	ldrsw	x8, [x25, #1184]
	add	x8, x8, #360            // =360
	cmp	x8, #400                // =400
	b.lo	.LBB29_1687
// %bb.1681:
	cmp	x25, x23
	b.eq	.LBB29_1688
// %bb.1682:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w28, w0
	cbz	w0, .LBB29_1689
// %bb.1683:
	tbnz	w21, #1, .LBB29_1676
.LBB29_1684:
	tbz	w21, #0, .LBB29_1686
.LBB29_1685:
	mov	x0, x20
	bl	fput
.LBB29_1686:
	sxtw	x0, w28
	b	.LBB29_1455
.LBB29_1687:
	mov	w28, #-95
	tbz	w21, #1, .LBB29_1684
	b	.LBB29_1676
.LBB29_1688:
	ldr	x3, [x23, #64]
	adrp	x1, .L.str.74
	adrp	x2, .L__func__.f2fs_ioc_dedup_file
	add	x1, x1, :lo12:.L.str.74
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_dedup_file
	mov	x0, x27
	mov	x4, x3
	b	.LBB29_1675
.LBB29_1689:
	add	x22, x23, #160          // =160
	mov	x0, x22
	bl	down_write
	add	x26, x25, #160          // =160
	mov	x0, x26
	bl	down_write_trylock
	cbz	w0, .LBB29_1694
// %bb.1690:
	ldr	x8, [x23, #736]
	tbnz	x8, #38, .LBB29_1695
// %bb.1691:
	ldr	w8, [x23, #72]
	cbz	w8, .LBB29_1701
// %bb.1692:
	ldr	x8, [x25, #736]
	tbnz	x8, #38, .LBB29_1702
.LBB29_1693:
	ldr	x2, [x25, #64]
	adrp	x1, .L.str.78
	add	x1, x1, :lo12:.L.str.78
	b	.LBB29_1696
.LBB29_1694:
	ldr	x2, [x25, #64]
	adrp	x1, .L.str.75
	add	x1, x1, :lo12:.L.str.75
	mov	x0, x27
	bl	f2fs_printk
	mov	w28, #-11
	b	.LBB29_1699
.LBB29_1695:
	ldr	x2, [x23, #64]
	adrp	x1, .L.str.76
	add	x1, x1, :lo12:.L.str.76
.LBB29_1696:
	mov	x0, x27
	bl	f2fs_printk
.LBB29_1697:
	mov	w28, #-22
.LBB29_1698:
	mov	x0, x26
	bl	up_write
.LBB29_1699:
	mov	x0, x22
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
	tbz	w21, #1, .LBB29_1684
	b	.LBB29_1676
.LBB29_1700:
	add	x8, sp, #88             // =88
	sub	x8, x8, x2
	add	x0, x8, #8              // =8
	mov	w1, wzr
	bl	memset
	mov	w28, #-14
	sxtw	x0, w28
	b	.LBB29_1455
.LBB29_1701:
	ldr	x2, [x23, #64]
	adrp	x1, .L.str.77
	add	x1, x1, :lo12:.L.str.77
	mov	x0, x27
	bl	f2fs_printk
	mov	w28, #-2
	b	.LBB29_1698
.LBB29_1702:
	ldr	x8, [x25, #736]
	tbz	x8, #38, .LBB29_1704
// %bb.1703:
	ldr	x8, [x25, #736]
	tbnz	x8, #39, .LBB29_1693
.LBB29_1704:
	ldr	x8, [x25, #40]
	ldr	x9, [x23, #40]
	cmp	x8, x9
	b.eq	.LBB29_1756
// %bb.1705:
	ldr	x3, [x25, #64]
	ldr	x4, [x23, #64]
	adrp	x1, .L.str.60
	adrp	x2, .L__func__.f2fs_dedup_param_check
	add	x1, x1, :lo12:.L.str.60
	add	x2, x2, :lo12:.L__func__.f2fs_dedup_param_check
	mov	x0, x27
	bl	f2fs_printk
	b	.LBB29_1697
.LBB29_1706:
	sub	w8, w23, w24
	str	wzr, [sp, #56]          // 4-byte Folded Spill
	str	wzr, [sp, #48]          // 4-byte Folded Spill
	stp	w24, w8, [sp, #148]
.LBB29_1707:                            // =>This Loop Header: Depth=1
                                        //     Child Loop BB29_1711 Depth 2
	sub	w8, w23, w24
	add	x1, sp, #144            // =144
	mov	x0, x22
	mov	w2, wzr
	mov	w3, wzr
	str	w8, [sp, #152]
	bl	f2fs_map_blocks
	cbnz	w0, .LBB29_1868
// %bb.1708:                            //   in Loop: Header=BB29_1707 Depth=1
	ldrh	w8, [sp, #156]
	mov	w9, #4192
	tst	w8, w9
	b.eq	.LBB29_1714
// %bb.1709:                            //   in Loop: Header=BB29_1707 Depth=1
	add	x8, x22, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4194304
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	w8, [sp, #72]           // 4-byte Folded Reload
	ldr	w9, [sp, #48]           // 4-byte Folded Reload
	ldr	w24, [sp, #148]
	cmp	w9, w8
	b.hs	.LBB29_1715
// %bb.1710:                            //   in Loop: Header=BB29_1707 Depth=1
	ldr	w8, [sp, #152]
	add	w8, w8, w24
	cmp	w24, w8
	b.hs	.LBB29_1715
.LBB29_1711:                            //   Parent Loop BB29_1707 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	mov	w2, #1
	mov	x0, x22
	mov	x1, x24
	bl	f2fs_get_lock_data_page
	mov	x25, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB29_1869
// %bb.1712:                            //   in Loop: Header=BB29_1711 Depth=2
	mov	x0, x25
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x25
	bl	f2fs_put_page
	ldr	w9, [sp, #48]           // 4-byte Folded Reload
	ldr	w8, [sp, #72]           // 4-byte Folded Reload
	add	x24, x24, #1            // =1
	add	w9, w9, #1              // =1
	cmp	w9, w8
	ldr	w8, [sp, #56]           // 4-byte Folded Reload
	str	w9, [sp, #48]           // 4-byte Folded Spill
	add	w8, w8, #1              // =1
	str	w8, [sp, #56]           // 4-byte Folded Spill
	b.hs	.LBB29_1715
// %bb.1713:                            //   in Loop: Header=BB29_1711 Depth=2
	ldp	w8, w9, [sp, #148]
	add	w8, w9, w8
	cmp	x24, x8
	b.lo	.LBB29_1711
	b	.LBB29_1715
.LBB29_1714:                            //   in Loop: Header=BB29_1707 Depth=1
	ldr	w24, [sp, #80]
.LBB29_1715:                            //   in Loop: Header=BB29_1707 Depth=1
	cmp	x23, w24, uxtw
	str	w24, [sp, #148]
	b.ls	.LBB29_1717
// %bb.1716:                            //   in Loop: Header=BB29_1707 Depth=1
	ldr	w8, [sp, #72]           // 4-byte Folded Reload
	ldr	w9, [sp, #48]           // 4-byte Folded Reload
	cmp	w9, w8
	b.lo	.LBB29_1707
.LBB29_1717:                            //   in Loop: Header=BB29_1707 Depth=1
	add	x8, x22, #736           // =736
	mov	w11, #4194304
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	bic	x9, x9, x11
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldr	x0, [x22, #48]
	bl	filemap_fdatawrite
	mov	w25, w0
	cbnz	w0, .LBB29_1546
// %bb.1718:                            //   in Loop: Header=BB29_1707 Depth=1
	ldr	w24, [sp, #148]
	str	wzr, [sp, #48]          // 4-byte Folded Spill
	cmp	x23, x24
	b.hi	.LBB29_1707
	b	.LBB29_1869
.LBB29_1719:
	add	x8, x22, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 34359738368
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x22
	bl	current_time
	stp	x0, x1, [x22, #120]
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_mark_inode_dirty_sync
	ldr	w8, [x22, #1280]
	cbz	w8, .LBB29_1821
// %bb.1720:
	add	x0, x22, #1040          // =1040
	str	x0, [sp, #56]           // 8-byte Folded Spill
	bl	down_write
	add	x0, x22, #1088          // =1088
	str	x0, [sp, #48]           // 8-byte Folded Spill
	bl	down_write
	ldr	x8, [sp, #64]           // 8-byte Folded Reload
	cbz	x8, .LBB29_1856
// %bb.1721:
	mov	x23, #2097152
	mov	x26, #-549755813888
	mov	x28, xzr
	mov	w24, wzr
	movk	x23, #1, lsl #32
	movk	x26, #4076
	b	.LBB29_1725
.LBB29_1722:                            //   in Loop: Header=BB29_1725 Depth=1
	stp	xzr, xzr, [sp, #152]
	tbnz	w27, #31, .LBB29_1857
// %bb.1723:                            //   in Loop: Header=BB29_1725 Depth=1
	add	x28, x25, x28
	add	w24, w27, w24
.LBB29_1724:                            //   in Loop: Header=BB29_1725 Depth=1
	ldr	x8, [sp, #64]           // 8-byte Folded Reload
	cmp	x8, x28
	b.ls	.LBB29_1857
.LBB29_1725:                            // =>This Inner Loop Header: Depth=1
	add	x0, sp, #144            // =144
	mov	w2, #48
	mov	w1, wzr
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	bl	memset
	add	x0, sp, #144            // =144
	mov	w2, #1
	mov	x1, x28
	stp	x22, xzr, [sp, #144]
	str	xzr, [sp, #160]
	str	wzr, [sp, #168]
	bl	f2fs_get_dnode_of_data
	cbz	w0, .LBB29_1728
// %bb.1726:                            //   in Loop: Header=BB29_1725 Depth=1
	mov	w27, w0
	cmn	w0, #2                  // =2
	b.ne	.LBB29_1857
// %bb.1727:                            //   in Loop: Header=BB29_1725 Depth=1
	add	x0, sp, #144            // =144
	mov	x1, x28
	bl	f2fs_get_next_page_offset
	mov	x28, x0
	mov	w27, wzr
	b	.LBB29_1724
.LBB29_1728:                            //   in Loop: Header=BB29_1725 Depth=1
	ldr	x8, [sp, #160]
	mov	x9, #-549755813888
	movk	x9, #4072
	add	x8, x8, x23
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	ldr	w9, [x8, x9]
	ldr	w8, [x8, x26]
	cmp	w9, w8
	b.ne	.LBB29_1732
// %bb.1729:                            //   in Loop: Header=BB29_1725 Depth=1
	ldr	w8, [x22, #1184]
	ldr	w9, [x22, #1192]
	ldrh	w10, [x22]
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	mov	w9, #923
	sub	w8, w9, w8
	b.ne	.LBB29_1737
// %bb.1730:                            //   in Loop: Header=BB29_1725 Depth=1
	ldr	x9, [x22, #736]
	tbz	w9, #31, .LBB29_1737
// %bb.1731:                            //   in Loop: Header=BB29_1725 Depth=1
	ldr	w9, [x22, #1292]
	neg	w9, w9
	b	.LBB29_1736
.LBB29_1732:                            //   in Loop: Header=BB29_1725 Depth=1
	ldrh	w8, [x22]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB29_1734
// %bb.1733:                            //   in Loop: Header=BB29_1725 Depth=1
	ldr	x8, [x22, #736]
	tbnz	w8, #31, .LBB29_1735
.LBB29_1734:                            //   in Loop: Header=BB29_1725 Depth=1
	mov	w8, #1018
	b	.LBB29_1737
.LBB29_1735:                            //   in Loop: Header=BB29_1725 Depth=1
	ldr	w8, [x22, #1292]
	mov	w9, #1018
	neg	w8, w8
.LBB29_1736:                            //   in Loop: Header=BB29_1725 Depth=1
	and	w8, w8, w9
.LBB29_1737:                            //   in Loop: Header=BB29_1725 Depth=1
	ldr	w9, [sp, #172]
	ldr	x10, [sp, #64]          // 8-byte Folded Reload
	mov	w8, w8
	ldr	w11, [x22, #1292]
	sub	x8, x8, x9
	sub	x10, x10, x28
	cmp	x8, x10
	csel	x8, x8, x10, lo
	sub	x8, x8, #1              // =1
	sub	w9, w11, #1             // =1
	orr	x8, x8, x9
	add	x25, x8, #1             // =1
	add	x0, sp, #144            // =144
	mov	x1, x25
	bl	release_compress_blocks
	ldr	x8, [sp, #160]
	mov	w27, w0
	cbz	x8, .LBB29_1739
// %bb.1738:                            //   in Loop: Header=BB29_1725 Depth=1
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_put_page
	ldr	x9, [sp, #160]
	b	.LBB29_1740
.LBB29_1739:                            //   in Loop: Header=BB29_1725 Depth=1
	mov	x9, xzr
.LBB29_1740:                            //   in Loop: Header=BB29_1725 Depth=1
	ldr	x8, [sp, #152]
	cbz	x8, .LBB29_1722
// %bb.1741:                            //   in Loop: Header=BB29_1725 Depth=1
	cmp	x9, x8
	b.eq	.LBB29_1722
// %bb.1742:                            //   in Loop: Header=BB29_1725 Depth=1
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
	cbnz	w9, .LBB29_1722
// %bb.1743:                            //   in Loop: Header=BB29_1725 Depth=1
	bl	__put_page
	b	.LBB29_1722
.LBB29_1744:
	ldr	x8, [x26, #48]
	ldr	x2, [x25, #64]
	adrp	x1, .L.str.70
	add	x1, x1, :lo12:.L.str.70
	ldr	x3, [x8, #64]
	mov	x0, x27
	bl	f2fs_printk
	mov	x0, x26
	bl	dput
	mov	w27, #-1
	b	.LBB29_1647
.LBB29_1745:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbz	w12, #4, .LBB29_1746
	b	.LBB29_683
.LBB29_1746:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_1748
// %bb.1747:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1748:
	ldr	x9, [x9, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x9
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1749:
	mov	w24, wzr
	//APP
	1:.if 1 == 1
661:
	str     x8, [x10]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr x8, [x10]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w24, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w8, [x11, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1752
// %bb.1750:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1753
.LBB29_1751:
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
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_1752:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1751
.LBB29_1753:
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
	cbnz	w9, .LBB29_1755
// %bb.1754:
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
.LBB29_1755:
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
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_1756:
	ldr	x3, [x25, #80]
	ldr	x5, [x23, #80]
	cmp	x3, x5
	b.ne	.LBB29_1833
// %bb.1757:
	mov	w2, #1
	mov	x0, x27
	mov	x1, x25
	bl	f2fs_inode_param_check
	mov	w28, w0
	cbnz	w0, .LBB29_1698
// %bb.1758:
	mov	w2, #1
	mov	x0, x27
	mov	x1, x23
	bl	f2fs_inode_param_check
	mov	w28, w0
	cbnz	w0, .LBB29_1698
// %bb.1759:
	ldr	x0, [x19, #24]
	ldrb	w8, [x0, #3]
	tbnz	w8, #2, .LBB29_1971
.LBB29_1760:
	bl	dget_parent
	mov	x28, x0
	add	x0, x25, #744           // =744
	mov	x27, x25
	str	x0, [sp, #64]           // 8-byte Folded Spill
	bl	down_read
	ldr	x8, [x25, #736]
	tbz	x8, #38, .LBB29_1923
// %bb.1761:
	ldr	x8, [x25, #736]
	tbz	x8, #38, .LBB29_1763
// %bb.1762:
	ldr	x8, [x25, #736]
	tbnz	x8, #39, .LBB29_1923
.LBB29_1763:
	ldr	x0, [x25, #1304]
	bl	igrab
	mov	x27, x0
	b	.LBB29_1924
.LBB29_1764:
	mov	w27, #-116
	b	.LBB29_1647
.LBB29_1765:
	mov	w2, #8
	b	.LBB29_1820
.LBB29_1766:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbz	w8, #4, .LBB29_1767
	b	.LBB29_393
.LBB29_1767:
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
	cbnz	w9, .LBB29_1769
// %bb.1768:
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
.LBB29_1769:
	ldr	x9, [x26, #16]
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	mov	x11, x9
	bfxil	x11, x10, #0, #48
	//APP
	msr ttbr1_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
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
.LBB29_1770:
	lsl	x9, x20, #8
	ldr	x8, [x26, #8]
	and	x28, x20, x9, asr #8
	//APP
		bics	xzr, x28, x8
	csel	x1, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x0, sp, #88             // =88
	mov	w2, #8
	bl	__arch_copy_from_user
	ldr	w8, [x27, :lo12:arm64_const_caps_ready]
	mov	x2, x0
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1773
// %bb.1771:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1774
.LBB29_1772:
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
	b	.LBB29_1777
.LBB29_1773:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1772
.LBB29_1774:
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
	cbnz	w9, .LBB29_1776
// %bb.1775:
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
.LBB29_1776:
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
.LBB29_1777:
	cbnz	x2, .LBB29_1820
// %bb.1778:
	mov	x0, x19
	bl	mnt_want_write_file
                                        // kill: def $w0 killed $w0 def $x0
	cbnz	w0, .LBB29_1899
// %bb.1779:
	add	x22, x21, #160          // =160
	mov	x0, x22
	bl	down_write
	ldr	w8, [sp, #92]
	tbnz	w8, #2, .LBB29_1783
// %bb.1780:
	tbnz	w8, #3, .LBB29_1785
// %bb.1781:
	tbnz	w8, #4, .LBB29_1793
// %bb.1782:
	mov	w24, #-22
	b	.LBB29_1803
.LBB29_1783:
	ldr	x25, [x21, #48]
	ldr	x8, [x25, #32]
	cbz	x8, .LBB29_1789
// %bb.1784:
	ldr	x2, [x21, #64]
	adrp	x1, .L.str.85
	add	x1, x1, :lo12:.L.str.85
	mov	x0, x23
	bl	f2fs_printk
	mov	w24, #-16
	b	.LBB29_1803
.LBB29_1785:
	add	x0, sp, #88             // =88
	mov	w2, #4
	mov	w1, wzr
	bl	memset
	ldr	x8, [x21, #736]
	tbz	x8, #43, .LBB29_1787
// %bb.1786:
	ldr	w8, [sp, #88]
	orr	w8, w8, #0x1
	str	w8, [sp, #88]
.LBB29_1787:
	ldr	x8, [x21, #736]
	tbz	x8, #44, .LBB29_1802
// %bb.1788:
	ldr	w8, [sp, #88]
	mov	w24, wzr
	orr	w8, w8, #0x2
	str	w8, [sp, #88]
	b	.LBB29_1803
.LBB29_1789:
	mov	w2, #3
	mov	x0, x23
	mov	x1, x21
	bl	f2fs_inode_param_check
	mov	w24, w0
	cbnz	w0, .LBB29_1803
// %bb.1790:
	ldrb	w8, [x25, #15]
	tst	w8, #0x6
	b.eq	.LBB29_1812
// %bb.1791:
	ldr	w3, [x21, #792]
	ldrb	w8, [sp, #88]
	tbnz	w8, #7, .LBB29_1810
// %bb.1792:
	ldr	x2, [x21, #64]
	adrp	x1, .L.str.87
	add	x1, x1, :lo12:.L.str.87
	mov	x0, x23
	bl	f2fs_printk
	mov	w24, #-22
	b	.LBB29_1803
.LBB29_1793:
	ldr	x9, [x21, #40]
	ldr	w8, [sp, #88]
	ldr	x23, [x9, #1008]
	tbz	w8, #0, .LBB29_1797
// %bb.1794:
	ldr	x8, [x21, #736]
	tbnz	x8, #43, .LBB29_1796
// %bb.1795:
	ldr	x2, [x21, #64]
	adrp	x1, .L.str.91
	add	x1, x1, :lo12:.L.str.91
	mov	x0, x23
	bl	f2fs_printk
.LBB29_1796:
	add	x8, x21, #736           // =736
	mov	x9, #8796093022208
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	ldr	w8, [sp, #88]
.LBB29_1797:
	tbz	w8, #1, .LBB29_1801
// %bb.1798:
	ldr	x8, [x21, #736]
	tbnz	x8, #44, .LBB29_1800
// %bb.1799:
	ldr	x2, [x21, #64]
	adrp	x1, .L.str.92
	add	x1, x1, :lo12:.L.str.92
	mov	x0, x23
	bl	f2fs_printk
.LBB29_1800:
	add	x8, x21, #736           // =736
	mov	x9, #17592186044416
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
.LBB29_1801:
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
.LBB29_1802:
	mov	w24, wzr
.LBB29_1803:
	mov	x0, x22
	bl	up_write
	mov	x0, x19
	bl	mnt_drop_write_file
	ldrb	w9, [x26, #62]
	ldr	x8, [x26, #8]
	tbnz	w9, #5, .LBB29_1805
// %bb.1804:
	ldr	x10, [x26]
	mov	x9, x20
	tbz	w10, #26, .LBB29_1806
.LBB29_1805:
	mov	x9, x28
.LBB29_1806:
	//APP
		adds	x9, x9, 8
	csel	x8, xzr, x8, hi
	csinv	x9, x9, xzr, cc
	sbcs	xzr, x9, x8
	cset	x9, ls

	//NO_APP
	cbz	x9, .LBB29_1250
// %bb.1807:
	ldr	w8, [x27, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1887
// %bb.1808:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1888
.LBB29_1809:
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
	b	.LBB29_1891
.LBB29_1810:
	adrp	x2, .L.str.88
	add	x2, x2, :lo12:.L.str.88
	add	x0, sp, #144            // =144
	mov	w1, #128
	stp	xzr, xzr, [sp, #256]
	stp	xzr, xzr, [sp, #240]
	stp	xzr, xzr, [sp, #224]
	stp	xzr, xzr, [sp, #208]
	stp	xzr, xzr, [sp, #192]
	stp	xzr, xzr, [sp, #176]
	stp	xzr, xzr, [sp, #160]
	stp	xzr, xzr, [sp, #144]
	bl	snprintf
	adrp	x2, .L__func__.f2fs_ioc_modify_check
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_modify_check
	add	x3, sp, #144            // =144
	mov	x0, x21
	mov	w1, wzr
	bl	f2fs_dedup_imonitor_send
	mov	x2, #9223372036854775807
	mov	x0, x25
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	cbz	w0, .LBB29_1812
// %bb.1811:
	ldr	x2, [x21, #64]
	mov	w24, w0
	adrp	x1, .L.str.86
	add	x1, x1, :lo12:.L.str.86
	mov	x0, x23
	mov	w3, w24
	bl	f2fs_printk
	b	.LBB29_1803
.LBB29_1812:
	ldr	x8, [x21, #40]
	ldrb	w9, [sp, #88]
	ldr	x23, [x8, #1008]
	tbz	w9, #0, .LBB29_1816
// %bb.1813:
	ldr	x8, [x21, #736]
	tbnz	x8, #43, .LBB29_1815
// %bb.1814:
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 8796093022208
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	b	.LBB29_1816
.LBB29_1815:
	ldr	x2, [x21, #64]
	adrp	x1, .L.str.89
	add	x1, x1, :lo12:.L.str.89
	mov	x0, x23
	bl	f2fs_printk
.LBB29_1816:
	ldrb	w8, [sp, #88]
	tbz	w8, #1, .LBB29_1802
// %bb.1817:
	ldr	x8, [x21, #736]
	tbnz	x8, #44, .LBB29_1819
// %bb.1818:
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 17592186044416
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB29_1801
.LBB29_1819:
	ldr	x2, [x21, #64]
	adrp	x1, .L.str.90
	add	x1, x1, :lo12:.L.str.90
	mov	x0, x23
	bl	f2fs_printk
	b	.LBB29_1802
.LBB29_1820:
	add	x8, sp, #88             // =88
	sub	x8, x8, x2
	add	x0, x8, #8              // =8
	mov	w1, wzr
	bl	memset
	b	.LBB29_1250
.LBB29_1821:
	mov	w27, wzr
	mov	w24, wzr
	b	.LBB29_1867
.LBB29_1822:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbz	w12, #4, .LBB29_1823
	b	.LBB29_848
.LBB29_1823:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_1825
// %bb.1824:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1825:
	ldr	x8, [x8, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x8
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1826:
	mov	w25, wzr
	//APP
	1:.if 1 == 1
661:
	str     x11, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr x11, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w25, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w8, [x10, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1829
// %bb.1827:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1830
.LBB29_1828:
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
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_1829:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1828
.LBB29_1830:
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
	cbnz	w9, .LBB29_1832
// %bb.1831:
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
.LBB29_1832:
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
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_1833:
	ldr	x2, [x25, #64]
	ldr	x4, [x23, #64]
	adrp	x1, .L.str.61
	add	x1, x1, :lo12:.L.str.61
	mov	x0, x27
	bl	f2fs_printk
	b	.LBB29_1697
.LBB29_1834:
	adrp	x11, cpu_hwcaps
	ldr	x11, [x11, :lo12:cpu_hwcaps]
	tbz	w11, #4, .LBB29_1835
	b	.LBB29_602
.LBB29_1835:
	//APP
	.if 1 == 1
661:
	mrs	x11, daif
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
	mrs_s x11, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w12, w11, #0x00000080
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
	eor	w12, w11, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w12, .LBB29_1837
// %bb.1836:
	mov	w12, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1837:
	ldr	x8, [x8, #16]
	//APP
	mrs x12, ttbr1_el1
	//NO_APP
	mov	x13, x8
	bfxil	x13, x12, #0, #48
	//APP
	msr ttbr1_el1, x13
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x11
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x11
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1838:
	mov	x8, xzr
	mov	w24, wzr
	//APP
	1:.if 1 == 1
661:
	str     x8, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr x8, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w24, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w8, [x10, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1848
// %bb.1839:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1849
.LBB29_1840:
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
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_1841:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbz	w12, #4, .LBB29_1842
	b	.LBB29_628
.LBB29_1842:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_1844
// %bb.1843:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1844:
	ldr	x8, [x8, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x8
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1845:
	mov	w25, wzr
	//APP
	1:.if 1 == 1
661:
	str     x11, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr x11, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w25, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w8, [x10, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1852
// %bb.1846:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1853
.LBB29_1847:
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
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_1848:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1840
.LBB29_1849:
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
	cbnz	w9, .LBB29_1851
// %bb.1850:
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
.LBB29_1851:
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
	sxtw	x0, w24
	b	.LBB29_1455
.LBB29_1852:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1847
.LBB29_1853:
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
	cbnz	w9, .LBB29_1855
// %bb.1854:
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
.LBB29_1855:
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
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_1856:
	mov	w24, wzr
	mov	w27, wzr
.LBB29_1857:
	ldr	w8, [x22, #1280]
	cbz	w8, .LBB29_1866
// %bb.1858:
	ldr	w3, [x22, #1280]
	ldr	x4, [x22, #64]
	adrp	x1, .L.str.52
	adrp	x2, .L__func__.f2fs_ioc_compress_release_file
	add	x1, x1, :lo12:.L.str.52
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_compress_release_file
	mov	x0, x21
	bl	f2fs_printk
	str	wzr, [x22, #1280]
	cbz	x21, .LBB29_1863
// %bb.1859:
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
	cbnz	w8, .LBB29_1863
// %bb.1860:
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB29_1863
// %bb.1861:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB29_1863
// %bb.1862:
	add	x2, x21, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
.LBB29_1863:
	adrp	x23, f2fs_dclient
	ldr	x0, [x23, :lo12:f2fs_dclient]
	cbz	x0, .LBB29_1866
// %bb.1864:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB29_1866
// %bb.1865:
	ldr	x0, [x23, :lo12:f2fs_dclient]
	adrp	x1, .L.str.17
	adrp	x2, .L__func__.f2fs_ioc_compress_release_file
	add	x1, x1, :lo12:.L.str.17
	add	x2, x2, :lo12:.L__func__.f2fs_ioc_compress_release_file
	mov	w3, #7804
	bl	dsm_client_record
	ldr	x0, [x23, :lo12:f2fs_dclient]
	mov	w1, #15244
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB29_1866:
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	up_write
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	bl	up_write
.LBB29_1867:
	mov	w25, wzr
	b	.LBB29_1113
.LBB29_1868:
	mov	w25, w0
.LBB29_1869:
	add	x8, x22, #736           // =736
	mov	w9, #4194304
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_write
	cbnz	w25, .LBB29_1547
	b	.LBB29_1568
.LBB29_1870:
	mov	x0, x21
	bl	f2fs_add_orphan_inode
	add	x27, x25, #744          // =744
	mov	x0, x27
	bl	down_write
	mov	x0, x25
	bl	igrab
	add	x28, x25, #736          // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x28]
1:	ldxr	x8, [x28]
	orr	x8, x8, 549755813888
	stxr	w9, x8, [x28]
	cbnz	w9, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x25
	bl	f2fs_mark_inode_dirty_sync
	//APP
	// atomic64_or
	prfm	pstl1strm, [x28]
1:	ldxr	x8, [x28]
	orr	x8, x8, 274877906944
	stxr	w9, x8, [x28]
	cbnz	w9, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x25
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x25
	bl	inc_nlink
	mov	w1, #1
	mov	x0, x25
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x27
	bl	up_write
	add	x27, x21, #744          // =744
	mov	x0, x27
	bl	down_write
	str	x25, [x21, #1304]
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 274877906944
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x27
	bl	up_write
	ldr	w1, [x25, #64]
	mov	x0, x26
	bl	f2fs_remove_orphan_inode
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_read
	ldr	w8, [x21, #1300]
	cbz	w8, .LBB29_1876
// %bb.1871:
	add	x0, sp, #144            // =144
	mov	w1, wzr
	stp	xzr, xzr, [sp, #168]
	stp	xzr, xzr, [sp, #152]
	str	xzr, [sp, #144]
	bl	init_wait_entry
	add	x27, x21, #1312         // =1312
.LBB29_1872:                            // =>This Inner Loop Header: Depth=1
	add	x1, sp, #144            // =144
	mov	w2, #2
	mov	x0, x27
	bl	prepare_to_wait_event
	ldr	w8, [x21, #1300]
	cbz	w8, .LBB29_1875
// %bb.1873:                            //   in Loop: Header=BB29_1872 Depth=1
	bl	schedule
	b	.LBB29_1872
.LBB29_1874:
	mov	x0, x21
	mov	x1, xzr
	bl	show_stack
	ldr	x1, [x25, #360]
	ldr	x2, [x27, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x25, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB29_948
.LBB29_1875:
	add	x1, sp, #144            // =144
	mov	x0, x27
	bl	finish_wait
.LBB29_1876:
	add	x27, x21, #992          // =992
	mov	x0, x27
	bl	down_write
	mov	x0, x21
	bl	inode_dio_wait
	mov	x0, x27
	bl	up_write
	add	x0, x21, #1040          // =1040
	str	x0, [sp, #56]           // 8-byte Folded Spill
	bl	down_write
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w27, w0
	cbnz	w0, .LBB29_1880
// %bb.1877:
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	_raw_spin_lock
	ldr	x8, [x26, #352]
	cbz	x8, .LBB29_1879
// %bb.1878:
	adrp	x27, jiffies
	ldr	x8, [x26, #360]
	ldr	x9, [x27, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB29_1948
.LBB29_1879:
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	_raw_spin_unlock
	ldr	x28, [sp, #64]          // 8-byte Folded Reload
	mov	x0, x28
	bl	down_read
	ldr	w1, [x21, #64]
	mov	x0, x26
	bl	f2fs_remove_orphan_inode
	mov	w1, #42
	mov	x0, x21
	bl	f2fs_truncate_dedup_inode
	ldr	x8, [x26, #224]
	mov	w27, w0
	mov	x0, x28
	ldr	x8, [x8]
	str	x8, [x21, #1336]
	bl	up_read
.LBB29_1880:
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	up_write
	adrp	x8, __tracepoint_f2fs_dedup_ioc_create_layered_inode+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_dedup_ioc_create_layered_inode+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1647
// %bb.1881:
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
	tbz	w8, #0, .LBB29_1647
// %bb.1882:
	ldr	w8, [x23, #24]
	add	w8, w8, #1              // =1
	str	w8, [x23, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_dedup_ioc_create_layered_inode+32
	ldr	x26, [x8, :lo12:__tracepoint_f2fs_dedup_ioc_create_layered_inode+32]
	cbz	x26, .LBB29_1884
.LBB29_1883:                            // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x26]
	mov	x1, x21
	mov	x2, x25
	blr	x8
	ldr	x8, [x26, #24]!
	cbnz	x8, .LBB29_1883
.LBB29_1884:
	//APP
	//NO_APP
	ldr	x8, [x23, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x23, #24]
	b.eq	.LBB29_1886
// %bb.1885:
	ldr	x8, [x23, #24]
	cbnz	x8, .LBB29_1647
.LBB29_1886:
	bl	preempt_schedule_notrace
	b	.LBB29_1647
.LBB29_1887:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1809
.LBB29_1888:
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
	cbnz	w9, .LBB29_1890
// %bb.1889:
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
.LBB29_1890:
	ldr	x9, [x26, #16]
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	mov	x11, x9
	bfxil	x11, x10, #0, #48
	//APP
	msr ttbr1_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
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
.LBB29_1891:
	ldr	x8, [x26, #8]
	//APP
		bics	xzr, x28, x8
	csel	x0, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x1, sp, #88             // =88
	mov	w2, #8
	bl	__arch_copy_to_user
	ldr	w8, [x27, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1894
// %bb.1892:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1895
.LBB29_1893:
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
	b	.LBB29_1898
.LBB29_1894:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1893
.LBB29_1895:
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
	cbnz	w9, .LBB29_1897
// %bb.1896:
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
.LBB29_1897:
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
.LBB29_1898:
	cmp	x0, #0                  // =0
	mov	w8, #-14
	csel	w0, w24, w8, eq
.LBB29_1899:
	sxtw	x0, w0
	b	.LBB29_1455
.LBB29_1900:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbz	w12, #4, .LBB29_1901
	b	.LBB29_1124
.LBB29_1901:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB29_1903
// %bb.1902:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1903:
	ldr	x8, [x8, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x8
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB29_1904:
	mov	w25, wzr
	//APP
	1:.if 1 == 1
661:
	str     x11, [x9]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	sttr x11, [x9]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup,"ax"
	.align	2
3:	mov	w25, -14
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w8, [x10, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1907
// %bb.1905:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1908
.LBB29_1906:
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
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_1907:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1906
.LBB29_1908:
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
	cbnz	w9, .LBB29_1910
// %bb.1909:
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
.LBB29_1910:
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
	sxtw	x0, w25
	b	.LBB29_1455
.LBB29_1911:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1554
.LBB29_1912:
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
	cbnz	w9, .LBB29_1914
// %bb.1913:
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
.LBB29_1914:
	ldr	x9, [x26, #16]
	//APP
	mrs x10, ttbr1_el1
	//NO_APP
	mov	x11, x9
	bfxil	x11, x10, #0, #48
	//APP
	msr ttbr1_el1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x9
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
.LBB29_1915:
	ldr	x8, [x26, #8]
	//APP
		bics	xzr, x28, x8
	csel	x0, x20, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	add	x1, sp, #88             // =88
	mov	w2, #16
	bl	__arch_copy_to_user
	ldr	w8, [x27, :lo12:arm64_const_caps_ready]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1918
// %bb.1916:
	adrp	x8, cpu_hwcap_keys+16
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB29_1919
.LBB29_1917:
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
	b	.LBB29_1922
.LBB29_1918:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB29_1917
.LBB29_1919:
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
	cbnz	w9, .LBB29_1921
// %bb.1920:
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
.LBB29_1921:
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
.LBB29_1922:
	cmp	x0, #0                  // =0
	mov	w8, #-14
	csel	w25, wzr, w8, eq
	b	.LBB29_1391
.LBB29_1923:
	mov	x27, xzr
.LBB29_1924:
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_read
	ldr	x0, [x28, #48]
	ldrb	w8, [x0, #13]
	tbz	w8, #6, .LBB29_1926
// %bb.1925:
	ldr	x8, [x27, #40]
	mov	x1, x27
	ldr	x8, [x8, #1008]
	str	x8, [sp, #72]           // 8-byte Folded Spill
	bl	fscrypt_has_permitted_context
	cbz	w0, .LBB29_1936
.LBB29_1926:
	mov	x0, x27
	bl	put_inner_inode
	mov	x0, x28
	bl	dput
	ldr	x0, [x19, #24]
	ldrb	w8, [x0, #3]
	tbnz	w8, #2, .LBB29_1972
.LBB29_1927:
	bl	dget_parent
	ldr	x8, [x20, #24]
	str	x0, [sp, #72]           // 8-byte Folded Spill
	ldrb	w9, [x8, #3]
	tbnz	w9, #2, .LBB29_1973
.LBB29_1928:
	mov	x0, x8
	bl	dget_parent
	ldr	x28, [x19, #32]
	ldr	x8, [sp, #72]           // 8-byte Folded Reload
	ldr	x10, [x20, #32]
	mov	x27, x0
	ldr	x9, [x28, #40]
	ldr	x8, [x8, #48]
	str	x10, [sp, #56]          // 8-byte Folded Spill
	ldr	x9, [x9, #1008]
	ldrb	w10, [x8, #13]
	str	x9, [sp, #48]           // 8-byte Folded Spill
	tbz	w10, #6, .LBB29_1930
// %bb.1929:
	ldr	x1, [sp, #56]           // 8-byte Folded Reload
	mov	x0, x8
	bl	fscrypt_has_permitted_context
	cbz	w0, .LBB29_1937
.LBB29_1930:
	ldr	x0, [x27, #48]
	ldrb	w8, [x0, #13]
	tbz	w8, #6, .LBB29_1932
// %bb.1931:
	mov	x1, x28
	bl	fscrypt_has_permitted_context
	cbz	w0, .LBB29_1938
.LBB29_1932:
	mov	x0, x27
	bl	dput
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	dput
	ldr	x0, [x23, #48]
	mov	x27, x23
	bl	filemap_fdatawrite
	ldr	x0, [x23, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w28, w0
	cbnz	w0, .LBB29_1698
// %bb.1933:
	ldr	x8, [x23, #40]
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	ldr	x8, [x8, #1008]
	str	x8, [sp, #72]           // 8-byte Folded Spill
	bl	down_read
	ldr	x8, [x25, #736]
	tbnz	x8, #38, .LBB29_1940
.LBB29_1934:
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_read
	mov	w28, #-9
	b	.LBB29_1698
.LBB29_1935:
	mov	x0, x23
	mov	x1, xzr
	bl	show_stack
	ldr	x1, [x26, #360]
	ldr	x2, [x27, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x26, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB29_1662
.LBB29_1936:
	ldr	x8, [x28, #48]
	ldr	x2, [x27, #64]
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	adrp	x1, .L.str.70
	ldr	x3, [x8, #64]
	add	x1, x1, :lo12:.L.str.70
	bl	f2fs_printk
	mov	x0, x27
	bl	put_inner_inode
	mov	x0, x28
	b	.LBB29_1939
.LBB29_1937:
	ldr	x23, [sp, #72]          // 8-byte Folded Reload
	ldr	x3, [x28, #64]
	adrp	x1, .L.str.79
	add	x1, x1, :lo12:.L.str.79
	ldr	x8, [x23, #48]
	ldr	x2, [x8, #64]
	ldp	x0, x8, [sp, #48]       // 16-byte Folded Reload
	ldr	x4, [x8, #64]
	bl	f2fs_printk
	mov	x0, x27
	bl	dput
	mov	x0, x23
	b	.LBB29_1939
.LBB29_1938:
	ldr	x8, [x27, #48]
	ldr	x2, [x28, #64]
	adrp	x1, .L.str.80
	add	x1, x1, :lo12:.L.str.80
	ldr	x3, [x8, #64]
	ldp	x0, x8, [sp, #48]       // 16-byte Folded Reload
	ldr	x4, [x8, #64]
	bl	f2fs_printk
	mov	x0, x27
	bl	dput
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
.LBB29_1939:
	bl	dput
	mov	w28, #-1
	b	.LBB29_1698
.LBB29_1940:
	ldr	x8, [x25, #736]
	tbz	x8, #38, .LBB29_1942
// %bb.1941:
	ldr	x8, [x25, #736]
	tbnz	x8, #39, .LBB29_1934
.LBB29_1942:
	ldr	x0, [x25, #1304]
	bl	igrab
	mov	x27, x0
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_read
	str	x27, [sp, #64]          // 8-byte Folded Spill
	cbz	x27, .LBB29_1952
// %bb.1943:
	mov	x0, x25
	mov	x1, x23
	bl	dedup_file_is_same
	tbz	w0, #0, .LBB29_1953
// %bb.1944:
	ldr	x27, [sp, #72]          // 8-byte Folded Reload
	add	x0, x27, #368           // =368
	str	x0, [sp, #56]           // 8-byte Folded Spill
	bl	_raw_spin_lock
	ldr	x8, [x27, #352]
	cbz	x8, .LBB29_1946
// %bb.1945:
	ldr	x8, [sp, #72]           // 8-byte Folded Reload
	adrp	x27, jiffies
	ldr	x8, [x8, #360]
	ldr	x9, [x27, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB29_1970
.LBB29_1946:
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	_raw_spin_unlock
	ldr	x27, [sp, #72]          // 8-byte Folded Reload
	add	x0, x27, #304           // =304
	str	x0, [sp, #48]           // 8-byte Folded Spill
	bl	down_read
	mov	x0, x27
	bl	f2fs_acquire_orphan_inode
	cbz	w0, .LBB29_1954
// %bb.1947:
	mov	w28, w0
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	bl	up_read
	ldr	x2, [x23, #64]
	ldr	x3, [x25, #64]
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	adrp	x1, .L.str.81
	add	x1, x1, :lo12:.L.str.81
	bl	f2fs_printk
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	put_inner_inode
	b	.LBB29_1698
.LBB29_1948:
	mov	x0, x23
	mov	x1, xzr
	bl	show_stack
	ldr	x1, [x26, #360]
	ldr	x2, [x27, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x26, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB29_1879
.LBB29_1949:
	add	x0, x8, #2185           // =2185
	bl	generate_random_uuid
	mov	x0, x22
	mov	w1, wzr
	bl	f2fs_commit_super
	ldr	x8, [x22, #16]
	cbz	w0, .LBB29_523
// %bb.1950:
	mov	w23, w0
	add	x0, x8, #2185           // =2185
	mov	w2, #16
	mov	w1, wzr
	bl	memset
	b	.LBB29_1452
.LBB29_1951:
	ldr	x8, [x0, #96]
	ldr	x1, [x19, #32]
	ldr	x8, [x8, #96]
	blr	x8
	b	.LBB29_1656
.LBB29_1952:
	mov	w28, #-9
	b	.LBB29_1698
.LBB29_1953:
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	put_inner_inode
	mov	w28, #-116
	b	.LBB29_1698
.LBB29_1954:
	mov	x0, x23
	mov	x25, x23
	bl	f2fs_add_orphan_inode
	add	x0, x23, #744           // =744
	str	x0, [sp, #40]           // 8-byte Folded Spill
	bl	down_write
	ldr	x27, [sp, #64]          // 8-byte Folded Reload
	add	x28, x23, #736          // =736
	mov	w1, #1
	mov	x0, x23
	str	x27, [x23, #1304]
	//APP
	// atomic64_or
	prfm	pstl1strm, [x28]
1:	ldxr	x8, [x28]
	orr	x8, x8, 274877906944
	stxr	w9, x8, [x28]
	cbnz	w9, 1b
	//NO_APP
	bl	f2fs_mark_inode_dirty_sync
	//APP
	// atomic64_or
	prfm	pstl1strm, [x28]
1:	ldxr	x8, [x28]
	orr	x8, x8, 4398046511104
	stxr	w9, x8, [x28]
	cbnz	w9, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_mark_inode_dirty_sync
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	bl	up_write
	add	x28, x27, #744          // =744
	mov	x0, x28
	bl	down_write
	mov	x0, x27
	bl	inc_nlink
	mov	w1, #1
	mov	x0, x27
	bl	f2fs_mark_inode_dirty_sync
	mov	x0, x28
	bl	up_write
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	bl	up_read
	ldr	w8, [x23, #1300]
	cbz	w8, .LBB29_1959
// %bb.1955:
	add	x0, sp, #144            // =144
	mov	w1, wzr
	stp	xzr, xzr, [sp, #168]
	stp	xzr, xzr, [sp, #152]
	str	xzr, [sp, #144]
	bl	init_wait_entry
	mov	x27, x23
	add	x25, x23, #1312         // =1312
.LBB29_1956:                            // =>This Inner Loop Header: Depth=1
	add	x1, sp, #144            // =144
	mov	w2, #2
	mov	x0, x25
	bl	prepare_to_wait_event
	ldr	w8, [x23, #1300]
	cbz	w8, .LBB29_1958
// %bb.1957:                            //   in Loop: Header=BB29_1956 Depth=1
	bl	schedule
	b	.LBB29_1956
.LBB29_1958:
	add	x1, sp, #144            // =144
	mov	x0, x25
	bl	finish_wait
.LBB29_1959:
	add	x25, x23, #992          // =992
	mov	x0, x25
	mov	x27, x23
	bl	down_write
	mov	x0, x23
	bl	inode_dio_wait
	mov	x0, x25
	bl	up_write
	add	x0, x23, #1040          // =1040
	str	x0, [sp, #40]           // 8-byte Folded Spill
	bl	down_write
	ldr	x0, [x23, #48]
	mov	x2, #9223372036854775807
	mov	x1, xzr
	bl	filemap_write_and_wait_range
	mov	w28, w0
	cbnz	w0, .LBB29_1963
// %bb.1960:
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	_raw_spin_lock
	ldr	x8, [sp, #72]           // 8-byte Folded Reload
	ldr	x8, [x8, #352]
	cbz	x8, .LBB29_1962
// %bb.1961:
	ldr	x8, [sp, #72]           // 8-byte Folded Reload
	adrp	x27, jiffies
	ldr	x8, [x8, #360]
	ldr	x9, [x27, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB29_1974
.LBB29_1962:
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	_raw_spin_unlock
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	bl	down_read
	ldr	x25, [sp, #72]          // 8-byte Folded Reload
	ldr	w1, [x23, #64]
	mov	x27, x23
	mov	x0, x25
	bl	f2fs_remove_orphan_inode
	mov	w1, #42
	mov	x0, x23
	bl	f2fs_truncate_dedup_inode
	ldr	x8, [x25, #224]
	mov	w28, w0
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	ldr	x8, [x8]
	str	x8, [x23, #1336]
	bl	up_read
.LBB29_1963:
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	bl	up_write
	adrp	x8, __tracepoint_f2fs_dedup_ioc_dedup_inode+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_dedup_ioc_dedup_inode+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB29_1698
// %bb.1964:
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
	tbz	w8, #0, .LBB29_1698
// %bb.1965:
	ldr	w9, [x24, #24]
	mov	x8, x24
	adrp	x8, __tracepoint_f2fs_dedup_ioc_dedup_inode+32
	add	w9, w9, #1              // =1
	str	w9, [x24, #24]
	//APP
	//NO_APP
	ldr	x25, [x8, :lo12:__tracepoint_f2fs_dedup_ioc_dedup_inode+32]
	cbz	x25, .LBB29_1967
.LBB29_1966:                            // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x25]
	ldr	x2, [sp, #64]           // 8-byte Folded Reload
	mov	x1, x23
	blr	x8
	ldr	x8, [x25, #24]!
	cbnz	x8, .LBB29_1966
.LBB29_1967:
	//APP
	//NO_APP
	ldr	x9, [x24, #24]
	mov	x8, x24
	subs	x9, x9, #1              // =1
	str	w9, [x24, #24]
	b.eq	.LBB29_1969
// %bb.1968:
	ldr	x8, [x24, #24]
	cbnz	x8, .LBB29_1698
.LBB29_1969:
	bl	preempt_schedule_notrace
	b	.LBB29_1698
.LBB29_1970:
	mov	x0, x24
	mov	x1, xzr
	bl	show_stack
	ldr	x28, [sp, #72]          // 8-byte Folded Reload
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	ldr	x1, [x28, #360]
	ldr	x2, [x27, :lo12:jiffies]
	bl	printk
	ldr	x0, [x28, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB29_1946
.LBB29_1971:
	ldr	x8, [x0, #96]
	ldr	x1, [x19, #32]
	ldr	x8, [x8, #96]
	blr	x8
	b	.LBB29_1760
.LBB29_1972:
	ldr	x8, [x0, #96]
	ldr	x1, [x19, #32]
	ldr	x8, [x8, #96]
	blr	x8
	b	.LBB29_1927
.LBB29_1973:
	ldr	x9, [x8, #96]
	ldr	x1, [x20, #32]
	mov	x0, x8
	ldr	x9, [x9, #96]
	blr	x9
	mov	x8, x0
	b	.LBB29_1928
.LBB29_1974:
	mov	x0, x24
	mov	x1, xzr
	bl	show_stack
	ldr	x25, [sp, #72]          // 8-byte Folded Reload
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	ldr	x1, [x25, #360]
	ldr	x2, [x27, :lo12:jiffies]
	bl	printk
	ldr	x0, [x25, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB29_1962
.Lfunc_end29:
	.size	__f2fs_ioctl, .Lfunc_end29-__f2fs_ioctl
	.section	.rodata,"a",@progbits
	.p2align	1
.LJTI29_0:
	.hword	(.LBB29_49-.LBB29_49)>>2
	.hword	(.LBB29_247-.LBB29_49)>>2
	.hword	(.LBB29_237-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_249-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_212-.LBB29_49)>>2
	.hword	(.LBB29_1455-.LBB29_49)>>2
	.hword	(.LBB29_216-.LBB29_49)>>2
	.p2align	1
.LJTI29_1:
	.hword	(.LBB29_33-.LBB29_33)>>2
	.hword	(.LBB29_303-.LBB29_33)>>2
	.hword	(.LBB29_282-.LBB29_33)>>2
	.hword	(.LBB29_299-.LBB29_33)>>2
	.hword	(.LBB29_276-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_336-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_275-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_323-.LBB29_33)>>2
	.hword	(.LBB29_308-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_327-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_1455-.LBB29_33)>>2
	.hword	(.LBB29_255-.LBB29_33)>>2
.LJTI29_2:
	.byte	(.LBB29_1216-.LBB29_1216)>>2
	.byte	(.LBB29_1233-.LBB29_1216)>>2
	.byte	(.LBB29_1224-.LBB29_1216)>>2
	.byte	(.LBB29_1226-.LBB29_1216)>>2
                                        // -- End function
	.text
	.globl	f2fs_compat_ioctl       // -- Begin function f2fs_compat_ioctl
	.p2align	2
	.type	f2fs_compat_ioctl,@function
f2fs_compat_ioctl:                      // @f2fs_compat_ioctl
.Lf2fs_compat_ioctl$local:
// %bb.0:
	sub	sp, sp, #96             // =96
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	add	x29, sp, #48            // =48
	str	x21, [sp, #64]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	stur	x8, [x29, #-8]
	ldr	x8, [x0, #32]
	mov	x19, x0
	ldr	x8, [x8, #40]
	ldr	x0, [x8, #1008]
	ldr	x8, [x0, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB30_79
// %bb.1:
	ldr	x8, [x0, #80]
	mov	x20, x2
	mov	w21, w1
	tbnz	w8, #8, .LBB30_80
.LBB30_2:
	mov	w8, #26134
	movk	w8, #49232, lsl #16
	cmp	w21, w8
	mov	x0, #-515
	b.gt	.LBB30_10
// %bb.3:
	mov	w8, #62801
	movk	w8, #32780, lsl #16
	cmp	w21, w8
	b.gt	.LBB30_15
// %bb.4:
	mov	w8, #62733
	movk	w8, #32772, lsl #16
	cmp	w21, w8
	b.gt	.LBB30_25
// %bb.5:
	mov	w8, #22652
	movk	w8, #32772, lsl #16
	cmp	w21, w8
	b.le	.LBB30_37
// %bb.6:
	mov	w8, #30208
	movk	w8, #32772, lsl #16
	cmp	w21, w8
	b.gt	.LBB30_65
// %bb.7:
	mov	w8, #22653
	movk	w8, #32772, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.8:
	mov	w8, #26113
	movk	w8, #32772, lsl #16
	cmp	w21, w8
	b.ne	.LBB30_73
// %bb.9:
	mov	w21, #26113
	movk	w21, #32776, lsl #16
	b	.LBB30_72
.LBB30_10:
	mov	w8, #26113
	movk	w8, #16388, lsl #16
	cmp	w21, w8
	b.gt	.LBB30_20
// %bb.11:
	mov	w8, #62720
	cmp	w21, w8
	b.le	.LBB30_63
// %bb.12:
	mov	w8, #-62721
	add	w8, w21, w8
	cmp	w8, #63                 // =63
	b.hi	.LBB30_14
// %bb.13:
	mov	w9, #1
	lsl	x8, x9, x8
	mov	x9, #16479
	movk	x9, #192, lsl #16
	movk	x9, #32768, lsl #48
	tst	x8, x9
	b.ne	.LBB30_72
.LBB30_14:
	mov	w8, #62742
	movk	w8, #16386, lsl #16
	b	.LBB30_71
.LBB30_15:
	mov	w8, #26133
	movk	w8, #49161, lsl #16
	cmp	w21, w8
	b.gt	.LBB30_29
// %bb.16:
	mov	w8, #22558
	movk	w8, #32796, lsl #16
	cmp	w21, w8
	b.le	.LBB30_40
// %bb.17:
	mov	w8, #26245
	movk	w8, #49156, lsl #16
	cmp	w21, w8
	b.gt	.LBB30_67
// %bb.18:
	mov	w8, #22559
	movk	w8, #32796, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.19:
	mov	w8, #37937
	movk	w8, #33024, lsl #16
	b	.LBB30_71
.LBB30_20:
	mov	w8, #62799
	movk	w8, #16397, lsl #16
	cmp	w21, w8
	b.le	.LBB30_33
// %bb.21:
	mov	w8, #62739
	movk	w8, #16408, lsl #16
	cmp	w21, w8
	b.le	.LBB30_43
// %bb.22:
	mov	w8, #26244
	movk	w8, #16512, lsl #16
	cmp	w21, w8
	b.gt	.LBB30_69
// %bb.23:
	mov	w8, #62740
	movk	w8, #16408, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.24:
	mov	w8, #22560
	movk	w8, #16412, lsl #16
	b	.LBB30_71
.LBB30_25:
	mov	w8, #2799
	movk	w8, #32759, lsl #16
	add	w8, w21, w8
	cmp	w8, #45                 // =45
	b.hi	.LBB30_27
// %bb.26:
	mov	w9, #1
	lsl	x8, x9, x8
	mov	x9, #7
	movk	x9, #10240, lsl #32
	tst	x8, x9
	b.ne	.LBB30_72
.LBB30_27:
	mov	w8, #62734
	movk	w8, #32772, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.28:
	mov	w8, #26131
	movk	w8, #32780, lsl #16
	b	.LBB30_71
.LBB30_29:
	mov	w8, #62728
	movk	w8, #49184, lsl #16
	cmp	w21, w8
	b.gt	.LBB30_52
// %bb.30:
	mov	w8, #26134
	movk	w8, #49161, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.31:
	mov	w8, #62728
	movk	w8, #49168, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.32:
	mov	w8, #22649
	movk	w8, #49176, lsl #16
	b	.LBB30_71
.LBB30_33:
	mov	w8, #62729
	movk	w8, #16392, lsl #16
	cmp	w21, w8
	b.gt	.LBB30_60
// %bb.34:
	mov	w8, #26114
	movk	w8, #16388, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_75
// %bb.35:
	mov	w8, #62726
	movk	w8, #16388, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.36:
	mov	w8, #62733
	movk	w8, #16388, lsl #16
	b	.LBB30_71
.LBB30_37:
	mov	w8, #62781
	movk	w8, #32769, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.38:
	mov	w8, #62783
	movk	w8, #32769, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.39:
	mov	w8, #62741
	movk	w8, #32770, lsl #16
	b	.LBB30_71
.LBB30_40:
	mov	w8, #62802
	movk	w8, #32780, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.41:
	mov	w8, #62801
	movk	w8, #32781, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.42:
	mov	w8, #26139
	movk	w8, #32784, lsl #16
	b	.LBB30_71
.LBB30_43:
	mov	w8, #62800
	movk	w8, #16397, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.44:
	mov	w8, #26132
	movk	w8, #16400, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.45:
	mov	w8, #62731
	movk	w8, #16408, lsl #16
	cmp	w21, w8
	b.ne	.LBB30_73
// %bb.46:
	stp	xzr, xzr, [sp, #16]
	str	xzr, [sp, #8]
	//APP
	mrs x10, sp_el0
	//NO_APP
	ldrb	w9, [x10, #62]
	ldr	x8, [x10, #8]
	and	x11, x20, #0xffffffff
	tbnz	w9, #5, .LBB30_48
// %bb.47:
	ldr	xzr, [x10]
.LBB30_48:
	mov	x12, x11
	//APP
		adds	x12, x12, 4
	csel	x8, xzr, x8, hi
	csinv	x12, x12, xzr, cc
	sbcs	xzr, x12, x8
	cset	x12, ls

	//NO_APP
	adrp	x9, arm64_const_caps_ready
	adrp	x8, cpu_hwcap_keys+16
	cbz	x12, .LBB30_77
// %bb.49:
	ldr	x12, [x10, #8]
	//APP
		bics	xzr, x11, x12
	csel	x13, x11, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	ldr	w12, [x9, :lo12:arm64_const_caps_ready]
	cmp	w12, #1                 // =1
	b.lt	.LBB30_82
// %bb.50:
	ldr	w12, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB30_83
.LBB30_51:
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
	b	.LBB30_86
.LBB30_52:
	mov	w8, #39400
	movk	w8, #16319, lsl #16
	add	w8, w21, w8
	cmp	w8, #2                  // =2
	b.lo	.LBB30_72
// %bb.53:
	mov	w8, #62729
	movk	w8, #49184, lsl #16
	cmp	w21, w8
	b.ne	.LBB30_73
// %bb.54:
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	//APP
	mrs x8, sp_el0
	//NO_APP
	ldrb	w11, [x8, #62]
	ldr	x10, [x8, #8]
	and	x9, x20, #0xffffffff
	tbnz	w11, #5, .LBB30_56
// %bb.55:
	ldr	xzr, [x8]
.LBB30_56:
	mov	x11, x9
	//APP
		adds	x11, x11, 4
	csel	x10, xzr, x10, hi
	csinv	x11, x11, xzr, cc
	sbcs	xzr, x11, x10
	cset	x11, ls

	//NO_APP
	cbz	x11, .LBB30_78
// %bb.57:
	ldr	x10, [x8, #8]
	//APP
		bics	xzr, x9, x10
	csel	x11, x9, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x12, arm64_const_caps_ready
	ldr	w10, [x12, :lo12:arm64_const_caps_ready]
	cmp	w10, #1                 // =1
	b.lt	.LBB30_89
// %bb.58:
	adrp	x10, cpu_hwcap_keys+16
	ldr	w10, [x10, :lo12:cpu_hwcap_keys+16]
	cmp	w10, #0                 // =0
	b.le	.LBB30_90
.LBB30_59:
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
	b	.LBB30_93
.LBB30_60:
	mov	w8, #62730
	movk	w8, #16392, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.61:
	mov	w8, #62803
	movk	w8, #16393, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.62:
	mov	w8, #26133
	movk	w8, #16396, lsl #16
	b	.LBB30_71
.LBB30_63:
	mov	w8, #26135
	movk	w8, #49232, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.64:
	mov	w8, #26138
	movk	w8, #49280, lsl #16
	b	.LBB30_71
.LBB30_65:
	mov	w8, #30209
	movk	w8, #32772, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_76
// %bb.66:
	mov	w8, #62732
	movk	w8, #32772, lsl #16
	b	.LBB30_71
.LBB30_67:
	mov	w8, #26246
	movk	w8, #49156, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.68:
	mov	w8, #62736
	movk	w8, #49160, lsl #16
	b	.LBB30_71
.LBB30_69:
	mov	w8, #26245
	movk	w8, #16512, lsl #16
	cmp	w21, w8
	b.eq	.LBB30_72
// %bb.70:
	mov	w8, #37938
	movk	w8, #16640, lsl #16
.LBB30_71:
	cmp	w21, w8
	b.ne	.LBB30_73
.LBB30_72:
	and	x2, x20, #0xffffffff
	mov	x0, x19
	mov	w1, w21
	bl	__f2fs_ioctl
.LBB30_73:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB30_202
// %bb.74:
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldr	x21, [sp, #64]          // 8-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #96             // =96
	ret
.LBB30_75:
	mov	w21, #26114
	movk	w21, #16392, lsl #16
	b	.LBB30_72
.LBB30_76:
	mov	w8, #26113
	movk	w8, #32776, lsl #16
	orr	w21, w8, #0x1000
	b	.LBB30_72
.LBB30_77:
	mov	w12, #-14
	str	wzr, [sp, #8]
	b	.LBB30_101
.LBB30_78:
	mov	w10, #-14
	str	wzr, [sp, #8]
	b	.LBB30_113
.LBB30_79:
	mov	x0, #-5
	b	.LBB30_73
.LBB30_80:
	mov	w1, wzr
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB30_2
// %bb.81:
	mov	x0, #-28
	b	.LBB30_73
.LBB30_82:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbnz	w12, #4, .LBB30_51
.LBB30_83:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w14, w12, #0x00000080
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
	eor	w14, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w14, .LBB30_85
// %bb.84:
	mov	w14, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x14
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_85:
	ldr	x14, [x10, #16]
	//APP
	mrs x15, ttbr1_el1
	//NO_APP
	mov	x16, x14
	bfxil	x16, x15, #0, #48
	//APP
	msr ttbr1_el1, x16
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_86:
	mov	w12, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     w14, [x13]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr w14, [x13]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w12, -14
	mov	x14, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w13, [x9, :lo12:arm64_const_caps_ready]
	cmp	w13, #1                 // =1
	b.lt	.LBB30_96
// %bb.87:
	ldr	w13, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w13, #0                 // =0
	b.le	.LBB30_97
.LBB30_88:
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
	b	.LBB30_100
.LBB30_89:
	adrp	x10, cpu_hwcaps
	ldr	x10, [x10, :lo12:cpu_hwcaps]
	tbnz	w10, #4, .LBB30_59
.LBB30_90:
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
	and	w13, w10, #0x00000080
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
	eor	w13, w10, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB30_92
// %bb.91:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_92:
	ldr	x13, [x8, #16]
	//APP
	mrs x14, ttbr1_el1
	//NO_APP
	mov	x15, x13
	bfxil	x15, x14, #0, #48
	//APP
	msr ttbr1_el1, x15
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x13
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
.LBB30_93:
	mov	w10, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     w13, [x11]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr w13, [x11]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w10, -14
	mov	x13, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w11, [x12, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	b.lt	.LBB30_108
// %bb.94:
	adrp	x11, cpu_hwcap_keys+16
	ldr	w11, [x11, :lo12:cpu_hwcap_keys+16]
	cmp	w11, #0                 // =0
	b.le	.LBB30_109
.LBB30_95:
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
	b	.LBB30_112
.LBB30_96:
	adrp	x13, cpu_hwcaps
	ldr	x13, [x13, :lo12:cpu_hwcaps]
	tbnz	w13, #4, .LBB30_88
.LBB30_97:
	//APP
	.if 1 == 1
661:
	mrs	x13, daif
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
	mrs_s x13, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w15, w13, #0x00000080
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
	eor	w15, w13, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w15, .LBB30_99
// %bb.98:
	mov	w15, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x15
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_99:
	//APP
	mrs x15, ttbr1_el1
	//NO_APP
	and	x15, x15, #0xffffffffffff
	sub	x16, x15, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x16
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x15
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x13
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_100:
	str	w14, [sp, #8]
.LBB30_101:
	ldrb	w15, [x10, #62]
	ldr	x14, [x10, #8]
	add	x13, x11, #8            // =8
	tbnz	w15, #5, .LBB30_103
// %bb.102:
	ldr	x15, [x10]
.LBB30_103:
	mov	x15, x13
	//APP
		adds	x15, x15, 8
	csel	x14, xzr, x14, hi
	csinv	x15, x15, xzr, cc
	sbcs	xzr, x15, x14
	cset	x15, ls

	//NO_APP
	cbz	x15, .LBB30_107
// %bb.104:
	ldr	x14, [x10, #8]
	//APP
		bics	xzr, x13, x14
	csel	x15, x13, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	ldr	w13, [x9, :lo12:arm64_const_caps_ready]
	cmp	w13, #1                 // =1
	b.lt	.LBB30_120
// %bb.105:
	ldr	w13, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w13, #0                 // =0
	b.le	.LBB30_121
.LBB30_106:
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
	b	.LBB30_124
.LBB30_107:
	mov	x14, xzr
	mov	w13, #-14
	b	.LBB30_138
.LBB30_108:
	adrp	x11, cpu_hwcaps
	ldr	x11, [x11, :lo12:cpu_hwcaps]
	tbnz	w11, #4, .LBB30_95
.LBB30_109:
	//APP
	.if 1 == 1
661:
	mrs	x11, daif
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
	mrs_s x11, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w12, w11, #0x00000080
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
	eor	w12, w11, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w12, .LBB30_111
// %bb.110:
	mov	w12, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_111:
	//APP
	mrs x12, ttbr1_el1
	//NO_APP
	and	x12, x12, #0xffffffffffff
	sub	x14, x12, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x11
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x11
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_112:
	str	w13, [sp, #8]
.LBB30_113:
	ldrb	w13, [x8, #62]
	ldr	x12, [x8, #8]
	add	x11, x9, #8             // =8
	tbnz	w13, #5, .LBB30_115
// %bb.114:
	ldr	x13, [x8]
.LBB30_115:
	mov	x13, x11
	//APP
		adds	x13, x13, 8
	csel	x12, xzr, x12, hi
	csinv	x13, x13, xzr, cc
	sbcs	xzr, x13, x12
	cset	x13, ls

	//NO_APP
	cbz	x13, .LBB30_119
// %bb.116:
	ldr	x12, [x8, #8]
	//APP
		bics	xzr, x11, x12
	csel	x13, x11, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x14, arm64_const_caps_ready
	ldr	w11, [x14, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	b.lt	.LBB30_127
// %bb.117:
	adrp	x11, cpu_hwcap_keys+16
	ldr	w11, [x11, :lo12:cpu_hwcap_keys+16]
	cmp	w11, #0                 // =0
	b.le	.LBB30_128
.LBB30_118:
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
	b	.LBB30_131
.LBB30_119:
	mov	x12, xzr
	mov	w11, #-14
	b	.LBB30_149
.LBB30_120:
	adrp	x13, cpu_hwcaps
	ldr	x13, [x13, :lo12:cpu_hwcaps]
	tbnz	w13, #4, .LBB30_106
.LBB30_121:
	//APP
	.if 1 == 1
661:
	mrs	x13, daif
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
	mrs_s x13, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w14, w13, #0x00000080
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
	eor	w14, w13, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w14, .LBB30_123
// %bb.122:
	mov	w14, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x14
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_123:
	ldr	x14, [x10, #16]
	//APP
	mrs x16, ttbr1_el1
	//NO_APP
	mov	x17, x14
	bfxil	x17, x16, #0, #48
	//APP
	msr ttbr1_el1, x17
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x13
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_124:
	mov	w13, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     x14, [x15]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr x14, [x15]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w13, -14
	mov	x14, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w15, [x9, :lo12:arm64_const_caps_ready]
	cmp	w15, #1                 // =1
	b.lt	.LBB30_134
// %bb.125:
	ldr	w15, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w15, #0                 // =0
	b.le	.LBB30_135
.LBB30_126:
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
	b	.LBB30_138
.LBB30_127:
	adrp	x11, cpu_hwcaps
	ldr	x11, [x11, :lo12:cpu_hwcaps]
	tbnz	w11, #4, .LBB30_118
.LBB30_128:
	//APP
	.if 1 == 1
661:
	mrs	x11, daif
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
	mrs_s x11, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w12, w11, #0x00000080
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
	eor	w12, w11, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w12, .LBB30_130
// %bb.129:
	mov	w12, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_130:
	ldr	x12, [x8, #16]
	//APP
	mrs x15, ttbr1_el1
	//NO_APP
	mov	x16, x12
	bfxil	x16, x15, #0, #48
	//APP
	msr ttbr1_el1, x16
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x11
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x11
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_131:
	mov	w11, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     x12, [x13]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr x12, [x13]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w11, -14
	mov	x12, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w13, [x14, :lo12:arm64_const_caps_ready]
	cmp	w13, #1                 // =1
	b.lt	.LBB30_145
// %bb.132:
	adrp	x13, cpu_hwcap_keys+16
	ldr	w13, [x13, :lo12:cpu_hwcap_keys+16]
	cmp	w13, #0                 // =0
	b.le	.LBB30_146
.LBB30_133:
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
	b	.LBB30_149
.LBB30_134:
	adrp	x15, cpu_hwcaps
	ldr	x15, [x15, :lo12:cpu_hwcaps]
	tbnz	w15, #4, .LBB30_126
.LBB30_135:
	//APP
	.if 1 == 1
661:
	mrs	x15, daif
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
	mrs_s x15, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w16, w15, #0x00000080
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
	eor	w16, w15, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w16, .LBB30_137
// %bb.136:
	mov	w16, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x16
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_137:
	//APP
	mrs x16, ttbr1_el1
	//NO_APP
	and	x16, x16, #0xffffffffffff
	sub	x17, x16, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x17
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x16
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x15
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x15
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_138:
	str	x14, [sp, #16]
	ldrb	w16, [x10, #62]
	ldr	x15, [x10, #8]
	add	x14, x11, #16           // =16
	tbnz	w16, #5, .LBB30_140
// %bb.139:
	ldr	x11, [x10]
.LBB30_140:
	mov	x16, x14
	orr	w11, w13, w12
	//APP
		adds	x16, x16, 8
	csel	x15, xzr, x15, hi
	csinv	x16, x16, xzr, cc
	sbcs	xzr, x16, x15
	cset	x16, ls

	//NO_APP
	cbz	x16, .LBB30_144
// %bb.141:
	ldr	x12, [x10, #8]
	//APP
		bics	xzr, x14, x12
	csel	x13, x14, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	ldr	w12, [x9, :lo12:arm64_const_caps_ready]
	cmp	w12, #1                 // =1
	b.lt	.LBB30_156
// %bb.142:
	ldr	w12, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB30_157
.LBB30_143:
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
	b	.LBB30_160
.LBB30_144:
	mov	x10, xzr
	mov	w12, #-14
	b	.LBB30_174
.LBB30_145:
	adrp	x13, cpu_hwcaps
	ldr	x13, [x13, :lo12:cpu_hwcaps]
	tbnz	w13, #4, .LBB30_133
.LBB30_146:
	//APP
	.if 1 == 1
661:
	mrs	x13, daif
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
	mrs_s x13, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w14, w13, #0x00000080
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
	eor	w14, w13, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w14, .LBB30_148
// %bb.147:
	mov	w14, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x14
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_148:
	//APP
	mrs x14, ttbr1_el1
	//NO_APP
	and	x14, x14, #0xffffffffffff
	sub	x15, x14, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x15
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x13
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_149:
	str	x12, [sp, #16]
	ldrb	w14, [x8, #62]
	ldr	x13, [x8, #8]
	add	x12, x9, #16            // =16
	tbnz	w14, #5, .LBB30_151
// %bb.150:
	ldr	x14, [x8]
.LBB30_151:
	mov	x14, x12
	//APP
		adds	x14, x14, 8
	csel	x13, xzr, x13, hi
	csinv	x14, x14, xzr, cc
	sbcs	xzr, x14, x13
	cset	x14, ls

	//NO_APP
	cbz	x14, .LBB30_155
// %bb.152:
	ldr	x13, [x8, #8]
	//APP
		bics	xzr, x12, x13
	csel	x14, x12, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x15, arm64_const_caps_ready
	ldr	w12, [x15, :lo12:arm64_const_caps_ready]
	cmp	w12, #1                 // =1
	b.lt	.LBB30_163
// %bb.153:
	adrp	x12, cpu_hwcap_keys+16
	ldr	w12, [x12, :lo12:cpu_hwcap_keys+16]
	cmp	w12, #0                 // =0
	b.le	.LBB30_164
.LBB30_154:
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
	b	.LBB30_167
.LBB30_155:
	mov	x13, xzr
	mov	w12, #-14
	b	.LBB30_180
.LBB30_156:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbnz	w12, #4, .LBB30_143
.LBB30_157:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w14, w12, #0x00000080
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
	eor	w14, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w14, .LBB30_159
// %bb.158:
	mov	w14, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x14
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_159:
	ldr	x10, [x10, #16]
	//APP
	mrs x14, ttbr1_el1
	//NO_APP
	mov	x15, x10
	bfxil	x15, x14, #0, #48
	//APP
	msr ttbr1_el1, x15
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_160:
	mov	w12, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     x10, [x13]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr x10, [x13]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w12, -14
	mov	x10, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w9, [x9, :lo12:arm64_const_caps_ready]
	cmp	w9, #1                  // =1
	b.lt	.LBB30_170
// %bb.161:
	ldr	w8, [x8, :lo12:cpu_hwcap_keys+16]
	cmp	w8, #0                  // =0
	b.le	.LBB30_171
.LBB30_162:
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
	b	.LBB30_174
.LBB30_163:
	adrp	x12, cpu_hwcaps
	ldr	x12, [x12, :lo12:cpu_hwcaps]
	tbnz	w12, #4, .LBB30_154
.LBB30_164:
	//APP
	.if 1 == 1
661:
	mrs	x12, daif
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
	mrs_s x12, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w13, w12, #0x00000080
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
	eor	w13, w12, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB30_166
// %bb.165:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_166:
	ldr	x13, [x8, #16]
	//APP
	mrs x16, ttbr1_el1
	//NO_APP
	mov	x17, x13
	bfxil	x17, x16, #0, #48
	//APP
	msr ttbr1_el1, x17
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x13
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x12
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_167:
	mov	w12, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     x13, [x14]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr x13, [x14]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w12, -14
	mov	x13, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w14, [x15, :lo12:arm64_const_caps_ready]
	cmp	w14, #1                 // =1
	b.lt	.LBB30_176
// %bb.168:
	adrp	x14, cpu_hwcap_keys+16
	ldr	w14, [x14, :lo12:cpu_hwcap_keys+16]
	cmp	w14, #0                 // =0
	b.le	.LBB30_177
.LBB30_169:
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
	b	.LBB30_180
.LBB30_170:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #4, .LBB30_162
.LBB30_171:
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
	cbnz	w9, .LBB30_173
// %bb.172:
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
.LBB30_173:
	//APP
	mrs x9, ttbr1_el1
	//NO_APP
	and	x9, x9, #0xffffffffffff
	sub	x13, x9, #1, lsl #12    // =4096
	//APP
	msr ttbr0_el1, x13
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
.LBB30_174:
	orr	w8, w11, w12
	str	x10, [sp, #24]
	cbnz	w8, .LBB30_199
// %bb.175:
	add	x1, sp, #8              // =8
	mov	x0, x19
	bl	__f2fs_ioc_gc_range
	b	.LBB30_201
.LBB30_176:
	adrp	x14, cpu_hwcaps
	ldr	x14, [x14, :lo12:cpu_hwcaps]
	tbnz	w14, #4, .LBB30_169
.LBB30_177:
	//APP
	.if 1 == 1
661:
	mrs	x14, daif
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
	mrs_s x14, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w15, w14, #0x00000080
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
	eor	w15, w14, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w15, .LBB30_179
// %bb.178:
	mov	w15, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x15
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_179:
	//APP
	mrs x15, ttbr1_el1
	//NO_APP
	and	x15, x15, #0xffffffffffff
	sub	x16, x15, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x16
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x15
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x14
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x14
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_180:
	str	x13, [sp, #24]
	ldrb	w14, [x8, #62]
	ldr	x13, [x8, #8]
	orr	w11, w11, w10
	add	x10, x9, #24            // =24
	tbnz	w14, #5, .LBB30_182
// %bb.181:
	ldr	x9, [x8]
.LBB30_182:
	mov	x14, x10
	orr	w9, w11, w12
	//APP
		adds	x14, x14, 8
	csel	x13, xzr, x13, hi
	csinv	x14, x14, xzr, cc
	sbcs	xzr, x14, x13
	cset	x14, ls

	//NO_APP
	cbz	x14, .LBB30_186
// %bb.183:
	ldr	x12, [x8, #8]
	//APP
		bics	xzr, x10, x12
	csel	x11, x10, xzr, eq

	//NO_APP
	//APP
	hint #20
	//NO_APP
	adrp	x12, arm64_const_caps_ready
	ldr	w10, [x12, :lo12:arm64_const_caps_ready]
	cmp	w10, #1                 // =1
	b.lt	.LBB30_187
// %bb.184:
	adrp	x10, cpu_hwcap_keys+16
	ldr	w10, [x10, :lo12:cpu_hwcap_keys+16]
	cmp	w10, #0                 // =0
	b.le	.LBB30_188
.LBB30_185:
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
	b	.LBB30_191
.LBB30_186:
	mov	x8, xzr
	mov	w10, #-14
	b	.LBB30_198
.LBB30_187:
	adrp	x10, cpu_hwcaps
	ldr	x10, [x10, :lo12:cpu_hwcaps]
	tbnz	w10, #4, .LBB30_185
.LBB30_188:
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
	and	w13, w10, #0x00000080
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
	eor	w13, w10, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w13, .LBB30_190
// %bb.189:
	mov	w13, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x13
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_190:
	ldr	x8, [x8, #16]
	//APP
	mrs x13, ttbr1_el1
	//NO_APP
	mov	x14, x8
	bfxil	x14, x13, #0, #48
	//APP
	msr ttbr1_el1, x14
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr0_el1, x8
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
.LBB30_191:
	mov	w10, wzr
	//APP
	1:.if 1 == 1
661:
	ldr     x8, [x11]

662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 9
 .byte 662b-661b
 .byte 664f-663f
.popsection
.pushsection .altinstr_replacement, "a"
663:
	ldtr x8, [x11]

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif
2:
	.section .fixup, "ax"
	.align	2
3:	mov	w10, -14
	mov	x8, #0
	b	2b
	.previous
	.pushsection	__ex_table, "a"
	.align		3
	.long		(1b - .), (3b - .)
	.popsection

	//NO_APP
	ldr	w11, [x12, :lo12:arm64_const_caps_ready]
	cmp	w11, #1                 // =1
	b.lt	.LBB30_194
// %bb.192:
	adrp	x11, cpu_hwcap_keys+16
	ldr	w11, [x11, :lo12:cpu_hwcap_keys+16]
	cmp	w11, #0                 // =0
	b.le	.LBB30_195
.LBB30_193:
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
	b	.LBB30_198
.LBB30_194:
	adrp	x11, cpu_hwcaps
	ldr	x11, [x11, :lo12:cpu_hwcaps]
	tbnz	w11, #4, .LBB30_193
.LBB30_195:
	//APP
	.if 1 == 1
661:
	mrs	x11, daif
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
	mrs_s x11, (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5))
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
	and	w12, w11, #0x00000080
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
	eor	w12, w11, #0xe0
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
	cbnz	w12, .LBB30_197
// %bb.196:
	mov	w12, #96
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x12
	.purgem	msr_s

664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_197:
	//APP
	mrs x12, ttbr1_el1
	//NO_APP
	and	x12, x12, #0xffffffffffff
	sub	x13, x12, #1, lsl #12   // =4096
	//APP
	msr ttbr0_el1, x13
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr ttbr1_el1, x12
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	.if 1 == 1
661:
	msr	daif, x11
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
	msr_s (((3) << 19) | ((0) << 16) | ((4) << 12) | ((6) << 8) | ((0) << 5)), x11
	.purgem	msr_s
dsb	sy
664:
	.popsection
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
.endif

	//NO_APP
.LBB30_198:
	orr	w9, w9, w10
	str	x8, [sp, #32]
	cbz	w9, .LBB30_200
.LBB30_199:
	mov	x0, #-14
	b	.LBB30_73
.LBB30_200:
	add	x1, sp, #8              // =8
	mov	x0, x19
	bl	__f2fs_ioc_move_range
.LBB30_201:
                                        // kill: def $w0 killed $w0 def $x0
	sxtw	x0, w0
	b	.LBB30_73
.LBB30_202:
	bl	__stack_chk_fail
.Lfunc_end30:
	.size	f2fs_compat_ioctl, .Lfunc_end30-f2fs_compat_ioctl
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_llseek
	.type	f2fs_llseek,@function
f2fs_llseek:                            // @f2fs_llseek
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
	ldr	x8, [x0, #248]
	mov	w21, w2
	mov	x20, x0
	mov	x24, x1
	ldr	x19, [x8]
	ldr	x8, [x19, #40]
	ldrh	w9, [x19]
	ldr	x3, [x8, #32]
	and	w8, w9, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB31_2
// %bb.1:
	ldr	x8, [x19, #736]
	tbnz	w8, #31, .LBB31_4
.LBB31_2:
	subs	w8, w21, #3             // =3
	b.hs	.LBB31_5
.LBB31_3:
	ldr	x4, [x19, #80]
	mov	x0, x20
	mov	x1, x24
	mov	w2, w21
	bl	generic_file_llseek_size
	b	.LBB31_102
.LBB31_4:
	mov	x0, x19
	bl	max_file_blocks
	lsl	x3, x0, #12
	subs	w8, w21, #3             // =3
	b.lo	.LBB31_3
.LBB31_5:
	cmp	w8, #2                  // =2
	b.hs	.LBB31_12
// %bb.6:
	tbnz	x24, #63, .LBB31_101
// %bb.7:
	ldr	x8, [x20, #248]
	ldr	x19, [x8]
	ldr	x8, [x19, #40]
	add	x22, x19, #160          // =160
	mov	x0, x22
	ldr	x23, [x8, #32]
	stp	xzr, xzr, [sp, #72]
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	bl	down_write
	ldr	x8, [x19, #80]
	cmp	x8, x24
	str	x8, [sp, #32]           // 8-byte Folded Spill
	b.le	.LBB31_100
// %bb.8:
	ldr	x8, [x19, #736]
	tbz	w8, #10, .LBB31_13
// %bb.9:
	cmp	w21, #4                 // =4
	b.eq	.LBB31_77
// %bb.10:
	cmp	w21, #3                 // =3
	b.ne	.LBB31_13
// %bb.11:
	ldr	x8, [sp, #32]           // 8-byte Folded Reload
	b	.LBB31_78
.LBB31_12:
	mov	x0, #-22
	b	.LBB31_102
.LBB31_13:
	add	x26, x19, #744          // =744
	mov	x0, x26
	lsr	x25, x24, #12
	bl	down_read
	ldr	x8, [x19, #736]
	str	x22, [sp, #24]          // 8-byte Folded Spill
	str	x23, [sp, #8]           // 8-byte Folded Spill
	tbnz	x8, #38, .LBB31_15
.LBB31_14:
	mov	x27, xzr
	b	.LBB31_18
.LBB31_15:
	ldr	x8, [x19, #736]
	tbz	x8, #38, .LBB31_17
// %bb.16:
	ldr	x8, [x19, #736]
	tbnz	x8, #39, .LBB31_14
.LBB31_17:
	ldr	x0, [x19, #1304]
	bl	igrab
	mov	x27, x0
.LBB31_18:
	mov	x28, #2097152
	mov	x0, x26
	movk	x28, #1, lsl #32
	bl	up_read
	cmp	x27, #0                 // =0
	str	x27, [sp, #16]          // 8-byte Folded Spill
	csel	x27, x19, x27, eq
	ldr	x19, [sp, #32]          // 8-byte Folded Reload
	mov	x22, #-549755813888
	mov	x23, #-549755813888
	movk	x22, #4072
	movk	x23, #4076
	b	.LBB31_20
.LBB31_19:                              //   in Loop: Header=BB31_20 Depth=1
	lsl	x24, x25, #12
	cmp	x24, x19
	stp	xzr, xzr, [sp, #48]
	b.ge	.LBB31_91
.LBB31_20:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB31_32 Depth 2
                                        //       Child Loop BB31_61 Depth 3
                                        //       Child Loop BB31_48 Depth 3
	add	x0, sp, #40             // =40
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	add	x0, sp, #40             // =40
	mov	w2, #1
	mov	x1, x25
	stp	x27, xzr, [sp, #40]
	str	xzr, [sp, #56]
	str	wzr, [sp, #64]
	bl	f2fs_get_dnode_of_data
	cbz	w0, .LBB31_24
// %bb.21:                              //   in Loop: Header=BB31_20 Depth=1
	cmn	w0, #2                  // =2
	b.ne	.LBB31_98
// %bb.22:                              //   in Loop: Header=BB31_20 Depth=1
	cmp	w21, #3                 // =3
	b.ne	.LBB31_85
// %bb.23:                              //   in Loop: Header=BB31_20 Depth=1
	add	x0, sp, #40             // =40
	mov	x1, x25
	bl	f2fs_get_next_page_offset
	lsl	x24, x0, #12
	mov	x25, x0
	cmp	x24, x19
	b.lt	.LBB31_20
	b	.LBB31_98
.LBB31_24:                              //   in Loop: Header=BB31_20 Depth=1
	ldr	x0, [sp, #56]
	add	x8, x0, x28
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	ldr	w9, [x8, x22]
	ldr	w8, [x8, x23]
	cmp	w9, w8
	b.ne	.LBB31_28
// %bb.25:                              //   in Loop: Header=BB31_20 Depth=1
	ldr	w8, [x27, #1184]
	ldr	w9, [x27, #1192]
	ldrh	w10, [x27]
	mov	x14, #-549755813888
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	mov	w9, #923
	sub	w19, w9, w8
	b.ne	.LBB31_31
// %bb.26:                              //   in Loop: Header=BB31_20 Depth=1
	ldr	x8, [x27, #736]
	tbz	w8, #31, .LBB31_31
// %bb.27:                              //   in Loop: Header=BB31_20 Depth=1
	ldr	w8, [x27, #1292]
	neg	w8, w8
	and	w19, w19, w8
	b	.LBB31_31
.LBB31_28:                              //   in Loop: Header=BB31_20 Depth=1
	ldrh	w8, [x27]
	mov	x14, #-549755813888
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB31_30
// %bb.29:                              //   in Loop: Header=BB31_20 Depth=1
	ldr	x8, [x27, #736]
	tbnz	w8, #31, .LBB31_74
.LBB31_30:                              //   in Loop: Header=BB31_20 Depth=1
	mov	w19, #1018
.LBB31_31:                              //   in Loop: Header=BB31_20 Depth=1
	ldr	w8, [sp, #68]
	cmp	w8, w19
	b.hs	.LBB31_68
.LBB31_32:                              //   Parent Loop BB31_20 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB31_61 Depth 3
                                        //       Child Loop BB31_48 Depth 3
	add	x9, x0, x28
	lsl	x9, x9, #6
	and	x11, x9, #0xfffffffffffff000
	ldr	w9, [x11, x22]
	ldr	w10, [x11, x23]
	add	x11, x11, x14
	cmp	w9, w10
	b.ne	.LBB31_37
// %bb.33:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	x12, [sp, #40]
	cbz	x12, .LBB31_36
// %bb.34:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	x13, [x12, #736]
	tbz	w13, #26, .LBB31_37
// %bb.35:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	w12, [x12, #1184]
	asr	w12, w12, #2
	b	.LBB31_38
.LBB31_36:                              //   in Loop: Header=BB31_32 Depth=2
	ldrb	w12, [x11, #3]
	tbnz	w12, #5, .LBB31_66
.LBB31_37:                              //   in Loop: Header=BB31_32 Depth=2
	mov	w12, wzr
.LBB31_38:                              //   in Loop: Header=BB31_32 Depth=2
	add	x13, x11, #360          // =360
	cmp	w9, w10
	csel	x9, x13, x11, eq
	add	w8, w12, w8
	ldr	w26, [x9, w8, uxtw #2]
	add	w8, w26, #3             // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB31_57
.LBB31_39:                              //   in Loop: Header=BB31_32 Depth=2
	cmp	w21, #4                 // =4
	b.eq	.LBB31_54
// %bb.40:                              //   in Loop: Header=BB31_32 Depth=2
	cmp	w21, #3                 // =3
	b.ne	.LBB31_55
// %bb.41:                              //   in Loop: Header=BB31_32 Depth=2
	add	w8, w26, #3             // =3
	cmp	w8, #2                  // =2
	b.lo	.LBB31_55
// %bb.42:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	x8, [x20, #248]
	cmn	w26, #1                 // =1
	b.eq	.LBB31_53
// %bb.43:                              //   in Loop: Header=BB31_32 Depth=2
	cbz	w26, .LBB31_55
// %bb.44:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	x9, [x8]
	ldr	x9, [x9, #40]
	ldr	x11, [x9, #1008]
	ldr	w10, [x11, #3128]
	cbz	w10, .LBB31_75
// %bb.45:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	x9, [x11]
	cmp	w10, #1                 // =1
	ldr	x9, [x9, #248]
	b.lt	.LBB31_51
// %bb.46:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	x11, [x11, #3136]
	add	x11, x11, #80           // =80
	b	.LBB31_48
.LBB31_47:                              //   in Loop: Header=BB31_48 Depth=3
	subs	x10, x10, #1            // =1
	add	x11, x11, #88           // =88
	b.eq	.LBB31_51
.LBB31_48:                              //   Parent Loop BB31_20 Depth=1
                                        //     Parent Loop BB31_32 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	ldur	w12, [x11, #-4]
	cmp	w12, w26
	b.hi	.LBB31_47
// %bb.49:                              //   in Loop: Header=BB31_48 Depth=3
	ldr	w12, [x11]
	cmp	w12, w26
	b.lo	.LBB31_47
// %bb.50:                              //   in Loop: Header=BB31_32 Depth=2
	ldur	x9, [x11, #-80]
.LBB31_51:                              //   in Loop: Header=BB31_32 Depth=2
	cbnz	x9, .LBB31_75
// %bb.52:                              //   in Loop: Header=BB31_32 Depth=2
	cmn	w26, #1                 // =1
	b.ne	.LBB31_55
.LBB31_53:                              //   in Loop: Header=BB31_32 Depth=2
	add	x0, x8, #8              // =8
	mov	x1, x25
	mov	w2, wzr
	bl	xa_get_mark
	mov	x14, #-549755813888
	tbz	w0, #0, .LBB31_55
	b	.LBB31_75
.LBB31_54:                              //   in Loop: Header=BB31_32 Depth=2
	cbz	w26, .LBB31_75
.LBB31_55:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	w8, [sp, #68]
	add	x25, x25, #1            // =1
	add	w8, w8, #1              // =1
	cmp	w8, w19
	str	w8, [sp, #68]
	b.hs	.LBB31_67
// %bb.56:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	x0, [sp, #56]
	lsl	x24, x25, #12
	b	.LBB31_32
.LBB31_57:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	x8, [x27, #40]
	ldr	x0, [x8, #1008]
	ldr	w9, [x0, #3128]
	cbz	w9, .LBB31_65
// %bb.58:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	x8, [x0]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB31_64
// %bb.59:                              //   in Loop: Header=BB31_32 Depth=2
	ldr	x10, [x0, #3136]
	add	x10, x10, #80           // =80
	b	.LBB31_61
.LBB31_60:                              //   in Loop: Header=BB31_61 Depth=3
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB31_64
.LBB31_61:                              //   Parent Loop BB31_20 Depth=1
                                        //     Parent Loop BB31_32 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	ldur	w11, [x10, #-4]
	cmp	w11, w26
	b.hi	.LBB31_60
// %bb.62:                              //   in Loop: Header=BB31_61 Depth=3
	ldr	w11, [x10]
	cmp	w11, w26
	b.lo	.LBB31_60
// %bb.63:                              //   in Loop: Header=BB31_32 Depth=2
	ldur	x8, [x10, #-80]
.LBB31_64:                              //   in Loop: Header=BB31_32 Depth=2
	cbz	x8, .LBB31_39
.LBB31_65:                              //   in Loop: Header=BB31_32 Depth=2
	mov	w2, #7
	mov	w1, w26
	bl	f2fs_is_valid_blkaddr
	mov	x14, #-549755813888
	tbnz	w0, #0, .LBB31_39
	b	.LBB31_89
.LBB31_66:                              //   in Loop: Header=BB31_32 Depth=2
	ldrh	w12, [x11, #360]
	lsr	w12, w12, #2
	b	.LBB31_38
.LBB31_67:                              //   in Loop: Header=BB31_20 Depth=1
	ldr	x0, [sp, #56]
.LBB31_68:                              //   in Loop: Header=BB31_20 Depth=1
	ldr	x19, [sp, #32]          // 8-byte Folded Reload
	cbz	x0, .LBB31_70
// %bb.69:                              //   in Loop: Header=BB31_20 Depth=1
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #56]
	ldr	x8, [sp, #48]
	cbnz	x8, .LBB31_71
	b	.LBB31_19
.LBB31_70:                              //   in Loop: Header=BB31_20 Depth=1
	mov	x9, xzr
	ldr	x8, [sp, #48]
	cbz	x8, .LBB31_19
.LBB31_71:                              //   in Loop: Header=BB31_20 Depth=1
	cmp	x9, x8
	b.eq	.LBB31_19
// %bb.72:                              //   in Loop: Header=BB31_20 Depth=1
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
	cbnz	w9, .LBB31_19
// %bb.73:                              //   in Loop: Header=BB31_20 Depth=1
	bl	__put_page
	b	.LBB31_19
.LBB31_74:                              //   in Loop: Header=BB31_20 Depth=1
	ldr	w8, [x27, #1292]
	mov	w9, #1018
	neg	w8, w8
	and	w19, w8, w9
	b	.LBB31_31
.LBB31_75:
	ldr	x0, [sp, #56]
	cbz	x0, .LBB31_79
// %bb.76:
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #56]
	b	.LBB31_80
.LBB31_77:
	ldr	x8, [sp, #32]           // 8-byte Folded Reload
	mov	x24, x8
.LBB31_78:
	cmp	x24, x8
	ccmp	w21, #4, #0, gt
	csel	x21, x8, x24, eq
	b	.LBB31_88
.LBB31_79:
	mov	x9, xzr
.LBB31_80:
	ldr	x19, [sp, #32]          // 8-byte Folded Reload
	ldr	x8, [sp, #48]
	cbz	x8, .LBB31_84
// %bb.81:
	cmp	x9, x8
	b.eq	.LBB31_84
// %bb.82:
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
	cbnz	w10, .LBB31_84
// %bb.83:
	bl	__put_page
.LBB31_84:
	stp	xzr, xzr, [sp, #48]
.LBB31_85:
	ldr	x0, [sp, #16]           // 8-byte Folded Reload
	cmp	x24, x19
	ccmp	w21, #4, #0, gt
	csel	x21, x19, x24, eq
	cbz	x0, .LBB31_87
// %bb.86:
	bl	put_inner_inode
.LBB31_87:
	ldr	x22, [sp, #24]          // 8-byte Folded Reload
	ldr	x23, [sp, #8]           // 8-byte Folded Reload
.LBB31_88:
	mov	x0, x22
	bl	up_write
	mov	x0, x20
	mov	x1, x21
	mov	x2, x23
	bl	vfs_setpos
	b	.LBB31_102
.LBB31_89:
	ldr	x0, [sp, #56]
	cbz	x0, .LBB31_92
// %bb.90:
	mov	w1, #1
	bl	f2fs_put_page
	ldr	x9, [sp, #56]
	b	.LBB31_93
.LBB31_91:
	cmp	w21, #3                 // =3
	b.ne	.LBB31_85
	b	.LBB31_98
.LBB31_92:
	mov	x9, xzr
.LBB31_93:
	ldr	x8, [sp, #48]
	cbz	x8, .LBB31_97
// %bb.94:
	cmp	x9, x8
	b.eq	.LBB31_97
// %bb.95:
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
	cbnz	w10, .LBB31_97
// %bb.96:
	bl	__put_page
.LBB31_97:
	stp	xzr, xzr, [sp, #48]
.LBB31_98:
	ldp	x0, x22, [sp, #16]      // 16-byte Folded Reload
	cbz	x0, .LBB31_100
// %bb.99:
	bl	put_inner_inode
.LBB31_100:
	mov	x0, x22
	bl	up_write
.LBB31_101:
	mov	x0, #-6
.LBB31_102:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB31_104
// %bb.103:
	ldp	x20, x19, [sp, #176]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #160]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #144]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #128]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #112]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #96]     // 16-byte Folded Reload
	add	sp, sp, #192            // =192
	ret
.LBB31_104:
	bl	__stack_chk_fail
.Lfunc_end31:
	.size	f2fs_llseek, .Lfunc_end31-f2fs_llseek
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_file_read_iter
	.type	f2fs_file_read_iter,@function
f2fs_file_read_iter:                    // @f2fs_file_read_iter
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x8, [x0]
	mov	x19, x0
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	ldr	x20, [x8, #32]
	mov	x21, x1
	mov	x0, x20
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB32_5
// %bb.1:
	mov	x0, x19
	mov	x1, x21
	bl	generic_file_read_iter
	mov	x19, x0
	cmp	w19, #1                 // =1
	b.lt	.LBB32_4
// %bb.2:
	ldr	x8, [x20, #40]
	ldr	x20, [x8, #1008]
	ldrb	w8, [x20, #2704]
	cbz	w8, .LBB32_4
// %bb.3:
	add	x21, x20, #2344         // =2344
	mov	x0, x21
	bl	_raw_spin_lock
	ldr	x8, [x20, #2464]
	ldr	x9, [x20, #2448]
	mov	x0, x21
	add	x8, x8, w19, sxtw
	str	x8, [x20, #2464]
	sub	x8, x8, x9
	str	x8, [x20, #2456]
	bl	_raw_spin_unlock
	mov	x0, x20
	bl	f2fs_record_iostat
.LBB32_4:
	sxtw	x0, w19
	b	.LBB32_6
.LBB32_5:
	mov	x0, #-95
.LBB32_6:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.Lfunc_end32:
	.size	f2fs_file_read_iter, .Lfunc_end32-f2fs_file_read_iter
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_file_write_iter
	.type	f2fs_file_write_iter,@function
f2fs_file_write_iter:                   // @f2fs_file_write_iter
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x0]
	mov	x19, x0
	mov	x21, x1
	str	x25, [sp, #16]          // 8-byte Folded Spill
	ldr	x20, [x8, #32]
	mov	x29, sp
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB33_71
// %bb.1:
	mov	x0, x20
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB33_4
// %bb.2:
	ldrb	w8, [x19, #32]
	add	x22, x20, #160          // =160
	mov	x0, x22
	tbnz	w8, #3, .LBB33_5
// %bb.3:
	bl	down_write
	b	.LBB33_6
.LBB33_4:
	mov	x23, #-95
	b	.LBB33_27
.LBB33_5:
	bl	down_write_trylock
	cbz	w0, .LBB33_24
.LBB33_6:
	ldrb	w8, [x20, #12]
	tbnz	w8, #3, .LBB33_85
// %bb.7:
	ldr	x8, [x20, #736]
	tbz	x8, #35, .LBB33_9
// %bb.8:
	mov	w1, #8
	mov	x0, x20
	bl	f2fs_reserve_compress_blocks_internal
                                        // kill: def $w0 killed $w0 def $x0
	cbnz	w0, .LBB33_15
.LBB33_9:
	ldr	x8, [x20, #736]
	tbz	x8, #44, .LBB33_11
// %bb.10:
	add	x8, x20, #736           // =736
	mov	x9, #17592186044416
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_mark_inode_dirty_sync
.LBB33_11:
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB33_16
// %bb.12:
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB33_14
// %bb.13:
	ldr	x8, [x20, #736]
	tbnz	x8, #39, .LBB33_16
.LBB33_14:
	adrp	x1, .L__func__.f2fs_file_write_iter
	add	x1, x1, :lo12:.L__func__.f2fs_file_write_iter
	mov	x0, x20
	bl	f2fs_revoke_deduped_inode
                                        // kill: def $w0 killed $w0 def $x0
	cbz	w0, .LBB33_16
.LBB33_15:
	sxtw	x23, w0
	b	.LBB33_26
.LBB33_16:
	mov	x0, x19
	mov	x1, x21
	bl	generic_write_checks
	cmp	x0, #1                  // =1
	b.lt	.LBB33_25
// %bb.17:
	ldr	x1, [x21, #16]
	mov	x0, x21
	bl	iov_iter_fault_in_readable
	cbz	w0, .LBB33_19
// %bb.18:
	add	x8, x20, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 16777216
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB33_19:
	ldr	w8, [x19, #32]
	tbnz	w8, #3, .LBB33_38
// %bb.20:
	ldr	x9, [x20, #736]
	tbnz	w9, #24, .LBB33_64
// %bb.21:
	tbnz	w8, #17, .LBB33_43
.LBB33_22:
	ldr	x24, [x19, #8]
	ldr	x25, [x21, #16]
	mov	x0, x19
	mov	x1, x21
	bl	f2fs_preallocate_blocks
	mov	w23, w0
	cbnz	w0, .LBB33_44
// %bb.23:
	add	x24, x25, x24
	mov	w25, #1
	b	.LBB33_65
.LBB33_24:
	mov	x23, #-11
	b	.LBB33_27
.LBB33_25:
	mov	x23, x0
.LBB33_26:
	mov	x0, x22
	bl	up_write
.LBB33_27:
	adrp	x8, __tracepoint_f2fs_file_write_iter+8
	ldr	x22, [x19, #8]
	ldr	x21, [x21, #16]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_file_write_iter+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB33_33
// %bb.28:
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
	tbz	w8, #0, .LBB33_33
// %bb.29:
	//APP
	mrs x24, sp_el0
	//NO_APP
	ldr	w8, [x24, #24]
	add	w8, w8, #1              // =1
	str	w8, [x24, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_file_write_iter+32
	ldr	x25, [x8, :lo12:__tracepoint_f2fs_file_write_iter+32]
	cbz	x25, .LBB33_31
.LBB33_30:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x25]
	mov	x1, x20
	mov	x2, x22
	mov	x3, x21
	mov	w4, w23
	blr	x8
	ldr	x8, [x25, #24]!
	cbnz	x8, .LBB33_30
.LBB33_31:
	//APP
	//NO_APP
	ldr	x8, [x24, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x24, #24]
	b.eq	.LBB33_84
// %bb.32:
	ldr	x8, [x24, #24]
	cbz	x8, .LBB33_84
.LBB33_33:
	cmp	x23, #1                 // =1
	b.lt	.LBB33_37
.LBB33_34:
	ldr	w8, [x19, #32]
	tbz	w8, #1, .LBB33_37
// %bb.35:
	ldp	x0, x9, [x19]
	mov	w10, #1
	bic	w3, w10, w8, lsr #2
	sub	x1, x9, x23
	sub	x2, x9, #1              // =1
	bl	vfs_fsync_range
                                        // kill: def $w0 killed $w0 def $x0
	cbz	w0, .LBB33_37
// %bb.36:
	sxtw	x23, w0
.LBB33_37:
	mov	x0, x23
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB33_38:
	ldr	x1, [x19, #8]
	ldr	x2, [x21, #16]
	mov	x0, x20
	bl	f2fs_overwrite_io
	tbz	w0, #0, .LBB33_42
// %bb.39:
	ldr	x8, [x20, #736]
	tbnz	w8, #10, .LBB33_42
// %bb.40:
	ldr	x8, [x20, #40]
	ldr	w24, [x21]
	mov	x0, x19
	mov	x1, x21
	ldr	x23, [x8, #1008]
	bl	fscrypt_dio_supported
	tbz	w0, #0, .LBB33_42
// %bb.41:
	ldr	x8, [x20, #664]
	cbz	x8, .LBB33_51
.LBB33_42:
	add	x8, x20, #736           // =736
	mov	w9, #16777216
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	x0, x22
	bl	up_write
	mov	x23, #-11
	b	.LBB33_27
.LBB33_43:
	mov	x0, x20
	bl	f2fs_convert_inline_inode
	mov	w23, w0
	cbz	w0, .LBB33_45
.LBB33_44:
	add	x8, x20, #736           // =736
	mov	w9, #16777216
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	x0, x22
	bl	up_write
	sxtw	x23, w23
	b	.LBB33_27
.LBB33_45:
	ldr	x8, [x20, #40]
	ldr	w24, [x21]
	mov	x0, x19
	mov	x1, x21
	ldr	x23, [x8, #1008]
	bl	fscrypt_dio_supported
	tbz	w0, #0, .LBB33_22
// %bb.46:
	ldr	x8, [x20, #664]
	cbnz	x8, .LBB33_22
// %bb.47:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	ldr	w9, [x8, #1532]
	cbz	w9, .LBB33_72
// %bb.48:
	ldr	w9, [x20, #1280]
	cbnz	w9, .LBB33_22
// %bb.49:
	ldr	x9, [x20, #736]
	tbnz	x9, #35, .LBB33_22
// %bb.50:
	ldr	x9, [x20, #736]
	tbnz	x9, #34, .LBB33_22
	b	.LBB33_74
.LBB33_51:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
	ldr	w9, [x8, #1532]
	cbz	w9, .LBB33_55
// %bb.52:
	ldr	w9, [x20, #1280]
	cbnz	w9, .LBB33_42
// %bb.53:
	ldr	x9, [x20, #736]
	tbnz	x9, #35, .LBB33_42
// %bb.54:
	ldr	x9, [x20, #736]
	tbnz	x9, #34, .LBB33_42
	b	.LBB33_57
.LBB33_55:
	ldrh	w9, [x20]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB33_57
// %bb.56:
	ldr	x9, [x20, #736]
	tbnz	w9, #31, .LBB33_42
.LBB33_57:
	ldr	x9, [x23, #16]
	ldrb	w9, [x9, #2180]
	tbnz	w9, #1, .LBB33_42
// %bb.58:
	tbz	w24, #0, .LBB33_63
// %bb.59:
	ldr	w9, [x23, #1504]
	cmp	w9, #1                  // =1
	b.ne	.LBB33_63
// %bb.60:
	ldrb	w8, [x20, #142]
	ldr	w24, [x19, #8]
	mov	w9, #-1
	mov	x0, x21
	lsl	w25, w9, w8
	bl	iov_iter_alignment
	orr	w8, w0, w24
	bics	wzr, w8, w25
	b.ne	.LBB33_42
// %bb.61:
	ldr	w8, [x23, #1432]
	mov	w9, #1
	lsl	w8, w9, w8
	cmp	w8, #1                  // =1
	b.gt	.LBB33_42
// %bb.62:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
.LBB33_63:
	ldr	x8, [x8, #80]
	tbnz	w8, #8, .LBB33_42
.LBB33_64:
	mov	w25, wzr
	mov	x24, xzr
.LBB33_65:
	mov	x0, x19
	mov	x1, x21
	bl	__generic_file_write_iter
	add	x8, x20, #736           // =736
	mov	w9, #16777216
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	x23, x0
	cbz	w25, .LBB33_68
// %bb.66:
	ldr	x8, [x20, #80]
	cmp	x8, x24
	b.hs	.LBB33_68
// %bb.67:
	add	x24, x20, #1040         // =1040
	mov	x0, x24
	bl	down_write
	add	x25, x20, #1088         // =1088
	mov	x0, x25
	bl	down_write
	mov	x0, x20
	bl	f2fs_truncate
	mov	x0, x25
	bl	up_write
	mov	x0, x24
	bl	up_write
.LBB33_68:
	cmp	x23, #1                 // =1
	b.lt	.LBB33_26
// %bb.69:
	ldr	x8, [x20, #40]
	ldr	x24, [x8, #1008]
	ldrb	w8, [x24, #2704]
	cbz	w8, .LBB33_26
// %bb.70:
	add	x25, x24, #2344         // =2344
	mov	x0, x25
	bl	_raw_spin_lock
	ldr	x8, [x24, #2368]
	ldr	x9, [x24, #2352]
	mov	x0, x25
	add	x8, x8, x23
	str	x8, [x24, #2368]
	sub	x8, x8, x9
	str	x8, [x24, #2360]
	bl	_raw_spin_unlock
	mov	x0, x24
	bl	f2fs_record_iostat
	b	.LBB33_26
.LBB33_71:
	mov	x23, #-5
	b	.LBB33_27
.LBB33_72:
	ldrh	w9, [x20]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB33_74
// %bb.73:
	ldr	x9, [x20, #736]
	tbnz	w9, #31, .LBB33_22
.LBB33_74:
	ldr	x9, [x23, #16]
	ldrb	w9, [x9, #2180]
	tbnz	w9, #1, .LBB33_22
// %bb.75:
	tbz	w24, #0, .LBB33_80
// %bb.76:
	ldr	w9, [x23, #1504]
	cmp	w9, #1                  // =1
	b.ne	.LBB33_80
// %bb.77:
	ldrb	w8, [x20, #142]
	ldr	w24, [x19, #8]
	mov	w9, #-1
	mov	x0, x21
	lsl	w25, w9, w8
	bl	iov_iter_alignment
	orr	w8, w0, w24
	bics	wzr, w8, w25
	b.ne	.LBB33_22
// %bb.78:
	ldr	w8, [x23, #1432]
	mov	w9, #1
	lsl	w8, w9, w8
	cmp	w8, #1                  // =1
	b.gt	.LBB33_22
// %bb.79:
	ldr	x8, [x20, #40]
	ldr	x8, [x8, #1008]
.LBB33_80:
	ldr	x9, [x8, #80]
	tbnz	w9, #8, .LBB33_22
// %bb.81:
	ldr	w8, [x8, #1504]
	cmp	w8, #1                  // =1
	b.ne	.LBB33_22
// %bb.82:
	ldr	w8, [x21]
	tbz	w8, #0, .LBB33_22
// %bb.83:
	ldrb	w8, [x20, #142]
	ldr	w23, [x19, #8]
	mov	w9, #-1
	mov	x0, x21
	lsl	w24, w9, w8
	bl	iov_iter_alignment
	orr	w8, w0, w23
	bics	wzr, w8, w24
	b.ne	.LBB33_22
	b	.LBB33_64
.LBB33_84:
	bl	preempt_schedule_notrace
	cmp	x23, #1                 // =1
	b.ge	.LBB33_34
	b	.LBB33_37
.LBB33_85:
	mov	x23, #-1
	b	.LBB33_26
.Lfunc_end33:
	.size	f2fs_file_write_iter, .Lfunc_end33-f2fs_file_write_iter
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_file_mmap
	.type	f2fs_file_mmap,@function
f2fs_file_mmap:                         // @f2fs_file_mmap
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	ldr	x19, [x0, #32]
	str	x21, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB34_7
// %bb.1:
	mov	x21, x0
	mov	x0, x19
	mov	x20, x1
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB34_5
// %bb.2:
	ldrb	w8, [x21, #66]
	tbnz	w8, #2, .LBB34_4
// %bb.3:
	add	x0, x21, #16            // =16
	bl	touch_atime
.LBB34_4:
	adrp	x8, f2fs_file_vm_ops
	add	x8, x8, :lo12:f2fs_file_vm_ops
	str	x8, [x20, #144]
	add	x8, x19, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 8589934592
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	mov	w0, wzr
	b	.LBB34_6
.LBB34_5:
	mov	w0, #-95
.LBB34_6:
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB34_7:
	mov	w0, #-5
	b	.LBB34_6
.Lfunc_end34:
	.size	f2fs_file_mmap, .Lfunc_end34-f2fs_file_mmap
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_file_open
	.type	f2fs_file_open,@function
f2fs_file_open:                         // @f2fs_file_open
// %bb.0:
	stp	x29, x30, [sp, #-48]!   // 16-byte Folded Spill
	str	x21, [sp, #16]          // 8-byte Folded Spill
	stp	x20, x19, [sp, #32]     // 16-byte Folded Spill
	mov	x29, sp
	mov	x19, x1
	mov	x20, x0
	bl	fscrypt_file_open
	mov	w21, w0
	cbnz	w0, .LBB35_7
// %bb.1:
	mov	x0, x20
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB35_6
// %bb.2:
	mov	x0, x20
	mov	x1, x19
	bl	fsverity_file_open
	mov	w21, w0
	cbnz	w0, .LBB35_7
// %bb.3:
	ldr	w8, [x19, #68]
	mov	x0, x20
	mov	x1, x19
	orr	w8, w8, #0x8000000
	str	w8, [x19, #68]
	bl	dquot_file_open
	mov	w21, w0
	cbnz	w0, .LBB35_7
// %bb.4:
	ldr	x8, [x20, #736]
	tbnz	x8, #38, .LBB35_8
.LBB35_5:
	mov	w21, wzr
	b	.LBB35_7
.LBB35_6:
	mov	w21, #-95
.LBB35_7:
	mov	w0, w21
	ldp	x20, x19, [sp, #32]     // 16-byte Folded Reload
	ldr	x21, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #48     // 16-byte Folded Reload
	ret
.LBB35_8:
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB35_10
// %bb.9:
	ldr	x8, [x20, #736]
	tbnz	x8, #39, .LBB35_5
.LBB35_10:
	add	x21, x20, #744          // =744
	mov	x0, x21
	bl	down_read
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB35_15
// %bb.11:
	ldr	x8, [x20, #736]
	tbz	x8, #38, .LBB35_13
// %bb.12:
	ldr	x8, [x20, #736]
	tbnz	x8, #39, .LBB35_15
.LBB35_13:
	ldr	x0, [x20, #1304]
	bl	igrab
	mov	x20, x0
	mov	x0, x21
	bl	up_read
	cbz	x20, .LBB35_16
// %bb.14:
	mov	x0, x20
	mov	x1, x19
	bl	f2fs_file_open
	mov	w21, w0
	b	.LBB35_17
.LBB35_15:
	mov	x0, x21
	bl	up_read
	mov	x20, xzr
.LBB35_16:
	mov	w21, wzr
.LBB35_17:
	mov	x0, x20
	bl	put_inner_inode
	b	.LBB35_7
.Lfunc_end35:
	.size	f2fs_file_open, .Lfunc_end35-f2fs_file_open
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_file_flush
	.type	f2fs_file_flush,@function
f2fs_file_flush:                        // @f2fs_file_flush
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldr	x0, [x0, #32]
	mov	x29, sp
	ldr	x8, [x0, #736]
	tbz	w8, #15, .LBB36_3
// %bb.1:
	ldr	x8, [x0, #944]
	//APP
	mrs x9, sp_el0
	//NO_APP
	cmp	x8, x9
	b.ne	.LBB36_3
// %bb.2:
	bl	f2fs_drop_inmem_pages
.LBB36_3:
	mov	w0, wzr
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.Lfunc_end36:
	.size	f2fs_file_flush, .Lfunc_end36-f2fs_file_flush
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_release_file
	.type	f2fs_release_file,@function
f2fs_release_file:                      // @f2fs_release_file
// %bb.0:
	stp	x29, x30, [sp, #-32]!   // 16-byte Folded Spill
	stp	x20, x19, [sp, #16]     // 16-byte Folded Spill
	ldrb	w8, [x1, #68]
	mov	x29, sp
	tbz	w8, #1, .LBB37_6
// %bb.1:
	ldr	w8, [x0, #392]
	mov	x19, x0
	cmp	w8, #1                  // =1
	b.ne	.LBB37_6
// %bb.2:
	ldr	x8, [x19, #736]
	tbz	w8, #15, .LBB37_4
// %bb.3:
	mov	x0, x19
	bl	f2fs_drop_inmem_pages
.LBB37_4:
	ldr	x8, [x19, #736]
	tbz	w8, #17, .LBB37_6
// %bb.5:
	add	x20, x19, #736          // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x20]
1:	ldxr	x8, [x20]
	orr	x8, x8, 524288
	stxr	w9, x8, [x20]
	cbnz	w9, 1b
	//NO_APP
	ldr	x0, [x19, #48]
	bl	filemap_fdatawrite
	mov	w8, #524288
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x20]
1:	ldxr	x10, [x20]
	bic	x10, x10, x8
	stxr	w11, x10, [x20]
	cbnz	w11, 1b
	//NO_APP
	mov	w9, #131072
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x20]
1:	ldxr	x8, [x20]
	bic	x8, x8, x9
	stxr	w10, x8, [x20]
	cbnz	w10, 1b
	//NO_APP
	ldr	x8, [x19, #40]
	mov	w9, #1
	ldr	x8, [x8, #1008]
	add	x8, x8, #2232           // =2232
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w10, [x8]
	sub	w10, w10, w9
	stxr	w11, w10, [x8]
	cbnz	w11, 1b

	//NO_APP
.LBB37_6:
	ldp	x20, x19, [sp, #16]     // 16-byte Folded Reload
	mov	w0, wzr
	ldp	x29, x30, [sp], #32     // 16-byte Folded Reload
	ret
.Lfunc_end37:
	.size	f2fs_release_file, .Lfunc_end37-f2fs_release_file
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_fallocate
	.type	f2fs_fallocate,@function
f2fs_fallocate:                         // @f2fs_fallocate
// %bb.0:
	sub	sp, sp, #272            // =272
	adrp	x8, __stack_chk_guard
	ldr	x8, [x8, :lo12:__stack_chk_guard]
	stp	x29, x30, [sp, #176]    // 16-byte Folded Spill
	add	x29, sp, #176           // =176
	stp	x28, x27, [sp, #192]    // 16-byte Folded Spill
	stp	x26, x25, [sp, #208]    // 16-byte Folded Spill
	stp	x24, x23, [sp, #224]    // 16-byte Folded Spill
	stp	x22, x21, [sp, #240]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #256]    // 16-byte Folded Spill
	stur	x8, [x29, #-16]
	ldr	x21, [x0, #32]
	mov	x23, x0
	ldr	x8, [x21, #40]
	ldr	x0, [x8, #1008]
	ldr	x8, [x0, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB38_70
// %bb.1:
	ldr	x8, [x0, #80]
	mov	x19, x3
	mov	x20, x2
	mov	w22, w1
	tbnz	w8, #8, .LBB38_71
.LBB38_2:
	mov	x0, x21
	bl	f2fs_is_compress_backend_ready
	tbz	w0, #0, .LBB38_6
// %bb.3:
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB38_7
// %bb.4:
	mov	w8, #40
	tst	w22, w8
	b.eq	.LBB38_10
// %bb.5:
	ldr	w8, [x21, #12]
	tbz	w8, #14, .LBB38_10
.LBB38_6:
	mov	x25, #-95
	b	.LBB38_8
.LBB38_7:
	mov	x25, #-22
.LBB38_8:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-16]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB38_205
// %bb.9:
	mov	x0, x25
	ldp	x20, x19, [sp, #256]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #240]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #224]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #208]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #192]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #176]    // 16-byte Folded Reload
	add	sp, sp, #272            // =272
	ret
.LBB38_10:
	ldr	x8, [x23, #32]
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB38_12
// %bb.11:
	ldr	x0, [x8, #40]
	mov	w1, #1
	mov	w2, #1
	bl	__sb_start_write
.LBB38_12:
	add	x24, x21, #160          // =160
	mov	x0, x24
	bl	down_write
	ldr	x8, [x21, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB38_17
// %bb.13:
	ldr	w8, [x21, #1280]
	cbnz	w8, .LBB38_16
// %bb.14:
	ldr	x8, [x21, #736]
	tbnz	x8, #35, .LBB38_16
// %bb.15:
	ldr	x8, [x21, #736]
	ubfx	x8, x8, #34, #1
	b	.LBB38_19
.LBB38_16:
	mov	w8, #1
	b	.LBB38_19
.LBB38_17:
	ldrh	w8, [x21]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB38_22
// %bb.18:
	ldr	x8, [x21, #736]
	ubfx	x8, x8, #31, #1
.LBB38_19:
	mov	w9, #58
	tst	w22, w9
	b.eq	.LBB38_22
// %bb.20:
	cbz	w8, .LBB38_22
// %bb.21:
	mov	w1, #2
	mov	x0, x23
	mov	w2, w22
	bl	f2fs_decompress_file_internal
	mov	x25, x0
	cbnz	x0, .LBB38_171
.LBB38_22:
	mov	w8, #-60
	tst	w22, w8
	b.eq	.LBB38_24
// %bb.23:
	mov	x25, #-95
	b	.LBB38_171
.LBB38_24:
	ldr	x8, [x21, #736]
	tbz	x8, #44, .LBB38_26
// %bb.25:
	add	x8, x21, #736           // =736
	mov	x9, #17592186044416
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_mark_inode_dirty_sync
.LBB38_26:
	ldr	x8, [x21, #736]
	tbz	x8, #38, .LBB38_31
// %bb.27:
	ldr	x8, [x21, #736]
	tbz	x8, #38, .LBB38_29
// %bb.28:
	ldr	x8, [x21, #736]
	tbnz	x8, #39, .LBB38_31
.LBB38_29:
	adrp	x1, .L__func__.f2fs_fallocate
	add	x1, x1, :lo12:.L__func__.f2fs_fallocate
	mov	x0, x21
	bl	f2fs_revoke_deduped_inode
	cbz	w0, .LBB38_31
// %bb.30:
	mov	x25, #-5
	b	.LBB38_171
.LBB38_31:
	tbnz	w22, #1, .LBB38_40
// %bb.32:
	tbnz	w22, #3, .LBB38_44
// %bb.33:
	tbnz	w22, #4, .LBB38_48
// %bb.34:
	ldr	x8, [x21, #40]
	ldr	x25, [x8, #1008]
	tbnz	w22, #5, .LBB38_53
// %bb.35:
	add	x26, x19, x20
	mov	w8, #8
	mov	w9, #1
	mov	x0, x21
	mov	x1, x26
	stp	xzr, xzr, [x29, #-40]
	stp	xzr, xzr, [x29, #-56]
	stur	xzr, [x29, #-64]
	stur	w8, [x29, #-32]
	sturb	w9, [x29, #-28]
	bl	inode_newsize_ok
	mov	w27, w0
	cbnz	w0, .LBB38_42
// %bb.36:
	mov	x0, x21
	bl	f2fs_convert_inline_inode
	mov	w27, w0
	cbnz	w0, .LBB38_42
// %bb.37:
	mov	w1, #1
	mov	x0, x25
	bl	f2fs_balance_fs
	lsr	x10, x20, #12
	lsr	x9, x26, #12
	tst	x26, #0xfff
	sub	w8, w9, w10
	cinc	w8, w8, ne
	stp	w10, w8, [x29, #-60]
	cbz	w8, .LBB38_73
// %bb.38:
	stp	x10, x9, [sp, #64]      // 16-byte Folded Spill
	str	x26, [sp, #48]          // 8-byte Folded Spill
	ldr	x9, [x21, #736]
	tbnz	w9, #28, .LBB38_129
// %bb.39:
	sub	x1, x29, #64            // =64
	mov	w2, #1
	mov	w3, #5
	mov	x0, x21
	bl	f2fs_map_blocks
	ldur	w26, [x29, #-56]
	mov	w27, w0
	str	x19, [sp, #56]          // 8-byte Folded Spill
	b	.LBB38_142
.LBB38_40:
	ldr	x8, [x21, #80]
	cmp	x8, x20
	b.le	.LBB38_47
// %bb.41:
	mov	x0, x21
	mov	x1, x20
	mov	x2, x19
	bl	punch_hole
	mov	w27, w0
.LBB38_42:
	cbnz	w27, .LBB38_170
.LBB38_43:
	mov	x0, x21
	bl	current_time
	stp	x0, x1, [x21, #120]
	stp	x0, x1, [x21, #104]
	mov	x0, x21
	mov	w1, wzr
	bl	f2fs_mark_inode_dirty_sync
	ldr	x8, [x21, #40]
	adrp	x9, jiffies
	mov	x25, xzr
	ldr	x8, [x8, #1008]
	ldr	x9, [x9, :lo12:jiffies]
	stp	x9, x9, [x8, #504]
	str	x9, [x8, #520]
	b	.LBB38_171
.LBB38_44:
	ldr	x8, [x21, #80]
	add	x25, x19, x20
	cmp	x25, x8
	b.ge	.LBB38_46
// %bb.45:
	orr	w8, w19, w20
	tst	x8, #0xfff
	b.eq	.LBB38_61
.LBB38_46:
	mov	w27, #-22
	b	.LBB38_170
.LBB38_47:
	mov	x25, xzr
	b	.LBB38_171
.LBB38_48:
	ldp	x8, x28, [x21, #40]
	ldr	x26, [x21, #80]
	add	x25, x19, x20
	mov	x0, x21
	ldr	x8, [x8, #1008]
	mov	x1, x25
	stur	x8, [x29, #-72]         // 8-byte Folded Spill
	bl	inode_newsize_ok
	mov	w27, w0
	cbnz	w0, .LBB38_170
// %bb.49:
	mov	x0, x21
	bl	f2fs_convert_inline_inode
	mov	w27, w0
	cbnz	w0, .LBB38_170
// %bb.50:
	sub	x2, x25, #1             // =1
	mov	x0, x28
	mov	x1, x20
	bl	filemap_write_and_wait_range
	mov	w27, w0
	cbnz	w0, .LBB38_170
// %bb.51:
	lsr	x1, x20, #12
	lsr	x8, x25, #12
	and	x2, x20, #0xfff
	str	x8, [sp, #88]           // 8-byte Folded Spill
	cmp	x1, x8
	and	x8, x25, #0xfff
	b.ne	.LBB38_74
// %bb.52:
	sub	x3, x8, x2
	mov	x0, x21
	bl	fill_zero
	mov	w27, w0
	cbnz	w0, .LBB38_170
	b	.LBB38_153
.LBB38_53:
	ldr	x8, [x21, #80]
	mov	x0, x21
	add	x26, x8, x19
	mov	x1, x26
	bl	inode_newsize_ok
	mov	w27, w0
	cbnz	w0, .LBB38_170
// %bb.54:
	ldr	x8, [x21, #80]
	cmp	x8, x20
	b.le	.LBB38_46
// %bb.55:
	orr	w8, w19, w20
	tst	x8, #0xfff
	b.ne	.LBB38_46
// %bb.56:
	mov	x0, x21
	bl	f2fs_convert_inline_inode
	mov	w27, w0
	cbnz	w0, .LBB38_170
// %bb.57:
	mov	w1, #1
	mov	x0, x25
	bl	f2fs_balance_fs
	add	x0, x21, #1088          // =1088
	str	x0, [sp, #64]           // 8-byte Folded Spill
	bl	down_write
	ldr	x8, [x21, #40]
	ldr	x9, [x21, #80]
	ldr	x8, [x8, #1008]
	stur	x9, [x29, #-72]         // 8-byte Folded Spill
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB38_165
// %bb.58:
	ldr	w8, [x21, #1280]
	cbnz	w8, .LBB38_167
// %bb.59:
	ldr	x8, [x21, #736]
	tbnz	x8, #35, .LBB38_167
// %bb.60:
	ldr	x8, [x21, #736]
	ldur	x28, [x29, #-72]        // 8-byte Folded Reload
	tbnz	x8, #34, .LBB38_167
	b	.LBB38_168
.LBB38_61:
	mov	x0, x21
	bl	f2fs_convert_inline_inode
	mov	w27, w0
	cbnz	w0, .LBB38_170
// %bb.62:
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, x20
	bl	filemap_write_and_wait_range
	mov	w27, w0
	cbnz	w0, .LBB38_170
// %bb.63:
	ldr	x8, [x21, #40]
	ldr	x9, [x21, #80]
	mov	w1, #1
	asr	x28, x25, #12
	ldr	x27, [x8, #1008]
	add	x8, x9, #4095           // =4095
	lsr	x8, x8, #12
	str	x8, [sp, #88]           // 8-byte Folded Spill
	asr	x8, x20, #12
	mov	x0, x27
	stur	x8, [x29, #-80]         // 8-byte Folded Spill
	bl	f2fs_balance_fs
	add	x26, x21, #1040         // =1040
	mov	x0, x26
	bl	down_write
	add	x0, x21, #1088          // =1088
	stur	x0, [x29, #-72]         // 8-byte Folded Spill
	bl	down_write
	add	x25, x27, #368          // =368
	mov	x0, x25
	bl	_raw_spin_lock
	ldr	x8, [x27, #352]
	cbz	x8, .LBB38_65
// %bb.64:
	adrp	x9, jiffies
	ldr	x8, [x27, #360]
	ldr	x9, [x9, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB38_198
.LBB38_65:
	mov	x0, x25
	bl	_raw_spin_unlock
	add	x25, x27, #304          // =304
	mov	x0, x25
	bl	down_read
	mov	x0, x21
	bl	f2fs_drop_extent_tree
	mov	x0, x21
	mov	x1, x20
	bl	truncate_pagecache
	ldr	x8, [sp, #88]           // 8-byte Folded Reload
	ldur	x3, [x29, #-80]         // 8-byte Folded Reload
	mov	w5, #1
	mov	x0, x21
	sub	x4, x8, x28
	mov	x1, x21
	mov	x2, x28
	bl	__exchange_data_block
	mov	w27, w0
	mov	x0, x25
	bl	up_read
	ldur	x25, [x29, #-72]        // 8-byte Folded Reload
	mov	x0, x25
	bl	up_write
	mov	x0, x26
	bl	up_write
	cbnz	w27, .LBB38_170
// %bb.66:
	mov	x0, x25
	bl	down_write
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, x20
	bl	filemap_write_and_wait_range
	mov	x0, x21
	mov	x1, x20
	bl	truncate_pagecache
	ldr	x8, [x21, #40]
	ldr	x9, [x21, #80]
	ldr	x8, [x8, #1008]
	sub	x26, x9, x19
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB38_154
// %bb.67:
	ldr	w8, [x21, #1280]
	cbnz	w8, .LBB38_156
// %bb.68:
	ldr	x8, [x21, #736]
	tbnz	x8, #35, .LBB38_156
// %bb.69:
	ldr	x8, [x21, #736]
	mov	x25, x26
	tbnz	x8, #34, .LBB38_156
	b	.LBB38_157
.LBB38_70:
	mov	x25, #-5
	b	.LBB38_8
.LBB38_71:
	mov	w1, wzr
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB38_2
// %bb.72:
	mov	x25, #-28
	b	.LBB38_8
.LBB38_73:
	mov	w27, wzr
	b	.LBB38_42
.LBB38_74:
	str	x8, [sp, #8]            // 8-byte Folded Spill
	cbz	x2, .LBB38_77
// %bb.75:
	mov	w8, #4096
	sub	x3, x8, x2
	mov	x0, x21
	mov	x28, x1
	bl	fill_zero
	mov	w27, w0
	cbnz	w0, .LBB38_170
// %bb.76:
	add	x28, x28, #1            // =1
	lsl	x8, x28, #12
	cmp	x26, x8
	mov	x1, x28
	csel	x26, x26, x8, gt
.LBB38_77:
	ldr	x8, [sp, #88]           // 8-byte Folded Reload
	cmp	x1, x8
	b.hs	.LBB38_151
// %bb.78:
	ldur	x28, [x29, #-72]        // 8-byte Folded Reload
	add	x8, x21, #1040          // =1040
	str	x8, [sp, #56]           // 8-byte Folded Spill
	add	x8, x21, #1088          // =1088
	str	x8, [sp, #48]           // 8-byte Folded Spill
	and	x8, x25, #0xfffffffffffff000
	add	x9, x28, #368           // =368
	str	x9, [sp, #64]           // 8-byte Folded Spill
	add	x9, x28, #304           // =304
	sub	x8, x8, #1              // =1
	mov	x27, x1
	str	x9, [sp, #40]           // 8-byte Folded Spill
	//APP
	mrs x9, sp_el0
	//NO_APP
	str	x9, [sp]                // 8-byte Folded Spill
	str	x8, [sp, #24]           // 8-byte Folded Spill
.LBB38_79:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB38_94 Depth 2
                                        //     Child Loop BB38_104 Depth 2
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	stp	xzr, xzr, [x29, #-32]
	stp	xzr, xzr, [x29, #-48]
	stp	xzr, xzr, [x29, #-64]
	bl	down_write
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	bl	down_write
	ldr	x2, [sp, #24]           // 8-byte Folded Reload
	lsl	x1, x27, #12
	mov	x0, x21
	bl	truncate_pagecache_range
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	_raw_spin_lock
	ldr	x8, [x28, #352]
	cbz	x8, .LBB38_81
// %bb.80:                              //   in Loop: Header=BB38_79 Depth=1
	adrp	x9, jiffies
	ldr	x8, [x28, #360]
	ldr	x9, [x9, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB38_128
.LBB38_81:                              //   in Loop: Header=BB38_79 Depth=1
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	_raw_spin_unlock
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	bl	down_read
	sub	x0, x29, #64            // =64
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	sub	x0, x29, #64            // =64
	mov	x1, x27
	mov	w2, wzr
	stp	x21, xzr, [x29, #-64]
	stur	xzr, [x29, #-48]
	stur	wzr, [x29, #-40]
	str	x27, [sp, #80]          // 8-byte Folded Spill
	bl	f2fs_get_dnode_of_data
	cbnz	w0, .LBB38_191
// %bb.82:                              //   in Loop: Header=BB38_79 Depth=1
	ldur	x8, [x29, #-48]
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	add	x8, x8, x9
	lsl	x8, x8, #6
	and	x12, x8, #0xfffffffffffff000
	mov	x8, #-549755813888
	mov	x9, #-549755813888
	movk	x8, #4072
	movk	x9, #4076
	ldr	w8, [x12, x8]
	ldr	w9, [x12, x9]
	cmp	w8, w9
	b.ne	.LBB38_86
// %bb.83:                              //   in Loop: Header=BB38_79 Depth=1
	ldr	w10, [x21, #1184]
	ldr	w11, [x21, #1192]
	ldrh	w13, [x21]
	ldr	x14, [sp, #80]          // 8-byte Folded Reload
	add	w10, w11, w10, asr #2
	and	w11, w13, #0xf000
	cmp	w11, #8, lsl #12        // =32768
	mov	w11, #923
	sub	w11, w11, w10
	b.ne	.LBB38_90
// %bb.84:                              //   in Loop: Header=BB38_79 Depth=1
	ldr	x10, [x21, #736]
	tbz	w10, #31, .LBB38_90
// %bb.85:                              //   in Loop: Header=BB38_79 Depth=1
	ldr	w10, [x21, #1292]
	neg	w10, w10
	and	w11, w11, w10
	b	.LBB38_90
.LBB38_86:                              //   in Loop: Header=BB38_79 Depth=1
	ldrh	w10, [x21]
	and	w10, w10, #0xf000
	cmp	w10, #8, lsl #12        // =32768
	b.ne	.LBB38_88
// %bb.87:                              //   in Loop: Header=BB38_79 Depth=1
	ldr	x10, [x21, #736]
	tbnz	w10, #31, .LBB38_115
.LBB38_88:                              //   in Loop: Header=BB38_79 Depth=1
	mov	w11, #1018
.LBB38_89:                              //   in Loop: Header=BB38_79 Depth=1
	ldr	x14, [sp, #80]          // 8-byte Folded Reload
.LBB38_90:                              //   in Loop: Header=BB38_79 Depth=1
	ldur	x10, [x29, #-64]
	ldur	w28, [x29, #-36]
	mov	x1, xzr
	ldr	x13, [x10, #40]
	sub	w11, w11, w28
	add	x15, x14, x11
	ldr	x11, [sp, #88]          // 8-byte Folded Reload
	ldr	x13, [x13, #1008]
	str	x15, [sp, #32]          // 8-byte Folded Spill
	cmp	x11, x15
	csel	x11, x11, x15, lo
	stur	x11, [x29, #-80]        // 8-byte Folded Spill
	subs	x11, x14, x11
	str	x13, [sp, #72]          // 8-byte Folded Spill
	b.hs	.LBB38_100
// %bb.91:                              //   in Loop: Header=BB38_79 Depth=1
	mov	x13, #-549755813888
	add	x12, x12, x13
	add	x13, x12, #360          // =360
	cmp	w8, w9
	csel	x13, x13, x12, eq
	mov	w14, w28
	b	.LBB38_94
.LBB38_92:                              //   in Loop: Header=BB38_94 Depth=2
	mov	w15, wzr
.LBB38_93:                              //   in Loop: Header=BB38_94 Depth=2
	add	w15, w15, w14
	ldr	w15, [x13, w15, uxtw #2]
	add	w14, w14, #1            // =1
	stur	w14, [x29, #-36]
	cmp	w15, #0                 // =0
	cinc	x1, x1, eq
	adds	x11, x11, #1            // =1
	b.hs	.LBB38_100
.LBB38_94:                              //   Parent Loop BB38_79 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	cmp	w8, w9
	b.ne	.LBB38_92
// %bb.95:                              //   in Loop: Header=BB38_94 Depth=2
	cbz	x10, .LBB38_98
// %bb.96:                              //   in Loop: Header=BB38_94 Depth=2
	ldr	x15, [x10, #736]
	tbz	w15, #26, .LBB38_92
// %bb.97:                              //   in Loop: Header=BB38_94 Depth=2
	ldr	w15, [x10, #1184]
	asr	w15, w15, #2
	b	.LBB38_93
.LBB38_98:                              //   in Loop: Header=BB38_94 Depth=2
	ldrb	w15, [x12, #3]
	tbz	w15, #5, .LBB38_92
// %bb.99:                              //   in Loop: Header=BB38_94 Depth=2
	ldrh	w15, [x12, #360]
	lsr	w15, w15, #2
	b	.LBB38_93
.LBB38_100:                             //   in Loop: Header=BB38_79 Depth=1
	sub	x0, x29, #64            // =64
	stur	w28, [x29, #-36]
	bl	f2fs_reserve_new_blocks
	mov	w27, w0
	cbnz	w0, .LBB38_119
// %bb.101:                             //   in Loop: Header=BB38_79 Depth=1
	str	x26, [sp, #16]          // 8-byte Folded Spill
	ldr	x26, [sp, #80]          // 8-byte Folded Reload
	ldur	x8, [x29, #-80]         // 8-byte Folded Reload
	stur	w28, [x29, #-36]
	cmp	x26, x8
	b.hs	.LBB38_117
// %bb.102:                             //   in Loop: Header=BB38_79 Depth=1
	mov	x27, #2097152
	movk	x27, #1, lsl #32
	b	.LBB38_104
.LBB38_103:                             //   in Loop: Header=BB38_104 Depth=2
	ldur	x8, [x29, #-80]         // 8-byte Folded Reload
	add	x26, x26, #1            // =1
	add	w28, w28, #1            // =1
	stur	w28, [x29, #-36]
	cmp	x8, x26
	b.eq	.LBB38_116
.LBB38_104:                             //   Parent Loop BB38_79 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	x8, [x29, #-48]
	mov	x9, #-549755813888
	movk	x9, #4076
	mov	x11, #-549755813888
	add	x8, x8, x27
	lsl	x8, x8, #6
	and	x10, x8, #0xfffffffffffff000
	mov	x8, #-549755813888
	movk	x8, #4072
	ldr	w8, [x10, x8]
	ldr	w9, [x10, x9]
	add	x10, x10, x11
	cmp	w8, w9
	b.ne	.LBB38_109
// %bb.105:                             //   in Loop: Header=BB38_104 Depth=2
	ldur	x11, [x29, #-64]
	cbz	x11, .LBB38_108
// %bb.106:                             //   in Loop: Header=BB38_104 Depth=2
	ldr	x12, [x11, #736]
	tbz	w12, #26, .LBB38_109
// %bb.107:                             //   in Loop: Header=BB38_104 Depth=2
	ldr	w11, [x11, #1184]
	asr	w11, w11, #2
	b	.LBB38_110
.LBB38_108:                             //   in Loop: Header=BB38_104 Depth=2
	ldrb	w11, [x10, #3]
	tbnz	w11, #5, .LBB38_113
.LBB38_109:                             //   in Loop: Header=BB38_104 Depth=2
	mov	w11, wzr
.LBB38_110:                             //   in Loop: Header=BB38_104 Depth=2
	add	x12, x10, #360          // =360
	cmp	w8, w9
	csel	x8, x12, x10, eq
	add	w9, w11, w28
	ldr	w1, [x8, w9, uxtw #2]
	cmn	w1, #1                  // =1
	stur	w1, [x29, #-28]
	b.eq	.LBB38_103
// %bb.111:                             //   in Loop: Header=BB38_104 Depth=2
	cbz	w1, .LBB38_114
// %bb.112:                             //   in Loop: Header=BB38_104 Depth=2
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	bl	f2fs_invalidate_blocks
	mov	w8, #-1
	sub	x0, x29, #64            // =64
	stur	w8, [x29, #-28]
	bl	f2fs_set_data_blkaddr
	ldur	w28, [x29, #-36]
	b	.LBB38_103
.LBB38_113:                             //   in Loop: Header=BB38_104 Depth=2
	ldrh	w11, [x10, #360]
	lsr	w11, w11, #2
	b	.LBB38_110
.LBB38_114:                             //   in Loop: Header=BB38_79 Depth=1
	mov	w27, #-28
	b	.LBB38_118
.LBB38_115:                             //   in Loop: Header=BB38_79 Depth=1
	ldr	w10, [x21, #1292]
	mov	w11, #1018
	neg	w10, w10
	and	w11, w10, w11
	b	.LBB38_89
.LBB38_116:                             //   in Loop: Header=BB38_79 Depth=1
	ldur	x26, [x29, #-80]        // 8-byte Folded Reload
.LBB38_117:                             //   in Loop: Header=BB38_79 Depth=1
	mov	w27, wzr
.LBB38_118:                             //   in Loop: Header=BB38_79 Depth=1
	ldr	x1, [sp, #80]           // 8-byte Folded Reload
	sub	x0, x29, #64            // =64
	mov	w2, wzr
	sub	w3, w26, w1
	bl	f2fs_update_extent_cache_range
	ldr	x26, [sp, #16]          // 8-byte Folded Reload
.LBB38_119:                             //   in Loop: Header=BB38_79 Depth=1
	ldur	x0, [x29, #-48]
	cbz	x0, .LBB38_121
// %bb.120:                             //   in Loop: Header=BB38_79 Depth=1
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-48]
	b	.LBB38_122
.LBB38_121:                             //   in Loop: Header=BB38_79 Depth=1
	mov	x9, xzr
.LBB38_122:                             //   in Loop: Header=BB38_79 Depth=1
	ldur	x28, [x29, #-72]        // 8-byte Folded Reload
	ldur	x8, [x29, #-56]
	cbz	x8, .LBB38_126
// %bb.123:                             //   in Loop: Header=BB38_79 Depth=1
	cmp	x9, x8
	b.eq	.LBB38_126
// %bb.124:                             //   in Loop: Header=BB38_79 Depth=1
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
	cbnz	w9, .LBB38_126
// %bb.125:                             //   in Loop: Header=BB38_79 Depth=1
	bl	__put_page
.LBB38_126:                             //   in Loop: Header=BB38_79 Depth=1
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	stp	xzr, xzr, [x29, #-56]
	bl	up_read
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	bl	up_write
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	up_write
	ldurb	w1, [x29, #-31]
	mov	x0, x28
	bl	f2fs_balance_fs
	cbnz	w27, .LBB38_192
// %bb.127:                             //   in Loop: Header=BB38_79 Depth=1
	ldur	x27, [x29, #-80]        // 8-byte Folded Reload
	ldr	x9, [sp, #32]           // 8-byte Folded Reload
	lsl	x8, x27, #12
	cmp	x26, x8
	csel	x26, x26, x8, gt
	ldr	x8, [sp, #88]           // 8-byte Folded Reload
	cmp	x8, x9
	b.hi	.LBB38_79
	b	.LBB38_151
.LBB38_128:                             //   in Loop: Header=BB38_79 Depth=1
	ldr	x0, [sp]                // 8-byte Folded Reload
	mov	x1, xzr
	bl	show_stack
	adrp	x8, jiffies
	ldr	x1, [x28, #360]
	ldr	x2, [x8, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x28, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB38_81
.LBB38_129:
	ldr	w9, [x25, #1192]
	ldr	w10, [x25, #1188]
	add	x11, x25, #1664         // =1664
	str	x11, [sp, #80]          // 8-byte Folded Spill
	add	x11, x25, #2040         // =2040
	stur	x11, [x29, #-72]        // 8-byte Folded Spill
	add	x11, x25, #368          // =368
	stur	x11, [x29, #-80]        // 8-byte Folded Spill
	add	x11, x25, #304          // =304
	str	x11, [sp, #88]          // 8-byte Folded Spill
	udiv	w11, w8, w10
	mul	w9, w10, w9
	msub	w11, w11, w10, w8
	add	w8, w8, w9
	cmp	w11, #0                 // =0
	sub	w8, w8, #1              // =1
	udiv	w8, w8, w9
	csel	w10, wzr, w10, eq
	mov	w26, wzr
	mul	w28, w8, w9
	add	x8, x10, x19
	//APP
	mrs x11, sp_el0
	//NO_APP
	str	x11, [sp, #40]          // 8-byte Folded Spill
	stur	w9, [x29, #-56]
	str	x8, [sp, #56]           // 8-byte Folded Spill
.LBB38_130:                             // =>This Inner Loop Header: Depth=1
	ldr	x8, [x25, #136]
	ldr	w8, [x8, #104]
	cmn	w8, #1                  // =1
	b.eq	.LBB38_132
// %bb.131:                             //   in Loop: Header=BB38_130 Depth=1
	ldr	w9, [x25, #1192]
	udiv	w1, w8, w9
	b	.LBB38_133
.LBB38_132:                             //   in Loop: Header=BB38_130 Depth=1
	mov	w1, #-1
.LBB38_133:                             //   in Loop: Header=BB38_130 Depth=1
	mov	x0, x25
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB38_136
// %bb.134:                             //   in Loop: Header=BB38_130 Depth=1
	ldr	x0, [sp, #80]           // 8-byte Folded Reload
	bl	down_write
	mov	w1, #1
	mov	w4, #-1
	mov	x0, x25
	mov	w2, wzr
	mov	w3, wzr
	bl	f2fs_gc
	add	w8, w0, #61             // =61
	mov	w27, w0
	cmp	w8, #61                 // =61
	b.hi	.LBB38_143
// %bb.135:                             //   in Loop: Header=BB38_130 Depth=1
	mov	w9, #1
	lsl	x8, x9, x8
	mov	x9, #1
	movk	x9, #8196, lsl #48
	tst	x8, x9
	b.eq	.LBB38_143
.LBB38_136:                             //   in Loop: Header=BB38_130 Depth=1
	ldur	x0, [x29, #-72]         // 8-byte Folded Reload
	bl	down_write
	ldur	x0, [x29, #-80]         // 8-byte Folded Reload
	bl	_raw_spin_lock
	ldr	x8, [x25, #352]
	cbz	x8, .LBB38_138
// %bb.137:                             //   in Loop: Header=BB38_130 Depth=1
	adrp	x9, jiffies
	ldr	x8, [x25, #360]
	ldr	x9, [x9, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB38_140
.LBB38_138:                             //   in Loop: Header=BB38_130 Depth=1
	ldur	x0, [x29, #-80]         // 8-byte Folded Reload
	bl	_raw_spin_unlock
	ldr	x27, [sp, #88]          // 8-byte Folded Reload
	mov	x0, x27
	bl	down_read
	mov	w1, #6
	mov	x0, x25
	mov	w2, wzr
	bl	f2fs_allocate_new_section
	mov	x0, x27
	bl	up_read
	mov	w8, #6
	sub	x1, x29, #64            // =64
	mov	w2, #1
	mov	w3, #4
	mov	x0, x21
	stur	w8, [x29, #-32]
	bl	f2fs_map_blocks
	mov	w27, w0
	ldur	x0, [x29, #-72]         // 8-byte Folded Reload
	bl	up_write
	ldp	w9, w8, [x29, #-60]
	add	w26, w8, w26
	add	w9, w9, w8
	stur	w9, [x29, #-60]
	cbnz	w27, .LBB38_141
// %bb.139:                             //   in Loop: Header=BB38_130 Depth=1
	sub	w28, w28, w8
	cbnz	w28, .LBB38_130
	b	.LBB38_141
.LBB38_140:                             //   in Loop: Header=BB38_130 Depth=1
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	mov	x1, xzr
	bl	show_stack
	adrp	x8, jiffies
	ldr	x1, [x25, #360]
	ldr	x2, [x8, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x25, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB38_138
.LBB38_141:
	stur	w26, [x29, #-56]
.LBB38_142:
	cbz	w27, .LBB38_145
.LBB38_143:
	cbz	w26, .LBB38_42
// %bb.144:
	ldp	x9, x8, [sp, #56]       // 16-byte Folded Reload
	ldr	x11, [sp, #72]          // 8-byte Folded Reload
	add	x8, x8, w26, uxtw
	sub	x10, x8, #1             // =1
	add	x9, x9, x20
	lsl	x8, x8, #12
	cmp	x10, x11
	csel	x9, x9, x8, eq
	b	.LBB38_146
.LBB38_145:
	ldr	x9, [sp, #48]           // 8-byte Folded Reload
.LBB38_146:
	ldr	x8, [x21, #80]
	cmp	x9, x8
	b.le	.LBB38_42
// %bb.147:
	tbnz	w22, #0, .LBB38_197
// %bb.148:
	ldr	x26, [x21, #736]
	ldr	x25, [x21, #736]
	mov	w1, #1
	mov	x0, x21
	str	x9, [x21, #80]
	bl	f2fs_mark_inode_dirty_sync
	tbz	w26, #1, .LBB38_150
// %bb.149:
	tbz	w25, #2, .LBB38_42
.LBB38_150:
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB38_42
.LBB38_151:
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	cbz	x8, .LBB38_190
// %bb.152:
	ldr	x1, [sp, #88]           // 8-byte Folded Reload
	ldr	x3, [sp, #8]            // 8-byte Folded Reload
	mov	x0, x21
	mov	x2, xzr
	bl	fill_zero
	mov	w27, w0
	cbnz	w0, .LBB38_192
.LBB38_153:
	cmp	x26, x25
	csel	x26, x26, x25, gt
	b	.LBB38_192
.LBB38_154:
	ldrh	w8, [x21]
	mov	x25, x26
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB38_157
// %bb.155:
	ldr	x8, [x21, #736]
	mov	x25, x26
	tbz	w8, #31, .LBB38_157
.LBB38_156:
	ldr	w8, [x21, #1292]
	sub	x9, x26, #1             // =1
	lsl	w8, w8, #12
	sub	w8, w8, #1              // =1
	orr	x8, x9, x8
	add	x25, x8, #1             // =1
.LBB38_157:
	mov	w2, #1
	mov	x0, x21
	mov	x1, x25
	bl	f2fs_do_truncate_blocks
	mov	w27, w0
	cbz	w0, .LBB38_159
.LBB38_158:
	ldur	x0, [x29, #-72]         // 8-byte Folded Reload
	bl	up_write
	b	.LBB38_170
.LBB38_159:
	cmp	x25, x26
	b.eq	.LBB38_161
// %bb.160:
	mov	w2, #1
	mov	x0, x21
	mov	x1, x26
	bl	f2fs_truncate_partial_cluster
	mov	w27, w0
	cbnz	w0, .LBB38_158
.LBB38_161:
	ldur	x0, [x29, #-72]         // 8-byte Folded Reload
	bl	up_write
	ldr	x27, [x21, #736]
	ldr	x25, [x21, #736]
	ldr	x8, [x21, #80]
	cmp	x8, x26
	b.eq	.LBB38_43
// %bb.162:
	mov	w1, #1
	mov	x0, x21
	str	x26, [x21, #80]
	bl	f2fs_mark_inode_dirty_sync
	tbz	w27, #1, .LBB38_164
// %bb.163:
	tbz	w25, #2, .LBB38_43
.LBB38_164:
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB38_43
.LBB38_165:
	ldrh	w8, [x21]
	ldur	x28, [x29, #-72]        // 8-byte Folded Reload
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB38_168
// %bb.166:
	ldr	x8, [x21, #736]
	ldur	x28, [x29, #-72]        // 8-byte Folded Reload
	tbz	w8, #31, .LBB38_168
.LBB38_167:
	ldr	w8, [x21, #1292]
	ldur	x9, [x29, #-72]         // 8-byte Folded Reload
	lsl	w8, w8, #12
	sub	x9, x9, #1              // =1
	sub	w8, w8, #1              // =1
	orr	x8, x9, x8
	add	x28, x8, #1             // =1
.LBB38_168:
	mov	w2, #1
	mov	x0, x21
	mov	x1, x28
	bl	f2fs_do_truncate_blocks
	mov	w27, w0
	cbz	w0, .LBB38_180
.LBB38_169:
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_write
.LBB38_170:
	sxtw	x25, w27
.LBB38_171:
	mov	x0, x24
	bl	up_write
	ldr	x8, [x23, #32]
	ldrh	w9, [x8]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB38_173
// %bb.172:
	ldr	x0, [x8, #40]
	mov	w1, #1
	bl	__sb_end_write
.LBB38_173:
	adrp	x8, __tracepoint_f2fs_fallocate+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_fallocate+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB38_8
// %bb.174:
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
	tbz	w8, #0, .LBB38_8
// %bb.175:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w8, [x23, #24]
	add	w8, w8, #1              // =1
	str	w8, [x23, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_fallocate+32
	ldr	x24, [x8, :lo12:__tracepoint_f2fs_fallocate+32]
	cbz	x24, .LBB38_177
.LBB38_176:                             // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x24]
	mov	x1, x21
	mov	w2, w22
	mov	x3, x20
	mov	x4, x19
	mov	w5, w25
	blr	x8
	ldr	x8, [x24, #24]!
	cbnz	x8, .LBB38_176
.LBB38_177:
	//APP
	//NO_APP
	ldr	x8, [x23, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x23, #24]
	b.eq	.LBB38_179
// %bb.178:
	ldr	x8, [x23, #24]
	cbnz	x8, .LBB38_8
.LBB38_179:
	bl	preempt_schedule_notrace
	b	.LBB38_8
.LBB38_180:
	ldur	x8, [x29, #-72]         // 8-byte Folded Reload
	cmp	x28, x8
	b.eq	.LBB38_182
// %bb.181:
	ldur	x1, [x29, #-72]         // 8-byte Folded Reload
	mov	w2, #1
	mov	x0, x21
	bl	f2fs_truncate_partial_cluster
	mov	w27, w0
	cbnz	w0, .LBB38_169
.LBB38_182:
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	up_write
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, x20
	bl	filemap_write_and_wait_range
	mov	w27, w0
	cbnz	w0, .LBB38_170
// %bb.183:
	ldr	x8, [x21, #80]
	add	x0, x21, #1040          // =1040
	asr	x27, x20, #12
	str	x0, [sp, #48]           // 8-byte Folded Spill
	add	x8, x8, #4095           // =4095
	lsr	x28, x8, #12
	bl	down_write
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	down_write
	mov	x0, x21
	mov	x1, x20
	bl	truncate_pagecache
	cmp	x28, x27
	stp	x27, x28, [x29, #-80]   // 16-byte Folded Spill
	b.ls	.LBB38_199
// %bb.184:
	add	x9, x25, #368           // =368
	str	x9, [sp, #88]           // 8-byte Folded Spill
	add	x9, x25, #304           // =304
	str	x9, [sp, #80]           // 8-byte Folded Spill
	//APP
	mrs x9, sp_el0
	//NO_APP
	str	x9, [sp, #40]           // 8-byte Folded Spill
	ldur	x9, [x29, #-80]         // 8-byte Folded Reload
	add	x8, x19, x20
	asr	x8, x8, #12
	sub	x8, x8, x9
	str	x8, [sp, #72]           // 8-byte Folded Spill
.LBB38_185:                             // =>This Inner Loop Header: Depth=1
	ldp	x8, x9, [x29, #-80]     // 16-byte Folded Reload
	ldr	x10, [sp, #72]          // 8-byte Folded Reload
	ldr	x0, [sp, #88]           // 8-byte Folded Reload
	sub	x8, x9, x8
	cmp	x8, x10
	csel	x27, x10, x8, hi
	sub	x9, x9, x27
	stur	x9, [x29, #-72]         // 8-byte Folded Spill
	bl	_raw_spin_lock
	ldr	x8, [x25, #352]
	cbz	x8, .LBB38_187
// %bb.186:                             //   in Loop: Header=BB38_185 Depth=1
	adrp	x9, jiffies
	ldr	x8, [x25, #360]
	ldr	x9, [x9, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB38_189
.LBB38_187:                             //   in Loop: Header=BB38_185 Depth=1
	ldr	x0, [sp, #88]           // 8-byte Folded Reload
	bl	_raw_spin_unlock
	ldr	x0, [sp, #80]           // 8-byte Folded Reload
	bl	down_read
	mov	x0, x21
	bl	f2fs_drop_extent_tree
	ldur	x28, [x29, #-72]        // 8-byte Folded Reload
	ldr	x8, [sp, #72]           // 8-byte Folded Reload
	mov	x0, x21
	mov	x1, x21
	mov	x2, x28
	add	x3, x28, x8
	mov	x4, x27
	mov	w5, wzr
	bl	__exchange_data_block
	mov	w27, w0
	ldr	x0, [sp, #80]           // 8-byte Folded Reload
	bl	up_read
	cmp	w27, #0                 // =0
	cset	w8, ne
	str	w8, [sp, #56]           // 4-byte Folded Spill
	ldur	x8, [x29, #-80]         // 8-byte Folded Reload
	cmp	x28, x8
	b.ls	.LBB38_200
// %bb.188:                             //   in Loop: Header=BB38_185 Depth=1
	cbz	w27, .LBB38_185
	b	.LBB38_200
.LBB38_189:                             //   in Loop: Header=BB38_185 Depth=1
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	mov	x1, xzr
	bl	show_stack
	adrp	x8, jiffies
	ldr	x1, [x25, #360]
	ldr	x2, [x8, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x25, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB38_187
.LBB38_190:
	mov	w27, wzr
	b	.LBB38_192
.LBB38_191:
	mov	w27, w0
	ldr	x0, [sp, #40]           // 8-byte Folded Reload
	bl	up_read
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	bl	up_write
	ldr	x0, [sp, #56]           // 8-byte Folded Reload
	bl	up_write
.LBB38_192:
	ldr	x8, [x21, #80]
	cmp	x26, x8
	b.le	.LBB38_42
// %bb.193:
	tbnz	w22, #0, .LBB38_197
// %bb.194:
	mov	x9, x26
	ldr	x26, [x21, #736]
	ldr	x25, [x21, #736]
	mov	w1, #1
	mov	x0, x21
	str	x9, [x21, #80]
	bl	f2fs_mark_inode_dirty_sync
	tbz	w26, #1, .LBB38_196
// %bb.195:
	tbz	w25, #2, .LBB38_42
.LBB38_196:
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB38_42
.LBB38_197:
	ldrb	w8, [x21, #712]
	mov	w1, #1
	mov	x0, x21
	orr	w8, w8, #0x10
	strb	w8, [x21, #712]
	bl	f2fs_mark_inode_dirty_sync
	b	.LBB38_42
.LBB38_198:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	adrp	x8, jiffies
	ldr	x1, [x27, #360]
	ldr	x2, [x8, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x27, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB38_65
.LBB38_199:
	mov	w27, wzr
	str	wzr, [sp, #56]          // 4-byte Folded Spill
.LBB38_200:
	ldr	x25, [sp, #64]          // 8-byte Folded Reload
	mov	x0, x25
	bl	up_write
	ldr	x0, [sp, #48]           // 8-byte Folded Reload
	bl	up_write
	mov	x0, x25
	bl	down_write
	ldr	x0, [x21, #48]
	mov	x2, #9223372036854775807
	mov	x1, x20
	bl	filemap_write_and_wait_range
	mov	x0, x21
	mov	x1, x20
	bl	truncate_pagecache
	mov	x0, x25
	bl	up_write
	ldr	w8, [sp, #56]           // 4-byte Folded Reload
	tbnz	w8, #0, .LBB38_42
// %bb.201:
	ldr	x27, [x21, #736]
	ldr	x25, [x21, #736]
	ldr	x8, [x21, #80]
	cmp	x8, x26
	b.eq	.LBB38_43
// %bb.202:
	mov	w1, #1
	mov	x0, x21
	str	x26, [x21, #80]
	bl	f2fs_mark_inode_dirty_sync
	tbz	w27, #1, .LBB38_204
// %bb.203:
	tbz	w25, #2, .LBB38_43
.LBB38_204:
	add	x8, x21, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB38_43
.LBB38_205:
	bl	__stack_chk_fail
.Lfunc_end38:
	.size	f2fs_fallocate, .Lfunc_end38-f2fs_fallocate
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_dedup_imonitor_send
	.type	f2fs_dedup_imonitor_send,@function
f2fs_dedup_imonitor_send:               // @f2fs_dedup_imonitor_send
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x0, #40]
	mov	x24, x0
	mov	w0, #42677
	movk	w0, #15148, lsl #16
	ldr	x19, [x8, #1008]
	str	x25, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	mov	x21, x3
	mov	x22, x2
	mov	w23, w1
	bl	hiview_hievent_create
	cbz	x0, .LBB39_5
// %bb.1:
	ldr	x8, [x24, #80]
	ldr	x2, [x24, #64]
	mov	w9, #2147483647
	adrp	x1, .L.str.22
	asr	x8, x8, #10
	cmp	x8, x9
	add	x1, x1, :lo12:.L.str.22
	mov	x20, x0
	csel	x24, x8, x9, lo
	bl	hiview_hievent_put_integral
	adrp	x1, .L.str.23
	mov	w25, w0
	mov	w2, w23
	add	x1, x1, :lo12:.L.str.23
	mov	x0, x20
	bl	hiview_hievent_put_integral
	adrp	x1, .L.str.24
	orr	w23, w0, w25
	add	x1, x1, :lo12:.L.str.24
	mov	x0, x20
	mov	x2, x24
	bl	hiview_hievent_put_integral
	adrp	x1, .L.str.25
	orr	w23, w23, w0
	add	x1, x1, :lo12:.L.str.25
	mov	x0, x20
	mov	x2, x22
	bl	hiview_hievent_put_string
	adrp	x1, .L.str.26
	orr	w22, w23, w0
	add	x1, x1, :lo12:.L.str.26
	mov	x0, x20
	mov	x2, x21
	bl	hiview_hievent_put_string
	orr	w3, w22, w0
	cbnz	w3, .LBB39_3
// %bb.2:
	mov	x0, x20
	bl	hiview_hievent_report
	mov	w3, w0
	cbz	w0, .LBB39_4
.LBB39_3:
	adrp	x1, .L.str.27
	adrp	x2, .L__func__.f2fs_dedup_imonitor_send
	add	x1, x1, :lo12:.L.str.27
	add	x2, x2, :lo12:.L__func__.f2fs_dedup_imonitor_send
	mov	x0, x19
	bl	f2fs_printk
.LBB39_4:
	mov	x0, x20
	bl	hiview_hievent_destroy
	b	.LBB39_6
.LBB39_5:
	adrp	x1, .L.str.21
	adrp	x2, .L__func__.f2fs_dedup_imonitor_send
	add	x1, x1, :lo12:.L.str.21
	add	x2, x2, :lo12:.L__func__.f2fs_dedup_imonitor_send
	mov	x0, x19
	bl	f2fs_printk
.LBB39_6:
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.Lfunc_end39:
	.size	f2fs_dedup_imonitor_send, .Lfunc_end39-f2fs_dedup_imonitor_send
                                        // -- End function
	.p2align	2               // -- Begin function reserve_compress_blocks
	.type	reserve_compress_blocks,@function
reserve_compress_blocks:                // @reserve_compress_blocks
// %bb.0:
	sub	sp, sp, #144            // =144
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #64]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #80]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #96]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #112]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #128]    // 16-byte Folded Spill
	add	x29, sp, #48            // =48
	cbz	x1, .LBB40_97
// %bb.1:
	ldr	x10, [x0]
	mov	x28, #2097152
	mov	x24, #-549755813888
	mov	x25, #-549755813888
	ldr	x8, [x10, #40]
	ldrsw	x27, [x10, #1292]
	mov	x19, x1
	mov	x20, x0
	ldr	x21, [x8, #1008]
	mov	w22, wzr
	movk	x28, #1, lsl #32
	mov	x23, #-549755813888
	movk	x24, #4072
	movk	x25, #4076
.LBB40_2:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB40_15 Depth 2
	ldr	x8, [x20, #16]
	ldr	w12, [x20, #28]
	add	x8, x8, x28
	lsl	x8, x8, #6
	and	x13, x8, #0xfffffffffffff000
	ldr	w9, [x13, x24]
	ldr	w11, [x13, x25]
	add	w8, w12, w22
	add	x12, x13, x23
	cmp	w9, w11
	b.ne	.LBB40_7
// %bb.3:                               //   in Loop: Header=BB40_2 Depth=1
	cbz	x10, .LBB40_6
// %bb.4:                               //   in Loop: Header=BB40_2 Depth=1
	ldr	x13, [x10, #736]
	tbz	w13, #26, .LBB40_7
// %bb.5:                               //   in Loop: Header=BB40_2 Depth=1
	ldr	w10, [x10, #1184]
	asr	w10, w10, #2
	b	.LBB40_8
.LBB40_6:                               //   in Loop: Header=BB40_2 Depth=1
	ldrb	w10, [x12, #3]
	tbnz	w10, #5, .LBB40_20
.LBB40_7:                               //   in Loop: Header=BB40_2 Depth=1
	mov	w10, wzr
.LBB40_8:                               //   in Loop: Header=BB40_2 Depth=1
	add	x13, x12, #360          // =360
	cmp	w9, w11
	csel	x9, x13, x12, eq
	add	w8, w8, w10
	ldr	w1, [x9, w8, uxtw #2]
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB40_11
.LBB40_9:                               //   in Loop: Header=BB40_2 Depth=1
	add	w22, w22, #1            // =1
	sxtw	x8, w22
	cmp	x8, x19
	b.hs	.LBB40_21
// %bb.10:                              //   in Loop: Header=BB40_2 Depth=1
	ldr	x10, [x20]
	b	.LBB40_2
.LBB40_11:                              //   in Loop: Header=BB40_2 Depth=1
	ldr	w9, [x21, #3128]
	cbz	w9, .LBB40_19
// %bb.12:                              //   in Loop: Header=BB40_2 Depth=1
	ldr	x8, [x21]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB40_18
// %bb.13:                              //   in Loop: Header=BB40_2 Depth=1
	ldr	x10, [x21, #3136]
	add	x10, x10, #80           // =80
	b	.LBB40_15
.LBB40_14:                              //   in Loop: Header=BB40_15 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB40_18
.LBB40_15:                              //   Parent Loop BB40_2 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB40_14
// %bb.16:                              //   in Loop: Header=BB40_15 Depth=2
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB40_14
// %bb.17:                              //   in Loop: Header=BB40_2 Depth=1
	ldur	x8, [x10, #-80]
.LBB40_18:                              //   in Loop: Header=BB40_2 Depth=1
	cbz	x8, .LBB40_9
.LBB40_19:                              //   in Loop: Header=BB40_2 Depth=1
	mov	w2, #7
	mov	x0, x21
	bl	f2fs_is_valid_blkaddr
	tbnz	w0, #0, .LBB40_9
	b	.LBB40_99
.LBB40_20:                              //   in Loop: Header=BB40_2 Depth=1
	ldrh	w10, [x12, #360]
	lsr	w10, w10, #2
	b	.LBB40_8
.LBB40_21:
	mov	w25, wzr
	cbz	x19, .LBB40_102
// %bb.22:
	ldr	w24, [x20, #28]
	add	x8, x21, #1336          // =1336
	str	x8, [sp, #24]           // 8-byte Folded Spill
	add	x8, x21, #2340          // =2340
	str	x8, [sp, #16]           // 8-byte Folded Spill
	//APP
	mrs x8, sp_el0
	//NO_APP
	str	x8, [sp, #8]            // 8-byte Folded Spill
	b	.LBB40_26
.LBB40_23:                              //   in Loop: Header=BB40_26 Depth=1
	ldur	x19, [x29, #-8]         // 8-byte Folded Reload
	ldur	w25, [x29, #-12]        // 4-byte Folded Reload
	mov	x28, #2097152
	movk	x28, #1, lsl #32
.LBB40_24:                              //   in Loop: Header=BB40_26 Depth=1
	add	w25, w25, w23
.LBB40_25:                              //   in Loop: Header=BB40_26 Depth=1
	subs	x19, x19, x27
	b.eq	.LBB40_102
.LBB40_26:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB40_30 Depth 2
                                        //       Child Loop BB40_42 Depth 3
                                        //     Child Loop BB40_73 Depth 2
                                        //       Child Loop BB40_86 Depth 3
	ldr	x26, [x20]
	mov	x11, #-549755813888
	cmp	w27, #1                 // =1
	b.lt	.LBB40_48
// %bb.27:                              //   in Loop: Header=BB40_26 Depth=1
	ldr	x9, [x20, #16]
	mov	w22, wzr
	mov	w8, wzr
	add	x9, x9, x28
	lsl	x9, x9, #6
	and	x10, x9, #0xfffffffffffff000
	add	x9, x10, x11
	mov	x11, #-549755813888
	movk	x11, #360
	add	x10, x10, x11
	mov	w11, w24
	b	.LBB40_30
.LBB40_28:                              //   in Loop: Header=BB40_30 Depth=2
	cmn	w12, #2                 // =2
	b.ne	.LBB40_91
.LBB40_29:                              //   in Loop: Header=BB40_30 Depth=2
	add	w8, w8, #1              // =1
	add	w11, w11, #1            // =1
	cmp	w8, w27
	str	w11, [x20, #28]
	b.eq	.LBB40_49
.LBB40_30:                              //   Parent Loop BB40_26 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB40_42 Depth 3
	ldr	w12, [x9, #4072]
	ldr	w13, [x9, #4076]
	cmp	w12, w13
	b.ne	.LBB40_35
// %bb.31:                              //   in Loop: Header=BB40_30 Depth=2
	cbz	x26, .LBB40_34
// %bb.32:                              //   in Loop: Header=BB40_30 Depth=2
	ldr	x14, [x26, #736]
	tbz	w14, #26, .LBB40_35
// %bb.33:                              //   in Loop: Header=BB40_30 Depth=2
	ldr	w14, [x26, #1184]
	asr	w14, w14, #2
	b	.LBB40_36
.LBB40_34:                              //   in Loop: Header=BB40_30 Depth=2
	ldrb	w14, [x9, #3]
	tbnz	w14, #5, .LBB40_47
.LBB40_35:                              //   in Loop: Header=BB40_30 Depth=2
	mov	w14, wzr
.LBB40_36:                              //   in Loop: Header=BB40_30 Depth=2
	cmp	w12, w13
	csel	x12, x10, x9, eq
	add	w13, w14, w11
	ldr	w12, [x12, w13, uxtw #2]
	cbz	w8, .LBB40_28
// %bb.37:                              //   in Loop: Header=BB40_30 Depth=2
	add	w13, w12, #3            // =3
	cmp	w13, #4                 // =4
	b.lo	.LBB40_29
// %bb.38:                              //   in Loop: Header=BB40_30 Depth=2
	ldr	w14, [x21, #3128]
	cbz	w14, .LBB40_46
// %bb.39:                              //   in Loop: Header=BB40_30 Depth=2
	ldr	x13, [x21]
	cmp	w14, #1                 // =1
	ldr	x13, [x13, #248]
	b.lt	.LBB40_45
// %bb.40:                              //   in Loop: Header=BB40_30 Depth=2
	ldr	x15, [x21, #3136]
	add	x15, x15, #80           // =80
	b	.LBB40_42
.LBB40_41:                              //   in Loop: Header=BB40_42 Depth=3
	subs	x14, x14, #1            // =1
	add	x15, x15, #88           // =88
	b.eq	.LBB40_45
.LBB40_42:                              //   Parent Loop BB40_26 Depth=1
                                        //     Parent Loop BB40_30 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	ldur	w16, [x15, #-4]
	cmp	w16, w12
	b.hi	.LBB40_41
// %bb.43:                              //   in Loop: Header=BB40_42 Depth=3
	ldr	w16, [x15]
	cmp	w16, w12
	b.lo	.LBB40_41
// %bb.44:                              //   in Loop: Header=BB40_30 Depth=2
	ldur	x13, [x15, #-80]
.LBB40_45:                              //   in Loop: Header=BB40_30 Depth=2
	cbz	x13, .LBB40_29
.LBB40_46:                              //   in Loop: Header=BB40_30 Depth=2
	add	w22, w22, #1            // =1
	b	.LBB40_29
.LBB40_47:                              //   in Loop: Header=BB40_30 Depth=2
	ldrh	w14, [x9, #360]
	lsr	w14, w14, #2
	b	.LBB40_36
.LBB40_48:                              //   in Loop: Header=BB40_26 Depth=1
	mov	w22, wzr
.LBB40_49:                              //   in Loop: Header=BB40_26 Depth=1
	ldrb	w8, [x26, #142]
	sub	w9, w27, w22
	sxtw	x23, w9
	mov	w2, #3
	lsl	x1, x23, x8
	mov	x0, x26
	bl	__dquot_alloc_space
	add	w8, w0, #28             // =28
	cmp	w8, #28                 // =28
	b.hi	.LBB40_51
// %bb.50:                              //   in Loop: Header=BB40_26 Depth=1
	mov	w9, #1
	lsl	w8, w9, w8
	mov	w9, #1
	movk	w9, #4224, lsl #16
	tst	w8, w9
	b.ne	.LBB40_52
.LBB40_51:                              //   in Loop: Header=BB40_26 Depth=1
	cmn	w0, #122                // =122
	b.ne	.LBB40_98
.LBB40_52:                              //   in Loop: Header=BB40_26 Depth=1
	adrp	x8, percpu_counter_batch
	ldr	w2, [x8, :lo12:percpu_counter_batch]
	ldr	x0, [sp, #24]           // 8-byte Folded Reload
	mov	x1, x23
	stur	w25, [x29, #-12]        // 4-byte Folded Spill
	bl	percpu_counter_add_batch
	ldr	x28, [sp, #16]          // 8-byte Folded Reload
	mov	x0, x28
	bl	_raw_spin_lock
	ldr	w8, [x21, #1240]
	ldr	w9, [x21, #1236]
	ldr	w10, [x21, #1256]
	add	w8, w8, w23
	str	w8, [x21, #1240]
	sub	w25, w9, w10
	cbz	x26, .LBB40_60
// %bb.53:                              //   in Loop: Header=BB40_26 Depth=1
	ldrb	w8, [x21, #1427]
	tbnz	w8, #0, .LBB40_55
.LBB40_54:                              //   in Loop: Header=BB40_26 Depth=1
	ldr	w8, [x21, #1436]
	sub	w25, w25, w8
	b	.LBB40_60
.LBB40_55:                              //   in Loop: Header=BB40_26 Depth=1
	ldrb	w8, [x26, #12]
	tbnz	w8, #5, .LBB40_60
// %bb.56:                              //   in Loop: Header=BB40_26 Depth=1
	ldr	x8, [sp, #8]            // 8-byte Folded Reload
	ldr	w9, [x21, #1440]
	ldr	x8, [x8, #1984]
	ldr	w8, [x8, #28]
	cmp	w9, w8
	b.eq	.LBB40_60
// %bb.57:                              //   in Loop: Header=BB40_26 Depth=1
	ldr	w0, [x21, #1444]
	cbz	w0, .LBB40_59
// %bb.58:                              //   in Loop: Header=BB40_26 Depth=1
	bl	in_group_p
	cbnz	w0, .LBB40_60
.LBB40_59:                              //   in Loop: Header=BB40_26 Depth=1
	mov	w0, #24
	bl	capable
	tbz	w0, #0, .LBB40_54
.LBB40_60:                              //   in Loop: Header=BB40_26 Depth=1
	ldr	x8, [x21, #80]
	tbnz	w8, #8, .LBB40_93
.LBB40_61:                              //   in Loop: Header=BB40_26 Depth=1
	ldr	w8, [x21, #1240]
	stur	x19, [x29, #-8]         // 8-byte Folded Spill
	subs	w9, w8, w25
	b.hi	.LBB40_94
// %bb.62:                              //   in Loop: Header=BB40_26 Depth=1
	mov	x0, x28
	bl	_raw_spin_unlock
	mov	x25, x23
.LBB40_63:                              //   in Loop: Header=BB40_26 Depth=1
	ldr	x19, [x26, #736]
	ldr	x28, [x26, #736]
	ldrb	w8, [x26, #142]
	and	x9, x25, #0xffffffff
	mov	x0, x26
	lsl	x1, x9, x8
	bl	dquot_claim_space_nodirty
	cbnz	w0, .LBB40_65
// %bb.64:                              //   in Loop: Header=BB40_26 Depth=1
	mov	w1, #1
	mov	x0, x26
	bl	__mark_inode_dirty
.LBB40_65:                              //   in Loop: Header=BB40_26 Depth=1
	mov	w1, #1
	mov	x0, x26
	bl	f2fs_mark_inode_dirty_sync
	tbz	w19, #1, .LBB40_67
// %bb.66:                              //   in Loop: Header=BB40_26 Depth=1
	tbz	w28, #2, .LBB40_68
.LBB40_67:                              //   in Loop: Header=BB40_26 Depth=1
	add	x8, x26, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB40_68:                              //   in Loop: Header=BB40_26 Depth=1
	cmp	x25, x23
	b.ne	.LBB40_101
// %bb.69:                              //   in Loop: Header=BB40_26 Depth=1
	ldr	x0, [x20]
	mov	w1, #1
	ldr	w8, [x0, #1292]
	add	x9, x0, #1280           // =1280
	sub	w8, w8, w22
	//APP
	// atomic_add
	prfm	pstl1strm, [x9]
1:	ldxr	w10, [x9]
	add	w10, w10, w8
	stxr	w11, w10, [x9]
	cbnz	w11, 1b

	//NO_APP
	ldr	x9, [x0, #40]
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
	cmp	w27, #1                 // =1
	str	w24, [x20, #28]
	b.lt	.LBB40_23
// %bb.70:                              //   in Loop: Header=BB40_26 Depth=1
	ldur	x19, [x29, #-8]         // 8-byte Folded Reload
	ldur	w25, [x29, #-12]        // 4-byte Folded Reload
	mov	x28, #2097152
	mov	w22, wzr
	movk	x28, #1, lsl #32
	mov	x26, #-549755813888
	b	.LBB40_73
.LBB40_71:                              //   in Loop: Header=BB40_73 Depth=2
	cmn	w8, #2                  // =2
	b.ne	.LBB40_92
.LBB40_72:                              //   in Loop: Header=BB40_73 Depth=2
	add	w22, w22, #1            // =1
	add	w24, w24, #1            // =1
	cmp	w22, w27
	str	w24, [x20, #28]
	b.eq	.LBB40_24
.LBB40_73:                              //   Parent Loop BB40_26 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB40_86 Depth 3
	ldr	x8, [x20, #16]
	mov	x9, #-549755813888
	movk	x9, #4076
	add	x8, x8, x28
	lsl	x8, x8, #6
	and	x10, x8, #0xfffffffffffff000
	mov	x8, #-549755813888
	movk	x8, #4072
	ldr	w8, [x10, x8]
	ldr	w9, [x10, x9]
	add	x10, x10, x26
	cmp	w8, w9
	b.ne	.LBB40_78
// %bb.74:                              //   in Loop: Header=BB40_73 Depth=2
	ldr	x11, [x20]
	cbz	x11, .LBB40_77
// %bb.75:                              //   in Loop: Header=BB40_73 Depth=2
	ldr	x12, [x11, #736]
	tbz	w12, #26, .LBB40_78
// %bb.76:                              //   in Loop: Header=BB40_73 Depth=2
	ldr	w11, [x11, #1184]
	asr	w11, w11, #2
	b	.LBB40_79
.LBB40_77:                              //   in Loop: Header=BB40_73 Depth=2
	ldrb	w11, [x10, #3]
	tbnz	w11, #5, .LBB40_90
.LBB40_78:                              //   in Loop: Header=BB40_73 Depth=2
	mov	w11, wzr
.LBB40_79:                              //   in Loop: Header=BB40_73 Depth=2
	add	x12, x10, #360          // =360
	cmp	w8, w9
	csel	x8, x12, x10, eq
	add	w9, w11, w24
	ldr	w8, [x8, w9, uxtw #2]
	cbz	w22, .LBB40_71
// %bb.80:                              //   in Loop: Header=BB40_73 Depth=2
	add	w9, w8, #3              // =3
	cmp	w9, #4                  // =4
	b.hs	.LBB40_82
.LBB40_81:                              //   in Loop: Header=BB40_73 Depth=2
	mov	w8, #-1
	mov	x0, x20
	str	w8, [x20, #36]
	bl	f2fs_set_data_blkaddr
	ldr	w24, [x20, #28]
	b	.LBB40_72
.LBB40_82:                              //   in Loop: Header=BB40_73 Depth=2
	ldr	w10, [x21, #3128]
	cbz	w10, .LBB40_72
// %bb.83:                              //   in Loop: Header=BB40_73 Depth=2
	ldr	x9, [x21]
	cmp	w10, #1                 // =1
	ldr	x9, [x9, #248]
	b.lt	.LBB40_89
// %bb.84:                              //   in Loop: Header=BB40_73 Depth=2
	ldr	x11, [x21, #3136]
	add	x11, x11, #80           // =80
	b	.LBB40_86
.LBB40_85:                              //   in Loop: Header=BB40_86 Depth=3
	subs	x10, x10, #1            // =1
	add	x11, x11, #88           // =88
	b.eq	.LBB40_89
.LBB40_86:                              //   Parent Loop BB40_26 Depth=1
                                        //     Parent Loop BB40_73 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	ldur	w12, [x11, #-4]
	cmp	w12, w8
	b.hi	.LBB40_85
// %bb.87:                              //   in Loop: Header=BB40_86 Depth=3
	ldr	w12, [x11]
	cmp	w12, w8
	b.lo	.LBB40_85
// %bb.88:                              //   in Loop: Header=BB40_73 Depth=2
	ldur	x9, [x11, #-80]
.LBB40_89:                              //   in Loop: Header=BB40_73 Depth=2
	cbnz	x9, .LBB40_72
	b	.LBB40_81
.LBB40_90:                              //   in Loop: Header=BB40_73 Depth=2
	ldrh	w11, [x10, #360]
	lsr	w11, w11, #2
	b	.LBB40_79
.LBB40_91:                              //   in Loop: Header=BB40_26 Depth=1
	add	w24, w11, w27
	str	w24, [x20, #28]
	b	.LBB40_25
.LBB40_92:                              //   in Loop: Header=BB40_26 Depth=1
	add	w24, w24, w27
	str	w24, [x20, #28]
	b	.LBB40_25
.LBB40_93:                              //   in Loop: Header=BB40_26 Depth=1
	ldr	w8, [x21, #1264]
	subs	w8, w25, w8
	csel	w25, wzr, w8, lo
	b	.LBB40_61
.LBB40_94:                              //   in Loop: Header=BB40_26 Depth=1
	cmp	x23, x9
	csel	x9, x23, x9, lo
	sub	w8, w8, w9
	mov	x0, x28
	sub	x25, x23, x9
	str	x9, [sp]                // 8-byte Folded Spill
	str	w8, [x21, #1240]
	bl	_raw_spin_unlock
	cbz	x25, .LBB40_100
// %bb.95:                              //   in Loop: Header=BB40_26 Depth=1
	ldr	x28, [sp]               // 8-byte Folded Reload
	cbz	x28, .LBB40_63
// %bb.96:                              //   in Loop: Header=BB40_26 Depth=1
	adrp	x8, percpu_counter_batch
	ldr	w2, [x8, :lo12:percpu_counter_batch]
	ldr	x0, [sp, #24]           // 8-byte Folded Reload
	neg	x1, x28
	bl	percpu_counter_add_batch
	ldrb	w8, [x26, #142]
	mov	w2, #2
	mov	x0, x26
	lsl	x1, x28, x8
	bl	__dquot_free_space
	b	.LBB40_63
.LBB40_97:
	mov	w25, wzr
	b	.LBB40_102
.LBB40_98:
	mov	w25, w0
	b	.LBB40_102
.LBB40_99:
	mov	w25, #-117
	b	.LBB40_102
.LBB40_100:
	adrp	x8, percpu_counter_batch
	ldr	x19, [sp]               // 8-byte Folded Reload
	ldr	w2, [x8, :lo12:percpu_counter_batch]
	ldr	x0, [sp, #24]           // 8-byte Folded Reload
	neg	x1, x19
	bl	percpu_counter_add_batch
	ldrb	w8, [x26, #142]
	mov	w2, #2
	mov	x0, x26
	lsl	x1, x19, x8
	bl	__dquot_free_space
.LBB40_101:
	mov	w25, #-28
.LBB40_102:
	mov	w0, w25
	ldp	x20, x19, [sp, #128]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #112]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #96]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #80]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #64]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #144            // =144
	ret
.Lfunc_end40:
	.size	reserve_compress_blocks, .Lfunc_end40-reserve_compress_blocks
                                        // -- End function
	.p2align	2               // -- Begin function release_compress_blocks
	.type	release_compress_blocks,@function
release_compress_blocks:                // @release_compress_blocks
// %bb.0:
	sub	sp, sp, #112            // =112
	stp	x29, x30, [sp, #16]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #32]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #48]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #64]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #80]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #96]     // 16-byte Folded Spill
	add	x29, sp, #16            // =16
	cbz	x1, .LBB41_54
// %bb.1:
	ldr	x10, [x0]
	mov	x28, #2097152
	mov	x23, #-549755813888
	mov	x24, #-549755813888
	ldr	x8, [x10, #40]
	ldrsw	x27, [x10, #1292]
	mov	x19, x1
	mov	x20, x0
	ldr	x25, [x8, #1008]
	mov	w21, wzr
	movk	x28, #1, lsl #32
	mov	x22, #-549755813888
	movk	x23, #4072
	movk	x24, #4076
.LBB41_2:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB41_15 Depth 2
	ldr	x8, [x20, #16]
	ldr	w12, [x20, #28]
	add	x8, x8, x28
	lsl	x8, x8, #6
	and	x13, x8, #0xfffffffffffff000
	ldr	w9, [x13, x23]
	ldr	w11, [x13, x24]
	add	w8, w12, w21
	add	x12, x13, x22
	cmp	w9, w11
	b.ne	.LBB41_7
// %bb.3:                               //   in Loop: Header=BB41_2 Depth=1
	cbz	x10, .LBB41_6
// %bb.4:                               //   in Loop: Header=BB41_2 Depth=1
	ldr	x13, [x10, #736]
	tbz	w13, #26, .LBB41_7
// %bb.5:                               //   in Loop: Header=BB41_2 Depth=1
	ldr	w10, [x10, #1184]
	asr	w10, w10, #2
	b	.LBB41_8
.LBB41_6:                               //   in Loop: Header=BB41_2 Depth=1
	ldrb	w10, [x12, #3]
	tbnz	w10, #5, .LBB41_20
.LBB41_7:                               //   in Loop: Header=BB41_2 Depth=1
	mov	w10, wzr
.LBB41_8:                               //   in Loop: Header=BB41_2 Depth=1
	add	x13, x12, #360          // =360
	cmp	w9, w11
	csel	x9, x13, x12, eq
	add	w8, w8, w10
	ldr	w1, [x9, w8, uxtw #2]
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	b.hs	.LBB41_11
.LBB41_9:                               //   in Loop: Header=BB41_2 Depth=1
	add	w21, w21, #1            // =1
	sxtw	x8, w21
	cmp	x8, x19
	b.hs	.LBB41_21
// %bb.10:                              //   in Loop: Header=BB41_2 Depth=1
	ldr	x10, [x20]
	b	.LBB41_2
.LBB41_11:                              //   in Loop: Header=BB41_2 Depth=1
	ldr	w9, [x25, #3128]
	cbz	w9, .LBB41_19
// %bb.12:                              //   in Loop: Header=BB41_2 Depth=1
	ldr	x8, [x25]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB41_18
// %bb.13:                              //   in Loop: Header=BB41_2 Depth=1
	ldr	x10, [x25, #3136]
	add	x10, x10, #80           // =80
	b	.LBB41_15
.LBB41_14:                              //   in Loop: Header=BB41_15 Depth=2
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB41_18
.LBB41_15:                              //   Parent Loop BB41_2 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB41_14
// %bb.16:                              //   in Loop: Header=BB41_15 Depth=2
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB41_14
// %bb.17:                              //   in Loop: Header=BB41_2 Depth=1
	ldur	x8, [x10, #-80]
.LBB41_18:                              //   in Loop: Header=BB41_2 Depth=1
	cbz	x8, .LBB41_9
.LBB41_19:                              //   in Loop: Header=BB41_2 Depth=1
	mov	w2, #7
	mov	x0, x25
	bl	f2fs_is_valid_blkaddr
	tbnz	w0, #0, .LBB41_9
	b	.LBB41_56
.LBB41_20:                              //   in Loop: Header=BB41_2 Depth=1
	ldrh	w10, [x12, #360]
	lsr	w10, w10, #2
	b	.LBB41_8
.LBB41_21:
	cbz	x19, .LBB41_54
// %bb.22:
	mov	x24, #-549755813888
	mov	x22, #-549755813888
	mov	x23, #-549755813888
	movk	x24, #4072
	movk	x22, #4076
	str	x25, [sp, #8]           // 8-byte Folded Spill
	str	wzr, [sp, #4]           // 4-byte Folded Spill
	b	.LBB41_25
.LBB41_23:                              //   in Loop: Header=BB41_25 Depth=1
	add	x8, x25, #1280          // =1280
	//APP
	// atomic_sub
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w26
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
	ldr	x8, [x25, #40]
	sxtw	x9, w26
	mov	w1, #1
	mov	x0, x25
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
	ldr	x25, [x20]
.LBB41_24:                              //   in Loop: Header=BB41_25 Depth=1
	ldr	x0, [sp, #8]            // 8-byte Folded Reload
	sub	w26, w27, w21
	mov	x1, x25
	mov	w2, w26
	bl	dec_valid_block_count
	ldr	w8, [sp, #4]            // 4-byte Folded Reload
	add	w8, w26, w8
	str	w8, [sp, #4]            // 4-byte Folded Spill
	subs	x19, x19, x27
	b.eq	.LBB41_55
.LBB41_25:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB41_29 Depth 2
                                        //       Child Loop BB41_43 Depth 3
	ldr	x25, [x20]
	cmp	w27, #1                 // =1
	b.lt	.LBB41_49
// %bb.26:                              //   in Loop: Header=BB41_25 Depth=1
	ldr	w8, [x20, #28]
	mov	w21, wzr
	mov	w26, wzr
	b	.LBB41_29
.LBB41_27:                              //   in Loop: Header=BB41_29 Depth=2
	cmn	w9, #2                  // =2
	b.ne	.LBB41_53
.LBB41_28:                              //   in Loop: Header=BB41_29 Depth=2
	add	w26, w26, #1            // =1
	add	w8, w8, #1              // =1
	cmp	w26, w27
	str	w8, [x20, #28]
	b.eq	.LBB41_50
.LBB41_29:                              //   Parent Loop BB41_25 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB41_43 Depth 3
	ldr	x9, [x20, #16]
	add	x9, x9, x28
	lsl	x9, x9, #6
	and	x11, x9, #0xfffffffffffff000
	ldr	w9, [x11, x24]
	ldr	w10, [x11, x22]
	add	x11, x11, x23
	cmp	w9, w10
	b.ne	.LBB41_34
// %bb.30:                              //   in Loop: Header=BB41_29 Depth=2
	cbz	x25, .LBB41_33
// %bb.31:                              //   in Loop: Header=BB41_29 Depth=2
	ldr	x12, [x25, #736]
	tbz	w12, #26, .LBB41_34
// %bb.32:                              //   in Loop: Header=BB41_29 Depth=2
	ldr	w12, [x25, #1184]
	asr	w12, w12, #2
	b	.LBB41_35
.LBB41_33:                              //   in Loop: Header=BB41_29 Depth=2
	ldrb	w12, [x11, #3]
	tbnz	w12, #5, .LBB41_48
.LBB41_34:                              //   in Loop: Header=BB41_29 Depth=2
	mov	w12, wzr
.LBB41_35:                              //   in Loop: Header=BB41_29 Depth=2
	add	x13, x11, #360          // =360
	cmp	w9, w10
	csel	x9, x13, x11, eq
	add	w10, w12, w8
	ldr	w9, [x9, w10, uxtw #2]
	cbz	w26, .LBB41_27
// %bb.36:                              //   in Loop: Header=BB41_29 Depth=2
	add	w10, w9, #3             // =3
	cmp	w10, #4                 // =4
	b.hs	.LBB41_39
.LBB41_37:                              //   in Loop: Header=BB41_29 Depth=2
	cmn	w9, #1                  // =1
	b.ne	.LBB41_28
// %bb.38:                              //   in Loop: Header=BB41_29 Depth=2
	mov	x0, x20
	str	wzr, [x20, #36]
	bl	f2fs_set_data_blkaddr
	ldr	w8, [x20, #28]
	ldr	x25, [x20]
	b	.LBB41_28
.LBB41_39:                              //   in Loop: Header=BB41_29 Depth=2
	ldr	x10, [sp, #8]           // 8-byte Folded Reload
	ldr	w11, [x10, #3128]
	cbz	w11, .LBB41_47
// %bb.40:                              //   in Loop: Header=BB41_29 Depth=2
	ldr	x12, [sp, #8]           // 8-byte Folded Reload
	cmp	w11, #1                 // =1
	ldr	x10, [x12]
	ldr	x10, [x10, #248]
	b.lt	.LBB41_46
// %bb.41:                              //   in Loop: Header=BB41_29 Depth=2
	ldr	x12, [x12, #3136]
	add	x12, x12, #80           // =80
	b	.LBB41_43
.LBB41_42:                              //   in Loop: Header=BB41_43 Depth=3
	subs	x11, x11, #1            // =1
	add	x12, x12, #88           // =88
	b.eq	.LBB41_46
.LBB41_43:                              //   Parent Loop BB41_25 Depth=1
                                        //     Parent Loop BB41_29 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	ldur	w13, [x12, #-4]
	cmp	w13, w9
	b.hi	.LBB41_42
// %bb.44:                              //   in Loop: Header=BB41_43 Depth=3
	ldr	w13, [x12]
	cmp	w13, w9
	b.lo	.LBB41_42
// %bb.45:                              //   in Loop: Header=BB41_29 Depth=2
	ldur	x10, [x12, #-80]
.LBB41_46:                              //   in Loop: Header=BB41_29 Depth=2
	cbz	x10, .LBB41_37
.LBB41_47:                              //   in Loop: Header=BB41_29 Depth=2
	add	w21, w21, #1            // =1
	b	.LBB41_37
.LBB41_48:                              //   in Loop: Header=BB41_29 Depth=2
	ldrh	w12, [x11, #360]
	lsr	w12, w12, #2
	b	.LBB41_35
.LBB41_49:                              //   in Loop: Header=BB41_25 Depth=1
	mov	w21, wzr
.LBB41_50:                              //   in Loop: Header=BB41_25 Depth=1
	ldr	w8, [x25, #1292]
	ldr	w9, [x25, #1280]
	cbz	w9, .LBB41_24
// %bb.51:                              //   in Loop: Header=BB41_25 Depth=1
	ldr	w9, [x25, #1280]
	sub	w26, w8, w21
	cmp	w26, w9
	b.le	.LBB41_23
// %bb.52:                              //   in Loop: Header=BB41_25 Depth=1
	ldr	x8, [x25, #40]
	adrp	x1, .L.str.29
	adrp	x2, .L__func__.f2fs_i_compr_blocks_update
	add	x1, x1, :lo12:.L.str.29
	ldr	x0, [x8, #1008]
	ldr	w4, [x25, #1280]
	ldr	x5, [x25, #64]
	add	x2, x2, :lo12:.L__func__.f2fs_i_compr_blocks_update
	mov	w3, w26
	bl	f2fs_printk
	b	.LBB41_23
.LBB41_53:                              //   in Loop: Header=BB41_25 Depth=1
	add	w8, w8, w27
	str	w8, [x20, #28]
	subs	x19, x19, x27
	b.ne	.LBB41_25
	b	.LBB41_55
.LBB41_54:
	str	wzr, [sp, #4]           // 4-byte Folded Spill
.LBB41_55:
	ldr	w0, [sp, #4]            // 4-byte Folded Reload
	ldp	x20, x19, [sp, #96]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #80]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #64]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #48]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #32]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #16]     // 16-byte Folded Reload
	add	sp, sp, #112            // =112
	ret
.LBB41_56:
	mov	w8, #-117
	str	w8, [sp, #4]            // 4-byte Folded Spill
	b	.LBB41_55
.Lfunc_end41:
	.size	release_compress_blocks, .Lfunc_end41-release_compress_blocks
                                        // -- End function
	.p2align	2               // -- Begin function redirty_blocks
	.type	redirty_blocks,@function
redirty_blocks:                         // @redirty_blocks
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	cmp	w2, #1                  // =1
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	mov	x29, sp
	b.lt	.LBB42_5
// %bb.1:
	ldr	x20, [x0, #48]
	mov	x19, x1
	mov	x23, xzr
	mov	w22, w2
.LBB42_2:                               // =>This Inner Loop Header: Depth=1
	add	x1, x19, x23
	mov	x0, x20
	mov	x2, xzr
	mov	x3, xzr
	bl	read_cache_page
	cmn	x0, #4095               // =4095
	b.hs	.LBB42_6
// %bb.3:                               //   in Loop: Header=BB42_2 Depth=1
	add	x23, x23, #1            // =1
	cmp	w22, w23
	b.ne	.LBB42_2
// %bb.4:
	mov	w21, wzr
	mov	w23, w22
	cmp	w23, #1                 // =1
	b.ge	.LBB42_7
	b	.LBB42_13
.LBB42_5:
	mov	w21, wzr
	b	.LBB42_13
.LBB42_6:
	mov	x21, x0
	cmp	w23, #1                 // =1
	b.lt	.LBB42_13
.LBB42_7:
	mov	w24, #1
	b	.LBB42_9
.LBB42_8:                               //   in Loop: Header=BB42_9 Depth=1
	subs	w23, w23, #1            // =1
	add	x19, x19, #1            // =1
	b.eq	.LBB42_13
.LBB42_9:                               // =>This Inner Loop Header: Depth=1
	mov	w2, #2
	mov	x0, x20
	mov	x1, x19
	mov	w3, wzr
	bl	pagecache_get_page
	cbz	x0, .LBB42_12
// %bb.10:                              //   in Loop: Header=BB42_9 Depth=1
	mov	x22, x0
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_put_page
	ldr	x8, [x22, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x22, x9, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w24
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB42_8
// %bb.11:                              //   in Loop: Header=BB42_9 Depth=1
	bl	__put_page
	b	.LBB42_8
.LBB42_12:
	mov	w21, #-12
.LBB42_13:
	mov	w0, w21
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.Lfunc_end42:
	.size	redirty_blocks, .Lfunc_end42-redirty_blocks
                                        // -- End function
	.p2align	2               // -- Begin function has_not_enough_free_secs
	.type	has_not_enough_free_secs,@function
has_not_enough_free_secs:               // @has_not_enough_free_secs
// %bb.0:
	ldr	w10, [x0, #1192]
	ldr	w16, [x0, #1188]
	ldrsw	x13, [x0, #1284]
	ldr	w11, [x0, #1184]
	ldrsw	x14, [x0, #1272]
	ldrsw	x15, [x0, #1296]
	ldr	x8, [x0, #80]
	tbnz	w8, #3, .LBB43_12
// %bb.1:
	ldr	x2, [x0, #136]
	ldr	x8, [x2, #8]
	ldr	w12, [x2, #100]
	ldr	w8, [x8, #8]
	cmn	w12, #1                 // =1
	udiv	w9, w12, w10
	b.eq	.LBB43_3
// %bb.2:
	mov	w17, w9
	b	.LBB43_4
.LBB43_3:
	mov	w17, #-1
.LBB43_4:
	add	w17, w17, w1
	cmp	w8, w17
	b.ne	.LBB43_9
// %bb.5:
	ldr	w3, [x0, #1284]
	ldr	w4, [x0, #1272]
	ldr	w17, [x0, #1272]
	ldr	x0, [x2, #24]
	ldr	x2, [x2]
	mov	w6, #40
	add	w3, w4, w3
	ldr	w5, [x0, #460]
	ldr	x2, [x2, #120]
	madd	x5, x5, x6, x2
	ldrh	w5, [x5, #2]
	and	w4, w5, #0x3ff
	sub	w4, w16, w4
	cmp	w4, w3
	b.lo	.LBB43_9
// %bb.6:
	ldr	w4, [x0, #580]
	mov	w5, #40
	madd	x4, x4, x5, x2
	ldrh	w4, [x4, #2]
	and	w4, w4, #0x3ff
	sub	w4, w16, w4
	cmp	w4, w3
	b.lo	.LBB43_9
// %bb.7:
	ldr	w4, [x0, #700]
	mov	w5, #40
	madd	x4, x4, x5, x2
	ldrh	w4, [x4, #2]
	and	w4, w4, #0x3ff
	sub	w4, w16, w4
	cmp	w4, w3
	b.lo	.LBB43_9
// %bb.8:
	ldr	w0, [x0, #100]
	mov	w3, #40
	madd	x0, x0, x3, x2
	ldrh	w0, [x0, #2]
	and	w0, w0, #0x3ff
	sub	w0, w16, w0
	cmp	w0, w17
	b.hs	.LBB43_12
.LBB43_9:
	mul	w16, w16, w10
	sub	x16, x16, #1            // =1
	add	x13, x16, x13
	add	x14, x16, x14
	add	x15, x16, x15
	asr	x13, x13, x11
	asr	x14, x14, x11
	asr	x15, x15, x11
	udiv	w11, w13, w10
	udiv	w13, w14, w10
	udiv	w10, w15, w10
	cmn	w12, #1                 // =1
	lsl	w12, w13, #1
	b.ne	.LBB43_11
// %bb.10:
	mov	w9, #-1
.LBB43_11:
	add	w11, w11, w1
	add	w11, w11, w12
	add	w10, w11, w10
	add	w9, w10, w9
	cmp	w8, w9
	cset	w0, ls
	ret
.LBB43_12:
	mov	w0, wzr
	ret
.Lfunc_end43:
	.size	has_not_enough_free_secs, .Lfunc_end43-has_not_enough_free_secs
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_setflags_common
	.type	f2fs_setflags_common,@function
f2fs_setflags_common:                   // @f2fs_setflags_common
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldrb	w8, [x0, #12]
	str	x23, [sp, #16]          // 8-byte Folded Spill
	mov	x29, sp
	tbnz	w8, #5, .LBB44_11
// %bb.1:
	ldr	w8, [x0, #704]
	and	w21, w2, w1
	mov	w20, w2
	mov	x19, x0
	and	w23, w8, w2
	eor	w22, w23, w21
	tbnz	w22, #30, .LBB44_12
.LBB44_2:
	mov	w8, #1028
	ands	w8, w21, w8
	b.eq	.LBB44_5
// %bb.3:
	ldr	x9, [x19, #40]
	ldr	x9, [x9, #1008]
	ldr	x9, [x9, #16]
	ldrb	w9, [x9, #2181]
	tbz	w9, #5, .LBB44_13
// %bb.4:
	cmp	w8, #1028               // =1028
	b.eq	.LBB44_32
.LBB44_5:
	tbnz	w22, #2, .LBB44_16
.LBB44_6:
	tbz	w23, #2, .LBB44_8
// %bb.7:
	tbnz	w22, #10, .LBB44_32
.LBB44_8:
	ldr	x8, [x19, #704]
	mvn	w10, w20
	mov	w9, w21
	and	x8, x8, x10
	orr	x8, x8, x9
	mvn	w9, w8
	mov	w10, #1028
	tst	x9, x10
	str	x8, [x19, #704]
	b.eq	.LBB44_43
.LBB44_9:
	ldrb	w8, [x19, #707]
	tbnz	w8, #5, .LBB44_33
// %bb.10:
	add	x8, x19, #736           // =736
	mov	w9, #134217728
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	b	.LBB44_34
.LBB44_11:
	mov	w0, #-1
	b	.LBB44_35
.LBB44_12:
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	ldr	x8, [x8, #16]
	ldrb	w8, [x8, #2181]
	tbnz	w8, #4, .LBB44_14
.LBB44_13:
	mov	w0, #-95
	b	.LBB44_35
.LBB44_14:
	mov	x0, x19
	bl	f2fs_empty_dir
	tbnz	w0, #0, .LBB44_2
// %bb.15:
	mov	w0, #-39
	b	.LBB44_35
.LBB44_16:
	tbz	w23, #2, .LBB44_23
// %bb.17:
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB44_23
// %bb.18:
	ldr	x8, [x19, #736]
	tbz	w8, #31, .LBB44_23
// %bb.19:
	ldr	w8, [x19, #824]
	ldr	x9, [x19, #144]
	cmp	w8, #0                  // =0
	cset	w8, ne
	cmp	x8, x9, lsr #3
	b.lo	.LBB44_32
// %bb.20:
	ldr	x8, [x19, #704]
	and	x8, x8, #0xfffffffffffffffb
	str	x8, [x19, #704]
	ldr	x8, [x19, #736]
	tbz	w8, #31, .LBB44_22
// %bb.21:
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
.LBB44_22:
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
.LBB44_23:
	tbnz	w21, #10, .LBB44_32
// %bb.24:
	tbz	w21, #2, .LBB44_6
// %bb.25:
	ldrb	w8, [x19, #13]
	tbnz	w8, #0, .LBB44_32
// %bb.26:
	ldr	x8, [x19, #736]
	tbnz	w8, #28, .LBB44_32
// %bb.27:
	ldr	x8, [x19, #736]
	tbnz	w8, #15, .LBB44_32
// %bb.28:
	ldr	x8, [x19, #736]
	tbnz	w8, #17, .LBB44_32
// %bb.29:
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #4, lsl #12         // =16384
	b.eq	.LBB44_36
// %bb.30:
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB44_32
// %bb.31:
	ldr	x8, [x19, #80]
	cbz	x8, .LBB44_36
.LBB44_32:
	mov	w0, #-22
	b	.LBB44_35
.LBB44_33:
	add	x8, x19, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 134217728
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB44_34:
	mov	x0, x19
	bl	current_time
	stp	x0, x1, [x19, #120]
	mov	x0, x19
	bl	f2fs_set_inode_flags
	mov	w1, #1
	mov	x0, x19
	bl	f2fs_mark_inode_dirty_sync
	mov	w0, wzr
.LBB44_35:
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldr	x23, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB44_36:
	ldr	x8, [x19, #40]
	mov	w12, #1
	ldr	x8, [x8, #1008]
	ldrb	w10, [x8, #1524]
	strb	w10, [x19, #1284]
	ldrb	w11, [x8, #1525]
	cmp	w10, #1                 // =1
	strb	w11, [x19, #1285]
	ldrb	w9, [x8, #1527]
	lsl	w10, w12, w11
	str	w10, [x19, #1292]
	strh	w9, [x19, #1288]
	b.ne	.LBB44_39
// %bb.37:
	ldrb	w8, [x8, #1526]
	cbz	w8, .LBB44_39
// %bb.38:
	bfi	w9, w8, #8, #8
	strh	w9, [x19, #1288]
.LBB44_39:
	ldr	x8, [x19, #704]
	orr	x8, x8, #0x4
	str	x8, [x19, #704]
	add	x8, x19, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 2147483648
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB44_42
// %bb.40:
	ldr	x8, [x19, #736]
	tbz	w8, #31, .LBB44_42
// %bb.41:
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	add	x8, x8, #2220           // =2220
	//APP
	// atomic_add
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	add	w9, w9, 1
	stxr	w10, w9, [x8]
	cbnz	w10, 1b

	//NO_APP
.LBB44_42:
	ldr	x8, [x19, #40]
	mov	w1, #1
	mov	x0, x19
	ldr	x8, [x8, #1008]
	ldr	w9, [x8, #3312]
	add	w9, w9, #1              // =1
	str	w9, [x8, #3312]
	bl	f2fs_mark_inode_dirty_sync
	b	.LBB44_6
.LBB44_43:
	bl	print_bdev_access_info
	adrp	x20, f2fs_dclient
	ldr	x0, [x20, :lo12:f2fs_dclient]
	cbz	x0, .LBB44_46
// %bb.44:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB44_46
// %bb.45:
	ldr	x0, [x20, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.f2fs_setflags_common
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.f2fs_setflags_common
	mov	w3, #4344
	bl	dsm_client_record
	ldr	x0, [x20, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB44_46:
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
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/file.c"; .popsection; .long 14472b - 14470b; .short 4344; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	cbz	x8, .LBB44_9
// %bb.47:
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
	cbnz	w9, .LBB44_9
// %bb.48:
	ldr	x1, [x8, #3392]
	cbz	x1, .LBB44_9
// %bb.49:
	adrp	x9, system_state
	ldr	w9, [x9, :lo12:system_state]
	cmp	w9, #2                  // =2
	b.hi	.LBB44_9
// %bb.50:
	add	x2, x8, #3408           // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB44_9
.Lfunc_end44:
	.size	f2fs_setflags_common, .Lfunc_end44-f2fs_setflags_common
                                        // -- End function
	.p2align	2               // -- Begin function punch_hole
	.type	punch_hole,@function
punch_hole:                             // @punch_hole
// %bb.0:
	stp	x29, x30, [sp, #-96]!   // 16-byte Folded Spill
	str	x27, [sp, #16]          // 8-byte Folded Spill
	stp	x26, x25, [sp, #32]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #48]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #64]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #80]     // 16-byte Folded Spill
	mov	x29, sp
	mov	x21, x2
	mov	x22, x1
	mov	x19, x0
	bl	f2fs_convert_inline_inode
	mov	w24, w0
	cbnz	w0, .LBB45_13
// %bb.1:
	add	x23, x21, x22
	lsr	x20, x22, #12
	lsr	x21, x23, #12
	and	x2, x22, #0xfff
	cmp	x20, x21
	and	x22, x23, #0xfff
	b.ne	.LBB45_3
// %bb.2:
	sub	x3, x22, x2
	mov	x0, x19
	mov	x1, x20
	bl	fill_zero
	mov	w24, w0
	b	.LBB45_13
.LBB45_3:
	cbz	x2, .LBB45_6
// %bb.4:
	mov	w8, #4096
	sub	x3, x8, x2
	mov	x0, x19
	mov	x1, x20
	bl	fill_zero
	mov	w24, w0
	cbnz	w0, .LBB45_13
// %bb.5:
	add	x20, x20, #1            // =1
.LBB45_6:
	cbz	x22, .LBB45_8
// %bb.7:
	mov	x0, x19
	mov	x1, x21
	mov	x2, xzr
	mov	x3, x22
	bl	fill_zero
	mov	w24, w0
	cbnz	w0, .LBB45_13
.LBB45_8:
	cmp	x20, x21
	b.hs	.LBB45_12
// %bb.9:
	ldp	x8, x25, [x19, #40]
	mov	w1, #1
	ldr	x24, [x8, #1008]
	mov	x0, x24
	bl	f2fs_balance_fs
	add	x22, x19, #1040         // =1040
	mov	x0, x22
	lsl	x26, x20, #12
	and	x27, x23, #0xfffffffffffff000
	bl	down_write
	add	x23, x19, #1088         // =1088
	mov	x0, x23
	bl	down_write
	sub	x2, x27, #1             // =1
	mov	x0, x25
	mov	x1, x26
	bl	truncate_inode_pages_range
	add	x25, x24, #368          // =368
	mov	x0, x25
	bl	_raw_spin_lock
	ldr	x8, [x24, #352]
	cbz	x8, .LBB45_11
// %bb.10:
	adrp	x26, jiffies
	ldr	x8, [x24, #360]
	ldr	x9, [x26, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB45_14
.LBB45_11:
	mov	x0, x25
	bl	_raw_spin_unlock
	add	x25, x24, #304          // =304
	mov	x0, x25
	bl	down_read
	mov	x0, x19
	mov	x1, x20
	mov	x2, x21
	bl	f2fs_truncate_hole
	mov	w24, w0
	mov	x0, x25
	bl	up_read
	mov	x0, x23
	bl	up_write
	mov	x0, x22
	bl	up_write
	b	.LBB45_13
.LBB45_12:
	mov	w24, wzr
.LBB45_13:
	mov	w0, w24
	ldp	x20, x19, [sp, #80]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #64]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #48]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #32]     // 16-byte Folded Reload
	ldr	x27, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #96     // 16-byte Folded Reload
	ret
.LBB45_14:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x24, #360]
	ldr	x2, [x26, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x24, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB45_11
.Lfunc_end45:
	.size	punch_hole, .Lfunc_end45-punch_hole
                                        // -- End function
	.p2align	2               // -- Begin function fill_zero
	.type	fill_zero,@function
fill_zero:                              // @fill_zero
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	str	x25, [sp, #16]          // 8-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	mov	x29, sp
	cbz	x3, .LBB46_10
// %bb.1:
	ldr	x8, [x0, #40]
	mov	x21, x1
	mov	x22, x0
	mov	w1, #1
	ldr	x23, [x8, #1008]
	mov	x19, x3
	mov	x20, x2
	mov	x0, x23
	bl	f2fs_balance_fs
	add	x24, x23, #368          // =368
	mov	x0, x24
	bl	_raw_spin_lock
	ldr	x8, [x23, #352]
	cbz	x8, .LBB46_3
// %bb.2:
	adrp	x25, jiffies
	ldr	x8, [x23, #360]
	ldr	x9, [x25, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB46_12
.LBB46_3:
	mov	x0, x24
	bl	_raw_spin_unlock
	add	x23, x23, #304          // =304
	mov	x0, x23
	bl	down_read
	mov	x0, x22
	mov	x1, xzr
	mov	x2, x21
	mov	w3, wzr
	bl	f2fs_get_new_data_page
	mov	x21, x0
	mov	x0, x23
	bl	up_read
	cmn	x21, #4095              // =4095
	b.hs	.LBB46_11
// %bb.4:
	mov	w2, #1
	mov	w3, #1
	mov	x0, x21
	mov	w1, wzr
	bl	f2fs_wait_on_page_writeback
	//APP
	mrs x22, sp_el0
	//NO_APP
	ldr	w8, [x22, #24]
	add	w8, w8, #1              // =1
	str	w8, [x22, #24]
	//APP
	//NO_APP
	ldr	w8, [x22, #2800]
	add	w8, w8, #1              // =1
	str	w8, [x22, #2800]
	//APP
	//NO_APP
	add	w8, w19, w20
	cmp	w8, #1, lsl #12         // =4096
	b.hi	.LBB46_14
// %bb.5:
	cmp	w8, w20
	b.ls	.LBB46_7
// %bb.6:
	mov	x8, #134217728
	movk	x8, #64, lsl #32
	add	x8, x8, x21, lsl #6
	mov	x9, #-549755813888
	and	x8, x8, #0xfffffffffffff000
	add	x8, x8, x9
	add	x0, x8, w20, uxtw
	and	x2, x19, #0xffffffff
	mov	w1, wzr
	bl	memset
.LBB46_7:
	//APP
	//NO_APP
	ldr	w8, [x22, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x22, #2800]
	//APP
	//NO_APP
	ldr	x8, [x22, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x22, #24]
	b.eq	.LBB46_13
// %bb.8:
	ldr	x8, [x22, #24]
	cbz	x8, .LBB46_13
.LBB46_9:
	mov	x0, x21
	bl	flush_dcache_page
	mov	x0, x21
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x21
	bl	f2fs_put_page
.LBB46_10:
	mov	w21, wzr
.LBB46_11:
	mov	w0, w21
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB46_12:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldr	x1, [x23, #360]
	ldr	x2, [x25, :lo12:jiffies]
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	bl	printk
	ldr	x0, [x23, #352]
	mov	x1, xzr
	bl	show_stack
	b	.LBB46_3
.LBB46_13:
	bl	preempt_schedule
	b	.LBB46_9
.LBB46_14:
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "include/linux/highmem.h"; .popsection; .long 14472b - 14470b; .short 226; .short 0; .popsection; 14471: brk 0x800
	//NO_APP
.Lfunc_end46:
	.size	fill_zero, .Lfunc_end46-fill_zero
                                        // -- End function
	.p2align	2               // -- Begin function __f2fs_ioc_gc_range
	.type	__f2fs_ioc_gc_range,@function
__f2fs_ioc_gc_range:                    // @__f2fs_ioc_gc_range
// %bb.0:
	stp	x29, x30, [sp, #-80]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #32]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #48]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #64]     // 16-byte Folded Spill
	ldr	x8, [x0, #32]
	mov	x19, x0
	mov	w0, #21
	str	x25, [sp, #16]          // 8-byte Folded Spill
	ldr	x8, [x8, #40]
	mov	x29, sp
	mov	x21, x1
	ldr	x20, [x8, #1008]
	bl	capable
	tbz	w0, #0, .LBB47_6
// %bb.1:
	ldr	x8, [x20]
	ldrb	w8, [x8, #80]
	tbnz	w8, #0, .LBB47_7
// %bb.2:
	ldp	x8, x9, [x21, #8]
	adds	x24, x9, x8
	b.hs	.LBB47_33
// %bb.3:
	ldr	x9, [x20, #136]
	cbz	x9, .LBB47_8
// %bb.4:
	ldr	w10, [x9, #84]
	cmp	x8, x10
	b.lo	.LBB47_33
// %bb.5:
	add	x8, x9, #80             // =80
	add	x9, x9, #92             // =92
	b	.LBB47_10
.LBB47_6:
	mov	w23, #-1
	b	.LBB47_34
.LBB47_7:
	mov	w23, #-30
	b	.LBB47_34
.LBB47_8:
	ldr	x9, [x20, #16]
	ldr	w10, [x9, #92]
	cmp	x8, x10
	b.lo	.LBB47_33
// %bb.9:
	add	x8, x9, #72             // =72
	add	x9, x9, #48             // =48
.LBB47_10:
	ldr	w9, [x9]
	ldr	w10, [x20, #1184]
	ldr	w8, [x8]
	lsl	w9, w9, w10
	add	w8, w9, w8
	cmp	x24, x8
	b.hs	.LBB47_33
// %bb.11:
	mov	x0, x19
	bl	mnt_want_write_file
	mov	w23, w0
	cbnz	w0, .LBB47_34
// %bb.12:
	add	x22, x20, #1664         // =1664
	//APP
	mrs x25, sp_el0
	//NO_APP
.LBB47_13:                              // =>This Loop Header: Depth=1
                                        //     Child Loop BB47_22 Depth 2
	ldr	w8, [x21]
	cbz	w8, .LBB47_15
// %bb.14:                              //   in Loop: Header=BB47_13 Depth=1
	mov	x0, x22
	bl	down_write
	b	.LBB47_16
.LBB47_15:                              //   in Loop: Header=BB47_13 Depth=1
	mov	x0, x22
	bl	down_write_trylock
	cbz	w0, .LBB47_37
.LBB47_16:                              //   in Loop: Header=BB47_13 Depth=1
	ldr	w8, [x25, #60]
	orr	w8, w8, #0x20000000
	str	w8, [x25, #60]
	ldr	x9, [x21, #8]
	ldr	w8, [x21]
	add	w10, w9, #3             // =3
	cmp	w10, #4                 // =4
	b.hs	.LBB47_18
.LBB47_17:                              //   in Loop: Header=BB47_13 Depth=1
	mov	w4, #-1
	b	.LBB47_30
.LBB47_18:                              //   in Loop: Header=BB47_13 Depth=1
	ldr	w11, [x20, #3128]
	cbz	w11, .LBB47_26
// %bb.19:                              //   in Loop: Header=BB47_13 Depth=1
	ldr	x10, [x20]
	cmp	w11, #1                 // =1
	ldr	x10, [x10, #248]
	b.lt	.LBB47_25
// %bb.20:                              //   in Loop: Header=BB47_13 Depth=1
	ldr	x12, [x20, #3136]
	add	x12, x12, #80           // =80
	b	.LBB47_22
.LBB47_21:                              //   in Loop: Header=BB47_22 Depth=2
	subs	x11, x11, #1            // =1
	add	x12, x12, #88           // =88
	b.eq	.LBB47_25
.LBB47_22:                              //   Parent Loop BB47_13 Depth=1
                                        // =>  This Inner Loop Header: Depth=2
	ldur	w13, [x12, #-4]
	cmp	w13, w9
	b.hi	.LBB47_21
// %bb.23:                              //   in Loop: Header=BB47_22 Depth=2
	ldr	w13, [x12]
	cmp	w13, w9
	b.lo	.LBB47_21
// %bb.24:                              //   in Loop: Header=BB47_13 Depth=1
	ldur	x10, [x12, #-80]
.LBB47_25:                              //   in Loop: Header=BB47_13 Depth=1
	cbz	x10, .LBB47_17
.LBB47_26:                              //   in Loop: Header=BB47_13 Depth=1
	ldr	x10, [x20, #136]
	cbz	x10, .LBB47_28
// %bb.27:                              //   in Loop: Header=BB47_13 Depth=1
	add	x11, x10, #80           // =80
	b	.LBB47_29
.LBB47_28:                              //   in Loop: Header=BB47_13 Depth=1
	ldr	x11, [x20, #16]
	add	x11, x11, #72           // =72
.LBB47_29:                              //   in Loop: Header=BB47_13 Depth=1
	ldr	x10, [x10, #8]
	ldr	w11, [x11]
	ldr	w12, [x20, #1184]
	ldr	w10, [x10]
	sub	x9, x9, x11
	lsr	x9, x9, x12
	sub	x4, x9, x10
.LBB47_30:                              //   in Loop: Header=BB47_13 Depth=1
	cmp	w8, #0                  // =0
	cset	w1, ne
	mov	w2, #1
	mov	x0, x20
	mov	w3, wzr
                                        // kill: def $w4 killed $w4 killed $x4
	bl	f2fs_gc
	cbnz	w0, .LBB47_35
// %bb.31:                              //   in Loop: Header=BB47_13 Depth=1
	ldr	w8, [x25, #60]
	and	w8, w8, #0xdfffffff
	str	w8, [x25, #60]
	ldr	w8, [x20, #1192]
	ldr	w9, [x20, #1188]
	ldr	x10, [x21, #8]
	mul	w8, w9, w8
	add	x8, x10, x8
	cmp	x8, x24
	str	x8, [x21, #8]
	b.ls	.LBB47_13
// %bb.32:
	mov	w23, wzr
	b	.LBB47_38
.LBB47_33:
	mov	w23, #-22
.LBB47_34:
	mov	w0, w23
	ldp	x20, x19, [sp, #64]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #48]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #32]     // 16-byte Folded Reload
	ldr	x25, [sp, #16]          // 8-byte Folded Reload
	ldp	x29, x30, [sp], #80     // 16-byte Folded Reload
	ret
.LBB47_35:
	mov	w23, w0
	cmn	w0, #16                 // =16
	b.ne	.LBB47_38
// %bb.36:
	mov	w23, #-11
	b	.LBB47_38
.LBB47_37:
	mov	w23, #-16
.LBB47_38:
	mov	x0, x19
	bl	mnt_drop_write_file
	b	.LBB47_34
.Lfunc_end47:
	.size	__f2fs_ioc_gc_range, .Lfunc_end47-__f2fs_ioc_gc_range
                                        // -- End function
	.p2align	2               // -- Begin function __f2fs_ioc_move_range
	.type	__f2fs_ioc_move_range,@function
__f2fs_ioc_move_range:                  // @__f2fs_ioc_move_range
// %bb.0:
	sub	sp, sp, #144            // =144
	stp	x29, x30, [sp, #48]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #64]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #80]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #96]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #112]    // 16-byte Folded Spill
	stp	x20, x19, [sp, #128]    // 16-byte Folded Spill
	ldr	w8, [x0, #68]
	add	x29, sp, #48            // =48
	mvn	w8, w8
	tst	w8, #0x3
	b.ne	.LBB48_4
// %bb.1:
	mov	x21, x0
	ldr	w0, [x1]
	mov	x24, x1
	bl	__fdget
	ands	x19, x0, #0xfffffffffffffffc
	b.eq	.LBB48_4
// %bb.2:
	ldrb	w8, [x19, #68]
	mov	x20, x0
	tbnz	w8, #1, .LBB48_6
// %bb.3:
	mov	w26, #-9
	b	.LBB48_11
.LBB48_4:
	mov	w26, #-9
.LBB48_5:
	mov	w0, w26
	ldp	x20, x19, [sp, #128]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #112]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #96]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #80]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #64]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #48]     // 16-byte Folded Reload
	add	sp, sp, #144            // =144
	ret
.LBB48_6:
	mov	x0, x21
	bl	mnt_want_write_file
	mov	w26, w0
	cbnz	w0, .LBB48_11
// %bb.7:
	ldr	x8, [x21, #16]
	ldr	x9, [x19, #16]
	cmp	x8, x9
	b.ne	.LBB48_9
// %bb.8:
	ldr	x23, [x21, #32]
	ldr	x22, [x19, #32]
	ldr	x8, [x23, #40]
	ldr	x9, [x22, #40]
	cmp	x8, x9
	b.eq	.LBB48_13
.LBB48_9:
	mov	w26, #-18
.LBB48_10:
	mov	x0, x21
	bl	mnt_drop_write_file
.LBB48_11:
	tbz	w20, #0, .LBB48_5
// %bb.12:
	mov	x0, x19
	bl	fput
	b	.LBB48_5
.LBB48_13:
	ldrb	w9, [x8, #80]
	tbnz	w9, #0, .LBB48_55
// %bb.14:
	ldrh	w9, [x23]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB48_49
// %bb.15:
	ldrh	w9, [x22]
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB48_49
// %bb.16:
	ldrb	w9, [x23, #13]
	tbnz	w9, #6, .LBB48_44
// %bb.17:
	ldrb	w9, [x22, #13]
	tbnz	w9, #6, .LBB48_44
// %bb.18:
	ldp	x25, x28, [x24, #8]
	orr	x9, x28, x25
	tbnz	x9, #63, .LBB48_49
// %bb.19:
	ldr	x27, [x24, #24]
	ldr	x8, [x8, #1008]
	cmp	x23, x22
	b.eq	.LBB48_45
.LBB48_20:
	add	x24, x23, #160          // =160
	mov	x0, x24
	stur	x8, [x29, #-8]          // 8-byte Folded Spill
	bl	down_write
	cmp	x23, x22
	b.eq	.LBB48_22
// %bb.21:
	add	x0, x22, #160           // =160
	bl	down_write_trylock
	cbz	w0, .LBB48_50
.LBB48_22:
	ldr	x8, [x23, #736]
	tbz	x8, #35, .LBB48_24
// %bb.23:
	mov	w1, #5
	mov	x0, x23
	bl	f2fs_reserve_compress_blocks_internal
	mov	w26, w0
	cbnz	w0, .LBB48_52
.LBB48_24:
	ldr	x8, [x22, #736]
	tbz	x8, #35, .LBB48_26
// %bb.25:
	mov	w1, #6
	mov	x0, x22
	bl	f2fs_reserve_compress_blocks_internal
	mov	w26, w0
	cbnz	w0, .LBB48_52
.LBB48_26:
	ldr	x8, [x23, #736]
	tbz	x8, #44, .LBB48_28
// %bb.27:
	add	x8, x23, #736           // =736
	mov	x9, #17592186044416
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x23
	bl	f2fs_mark_inode_dirty_sync
.LBB48_28:
	ldr	x8, [x23, #736]
	tbz	x8, #38, .LBB48_32
// %bb.29:
	ldr	x8, [x23, #736]
	tbz	x8, #38, .LBB48_31
// %bb.30:
	ldr	x8, [x23, #736]
	tbnz	x8, #39, .LBB48_32
.LBB48_31:
	adrp	x1, .L__func__.f2fs_move_file_range
	add	x1, x1, :lo12:.L__func__.f2fs_move_file_range
	mov	x0, x23
	bl	f2fs_revoke_deduped_inode
	mov	w26, w0
	cbnz	w0, .LBB48_52
.LBB48_32:
	ldr	x8, [x22, #736]
	tbz	x8, #44, .LBB48_34
// %bb.33:
	add	x8, x22, #736           // =736
	mov	x9, #17592186044416
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
1:	ldxr	x10, [x8]
	bic	x10, x10, x9
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	//NO_APP
	mov	w1, #1
	mov	x0, x22
	bl	f2fs_mark_inode_dirty_sync
.LBB48_34:
	ldr	x8, [x22, #736]
	tbz	x8, #38, .LBB48_38
// %bb.35:
	ldr	x8, [x22, #736]
	tbz	x8, #38, .LBB48_37
// %bb.36:
	ldr	x8, [x22, #736]
	tbnz	x8, #39, .LBB48_38
.LBB48_37:
	adrp	x1, .L__func__.f2fs_move_file_range
	add	x1, x1, :lo12:.L__func__.f2fs_move_file_range
	mov	x0, x22
	bl	f2fs_revoke_deduped_inode
	mov	w26, w0
	cbnz	w0, .LBB48_52
.LBB48_38:
	adds	x9, x27, x25
	mov	w26, #-22
	b.hs	.LBB48_52
// %bb.39:
	ldr	x8, [x23, #80]
	cmp	x9, x8
	b.hi	.LBB48_52
// %bb.40:
	sub	x9, x8, x25
	cmp	x27, #0                 // =0
	add	x10, x8, #4095          // =4095
	csel	x9, x9, x27, eq
	and	x10, x10, #0xfffffffffffff000
	add	x11, x9, x25
	sub	x10, x10, x25
	cmp	x11, x8
	csel	x27, x10, x9, eq
	cbz	x27, .LBB48_51
// %bb.41:
	ldr	x10, [x22, #80]
	add	x8, x9, x28
	cmp	x8, x10
	csel	x8, x8, xzr, hi
	tst	x25, #0xfff
	str	x10, [sp, #24]          // 8-byte Folded Spill
	stur	x8, [x29, #-16]         // 8-byte Folded Spill
	b.ne	.LBB48_43
// %bb.42:
	add	x8, x27, x25
	str	x8, [sp, #16]           // 8-byte Folded Spill
	orr	w8, w8, w28
	tst	x8, #0xfff
	b.eq	.LBB48_56
.LBB48_43:
	mov	w26, #-22
	b	.LBB48_52
.LBB48_44:
	mov	w26, #-95
	b	.LBB48_10
.LBB48_45:
	cmp	x25, x28
	b.ne	.LBB48_47
// %bb.46:
	mov	w26, wzr
	b	.LBB48_10
.LBB48_47:
	cmp	x28, x25
	b.le	.LBB48_20
// %bb.48:
	add	x9, x27, x25
	cmp	x9, x28
	b.ls	.LBB48_20
.LBB48_49:
	mov	w26, #-22
	b	.LBB48_10
.LBB48_50:
	mov	w26, #-16
	b	.LBB48_54
.LBB48_51:
	mov	w26, wzr
.LBB48_52:
	cmp	x23, x22
	b.eq	.LBB48_54
// %bb.53:
	add	x0, x22, #160           // =160
	bl	up_write
.LBB48_54:
	mov	x0, x24
	bl	up_write
	b	.LBB48_10
.LBB48_55:
	mov	w26, #-30
	b	.LBB48_10
.LBB48_56:
	mov	x0, x23
	bl	f2fs_convert_inline_inode
	mov	w26, w0
	cbnz	w0, .LBB48_52
// %bb.57:
	mov	x0, x22
	bl	f2fs_convert_inline_inode
	mov	w26, w0
	cbnz	w0, .LBB48_52
// %bb.58:
	ldr	x0, [x23, #48]
	ldr	x2, [sp, #16]           // 8-byte Folded Reload
	mov	x1, x25
	bl	filemap_write_and_wait_range
	mov	w26, w0
	cbnz	w0, .LBB48_52
// %bb.59:
	ldr	x0, [x22, #48]
	mov	x1, x28
	add	x2, x27, x28
	bl	filemap_write_and_wait_range
	mov	w26, w0
	cbnz	w0, .LBB48_52
// %bb.60:
	ldur	x0, [x29, #-8]          // 8-byte Folded Reload
	mov	w1, #1
	bl	f2fs_balance_fs
	add	x0, x23, #1040          // =1040
	str	x0, [sp, #16]           // 8-byte Folded Spill
	bl	down_write
	cmp	x23, x22
	b.eq	.LBB48_62
// %bb.61:
	add	x0, x22, #1040          // =1040
	bl	down_write_trylock
	cbz	w0, .LBB48_72
.LBB48_62:
	ldur	x26, [x29, #-8]         // 8-byte Folded Reload
	add	x0, x26, #368           // =368
	str	x0, [sp, #8]            // 8-byte Folded Spill
	bl	_raw_spin_lock
	ldr	x8, [x26, #352]
	cbz	x8, .LBB48_64
// %bb.63:
	ldur	x8, [x29, #-8]          // 8-byte Folded Reload
	adrp	x9, jiffies
	mov	x26, x9
	ldr	x8, [x8, #360]
	ldr	x9, [x9, :lo12:jiffies]
	sub	x8, x8, x9
	cmn	x8, #2500               // =2500
	b.mi	.LBB48_78
.LBB48_64:
	ldr	x0, [sp, #8]            // 8-byte Folded Reload
	bl	_raw_spin_unlock
	ldur	x8, [x29, #-8]          // 8-byte Folded Reload
	add	x0, x8, #304            // =304
	stur	x0, [x29, #-8]          // 8-byte Folded Spill
	bl	down_read
	asr	x2, x25, #12
	asr	x3, x28, #12
	lsr	x4, x27, #12
	mov	x0, x23
	mov	x1, x22
	mov	w5, wzr
	bl	__exchange_data_block
	mov	w26, w0
	cbz	w0, .LBB48_67
.LBB48_65:
	ldur	x0, [x29, #-8]          // 8-byte Folded Reload
	bl	up_read
	cmp	x23, x22
	b.eq	.LBB48_73
// %bb.66:
	add	x0, x22, #1040          // =1040
	bl	up_write
	b	.LBB48_73
.LBB48_67:
	ldur	x8, [x29, #-16]         // 8-byte Folded Reload
	cbz	x8, .LBB48_74
// %bb.68:
	ldr	x27, [x22, #736]
	ldr	x25, [x22, #736]
	ldr	x8, [x22, #80]
	ldur	x9, [x29, #-16]         // 8-byte Folded Reload
	cmp	x8, x9
	b.eq	.LBB48_65
// %bb.69:
	ldur	x8, [x29, #-16]         // 8-byte Folded Reload
	mov	w1, #1
	mov	x0, x22
	str	x8, [x22, #80]
	bl	f2fs_mark_inode_dirty_sync
	tbz	w27, #1, .LBB48_71
// %bb.70:
	tbz	w25, #2, .LBB48_65
.LBB48_71:
	add	x8, x22, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB48_65
.LBB48_72:
	mov	w26, #-16
.LBB48_73:
	ldr	x0, [sp, #16]           // 8-byte Folded Reload
	bl	up_write
	b	.LBB48_52
.LBB48_74:
	ldr	x8, [x22, #80]
	ldr	x9, [sp, #24]           // 8-byte Folded Reload
	cmp	x9, x8
	b.eq	.LBB48_65
// %bb.75:
	ldr	x27, [x22, #736]
	ldr	x25, [x22, #736]
	ldr	x8, [sp, #24]           // 8-byte Folded Reload
	mov	w1, #1
	mov	x0, x22
	str	x8, [x22, #80]
	bl	f2fs_mark_inode_dirty_sync
	tbz	w27, #1, .LBB48_77
// %bb.76:
	tbz	w25, #2, .LBB48_65
.LBB48_77:
	add	x8, x22, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
	b	.LBB48_65
.LBB48_78:
	mov	x1, xzr
	//APP
	mrs x0, sp_el0
	//NO_APP
	bl	show_stack
	ldur	x8, [x29, #-8]          // 8-byte Folded Reload
	adrp	x0, .L.str.31
	add	x0, x0, :lo12:.L.str.31
	ldr	x1, [x8, #360]
	ldr	x2, [x26, :lo12:jiffies]
	bl	printk
	ldur	x8, [x29, #-8]          // 8-byte Folded Reload
	mov	x1, xzr
	ldr	x0, [x8, #352]
	bl	show_stack
	b	.LBB48_64
.Lfunc_end48:
	.size	__f2fs_ioc_move_range, .Lfunc_end48-__f2fs_ioc_move_range
                                        // -- End function
	.p2align	2               // -- Begin function __exchange_data_block
	.type	__exchange_data_block,@function
__exchange_data_block:                  // @__exchange_data_block
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
	str	x3, [sp, #48]           // 8-byte Folded Spill
	stur	x8, [x29, #-8]
	cbz	x4, .LBB49_117
// %bb.1:
	mov	x21, x4
	mov	x19, x2
	mov	x26, x1
	mov	x20, x0
	mov	w28, w5
	str	w5, [sp, #36]           // 4-byte Folded Spill
.LBB49_2:                               // =>This Loop Header: Depth=1
                                        //     Child Loop BB49_10 Depth 2
                                        //       Child Loop BB49_26 Depth 3
                                        //         Child Loop BB49_37 Depth 4
                                        //     Child Loop BB49_63 Depth 2
                                        //       Child Loop BB49_84 Depth 3
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB49_4
// %bb.3:                               //   in Loop: Header=BB49_2 Depth=1
	ldr	x8, [x20, #736]
	tbnz	w8, #31, .LBB49_58
.LBB49_4:                               //   in Loop: Header=BB49_2 Depth=1
	mov	w23, #1018
.LBB49_5:                               //   in Loop: Header=BB49_2 Depth=1
	lsl	w8, w23, #2
	cmp	x21, x8
	csel	x25, x8, x21, hi
	lsl	x22, x25, #2
	mov	w1, #3392
	mov	w2, #-1
	mov	x0, x22
	bl	kvmalloc_node
	str	x0, [sp, #80]           // 8-byte Folded Spill
	cbz	x0, .LBB49_152
// %bb.6:                               //   in Loop: Header=BB49_2 Depth=1
	mov	w1, #3392
	mov	w2, #-1
	mov	x0, x22
	bl	kvmalloc_node
	cbz	x0, .LBB49_153
// %bb.7:                               //   in Loop: Header=BB49_2 Depth=1
	ldr	x8, [x20, #40]
	str	x23, [sp, #24]          // 8-byte Folded Spill
	ldr	x23, [sp, #80]          // 8-byte Folded Reload
	mov	x28, x19
	ldr	x27, [x8, #1008]
	mov	x22, x0
	str	x21, [sp, #40]          // 8-byte Folded Spill
	stp	xzr, xzr, [x29, #-24]
	stp	xzr, xzr, [x29, #-40]
	stp	xzr, xzr, [x29, #-56]
	stp	x25, x0, [sp, #56]      // 16-byte Folded Spill
	b	.LBB49_10
.LBB49_8:                               //   in Loop: Header=BB49_10 Depth=2
	sxtw	x8, w24
	stp	xzr, xzr, [x29, #-48]
.LBB49_9:                               //   in Loop: Header=BB49_10 Depth=2
	subs	x25, x25, x8
	add	x28, x8, x28
	b.eq	.LBB49_59
.LBB49_10:                              //   Parent Loop BB49_2 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB49_26 Depth 3
                                        //         Child Loop BB49_37 Depth 4
	sub	x0, x29, #56            // =56
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	sub	x0, x29, #56            // =56
	mov	w2, #2
	mov	x1, x28
	stp	x20, xzr, [x29, #-56]
	stur	xzr, [x29, #-40]
	stur	wzr, [x29, #-32]
	bl	f2fs_get_dnode_of_data
	cbz	w0, .LBB49_17
// %bb.11:                              //   in Loop: Header=BB49_10 Depth=2
	mov	w24, w0
	cmn	w0, #2                  // =2
	b.ne	.LBB49_135
// %bb.12:                              //   in Loop: Header=BB49_10 Depth=2
	ldurb	w8, [x29, #-21]
	cbz	w8, .LBB49_134
// %bb.13:                              //   in Loop: Header=BB49_10 Depth=2
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB49_15
// %bb.14:                              //   in Loop: Header=BB49_10 Depth=2
	ldr	x8, [x20, #736]
	tbnz	w8, #31, .LBB49_56
.LBB49_15:                              //   in Loop: Header=BB49_10 Depth=2
	mov	w8, #1018
.LBB49_16:                              //   in Loop: Header=BB49_10 Depth=2
	ldur	w9, [x29, #-28]
	sub	x8, x8, x9
	cmp	x8, x25
	csel	x9, x8, x25, lo
	sxtw	x8, w9
	sbfiz	x9, x9, #2, #32
	add	x23, x23, x9
	add	x22, x22, x9
	b	.LBB49_9
.LBB49_17:                              //   in Loop: Header=BB49_10 Depth=2
	ldur	x0, [x29, #-40]
	mov	x8, #2097152
	movk	x8, #1, lsl #32
	mov	x9, #-549755813888
	add	x8, x0, x8
	lsl	x8, x8, #6
	mov	x10, #-549755813888
	and	x8, x8, #0xfffffffffffff000
	movk	x9, #4072
	movk	x10, #4076
	ldr	w9, [x8, x9]
	ldr	w8, [x8, x10]
	cmp	w9, w8
	b.ne	.LBB49_21
// %bb.18:                              //   in Loop: Header=BB49_10 Depth=2
	ldr	w8, [x20, #1184]
	ldr	w9, [x20, #1192]
	ldrh	w10, [x20]
	add	w8, w9, w8, asr #2
	and	w9, w10, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	mov	w9, #923
	sub	w9, w9, w8
	b.ne	.LBB49_24
// %bb.19:                              //   in Loop: Header=BB49_10 Depth=2
	ldr	x8, [x20, #736]
	tbz	w8, #31, .LBB49_24
// %bb.20:                              //   in Loop: Header=BB49_10 Depth=2
	ldr	w8, [x20, #1292]
	neg	w8, w8
	and	w9, w9, w8
	b	.LBB49_24
.LBB49_21:                              //   in Loop: Header=BB49_10 Depth=2
	ldrh	w8, [x20]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB49_23
// %bb.22:                              //   in Loop: Header=BB49_10 Depth=2
	ldr	x8, [x20, #736]
	tbnz	w8, #31, .LBB49_57
.LBB49_23:                              //   in Loop: Header=BB49_10 Depth=2
	mov	w9, #1018
.LBB49_24:                              //   in Loop: Header=BB49_10 Depth=2
	ldur	w8, [x29, #-28]
	mov	w9, w9
	sub	x9, x9, x8
	cmp	x9, x25
	csel	x24, x9, x25, lo
	cmp	w24, #1                 // =1
	b.lt	.LBB49_50
// %bb.25:                              //   in Loop: Header=BB49_10 Depth=2
	mov	w21, wzr
.LBB49_26:                              //   Parent Loop BB49_2 Depth=1
                                        //     Parent Loop BB49_10 Depth=2
                                        // =>    This Loop Header: Depth=3
                                        //         Child Loop BB49_37 Depth 4
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	add	x9, x0, x9
	lsl	x9, x9, #6
	and	x11, x9, #0xfffffffffffff000
	mov	x9, #-549755813888
	mov	x10, #-549755813888
	movk	x9, #4072
	movk	x10, #4076
	ldr	w9, [x11, x9]
	ldr	w10, [x11, x10]
	mov	x12, #-549755813888
	add	x11, x11, x12
	cmp	w9, w10
	b.ne	.LBB49_31
// %bb.27:                              //   in Loop: Header=BB49_26 Depth=3
	ldur	x12, [x29, #-56]
	cbz	x12, .LBB49_30
// %bb.28:                              //   in Loop: Header=BB49_26 Depth=3
	ldr	x13, [x12, #736]
	tbz	w13, #26, .LBB49_31
// %bb.29:                              //   in Loop: Header=BB49_26 Depth=3
	ldr	w12, [x12, #1184]
	asr	w12, w12, #2
	b	.LBB49_32
.LBB49_30:                              //   in Loop: Header=BB49_26 Depth=3
	ldrb	w12, [x11, #3]
	tbnz	w12, #5, .LBB49_48
.LBB49_31:                              //   in Loop: Header=BB49_26 Depth=3
	mov	w12, wzr
.LBB49_32:                              //   in Loop: Header=BB49_26 Depth=3
	add	x13, x11, #360          // =360
	cmp	w9, w10
	csel	x9, x13, x11, eq
	add	w8, w12, w8
	ldr	w1, [x9, w8, uxtw #2]
	add	w8, w1, #3              // =3
	cmp	w8, #4                  // =4
	str	w1, [x23]
	b.lo	.LBB49_43
// %bb.33:                              //   in Loop: Header=BB49_26 Depth=3
	ldr	w9, [x27, #3128]
	cbz	w9, .LBB49_41
// %bb.34:                              //   in Loop: Header=BB49_26 Depth=3
	ldr	x8, [x27]
	cmp	w9, #1                  // =1
	ldr	x8, [x8, #248]
	b.lt	.LBB49_40
// %bb.35:                              //   in Loop: Header=BB49_26 Depth=3
	ldr	x10, [x27, #3136]
	add	x10, x10, #80           // =80
	b	.LBB49_37
.LBB49_36:                              //   in Loop: Header=BB49_37 Depth=4
	subs	x9, x9, #1              // =1
	add	x10, x10, #88           // =88
	b.eq	.LBB49_40
.LBB49_37:                              //   Parent Loop BB49_2 Depth=1
                                        //     Parent Loop BB49_10 Depth=2
                                        //       Parent Loop BB49_26 Depth=3
                                        // =>      This Inner Loop Header: Depth=4
	ldur	w11, [x10, #-4]
	cmp	w11, w1
	b.hi	.LBB49_36
// %bb.38:                              //   in Loop: Header=BB49_37 Depth=4
	ldr	w11, [x10]
	cmp	w11, w1
	b.lo	.LBB49_36
// %bb.39:                              //   in Loop: Header=BB49_26 Depth=3
	ldur	x8, [x10, #-80]
.LBB49_40:                              //   in Loop: Header=BB49_26 Depth=3
	cbz	x8, .LBB49_43
.LBB49_41:                              //   in Loop: Header=BB49_26 Depth=3
	mov	w2, #7
	mov	x0, x27
	bl	f2fs_is_valid_blkaddr
	tbz	w0, #0, .LBB49_120
// %bb.42:                              //   in Loop: Header=BB49_26 Depth=3
	ldr	w1, [x23]
.LBB49_43:                              //   in Loop: Header=BB49_26 Depth=3
	mov	x0, x27
	bl	f2fs_is_checkpointed_data
	tbnz	w0, #0, .LBB49_46
// %bb.44:                              //   in Loop: Header=BB49_26 Depth=3
	ldr	w8, [x27, #1504]
	cmp	w8, #1                  // =1
	b.eq	.LBB49_118
// %bb.45:                              //   in Loop: Header=BB49_26 Depth=3
	sub	x0, x29, #56            // =56
	mov	w1, wzr
	bl	f2fs_update_data_blkaddr
	mov	w8, #1
	str	w8, [x22]
.LBB49_46:                              //   in Loop: Header=BB49_26 Depth=3
	ldur	w8, [x29, #-28]
	add	w21, w21, #1            // =1
	add	x23, x23, #4            // =4
	add	x22, x22, #4            // =4
	add	w8, w8, #1              // =1
	cmp	w21, w24
	stur	w8, [x29, #-28]
	b.ge	.LBB49_49
// %bb.47:                              //   in Loop: Header=BB49_26 Depth=3
	ldur	x0, [x29, #-40]
	b	.LBB49_26
.LBB49_48:                              //   in Loop: Header=BB49_26 Depth=3
	ldrh	w12, [x11, #360]
	lsr	w12, w12, #2
	b	.LBB49_32
.LBB49_49:                              //   in Loop: Header=BB49_10 Depth=2
	ldur	x0, [x29, #-40]
.LBB49_50:                              //   in Loop: Header=BB49_10 Depth=2
	cbz	x0, .LBB49_52
// %bb.51:                              //   in Loop: Header=BB49_10 Depth=2
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	ldur	x8, [x29, #-48]
	cbnz	x8, .LBB49_53
	b	.LBB49_8
.LBB49_52:                              //   in Loop: Header=BB49_10 Depth=2
	mov	x9, xzr
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB49_8
.LBB49_53:                              //   in Loop: Header=BB49_10 Depth=2
	cmp	x9, x8
	b.eq	.LBB49_8
// %bb.54:                              //   in Loop: Header=BB49_10 Depth=2
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
	cbnz	w9, .LBB49_8
// %bb.55:                              //   in Loop: Header=BB49_10 Depth=2
	bl	__put_page
	b	.LBB49_8
.LBB49_56:                              //   in Loop: Header=BB49_10 Depth=2
	ldr	w8, [x20, #1292]
	mov	w9, #1018
	neg	w8, w8
	and	w8, w8, w9
	b	.LBB49_16
.LBB49_57:                              //   in Loop: Header=BB49_10 Depth=2
	ldr	w8, [x20, #1292]
	mov	w9, #1018
	neg	w8, w8
	and	w9, w8, w9
	b	.LBB49_24
.LBB49_58:                              //   in Loop: Header=BB49_2 Depth=1
	ldr	w8, [x20, #1292]
	mov	w9, #1018
	neg	w8, w8
	and	w23, w8, w9
	b	.LBB49_5
.LBB49_59:                              //   in Loop: Header=BB49_2 Depth=1
	ldr	x27, [sp, #56]          // 8-byte Folded Reload
	ldr	w28, [sp, #36]          // 4-byte Folded Reload
	cbz	x27, .LBB49_116
// %bb.60:                              //   in Loop: Header=BB49_2 Depth=1
	ldr	x8, [x20, #40]
	mov	x23, xzr
	ldr	x8, [x8, #1008]
	str	x8, [sp, #72]           // 8-byte Folded Spill
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	add	x8, x8, #1              // =1
	str	x8, [sp, #16]           // 8-byte Folded Spill
	ldr	x8, [sp, #80]           // 8-byte Folded Reload
	add	x8, x8, #4              // =4
	str	x8, [sp, #8]            // 8-byte Folded Spill
	b	.LBB49_63
.LBB49_61:                              //   in Loop: Header=BB49_63 Depth=2
	add	x22, x23, #1            // =1
.LBB49_62:                              //   in Loop: Header=BB49_63 Depth=2
	cmp	x22, x27
	mov	x23, x22
	b.hs	.LBB49_116
.LBB49_63:                              //   Parent Loop BB49_2 Depth=1
                                        // =>  This Loop Header: Depth=2
                                        //       Child Loop BB49_84 Depth 3
	ldr	x8, [sp, #80]           // 8-byte Folded Reload
	ldr	w8, [x8, x23, lsl #2]
	cbnz	w8, .LBB49_65
// %bb.64:                              //   in Loop: Header=BB49_63 Depth=2
	tbz	w28, #0, .LBB49_61
.LBB49_65:                              //   in Loop: Header=BB49_63 Depth=2
	cbz	w8, .LBB49_71
// %bb.66:                              //   in Loop: Header=BB49_63 Depth=2
	ldr	x8, [sp, #64]           // 8-byte Folded Reload
	ldr	w8, [x8, x23, lsl #2]
	cbnz	w8, .LBB49_71
// %bb.67:                              //   in Loop: Header=BB49_63 Depth=2
	add	x28, x23, x19
	mov	w2, #1
	mov	x0, x20
	mov	x1, x28
	bl	f2fs_get_lock_data_page
	mov	x25, x0
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB49_113
// %bb.68:                              //   in Loop: Header=BB49_63 Depth=2
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	mov	w3, #1
	mov	x0, x26
	mov	x1, xzr
	add	x2, x23, x8
	bl	f2fs_get_new_data_page
	mov	x24, x0
	cmn	x0, #4095               // =4095
	b.hs	.LBB49_114
// %bb.69:                              //   in Loop: Header=BB49_63 Depth=2
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	add	x8, x25, x9
	add	x9, x24, x9
	lsl	x8, x8, #6
	lsl	x9, x9, #6
	and	x8, x8, #0xfffffffffffff000
	and	x9, x9, #0xfffffffffffff000
	mov	x10, #-549755813888
	add	x1, x8, x10
	add	x0, x9, x10
	mov	w2, #4096
	bl	memcpy
	mov	x0, x24
	bl	set_page_dirty
	mov	w1, #1
	mov	x0, x24
	bl	f2fs_put_page
	mov	w1, #1
	mov	x0, x25
	bl	f2fs_put_page
	add	x2, x28, #1             // =1
	mov	x0, x20
	mov	x1, x28
	bl	f2fs_truncate_hole
	cbnz	w0, .LBB49_158
// %bb.70:                              //   in Loop: Header=BB49_63 Depth=2
	ldr	x27, [sp, #56]          // 8-byte Folded Reload
	ldr	w28, [sp, #36]          // 4-byte Folded Reload
	b	.LBB49_61
.LBB49_71:                              //   in Loop: Header=BB49_63 Depth=2
	sub	x0, x29, #56            // =56
	mov	w2, #48
	mov	w1, wzr
	stp	xzr, xzr, [x29, #-24]
	stp	xzr, xzr, [x29, #-40]
	stp	xzr, xzr, [x29, #-56]
	stp	xzr, xzr, [x29, #-72]
	bl	memset
	ldr	x8, [sp, #48]           // 8-byte Folded Reload
	sub	x0, x29, #56            // =56
	mov	w2, wzr
	stp	x26, xzr, [x29, #-56]
	add	x1, x23, x8
	stur	xzr, [x29, #-40]
	stur	wzr, [x29, #-32]
	bl	f2fs_get_dnode_of_data
	cbnz	w0, .LBB49_154
// %bb.72:                              //   in Loop: Header=BB49_63 Depth=2
	ldur	w1, [x29, #-32]
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	sub	x2, x29, #72            // =72
	mov	w3, wzr
	bl	get_node_info_ex
	ldur	x8, [x29, #-40]
	cbnz	w0, .LBB49_156
// %bb.73:                              //   in Loop: Header=BB49_63 Depth=2
	mov	x9, #2097152
	movk	x9, #1, lsl #32
	add	x9, x8, x9
	lsl	x9, x9, #6
	mov	x10, #-549755813888
	mov	x11, #-549755813888
	and	x9, x9, #0xfffffffffffff000
	movk	x10, #4072
	movk	x11, #4076
	ldr	w10, [x9, x10]
	ldr	w9, [x9, x11]
	cmp	w10, w9
	b.ne	.LBB49_77
// %bb.74:                              //   in Loop: Header=BB49_63 Depth=2
	ldr	w9, [x26, #1184]
	ldr	w10, [x26, #1192]
	ldrh	w11, [x26]
	ldr	x14, [sp, #64]          // 8-byte Folded Reload
	add	w9, w10, w9, asr #2
	and	w10, w11, #0xf000
	cmp	w10, #8, lsl #12        // =32768
	mov	w10, #923
	sub	w9, w10, w9
	b.ne	.LBB49_82
// %bb.75:                              //   in Loop: Header=BB49_63 Depth=2
	ldr	x10, [x26, #736]
	tbz	w10, #31, .LBB49_82
// %bb.76:                              //   in Loop: Header=BB49_63 Depth=2
	ldr	w10, [x26, #1292]
	neg	w10, w10
	b	.LBB49_81
.LBB49_77:                              //   in Loop: Header=BB49_63 Depth=2
	ldrh	w9, [x26]
	ldr	x14, [sp, #64]          // 8-byte Folded Reload
	and	w9, w9, #0xf000
	cmp	w9, #8, lsl #12         // =32768
	b.ne	.LBB49_79
// %bb.78:                              //   in Loop: Header=BB49_63 Depth=2
	ldr	x9, [x26, #736]
	tbnz	w9, #31, .LBB49_80
.LBB49_79:                              //   in Loop: Header=BB49_63 Depth=2
	mov	w9, #1018
	b	.LBB49_82
.LBB49_80:                              //   in Loop: Header=BB49_63 Depth=2
	ldr	w9, [x26, #1292]
	mov	w10, #1018
	neg	w9, w9
.LBB49_81:                              //   in Loop: Header=BB49_63 Depth=2
	and	w9, w9, w10
.LBB49_82:                              //   in Loop: Header=BB49_63 Depth=2
	mov	w10, w9
	ldur	w9, [x29, #-28]
	sub	x11, x27, x23
	ldr	x12, [sp, #16]          // 8-byte Folded Reload
	lsl	x13, x23, #2
	sub	x10, x10, x9
	cmp	x10, x11
	csel	x10, x10, x11, lo
	mov	w11, #1
	sub	x25, x11, x10
	ldr	x10, [sp, #8]           // 8-byte Folded Reload
	add	x12, x12, x23
	add	x22, x23, #1            // =1
	lsl	x23, x12, #12
	add	x24, x14, x13
	add	x28, x10, x13
	b	.LBB49_84
.LBB49_83:                              //   in Loop: Header=BB49_84 Depth=3
	ldur	x8, [x29, #-40]
	ldur	w9, [x29, #-28]
	add	x22, x22, #1            // =1
	add	x23, x23, #1, lsl #12   // =4096
	add	x25, x25, #1            // =1
	add	x24, x24, #4            // =4
	add	x28, x28, #4            // =4
.LBB49_84:                              //   Parent Loop BB49_2 Depth=1
                                        //     Parent Loop BB49_63 Depth=2
                                        // =>    This Inner Loop Header: Depth=3
	mov	x10, #2097152
	movk	x10, #1, lsl #32
	add	x8, x8, x10
	lsl	x8, x8, #6
	and	x11, x8, #0xfffffffffffff000
	mov	x8, #-549755813888
	mov	x10, #-549755813888
	movk	x8, #4072
	movk	x10, #4076
	ldr	w8, [x11, x8]
	ldr	w10, [x11, x10]
	mov	x12, #-549755813888
	add	x11, x11, x12
	cmp	w8, w10
	b.ne	.LBB49_89
// %bb.85:                              //   in Loop: Header=BB49_84 Depth=3
	ldur	x12, [x29, #-56]
	cbz	x12, .LBB49_88
// %bb.86:                              //   in Loop: Header=BB49_84 Depth=3
	ldr	x13, [x12, #736]
	tbz	w13, #26, .LBB49_89
// %bb.87:                              //   in Loop: Header=BB49_84 Depth=3
	ldr	w12, [x12, #1184]
	asr	w12, w12, #2
	b	.LBB49_90
.LBB49_88:                              //   in Loop: Header=BB49_84 Depth=3
	ldrb	w12, [x11, #3]
	tbnz	w12, #5, .LBB49_105
.LBB49_89:                              //   in Loop: Header=BB49_84 Depth=3
	mov	w12, wzr
.LBB49_90:                              //   in Loop: Header=BB49_84 Depth=3
	add	x13, x11, #360          // =360
	cmp	w8, w10
	csel	x8, x13, x11, eq
	add	w9, w12, w9
	ldr	w8, [x8, w9, uxtw #2]
	sub	x0, x29, #56            // =56
	mov	w1, #1
	stur	w8, [x29, #-20]
	bl	f2fs_truncate_data_blocks_range
	ldr	w8, [x24]
	cbz	w8, .LBB49_98
// %bb.91:                              //   in Loop: Header=BB49_84 Depth=3
	ldr	x27, [x20, #736]
	ldr	x21, [x20, #736]
	ldrb	w8, [x20, #142]
	mov	w9, #1
	mov	x0, x20
	mov	w2, wzr
	lsl	x1, x9, x8
	bl	__dquot_free_space
	mov	w1, #1
	mov	x0, x20
	bl	__mark_inode_dirty
	mov	w1, #1
	mov	x0, x20
	bl	f2fs_mark_inode_dirty_sync
	tbz	w27, #1, .LBB49_93
// %bb.92:                              //   in Loop: Header=BB49_84 Depth=3
	tbz	w21, #2, .LBB49_94
.LBB49_93:                              //   in Loop: Header=BB49_84 Depth=3
	add	x8, x20, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB49_94:                              //   in Loop: Header=BB49_84 Depth=3
	ldr	x27, [x26, #736]
	ldr	x21, [x26, #736]
	ldrb	w8, [x26, #142]
	mov	w9, #1
	mov	w2, #5
	mov	x0, x26
	lsl	x1, x9, x8
	bl	__dquot_alloc_space
	mov	w1, #1
	mov	x0, x26
	bl	__mark_inode_dirty
	mov	w1, #1
	mov	x0, x26
	bl	f2fs_mark_inode_dirty_sync
	tbz	w27, #1, .LBB49_96
// %bb.95:                              //   in Loop: Header=BB49_84 Depth=3
	tbz	w21, #2, .LBB49_97
.LBB49_96:                              //   in Loop: Header=BB49_84 Depth=3
	add	x8, x26, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB49_97:                              //   in Loop: Header=BB49_84 Depth=3
	ldur	w2, [x29, #-20]
	ldur	w3, [x28, #-4]
	ldurb	w4, [x29, #-60]
	ldr	x0, [sp, #72]           // 8-byte Folded Reload
	sub	x1, x29, #56            // =56
	mov	w5, #1
	mov	w6, wzr
	bl	f2fs_replace_block
	str	wzr, [x24]
.LBB49_98:                              //   in Loop: Header=BB49_84 Depth=3
	ldur	w8, [x29, #-28]
	add	w8, w8, #1              // =1
	stur	w8, [x29, #-28]
	ldr	x8, [x26, #80]
	cmp	x8, x23
	b.hs	.LBB49_102
// %bb.99:                              //   in Loop: Header=BB49_84 Depth=3
	ldr	x27, [x26, #736]
	ldr	x21, [x26, #736]
	mov	w1, #1
	mov	x0, x26
	str	x23, [x26, #80]
	bl	f2fs_mark_inode_dirty_sync
	tbz	w27, #1, .LBB49_101
// %bb.100:                             //   in Loop: Header=BB49_84 Depth=3
	tbz	w21, #2, .LBB49_102
.LBB49_101:                             //   in Loop: Header=BB49_84 Depth=3
	add	x8, x26, #736           // =736
	//APP
	// atomic64_or
	prfm	pstl1strm, [x8]
1:	ldxr	x9, [x8]
	orr	x9, x9, 4
	stxr	w10, x9, [x8]
	cbnz	w10, 1b
	//NO_APP
.LBB49_102:                             //   in Loop: Header=BB49_84 Depth=3
	cbz	x25, .LBB49_106
// %bb.103:                             //   in Loop: Header=BB49_84 Depth=3
	ldr	w8, [x24, #4]
	cbnz	w8, .LBB49_83
// %bb.104:                             //   in Loop: Header=BB49_84 Depth=3
	ldr	w8, [x28]
	cbz	w8, .LBB49_83
	b	.LBB49_106
.LBB49_105:                             //   in Loop: Header=BB49_84 Depth=3
	ldrh	w12, [x11, #360]
	lsr	w12, w12, #2
	b	.LBB49_90
.LBB49_106:                             //   in Loop: Header=BB49_63 Depth=2
	ldur	x0, [x29, #-40]
	cbz	x0, .LBB49_108
// %bb.107:                             //   in Loop: Header=BB49_63 Depth=2
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	b	.LBB49_109
.LBB49_108:                             //   in Loop: Header=BB49_63 Depth=2
	mov	x9, xzr
.LBB49_109:                             //   in Loop: Header=BB49_63 Depth=2
	ldr	x27, [sp, #56]          // 8-byte Folded Reload
	ldr	w28, [sp, #36]          // 4-byte Folded Reload
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB49_62
// %bb.110:                             //   in Loop: Header=BB49_63 Depth=2
	cmp	x9, x8
	b.eq	.LBB49_62
// %bb.111:                             //   in Loop: Header=BB49_63 Depth=2
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
	cbnz	w9, .LBB49_62
// %bb.112:                             //   in Loop: Header=BB49_63 Depth=2
	bl	__put_page
	b	.LBB49_62
.LBB49_113:                             //   in Loop: Header=BB49_2 Depth=1
	mov	x24, x25
	b	.LBB49_115
.LBB49_114:                             //   in Loop: Header=BB49_2 Depth=1
	mov	w1, #1
	mov	x0, x25
	bl	f2fs_put_page
.LBB49_115:                             //   in Loop: Header=BB49_2 Depth=1
	ldr	x27, [sp, #56]          // 8-byte Folded Reload
	cbnz	w24, .LBB49_155
.LBB49_116:                             //   in Loop: Header=BB49_2 Depth=1
	ldp	x21, x8, [sp, #40]      // 16-byte Folded Reload
	ldr	x0, [sp, #80]           // 8-byte Folded Reload
	add	x19, x27, x19
	add	x8, x27, x8
	str	x8, [sp, #48]           // 8-byte Folded Spill
	sub	x21, x21, x27
	bl	kvfree
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	kvfree
	mov	w24, wzr
	cbnz	x21, .LBB49_2
	b	.LBB49_150
.LBB49_117:
	mov	w24, wzr
	b	.LBB49_150
.LBB49_118:
	ldur	x0, [x29, #-40]
	cbz	x0, .LBB49_122
// %bb.119:
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	b	.LBB49_123
.LBB49_120:
	ldur	x0, [x29, #-40]
	cbz	x0, .LBB49_128
// %bb.121:
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	b	.LBB49_129
.LBB49_122:
	mov	x9, xzr
.LBB49_123:
	ldr	x21, [sp, #40]          // 8-byte Folded Reload
	ldr	x27, [sp, #56]          // 8-byte Folded Reload
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB49_127
// %bb.124:
	cmp	x9, x8
	b.eq	.LBB49_127
// %bb.125:
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
	cbnz	w10, .LBB49_127
// %bb.126:
	bl	__put_page
.LBB49_127:
	stp	xzr, xzr, [x29, #-48]
	mov	w24, #-95
	b	.LBB49_136
.LBB49_128:
	mov	x9, xzr
.LBB49_129:
	ldr	x21, [sp, #40]          // 8-byte Folded Reload
	ldr	x27, [sp, #56]          // 8-byte Folded Reload
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB49_133
// %bb.130:
	cmp	x9, x8
	b.eq	.LBB49_133
// %bb.131:
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
	cbnz	w10, .LBB49_133
// %bb.132:
	bl	__put_page
.LBB49_133:
	stp	xzr, xzr, [x29, #-48]
	mov	w24, #-117
	b	.LBB49_136
.LBB49_134:
	mov	w24, #-2
.LBB49_135:
	ldr	x21, [sp, #40]          // 8-byte Folded Reload
	ldr	x27, [sp, #56]          // 8-byte Folded Reload
.LBB49_136:
	ldr	x8, [x20, #40]
	ldr	x25, [x8, #1008]
	stp	xzr, xzr, [x29, #-24]
	stp	xzr, xzr, [x29, #-40]
	stp	xzr, xzr, [x29, #-56]
	cbz	x27, .LBB49_149
// %bb.137:
	ldr	x8, [sp, #24]           // 8-byte Folded Reload
	ldr	x23, [sp, #80]          // 8-byte Folded Reload
	ldr	x26, [sp, #64]          // 8-byte Folded Reload
	mov	w22, #1
	lsl	x8, x8, #2
	cmp	x21, x8
	csel	x21, x21, x8, lo
	b	.LBB49_140
.LBB49_138:                             //   in Loop: Header=BB49_140 Depth=1
	stp	xzr, xzr, [x29, #-48]
.LBB49_139:                             //   in Loop: Header=BB49_140 Depth=1
	add	x26, x26, #4            // =4
	add	x23, x23, #4            // =4
	subs	x21, x21, #1            // =1
	add	x19, x19, #1            // =1
	b.eq	.LBB49_149
.LBB49_140:                             // =>This Inner Loop Header: Depth=1
	ldr	w8, [x26]
	cbz	w8, .LBB49_139
// %bb.141:                             //   in Loop: Header=BB49_140 Depth=1
	sub	x0, x29, #56            // =56
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	sub	x0, x29, #56            // =56
	mov	w2, #2
	mov	x1, x19
	stp	x20, xzr, [x29, #-56]
	stur	xzr, [x29, #-40]
	stur	wzr, [x29, #-32]
	bl	f2fs_get_dnode_of_data
	cbz	w0, .LBB49_144
// %bb.142:                             //   in Loop: Header=BB49_140 Depth=1
	mov	w2, #1
	mov	x0, x25
	mov	x1, x20
	bl	dec_valid_block_count
	ldr	w1, [x23]
	mov	x0, x25
	bl	f2fs_invalidate_blocks
	ldur	x0, [x29, #-40]
	cbz	x0, .LBB49_145
.LBB49_143:                             //   in Loop: Header=BB49_140 Depth=1
	mov	w1, #1
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	ldur	x8, [x29, #-48]
	cbnz	x8, .LBB49_146
	b	.LBB49_138
.LBB49_144:                             //   in Loop: Header=BB49_140 Depth=1
	ldr	w1, [x23]
	sub	x0, x29, #56            // =56
	bl	f2fs_update_data_blkaddr
	ldur	x0, [x29, #-40]
	cbnz	x0, .LBB49_143
.LBB49_145:                             //   in Loop: Header=BB49_140 Depth=1
	mov	x9, xzr
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB49_138
.LBB49_146:                             //   in Loop: Header=BB49_140 Depth=1
	cmp	x9, x8
	b.eq	.LBB49_138
// %bb.147:                             //   in Loop: Header=BB49_140 Depth=1
	ldr	x9, [x8, #8]
	sub	x10, x9, #1             // =1
	tst	x9, #0x1
	csel	x0, x8, x10, eq
	add	x8, x0, #52             // =52
	//APP
	// atomic_sub_return
	prfm	pstl1strm, [x8]
1:	ldxr	w9, [x8]
	sub	w9, w9, w22
	stlxr	w10, w9, [x8]
	cbnz	w10, 1b
	dmb ish
	//NO_APP
	cbnz	w9, .LBB49_138
// %bb.148:                             //   in Loop: Header=BB49_140 Depth=1
	bl	__put_page
	b	.LBB49_138
.LBB49_149:
	ldr	x0, [sp, #80]           // 8-byte Folded Reload
	bl	kvfree
	ldr	x0, [sp, #64]           // 8-byte Folded Reload
	bl	kvfree
.LBB49_150:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB49_165
// %bb.151:
	mov	w0, w24
	ldp	x20, x19, [sp, #240]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #224]    // 16-byte Folded Reload
	ldp	x24, x23, [sp, #208]    // 16-byte Folded Reload
	ldp	x26, x25, [sp, #192]    // 16-byte Folded Reload
	ldp	x28, x27, [sp, #176]    // 16-byte Folded Reload
	ldp	x29, x30, [sp, #160]    // 16-byte Folded Reload
	add	sp, sp, #256            // =256
	ret
.LBB49_152:
	mov	w24, #-12
	b	.LBB49_150
.LBB49_153:
	ldr	x0, [sp, #80]           // 8-byte Folded Reload
	bl	kvfree
	mov	w24, #-12
	b	.LBB49_150
.LBB49_154:
	mov	w24, w0
.LBB49_155:
	ldr	x21, [sp, #40]          // 8-byte Folded Reload
	b	.LBB49_136
.LBB49_156:
	mov	w24, w0
	cbz	x8, .LBB49_159
// %bb.157:
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_put_page
	ldur	x9, [x29, #-40]
	b	.LBB49_160
.LBB49_158:
	mov	w24, w0
	b	.LBB49_135
.LBB49_159:
	mov	x9, xzr
.LBB49_160:
	ldr	x21, [sp, #40]          // 8-byte Folded Reload
	ldur	x8, [x29, #-48]
	cbz	x8, .LBB49_164
// %bb.161:
	cmp	x9, x8
	b.eq	.LBB49_164
// %bb.162:
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
	cbnz	w10, .LBB49_164
// %bb.163:
	bl	__put_page
.LBB49_164:
	stp	xzr, xzr, [x29, #-48]
	b	.LBB49_136
.LBB49_165:
	bl	__stack_chk_fail
.Lfunc_end49:
	.size	__exchange_data_block, .Lfunc_end49-__exchange_data_block
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_inode_param_check
	.type	f2fs_inode_param_check,@function
f2fs_inode_param_check:                 // @f2fs_inode_param_check
// %bb.0:
	stp	x29, x30, [sp, #-16]!   // 16-byte Folded Spill
	ldrh	w8, [x1]
	mov	x29, sp
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB50_4
// %bb.1:
	cmp	w2, #2                  // =2
	b.eq	.LBB50_8
// %bb.2:
	ldr	x8, [x1, #80]
	cmp	x8, #16, lsl #12        // =65536
	b.ge	.LBB50_5
// %bb.3:
	ldr	x2, [x1, #64]
	adrp	x1, .L.str.62
	add	x1, x1, :lo12:.L.str.62
	mov	w3, #65536
	bl	f2fs_printk
.LBB50_4:
	mov	w8, #-22
	b	.LBB50_14
.LBB50_5:
	cmp	w2, #1                  // =1
	b.ne	.LBB50_8
// %bb.6:
	ldr	x8, [x1, #736]
	tbnz	x8, #44, .LBB50_8
// %bb.7:
	ldr	x2, [x1, #64]
	adrp	x1, .L.str.63
	add	x1, x1, :lo12:.L.str.63
	bl	f2fs_printk
	mov	w8, #-22
	b	.LBB50_14
.LBB50_8:
	ldr	x8, [x1, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB50_15
// %bb.9:
	ldr	w8, [x1, #1280]
	cbnz	w8, .LBB50_12
// %bb.10:
	ldr	x8, [x1, #736]
	tbnz	x8, #35, .LBB50_12
// %bb.11:
	ldr	x8, [x1, #736]
	tbz	x8, #34, .LBB50_16
.LBB50_12:
	ldr	x2, [x1, #64]
	adrp	x1, .L.str.64
	add	x1, x1, :lo12:.L.str.64
.LBB50_13:
	bl	f2fs_printk
	mov	w8, #-13
.LBB50_14:
	mov	w0, w8
	ldp	x29, x30, [sp], #16     // 16-byte Folded Reload
	ret
.LBB50_15:
	ldr	x8, [x1, #736]
	tbnz	w8, #31, .LBB50_12
.LBB50_16:
	ldr	w9, [x1, #12]
	tbnz	w9, #16, .LBB50_23
// %bb.17:
	ldr	x8, [x1, #736]
	tbnz	w8, #15, .LBB50_24
// %bb.18:
	ldr	x8, [x1, #736]
	tbnz	w8, #17, .LBB50_25
// %bb.19:
	ldr	x8, [x1, #736]
	tbnz	w8, #28, .LBB50_26
// %bb.20:
	mov	w8, wzr
	tbz	w9, #3, .LBB50_14
// %bb.21:
	cmp	w2, #2                  // =2
	b.eq	.LBB50_14
// %bb.22:
	ldr	x2, [x1, #64]
	adrp	x1, .L.str.69
	add	x1, x1, :lo12:.L.str.69
	b	.LBB50_13
.LBB50_23:
	ldr	x2, [x1, #64]
	adrp	x1, .L.str.65
	add	x1, x1, :lo12:.L.str.65
	b	.LBB50_13
.LBB50_24:
	ldr	x2, [x1, #64]
	adrp	x1, .L.str.66
	add	x1, x1, :lo12:.L.str.66
	b	.LBB50_13
.LBB50_25:
	ldr	x2, [x1, #64]
	adrp	x1, .L.str.67
	add	x1, x1, :lo12:.L.str.67
	b	.LBB50_13
.LBB50_26:
	ldr	x2, [x1, #64]
	adrp	x1, .L.str.68
	add	x1, x1, :lo12:.L.str.68
	b	.LBB50_13
.Lfunc_end50:
	.size	f2fs_inode_param_check, .Lfunc_end50-f2fs_inode_param_check
                                        // -- End function
	.p2align	2               // -- Begin function dedup_file_is_same
	.type	dedup_file_is_same,@function
dedup_file_is_same:                     // @dedup_file_is_same
// %bb.0:
	sub	sp, sp, #128            // =128
	stp	x29, x30, [sp, #32]     // 16-byte Folded Spill
	stp	x28, x27, [sp, #48]     // 16-byte Folded Spill
	stp	x26, x25, [sp, #64]     // 16-byte Folded Spill
	stp	x24, x23, [sp, #80]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #96]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #112]    // 16-byte Folded Spill
	ldr	x8, [x0, #80]
	ldr	x9, [x1, #80]
	add	x29, sp, #32            // =32
	cmp	x8, x9
	b.ne	.LBB51_34
// %bb.1:
	add	x8, x8, #4095           // =4095
	mov	x19, x1
	mov	x20, x0
	mov	w26, #10
	mov	w27, #1
	lsr	x25, x8, #12
	//APP
	mrs x9, sp_el0
	//NO_APP
	str	x9, [sp, #16]           // 8-byte Folded Spill
.LBB51_2:                               // =>This Inner Loop Header: Depth=1
	bl	get_random_u32
	ldr	x8, [x20, #48]
	mov	w9, w0
	udiv	x10, x9, x25
	msub	x21, x10, x25, x9
	mov	x0, x8
	mov	x1, x21
	mov	x2, xzr
	mov	x3, xzr
	bl	read_cache_page
	cmn	x0, #1, lsl #12         // =4096
	b.hi	.LBB51_34
// %bb.3:                               //   in Loop: Header=BB51_2 Depth=1
	mov	x22, x0
	ldr	x0, [x19, #48]
	mov	x1, x21
	mov	x2, xzr
	mov	x3, xzr
	bl	read_cache_page
	ldr	x8, [x22, #8]
	mov	x23, x0
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x22, x9, eq
	cmn	x23, #4095              // =4095
	b.hs	.LBB51_31
// %bb.4:                               //   in Loop: Header=BB51_2 Depth=1
	ldr	x8, [x0]
	stur	x8, [x29, #-8]
	ldur	x8, [x29, #-8]
	tbnz	w8, #0, .LBB51_6
// %bb.5:                               //   in Loop: Header=BB51_2 Depth=1
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x0]
1:	ldaxr	x8, [x0]
	orr	x9, x8, 1
	stxr	w10, x9, [x0]
	cbnz	w10, 1b
	
	//NO_APP
	tbz	w8, #0, .LBB51_7
.LBB51_6:                               //   in Loop: Header=BB51_2 Depth=1
	mov	x0, x22
	bl	__lock_page
.LBB51_7:                               //   in Loop: Header=BB51_2 Depth=1
	cmp	x22, x23
	b.eq	.LBB51_11
// %bb.8:                               //   in Loop: Header=BB51_2 Depth=1
	ldr	x8, [x23, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x23, x9, eq
	ldr	x9, [x8]
	stur	x9, [x29, #-8]
	ldur	x9, [x29, #-8]
	tbnz	w9, #0, .LBB51_10
// %bb.9:                               //   in Loop: Header=BB51_2 Depth=1
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbz	w9, #0, .LBB51_11
.LBB51_10:                              //   in Loop: Header=BB51_2 Depth=1
	mov	x0, x23
	bl	__lock_page
.LBB51_11:                              //   in Loop: Header=BB51_2 Depth=1
	ldr	x8, [x22, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x22, x9, eq
	ldr	x8, [x8]
	tbz	w8, #2, .LBB51_15
// %bb.12:                              //   in Loop: Header=BB51_2 Depth=1
	//APP
	dmb ishld
	//NO_APP
	ldr	x8, [x23, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x23, x9, eq
	ldr	x8, [x8]
	tbz	w8, #2, .LBB51_15
// %bb.13:                              //   in Loop: Header=BB51_2 Depth=1
	//APP
	dmb ishld
	//NO_APP
	ldr	x8, [x22, #24]
	ldr	x9, [x20, #48]
	cmp	x8, x9
	b.ne	.LBB51_15
// %bb.14:                              //   in Loop: Header=BB51_2 Depth=1
	ldr	x8, [x23, #24]
	ldr	x9, [x19, #48]
	cmp	x8, x9
	b.eq	.LBB51_24
.LBB51_15:                              //   in Loop: Header=BB51_2 Depth=1
	mov	w28, wzr
	mov	w24, #-22
.LBB51_16:                              //   in Loop: Header=BB51_2 Depth=1
	mov	x0, x22
	bl	unlock_page
	cmp	x22, x23
	b.eq	.LBB51_18
// %bb.17:                              //   in Loop: Header=BB51_2 Depth=1
	mov	x0, x23
	bl	unlock_page
.LBB51_18:                              //   in Loop: Header=BB51_2 Depth=1
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
	cbnz	w9, .LBB51_20
// %bb.19:                              //   in Loop: Header=BB51_2 Depth=1
	bl	__put_page
.LBB51_20:                              //   in Loop: Header=BB51_2 Depth=1
	ldr	x8, [x22, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x0, x22, x9, eq
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
	cbnz	w9, .LBB51_22
// %bb.21:                              //   in Loop: Header=BB51_2 Depth=1
	bl	__put_page
.LBB51_22:                              //   in Loop: Header=BB51_2 Depth=1
	tbz	w28, #0, .LBB51_33
// %bb.23:                              //   in Loop: Header=BB51_2 Depth=1
	subs	w26, w26, #1            // =1
	b.ne	.LBB51_2
	b	.LBB51_36
.LBB51_24:                              //   in Loop: Header=BB51_2 Depth=1
	ldr	x28, [sp, #16]          // 8-byte Folded Reload
	mov	x10, #2097152
	movk	x10, #1, lsl #32
	add	x9, x22, x10
	ldr	w8, [x28, #24]
	lsl	x9, x9, #6
	and	x9, x9, #0xfffffffffffff000
	mov	x11, #-549755813888
	add	w8, w8, #1              // =1
	str	w8, [x28, #24]
	//APP
	//NO_APP
	ldr	w8, [x28, #2800]
	add	x9, x9, x11
	mov	x0, x22
	add	w8, w8, #1              // =1
	str	w8, [x28, #2800]
	//APP
	//NO_APP
	ldr	w8, [x28, #24]
	add	w8, w8, #1              // =1
	str	w8, [x28, #24]
	//APP
	//NO_APP
	ldr	w8, [x28, #2800]
	str	x9, [sp, #8]            // 8-byte Folded Spill
	add	w8, w8, #1              // =1
	str	w8, [x28, #2800]
	//APP
	//NO_APP
	add	x8, x23, x10
	lsl	x8, x8, #6
	and	x8, x8, #0xfffffffffffff000
	add	x24, x8, x11
	bl	flush_dcache_page
	mov	x0, x23
	bl	flush_dcache_page
	ldr	x1, [sp, #8]            // 8-byte Folded Reload
	mov	w2, #4096
	mov	x0, x24
	bl	memcmp
	//APP
	//NO_APP
	ldr	w8, [x28, #2800]
	mov	w24, w0
	sub	w8, w8, #1              // =1
	str	w8, [x28, #2800]
	//APP
	//NO_APP
	ldr	x8, [x28, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x28, #24]
	b.eq	.LBB51_29
// %bb.25:                              //   in Loop: Header=BB51_2 Depth=1
	ldr	x8, [x28, #24]
	mov	x9, x28
	cbz	x8, .LBB51_29
.LBB51_26:                              //   in Loop: Header=BB51_2 Depth=1
	//APP
	//NO_APP
	ldr	w8, [x9, #2800]
	sub	w8, w8, #1              // =1
	str	w8, [x9, #2800]
	//APP
	//NO_APP
	ldr	x8, [x9, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x9, #24]
	b.eq	.LBB51_30
// %bb.27:                              //   in Loop: Header=BB51_2 Depth=1
	ldr	x8, [x9, #24]
	cbz	x8, .LBB51_30
.LBB51_28:                              //   in Loop: Header=BB51_2 Depth=1
	cmp	w24, #0                 // =0
	cset	w28, eq
	b	.LBB51_16
.LBB51_29:                              //   in Loop: Header=BB51_2 Depth=1
	bl	preempt_schedule
	ldr	x9, [sp, #16]           // 8-byte Folded Reload
	b	.LBB51_26
.LBB51_30:                              //   in Loop: Header=BB51_2 Depth=1
	bl	preempt_schedule
	b	.LBB51_28
.LBB51_31:
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
	cbnz	w10, .LBB51_34
// %bb.32:
	bl	__put_page
	b	.LBB51_34
.LBB51_33:
	ldr	x8, [x20, #40]
	ldr	x2, [x20, #64]
	ldr	x3, [x19, #64]
	adrp	x1, .L.str.72
	ldr	x0, [x8, #1008]
	add	x1, x1, :lo12:.L.str.72
	mov	x4, x21
	mov	w5, w24
	bl	f2fs_printk
.LBB51_34:
	mov	w0, wzr
.LBB51_35:
	ldp	x20, x19, [sp, #112]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #96]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #80]     // 16-byte Folded Reload
	ldp	x26, x25, [sp, #64]     // 16-byte Folded Reload
	ldp	x28, x27, [sp, #48]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #32]     // 16-byte Folded Reload
	add	sp, sp, #128            // =128
	ret
.LBB51_36:
	mov	w0, #1
	b	.LBB51_35
.Lfunc_end51:
	.size	dedup_file_is_same, .Lfunc_end51-dedup_file_is_same
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_filemap_fault
	.type	f2fs_filemap_fault,@function
f2fs_filemap_fault:                     // @f2fs_filemap_fault
// %bb.0:
	stp	x29, x30, [sp, #-64]!   // 16-byte Folded Spill
	stp	x24, x23, [sp, #16]     // 16-byte Folded Spill
	stp	x22, x21, [sp, #32]     // 16-byte Folded Spill
	stp	x20, x19, [sp, #48]     // 16-byte Folded Spill
	ldr	x8, [x0]
	mov	x21, x0
	mov	x29, sp
	ldr	x8, [x8, #160]
	ldr	x20, [x8, #32]
	add	x22, x20, #1088         // =1088
	mov	x0, x22
	bl	down_read
	mov	x0, x21
	bl	filemap_fault
	mov	w19, w0
	mov	x0, x22
	bl	up_read
	cbnz	w19, .LBB52_3
// %bb.1:
	ldr	x8, [x20, #40]
	ldr	x22, [x8, #1008]
	ldrb	w8, [x22, #2704]
	cbz	w8, .LBB52_3
// %bb.2:
	add	x23, x22, #2344         // =2344
	mov	x0, x23
	bl	_raw_spin_lock
	ldr	x8, [x22, #2472]
	mov	x0, x23
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x22, #2472]
	bl	_raw_spin_unlock
	mov	x0, x22
	bl	f2fs_record_iostat
.LBB52_3:
	adrp	x8, __tracepoint_f2fs_filemap_fault+8
	ldr	x21, [x21, #16]
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_filemap_fault+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB52_10
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
	tbz	w8, #0, .LBB52_10
// %bb.5:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w8, [x23, #24]
	add	w8, w8, #1              // =1
	str	w8, [x23, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_filemap_fault+32
	ldr	x24, [x8, :lo12:__tracepoint_f2fs_filemap_fault+32]
	cbz	x24, .LBB52_8
// %bb.6:
	mov	w22, w19
.LBB52_7:                               // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x24]
	mov	x1, x20
	mov	x2, x21
	mov	x3, x22
	blr	x8
	ldr	x8, [x24, #24]!
	cbnz	x8, .LBB52_7
.LBB52_8:
	//APP
	//NO_APP
	ldr	x8, [x23, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x23, #24]
	b.eq	.LBB52_11
// %bb.9:
	ldr	x8, [x23, #24]
	cbz	x8, .LBB52_11
.LBB52_10:
	mov	w0, w19
	ldp	x20, x19, [sp, #48]     // 16-byte Folded Reload
	ldp	x22, x21, [sp, #32]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #16]     // 16-byte Folded Reload
	ldp	x29, x30, [sp], #64     // 16-byte Folded Reload
	ret
.LBB52_11:
	bl	preempt_schedule_notrace
	b	.LBB52_10
.Lfunc_end52:
	.size	f2fs_filemap_fault, .Lfunc_end52-f2fs_filemap_fault
                                        // -- End function
	.p2align	2               // -- Begin function f2fs_vm_page_mkwrite
	.type	f2fs_vm_page_mkwrite,@function
f2fs_vm_page_mkwrite:                   // @f2fs_vm_page_mkwrite
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
	ldr	x8, [x0]
	ldr	x20, [x0, #88]
	ldr	x8, [x8, #160]
	ldr	x19, [x8, #32]
	ldr	x8, [x19, #40]
	ldr	x21, [x8, #1008]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	ldrb	w8, [x19, #12]
	tbnz	w8, #3, .LBB53_65
// %bb.1:
	add	x24, x19, #160          // =160
	mov	x22, x0
	mov	x0, x24
	bl	down_write
	ldr	x8, [x19, #736]
	tbz	x8, #35, .LBB53_4
// %bb.2:
	mov	x0, x19
	mov	w1, wzr
	bl	f2fs_reserve_compress_blocks_internal
	cbz	w0, .LBB53_4
.LBB53_3:
	mov	w23, w0
	mov	x0, x24
	bl	up_write
	b	.LBB53_60
.LBB53_4:
	mov	x0, x24
	bl	up_write
	ldr	x8, [x21, #224]
	ldrb	w8, [x8, #132]
	tbnz	w8, #3, .LBB53_65
// %bb.5:
	ldr	x8, [x21, #80]
	tbnz	w8, #8, .LBB53_64
.LBB53_6:
	mov	x0, x19
	bl	f2fs_convert_inline_inode
	mov	w23, w0
	cbnz	w0, .LBB53_60
// %bb.7:
	mov	x0, x24
	bl	down_write
	ldr	x8, [x19, #736]
	tbz	x8, #44, .LBB53_9
// %bb.8:
	add	x8, x19, #736           // =736
	mov	x9, #17592186044416
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
.LBB53_9:
	ldr	x8, [x19, #736]
	tbz	x8, #38, .LBB53_13
// %bb.10:
	ldr	x8, [x19, #736]
	tbz	x8, #38, .LBB53_12
// %bb.11:
	ldr	x8, [x19, #736]
	tbnz	x8, #39, .LBB53_13
.LBB53_12:
	adrp	x1, .L__func__.f2fs_vm_page_mkwrite
	add	x1, x1, :lo12:.L__func__.f2fs_vm_page_mkwrite
	mov	x0, x19
	bl	f2fs_revoke_deduped_inode
	cbnz	w0, .LBB53_3
.LBB53_13:
	mov	x0, x24
	bl	up_write
	ldr	x8, [x19, #40]
	ldr	x8, [x8, #1008]
	ldr	w8, [x8, #1532]
	cbz	w8, .LBB53_17
// %bb.14:
	ldr	w8, [x19, #1280]
	cbnz	w8, .LBB53_19
// %bb.15:
	ldr	x8, [x19, #736]
	tbnz	x8, #35, .LBB53_19
// %bb.16:
	ldr	x8, [x19, #736]
	tbnz	x8, #34, .LBB53_19
	b	.LBB53_22
.LBB53_17:
	ldrh	w8, [x19]
	and	w8, w8, #0xf000
	cmp	w8, #8, lsl #12         // =32768
	b.ne	.LBB53_22
// %bb.18:
	ldr	x8, [x19, #736]
	tbz	w8, #31, .LBB53_22
.LBB53_19:
	ldr	x1, [x20, #32]
	mov	x0, x19
	bl	f2fs_is_compressed_cluster
	mov	w23, w0
	tbnz	w0, #31, .LBB53_60
// %bb.20:
	cbz	w23, .LBB53_22
// %bb.21:
	mov	w23, wzr
	b	.LBB53_23
.LBB53_22:
	mov	w1, #1
	mov	x0, x21
	mov	w23, #1
	bl	f2fs_balance_fs
.LBB53_23:
	ldr	x0, [x19, #40]
	mov	w1, #2
	mov	w2, #1
	bl	__sb_start_write
	ldr	x8, [x19, #736]
	tbnz	w8, #10, .LBB53_68
.LBB53_24:
	ldr	x8, [x22]
	ldr	x0, [x8, #160]
	bl	file_update_time
	add	x22, x19, #1088         // =1088
	mov	x0, x22
	bl	down_read
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x9, [x8]
	str	x9, [sp]
	ldr	x9, [sp]
	tbnz	w9, #0, .LBB53_26
// %bb.25:
	//APP
	// atomic64_fetch_or_acquire
	prfm	pstl1strm, [x8]
1:	ldaxr	x9, [x8]
	orr	x10, x9, 1
	stxr	w11, x10, [x8]
	cbnz	w11, 1b
	
	//NO_APP
	tbz	w9, #0, .LBB53_27
.LBB53_26:
	mov	x0, x20
	bl	__lock_page
.LBB53_27:
	ldr	x8, [x20, #24]
	ldr	x9, [x19, #48]
	cmp	x8, x9
	b.ne	.LBB53_30
// %bb.28:
	ldr	x8, [x20, #32]
	ldr	x9, [x19, #80]
	cmp	x9, x8, lsl #12
	b.lt	.LBB53_30
// %bb.29:
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB53_31
.LBB53_30:
	mov	x0, x20
	bl	unlock_page
	mov	w23, #-14
	b	.LBB53_59
.LBB53_31:
	//APP
	dmb ishld
	//NO_APP
	cbz	w23, .LBB53_34
// %bb.32:
	mov	w1, #5
	mov	w2, #1
	mov	x0, x21
	mov	w3, wzr
	bl	f2fs_do_map_lock
	mov	w24, w0
	add	x0, sp, #8              // =8
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	stp	x19, xzr, [sp, #8]
	str	xzr, [sp, #24]
	str	wzr, [sp, #32]
	ldr	x1, [x20, #32]
	add	x0, sp, #8              // =8
	bl	f2fs_get_block
	mov	w23, w0
	and	w3, w24, #0x1
	mov	w1, #5
	mov	x0, x21
	mov	w2, wzr
	bl	f2fs_do_map_lock
	cbz	w23, .LBB53_42
.LBB53_33:
	mov	x0, x20
	bl	unlock_page
	b	.LBB53_59
.LBB53_34:
	add	x0, sp, #8              // =8
	mov	w2, #48
	mov	w1, wzr
	bl	memset
	stp	x19, xzr, [sp, #8]
	str	xzr, [sp, #24]
	str	wzr, [sp, #32]
	ldr	x1, [x20, #32]
	add	x0, sp, #8              // =8
	mov	w2, #1
	bl	f2fs_get_dnode_of_data
	ldr	x8, [sp, #24]
	mov	w23, w0
	cbz	x8, .LBB53_36
// %bb.35:
	mov	w1, #1
	mov	x0, x8
	bl	f2fs_put_page
	ldr	x9, [sp, #24]
	b	.LBB53_37
.LBB53_36:
	mov	x9, xzr
.LBB53_37:
	ldr	x8, [sp, #16]
	cbz	x8, .LBB53_41
// %bb.38:
	cmp	x9, x8
	b.eq	.LBB53_41
// %bb.39:
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
	cbnz	w10, .LBB53_41
// %bb.40:
	bl	__put_page
.LBB53_41:
	stp	xzr, xzr, [sp, #16]
	cbnz	w23, .LBB53_33
.LBB53_42:
	mov	w3, #1
	mov	x0, x20
	mov	w1, wzr
	mov	w2, wzr
	bl	f2fs_wait_on_page_writeback
	ldr	w1, [sp, #44]
	mov	x0, x19
	bl	f2fs_wait_on_block_writeback
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #17, .LBB53_59
// %bb.43:
	ldr	x9, [x20, #32]
	ldr	x8, [x19, #80]
	lsl	x9, x9, #12
	add	x9, x9, #1, lsl #12     // =4096
	cmp	x9, x8
	b.le	.LBB53_47
// %bb.44:
	//APP
	mrs x23, sp_el0
	//NO_APP
	ldr	w9, [x23, #24]
	mov	x10, #134217728
	movk	x10, #64, lsl #32
	add	x10, x10, x20, lsl #6
	add	w9, w9, #1              // =1
	str	w9, [x23, #24]
	//APP
	//NO_APP
	ldr	w9, [x23, #2800]
	and	w8, w8, #0xfff
	and	x10, x10, #0xfffffffffffff000
	orr	x10, x10, x8
	add	w9, w9, #1              // =1
	str	w9, [x23, #2800]
	//APP
	//NO_APP
	mov	x9, #-549755813888
	add	x0, x10, x9
	mov	w9, #4096
	sub	w2, w9, w8
	mov	w1, wzr
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
	b.eq	.LBB53_76
// %bb.45:
	ldr	x8, [x23, #24]
	cbz	x8, .LBB53_76
.LBB53_46:
	mov	x0, x20
	bl	flush_dcache_page
.LBB53_47:
	mov	x0, x20
	bl	set_page_dirty
	ldr	x8, [x20, #8]
	sub	x9, x8, #1              // =1
	tst	x8, #0x1
	csel	x8, x20, x9, eq
	ldr	x8, [x8]
	tbnz	w8, #2, .LBB53_49
// %bb.48:
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
	b	.LBB53_50
.LBB53_49:
	//APP
	dmb ishld
	//NO_APP
.LBB53_50:
	ldrb	w8, [x21, #2704]
	cbz	w8, .LBB53_52
// %bb.51:
	add	x23, x21, #2344         // =2344
	mov	x0, x23
	bl	_raw_spin_lock
	ldr	x8, [x21, #2376]
	mov	x0, x23
	add	x8, x8, #1, lsl #12     // =4096
	str	x8, [x21, #2376]
	bl	_raw_spin_unlock
	mov	x0, x21
	bl	f2fs_record_iostat
.LBB53_52:
	adrp	x8, jiffies
	ldr	x8, [x8, :lo12:jiffies]
	stp	x8, x8, [x21, #504]
	str	x8, [x21, #520]
	adrp	x8, __tracepoint_f2fs_vm_page_mkwrite+8
	ldr	w8, [x8, :lo12:__tracepoint_f2fs_vm_page_mkwrite+8]
	cmp	w8, #1                  // =1
	b.lt	.LBB53_58
// %bb.53:
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
	tbz	w8, #0, .LBB53_58
// %bb.54:
	//APP
	mrs x21, sp_el0
	//NO_APP
	ldr	w8, [x21, #24]
	add	w8, w8, #1              // =1
	str	w8, [x21, #24]
	//APP
	//NO_APP
	adrp	x8, __tracepoint_f2fs_vm_page_mkwrite+32
	ldr	x23, [x8, :lo12:__tracepoint_f2fs_vm_page_mkwrite+32]
	cbz	x23, .LBB53_56
.LBB53_55:                              // =>This Inner Loop Header: Depth=1
	ldp	x8, x0, [x23]
	mov	x1, x20
	mov	w2, wzr
	blr	x8
	ldr	x8, [x23, #24]!
	cbnz	x8, .LBB53_55
.LBB53_56:
	//APP
	//NO_APP
	ldr	x8, [x21, #24]
	subs	x8, x8, #1              // =1
	str	w8, [x21, #24]
	b.eq	.LBB53_77
// %bb.57:
	ldr	x8, [x21, #24]
	cbz	x8, .LBB53_77
.LBB53_58:
	mov	w23, wzr
.LBB53_59:
	mov	x0, x22
	bl	up_read
	ldr	x0, [x19, #40]
	mov	w1, #2
	bl	__sb_end_write
.LBB53_60:
	add	w8, w23, #14            // =14
	cmp	w8, #14                 // =14
	b.hi	.LBB53_65
// %bb.61:
	adrp	x9, .LJTI53_0
	add	x9, x9, :lo12:.LJTI53_0
	adr	x10, .LBB53_62
	ldrb	w11, [x9, x8]
	add	x10, x10, x11, lsl #2
	mov	w0, #512
	br	x10
.LBB53_62:
	mov	w0, #256
	b	.LBB53_66
.LBB53_63:
	mov	w0, #1
	b	.LBB53_66
.LBB53_64:
	mov	x0, x21
	mov	w1, wzr
	bl	has_not_enough_free_secs
	tbz	w0, #0, .LBB53_6
.LBB53_65:
	mov	w0, #2
.LBB53_66:
	adrp	x9, __stack_chk_guard
	ldur	x8, [x29, #-8]
	ldr	x9, [x9, :lo12:__stack_chk_guard]
	cmp	x9, x8
	b.ne	.LBB53_78
// %bb.67:
	ldp	x20, x19, [sp, #112]    // 16-byte Folded Reload
	ldp	x22, x21, [sp, #96]     // 16-byte Folded Reload
	ldp	x24, x23, [sp, #80]     // 16-byte Folded Reload
	ldp	x29, x30, [sp, #64]     // 16-byte Folded Reload
	add	sp, sp, #128            // =128
	ret
.LBB53_68:
	bl	print_bdev_access_info
	adrp	x24, f2fs_dclient
	ldr	x0, [x24, :lo12:f2fs_dclient]
	cbz	x0, .LBB53_71
// %bb.69:
	bl	dsm_client_ocuppy
	cbnz	w0, .LBB53_71
// %bb.70:
	ldr	x0, [x24, :lo12:f2fs_dclient]
	adrp	x1, .L.str.8
	adrp	x2, .L__func__.f2fs_vm_page_mkwrite
	add	x1, x1, :lo12:.L.str.8
	add	x2, x2, :lo12:.L__func__.f2fs_vm_page_mkwrite
	mov	w3, #1935
	bl	dsm_client_record
	ldr	x0, [x24, :lo12:f2fs_dclient]
	mov	w1, #15241
	movk	w1, #14160, lsl #16
	bl	dsm_client_notify
.LBB53_71:
	mov	x0, x21
	bl	f2fs_print_raw_sb_info
	mov	x0, x21
	bl	f2fs_print_ckpt_info
	mov	x0, x21
	bl	f2fs_print_sbi_info
	//APP
	.pushsection __bug_table,"aw"; .align 2; 14470: .long 14471f - 14470b; .pushsection .rodata.str,"aMS",@progbits,1; 14472: .string "fs/hw_f2fs/file.c"; .popsection; .long 14472b - 14470b; .short 1935; .short (1 << 0)|(((9) << 8)); .popsection; 14471: brk 0x800
	//NO_APP
	cbz	x21, .LBB53_24
// %bb.72:
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
	cbnz	w8, .LBB53_24
// %bb.73:
	ldr	x1, [x21, #3392]
	cbz	x1, .LBB53_24
// %bb.74:
	adrp	x8, system_state
	ldr	w8, [x8, :lo12:system_state]
	cmp	w8, #2                  // =2
	b.hi	.LBB53_24
// %bb.75:
	add	x2, x21, #3408          // =3408
	mov	w0, #8
	bl	queue_work_on
	b	.LBB53_24
.LBB53_76:
	bl	preempt_schedule
	b	.LBB53_46
.LBB53_77:
	bl	preempt_schedule_notrace
	b	.LBB53_58
.LBB53_78:
	bl	__stack_chk_fail
.Lfunc_end53:
	.size	f2fs_vm_page_mkwrite, .Lfunc_end53-f2fs_vm_page_mkwrite
	.section	.rodata,"a",@progbits
.LJTI53_0:
	.byte	(.LBB53_62-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_63-.LBB53_62)>>2
	.byte	(.LBB53_62-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_65-.LBB53_62)>>2
	.byte	(.LBB53_66-.LBB53_62)>>2
                                        // -- End function
	.type	.L.str,@object          // @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"f2fs_page_info_entry"
	.size	.L.str, 21

	.type	page_info_slab,@object  // @page_info_slab
	.local	page_info_slab
	.comm	page_info_slab,8,8
	.type	.L.str.1,@object        // @.str.1
.L.str.1:
	.asciz	"\0013%s: try to get node page %d"
	.size	.L.str.1, 30

	.type	.L__func__.f2fs_set_inode_addr,@object // @__func__.f2fs_set_inode_addr
.L__func__.f2fs_set_inode_addr:
	.asciz	"f2fs_set_inode_addr"
	.size	.L__func__.f2fs_set_inode_addr, 20

	.type	.L.str.3,@object        // @.str.3
.L.str.3:
	.asciz	"\0013%s: get node page fail"
	.size	.L.str.3, 25

	.type	.L.str.4,@object        // @.str.4
.L.str.4:
	.asciz	"\0013%s: inode[%lu] leak data addr[%d:%u]"
	.size	.L.str.4, 39

	.type	.L.str.5,@object        // @.str.5
.L.str.5:
	.asciz	"\0013%s: inode[%lu] leak node addr[%d:%u]"
	.size	.L.str.5, 39

	.type	f2fs_revoke_deduped_inode.revoke_bigdata_rs,@object // @f2fs_revoke_deduped_inode.revoke_bigdata_rs
	.data
	.p2align	3
f2fs_revoke_deduped_inode.revoke_bigdata_rs:
	.zero	4
	.word	250                     // 0xfa
	.word	10                      // 0xa
	.word	0                       // 0x0
	.word	0                       // 0x0
	.zero	4
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	f2fs_revoke_deduped_inode.revoke_bigdata_rs, 40

	.type	.L.str.6,@object        // @.str.6
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.6:
	.asciz	"\0013dedup ino[%lu] inner_inode is NULL"
	.size	.L.str.6, 37

	.type	.L__func__.f2fs_revoke_deduped_inode,@object // @__func__.f2fs_revoke_deduped_inode
.L__func__.f2fs_revoke_deduped_inode:
	.asciz	"f2fs_revoke_deduped_inode"
	.size	.L__func__.f2fs_revoke_deduped_inode, 26

	.type	.L.str.7,@object        // @.str.7
.L.str.7:
	.asciz	"\0013inode:%lu is not dedup inode"
	.size	.L.str.7, 31

	.type	.L.str.8,@object        // @.str.8
.L.str.8:
	.asciz	"F2FS bug: %s:%d\n"
	.size	.L.str.8, 17

	.type	.L__func__.f2fs_truncate_dedup_inode,@object // @__func__.f2fs_truncate_dedup_inode
.L__func__.f2fs_truncate_dedup_inode:
	.asciz	"f2fs_truncate_dedup_inode"
	.size	.L__func__.f2fs_truncate_dedup_inode, 26

	.type	.L__func__.f2fs_do_truncate_blocks,@object // @__func__.f2fs_do_truncate_blocks
.L__func__.f2fs_do_truncate_blocks:
	.asciz	"f2fs_do_truncate_blocks"
	.size	.L__func__.f2fs_do_truncate_blocks, 24

	.type	.L__func__.f2fs_getattr,@object // @__func__.f2fs_getattr
.L__func__.f2fs_getattr:
	.asciz	"f2fs_getattr"
	.size	.L__func__.f2fs_getattr, 13

	.type	.L__func__.f2fs_setattr,@object // @__func__.f2fs_setattr
.L__func__.f2fs_setattr:
	.asciz	"f2fs_setattr"
	.size	.L__func__.f2fs_setattr, 13

	.type	f2fs_file_inode_operations,@object // @f2fs_file_inode_operations
	.section	.rodata,"a",@progbits
	.globl	f2fs_file_inode_operations
	.p2align	6
f2fs_file_inode_operations:
.Lf2fs_file_inode_operations$local:
	.xword	0
	.xword	0
	.xword	0
	.xword	f2fs_get_acl
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	f2fs_setattr
	.xword	f2fs_getattr
	.xword	f2fs_listxattr
	.xword	f2fs_fiemap
	.xword	0
	.xword	0
	.xword	0
	.xword	f2fs_set_acl
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.zero	56
	.size	f2fs_file_inode_operations, 256

	.type	.L__func__.f2fs_truncate_hole,@object // @__func__.f2fs_truncate_hole
	.section	.rodata.str1.1,"aMS",@progbits,1
.L__func__.f2fs_truncate_hole:
	.asciz	"f2fs_truncate_hole"
	.size	.L__func__.f2fs_truncate_hole, 19

	.type	.L.str.9,@object        // @.str.9
.L.str.9:
	.asciz	"\0014%s: Enable GC = ino %lx after %x GC trials"
	.size	.L.str.9, 45

	.type	.L__func__.f2fs_pin_file_control,@object // @__func__.f2fs_pin_file_control
.L__func__.f2fs_pin_file_control:
	.asciz	"f2fs_pin_file_control"
	.size	.L__func__.f2fs_pin_file_control, 22

	.type	.L.str.10,@object       // @.str.10
.L.str.10:
	.asciz	"\0014%s: reserve released blocks failed ret=%d, i_ino=%lx, iblocks=%llu, reserved=%u, compr_blocks=%u\n"
	.size	.L.str.10, 100

	.type	.L__func__.f2fs_reserve_compress_blocks_internal,@object // @__func__.f2fs_reserve_compress_blocks_internal
.L__func__.f2fs_reserve_compress_blocks_internal:
	.asciz	"f2fs_reserve_compress_blocks_internal"
	.size	.L__func__.f2fs_reserve_compress_blocks_internal, 38

	.type	.L.str.11,@object       // @.str.11
.L.str.11:
	.asciz	"\0014%s mode:%d, reserve failed:%d"
	.size	.L.str.11, 32

	.type	.L__func__.f2fs_decompress_file_internal,@object // @__func__.f2fs_decompress_file_internal
.L__func__.f2fs_decompress_file_internal:
	.asciz	"f2fs_decompress_file_internal"
	.size	.L__func__.f2fs_decompress_file_internal, 30

	.type	.L.str.12,@object       // @.str.12
.L.str.12:
	.asciz	"\0014%s dedup file"
	.size	.L.str.12, 16

	.type	.L.str.13,@object       // @.str.13
.L.str.13:
	.asciz	"\0014%s mode:%d decompressed err:%d"
	.size	.L.str.13, 33

	.type	.L.str.14,@object       // @.str.14
.L.str.14:
	.asciz	"\0016%s mode:%d decompressed succ"
	.size	.L.str.14, 31

	.type	f2fs_file_operations,@object // @f2fs_file_operations
	.section	.rodata,"a",@progbits
	.globl	f2fs_file_operations
	.p2align	3
f2fs_file_operations:
.Lf2fs_file_operations$local:
	.xword	0
	.xword	f2fs_llseek
	.xword	0
	.xword	0
	.xword	f2fs_file_read_iter
	.xword	f2fs_file_write_iter
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	f2fs_ioctl
	.xword	f2fs_compat_ioctl
	.xword	f2fs_file_mmap
	.xword	0                       // 0x0
	.xword	f2fs_file_open
	.xword	f2fs_file_flush
	.xword	f2fs_release_file
	.xword	f2fs_sync_file
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	iter_file_splice_write
	.xword	generic_file_splice_read
	.xword	0
	.xword	f2fs_fallocate
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	f2fs_file_operations, 288

	.type	current_flush_merge,@object // @current_flush_merge
	.bss
	.globl	current_flush_merge
	.p2align	3
current_flush_merge:
.Lcurrent_flush_merge$local:
	.xword	0                       // 0x0
	.size	current_flush_merge, 8

	.type	.L__func__.f2fs_put_page,@object // @__func__.f2fs_put_page
	.section	.rodata.str1.1,"aMS",@progbits,1
.L__func__.f2fs_put_page:
	.asciz	"f2fs_put_page"
	.size	.L__func__.f2fs_put_page, 14

	.type	.L.str.15,@object       // @.str.15
.L.str.15:
	.asciz	"\0013revoke inode[%lu] begin fail, ret:%d"
	.size	.L.str.15, 39

	.type	.L.str.16,@object       // @.str.16
.L.str.16:
	.asciz	"\0014%s: The deduped inode[%lu] revoked fail(errno=%d,%d)."
	.size	.L.str.16, 56

	.type	.L__func__.__revoke_deduped_data,@object // @__func__.__revoke_deduped_data
.L__func__.__revoke_deduped_data:
	.asciz	"__revoke_deduped_data"
	.size	.L__func__.__revoke_deduped_data, 22

	.type	.L.str.17,@object       // @.str.17
.L.str.17:
	.asciz	"SBI_NEED_FSCK:%s:%d\n"
	.size	.L.str.17, 21

	.type	.L__func__.__revoke_deduped_inode_end,@object // @__func__.__revoke_deduped_inode_end
.L__func__.__revoke_deduped_inode_end:
	.asciz	"__revoke_deduped_inode_end"
	.size	.L__func__.__revoke_deduped_inode_end, 27

	.type	.L.str.18,@object       // @.str.18
.L.str.18:
	.asciz	"\0014%s: orphan failed (ino=%lx), run fsck to fix."
	.size	.L.str.18, 48

	.type	.L.str.19,@object       // @.str.19
.L.str.19:
	.asciz	"\0016%s trigger revoke ret[%d], inode[%lu], inner ino[%lu]"
	.size	.L.str.19, 56

	.type	.L.str.20,@object       // @.str.20
.L.str.20:
	.asciz	"%d"
	.size	.L.str.20, 3

	.type	.L.str.21,@object       // @.str.21
.L.str.21:
	.asciz	"\0014%s:create eventobj failed"
	.size	.L.str.21, 28

	.type	.L__func__.f2fs_dedup_imonitor_send,@object // @__func__.f2fs_dedup_imonitor_send
.L__func__.f2fs_dedup_imonitor_send:
	.asciz	"f2fs_dedup_imonitor_send"
	.size	.L__func__.f2fs_dedup_imonitor_send, 25

	.type	.L.str.22,@object       // @.str.22
.L.str.22:
	.asciz	"ReDedupFileIno"
	.size	.L.str.22, 15

	.type	.L.str.23,@object       // @.str.23
.L.str.23:
	.asciz	"ReDedupInnerIno"
	.size	.L.str.23, 16

	.type	.L.str.24,@object       // @.str.24
.L.str.24:
	.asciz	"ReDedupSize"
	.size	.L.str.24, 12

	.type	.L.str.25,@object       // @.str.25
.L.str.25:
	.asciz	"ReDedupReason"
	.size	.L.str.25, 14

	.type	.L.str.26,@object       // @.str.26
.L.str.26:
	.asciz	"ReDedupFailedReason"
	.size	.L.str.26, 20

	.type	.L.str.27,@object       // @.str.27
.L.str.27:
	.asciz	"\0014%s send hievent failed, err: %d"
	.size	.L.str.27, 34

	.type	.L__func__._truncate_error_handle,@object // @__func__._truncate_error_handle
.L__func__._truncate_error_handle:
	.asciz	"_truncate_error_handle"
	.size	.L__func__._truncate_error_handle, 23

	.type	.L.str.28,@object       // @.str.28
.L.str.28:
	.asciz	"\0013truncate data failed, inode:%lu ret:%d"
	.size	.L.str.28, 41

	.type	delayflush,@object      // @delayflush
	.local	delayflush
	.comm	delayflush,4,4
	.type	.L.str.29,@object       // @.str.29
.L.str.29:
	.asciz	"\0014%s, potential error with diff:%d, i_compr_blocks:%d, i_ino:%lu"
	.size	.L.str.29, 65

	.type	.L__func__.f2fs_i_compr_blocks_update,@object // @__func__.f2fs_i_compr_blocks_update
.L__func__.f2fs_i_compr_blocks_update:
	.asciz	"f2fs_i_compr_blocks_update"
	.size	.L__func__.f2fs_i_compr_blocks_update, 27

	.type	.L__func__.dec_valid_block_count,@object // @__func__.dec_valid_block_count
.L__func__.dec_valid_block_count:
	.asciz	"dec_valid_block_count"
	.size	.L__func__.dec_valid_block_count, 22

	.type	.L.str.30,@object       // @.str.30
.L.str.30:
	.asciz	"\0014Inconsistent i_blocks, ino:%lu, iblocks:%llu, sectors:%llu"
	.size	.L.str.30, 61

	.type	.L.str.31,@object       // @.str.31
.L.str.31:
	.asciz	"\0013cp_rwsem_timeout: start: %lu now: %lu\n"
	.size	.L.str.31, 41

	.type	.L__func__.truncate_partial_data_page,@object // @__func__.truncate_partial_data_page
.L__func__.truncate_partial_data_page:
	.asciz	"truncate_partial_data_page"
	.size	.L__func__.truncate_partial_data_page, 27

	.type	.L__func__.put_inner_inode,@object // @__func__.put_inner_inode
.L__func__.put_inner_inode:
	.asciz	"put_inner_inode"
	.size	.L__func__.put_inner_inode, 16

	.type	.L__func__.f2fs_release_reserved_blocks,@object // @__func__.f2fs_release_reserved_blocks
.L__func__.f2fs_release_reserved_blocks:
	.asciz	"f2fs_release_reserved_blocks"
	.size	.L__func__.f2fs_release_reserved_blocks, 29

	.type	.L__func__.f2fs_compress_imonitor_send,@object // @__func__.f2fs_compress_imonitor_send
.L__func__.f2fs_compress_imonitor_send:
	.asciz	"f2fs_compress_imonitor_send"
	.size	.L__func__.f2fs_compress_imonitor_send, 28

	.type	.L.str.32,@object       // @.str.32
.L.str.32:
	.asciz	"RevExeScence"
	.size	.L.str.32, 13

	.type	.L.str.33,@object       // @.str.33
.L.str.33:
	.asciz	"RevExeFileIno"
	.size	.L.str.33, 14

	.type	.L.str.34,@object       // @.str.34
.L.str.34:
	.asciz	"RevAppliedSize"
	.size	.L.str.34, 15

	.type	.L.str.35,@object       // @.str.35
.L.str.35:
	.asciz	"RevExeResult"
	.size	.L.str.35, 13

	.type	.L.str.37,@object       // @.str.37
.L.str.37:
	.asciz	"\0016%s scene[%u] send hievent ok"
	.size	.L.str.37, 31

	.type	.L__func__.f2fs_setflags_common,@object // @__func__.f2fs_setflags_common
.L__func__.f2fs_setflags_common:
	.asciz	"f2fs_setflags_common"
	.size	.L__func__.f2fs_setflags_common, 21

	.type	.L__func__.f2fs_ioc_start_atomic_write,@object // @__func__.f2fs_ioc_start_atomic_write
.L__func__.f2fs_ioc_start_atomic_write:
	.asciz	"f2fs_ioc_start_atomic_write"
	.size	.L__func__.f2fs_ioc_start_atomic_write, 28

	.type	.L.str.38,@object       // @.str.38
.L.str.38:
	.asciz	"\0014Unexpected flush for atomic writes: ino=%lu, npages=%u"
	.size	.L.str.38, 57

	.type	.L__func__.f2fs_ioc_start_volatile_write,@object // @__func__.f2fs_ioc_start_volatile_write
.L__func__.f2fs_ioc_start_volatile_write:
	.asciz	"f2fs_ioc_start_volatile_write"
	.size	.L__func__.f2fs_ioc_start_volatile_write, 30

	.type	f2fs_ioc_fitrim._rs,@object // @f2fs_ioc_fitrim._rs
	.data
	.p2align	3
f2fs_ioc_fitrim._rs:
	.zero	4
	.word	1250                    // 0x4e2
	.word	10                      // 0xa
	.word	0                       // 0x0
	.word	0                       // 0x0
	.zero	4
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	f2fs_ioc_fitrim._rs, 40

	.type	.L__func__.f2fs_ioc_fitrim,@object // @__func__.f2fs_ioc_fitrim
	.section	.rodata.str1.1,"aMS",@progbits,1
.L__func__.f2fs_ioc_fitrim:
	.asciz	"f2fs_ioc_fitrim"
	.size	.L__func__.f2fs_ioc_fitrim, 16

	.type	.L.str.39,@object       // @.str.39
.L.str.39:
	.asciz	"\0016%s: Recive fstrim command from userspace!\n"
	.size	.L.str.39, 45

	.type	.L.str.41,@object       // @.str.41
.L.str.41:
	.asciz	"\0016IOC_GC: Size=%lldMB,Free=%lldMB,count=%d,sync=%d\n"
	.size	.L.str.41, 52

	.type	IOC_GC_count,@object    // @IOC_GC_count
	.local	IOC_GC_count
	.comm	IOC_GC_count,4,4
	.type	.L.str.42,@object       // @.str.42
.L.str.42:
	.asciz	"\0016Skipping Checkpoint. Checkpoints currently disabled."
	.size	.L.str.42, 55

	.type	.L__func__.f2fs_defragment_range,@object // @__func__.f2fs_defragment_range
.L__func__.f2fs_defragment_range:
	.asciz	"f2fs_defragment_range"
	.size	.L__func__.f2fs_defragment_range, 22

	.type	.L__func__.f2fs_move_file_range,@object // @__func__.f2fs_move_file_range
.L__func__.f2fs_move_file_range:
	.asciz	"f2fs_move_file_range"
	.size	.L__func__.f2fs_move_file_range, 21

	.type	.L.str.43,@object       // @.str.43
.L.str.43:
	.asciz	"\0014Can't flush %u in %d for segs_per_sec %u != 1"
	.size	.L.str.43, 48

	.type	.L__func__.f2fs_ioc_set_pin_file,@object // @__func__.f2fs_ioc_set_pin_file
.L__func__.f2fs_ioc_set_pin_file:
	.asciz	"f2fs_ioc_set_pin_file"
	.size	.L__func__.f2fs_ioc_set_pin_file, 22

	.type	.L.str.44,@object       // @.str.44
.L.str.44:
	.asciz	"\0014Can't enable fs-verity on inode %lu: the verity feature is not enabled on this filesystem\n"
	.size	.L.str.44, 93

	.type	.L.str.45,@object       // @.str.45
.L.str.45:
	.asciz	"\0014%s: partial blocks were released i_ino=%lx iblocks=%llu, released=%u, compr_blocks=%u, run fsck to fix."
	.size	.L.str.45, 106

	.type	.L__func__.f2fs_release_compress_blocks,@object // @__func__.f2fs_release_compress_blocks
.L__func__.f2fs_release_compress_blocks:
	.asciz	"f2fs_release_compress_blocks"
	.size	.L__func__.f2fs_release_compress_blocks, 29

	.type	.L.str.46,@object       // @.str.46
.L.str.46:
	.asciz	"\0014%s: partial blocks were released i_ino=%lx iblocks=%llu, reserved=%u, compr_blocks=%u, run fsck to fix."
	.size	.L.str.46, 106

	.type	.L__func__.f2fs_reserve_compress_blocks,@object // @__func__.f2fs_reserve_compress_blocks
.L__func__.f2fs_reserve_compress_blocks:
	.asciz	"f2fs_reserve_compress_blocks"
	.size	.L__func__.f2fs_reserve_compress_blocks, 29

	.type	.L.str.47,@object       // @.str.47
.L.str.47:
	.asciz	"\0014%s: decompressed blocks=%u or valid blocks=%u errror."
	.size	.L.str.47, 56

	.type	.L__func__.f2fs_ioc_get_saved_blocks,@object // @__func__.f2fs_ioc_get_saved_blocks
.L__func__.f2fs_ioc_get_saved_blocks:
	.asciz	"f2fs_ioc_get_saved_blocks"
	.size	.L__func__.f2fs_ioc_get_saved_blocks, 26

	.type	.L.str.48,@object       // @.str.48
.L.str.48:
	.asciz	"\0014inline data [%lu], size:%lu."
	.size	.L.str.48, 31

	.type	.L.str.49,@object       // @.str.49
.L.str.49:
	.asciz	"\0013compress fails, atomic file [%lu]."
	.size	.L.str.49, 37

	.type	.L.str.50,@object       // @.str.50
.L.str.50:
	.asciz	"\0013compress fails, pin file [%lu]."
	.size	.L.str.50, 34

	.type	.L.str.51,@object       // @.str.51
.L.str.51:
	.asciz	"\0014%s: The file might be partially compressed (errno=%d). Please delete the file."
	.size	.L.str.51, 81

	.type	.L__func__.f2fs_ioc_compress_release_file,@object // @__func__.f2fs_ioc_compress_release_file
.L__func__.f2fs_ioc_compress_release_file:
	.asciz	"f2fs_ioc_compress_release_file"
	.size	.L__func__.f2fs_ioc_compress_release_file, 31

	.type	.L.str.52,@object       // @.str.52
.L.str.52:
	.asciz	"\0014%s, potential error with i_compr_blocks:%d, ino:%lu"
	.size	.L.str.52, 54

	.type	.L.str.53,@object       // @.str.53
.L.str.53:
	.asciz	"\0014%s: i_ino:%d decompress err:%d."
	.size	.L.str.53, 34

	.type	.L__func__.f2fs_ioc_reserve_decompress_file,@object // @__func__.f2fs_ioc_reserve_decompress_file
.L__func__.f2fs_ioc_reserve_decompress_file:
	.asciz	"f2fs_ioc_reserve_decompress_file"
	.size	.L__func__.f2fs_ioc_reserve_decompress_file, 33

	.type	.L.str.54,@object       // @.str.54
.L.str.54:
	.asciz	"\0014compression algorithm is successfully set, but current kernel doesn't support this algorithm."
	.size	.L.str.54, 96

	.type	.L.str.55,@object       // @.str.55
.L.str.55:
	.asciz	"\0014%s: The file might be partially decompressed (errno=%d). Please delete the file."
	.size	.L.str.55, 83

	.type	.L__func__.f2fs_ioc_decompress_file,@object // @__func__.f2fs_ioc_decompress_file
.L__func__.f2fs_ioc_decompress_file:
	.asciz	"f2fs_ioc_decompress_file"
	.size	.L__func__.f2fs_ioc_decompress_file, 25

	.type	.L__func__.f2fs_ioc_compress_file,@object // @__func__.f2fs_ioc_compress_file
.L__func__.f2fs_ioc_compress_file:
	.asciz	"f2fs_ioc_compress_file"
	.size	.L__func__.f2fs_ioc_compress_file, 23

	.type	.L.str.56,@object       // @.str.56
.L.str.56:
	.asciz	"\0013The inode[%lu] has been two layer file."
	.size	.L.str.56, 42

	.type	.L.str.57,@object       // @.str.57
.L.str.57:
	.asciz	"\0013%s files should be in same FS ino:%lu, inner_ino:%lu"
	.size	.L.str.57, 55

	.type	.L__func__.f2fs_ioc_create_layered_inode,@object // @__func__.f2fs_ioc_create_layered_inode
.L__func__.f2fs_ioc_create_layered_inode:
	.asciz	"f2fs_ioc_create_layered_inode"
	.size	.L__func__.f2fs_ioc_create_layered_inode, 30

	.type	.L.str.58,@object       // @.str.58
.L.str.58:
	.asciz	"\0013create layer fail inner[%lu] is not immutable."
	.size	.L.str.58, 49

	.type	.L.str.59,@object       // @.str.59
.L.str.59:
	.asciz	"\0013The inode[%lu] has been removed."
	.size	.L.str.59, 35

	.type	.L.str.60,@object       // @.str.60
.L.str.60:
	.asciz	"\0013%s: input inode[%lu] and [%lu] are illegal."
	.size	.L.str.60, 46

	.type	.L__func__.f2fs_dedup_param_check,@object // @__func__.f2fs_dedup_param_check
.L__func__.f2fs_dedup_param_check:
	.asciz	"f2fs_dedup_param_check"
	.size	.L__func__.f2fs_dedup_param_check, 23

	.type	.L.str.61,@object       // @.str.61
.L.str.61:
	.asciz	"\0013dedup file size not match inode1[%lu] %u, inode2[%lu] %u"
	.size	.L.str.61, 59

	.type	.L.str.62,@object       // @.str.62
.L.str.62:
	.asciz	"\0013dedup fails, inode[%lu] size < %lu bytes."
	.size	.L.str.62, 44

	.type	.L.str.63,@object       // @.str.63
.L.str.63:
	.asciz	"\0013dedup fails, inode[%lu] has been modified."
	.size	.L.str.63, 45

	.type	.L.str.64,@object       // @.str.64
.L.str.64:
	.asciz	"\0013dedup fails, inode[%lu] has been compressed."
	.size	.L.str.64, 47

	.type	.L.str.65,@object       // @.str.65
.L.str.65:
	.asciz	"\0013dedup fails, inode[%lu] enable verity."
	.size	.L.str.65, 41

	.type	.L.str.66,@object       // @.str.66
.L.str.66:
	.asciz	"\0013dedup fails, inode[%lu] is atomic file."
	.size	.L.str.66, 42

	.type	.L.str.67,@object       // @.str.67
.L.str.67:
	.asciz	"\0013dedup fails, inode[%lu] is volatile file."
	.size	.L.str.67, 44

	.type	.L.str.68,@object       // @.str.68
.L.str.68:
	.asciz	"\0013dedup fails, inode[%lu] is pinned file."
	.size	.L.str.68, 42

	.type	.L.str.69,@object       // @.str.69
.L.str.69:
	.asciz	"\0013dedup fails, inode[%lu] is immutable."
	.size	.L.str.69, 40

	.type	.L.str.70,@object       // @.str.70
.L.str.70:
	.asciz	"\0013inode[%lu] not match dir[%lu] fscrypt policy"
	.size	.L.str.70, 47

	.type	.L.str.71,@object       // @.str.71
.L.str.71:
	.asciz	"\0013create layer file acquire orphan fail, ino[%lu], inner[%lu]"
	.size	.L.str.71, 62

	.type	.L.str.72,@object       // @.str.72
.L.str.72:
	.asciz	"\0013src: %lu, dst: %lu page index: %lu is diff ret[%d]"
	.size	.L.str.72, 53

	.type	.L.str.73,@object       // @.str.73
.L.str.73:
	.asciz	"\0013%s: files should be in same FS ino:%lu, base_ino:%lu"
	.size	.L.str.73, 55

	.type	.L__func__.f2fs_ioc_dedup_file,@object // @__func__.f2fs_ioc_dedup_file
.L__func__.f2fs_ioc_dedup_file:
	.asciz	"f2fs_ioc_dedup_file"
	.size	.L__func__.f2fs_ioc_dedup_file, 20

	.type	.L.str.74,@object       // @.str.74
.L.str.74:
	.asciz	"\0013%s: input inode[%lu] and [%lu] are same."
	.size	.L.str.74, 43

	.type	.L.str.75,@object       // @.str.75
.L.str.75:
	.asciz	"\0013inode[%lu] can't get lock"
	.size	.L.str.75, 28

	.type	.L.str.76,@object       // @.str.76
.L.str.76:
	.asciz	"\0013dedup inode[%lu] has been two layer inode"
	.size	.L.str.76, 44

	.type	.L.str.77,@object       // @.str.77
.L.str.77:
	.asciz	"\0013dedup inode[%lu] has been removed."
	.size	.L.str.77, 37

	.type	.L.str.78,@object       // @.str.78
.L.str.78:
	.asciz	"\0013base inode[%lu] is not outer inode"
	.size	.L.str.78, 37

	.type	.L.str.79,@object       // @.str.79
.L.str.79:
	.asciz	"\0013dir[%lu] inode[%lu] and inode[%lu] fscrypt policy not match."
	.size	.L.str.79, 63

	.type	.L.str.80,@object       // @.str.80
.L.str.80:
	.asciz	"\0013inode[%lu] and dir[%lu] inode[%lu] fscrypt policy not match."
	.size	.L.str.80, 63

	.type	.L.str.81,@object       // @.str.81
.L.str.81:
	.asciz	"\0013dedup file acquire orphan fail, ino[%lu], base ino[%lu]"
	.size	.L.str.81, 58

	.type	.L__func__.f2fs_ioc_dedup_revoke,@object // @__func__.f2fs_ioc_dedup_revoke
.L__func__.f2fs_ioc_dedup_revoke:
	.asciz	"f2fs_ioc_dedup_revoke"
	.size	.L__func__.f2fs_ioc_dedup_revoke, 22

	.type	.L.str.82,@object       // @.str.82
.L.str.82:
	.asciz	"\0013%s: files should be in same FS ino:%lu, src_ino:%lu"
	.size	.L.str.82, 54

	.type	.L__func__.f2fs_ioc_clone_file,@object // @__func__.f2fs_ioc_clone_file
.L__func__.f2fs_ioc_clone_file:
	.asciz	"f2fs_ioc_clone_file"
	.size	.L__func__.f2fs_ioc_clone_file, 20

	.type	.L.str.83,@object       // @.str.83
.L.str.83:
	.asciz	"\0013inode[%lu] convert inline inode failed, ret:%d"
	.size	.L.str.83, 49

	.type	.L.str.84,@object       // @.str.84
.L.str.84:
	.asciz	"\0013src inode[%lu] dst inode[%lu] ioc clone failed. ret=%d"
	.size	.L.str.84, 57

	.type	.L.str.85,@object       // @.str.85
.L.str.85:
	.asciz	"\0013inode[%lu] has mapped vma"
	.size	.L.str.85, 28

	.type	.L__func__.f2fs_ioc_modify_check,@object // @__func__.f2fs_ioc_modify_check
.L__func__.f2fs_ioc_modify_check:
	.asciz	"f2fs_ioc_modify_check"
	.size	.L__func__.f2fs_ioc_modify_check, 22

	.type	.L.str.86,@object       // @.str.86
.L.str.86:
	.asciz	"\0013inode[%lu] write data fail(%d)\n"
	.size	.L.str.86, 34

	.type	.L.str.87,@object       // @.str.87
.L.str.87:
	.asciz	"\0013inode[%lu] have dirty page[%d]\n"
	.size	.L.str.87, 34

	.type	.L.str.88,@object       // @.str.88
.L.str.88:
	.asciz	"set unmodify, dirty pages: %d"
	.size	.L.str.88, 30

	.type	.L.str.89,@object       // @.str.89
.L.str.89:
	.asciz	"\0013inode[%lu] had set meta unmodified flag"
	.size	.L.str.89, 42

	.type	.L.str.90,@object       // @.str.90
.L.str.90:
	.asciz	"\0013inode[%lu] had set data unmodified flag"
	.size	.L.str.90, 42

	.type	.L.str.91,@object       // @.str.91
.L.str.91:
	.asciz	"\0013inode[%lu] had clear unmodified meta flag"
	.size	.L.str.91, 44

	.type	.L.str.92,@object       // @.str.92
.L.str.92:
	.asciz	"\0013inode[%lu] had clear unmodified data flag"
	.size	.L.str.92, 44

	.type	.L__func__.f2fs_file_write_iter,@object // @__func__.f2fs_file_write_iter
.L__func__.f2fs_file_write_iter:
	.asciz	"f2fs_file_write_iter"
	.size	.L__func__.f2fs_file_write_iter, 21

	.type	f2fs_file_vm_ops,@object // @f2fs_file_vm_ops
	.section	.rodata,"a",@progbits
	.p2align	3
f2fs_file_vm_ops:
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	f2fs_filemap_fault
	.xword	0
	.xword	filemap_map_pages
	.xword	0
	.xword	f2fs_vm_page_mkwrite
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.xword	0                       // 0x0
	.size	f2fs_file_vm_ops, 136

	.type	.L__func__.f2fs_vm_page_mkwrite,@object // @__func__.f2fs_vm_page_mkwrite
	.section	.rodata.str1.1,"aMS",@progbits,1
.L__func__.f2fs_vm_page_mkwrite:
	.asciz	"f2fs_vm_page_mkwrite"
	.size	.L__func__.f2fs_vm_page_mkwrite, 21

	.type	.L__func__.f2fs_fallocate,@object // @__func__.f2fs_fallocate
.L__func__.f2fs_fallocate:
	.asciz	"f2fs_fallocate"
	.size	.L__func__.f2fs_fallocate, 15

	.ident	"Android (6443078 based on r383902) clang version 11.0.1 (https://android.googlesource.com/toolchain/llvm-project b397f81060ce6d701042b782172ed13bee898b79)"
	.section	".note.GNU-stack","",@progbits
