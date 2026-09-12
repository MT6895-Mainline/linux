// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/stacktrace.h>

#include "hang_unwind.h"

/*
 * qqcandy: the 5.10 vendor code walked the arm64 frame-record chain by hand
 * with struct stackframe / unwind_frame() / on_task_stack() and stripped the
 * PAC with ptrauth_strip_insn_pac(). The 6.18 arm64 unwinder replaced all of
 * those (see arch/arm64/kernel/stacktrace.c and the kunwind state machine)
 * and the old names no longer exist, so use the stable public interface:
 * stack_trace_save_tsk() wraps arch_stack_walk() and returns exactly the
 * array of saved PCs the detector wants, for the current task, a blocked
 * task or a running task alike.
 */
unsigned int hang_kernel_trace(struct task_struct *tsk,
			       unsigned long *store, unsigned int size)
{
#ifdef CONFIG_STACKTRACE
	return stack_trace_save_tsk(tsk, store, size, 0);
#else
	return 0;
#endif
}
EXPORT_SYMBOL(hang_kernel_trace);

const char *hang_arch_vma_name(struct vm_area_struct *vma)
{
	/*
	 * qqcandy: built-in only, so the vendor's #ifdef MODULE variant is
	 * dropped and this always resolves through the generic hook.
	 */
	return arch_vma_name(vma);
}
EXPORT_SYMBOL(hang_arch_vma_name);
